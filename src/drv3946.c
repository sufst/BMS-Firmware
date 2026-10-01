/**
 * @file drv3946.c
 * @brief TI DRV3946-Q1 Contactor / Solenoid Driver implementation for Zephyr RTOS.
 *
 * Ported from SUFST STM32 HAL driver to Zephyr RTOS.
 * Completely independent of any specific SPI peripheral or MCU pinout:
 *   - When hdrv->spi.bus != NULL, performs real 24-bit SPI transfers via spi_transceive_dt().
 *   - When hdrv->spi.bus == NULL (before PCB pinout is finalized), runs in
 *     hardware-unbound mode while still executing all shadow register updates
 *     and CRC-8 (0x2F) calculations.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "drv3946.h"

LOG_MODULE_REGISTER(drv3946, LOG_LEVEL_INF);

/* ==========================================================================
 * CRC-8 Lookup Table (Polynomial 0x2F: x^8 + x^5 + x^3 + x^2 + x + 1, Init 0xFF)
 * Matching TI DRV3946-Q1 Specification
 * ========================================================================== */

static const uint8_t crc8_table[256] = {
	0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A,
	0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63, 0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
	0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
	0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
	0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B, 0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
	0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
	0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
	0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6, 0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
	0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
	0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
	0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D, 0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
	0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
	0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F,
	0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66, 0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
	0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
	0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42
};

/* ==========================================================================
 * Static CRC-8 Helpers
 * ========================================================================== */

static uint8_t DRV3946_ComputeCrcCmd(uint8_t addr_byte, uint16_t reg_data)
{
	uint8_t crc = 0xFFU;

	crc = crc8_table[addr_byte ^ crc];
	crc = crc8_table[((uint8_t)(reg_data >> 8)) ^ crc];
	return crc;
}

static uint8_t DRV3946_ComputeCrcConfigA(const uint16_t configA[7])
{
	uint8_t crc = 0xFFU;
	uint8_t bytes[13];

	/* Split A0..A5 into high and low bytes + high byte of A6 */
	for (int i = 0; i < 6; i++) {
		bytes[2 * i]     = (uint8_t)(configA[i] >> 8);
		bytes[2 * i + 1] = (uint8_t)(configA[i] & 0xFFU);
	}
	bytes[12] = (uint8_t)(configA[6] >> 8);

	for (int i = 0; i < 13; i++) {
		crc = crc8_table[bytes[i] ^ crc];
	}
	return crc;
}

static uint8_t DRV3946_ComputeCrcConfigB(const uint16_t configB[5])
{
	uint8_t crc = 0xFFU;
	uint8_t bytes[9];

	/* Split B0..B3 into high and low bytes + high byte of B4 */
	for (int i = 0; i < 4; i++) {
		bytes[2 * i]     = (uint8_t)(configB[i] >> 8);
		bytes[2 * i + 1] = (uint8_t)(configB[i] & 0xFFU);
	}
	bytes[8] = (uint8_t)(configB[4] >> 8);

	for (int i = 0; i < 9; i++) {
		crc = crc8_table[bytes[i] ^ crc];
	}
	return crc;
}

/* ==========================================================================
 * SPI Low-Level Helper (24-bit Frame, Peripheral-Agnostic)
 * ========================================================================== */

static DRV3946_Status_t DRV3946_SpiTransfer(DRV3946_HandleTypeDef *hdrv,
					    uint8_t tx_buf[3],
					    uint8_t rx_buf[3])
{
	if (hdrv == NULL) {
		return DRV3946_ERROR;
	}

	/*
	 * If no SPI peripheral is bound yet (pinout pending), operate in
	 * stub/simulated mode so upper-layer logic works without hardware.
	 */
	if (hdrv->spi.bus == NULL) {
		rx_buf[0] = 0U;
		rx_buf[1] = 0U;
		rx_buf[2] = 0U;
		LOG_DBG("DRV3946 SPI (unbound): TX [%02X %02X %02X]",
			tx_buf[0], tx_buf[1], tx_buf[2]);
		return DRV3946_OK;
	}

	if (!spi_is_ready_dt(&hdrv->spi)) {
		LOG_ERR("DRV3946 SPI bus device not ready");
		return DRV3946_ERROR;
	}

	struct spi_buf tx_spi_buf = {
		.buf = tx_buf,
		.len = 3U,
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_spi_buf,
		.count   = 1U,
	};

	struct spi_buf rx_spi_buf = {
		.buf = rx_buf,
		.len = 3U,
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx_spi_buf,
		.count   = 1U,
	};

	int ret = spi_transceive_dt(&hdrv->spi, &tx_set, &rx_set);
	return (ret == 0) ? DRV3946_OK : DRV3946_ERROR;
}

/* ==========================================================================
 * Public Driver Implementation
 * ========================================================================== */

DRV3946_Status_t DRV3946_Init(DRV3946_HandleTypeDef *hdrv)
{
	if (hdrv == NULL) {
		return DRV3946_ERROR;
	}

	/* Configure optional hardware GPIO pins if bound to DeviceTree */
	if (hdrv->en1_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&hdrv->en1_gpio) ||
		    gpio_pin_configure_dt(&hdrv->en1_gpio, GPIO_OUTPUT_INACTIVE) < 0) {
			LOG_ERR("Failed to configure DRV3946 ENABLE1 GPIO");
			return DRV3946_ERROR;
		}
	}

	if (hdrv->en2_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&hdrv->en2_gpio) ||
		    gpio_pin_configure_dt(&hdrv->en2_gpio, GPIO_OUTPUT_INACTIVE) < 0) {
			LOG_ERR("Failed to configure DRV3946 ENABLE2 GPIO");
			return DRV3946_ERROR;
		}
	}

	if (hdrv->nfault_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&hdrv->nfault_gpio) ||
		    gpio_pin_configure_dt(&hdrv->nfault_gpio, GPIO_INPUT) < 0) {
			LOG_ERR("Failed to configure DRV3946 nFAULT GPIO");
			return DRV3946_ERROR;
		}
	}

	/* Start with both contactors (AIR- and AIR+) in safe, OPEN state */
	DRV3946_EmergencyOpenAll(hdrv);

	/* Test SPI bus by reading STATUS1 (Silicon ID) */
	uint16_t status1 = 0U;
	if (DRV3946_ReadRegister(hdrv, DRV3946_REG_STATUS1, &status1) != DRV3946_OK) {
		return DRV3946_ERROR;
	}

	/* Clear any initial Power-On-Reset (POR) faults */
	if (DRV3946_ClearFaults(hdrv) != DRV3946_OK) {
		return DRV3946_ERROR;
	}
	k_msleep(1);

	/* Read initial status registers */
	if (DRV3946_UpdateStatus(hdrv) != DRV3946_OK) {
		return DRV3946_ERROR;
	}

	LOG_INF("DRV3946-Q1 initialized (%s mode, STATUS1=0x%04X)",
		(hdrv->spi.bus != NULL) ? "hardware SPI" : "unbound/stub",
		status1);

	return DRV3946_OK;
}

DRV3946_Status_t DRV3946_ReadRegister(DRV3946_HandleTypeDef *hdrv,
				      uint8_t reg_addr,
				      uint16_t *reg_data)
{
	uint8_t tx[3] = {0};
	uint8_t rx[3] = {0};

	/* Build Byte 0: [NAD: 7..6] | [RegAddr: 5..1] | [RW: 0] (1 for Read) */
	tx[0] = (uint8_t)(((hdrv->nad & 0x03U) << 6) |
			  ((reg_addr & 0x1FU) << 1) |
			  DRV3946_SPI_RW_READ);
	tx[1] = 0x00U;
	tx[2] = 0x00U;

	if (DRV3946_SpiTransfer(hdrv, tx, rx) != DRV3946_OK) {
		return DRV3946_ERROR;
	}

	if (reg_data != NULL) {
		*reg_data = ((uint16_t)rx[1] << 8) | rx[2];
	}

	return DRV3946_OK;
}

DRV3946_Status_t DRV3946_WriteRegister(DRV3946_HandleTypeDef *hdrv,
				       uint8_t reg_addr,
				       uint16_t reg_data)
{
	uint8_t tx[3] = {0};
	uint8_t rx[3] = {0};
	uint8_t addr_byte = (uint8_t)(((hdrv->nad & 0x03U) << 6) |
				      ((reg_addr & 0x1FU) << 1) |
				      DRV3946_SPI_RW_WRITE);

	/* Handle CMD registers (CMD0, CMD1, CMD2): CRC-8 replaces lower data byte */
	if (reg_addr >= DRV3946_REG_CMD0 && reg_addr <= DRV3946_REG_CMD2) {
		tx[0] = addr_byte;
		tx[1] = (uint8_t)(reg_data >> 8);
		tx[2] = DRV3946_ComputeCrcCmd(addr_byte, reg_data);
		return DRV3946_SpiTransfer(hdrv, tx, rx);
	}

	/* Handle CONFIGA block (A0..A6): Write data, then commit block CRC via CONFIGA6 */
	if (reg_addr >= DRV3946_REG_CONFIGA0 && reg_addr <= DRV3946_REG_CONFIGA6) {
		uint8_t idx = reg_addr - DRV3946_REG_CONFIGA0;
		hdrv->configA[idx] = reg_data;

		tx[0] = addr_byte;
		tx[1] = (uint8_t)(reg_data >> 8);
		tx[2] = (reg_addr == DRV3946_REG_CONFIGA6)
		      ? DRV3946_ComputeCrcConfigA(hdrv->configA)
		      : (uint8_t)(reg_data & 0xFFU);

		if (DRV3946_SpiTransfer(hdrv, tx, rx) != DRV3946_OK) {
			return DRV3946_ERROR;
		}

		/* If writing any register other than A6, automatically commit CRC to A6 */
		if (reg_addr != DRV3946_REG_CONFIGA6) {
			uint8_t crc_a = DRV3946_ComputeCrcConfigA(hdrv->configA);
			uint8_t a6_addr = (uint8_t)(((hdrv->nad & 0x03U) << 6) |
						    ((DRV3946_REG_CONFIGA6 & 0x1FU) << 1) |
						    DRV3946_SPI_RW_WRITE);
			tx[0] = a6_addr;
			tx[1] = (uint8_t)(hdrv->configA[6] >> 8);
			tx[2] = crc_a;
			return DRV3946_SpiTransfer(hdrv, tx, rx);
		}
		return DRV3946_OK;
	}

	/* Handle CONFIGB block (B0..B4): Write data, then commit block CRC via CONFIGB4 */
	if (reg_addr >= DRV3946_REG_CONFIGB0 && reg_addr <= DRV3946_REG_CONFIGB4) {
		uint8_t idx = reg_addr - DRV3946_REG_CONFIGB0;
		hdrv->configB[idx] = reg_data;

		tx[0] = addr_byte;
		tx[1] = (uint8_t)(reg_data >> 8);
		tx[2] = (reg_addr == DRV3946_REG_CONFIGB4)
		      ? DRV3946_ComputeCrcConfigB(hdrv->configB)
		      : (uint8_t)(reg_data & 0xFFU);

		if (DRV3946_SpiTransfer(hdrv, tx, rx) != DRV3946_OK) {
			return DRV3946_ERROR;
		}

		/* If writing any register other than B4, automatically commit CRC to B4 */
		if (reg_addr != DRV3946_REG_CONFIGB4) {
			uint8_t crc_b = DRV3946_ComputeCrcConfigB(hdrv->configB);
			uint8_t b4_addr = (uint8_t)(((hdrv->nad & 0x03U) << 6) |
						    ((DRV3946_REG_CONFIGB4 & 0x1FU) << 1) |
						    DRV3946_SPI_RW_WRITE);
			tx[0] = b4_addr;
			tx[1] = (uint8_t)(hdrv->configB[4] >> 8);
			tx[2] = crc_b;
			return DRV3946_SpiTransfer(hdrv, tx, rx);
		}
		return DRV3946_OK;
	}

	/* Standard unprotected register write */
	tx[0] = addr_byte;
	tx[1] = (uint8_t)(reg_data >> 8);
	tx[2] = (uint8_t)(reg_data & 0xFFU);
	return DRV3946_SpiTransfer(hdrv, tx, rx);
}

void DRV3946_SetContactorState(DRV3946_HandleTypeDef *hdrv,
			       DRV3946_Channel_t channel,
			       Contactor_State_t state)
{
	if (hdrv == NULL) {
		return;
	}

	int pin_val = (state == CONTACTOR_CLOSED) ? 1 : 0;

	if (channel & DRV3946_CHANNEL_1) {
		hdrv->ch1_state = state;
		if (hdrv->en1_gpio.port != NULL) {
			gpio_pin_set_dt(&hdrv->en1_gpio, pin_val);
		}
	}

	if (channel & DRV3946_CHANNEL_2) {
		hdrv->ch2_state = state;
		if (hdrv->en2_gpio.port != NULL) {
			gpio_pin_set_dt(&hdrv->en2_gpio, pin_val);
		}
	}
}

void DRV3946_EmergencyOpenAll(DRV3946_HandleTypeDef *hdrv)
{
	if (hdrv == NULL) {
		return;
	}

	hdrv->ch1_state = CONTACTOR_OPEN;
	hdrv->ch2_state = CONTACTOR_OPEN;

	if (hdrv->en1_gpio.port != NULL) {
		gpio_pin_set_dt(&hdrv->en1_gpio, 0);
	}
	if (hdrv->en2_gpio.port != NULL) {
		gpio_pin_set_dt(&hdrv->en2_gpio, 0);
	}
}

bool DRV3946_IsFaultAsserted(DRV3946_HandleTypeDef *hdrv)
{
	if (hdrv == NULL) {
		return false;
	}

	if (hdrv->nfault_gpio.port != NULL) {
		/*
		 * If nfault_gpio is configured with GPIO_ACTIVE_LOW in DeviceTree,
		 * gpio_pin_get_dt() returns 1 when the fault is active.
		 */
		hdrv->fault_active = (gpio_pin_get_dt(&hdrv->nfault_gpio) > 0);
	}

	return hdrv->fault_active;
}

DRV3946_Status_t DRV3946_UpdateStatus(DRV3946_HandleTypeDef *hdrv)
{
	static const uint8_t status_regs[6] = {
		DRV3946_REG_STATUS0, DRV3946_REG_STATUS1, DRV3946_REG_STATUS2,
		DRV3946_REG_STATUS3, DRV3946_REG_STATUS4, DRV3946_REG_STATUS5
	};

	for (int i = 0; i < 6; i++) {
		if (DRV3946_ReadRegister(hdrv, status_regs[i], &hdrv->last_status[i]) != DRV3946_OK) {
			return DRV3946_ERROR;
		}
	}

	return DRV3946_OK;
}

DRV3946_Status_t DRV3946_ClearFaults(DRV3946_HandleTypeDef *hdrv)
{
	if (hdrv == NULL) {
		return DRV3946_ERROR;
	}

	hdrv->fault_active = false;
	return DRV3946_WriteRegister(hdrv, DRV3946_REG_CMD1, DRV3946_CMD_CLEAR_FAULT);
}

DRV3946_Status_t DRV3946_ReadCurrentRaw(DRV3946_HandleTypeDef *hdrv,
					DRV3946_Channel_t ch,
					uint16_t *current_raw)
{
	uint8_t meas_reg = (ch == DRV3946_CHANNEL_1) ? DRV3946_REG_MEAS0 : DRV3946_REG_MEAS2;
	return DRV3946_ReadRegister(hdrv, meas_reg, current_raw);
}

DRV3946_Status_t DRV3946_ConfigureDCNLEV100(DRV3946_HandleTypeDef *hdrv,
					    DRV3946_Channel_t channel)
{
	/*
	 * Sizing for Littelfuse DCNLEV100 (12V Nominal, 26 Ohm coil):
	 * - Inrush/Pull-in: ~460 mA (Operate time = 25 ms)
	 * - Configured Peak Time: ~50 ms (margin over 25 ms mechanical operate time)
	 * - Configured Hold Current: ~180 mA - 200 mA (40% hold ratio, drops coil power to ~0.9 W)
	 * - Active freewheeling clamp enabled for rapid ~10 ms contact break
	 */
	DRV3946_Status_t status = DRV3946_OK;

	if (channel & DRV3946_CHANNEL_1) {
		/* Configure Channel 1 (AIR-) Peak & Hold in CONFIGA0 / CONFIGA1 */
		status = DRV3946_WriteRegister(hdrv, DRV3946_REG_CONFIGA0, 0x1E0BU);
		if (status != DRV3946_OK) {
			return status;
		}
		status = DRV3946_WriteRegister(hdrv, DRV3946_REG_CONFIGA1, 0x0A28U);
		if (status != DRV3946_OK) {
			return status;
		}
	}

	if (channel & DRV3946_CHANNEL_2) {
		/* Configure Channel 2 (AIR+) Peak & Hold in CONFIGA2 / CONFIGA3 */
		status = DRV3946_WriteRegister(hdrv, DRV3946_REG_CONFIGA2, 0x1E0BU);
		if (status != DRV3946_OK) {
			return status;
		}
		status = DRV3946_WriteRegister(hdrv, DRV3946_REG_CONFIGA3, 0x0A28U);
		if (status != DRV3946_OK) {
			return status;
		}
	}

	return DRV3946_OK;
}

