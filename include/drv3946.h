/**
 * @file drv3946.h
 * @brief TI DRV3946-Q1 Dual-Channel Contactor / Solenoid Driver for Zephyr RTOS.
 *
 * Ported from SUFST STM32 DRV3946-Q1 driver to Zephyr RTOS.
 * Hardware-agnostic: uses Zephyr's generic spi_dt_spec and gpio_dt_spec pointers,
 * so it does not depend on any specific SPI peripheral (SPI1/SPI2/SPI3) or pinout.
 * If no SPI bus / GPIO pins are attached yet (bus == NULL / port == NULL), the
 * driver safely operates in unbound/stubbed hardware mode until PCB bring-up.
 */

#ifndef SUFST_DRV3946_H_
#define SUFST_DRV3946_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * DRV3946-Q1 SPI Register Map Definitions
 * ========================================================================== */

#define DRV3946_REG_STATUS0        0x01U
#define DRV3946_REG_STATUS1        0x02U
#define DRV3946_REG_STATUS2        0x03U
#define DRV3946_REG_STATUS3        0x04U
#define DRV3946_REG_STATUS4        0x0AU
#define DRV3946_REG_STATUS5        0x0BU

#define DRV3946_REG_MEAS0          0x05U
#define DRV3946_REG_MEAS1          0x06U
#define DRV3946_REG_MEAS2          0x07U
#define DRV3946_REG_MEAS3          0x08U
#define DRV3946_REG_MEAS4          0x09U
#define DRV3946_REG_MEAS5          0x0CU
#define DRV3946_REG_MEAS6          0x0DU

#define DRV3946_REG_CONFIGA0       0x10U
#define DRV3946_REG_CONFIGA1       0x11U
#define DRV3946_REG_CONFIGA2       0x12U
#define DRV3946_REG_CONFIGA3       0x13U
#define DRV3946_REG_CONFIGA4       0x14U
#define DRV3946_REG_CONFIGA5       0x15U
#define DRV3946_REG_CONFIGA6       0x16U

#define DRV3946_REG_CONFIGB0       0x17U
#define DRV3946_REG_CONFIGB1       0x18U
#define DRV3946_REG_CONFIGB2       0x19U
#define DRV3946_REG_CONFIGB3       0x1AU
#define DRV3946_REG_CONFIGB4       0x1BU

#define DRV3946_REG_CMD0           0x1CU
#define DRV3946_REG_CMD1           0x1DU
#define DRV3946_REG_CMD2           0x1EU

/* ==========================================================================
 * Bitfields & Constants
 * ========================================================================== */

#define DRV3946_SPI_RW_READ        0x01U
#define DRV3946_SPI_RW_WRITE       0x00U
#define DRV3946_CMD_CLEAR_FAULT    0x8000U
#define DRV3946_CMD_BIST_RUN       0x4000U

typedef enum {
	DRV3946_OK      = 0x00U,
	DRV3946_ERROR   = 0x01U,
	DRV3946_BUSY    = 0x02U,
	DRV3946_TIMEOUT = 0x03U,
	DRV3946_CRC_ERR = 0x04U
} DRV3946_Status_t;

typedef enum {
	DRV3946_CHANNEL_1    = 0x01U, /**< Channel 1: Negative AIR (AIR-) */
	DRV3946_CHANNEL_2    = 0x02U, /**< Channel 2: Positive AIR (AIR+) */
	DRV3946_CHANNEL_BOTH = 0x03U
} DRV3946_Channel_t;

typedef enum {
	CONTACTOR_OPEN   = 0x00U,
	CONTACTOR_CLOSED = 0x01U
} Contactor_State_t;

/**
 * @brief DRV3946-Q1 Device Handle Struct (Zephyr hardware-agnostic)
 *
 * Note: Any SPI bus (SPI1, SPI2, SPI3, etc.) and any GPIO pins can be bound here
 * once the MCU pinout is finalized. If left zero-initialized (spi.bus == NULL),
 * the driver runs safely without touching physical hardware.
 */
typedef struct {
	/* Generic Zephyr SPI & GPIO descriptors (optional until pinout is finalized) */
	struct spi_dt_spec  spi;         /**< Generic SPI bus + CS spec (any SPI peripheral) */
	struct gpio_dt_spec en1_gpio;    /**< ENABLE1 GPIO (Channel 1 -> AIR-) */
	struct gpio_dt_spec en2_gpio;    /**< ENABLE2 GPIO (Channel 2 -> AIR+) */
	struct gpio_dt_spec nfault_gpio; /**< nFAULT GPIO (Input, active-low fault from DRV3946) */

	uint8_t             nad;         /**< Node Address (0..3) for addressable SPI */

	/* Local Shadow Memory of Configuration Registers */
	uint16_t            configA[7];  /**< CONFIGA0 .. CONFIGA6 */
	uint16_t            configB[5];  /**< CONFIGB0 .. CONFIGB4 */

	/* Cached Status & Output State */
	uint16_t            last_status[6]; /**< STATUS0 .. STATUS5 */
	Contactor_State_t   ch1_state;      /**< Current commanded state of Channel 1 (AIR-) */
	Contactor_State_t   ch2_state;      /**< Current commanded state of Channel 2 (AIR+) */
	bool                fault_active;   /**< Cached fault state from nFAULT pin / status */
} DRV3946_HandleTypeDef;

/* ==========================================================================
 * Driver Public API
 * ========================================================================== */

/**
 * @brief Initialize the DRV3946 handle, configure GPIOs (if bound),
 *        validate the SPI link, clear POR faults, and read initial status.
 */
DRV3946_Status_t DRV3946_Init(DRV3946_HandleTypeDef *hdrv);

/**
 * @brief Raw 24-bit SPI register read.
 */
DRV3946_Status_t DRV3946_ReadRegister(DRV3946_HandleTypeDef *hdrv,
				      uint8_t reg_addr,
				      uint16_t *reg_data);

/**
 * @brief Raw 24-bit SPI register write with automatic CRC-8 (0x2F) calculation
 *        for CMDx, CONFIGA, and CONFIGB register blocks.
 */
DRV3946_Status_t DRV3946_WriteRegister(DRV3946_HandleTypeDef *hdrv,
				       uint8_t reg_addr,
				       uint16_t reg_data);

/**
 * @brief Control contactor state (Open/Close) using hardware ENABLE1/ENABLE2 pins.
 */
void DRV3946_SetContactorState(DRV3946_HandleTypeDef *hdrv,
			       DRV3946_Channel_t channel,
			       Contactor_State_t state);

/**
 * @brief Immediately force both DRV3946 channels OPEN (ENABLE1 = 0, ENABLE2 = 0).
 */
void DRV3946_EmergencyOpenAll(DRV3946_HandleTypeDef *hdrv);

/**
 * @brief Poll the hardware nFAULT pin (active low) and update cached fault state.
 * @return true if DRV3946 nFAULT is asserted, false otherwise.
 */
bool DRV3946_IsFaultAsserted(DRV3946_HandleTypeDef *hdrv);

/**
 * @brief Read all STATUS registers (STATUS0..STATUS5) over SPI.
 */
DRV3946_Status_t DRV3946_UpdateStatus(DRV3946_HandleTypeDef *hdrv);

/**
 * @brief Clear latched faults on the DRV3946 using CMD1 (0x8000) with CRC-8.
 */
DRV3946_Status_t DRV3946_ClearFaults(DRV3946_HandleTypeDef *hdrv);

/**
 * @brief Read raw 10-bit coil current measurement from MEAS0 (Ch1) or MEAS2 (Ch2).
 */
DRV3946_Status_t DRV3946_ReadCurrentRaw(DRV3946_HandleTypeDef *hdrv,
					DRV3946_Channel_t ch,
					uint16_t *current_raw);

/**
 * @brief Configure peak-and-hold profile and fast demagnetization timings
 *        tailored for Littelfuse DCNLEV100 contactors (~460 mA peak inrush for
 *        50 ms, ~180 mA hold current, and active freewheeling clamp).
 */
DRV3946_Status_t DRV3946_ConfigureDCNLEV100(DRV3946_HandleTypeDef *hdrv,
					    DRV3946_Channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* SUFST_DRV3946_H_ */

