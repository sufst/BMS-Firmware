/**
 * @file bms_hv_sense.c
 * @brief AMC3330DWER Isolated High-Voltage Sensing Implementation.
 *
 * Handles both AMC3330 channels from the schematic:
 *   - U4 (accu):   Measures ACCU TS+ to ACCU TS- (before contactors)
 *   - U3 (dclink): Measures DC-LINK TS+ to DC-LINK TS- (after contactors / inverter side)
 *
 * Each channel validates its reading via the AMC3330 DIAG pin (HIGH = valid,
 * LOW = invalid at power-up or high-side fault) before converting the
 * differential analog output (OUTP - OUTN) into millivolts.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bms_hv_sense.h"

LOG_MODULE_REGISTER(bms_hv_sense, LOG_LEVEL_INF);

/* ==========================================================================
 * Scaling Helpers (Exact 64-bit Integer Math with Rounding)
 * ========================================================================== */

uint32_t amc3330_diff_mv_to_hv_mv(int32_t diff_mv)
{
	/* Clamp small negative ADC offset noise to 0 mV */
	if (diff_mv <= 0) {
		return 0U;
	}

	/*
	 * V_HV (mV) = diff_mv * (R_top + R_bot) / (Gain * R_bot)
	 *           = diff_mv * 1,690,430 / 4,860
	 */
	uint64_t num = ((uint64_t)diff_mv * AMC3330_SCALE_NUM) + (AMC3330_SCALE_DEN / 2ULL);
	return (uint32_t)(num / AMC3330_SCALE_DEN);
}

int32_t amc3330_hv_mv_to_diff_mv(uint32_t hv_mv)
{
	uint64_t num = ((uint64_t)hv_mv * AMC3330_SCALE_DEN) + (AMC3330_SCALE_NUM / 2ULL);
	return (int32_t)(num / AMC3330_SCALE_NUM);
}

/* ==========================================================================
 * Per-Channel Hardware Helpers
 * ========================================================================== */

static int init_single_channel(struct amc3330_chan *ch, const char *name)
{
	/* 1. Configure DIAG input pin if bound in DeviceTree */
	if (ch->diag_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&ch->diag_gpio)) {
			LOG_ERR("AMC3330 %s DIAG GPIO device not ready", name);
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(&ch->diag_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure AMC3330 %s DIAG GPIO (%d)", name, ret);
			return ret;
		}
	}

	/* 2. Configure differential ADC channel if bound in DeviceTree */
	if (ch->adc_ch.dev != NULL) {
		if (!adc_is_ready_dt(&ch->adc_ch)) {
			LOG_ERR("AMC3330 %s ADC device not ready", name);
			return -ENODEV;
		}
		int ret = adc_channel_setup_dt(&ch->adc_ch);
		if (ret < 0) {
			LOG_ERR("Failed to setup AMC3330 %s ADC channel (%d)", name, ret);
			return ret;
		}
	}

	LOG_INF("AMC3330 %s channel initialized (%s mode)",
		name,
		(ch->adc_ch.dev != NULL) ? "hardware ADC" : "unbound/mock");

	return 0;
}

static int sample_single_channel(struct amc3330_chan *ch)
{
	/* 1. Validate DIAG pin first (HIGH = valid, LOW = invalid / power-up) */
	if (ch->diag_gpio.port != NULL) {
		ch->diag_valid = (gpio_pin_get_dt(&ch->diag_gpio) > 0);
	} else {
		ch->diag_valid = ch->mock_diag_ok;
	}

	/*
	 * If DIAG is LOW (e.g. AMC3330 isolated DC/DC still powering up or
	 * high-side fault), do not trust the analog output.
	 */
	if (!ch->diag_valid) {
		ch->diff_mv = 0;
		ch->hv_mv   = 0U;
		return 0;
	}

	/* 2. Read differential analog output (OUTP - OUTN) */
	if (ch->adc_ch.dev != NULL) {
		int16_t raw_sample = 0;
		struct adc_sequence seq = {
			.buffer      = &raw_sample,
			.buffer_size = sizeof(raw_sample),
		};

		adc_sequence_init_dt(&ch->adc_ch, &seq);

		int ret = adc_read_dt(&ch->adc_ch, &seq);
		if (ret < 0) {
			ch->diag_valid = false;
			ch->hv_mv      = 0U;
			return ret;
		}

		int32_t val_mv = (int32_t)raw_sample;
		ret = adc_raw_to_millivolts_dt(&ch->adc_ch, &val_mv);
		if (ret < 0) {
			ch->diag_valid = false;
			ch->hv_mv      = 0U;
			return ret;
		}

		ch->diff_mv = val_mv;
	} else {
		ch->diff_mv = ch->mock_diff_mv;
	}

	/* 3. Convert differential mV to High-Voltage mV via divider ratio */
	ch->hv_mv = amc3330_diff_mv_to_hv_mv(ch->diff_mv);
	return 0;
}

/* ==========================================================================
 * Public API Implementation
 * ========================================================================== */

int bms_hv_sense_init(struct bms_hv_sense *hv)
{
	if (hv == NULL) {
		return -EINVAL;
	}

	int ret = init_single_channel(&hv->accu, "ACCU (U4)");
	if (ret < 0) {
		return ret;
	}

	ret = init_single_channel(&hv->dclink, "DC-LINK (U3)");
	if (ret < 0) {
		return ret;
	}

	return 0;
}

int bms_hv_sense_sample(struct bms_hv_sense *hv)
{
	if (hv == NULL) {
		return -EINVAL;
	}

	int ret_accu   = sample_single_channel(&hv->accu);
	int ret_dclink = sample_single_channel(&hv->dclink);

	if (ret_accu < 0) {
		return ret_accu;
	}
	return ret_dclink;
}
