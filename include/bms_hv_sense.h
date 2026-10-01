/**
 * @file bms_hv_sense.h
 * @brief AMC3330DWER Isolated High-Voltage Sensing Driver (Accumulator & DC-Link).
 *
 * Schematic Parameters (U3 = DC-Link TS, U4 = Accumulator TS):
 *   - High-side top divider:    4 x 422 kOhm in series = 1,688,000 Ohm
 *     (U3: R13+R14+R16+R18, U4: R22+R23+R25+R27)
 *   - High-side sense resistor: 2.43 kOhm = 2,430 Ohm
 *     (U3: R20, U4: R29)
 *   - AMC3330 nominal gain:     G = 2.0 V/V
 *   - Output filter:            2 x 100 Ohm + 1 nF differential RC filter
 *   - DIAG output (Pin 14):     Open-drain with 47 kOhm pull-up to +3V3
 *                               HIGH (1) = Valid reading
 *                               LOW  (0) = Invalid reading (startup / fault)
 *
 * Transfer Function:
 *   V_diff = V_OUTP - V_OUTN = 2.0 * V_HV * (R_bot / (R_top + R_bot))
 *   V_HV   = V_diff * (R_top + R_bot) / (2.0 * R_bot)
 *          = V_diff * (1,688,000 + 2,430) / (2 * 2,430)
 *          = V_diff * 1,690,430 / 4,860
 *          ~= V_diff * 347.8251
 */

#ifndef SUFST_BMS_HV_SENSE_H_
#define SUFST_BMS_HV_SENSE_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Schematic Resistor & AMC3330 Constants
 * ========================================================================== */

/** Top series divider resistance: 4 * 422 kOhm = 1,688,000 Ohm */
#define AMC3330_R_TOP_OHM          (4ULL * 422000ULL)

/** Bottom sense resistance: 2.43 kOhm = 2,430 Ohm */
#define AMC3330_R_BOT_OHM          (2430ULL)

/** AMC3330 fixed differential voltage gain (2 V/V) */
#define AMC3330_GAIN_NUM           (2ULL)

/** Numerator for V_diff -> V_HV conversion: R_top + R_bot = 1,690,430 */
#define AMC3330_SCALE_NUM          (AMC3330_R_TOP_OHM + AMC3330_R_BOT_OHM)

/** Denominator for V_diff -> V_HV conversion: Gain * R_bot = 4,860 */
#define AMC3330_SCALE_DEN          (AMC3330_GAIN_NUM * AMC3330_R_BOT_OHM)

/* ==========================================================================
 * Data Structures
 * ========================================================================== */

/**
 * @brief Single AMC3330DWER isolated voltage sensor channel descriptor.
 *
 * Pinout-independent: if adc_ch.dev == NULL or diag_gpio.port == NULL
 * (before MCU pinout is finalized), the driver uses simulated/mock values.
 */
struct amc3330_chan {
	struct adc_dt_spec  adc_ch;        /**< Differential STM32 ADC channel (OUTP - OUTN) */
	struct gpio_dt_spec diag_gpio;     /**< DIAG input pin (High = Valid, Low = Invalid) */

	/* Mock / fallback values used when hardware pins are not yet bound */
	int32_t             mock_diff_mv;  /**< Simulated differential output (V_OUTP - V_OUTN) in mV */
	bool                mock_diag_ok;  /**< Simulated DIAG state when diag_gpio.port == NULL */

	/* Latest sampled outputs */
	int32_t             diff_mv;       /**< Measured differential output (V_OUTP - V_OUTN) in mV */
	uint32_t            hv_mv;         /**< Calculated high-side voltage in mV */
	bool                diag_valid;    /**< True if DIAG pin is HIGH (valid reading) */
};

/**
 * @brief Dual AMC3330 HV sensing context (U4 Accumulator & U3 DC-Link).
 */
struct bms_hv_sense {
	struct amc3330_chan accu;          /**< U4: Accumulator side (ACCU TS+ / ACCU TS-) */
	struct amc3330_chan dclink;        /**< U3: Tractive System / Inverter side (DC-LINK TS+ / TS-) */
};

/* ==========================================================================
 * Public API
 * ========================================================================== */

/**
 * @brief Initialize both AMC3330 channels (ADC channels & DIAG GPIOs if bound).
 *
 * @param hv Pointer to HV sensing structure.
 * @return 0 on success, negative errno on hardware configuration failure.
 */
int bms_hv_sense_init(struct bms_hv_sense *hv);

/**
 * @brief Sample DIAG pins and differential ADC channels for both U3 (DC-Link)
 *        and U4 (Accumulator), and convert to high-voltage millivolts.
 *
 * @param hv Pointer to HV sensing structure.
 * @return 0 on success, negative errno on ADC read error.
 */
int bms_hv_sense_sample(struct bms_hv_sense *hv);

/**
 * @brief Convert AMC3330 differential output voltage (V_OUTP - V_OUTN in mV)
 *        to high-side bus voltage (in mV) using the schematic divider ratio.
 *
 * @param diff_mv Differential voltage (V_OUTP - V_OUTN) in millivolts.
 * @return High-side voltage in millivolts (clamped to >= 0 mV).
 */
uint32_t amc3330_diff_mv_to_hv_mv(int32_t diff_mv);

/**
 * @brief Convert a high-side bus voltage (in mV) to the expected AMC3330
 *        differential output voltage (V_OUTP - V_OUTN in mV). Useful for testing.
 */
int32_t amc3330_hv_mv_to_diff_mv(uint32_t hv_mv);

#ifdef __cplusplus
}
#endif

#endif /* SUFST_BMS_HV_SENSE_H_ */
