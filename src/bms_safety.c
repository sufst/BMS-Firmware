/**
 * @file bms_safety.c
 * @brief SUFST BMS Safety & Fault Monitor with Formula Student EV debouncing.
 *
 * Evaluates all accumulator safety limits every cycle:
 *   - Cell Over-Voltage (OV) & Under-Voltage (UV)      [100 ms debounce]
 *   - Cell Over-Temperature (> 60 C) & Under-Temp      [500 ms debounce]
 *   - Discharge / Regen Over-Current (asymmetric)      [100 ms debounce]
 *   - Charging / Regen Below Min Charge Temperature    [500 ms debounce]
 *   - Cell / Thermistor Open-Wire                      [100 ms debounce]
 *   - Sensor Implausibility (out-of-range / V mismatch)[100 ms debounce]
 *   - AFE Communication Heartbeat Timeout              [250 ms timeout]
 *   - IVT-S Current Sensor Timeout / Reported Error    [100 ms]
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bms.h"

LOG_MODULE_REGISTER(bms_safety, LOG_LEVEL_INF);

/* ==========================================================================
 * Internal Debounce Helper
 * ========================================================================== */

/**
 * @brief Update a single fault debounce timer and set/clear its live fault bit.
 *
 * @param condition_violated true if the safety threshold is currently exceeded.
 * @param timer_ms           Pointer to the accumulated violation time (ms).
 * @param debounce_limit_ms  Required duration (ms) before the fault trips.
 * @param dt_ms              Elapsed time since last check (ms).
 * @param fault_bit          The BMS_FAULT_* bit corresponding to this check.
 * @param live_faults        Pointer to the active live_faults bitmask.
 */
static void evaluate_debounced_fault(bool condition_violated,
				     uint32_t *timer_ms,
				     uint32_t debounce_limit_ms,
				     uint32_t dt_ms,
				     uint32_t fault_bit,
				     uint32_t *live_faults)
{
	if (condition_violated) {
		if (*timer_ms < debounce_limit_ms) {
			*timer_ms += dt_ms;
		}
		if (*timer_ms >= debounce_limit_ms) {
			*live_faults |= fault_bit;
		}
	} else {
		/* Condition returned to safe range: reset timer and clear live bit */
		*timer_ms = 0U;
		*live_faults &= ~fault_bit;
	}
}

/**
 * @brief Check if sensor readings are physically implausible (AMC3330 DIAG low,
 *        broken wire, shorted NTC, or > 5V mismatch between sum-of-cells and
 *        AMC3330 accumulator HV sensor).
 */
static bool is_sensor_implausible(const struct bms_ctx *ctx)
{
	/* 0. AMC3330 DIAG pin low on either U4 (ACCU) or U3 (DC-LINK) */
	if (!ctx->hv_sense_valid) {
		return true;
	}

	/* 1. Cell voltage outside physical Li-ion sense range (0.5V .. 5.0V) */
	if (ctx->max_cell_mv > BMS_CELL_IMPLAUSIBLE_HIGH_MV ||
	    ctx->min_cell_mv < BMS_CELL_IMPLAUSIBLE_LOW_MV) {
		return true;
	}

	/* 2. Thermistor reading outside physical range (-40 C .. +120 C) */
	if (ctx->max_temp_deci_c > BMS_TEMP_IMPLAUSIBLE_HIGH_DECI_C ||
	    ctx->min_temp_deci_c < BMS_TEMP_IMPLAUSIBLE_LOW_DECI_C) {
		return true;
	}

	/* 3. Sum of cells vs. AMC3330 HV accumulator sensor mismatch > 5.0 V */
	uint32_t diff_mv = (ctx->sum_cells_mv > ctx->pack_voltage_mv)
			 ? (ctx->sum_cells_mv - ctx->pack_voltage_mv)
			 : (ctx->pack_voltage_mv - ctx->sum_cells_mv);

	if (diff_mv > BMS_PACK_SUM_MAX_MISMATCH_MV) {
		return true;
	}

	return false;
}

/* ==========================================================================
 * Regen Limit Derating
 * ========================================================================== */

/**
 * @brief Linear derating factor in permille (0..1000) for one measurement.
 *
 * Returns 1000 at or beyond @p full_at, 0 at or beyond @p zero_at, and a
 * linear ramp in between. Works for either direction (full_at above or
 * below zero_at).
 */
static uint32_t derate_permille(int32_t value, int32_t full_at, int32_t zero_at)
{
	if (full_at > zero_at) {
		/* Mirror so that the ramp always falls as the value rises */
		value   = -value;
		full_at = -full_at;
		zero_at = -zero_at;
	}

	if (value <= full_at) {
		return 1000U;
	}
	if (value >= zero_at) {
		return 0U;
	}

	return (uint32_t)(((zero_at - value) * 1000) / (zero_at - full_at));
}

uint32_t bms_regen_limit_ma(const struct bms_ctx *ctx)
{
	if (ctx->state != BMS_STATE_ACTIVE ||
	    ctx->live_faults != BMS_FAULT_NONE ||
	    ctx->latched_faults != BMS_FAULT_NONE) {
		return 0U;
	}

	uint32_t factor = derate_permille(ctx->max_cell_mv,
					  BMS_REGEN_FULL_CELL_MV,
					  BMS_REGEN_ZERO_CELL_MV);

	factor = MIN(factor, derate_permille(ctx->min_temp_deci_c,
					     BMS_REGEN_FULL_COLD_DECI_C,
					     BMS_REGEN_ZERO_COLD_DECI_C));

	factor = MIN(factor, derate_permille(ctx->max_temp_deci_c,
					     BMS_REGEN_FULL_HOT_DECI_C,
					     BMS_REGEN_ZERO_HOT_DECI_C));

	return ((uint32_t)BMS_CURRENT_MAX_REGEN_MA * factor) / 1000U;
}

static void update_regen_outputs(struct bms_ctx *ctx)
{
	ctx->out.max_regen_current_ma = bms_regen_limit_ma(ctx);
	ctx->out.regen_allowed        = (ctx->out.max_regen_current_ma > 0U);
}

/* ==========================================================================
 * Public API Implementation
 * ========================================================================== */

void bms_safety_check(struct bms_ctx *ctx, uint32_t dt_ms)
{
	/*
	 * Wait until we have left BMS_STATE_INIT (AFE initialized & AMC3330
	 * isolated DC/DC converters powered up with DIAG = HIGH).
	 */
	if (ctx->state == BMS_STATE_INIT || !ctx->afe_init_ok) {
		update_regen_outputs(ctx);
		return;
	}

	uint32_t prev_live = ctx->live_faults;

	/* 1. Cell Over-Voltage (> 4.200 V for >= 100 ms) */
	evaluate_debounced_fault(
		(ctx->max_cell_mv > BMS_CELL_MAX_MV),
		&ctx->fault_timers.cell_ov_ms,
		BMS_DEBOUNCE_VOLTAGE_MS,
		dt_ms,
		BMS_FAULT_CELL_OV,
		&ctx->live_faults);

	/* 2. Cell Under-Voltage (< 2.500 V for >= 100 ms) */
	evaluate_debounced_fault(
		(ctx->min_cell_mv < BMS_CELL_MIN_MV),
		&ctx->fault_timers.cell_uv_ms,
		BMS_DEBOUNCE_VOLTAGE_MS,
		dt_ms,
		BMS_FAULT_CELL_UV,
		&ctx->live_faults);

	/* 3. Cell Over-Temperature (> 60.0 C for >= 500 ms) */
	evaluate_debounced_fault(
		(ctx->max_temp_deci_c > BMS_TEMP_MAX_DECI_C),
		&ctx->fault_timers.over_temp_ms,
		BMS_DEBOUNCE_TEMP_MS,
		dt_ms,
		BMS_FAULT_OVER_TEMP,
		&ctx->live_faults);

	/* 4. Cell Under-Temperature (< -20.0 C for >= 500 ms) */
	evaluate_debounced_fault(
		(ctx->min_temp_deci_c < BMS_TEMP_MIN_DECI_C),
		&ctx->fault_timers.under_temp_ms,
		BMS_DEBOUNCE_TEMP_MS,
		dt_ms,
		BMS_FAULT_UNDER_TEMP,
		&ctx->live_faults);

	/* 5a. Discharge Over-Current (I_pack > +I_max_discharge for >= 100 ms) */
	evaluate_debounced_fault(
		(ctx->pack_current_ma > BMS_CURRENT_MAX_DISCHARGE_MA),
		&ctx->fault_timers.over_current_ms,
		BMS_DEBOUNCE_CURRENT_MS,
		dt_ms,
		BMS_FAULT_OVER_CURRENT,
		&ctx->live_faults);

	/* 5b. Regen / Charge Over-Current (I_pack < -I_max_regen for >= 100 ms) */
	evaluate_debounced_fault(
		(ctx->pack_current_ma < -BMS_CURRENT_MAX_REGEN_MA),
		&ctx->fault_timers.regen_over_current_ms,
		BMS_DEBOUNCE_CURRENT_MS,
		dt_ms,
		BMS_FAULT_REGEN_OVER_CURRENT,
		&ctx->live_faults);

	/* 5c. Charging / regen while coldest cell is below 0.0 C (>= 500 ms) */
	evaluate_debounced_fault(
		(ctx->pack_current_ma < -BMS_CHARGE_DETECT_MA &&
		 ctx->min_temp_deci_c < BMS_TEMP_CHARGE_MIN_DECI_C),
		&ctx->fault_timers.charge_under_temp_ms,
		BMS_DEBOUNCE_CHARGE_TEMP_MS,
		dt_ms,
		BMS_FAULT_CHARGE_UNDER_TEMP,
		&ctx->live_faults);

	/* 5d. IVT-S current sensor: reported error (>= 100 ms) or frame timeout (> 100 ms) */
	const struct ivt_s_reading *ivt_i = &ctx->ivt.ch[IVT_S_CH_I];

	evaluate_debounced_fault(
		((ivt_i->state & (IVT_S_STATE_RESULT_ERROR | IVT_S_STATE_SYSTEM_ERROR)) != 0U),
		&ctx->fault_timers.current_sensor_ms,
		BMS_DEBOUNCE_CURRENT_SENSOR_MS,
		dt_ms,
		BMS_FAULT_CURRENT_SENSOR,
		&ctx->live_faults);

	if (!ivt_i->seen ||
	    (k_uptime_get() - ivt_i->rx_ms) > (int64_t)BMS_CURRENT_SENSOR_TIMEOUT_MS) {
		ctx->live_faults |= BMS_FAULT_CURRENT_SENSOR;
	}

	/* 6. Sense Wire / Thermistor Open-Wire (>= 100 ms) */
	evaluate_debounced_fault(
		ctx->afe_open_wire,
		&ctx->fault_timers.open_wire_ms,
		BMS_DEBOUNCE_OPEN_WIRE_MS,
		dt_ms,
		BMS_FAULT_OPEN_WIRE,
		&ctx->live_faults);

	/* 7. Sensor Implausibility (>= 100 ms) */
	evaluate_debounced_fault(
		is_sensor_implausible(ctx),
		&ctx->fault_timers.implausibility_ms,
		BMS_DEBOUNCE_IMPLAUSIBLE_MS,
		dt_ms,
		BMS_FAULT_IMPLAUSIBILITY,
		&ctx->live_faults);

	/* 8. AFE Communication Heartbeat Timeout (> 250 ms since last valid sample) */
	int64_t since_afe_rx_ms = k_uptime_get() - ctx->last_afe_rx_ms;
	if (since_afe_rx_ms > (int64_t)BMS_AFE_COMM_TIMEOUT_MS) {
		ctx->live_faults |= BMS_FAULT_AFE_COMM;
	} else {
		ctx->live_faults &= ~BMS_FAULT_AFE_COMM;
	}

	/* Log if any new live fault bit just tripped or cleared */
	if (ctx->live_faults != prev_live) {
		LOG_WRN("Live faults changed: 0x%08X -> 0x%08X",
			prev_live, ctx->live_faults);
	}

	/* Immediately latch and trigger sequenced shutdown if any fault is active */
	if (ctx->live_faults != BMS_FAULT_NONE) {
		bms_sm_raise_fault(ctx, ctx->live_faults);
	}

	/* Refresh the advisory regen limit last so a fault raised above zeroes it */
	update_regen_outputs(ctx);
}
