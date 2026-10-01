/**
 * @file bms_sm.c
 * @brief SUFST BMS State Machine (Table-Driven C Implementation).
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bms.h"

LOG_MODULE_REGISTER(bms_sm, LOG_LEVEL_INF);

/* ==========================================================================
 * State Truth Table (Name & Moore Relay Outputs per State)
 * ========================================================================== */

struct state_cfg {
	const char *name;
	bool precharge;
	bool air_neg;
	bool air_pos;
	bool ts_active;
	bool balancing;
};

static const struct state_cfg state_table[] = {
	/*                                  Name                   PC     AIR-   AIR+   TS_ACT BAL   */
	[BMS_STATE_INIT]                = { "INIT",                false, false, false, false, false },
	[BMS_STATE_STANDBY]             = { "STANDBY",             false, false, false, false, false },
	[BMS_STATE_PRECHARGE_CLOSE_PC]  = { "PRECHARGE_CLOSE_PC",  true,  false, false, false, false },
	[BMS_STATE_PRECHARGE_WAIT]      = { "PRECHARGE_WAIT",      true,  true,  false, false, false },
	[BMS_STATE_PRECHARGE_CLOSE_POS] = { "PRECHARGE_CLOSE_POS", true,  true,  true,  false, false },
	[BMS_STATE_PRECHARGE_OPEN_NEG]  = { "PRECHARGE_OPEN_NEG",  true,  false, false, false, false },
	[BMS_STATE_ACTIVE]              = { "ACTIVE",              false, true,  true,  true,  false },
	[BMS_STATE_CHARGING]            = { "CHARGING",            false, true,  true,  true,  true  },
	[BMS_STATE_FAULT]               = { "FAULT",               false, false, false, false, false },
};

const char *bms_state_to_str(bms_state_t state)
{
	if ((size_t)state < ARRAY_SIZE(state_table) && state_table[state].name != NULL) {
		return state_table[state].name;
	}
	return "UNKNOWN";
}

/* ==========================================================================
 * Hardware & NVM Hooks
 * ========================================================================== */

static void sample_hardware_inputs(struct bms_ctx *ctx)
{
	/* Sample AMC3330 isolated HV sensors (U4 ACCU & U3 DC-LINK) + DIAG pins */
	bms_hv_sense_sample(&ctx->hv_sense);
	ctx->hv_sense_valid  = ctx->hv_sense.accu.diag_valid &&
			       ctx->hv_sense.dclink.diag_valid;
	ctx->pack_voltage_mv = ctx->hv_sense.accu.hv_mv;
	ctx->ts_voltage_mv   = ctx->hv_sense.dclink.hv_mv;

	/* TODO (Hardware bring-up): read sdc_sense_gpio and ts_request_gpio */
	ctx->req.ts_request = ctx->req.ts_request_can || ctx->req.ts_request_gpio;
}

static void apply_hardware_outputs(struct bms_ctx *ctx)
{
	const struct bms_outputs *out = &ctx->out;

	/* Drive AIR- (Ch1) and AIR+ (Ch2) via DRV3946-Q1 */
	if (!out->air_neg_closed && !out->air_pos_closed) {
		DRV3946_EmergencyOpenAll(&ctx->drv_airs);
	} else {
		DRV3946_SetContactorState(&ctx->drv_airs, DRV3946_CHANNEL_1,
			out->air_neg_closed ? CONTACTOR_CLOSED : CONTACTOR_OPEN);
		DRV3946_SetContactorState(&ctx->drv_airs, DRV3946_CHANNEL_2,
			out->air_pos_closed ? CONTACTOR_CLOSED : CONTACTOR_OPEN);
	}

	/* TODO (Hardware bring-up): set Precharge, SDC_FAULT, TS_READY, TS_ACTIVE GPIOs & CAN */
	LOG_DBG("Outputs: TS_READY=%d, TS_ACTIVE=%d, PC=%d, AIR-=%d, AIR+=%d, SDC_FAULT=%d, BAL=%d",
		out->ts_ready, out->ts_active, out->precharge_closed,
		out->air_neg_closed, out->air_pos_closed,
		out->sdc_fault_pin_active, out->balancing_enabled);
}

static uint32_t nvm_load_latched_faults(void)
{
	/* TODO (NVS integration): Read persistent fault mask from storage_partition */
	return BMS_FAULT_NONE;
}

static void nvm_save_latched_faults(uint32_t fault_mask)
{
	/* TODO (NVS integration): Write persistent fault mask to storage_partition */
	LOG_WRN("NVM: Saved persistent fault mask 0x%08X to Flash", fault_mask);
}

/* ==========================================================================
 * Internal Helpers
 * ========================================================================== */

bool bms_sm_is_ts_ready(const struct bms_ctx *ctx)
{
	return ctx->afe_init_ok &&
	       ctx->hv_sense_valid &&
	       (ctx->live_faults == BMS_FAULT_NONE) &&
	       (ctx->latched_faults == BMS_FAULT_NONE) &&
	       (ctx->state != BMS_STATE_INIT) &&
	       (ctx->state != BMS_STATE_FAULT) &&
	       ctx->sdc_airs_powered;
}

static void update_outputs_for_state(struct bms_ctx *ctx)
{
	const struct state_cfg *cfg = &state_table[ctx->state];

	ctx->out.precharge_closed     = cfg->precharge;
	ctx->out.air_neg_closed       = cfg->air_neg;
	ctx->out.air_pos_closed       = cfg->air_pos;
	ctx->out.ts_active            = cfg->ts_active;
	ctx->out.balancing_enabled    = cfg->balancing;
	ctx->out.ts_ready             = bms_sm_is_ts_ready(ctx);
	ctx->out.sdc_fault_pin_active = (ctx->state == BMS_STATE_FAULT) ||
					(ctx->latched_faults != BMS_FAULT_NONE);

	apply_hardware_outputs(ctx);
}

static void transition_to(struct bms_ctx *ctx, bms_state_t next_state)
{
	if (ctx->state == next_state) {
		return;
	}

	LOG_INF("State transition: %s -> %s",
		bms_state_to_str(ctx->state),
		bms_state_to_str(next_state));

	ctx->state = next_state;
	ctx->state_entry_time_ms = k_uptime_get();
	update_outputs_for_state(ctx);
}

static bool is_precharge_voltage_reached(const struct bms_ctx *ctx)
{
	if (!ctx->hv_sense_valid || ctx->pack_voltage_mv < 10000U) {
		return false;
	}

	uint32_t target_mv = (ctx->pack_voltage_mv * BMS_PRECHARGE_THRESHOLD_PCT) / 100U;
	return (ctx->ts_voltage_mv >= target_mv);
}

/* ==========================================================================
 * Public API Implementation
 * ========================================================================== */

void bms_sm_init(struct bms_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));

	ctx->state = BMS_STATE_INIT;
	ctx->state_entry_time_ms = k_uptime_get();
	ctx->latched_faults = nvm_load_latched_faults();

	if (bms_hv_sense_init(&ctx->hv_sense) < 0) {
		LOG_ERR("AMC3330 HV voltage sense initialization failed!");
		ctx->live_faults    |= BMS_FAULT_IMPLAUSIBILITY;
		ctx->latched_faults |= BMS_FAULT_IMPLAUSIBILITY;
	}

	if (DRV3946_Init(&ctx->drv_airs) != DRV3946_OK ||
	    DRV3946_ConfigureDCNLEV100(&ctx->drv_airs, DRV3946_CHANNEL_BOTH) != DRV3946_OK) {
		LOG_ERR("DRV3946-Q1 initialization failed!");
		ctx->live_faults    |= BMS_FAULT_CONTACTOR_DRV;
		ctx->latched_faults |= BMS_FAULT_CONTACTOR_DRV;
	}

	update_outputs_for_state(ctx);
	LOG_INF("BMS State Machine initialized (latched_faults=0x%08X)", ctx->latched_faults);
}

void bms_sm_raise_fault(struct bms_ctx *ctx, uint32_t fault_mask)
{
	if (fault_mask == BMS_FAULT_NONE) {
		return;
	}

	ctx->live_faults |= fault_mask;

	if ((ctx->latched_faults | fault_mask) != ctx->latched_faults) {
		ctx->latched_faults |= fault_mask;
		LOG_ERR("Safety fault latched! new=0x%08X, total=0x%08X",
			fault_mask, ctx->latched_faults);
		nvm_save_latched_faults(ctx->latched_faults);
	}

	/* If faulting while precharging (AIR- closed, AIR+ open), open AIR- first */
	if (ctx->state == BMS_STATE_PRECHARGE_WAIT) {
		transition_to(ctx, BMS_STATE_PRECHARGE_OPEN_NEG);
	} else if (ctx->state == BMS_STATE_PRECHARGE_OPEN_NEG) {
		update_outputs_for_state(ctx);
	} else {
		transition_to(ctx, BMS_STATE_FAULT);
	}
}

bool bms_sm_clear_faults_from_pc(struct bms_ctx *ctx)
{
	sample_hardware_inputs(ctx);

	if (ctx->live_faults != BMS_FAULT_NONE || ctx->req.ts_request) {
		LOG_WRN("PC fault clear rejected (live=0x%08X, ts_req=%d)",
			ctx->live_faults, ctx->req.ts_request);
		return false;
	}

	DRV3946_ClearFaults(&ctx->drv_airs);
	ctx->latched_faults = BMS_FAULT_NONE;
	ctx->req.pc_clear_faults = false;
	nvm_save_latched_faults(BMS_FAULT_NONE);

	LOG_INF("Latched faults manually cleared by PC application");
	transition_to(ctx, BMS_STATE_STANDBY);
	return true;
}

void bms_sm_tick(struct bms_ctx *ctx)
{
	sample_hardware_inputs(ctx);

	/* 1. Fault check (allow PRECHARGE_OPEN_NEG to finish opening AIR- first) */
	if (ctx->live_faults != BMS_FAULT_NONE) {
		bms_sm_raise_fault(ctx, ctx->live_faults);
	} else if (ctx->latched_faults != BMS_FAULT_NONE &&
		   ctx->state != BMS_STATE_FAULT &&
		   ctx->state != BMS_STATE_PRECHARGE_OPEN_NEG) {
		bms_sm_raise_fault(ctx, ctx->latched_faults);
	}

	/* 2. Refresh ts_ready if SDC power state changed */
	bool new_ts_ready = bms_sm_is_ts_ready(ctx);
	if (new_ts_ready != ctx->out.ts_ready) {
		ctx->out.ts_ready = new_ts_ready;
		apply_hardware_outputs(ctx);
	}

	bool ts_keep_active = ctx->out.ts_ready && ctx->req.ts_request;
	int64_t elapsed_ms  = k_uptime_get() - ctx->state_entry_time_ms;

	/* 3. Shared deactivation check for all energized / precharging states */
	if (!ts_keep_active) {
		if (ctx->state == BMS_STATE_PRECHARGE_WAIT) {
			transition_to(ctx, BMS_STATE_PRECHARGE_OPEN_NEG);
			return;
		}
		if (ctx->state == BMS_STATE_PRECHARGE_CLOSE_PC ||
		    ctx->state == BMS_STATE_PRECHARGE_CLOSE_POS ||
		    ctx->state == BMS_STATE_ACTIVE ||
		    ctx->state == BMS_STATE_CHARGING) {
			transition_to(ctx, BMS_STATE_STANDBY);
			return;
		}
	}

	/* 4. Forward state progression */
	switch (ctx->state) {
	case BMS_STATE_INIT:
		if (ctx->afe_init_ok && ctx->hv_sense_valid) {
			transition_to(ctx, BMS_STATE_STANDBY);
		}
		break;

	case BMS_STATE_STANDBY:
		if (ts_keep_active) {
			transition_to(ctx, BMS_STATE_PRECHARGE_CLOSE_PC);
		}
		break;

	case BMS_STATE_PRECHARGE_CLOSE_PC:
		if (elapsed_ms >= BMS_RELAY_SETTLE_MS) {
			transition_to(ctx, BMS_STATE_PRECHARGE_WAIT);
		}
		break;

	case BMS_STATE_PRECHARGE_WAIT:
		if (elapsed_ms >= BMS_PRECHARGE_MIN_TIME_MS && is_precharge_voltage_reached(ctx)) {
			transition_to(ctx, BMS_STATE_PRECHARGE_CLOSE_POS);
		} else if (elapsed_ms >= BMS_PRECHARGE_TIMEOUT_MS) {
			LOG_ERR("Precharge timeout (%lld ms, Pack=%u mV, TS=%u mV)",
				elapsed_ms, ctx->pack_voltage_mv, ctx->ts_voltage_mv);
			bms_sm_raise_fault(ctx, BMS_FAULT_PRECHARGE_TIMEOUT);
		}
		break;

	case BMS_STATE_PRECHARGE_CLOSE_POS:
		if (elapsed_ms >= BMS_RELAY_SETTLE_MS) {
			transition_to(ctx, ctx->req.charge_enable ? BMS_STATE_CHARGING
								  : BMS_STATE_ACTIVE);
		}
		break;

	case BMS_STATE_PRECHARGE_OPEN_NEG:
		if (elapsed_ms >= BMS_RELAY_SETTLE_MS) {
			transition_to(ctx, (ctx->latched_faults != BMS_FAULT_NONE)
					 ? BMS_STATE_FAULT : BMS_STATE_STANDBY);
		}
		break;

	case BMS_STATE_ACTIVE:
		if (ctx->req.charge_enable) {
			transition_to(ctx, BMS_STATE_CHARGING);
		}
		break;

	case BMS_STATE_CHARGING:
		if (!ctx->req.charge_enable) {
			transition_to(ctx, BMS_STATE_ACTIVE);
		}
		break;

	case BMS_STATE_FAULT:
		if (ctx->req.pc_clear_faults) {
			bms_sm_clear_faults_from_pc(ctx);
		}
		break;

	default:
		transition_to(ctx, BMS_STATE_FAULT);
		break;
	}
}
