/**
 * @file bms_shell.c
 * @brief "bms" shell commands, served over the USB CDC-ACM virtual COM port.
 *
 *   bms status        Print state, faults, measurements and outputs.
 *   bms clear_faults  Manually clear latched faults (PC application command).
 */

#include <errno.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "bms.h"

static const struct {
	uint32_t bit;
	const char *name;
} fault_names[] = {
	{ BMS_FAULT_CELL_OV,            "CELL_OV" },
	{ BMS_FAULT_CELL_UV,            "CELL_UV" },
	{ BMS_FAULT_OVER_TEMP,          "OVER_TEMP" },
	{ BMS_FAULT_UNDER_TEMP,         "UNDER_TEMP" },
	{ BMS_FAULT_OVER_CURRENT,       "OVER_CURRENT" },
	{ BMS_FAULT_AFE_COMM,           "AFE_COMM" },
	{ BMS_FAULT_OPEN_WIRE,          "OPEN_WIRE" },
	{ BMS_FAULT_PRECHARGE_TIMEOUT,  "PRECHARGE_TIMEOUT" },
	{ BMS_FAULT_IMPLAUSIBILITY,     "IMPLAUSIBILITY" },
	{ BMS_FAULT_CONTACTOR_DRV,      "CONTACTOR_DRV" },
	{ BMS_FAULT_REGEN_OVER_CURRENT, "REGEN_OVER_CURRENT" },
	{ BMS_FAULT_CHARGE_UNDER_TEMP,  "CHARGE_UNDER_TEMP" },
	{ BMS_FAULT_CURRENT_SENSOR,     "CURRENT_SENSOR" },
};

static void print_faults(const struct shell *sh, const char *label, uint32_t mask)
{
	shell_fprintf(sh, SHELL_NORMAL, "%-14s 0x%08X", label, mask);

	if (mask == BMS_FAULT_NONE) {
		shell_fprintf(sh, SHELL_NORMAL, " (none)");
	}

	ARRAY_FOR_EACH(fault_names, i) {
		if (mask & fault_names[i].bit) {
			shell_fprintf(sh, SHELL_NORMAL, " %s", fault_names[i].name);
		}
	}

	shell_fprintf(sh, SHELL_NORMAL, "\n");
}

static void print_ivt(const struct shell *sh, const struct ivt_s_data *ivt, int64_t now_ms)
{
	static const struct {
		enum ivt_s_channel ch;
		const char *name;
		const char *unit;
	} rows[] = {
		{ IVT_S_CH_I,  "I",  "mA" },
		{ IVT_S_CH_U1, "U1", "mV" },
		{ IVT_S_CH_U2, "U2", "mV" },
		{ IVT_S_CH_U3, "U3", "mV" },
	};

	ARRAY_FOR_EACH(rows, i) {
		const struct ivt_s_reading *r = &ivt->ch[rows[i].ch];

		if (!r->seen) {
			shell_print(sh, "%-14s %-2s no frames received", "IVT-S:", rows[i].name);
			continue;
		}

		shell_print(sh, "%-14s %-2s %d %s, %lld ms ago, state 0x%X%s%s%s%s", "IVT-S:",
			    rows[i].name, r->value, rows[i].unit, now_ms - r->rx_ms, r->state,
			    (r->state & IVT_S_STATE_OCS) ? " OCS" : "",
			    (r->state & IVT_S_STATE_RESULT_ERROR) ? " RESULT_ERROR" : "",
			    (r->state & IVT_S_STATE_ANY_MEAS_ERROR) ? " ANY_MEAS_ERROR" : "",
			    (r->state & IVT_S_STATE_SYSTEM_ERROR) ? " SYSTEM_ERROR" : "");
	}
}

static int cmd_bms_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Copy under the lock so the printout is one consistent tick */
	struct bms_ctx c;

	bms_ctx_lock();
	c = *bms_ctx_get();
	bms_ctx_unlock();

	int64_t now_ms = k_uptime_get();

	shell_print(sh, "%-14s %s", "State:", bms_state_to_str(c.state));
	print_faults(sh, "Live faults:", c.live_faults);
	print_faults(sh, "Latched:", c.latched_faults);

	shell_print(sh, "%-14s min %u mV, max %u mV, sum %u mV", "Cells:",
		    c.min_cell_mv, c.max_cell_mv, c.sum_cells_mv);
	shell_print(sh, "%-14s min %d.%d C, max %d.%d C", "Temperature:",
		    c.min_temp_deci_c / 10, abs(c.min_temp_deci_c % 10),
		    c.max_temp_deci_c / 10, abs(c.max_temp_deci_c % 10));
	shell_print(sh, "%-14s pack %u mV, TS %u mV (sense %s)", "HV:",
		    c.pack_voltage_mv, c.ts_voltage_mv,
		    c.hv_sense_valid ? "valid" : "INVALID");
	shell_print(sh, "%-14s %d mA (positive = discharge)", "Current:",
		    c.pack_current_ma);
	print_ivt(sh, &c.ivt, now_ms);
	shell_print(sh, "%-14s %u mA (%s)", "Regen limit:",
		    c.out.max_regen_current_ma,
		    c.out.regen_allowed ? "allowed" : "blocked");

	shell_print(sh, "%-14s PC=%d AIR-=%d AIR+=%d SDC_FAULT=%d BAL=%d", "Outputs:",
		    c.out.precharge_closed, c.out.air_neg_closed,
		    c.out.air_pos_closed, c.out.sdc_fault_pin_active,
		    c.out.balancing_enabled);
	shell_print(sh, "%-14s TS_READY=%d TS_ACTIVE=%d TS_REQUEST=%d SDC_POWERED=%d",
		    "Handshake:", c.out.ts_ready, c.out.ts_active,
		    c.req.ts_request, c.sdc_airs_powered);

	return 0;
}

static int cmd_bms_clear_faults(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct bms_ctx *ctx = bms_ctx_get();
	bool in_fault_state;
	bool cleared = false;
	uint32_t live;
	bool ts_request;

	bms_ctx_lock();
	in_fault_state = (ctx->state == BMS_STATE_FAULT);
	if (in_fault_state) {
		cleared = bms_sm_clear_faults_from_pc(ctx);
	}
	live       = ctx->live_faults;
	ts_request = ctx->req.ts_request;
	bms_ctx_unlock();

	if (!in_fault_state) {
		shell_print(sh, "Nothing to clear: BMS is not in the FAULT state");
		return 0;
	}

	if (!cleared) {
		shell_error(sh, "Rejected: %s",
			    (live != BMS_FAULT_NONE) ? "a fault condition is still present"
						     : "TS request is still asserted");
		print_faults(sh, "Live faults:", live);
		shell_print(sh, "%-14s %d", "TS_REQUEST:", ts_request);
		return -EBUSY;
	}

	shell_print(sh, "Latched faults cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(bms_cmds,
	SHELL_CMD(status, NULL, "Print BMS state, faults, measurements and outputs",
		  cmd_bms_status),
	SHELL_CMD(clear_faults, NULL, "Clear latched faults (only when no fault is present)",
		  cmd_bms_clear_faults),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(bms, &bms_cmds, "SUFST BMS commands", NULL);
