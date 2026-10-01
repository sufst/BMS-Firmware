#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bms.h"

LOG_MODULE_REGISTER(bms_main, LOG_LEVEL_INF);

static struct bms_ctx bms;

/**
 * @brief Temporary stub to populate nominal pack & AMC3330 measurements
 *        until the physical ADC/GPIO pins and AFE driver are wired up.
 */
static void update_mock_measurements(struct bms_ctx *ctx)
{
	ctx->afe_init_ok      = true;
	ctx->last_afe_rx_ms   = k_uptime_get();
	ctx->min_cell_mv      = 3700U;   /* 3.70 V */
	ctx->max_cell_mv      = 3720U;   /* 3.72 V */
	ctx->min_temp_deci_c  = 240;     /* 24.0 C */
	ctx->max_temp_deci_c  = 265;     /* 26.5 C */
	ctx->sum_cells_mv     = 371000U; /* 371.0 V */
	ctx->pack_current_ma  = 0;

	/* Mock AMC3330 U4 (ACCU) & U3 (DC-LINK) differential outputs + DIAG pins */
	ctx->hv_sense.accu.mock_diag_ok   = true;
	ctx->hv_sense.accu.mock_diff_mv   = amc3330_hv_mv_to_diff_mv(371000U); /* ~1067 mV diff */
	ctx->hv_sense.dclink.mock_diag_ok = true;
	ctx->hv_sense.dclink.mock_diff_mv = amc3330_hv_mv_to_diff_mv(0U);      /* 0 V when AIRs open */
}

int main(void)
{
	LOG_INF("SUFST BMS Firmware Initialized (%s)", CONFIG_BOARD_TARGET);

	bms_sm_init(&bms);

	while (1) {
		update_mock_measurements(&bms);
		bms_sm_tick(&bms);
		bms_safety_check(&bms, BMS_SM_TICK_MS);

		k_sleep(K_MSEC(BMS_SM_TICK_MS));
	}

	return 0;
}
