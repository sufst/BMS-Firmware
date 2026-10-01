#ifndef SUFST_BMS_H_
#define SUFST_BMS_H_

/**
 * @file bms.h
 * @brief SUFST BMS core data structures, safety limits, fault flags,
 *        and state machine / safety monitor API.
 */

#include <stdbool.h>
#include <stdint.h>

#include "bms_hv_sense.h"
#include "drv3946.h"

/* ==========================================================================
 * Timing & Precharge Configuration Constants
 * ========================================================================== */

/** Periodic state machine & safety monitor tick interval (ms) */
#define BMS_SM_TICK_MS                     50U

/** Mechanical relay settling delay between sequenced contactor steps (ms) */
#define BMS_RELAY_SETTLE_MS                100U

/** Maximum allowed time in PRECHARGE_WAIT before faulting (ms) */
#define BMS_PRECHARGE_TIMEOUT_MS           3000U

/** Minimum time in PRECHARGE_WAIT before checking voltage threshold (ms) */
#define BMS_PRECHARGE_MIN_TIME_MS          500U

/** Precharge complete threshold: TS voltage >= 95% of pack voltage */
#define BMS_PRECHARGE_THRESHOLD_PCT        95U

/* ==========================================================================
 * Safety Threshold Constants (Standard Li-Ion / Formula Student EV Defaults)
 * ========================================================================== */

/** Maximum allowed single cell voltage (4.200 V) */
#define BMS_CELL_MAX_MV                    4200U

/** Minimum allowed single cell voltage (2.500 V) */
#define BMS_CELL_MIN_MV                    2500U

/** Implausibly high cell reading -> sensor/sense-wire fault (5.000 V) */
#define BMS_CELL_IMPLAUSIBLE_HIGH_MV       5000U

/** Implausibly low cell reading -> sensor/sense-wire fault (0.500 V) */
#define BMS_CELL_IMPLAUSIBLE_LOW_MV        500U

/** Maximum allowed cell temperature in 0.1 deg C (60.0 C per FS EV rules) */
#define BMS_TEMP_MAX_DECI_C                600

/** Minimum allowed cell temperature in 0.1 deg C (-20.0 C) */
#define BMS_TEMP_MIN_DECI_C                (-200)

/** Implausibly high temperature reading in 0.1 deg C (120.0 C, shorted NTC) */
#define BMS_TEMP_IMPLAUSIBLE_HIGH_DECI_C   1200

/** Implausibly low temperature reading in 0.1 deg C (-40.0 C, open NTC) */
#define BMS_TEMP_IMPLAUSIBLE_LOW_DECI_C    (-400)

/** Maximum allowed absolute pack current in mA (250.0 A) */
#define BMS_CURRENT_MAX_MA                 250000

/** Maximum allowed mismatch between sum of cells and HV pack sensor (5.0 V) */
#define BMS_PACK_SUM_MAX_MISMATCH_MV       5000U

/* ==========================================================================
 * Fault Debounce Timings (FS EV Rule Compliant)
 * ========================================================================== */

/** Over/under-voltage debounce duration (ms) - FS max allowed: 500 ms */
#define BMS_DEBOUNCE_VOLTAGE_MS            100U

/** Over-current debounce duration (ms) - FS max allowed: 500 ms */
#define BMS_DEBOUNCE_CURRENT_MS            100U

/** Over/under-temperature debounce duration (ms) - FS max allowed: 1000 ms */
#define BMS_DEBOUNCE_TEMP_MS               500U

/** Open-wire detection debounce duration (ms) */
#define BMS_DEBOUNCE_OPEN_WIRE_MS          100U

/** Sensor implausibility debounce duration (ms) */
#define BMS_DEBOUNCE_IMPLAUSIBLE_MS        100U

/** Maximum time without a valid AFE update before faulting (ms) */
#define BMS_AFE_COMM_TIMEOUT_MS            250U

/* ==========================================================================
 * BMS States
 * ========================================================================== */

typedef enum {
	BMS_STATE_INIT = 0,            /**< Boot, self-test, and persistent fault check */
	BMS_STATE_STANDBY,             /**< Idle; all relays open, outputs TS_READY when SDC powered */
	BMS_STATE_PRECHARGE_CLOSE_PC,  /**< Step 1: Close Precharge first (PC=1, AIR-=0, AIR+=0) */
	BMS_STATE_PRECHARGE_WAIT,      /**< Step 2: Close AIR- & charge DC bus (PC=1, AIR-=1, AIR+=0) */
	BMS_STATE_PRECHARGE_CLOSE_POS, /**< Step 3: Close AIR+ while PC stays closed (PC=1, AIR-=1, AIR+=1) */
	BMS_STATE_PRECHARGE_OPEN_NEG,  /**< Precharge abort/fault: Open AIR- first (PC=1, AIR-=0, AIR+=0) */
	BMS_STATE_ACTIVE,              /**< Step 4: Open PC, TS active (PC=0, AIR-=1, AIR+=1) */
	BMS_STATE_CHARGING,            /**< Pack charging active (PC=0, AIR-=1, AIR+=1, balancing) */
	BMS_STATE_FAULT,               /**< Fault latched; SDC open, all relays open (PC=0, AIR-=0, AIR+=0) */
} bms_state_t;

/* ==========================================================================
 * Safety Fault Bitmask
 * ========================================================================== */

#define BMS_FAULT_NONE                (0U)
#define BMS_FAULT_CELL_OV             (1U << 0) /**< Cell over-voltage */
#define BMS_FAULT_CELL_UV             (1U << 1) /**< Cell under-voltage */
#define BMS_FAULT_OVER_TEMP           (1U << 2) /**< Cell/pack over-temperature */
#define BMS_FAULT_UNDER_TEMP          (1U << 3) /**< Cell/pack under-temperature */
#define BMS_FAULT_OVER_CURRENT        (1U << 4) /**< Pack over-current */
#define BMS_FAULT_AFE_COMM            (1U << 5) /**< AFE isoSPI communication loss */
#define BMS_FAULT_OPEN_WIRE           (1U << 6) /**< Cell sense/thermistor open wire */
#define BMS_FAULT_PRECHARGE_TIMEOUT   (1U << 7) /**< Precharge failed to reach target V */
#define BMS_FAULT_IMPLAUSIBILITY      (1U << 8) /**< Sensor reading implausibility */
#define BMS_FAULT_CONTACTOR_DRV       (1U << 9) /**< DRV3946-Q1 contactor driver nFAULT / SPI error */

/* ==========================================================================
 * BMS Context & IO State
 * ========================================================================== */

/**
 * @brief Relay, Shutdown Circuit (SDC), and VCU handshake outputs.
 */
struct bms_outputs {
	bool precharge_closed;     /**< Precharge relay (discrete GPIO) */
	bool air_neg_closed;       /**< Low-side contactor (AIR- via DRV3946 Ch1) */
	bool air_pos_closed;       /**< High-side contactor (AIR+ via DRV3946 Ch2) */
	bool sdc_fault_pin_active; /**< GPIO output: true = open SDC relay (FAULT) */
	bool ts_ready;             /**< BMS -> VCU (CAN/GPIO): AIRs have power & SDC can close */
	bool ts_active;            /**< BMS -> VCU (CAN/GPIO): Precharge complete & both AIRs closed */
	bool balancing_enabled;    /**< Cell balancing active flag */
};

/**
 * @brief External commands / requests into the BMS state machine (from VCU / PC).
 */
struct bms_requests {
	bool ts_request_can;       /**< VCU -> BMS "TS REQUEST" via CAN */
	bool ts_request_gpio;      /**< VCU -> BMS "TS REQUEST" via GPIO */
	bool ts_request;           /**< Combined effective TS REQUEST signal (CAN || GPIO) */
	bool charge_enable;        /**< Request to enter CHARGING mode after precharge */
	bool pc_clear_faults;      /**< Manual fault-clear command from PC application */
};

/**
 * @brief Accumulator timers (ms) for fault condition debouncing.
 */
struct bms_safety_timers {
	uint32_t cell_ov_ms;
	uint32_t cell_uv_ms;
	uint32_t over_temp_ms;
	uint32_t under_temp_ms;
	uint32_t over_current_ms;
	uint32_t open_wire_ms;
	uint32_t implausibility_ms;
};

/**
 * @brief Main BMS runtime context.
 */
struct bms_ctx {
	bms_state_t state;
	int64_t state_entry_time_ms;

	/* Hardware Drivers: DRV3946-Q1 Contactors & Dual AMC3330 HV Voltage Sense */
	DRV3946_HandleTypeDef drv_airs;
	struct bms_hv_sense   hv_sense;

	/* Fault tracking & debounce state */
	uint32_t live_faults;      /**< Currently present (debounced) fault conditions */
	uint32_t latched_faults;   /**< Persistent fault bitmask (saved across power cycles) */
	struct bms_safety_timers fault_timers;

	/* Shutdown circuit, AFE, & AMC3330 DIAG readiness monitoring */
	bool afe_init_ok;          /**< Set true once AFE passes startup self-check */
	bool hv_sense_valid;       /**< True when both AMC3330 DIAG pins (U3 & U4) are HIGH */
	bool sdc_airs_powered;     /**< Continuous sense: true when SDC is closed & AIRs have power */
	bool afe_open_wire;        /**< Reported by AFE open-wire diagnostic check */
	int64_t last_afe_rx_ms;    /**< Uptime timestamp (ms) of last valid AFE sample */

	/* Pack & DC bus measurements (updated by sensor/AFE layer) */
	uint32_t sum_cells_mv;     /**< Sum of all individual cell voltages from AFE (mV) */
	uint32_t pack_voltage_mv;  /**< Total accumulator voltage from HV pack sensor (mV) */
	uint32_t ts_voltage_mv;    /**< Tractive System / inverter DC-link voltage (mV) */
	int32_t  pack_current_ma;  /**< Pack current (mA, positive = discharge) */
	uint16_t min_cell_mv;      /**< Lowest single cell voltage (mV) */
	uint16_t max_cell_mv;      /**< Highest single cell voltage (mV) */
	int16_t  min_temp_deci_c;  /**< Lowest cell temperature (0.1 deg C) */
	int16_t  max_temp_deci_c;  /**< Highest cell temperature (0.1 deg C) */

	/*
	 * TODO (Regen):
	 *   - Split over-current fault check into asymmetric discharge vs. regen/charge limits.
	 *   - Enforce low-temperature charge cutoff (0 C) when current is negative (charging/regen).
	 *   - Compute regen_allowed / max_regen_current_ma in bms_outputs for VCU/Inverter CAN.
	 */

	struct bms_requests req;
	struct bms_outputs out;
};

/* ==========================================================================
 * State Machine Public API (bms_sm.c)
 * ========================================================================== */

/**
 * @brief Initialize the BMS context, load persistent faults from Flash,
 *        and set safe initial hardware outputs.
 */
void bms_sm_init(struct bms_ctx *ctx);

/**
 * @brief Execute one periodic cycle of the BMS state machine.
 *        Samples SDC / ts_request inputs, updates ts_ready, and runs state transitions.
 */
void bms_sm_tick(struct bms_ctx *ctx);

/**
 * @brief Evaluate whether the BMS is ready for Tractive System activation
 *        (no faults, AFE initialized, and AIRs have power / SDC can close).
 */
bool bms_sm_is_ts_ready(const struct bms_ctx *ctx);

/**
 * @brief Raise one or more safety fault flags. Immediately latches into
 *        non-volatile storage and initiates sequenced fault shutdown.
 */
void bms_sm_raise_fault(struct bms_ctx *ctx, uint32_t fault_mask);

/**
 * @brief Request manual clearance of latched faults (from PC application).
 *        Faults will only clear if live_faults == 0 and ts_request is off.
 */
bool bms_sm_clear_faults_from_pc(struct bms_ctx *ctx);

/**
 * @brief Convert a BMS state enum to a human-readable string.
 */
const char *bms_state_to_str(bms_state_t state);

/* ==========================================================================
 * Safety & Fault Monitor Public API (bms_safety.c)
 * ========================================================================== */

/**
 * @brief Evaluate all cell voltages, temperatures, pack current, open-wire,
 *        sensor plausibility, and AFE communication heartbeat with debouncing.
 *        Updates ctx->live_faults and raises latched faults when thresholds
 *        are exceeded for their respective debounce durations.
 *
 * @param ctx   Pointer to BMS context structure.
 * @param dt_ms Elapsed time since last check in milliseconds (e.g. BMS_SM_TICK_MS).
 */
void bms_safety_check(struct bms_ctx *ctx, uint32_t dt_ms);

#endif /* SUFST_BMS_H_ */
