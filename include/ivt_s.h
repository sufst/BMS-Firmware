#ifndef SUFST_IVT_S_H_
#define SUFST_IVT_S_H_

/**
 * @file ivt_s.h
 * @brief Isabellenhuette IVT-S current / voltage sensor (CAN) receive driver.
 *
 * Decodes the sensor's cyclic result messages (datasheet v1.01, chapter 8.1)
 * using its factory-default configuration:
 *   - 500 kbit/s, 11-bit IDs 0x521 (I) .. 0x528 (Wh), DLC 6
 *   - DB0      MuxID 0x00..0x07 (channel number)
 *   - DB1      low nibble: per-channel cyclic message counter
 *              high nibble: state bits (IVT_S_STATE_*)
 *   - DB2..5   signed 32-bit result, big endian
 *   - Current cyclic every 20 ms, U1..U3 cyclic every 60 ms, rest disabled
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Default CAN ID of IVT_Msg_Result_I; the other results follow consecutively */
#define IVT_S_CAN_ID_RESULT_BASE   0x521U

/** Result payload length (bytes) */
#define IVT_S_RESULT_DLC           6U

/** Result channels, equal to the MuxID in DB0 */
enum ivt_s_channel {
	IVT_S_CH_I = 0,   /**< Current, 1 mA */
	IVT_S_CH_U1,      /**< Voltage 1, 1 mV */
	IVT_S_CH_U2,      /**< Voltage 2, 1 mV */
	IVT_S_CH_U3,      /**< Voltage 3, 1 mV */
	IVT_S_CH_T,       /**< Temperature, 0.1 deg C */
	IVT_S_CH_W,       /**< Power, 1 W */
	IVT_S_CH_AS,      /**< Current counter, 1 As */
	IVT_S_CH_WH,      /**< Energy counter, 1 Wh */
	IVT_S_CH_COUNT,
};

/* State bits (DB1 high nibble, shifted down to bits 0..3) */
#define IVT_S_STATE_OCS            (1U << 0) /**< Over-current signal is active */
#define IVT_S_STATE_RESULT_ERROR   (1U << 1) /**< This result: out of range / reduced precision / measurement error */
#define IVT_S_STATE_ANY_MEAS_ERROR (1U << 2) /**< Any result has a measurement error */
#define IVT_S_STATE_SYSTEM_ERROR   (1U << 3) /**< System error: sensor function not ensured */

/** One decoded result message */
struct ivt_s_result {
	enum ivt_s_channel channel;
	uint8_t msg_count;   /**< Cyclic counter 0..15 */
	uint8_t state;       /**< IVT_S_STATE_* bits */
	int32_t value;       /**< Raw result in the channel's unit */
};

/** Latest reading of one channel */
struct ivt_s_reading {
	bool     seen;       /**< True once at least one frame has been received */
	int64_t  rx_ms;      /**< Uptime (ms) of the last frame */
	uint8_t  state;      /**< IVT_S_STATE_* bits of the last frame */
	int32_t  value;      /**< Last result in the channel's unit */
};

/** Snapshot of all channels */
struct ivt_s_data {
	struct ivt_s_reading ch[IVT_S_CH_COUNT];
};

/**
 * @brief Decode one IVT-S result message payload (big-endian default config).
 *
 * @param data Frame payload.
 * @param dlc  Frame payload length.
 * @param out  Decoded result.
 * @return true if the payload is a valid result message.
 */
static inline bool ivt_s_decode(const uint8_t *data, uint8_t dlc, struct ivt_s_result *out)
{
	if (dlc < IVT_S_RESULT_DLC || data[0] >= IVT_S_CH_COUNT) {
		return false;
	}

	out->channel   = (enum ivt_s_channel)data[0];
	out->msg_count = data[1] & 0x0FU;
	out->state     = data[1] >> 4;
	out->value     = (int32_t)(((uint32_t)data[2] << 24) |
				   ((uint32_t)data[3] << 16) |
				   ((uint32_t)data[4] << 8)  |
				    (uint32_t)data[5]);
	return true;
}

/**
 * @brief Start the CAN controller and begin receiving IVT-S result messages.
 * @return 0 on success, negative errno otherwise.
 */
int ivt_s_init(void);

/**
 * @brief Copy the latest readings of all channels.
 */
void ivt_s_get(struct ivt_s_data *out);

#endif /* SUFST_IVT_S_H_ */
