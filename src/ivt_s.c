/**
 * @file ivt_s.c
 * @brief Isabellenhuette IVT-S CAN receive driver (see ivt_s.h).
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ivt_s.h"

LOG_MODULE_REGISTER(ivt_s, LOG_LEVEL_INF);

static const struct device *const can_dev = DEVICE_DT_GET(DT_ALIAS(ivt_can));

static struct ivt_s_data latest;
static struct k_spinlock latest_lock;

/* Runs in the CAN RX interrupt context */
static void ivt_s_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	struct ivt_s_result res;

	if ((frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0U ||
	    !ivt_s_decode(frame->data, frame->dlc, &res)) {
		return;
	}

	/* With default IDs each result has its own ID matching its MuxID */
	if (frame->id != IVT_S_CAN_ID_RESULT_BASE + (uint32_t)res.channel) {
		return;
	}

	int64_t now_ms = k_uptime_get();
	k_spinlock_key_t key = k_spin_lock(&latest_lock);

	latest.ch[res.channel].seen  = true;
	latest.ch[res.channel].rx_ms = now_ms;
	latest.ch[res.channel].state = res.state;
	latest.ch[res.channel].value = res.value;

	k_spin_unlock(&latest_lock, key);
}

int ivt_s_init(void)
{
	/* 0x520..0x52F: covers the eight result IDs 0x521..0x528 */
	const struct can_filter filter = {
		.id    = 0x520U,
		.mask  = 0x7F0U,
		.flags = 0U,
	};
	int ret;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device %s not ready", can_dev->name);
		return -ENODEV;
	}

	ret = can_add_rx_filter(can_dev, ivt_s_rx_cb, NULL, &filter);
	if (ret < 0) {
		LOG_ERR("Failed to add IVT-S RX filter (%d)", ret);
		return ret;
	}

	ret = can_start(can_dev);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to start CAN controller (%d)", ret);
		return ret;
	}

	LOG_INF("IVT-S receiver started on %s", can_dev->name);
	return 0;
}

void ivt_s_get(struct ivt_s_data *out)
{
	k_spinlock_key_t key = k_spin_lock(&latest_lock);

	*out = latest;

	k_spin_unlock(&latest_lock, key);
}
