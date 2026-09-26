/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/smf.h>
#include <date_time.h>

#include "app_common.h"
#include "orientation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Register log module */
LOG_MODULE_REGISTER(orientation, CONFIG_APP_ORIENTATION_LOG_LEVEL);

/* Define channels provided by this module */
ZBUS_CHAN_DEFINE(orientation_chan,
		 struct orientation_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0)
);

/* Register subscriber */
ZBUS_MSG_SUBSCRIBER_DEFINE(orientation);

/* Observe channels */
ZBUS_CHAN_ADD_OBS(orientation_chan, orientation, 0);

#define MAX_MSG_SIZE sizeof(struct orientation_msg)

/* Standard gravity, used to normalize accelerometer readings that come back
 * already scaled to g (e.g. the ADXL367 on Thingy:91 X) to m/s^2, so the
 * rest of this file works the same regardless of the underlying chip.
 */
#define STANDARD_GRAVITY_M_S2 9.80665

/* NOTE: this module deliberately does NOT register with the task watchdog
 * (CONFIG_TASK_WDT). The nRF91 WDT peripheral has a fixed 8 hardware reload
 * channels (CONFIG_TASK_WDT_CHANNELS=8 is a hardware ceiling, not a
 * software default -- raising it faults at boot with a SecureFault before
 * main() even runs). All 8 channels are already claimed by main, cloud,
 * fota, location, network, power, storage and environmental, so this
 * module runs unsupervised, the same as led/button.
 */

/* State machine */

enum orientation_module_state {
	/* The module is running and waiting for orientation sample requests */
	STATE_RUNNING,
};

/* State object.
 * Used to transfer context data between state changes.
 */
struct orientation_state_object {
	/* This must be first */
	struct smf_ctx ctx;

	/* Last channel type that a message was received on */
	const struct zbus_channel *chan;

	/* Buffer for last zbus message */
	uint8_t msg_buf[MAX_MSG_SIZE];

	/* Pointer to the low-power accelerometer device (ADXL362 or ADXL367
	 * depending on board).
	 */
	const struct device *const accel;
};

/* Forward declarations of state handlers */
static enum smf_state_result state_running_run(void *obj);

/* State machine definition */
static const struct smf_state states[] = {
	[STATE_RUNNING] = SMF_CREATE_STATE(NULL, state_running_run, NULL, NULL, NULL),
};

const char *orientation_face_to_str(enum orientation_face face)
{
	switch (face) {
	case ORIENTATION_FACE_X_UP:
		return "X_UP";
	case ORIENTATION_FACE_X_DOWN:
		return "X_DOWN";
	case ORIENTATION_FACE_Y_UP:
		return "Y_UP";
	case ORIENTATION_FACE_Y_DOWN:
		return "Y_DOWN";
	case ORIENTATION_FACE_Z_UP:
		return "Z_UP";
	case ORIENTATION_FACE_Z_DOWN:
		return "Z_DOWN";
	default:
		return "UNKNOWN";
	}
}

/* Some accelerometer drivers (e.g. ADXL367 on Thingy:91 X) return readings
 * already scaled to g; others (e.g. ADXL362 on the original Thingy:91) return
 * m/s^2. At rest, one axis reads ~1 g (~9.81 m/s^2) and the others read near
 * 0, so checking the magnitude against a midpoint reliably tells the two
 * apart; scale up only if it looks like g's.
 */
static void normalize_to_m_s2(double *x, double *y, double *z)
{
	double mag = sqrt((*x) * (*x) + (*y) * (*y) + (*z) * (*z));

	if (mag > 0.0 && mag < 4.0) {
		*x *= STANDARD_GRAVITY_M_S2;
		*y *= STANDARD_GRAVITY_M_S2;
		*z *= STANDARD_GRAVITY_M_S2;
	}
}

static void sample_orientation(const struct device *const accel)
{
	/* TEMPORARY DIAGNOSTIC STUB: sensor calls disabled to bisect a
	 * SecureFault that occurs before any console output when this module
	 * is enabled. If the device boots cleanly with this stub in place,
	 * the fault is inside the sensor_sample_fetch_chan()/sensor_channel_get()
	 * calls below (or the ADXL367 driver itself on this board); if it
	 * still faults, the problem is in module init/threading instead.
	 * Restore the real body once the cause is confirmed.
	 */
	ARG_UNUSED(accel);
	LOG_DBG("sample_orientation called (sensor calls disabled for testing)");
}

/* State handlers */

static enum smf_state_result state_running_run(void *obj)
{
	struct orientation_state_object const *state_object = obj;

	if (&orientation_chan == state_object->chan) {
		const struct orientation_msg *msg =
			(const struct orientation_msg *)state_object->msg_buf;

		if (msg->type == ORIENTATION_SAMPLE_REQUEST) {
			LOG_DBG("Orientation sample request received, getting data");
			sample_orientation(state_object->accel);

			return SMF_EVENT_HANDLED;
		}
	}

	return SMF_EVENT_PROPAGATE;
}

static void orientation_module_thread(void)
{
	int err;
	__attribute__((section(".data")))
	static struct orientation_state_object orientation_state = {
		.accel = DEVICE_DT_GET(DT_ALIAS(accelerometer)),
	};

	LOG_DBG("Orientation module task started");

	if (!device_is_ready(orientation_state.accel)) {
		LOG_ERR("Accelerometer device is not ready");
		SEND_FATAL_ERROR();
		return;
	}

	smf_set_initial(SMF_CTX(&orientation_state), &states[STATE_RUNNING]);

	while (true) {
		/* No watchdog on this thread (see note above), so just block
		 * indefinitely for the next sample request instead of waking
		 * periodically to feed a channel that doesn't exist.
		 */
		err = zbus_sub_wait_msg(&orientation,
					&orientation_state.chan,
					orientation_state.msg_buf,
					K_FOREVER);
		if (err == -ENOMSG) {
			continue;
		} else if (err) {
			LOG_ERR("zbus_sub_wait_msg, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = smf_run_state(SMF_CTX(&orientation_state));
		if (err) {
			LOG_ERR("smf_run_state(), error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}
	}
}

K_THREAD_DEFINE(orientation_module_thread_id,
		CONFIG_APP_ORIENTATION_THREAD_STACK_SIZE,
		orientation_module_thread, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);