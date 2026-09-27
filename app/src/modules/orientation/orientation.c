/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>
#include <date_time.h>

#include "orientation.h"

/* Register log module */
LOG_MODULE_REGISTER(orientation, CONFIG_APP_ORIENTATION_LOG_LEVEL);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PUB_TIMEOUT K_SECONDS(1)

/* Threshold (in m/s^2) below which we assume the accelerometer is reporting
 * in units of g rather than m/s^2 (ADXL367 on Thingy:91 X reports in g,
 * while ADXL362 on the original Thingy:91 reports in m/s^2). If the
 * magnitude of the raw reading is below this, scale it up by standard
 * gravity to normalize everything to m/s^2.
 */
#define G_VS_MS2_MAGNITUDE_THRESHOLD 4.0
#define STANDARD_GRAVITY 9.80665

ZBUS_CHAN_DEFINE(orientation_chan,
		  struct orientation_msg,
		  NULL,
		  NULL,
		  ZBUS_OBSERVERS_EMPTY,
		  ZBUS_MSG_INIT(0)
);

ZBUS_MSG_SUBSCRIBER_DEFINE(orientation);
ZBUS_CHAN_ADD_OBS(orientation_chan, orientation, 0);

/* State machine states */
enum orientation_module_state {
	STATE_RUNNING,
};

struct orientation_state_object {
	/* This must be first */
	struct smf_ctx ctx;

	/* Last zbus message received */
	const struct zbus_channel *chan;

	/* Buffer for zbus message data */
	uint8_t msg_buf[sizeof(struct orientation_msg)];
};

static const struct device *const accel_dev = DEVICE_DT_GET(DT_ALIAS(accelerometer));

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

/* Normalize a raw accelerometer reading to m/s^2 regardless of whether the
 * underlying chip natively reports in g (ADXL367) or m/s^2 (ADXL362).
 */
static void normalize_to_m_s2(double *x, double *y, double *z)
{
	double magnitude = sqrt((*x) * (*x) + (*y) * (*y) + (*z) * (*z));

	if (magnitude < G_VS_MS2_MAGNITUDE_THRESHOLD) {
		*x *= STANDARD_GRAVITY;
		*y *= STANDARD_GRAVITY;
		*z *= STANDARD_GRAVITY;
	}
}

/* Read the accelerometer, compute dominant-axis face and tilt, and publish
 * an ORIENTATION_SAMPLE_RESPONSE message on orientation_chan.
 */
static void sample_orientation(const struct device *const accel)
{
	struct sensor_value accel_xyz[3];
	int err;
	double x, y, z;
	double abs_x, abs_y, abs_z;
	double dominant_magnitude;
	double tilt_deg;
	enum orientation_face face;
	struct orientation_msg msg = {
		.type = ORIENTATION_SAMPLE_RESPONSE,
	};

	err = sensor_sample_fetch_chan(accel, SENSOR_CHAN_ALL);
	if (err) {
		LOG_ERR("sensor_sample_fetch_chan, error: %d", err);
		return;
	}

	err = sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);
	if (err) {
		LOG_ERR("sensor_channel_get, error: %d", err);
		return;
	}

	x = sensor_value_to_double(&accel_xyz[0]);
	y = sensor_value_to_double(&accel_xyz[1]);
	z = sensor_value_to_double(&accel_xyz[2]);

	normalize_to_m_s2(&x, &y, &z);

	abs_x = fabs(x);
	abs_y = fabs(y);
	abs_z = fabs(z);

	/* Determine which axis is dominant (closest to aligned with gravity)
	 * and whether that axis is pointing up or down.
	 */
	if (abs_x >= abs_y && abs_x >= abs_z) {
		face = (x >= 0) ? ORIENTATION_FACE_X_UP : ORIENTATION_FACE_X_DOWN;
		dominant_magnitude = abs_x;
	} else if (abs_y >= abs_x && abs_y >= abs_z) {
		face = (y >= 0) ? ORIENTATION_FACE_Y_UP : ORIENTATION_FACE_Y_DOWN;
		dominant_magnitude = abs_y;
	} else {
		face = (z >= 0) ? ORIENTATION_FACE_Z_UP : ORIENTATION_FACE_Z_DOWN;
		dominant_magnitude = abs_z;
	}

	/* Tilt = angle between the dominant axis and gravity, derived from
	 * how much of total gravity magnitude is captured by the dominant
	 * axis alone. 0 deg = flat on that face, 90 deg = on its side.
	 */
	double total_magnitude = sqrt(x * x + y * y + z * z);

	if (total_magnitude > 0.0) {
		double ratio = dominant_magnitude / total_magnitude;

		/* Clamp for floating point safety before acos() */
		if (ratio > 1.0) {
			ratio = 1.0;
		} else if (ratio < -1.0) {
			ratio = -1.0;
		}

		tilt_deg = acos(ratio) * (180.0 / M_PI);
	} else {
		tilt_deg = 0.0;
	}

	msg.face = face;
	msg.tilt_deg = tilt_deg;
	msg.x = x;
	msg.y = y;
	msg.z = z;

	if (date_time_now(&msg.timestamp)) {
		/* Fall back to uptime if the system clock isn't synchronized yet */
		msg.timestamp = k_uptime_get();
	}

	LOG_DBG("Orientation: %s, tilt %.1f deg, x=%.2f y=%.2f z=%.2f",
		orientation_face_to_str(face), tilt_deg, x, y, z);

	err = zbus_chan_pub(&orientation_chan, &msg, PUB_TIMEOUT);
	if (err) {
		LOG_ERR("zbus_chan_pub, error: %d", err);
	}
}

/* State handlers */
static void state_running_entry(void *o)
{
	ARG_UNUSED(o);

	LOG_DBG("state_running_entry");

	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Accelerometer device not ready");
	}
}

static enum smf_state_result state_running_run(void *o)
{
	struct orientation_state_object *state_object = o;
	struct orientation_msg msg;

	if (state_object->chan != &orientation_chan) {
		return SMF_EVENT_HANDLED;
	}

	memcpy(&msg, state_object->msg_buf, sizeof(msg));

	if (msg.type == ORIENTATION_SAMPLE_REQUEST) {
		LOG_DBG("Orientation sample request received, getting data");
		sample_orientation(accel_dev);
	}

	return SMF_EVENT_HANDLED;
}

static const struct smf_state orientation_states[] = {
	[STATE_RUNNING] = SMF_CREATE_STATE(
		state_running_entry,
		state_running_run,
		NULL,
		NULL,
		NULL
	),
};

static void orientation_module_thread(void)
{
	int err;
	struct orientation_state_object orientation_state = { 0 };

	smf_set_initial(SMF_CTX(&orientation_state), &orientation_states[STATE_RUNNING]);

	while (true) {
		err = zbus_sub_wait_msg(&orientation,
					 &orientation_state.chan,
					 orientation_state.msg_buf,
					 K_FOREVER);
		if (err) {
			LOG_ERR("zbus_sub_wait_msg, error: %d", err);
			continue;
		}

		err = smf_run_state(SMF_CTX(&orientation_state));
		if (err) {
			LOG_ERR("smf_run_state, error: %d", err);
		}
	}
}

K_THREAD_DEFINE(orientation_module_thread_id,
		 CONFIG_APP_ORIENTATION_THREAD_STACK_SIZE,
		 orientation_module_thread, NULL, NULL, NULL,
		 K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);