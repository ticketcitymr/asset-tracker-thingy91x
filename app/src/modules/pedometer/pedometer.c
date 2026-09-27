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

#include "pedometer.h"

LOG_MODULE_REGISTER(pedometer, CONFIG_APP_PEDOMETER_LOG_LEVEL);

#define PUB_TIMEOUT K_SECONDS(1)

/* Same g-vs-m/s^2 normalization used by the orientation module: ADXL367
 * (Thingy:91 X) reports in g, ADXL362 (original Thingy:91) reports in m/s^2.
 */
#define G_VS_MS2_MAGNITUDE_THRESHOLD 4.0
#define STANDARD_GRAVITY 9.80665

/* Kconfig values arrive as integers to avoid floating point in Kconfig;
 * convert once here.
 */
#define STEP_THRESHOLD_MS2 (CONFIG_APP_PEDOMETER_STEP_THRESHOLD_MM_S2 / 1000.0)
#define STEP_DEBOUNCE_MS   CONFIG_APP_PEDOMETER_STEP_DEBOUNCE_MS

ZBUS_CHAN_DEFINE(pedometer_chan,
		  struct pedometer_msg,
		  NULL,
		  NULL,
		  ZBUS_OBSERVERS_EMPTY,
		  ZBUS_MSG_INIT(0)
);

ZBUS_MSG_SUBSCRIBER_DEFINE(pedometer);
ZBUS_CHAN_ADD_OBS(pedometer_chan, pedometer, 0);

/* State machine states */
enum pedometer_module_state {
	STATE_RUNNING,
};

struct pedometer_state_object {
	/* This must be first */
	struct smf_ctx ctx;

	/* Last zbus message received */
	const struct zbus_channel *chan;

	/* Buffer for zbus message data */
	uint8_t msg_buf[sizeof(struct pedometer_msg)];
};

static const struct device *const accel_dev = DEVICE_DT_GET(DT_ALIAS(accelerometer));

/* Step counters. Updated from the high-rate sampling work item, read/reset
 * from the zbus consumer thread on a PEDOMETER_SAMPLE_REQUEST. Both run in
 * system work queue / cooperative threads at the same priority tier, but we
 * still use atomics since the two are logically independent producers/
 * consumers of this state.
 */
static atomic_t total_steps = ATOMIC_INIT(0);
static atomic_t steps_since_last_report = ATOMIC_INIT(0);

/* Rising-edge + debounce state for the step detector. Only ever touched from
 * the sampling work item, so no synchronization needed here.
 */
static bool above_threshold;
static int64_t last_step_uptime_ms;

static void normalize_to_m_s2(double *x, double *y, double *z)
{
	double magnitude = sqrt((*x) * (*x) + (*y) * (*y) + (*z) * (*z));

	if (magnitude < G_VS_MS2_MAGNITUDE_THRESHOLD) {
		*x *= STANDARD_GRAVITY;
		*y *= STANDARD_GRAVITY;
		*z *= STANDARD_GRAVITY;
	}
}

/* High-rate accelerometer poll used only for step detection. Runs
 * independently of the module's zbus-driven sample/report cycle.
 *
 * Algorithm: track the "dynamic" acceleration (deviation of the vector
 * magnitude from standard gravity). A step is counted on the rising edge
 * of that dynamic acceleration crossing STEP_THRESHOLD_MS2, gated by a
 * debounce window so a single footfall's ringing isn't counted multiple
 * times.
 */
static void sample_for_step_detection(struct k_work *work)
{
	ARG_UNUSED(work);

	struct sensor_value accel_xyz[3];
	int err;
	double x, y, z, magnitude, dynamic_accel;
	int64_t now_ms;

	if (!device_is_ready(accel_dev)) {
		return;
	}

	err = sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_ALL);
	if (err) {
		LOG_ERR("sensor_sample_fetch_chan, error: %d", err);
		return;
	}

	err = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);
	if (err) {
		LOG_ERR("sensor_channel_get, error: %d", err);
		return;
	}

	x = sensor_value_to_double(&accel_xyz[0]);
	y = sensor_value_to_double(&accel_xyz[1]);
	z = sensor_value_to_double(&accel_xyz[2]);

	normalize_to_m_s2(&x, &y, &z);

	magnitude = sqrt(x * x + y * y + z * z);
	dynamic_accel = fabs(magnitude - STANDARD_GRAVITY);

	now_ms = k_uptime_get();

	if (!above_threshold && dynamic_accel >= STEP_THRESHOLD_MS2) {
		/* Rising edge. Only count it if we're past the debounce
		 * window since the last counted step.
		 */
		if ((now_ms - last_step_uptime_ms) >= STEP_DEBOUNCE_MS) {
			atomic_inc(&total_steps);
			atomic_inc(&steps_since_last_report);
			last_step_uptime_ms = now_ms;

			LOG_DBG("Step detected, dynamic accel %.2f m/s^2, total steps %d",
				dynamic_accel, (int)atomic_get(&total_steps));
		}
		above_threshold = true;
	} else if (above_threshold && dynamic_accel < STEP_THRESHOLD_MS2) {
		above_threshold = false;
	}
}

K_WORK_DEFINE(step_sample_work, sample_for_step_detection);

static void step_sample_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	/* Defer the actual sensor I/O to the system work queue; keep the
	 * timer ISR itself trivial.
	 */
	k_work_submit(&step_sample_work);
}

K_TIMER_DEFINE(step_sample_timer, step_sample_timer_handler, NULL);

/* Build and publish a PEDOMETER_SAMPLE_RESPONSE with the steps accumulated
 * since the previous report, then reset the delta counter.
 */
static void report_pedometer(void)
{
	int err;
	struct pedometer_msg msg = {
		.type = PEDOMETER_SAMPLE_RESPONSE,
	};

	msg.steps_since_last_report = (uint32_t)atomic_set(&steps_since_last_report, 0);
	msg.total_steps = (uint64_t)atomic_get(&total_steps);

	if (date_time_now(&msg.timestamp)) {
		msg.timestamp = k_uptime_get();
	}

	LOG_DBG("Pedometer report: %u steps since last report, %llu total",
		msg.steps_since_last_report, msg.total_steps);

	err = zbus_chan_pub(&pedometer_chan, &msg, PUB_TIMEOUT);
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
		return;
	}

	last_step_uptime_ms = k_uptime_get();

	k_timer_start(&step_sample_timer,
		      K_MSEC(CONFIG_APP_PEDOMETER_SAMPLE_INTERVAL_MS),
		      K_MSEC(CONFIG_APP_PEDOMETER_SAMPLE_INTERVAL_MS));
}

static enum smf_state_result state_running_run(void *o)
{
	struct pedometer_state_object *state_object = o;
	struct pedometer_msg msg;

	if (state_object->chan != &pedometer_chan) {
		return SMF_EVENT_HANDLED;
	}

	memcpy(&msg, state_object->msg_buf, sizeof(msg));

	if (msg.type == PEDOMETER_SAMPLE_REQUEST) {
		LOG_DBG("Pedometer sample request received, reporting steps");
		report_pedometer();
	}

	return SMF_EVENT_HANDLED;
}

static const struct smf_state pedometer_states[] = {
	[STATE_RUNNING] = SMF_CREATE_STATE(
		state_running_entry,
		state_running_run,
		NULL,
		NULL,
		NULL
	),
};

static void pedometer_module_thread(void)
{
	int err;
	struct pedometer_state_object pedometer_state = { 0 };

	smf_set_initial(SMF_CTX(&pedometer_state), &pedometer_states[STATE_RUNNING]);

	while (true) {
		err = zbus_sub_wait_msg(&pedometer,
					 &pedometer_state.chan,
					 pedometer_state.msg_buf,
					 K_FOREVER);
		if (err) {
			LOG_ERR("zbus_sub_wait_msg, error: %d", err);
			continue;
		}

		err = smf_run_state(SMF_CTX(&pedometer_state));
		if (err) {
			LOG_ERR("smf_run_state, error: %d", err);
		}
	}
}

K_THREAD_DEFINE(pedometer_module_thread_id,
		 CONFIG_APP_PEDOMETER_THREAD_STACK_SIZE,
		 pedometer_module_thread, NULL, NULL, NULL,
		 K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
