/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <date_time.h>

#include "app_common.h"
#include "impact.h"
#include "storage.h"

/* Register log module */
LOG_MODULE_REGISTER(impact, CONFIG_APP_IMPACT_LOG_LEVEL);

/* Standard gravity, m/s^2. Used to convert m/s^2 <-> g. */
#define STANDARD_GRAVITY 9.80665

/* Threshold below which a magnitude reading is assumed to already be in m/s^2
 * (ADXL362, original Thingy:91) rather than in g (ADXL367, Thingy:91 X).
 * Mirrors the same heuristic used in orientation.c and pedometer.c.
 */
#define G_VS_MS2_HEURISTIC_THRESHOLD 4.0

/* Number of sample ticks to keep tracking the peak after a threshold crossing,
 * before publishing the highest value seen and entering debounce.
 */
#define CAPTURE_TICKS \
	MAX(1, (CONFIG_APP_IMPACT_CAPTURE_WINDOW_MS / CONFIG_APP_IMPACT_SAMPLE_INTERVAL_MS))

ZBUS_CHAN_DEFINE(impact_chan,
		 struct impact_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0)
);

ZBUS_CHAN_DEFINE(motion_chan,
                 struct motion_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT(0)
);

static const struct device *const accel_dev = DEVICE_DT_GET(DT_ALIAS(accelerometer));

/* Capture-in-progress state. Only ever touched from the impact sample work
 * item, which always runs on the system workqueue, so no locking is needed.
 */
static bool capturing;
static double capture_peak_g;
static int64_t capture_peak_ts;
static int32_t capture_ticks_left;
static int64_t debounce_until_uptime_ms;

static void sample_for_impact_detection(struct k_work *work);

static K_WORK_DEFINE(impact_sample_work, sample_for_impact_detection);

static void impact_sample_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_work_submit(&impact_sample_work);
}

static K_TIMER_DEFINE(impact_sample_timer, impact_sample_timer_handler, NULL);

/**
 * @brief Normalize an accelerometer magnitude to m/s^2.
 *
 * ADXL367 (Thingy:91 X) reports acceleration in g, ADXL362 (original
 * Thingy:91) reports in m/s^2. If the magnitude looks like it's already in
 * g (i.e. well below what any real m/s^2 reading would be at rest), scale it
 * up. Identical heuristic to the one used in orientation.c and pedometer.c.
 */
static double normalize_to_m_s2(double magnitude)
{
	if (magnitude < G_VS_MS2_HEURISTIC_THRESHOLD) {
		return magnitude * STANDARD_GRAVITY;
	}

	return magnitude;
}

static int64_t timestamp_now(void)
{
	int64_t timestamp = k_uptime_get();

	/* Best-effort convert to unix time; on failure, keep the uptime value.
	 * cloud.c performs the same conversion again downstream and will
	 * handle it per CONFIG_APP_CLOUD_HANDLE_WRONG_SAMPLE_TIMESTAMPS_*.
	 */
	(void)date_time_now(&timestamp);

	return timestamp;
}

/* Number of sample ticks that make up one ~1-second motion-detection window. */
#define MOTION_WINDOW_TICKS \
        MAX(1, (1000 / CONFIG_APP_IMPACT_SAMPLE_INTERVAL_MS))

/* Number of consecutive still windows (each ~1 second) required before
 * declaring the device stationary again.
 */
#define MOTION_STILL_WINDOWS \
        MAX(1, ((CONFIG_APP_IMPACT_MOTION_STILL_TIMEOUT_SECONDS * 1000) / \
                (MOTION_WINDOW_TICKS * CONFIG_APP_IMPACT_SAMPLE_INTERVAL_MS)))

/* Motion-detection state. Only ever touched from the impact sample work
 * item, same as the capture state above - no locking needed.
 */
static double motion_window_min_g = 1000.0;
static double motion_window_max_g = -1000.0;
static int32_t motion_window_ticks_left = MOTION_WINDOW_TICKS;
static bool is_moving;
static uint32_t still_windows_count;

/* Number of sample ticks between periodic activity-summary reports. */
#define ACTIVITY_REPORT_TICKS \
        MAX(1, ((CONFIG_APP_IMPACT_ACTIVITY_REPORT_INTERVAL_SECONDS * 1000) / \
                CONFIG_APP_IMPACT_SAMPLE_INTERVAL_MS))

/* Activity-summary state. Tracks the highest g seen (regardless of whether it
 * crossed the hard impact threshold) since the last periodic report. Only
 * ever touched from the impact sample work item - no locking needed.
 */
static double activity_peak_g = -1000.0;
static int64_t activity_peak_ts;
static int32_t activity_ticks_left = ACTIVITY_REPORT_TICKS;

static void report_motion_state(bool moving)
{
        int err;
        struct motion_msg msg = {
                .is_moving = moving,
        };

        LOG_INF("Motion %s", moving ? "started" : "stopped");

        err = zbus_chan_pub(&motion_chan, &msg, PUB_TIMEOUT);
        if (err) {
                LOG_ERR("Failed to publish motion message, error: %d", err);
                SEND_FATAL_ERROR();
        }
}

/**
 * @brief Track whether the device is moving, based on peak-to-peak
 * acceleration variation over rolling ~1-second windows.
 *
 * Independent of the impact-capture state machine below - this always runs,
 * even while an impact capture is in progress or during debounce, since
 * motion and impact are different questions answered from the same stream
 * of samples.
 */
static void update_motion_state(double g)
{
        double range_g;
        bool window_had_motion;

        if (g < motion_window_min_g) {
                motion_window_min_g = g;
        }
        if (g > motion_window_max_g) {
                motion_window_max_g = g;
        }

        if (--motion_window_ticks_left > 0) {
                return;
        }

        range_g = motion_window_max_g - motion_window_min_g;
        window_had_motion = range_g >= (CONFIG_APP_IMPACT_MOTION_THRESHOLD_MG / 1000.0);

        if (window_had_motion) {
                still_windows_count = 0;

                if (!is_moving) {
                        is_moving = true;
                        report_motion_state(true);
                }
        } else if (is_moving) {
                still_windows_count++;

                if (still_windows_count >= MOTION_STILL_WINDOWS) {
                        is_moving = false;
                        report_motion_state(false);
                }
        }

        motion_window_min_g = g;
        motion_window_max_g = g;
        motion_window_ticks_left = MOTION_WINDOW_TICKS;
}

static void report_impact(double peak_g, int64_t timestamp)
{
	int err;
	struct impact_msg msg = {
		.type = IMPACT_SAMPLE_RESPONSE,
		.peak_g = peak_g,
		.timestamp = timestamp,
	};
	struct storage_msg flush_msg = {
		.type = STORAGE_FLUSH,
	};

	LOG_WRN("Impact detected: %.2f g", peak_g);

	err = zbus_chan_pub(&impact_chan, &msg, PUB_TIMEOUT);
	if (err) {
		LOG_ERR("Failed to publish impact message, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	/* Force an immediate cloud delivery instead of waiting for the next
	 * periodic batch cycle (which can be many minutes away). storage_chan
	 * and impact_chan share a single storage_subscriber processing thread,
	 * so this flush is guaranteed to be handled after the impact message
	 * above has already been stored.
	 */
	err = zbus_chan_pub(&storage_chan, &flush_msg, PUB_TIMEOUT);
	if (err) {
		LOG_ERR("Failed to publish storage flush request, error: %d", err);
		SEND_FATAL_ERROR();
	}
}

/**
 * @brief Publish a periodic activity summary - the peak g-force seen since
 * the last report, regardless of whether it crossed the hard impact
 * threshold.
 *
 * Deliberately not urgent: no STORAGE_FLUSH here, unlike report_impact()
 * above. This just rides along on the normal storage/cloud batch cycle -
 * there's no reason a routine "here's how active things have been" update
 * needs to jump the queue the way a real impact event does.
 */
static void report_activity_summary(double peak_g, int64_t timestamp)
{
	int err;
	struct impact_msg msg = {
		.type = IMPACT_ACTIVITY_SUMMARY,
		.peak_g = peak_g,
		.timestamp = timestamp,
	};

	LOG_INF("Activity summary: peak %.2f g since last report", peak_g);

	err = zbus_chan_pub(&impact_chan, &msg, PUB_TIMEOUT);
	if (err) {
		LOG_ERR("Failed to publish activity summary message, error: %d", err);
		SEND_FATAL_ERROR();
	}
}

/**
 * @brief Track the peak g seen since the last activity-summary report, and
 * publish/reset once the report interval elapses.
 *
 * Runs unconditionally on every sample, regardless of impact-capture or
 * debounce state, so a hard impact's own peak still counts toward the
 * activity summary too (it's still exertion, just also big enough to have
 * triggered its own separate alert).
 */
static void update_activity_summary(double g, int64_t now_ts)
{
	if (g > activity_peak_g) {
		activity_peak_g = g;
		activity_peak_ts = now_ts;
	}

	if (--activity_ticks_left > 0) {
		return;
	}

	report_activity_summary(activity_peak_g, activity_peak_ts);

	activity_peak_g = -1000.0;
	activity_ticks_left = ACTIVITY_REPORT_TICKS;
}

static void sample_for_impact_detection(struct k_work *work)
{
	int err;
	struct sensor_value accel[3];
	double x, y, z, magnitude, g;
	int64_t now = k_uptime_get();

	ARG_UNUSED(work);

	err = sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_ACCEL_XYZ);
	if (err) {
		LOG_ERR("sensor_sample_fetch_chan, error: %d", err);
		return;
	}

	err = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (err) {
		LOG_ERR("sensor_channel_get, error: %d", err);
		return;
	}

	x = sensor_value_to_double(&accel[0]);
	y = sensor_value_to_double(&accel[1]);
	z = sensor_value_to_double(&accel[2]);

	magnitude = normalize_to_m_s2(sqrt((x * x) + (y * y) + (z * z)));
	g = magnitude / STANDARD_GRAVITY;
	static uint32_t dbg_counter;
if (++dbg_counter % 25 == 0) {
	LOG_DBG("impact poll: g=%.3f (threshold=%.3f)", g,
		CONFIG_APP_IMPACT_THRESHOLD_MG / 1000.0);
}
	update_motion_state(g);
	update_activity_summary(g, timestamp_now());
	if (capturing) {
		if (g > capture_peak_g) {
			capture_peak_g = g;
			capture_peak_ts = timestamp_now();
		}

		capture_ticks_left--;
		if (capture_ticks_left <= 0) {
			report_impact(capture_peak_g, capture_peak_ts);
			capturing = false;
			debounce_until_uptime_ms = now + CONFIG_APP_IMPACT_DEBOUNCE_MS;
		}

		return;
	}

	if (now < debounce_until_uptime_ms) {
		/* Still cooling down from the last reported event. */
		return;
	}

	if (g >= (CONFIG_APP_IMPACT_THRESHOLD_MG / 1000.0)) {
		capturing = true;
		capture_peak_g = g;
		capture_peak_ts = timestamp_now();
		capture_ticks_left = CAPTURE_TICKS;
	}
}

static void impact_module_thread(void)
{
	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Accelerometer device not ready");
		SEND_FATAL_ERROR();
		return;
	}

	LOG_INF("Impact detection active: sampling every %d ms, threshold %d mg (%.1f g), "
		"capture window %d ms, debounce %d ms, activity report every %d s",
		CONFIG_APP_IMPACT_SAMPLE_INTERVAL_MS,
		CONFIG_APP_IMPACT_THRESHOLD_MG,
		CONFIG_APP_IMPACT_THRESHOLD_MG / 1000.0,
		CONFIG_APP_IMPACT_CAPTURE_WINDOW_MS,
		CONFIG_APP_IMPACT_DEBOUNCE_MS,
		CONFIG_APP_IMPACT_ACTIVITY_REPORT_INTERVAL_SECONDS);

	/* No zbus subscriber loop: this module never consumes external
	 * messages, it only detects and publishes. Everything after this runs
	 * off the k_timer/k_work pair above; this thread has nothing further
	 * to do and is not registered with the task watchdog (same rationale
	 * as orientation/pedometer: the nRF91 has a fixed 8-channel hardware
	 * watchdog ceiling already claimed by other modules).
	 */
	k_timer_start(&impact_sample_timer,
		     K_MSEC(CONFIG_APP_IMPACT_SAMPLE_INTERVAL_MS),
		     K_MSEC(CONFIG_APP_IMPACT_SAMPLE_INTERVAL_MS));
}

K_THREAD_DEFINE(impact_module_thread_id, CONFIG_APP_IMPACT_THREAD_STACK_SIZE,
		impact_module_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);