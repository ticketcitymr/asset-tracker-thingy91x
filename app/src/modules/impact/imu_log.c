/*
 * BMI270 log-only bring-up.
 *
 * Every CONFIG_APP_BMI270_LOG_PERIOD_SECONDS this wakes the BMI270 (6-axis IMU), takes a one second
 * burst of readings at 100 Hz with the accelerometer at +/-16 g and the gyroscope at +/-2000 dps,
 * writes a one-line summary to the log, and puts both sensors back to sleep.
 *
 * It sends nothing to the cloud and changes no alerts. Purpose: prove the chip works on this board,
 * see what the +/-16 g range and the gyroscope read in real use, and measure the battery cost
 * before any alert logic depends on it.
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_log, CONFIG_APP_IMPACT_LOG_LEVEL);

#define STANDARD_GRAVITY 9.80665
#define RAD_TO_DEG 57.29577951308232

#define BURST_SAMPLES 100
#define BURST_SAMPLE_MS 10
/* Time the sensors need after power-up before the first reading is valid. */
#define STARTUP_MS 80

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(accelerometer_hp));

static int imu_set(enum sensor_channel chan, enum sensor_attribute attr, int32_t value)
{
	struct sensor_value val = { .val1 = value, .val2 = 0 };

	return sensor_attr_set(imu, chan, attr, &val);
}

/* Both sensors on (hz > 0) or off (hz == 0). */
static int imu_rate(int32_t hz)
{
	int err = imu_set(SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, hz);

	if (err) {
		return err;
	}
	return imu_set(SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, hz);
}

static void burst(void)
{
	struct sensor_value a[3], g[3];
	double peak_a = 0.0, peak_g = 0.0, sum[3] = { 0.0, 0.0, 0.0 };
	int n = 0;
	int err = imu_rate(100);

	if (err) {
		LOG_ERR("imu: could not start sensors: %d", err);
		return;
	}
	k_msleep(STARTUP_MS);

	for (int i = 0; i < BURST_SAMPLES; i++) {
		/* This driver only supports fetching all channels at once. */
		err = sensor_sample_fetch(imu);
		if (err) {
			LOG_ERR("imu: fetch failed: %d", err);
			break;
		}
		if (sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, a) ||
		    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, g)) {
			LOG_ERR("imu: channel read failed");
			break;
		}

		double ax = sensor_value_to_double(&a[0]) / STANDARD_GRAVITY;
		double ay = sensor_value_to_double(&a[1]) / STANDARD_GRAVITY;
		double az = sensor_value_to_double(&a[2]) / STANDARD_GRAVITY;
		double gx = sensor_value_to_double(&g[0]) * RAD_TO_DEG;
		double gy = sensor_value_to_double(&g[1]) * RAD_TO_DEG;
		double gz = sensor_value_to_double(&g[2]) * RAD_TO_DEG;
		double am = sqrt(ax * ax + ay * ay + az * az);
		double gm = sqrt(gx * gx + gy * gy + gz * gz);

		if (am > peak_a) {
			peak_a = am;
		}
		if (gm > peak_g) {
			peak_g = gm;
		}
		sum[0] += ax;
		sum[1] += ay;
		sum[2] += az;
		n++;
		k_msleep(BURST_SAMPLE_MS);
	}

	(void)imu_rate(0);

	if (n > 0) {
		LOG_INF("imu: n=%d accel peak %d mg, mean x/y/z %d/%d/%d mg, gyro peak %d dps",
			n, (int)(peak_a * 1000.0), (int)(sum[0] / n * 1000.0),
			(int)(sum[1] / n * 1000.0), (int)(sum[2] / n * 1000.0), (int)peak_g);
	}
}

static void imu_log_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Let the rest of the application start first. */
	k_sleep(K_SECONDS(30));

	if (!device_is_ready(imu)) {
		LOG_ERR("imu: BMI270 is not ready (check the devicetree overlay and SPI wiring)");
		return;
	}
	if (imu_set(SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, 16) ||
	    imu_set(SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, 2000)) {
		LOG_ERR("imu: could not set ranges");
		return;
	}
	LOG_INF("imu: BMI270 ready, accel +/-16 g, gyro +/-2000 dps, one burst every %d s",
		CONFIG_APP_BMI270_LOG_PERIOD_SECONDS);

	while (true) {
		burst();
		k_sleep(K_SECONDS(CONFIG_APP_BMI270_LOG_PERIOD_SECONDS));
	}
}

K_THREAD_DEFINE(imu_log_tid, 1536, imu_log_thread, NULL, NULL, NULL, 10, 0, 0);
