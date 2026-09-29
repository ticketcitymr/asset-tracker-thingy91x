/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <net/nrf_cloud_coap.h>

#include "cloud_environmental.h"

LOG_MODULE_DECLARE(cloud, CONFIG_APP_CLOUD_LOG_LEVEL);

#if defined(CONFIG_BME68X_IAQ)
/* IAQ is a custom appId. It carries the air quality index, CO2 and VOC equivalents and the
 * BSEC accuracy together in one JSON payload, so it is sent as a raw message.
 */
#define CUSTOM_JSON_APPID_VAL_IAQ "IAQ"

static int cloud_iaq_send(const struct environmental_msg *env,
			  int64_t timestamp_ms,
			  bool confirmable)
{
	int err;
	char json[192];

	if (timestamp_ms > 0) {
		err = snprintk(json, sizeof(json),
			"{\"appId\":\"%s\",\"messageType\":\"DATA\",\"ts\":%lld,"
			"\"data\":{\"iaq\":%.1f,\"co2\":%.0f,\"voc\":%.2f,\"accuracy\":%d}}",
			CUSTOM_JSON_APPID_VAL_IAQ, timestamp_ms,
			env->iaq, env->co2, env->voc, env->iaq_accuracy);
	} else {
		err = snprintk(json, sizeof(json),
			"{\"appId\":\"%s\",\"messageType\":\"DATA\","
			"\"data\":{\"iaq\":%.1f,\"co2\":%.0f,\"voc\":%.2f,\"accuracy\":%d}}",
			CUSTOM_JSON_APPID_VAL_IAQ,
			env->iaq, env->co2, env->voc, env->iaq_accuracy);
	}

	if (err < 0 || err >= (int)sizeof(json)) {
		LOG_ERR("IAQ JSON too long");
		return -ENOMEM;
	}

	err = nrf_cloud_coap_json_message_send(json, false, confirmable);
	if (err) {
		LOG_ERR("Failed to send IAQ data to cloud, error: %d", err);
		return err;
	}

	LOG_DBG("IAQ data sent to cloud: IAQ=%.1f, accuracy=%d",
		env->iaq, env->iaq_accuracy);

	return 0;
}
#endif /* CONFIG_BME68X_IAQ */

int cloud_environmental_send(const struct environmental_msg *env,
			     int64_t timestamp_ms,
			     bool confirmable)
{
	int err;

	err = nrf_cloud_coap_sensor_send(NRF_CLOUD_JSON_APPID_VAL_TEMP,
					 env->temperature,
					 timestamp_ms,
					 confirmable);
	if (err) {
		LOG_ERR("Failed to send temperature data to cloud, error: %d", err);
		return err;
	}

	err = nrf_cloud_coap_sensor_send(NRF_CLOUD_JSON_APPID_VAL_AIR_PRESS,
					 env->pressure,
					 timestamp_ms,
					 confirmable);
	if (err) {
		LOG_ERR("Failed to send pressure data to cloud, error: %d", err);
		return err;
	}

	err = nrf_cloud_coap_sensor_send(NRF_CLOUD_JSON_APPID_VAL_HUMID,
					 env->humidity,
					 timestamp_ms,
					 confirmable);
	if (err) {
		LOG_ERR("Failed to send humidity data to cloud, error: %d", err);
		return err;
	}

#if defined(CONFIG_BME68X_IAQ)
	err = cloud_iaq_send(env, timestamp_ms, confirmable);
	if (err) {
		return err;
	}
#endif

	LOG_DBG("Environmental data sent to cloud: T=%.1f°C, P=%.1fhPa, H=%.1f%%",
		(double)env->temperature, (double)env->pressure, (double)env->humidity);

	return 0;
}
