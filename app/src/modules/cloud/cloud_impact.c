/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <net/nrf_cloud_coap.h>

#include "cloud_impact.h"

LOG_MODULE_REGISTER(cloud_impact, CONFIG_APP_CLOUD_LOG_LEVEL);

#define CUSTOM_JSON_APPID_VAL_IMPACT "IMPACT"
#define CUSTOM_JSON_APPID_VAL_ACTIVITY "ACTIVITY"
/* FALL carries the free-fall duration and the landing peak together, so it is
 * sent as a raw JSON message rather than through nrf_cloud_coap_sensor_send(),
 * which only accepts a single double value.
 */
#define CUSTOM_JSON_APPID_VAL_FALL "FALL"

static int cloud_fall_send(const struct impact_msg *fall, int64_t timestamp_ms, bool confirmable)
{
	int err;
	char json[160];

	if (timestamp_ms > 0) {
		err = snprintk(json, sizeof(json),
			"{\"appId\":\"%s\",\"messageType\":\"DATA\",\"ts\":%lld,"
			"\"data\":{\"ms\":%u,\"peak\":%.2f}}",
			CUSTOM_JSON_APPID_VAL_FALL, timestamp_ms,
			(unsigned int)fall->fall_ms, fall->peak_g);
	} else {
		err = snprintk(json, sizeof(json),
			"{\"appId\":\"%s\",\"messageType\":\"DATA\","
			"\"data\":{\"ms\":%u,\"peak\":%.2f}}",
			CUSTOM_JSON_APPID_VAL_FALL,
			(unsigned int)fall->fall_ms, fall->peak_g);
	}

	if (err < 0 || err >= (int)sizeof(json)) {
		LOG_ERR("Fall JSON too long");
		return -ENOMEM;
	}

	err = nrf_cloud_coap_json_message_send(json, false, confirmable);
	if (err) {
		LOG_ERR("Failed to send fall data to cloud, error: %d", err);
		return err;
	}

	LOG_WRN("Fall data sent to cloud: %u ms, landing peak %.2f g",
		(unsigned int)fall->fall_ms, fall->peak_g);

	return 0;
}

int cloud_impact_send(const struct impact_msg *impact, int64_t timestamp_ms, bool confirmable)
{
	int err;

	if (impact->type == IMPACT_FREE_FALL) {
		return cloud_fall_send(impact, timestamp_ms, confirmable);
	}

	const char *appid = (impact->type == IMPACT_ACTIVITY_SUMMARY) ?
		CUSTOM_JSON_APPID_VAL_ACTIVITY : CUSTOM_JSON_APPID_VAL_IMPACT;

	err = nrf_cloud_coap_sensor_send(appid,
					 impact->peak_g,
					 timestamp_ms,
					 confirmable);
	if (err) {
		LOG_ERR("Failed to send %s data to cloud, error: %d", appid, err);
		return err;
	}

	if (impact->type == IMPACT_ACTIVITY_SUMMARY) {
		LOG_INF("Activity summary sent to cloud: %.2f g", impact->peak_g);
	} else {
		LOG_WRN("Impact data sent to cloud: %.2f g", impact->peak_g);
	}

	return 0;
}