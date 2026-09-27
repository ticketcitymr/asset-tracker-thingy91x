/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/logging/log.h>
#include <net/nrf_cloud_coap.h>

#include "cloud_impact.h"

LOG_MODULE_REGISTER(cloud_impact, CONFIG_APP_CLOUD_LOG_LEVEL);

#define CUSTOM_JSON_APPID_VAL_IMPACT "IMPACT"
#define CUSTOM_JSON_APPID_VAL_ACTIVITY "ACTIVITY"

int cloud_impact_send(const struct impact_msg *impact, int64_t timestamp_ms, bool confirmable)
{
	int err;
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