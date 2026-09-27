/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <net/nrf_cloud_coap.h>

#include "cloud_pedometer.h"

LOG_MODULE_DECLARE(cloud, CONFIG_APP_CLOUD_LOG_LEVEL);

/* Custom appId. PEDOMETER isn't one of nRF Cloud's built-in appIds (unlike
 * TEMP/HUMID/AIR_PRESS/BATTERY), so it carries multiple fields and goes out
 * as raw JSON via nrf_cloud_coap_json_message_send(), exactly like
 * cloud_orientation_send() does for ORIENTATION.
 */
#define CUSTOM_JSON_APPID_VAL_PEDOMETER "PEDOMETER"

int cloud_pedometer_send(const struct pedometer_msg *ped, int64_t timestamp_ms, bool confirmable)
{
	int err;
	char payload[192];
	int len;

	if (ped == NULL) {
		return -EINVAL;
	}

	len = snprintf(payload, sizeof(payload),
		"{\"appId\":\"%s\",\"messageType\":\"DATA\",\"ts\":%lld,"
		"\"data\":{\"steps\":%u,\"total\":%llu}}",
		CUSTOM_JSON_APPID_VAL_PEDOMETER,
		timestamp_ms,
		ped->steps_since_last_report,
		ped->total_steps);

	if (len < 0 || len >= (int)sizeof(payload)) {
		LOG_ERR("Pedometer JSON payload truncated or encoding failed");
		return -ENOMEM;
	}

	err = nrf_cloud_coap_json_message_send(payload, confirmable, false);
	if (err) {
		LOG_ERR("nrf_cloud_coap_json_message_send, error: %d", err);
		return err;
	}

	LOG_DBG("Pedometer data sent to cloud: %u steps (total %llu)",
		ped->steps_since_last_report, ped->total_steps);

	return 0;
}
