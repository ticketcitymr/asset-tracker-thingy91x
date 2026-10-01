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

#include "cloud_orientation.h"

LOG_MODULE_DECLARE(cloud, CONFIG_APP_CLOUD_LOG_LEVEL);

/* ORIENTATION is a custom appId, not one of nRF Cloud's built-in sensor types
 * (see nrf_cloud_defs.h for the built-ins like TEMP/HUMID/FLIP). It carries
 * face, tilt and raw axis values together in one JSON payload, so it is sent
 * as a raw message rather than through nrf_cloud_coap_sensor_send(), which
 * only accepts a single double value.
 */
#define CUSTOM_JSON_APPID_VAL_ORIENTATION "ORIENTATION"

int cloud_orientation_send(const struct orientation_msg *ori,
			   int64_t timestamp_ms,
			   bool confirmable)
{
	int err;
	char json[320];
	char mag[64] = "";

	if (ori->mag_valid) {
		snprintk(mag, sizeof(mag), ",\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f",
			 (double)ori->mx, (double)ori->my, (double)ori->mz);
	}

	if (timestamp_ms > 0) {
		err = snprintk(json, sizeof(json),
			"{\"appId\":\"%s\",\"messageType\":\"DATA\",\"ts\":%lld,"
			"\"data\":{\"face\":\"%s\",\"tilt\":%.1f,"
			"\"x\":%.2f,\"y\":%.2f,\"z\":%.2f%s}}",
			CUSTOM_JSON_APPID_VAL_ORIENTATION, timestamp_ms,
			orientation_face_to_str(ori->face), ori->tilt_deg,
			ori->x, ori->y, ori->z, mag);
	} else {
		err = snprintk(json, sizeof(json),
			"{\"appId\":\"%s\",\"messageType\":\"DATA\","
			"\"data\":{\"face\":\"%s\",\"tilt\":%.1f,"
			"\"x\":%.2f,\"y\":%.2f,\"z\":%.2f%s}}",
			CUSTOM_JSON_APPID_VAL_ORIENTATION,
			orientation_face_to_str(ori->face), ori->tilt_deg,
			ori->x, ori->y, ori->z, mag);
	}

	if (err < 0 || err >= (int)sizeof(json)) {
		LOG_ERR("Orientation JSON too long");
		return -ENOMEM;
	}

	err = nrf_cloud_coap_json_message_send(json, false, confirmable);
	if (err) {
		LOG_ERR("Failed to send orientation data to cloud, error: %d", err);
		return err;
	}

	LOG_DBG("Orientation data sent to cloud: %s, tilt %.1f deg",
		orientation_face_to_str(ori->face), (double)ori->tilt_deg);

	return 0;
}
