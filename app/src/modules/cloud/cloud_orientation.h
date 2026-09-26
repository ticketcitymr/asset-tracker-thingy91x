/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _CLOUD_ORIENTATION_H_
#define _CLOUD_ORIENTATION_H_

#include <stdint.h>
#include <stdbool.h>

#include "orientation.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send an orientation sample to nRF Cloud as a custom "ORIENTATION" message.
 *
 * This is a custom appId (not one of nRF Cloud's built-in sensor types), sent as a raw
 * JSON message so it can carry face/tilt/x/y/z together in one payload, matching the
 * ORIENTATION message shape used on the original Thingy:91 asset_tracker_v2 build.
 *
 * @param ori Orientation sample to send.
 * @param timestamp_ms Timestamp of the sample, in unix time ms or NRF_CLOUD_NO_TIMESTAMP.
 * @param confirmable Select whether to use a CON or NON CoAP transfer.
 *
 * @return 0 if successful, negative errno on failure.
 */
int cloud_orientation_send(const struct orientation_msg *ori,
			   int64_t timestamp_ms,
			   bool confirmable);

#ifdef __cplusplus
}
#endif

#endif /* _CLOUD_ORIENTATION_H_ */
