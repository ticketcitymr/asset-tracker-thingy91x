/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _ORIENTATION_H_
#define _ORIENTATION_H_

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Channels provided by this module */
ZBUS_CHAN_DECLARE(
	orientation_chan
);

enum orientation_msg_type {
	/* Output message types */

	/* Response message to a request for the current orientation.
	 * The sampled values are found in the respective fields of the message structure.
	 */
	ORIENTATION_SAMPLE_RESPONSE = 0x1,

	/* Input message types */

	/* Request to sample the current orientation.
	 * The response is sent as an ORIENTATION_SAMPLE_RESPONSE message.
	 */
	ORIENTATION_SAMPLE_REQUEST,
};

/* Dominant-axis face the device is resting on, matching the convention used on the
 * original Thingy:91 asset_tracker_v2 orientation work.
 */
enum orientation_face {
	ORIENTATION_FACE_X_UP,
	ORIENTATION_FACE_X_DOWN,
	ORIENTATION_FACE_Y_UP,
	ORIENTATION_FACE_Y_DOWN,
	ORIENTATION_FACE_Z_UP,
	ORIENTATION_FACE_Z_DOWN,
};

struct orientation_msg {
	enum orientation_msg_type type;

	/** Dominant axis / face the device is resting on.
	 *  Only valid for ORIENTATION_SAMPLE_RESPONSE events.
	 */
	enum orientation_face face;

	/** Angle in degrees between the dominant axis and gravity.
	 *  Only valid for ORIENTATION_SAMPLE_RESPONSE events.
	 */
	double tilt_deg;

	/** Raw accelerometer reading, normalized to m/s^2 regardless of the
	 *  underlying chip's native units.
	 *  Only valid for ORIENTATION_SAMPLE_RESPONSE events.
	 */
	double x;
	double y;
	double z;

	/** Timestamp when the sample was taken in milliseconds.
	 *  This is either:
	 * - Unix time in milliseconds if the system clock was synchronized at sampling time, or
	 * - Uptime in milliseconds if the system clock was not synchronized at sampling time.
	 * Only valid for ORIENTATION_SAMPLE_RESPONSE events.
	 */
	int64_t timestamp;
};

/** @brief Convert a face enum value to its string name (e.g. "Z_UP"). */
const char *orientation_face_to_str(enum orientation_face face);

#ifdef __cplusplus
}
#endif

#endif /* _ORIENTATION_H_ */
