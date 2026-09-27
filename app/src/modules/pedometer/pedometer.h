/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _PEDOMETER_H_
#define _PEDOMETER_H_

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Channels provided by this module */
ZBUS_CHAN_DECLARE(
	pedometer_chan
);

enum pedometer_msg_type {
	/* Output message types */

	/* Response to a request for the current step count.
	 * Carries the step delta accumulated since the last report and the
	 * lifetime total since boot.
	 */
	PEDOMETER_SAMPLE_RESPONSE = 0x1,

	/* Input message types */

	/* Request the module to report accumulated steps.
	 * The response is sent as a PEDOMETER_SAMPLE_RESPONSE message, and the
	 * delta counter is reset to 0 immediately after reporting.
	 */
	PEDOMETER_SAMPLE_REQUEST,
};

struct pedometer_msg {
	enum pedometer_msg_type type;

	/** Steps counted since the previous PEDOMETER_SAMPLE_RESPONSE.
	 *  Only valid for PEDOMETER_SAMPLE_RESPONSE events.
	 */
	uint32_t steps_since_last_report;

	/** Lifetime step count since boot (does not reset on report).
	 *  Only valid for PEDOMETER_SAMPLE_RESPONSE events.
	 */
	uint64_t total_steps;

	/** Timestamp when the report was generated in milliseconds.
	 *  This is either:
	 * - Unix time in milliseconds if the system clock was synchronized at report time, or
	 * - Uptime in milliseconds if the system clock was not synchronized at report time.
	 * Only valid for PEDOMETER_SAMPLE_RESPONSE events.
	 */
	int64_t timestamp;
};

#ifdef __cplusplus
}
#endif

#endif /* _PEDOMETER_H_ */
