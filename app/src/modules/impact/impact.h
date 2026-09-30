/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _IMPACT_H_
#define _IMPACT_H_

#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

enum impact_msg_type {
	/* Published the moment a peak-acceleration event is detected and its
	 * capture window has closed. This is the only message type ever
	 * published on impact_chan - detection is self-triggered, there is no
	 * external "sample request" the way orientation/pedometer have.
	 */
	IMPACT_SAMPLE_RESPONSE = 0x1,

	/* Published periodically (see
	 * CONFIG_APP_IMPACT_ACTIVITY_REPORT_INTERVAL_SECONDS) with the peak
	 * g-force seen since the last report, regardless of whether it
	 * crossed the hard impact threshold. Lets an owner see how hard a
	 * dog has been playing/exerting even when nothing rose to the level
	 * of a reportable "impact".
	 */
	IMPACT_ACTIVITY_SUMMARY = 0x2,

	/* Published when free fall (near-zero acceleration for a short time,
	 * see CONFIG_APP_IMPACT_FREE_FALL) has ended and the landing capture
	 * window has closed. fall_ms is the free-fall duration and peak_g is
	 * the highest acceleration seen while landing.
	 */
	IMPACT_FREE_FALL = 0x3,
};

struct impact_msg {
	enum impact_msg_type type;

	/* Free-fall duration in milliseconds. Only set for IMPACT_FREE_FALL.
	 * Placed here so it fills the padding after the enum and the struct
	 * does not grow (the storage ring buffers are sized by struct size).
	 */
	uint32_t fall_ms;

	/* Peak acceleration magnitude observed during the triggering event, in g. */
	double peak_g;

	/* Timestamp (uptime ms, converted to unix ms downstream) of the peak sample. */
	int64_t timestamp;
};

/* Published whenever the impact module's motion detector transitions
 * between "moving" and "still". Deliberately a separate channel from
 * impact_chan (rather than reusing impact_msg with a new type) so this
 * never touches the storage/cloud pipeline built around impact_chan -
 * motion state is purely local, used to adjust the main sampling interval.
 */
struct motion_msg {
        bool is_moving;
};

ZBUS_CHAN_DECLARE(motion_chan);

ZBUS_CHAN_DECLARE(impact_chan);

#ifdef __cplusplus
}
#endif

#endif /* _IMPACT_H_ */