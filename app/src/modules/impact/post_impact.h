/*
 * Post-impact watcher ("man down" check).
 *
 * After a hard impact or a fall the watcher looks at the next few seconds of accelerometer data:
 *   - did the tracker stay still (a person lying where they fell), or keep moving (walking away
 *     after a slammed door or a dropped bag)?
 *   - did the way the tracker is oriented change (lying down) compared with just before the event
 *     (sitting in a car seat changes it much less)?
 * It only reports when the tracker stayed still AND its orientation changed by at least a minimum angle.
 *
 * Plain C, no Zephyr dependencies, so it can be unit-tested on a PC (see tools/test_post_impact.c).
 * Call pi_sample() for EVERY accelerometer sample, pi_trigger() when an impact or fall is reported,
 * and pi_poll() after each sample to collect a finished result.
 */

#ifndef POST_IMPACT_H_
#define POST_IMPACT_H_

#include <stdbool.h>
#include <stdint.h>

struct pi_params {
	uint32_t settle_ms;       /* ignore this long after the trigger (landing, bouncing) */
	uint32_t watch_ms;        /* then watch this long for stillness */
	uint32_t still_mg;        /* a 1 s window whose magnitude varies less than this is "still" */
	uint32_t max_moving_win;  /* more moving 1 s windows than this cancels the watch */
	uint32_t tilt_min_deg;    /* report only if the orientation changed at least this much */
};

struct pi_result {
	double peak_g;       /* peak g of the triggering impact or landing */
	double tilt_deg;     /* orientation change between before and after, 0..180 */
	uint32_t still_s;    /* seconds of stillness seen during the watch */
	bool from_fall;      /* the trigger was a free fall + landing, not just an impact */
	int64_t ts;          /* timestamp of the triggering event, as given to pi_trigger() */
};

struct pi_vec {
	double x, y, z;
};

struct post_impact {
	struct pi_params p;

	/* slow average of the acceleration vector (orientation) and a delayed snapshot of it */
	struct pi_vec ema, snap_new, snap_old;
	bool ema_init;
	int64_t last_ms, last_snap_ms;

	/* watch state */
	bool watching;
	int64_t trigger_ms;
	struct pi_result res;
	uint32_t moving_win, still_win;
	struct pi_vec win_min, win_max;
	int64_t win_start_ms;
	struct pi_vec baseline;
	bool done;
};

void pi_init(struct post_impact *s, const struct pi_params *p);

/* Feed one sample: acceleration vector in g, time in ms (monotonic). */
void pi_sample(struct post_impact *s, double gx, double gy, double gz, int64_t now_ms);

/* An impact or a fall was reported. Ignored while a watch is already running. */
void pi_trigger(struct post_impact *s, double peak_g, bool from_fall, int64_t ts, int64_t now_ms);

/* True once, when a watch finished with "still and tilted". Fills *out. */
bool pi_poll(struct post_impact *s, struct pi_result *out);

#endif /* POST_IMPACT_H_ */
