/*
 * Post-impact watcher ("man down" check). See post_impact.h.
 */

#include <math.h>
#include <string.h>

#include "post_impact.h"

/* time constant of the orientation average, and how often the delayed snapshot is taken */
#define EMA_TAU_MS 2000.0
#define SNAP_EVERY_MS 500
#define WINDOW_MS 1000
#define RAD2DEG 57.29577951308232

static double vlen(const struct pi_vec *v)
{
	return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

void pi_init(struct post_impact *s, const struct pi_params *p)
{
	memset(s, 0, sizeof(*s));
	s->p = *p;
}

void pi_trigger(struct post_impact *s, double peak_g, bool from_fall, int64_t ts, int64_t now_ms)
{
	if (s->watching) {
		/* keep the largest peak seen while a watch is already running */
		if (peak_g > s->res.peak_g) {
			s->res.peak_g = peak_g;
		}
		s->res.from_fall = s->res.from_fall || from_fall;
		return;
	}
	if (!s->ema_init) {
		return;   /* no orientation reference yet */
	}

	s->watching = true;
	s->done = false;
	s->trigger_ms = now_ms;
	s->res.peak_g = peak_g;
	s->res.from_fall = from_fall;
	s->res.ts = ts;
	s->res.tilt_deg = 0.0;
	s->res.still_s = 0;
	s->moving_win = 0;
	s->still_win = 0;
	s->win_start_ms = 0;
	/* the snapshot taken 0.5-1 s before the trigger is untouched by the impact itself */
	s->baseline = s->snap_old;
}

void pi_sample(struct post_impact *s, double gx, double gy, double gz, int64_t now_ms)
{
	struct pi_vec v = { gx, gy, gz };
	double dt = (double)(now_ms - s->last_ms);

	if (!s->ema_init) {
		s->ema = v;
		s->snap_new = v;
		s->snap_old = v;
		s->ema_init = true;
		s->last_ms = now_ms;
		s->last_snap_ms = now_ms;
		return;
	}
	s->last_ms = now_ms;

	/* the average keeps running during a watch too: at the end it is the "after" orientation */
	if (dt < 1.0) {
		dt = 1.0;
	}
	double a = dt / (EMA_TAU_MS + dt);

	s->ema.x += a * (gx - s->ema.x);
	s->ema.y += a * (gy - s->ema.y);
	s->ema.z += a * (gz - s->ema.z);

	if (!s->watching) {
		if (now_ms - s->last_snap_ms >= SNAP_EVERY_MS) {
			s->snap_old = s->snap_new;
			s->snap_new = s->ema;
			s->last_snap_ms = now_ms;
		}
		return;
	}

	int64_t since = now_ms - s->trigger_ms;

	if (since < (int64_t)s->p.settle_ms) {
		return;
	}

	/* 1-second windows: how much each axis varies. Looking at each axis (not only the magnitude)
	 * catches movement sideways to gravity, which barely changes the magnitude. */
	if (s->win_start_ms == 0) {
		s->win_start_ms = now_ms;
		s->win_min = s->win_max = v;
	} else {
		if (gx < s->win_min.x) { s->win_min.x = gx; }
		if (gx > s->win_max.x) { s->win_max.x = gx; }
		if (gy < s->win_min.y) { s->win_min.y = gy; }
		if (gy > s->win_max.y) { s->win_max.y = gy; }
		if (gz < s->win_min.z) { s->win_min.z = gz; }
		if (gz > s->win_max.z) { s->win_max.z = gz; }
	}

	if (now_ms - s->win_start_ms >= WINDOW_MS) {
		double rx = s->win_max.x - s->win_min.x;
		double ry = s->win_max.y - s->win_min.y;
		double rz = s->win_max.z - s->win_min.z;
		double range = rx > ry ? rx : ry;

		if (rz > range) {
			range = rz;
		}
		bool still = range * 1000.0 < (double)s->p.still_mg;

		if (still) {
			s->still_win++;
		} else {
			s->moving_win++;
		}
		s->win_start_ms = now_ms;
		s->win_min = s->win_max = v;

		if (s->moving_win > s->p.max_moving_win) {
			s->watching = false;   /* they got up and walked off, or it was a bump: cancel */
			return;
		}
	}

	if (since >= (int64_t)(s->p.settle_ms + s->p.watch_ms)) {
		double lb = vlen(&s->baseline);
		double la = vlen(&s->ema);
		double c = 0.0;

		if (lb > 0.2 && la > 0.2) {
			c = (s->baseline.x * s->ema.x + s->baseline.y * s->ema.y + s->baseline.z * s->ema.z) / (lb * la);
			if (c > 1.0) {
				c = 1.0;
			}
			if (c < -1.0) {
				c = -1.0;
			}
			s->res.tilt_deg = acos(c) * RAD2DEG;
		}
		s->res.still_s = s->still_win;
		s->watching = false;
		s->done = (s->res.tilt_deg >= (double)s->p.tilt_min_deg);
	}
}

bool pi_poll(struct post_impact *s, struct pi_result *out)
{
	if (!s->done) {
		return false;
	}
	s->done = false;
	*out = s->res;
	return true;
}
