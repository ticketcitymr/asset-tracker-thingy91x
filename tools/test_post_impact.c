/* Host test for the post-impact watcher.  Build and run on a PC:
 *   gcc -O1 -Wall -I app/src/modules/impact tools/test_post_impact.c app/src/modules/impact/post_impact.c -lm -o t && ./t
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "post_impact.h"

#define DT 20                       /* 50 Hz, same as CONFIG_APP_IMPACT_SAMPLE_INTERVAL_MS */
static int fails;

/* orientation helpers: unit gravity vector for a tilt of `deg` degrees away from +Z about the X axis */
static void orient(double deg, double *x, double *y, double *z)
{
	double r = deg * M_PI / 180.0;
	*x = 0; *y = sin(r); *z = cos(r);
}

static double noise(double amp) { return amp * ((rand() / (double)RAND_MAX) * 2.0 - 1.0); }

/* run a scenario: `before_s` upright, trigger, then `after_s` in orientation `after_deg`,
 * with `shake` g of 2 Hz movement for the first `shake_s` seconds of the after-period (after settling) */
static int run(const char *name, double after_deg, double shake, double shake_s, int twitch_windows, int expect)
{
	struct pi_params p = { .settle_ms = 3000, .watch_ms = 45000, .still_mg = 80, .max_moving_win = 2, .tilt_min_deg = 40 };
	struct post_impact s; struct pi_result r; int got = 0;
	int64_t t = 1000;
	double x, y, z;

	pi_init(&s, &p);
	for (int i = 0; i < 300; i++, t += DT) {          /* 6 s upright */
		orient(0, &x, &y, &z);
		pi_sample(&s, x + noise(0.01), y + noise(0.01), z + noise(0.01), t);
	}
	pi_trigger(&s, 6.5, false, 12345, t);
	int64_t t0 = t;
	for (int i = 0; i < 4000; i++, t += DT) {          /* 80 s after */
		double sec = (t - t0) / 1000.0;
		orient(after_deg, &x, &y, &z);
		double mov = 0.0;
		if (sec < 3.0) mov = 0.8 * sin(sec * 20);                 /* the impact and bounce, inside settle time */
		else if (sec - 3.0 < shake_s) mov = shake * sin(sec * 12.5);   /* walking / shaking */
		else if (twitch_windows > 0 && sec > 10 && sec < 10 + twitch_windows) mov = 0.3 * sin(sec * 12.5);
		pi_sample(&s, x + noise(0.01), y + noise(0.01), z + mov + noise(0.01), t);
		if (pi_poll(&s, &r)) {
			got = 1;
			printf("   %-44s -> REPORT tilt %.0f deg, still %us, peak %.1f g, at +%.0f s\n", name, r.tilt_deg, r.still_s, r.peak_g, sec);
		}
	}
	if (!got) printf("   %-44s -> no report\n", name);
	if (got != expect) { printf("   ^^^ FAIL (expected %s)\n", expect ? "report" : "no report"); fails++; }
	return got;
}

int main(void)
{
	srand(1);
	printf("post-impact watcher scenarios (tilt_min 40 deg, watch 45 s, settle 3 s):\n");
	run("fall, lies flat and still (tilt 90)",       90, 0,   0, 0, 1);
	run("fall, ends on side, still (tilt 60)",       60, 0,   0, 0, 1);
	run("fall, one twitch window then still",        90, 0,   0, 1, 1);
	run("fall, two twitch windows then still",       90, 0,   0, 2, 1);
	run("fall, three twitch windows (gets up?)",     90, 0,   0, 3, 0);
	run("door slam then walks away (tilt 0)",         0, 0.4, 40, 0, 0);
	run("fall then gets up and walks (tilt 90)",     90, 0.4, 20, 0, 0);
	run("door slam, sits still, same orientation",    0, 0,   0, 0, 0);
	run("sits down hard in car seat (tilt 30)",      30, 0,   0, 0, 0);
	run("bag dropped, still, tilt 25",               25, 0,   0, 0, 0);
	printf("%s\n", fails ? "FAILURES" : "all scenarios as expected");
	return fails ? 1 : 0;
}
