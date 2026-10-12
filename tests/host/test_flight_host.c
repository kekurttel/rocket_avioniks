/* Host tests for the flight lifecycle (flight.c, HAL-independent).
 * Complements tests/host/test_avionics_host.c (untouched): covers the
 * spec CASE 1..11 with production + test configurations.
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "flight.h"

static int n_pass = 0;
#define CHECK(c) do { assert(c); n_pass++; } while (0)

static int drogue_events_count(const flight_t *f) { return f->drogue_deployed ? 1 : 0; }

/* CASE 1: startup -> calibration -> wait for launch */
static void t_case1_startup(void) {
  flight_cfg_t c; flight_config_production(&c);
  flight_t f; flight_init(&f, &c);
  CHECK(f.state == FLIGHT_INIT);
  flight_set_state(&f, FLIGHT_CALIBRATION);
  CHECK(f.state == FLIGHT_CALIBRATION);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  CHECK(f.state == FLIGHT_WAIT_FOR_LAUNCH);
  printf("  case1: INIT->CAL->WAIT ok\n");
}

/* CASE 2: accel > 1g (debounced N=3) -> launch; single spike -> no launch */
static void t_case2_launch(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  float gref = 16384.0f;
  /* single spike then rest: never confirms */
  CHECK(!flight_launch_push(&f, gref * 1.5f, gref));
  CHECK(!flight_launch_push(&f, gref * 0.9f, gref));
  CHECK(!flight_launch_push(&f, gref * 1.5f, gref));
  /* three consecutive over-threshold: confirms */
  flight_init(&f, &c); flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  CHECK(!flight_launch_push(&f, gref * 1.5f, gref));
  CHECK(!flight_launch_push(&f, gref * 1.6f, gref));
  CHECK(flight_launch_push(&f, gref * 1.5f, gref));
  /* step into ASCENT */
  flight_state_t s = flight_step(&f, true, false, 0.0f, true, 0.05f, true);
  CHECK(s == FLIGHT_ASCENT);
  /* invalid input never fires, resets streak */
  flight_init(&f, &c); flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  CHECK(!flight_launch_push(&f, NAN, gref));
  CHECK(!flight_launch_push(&f, gref * 1.5f, 0.0f));
  printf("  case2: launch debounce ok\n");
}

/* CASE 3: altitude rises -> max tracked */
static void t_case3_maxtrack(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  flight_alt_push(&f, 1.0f, true, 0.05f);
  flight_alt_push(&f, 3.0f, true, 0.05f);
  flight_alt_push(&f, 2.0f, true, 0.05f);
  CHECK(f.has_max && fabsf(f.max_alt_m - 3.0f) < 1e-6f);
  printf("  case3: max=%.1f ok\n", (double)f.max_alt_m);
}

/* CASE 4: fall > 2 m -> drop flag */
static void t_case4_drop(void) {
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  flight_alt_push(&f, 10.0f, true, 0.05f);
  CHECK(!f.drop_flag);
  flight_alt_push(&f, 7.9f, true, 0.05f);
  CHECK(f.drop_flag);
  printf("  case4: drop flag ok\n");
}

/* CASE 5: velocity < -2 m/s -> velocity flag; dt guards */
static void t_case5_vel(void) {
  float v = 0.0f;
  CHECK(flight_velocity(10.0f, 10.2f, 0.05f, &v) && fabsf(v + 4.0f) < 1e-4f);
  CHECK(!flight_velocity(1.0f, 0.0f, 0.0f, &v));   /* dt=0 banned */
  CHECK(!flight_velocity(1.0f, 0.0f, NAN, &v));
  CHECK(!flight_velocity(NAN, 0.0f, 0.05f, &v));
  CHECK(!flight_velocity(1.0f, 0.0f, 0.05f, NULL));
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  flight_alt_push(&f, 10.0f, true, 0.05f);
  flight_alt_push(&f, 9.8f, true, 0.05f); /* -4 m/s */
  CHECK(f.vel_valid && f.vel_flag);
  printf("  case5: vel=%.1f flag=%d ok\n", (double)f.vel_mps, (int)f.vel_flag);
}

/* CASE 6: both conditions (+alt-min) -> apogee, latched, never back */
static void t_case6_apogee(void) {
  /* TEST: no 1250 m gate */
  {
    flight_cfg_t c; flight_config_test(&c, 50.0f);
    flight_t f; flight_init(&f, &c);
    flight_set_state(&f, FLIGHT_ASCENT);
    float seq[] = {0,2,4,6,8,10, 9.8f,9.5f,9.0f,8.0f,7.0f,6.0f,5.0f};
    flight_state_t s = FLIGHT_ASCENT;
    for (int i = 0; i < 13; i++)
      s = flight_step(&f, false, false, seq[i], true, 0.05f, true);
    CHECK(s == FLIGHT_APOGEE_CONFIRMED);
    /* latched: climbing again never returns to ASCENT */
    s = flight_step(&f, false, false, 20.0f, true, 0.05f, true);
    CHECK(s == FLIGHT_APOGEE_CONFIRMED);
  }
  /* PROD: blocks below 1250 m, fires above */
  {
    flight_cfg_t c; flight_config_production(&c);
    flight_t f; flight_init(&f, &c);
    flight_set_state(&f, FLIGHT_ASCENT);
    float seq[] = {0,2,4,6,8,10, 9.8f,9.5f,9.0f,8.0f,7.0f,6.0f,5.0f};
    flight_state_t s = FLIGHT_ASCENT;
    for (int i = 0; i < 13; i++)
      s = flight_step(&f, false, false, seq[i], true, 0.05f, true);
    CHECK(s == FLIGHT_ASCENT); /* blocked by 1250 m gate */
    CHECK(fabsf(c.apogee_alt_min_m - 1250.0f) < 1e-6f);
    flight_t f2; flight_init(&f2, &c);
    flight_set_state(&f2, FLIGHT_ASCENT);
    float big[] = {0,400,800,1250,1300, 1299.8f,1299.5f,1290.0f,1280.0f,1270.0f,1260.0f,1250.0f,1240.0f};
    s = FLIGHT_ASCENT;
    for (int i = 0; i < 13; i++)
      s = flight_step(&f2, false, false, big[i], true, 0.05f, true);
    CHECK(s == FLIGHT_APOGEE_CONFIRMED);
  }
  printf("  case6: apogee TEST fires / PROD gates 1250m ok\n");
}

/* CASE 7: accel < 1g after apogee -> free fall (debounced) */
static void t_case7_freefall(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_APOGEE_CONFIRMED);
  float gref = 16384.0f;
  CHECK(!flight_freefall_push(&f, gref * 0.2f, gref));
  CHECK(!flight_freefall_push(&f, gref * 0.9f, gref)); /* reset */
  CHECK(!flight_freefall_push(&f, gref * 0.2f, gref));
  CHECK(!flight_freefall_push(&f, gref * 0.1f, gref));
  CHECK(flight_freefall_push(&f, gref * 0.2f, gref));
  flight_state_t s = flight_step(&f, false, true, 5.0f, true, 0.05f, true);
  CHECK(s == FLIGHT_FREE_FALL);
  printf("  case7: free-fall debounce ok\n");
}

/* CASE 8: free-fall confirmed -> drogue exactly once */
static void t_case8_drogue(void) {
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_FREE_FALL);
  flight_state_t s = flight_step(&f, false, false, 8.0f, true, 0.05f, true);
  CHECK(s == FLIGHT_DROGUE_DEPLOYED && f.drogue_deployed && f.drogue_event);
  CHECK(drogue_events_count(&f) == 1);
  /* next steps never re-fire */
  s = flight_step(&f, false, false, 7.0f, true, 0.05f, true);
  CHECK(s == FLIGHT_DESCENT);
  CHECK(f.drogue_deployed && !f.drogue_event);
  /* direct double-call guard */
  CHECK(!flight_try_drogue(&f, true));
  /* wrong-state + no-sensor guards */
  flight_t g; flight_init(&g, &c);
  flight_set_state(&g, FLIGHT_ASCENT);
  CHECK(!flight_try_drogue(&g, true));
  flight_set_state(&g, FLIGHT_FREE_FALL);
  CHECK(!flight_try_drogue(&g, false));
  printf("  case8: drogue one-shot ok\n");
}

/* CASE 9: altitude below test main threshold -> main exactly once */
static void t_case9_main(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  CHECK(fabsf(c.main_alt_m - 5.0f) < 1e-6f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_FREE_FALL);
  flight_step(&f, false, false, 8.0f, true, 0.05f, true); /* drogue */
  CHECK(f.state == FLIGHT_DROGUE_DEPLOYED);
  flight_step(&f, false, false, 7.0f, true, 0.05f, true); /* -> DESCENT */
  CHECK(f.state == FLIGHT_DESCENT);
  flight_state_t s = flight_step(&f, false, false, 4.9f, true, 0.05f, true);
  CHECK(s == FLIGHT_MAIN_DEPLOYED && f.main_deployed && f.main_event);
  s = flight_step(&f, false, false, 4.0f, true, 0.05f, true);
  CHECK(s == FLIGHT_COMPLETE);
  CHECK(!flight_try_main(&f, true)); /* never re-fires */
  /* production keeps 600 m */
  flight_cfg_t p; flight_config_production(&p);
  CHECK(fabsf(p.main_alt_m - 600.0f) < 1e-6f);
  printf("  case9: main one-shot + COMPLETE ok\n");
}

/* CASE 10: noise/spikes -> no false deployment */
static void t_case10_nofalse(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  /* jitter around rest: no launch, no apogee, no deploy */
  float gref = 16384.0f;
  for (int i = 0; i < 40; i++) {
    float am = gref + ((i % 2) ? 200.0f : -200.0f);
    bool lc = flight_launch_push(&f, am, gref);
    flight_state_t s = flight_step(&f, lc, false, ((i % 2) ? 0.05f : -0.05f), true, 0.05f, true);
    CHECK(s == FLIGHT_WAIT_FOR_LAUNCH);
  }
  CHECK(!f.drogue_deployed && !f.main_deployed);
  printf("  case10: noise silent ok\n");
}

/* CASE 11: sensor invalid -> safe fault handling (no crash, no deploy) */
static void t_case11_fault(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  /* single invalid: holds, no fault, no deploy */
  flight_state_t s = flight_step(&f, false, false, 0.0f, false, 0.05f, false);
  CHECK(s == FLIGHT_ASCENT && !f.drogue_deployed && !f.main_deployed);
  /* sustained invalid: latches FAULT, sticky */
  for (int i = 0; i < 60; i++)
    s = flight_step(&f, false, false, 0.0f, false, 0.05f, false);
  CHECK(s == FLIGHT_FAULT);
  s = flight_step(&f, true, true, 10.0f, true, 0.05f, true);
  CHECK(s == FLIGHT_FAULT && !f.drogue_deployed);
  printf("  case11: fault latch ok\n");
}

/* combine helper: average / single / none */
static void t_combine(void) {
  float o = 0.0f;
  CHECK(flight_combine_pressure(true, true, 100000.0f, 100002.0f, &o) && fabsf(o - 100001.0f) < 1e-3f);
  CHECK(flight_combine_pressure(true, false, 100000.0f, 0.0f, &o) && fabsf(o - 100000.0f) < 1e-3f);
  CHECK(flight_combine_pressure(false, true, 0.0f, 99999.0f, &o) && fabsf(o - 99999.0f) < 1e-3f);
  CHECK(!flight_combine_pressure(false, false, 0.0f, 0.0f, &o));
  /* one NaN -> safe fallback to the survivor (never a crash) */
  CHECK(flight_combine_pressure(true, true, NAN, 100000.0f, &o) && fabsf(o - 100000.0f) < 1e-3f);
  CHECK(!flight_combine_pressure(true, false, 1.0f, 0.0f, NULL));
}

/* full lifecycle smoke: WAIT->ASCENT->APOGEE->FREE_FALL->DROGUE->DESCENT->MAIN->COMPLETE */
static void t_full_cycle(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  float gref = 16384.0f;
  for (int i = 0; i < 3; i++) flight_launch_push(&f, gref * 1.5f, gref);
  CHECK(flight_step(&f, true, false, 0.0f, true, 0.05f, true) == FLIGHT_ASCENT);
  float up[] = {1,3,5,7,9,11};
  for (int i = 0; i < 6; i++) flight_step(&f, false, false, up[i], true, 0.05f, true);
  float dn[] = {10.8f,10.4f,9.5f,8.5f,7.5f,6.5f};
  flight_state_t s = FLIGHT_ASCENT;
  for (int i = 0; i < 6; i++) s = flight_step(&f, false, false, dn[i], true, 0.05f, true);
  CHECK(s == FLIGHT_APOGEE_CONFIRMED);
  for (int i = 0; i < 3; i++) flight_freefall_push(&f, gref * 0.1f, gref);
  CHECK(flight_step(&f, false, true, 6.0f, true, 0.05f, true) == FLIGHT_FREE_FALL);
  CHECK(flight_step(&f, false, false, 5.8f, true, 0.05f, true) == FLIGHT_DROGUE_DEPLOYED);
  CHECK(flight_step(&f, false, false, 5.5f, true, 0.05f, true) == FLIGHT_DESCENT);
  CHECK(flight_step(&f, false, false, 4.9f, true, 0.05f, true) == FLIGHT_MAIN_DEPLOYED);
  CHECK(flight_step(&f, false, false, 4.5f, true, 0.05f, true) == FLIGHT_COMPLETE);
  printf("  full-cycle TEST ok (states=%s)\n", flight_state_str(f.state));
}

/* ---------- bench vertical-accel trigger (TEST-only, gravity-referenced) ----------
 * G = one gravity in LSB (matches the 16384-scale convention used above).
 * dyn_g ~0 at rest, -1 in free-fall. Threshold -0.30, N=5, mag gate 0.95. */
#define BG (16384.0f)

/* helper: feed one bench sample from a raw accel vector + gref */
static bool bpush(flight_t *f, float ax, float ay, float az) {
  float dyn = 0.0f, mr = 1.0f;
  bool ok = flight_vert_dyn_g(ax, ay, az, 0.0f, 0.0f, BG, &dyn, &mr);
  return flight_bench_push(f, dyn, mr, ok);
}

/* projection unit checks */
static void tB0_proj(void) {
  float dyn = 9.0f, mr = 9.0f;
  CHECK(flight_vert_dyn_g(0, 0, BG, 0, 0, BG, &dyn, &mr) &&
        fabsf(dyn) < 1e-5f && fabsf(mr - 1.0f) < 1e-5f);          /* rest */
  CHECK(flight_vert_dyn_g(0, 0, 0, 0, 0, BG, &dyn, &mr) &&
        fabsf(dyn + 1.0f) < 1e-5f && fabsf(mr) < 1e-5f);          /* free-fall */
  CHECK(flight_vert_dyn_g(0, 0, 1.5f * BG, 0, 0, BG, &dyn, &mr) &&
        fabsf(dyn - 0.5f) < 1e-4f);                              /* pushed up */
  CHECK(!flight_vert_dyn_g(1, 2, 3, 0, 0, 0, &dyn, &mr));         /* gref 0 */
  CHECK(!flight_vert_dyn_g(NAN, 0, BG, 0, 0, BG, &dyn, &mr));     /* NaN */
  CHECK(!flight_vert_dyn_g(0, 0, BG, 0, 0, BG, NULL, &mr));
  CHECK(!flight_vert_dyn_g(0, 0, BG, 0, 0, BG, &dyn, NULL));
  printf("  bench-proj ok\n");
}
/* B1: quiet stationary -> no trigger */
static void tB1_quiet(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 40; i++)
    CHECK(!bpush(&f, (i % 2) ? 30.0f : -30.0f, 0.0f, BG));
  CHECK(!f.bench_confirmed && f.bench_n == 0);
  printf("  B1 quiet silent ok\n");
}
/* B2: single negative spike -> no trigger */
static void tB2_spike(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  CHECK(!bpush(&f, 0.0f, 0.0f, 0.2f * BG)); /* dyn -0.8, one sample */
  CHECK(!bpush(&f, 0.0f, 0.0f, BG));
  CHECK(!f.bench_confirmed && f.bench_n == 0);
  printf("  B2 single-spike silent ok\n");
}
/* B3: pulse shorter than N=3 -> no trigger */
static void tB3_short(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 2; i++) CHECK(!bpush(&f, 0.0f, 0.0f, 0.5f * BG));
  CHECK(!f.bench_confirmed);
  CHECK(!bpush(&f, 0.0f, 0.0f, BG)); /* rest breaks streak */
  CHECK(f.bench_n == 0);
  printf("  B3 short-pulse silent ok\n");
}
/* B4: sustained negative -> trigger on the Nth sample (N=3, bench) */
static void tB4_sustain(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 2; i++)
    CHECK(!bpush(&f, 0.0f, 0.0f, 0.5f * BG));
  CHECK(!f.bench_confirmed);
  CHECK(bpush(&f, 0.0f, 0.0f, 0.5f * BG)); /* Nth: confirm */
  CHECK(f.bench_confirmed);
  printf("  B4 sustained fires@%d ok\n", (int)FLIGHT_BENCH_CONFIRM_N_TEST);
}
/* B5: upward motion then rapid downward -> trigger; up never counts */
static void tB5_updown(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 3; i++) CHECK(!bpush(&f, 0.0f, 0.0f, 1.5f * BG));
  CHECK(f.bench_n == 0); /* up-motion leaves no streak */
  CHECK(!bpush(&f, 0.0f, 0.0f, BG));
  for (int i = 0; i < 2; i++) CHECK(!bpush(&f, 0.0f, 0.0f, 0.4f * BG));
  CHECK(bpush(&f, 0.0f, 0.0f, 0.4f * BG));
  printf("  B5 up-then-down fires ok\n");
}
/* B6: noise + downward -> still triggers reliably.
 * Alternating noise breaks the streak every other sample, so it needs a run
 * long enough to contain N consecutive sub-threshold samples. */
static void tB6_noisydown(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  bool seen = false;
  for (int i = 0; i < 8; i++) {
    float z = 0.5f * BG + ((i % 2) ? 0.05f * BG : -0.05f * BG);
    if (bpush(&f, 0.0f, 0.0f, z)) seen = true;
  }
  CHECK(seen && f.bench_confirmed);
  printf("  B6 noisy-down fires ok\n");
}
/* B7: noise only -> no trigger */
static void tB7_noiseonly(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 40; i++) {
    float z = BG + ((i % 2) ? 0.03f * BG : -0.03f * BG);
    CHECK(!bpush(&f, 0.0f, 0.0f, z));
  }
  CHECK(!f.bench_confirmed);
  printf("  B7 noise-only silent ok\n");
}
/* B8: invalid sample resets streak, never confirms; recovery works */
static void tB8_invalid(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  CHECK(!bpush(&f, 0.0f, 0.0f, 0.5f * BG));
  CHECK(!bpush(&f, 0.0f, 0.0f, 0.5f * BG));
  CHECK(!flight_bench_push(&f, NAN, 0.5f, true));   /* NaN dyn */
  CHECK(!flight_bench_push(&f, -0.5f, 0.5f, false)); /* invalid flag */
  CHECK(f.bench_n == 0 && !f.bench_confirmed);
  for (int i = 0; i < 5; i++) (void)bpush(&f, 0.0f, 0.0f, 0.5f * BG);
  CHECK(f.bench_confirmed); /* recovers after, on the Nth clean sample */
  printf("  B8 invalid-safe ok\n");
}
/* B9: latch one-shot: rest after confirm stays confirmed */
static void tB9_latch(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 5; i++) (void)bpush(&f, 0.0f, 0.0f, 0.5f * BG);
  CHECK(f.bench_confirmed);
  for (int i = 0; i < 20; i++) CHECK(bpush(&f, 0.0f, 0.0f, BG));
  printf("  B9 latch ok\n");
}
/* B10: repeated bench hammering after/around flight -> never advances state,
 * never deploys (BENCH_FALL != DROGUE_DEPLOY). Real gates still work once. */
static void tB10_nodup(void) {
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  for (int i = 0; i < 10; i++) {
    (void)bpush(&f, 0.0f, 0.0f, 0.3f * BG); /* bench confirms... */
    (void)flight_bench_baro_push(&f, 8.0f, true, 0.1f);
    (void)flight_step(&f, false, false, 8.0f, true, 0.05f, true);
    CHECK(f.state == FLIGHT_ASCENT); /* ...but state never moves... */
    CHECK(!f.drogue_event && !f.drogue_deployed); /* ...and never deploys */
  }
  CHECK(flight_bench_fall(&f)); /* display did rise (accel leg) */
  /* real gates still deploy exactly once (bench had nothing to do with it) */
  float alt = 8.0f;
  int guard = 0;
  while (f.state == FLIGHT_ASCENT && guard++ < 40) {
    alt -= 0.4f;
    (void)flight_step(&f, false, false, alt, true, 0.05f, true);
  }
  CHECK(f.state == FLIGHT_APOGEE_CONFIRMED);
  CHECK(flight_step(&f, false, true, alt, true, 0.05f, true) == FLIGHT_FREE_FALL);
  CHECK(flight_step(&f, false, false, alt, true, 0.05f, true) == FLIGHT_DROGUE_DEPLOYED);
  CHECK(f.drogue_deployed && f.drogue_event);
  CHECK(flight_step(&f, false, false, alt, true, 0.05f, true) == FLIGHT_DESCENT);
  CHECK(!f.drogue_event); /* one-shot: no re-fire on later steps */
  printf("  B10 bench-never-deploys ok\n");
}
/* B11: tilt-only rotations (30/45/60/90 deg, |a| = G) -> NO trigger.
 * 90 deg gives dyn = -1.0 yet the magnitude gate blocks it: rotation,
 * not descent. This proves the angle path cannot fire the trigger. */
static void tB11_tiltonly(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  static const float degs[] = {30.0f, 45.0f, 60.0f, 90.0f};
  for (int d = 0; d < 4; d++) {
    flight_t f; flight_init(&f, &c);
    float th = degs[d] * 3.14159265f / 180.0f;
    float ay = -sinf(th) * BG, az = cosf(th) * BG;
    for (int i = 0; i < 10; i++) CHECK(!bpush(&f, 0.0f, ay, az));
    CHECK(!f.bench_confirmed);
  }
  printf("  B11 tilt-only silent ok\n");
}
/* B12: tilted board (60 deg, outside old 30-45 band) + real downward
 * accel -> STILL triggers. Angle never gates the decision. */
static void tB12_tilteddown(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  float th = 60.0f * 3.14159265f / 180.0f;
  float ay = -sinf(th) * 0.5f * BG, az = cosf(th) * 0.5f * BG;
  bool seen = false;
  for (int i = 0; i < 6; i++) { if (bpush(&f, 0.0f, ay, az)) seen = true; }
  CHECK(seen && f.bench_confirmed);
  printf("  B12 tilted-down fires ok\n");
}
/* B13: production config: bench is display-only but ACTIVE (hand testing
 * works on the production build too). Identical profile confirms the
 * bench trigger, yet state never advances on bench alone. */
static void tB13_proddisplay(void) {
  flight_cfg_t c; flight_config_production(&c);
  CHECK(c.bench_enable);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 10; i++) (void)bpush(&f, 0.0f, 0.0f, 0.2f * BG);
  CHECK(f.bench_confirmed); /* display leg fires ... */
  flight_set_state(&f, FLIGHT_ASCENT);
  CHECK(flight_step(&f, false, false, 8.0f, true, 0.05f, true) == FLIGHT_ASCENT);
  CHECK(!f.drogue_deployed && !f.main_deployed); /* ... but never deploys */
  printf("  B13 production-display ok\n");
}
/* B14: bench confirm while WAIT -> state never advances on bench alone */
static void tB14_waitignores(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  for (int i = 0; i < 6; i++) (void)bpush(&f, 0.0f, 0.0f, 0.4f * BG);
  CHECK(f.bench_confirmed);
  CHECK(flight_step(&f, false, false, 0.0f, true, 0.05f, true) == FLIGHT_WAIT_FOR_LAUNCH);
  printf("  B14 wait-ignores-bench ok\n");
}
/* B15: TEST bench display chain is DECOUPLED from state/deployment:
 * bench confirms (accel OR baro) -> bench_fall rises, BLUE source ON,
 * but state stays ASCENT and nothing deploys. Deployment still flows only
 * through the real gates (covered by t_full_cycle). */
static void tB15_benchchain(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  for (int i = 0; i < 3; i++) flight_launch_push(&f, BG * 1.5f, BG);
  CHECK(flight_step(&f, true, false, 0.0f, true, 0.05f, true) == FLIGHT_ASCENT);
  CHECK(!flight_bench_fall(&f));
  for (int i = 0; i < 5; i++) (void)bpush(&f, 0.0f, 0.0f, 0.4f * BG);
  CHECK(flight_bench_fall(&f)); /* accel leg */
  CHECK(flight_step(&f, false, false, 0.0f, true, 0.05f, true) == FLIGHT_ASCENT);
  CHECK(!f.drogue_deployed && !f.main_deployed);
  /* baro leg alone also raises the OR while state holds. Both conditions are
   * required: below own max AND descending, for N consecutive samples. */
  flight_t g; flight_init(&g, &c);
  flight_set_state(&g, FLIGHT_ASCENT);
  (void)flight_bench_baro_push(&g, 0.0f, true, 0.1f);
  (void)flight_bench_baro_push(&g, -0.10f, true, 0.1f);
  (void)flight_bench_baro_push(&g, -0.20f, true, 0.1f);
  /* Descending fast, but still above (max - drop): the AND requirement holds
   * the streak at zero. This is exactly the bench noise that used to fire. */
  CHECK(g.bench_baro_n == 0);
  CHECK(!flight_bench_fall(&g));
  (void)flight_bench_baro_push(&g, -0.40f, true, 0.1f); /* now below max-0.3 */
  CHECK(g.bench_baro_n == 1);
  CHECK(!flight_bench_fall(&g)); /* 1 < N=3: still silent */
  (void)flight_bench_baro_push(&g, -0.55f, true, 0.1f);
  (void)flight_bench_baro_push(&g, -0.70f, true, 0.1f);
  CHECK(flight_bench_fall(&g));
  CHECK(flight_step(&g, false, false, -0.45f, true, 0.05f, true) == FLIGHT_ASCENT);
  CHECK(!g.drogue_deployed);
  printf("  B15 bench-decoupled ok\n");
}

/* ---------- BLUE<-BENCH_FALL mapping tests (A-J) ----------
 * flight_bench_fall() is the exact BLUE LED source in main.c, so these
 * assertions ARE the BLUE behavior contract. Tilt stays TUI-only. */
static bool bbpush(flight_t *f, float alt, bool valid, float dt) {
  f->bench_motion_ok = true; /* harness: motion present unless a test says so */
  return flight_bench_baro_push(f, alt, valid, dt);
}
/* A) tilt only -> BLUE OFF (bench_fall false) */
static void tC1_tiltblue(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  float th = 40.0f * 3.14159265f / 180.0f;
  for (int i = 0; i < 10; i++) {
    (void)bpush(&f, 0.0f, -sinf(th) * BG, cosf(th) * BG);
    (void)bbpush(&f, 0.0f, true, 0.1f);
    CHECK(!flight_bench_fall(&f));
  }
  printf("  C1 tilt-only BLUE-OFF ok\n");
}
/* B) negative accel only -> BLUE ON */
static void tC2_accelblue(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  CHECK(!flight_bench_fall(&f));
  for (int i = 0; i < 5; i++) (void)bpush(&f, 0.0f, 0.0f, 0.4f * BG);
  CHECK(flight_bench_fall(&f)); /* BLUE source ON */
  printf("  C2 accel-only BLUE-ON ok\n");
}
/* C) barometric rapid drop only (accel at rest) -> BLUE ON */
static void tC3_baroblue(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  (void)bpush(&f, 0.0f, 0.0f, BG); /* accel quiet */
  (void)bbpush(&f, 0.0f, true, 0.1f);
  (void)bbpush(&f, -0.10f, true, 0.1f);
  CHECK(!flight_bench_fall(&f));
  (void)bbpush(&f, -0.20f, true, 0.1f);
  CHECK(!flight_bench_fall(&f)); /* above max-drop: silent */
  (void)bbpush(&f, -0.40f, true, 0.1f);
  CHECK(!flight_bench_fall(&f)); /* 1 below max-drop < N=3: silent */
  (void)bbpush(&f, -0.55f, true, 0.1f);
  (void)bbpush(&f, -0.70f, true, 0.1f);
  CHECK(flight_bench_fall(&f)); /* baro leg alone raises the OR */
  CHECK(f.bench_baro_vel_valid && f.bench_baro_vel_mps < -1.0f);
  printf("  C3 baro-only BLUE-ON ok\n");
}
/* D) tilt + no fall -> no fall event */
static void tC4_tiltnofall(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  float th = 90.0f * 3.14159265f / 180.0f;
  for (int i = 0; i < 12; i++) {
    (void)bpush(&f, 0.0f, -sinf(th) * BG, cosf(th) * BG);
    (void)bbpush(&f, 0.05f, true, 0.1f);
  }
  CHECK(!flight_bench_fall(&f) && !f.bench_confirmed &&
        !f.bench_baro_confirmed);
  printf("  C4 tilt-no-fall ok\n");
}
/* E) negative accel + arbitrary tilt (70 deg) -> fall event */
static void tC5_tiltedaccel(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  float th = 70.0f * 3.14159265f / 180.0f;
  float ay = -sinf(th) * 0.5f * BG, az = cosf(th) * 0.5f * BG;
  for (int i = 0; i < 5; i++) (void)bpush(&f, 0.0f, ay, az);
  CHECK(flight_bench_fall(&f));
  printf("  C5 tilted-accel fires ok\n");
}
/* B15b: baro channel needs BOTH below-max-drop AND descending velocity for
 * N consecutive samples. The bench capture that motivated this showed -1.2
 * m/s velocity spikes on a board sitting still, so velocity alone cannot be
 * the trigger. Each condition alone must stay silent; together they fire. */
static void tB15b_baro_and_gate(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  /* Leg 1 needs an impossible drop distance so ONLY velocity can be true:
   * altitude keeps descending at -2 m/s but can never satisfy the drop. */
  c.bench_baro_drop_m = 1000.0f;
  flight_t g; flight_init(&g, &c);
  flight_set_state(&g, FLIGHT_ASCENT);
  (void)flight_bench_baro_push(&g, 0.0f, true, 0.1f);
  for (int i = 1; i <= 20; i++)
    (void)flight_bench_baro_push(&g, -0.2f * (float)i, true, 0.1f);
  CHECK(g.bench_baro_vel_mps < g.cfg.bench_baro_vel_mps); /* velocity ok */
  CHECK(!g.bench_baro_drop_flag);                       /* but no drop */
  CHECK(g.bench_baro_n == 0 && !flight_bench_fall(&g));
  /* Leg 2: hold below max-drop but with zero velocity (flat, static). */
  c.bench_baro_drop_m = 0.5f;
  flight_t h; flight_init(&h, &c);
  flight_set_state(&h, FLIGHT_ASCENT);
  (void)flight_bench_baro_push(&h, 0.0f, true, 0.1f);
  (void)flight_bench_baro_push(&h, -2.0f, true, 0.1f); /* big drop */
  for (int i = 0; i < 20; i++) (void)flight_bench_baro_push(&h, -2.0f, true, 0.1f);
  CHECK(h.bench_baro_drop_flag);                          /* drop ok */
  CHECK(h.bench_baro_vel_mps > h.cfg.bench_baro_vel_mps); /* but no fall */
  CHECK(h.bench_baro_n == 0 && !flight_bench_fall(&h));
  printf("  B15b baro AND-gate ok\n");
}
/* B16: max holds across a pause (no washout) and the motion gate keeps it
 * safe. Lift to 1 m, set down to 0 m, sit still: the max MUST stay at 1 m
 * (a washout here erased the reference and muted normal push-downs), the
 * drop flag stays TRUE, yet nothing confirms while motionless. The same
 * descent with motion confirms at once. */
static void tB16_stalemax(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  f.bench_motion_ok = false; /* motionless throughout */
  (void)flight_bench_baro_push(&f, 0.0f, true, 0.1f);
  (void)flight_bench_baro_push(&f, 1.0f, true, 0.1f);  /* lift: max = 1.0 */
  (void)flight_bench_baro_push(&f, 0.0f, true, 0.1f);  /* set down */
  for (int i = 0; i < 60; i++)                         /* sit still 6 s */
    (void)flight_bench_baro_push(&f, 0.0f, true, 0.1f);
  CHECK(f.bench_baro_max_m > 0.99f);   /* reference survives the pause */
  CHECK(f.bench_baro_drop_flag);       /* still below peak-drop ... */
  CHECK(f.bench_baro_n == 0);          /* ... but streak frozen ... */
  CHECK(!flight_bench_fall(&f));       /* ... so no BLUE while still. */
  f.bench_motion_ok = true; /* same descent, board now handled */
  for (int i = 1; i <= 8; i++)
    (void)flight_bench_baro_push(&f, -0.2f * (float)i, true, 0.1f);
  CHECK(flight_bench_fall(&f));
  printf("  B16 max-holds-motion-gates ok\n");
}
/* B17: motion liveness gates the baro channel. A room pressure step is
 * barometrically identical to a hand-down (captured: -0.5 m at -4.6 m/s,
 * board motionless), so without motion the perfect descent profile must
 * stay silent; with motion it confirms. Tracking (max/vel/drop_flag)
 * continues in both cases -- only the streak is gated. */
static void tB17_motiongate(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  CHECK(f.bench_motion_ok); /* init fail-open for harnesses */
  f.bench_motion_ok = false; /* motionless board, gust profile */
  (void)flight_bench_baro_push(&f, 0.0f, true, 0.1f);
  for (int i = 1; i <= 8; i++)
    (void)flight_bench_baro_push(&f, -0.2f * (float)i, true, 0.1f);
  CHECK(f.bench_baro_drop_flag);      /* position crossed ... */
  CHECK(f.bench_baro_vel_valid &&
        f.bench_baro_vel_mps < -1.0f); /* ... velocity crossed ... */
  CHECK(f.bench_baro_n == 0);          /* ... but streak frozen ... */
  CHECK(!flight_bench_fall(&f));       /* ... so no BLUE. */
  f.bench_motion_ok = true; /* same descent, board now handled */
  for (int i = 9; i <= 12; i++)
    (void)flight_bench_baro_push(&f, -0.2f * (float)i, true, 0.1f);
  CHECK(flight_bench_fall(&f));
  printf("  B17 motion-gate ok\n");
}
/* B18: slow hand lowering (-0.15 m/s) with motion confirms. The velocity
 * floor sits at -0.1: anything slower is gust-speed and must stay silent
 * (see tB17), anything at hand-speed fires. */
static void tB18_gentle(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  f.bench_motion_ok = true;
  (void)flight_bench_baro_push(&f, 0.0f, true, 0.1f);
  for (int i = 1; i <= 30; i++) /* -0.015 m / 0.1 s = -0.15 m/s */
    (void)flight_bench_baro_push(&f, -0.015f * (float)i, true, 0.1f);
  CHECK(flight_bench_fall(&f));
  printf("  B18 gentle-down fires ok\n");
}
/* Motion helper: tremor rests silent, handling fires, invalid never. */
static void tB19_motion(void) {
  CHECK(!flight_motion_seen(0.0f, 1.0f, true));    /* perfect rest */
  CHECK(!flight_motion_seen(0.003f, 1.002f, true)); /* bench rest noise */
  CHECK(!flight_motion_seen(0.05f, 1.02f, true));   /* still hand tremor */
  CHECK(flight_motion_seen(-0.13f, 1.0f, true));    /* 30 deg tilt */
  CHECK(flight_motion_seen(0.0f, 0.85f, true));     /* 15% mag dip */
  CHECK(flight_motion_seen(-0.5f, 0.5f, true));     /* free-fall */
  CHECK(!flight_motion_seen(-0.5f, 0.5f, false));   /* invalid never */
  printf("  B19 motion helper ok\n");
}
/* F) noise only (accel +-0.03g, alt +-0.02m) -> no event */
static void tC6_noiseboth(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 40; i++) {
    float s = (i % 2) ? 1.0f : -1.0f;
    (void)bpush(&f, 0.0f, s * 0.03f * BG, BG);
    (void)bbpush(&f, s * 0.02f, true, 0.1f);
    CHECK(!flight_bench_fall(&f));
  }
  printf("  C6 dual-noise silent ok\n");
}
/* G) one-sample spikes (accel + baro) -> no event */
static void tC7_singlespike(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  CHECK(!bpush(&f, 0.0f, 0.0f, 0.0f)); /* deep accel spike, 1 sample */
  (void)bpush(&f, 0.0f, 0.0f, BG);
  (void)bbpush(&f, 0.0f, true, 0.1f);
  (void)bbpush(&f, -0.6f, true, 0.1f); /* deep baro spike, 1 sample */
  (void)bbpush(&f, 0.0f, true, 0.1f);
  CHECK(!flight_bench_fall(&f));
  printf("  C7 single-spikes silent ok\n");
}
/* H) repeated detection -> one latched bench event (stays, never dupes) */
static void tC8_latchfall(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  for (int i = 0; i < 5; i++) (void)bpush(&f, 0.0f, 0.0f, 0.4f * BG);
  CHECK(flight_bench_fall(&f));
  for (int i = 0; i < 30; i++) {
    (void)bpush(&f, 0.0f, 0.0f, (i % 3 == 0) ? 0.4f * BG : BG);
    (void)bbpush(&f, 0.0f, true, 0.1f);
    CHECK(flight_bench_fall(&f)); /* latched, stable */
  }
  printf("  C8 latch ok\n");
}
/* I) sensor invalid -> no false fall (both channels) */
static void tC9_invalidboth(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  CHECK(!flight_bench_push(&f, -0.9f, 0.1f, false));
  CHECK(!flight_bench_baro_push(&f, -5.0f, false, 0.1f));
  CHECK(!flight_bench_push(&f, NAN, NAN, true));
  CHECK(!flight_bench_baro_push(&f, NAN, true, 0.1f));
  CHECK(!flight_bench_fall(&f));
  CHECK(!flight_bench_fall(NULL));
  printf("  C9 invalid-safe ok\n");
}
/* J) production deployment logic unchanged: full chain via real gates,
 * bench fully inert throughout (no confirms, no fall, no influence). */
static void tC10_prodchain(void) {
  flight_cfg_t c; flight_config_production(&c);
  flight_t f; flight_init(&f, &c);
  CHECK(!flight_bench_fall(&f));
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  for (int i = 0; i < 3; i++) flight_launch_push(&f, BG * 1.5f, BG);
  CHECK(flight_step(&f, true, false, 0.0f, true, 0.05f, true) == FLIGHT_ASCENT);
  for (int i = 0; i < 6; i++) { /* bench hammering during ascent */
    (void)bpush(&f, 0.0f, 0.0f, 0.2f * BG);
    (void)bbpush(&f, -1.0f * (float)i, true, 0.1f);
    /* Display legs MAY confirm (bench is active in production too) ... */
    (void)flight_bench_fall(&f);
    /* ... but the lifecycle must not move a millimeter on bench input. */
    CHECK(flight_step(&f, false, false, 0.0f, true, 0.05f, true) ==
          FLIGHT_ASCENT);
    CHECK(!f.drogue_deployed && !f.main_deployed);
  }
  float alt = 0.0f;
  for (int i = 0; i < 10; i++) { /* real climb past 1250 */
    alt += 200.0f;
    (void)flight_step(&f, false, false, alt, true, 0.05f, true);
  }
  int guard = 0;
  while (f.state == FLIGHT_ASCENT && guard++ < 40) {
    alt -= 30.0f;
    (void)flight_step(&f, false, false, alt, true, 0.05f, true);
  }
  CHECK(f.state == FLIGHT_APOGEE_CONFIRMED);
  CHECK(flight_step(&f, false, true, alt, true, 0.05f, true) == FLIGHT_FREE_FALL);
  CHECK(flight_step(&f, false, false, alt, true, 0.05f, true) == FLIGHT_DROGUE_DEPLOYED);
  CHECK(f.drogue_deployed && f.drogue_event);
  while (f.state == FLIGHT_DROGUE_DEPLOYED || f.state == FLIGHT_DESCENT) {
    alt -= 50.0f;
    (void)flight_step(&f, false, false, alt, true, 0.05f, true);
    if (guard++ > 200) break;
  }
  CHECK(f.state == FLIGHT_MAIN_DEPLOYED || f.state == FLIGHT_COMPLETE);
  CHECK(f.main_deployed);
  CHECK(flight_step(&f, false, false, alt, true, 0.05f, true) == FLIGHT_COMPLETE);
  printf("  C10 prod-chain unchanged ok\n");
}

/* ---------- A9 fault injection: detect -> isolate/fallback, never deploy.
 * Health snapshot matrix + pipeline faults. Telemetry fields mirror the
 * SENSOR_HEALTH TUI line (present/valid/stale/fused/disagree). */
static void tH_snap(void) {
  flight_health_t h;
  h = flight_health_snapshot(true, true, false, true, true, false,
                             true, true, false, AVT_BOTH_OK);
  CHECK(h.mpu_valid && !h.mpu_stale && h.fused_src == FLIGHT_FUSED_AVG &&
        !h.disagree);
  h = flight_health_snapshot(true, true, true, true, false, true,
                             false, false, false, AVT_BOTH_FAILED);
  CHECK(h.mpu_stale && !h.bmp_valid && h.bmp_stale && !h.b2_present &&
        h.fused_src == FLIGHT_FUSED_NONE);
  h = flight_health_snapshot(true, true, false, true, true, false,
                             true, false, false, AVT_BMP1_ONLY);
  CHECK(h.fused_src == FLIGHT_FUSED_BMP && !h.disagree);
  h = flight_health_snapshot(true, true, false, true, true, false,
                             true, true, false, AVT_DISAGREE);
  CHECK(h.disagree && h.fused_src == FLIGHT_FUSED_AVG);
  printf("  H snapshot matrix ok\n");
}
static void tF9_disagree(void) {
  CHECK(avt_bmp_state(true, true, 100000.0f, 100049.0f) == AVT_BOTH_OK);
  CHECK(avt_bmp_state(true, true, 100000.0f, 100051.0f) == AVT_DISAGREE);
  printf("  F9 disagree boundary 50Pa ok\n");
}
/* frozen BMP: constant pressure in ASCENT -> no apogee, no deploy */
static void tF1_frozen(void) {
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  for (int i = 0; i < 30; i++)
    CHECK(flight_step(&f, false, false, 5.0f, true, 0.05f, true) ==
          FLIGHT_ASCENT);
  CHECK(!f.drogue_deployed && !f.main_deployed);
  printf("  F1 frozen ok\n");
}
/* single + repeated spikes through med3->EMA->alt->step: no apogee */
static void tF2_spikes(void) {
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  float prof[40];
  for (int i = 0; i < 40; i++) prof[i] = 5.0f;
  prof[10] = -3.0f;                       /* single deep spike */
  prof[20] = -3.0f; prof[23] = -3.0f; prof[26] = -3.0f; /* repeated */
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  avt_med3_t m; avt_med3_init(&m);
  float pf = 0.0f; bool finit = false;
  for (int i = 0; i < 40; i++) {
    /* synthetic pressure wiggles around P0=100000 for alt~5 m rest */
    float praw = 100000.0f - prof[i] * 12.0f;
    float pm = avt_med3_push(&m, praw);
    if (!finit) { pf = pm; finit = true; } else pf = avt_ema(pf, pm);
    float h = 0.0f; bool ok = avt_altitude_m(pf, 100000.0f, &h);
    CHECK(flight_step(&f, false, false, h, ok, 0.05f, true) == FLIGHT_ASCENT);
  }
  CHECK(!f.drogue_deployed);
  printf("  F2 spikes ok\n");
}
/* MPU frozen at 1 g in WAIT -> launch never confirms */
static void tF4_mpufrozen(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  for (int i = 0; i < 20; i++) {
    bool lc = flight_launch_push(&f, BG, BG);
    CHECK(flight_step(&f, lc, false, 0.0f, true, 0.05f, true) ==
          FLIGHT_WAIT_FOR_LAUNCH);
  }
  printf("  F4 mpu-frozen ok\n");
}
/* impossible magnitudes rejected by launch/ff gates */
static void tF5_impossible(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  CHECK(!flight_launch_push(&f, 0.0f, BG));
  CHECK(!flight_launch_push(&f, NAN, BG));
  CHECK(!flight_launch_push(&f, 10.0f * BG, BG)); /* 1 sample never fires */
  CHECK(!flight_launch_push(&f, BG, -1.0f));      /* bad gref */
  flight_set_state(&f, FLIGHT_APOGEE_CONFIRMED);
  CHECK(!flight_freefall_push(&f, 0.0f, 0.0f)); /* gref 0 */
  CHECK(!flight_freefall_push(&f, NAN, BG));
  CHECK(flight_step(&f, false, false, 5.0f, true, 0.05f, true) ==
        FLIGHT_APOGEE_CONFIRMED);
  printf("  F5 impossible-mag ok\n");
}
/* interleaved missing samples: apogee may still confirm on sustained
 * drop+vel across valid samples (single-sample gate is by design; spikes
 * are killed upstream by median3), but FREE_FALL/DROGUE/FAULT can never
 * come from invalid samples (ff streak + watchdog need valid/usable). */
static void tF6_missing(void) {
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  float alt = 10.0f;
  for (int i = 0; i < 30; i++) {
    alt -= 0.4f;
    bool ok = (i % 2 == 0); /* every other sample missing */
    flight_state_t s = flight_step(&f, false, false, alt, ok, 0.05f, true);
    CHECK(s == FLIGHT_ASCENT || s == FLIGHT_APOGEE_CONFIRMED);
  }
  CHECK(!f.drogue_deployed && f.state != FLIGHT_FAULT);
  printf("  F6 missing-samples ok\n");
}
/* BMP2 invalid fallback + frozen-BMP2 vs drifting BMP1 */
static void tF7_b2fallback(void) {
  float o = 0.0f;
  CHECK(flight_combine_pressure(true, false, 100000.0f, 0.0f, &o));
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  for (int i = 0; i < 10; i++) { /* BMP1 drifts down, BMP2 frozen high */
    float pb = 100000.0f - (float)i * 2.0f, pm = 100050.0f;
    CHECK(flight_combine_pressure(true, true, pb, pm, &o));
    CHECK(flight_step(&f, false, false, 8.0f, true, 0.05f, true) ==
          FLIGHT_ASCENT || f.state == FLIGHT_APOGEE_CONFIRMED);
  }
  CHECK(!f.drogue_deployed); /* disagreement alone never deploys */
  printf("  F7 ms-fallback ok\n");
}
/* Dual-BMP280 P0 fusion (§2): mean when both valid, survivor when one
 * valid, and the 108k regression: BMP1 ~= 100 kPa + BMP2 invalid MUST give
 * FUSED == BMP1 (an invalid second sensor must never enter P0 fusion). */
static float fused_p0_mirror(bool b1v, float p1, bool b2v, float p2,
                             bool *ok) {
  if (b1v && b2v) { if (ok) *ok = true; return (p1 + p2) * 0.5f; }
  if (b1v) { if (ok) *ok = true; return p1; }
  if (b2v) { if (ok) *ok = true; return p2; }
  if (ok) *ok = false;
  return 0.0f;
}
static void tP0_dual(void) {
  bool ok = false;
  CHECK(fabsf(fused_p0_mirror(true, 100000.0f, true, 100004.0f, &ok) -
              100002.0f) < 1e-3f && ok);
  CHECK(fabsf(fused_p0_mirror(true, 100600.0f, false, 0.0f, &ok) -
              100600.0f) < 1e-3f && ok); /* 108k regression shape */
  CHECK(fabsf(fused_p0_mirror(false, 0.0f, true, 99800.0f, &ok) -
              99800.0f) < 1e-3f && ok);
  (void)fused_p0_mirror(false, 0.0f, false, 0.0f, &ok);
  CHECK(!ok);
  /* same rule through the real pressure-combine path */
  float o = 0.0f;
  CHECK(flight_combine_pressure(true, false, 100600.0f, 0.0f, &o) &&
        fabsf(o - 100600.0f) < 1e-3f);
  printf("  P0 dual-fusion ok\n");
}
/* stale MPU in APOGEE (frozen 1 g): ff never confirms, no drogue */
static void tF11_stale(void) {
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_APOGEE_CONFIRMED);
  for (int i = 0; i < 20; i++) {
    bool ff = flight_freefall_push(&f, BG, BG);
    CHECK(flight_step(&f, false, ff, 5.0f, true, 0.05f, true) ==
          FLIGHT_APOGEE_CONFIRMED);
  }
  CHECK(!f.drogue_deployed);
  printf("  F11 stale-mpu ok\n");
}

/* one 2 m spike sample sets drop&&vel flags once; apogee must not latch */
static void t_spike_nolatch(void) {
  flight_cfg_t c; flight_config_test(&c, 50.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  for (int i = 0; i < 10; i++)
    flight_step(&f, true, false, 10.0f + (float)i, true, 0.05f, true);
  CHECK(flight_step(&f, true, false, 16.0f, true, 0.05f, true) == FLIGHT_ASCENT);
  CHECK(f.apogee_n == 1);
  CHECK(flight_step(&f, true, false, 20.0f, true, 0.05f, true) == FLIGHT_ASCENT);
  CHECK(f.apogee_n == 0);
  /* invalid sample clears stale flags */
  flight_alt_push(&f, 0.0f, false, 0.05f);
  CHECK(!f.drop_flag && !f.vel_flag);
  printf("  spike-nolatch ok\n");
}

/* ---------- fuzyon gecersizlik disiplini (400000000 m olayi) ----------
 * main.c zincir aynasi: combine -> EMA -> baro (gecici degisken, sonlu +
 * zarf dogrulamasi, alt_m'ye yazma yok). Bozuk basinc + NaN/inf
 * enjeksiyonunda alt_valid=false; sagkalan sensor fallback'u yasar. */
static void t_combine_garbage(void) {
  float o = 0.0f;
  /* Sifir-veri / zarf-disi cop tek basina elenir, sagkalana dusulur. */
  o = 0.0f;
  CHECK(flight_combine_pressure(true, true, 0.0f, 100000.0f, &o) && fabsf(o - 100000.0f) < 1e-3f);
  o = 0.0f;
  CHECK(flight_combine_pressure(true, true, 100000.0f, 0.0f, &o) && fabsf(o - 100000.0f) < 1e-3f);
  o = 0.0f;
  CHECK(flight_combine_pressure(true, true, NAN, 100000.0f, &o) && fabsf(o - 100000.0f) < 1e-3f);
  o = 0.0f;
  CHECK(flight_combine_pressure(true, true, INFINITY, 99999.0f, &o) && fabsf(o - 99999.0f) < 1e-3f);
  o = 0.0f;
  CHECK(flight_combine_pressure(true, true, 5000000.0f, 100002.0f, &o) && fabsf(o - 100002.0f) < 1e-3f);
  /* Ikisi de cop -> false, cikisa YAZMA YOK (nobetci korunur). */
  o = 4242.0f;
  CHECK(!flight_combine_pressure(true, true, 0.0f, 0.0f, &o) && o == 4242.0f);
  o = 4242.0f;
  CHECK(!flight_combine_pressure(true, true, NAN, INFINITY, &o) && o == 4242.0f);
  o = 4242.0f;
  CHECK(!flight_combine_pressure(false, false, 100000.0f, 100000.0f, &o) && o == 4242.0f);
  /* Gecerli cift ayni ortalamayi uretir (sayi degisikligi yok). */
  o = 0.0f;
  CHECK(flight_combine_pressure(true, true, 100000.0f, 100002.0f, &o) && fabsf(o - 100001.0f) < 1e-3f);
  printf("  combine-garbage: fallback + no-write ok\n");
}
/* Tam zincir aynasi (combine -> EMA -> baro, main.c sirayla): cop giriste
 * invalid + alt_m nobetcisi saglam; saglikli giriste ayni sayi. */
static void t_fusion_mirror(void) {
  const float p0 = 100000.0f;
  /* Saglikli: 99990 Pa -> ~+0.84 m, gecerli. */
  {
    float p_comb = 0.0f, alt_m = 0.0f;
    bool ok = flight_combine_pressure(true, false, 99990.0f, 0.0f, &p_comb);
    float alt_tmp = 0.0f;
    bool valid = false;
    if (ok && avt_press_ok(p0) && isfinite(p_comb) && avt_press_ok(p_comb) &&
        avt_altitude_m(p_comb, p0, &alt_tmp) && isfinite(alt_tmp) &&
        alt_tmp <= AVT_ALT_LIMIT_M && alt_tmp >= -AVT_ALT_LIMIT_M)
    {
      alt_m = alt_tmp;
      valid = true;
    }
    CHECK(valid && alt_m > 0.5f && alt_m < 1.2f);
  }
  /* Cop: sifir + NaN karisimi -> invalid, alt_m nobetcisi saglam. */
  {
    float p_comb = 0.0f, alt_m = 5150.25f;
    bool ok = flight_combine_pressure(true, true, 0.0f, NAN, &p_comb);
    float alt_tmp = 0.0f;
    bool valid = false;
    if (ok && avt_press_ok(p0) && isfinite(p_comb) && avt_press_ok(p_comb) &&
        avt_altitude_m(p_comb, p0, &alt_tmp) && isfinite(alt_tmp) &&
        alt_tmp <= AVT_ALT_LIMIT_M && alt_tmp >= -AVT_ALT_LIMIT_M)
    {
      alt_m = alt_tmp;
      valid = true;
    }
    CHECK(!valid && alt_m == 5150.25f);
  }
  /* Asiri oran (P0 cok kucuk, cop referans) -> invalid, yazma yok. */
  {
    float alt_m = 5150.25f, alt_tmp = 0.0f;
    bool valid = false;
    if (avt_altitude_m(100000.0f, 100.0f, &alt_tmp) && isfinite(alt_tmp) &&
        alt_tmp <= AVT_ALT_LIMIT_M && alt_tmp >= -AVT_ALT_LIMIT_M)
    {
      alt_m = alt_tmp;
      valid = true;
    }
    CHECK(!valid && alt_m == 5150.25f);
  }
  /* flight_alt_push NaN/inf'yi yutar: bayraklar duser, max kirlenmez. */
  {
    flight_cfg_t c; flight_config_test(&c, 50.0f);
    flight_t f; flight_init(&f, &c);
    flight_set_state(&f, FLIGHT_ASCENT);
    flight_alt_push(&f, 10.0f, true, 0.05f);
    flight_alt_push(&f, NAN, true, 0.05f);
    flight_alt_push(&f, INFINITY, true, 0.05f);
    CHECK(fabsf(f.max_alt_m - 10.0f) < 1e-6f && !f.drop_flag && !f.vel_flag);
  }
  printf("  fusion-mirror: garbage invalid + healthy same ok\n");
}

int main(void) {
  t_case1_startup();
  t_spike_nolatch();
  t_case2_launch();
  t_case3_maxtrack();
  t_case4_drop();
  t_case5_vel();
  t_case6_apogee();
  t_case7_freefall();
  t_case8_drogue();
  t_case9_main();
  t_case10_nofalse();
  t_case11_fault();
  t_combine();
  t_full_cycle();
  tB0_proj();
  tB1_quiet(); tB2_spike(); tB3_short(); tB4_sustain(); tB5_updown();
  tB6_noisydown(); tB7_noiseonly(); tB8_invalid(); tB9_latch(); tB10_nodup();
  tB11_tiltonly(); tB12_tilteddown(); tB13_proddisplay(); tB14_waitignores();
  tB15_benchchain(); tB15b_baro_and_gate(); tB16_stalemax(); tB17_motiongate(); tB18_gentle(); tB19_motion();
  tC1_tiltblue(); tC2_accelblue(); tC3_baroblue(); tC4_tiltnofall();
  tC5_tiltedaccel(); tC6_noiseboth(); tC7_singlespike(); tC8_latchfall();
  tC9_invalidboth(); tC10_prodchain();
  tH_snap(); tF9_disagree();
  tF1_frozen(); tF2_spikes(); tF4_mpufrozen(); tF5_impossible();
  tF6_missing(); tF7_b2fallback(); tF11_stale(); tP0_dual();
  t_combine_garbage(); t_fusion_mirror();
  /* production constants live in exactly one place */
  CHECK(fabsf(FLIGHT_APOGEE_ALT_MIN_PROD_M - 1250.0f) < 1e-6f);
  CHECK(fabsf(FLIGHT_MAIN_ALT_PROD_M - 600.0f) < 1e-6f);
  CHECK(fabsf(FLIGHT_APOGEE_DROP_M - 2.0f) < 1e-6f);
  CHECK(fabsf(FLIGHT_DESCENT_VEL_MPS + 2.0f) < 1e-6f);
  printf("HOST_FLIGHT_TESTS_PASSED n=%d\n", n_pass);
  return 0;
}
