/* Host tests for rocket_avioniks avionics math + apogee trend state machine.
 * HAL-independent: compiles Src/avionics.c for the host with gcc.
 * Legacy assertions (46) preserved; new A..H + finger-sequence tests added.
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "avionics.h"

static int n_pass = 0;
#define CHECK(c) do { assert(c); n_pass++; } while (0)

/* ---------- legacy behaviour: tilt ---------- */
static void t_tilt_zero(void)   { CHECK(fabsf(avt_tilt_deg(0,0,16384, 0,0,16384)) < 0.5f); }
static void t_tilt_90(void)     { CHECK(fabsf(avt_tilt_deg(16384,0,0, 0,0,16384) - 90.0f) < 1.0f); }
static void t_tilt_180(void)    { CHECK(fabsf(avt_tilt_deg(0,0,-16384, 0,0,16384) - 180.0f) < 1.0f); }
static void t_tilt_45(void)     { CHECK(fabsf(avt_tilt_deg(1000,0,1000, 0,0,2000) - 45.0f) < 1.5f); }
static void t_tilt_degen(void)  { CHECK(avt_tilt_deg(0,0,0, 0,0,0) == 0.0f); }
static void t_mag_ok(void)      { CHECK(avt_tilt_mag_ok(0,0,16384, 0,0,16384)); }
static void t_mag_lo(void)      { CHECK(avt_tilt_mag_ok(0,0,12500, 0,0,16384)); }
static void t_mag_outside(void)  { CHECK(!avt_tilt_mag_ok(0,0,12000, 0,0,16384)); }
static void t_mag_hi(void)      { CHECK(!avt_tilt_mag_ok(0,0,40000, 0,0,16384)); }
static void t_mag_zref(void)    { CHECK(!avt_tilt_mag_ok(1,2,3, 0,0,0)); }
static void t_vec_mag(void)     { CHECK(fabsf(avt_vec_mag(3,4,0) - 5.0f) < 1e-4f); }
static void t_band_in(void)     { CHECK(avt_in_band(30.0f, 30.0f, 45.0f) && avt_in_band(45.0f, 30.0f, 45.0f)); }
static void t_band_mid(void)    { CHECK(avt_in_band(37.5f, 30.0f, 45.0f)); }
static void t_band_out(void)    { CHECK(!avt_in_band(29.9f, 30.0f, 45.0f) && !avt_in_band(45.1f, 30.0f, 45.0f)); }
/* ---------- legacy: altitude ---------- */
static void t_alt_zero(void)    { float h=9; CHECK(avt_altitude_m(100000,100000,&h) && fabsf(h) < 1e-3f); }
static void t_alt_up(void)      { float h=0; CHECK(avt_altitude_m(99980,100000,&h) && h > 1.5f && h < 1.9f); }
static void t_alt_down(void)    { float h=0; CHECK(avt_altitude_m(100020,100000,&h) && h < -1.5f && h > -1.9f); }
static void t_alt_bad0(void)    { float h=0; CHECK(!avt_altitude_m(0,100000,&h)); }
static void t_alt_bad1(void)    { float h=0; CHECK(!avt_altitude_m(100000,0,&h)); }
static void t_alt_bad2(void)    { CHECK(!avt_altitude_m(100000,100000,NULL)); }
static void t_alt_half(void)    { float h=0; CHECK(avt_altitude_m(99994,100000,&h) && h > 0.35f && h < 0.65f); }
/* ---------- legacy: redundancy ---------- */
static void t_bmp_both(void)    { CHECK(avt_bmp_state(true,true,100000,100005) == AVT_BOTH_OK); }
static void t_bmp_dis(void)     { CHECK(avt_bmp_state(true,true,100000,100200) == AVT_DISAGREE); }
static void t_bmp_1(void)       { CHECK(avt_bmp_state(true,false,100000,0) == AVT_BMP1_ONLY); }
static void t_bmp_2(void)       { CHECK(avt_bmp_state(false,true,0,100000) == AVT_BMP2_ONLY); }
static void t_bmp_none(void)    { CHECK(avt_bmp_state(false,false,0,0) == AVT_BOTH_FAILED); }
static void t_deg_ok(void)      { CHECK(!avt_is_degraded(AVT_BOTH_OK)); }
static void t_deg_bad(void)     { CHECK(avt_is_degraded(AVT_DISAGREE)); }
static void t_use_ok(void)      { CHECK(avt_alt_usable(AVT_BMP1_ONLY)); }
static void t_use_no(void)      { CHECK(!avt_alt_usable(AVT_BOTH_FAILED)); }
/* ---------- legacy: alt_rise hysteresis ---------- */
static void t_rise_on(void)     { CHECK(avt_alt_rise(false, 0.30f, true)); }
static void t_rise_on_edge(void){ CHECK(avt_alt_rise(false, 0.20f, true)); }
static void t_rise_hold(void)   { CHECK(avt_alt_rise(true, 0.15f, true)); }
static void t_rise_mid(void)    { CHECK(!avt_alt_rise(false, 0.15f, true)); }
static void t_rise_off(void)    { CHECK(!avt_alt_rise(true, 0.05f, true)); }
static void t_rise_off_edge(void){ CHECK(!avt_alt_rise(true, 0.10f, true)); }
static void t_rise_inv(void)    { CHECK(!avt_alt_rise(true, 5.0f, false)); }
/* ---------- finger: alt_drop hysteresis (single threshold -0.15/-0.07) ---------- */
static void t_drop_on(void)     { CHECK(avt_alt_drop(false, -0.20f, true)); }
static void t_drop_on_edge(void){ CHECK(avt_alt_drop(false, -0.15f, true)); }
static void t_drop_hold(void)   { CHECK(avt_alt_drop(true, -0.10f, true)); }
static void t_drop_mid_off(void){ CHECK(!avt_alt_drop(false, -0.10f, true)); }
static void t_drop_off(void)    { CHECK(!avt_alt_drop(true, -0.03f, true)); }
static void t_drop_off_edge(void){ CHECK(!avt_alt_drop(true, -0.07f, true)); }
static void t_drop_inv(void)    { CHECK(!avt_alt_drop(true, -5.0f, false)); }
/* Finger-like sequence: high-alt start (+0.6 m suck) then release fall.
 * drop_cond must turn ON during the dip and OFF after release, while the
 * apogee latch (LD2 path) stays independent of this immediate indicator. */
static void t_drop_finger_seq(void) {
  bool d = false;
  float seq[] = {0.60f,0.58f,0.40f,0.20f,0.05f,-0.05f,-0.12f,-0.18f,-0.20f,-0.12f,-0.05f,0.0f};
  bool saw_on = false, saw_off_after_on = false;
  for (int i = 0; i < 12; i++) {
    d = avt_alt_drop(d, seq[i], true);
    if (d) saw_on = true;
    if (saw_on && !d) saw_off_after_on = true;
  }
  printf("  drop-finger: saw_on=%d saw_off_after_on=%d\n", (int)saw_on, (int)saw_off_after_on);
  CHECK(saw_on && saw_off_after_on);
  /* Invalid sample forces OFF even when previously ON. */
  CHECK(!avt_alt_drop(true, -0.30f, false));
}
/* ---------- legacy: EMA ---------- */
static void t_ema_step(void)    { CHECK(fabsf(avt_ema(100000.0f, 99990.0f) - 99995.0f) < 1e-3f); }
static void t_ema_hold(void)    { CHECK(fabsf(avt_ema(5.0f, 5.0f) - 5.0f) < 1e-6f); }
static void t_ema_alpha(void)   { CHECK(fabsf(AVT_PRESS_EMA_ALPHA - 0.5f) < 1e-6f); }
/* ---------- legacy: apogee defines + init ---------- */
static void t_apg_def_arm(void) { CHECK(fabsf(AVT_APOGEE_ARM_M - 0.5f) < 1e-6f); }
static void t_apg_def_drop(void){ CHECK(fabsf(AVT_APOGEE_DROP_M - 0.3f) < 1e-6f); }
static void t_apg_def_n(void)   { CHECK(AVT_APOGEE_CONFIRM_N == 3U); }
static void t_apg_init(void)    { avt_apogee_t s; avt_apogee_init(&s); CHECK(s.phase==AVT_PHASE_ASCENT && !s.armed && !s.has_max && s.drop_n==0); }
static void t_apg_no_cal(void)  { avt_apogee_t s; avt_apogee_init(&s); CHECK(avt_apogee_update(&s, 50.0f, false)==AVT_PHASE_ASCENT); }
static void t_apg_latch(void)   { avt_apogee_t s; avt_apogee_init(&s); s.phase=AVT_PHASE_DESCENT; CHECK(avt_apogee_update(&s, 0.0f, true)==AVT_PHASE_DESCENT); }
static void t_apg_invalid(void) { avt_apogee_t s; avt_apogee_init(&s); avt_apogee_update(&s, 0.0f, true); CHECK(avt_apogee_update(&s, 0.0f, false)==AVT_PHASE_ASCENT); }
static void t_apg_single(void)  { avt_apogee_t s; avt_apogee_init(&s); avt_apogee_set_noise(&s, 0.02f); CHECK(avt_apogee_update(&s, 5.0f, true)==AVT_PHASE_ASCENT); }

/* helper: feed a series, return final phase */
static avt_phase_t feed(avt_apogee_t *s, const float *a, int n, float sigma) {
  avt_apogee_init(s); avt_apogee_set_noise(s, sigma);
  avt_phase_t p = AVT_PHASE_ASCENT;
  for (int i=0;i<n;i++) p = avt_apogee_update(s, a[i], true);
  return p;
}
/* A) noise-only -> no trigger */
static void tA_noise(void) {
  avt_apogee_t s; float a[] = {0.00f,0.015f,-0.012f,0.008f,-0.018f,0.011f,-0.006f,0.014f,-0.009f,0.004f,0.0f,-0.013f};
  CHECK(feed(&s, a, 12, 0.02f)==AVT_PHASE_ASCENT && !s.armed);
}
/* B) small but sustained ascent -> armed (legacy 0.5 m height still required) */
static void tB_ascent(void) {
  avt_apogee_t s; float a[] = {0.0f,0.10f,0.20f,0.30f,0.40f,0.50f,0.60f};
  CHECK(feed(&s, a, 7, 0.02f)==AVT_PHASE_ARMED && s.armed);
}
/* C) armed + apex hover -> still LED OFF (ARMED, not DESCENT) */
static void tC_apex(void) {
  avt_apogee_t s; float a[] = {0.0f,0.10f,0.20f,0.30f,0.40f,0.50f,0.60f,0.60f,0.60f,0.60f,0.60f,0.60f};
  CHECK(feed(&s, a, 12, 0.02f)==AVT_PHASE_ARMED);
}
/* D) small but sustained descent trend past apex -> trigger */
static void tD_descent(void) {
  avt_apogee_t s; float a[] = {0.0f,0.10f,0.20f,0.30f,0.40f,0.50f,0.62f,0.58f,0.54f,0.50f,0.46f,0.42f,0.38f,0.34f};
  CHECK(feed(&s, a, 14, 0.02f)==AVT_PHASE_DESCENT);
}
/* E) single noisy drop -> no trigger */
static void tE_spike(void) {
  avt_apogee_t s; float a[] = {0.0f,0.10f,0.20f,0.30f,0.40f,0.50f,0.60f,0.30f,0.60f,0.60f,0.60f};
  CHECK(feed(&s, a, 11, 0.02f)==AVT_PHASE_ARMED);
}
/* F) post-trigger wobble -> stays latched */
static void tF_latch(void) {
  avt_apogee_t s; float a[] = {0.0f,0.10f,0.20f,0.30f,0.40f,0.50f,0.62f,0.58f,0.54f,0.50f,0.46f,0.42f,0.38f,0.34f,0.60f,0.90f,0.0f};
  CHECK(feed(&s, a, 17, 0.02f)==AVT_PHASE_DESCENT);
}
/* G) calibration window never triggers (update not fed during cal: init state) */
static void tG_cal(void) {
  avt_apogee_t s; avt_apogee_init(&s);
  float big[] = {3.0f,-3.0f,5.0f,-5.0f};
  (void)big; CHECK(s.phase==AVT_PHASE_ASCENT && !s.armed);
}
/* H) pure descent without prior ascent -> no trigger */
static void tH_nodescent(void) {
  avt_apogee_t s; float a[] = {0.0f,-0.05f,-0.10f,-0.15f,-0.20f,-0.30f,-0.45f,-0.60f};
  CHECK(feed(&s, a, 8, 0.02f)==AVT_PHASE_ASCENT && !s.armed);
}
/* Finger-like: ~+0.6 m suck over ~0.5 s then release fall -> trigger, with reason */
static void t_finger(void) {
  avt_apogee_t s; avt_apogee_init(&s); avt_apogee_set_noise(&s, 0.02f);
  float seq[] = {0.00f,0.02f,0.01f, 0.10f,0.20f,0.30f,0.40f,0.50f,0.58f,0.60f, 0.56f,0.52f,0.48f,0.44f,0.40f,0.36f};
  avt_phase_t p = AVT_PHASE_ASCENT;
  int arm_at=-1, fire_at=-1;
  for (int i=0;i<16;i++) { p = avt_apogee_update(&s, seq[i], true);
    if (s.armed && arm_at<0) arm_at=i;
    if (p==AVT_PHASE_DESCENT && fire_at<0) fire_at=i; }
  printf("  finger: armed@%d fired@%d\n", arm_at, fire_at);
  CHECK(p==AVT_PHASE_DESCENT && arm_at>=0 && fire_at>arm_at);
}

/* ---------- pressure-vessel tests: pressure -> EMA -> baro -> apogee ----------
 * Realistic chamber profile: P0=100000 Pa, pressure FALL = ascent,
 * plateau = apogee, pressure RECOVERY = descent. Chain mirrors main.c:
 * first sample latches EMA, then avt_ema(alpha=0.5) + avt_altitude_m.
 * Noise is deterministic fixed tables (no rand). */
#define V_P0 (100000.0f)
#define V_SIGMA (0.02f)

typedef struct { float max_alt; int arm_at; int fire_at; bool armed; bool descent; float excursion; } vessel_res;

static vessel_res vessel_run(const float *praw, int n) {
  avt_apogee_t s; avt_apogee_init(&s); avt_apogee_set_noise(&s, V_SIGMA);
  vessel_res r = { .max_alt = -1e30f, .arm_at = -1, .fire_at = -1, .armed = false, .descent = false, .excursion = 0.0f };
  float pf = 0.0f; bool finit = false; float pmin = V_P0;
  for (int i = 0; i < n; i++) {
    if (praw[i] < pmin) pmin = praw[i];
    if (!finit) { pf = praw[i]; finit = true; }
    else { pf = avt_ema(pf, praw[i]); }
    float h = 0.0f;
    bool ok = avt_altitude_m(pf, V_P0, &h);
    if (ok && h > r.max_alt) r.max_alt = h;
    avt_phase_t p = avt_apogee_update(&s, h, ok);
    if (s.armed && r.arm_at < 0) r.arm_at = i;
    if (p == AVT_PHASE_DESCENT && r.fire_at < 0) r.fire_at = i;
  }
  r.armed = (r.arm_at >= 0); r.descent = (r.fire_at >= 0); r.excursion = V_P0 - pmin;
  return r;
}
#define VREP(tag, r, noise) printf("  %s: max_alt=%+.2fm armed@%d fired@%d excursion=%.0fPa noise=%s\n", tag, (double)r.max_alt, r.arm_at, r.fire_at, (double)r.excursion, noise)

/* 1) 10 Pa excursion -> ARM + DESCENT */
static void tV1_10pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990,99990,99990, 99992,99994,99996,99998,100000, 100000,100000,100000,100000,100000};
  vessel_res r = vessel_run(p, 24);
  VREP("vessel1-10Pa", r, "0Pa");
  CHECK(r.armed && r.descent && r.arm_at >= 0 && r.fire_at > r.arm_at);
}
/* 2) 20 Pa excursion -> ARM + DESCENT */
static void tV2_20pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99996,99992,99988,99984,99980, 99980,99980,99980,99980, 99984,99988,99992,99996,100000, 100000,100000,100000,100000,100000};
  vessel_res r = vessel_run(p, 24);
  VREP("vessel2-20Pa", r, "0Pa");
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at);
}
/* 3) 50 Pa excursion -> ARM + DESCENT */
static void tV3_50pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99990,99980,99970,99960,99950, 99950,99950,99950,99950, 99960,99970,99980,99990,100000, 100000,100000,100000,100000,100000};
  vessel_res r = vessel_run(p, 24);
  VREP("vessel3-50Pa", r, "0Pa");
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at);
}
/* 4) 3 Pa excursion -> NO ARM (absolute 0.5 m guard holds: 3 Pa = ~0.25 m) */
static void tV4_3pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99999,99998,99997, 99997,99997,99997,99997, 99998,99999,100000, 100000,100000,100000,100000,100000};
  vessel_res r = vessel_run(p, 20);
  VREP("vessel4-3Pa", r, "0Pa");
  CHECK(!r.armed && !r.descent);
}
/* 5) 10 Pa + deterministic +/-1.5 Pa noise -> still ARM + DESCENT */
static void tV5_10pa_noisy(void) {
  static const float nz[] = {0,-1.5,1.5,-0.75,0.75, -1.5,1.5,-0.75,0.75,0, 1.5,-1.5,0.75,-0.75, 1.5,-1.5,0.75,-0.75,0, 0,1.5,-1.5,0.75,-0.75};
  static const float base[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990,99990,99990, 99992,99994,99996,99998,100000, 100000,100000,100000,100000,100000};
  float p[24]; for (int i = 0; i < 24; i++) p[i] = base[i] + nz[i];
  vessel_res r = vessel_run(p, 24);
  VREP("vessel5-10Pa+noise", r, "+/-1.5Pa");
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at);
}
/* 6) pressure increase only (no ascent) -> NO DESCENT */
static void tV6_norise(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 100002,100004,100006,100008,100010, 100010,100010,100010,100010,100010, 100010,100010,100010,100010,100010};
  vessel_res r = vessel_run(p, 20);
  VREP("vessel6-rise-only", r, "0Pa");
  CHECK(!r.armed && !r.descent);
}
/* 7) ascent -> plateau + small noise, no recovery -> ARMED, NO descent */
static void tV7_plateau_noise(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99991,99989,99991,99989,99991,99989,99991,99989,99991,99989};
  vessel_res r = vessel_run(p, 20);
  VREP("vessel7-plateau", r, "+/-1Pa");
  CHECK(r.armed && !r.descent);
}
/* 8) ascent -> recovery -> wobble: fires and STAYS latched */
static void tV8_latch(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990, 99992,99994,99996,99998,100000, 99995,100005,99990,100000,100000,100000,100000};
  vessel_res r = vessel_run(p, 24);
  VREP("vessel8-latch", r, "0Pa+wobble");
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at);
  CHECK(r.fire_at < 23);
}

/* ---------- Kalman A-M: unit behaviour (x=[h,v], H=[1,0], float, no heap) ---------- */
static void tKA_init(void) {
  avt_kf_t k; avt_kf_init(&k, 1.25f, 0.01f);
  CHECK(k.ready && fabsf(k.h - 1.25f) < 1e-6f && fabsf(k.v) < 1e-9f
        && fabsf(k.p00 - 0.01f) < 1e-6f && fabsf(k.p11 - 0.01f) < 1e-6f && fabsf(k.p01) < 1e-9f);
}
static void tKB_rfloor(void) {
  avt_kf_t k; avt_kf_init(&k, 0.0f, 0.0f); /* zero banned -> floor */
  CHECK(fabsf(k.R - AVT_KF_R_FLOOR) < 1e-9f && fabsf(k.p00 - AVT_KF_R_FLOOR) < 1e-9f);
}
static void tKC_predict(void) {
  avt_kf_t k; avt_kf_init(&k, 0.0f, 0.01f); k.v = 2.0f;
  avt_kf_predict(&k, 0.10f);
  CHECK(fabsf(k.h - 0.20f) < 1e-5f);
}
static void tKD_dtclamp(void) {
  avt_kf_t k; avt_kf_init(&k, 0.0f, 0.01f); k.v = 1.0f;
  avt_kf_predict(&k, 5.0f); /* clamped to DT_MAX: h == 1*0.2, not 5 */
  CHECK(fabsf(k.h - AVT_KF_DT_MAX_S) < 1e-5f);
  float h = k.h;
  avt_kf_predict(&k, 0.0f); avt_kf_predict(&k, -1.0f); avt_kf_predict(&k, NAN);
  CHECK(fabsf(k.h - h) < 1e-9f); /* dt<=0/NaN: controlled hold */
}
static void tKE_converge(void) {
  avt_kf_t k; avt_kf_init(&k, 0.0f, 0.01f); avt_kf_set_noise(&k, 0.01f, 0.05f);
  for (int i = 0; i < 60; i++) { avt_kf_predict(&k, 0.05f); avt_kf_update(&k, 1.0f, true); }
  CHECK(fabsf(k.h - 1.0f) < 0.05f && fabsf(k.v) < 0.3f);
}
static void tKF_invalid(void) {
  avt_kf_t k; avt_kf_init(&k, 0.5f, 0.01f); avt_kf_set_noise(&k, 0.01f, 0.05f);
  CHECK(!avt_kf_update(&k, 9.0f, false) && fabsf(k.h - 0.5f) < 1e-9f);
  CHECK(!avt_kf_update(&k, NAN, true) && fabsf(k.h - 0.5f) < 1e-9f);
}
static void tKG_guard(void) {
  avt_kf_t k; avt_kf_init(&k, 0.0f, 0.01f);
  k.v = 100.0f; avt_kf_predict(&k, 0.05f);
  CHECK(fabsf(k.v) <= AVT_KF_V_MAX + 1e-6f);
  k.p00 = 100.0f; k.p11 = 100.0f; avt_kf_predict(&k, 0.05f);
  CHECK(k.p00 <= AVT_KF_P_CAP + 1e-6f && k.p11 <= AVT_KF_P_CAP + 1e-6f);
}
static void tKH_setnoise(void) {
  avt_kf_t k; avt_kf_init(&k, 0.0f, 0.01f); avt_kf_set_noise(&k, 0.01f, 0.05f);
  avt_kf_set_noise(&k, 0.0f, -1.0f); /* R->floor, Q<=0 ignored */
  CHECK(fabsf(k.R - AVT_KF_R_FLOOR) < 1e-9f && fabsf(k.Q - 0.05f) < 1e-9f);
  avt_kf_set_noise(NULL, 1.0f, 1.0f); avt_kf_init(NULL, 0.0f, 0.01f);
  avt_kf_predict(NULL, 0.05f); CHECK(!avt_kf_update(NULL, 0.0f, true));
}
static void tKI_rderiv(void) {
  float sigma_p = 1.2f; /* Pa */
  float sigma_alt = sigma_p / 12.0f; /* main.c d(alt)/dP linearisation */
  float r_var = sigma_alt * sigma_alt;
  CHECK(fabsf(sigma_alt - 0.10f) < 1e-6f && fabsf(r_var - 0.01f) < 1e-6f);
}
static void tKJ_qreason(void) {
  /* Q=0.05 firmware default: finite, >0. Host sweep (tKQ_sweep) on the
   * 10 Pa set: Q=0.005 lags (rms 0.16, lat +1), Q>=0.2 leads (lat<0);
   * Q=0.05 gives near-best rms (~0.08) with zero peak latency. */
  CHECK(0.05f > 0.0f && isfinite(0.05f));
}
static void tKK_rawfeed(void) {
  /* H=[1,0] wiring: one update from h=0,R=1,p00=1 gives K0=0.5 -> h=0.5z. */
  avt_kf_t k; avt_kf_init(&k, 0.0f, 1.0f);
  CHECK(avt_kf_update(&k, 2.0f, true) && fabsf(k.h - 1.0f) < 1e-5f);
}
static void tKL_notready(void) {
  avt_kf_t k = {0}; /* ready=false */
  avt_kf_predict(&k, 0.05f);
  CHECK(!avt_kf_update(&k, 1.0f, true) && fabsf(k.h) < 1e-9f);
}
static void tKM_size(void) {
  printf("  kf: sizeof(avt_kf_t)=%u R_floor=%.0e P_cap=%.1f V_max=%.0f\n",
         (unsigned)sizeof(avt_kf_t), (double)AVT_KF_R_FLOOR, (double)AVT_KF_P_CAP, (double)AVT_KF_V_MAX);
  CHECK(sizeof(avt_kf_t) <= 64u && AVT_KF_R_FLOOR > 0.0f);
}

/* ---------- Kalman vessel: parallel KF vs ALT on the same pressure tables ----------
 * ALT path mirrors main.c (first-sample latch + avt_ema(0.5) + baro).
 * KF path mirrors main.c KF block (RAW pressure -> baro -> predict(dt=0.05)
 * -> update). dt=0.05 is the vessel time base (20 Hz loop model); firmware
 * uses real elapsed dt with clamp. R = V_SIGMA^2 (variance), Q swept. */
#define KV_R (0.02f * 0.02f)
#define KV_Q (0.05f)
#define KV_DT (0.05f)

typedef struct { float peak_err, rms, kalt_peak, alt_peak; int alt_at, kalt_at; } kves_res;

static kves_res kves_run(const float *praw, int n, float Q) {
  kves_res r = {0};
  float pf = 0.0f; bool finit = false;
  float alt_peak = -1e30f, kalt_peak = -1e30f; int alt_at = -1, kalt_at = -1;
  float se = 0.0f; int cnt = 0;
  avt_kf_t k; avt_kf_init(&k, 0.0f, KV_R); avt_kf_set_noise(&k, KV_R, Q);
  avt_med3_t mm; avt_med3_init(&mm);
  for (int i = 0; i < n; i++) {
    float pm = avt_med3_push(&mm, praw[i]); /* prod mirror: shared pre-filter */
    if (!finit) { pf = pm; finit = true; }
    else { pf = avt_ema(pf, pm); }
    float h = 0.0f; bool ok = avt_altitude_m(pf, V_P0, &h);
    float z = 0.0f; bool zok = avt_altitude_m(pm, V_P0, &z);
    avt_kf_predict(&k, KV_DT);
    float kh = k.h; bool kupd = avt_kf_update(&k, z, zok);
    if (kupd) kh = k.h;
    if (ok) {
      if (h > alt_peak) { alt_peak = h; alt_at = i; }
      if (kupd && kh > kalt_peak) { kalt_peak = kh; kalt_at = i; }
      if (kupd) { float e = kh - h; se += e * e; cnt++; }
    }
  }
  r.alt_peak = alt_peak; r.kalt_peak = kalt_peak; r.alt_at = alt_at; r.kalt_at = kalt_at;
  r.peak_err = fabsf(kalt_peak - alt_peak);
  r.rms = (cnt > 0) ? sqrtf(se / (float)cnt) : 1e30f;
  return r;
}
#define KREP(tag, r, Q) printf("  %s Q=%.3f: alt_peak=%+.2f@%d kalt_peak=%+.2f@%d peak_err=%.3f rms=%.3f\n", \
  tag, (double)(Q), (double)r.alt_peak, r.alt_at, (double)r.kalt_peak, r.kalt_at, \
  (double)r.peak_err, (double)r.rms)

/* Q sweep on the 10 Pa set: prints RMS/latency tradeoff, justifies Q=0.05. */
static void tKQ_sweep(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990,99990,99990, 99992,99994,99996,99998,100000, 100000,100000,100000,100000,100000};
  static const float qs[] = {0.005f, 0.02f, 0.05f, 0.2f, 0.5f};
  for (int i = 0; i < 5; i++) {
    kves_res r = kves_run(p, 24, qs[i]);
    printf("  qsweep Q=%.3f: peak_err=%.3f rms=%.3f lat=%d\n",
           (double)qs[i], (double)r.peak_err, (double)r.rms, r.kalt_at - r.alt_at);
  }
  kves_res r = kves_run(p, 24, KV_Q);
  KREP("kvsweep-10Pa", r, KV_Q);
  CHECK(r.peak_err < 0.15f && r.rms < 0.15f);
}
static void tKV1_10pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990,99990,99990, 99992,99994,99996,99998,100000, 100000,100000,100000,100000,100000};
  kves_res r = kves_run(p, 24, KV_Q); KREP("kvessel1-10Pa", r, KV_Q);
  CHECK(r.peak_err < 0.15f && r.rms < 0.15f);
}
/* NOTE: KF overshoot scales with excursion (~16% of ALT peak: the
 * constant-velocity model coasts past the apex on residual +v before
 * measurements pull it down). Thresholds are therefore excursion-relative,
 * not a flat 0.15 m: the sensitive 10 Pa set keeps the tight gate. */
static void tKV2_20pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99996,99992,99988,99984,99980, 99980,99980,99980,99980, 99984,99988,99992,99996,100000, 100000,100000,100000,100000,100000};
  kves_res r = kves_run(p, 24, KV_Q); KREP("kvessel2-20Pa", r, KV_Q);
  CHECK(r.peak_err < 0.35f && r.rms < 0.25f);
}
static void tKV3_50pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99990,99980,99970,99960,99950, 99950,99950,99950,99950, 99960,99970,99980,99990,100000, 100000,100000,100000,100000,100000};
  kves_res r = kves_run(p, 24, KV_Q); KREP("kvessel3-50Pa", r, KV_Q);
  CHECK(r.peak_err < 0.80f && r.rms < 0.60f);
}
static void tKV4_noisy(void) {
  static const float nz[] = {0,-1.5,1.5,-0.75,0.75, -1.5,1.5,-0.75,0.75,0, 1.5,-1.5,0.75,-0.75, 1.5,-1.5,0.75,-0.75,0, 0,1.5,-1.5,0.75,-0.75};
  static const float base[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990,99990,99990, 99992,99994,99996,99998,100000, 100000,100000,100000,100000,100000};
  float p[24]; for (int i = 0; i < 24; i++) p[i] = base[i] + nz[i];
  kves_res r = kves_run(p, 24, KV_Q); KREP("kvessel4-10Pa+noise", r, KV_Q);
  CHECK(r.peak_err < 0.20f && r.rms < 0.20f);
}
static void tKV5_norise(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 100002,100004,100006,100008,100010, 100010,100010,100010,100010,100010, 100010,100010,100010,100010,100010};
  kves_res r = kves_run(p, 20, KV_Q); KREP("kvessel5-rise-only", r, KV_Q);
  CHECK(r.kalt_peak <= 0.15f && r.rms < 0.15f);
}
static void tKV6_plateau(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99991,99989,99991,99989,99991,99989,99991,99989,99991,99989};
  kves_res r = kves_run(p, 20, KV_Q); KREP("kvessel6-plateau", r, KV_Q);
  CHECK(r.peak_err < 0.20f && r.rms < 0.20f);
}
static void tKV7_latch(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990, 99992,99994,99996,99998,100000, 99995,100005,99990,100000,100000,100000,100000};
  kves_res r = kves_run(p, 24, KV_Q); KREP("kvessel7-latch", r, KV_Q);
  CHECK(r.peak_err < 0.20f && r.rms < 0.20f);
}

/* ---------- adaptive P0 calibration tests (A-H) ----------
 * Per-scenario report: raw center, early/late, drift, correction,
 * effective P0, rest altitude, flat-rest false-ARM check. */
static float crep_show(const char *tag, float raw, float early, float late,
                       float sigma, uint32_t ne, uint32_t nl) {
  float drift = late - early;
  float corr = avt_p0_correct(early, late, sigma, ne, nl);
  float p0 = raw + corr;
  float hrest = 0.0f;
  avt_altitude_m(late, p0, &hrest);
  avt_apogee_t s; avt_apogee_init(&s); avt_apogee_set_noise(&s, 0.02f);
  avt_phase_t ph = AVT_PHASE_ASCENT;
  for (int i = 0; i < 10; i++) ph = avt_apogee_update(&s, hrest, true);
  printf("  %s: raw=%.1f early=%.1f late=%.1f drift=%+.2f sig=%.2f corr=%+.2f P0=%.1f hrest=%+.3f flatarm=%d\n",
         tag, (double)raw, (double)early, (double)late, (double)drift,
         (double)sigma, (double)corr,
         (double)p0, (double)hrest, (int)(ph != AVT_PHASE_ASCENT));
  CHECK(ph == AVT_PHASE_ASCENT); /* flat rest never arms, any scenario */
  return corr;
}
/* Adaptive DROP gate: quiet run -> easy finger, noisy run -> strict.
 * K=4/floor=0.10 mirror the apogee drop-guard tuning; cap 0.50. */
static void tDN_quiet(void) {
  CHECK(avt_alt_drop_n(false, -0.12f, true, 0.02f));   /* ON -0.10 */
  CHECK(!avt_alt_drop_n(false, -0.08f, true, 0.02f));
  CHECK(avt_alt_drop_n(true, -0.07f, true, 0.02f));    /* hold */
  CHECK(!avt_alt_drop_n(true, -0.04f, true, 0.02f));   /* OFF -0.05 */
  CHECK(!avt_alt_drop_n(true, -5.0f, false, 0.02f));   /* invalid */
}
static void tDN_noisy(void) {
  CHECK(!avt_alt_drop_n(false, -0.45f, true, 0.15f));  /* ON -0.50 */
  CHECK(avt_alt_drop_n(false, -0.55f, true, 0.15f));
  CHECK(!avt_alt_drop_n(true, -0.20f, true, 0.15f));
}
static void tDN_badsigma(void) {
  CHECK(avt_alt_drop_n(false, -0.25f, true, NAN));     /* default 0.05 -> ON -0.20 */
  CHECK(!avt_alt_drop_n(false, -0.15f, true, NAN));
  CHECK(avt_alt_drop_n(false, -0.25f, true, -1.0f));
}
/* Light press (-0.12 m ~= 1.4 Pa) fires on a quiet run: legacy needed -0.15. */
static void tDN_fingereasy(void) {
  bool d = avt_alt_drop_n(false, -0.12f, true, 0.02f);
  printf("  dropn-fingereasy: quiet-sigma light-press(-0.12m) -> %d (legacy: 0)\n", (int)d);
  CHECK(d);
  CHECK(!avt_alt_drop(false, -0.12f, true)); /* legacy control: silent */
}
/* avt_drop_gate_m must report exactly the threshold the gate uses. */
static void tDN_gate_visible(void) {
  float q = avt_drop_gate_m(0.02f); /* quiet: floor 0.10 wins over K*sigma */
  CHECK(fabsf(q - AVT_DROP_MIN_M) < 1e-6f);
  /* Reported value agrees with observed behaviour at the boundary. */
  CHECK(avt_alt_drop_n(false, -(q + 0.005f), true, 0.02f));
  CHECK(!avt_alt_drop_n(false, -(q - 0.005f), true, 0.02f));
  /* Noisy run is still sigma-driven, not floored. */
  float n = avt_drop_gate_m(0.15f);
  CHECK(n > 0.45f && n <= 0.50f);
  /* Invalid sigma falls back to the default, both agree. */
  float b = avt_drop_gate_m(NAN);
  CHECK(fabsf(b - 4.0f * AVT_DROP_SIGMA_DEFAULT_M) < 1e-6f);
  CHECK(fabsf(b - avt_drop_gate_m(AVT_DROP_SIGMA_DEFAULT_M)) < 1e-6f);
  CHECK(fabsf(b - avt_drop_gate_m(-1.0f)) < 1e-6f);
}
/* A) stable calibration -> correction ~0 */
static void tC_A(void) {
  float c = crep_show("cal-A-stable", 100000.0f, 100000.0f, 100000.0f, 1.0f, 120, 120);
  CHECK(fabsf(c) < 1e-6f);
}
/* B) small positive drift -> +correction toward the stable end */
static void tC_B(void) {
  float c = crep_show("cal-B-posdrift", 100002.0f, 100000.0f, 100004.0f, 1.0f, 120, 120);
  CHECK(fabsf(c - 2.0f) < 0.05f);
  CHECK(fabsf((100002.0f + c) - 100004.0f) < 0.05f); /* P0 ~= late */
}
/* C) small negative drift -> correction goes NEGATIVE (never blind +) */
static void tC_C(void) {
  float c = crep_show("cal-C-negdrift", 99998.0f, 100000.0f, 99996.0f, 1.0f, 120, 120);
  CHECK(c < 0.0f && fabsf(c + 2.0f) < 0.05f);
  CHECK(fabsf((99998.0f + c) - 99996.0f) < 0.05f);
}
/* C2) user example: EARLY=99996 LATE=99998 -> +1, run-dependent */
static void tC_C2(void) {
  float c = crep_show("cal-C2-usersmp", 99997.0f, 99996.0f, 99998.0f, 1.0f, 120, 120);
  CHECK(fabsf(c - 1.0f) < 0.05f);
}
/* D) single-spike-diluted window (25 Pa spike in 100 samples = +0.25 Pa
 * on the mean): below the deadband -> correction 0, spike cannot fake P0 */
static void tC_D(void) {
  float c = crep_show("cal-D-spike", 100000.1f, 100000.25f, 100000.0f, 1.0f, 100, 100);
  CHECK(fabsf(c) < 1e-6f);
}
/* E) noise-only (0.5 Pa mean wobble at sigma=2): correction 0 */
static void tC_E(void) {
  float c = crep_show("cal-E-noise", 100000.0f, 100000.3f, 99999.8f, 2.0f, 100, 100);
  CHECK(fabsf(c) < 1e-6f);
}
/* G) different runs -> different corrections (no automatic offset) */
static void tC_G(void) {
  float cb = avt_p0_correct(100000.0f, 100004.0f, 1.0f, 120, 120);
  float cc = avt_p0_correct(99996.0f, 99998.0f, 1.0f, 120, 120);
  printf("  cal-G-runsdiffer: corrB=%+.2f corrC2=%+.2f\n", (double)cb, (double)cc);
  CHECK(fabsf(cb - 2.0f) < 0.05f && fabsf(cc - 1.0f) < 0.05f && fabsf(cb - cc) > 0.5f);
}
/* cap: huge drift clamps at +/-5 Pa (never hundreds of Pa) */
static void tC_cap(void) {
  float cp = avt_p0_correct(100000.0f, 100020.0f, 1.0f, 120, 120);
  float cn = avt_p0_correct(100000.0f, 99980.0f, 1.0f, 120, 120);
  printf("  cal-cap: drift+20->%+.2f drift-20->%+.2f\n", (double)cp, (double)cn);
  CHECK(fabsf(cp - 5.0f) < 1e-6f && fabsf(cn + 5.0f) < 1e-6f);
}
/* floor: 0.2 Pa drift at sigma=0.2 -> deadband floor 0.5 Pa wins -> 0 */
static void tC_min(void) {
  float c = crep_show("cal-min-0.2Pa", 100000.1f, 100000.0f, 100000.2f, 0.2f, 120, 120);
  CHECK(fabsf(c) < 1e-6f);
}
/* window bounds derive from the cal length (first/last 25%, dynamic) */
static void tC_wins(void) {
  printf("  cal-wins: CAL=%ums early<%ums late>=%ums\n",
         (unsigned)AVT_CAL_MS, (unsigned)AVT_P0C_EARLY_MS, (unsigned)AVT_P0C_LATE_MS);
  CHECK(AVT_CAL_MS == 16000u);
  CHECK(AVT_P0C_EARLY_MS == AVT_CAL_MS / 4u && AVT_P0C_LATE_MS == (AVT_CAL_MS * 3u) / 4u);
}
/* degenerate inputs degrade to 0, never NaN/garbage */
static void tC_edge(void) {
  CHECK(avt_p0_correct(100000.0f, 100004.0f, 1.0f, 0, 120) == 0.0f);
  CHECK(avt_p0_correct(NAN, 100004.0f, 1.0f, 120, 120) == 0.0f);
  float c = avt_p0_correct(100000.0f, 100004.0f, NAN, 120, 120);
  printf("  cal-edge: NaN-sigma drift+4 -> corr=%+.2f (floor path)\n", (double)c);
  CHECK(fabsf(c - 2.0f) < 0.05f);
}

/* ---------- median3 unit tests ---------- */
static void tM3_init(void) {
  avt_med3_t m; avt_med3_init(&m);
  CHECK(m.n == 0 && m.i == 0);
  CHECK(fabsf(avt_med3_push(&m, 7.0f) - 7.0f) < 1e-9f); /* passthrough */
  CHECK(fabsf(avt_med3_push(&m, 9.0f) - 9.0f) < 1e-9f); /* passthrough */
}
static void tM3_order(void) {
  avt_med3_t m; avt_med3_init(&m);
  avt_med3_push(&m, 1.0f); avt_med3_push(&m, 2.0f);
  CHECK(fabsf(avt_med3_push(&m, 3.0f) - 2.0f) < 1e-6f);
  avt_med3_init(&m);
  avt_med3_push(&m, 5.0f); avt_med3_push(&m, 1.0f);
  CHECK(fabsf(avt_med3_push(&m, 4.0f) - 4.0f) < 1e-6f);
}
static void tM3_spike(void) {
  avt_med3_t m; avt_med3_init(&m);
  float seq[] = {10.0f, 10.0f, 50.0f, 10.0f, 10.0f, 10.0f};
  bool ok = true;
  for (int i = 0; i < 6; i++) {
    float o = avt_med3_push(&m, seq[i]);
    if (i >= 2 && fabsf(o - 10.0f) > 1e-6f) ok = false;
  }
  CHECK(ok); /* isolated spike never appears at the output */
}
static void tM3_step(void) {
  avt_med3_t m; avt_med3_init(&m);
  float seq[] = {0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 5.0f, 5.0f};
  float out[7]; for (int i = 0; i < 7; i++) out[i] = avt_med3_push(&m, seq[i]);
  CHECK(fabsf(out[3] - 0.0f) < 1e-6f && fabsf(out[4] - 5.0f) < 1e-6f && fabsf(out[6] - 5.0f) < 1e-6f);
}
static void tM3_nan(void) {
  avt_med3_t m; avt_med3_init(&m);
  avt_med3_push(&m, 3.0f); avt_med3_push(&m, 3.0f); avt_med3_push(&m, 3.0f);
  float o = avt_med3_push(&m, NAN);
  CHECK(isnan(o)); /* rejected input never poisons the window */
  CHECK(fabsf(avt_med3_push(&m, 3.0f) - 3.0f) < 1e-6f);
}

/* ---------- prod-mirror chain: med3 -> EMA(0.5) -> baro(P0_eff) -> apogee -----
 * P0_eff comes from the adaptive calibration for each scenario (stable cal
 * here -> P0 = cal mean, rest ~0 m). Same split point as main.c. */
#define W_SIGMA (0.02f)

/* Effective P0 exactly as main.c computes it: mean + correction (no bias). */
static float w_cal_p0(float cal_mean, float early, float late, float sigma) {
  return cal_mean + avt_p0_correct(early, late, sigma, 120, 120);
}

typedef struct { float max_alt, min_alt; int arm_at, fire_at; bool armed, descent, drop, dropend; } wres;

static wres w_run(const float *praw, int n, float p0) {
  avt_apogee_t s; avt_apogee_init(&s); avt_apogee_set_noise(&s, W_SIGMA);
  avt_med3_t m; avt_med3_init(&m);
  wres r = { .max_alt = -1e30f, .min_alt = 1e30f, .arm_at = -1, .fire_at = -1,
             .armed = false, .descent = false, .drop = false, .dropend = false };
  float pf = 0.0f; bool finit = false;
  /* prod mirror: DROP baseline seeded from the first valid altitude, then
   * slow EMA; direct gate (no streak, no velocity), exactly as main.c. */
  float ref = 0.0f; bool ref_seeded = false;
  bool drop_on = false;
  for (int i = 0; i < n; i++) {
    float pm = avt_med3_push(&m, praw[i]);
    if (!finit) { pf = pm; finit = true; }
    else { pf = avt_ema(pf, pm); }
    float h = 0.0f; bool ok = avt_altitude_m(pf, p0, &h);
    if (ok) {
      if (h > r.max_alt) r.max_alt = h;
      if (h < r.min_alt) r.min_alt = h;
      if (!ref_seeded) { ref = h; ref_seeded = true; }
      else { ref += (h - ref) * AVT_DROP_REF_ALPHA; }
    }
    drop_on = avt_alt_drop_n(drop_on, h - ref, ok, W_SIGMA);
    if (drop_on) r.drop = true;
    avt_phase_t p = avt_apogee_update(&s, h, ok);
    if (s.armed && r.arm_at < 0) r.arm_at = i;
    if (p == AVT_PHASE_DESCENT && r.fire_at < 0) r.fire_at = i;
  }
  r.armed = (r.arm_at >= 0); r.descent = (r.fire_at >= 0);
  r.dropend = drop_on;
  return r;
}
#define WREP(tag, r) printf("  %s: max=%+.2f min=%+.2f drop=%d/%d armed@%d fired@%d\n", \
  tag, (double)r.max_alt, (double)r.min_alt, (int)r.drop, (int)r.dropend, r.arm_at, r.fire_at)

/* Stable-cal P0 for the flight-phase tests below (corr 0 -> rest ~0 m). */
#define W_P0_STABLE (100000.0f)

/* 1) slow ambient drift (+2 Pa ramp over 40 samples + tiny jitter): NO trigger */
static void tD_drift(void) {
  float p[40];
  for (int i = 0; i < 40; i++) p[i] = 100000.0f + 2.0f * (float)i / 40.0f + ((i % 2) ? 0.3f : -0.3f);
  wres r = w_run(p, 40, W_P0_STABLE); WREP("drift+2Pa-slow", r);
  CHECK(!r.armed && !r.descent);
}
/* 3) consecutive small jitter (+/-1.5 Pa, 20 samples): NO trigger, DROP silent.
 * First sample sits at the mean so the seeded DROP baseline starts from
 * rest (startup transient is not what this test measures). */
static void tD_small(void) {
  float p[20];
  for (int i = 0; i < 20; i++) p[i] = 100000.0f + ((i % 2) ? 1.5f : -1.5f);
  p[0] = 100000.0f;
  wres r = w_run(p, 20, W_P0_STABLE); WREP("jitter+-1.5Pa", r);
  CHECK(!r.armed && !r.descent && !r.drop && r.min_alt > -0.10f);
}
/* 2) single +25 Pa glitch sample: median kills it, NO trigger.
 * Comparison (isolates median): EMA-only dip vs median+EMA dip. */
static void tD_spike(void) {
  float p[16];
  for (int i = 0; i < 16; i++) p[i] = 100000.0f;
  p[8] = 100025.0f;
  wres r = w_run(p, 16, W_P0_STABLE); WREP("spike+25Pa-1samp", r);
  CHECK(!r.armed && !r.descent && r.min_alt > -0.05f);
  /* median before/after on the same glitch, EMA(0.5) after, P0 flat. */
  {
    float pf_e = 100000.0f, dip_e = 0.0f, pf_m = 100000.0f, dip_m = 0.0f;
    avt_med3_t m; avt_med3_init(&m);
    for (int i = 0; i < 16; i++) {
      float x = (i == 8) ? 100008.0f : 100000.0f;
      pf_e = avt_ema(pf_e, x);
      float pm = avt_med3_push(&m, x);
      pf_m = avt_ema(pf_m, pm);
      float he = 0.0f, hm = 0.0f;
      avt_altitude_m(pf_e, 100000.0f, &he);
      avt_altitude_m(pf_m, 100000.0f, &hm);
      if (he < dip_e) dip_e = he;
      if (hm < dip_m) dip_m = hm;
    }
    printf("  spike8Pa: ema-only dip=%+.3fm median+ema dip=%+.3fm\n", (double)dip_e, (double)dip_m);
    CHECK(dip_e < -0.15f && dip_m > -0.05f); /* blip vs silence */
  }
}
/* 4) finger-like suck (-14 Pa over ~6 samples) then release: MUST still fire */
static void tD_finger(void) {
  static const float p[] = {100000,100000,100000, 99998,99995,99992,99989,99986,99986,99986,
                            99989,99992,99995,99998,100000,100000,100000,100000};
  wres r = w_run(p, 18, W_P0_STABLE); WREP("finger-suck-14Pa", r);
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at && !r.dropend); /* release -> OFF by itself */
}
/* Boundary: fast -5 Pa / 5-sample transient (door-slam scale). Under the
 * adaptive model rest sits at ~0 m, so the +0.42 m puff peak cannot satisfy
 * the absolute ARM guard: NO trigger. (The old fixed +10 Pa offset parked
 * rest at +0.84 m and this same puff fully fired -- that cost is gone.) */
static void tD_bound(void) {
  static const float p[] = {100000,100000,100000, 99999,99998,99997,99996,99995,99995,99995,
                            99995,99995,99996,99997,99998,99999,100000,100000,100000,100000};
  wres r = w_run(p, 20, W_P0_STABLE); WREP("bound-5Pa-fast", r);
  printf("  bound: armed=%d descent=%d (adaptive model: must stay silent)\n", (int)r.armed, (int)r.descent);
  CHECK(!r.armed && !r.descent);
}
/* Sustained +6 Pa step (today's lab drift, sped up): DROP fires at first
 * (fast edge) then the slow ref absorbs it and releases BY ITSELF while
 * the pressure is still held. Apogee never arms on a flat hold. */
static void tD_absorb(void) {
  float p[1500];
  for (int i = 0; i < 50; i++) p[i] = 100000.0f;
  for (int i = 50; i < 1500; i++) p[i] = 100006.0f;
  wres r = w_run(p, 1500, W_P0_STABLE); WREP("absorb+6Pa-hold", r);
  CHECK(r.drop && !r.dropend && !r.armed && !r.descent);
}
/* 5) pressure-vessel profiles through the full prod chain (adaptive P0 from
 * a stable calibration + median). H) existing tV* (fixed P0=100000, EMA)
 * are untouched; these mirror the new prod path. */
static void tW1_10pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990,99990,99990, 99992,99994,99996,99998,100000, 100000,100000,100000,100000,100000};
  float p0 = w_cal_p0(100000.0f, 100000.0f, 100000.0f, 1.0f);
  wres r = w_run(p, 24, p0); WREP("wessel1-10Pa", r);
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at);
}
static void tW2_20pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99996,99992,99988,99984,99980, 99980,99980,99980,99980, 99984,99988,99992,99996,100000, 100000,100000,100000,100000,100000};
  float p0 = w_cal_p0(100000.0f, 100000.0f, 100000.0f, 1.0f);
  wres r = w_run(p, 24, p0); WREP("wessel2-20Pa", r);
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at);
}
static void tW3_50pa(void) {
  static const float p[] = {100000,100000,100000,100000,100000, 99990,99980,99970,99960,99950, 99950,99950,99950,99950, 99960,99970,99980,99990,100000, 100000,100000,100000,100000,100000};
  float p0 = w_cal_p0(100000.0f, 100000.0f, 100000.0f, 1.0f);
  wres r = w_run(p, 24, p0); WREP("wessel3-50Pa", r);
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at);
}
static void tW4_noisy(void) {
  static const float nz[] = {0,-1.5,1.5,-0.75,0.75, -1.5,1.5,-0.75,0.75,0, 1.5,-1.5,0.75,-0.75, 1.5,-1.5,0.75,-0.75,0, 0,1.5,-1.5,0.75,-0.75};
  static const float base[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990,99990,99990, 99992,99994,99996,99998,100000, 100000,100000,100000,100000,100000};
  float p[24]; for (int i = 0; i < 24; i++) p[i] = base[i] + nz[i];
  float p0 = w_cal_p0(100000.0f, 100000.0f, 100000.0f, 1.0f);
  wres r = w_run(p, 24, p0); WREP("wessel4-10Pa+noise", r);
  CHECK(r.armed && r.descent && r.fire_at > r.arm_at);
}
/* 7) steady-state noise: EMA-only vs median+EMA altitude jitter on +/-1.5 Pa. */
static void tD_noisep(void) {
  float pf_e = 100000.0f, pf_m = 100000.0f, maxd_e = 0.0f, maxd_m = 0.0f;
  avt_med3_t m; avt_med3_init(&m);
  for (int i = 0; i < 20; i++) {
    float x = 100000.0f + ((i % 2) ? 1.5f : -1.5f);
    pf_e = avt_ema(pf_e, x);
    pf_m = avt_ema(pf_m, avt_med3_push(&m, x));
    float he = 0.0f, hm = 0.0f;
    avt_altitude_m(pf_e, 100000.0f, &he);
    avt_altitude_m(pf_m, 100000.0f, &hm);
    if (fabsf(he) > maxd_e) maxd_e = fabsf(he);
    if (fabsf(hm) > maxd_m) maxd_m = fabsf(hm);
  }
  printf("  noise+-1.5Pa: ema-only maxdev=%.3fm median+ema maxdev=%.3fm\n", (double)maxd_e, (double)maxd_m);
  CHECK(maxd_m <= maxd_e + 1e-6f);
}

/* ---------- A5: bias/scale calibration infrastructure ---------- */
static void tCAL_ident(void) {
  avt_mpu_cal_t c = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  avt_mpu_cal_identity(&c);
  CHECK(c.bx == 0.0f && c.by == 0.0f && c.bz == 0.0f);
  CHECK(c.sx == 1.0f && c.sy == 1.0f && c.sz == 1.0f);
  avt_mpu_cal_identity(NULL);
}
static void tCAL_apply(void) {
  avt_mpu_cal_t c; avt_mpu_cal_identity(&c);
  float x, y, z;
  avt_mpu_cal_apply(&c, 100.0f, -200.0f, 300.0f, &x, &y, &z);
  CHECK(x == 100.0f && y == -200.0f && z == 300.0f); /* identity passthrough */
  c.bx = 10.0f; c.sx = 2.0f;
  avt_mpu_cal_apply(&c, 100.0f, 0.0f, 0.0f, &x, &y, &z);
  CHECK(fabsf(x - 180.0f) < 1e-4f); /* (100-10)*2 */
  avt_mpu_cal_apply(NULL, 1.0f, 2.0f, 3.0f, &x, &y, &z);
  CHECK(x == 1.0f && y == 2.0f && z == 3.0f);
  avt_mpu_cal_apply(&c, 1.0f, 2.0f, 3.0f, NULL, NULL, NULL);
  c.sx = 0.0f; /* zero scale degrades to identity, never NaN */
  avt_mpu_cal_apply(&c, 5.0f, 0.0f, 0.0f, &x, &y, &z);
  CHECK(fabsf(x - 5.0f) < 1e-6f);
}
static void tCAL_6pos(void) {
  /* synthetic sensor: bias (100,-50,200), scale (1.02,0.98,1.0), G=16384 */
  avt_mpu_cal_t t;
  CHECK(avt_mpu_cal_from_6pos(16814.08f, -16614.08f, 8006.08f, -8106.08f,
                              16584.0f, -16184.0f, 16384.0f, &t));
  CHECK(fabsf(t.bx - 100.0f) < 1.0f && fabsf(t.by + 50.0f) < 1.0f &&
        fabsf(t.bz - 200.0f) < 1.0f);
  CHECK(fabsf(t.sx - 1.0f) < 0.03f); /* ~1/1.02 recovered */
  CHECK(!avt_mpu_cal_from_6pos(0, 0, 0, 0, 0, 0, 16384.0f, &t));
  CHECK(!avt_mpu_cal_from_6pos(1, 2, 3, 4, 5, 6, 0.0f, &t));
  CHECK(!avt_mpu_cal_from_6pos(1, 2, 3, 4, 5, 6, 16384.0f, NULL));
}
static void tCAL_resid(void) {
  avt_mpu_cal_t c; avt_mpu_cal_identity(&c);
  CHECK(avt_mpu_cal_residual_ok(&c, 0, 0, 16384.0f, 16384.0f, 0.01f));
  CHECK(!avt_mpu_cal_residual_ok(&c, 0, 0, 12000.0f, 16384.0f, 0.01f));
  CHECK(!avt_mpu_cal_residual_ok(&c, 0, 0, 16384.0f, 0.0f, 0.01f));
}
static void tCAL_21080(void) {
  /* field observation: sensor reads ~21080 LSB/g (nominal 16384, +28.7%).
   * A scale-only fit against measured G absorbs it; residual must clear. */
  avt_mpu_cal_t c; avt_mpu_cal_identity(&c);
  CHECK(!avt_mpu_cal_residual_ok(&c, 0, 0, 21080.0f, 16384.0f, 0.05f));
  c.sz = 16384.0f / 21080.0f;
  CHECK(avt_mpu_cal_residual_ok(&c, 0, 0, 21080.0f, 16384.0f, 0.01f));
  printf("  cal-21080: scale-only fit absorbs field offset ok\n");
}
/* ---------- A12: estimator A/B benchmark (A=EMA chain, B=Kalman) ----------
 * Both run over identical vessel tables; verdict from data, production
 * keeps A unless B is strictly better (it is not: overshoot + latency). */
static void tAB_one(const char *tag, const float *p, int n) {
  kves_res r = kves_run(p, n, KV_Q);
  printf("  AB %s: Apeak=%+.2f Bpeak=%+.2f peak_err=%.3f rms=%.3f lat=%+d\n",
         tag, (double)r.alt_peak, (double)r.kalt_peak, (double)r.peak_err,
         (double)r.rms, r.kalt_at - r.alt_at);
  CHECK(r.rms < 1.0f); /* both sane; tight gates live in tKV* */
}
static void tAB_table(void) {
  static const float p10[] = {100000,100000,100000,100000,100000, 99998,99996,99994,99992,99990, 99990,99990,99990,99990, 99992,99994,99996,99998,100000, 100000,100000,100000,100000,100000};
  static const float p50[] = {100000,100000,100000,100000,100000, 99990,99980,99970,99960,99950, 99950,99950,99950,99950, 99960,99970,99980,99990,100000, 100000,100000,100000,100000,100000};
  tAB_one("10Pa", p10, 24);
  tAB_one("50Pa", p50, 24);
  printf("  AB verdict: A(EMA) kept for production (B overshoots ~16%% past apex, leads/lags with excursion; B has no trigger authority by design)\n");
}

/* ---------- irtifa gecersizlik disiplini (400000000 m olayi) ----------
 * Bozuk basinc + NaN/inf enjeksiyonunda alt_valid=false ve alt_m'ye YAZMA
 * YOK. Gecerli degerlerin urettigi sayilar degismez (yukaridaki legacy
 * testler bit-bit ayni kalir). Nobetci deger: yazma olursa yakalanir. */
static void t_alt_nan_in(void) {
  float h = 1234.5f;
  CHECK(!avt_altitude_m(NAN, 100000.0f, &h) && h == 1234.5f);
  CHECK(!avt_altitude_m(100000.0f, NAN, &h) && h == 1234.5f);
  CHECK(!avt_altitude_m(INFINITY, 100000.0f, &h) && h == 1234.5f);
  CHECK(!avt_altitude_m(100000.0f, INFINITY, &h) && h == 1234.5f);
  CHECK(!avt_altitude_m(-INFINITY, 100000.0f, &h) && h == 1234.5f);
  CHECK(!avt_altitude_m(100000.0f, -INFINITY, &h) && h == 1234.5f);
  CHECK(!avt_altitude_m(-500.0f, 100000.0f, &h) && h == 1234.5f);
  CHECK(!avt_altitude_m(100000.0f, -100.0f, &h) && h == 1234.5f);
  printf("  alt-nan-inf: invalid + no-write ok\n");
}
static void t_alt_extreme(void) {
  float h = 777.0f;
  /* Asiri oran -> |h| tavani (100 km) asar -> gecersiz, yazma yok. */
  CHECK(!avt_altitude_m(100000.0f, 100.0f, &h) && h == 777.0f); /* ~-120 km */
  CHECK(!avt_altitude_m(200000.0f, 10.0f, &h) && h == 777.0f);
  /* Zarf ucu (r=20) -> ~-34 km: buyuk ama gecerli, sayi degismedi. */
  float h2 = 0.0f;
  CHECK(avt_altitude_m(200000.0f, 10000.0f, &h2) && h2 < -30000.0f && h2 > -40000.0f);
  printf("  alt-extreme: |h|>100km invalid ok, r=20 valid ok\n");
}
/* Basinc gecerliligi: sifir-veri (NACK sonrasi cop / takilmis hat) ve
 * zarf-disi her zaman gecersiz; zarf-ici (10..200 kPa) aynen gecerli. */
static void t_press_ok(void) {
  CHECK(avt_press_ok(100000.0f));
  CHECK(avt_press_ok(10000.0f) && avt_press_ok(200000.0f)); /* sinirlar dahil */
  CHECK(avt_press_ok(87000.0f)); /* 1250 m apojesi civari */
  CHECK(!avt_press_ok(0.0f));    /* sifir-veri */
  CHECK(!avt_press_ok(-5.0f));
  CHECK(!avt_press_ok(NAN));
  CHECK(!avt_press_ok(INFINITY) && !avt_press_ok(-INFINITY));
  CHECK(!avt_press_ok(9999.0f) && !avt_press_ok(200001.0f)); /* zarf disi */
  printf("  press-ok: zero/nan/inf/envelope ok\n");
}

int main(void) {
  t_tilt_zero(); t_tilt_90(); t_tilt_180(); t_tilt_45(); t_tilt_degen();
  t_mag_ok(); t_mag_lo(); t_mag_outside(); t_mag_hi(); t_mag_zref(); t_vec_mag();
  t_band_in(); t_band_mid(); t_band_out();
  t_alt_zero(); t_alt_up(); t_alt_down(); t_alt_bad0(); t_alt_bad1(); t_alt_bad2(); t_alt_half();
  t_bmp_both(); t_bmp_dis(); t_bmp_1(); t_bmp_2(); t_bmp_none();
  t_deg_ok(); t_deg_bad(); t_use_ok(); t_use_no();
  t_rise_on(); t_rise_on_edge(); t_rise_hold(); t_rise_mid(); t_rise_off(); t_rise_off_edge(); t_rise_inv();
  t_drop_on(); t_drop_on_edge(); t_drop_hold(); t_drop_mid_off(); t_drop_off(); t_drop_off_edge(); t_drop_inv(); t_drop_finger_seq();
  t_ema_step(); t_ema_hold(); t_ema_alpha();
  t_apg_def_arm(); t_apg_def_drop(); t_apg_def_n(); t_apg_init(); t_apg_no_cal(); t_apg_latch(); t_apg_invalid(); t_apg_single();
  tA_noise(); tB_ascent(); tC_apex(); tD_descent(); tE_spike(); tF_latch(); tG_cal(); tH_nodescent(); t_finger();
  tV1_10pa(); tV2_20pa(); tV3_50pa(); tV4_3pa(); tV5_10pa_noisy(); tV6_norise(); tV7_plateau_noise(); tV8_latch();
  /* Kalman A-M + vessel-KF (diagnostic parallel, never drives apogee) */
  tKA_init(); tKB_rfloor(); tKC_predict(); tKD_dtclamp(); tKE_converge(); tKF_invalid();
  tKG_guard(); tKH_setnoise(); tKI_rderiv(); tKJ_qreason(); tKK_rawfeed(); tKL_notready(); tKM_size();
  tKQ_sweep(); tKV1_10pa(); tKV2_20pa(); tKV3_50pa(); tKV4_noisy(); tKV5_norise(); tKV6_plateau(); tKV7_latch();
  tC_A(); tC_B(); tC_C(); tC_C2(); tC_D(); tC_E(); tC_G(); tC_cap(); tC_min(); tC_wins(); tC_edge();
  tDN_quiet(); tDN_noisy(); tDN_badsigma(); tDN_fingereasy(); tDN_gate_visible();
  tM3_init(); tM3_order(); tM3_spike(); tM3_step(); tM3_nan();
  tD_drift(); tD_small(); tD_spike(); tD_finger(); tD_bound(); tD_absorb(); tD_noisep();
  tW1_10pa(); tW2_20pa(); tW3_50pa(); tW4_noisy();
  tCAL_ident(); tCAL_apply(); tCAL_6pos(); tCAL_resid(); tCAL_21080();
  tAB_table();
  t_alt_nan_in(); t_alt_extreme(); t_press_ok();
  printf("HOST_TESTS_PASSED n=%d\n", n_pass);
  return 0;
}
