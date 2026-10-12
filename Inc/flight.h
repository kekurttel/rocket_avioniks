/* Flight state machine + launch/apogee/free-fall/deployment logic.
 *
 * HAL-independent, deterministic, host-testable. Complements avionics.h:
 * - avionics.h keeps the verified bench math (tilt atan2, Bosch integer
 *   compensation, median3, EMA, baro relation, trend apogee, Mahony/Kalman
 *   diagnostics). Nothing there is modified by this layer.
 * - flight.h adds the REAL flight lifecycle on top: launch detection,
 *   max-altitude + drop + vertical-velocity apogee gate with a
 *   production/test altitude-minimum split, free-fall gate, one-shot
 *   drogue/main deployment, and a central latched state machine.
 *
 * Production vs test is a PARAMETER split, not a second algorithm:
 *   PRODUCTION: apogee_alt_min_m = 1250 m, main_alt_m = 600 m
 *   TEST:       apogee_alt_min_m disabled (0 m), main_alt_m overrideable
 * The drop (2 m) and velocity (-2 m/s) gates are identical in both.
 */
#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Build-time mode select (overridable via -DFLIGHT_TEST_MODE=1) ---- */
#ifndef FLIGHT_TEST_MODE
#define FLIGHT_TEST_MODE (0)
#endif

/* ---- Production thresholds (real flight, never tuned for the bench) ---- */
#define FLIGHT_APOGEE_ALT_MIN_PROD_M (1250.0f)
#define FLIGHT_MAIN_ALT_PROD_M (600.0f)

/* ---- Test defaults (SIT/SUT bench reachable; main is runtime-overridable) ---- */
#define FLIGHT_APOGEE_ALT_MIN_TEST_M (0.0f) /* disabled: max+drop+vel only */
#define FLIGHT_MAIN_ALT_TEST_M (5.0f)

/* ---- Shared flight gates (identical in PROD and TEST) ---- */
#define FLIGHT_APOGEE_CONFIRM_N 3u /* consecutive apogee samples */
#define FLIGHT_APOGEE_DROP_M (2.0f)      /* max - current >= 2 m */
#define FLIGHT_DESCENT_VEL_MPS (-2.0f)   /* vertical velocity < -2 m/s */
#define FLIGHT_VEL_DT_MIN_S (0.005f)     /* dt clamp for velocity */
#define FLIGHT_VEL_DT_MAX_S (0.500f)

/* ---- Launch / free-fall accel gates ----
 * Magnitude is compared against the startup gravity-reference magnitude
 * (LSB units), so no LSB/g assumption is baked in. Ratios + N-consecutive
 * confirmation reject single-sample spikes (existing median3/EMA cover
 * pressure; accel has the tilt EMA + these counters). */
#define FLIGHT_LAUNCH_RATIO (1.20f)      /* |a| > 1.2 * |g_ref| */
#define FLIGHT_FREEFALL_RATIO (0.80f)    /* |a| < 0.8 * |g_ref| */
#define FLIGHT_LAUNCH_CONFIRM_N (3u)
#define FLIGHT_FREEFALL_CONFIRM_N (3u)

/* ---- Sensor-fault latch ----
 * Single invalid samples never fault (apogee holds, counters reset).
 * Only a sustained run of invalid fused-altitude samples in a
 * flight-critical state latches FAULT. */
#define FLIGHT_FAULT_INVALID_N (50u)

/* ---- Explicit sensor health snapshot (A7) ----
 * present: init-time probe OK. valid: last read OK. stale: no good sample
 * for >STALE_MS (caller compares ticks). Outlier rejection lives in the
 * median3 pre-filter (shared, single point); disagreement uses
 * AVT_BMP_DISAGREE_PA (50 Pa ~= 4 m: >> quiet noise ~1-2 Pa and bench
 * drift, << absolute sensor tolerance; never a deployment gate). */
typedef enum
{
  FLIGHT_FUSED_NONE = 0,
  FLIGHT_FUSED_BMP,
  FLIGHT_FUSED_B2,
  FLIGHT_FUSED_AVG
} flight_fused_src_t;

typedef struct
{
  bool mpu_present;
  bool mpu_valid;
  bool mpu_stale;
  bool bmp_present;
  bool bmp_valid;
  bool bmp_stale;
  bool b2_present;
  bool b2_valid;
  bool b2_stale;
  bool disagree;
  flight_fused_src_t fused_src;
} flight_health_t;

flight_health_t flight_health_snapshot(bool mpu_present, bool mpu_ok,
                                       bool mpu_stale, bool bmp_present,
                                       bool b1_ok, bool bmp_stale,
                                       bool b2_present, bool b2_ok,
                                       bool b2_stale,
                                       avt_bmp_state_t bmp_state);

/* ---- Bench negative-vertical-accel trigger (display + BLUE LED only) ----
 * Gravity-referenced signed vertical channel (no fixed Z assumption):
 *   u       = g_ref / |g_ref|          (unit, board-up at cal pose)
 *   vert_g  = dot(a_filt, u) / |g_ref| (≈ +1.0 at rest, in g units)
 *   dyn_g   = vert_g - 1.0             (≈ 0 at rest, -1.0 in free-fall)
 * Sign (physics, specific force): board accelerating DOWN reduces the
 * measured specific force along board-up -> dyn NEGATIVE. Board pushed UP
 * -> dyn POSITIVE. Pure quasi-static tilt keeps |a| ~= |g_ref| while dyn
 * goes cos(tilt)-1, so the trigger ALSO requires a magnitude drop
 * (mag_ratio < BENCH_MAG_RATIO) to tell rotation apart from translation.
 * Threshold + N-consecutive confirmation; latches once confirmed.
 * Active in ALL modes (bench and production): it drives display + BLUE
 * LED + UART only and NEVER touches state/deployment, so flight safety
 * is unaffected. In SUT its LED contribution is suppressed (SUT blue
 * means synthetic apogee instead); the gate itself keeps tracking. */
#define FLIGHT_BENCH_NEG_THR_TEST_G (-0.30f)
/* N=3 (~300 ms at ~10 Hz): a genuine hand drop sustains this; isolated
 * vibration spikes cannot. Bench logs never showed accel self-fire. */
#define FLIGHT_BENCH_CONFIRM_N_TEST (3u)
#define FLIGHT_BENCH_MAG_RATIO (0.95f)
/* ---- Bench barometric fall channel (display + BLUE LED only) ----
 * Second, independent bench signal from the REAL fused altitude path
 * (BMP1/BMP2 -> compensation -> median -> combine -> EMA -> baro;
 * no new smoother: velocity is derived from the delivered altitude with
 * real elapsed dt, same helper as the flight velocity gate).
 * Trigger: sustained vertical velocity below threshold (hand-down
 * motion), N consecutive valid samples, latched. A drop-from-max display
 * flag (BARO_DROP) is tracked alongside for telemetry. Active in ALL
 * modes like the accel channel; NEITHER bench channel ever touches
 * deployment/state (display + BLUE LED + UART only). */
#define FLIGHT_BENCH_BARO_VEL_TEST_MPS (-0.1f)
#define FLIGHT_BENCH_BARO_CONFIRM_N_TEST (3u)
#define FLIGHT_BENCH_BARO_DROP_M (0.3f)
/* Motion liveness for the bench baro channel (all modes).
 * A room pressure step is barometrically IDENTICAL to a hand-down
 * (bench capture: -0.5 m at up to -4.6 m/s with the board motionless:
 * ACC +/-0.1%, VERT 2-3 mg). The only signal that separates "the board
 * fell" from "the room breathed" is the accelerometer, so the baro gate
 * is deliberately LOOSE on velocity (-0.1 m/s: even a slow hand lowering
 * passes) and position (0.3 m below its own max) and STRICT on motion:
 * the streak counts only while the caller reports recent board motion.
 * Gust rejection lives in the motion term, not in the velocity number.
 * Motion thresholds sit ABOVE hand tremor but far below real handling:
 * a board merely resting in a still hand shows <0.05 g of disturbance,
 * while any deliberate move (lift, tilt, lower, catch) exceeds 0.1 g in
 * transients (30 deg of tilt alone gives dyn = -0.13). So a gust arriving
 * while the board rests stays silent, and any intentional up-down arms the
 * gate. Latched for MOTION_TMO_MS by the caller (main.c, which owns the
 * clock; SUT bypasses with always-true).
 * The accel bench channel needs no such gate: it IS motion by
 * construction. Neither channel touches deployment/state in any mode. */
#define FLIGHT_BENCH_MOTION_DYN_G (0.10f)
#define FLIGHT_BENCH_MOTION_MAG (0.10f)
#define FLIGHT_BENCH_MOTION_TMO_MS (2000u)

/* Pure motion test: does this vert-channel sample show handling-level
 * disturbance? Used by main.c to drive bench_motion_ok; host-testable so
 * the tremor/handling boundary is pinned by tests, not by feel. */
static inline bool flight_motion_seen(float vert_dyn_g, float mag_ratio,
                                      bool valid)
{
  if (!valid)
  {
    return false;
  }
  float d = (vert_dyn_g < 0.0f) ? -vert_dyn_g : vert_dyn_g;
  float m = (mag_ratio < 1.0f) ? (1.0f - mag_ratio) : (mag_ratio - 1.0f);
  return (d > FLIGHT_BENCH_MOTION_DYN_G) || (m > FLIGHT_BENCH_MOTION_MAG);
}

/* ---- Central flight states (single-direction lifecycle) ---- */
typedef enum
{
  FLIGHT_INIT = 0,
  FLIGHT_CALIBRATION,
  FLIGHT_WAIT_FOR_LAUNCH,
  FLIGHT_ASCENT,
  FLIGHT_APOGEE_CONFIRMED,
  FLIGHT_FREE_FALL,
  FLIGHT_DROGUE_DEPLOYED,
  FLIGHT_DESCENT,
  FLIGHT_MAIN_DEPLOYED,
  FLIGHT_COMPLETE,
  FLIGHT_FAULT
} flight_state_t;

typedef struct
{
  float apogee_alt_min_m; /* PROD 1250, TEST 0 (disabled) */
  float main_alt_m;       /* PROD 600, TEST overrideable */
  float apogee_drop_m;    /* 2.0 */
  float descent_vel_mps;  /* -2.0 */
  uint8_t apogee_confirm_n; /* 3: consecutive drop&&vel samples */
  bool test_mode;
  /* Bench trigger (TEST only; production leaves bench_enable=false). */
  bool bench_enable;
  float bench_neg_thr_g;  /* TEST -0.30 */
  uint8_t bench_confirm_n; /* TEST 5 */
  float bench_baro_vel_mps; /* TEST -0.1 */
  uint8_t bench_baro_confirm_n; /* TEST 3 */
  float bench_baro_drop_m; /* TEST 0.3 (display flag) */
} flight_cfg_t;

typedef struct
{
  flight_cfg_t cfg;
  flight_state_t state;
  /* Altitude tracking */
  float max_alt_m;
  bool has_max;
  float prev_alt_m;
  bool has_prev;
  float vel_mps; /* last vertical velocity estimate */
  bool vel_valid;
  /* Apogee sub-conditions (observable for telemetry) */
  bool drop_flag;
  bool vel_flag;
  /* Debounce counters */
  uint8_t launch_n;
  uint8_t ff_n;
  uint8_t apogee_n;
  uint16_t invalid_n;
  /* Bench vertical-accel channel (diagnostic in PROD, trigger in TEST) */
  float vert_dyn_g;    /* signed dynamic vertical accel, g units (~0 rest) */
  float vert_mag_ratio; /* |a_filt| / |g_ref| (~1.0 rest) */
  bool vert_valid;
  uint8_t bench_n;     /* consecutive sub-threshold streak */
  bool bench_confirmed; /* latched TEST accel fall trigger */
  /* Bench barometric channel (independent, same TEST-only gating) */
  float bench_baro_max_m;
  float bench_baro_prev_m;
  bool bench_baro_has_prev;
  float bench_baro_vel_mps;
  bool bench_baro_vel_valid;
  bool bench_baro_drop_flag; /* alt < bench_max - drop_m (telemetry) */
  uint8_t bench_baro_n;      /* consecutive sub-threshold vel streak */
  bool bench_baro_confirmed; /* latched TEST baro fall trigger */
  /* Motion liveness, caller-owned (main.c refreshes every cycle from the
   * vert channel with a timeout latch; SUT bypasses with always-true).
   * Init TRUE (fail-open for harnesses); the live loop drives it. */
  bool bench_motion_ok;
  /* One-shot deployment latches */
  bool drogue_deployed;
  bool main_deployed;
  bool drogue_event; /* true exactly once, on the deploying step */
  bool main_event;
} flight_t;

/* Config builders (no magic numbers at call sites). */
void flight_config_production(flight_cfg_t *c);
void flight_config_test(flight_cfg_t *c, float main_alt_m);

void flight_init(flight_t *f, const flight_cfg_t *c);
void flight_set_state(flight_t *f, flight_state_t s); /* cal/launch wiring */

/* Pure helpers (all host-testable, all guard invalid inputs): */
/* Vertical velocity from real elapsed dt. Returns false on any degenerate
 * input (non-finite, dt out of (0, ..]); dt is clamped to [MIN, MAX]. */
bool flight_velocity(float cur_m, float prev_m, float dt_s, float *out_v);
/* Launch gate on one accel-magnitude sample. Counter-based debounce:
 * needs CONFIRM_N consecutive over-threshold samples. Invalid/garbage
 * input resets the streak. gref_mag <= 0 never fires. */
bool flight_launch_push(flight_t *f, float acc_mag, float gref_mag);
/* Free-fall gate, symmetric to launch (under-threshold streak). */
bool flight_freefall_push(flight_t *f, float acc_mag, float gref_mag);
/* Gravity-referenced vertical projection. Computes signed dynamic vertical
 * accel (dyn_g ~0 at rest, negative on downward accel) and the magnitude
 * ratio (|a|/|gref| ~1.0 at rest) from raw/LSB vectors. No fixed-axis
 * assumption. Returns false (no output) on degenerate input. */
bool flight_vert_dyn_g(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float *out_dyn_g, float *out_mag_ratio);
/* Bench descent trigger: sustained (dyn_g < thr AND mag_ratio < MAG_RATIO)
 * over bench_confirm_n consecutive valid samples. Returns latched
 * bench_confirmed. Disabled config (production) always returns false with
 * zero side effects. Invalid input resets the streak, never confirms,
 * never un-latches. Tilt-only rotations (|a| ~= |gref|) can never confirm. */
bool flight_bench_push(flight_t *f, float vert_dyn_g, float mag_ratio,
                       bool valid);
/* Bench barometric fall trigger: sustained vertical velocity (from the
 * delivered fused altitude, real dt) below threshold over
 * bench_baro_confirm_n consecutive valid samples. Returns latched
 * bench_baro_confirmed. Same TEST-only gating and invalid-safety as the
 * accel channel. Tracks its own max for the BARO_DROP display flag. */
bool flight_bench_baro_push(flight_t *f, float alt_m, bool valid, float dt_s);
/* Combined bench fall signal = BENCH_ACCEL_FALL OR BENCH_BARO_FALL.
 * TEST-only (false whenever bench_enable=false). Display + BLUE LED +
 * UART source. NEVER drives state/deployment (decoupled by design:
 * BENCH_FALL_DETECTED != DROGUE_DEPLOY_EVENT). NULL-safe. */
bool flight_bench_fall(const flight_t *f);
/* Dual-pressure combine: average when both valid, single when one valid,
 * false when none. Never crashes on disagreement; health is reported
 * separately via avt_bmp_state by the caller. */
bool flight_combine_pressure(bool ok1, bool ok2, float p1_pa, float p2_pa,
                             float *out_pa);
/* Altitude sample: tracks max, computes velocity (real dt), evaluates
 * drop + velocity flags. Only advances ASCENT -> APOGEE_CONFIRMED here;
 * every other transition is handled by flight_step(). Invalid samples
 * reset streak state but never un-latch. */
void flight_alt_push(flight_t *f, float alt_m, bool valid, float dt_s);

/* One-shot deployment guards. Return true exactly once (the firing call),
 * only from the correct state, only when sensors are usable. */
bool flight_try_drogue(flight_t *f, bool sensors_usable);
bool flight_try_main(flight_t *f, bool sensors_usable);

/* Central step: advances the latched single-direction lifecycle.
 * Inputs are the already-computed per-domain confirmations:
 *   launch_confirmed: flight_launch_push latched (WAIT_FOR_LAUNCH -> ASCENT)
 *   ff_confirmed:     flight_freefall_push latched (APOGEE_CONFIRMED -> FREE_FALL)
 * Bench channels (accel/baro) are DISPLAY + BLUE LED + UART ONLY: they are
 * deliberately NOT consulted here, so BENCH_FALL_DETECTED can never advance
 * state or fire deployment (BENCH_FALL_DETECTED != DROGUE_DEPLOY_EVENT).
 *   alt_m/valid/dt:   fused altitude path (apogee gate inside ASCENT)
 *   sensors_usable:   altitude computable (avt_alt_usable equivalent)
 * Deployment firing is attempted inside DROGUE_DEPLOYED/DESCENT-adjacent
 * transitions via the one-shot guards above; edge flags (drogue_event /
 * main_event) are set exactly on the firing step for the caller to emit. */
flight_state_t flight_step(flight_t *f, bool launch_confirmed,
                           bool ff_confirmed, float alt_m, bool alt_valid,
                           float dt_s, bool sensors_usable);

const char *flight_state_str(flight_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_H */
