/* Avionics demo math: HAL-independent, deterministic, testable.
 *
 * All thresholds live here (single place). Contracts:
 * - tilt functions take raw integer sensor vectors (LSB) as floats.
 * - altitude uses the barometric relation against a ground reference.
 * - redundancy maps two-sensor health to one state enum.
 */
#ifndef AVIONICS_H
#define AVIONICS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Tilt band: TILT_30_45 is true for 30..45 deg inclusive. */
#define AVT_TILT_LO_DEG (30.0f)
#define AVT_TILT_HI_DEG (45.0f)

/* |accel| may deviate this much (relative) from |gravity ref|
 * before the tilt result is declared invalid (high dynamics). */
#define AVT_TILT_MAG_TOL (0.25f)

/* Relative altitude rise: DEMO ONLY thresholds (bench-top finger demo,
 * NOT flight thresholds). Hysteresis against pressure-noise LED flicker:
 * ON at/above +0.20 m, OFF at/below +0.10 m, hold state between. */
/* Relative altitude drop: finger-press indicator (pressure rise).
 * Mirror of ALT_RISE, slightly more sensitive: ON at/below -0.15 m
 * (~+1.8 Pa), OFF at/above -0.07 m, hold between. ponytail: single
 * pair covers finger test; split tuning when flight data exists. */
#define AVT_ALT_ON_M (0.20f)
#define AVT_ALT_OFF_M (0.10f)
#define AVT_ALT_DROP_ON_M (-0.15f)
#define AVT_ALT_DROP_OFF_M (-0.07f)

/* |P1-P2| above this (Pa) is sensor disagreement. */
#define AVT_BMP_DISAGREE_PA (50.0f)

typedef enum
{
  AVT_BOTH_OK = 0,
  AVT_BMP1_ONLY,
  AVT_BMP2_ONLY,
  AVT_BOTH_FAILED,
  AVT_DISAGREE
} avt_bmp_state_t;

/* Angle (deg) between calibrated accel vector and gravity reference.
 * Yaw-free by construction (full 3D angle). Returns 0 when degenerate. */
float avt_tilt_deg(float ax, float ay, float az,
                   float gx, float gy, float gz);

/* True when |accel| is close enough to |gravity ref| to trust the angle. */
bool avt_tilt_mag_ok(float ax, float ay, float az,
                     float gx, float gy, float gz);

/* Vector magnitude, e.g. for TILTDBG diagnostics. */
float avt_vec_mag(float x, float y, float z);

/* Inclusive band check, e.g. 30 -> true, 45 -> true, 29/46 -> false. */
bool avt_in_band(float v, float lo, float hi);

/* Relative altitude (m) from barometric relation. Positive when P < P0.
 * Returns false (no output) when inputs are non-positive.
 * Gecerlilik disiplini (400000000 m olayi): NaN/inf giris, NaN/inf sonuc
 * veya |h| > AVT_ALT_LIMIT_M her zaman false doner ve *out_m'ye YAZMAZ.
 * Gecerli aralik disina tasan hicbir deger "gecerli" cikamaz; gecerli
 * degerlerin urettigi sayilar bit-bit aynidir (esik degisikligi yok). */
bool avt_altitude_m(float p_pa, float p0_pa, float *out_m);

/* Fiziksel basinc zarfi (Pa): BMP280 olcum araligi + tum gercek ucus
 * profili (1250 m uretim apojesi ~87 kPa) + REPLAY araligi (10..200 kPa)
 * icinde kalir. Disi: kopuk/kilitlenmis I2C hattinin urettigi sacma
 * telafi degerleri (sifir-veri, takilmis ham bayt). Gecerli basinci
 * degistirmez; sadece gecersiz isaretlemeyi guclendirir. */
#define AVT_PRESS_MIN_PA (10000.0f)
#define AVT_PRESS_MAX_PA (200000.0f)

/* Fuzyon irtifa tavani (m): |h| bunun ustundeyse fiziksel olarak imkansiz
 * kabul edilir ve gecersiz isaretlenir. Gercek ucus (<= birkac km) ve
 * tum bench/vessel profilleri etkilenmez. */
#define AVT_ALT_LIMIT_M (100000.0f)

/* Basinc gecerliligi: sonlu + pozitif + fiziksel zarf icinde. NACK sonrasi
 * okunmamis/cop degerleri ve sifir-veri burada elenir; mevcut tek-sensor
 * fallback'u korunur (combine ayni kuralla tek sagkalana duser). */
bool avt_press_ok(float p_pa);

/* Two-sensor redundancy state. p1/p2 only read when the matching ok flag
 * is true. */
avt_bmp_state_t avt_bmp_state(bool ok1, bool ok2, float p1_pa, float p2_pa);

/* Console/LED helpers over the state enum. */
bool avt_is_degraded(avt_bmp_state_t s); /* yellow LED on */
bool avt_alt_usable(avt_bmp_state_t s);  /* altitude computable */

/* Hysteresis latch for ALT_RISE. Invalid altitude forces false.
 * +0.15 (prev false) -> false, +0.20/+0.30 -> true,
 * active +0.15 -> true (hold), active +0.05 -> false. */
bool avt_alt_rise(bool prev, float alt_m, bool valid);

/* Hysteresis latch for ALT_DROP (finger-press: pressure rise).
 * Mirror of ALT_RISE, negative side. Invalid forces false.
 * -0.07 (prev false) -> false, -0.15/-0.30 -> true,
 * active -0.07 -> true (hold), active -0.03 -> false. */
bool avt_alt_drop(bool prev, float alt_m, bool valid);

/* Apogee/descent detector (rocket logic: trigger on the way DOWN, not up).
 *
 * Natural flight profile gate:
 *   1. After calibration, a genuine ascent trend must be seen first.
 *      A single sample is never enough: N=3 consecutive positive
 *      per-sample steps (each >= ascent step) ARM the detector.
 *   2. While ARMED, the peak (alt_max) is tracked.
 *   3. Descent needs a consistent negative trend: N=3 consecutive
 *      negative per-sample steps (each <= -descent step) past the apex.
 *   4. Single noisy samples never fire; invalid samples reset the
 *      in-progress streak counters but keep latched state.
 *   5. Once DESCENT, the phase latches and never un-fires.
 *
 * Thresholds are noise-relative: the calibration noise floor (sigma_alt_m,
 * measured by the caller during the startup cal window and handed in via
 * avt_apogee_set_noise()) scales the per-step thresholds, with hard minimum
 * floors so the detector never becomes hypersensitive on unusually quiet
 * calibrations. Defaults keep legacy behaviour until noise is provided.
 *
 * Legacy absolute guards are kept for real-flight semantics: ARM still
 * requires a minimum absolute height, and DROP still requires a minimum
 * absolute fall from the peak, so tiny wiggles alone can never trigger. */
#define AVT_APOGEE_ARM_M (0.5f)
#define AVT_APOGEE_DROP_M (0.3f)
#define AVT_APOGEE_CONFIRM_N (3U)

/* Trend-detection tuning: consecutive steps needed for ascent/descent. */
#define AVT_APOGEE_TREND_N (3U)
/* Step floors (m per sample @ ~20 Hz): ascent/drop must each exceed these
 * even when the calibration noise floor is near zero, so the detector
 * never becomes hypersensitive on unusually quiet calibrations. */
#define AVT_APOGEE_ASC_STEP_MIN_M (0.02f)
#define AVT_APOGEE_DSC_STEP_MIN_M (0.02f)
/* Noise scaling: step threshold = max(floor, k * sigma_alt).
 * k=2 per step with N=3 consecutive steps gives a per-gate false-alarm
 * probability ~1e-5 under Gaussian noise, while staying sensitive to
 * small but consistent bench-top trends. */
#define AVT_APOGEE_NOISE_K (2.0f)
/* Absolute drop guard is noise-relative too: drop_need =
 *   min(AVT_APOGEE_DROP_M, max(AVT_APOGEE_DROP_MIN_M, 4*sigma_alt)).
 * Quiet bench -> small consistent dips past the apex can fire; noisy
 * calibration -> guard saturates at the legacy 0.3 m. Never a tiny
 * fixed trip-wire: the floor (0.10 m) is itself >> single-sample noise. */
#define AVT_APOGEE_DROP_MIN_M (0.10f)
#define AVT_APOGEE_DROP_NOISE_K (4.0f)
/* Default sigma (m) before the caller reports calibration noise.
 * Keeps a sane gate for host tests that skip set_noise(). */
#define AVT_APOGEE_SIGMA_DEFAULT_M (0.05f)

typedef enum
{
  AVT_PHASE_ASCENT = 0,
  AVT_PHASE_ARMED = 1,
  AVT_PHASE_DESCENT = 2
} avt_phase_t;

typedef struct
{
  avt_phase_t phase;
  float alt_max; /* tracked only while armed (spikes can't poison it) */
  uint8_t drop_n;
  uint8_t asc_n;   /* consecutive ascent steps */
  uint8_t dsc_n;   /* consecutive descent steps past apex */
  float prev_alt;  /* previous valid sample for step computation */
  bool has_prev;
  bool has_max;
  bool armed;
  /* Calibration noise floor (1-sigma of relative altitude, metres).
   * Set once after calibration via avt_apogee_set_noise(). */
  float sigma_alt_m;
  bool has_sigma;
} avt_apogee_t;

void avt_apogee_init(avt_apogee_t *s);

/* Report the calibration noise floor (altitude sigma, metres) measured
 * during the startup calibration window. Must be called after
 * avt_apogee_init() and before flight samples; ignored during DESCENT. */
void avt_apogee_set_noise(avt_apogee_t *s, float sigma_alt_m);

/* Feed relative altitude each loop (valid=false on sensor fault: counter
 * resets, phase holds). Returns current phase. Not called during cal. */
avt_phase_t avt_apogee_update(avt_apogee_t *s, float alt_m, bool valid);

/* Pressure EMA weight (alpha = 0.5f).
 * Fast response for bench-top testing while smoothing high-frequency noise.
 * Variance gain = alpha/(2-alpha) = 1/3 (std x0.58); a single-sample spike
 * of amplitude A still passes at A/2, so spike rejection is NOT its job. */
#define AVT_PRESS_EMA_ALPHA (0.5f)

/* One first-order low-pass step: prev + (x - prev) * ALPHA. */
float avt_ema(float prev, float x);

/* Adaptive startup-drift correction (Pa), computed from the calibration
 * data itself (main.c tracks early/late window means) and added ONCE to the
 * calibration mean. NO fixed offset is ever applied: with no external
 * absolute reference, P0 = "most stable baseline at the end of startup".
 *
 * Model: drift = late_center - early_center over the calibration window.
 * - Significance deadband: |drift| < max(2*SE, MIN_DB) -> correction 0,
 *   with SE = sigma_p*sqrt(1/ne+1/nl) (standard error of the mean
 *   difference) and MIN_DB = 0.5 Pa (sensor resolution floor: chasing
 *   sub-0.5 Pa means baking noise into P0). Stable/noise-only/spike-
 *   diluted calibrations therefore correct ~0.
 * - Else correction = clamp(0.5*drift, +/-CAP). Gain 0.5 is exact for
 *   linear drift: window mean ~= midpoint, late ~= midpoint+d/2, so mean +
 *   d/2 ~= late (the stable end). CAP = 5 Pa: genuine startup settle
 *   (thermal equalisation, still air, untouched board) is a few Pa; beyond
 *   that the calibration was disturbed (motion/doors/gusts) and a larger
 *   correction would bake the disturbance into P0 permanently.
 * Never touches calibration samples, Welford M2/sigma, or Bosch raw. */
/* Startup calibration length (ms). Windows below derive from it, so a
 * duration change never silently desynchronises the drift estimator. */
#define AVT_CAL_MS (16000u)
#define AVT_P0C_EARLY_MS (AVT_CAL_MS / 4u)
#define AVT_P0C_LATE_MS ((AVT_CAL_MS * 3u) / 4u)
/* Noise-relative ALT_DROP thresholds (finger indicator, yellow LED).
 * Fixed 1.8 Pa-style tripwires cannot balance "easy finger" against
 * "no self-fire" across runs, and any fixed P0 bias big enough to matter
 * pre-satisfies the apogee ARM guard (measured: 2-3 Pa lifts let a 5 Pa
 * puff fire the apogee). So the DROP gate adapts to the run's own
 * calibration noise instead:
 *   ON  = -(max(DROP_MIN, DROP_K * sigma_alt))   (capped at DROP_MAX)
 *   OFF = ON / 2  (same hysteresis ratio as the legacy -0.15/-0.07)
 * Quiet room (sigma 0.02 m): ON -0.10 m (~1.2 Pa) -- a light finger
 * press fires easily. Noisy cal (sigma 0.15 m): ON -0.50 m (capped) --
 * ambient wiggles cannot self-fire. K=4 and floor 0.10 mirror the proven
 * apogee drop-guard tuning (AVT_APOGEE_DROP_NOISE_K/MIN), so no new magic
 * numbers; the cap keeps a disturbed calibration from muting the finger. */
#define AVT_DROP_NOISE_K (4.0f)
#define AVT_DROP_MIN_M (0.10f)
#define AVT_DROP_MAX_M (0.50f)
#define AVT_DROP_SIGMA_DEFAULT_M (0.05f)

/* Hysteresis latch like avt_alt_drop but with noise-relative thresholds
 * derived from sigma_alt_m each call (cheap, ~20 Hz). Invalid sigma
 * falls back to the default; invalid sample forces false. */
bool avt_alt_drop_n(bool prev, float alt_m, bool valid, float sigma_alt_m);

/* Effective ON threshold (metres) that avt_alt_drop_n would use for this
 * noise estimate: clamp(AVT_DROP_NOISE_K * sigma, MIN, MAX), with an invalid
 * sigma falling back to AVT_DROP_SIGMA_DEFAULT_M. Single source of truth for
 * the clamp chain so telemetry cannot report a different number than the
 * gate actually uses. */
float avt_drop_gate_m(float sigma_alt_m);

/* Debounce counter shared by every N-consecutive trigger (apogee drop guard,
 * finger gate, launch, free-fall). Increments with saturation at 255 and
 * resets to 0 on any non-hit, so the caller only supplies "did this sample
 * satisfy the condition". */
uint8_t avt_streak_push(uint8_t n, bool hit);

/* Slow-baseline weight for the finger (DROP) indicator reference.
 * The DROP gate compares (alt - ref), where ref follows live altitude
 * with this gain per valid sample (~20 Hz): alpha 0.002 -> ~25 s time
 * constant. A finger press (seconds) outruns ref and fires immediately;
 * ambient drift (minutes) is absorbed into ref and never fires; after
 * release the gate relaxes by itself instead of latching on a drifted
 * rest. Apogee/Kalman/absolute ALT never use ref (flight logic untouched).
 * One float of state, no buffer, no heap. */
#define AVT_DROP_REF_ALPHA (0.002f)
#define AVT_P0C_GAIN (0.5f)
#define AVT_P0C_CAP_PA (5.0f)
#define AVT_P0C_MIN_DB_PA (0.5f)
#define AVT_P0C_K_SIGMA (2.0f)

/* Signed startup correction in Pa. Returns 0 on any degenerate input
 * (n==0, non-finite). Pure function over calibration statistics. */
float avt_p0_correct(float early_mean, float late_mean, float sigma_p,
                     uint32_t n_early, uint32_t n_late);

/* 3-sample median pre-filter for compensated pressure (spike rejection).
 * Kills any isolated single-sample outlier entirely; step delay is exactly
 * 1 sample (50 ms @20 Hz) << apogee N=3 gate. Nonlinear selector, NOT a
 * moving average: stacking it in front of the EMA is pre-filter + smoother,
 * not double averaging. Placed right after Bosch compensation, before the
 * EMA/altitude/Kalman split, so ALL branches share one rejection point.
 * Fixed 3-float buffer, no heap, no dependency. Until 3 samples are seen
 * the latest input passes through (no fake averaging of a partial window). */
#define AVT_MED3_N (3u)

typedef struct
{
  float b[AVT_MED3_N];
  uint8_t n; /* samples seen, saturates at 3 */
  uint8_t i; /* next write index (ring) */
} avt_med3_t;

void avt_med3_init(avt_med3_t *m);
/* Push x, return filtered pressure. Returns x while n < 3. */
float avt_med3_push(avt_med3_t *m, float x);

/* ---- MPU6050 bias/scale calibration infrastructure (A5) ----
 * Additive layer OVER the existing g_ref mechanism (which is untouched):
 * identity default => production behavior bit-identical until a physical
 * 6-orientation calibration is performed (NOT EXECUTED without hands).
 * Units are sensor LSB throughout; G = 1 g reference in LSB (measured). */
typedef struct
{
  float bx, by, bz; /* bias (LSB), subtracted first */
  float sx, sy, sz; /* scale (dimensionless), applied second */
} avt_mpu_cal_t;

void avt_mpu_cal_identity(avt_mpu_cal_t *c);
/* out = (raw - bias) * scale per axis. NULL-safe, identity on bad input. */
void avt_mpu_cal_apply(const avt_mpu_cal_t *c, float rx, float ry, float rz,
                       float *ox, float *oy, float *oz);
/* 6-orientation fit: bias = (pos+neg)/2, scale = 2*G/(pos-neg) per axis.
 * G = measured 1 g magnitude (LSB). Returns false on degenerate input. */
bool avt_mpu_cal_from_6pos(float px, float nx, float py, float ny,
                           float pz, float nz, float G, avt_mpu_cal_t *out);
/* Residual check: |corrected| magnitude vs G within tol (relative). */
bool avt_mpu_cal_residual_ok(const avt_mpu_cal_t *c, float rx, float ry,
                             float rz, float G, float tol);

/* ---- UART pressure replay (TEST/DEBUG ONLY) ----
 * Thin source override for chamber/bench tests over UART. Default OFF:
 * avt_replay_init() leaves active=false, so numeric UART lines never touch
 * the sensor chain unless the user sends REPLAY START first. After
 * REPLAY STOP the caller must resume the live BMP280 path immediately.
 * Power-cycle ends replay (plain RAM state, no persistence).
 *
 * Wire-up contract (caller, e.g. main.c):
 * - feed every RX byte via avt_replay_put(); on true, handle the message.
 * - SAMPLE must always flow pressure -> avt_ema -> avt_altitude_m ->
 *   avt_apogee_update (never straight to LED logic).
 * TR note: ornek her zaman basinc -> EMA -> irtifa -> apogee'e baglanmaz).
 * - START/STOP mutate active; RESET/STATUS mutate nothing (caller resets
 *   its own apogee/EMA state and prints).
 * - Numeric sample range 10..200 kPa; replay P0 override 80..110 kPa.
 * - Invalid lines yield AVT_RPL_ERR (or EMPTY for blank), never a crash. */
#define AVT_REPLAY_LINE_MAX (40u)
#define AVT_REPLAY_MIN_P_PA (10000.0f)
#define AVT_REPLAY_MAX_P_PA (200000.0f)
#define AVT_REPLAY_MIN_P0_PA (80000.0f)
#define AVT_REPLAY_MAX_P0_PA (110000.0f)

typedef enum
{
  AVT_RPL_EMPTY = 0,
  AVT_RPL_SAMPLE,
  AVT_RPL_START,
  AVT_RPL_STOP,
  AVT_RPL_RESET,
  AVT_RPL_P0,
  AVT_RPL_STATUS,
  AVT_RPL_ERR
} avt_rpl_kind_t;

typedef struct
{
  avt_rpl_kind_t kind;
  float value; /* SAMPLE pressure Pa, or P0 value for AVT_RPL_P0 */
} avt_rpl_msg_t;

typedef struct
{
  bool active;  /* false after init / STOP / power-cycle */
  bool has_p0;  /* replay-only baseline override */
  float p0;
  char buf[AVT_REPLAY_LINE_MAX];
  uint8_t len;
  bool ovf; /* line exceeded buffer: discard to newline, report ERR */
} avt_replay_t;

void avt_replay_init(avt_replay_t *r);
/* Feed one UART byte. Returns true once per completed '\n' line with *m
 * filled; '\r' is ignored. Over-long lines are discarded to newline. */
bool avt_replay_put(avt_replay_t *r, char c, avt_rpl_msg_t *m);

/* ---- Parallel Kalman altitude (DIAGNOSTIC ONLY) ----
 * Constant-velocity model over relative altitude:
 *   x = [h, v], h += v*dt, v constant, z = raw (unfiltered) relative
 *   altitude from avt_altitude_m(), H = [1, 0].
 * Scalar 2x2 implementation, float, no heap, no matrix lib.
 *
 * Input contract (caller, e.g. main.c):
 * - Bosch raw pressure -> P0 relative -> avt_altitude_m -> z.
 * - EMA output NEVER feeds the filter (double smoothing forbidden).
 * - ALT/EMA/apogee path is untouched; KALT runs in parallel and never
 *   drives avt_apogee_update in this phase (no Kalman trigger).
 * - dt is real elapsed seconds, clamped to [DT_MIN, DT_MAX].
 * - R comes from calibration (sigma_p/12); floor enforced, zero banned.
 * - Q is chosen by host-test sweep (see tests/host), set once via
 *   avt_kf_set_noise(). Invalid z -> no update; predict stays clamped
 *   with drift guards (|v|<=V_MAX, P<=P_CAP). */
#define AVT_KF_DT_MIN_S (0.005f)
#define AVT_KF_DT_MAX_S (0.200f)
#define AVT_KF_R_FLOOR (1e-4f)
#define AVT_KF_P_CAP (10.0f)
#define AVT_KF_V_MAX (30.0f)

typedef struct
{
  float h;          /* relative altitude estimate (m) */
  float v;          /* vertical speed estimate (m/s) */
  float p00, p01;   /* covariance P = [[p00,p01],[p01,p11]] */
  float p11;
  float R;          /* measurement variance (m^2), floored */
  float Q;          /* velocity random-walk PSD (m^2/s^3) */
  bool ready;       /* true after avt_kf_init() */
} avt_kf_t;

/* Init: h = first relative altitude, v = 0, P = R logic
 * (p00 = R, p11 = R, p01 = 0). R below floor -> floor. */
void avt_kf_init(avt_kf_t *k, float h0, float R);
/* Set noise: R floored, Q must be finite and > 0 else ignored.
 * NULL-safe. Never touches h/v/P. */
void avt_kf_set_noise(avt_kf_t *k, float R, float Q);
/* Time update with clamped dt. No-op when !ready or k == NULL.
 * Non-finite or non-positive dt is clamped/skipped in a controlled way:
 * dt<=0 or NaN -> no predict step (state held). */
void avt_kf_predict(avt_kf_t *k, float dt);
/* Measurement update. Returns false (no update) when !ready, !valid,
 * or z non-finite. Returns true after a fused update. */
bool avt_kf_update(avt_kf_t *k, float z, bool valid);

#endif /* AVIONICS_H */
