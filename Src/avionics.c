/* Avionics demo math. See avionics.h. */
#include "avionics.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

float avt_tilt_deg(float ax, float ay, float az,
                   float gx, float gy, float gz)
{
  /* angle = atan2(|a x g|, a . g): same physics as acos(dot/|a||g|)
   * (0 = flat reference pose, 90 = perpendicular, 180 = opposite)
   * but numerically stable near 0 deg where acos flattens out. */
  float dot = ax * gx + ay * gy + az * gz;
  float cx = ay * gz - az * gy;
  float cy = az * gx - ax * gz;
  float cz = ax * gy - ay * gx;
  float cross = sqrtf(cx * cx + cy * cy + cz * cz);
  if (dot == 0.0f && cross == 0.0f)
  {
    return 0.0f;
  }
  return atan2f(cross, dot) * 57.29578f; /* rad -> deg */
}

bool avt_tilt_mag_ok(float ax, float ay, float az,
                     float gx, float gy, float gz)
{
  float ma = sqrtf(ax * ax + ay * ay + az * az);
  float mg = sqrtf(gx * gx + gy * gy + gz * gz);
  if (mg <= 0.0f)
  {
    return false;
  }
  float dev = (ma > mg) ? (ma - mg) : (mg - ma);
  return (dev / mg) <= AVT_TILT_MAG_TOL;
}

bool avt_in_band(float v, float lo, float hi)
{
  return (v >= lo) && (v <= hi);
}

float avt_vec_mag(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

bool avt_altitude_m(float p_pa, float p0_pa, float *out_m)
{
  if (out_m == NULL)
  {
    return false;
  }
  /* DELIK NOTU (bulgu, dosya:satir): eski kosul `p_pa <= 0.0f` NaN'i
   * yakalayamazdi (NaN <= 0.0f her zaman false), boylece NaN basinc
   * powf'e girip NaN irtifa uretiyor ve fonksiyon true donuyordu.
   * `!(x > 0.0f)` formu NaN-guvenlidir (NaN > 0 false -> kosul true). */
  if (!isfinite(p_pa) || !isfinite(p0_pa))
  {
    return false; /* NaN/inf giris: cikisa YAZMA */
  }
  if (!(p_pa > 0.0f) || !(p0_pa > 0.0f))
  {
    return false;
  }
  /* Barometric relation, ground reference (not sea level):
   * h = 44330 * (1 - (P/P0)^(1/5.255)). P<P0 -> positive. */
  float r = p_pa / p0_pa;
  if (!isfinite(r) || !(r > 0.0f))
  {
    return false; /* tasma/bolme copu: cikisa YAZMA */
  }
  float h = 44330.0f * (1.0f - powf(r, 1.0f / 5.255f));
  if (!isfinite(h))
  {
    return false; /* NaN/inf sonuc: cikisa YAZMA */
  }
  if (h > AVT_ALT_LIMIT_M || h < -AVT_ALT_LIMIT_M)
  {
    return false; /* asiri deger (orn. 400000000 m): cikisa YAZMA */
  }
  *out_m = h;
  return true;
}

/* Basinc gecerliligi: sonlu + fiziksel zarf. NaN/inf/sifir/zarf-disi
 * her zaman false; gecerli basincin degeri degismez. */
bool avt_press_ok(float p_pa)
{
  if (!isfinite(p_pa))
  {
    return false;
  }
  return (p_pa >= AVT_PRESS_MIN_PA) && (p_pa <= AVT_PRESS_MAX_PA);
}

avt_bmp_state_t avt_bmp_state(bool ok1, bool ok2, float p1_pa, float p2_pa)
{
  if (ok1 && ok2)
  {
    float d = (p1_pa > p2_pa) ? (p1_pa - p2_pa) : (p2_pa - p1_pa);
    return (d > AVT_BMP_DISAGREE_PA) ? AVT_DISAGREE : AVT_BOTH_OK;
  }
  if (ok1)
  {
    return AVT_BMP1_ONLY;
  }
  if (ok2)
  {
    return AVT_BMP2_ONLY;
  }
  return AVT_BOTH_FAILED;
}

bool avt_is_degraded(avt_bmp_state_t s)
{
  return s != AVT_BOTH_OK;
}

bool avt_alt_usable(avt_bmp_state_t s)
{
  return s != AVT_BOTH_FAILED;
}

bool avt_alt_rise(bool prev, float alt_m, bool valid)
{
  if (!valid)
  {
    return false;
  }
  if (alt_m >= AVT_ALT_ON_M)
  {
    return true;
  }
  if (alt_m <= AVT_ALT_OFF_M)
  {
    return false;
  }
  return prev;
}

bool avt_alt_drop(bool prev, float alt_m, bool valid)
{
  if (!valid)
  {
    return false;
  }
  if (alt_m <= AVT_ALT_DROP_ON_M)
  {
    return true;
  }
  if (alt_m >= AVT_ALT_DROP_OFF_M)
  {
    return false;
  }
  return prev;
}

/* Effective ON threshold of the DROP gate for a given noise estimate.
 * Exposed so the caller can display it and so the TUI/telemetry show the
 * real number instead of a guessed one. Same clamp chain as
 * avt_alt_drop_n; kept in one place so the two can never diverge. */
/* Consecutive-hit counter with saturation. Shared by the apogee drop guard
 * and the finger (DROP) gate so both use the identical debounce rule:
 * a single noisy sample can never arm a trigger. Returns the new streak. */
uint8_t avt_streak_push(uint8_t n, bool hit)
{
  if (hit)
  {
    return (n < 255u) ? (uint8_t)(n + 1u) : (uint8_t)255u;
  }
  return 0u;
}

float avt_drop_gate_m(float sigma_alt_m)
{
  float sigma = sigma_alt_m;
  if (!(sigma >= 0.0f) || !isfinite(sigma))
  {
    sigma = AVT_DROP_SIGMA_DEFAULT_M;
  }
  float on = AVT_DROP_NOISE_K * sigma;
  if (on < AVT_DROP_MIN_M)
  {
    on = AVT_DROP_MIN_M;
  }
  if (on > AVT_DROP_MAX_M)
  {
    on = AVT_DROP_MAX_M;
  }
  return on;
}

bool avt_alt_drop_n(bool prev, float alt_m, bool valid, float sigma_alt_m)
{
  if (!valid)
  {
    return false;
  }
  float on = avt_drop_gate_m(sigma_alt_m);
  float off = on * 0.5f;
  if (alt_m <= -on)
  {
    return true;
  }
  if (alt_m >= -off)
  {
    return false;
  }
  return prev;
}

float avt_ema(float prev, float x)
{
  return prev + (x - prev) * AVT_PRESS_EMA_ALPHA;
}

float avt_p0_correct(float early_mean, float late_mean, float sigma_p,
                     uint32_t n_early, uint32_t n_late)
{
  if (!isfinite(early_mean) || !isfinite(late_mean))
  {
    return 0.0f;
  }
  if (n_early == 0u || n_late == 0u)
  {
    return 0.0f;
  }
  float drift = late_mean - early_mean;
  if (!isfinite(drift))
  {
    return 0.0f;
  }
  /* Standard error of the (late-early) mean difference. Invalid sigma
   * degrades gracefully to the resolution floor via fmaxf. */
  float se = sigma_p * sqrtf(1.0f / (float)n_early + 1.0f / (float)n_late);
  float db = AVT_P0C_K_SIGMA * se;
  if (!(db >= 0.0f) || !isfinite(db) || db < AVT_P0C_MIN_DB_PA)
  {
    db = AVT_P0C_MIN_DB_PA;
  }
  if (fabsf(drift) < db)
  {
    return 0.0f; /* stable / noise-only / spike-diluted: no correction */
  }
  float corr = AVT_P0C_GAIN * drift;
  if (corr > AVT_P0C_CAP_PA)
  {
    corr = AVT_P0C_CAP_PA;
  }
  else if (corr < -AVT_P0C_CAP_PA)
  {
    corr = -AVT_P0C_CAP_PA;
  }
  return corr;
}

void avt_med3_init(avt_med3_t *m)
{
  if (m == NULL)
  {
    return;
  }
  m->b[0] = 0.0f;
  m->b[1] = 0.0f;
  m->b[2] = 0.0f;
  m->n = 0;
  m->i = 0;
}

float avt_med3_push(avt_med3_t *m, float x)
{
  if (m == NULL || !isfinite(x))
  {
    return x;
  }
  m->b[m->i] = x;
  m->i = (uint8_t)((m->i + 1u) % AVT_MED3_N);
  if (m->n < AVT_MED3_N)
  {
    m->n++;
    if (m->n < AVT_MED3_N)
    {
      return x; /* partial window: pass through, never average */
    }
  }
  /* Median of 3 via 3-compare sort network on a local copy. */
  float a = m->b[0], b = m->b[1], c = m->b[2];
  if (a > b)
  {
    float t = a;
    a = b;
    b = t;
  }
  if (b > c)
  {
    float t = b;
    b = c;
    c = t;
  }
  if (a > b)
  {
    float t = a;
    a = b;
    b = t;
  }
  return b;
}

void avt_apogee_init(avt_apogee_t *s)
{
  s->phase = AVT_PHASE_ASCENT;
  s->alt_max = 0.0f;
  s->drop_n = 0;
  s->asc_n = 0;
  s->dsc_n = 0;
  s->prev_alt = 0.0f;
  s->has_prev = false;
  s->has_max = false;
  s->armed = false;
  s->sigma_alt_m = AVT_APOGEE_SIGMA_DEFAULT_M;
  s->has_sigma = false;
}

void avt_apogee_set_noise(avt_apogee_t *s, float sigma_alt_m)
{
  if (s == NULL)
  {
    return;
  }
  if (s->phase == AVT_PHASE_DESCENT)
  {
    return; /* latched: never re-tune after firing */
  }
  if (!(sigma_alt_m >= 0.0f) || !isfinite(sigma_alt_m))
  {
    return;
  }
  s->sigma_alt_m = sigma_alt_m;
  s->has_sigma = true;
}

/* Noise-relative per-sample step gate: max(floor, k * sigma). */
static float avt_apogee_step_thr(float floor_m, float sigma_m)
{
  float scaled = AVT_APOGEE_NOISE_K * sigma_m;
  return (scaled > floor_m) ? scaled : floor_m;
}

avt_phase_t avt_apogee_update(avt_apogee_t *s, float alt_m, bool valid)
{
  if (s->phase == AVT_PHASE_DESCENT)
  {
    return AVT_PHASE_DESCENT; /* latched: never un-fires */
  }
  if (!valid || !isfinite(alt_m))
  {
    /* Invalid sample: break in-progress streaks, keep latched/armed state. */
    s->drop_n = 0;
    s->asc_n = 0;
    s->dsc_n = 0;
    return s->phase;
  }

  float sigma = s->has_sigma ? s->sigma_alt_m : AVT_APOGEE_SIGMA_DEFAULT_M;
  if (!(sigma >= 0.0f) || !isfinite(sigma))
  {
    sigma = AVT_APOGEE_SIGMA_DEFAULT_M;
  }
  float asc_step = avt_apogee_step_thr(AVT_APOGEE_ASC_STEP_MIN_M, sigma);
  float dsc_step = avt_apogee_step_thr(AVT_APOGEE_DSC_STEP_MIN_M, sigma);

  /* Per-sample altitude delta (first valid sample only seeds the tracker). */
  float step = 0.0f;
  bool have_step = s->has_prev;
  if (s->has_prev)
  {
    step = alt_m - s->prev_alt;
  }
  s->prev_alt = alt_m;
  s->has_prev = true;

  if (!s->armed)
  {
    /* --- ASCENT TREND GATE -------------------------------------------
     * Need TREND_N consecutive positive steps, each >= asc_step, AND the
     * legacy absolute height guard. No ascent trend -> never armed, so
     * pressure wiggles or pure drops without a climb can never trigger. */
    if (have_step && step >= asc_step - 1e-4f)
    {
      if (s->asc_n < 255U)
      {
        s->asc_n++;
      }
    }
    else if (have_step)
    {
      s->asc_n = 0;
    }
    if (s->asc_n >= AVT_APOGEE_TREND_N && alt_m >= AVT_APOGEE_ARM_M)
    {
      s->armed = true;
      s->has_max = true;
      s->alt_max = alt_m;
      s->drop_n = 0;
      s->dsc_n = 0;
      s->phase = AVT_PHASE_ARMED;
      return AVT_PHASE_ARMED;
    }
    return AVT_PHASE_ASCENT;
  }

  /* --- ARMED: track the peak. A new high resets descent evidence. --- */
  if (!s->has_max || alt_m > s->alt_max)
  {
    s->alt_max = alt_m;
    s->has_max = true;
    s->drop_n = 0;
    s->dsc_n = 0;
    s->phase = AVT_PHASE_ARMED;
    return AVT_PHASE_ARMED;
  }

  /* --- DESCENT TREND GATE --------------------------------------------
   * Below the peak, need TREND_N consecutive negative steps (each at
   * least dsc_step down) AND the legacy absolute drop guard. A single
   * noisy drop or a flat hover at the apex keeps the phase ARMED. */
  if (have_step && step <= -dsc_step + 1e-4f)
  {
    if (s->dsc_n < 255U)
    {
      s->dsc_n++;
    }
  }
  else if (have_step)
  {
    s->dsc_n = 0;
  }
  /* Noise-relative absolute drop guard (saturates at legacy 0.3 m). */
  float drop_need = AVT_APOGEE_DROP_NOISE_K * sigma;
  if (drop_need < AVT_APOGEE_DROP_MIN_M)
  {
    drop_need = AVT_APOGEE_DROP_MIN_M;
  }
  if (drop_need > AVT_APOGEE_DROP_M)
  {
    drop_need = AVT_APOGEE_DROP_M;
  }
  if (alt_m <= s->alt_max - drop_need)
  {
    if (s->drop_n < 255U)
    {
      s->drop_n++;
    }
  }
  else
  {
    s->drop_n = 0;
  }
  if (s->dsc_n >= AVT_APOGEE_TREND_N &&
      s->drop_n >= AVT_APOGEE_CONFIRM_N)
  {
    s->phase = AVT_PHASE_DESCENT;
    return AVT_PHASE_DESCENT;
  }
  s->phase = AVT_PHASE_ARMED;
  return AVT_PHASE_ARMED;
}

/* ---- UART pressure replay (TEST/DEBUG ONLY) -----------------------------
 * Minimal byte-wise line parser; no heap, no stdio, no HAL dependency.
 * Case-insensitive exact match with collapsed single spaces for the REPLAY
 * command set. A bare decimal number = one pressure sample in Pa.
 * Over-long lines are consumed to newline and reported once as ERR. */

void avt_replay_init(avt_replay_t *r)
{
  if (r == NULL)
  {
    return;
  }
  r->active = false; /* default OFF: production path untouched */
  r->has_p0 = false;
  r->p0 = 0.0f;
  r->len = 0;
  r->ovf = false;
  memset(r->buf, 0, sizeof(r->buf));
}

/* strtof wrapper: full-string decimal, finite, no trailing junk. */
static bool avt_replay_num(const char *s, float *out)
{
  if (s == NULL || *s == '\0' || out == NULL)
  {
    return false;
  }
  char *end = NULL;
  float v = strtof(s, &end);
  if (end == s || *end != '\0' || !isfinite(v))
  {
    return false;
  }
  *out = v;
  return true;
}

/* Uppercase in place + collapse runs of space/tab to one space, trim ends. */
static void avt_replay_norm(char *s)
{
  size_t w = 0;
  bool sp = true; /* skip leading space */
  for (size_t i = 0; s[i] != '\0'; i++)
  {
    char c = s[i];
    if (c >= 'a' && c <= 'z')
    {
      c = (char)(c - 'a' + 'A');
    }
    if (c == ' ' || c == '\t')
    {
      if (!sp)
      {
        sp = true;
      }
      continue;
    }
    if (sp && w > 0)
    {
      s[w++] = ' ';
    }
    sp = false;
    s[w++] = c;
  }
  s[w] = '\0';
}

static void avt_replay_decode(avt_replay_t *r, avt_rpl_msg_t *m)
{
  m->kind = AVT_RPL_ERR;
  m->value = 0.0f;
  if (r->ovf)
  {
    return; /* discarded over-long line */
  }
  r->buf[r->len] = '\0';
  avt_replay_norm(r->buf);
  const char *s = r->buf;
  if (*s == '\0')
  {
    m->kind = AVT_RPL_EMPTY;
    return;
  }
  if (strcmp(s, "REPLAY START") == 0)
  {
    r->active = true;
    m->kind = AVT_RPL_START;
    return;
  }
  if (strcmp(s, "REPLAY STOP") == 0)
  {
    r->active = false; /* immediate return to BMP280 path (caller-owned) */
    m->kind = AVT_RPL_STOP;
    return;
  }
  if (strcmp(s, "REPLAY RESET") == 0)
  {
    m->kind = AVT_RPL_RESET; /* caller resets apogee/EMA state */
    return;
  }
  if (strcmp(s, "REPLAY STATUS") == 0)
  {
    m->kind = AVT_RPL_STATUS;
    return;
  }
  if (strncmp(s, "REPLAY P0 ", 10) == 0)
  {
    float v = 0.0f;
    if (avt_replay_num(s + 10, &v) &&
        v >= AVT_REPLAY_MIN_P0_PA && v <= AVT_REPLAY_MAX_P0_PA)
    {
      r->has_p0 = true;
      r->p0 = v;
      m->kind = AVT_RPL_P0;
      m->value = v;
      return;
    }
    return; /* AVT_RPL_ERR */
  }
  /* Bare number = pressure sample. honoured only while active; parse is
   * identical either way so STATUS/error behaviour stays predictable. */
  float p = 0.0f;
  if (avt_replay_num(s, &p) &&
      p >= AVT_REPLAY_MIN_P_PA && p <= AVT_REPLAY_MAX_P_PA)
  {
    if (r->active)
    {
      m->kind = AVT_RPL_SAMPLE;
      m->value = p;
    }
    else
    {
      m->kind = AVT_RPL_EMPTY; /* debug layer swallows it: no sensor effect */
    }
    return;
  }
}

bool avt_replay_put(avt_replay_t *r, char c, avt_rpl_msg_t *m)
{
  if (r == NULL || m == NULL)
  {
    return false;
  }
  if (c == '\r')
  {
    return false;
  }
  if (c != '\n')
  {
    if (!r->ovf)
    {
      if (r->len < (uint8_t)(AVT_REPLAY_LINE_MAX - 1u))
      {
        r->buf[r->len++] = c;
      }
      else
      {
        r->ovf = true; /* keep consuming to newline */
      }
    }
    return false;
  }
  avt_replay_decode(r, m);
  r->len = 0;
  r->ovf = false;
  return true;
}

/* ---- Parallel Kalman altitude (DIAGNOSTIC ONLY) ---------------------------
 * Constant-velocity, scalar 2x2. Process model: velocity random walk with
 * PSD Q (m^2/s^3); position integrates velocity kinematically:
 *   h += v*dt
 *   P00 += dt*(P01+P10) + dt*dt*P11
 *   P01 += dt*P11
 *   P11 += Q*dt
 * Measurement: z = h + noise(R), H = [1,0]:
 *   S = P00 + R; K0 = P00/S; K1 = P01/S
 *   h += K0*y; v += K1*y; P -= K*H*P (scalar Joseph-lite).
 * Drift guards: |v| <= V_MAX, 0 <= Pii <= P_CAP. */

void avt_kf_init(avt_kf_t *k, float h0, float R)
{
  if (k == NULL)
  {
    return;
  }
  if (!(R >= 0.0f) || !isfinite(R) || R < AVT_KF_R_FLOOR)
  {
    R = AVT_KF_R_FLOOR;
  }
  k->h = isfinite(h0) ? h0 : 0.0f;
  k->v = 0.0f;
  k->p00 = R;
  k->p01 = 0.0f;
  k->p11 = R;
  k->R = R;
  k->Q = 0.0f; /* caller must set via avt_kf_set_noise() */
  k->ready = true;
}

void avt_kf_set_noise(avt_kf_t *k, float R, float Q)
{
  if (k == NULL)
  {
    return;
  }
  if (R >= 0.0f && isfinite(R))
  {
    k->R = (R < AVT_KF_R_FLOOR) ? AVT_KF_R_FLOOR : R;
  }
  if (Q > 0.0f && isfinite(Q))
  {
    k->Q = Q;
  }
}

void avt_kf_predict(avt_kf_t *k, float dt)
{
  if (k == NULL || !k->ready)
  {
    return;
  }
  if (!isfinite(dt) || dt <= 0.0f)
  {
    return; /* controlled: hold state, never invent motion */
  }
  if (dt < AVT_KF_DT_MIN_S)
  {
    dt = AVT_KF_DT_MIN_S;
  }
  else if (dt > AVT_KF_DT_MAX_S)
  {
    dt = AVT_KF_DT_MAX_S;
  }
  float q = (k->Q > 0.0f && isfinite(k->Q)) ? k->Q : 0.0f;
  k->h += k->v * dt;
  k->p00 += dt * (k->p01 + k->p01) + dt * dt * k->p11;
  k->p01 += dt * k->p11;
  k->p11 += q * dt;
  /* Drift guards: cap covariance, clamp velocity. */
  if (k->p00 > AVT_KF_P_CAP)
  {
    k->p00 = AVT_KF_P_CAP;
  }
  if (k->p11 > AVT_KF_P_CAP)
  {
    k->p11 = AVT_KF_P_CAP;
  }
  if (k->p00 < 0.0f)
  {
    k->p00 = 0.0f;
  }
  if (k->p11 < 0.0f)
  {
    k->p11 = 0.0f;
  }
  if (k->v > AVT_KF_V_MAX)
  {
    k->v = AVT_KF_V_MAX;
  }
  else if (k->v < -AVT_KF_V_MAX)
  {
    k->v = -AVT_KF_V_MAX;
  }
}

bool avt_kf_update(avt_kf_t *k, float z, bool valid)
{
  if (k == NULL || !k->ready)
  {
    return false;
  }
  if (!valid || !isfinite(z))
  {
    return false; /* update yok; caller still runs clamped predict */
  }
  float R = (k->R >= AVT_KF_R_FLOOR && isfinite(k->R)) ? k->R : AVT_KF_R_FLOOR;
  float y = z - k->h;
  float S = k->p00 + R;
  if (!(S > 0.0f) || !isfinite(S))
  {
    return false;
  }
  float K0 = k->p00 / S;
  float K1 = k->p01 / S;
  if (!isfinite(K0) || !isfinite(K1))
  {
    return false;
  }
  k->h += K0 * y;
  k->v += K1 * y;
  /* P -= K*H*P with H=[1,0]: row0 scales by (1-K0), cross terms shrink. */
  float p00 = k->p00 - K0 * k->p00;
  float p01 = k->p01 - K0 * k->p01;
  float p11 = k->p11 - K1 * k->p01;
  k->p00 = (p00 >= 0.0f && isfinite(p00)) ? p00 : 0.0f;
  k->p01 = isfinite(p01) ? p01 : 0.0f;
  k->p11 = (p11 >= 0.0f && isfinite(p11)) ? p11 : 0.0f;
  if (k->p00 > AVT_KF_P_CAP)
  {
    k->p00 = AVT_KF_P_CAP;
  }
  if (k->p11 > AVT_KF_P_CAP)
  {
    k->p11 = AVT_KF_P_CAP;
  }
  if (k->v > AVT_KF_V_MAX)
  {
    k->v = AVT_KF_V_MAX;
  }
  else if (k->v < -AVT_KF_V_MAX)
  {
    k->v = -AVT_KF_V_MAX;
  }
  return true;
}

/* ---- MPU6050 bias/scale calibration (additive, identity by default) ---- */

void avt_mpu_cal_identity(avt_mpu_cal_t *c)
{
  if (c == NULL)
  {
    return;
  }
  c->bx = 0.0f;
  c->by = 0.0f;
  c->bz = 0.0f;
  c->sx = 1.0f;
  c->sy = 1.0f;
  c->sz = 1.0f;
}

void avt_mpu_cal_apply(const avt_mpu_cal_t *c, float rx, float ry, float rz,
                       float *ox, float *oy, float *oz)
{
  float bx = 0.0f, by = 0.0f, bz = 0.0f;
  float sx = 1.0f, sy = 1.0f, sz = 1.0f;
  if (c != NULL && isfinite(c->bx) && isfinite(c->by) &&
      isfinite(c->bz) && isfinite(c->sx) && isfinite(c->sy) &&
      isfinite(c->sz) && c->sx != 0.0f && c->sy != 0.0f && c->sz != 0.0f)
  {
    bx = c->bx;
    by = c->by;
    bz = c->bz;
    sx = c->sx;
    sy = c->sy;
    sz = c->sz;
  }
  if (ox != NULL)
  {
    *ox = isfinite(rx) ? (rx - bx) * sx : rx;
  }
  if (oy != NULL)
  {
    *oy = isfinite(ry) ? (ry - by) * sy : ry;
  }
  if (oz != NULL)
  {
    *oz = isfinite(rz) ? (rz - bz) * sz : rz;
  }
}

bool avt_mpu_cal_from_6pos(float px, float nx, float py, float ny,
                           float pz, float nz, float G, avt_mpu_cal_t *out)
{
  const float *p[3] = {&px, &py, &pz};
  const float *n[3] = {&nx, &ny, &nz};
  float b[3], s[3];
  if (out == NULL || !(G > 0.0f) || !isfinite(G))
  {
    return false;
  }
  for (int i = 0; i < 3; i++)
  {
    if (!isfinite(*p[i]) || !isfinite(*n[i]))
    {
      return false;
    }
    float span = *p[i] - *n[i];
    if (!(span > 0.0f) || !isfinite(span))
    {
      return false; /* wrong orientation order or degenerate */
    }
    b[i] = (*p[i] + *n[i]) * 0.5f;
    s[i] = (2.0f * G) / span;
  }
  out->bx = b[0];
  out->by = b[1];
  out->bz = b[2];
  out->sx = s[0];
  out->sy = s[1];
  out->sz = s[2];
  return true;
}

bool avt_mpu_cal_residual_ok(const avt_mpu_cal_t *c, float rx, float ry,
                             float rz, float G, float tol)
{
  float ox = 0.0f, oy = 0.0f, oz = 0.0f;
  if (!(G > 0.0f) || !isfinite(G) || !(tol >= 0.0f) || !isfinite(tol))
  {
    return false;
  }
  avt_mpu_cal_apply(c, rx, ry, rz, &ox, &oy, &oz);
  float m = sqrtf(ox * ox + oy * oy + oz * oz);
  if (!isfinite(m))
  {
    return false;
  }
  float dev = (m > G) ? (m - G) : (G - m);
  return (dev / G) <= tol;
}
