/* Flight lifecycle. See flight.h. Pure C, no HAL, no heap. */
#include "flight.h"

#include <math.h>

void flight_config_production(flight_cfg_t *c)
{
  if (c == NULL)
  {
    return;
  }
  c->apogee_alt_min_m = FLIGHT_APOGEE_ALT_MIN_PROD_M;
  c->main_alt_m = FLIGHT_MAIN_ALT_PROD_M;
  c->apogee_drop_m = FLIGHT_APOGEE_DROP_M;
  c->descent_vel_mps = FLIGHT_DESCENT_VEL_MPS;
  c->test_mode = false;
  /* Bench fall trigger is display + BLUE LED only in EVERY mode (it can
   * never advance state or fire deployment by construction), so hand
   * testing works on the production build too. */
  c->bench_enable = true;
  c->bench_neg_thr_g = FLIGHT_BENCH_NEG_THR_TEST_G;
  c->bench_confirm_n = FLIGHT_BENCH_CONFIRM_N_TEST;
  c->apogee_confirm_n = FLIGHT_APOGEE_CONFIRM_N;
  c->bench_baro_vel_mps = FLIGHT_BENCH_BARO_VEL_TEST_MPS;
  c->bench_baro_confirm_n = FLIGHT_BENCH_BARO_CONFIRM_N_TEST;
  c->bench_baro_drop_m = FLIGHT_BENCH_BARO_DROP_M;
}

void flight_config_test(flight_cfg_t *c, float main_alt_m)
{
  if (c == NULL)
  {
    return;
  }
  c->apogee_alt_min_m = FLIGHT_APOGEE_ALT_MIN_TEST_M;
  c->main_alt_m = (isfinite(main_alt_m) && main_alt_m > 0.0f)
                      ? main_alt_m
                      : FLIGHT_MAIN_ALT_TEST_M;
  c->apogee_drop_m = FLIGHT_APOGEE_DROP_M;
  c->descent_vel_mps = FLIGHT_DESCENT_VEL_MPS;
  c->test_mode = true;
  c->bench_enable = true; /* bench fall = display + BLUE LED, any mode */
  c->bench_neg_thr_g = FLIGHT_BENCH_NEG_THR_TEST_G;
  c->bench_confirm_n = FLIGHT_BENCH_CONFIRM_N_TEST;
  c->apogee_confirm_n = FLIGHT_APOGEE_CONFIRM_N;
  c->bench_baro_vel_mps = FLIGHT_BENCH_BARO_VEL_TEST_MPS;
  c->bench_baro_confirm_n = FLIGHT_BENCH_BARO_CONFIRM_N_TEST;
  c->bench_baro_drop_m = FLIGHT_BENCH_BARO_DROP_M;
}

void flight_init(flight_t *f, const flight_cfg_t *c)
{
  if (f == NULL)
  {
    return;
  }
  if (c != NULL)
  {
    f->cfg = *c;
  }
  else
  {
    flight_config_production(&f->cfg);
  }
  f->state = FLIGHT_INIT;
  f->max_alt_m = 0.0f;
  f->has_max = false;
  f->prev_alt_m = 0.0f;
  f->has_prev = false;
  f->vel_mps = 0.0f;
  f->vel_valid = false;
  f->drop_flag = false;
  f->vel_flag = false;
  f->apogee_n = 0;
  f->launch_n = 0;
  f->ff_n = 0;
  f->invalid_n = 0;
  f->vert_dyn_g = 0.0f;
  f->vert_mag_ratio = 1.0f;
  f->vert_valid = false;
  f->bench_n = 0;
  f->bench_confirmed = false;
  f->bench_baro_max_m = 0.0f;
  f->bench_baro_prev_m = 0.0f;
  f->bench_baro_has_prev = false;
  f->bench_baro_vel_mps = 0.0f;
  f->bench_baro_vel_valid = false;
  f->bench_baro_drop_flag = false;
  f->bench_baro_n = 0;
  f->bench_baro_confirmed = false;
  f->bench_motion_ok = true;
  f->drogue_deployed = false;
  f->main_deployed = false;
  f->drogue_event = false;
  f->main_event = false;
}

void flight_set_state(flight_t *f, flight_state_t s)
{
  if (f == NULL)
  {
    return;
  }
  /* Only direct wiring used by startup (INIT->CALIBRATION->WAIT).
   * In-flight transitions go through flight_step() so the lifecycle
   * stays single-direction and latched. FAULT is sticky once set. */
  if (f->state == FLIGHT_FAULT && s != FLIGHT_FAULT)
  {
    return;
  }
  f->state = s;
}

bool flight_velocity(float cur_m, float prev_m, float dt_s, float *out_v)
{
  if (out_v == NULL)
  {
    return false;
  }
  if (!isfinite(cur_m) || !isfinite(prev_m) || !isfinite(dt_s))
  {
    return false;
  }
  if (!(dt_s > 0.0f) || !isfinite(dt_s))
  {
    return false;
  }
  float dt = dt_s;
  if (dt < FLIGHT_VEL_DT_MIN_S)
  {
    dt = FLIGHT_VEL_DT_MIN_S;
  }
  else if (dt > FLIGHT_VEL_DT_MAX_S)
  {
    dt = FLIGHT_VEL_DT_MAX_S;
  }
  float v = (cur_m - prev_m) / dt;
  if (!isfinite(v))
  {
    return false;
  }
  *out_v = v;
  return true;
}

bool flight_launch_push(flight_t *f, float acc_mag, float gref_mag)
{
  if (f == NULL)
  {
    return false;
  }
  if (!isfinite(acc_mag) || !isfinite(gref_mag) || gref_mag <= 0.0f)
  {
    f->launch_n = 0;
    return false;
  }
  if (acc_mag > gref_mag * FLIGHT_LAUNCH_RATIO)
  {
    if (f->launch_n < 255u)
    {
      f->launch_n++;
    }
  }
  else
  {
    f->launch_n = 0;
  }
  return f->launch_n >= FLIGHT_LAUNCH_CONFIRM_N;
}

bool flight_freefall_push(flight_t *f, float acc_mag, float gref_mag)
{
  if (f == NULL)
  {
    return false;
  }
  if (!isfinite(acc_mag) || !isfinite(gref_mag) || gref_mag <= 0.0f)
  {
    f->ff_n = 0;
    return false;
  }
  if (acc_mag < gref_mag * FLIGHT_FREEFALL_RATIO)
  {
    if (f->ff_n < 255u)
    {
      f->ff_n++;
    }
  }
  else
  {
    f->ff_n = 0;
  }
  return f->ff_n >= FLIGHT_FREEFALL_CONFIRM_N;
}

bool flight_vert_dyn_g(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float *out_dyn_g, float *out_mag_ratio)
{
  if (out_dyn_g == NULL || out_mag_ratio == NULL)
  {
    return false;
  }
  if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
      !isfinite(gx) || !isfinite(gy) || !isfinite(gz))
  {
    return false;
  }
  float gmag = sqrtf(gx * gx + gy * gy + gz * gz);
  if (!(gmag > 0.0f) || !isfinite(gmag))
  {
    return false;
  }
  float amag = sqrtf(ax * ax + ay * ay + az * az);
  if (!isfinite(amag))
  {
    return false;
  }
  /* Unit board-up from calibration, then signed projection in g units. */
  float ux = gx / gmag, uy = gy / gmag, uz = gz / gmag;
  float vert_g = (ax * ux + ay * uy + az * uz) / gmag;
  if (!isfinite(vert_g))
  {
    return false;
  }
  *out_dyn_g = vert_g - 1.0f;
  *out_mag_ratio = amag / gmag;
  return true;
}

bool flight_bench_push(flight_t *f, float vert_dyn_g, float mag_ratio,
                       bool valid)
{
  if (f == NULL)
  {
    return false;
  }
  if (!f->cfg.bench_enable)
  {
    return false; /* production: fully inert, zero side effects */
  }
  if (f->bench_confirmed)
  {
    return true; /* latched one-shot episode: never un-confirms */
  }
  uint8_t need = (f->cfg.bench_confirm_n > 0u)
                     ? f->cfg.bench_confirm_n
                     : FLIGHT_BENCH_CONFIRM_N_TEST;
  if (!valid || !isfinite(vert_dyn_g) || !isfinite(mag_ratio))
  {
    f->bench_n = 0; /* invalid sample: streak resets, state never advances */
    return false;
  }
  f->bench_n = avt_streak_push(f->bench_n,
                               vert_dyn_g < f->cfg.bench_neg_thr_g &&
                                   mag_ratio < FLIGHT_BENCH_MAG_RATIO);
  if (f->bench_n >= need)
  {
    f->bench_confirmed = true;
    return true;
  }
  return false;
}

bool flight_bench_baro_push(flight_t *f, float alt_m, bool valid, float dt_s)
{
  if (f == NULL)
  {
    return false;
  }
  if (!f->cfg.bench_enable)
  {
    return false; /* production: fully inert, zero side effects */
  }
  if (f->bench_baro_confirmed)
  {
    return true; /* latched one-shot episode: never un-confirms */
  }
  uint8_t need = (f->cfg.bench_baro_confirm_n > 0u)
                     ? f->cfg.bench_baro_confirm_n
                     : FLIGHT_BENCH_BARO_CONFIRM_N_TEST;
  if (!valid || !isfinite(alt_m))
  {
    f->bench_baro_n = 0; /* invalid: streak resets, never confirms */
    f->bench_baro_vel_valid = false;
    return false;
  }
  /* Own max tracker (display drop flag); never touches flight apogee max.
   * Deliberately monotonic (no washout): the max must survive a pause at
   * the top of a hand lift, otherwise holding still a few seconds erases
   * the reference and a normal 0.5 m push-down can never satisfy
   * (max - drop_m). A stale peak pinned after set-down is harmless: the
   * motion-liveness term below refuses confirmation while the board sits
   * still, so only real handling can complete the trigger. */
  if (!f->bench_baro_has_prev || !isfinite(f->bench_baro_max_m) ||
      alt_m > f->bench_baro_max_m)
  {
    f->bench_baro_max_m = alt_m;
  }
  float drop_m = (f->cfg.bench_baro_drop_m > 0.0f)
                     ? f->cfg.bench_baro_drop_m
                     : FLIGHT_BENCH_BARO_DROP_M;
  f->bench_baro_drop_flag = (alt_m < f->bench_baro_max_m - drop_m);
  /* Velocity from the delivered altitude with real dt (same helper as the
   * flight gate; no new smoother). */
  if (f->bench_baro_has_prev)
  {
    float v = 0.0f;
    if (flight_velocity(alt_m, f->bench_baro_prev_m, dt_s, &v))
    {
      f->bench_baro_vel_mps = v;
      f->bench_baro_vel_valid = true;
    }
    else
    {
      f->bench_baro_vel_valid = false;
    }
  }
  f->bench_baro_prev_m = alt_m;
  f->bench_baro_has_prev = true;
  /* BOTH conditions must hold: below own max AND descending. Bench capture
   * showed -1.2 m/s velocity spikes while the board sat still, so velocity
   * alone self-triggered. Ambient wander satisfies one at a time.
   * Motion liveness is the third AND term: a room pressure step is
   * barometrically identical to a hand-down (captured: -0.5 m at -4.6 m/s,
   * board motionless), so the streak counts only while the caller reports
   * recent board motion. Tracking (max/vel/drop_flag telemetry) continues
   * regardless; only the confirmation streak is gated. */
  bool falling_vel = (f->bench_baro_vel_valid &&
                      f->bench_baro_vel_mps < f->cfg.bench_baro_vel_mps);
  f->bench_baro_n = avt_streak_push(f->bench_baro_n,
                                    f->bench_baro_drop_flag && falling_vel &&
                                        f->bench_motion_ok);
  if (f->bench_baro_n >= need)
  {
    f->bench_baro_confirmed = true;
    return true;
  }
  return false;
}

bool flight_bench_fall(const flight_t *f)
{
  if (f == NULL)
  {
    return false;
  }
  if (!f->cfg.bench_enable)
  {
    return false;
  }
  return f->bench_confirmed || f->bench_baro_confirmed;
}

bool flight_combine_pressure(bool ok1, bool ok2, float p1_pa, float p2_pa,
                             float *out_pa)
{
  if (out_pa == NULL)
  {
    return false;
  }
  /* Gecerlilik disiplini: ok bayragi + sonlu + fiziksel basinc zarfi
   * (avt_press_ok: sifir-veri, NACK copu, takilmis-hat sacmaligi elenir).
   * Zarf ici gecerli basincin degeri degismez; fallback aynen korunur
   * (cift -> ortalama, tek sagkalan -> o, hicbiri -> false). */
  bool v1 = ok1 && avt_press_ok(p1_pa);
  bool v2 = ok2 && avt_press_ok(p2_pa);
  if (v1 && v2)
  {
    float avg = (p1_pa + p2_pa) * 0.5f;
    if (!isfinite(avg) || avg <= 0.0f)
    {
      return false;
    }
    *out_pa = avg;
    return true;
  }
  if (v1)
  {
    *out_pa = p1_pa;
    return true;
  }
  if (v2)
  {
    *out_pa = p2_pa;
    return true;
  }
  return false;
}

void flight_alt_push(flight_t *f, float alt_m, bool valid, float dt_s)
{
  if (f == NULL)
  {
    return;
  }
  f->drogue_event = false;
  f->main_event = false;
  if (!valid || !isfinite(alt_m))
  {
    f->vel_valid = false;
    f->drop_flag = false;
    f->vel_flag = false;
    return;
  }
  /* Max tracking is continuous (spec CASE 3). */
  if (!f->has_max || alt_m > f->max_alt_m)
  {
    f->max_alt_m = alt_m;
    f->has_max = true;
  }
  /* Velocity from real elapsed dt (spec: (cur-prev)/dT). */
  if (f->has_prev)
  {
    float v = 0.0f;
    if (flight_velocity(alt_m, f->prev_alt_m, dt_s, &v))
    {
      f->vel_mps = v;
      f->vel_valid = true;
    }
    else
    {
      f->vel_valid = false;
    }
  }
  f->prev_alt_m = alt_m;
  f->has_prev = true;

  if (!f->has_max)
  {
    return;
  }
  /* Drop gate: current < max - 2 m (spec CASE 4). */
  f->drop_flag = (alt_m < f->max_alt_m - f->cfg.apogee_drop_m);
  /* Descent-velocity gate: v < -2 m/s (spec CASE 5). */
  f->vel_flag = (f->vel_valid && f->vel_mps < f->cfg.descent_vel_mps);
}

bool flight_try_drogue(flight_t *f, bool sensors_usable)
{
  if (f == NULL)
  {
    return false;
  }
  if (f->drogue_deployed)
  {
    return false; /* one-shot: never re-fire */
  }
  if (!sensors_usable)
  {
    return false; /* interlock: no deployment on failed sensors */
  }
  /* State validation: drogue only from FREE_FALL. */
  if (f->state != FLIGHT_FREE_FALL)
  {
    return false;
  }
  f->drogue_deployed = true;
  f->drogue_event = true;
  return true;
}

bool flight_try_main(flight_t *f, bool sensors_usable)
{
  if (f == NULL)
  {
    return false;
  }
  if (f->main_deployed)
  {
    return false;
  }
  if (!sensors_usable)
  {
    return false;
  }
  /* Main only once drogue has flown (DESCENT or later). */
  if (f->state != FLIGHT_DESCENT && f->state != FLIGHT_DROGUE_DEPLOYED)
  {
    return false;
  }
  f->main_deployed = true;
  f->main_event = true;
  return true;
}

static bool flight_apogee_ready(flight_t *f)
{
  /* Production gate: must have climbed past 1250 m. Test: disabled (0). */
  bool hit = f->has_max && f->drop_flag && f->vel_flag &&
             f->max_alt_m >= f->cfg.apogee_alt_min_m;
  uint8_t need = (f->cfg.apogee_confirm_n > 0u) ? f->cfg.apogee_confirm_n
                                                : (uint8_t)FLIGHT_APOGEE_CONFIRM_N;
  f->apogee_n = hit ? (uint8_t)(f->apogee_n < 255u ? f->apogee_n + 1u : 255u) : 0u;
  return f->apogee_n >= need;
}

flight_state_t flight_step(flight_t *f, bool launch_confirmed,
                           bool ff_confirmed, float alt_m, bool alt_valid,
                           float dt_s, bool sensors_usable)
{
  if (f == NULL)
  {
    return FLIGHT_FAULT;
  }
  f->drogue_event = false;
  f->main_event = false;

  /* FAULT is sticky. */
  if (f->state == FLIGHT_FAULT)
  {
    return FLIGHT_FAULT;
  }
  /* COMPLETE is terminal. */
  if (f->state == FLIGHT_COMPLETE)
  {
    return FLIGHT_COMPLETE;
  }

  /* Sustained-invalid watchdog in flight-critical states. */
  bool critical = (f->state == FLIGHT_WAIT_FOR_LAUNCH ||
                   f->state == FLIGHT_ASCENT ||
                   f->state == FLIGHT_APOGEE_CONFIRMED ||
                   f->state == FLIGHT_FREE_FALL ||
                   f->state == FLIGHT_DROGUE_DEPLOYED ||
                   f->state == FLIGHT_DESCENT);
  if (critical)
  {
    if (!alt_valid && !sensors_usable)
    {
      if (f->invalid_n < 65535u)
      {
        f->invalid_n++;
      }
      if (f->invalid_n >= FLIGHT_FAULT_INVALID_N)
      {
        f->state = FLIGHT_FAULT;
        return FLIGHT_FAULT;
      }
    }
    else
    {
      f->invalid_n = 0;
    }
  }

  switch (f->state)
  {
    case FLIGHT_INIT:
    case FLIGHT_CALIBRATION:
      break; /* wired by flight_set_state during startup */

    case FLIGHT_WAIT_FOR_LAUNCH:
      if (launch_confirmed)
      {
        f->state = FLIGHT_ASCENT;
      }
      break;

    case FLIGHT_ASCENT:
      flight_alt_push(f, alt_m, alt_valid, dt_s);
      /* Apogee latches once; never returns to ASCENT. Bench channels are
       * display-only and deliberately not consulted: BENCH_FALL_DETECTED
       * can never advance state or fire deployment. */
      if (flight_apogee_ready(f))
      {
        f->state = FLIGHT_APOGEE_CONFIRMED;
      }
      break;

    case FLIGHT_APOGEE_CONFIRMED:
      flight_alt_push(f, alt_m, alt_valid, dt_s);
      if (ff_confirmed)
      {
        f->state = FLIGHT_FREE_FALL;
      }
      break;

    case FLIGHT_FREE_FALL:
      flight_alt_push(f, alt_m, alt_valid, dt_s);
      /* Drogue fires exactly once on entry to DROGUE_DEPLOYED. */
      if (flight_try_drogue(f, sensors_usable))
      {
        f->state = FLIGHT_DROGUE_DEPLOYED;
      }
      break;

    case FLIGHT_DROGUE_DEPLOYED:
      flight_alt_push(f, alt_m, alt_valid, dt_s);
      /* Lifecycle: DROGUE_DEPLOYED -> DESCENT (spec). */
      f->state = FLIGHT_DESCENT;
      break;

    case FLIGHT_DESCENT:
      flight_alt_push(f, alt_m, alt_valid, dt_s);
      /* Main gate: current < main_alt (PROD 600 m / TEST override).
       * One-shot; then MAIN_DEPLOYED -> COMPLETE. */
      if (alt_valid && isfinite(alt_m) && alt_m < f->cfg.main_alt_m)
      {
        if (flight_try_main(f, sensors_usable))
        {
          f->state = FLIGHT_MAIN_DEPLOYED;
        }
      }
      break;

    case FLIGHT_MAIN_DEPLOYED:
      f->state = FLIGHT_COMPLETE;
      break;

    case FLIGHT_COMPLETE:
    case FLIGHT_FAULT:
      break;
  }
  return f->state;
}

flight_health_t flight_health_snapshot(bool mpu_present, bool mpu_ok,
                                       bool mpu_stale, bool bmp_present,
                                       bool b1_ok, bool bmp_stale,
                                       bool b2_present, bool b2_ok,
                                       bool b2_stale,
                                       avt_bmp_state_t bmp_state)
{
  flight_health_t h;
  h.mpu_present = mpu_present;
  h.mpu_valid = mpu_ok;
  h.mpu_stale = mpu_stale;
  h.bmp_present = bmp_present;
  h.bmp_valid = b1_ok;
  h.bmp_stale = bmp_stale;
  h.b2_present = b2_present;
  h.b2_valid = b2_ok;
  h.b2_stale = b2_stale;
  h.disagree = (bmp_state == AVT_DISAGREE);
  /* Invalid values never enter fusion (same rule as flight_combine): */
  bool v1 = b1_ok, v2 = b2_ok;
  if (v1 && v2)
  {
    h.fused_src = FLIGHT_FUSED_AVG;
  }
  else if (v1)
  {
    h.fused_src = FLIGHT_FUSED_BMP;
  }
  else if (v2)
  {
    h.fused_src = FLIGHT_FUSED_B2;
  }
  else
  {
    h.fused_src = FLIGHT_FUSED_NONE;
  }
  return h;
}

const char *flight_state_str(flight_state_t s)
{
  switch (s)
  {
    case FLIGHT_INIT:
      return "INIT";
    case FLIGHT_CALIBRATION:
      return "CALIBRATION";
    case FLIGHT_WAIT_FOR_LAUNCH:
      return "WAIT_FOR_LAUNCH";
    case FLIGHT_ASCENT:
      return "ASCENT";
    case FLIGHT_APOGEE_CONFIRMED:
      return "APOGEE_CONFIRMED";
    case FLIGHT_FREE_FALL:
      return "FREE_FALL";
    case FLIGHT_DROGUE_DEPLOYED:
      return "DROGUE_DEPLOYED";
    case FLIGHT_DESCENT:
      return "DESCENT";
    case FLIGHT_MAIN_DEPLOYED:
      return "MAIN_DEPLOYED";
    case FLIGHT_COMPLETE:
      return "COMPLETE";
    case FLIGHT_FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}
