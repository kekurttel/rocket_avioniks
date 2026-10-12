/* Mahony 6-DOF complementary filter. See mahony.h. */
#include "mahony.h"

#include <math.h>

static float clamp1(float v)
{
  if (v > 1.0f)
  {
    return 1.0f;
  }
  if (v < -1.0f)
  {
    return -1.0f;
  }
  return v;
}

static void quat_normalize(float *w, float *x, float *y, float *z)
{
  float n = sqrtf((*w) * (*w) + (*x) * (*x) + (*y) * (*y) + (*z) * (*z));
  if (n > 0.0f && isfinite(n))
  {
    *w /= n;
    *x /= n;
    *y /= n;
    *z /= n;
  }
  else
  {
    *w = 1.0f;
    *x = 0.0f;
    *y = 0.0f;
    *z = 0.0f;
  }
}

void mahony_init(mahony_t *m, float gx, float gy, float gz)
{
  float n = sqrtf(gx * gx + gy * gy + gz * gz);
  if (n <= 0.0f || !isfinite(n))
  {
    /* Degenerate reference: identity (startup pose = 0 deg by definition). */
    m->qw = 1.0f;
    m->qx = 0.0f;
    m->qy = 0.0f;
    m->qz = 0.0f;
  }
  else
  {
    gx /= n;
    gy /= n;
    gz /= n;
    if (gz < -0.9999f)
    {
      /* Antipodal: shortest arc undefined; any 180 deg map [0,0,1]->-Z.
         Conjugated form to match the estimator convention. */
      m->qw = 0.0f;
      m->qx = -1.0f;
      m->qy = 0.0f;
      m->qz = 0.0f;
    }
    else
    {
      /* Shortest arc rotating [0,0,1] onto g, CONJUGATED: the estimator
         reads gravity through the transpose convention (see v_est), so
         the reference must satisfy v_est(q) = g. The unconjugated arc
         satisfies R(q)*[0,0,1] = g instead, which diverges on any
         non-flat startup pose (invisible only when g = +Z). */
      m->qw = 1.0f + gz;
      m->qx = gy;
      m->qy = -gx;
      m->qz = 0.0f;
      quat_normalize(&m->qw, &m->qx, &m->qy, &m->qz);
    }
  }
  m->rw = m->qw;
  m->rx = m->qx;
  m->ry = m->qy;
  m->rz = m->qz;
  m->ix = 0.0f;
  m->iy = 0.0f;
  m->iz = 0.0f;
}

bool mahony_update(mahony_t *m,
                   float wx, float wy, float wz, bool gyro_ok,
                   float ax, float ay, float az, bool accel_ok,
                   float gref_mag, float dt, float *tilt_deg)
{
  if (tilt_deg == NULL || !gyro_ok)
  {
    return false;
  }
  if (!isfinite(wx) || !isfinite(wy) || !isfinite(wz) ||
      !isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
      !isfinite(gref_mag) || !isfinite(dt))
  {
    return false;
  }
  if (dt <= 0.0f)
  {
    return false;
  }
  if (dt > MAHONY_DT_MAX)
  {
    dt = MAHONY_DT_MAX;
  }

  /* Adaptive accel gain from magnitude error (existing guard logic). */
  float gain = 0.0f;
  float anx = 0.0f, any = 0.0f, anz = 0.0f;
  if (accel_ok && gref_mag > 0.0f)
  {
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0.0f && isfinite(n))
    {
      anx = ax / n;
      any = ay / n;
      anz = az / n;
      float dev = (n > gref_mag) ? (n - gref_mag) : (gref_mag - n);
      float err = dev / gref_mag;
      if (err <= MAHONY_ACCEL_REDUCED_ERROR)
      {
        gain = MAHONY_ACCEL_GAIN_NORMAL;
      }
      else if (err <= MAHONY_ACCEL_DISABLE_ERROR)
      {
        gain = MAHONY_ACCEL_GAIN_REDUCED;
      }
    }
  }

  /* Estimated gravity direction from current quaternion. */
  float vx = 2.0f * (m->qx * m->qz - m->qw * m->qy);
  float vy = 2.0f * (m->qw * m->qx + m->qy * m->qz);
  float vz = m->qw * m->qw - m->qx * m->qx - m->qy * m->qy + m->qz * m->qz;

  /* Correction error: measured x estimated gravity. */
  float ex = any * vz - anz * vy;
  float ey = anz * vx - anx * vz;
  float ez = anx * vy - any * vx;

  if (gain > 0.0f)
  {
    m->ix += ex * MAHONY_KI * dt;
    m->iy += ey * MAHONY_KI * dt;
    m->iz += ez * MAHONY_KI * dt;
    if (m->ix > MAHONY_INT_MAX)
    {
      m->ix = MAHONY_INT_MAX;
    }
    else if (m->ix < -MAHONY_INT_MAX)
    {
      m->ix = -MAHONY_INT_MAX;
    }
    if (m->iy > MAHONY_INT_MAX)
    {
      m->iy = MAHONY_INT_MAX;
    }
    else if (m->iy < -MAHONY_INT_MAX)
    {
      m->iy = -MAHONY_INT_MAX;
    }
    if (m->iz > MAHONY_INT_MAX)
    {
      m->iz = MAHONY_INT_MAX;
    }
    else if (m->iz < -MAHONY_INT_MAX)
    {
      m->iz = -MAHONY_INT_MAX;
    }
  }

  float cx = wx + MAHONY_KP * gain * ex + m->ix;
  float cy = wy + MAHONY_KP * gain * ey + m->iy;
  float cz = wz + MAHONY_KP * gain * ez + m->iz;

  /* Integrate quaternion with corrected gyro. */
  float qw = m->qw, qx = m->qx, qy = m->qy, qz = m->qz;
  float h = 0.5f * dt;
  m->qw += h * (-qx * cx - qy * cy - qz * cz);
  m->qx += h * (qw * cx + qy * cz - qz * cy);
  m->qy += h * (qw * cy - qx * cz + qz * cx);
  m->qz += h * (qw * cz + qx * cy - qy * cx);
  quat_normalize(&m->qw, &m->qx, &m->qy, &m->qz);

  /* Relative attitude: q_rel = conj(q_ref) * q (reference is unit). */
  float rqw = m->rw * m->qw + m->rx * m->qx + m->ry * m->qy + m->rz * m->qz;
  float tilt = 2.0f * acosf(clamp1(rqw > 0.0f ? rqw : -rqw));
  if (!isfinite(tilt))
  {
    return false;
  }
  *tilt_deg = tilt * 57.29578f;
  return true;
}
