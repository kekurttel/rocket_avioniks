/* Mahony 6-DOF nonlinear complementary filter (IMU only, no magnetometer).
 * Reference: Mahony, Hamel, Pflimlin, "Non-linear complementary filters
 * on the special orthogonal group", IEEE TAC, 2008.
 *
 * HAL-independent: pure math over caller-supplied samples, hence host
 * testable. First-stage use is DIAGNOSTIC ONLY: it never drives the
 * existing TILT condition or any LED.
 */
#ifndef MAHONY_H
#define MAHONY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* MPU6050 reset defaults (GYRO_CONFIG is never written by this project):
 * +-250 dps -> 131 LSB per deg/s. Accel scale is irrelevant (normalized). */
#define MAHONY_GYRO_LSB_PER_DPS (131.0f)
#define MAHONY_DEG2RAD (0.0174533f)

/* Gains (Mahony et al. 2008; standard embedded practice at ~20 Hz):
 * Kp=2.0 -> correction time constant ~0.5 s: tracks hand motion without
 *   passing raw sensor jitter straight into the attitude.
 * Ki=0.01 -> slow bias trim only; the real bias comes from the startup
 *   calibration. Integral clamped to +-0.1 rad/s (anti-windup). */
#define MAHONY_KP (2.0f)
#define MAHONY_KI (0.01f)
#define MAHONY_INT_MAX (0.1f)

/* Accel-magnitude adaptive gain, anchored on the existing 25% guard:
 * <=12.5% deviation: full correction; <=25%: quarter; above: gyro-only
 * propagation (accel correction disabled, quaternion still integrated). */
#define MAHONY_ACCEL_GAIN_NORMAL (1.0f)
#define MAHONY_ACCEL_GAIN_REDUCED (0.25f)
#define MAHONY_ACCEL_REDUCED_ERROR (0.125f)
#define MAHONY_ACCEL_DISABLE_ERROR (0.25f)

/* Timestep clamp: dt<=0 skips the update; large dt is clamped. */
#define MAHONY_DT_MAX (0.1f)

typedef struct
{
  float qw, qx, qy, qz; /* current attitude estimate */
  float rw, rx, ry, rz; /* startup reference attitude */
  float ix, iy, iz;     /* integral (bias) correction, rad/s */
} mahony_t;

/* Init from the startup gravity direction (raw LSB ok, normalized inside).
 * Zero vector -> identity. Reference = initial attitude (0 deg at start). */
void mahony_init(mahony_t *m, float gx, float gy, float gz);

/* One filter step. Gyro in rad/s (bias-corrected by caller), accel in any
 * consistent unit (normalized inside). Returns false (state untouched
 * except gyro propagation rules below) when: bad dt, bad gyro, or
 * non-finite inputs. Invalid/zero accel disables the accel correction
 * but gyro propagation continues (returns true, tilt still produced).
 * tilt_deg: rotation magnitude of q_rel = inv(q_ref)*q in [0,180]. */
bool mahony_update(mahony_t *m,
                   float wx, float wy, float wz, bool gyro_ok,
                   float ax, float ay, float az, bool accel_ok,
                   float gref_mag, float dt, float *tilt_deg);

#endif /* MAHONY_H */
