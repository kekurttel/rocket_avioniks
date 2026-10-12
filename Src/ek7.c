/* Ek-7 protokolu. Bkz. ek7.h (dokuman sapmalari D1-D5 orada). */
#include "ek7.h"

#include <math.h>
#include <string.h>

uint16_t ek7_status_word(const flight_t *f, bool tilt_band, bool alt_valid,
                         float alt_m)
{
  uint16_t st = 0;
  if (f == NULL)
  {
    return 0;
  }
  /* FAULT/COMPLETE disiplini: FAULT hicbir ucus bitini tasimaz (Tablo 5'te
   * fault biti yok; fault RED LED + UART ile yerelde gosterilir). */
  bool flown = (f->state >= FLIGHT_ASCENT && f->state <= FLIGHT_COMPLETE);
  bool past_apogee = (flown && f->state >= FLIGHT_APOGEE_CONFIRMED);
  if (flown)
  {
    st |= EK7_ST_LAUNCH;
  }
  /* bit1 (burnout timer): tasarimda yok -> 0 (D5). */
  if (f->has_max && f->max_alt_m >= f->cfg.apogee_alt_min_m)
  {
    st |= EK7_ST_MINALT;
  }
  if (tilt_band)
  {
    st |= EK7_ST_ANGLE;
  }
  if (past_apogee)
  {
    st |= EK7_ST_DESCENT;
  }
  if (f->drogue_deployed)
  {
    st |= EK7_ST_DROGUE;
  }
  if (alt_valid && isfinite(alt_m) && alt_m < f->cfg.main_alt_m)
  {
    st |= EK7_ST_BELOWMAIN;
  }
  if (f->main_deployed)
  {
    st |= EK7_ST_MAIN;
  }
  return st;
}

void ek7_rx_init(ek7_rx_t *r)
{
  if (r == NULL)
  {
    return;
  }
  memset(r->buf, 0, sizeof(r->buf));
  r->n = 0;
}

bool ek7_rx_feed(ek7_rx_t *r, uint8_t b, ek7_cmd_t *out)
{
  static ek7_cmd_t none = EK7_EV_NONE;
  if (r == NULL)
  {
    if (out != NULL)
    {
      *out = EK7_EV_NONE;
    }
    return false;
  }
  if (out == NULL)
  {
    out = &none;
  }
  *out = EK7_EV_NONE;
  if (r->n == 0 && b != EK7_CMD_HDR)
  {
    return false; /* header disi bayt: yoksay (senkron bekle) */
  }
  if (b == EK7_CMD_HDR && r->n > 0)
  {
    r->n = 0; /* cerceve ici header: yeniden senkronize ol */
  }
  if (r->n < EK7_CMD_LEN)
  {
    r->buf[r->n++] = b;
  }
  if (r->n < EK7_CMD_LEN)
  {
    return false;
  }
  /* 5 bayt tamam: AA CMD SUM 0D 0A. Checksum iki kuraldan birini tutmali:
   * - Resmi (calisan test yazilimi): Bolum 3 toplama kurali,
   *   SUM = (0xAA + CMD) & 0xFF  ->  SIT 0xCA, SUT 0xCC, STOP 0xCE.
   * - Dokuman-literal (Tablo 1): SIT 0x8C, SUT 0x8E, STOP 0x90.
   *   (Tablo 1 ile Bolum 3 birbirini tutmuyor; uygulama toplama
   *   kuralini kullaniyor. Geriye uyumluluk icin ikisi de kabul.) */
  r->n = 0;
  if (r->buf[0] != EK7_CMD_HDR || r->buf[3] != EK7_FTR1 ||
      r->buf[4] != EK7_FTR2)
  {
    return false;
  }
  uint8_t cmd = r->buf[1], sum = r->buf[2];
  uint8_t official = (uint8_t)((EK7_CMD_HDR + cmd) & 0xFFu);
  bool sum_ok = (sum == official);
  bool legacy_ok = ((cmd == EK7_CMD_SIT && sum == EK7_SUM_SIT) ||
                    (cmd == EK7_CMD_SUT && sum == EK7_SUM_SUT) ||
                    (cmd == EK7_CMD_STOP && sum == EK7_SUM_STOP));
  if (!sum_ok && !legacy_ok)
  {
    return false; /* bilinmeyen komut / checksum uyumsuzlugu */
  }
  if (cmd == EK7_CMD_SIT)
  {
    *out = EK7_EV_SIT;
    return true;
  }
  if (cmd == EK7_CMD_SUT)
  {
    *out = EK7_EV_SUT;
    return true;
  }
  if (cmd == EK7_CMD_STOP)
  {
    *out = EK7_EV_STOP;
    return true;
  }
  return false; /* bilinmeyen komut kodu */
}

uint8_t ek7_sum(const uint8_t *d, size_t n)
{
  uint32_t s = 0;
  if (d == NULL)
  {
    return 0;
  }
  for (size_t i = 0; i < n; i++)
  {
    s += d[i];
  }
  return (uint8_t)(s & 0xFFu);
}

void ek7_pack_f32be(float v, uint8_t out[4])
{
  if (out == NULL)
  {
    return;
  }
  float q = (isfinite(v) ? v : 0.0f);
  q = roundf(q * 100.0f) / 100.0f; /* Ek-7: virgulden sonra 2 basamak */
  uint32_t u = 0;
  memcpy(&u, &q, sizeof(u));
  out[0] = (uint8_t)(u >> 24);
  out[1] = (uint8_t)(u >> 16);
  out[2] = (uint8_t)(u >> 8);
  out[3] = (uint8_t)u;
}

float ek7_unpack_f32be(const uint8_t in[4])
{
  uint32_t u;
  if (in == NULL)
  {
    return 0.0f;
  }
  u = ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) |
      ((uint32_t)in[2] << 8) | in[3];
  float v = 0.0f;
  memcpy(&v, &u, sizeof(v));
  return v;
}

size_t ek7_build_tlm(uint8_t out[EK7_TLM_LEN], float alt_m, float press_mbar,
                     float ax, float ay, float az, float ex, float ey, float ez)
{
  const float f[EK7_NFIELD] = {alt_m, press_mbar, ax, ay, az, ex, ey, ez};
  if (out == NULL)
  {
    return 0;
  }
  out[0] = EK7_SYN_HDR;
  for (uint8_t i = 0; i < EK7_NFIELD; i++)
  {
    ek7_pack_f32be(f[i], &out[1 + 4 * i]);
  }
  /* Resmi checksum: header + 32 float bayti, mod 256 (footer haric).
   * Calisan test yazilimi boyle dogruluyor (pack_telemetry docstring). */
  out[33] = ek7_sum(&out[0], 33);
  out[34] = EK7_FTR1;
  out[35] = EK7_FTR2;
  return EK7_TLM_LEN;
}

bool ek7_parse_tlm(const uint8_t in[EK7_TLM_LEN], float *alt_m,
                   float *press_mbar, float *ax, float *ay, float *az,
                   float *ex, float *ey, float *ez)
{
  float *f[EK7_NFIELD] = {alt_m, press_mbar, ax, ay, az, ex, ey, ez};
  if (in == NULL)
  {
    return false;
  }
  if (in[0] != EK7_SYN_HDR || in[34] != EK7_FTR1 || in[35] != EK7_FTR2)
  {
    return false;
  }
  if (in[33] != ek7_sum(&in[0], 33))
  {
    return false;
  }
  for (uint8_t i = 0; i < EK7_NFIELD; i++)
  {
    float v = ek7_unpack_f32be(&in[1 + 4 * i]);
    if (!isfinite(v))
    {
      return false;
    }
    if (f[i] != NULL)
    {
      *f[i] = v;
    }
  }
  return true;
}

size_t ek7_build_status(uint8_t out[EK7_STATUS_LEN], uint16_t st)
{
  if (out == NULL)
  {
    return 0;
  }
  out[0] = EK7_CMD_HDR;
  out[1] = (uint8_t)(st & 0xFF); /* Data1: bit 0-7 (once dusuk byte) */
  out[2] = (uint8_t)(st >> 8);   /* Data2: bit 8-15 */
  /* Resmi checksum: header + 2 data bayti, mod 256. */
  out[3] = ek7_sum(&out[0], 3);
  out[4] = EK7_FTR1;
  out[5] = EK7_FTR2;
  return EK7_STATUS_LEN;
}

void ek7_map_acc(float ax_lsb, float ay_lsb, float az_lsb, float gref_mag,
                 float *ox, float *oy, float *oz)
{
  float ref = (gref_mag > 0.0f && isfinite(gref_mag))
                  ? gref_mag
                  : EK7_MPU_LSB_PER_G;
  const float k = EK7_MS2_PER_G / ref;
  float x = ax_lsb * k, y = ay_lsb * k, z = -az_lsb * k; /* Z asagi (+asagi) */
  if (ox != NULL)
  {
    *ox = x;
  }
  if (oy != NULL)
  {
    *oy = y;
  }
  if (oz != NULL)
  {
    *oz = z;
  }
}

void ek7_map_ang(float ax, float ay, float az,
                 float *ex, float *ey, float *ez)
{
  float exv = 0.0f, eyv = 0.0f;
  if (isfinite(ax) && isfinite(ay) && isfinite(az))
  {
    float ayz = sqrtf(ay * ay + az * az);
    float axz = sqrtf(ax * ax + az * az);
    if (ayz > 0.0f)
    {
      exv = atan2f(ax, ayz) * 57.29578f;
    }
    if (axz > 0.0f)
    {
      eyv = atan2f(ay, axz) * 57.29578f;
    }
  }
  if (ex != NULL)
  {
    *ex = exv;
  }
  if (ey != NULL)
  {
    *ey = eyv;
  }
  if (ez != NULL)
  {
    *ez = 0.0f; /* yaw: manyetometre yok (D5 notu) */
  }
}
