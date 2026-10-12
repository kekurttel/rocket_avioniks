/* Host tests for the LoRa telemetry link (Src/lora.c, HAL-independent).
 * Covers the exact byte layout, the additive CRC, the sequence counter and its
 * wrap, the invalid-value sentinels round-tripping, the argument guards, the
 * cadence/SIT-SUT gate and a hand-derived golden 36-byte vector. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> /* NAN / INFINITY: gozcu regresyonu gercek degerlerle */

#include "flight.h"
#include "lora.h"

static int n_pass = 0;
#define CHECK(c) do { assert(c); n_pass++; } while (0)

/* Poll emits the air image: with LORA_FIXED_HDR the 36-byte core sits
 * behind the 3-byte peer header. Ground-station side strips it. */
static const uint8_t *core_of(const uint8_t *tx)
{
#if LORA_FIXED_HDR
  return tx + 3u;
#else
  return tx;
#endif
}

/* ---- Frame used for the golden vector and the round trip ---- */
static lora_frame_t sample(void)
{
  lora_frame_t f;
  f.seq = 0x1234u;        /* -> 34 12                */
  f.t_ms = 0x11223344u;    /* -> 44 33 22 11         */
  f.flight_state = 3u;    /* FLIGHT_ASCENT -> 03 00 */
  f.ek7_mode = 0u;        /* NORMAL         -> 00 00 */
  f.alt_m = 1250;         /* 1250 m         -> E2 04 */
  f.vel_cms = -1500;      /* -15.0 m/s      -> 24 FA */
  f.sats = 9u;            /* -> 09 00                */
  f.lat_e5 = 41012345;    /* 41.012345 deg -> 79 CC 71 02 */
  f.lon_e5 = 29098765;    /* 29.098765 deg -> 0D 03 BC 01 */
  f.batt_mv = 12250u;     /* 12250        -> DA 2F  */
  f.flags = LORA_F_GPS_FIX | LORA_F_BLUE_LED | LORA_F_RED_LED; /* 0B */
  f.tilt_ddeg = 123u;     /* 12.3 deg     -> 7B 00 */
  f.lean_x = 100;         /* 0.100        -> 64 00 */
  f.lean_y = -50;         /* -0.050       -> CE FF */
  return f;
}

/* ---- Hand-derived golden vector ------------------------------------------
 * Every field was converted to little-endian by hand BEFORE the builder was
 * run:
 *   seq          0x1234      = 34 12
 *   t_ms         0x11223344  = 44 33 22 11
 *   flight_state 3           = 03 00
 *   ek7_mode     0           = 00 00
 *   alt_m        1250        = 1250 = 0x04E2  -> E2 04   (WHOLE METRES)
 *   vel_cms      -1500       = 65536-1500 = 64036 = 0xFA24 -> 24 FA
 *   sats         9           = 09 00
 *   lat_e5       41012345    = 0x0271CC79 -> 79 CC 71 02
 *   lon_e5       29098765    = 0x01BC030D -> 0D 03 BC 01
 *   batt_mv      12250       = 0x2FDA    -> DA 2F
 *   flags        0x0B        = 0B
 *   tilt_ddeg    123         = 0x007B    -> 7B 00   (12.3 derece)
 *   lean_x       100         = 0x0064    -> 64 00   (0.100 govde X)
 *   lean_y       -50         = 65536-50 = 65486 = 0xFFCE -> CE FF
 * CRC = sum(bytes 0..34) mod 256. Eski 0..28 toplami 1855 (yukarida);
 * ver 0x01->0x02 (+1) ve yeni baytlar: 7B+00+64+00+CE+FF =
 * 123+0+100+0+206+255 = 684.
 *   1855 + 1 + 684 = 2540; 2540 - 9*256 = 2540 - 2304 = 236 = 0xEC.
 * lat_e5/lon_e5 are degrees x 1e-5: 41012345 = 41.012345 deg,
 * 29098765 = 29.098765 deg. */
static const uint8_t GOLDEN[36] = {
    0xA5, 0x02, 0x34, 0x12, 0x44, 0x33, 0x22, 0x11, 0x03, 0x00,
    0x00, 0x00, 0xE2, 0x04, 0x24, 0xFA, 0x09, 0x00, 0x79, 0xCC,
    0x71, 0x02, 0x0D, 0x03, 0xBC, 0x01, 0xDA, 0x2F, 0x0B, 0x7B,
    0x00, 0x64, 0x00, 0xCE, 0xFF, 0xEC};

/* Independent additive checksum, deliberately NOT calling lora_crc(). */
static uint8_t ref_crc(const uint8_t *d, size_t n)
{
  unsigned s = 0;
  for (size_t i = 0; i < n; i++)
  {
    s = (s + d[i]) & 0xFFu;
  }
  return (uint8_t)s;
}

/* Raw telemetry, values chosen so the float->int conversions are exact. */
static lora_tlm_t tlm(void)
{
  lora_tlm_t t;
  t.flight_state = 4u; /* FLIGHT_APOGEE_CONFIRMED */
  t.ek7_mode = 0u;
  t.alt_valid = true;
  t.alt_m = 1250.0f;   /* -> 1250 m   (exact in binary32) */
  t.vel_valid = true;
  t.vel_mps = -15.0f;  /* -> -1500 cm/s (exact)            */
  t.gps_fix = true;
  t.sats = 9u;
  t.lat_e5 = 41012345;
  t.lon_e5 = 29098765;
  t.batt_mv = 12250u;
  t.flags = LORA_F_GPS_FIX;
  t.tilt_valid = true;
  t.tilt_deg = 12.3f;   /* -> 123 decidegree (tam ikili karsilikli) */
  t.lean_x = 0.1f;      /* -> 100 x1000 */
  t.lean_y = -0.05f;    /* -> -50 x1000 */
  return t;
}

int main(void)
{
  uint8_t f[64];

  /* ================= golden vector ================= */
  {
    lora_frame_t in = sample();
    memset(f, 0xEE, sizeof f); /* poison: nothing may be left unwritten */
    CHECK(lora_build(f, sizeof f, &in) == LORA_FRAME_LEN);
    CHECK(memcmp(f, GOLDEN, LORA_FRAME_LEN) == 0);
    /* Byte-at-a-time offsets, so a failure names the exact field. */
    CHECK(f[0] == 0xA5u);                    /* magic   */
    CHECK(f[1] == 0x02u);                    /* ver     */
    CHECK(f[2] == 0x34u && f[3] == 0x12u);            /* seq LE      */
    CHECK(f[4] == 0x44u && f[5] == 0x33u);            /* t_ms LE     */
    CHECK(f[6] == 0x22u && f[7] == 0x11u);
    CHECK(f[8] == 0x03u && f[9] == 0x00u);            /* fstate LE   */
    CHECK(f[10] == 0x00u && f[11] == 0x00u);          /* ek7_mode LE */
    CHECK(f[12] == 0xE2u && f[13] == 0x04u);          /* alt_m LE    */
    CHECK(f[14] == 0x24u && f[15] == 0xFAu);          /* vel_cms LE  */
    CHECK(f[16] == 0x09u && f[17] == 0x00u);          /* sats LE     */
    CHECK(f[18] == 0x79u && f[19] == 0xCCu);          /* lat_e5 LE   */
    CHECK(f[20] == 0x71u && f[21] == 0x02u);
    CHECK(f[22] == 0x0Du && f[23] == 0x03u);          /* lon_e5 LE   */
    CHECK(f[24] == 0xBCu && f[25] == 0x01u);
    CHECK(f[26] == 0xDAu && f[27] == 0x2Fu);          /* batt_mv LE  */
    CHECK(f[28] == 0x0Bu);                            /* flags       */
    CHECK(f[29] == 0x7Bu && f[30] == 0x00u);          /* tilt 123 LE */
    CHECK(f[31] == 0x64u && f[32] == 0x00u);          /* lean_x 100  */
    CHECK(f[33] == 0xCEu && f[34] == 0xFFu);          /* lean_y -50  */
    CHECK(f[35] == 0xECu);                            /* crc         */
  }

  /* ================= CRC ================= */
  {
    lora_frame_t in = sample();
    CHECK(lora_build(f, sizeof f, &in) == LORA_FRAME_LEN);
    /* Builder's CRC matches an independent sum over bytes 0..34... */
    CHECK(lora_crc(f, LORA_CRC_OFF) == f[LORA_OFF_CRC]);
    CHECK(ref_crc(f, LORA_CRC_OFF) == f[LORA_OFF_CRC]);
    /* ...and re-verifying is a zero residual. */
    CHECK((uint8_t)(lora_crc(f, LORA_CRC_OFF) - f[LORA_OFF_CRC]) == 0u);
    CHECK(lora_crc(f, 0u) == 0u);
    CHECK(lora_crc(NULL, 8) == 0u);
    CHECK(lora_verify(f, LORA_FRAME_LEN));
    CHECK(LORA_CRC_OFF == 35u && LORA_FRAME_LEN == 36u);
    /* Magic / version / length rejection. */
    uint8_t bad[36];
    memcpy(bad, f, 36);
    bad[LORA_OFF_MAGIC] ^= 0xFFu;
    CHECK(!lora_verify(bad, 36));
    memcpy(bad, f, 36);
    bad[LORA_OFF_VER] ^= 0x02u;
    CHECK(!lora_verify(bad, 36));
    CHECK(!lora_verify(f, 35) && !lora_verify(f, 37));
    CHECK(!lora_verify(NULL, LORA_FRAME_LEN));
    /* A single flipped bit anywhere in bytes 0..34 must be caught: the
     * additive sum changes by a non-zero amount, so the CRC cannot match. */
    int caught = 0;
    for (size_t i = 0; i < LORA_CRC_OFF; i++)
    {
      memcpy(bad, f, 36);
      bad[i] ^= 0x01u;
      if (!lora_verify(bad, 36))
      {
        caught++;
      }
    }
    CHECK(caught == (int)LORA_CRC_OFF);
  }

  /* ================= bounds: no write past byte 35 ================= */
  {
    lora_frame_t in = sample();
    memset(f, 0xEE, sizeof f);
    CHECK(lora_build(f, sizeof f, &in) == LORA_FRAME_LEN);
    CHECK(LORA_FRAME_LEN == 36u);
    for (size_t i = LORA_FRAME_LEN; i < sizeof f; i++)
    {
      CHECK(f[i] == 0xEEu); /* the guard byte survived */
    }
    /* Exact-size heap buffer: under ASan a single byte past 35 would abort,
     * so this is the hard proof that the write is bounded by LORA_FRAME_LEN. */
    {
      uint8_t *exact = (uint8_t *)malloc(LORA_FRAME_LEN);
      assert(exact != NULL);
      memset(exact, 0xEE, LORA_FRAME_LEN);
      CHECK(lora_build(exact, LORA_FRAME_LEN, &in) == LORA_FRAME_LEN);
      CHECK(memcmp(exact, GOLDEN, LORA_FRAME_LEN) == 0);
      free(exact);
    }
    /* No unused gap: every byte 0..35 is accounted for by a field. */
    CHECK(LORA_OFF_MAGIC == 0u && LORA_OFF_VER == 1u && LORA_OFF_SEQ == 2u &&
          LORA_OFF_T_MS == 4u && LORA_OFF_FLIGHT_STATE == 8u &&
          LORA_OFF_EK7_MODE == 10u && LORA_OFF_ALT_M == 12u &&
          LORA_OFF_VEL_CMS == 14u && LORA_OFF_SATS == 16u &&
          LORA_OFF_LAT_E5 == 18u && LORA_OFF_LON_E5 == 22u &&
          LORA_OFF_BATT_MV == 26u && LORA_OFF_FLAGS == 28u &&
          LORA_OFF_TILT == 29u && LORA_OFF_LEANX == 31u &&
          LORA_OFF_LEANY == 33u && LORA_OFF_CRC == 35u);
  }

  /* ================= argument guards ================= */
  {
    lora_frame_t in = sample();
    memset(f, 0xEE, sizeof f);
    CHECK(lora_build(f, LORA_FRAME_LEN - 1u, &in) == 0u); /* cap too small */
    CHECK(lora_build(f, 0u, &in) == 0u);
    CHECK(lora_build(NULL, 64u, &in) == 0u);
    CHECK(lora_build(f, 64u, NULL) == 0u);
    for (size_t i = 0; i < sizeof f; i++)
    {
      CHECK(f[i] == 0xEEu); /* a refused build must not touch the buffer */
    }
    CHECK(lora_build(f, LORA_FRAME_LEN, &in) == LORA_FRAME_LEN); /* exact ok */
    lora_frame_t out;
    CHECK(lora_parse(f, LORA_FRAME_LEN, NULL) == false);
    CHECK(lora_parse(NULL, LORA_FRAME_LEN, &out) == false);
    CHECK(!lora_parse(f, LORA_FRAME_LEN - 1u, &out));
  }

  /* ================= build/parse round trip ================= */
  {
    lora_frame_t in = sample();
    lora_frame_t out;
    memset(&out, 0, sizeof out);
    CHECK(lora_build(f, sizeof f, &in) == LORA_FRAME_LEN);
    CHECK(lora_parse(f, LORA_FRAME_LEN, &out));
    CHECK(out.seq == in.seq && out.t_ms == in.t_ms);
    CHECK(out.flight_state == in.flight_state && out.ek7_mode == in.ek7_mode);
    CHECK(out.alt_m == in.alt_m && out.vel_cms == in.vel_cms);
    CHECK(out.sats == in.sats);
    CHECK(out.lat_e5 == in.lat_e5 && out.lon_e5 == in.lon_e5);
    CHECK(out.batt_mv == in.batt_mv && out.flags == in.flags);
    CHECK(out.tilt_ddeg == in.tilt_ddeg);
    CHECK(out.lean_x == in.lean_x && out.lean_y == in.lean_y);
  }

  /* ================= invalid sentinels survive a round trip ============
   * sats is deliberately NOT fix-gated: the station watches the count climb
   * before the first fix. Only lat/lon are fix-gated (0/0 = no position). */
  {
    lora_tlm_t t = tlm();
    lora_frame_t fr, out;
    t.alt_valid = false;
    t.vel_valid = false;
    t.tilt_valid = false;
    t.gps_fix = false;
    t.batt_mv = LORA_U16_NOBATT;
    t.flags = 0u;
    lora_frame_from_tlm(&fr, 7u, 12345u, &t);
    CHECK(fr.alt_m == LORA_ALT_INVALID);
    CHECK(fr.vel_cms == LORA_I16_INVALID);
    CHECK(fr.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(fr.lean_x == 0 && fr.lean_y == 0);
    CHECK(fr.sats == 9u); /* tlm() count passes straight through, no fix */
    CHECK(fr.lat_e5 == 0 && fr.lon_e5 == 0);
    CHECK(fr.batt_mv == 0u);
    memset(&out, 0, sizeof out);
    CHECK(lora_build(f, sizeof f, &fr) == LORA_FRAME_LEN);
    CHECK(lora_parse(f, LORA_FRAME_LEN, &out));
    CHECK(out.alt_m == LORA_ALT_INVALID);
    CHECK(out.vel_cms == LORA_I16_INVALID);
    CHECK(out.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(out.lean_x == 0 && out.lean_y == 0);
    CHECK(out.sats == 9u);
    CHECK(out.lat_e5 == 0 && out.lon_e5 == 0);
    CHECK(out.batt_mv == 0u);
    CHECK(out.seq == 7u && out.t_ms == 12345u);
    /* The sentinels are visible on the wire at the documented offsets. */
    CHECK(f[LORA_OFF_ALT_M] == 0x00u && f[LORA_OFF_ALT_M + 1u] == 0x80u);
    CHECK(f[LORA_OFF_VEL_CMS] == 0x00u && f[LORA_OFF_VEL_CMS + 1u] == 0x80u);
    CHECK(f[LORA_OFF_SATS] == 0x09u && f[LORA_OFF_SATS + 1u] == 0x00u);
    CHECK(f[LORA_OFF_LAT_E5] == 0x00u && f[LORA_OFF_LON_E5] == 0x00u);
    CHECK(f[LORA_OFF_BATT_MV] == 0x00u && f[LORA_OFF_FLAGS] == 0x00u);
    /* Egiklik gecersiz: telde 0xFFFF + sifir yon. */
    CHECK(f[LORA_OFF_TILT] == 0xFFu && f[LORA_OFF_TILT + 1u] == 0xFFu);
    CHECK(f[LORA_OFF_LEANX] == 0x00u && f[LORA_OFF_LEANX + 1u] == 0x00u);
    CHECK(f[LORA_OFF_LEANY] == 0x00u && f[LORA_OFF_LEANY + 1u] == 0x00u);
    CHECK(lora_verify(f, LORA_FRAME_LEN));
    /* A stale last-known position must NOT ride along with a dead fix,
     * but the live satellite count still does. */
    t.sats = 12u;
    t.lat_e5 = 41012345;
    t.lon_e5 = 29098765;
    lora_frame_from_tlm(&fr, 8u, 0u, &t);
    CHECK(fr.sats == 12u && fr.lat_e5 == 0 && fr.lon_e5 == 0);
  }

  /* ================= FLIGHT-CRITICAL: altitude round trip ============
   * The frame carried int16 CENTIMETRES, which saturates at +/-327.67 m: a
   * real 1250 m apogee came out of the ground station as 327.67 m. These are
   * the regression tests for that: the production apogee (1250 m) and a
   * high-power 3000 m must both survive lora_frame_from_tlm -> lora_build ->
   * lora_parse EXACTLY, with no scaling, no saturation and no wrap. */
  {
    const float real_alts_m[] = {1250.0f, 3000.0f};
    for (unsigned i = 0; i < 2u; i++)
    {
      lora_tlm_t t = tlm();
      lora_frame_t fr, out;
      t.alt_valid = true;
      t.alt_m = real_alts_m[i];
      lora_frame_from_tlm(&fr, 11u, 76543u, &t);
      /* 1: no truncation, no saturation, no sentinel. */
      CHECK(fr.alt_m == (int16_t)real_alts_m[i]);
      CHECK(fr.alt_m != LORA_ALT_INVALID);
      CHECK(fr.alt_m == 1250 || fr.alt_m == 3000);
      memset(f, 0xEE, sizeof f);
      CHECK(lora_build(f, sizeof f, &fr) == LORA_FRAME_LEN);
      CHECK(lora_verify(f, LORA_FRAME_LEN));
      /* 2: the exact expected little-endian bytes for the whole-metre field. */
      uint16_t u = (uint16_t)real_alts_m[i];
      CHECK(f[LORA_OFF_ALT_M] == (uint8_t)(u & 0xFFu));
      CHECK(f[LORA_OFF_ALT_M + 1u] == (uint8_t)((u >> 8) & 0xFFu));
      /* 3: the ground station's read-back is exact. */
      memset(&out, 0, sizeof out);
      CHECK(lora_parse(f, LORA_FRAME_LEN, &out));
      CHECK(out.alt_m == (int16_t)real_alts_m[i]);
      /* 4: through the real driver path too (poll + parse). The driver
       * emits the air image (fixed header + core when LORA_FIXED_HDR);
       * the ground station strips the 3 header bytes before parsing. */
      lora_init(0);
      CHECK(lora_poll(0u, &t, f, sizeof f));
#if LORA_FIXED_HDR
      CHECK(f[0] == LORA_FIXED_HDR_0 && f[1] == LORA_FIXED_HDR_1 &&
            f[2] == LORA_FIXED_HDR_2);
      memset(&out, 0, sizeof out);
      CHECK(lora_parse(f + 3u, LORA_FRAME_LEN, &out));
#else
      memset(&out, 0, sizeof out);
      CHECK(lora_parse(f, LORA_FRAME_LEN, &out));
#endif
      CHECK(out.alt_m == (int16_t)real_alts_m[i]);
      CHECK(out.alt_m != LORA_ALT_INVALID);
      CHECK(out.seq == 0u && out.t_ms == 0u);
      /* 5: a 20 km synthetic ceiling is still inside the range. */
      t.alt_m = 20000.0f;
      lora_frame_from_tlm(&fr, 12u, 0u, &t);
      CHECK(fr.alt_m == 20000);
    }
  }

  /* ================= unit conversion and saturation ================= */
  {
    lora_tlm_t t = tlm();
    lora_frame_t fr;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == 1250);   /* 1250.0 m -> 1250 m    */
    CHECK(fr.vel_cms == -1500); /* -15.0 m/s -> -1500 cm/s */
    t.alt_m = -1250.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == -1250);
    /* Sub-metre resolution is rounded to the nearest metre, never truncated
     * low on a long climb. */
    t.alt_m = 1250.6f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == 1251);
    t.alt_m = -1250.6f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == -1251);
    t.alt_m = 1250.0f;
    t.vel_mps = 15.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.vel_cms == 1500);
    /* Doyurma YOK: hedef araliga sigmayan sonlu deger gozcuye duser
     * (kelepceleme sahte okuma uretirdi). Sinir 32767.0 gecerli, 40000
     * gozcu; ayni kural hizda 32768.0 cm/s esigiyle calisir. */
    t.alt_m = 32767.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == 32767 && fr.alt_m != LORA_ALT_INVALID);
    t.alt_m = 40000.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == LORA_ALT_INVALID);
    t.alt_m = -40000.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == LORA_ALT_INVALID);
    t.alt_m = 1250.0f;
    t.vel_mps = 327.0f; /* 32700 cm/s: aralikta, kesme aynen */
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.vel_cms == 32700);
    t.vel_mps = -900.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.vel_cms == LORA_I16_INVALID);
    t.vel_mps = 400.0f; /* 40000 cm/s: tasan sonlu deger gozcu */
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.vel_cms == LORA_I16_INVALID);
    /* NaN/inf hicbir daraltmaya giremez: dort alan da gozcuye duser.
     * inf sonlu karsilastirmalari gecer (buyuktur), isfinite kapida
     * yakalanir; bu testler tanimsiz davranisin yoklugunu kilitler. */
    t.alt_m = NAN;
    t.vel_mps = -15.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == LORA_ALT_INVALID);
    t.alt_m = INFINITY;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == LORA_ALT_INVALID);
    t.alt_m = -INFINITY;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == LORA_ALT_INVALID);
    t.alt_m = 1250.0f;
    t.vel_mps = NAN;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.vel_cms == LORA_I16_INVALID);
    t.vel_mps = INFINITY;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.vel_cms == LORA_I16_INVALID);
    /* x100 carpimi tasan hiz (sonlu giris, sonsuz ara deger) gozcu. */
    t.vel_mps = 4.0e36f; /* x100 = ~4e38 > FLT_MAX -> inf ara deger */
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.vel_cms == LORA_I16_INVALID);
    t.vel_mps = 15.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.vel_cms == 1500);
    lora_frame_from_tlm(NULL, 0u, 0u, &t);
    lora_frame_from_tlm(&fr, 0u, 0u, NULL);
  }

  /* ================= cadence, seq increment, tick wrap ================ */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0); /* 0 == HAL_OK on the host build */
    CHECK(lora_ready());
    CHECK(lora_seq() == 0u && lora_ok_count() == 0u &&
          lora_drop_count() == 0u);
    /* The first poll transmits immediately: the deadline starts at 0. */
    memset(tx, 0xEE, sizeof tx);
    CHECK(lora_poll(0u, &t, tx, sizeof tx));
    CHECK(lora_seq() == 1u && lora_ok_count() == 1u);
    CHECK(lora_verify(core_of(tx), LORA_FRAME_LEN));
    {
      lora_frame_t out;
      CHECK(lora_parse(core_of(tx), LORA_FRAME_LEN, &out));
      CHECK(out.seq == 0u); /* the first packet carries seq 0 */
      CHECK(out.t_ms == 0u);
      CHECK(out.flight_state == t.flight_state);
      CHECK(out.lat_e5 == t.lat_e5 && out.lon_e5 == t.lon_e5);
    }
    /* Too early: nothing goes out and the counters do not move. */
    CHECK(!lora_poll(100u, &t, tx, sizeof tx));
    CHECK(!lora_poll(LORA_SEND_INTERVAL_MS - 1u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 1u && lora_seq() == 1u);
    CHECK(lora_poll(LORA_SEND_INTERVAL_MS, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 2u && lora_seq() == 2u);
    {
      lora_frame_t out;
      CHECK(lora_parse(core_of(tx), LORA_FRAME_LEN, &out));
      CHECK(out.seq == 1u); /* seq increments by exactly one */
      CHECK(out.t_ms == LORA_SEND_INTERVAL_MS);
    }
    /* Cadence is a pure function of the flight state; pad and flight both
     * run 5 Hz (200 ms) by request. */
    CHECK(LORA_SEND_INTERVAL_MS == 200u);
    CHECK(LORA_SEND_INTERVAL_WAIT_MS == 200u);
    CHECK(lora_interval_ms((uint16_t)FLIGHT_WAIT_FOR_LAUNCH) == 200u);
    CHECK(lora_interval_ms((uint16_t)FLIGHT_ASCENT) == 200u);
    CHECK(lora_interval_ms((uint16_t)FLIGHT_FAULT) == 200u);
    CHECK(lora_interval_ms(0u) >= LORA_SEND_GAP_MS);
    CHECK(lora_interval_ms(0xFFFFu) >= LORA_SEND_GAP_MS);
    /* Clear the outstanding 200 ms deadline first, so the next gate is the
     * one armed by the pad interval. */
    CHECK(lora_poll(LORA_SEND_INTERVAL_MS + LORA_SEND_INTERVAL_MS, &t, tx,
                    sizeof tx));
    CHECK(lora_ok_count() == 3u);
    /* Absolute timeline so far, all in ASCENT (200 ms): sends at 0, 200 and
     * 400 -> the outstanding deadline is 600. On the pad the period is also
     * 200 ms. (The interval in force is the one read at the send that armed
     * the deadline, not one re-read afterwards.) */
    CHECK(lora_poll(600u, &t, tx, sizeof tx)); /* still ASCENT: next = 800 */
    CHECK(lora_ok_count() == 4u);
    CHECK(lora_poll(800u, &t, tx, sizeof tx)); /* still ASCENT: next = 1000 */
    CHECK(lora_ok_count() == 5u);
    t.flight_state = (uint16_t)FLIGHT_WAIT_FOR_LAUNCH;
    CHECK(lora_poll(1000u, &t, tx, sizeof tx)); /* pad: next = 1000 + 200 */
    CHECK(lora_ok_count() == 6u);
    CHECK(!lora_poll(1199u, &t, tx, sizeof tx)); /* 199 ms: too early */
    CHECK(lora_ok_count() == 6u);
    CHECK(lora_poll(1200u, &t, tx, sizeof tx)); /* 200 ms: due */
    CHECK(lora_ok_count() == 7u);
    t.flight_state = (uint16_t)FLIGHT_ASCENT;
  }

  /* ================= 200 s soak at ~1 kHz loop rate =================
   * Bounded by the counter, not by assertion spam: vade boot'taki 0'a
   * cipali 200 ms izgara uzerindedir (birikimli): ilk yoklama (1 ms)
   * hemen gonderir (vade 0 dolmustur), sonrasi 200, 400, ..., 200000
   * izgarasinda -> 1 + 1000 = 1001 paket, sifir drop, faz kaymasi yok. */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0); /* fresh 0 deadline: first send at tick 0 */
    /* 200001 ms of ticks at a 200 ms cadence: ilk gonderim 1 ms'de
     * (vade 0 hemen dolar), sonrasi 200, 400, ..., 200000 izgarasi. */
    uint32_t ok0 = lora_ok_count();
    for (uint32_t ms = 1u; ms <= 200000u; ms++)
    {
      (void)lora_poll(ms, &t, NULL, 0u); /* NULL buffer: internal fallback */
    }
    CHECK(lora_ok_count() - ok0 == 1001u); /* 1 + 200000 / 200 */
    CHECK(lora_drop_count() == 0u);
    /* The internal fallback buffer still produces a well-formed frame. */
    CHECK(lora_poll(200000u + LORA_SEND_INTERVAL_MS, &t, tx, sizeof tx));
    CHECK(lora_verify(core_of(tx), LORA_FRAME_LEN));
  }

  /* ================= seq wrap across 2^16 =================
   * 66000 packets: the counter wraps once (0xFFFF -> 0x0000) and the +1
   * relation must hold across the seam. Counters are accumulated inside the
   * loop and asserted once, so a failure names the loop, not a byte index. */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0);
    uint32_t now = 0u;
    uint32_t sent = 0u;
    uint16_t expect = 0u;
    uint16_t last = 0u;
    bool first = true;
    bool seam_ok = true;
    bool seq_ok = true;
    bool parse_ok = true;
    while (sent < 66000u)
    {
      if (!lora_poll(now, &t, tx, sizeof tx))
      {
        now++;
        continue;
      }
      lora_frame_t out;
      if (!lora_parse(core_of(tx), LORA_FRAME_LEN, &out))
      {
        parse_ok = false;
        break;
      }
      if (out.seq != expect)
      {
        seq_ok = false;
        break;
      }
      if (!first && (uint16_t)(last + 1u) != out.seq)
      {
        seam_ok = false;
        break;
      }
      last = out.seq;
      first = false;
      expect++;
      sent++;
      now += LORA_SEND_INTERVAL_MS;
    }
    CHECK(parse_ok && seq_ok && seam_ok);
    CHECK(sent == 66000u);
    CHECK(lora_ok_count() == 66000u);
    /* 66000 mod 65536 = 464: the next seq is 464, the last sent was 463. */
    CHECK(lora_seq() == 464u);
    CHECK(last == 463u);
    /* Still healthy after the wrap. */
    CHECK(lora_poll(now, &t, tx, sizeof tx));
    CHECK(lora_seq() == 465u);
  }

  /* ================= HAL tick wrap must not freeze the link =============
   * s_next_ms is compared with a signed difference, so a tick wrap past
   * 2^32 ms leaves the gate usable. A signed "now >= deadline" read froze
   * the GPS sequencer for exactly this reason. */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0);
    CHECK(lora_poll(1000u, &t, tx, sizeof tx)); /* deadline now 1200 */
    uint32_t ok0 = lora_ok_count();
    /* Jump to just before the wrap: the 200 ms gap has not elapsed. */
    CHECK(!lora_poll(0xFFFFFFF0u, &t, tx, sizeof tx));
    CHECK(!lora_poll(0x00000000u, &t, tx, sizeof tx)); /* post-wrap: not due */
    CHECK(lora_poll(0x000004B0u, &t, tx, sizeof tx)); /* 1200, due */
    CHECK(lora_ok_count() - ok0 == 1u);
    /* That send armed deadline 1400 (1200 + 200), also reached after the
     * wrap: one tick short must stay silent, the deadline itself must fire. */
    CHECK(!lora_poll(0x00000577u, &t, tx, sizeof tx)); /* 1399 */
    CHECK(!lora_poll(0x000004AFu, &t, tx, sizeof tx)); /* before 1200 */
    CHECK(lora_poll(0x00000578u, &t, tx, sizeof tx)); /* 1400, due */
  }

  /* ================= SIT/SUT gate (LORA_ALLOW_SIT_SUT == 0) ============= */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0);
    CHECK(LORA_ALLOW_SIT_SUT == 0u);
    CHECK(LORA_EK7MODE_NORMAL == 0u && LORA_EK7MODE_SIT == 1u &&
          LORA_EK7MODE_SUT == 2u);
    CHECK(lora_link_allowed(LORA_EK7MODE_NORMAL));
    CHECK(!lora_link_allowed(LORA_EK7MODE_SIT));
    CHECK(!lora_link_allowed(LORA_EK7MODE_SUT));
    /* 5 s of aggressive polling in SIT must put nothing on the air and must
     * not consume the sequence number either. */
    t.ek7_mode = LORA_EK7MODE_SIT;
    uint32_t now = 0u;
    for (uint32_t i = 0; i < 5000u; i++)
    {
      now += 10u;
      CHECK(!lora_poll(now, &t, tx, sizeof tx));
    }
    CHECK(lora_ok_count() == 0u && lora_seq() == 0u);
    CHECK(lora_drop_count() == 0u); /* gated, not failed */
    t.ek7_mode = LORA_EK7MODE_SUT;
    CHECK(!lora_poll(now + 5000u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 0u);
    /* Back to NORMAL the link resumes on the normal cadence. */
    t.ek7_mode = LORA_EK7MODE_NORMAL;
    CHECK(lora_poll(now + LORA_SEND_INTERVAL_MS, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 1u && lora_seq() == 1u);
  }

  /* ================= init failure is not a fault ======================
   * No fault, no exception, no state change: the link just stays closed. */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(1); /* non-zero == HAL_ERROR / HAL_BUSY */
    CHECK(!lora_ready());
    for (uint32_t i = 0; i < 1000u; i++)
    {
      CHECK(!lora_poll(i * 1000u, &t, tx, sizeof tx));
    }
    CHECK(lora_ok_count() == 0u && lora_drop_count() == 0u &&
          lora_seq() == 0u);
    lora_init(0); /* init is the only gate */
    CHECK(lora_ready() && lora_poll(0u, &t, tx, sizeof tx));
  }

  /* ================= NULL / bad-argument safety ======================= */
  {
    lora_init(0);
    CHECK(!lora_poll(0u, NULL, NULL, 0u));
    CHECK(!lora_verify(NULL, LORA_FRAME_LEN));
  }

  /* ================= constants pinned against the real enums =========== */
  {
    /* lora.c restates FLIGHT_WAIT_FOR_LAUNCH numerically so it can stay free
     * of flight.h; this is what proves the restatement is right. */
    CHECK((uint16_t)FLIGHT_WAIT_FOR_LAUNCH == 2u);
    CHECK((uint16_t)FLIGHT_ASCENT == 3u);
    CHECK((uint16_t)FLIGHT_APOGEE_CONFIRMED == 4u);
    CHECK((uint16_t)FLIGHT_FAULT == 10u);
    CHECK(LORA_MAGIC == 0xA5u && LORA_VER == 0x02u);
    CHECK(LORA_I16_INVALID == (int16_t)-32768);
    /* Altitude sentinel: 0x8000 on the wire, outside the real -32767..+32767
     * metre range so it can only ever mean "no altitude". */
    CHECK(LORA_ALT_INVALID == (int16_t)0x8000);
    CHECK(LORA_ALT_INVALID < (int16_t)-32767);
    /* Egiklik gozcu: 0xFFFF telde, gecerli aralik 0..1800 decidegree. */
    CHECK(LORA_TILT_INVALID == 0xFFFFu);
    CHECK(LORA_U16_NOSATS == 0xFFFFu && LORA_U16_NOBATT == 0u);
    CHECK(LORA_F_GPS_FIX == 1u && LORA_F_BLUE_LED == 2u &&
          LORA_F_YELLOW_LED == 4u && LORA_F_RED_LED == 8u);
    /* Every transmit is bounded by 50 ms: a lost packet cannot cost more. */
    CHECK(LORA_TX_TMO_MS <= 50u);
    /* 36 bytes at 9600 8N1 = 360 bit times = 37.5 ms of air time; the
     * cadence clears that many times over, and the 400-byte module buffer
     * drains in ~417 ms at 960 B/s. */
    CHECK(LORA_SEND_GAP_MS > 31u);
    CHECK(LORA_SEND_INTERVAL_MS >= 2u * LORA_SEND_GAP_MS);
    CHECK(LORA_SEND_INTERVAL_WAIT_MS >= LORA_SEND_INTERVAL_MS);
  }

  /* ================= fixed-header air image =========================== */
  {
    /* A peer stuck in fixed-transmission mode (REG3 bit6) drops headerless
     * packets; LORA_FIXED_HDR prepends its {ADDH, ADDL, CH} so the address
     * filter accepts them. Core frame + CRC untouched. */
    lora_frame_t f;
    uint8_t core[LORA_FRAME_LEN];
    uint8_t air[LORA_AIR_LEN + 4];
    lora_tlm_t t = tlm();
    lora_frame_from_tlm(&f, 0x1234u, 0x11223344u, &t);
    CHECK(lora_build(core, sizeof core, &f) == LORA_FRAME_LEN);
    CHECK(lora_build_air(air, sizeof air, &f) == LORA_AIR_LEN);
    CHECK(LORA_AIR_LEN == LORA_FRAME_LEN + 3u);
    CHECK(air[0] == LORA_FIXED_HDR_0 && air[1] == LORA_FIXED_HDR_1 &&
          air[2] == LORA_FIXED_HDR_2);
    CHECK(memcmp(air + 3u, core, LORA_FRAME_LEN) == 0);
    /* Core still verifies standalone after stripping the header. */
    CHECK(lora_verify(air + 3u, LORA_FRAME_LEN));
    CHECK(lora_parse(air + 3u, LORA_FRAME_LEN, &f) && f.seq == 0x1234u);
    /* Guards. */
    CHECK(lora_build_air(NULL, sizeof air, &f) == 0u);
    CHECK(lora_build_air(air, LORA_AIR_LEN - 1u, &f) == 0u);
    CHECK(lora_build_air(air, sizeof air, NULL) == 0u);
  }

  /* ================= legacy v1 frames must FAIL v2 parse ============ */
  {
    /* 30 baytlik v1 cerceve (ver 0x01): uzunluk kapisinda reddedilir. */
    uint8_t v1[30];
    memcpy(v1, GOLDEN, 30);
    v1[1] = 0x01u;
    {
      unsigned s = 0;
      for (size_t i = 0; i < 29u; i++)
      {
        s = (s + v1[i]) & 0xFFu;
      }
      v1[29] = (uint8_t)s;
    }
    lora_frame_t out;
    CHECK(!lora_verify(v1, 30));
    CHECK(!lora_parse(v1, 30, &out));
    /* 30 bayt v2 CRC ile bile olsa uzunluk tutmaz. */
    CHECK(!lora_verify(GOLDEN, 30));
    CHECK(!lora_parse(GOLDEN, 30, &out));
  }

  /* ================= tilt/lean donusumleri ================= */
  {
    lora_tlm_t t = tlm();
    lora_frame_t fr, out;
    /* Gecerli egiklik: derece -> decidegree yuvarlama. */
    t.tilt_valid = true;
    t.tilt_deg = 12.3f;
    t.lean_x = 0.1f;
    t.lean_y = -0.05f;
    lora_frame_from_tlm(&fr, 21u, 999u, &t);
    CHECK(fr.tilt_ddeg == 123u);
    CHECK(fr.lean_x == 100 && fr.lean_y == -50);
    CHECK(lora_build(f, sizeof f, &fr) == LORA_FRAME_LEN);
    CHECK(lora_verify(f, LORA_FRAME_LEN));
    memset(&out, 0, sizeof out);
    CHECK(lora_parse(f, LORA_FRAME_LEN, &out));
    CHECK(out.tilt_ddeg == 123u && out.lean_x == 100 && out.lean_y == -50);
    /* Gecersiz -> 0xFFFF + sifir yon. */
    t.tilt_valid = false;
    lora_frame_from_tlm(&fr, 22u, 0u, &t);
    CHECK(fr.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(fr.lean_x == 0 && fr.lean_y == 0);
    /* Menzil-disi sonlu aci gozcuye duser (kelepceleme YOK): 180 derecenin
     * ustu fiziksel olarak olculemez, bozuk okumadir; negatif aci ayni. */
    t.tilt_valid = true;
    t.tilt_deg = 250.0f;
    t.lean_x = 5.0f;
    t.lean_y = -5.0f;
    lora_frame_from_tlm(&fr, 23u, 0u, &t);
    CHECK(fr.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(fr.lean_x == 0 && fr.lean_y == 0);
    /* Negatif aci gozcu (sifira kelepceleme YOK). */
    t.tilt_deg = -3.0f;
    t.lean_x = 0.0f;
    t.lean_y = 0.0f;
    lora_frame_from_tlm(&fr, 24u, 0u, &t);
    CHECK(fr.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(fr.lean_x == 0 && fr.lean_y == 0);
    /* Sinir degerler gecerli: 0 ve 180 derece ayni yuvarlamayla paketlenir. */
    t.tilt_deg = 0.0f;
    lora_frame_from_tlm(&fr, 25u, 0u, &t);
    CHECK(fr.tilt_ddeg == 0u);
    t.tilt_deg = 180.0f;
    t.lean_x = 1.0f;
    t.lean_y = -1.0f;
    lora_frame_from_tlm(&fr, 26u, 0u, &t);
    CHECK(fr.tilt_ddeg == 1800u);
    CHECK(fr.lean_x == 1000 && fr.lean_y == -1000);
    /* NaN/inf egiklik + yon: daraltmaya girmeden gozcu/sifir. */
    t.tilt_deg = NAN;
    t.lean_x = 0.1f;
    t.lean_y = -0.05f;
    lora_frame_from_tlm(&fr, 27u, 0u, &t);
    CHECK(fr.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(fr.lean_x == 0 && fr.lean_y == 0);
    t.tilt_deg = INFINITY;
    lora_frame_from_tlm(&fr, 28u, 0u, &t);
    CHECK(fr.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(fr.lean_x == 0 && fr.lean_y == 0);
    /* Carpimi tasan aci (sonlu giris, sonsuz decidegree) gozcu. */
    t.tilt_deg = 4.0e37f; /* x10 -> inf */
    lora_frame_from_tlm(&fr, 29u, 0u, &t);
    CHECK(fr.tilt_ddeg == LORA_TILT_INVALID);
    /* Gecerli egiklikte bozuk yon bileseni sifirlanir, saglam bilesen
     * korunur (eski yuvarlama aynen). */
    t.tilt_deg = 12.3f;
    t.lean_x = NAN;
    t.lean_y = 0.5f;
    lora_frame_from_tlm(&fr, 30u, 0u, &t);
    CHECK(fr.tilt_ddeg == 123u);
    CHECK(fr.lean_x == 0 && fr.lean_y == 500);
    t.lean_x = 5.0f; /* menzil-disi yon -> 0, obur bilesen saglam */
    t.lean_y = -0.05f;
    lora_frame_from_tlm(&fr, 31u, 0u, &t);
    CHECK(fr.tilt_ddeg == 123u);
    CHECK(fr.lean_x == 0 && fr.lean_y == -50);
    t.lean_x = INFINITY;
    t.lean_y = -INFINITY;
    lora_frame_from_tlm(&fr, 32u, 0u, &t);
    CHECK(fr.tilt_ddeg == 123u);
    CHECK(fr.lean_x == 0 && fr.lean_y == 0);
  }

  /* ================= birikimli vade: faz korunur, patlama yok ========
   * Geciken bir tur vadeyi kaydirmaz (vade bir periyot ileri alinir);
   * uzun duraklamada kacirilan kadans atlanir, tek gonderimle faz
   * yeniden kurulur. Eski "vade = simdi + periyot" davranisinda 350'deki
   * gonderim vadeyi 550'ye oteler ve 400'deki paket kacardi. */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0);
    CHECK(lora_poll(0u, &t, tx, sizeof tx));   /* vade 200 */
    CHECK(lora_ok_count() == 1u);
    CHECK(lora_poll(350u, &t, tx, sizeof tx)); /* gec gonderim, vade 400'de */
    CHECK(lora_ok_count() == 2u);
    CHECK(!lora_poll(399u, &t, tx, sizeof tx));
    CHECK(lora_poll(400u, &t, tx, sizeof tx)); /* faz korundu */
    CHECK(lora_ok_count() == 3u);
    CHECK(!lora_poll(599u, &t, tx, sizeof tx));
    CHECK(lora_poll(600u, &t, tx, sizeof tx)); /* vade 800 */
    CHECK(lora_ok_count() == 4u);
    /* 4.4 sn duraklama: tek paket, vade 5200'e kurulur, patlama yok. */
    uint32_t ok0 = lora_ok_count();
    CHECK(lora_poll(5000u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() - ok0 == 1u);
    CHECK(lora_drop_count() == 0u);
    CHECK(!lora_poll(5199u, &t, tx, sizeof tx));
    CHECK(lora_poll(5200u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() - ok0 == 2u);
  }

  /* ================= TX-hata ayrimi + re-init esigi (K) ==============
   * Hedefte HAL_UART_Transmit patlarsa s_txerr artar, s_drop (build) ayri
   * kalir; lora_drop_count() toplam dondurur (TUI uyumu). Ardisik K hata
   * re-init tetikler, basari sayaci kirar. Hostta HAL yok:
   * lora_host_inject_txerr() AYNI muhasebeyi surer; gercek poll yolu hep
   * basarili sayar (asagidaki ilk CHECK'ler bu davranisi kilitler). */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0);
    CHECK(lora_txerr_count() == 0u && lora_reinit_count() == 0u);
    /* Gercek poll yolu: hep ok, txerr/reinit kipirdamaz (davranis korunur). */
    CHECK(lora_poll(0u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 1u && lora_txerr_count() == 0u);
    CHECK(lora_reinit_count() == 0u && lora_drop_count() == 0u);
    /* K-1 enjeksiyon: re-init yok, txerr birikir, toplama yansir. */
    for (uint32_t i = 0u; i < LORA_TXERR_REINIT_K - 1u; i++)
    {
      lora_host_inject_txerr();
    }
    CHECK(lora_txerr_count() == LORA_TXERR_REINIT_K - 1u);
    CHECK(lora_reinit_count() == 0u);
    CHECK(lora_drop_count() == LORA_TXERR_REINIT_K - 1u); /* toplam = txerr */
    /* K'nci ardisik hata: tek re-init, ardisik sayac sifirlanir. */
    lora_host_inject_txerr();
    CHECK(lora_txerr_count() == LORA_TXERR_REINIT_K);
    CHECK(lora_reinit_count() == 1u);
    /* Esik kirildi: bir hata daha re-init DEGIL. */
    lora_host_inject_txerr();
    CHECK(lora_reinit_count() == 1u);
    CHECK(lora_txerr_count() == LORA_TXERR_REINIT_K + 1u);
    /* Basarili poll ardisik sayaci kirar: sonraki K-1 hata re-init yapmaz. */
    CHECK(lora_poll(LORA_SEND_INTERVAL_MS, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 2u);
    for (uint32_t i = 0u; i < LORA_TXERR_REINIT_K - 1u; i++)
    {
      lora_host_inject_txerr();
    }
    CHECK(lora_reinit_count() == 1u);
    /* Esik makul aralikta: parazit (1-2) tetiklemez, kalici ariza cabuk. */
    CHECK(LORA_TXERR_REINIT_K >= 3u && LORA_TXERR_REINIT_K <= 50u);
    /* lora_init yeni sayaclar dahil hepsini sifirlar. */
    lora_init(0);
    CHECK(lora_txerr_count() == 0u && lora_reinit_count() == 0u);
    CHECK(lora_ok_count() == 0u && lora_drop_count() == 0u &&
          lora_seq() == 0u);
  }

  /* ================= TX-oncesi AUX-atlama (airbusy) ==================
   * Vade dolmusken AUX dusukse (modul hâlâ havaya veriyor) paket
   * gonderilmez, bekleme YOK, vade aynen tuketilir (faz korunur). seq/ok/
   * drop/txerr kipirdamaz; yalniz s_airbusy +1. drop toplami (build + TX)
   * bozulmaz. Hostta gercek pin yerine lora_host_set_aux_low() ayni karari
   * surer; sahte veri yok, kadans ayni. */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0);
    CHECK(lora_airbusy_count() == 0u);
    /* Vade dolmusken AUX dusuk: atlama, sayac +1, gerisi sabit. */
    lora_host_set_aux_low(true);
    CHECK(!lora_poll(0u, &t, tx, sizeof tx));
    CHECK(lora_airbusy_count() == 1u);
    CHECK(lora_ok_count() == 0u && lora_seq() == 0u);
    CHECK(lora_drop_count() == 0u); /* drop'a karismadi */
    CHECK(lora_txerr_count() == 0u && lora_reinit_count() == 0u);
    /* Vade tuketildi: 200'den once sessiz (sayac bile kipirdamaz). */
    CHECK(!lora_poll(100u, &t, tx, sizeof tx));
    CHECK(lora_airbusy_count() == 1u);
    /* AUX bosalinca ayni fazdan devam: ilk basarili paket seq 0 tasir
     * (atlama seq tuketmedi). */
    lora_host_set_aux_low(false);
    CHECK(lora_poll(200u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 1u && lora_seq() == 1u);
    CHECK(lora_airbusy_count() == 1u);
    {
      lora_frame_t out;
      CHECK(lora_parse(core_of(tx), LORA_FRAME_LEN, &out));
      CHECK(out.seq == 0u);
    }
    /* Ust uste atlama: her vade-tuketimi +1, ok/seq/drop sabit. */
    lora_host_set_aux_low(true);
    CHECK(!lora_poll(400u, &t, tx, sizeof tx));
    CHECK(!lora_poll(600u, &t, tx, sizeof tx));
    CHECK(lora_airbusy_count() == 3u);
    CHECK(lora_ok_count() == 1u && lora_seq() == 1u);
    CHECK(lora_drop_count() == 0u);
    /* Kaldigi fazdan devam: vade 800'de dolar, 799 sessiz. */
    lora_host_set_aux_low(false);
    CHECK(!lora_poll(799u, &t, tx, sizeof tx));
    CHECK(lora_poll(800u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 2u && lora_seq() == 2u);
    /* lora_init airbusy'yi de sifirlar, enjeksiyonu temizler. */
    lora_init(0);
    CHECK(lora_airbusy_count() == 0u);
    CHECK(lora_poll(0u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 1u && lora_airbusy_count() == 0u);
  }

  /* ================= baud geri-donus karari (saf, HAL'siz) =============
   * Hedef cfg motoru da bu karari kullanir: HI (REG0=0xC5) tam eslesme ->
   * 57600 link; her baska durum (9600 imaji, bozuk imaj, NULL) -> 9600'de
   * kal (huart yazisiz degismez). Iki imaj yalniz REG0'da ayrilir: hava
   * hizi/guc/kanal bitleri bit-bit ayni (yasak bolgeye dokunulmadi).
   * Iki imajin hava bitleri de ayni ve 19,2 kbps'tir (101, Tablo 7-2). */
  {
    CHECK(LORA_LINK_BAUD == 57600u);
    CHECK(LORA_LINK_BAUD_FALLBACK == 9600u);
    CHECK(LORA_REG0_9600 == 0x65u && LORA_REG0_57600 == 0xC5u);
    /* Yalniz ust 3 bit (UART baud) degisir: alt 5 bit (parite + hava) ayni. */
    CHECK((LORA_REG0_57600 & 0x1Fu) == (LORA_REG0_9600 & 0x1Fu));
    /* Hava bitleri iki imajda da 101 = 19,2 kbps. */
    CHECK((LORA_REG0_9600 & LORA_REG0_AIR_MASK) == LORA_REG0_AIR_19K2_BITS);
    CHECK((LORA_REG0_57600 & LORA_REG0_AIR_MASK) == LORA_REG0_AIR_19K2_BITS);
    CHECK(LORA_CFG_EXPECTED[2] == LORA_REG0_9600);
    CHECK(LORA_CFG_EXPECTED_HI[2] == LORA_REG0_57600);
    for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
    {
      if (i == 2u)
      {
        continue; /* REG0: beklenen tek fark */
      }
      CHECK(LORA_CFG_EXPECTED_HI[i] == LORA_CFG_EXPECTED[i]);
    }
    /* Karar tablosu: HI -> 57600, gerisi -> 9600. */
    CHECK(lora_cfg_verified_baud(LORA_CFG_EXPECTED_HI) == LORA_LINK_BAUD);
    CHECK(lora_cfg_verified_baud(LORA_CFG_EXPECTED_HI) == 57600u);
    CHECK(lora_cfg_verified_baud(LORA_CFG_EXPECTED) ==
          LORA_LINK_BAUD_FALLBACK);
    {
      uint8_t bozuk[LORA_CFG_LEN];
      memcpy(bozuk, LORA_CFG_EXPECTED_HI, LORA_CFG_LEN);
      bozuk[2] ^= 0x01u; /* tek bit: ne HI ne 9600 imaji */
      CHECK(lora_cfg_verified_baud(bozuk) == LORA_LINK_BAUD_FALLBACK);
      CHECK(lora_cfg_verified_baud(bozuk) == 9600u);
    }
    CHECK(lora_cfg_verified_baud(NULL) == LORA_LINK_BAUD_FALLBACK);
    /* Varsayilan link 9600: dogrulama olmadan baud degismez. */
    lora_init(0);
    CHECK(lora_link_baud() == LORA_LINK_BAUD_FALLBACK);
    CHECK(lora_link_baud() == 9600u);
  }

  /* ================= imaj cozumu + RF tek-nokta (kilavuz Tablo 7-2/7-3) ==
   * Mevcut imaj 00 00 65 00 17 00 (fabrika 00 00 62 00 17 00, Tablo 7-3;
   * tek fark REG0 hava bitleri 010 -> 101):
   *   ADDH/ADDL = 0x0000, REG0 = 0x65, REG1 = 0x00, CH = 0x17, REG3 = 0x00.
   *   REG0 = 011_00_101: UART 011 = 9600, parite 00 = 8N1, hava 101 = 19,2k.
   *   REG1 = 00: alt-paket 200 B, ortam-RSSI kapali, guc 00 = 22 dBm (MAX).
   *   REG2 = 0x17 = 23: 850.125 + 23 = 873.125 MHz.
   *   REG3 = 00: seffaf iletim, RSSI bayti yok, LBT yok.
   * Guc ZATEN tavandadir (artirilamaz, 22 dBm). IKI modul de ayni havada
   * OLMALIDIR (19,2k): HI/LO yalniz UART bitlerinde ayrilir, asagida
   * kilitlenir. */
  {
    CHECK(LORA_CFG_LEN == 6u);
    {
      const uint8_t fabrika[6] = {0x00u, 0x00u, 0x62u, 0x00u, 0x17u, 0x00u};
      const uint8_t yeni[6] = {0x00u, 0x00u, 0x65u, 0x00u, 0x17u, 0x00u};
      CHECK(memcmp(LORA_CFG_EXPECTED, yeni, 6) == 0);
      /* Fabrikadan tek fark hava bitleri: 0x62 ^ 0x65 = 0x07 (bit2-0). */
      CHECK((fabrika[2] ^ LORA_CFG_EXPECTED[2]) == LORA_REG0_AIR_MASK);
      CHECK((LORA_CFG_EXPECTED[2] & LORA_REG0_AIR_MASK) ==
            LORA_REG0_AIR_19K2_BITS);
      CHECK(LORA_CFG_EXPECTED_HI[0] == 0x00u &&
            LORA_CFG_EXPECTED_HI[1] == 0x00u);
      CHECK(LORA_CFG_EXPECTED_HI[3] == 0x00u &&
            LORA_CFG_EXPECTED_HI[4] == 0x17u &&
            LORA_CFG_EXPECTED_HI[5] == 0x00u);
    }
    /* REG0 cozumu: iki imajda da parite/hava bitleri ayni, yalniz UART. */
    CHECK((LORA_REG0_9600 >> 5) == 0x03u);   /* 011 = 9600 */
    CHECK((LORA_REG0_57600 >> 5) == 0x06u);  /* 110 = 57600 */
    CHECK(((LORA_REG0_9600 >> 3) & 0x03u) == 0x00u); /* 8N1 */
    CHECK((LORA_REG0_9600 & LORA_REG0_AIR_MASK) == LORA_REG0_AIR_19K2_BITS);
    CHECK((LORA_REG0_57600 & LORA_REG0_AIR_MASK) == LORA_REG0_AIR_19K2_BITS);
    CHECK(LORA_AIR_RATE_BPS == 19200u);
    /* REG1 cozumu: guc bitleri 00 = 22 dBm = tavan (MAX). */
    CHECK((LORA_CFG_EXPECTED[3] & LORA_REG1_POWER_MASK) ==
          LORA_REG1_POWER_22DBM_BITS);
    CHECK(LORA_TX_POWER_DBM == 22u);
    CHECK((LORA_CFG_EXPECTED[3] & LORA_REG1_SUBPKT_MASK) ==
          LORA_REG1_SUBPKT_200B_BITS);
    CHECK(LORA_REG1_IMAGE == LORA_CFG_EXPECTED[3]);
    /* Kanal cozumu: 0x17 = 23 -> 850125 + 23*1000 = 873125 kHz. */
    CHECK(LORA_REG2_CH == 0x17u);
    CHECK(LORA_CFG_EXPECTED[4] == LORA_REG2_CH);
    CHECK(850125u + (uint32_t)LORA_REG2_CH * 1000u == 873125u);
    CHECK(LORA_REG3_TRANSPARENT_BITS == LORA_CFG_EXPECTED[5]);
    /* Hava suresi hesabi (HESAP, olcum degil): 39 baytlik hava imaji
     * 19,2 kbps'te 39*8 = 312 bit -> 312/19200 = 16,25 ms seri-esdeger;
     * preambul/baslikla ~25-35 ms. 200 ms kadansta doluluk %8-18, marj
     * ~165-175 ms: AUX her kapida BOSTA olmali, 5 Hz surdurulebilir.
     * Surdurulebilir ham hiz 39/0.2 = 195 B/s < 19200/8 = 2400 B/s kapasite. */
    CHECK(LORA_AIR_LEN == 39u);
    CHECK((39u * 8u * 1000u) / 19200u == 16u); /* seri-esdeger ms (16,25) */
    CHECK(LORA_SEND_INTERVAL_MS == 200u);
    CHECK((39u * 1000u) / LORA_SEND_INTERVAL_MS == 195u); /* B/s ham */
    CHECK((19200u / 8u) == 2400u);                        /* B/s kapasite */
  }

  /* ================= kesif-onarim karar cekirdegi (OLAY regresyonu) ======
   * OLAY: yankisi gorulmeyen C0/C2 yazisi modulde TUTMUS olabilir (RX kirikken
   * echo kaybolur). Motor "eski imajda kaldi" varsayip 9600'den yazmaya devam
   * ederse gercekte 57600'de kosan modul sagir kalir (link olu). Hedef cfg
   * motorundaki kesif (ADIM 1: yeni baud'da geri-okuma, ADIM 2: tarama tekrari,
   * ADIM 3: onar ya da bulunan baud'a uy) HAL'li oldugu icin hostta kosamaz;
   * ama kararin tamami saf yardimcilardadir (Src/lora.c ortak alan) ve bu blok
   * o karari kilitler: hedef motor + host ayni dogrulukta calisir. */
  {
    uint8_t hi[LORA_CFG_LEN];
    uint8_t lo[LORA_CFG_LEN];
    uint8_t bozuk[LORA_CFG_LEN];
    uint8_t fabrika[6] = {0x00u, 0x00u, 0x62u, 0x00u, 0x17u, 0x00u};
    memcpy(hi, LORA_CFG_EXPECTED_HI, LORA_CFG_LEN);
    memcpy(lo, LORA_CFG_EXPECTED, LORA_CFG_LEN);
    memcpy(bozuk, hi, LORA_CFG_LEN);
    bozuk[2] ^= 0x01u; /* tek bit: ne HI ne LO */
    /* Siniflar ayrik ve sabit. */
    CHECK(LORA_CFG_IMG_HI == 0u && LORA_CFG_IMG_LO == 1u);
    CHECK(LORA_CFG_IMG_OTHER == 2u && LORA_CFG_IMG_NONE == 3u);
    /* Siniflandirma: HI/LO/OTHER/NONE. */
    CHECK(lora_cfg_classify(hi) == LORA_CFG_IMG_HI);
    CHECK(lora_cfg_classify(lo) == LORA_CFG_IMG_LO);
    CHECK(lora_cfg_classify(bozuk) == LORA_CFG_IMG_OTHER);
    CHECK(lora_cfg_classify(fabrika) == LORA_CFG_IMG_OTHER); /* fabrika 2,4k */
    CHECK(lora_cfg_classify(NULL) == LORA_CFG_IMG_NONE);
    /* Tarama listesi: 8 baud bilinir, gerisi degil. */
    CHECK(lora_cfg_baud_known(9600u) && lora_cfg_baud_known(57600u));
    CHECK(lora_cfg_baud_known(115200u) && lora_cfg_baud_known(38400u));
    CHECK(lora_cfg_baud_known(19200u) && lora_cfg_baud_known(4800u));
    CHECK(lora_cfg_baud_known(2400u) && lora_cfg_baud_known(1200u));
    CHECK(!lora_cfg_baud_known(0u) && !lora_cfg_baud_known(12345u));
    CHECK(!lora_cfg_baud_known(9601u) && !lora_cfg_baud_known(115201u));
    /* Onarim ihtiyaci: yalniz HI temizdir. */
    CHECK(!lora_cfg_needs_repair(hi));
    CHECK(lora_cfg_needs_repair(lo));
    CHECK(lora_cfg_needs_repair(bozuk) && lora_cfg_needs_repair(NULL));
    /* OLAY senaryosu 1 (yanki-kaybi): HI yazisi tutmus, modul 57600/HI'da.
     * ADIM 1 geri-okumasi HI gorur -> link 57600 (onarimsiz FIXED). */
    CHECK(lora_cfg_found_link_baud(hi, 57600u) == 57600u);
    CHECK(lora_cfg_found_link_baud(hi, 57600u) == LORA_LINK_BAUD);
    /* Kanonik geri-donus: LO imaj 9600'de -> 9600. */
    CHECK(lora_cfg_found_link_baud(lo, 9600u) == 9600u);
    CHECK(lora_cfg_found_link_baud(lo, 9600u) == LORA_LINK_BAUD_FALLBACK);
    /* OLAY senaryosu 2 (uyum): modul 115200'de baska imajda, onarim tutmadi.
     * Kanitlanmis degisim kazanir -> huart 115200'e uyulur. */
    CHECK(lora_cfg_found_link_baud(bozuk, 115200u) == 115200u);
    CHECK(lora_cfg_found_link_baud(fabrika, 1200u) == 1200u);
    CHECK(lora_cfg_found_link_baud(NULL, 38400u) == 38400u);
    /* Kanit yoksa imaj konusmasi gecerlidir. */
    CHECK(lora_cfg_found_link_baud(hi, 0u) == 57600u);
    CHECK(lora_cfg_found_link_baud(lo, 0u) == 9600u);
    /* Cope suruklenen baud + bozuk imaj -> guvenli geri-donus. */
    CHECK(lora_cfg_found_link_baud(bozuk, 12345u) == 9600u);
    CHECK(lora_cfg_found_link_baud(NULL, 0u) == 9600u);
    CHECK(lora_cfg_found_link_baud(NULL, 0u) == LORA_LINK_BAUD_FALLBACK);
    /* Imaj UART bitleri baska soylese bile tele guvenilir: 57600 diyen imaj
     * 115200'de cevap verdiyse modul 115200 dinliyordur. */
    CHECK(lora_cfg_found_link_baud(hi, 115200u) == 115200u);
    CHECK(lora_cfg_found_link_baud(lo, 115200u) == 115200u);
  }

  /* Kor urtak donus: diyalog kanitliyken 8 baytlik 65-imaji kurulur
   * (UART 9600 + 19,2k hava + anahtar 0), kanitsizken tampona dokunulmaz. */
  {
    uint8_t govde[8u] = {0xAAu, 0xAAu, 0xAAu, 0xAAu,
                         0xAAu, 0xAAu, 0xAAu, 0xAAu};
    CHECK(lora_cfg_common_img(govde, true));
    CHECK(govde[0] == 0x00u && govde[1] == 0x00u && govde[2] == 0x65u);
    CHECK(govde[3] == 0x00u && govde[4] == 0x17u && govde[5] == 0x00u);
    CHECK(govde[6] == 0x00u && govde[7] == 0x00u);
    CHECK(memcmp(govde, LORA_CFG_COMMON, 6) == 0);
    govde[0] = 0xAAu;
    CHECK(!lora_cfg_common_img(govde, false));
    CHECK(govde[0] == 0xAAu); /* kanitsiz yazim tampona dokunmaz */
    CHECK(!lora_cfg_common_img(NULL, true));
  }

  /* ================= tarama-dayaniklilik karari (3 tur, AUX kapili) ======
   * Kanitli sorun: tek-gecis tarama bazen B=-- bos donuyor (marjinal Mode-3
   * girisi). Hedef en fazla 3 tur dener (tur arasi Mode-0 + 100 ms), ilk
   * tutan kazanir; her C1 oncesi AUX-high en fazla 300 ms beklenir, gelmezse
   * YINE DENENIR (bag siz AUX guvenligi). Bu blok saf karar yuvasini kilitler
   * (hedef dongu + host ayni karar). */
  {
    /* Tur sayisi sabittir: 3 (tek gecis YASAK, sinirsiz dongu YASAK). */
    CHECK(LORA_CFG_SCAN_ROUNDS == 3u);
    /* Kazanimda dur: tutan tur tekrarsiz kazanir. */
    CHECK(!lora_cfg_scan_retry(true, 0u));
    CHECK(!lora_cfg_scan_retry(true, 1u));
    CHECK(!lora_cfg_scan_retry(true, 2u));
    CHECK(!lora_cfg_scan_retry(true, 3u));
    /* Bos turlarda devam: 0/1/2 bitmis turdan sonra bir tur daha. */
    CHECK(lora_cfg_scan_retry(false, 0u));
    CHECK(lora_cfg_scan_retry(false, 1u));
    CHECK(lora_cfg_scan_retry(false, 2u));
    /* 3 tur dolunca birak: 4. tur YOK (sinirli boot suresi). */
    CHECK(!lora_cfg_scan_retry(false, 3u));
    /* Sinir-disi giris guvenli: tasmanin ustu de durur (takilma yok). */
    CHECK(!lora_cfg_scan_retry(false, 4u));
    CHECK(!lora_cfg_scan_retry(false, 255u));
  }

  /* ================= kosullu C0-kalicilik karari (flash omru) ============
   * Kanitli sorun: C2-RAM guc cevriminde ucar, modul bilinmeyen FLASH imajina
   * doner; C0-kalicilik hic denenmiyordu. Kural: SADECE ilk tarama okumasi
   * 65-imajindan FARKLIYSA ve C2-RAM yaz+dog rula BASARILI olduktan SONRA ayni
   * imaj bir kez C0 ile yazilip tekrar okunur (uzun settle: AUX 1500 ms +
   * 500 ms). Basarisizlik sessizce C2-RAM'e duser (link yolu ayni). Eslesen
   * imaja C0 YAZILMAZ (flash omru). 57600/C5 dali YOKTUR, huart hep 9600. */
  {
    uint8_t lo[LORA_CFG_LEN];
    uint8_t hi[LORA_CFG_LEN];
    uint8_t bozuk[LORA_CFG_LEN];
    uint8_t fabrika[6] = {0x00u, 0x00u, 0x62u, 0x00u, 0x17u, 0x00u};
    memcpy(lo, LORA_CFG_EXPECTED, LORA_CFG_LEN);
    memcpy(hi, LORA_CFG_EXPECTED_HI, LORA_CFG_LEN);
    memcpy(bozuk, hi, LORA_CFG_LEN);
    bozuk[2] ^= 0x01u; /* tek bit: ne HI ne 65-imaji */
    /* Adaylik: eslesen 65-imajina C0 YOK (flash omru). */
    CHECK(!lora_cfg_needs_persist(lo));
    /* Irasamis her imaj adaydir: HI/bozuk/fabrika (2,4k) hepsi farklidir. */
    CHECK(lora_cfg_needs_persist(hi));
    CHECK(lora_cfg_needs_persist(bozuk));
    CHECK(lora_cfg_needs_persist(fabrika));
    /* Okuma yoksa kanit yok: aday DEGIL (sagir modul isitilmaz). */
    CHECK(!lora_cfg_needs_persist(NULL));
    /* Kapi dogruluk tablosu: yalniz (irasamis + C2-basarili) yazar. */
    CHECK(lora_cfg_persist_now(true, true));
    CHECK(!lora_cfg_persist_now(true, false));  /* C2 tutmamissa C0 YOK */
    CHECK(!lora_cfg_persist_now(false, true));  /* eslesmisse C0 YOK */
    CHECK(!lora_cfg_persist_now(false, false));
    /* Uctan-uca karar: ilk okuma + C2-sonucu birlesimi. */
    CHECK(lora_cfg_persist_now(lora_cfg_needs_persist(hi), true));
    CHECK(!lora_cfg_persist_now(lora_cfg_needs_persist(lo), true));
    CHECK(!lora_cfg_persist_now(lora_cfg_needs_persist(hi), false));
    CHECK(!lora_cfg_persist_now(lora_cfg_needs_persist(NULL), true));
  }

  printf("HOST_LORA_TESTS_PASSED n=%d\n", n_pass);
  return 0;
}