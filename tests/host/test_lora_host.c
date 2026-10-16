/* Host tests for the LoRa telemetry link (Src/lora.c, HAL-independent).
 * Covers the exact byte layout, the additive CRC, the sequence counter and its
 * wrap, the invalid-value sentinels round-tripping, the argument guards, the
 * cadence/SIT-SUT gate, 0.1 m signed altitude, V1/V2 compatibility, and a
 * hand-derived V3 38-byte golden vector. */
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

/* Poll emits the air image: with LORA_FIXED_HDR the 38-byte V3 core sits
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
  f.alt_m = 1250;         /* compatibility view */
  f.alt_dm = 12500;       /* 1250.0 m       -> D4 30 00 00 */
  f.vel_cms = -1500;      /* -15.0 m/s      -> 24 FA */
  f.sats = 9u;            /* -> 09 00                */
  f.lat_e6 = 39933400;    /* 39.933400 deg -> D8 55 61 02 */
  f.lon_e6 = 32859700;    /* 32.859700 deg -> 34 66 F5 01 */
  f.batt_mv = 12250u;     /* 12250        -> DA 2F  */
  f.flags = LORA_F_GPS_FIX | LORA_F_BLUE_LED | LORA_F_RED_LED; /* 0B */
  f.tilt_ddeg = 123u;     /* 12.3 deg     -> 7B 00 */
  f.lean_x = 100;         /* 0.100        -> 64 00 */
  f.lean_y = -50;         /* -0.050       -> CE FF */
  return f;
}

/* V3 golden vector (38 bytes), independently derived by field and CRC.
 * 1250.0 m -> 12500 decimetres (0x000030D4 -> D4 30 00 00); signed velocity
 * -1500 cm/s -> 24 FA; the CRC is sum(bytes 0..36) modulo 256. */
static const uint8_t GOLDEN[38] = {
    0xA5, 0x03, 0x34, 0x12, 0x44, 0x33, 0x22, 0x11, 0x03, 0x00,
    0x00, 0x00, 0xD4, 0x30, 0x00, 0x00, 0x24, 0xFA, 0x09, 0x00,
    0xD8, 0x55, 0x61, 0x02, 0x34, 0x66, 0xF5, 0x01, 0xDA, 0x2F,
    0x0B, 0x7B, 0x00, 0x64, 0x00, 0xCE, 0xFF, 0xA6};

/* Frozen V2 golden image retained to prove decode compatibility. */
static const uint8_t GOLDEN_V2[36] = {
    0xA5, 0x02, 0x34, 0x12, 0x44, 0x33, 0x22, 0x11, 0x03, 0x00,
    0x00, 0x00, 0xE2, 0x04, 0x24, 0xFA, 0x09, 0x00, 0xD8, 0x55,
    0x61, 0x02, 0x34, 0x66, 0xF5, 0x01, 0xDA, 0x2F, 0x0B, 0x7B,
    0x00, 0x64, 0x00, 0xCE, 0xFF, 0x87};

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
  t.lat_e6 = 39933400;
  t.lon_e6 = 32859700;
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
    /* Byte-at-a-time V3 offsets, so failures name the exact field. */
    CHECK(f[0] == 0xA5u && f[1] == 0x03u);       /* magic/version */
    CHECK(f[2] == 0x34u && f[3] == 0x12u);       /* seq */
    CHECK(f[4] == 0x44u && f[7] == 0x11u);       /* tick */
    CHECK(f[8] == 0x03u && f[9] == 0x00u);       /* flight state */
    CHECK(f[10] == 0x00u && f[11] == 0x00u);     /* EK7 mode */
    CHECK(f[12] == 0xD4u && f[13] == 0x30u &&
          f[14] == 0x00u && f[15] == 0x00u);     /* 12500 dm */
    CHECK(f[16] == 0x24u && f[17] == 0xFAu);     /* velocity */
    CHECK(f[18] == 0x09u && f[19] == 0x00u);     /* satellites */
    CHECK(f[20] == 0xD8u && f[21] == 0x55u &&
          f[22] == 0x61u && f[23] == 0x02u);     /* latitude */
    CHECK(f[24] == 0x34u && f[25] == 0x66u &&
          f[26] == 0xF5u && f[27] == 0x01u);     /* longitude */
    CHECK(f[28] == 0xDAu && f[29] == 0x2Fu);     /* battery */
    CHECK(f[30] == 0x0Bu);                       /* flags */
    CHECK(f[31] == 0x7Bu && f[32] == 0x00u);     /* tilt */
    CHECK(f[33] == 0x64u && f[34] == 0x00u);     /* lean X */
    CHECK(f[35] == 0xCEu && f[36] == 0xFFu);     /* lean Y */
    CHECK(f[37] == 0xA6u);                       /* CRC */
  }

  /* ================= CRC ================= */
  {
    lora_frame_t in = sample();
    CHECK(lora_build(f, sizeof f, &in) == LORA_FRAME_LEN);
    /* Builder's CRC matches an independent sum over bytes 0..36... */
    CHECK(lora_crc(f, LORA_CRC_OFF) == f[LORA_OFF_CRC]);
    CHECK(ref_crc(f, LORA_CRC_OFF) == f[LORA_OFF_CRC]);
    /* ...and re-verifying is a zero residual. */
    CHECK((uint8_t)(lora_crc(f, LORA_CRC_OFF) - f[LORA_OFF_CRC]) == 0u);
    CHECK(lora_crc(f, 0u) == 0u);
    CHECK(lora_crc(NULL, 8) == 0u);
    CHECK(lora_verify(f, LORA_FRAME_LEN));
    CHECK(LORA_CRC_OFF == 37u && LORA_FRAME_LEN == 38u);
    /* Magic / version / length rejection. */
    uint8_t bad[LORA_FRAME_LEN];
    memcpy(bad, f, LORA_FRAME_LEN);
    bad[LORA_OFF_MAGIC] ^= 0xFFu;
    CHECK(!lora_verify(bad, LORA_FRAME_LEN));
    memcpy(bad, f, LORA_FRAME_LEN);
    bad[LORA_OFF_VER] ^= 0x02u;
    CHECK(!lora_verify(bad, LORA_FRAME_LEN));
    CHECK(!lora_verify(f, 37) && !lora_verify(f, 39));
    CHECK(!lora_verify(NULL, LORA_FRAME_LEN));
    /* A single flipped bit anywhere in bytes 0..36 must be caught: the
     * additive sum changes by a non-zero amount, so the CRC cannot match. */
    int caught = 0;
    for (size_t i = 0; i < LORA_CRC_OFF; i++)
    {
      memcpy(bad, f, LORA_FRAME_LEN);
      bad[i] ^= 0x01u;
      if (!lora_verify(bad, LORA_FRAME_LEN))
      {
        caught++;
      }
    }
    CHECK(caught == (int)LORA_CRC_OFF);
  }

  /* ================= bounds: no write past byte 37 ================= */
  {
    lora_frame_t in = sample();
    memset(f, 0xEE, sizeof f);
    CHECK(lora_build(f, sizeof f, &in) == LORA_FRAME_LEN);
    CHECK(LORA_FRAME_LEN == 38u);
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
    /* No unused gap: every byte 0..37 is accounted for by a field. */
    CHECK(LORA_OFF_MAGIC == 0u && LORA_OFF_VER == 1u && LORA_OFF_SEQ == 2u &&
          LORA_OFF_T_MS == 4u && LORA_OFF_FLIGHT_STATE == 8u &&
          LORA_OFF_EK7_MODE == 10u && LORA_OFF_ALT_DM == 12u &&
          LORA_OFF_VEL_CMS == 16u && LORA_OFF_SATS == 18u &&
          LORA_OFF_LAT_E6 == 20u && LORA_OFF_LON_E6 == 24u &&
          LORA_OFF_BATT_MV == 28u && LORA_OFF_FLAGS == 30u &&
          LORA_OFF_TILT == 31u && LORA_OFF_LEANX == 33u &&
          LORA_OFF_LEANY == 35u && LORA_OFF_CRC == 37u);
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
    CHECK(out.lat_e6 == in.lat_e6 && out.lon_e6 == in.lon_e6);
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
    CHECK(fr.lat_e6 == 0 && fr.lon_e6 == 0);
    CHECK(fr.batt_mv == 0u);
    memset(&out, 0, sizeof out);
    CHECK(lora_build(f, sizeof f, &fr) == LORA_FRAME_LEN);
    CHECK(lora_parse(f, LORA_FRAME_LEN, &out));
    CHECK(out.alt_m == LORA_ALT_INVALID);
    CHECK(out.vel_cms == LORA_I16_INVALID);
    CHECK(out.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(out.lean_x == 0 && out.lean_y == 0);
    CHECK(out.sats == 9u);
    CHECK(out.lat_e6 == 0 && out.lon_e6 == 0);
    CHECK(out.batt_mv == 0u);
    CHECK(out.seq == 7u && out.t_ms == 12345u);
    /* The sentinels are visible on the wire at the documented offsets. */
    CHECK(f[LORA_OFF_ALT_DM] == 0x00u && f[LORA_OFF_ALT_DM + 1u] == 0x00u &&
          f[LORA_OFF_ALT_DM + 2u] == 0x00u && f[LORA_OFF_ALT_DM + 3u] == 0x80u);
    CHECK(f[LORA_OFF_VEL_CMS] == 0x00u && f[LORA_OFF_VEL_CMS + 1u] == 0x80u);
    CHECK(f[LORA_OFF_SATS] == 0x09u && f[LORA_OFF_SATS + 1u] == 0x00u);
    CHECK(f[LORA_OFF_LAT_E6] == 0x00u && f[LORA_OFF_LON_E6] == 0x00u);
    CHECK(f[LORA_OFF_BATT_MV] == 0x00u && f[LORA_OFF_FLAGS] == 0x00u);
    /* Egiklik gecersiz: telde 0xFFFF + sifir yon. */
    CHECK(f[LORA_OFF_TILT] == 0xFFu && f[LORA_OFF_TILT + 1u] == 0xFFu);
    CHECK(f[LORA_OFF_LEANX] == 0x00u && f[LORA_OFF_LEANX + 1u] == 0x00u);
    CHECK(f[LORA_OFF_LEANY] == 0x00u && f[LORA_OFF_LEANY + 1u] == 0x00u);
    CHECK(lora_verify(f, LORA_FRAME_LEN));
    /* A stale last-known position must NOT ride along with a dead fix,
     * but the live satellite count still does. */
    t.sats = 12u;
    t.lat_e6 = 39933400;
    t.lon_e6 = 32859700;
    lora_frame_from_tlm(&fr, 8u, 0u, &t);
    CHECK(fr.sats == 12u && fr.lat_e6 == 0 && fr.lon_e6 == 0);
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
      /* 1: whole-metre compatibility field and V3 decimetres remain valid. */
      CHECK(fr.alt_m == (int16_t)real_alts_m[i]);
      CHECK(fr.alt_dm == (int32_t)(real_alts_m[i] * 10.0f));
      CHECK(fr.alt_m != LORA_ALT_INVALID);
      CHECK(fr.alt_m == 1250 || fr.alt_m == 3000);
      memset(f, 0xEE, sizeof f);
      CHECK(lora_build(f, sizeof f, &fr) == LORA_FRAME_LEN);
      CHECK(lora_verify(f, LORA_FRAME_LEN));
      /* 2: exact signed-decimetre little-endian field. */
      uint32_t u = (uint32_t)(real_alts_m[i] * 10.0f);
      CHECK(f[LORA_OFF_ALT_DM] == (uint8_t)(u & 0xFFu));
      CHECK(f[LORA_OFF_ALT_DM + 1u] == (uint8_t)((u >> 8) & 0xFFu));
      CHECK(f[LORA_OFF_ALT_DM + 2u] == (uint8_t)((u >> 16) & 0xFFu));
      CHECK(f[LORA_OFF_ALT_DM + 3u] == (uint8_t)((u >> 24) & 0xFFu));
      /* 3: the ground station's read-back is exact. */
      memset(&out, 0, sizeof out);
      CHECK(lora_parse(f, LORA_FRAME_LEN, &out));
      CHECK(out.alt_m == (int16_t)real_alts_m[i]);
      CHECK(out.alt_dm == (int32_t)(real_alts_m[i] * 10.0f));
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
      CHECK(out.alt_dm == (int32_t)(real_alts_m[i] * 10.0f));
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
    CHECK(fr.alt_m == -1250 && fr.alt_dm == -12500);
    t.alt_m = -0.4f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_dm == -4); /* negative below the pad is preserved */
    uint8_t frac[LORA_FRAME_LEN];
    lora_frame_t frac_out;
    CHECK(lora_build(frac, sizeof frac, &fr) == LORA_FRAME_LEN);
    CHECK(lora_parse(frac, sizeof frac, &frac_out));
    CHECK(frac_out.alt_dm == -4 && frac_out.alt_m == 0);
    t.alt_m = -1250.0f;
    /* Legacy whole-metre view rounds; new V3 wire value retains one decimal. */
    t.alt_m = 1250.6f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == 1251 && fr.alt_dm == 12506);
    t.alt_m = -1250.6f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == -1251 && fr.alt_dm == -12506);
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
    CHECK(fr.alt_m == LORA_ALT_INVALID && fr.alt_dm == 400000);
    t.alt_m = -40000.0f;
    lora_frame_from_tlm(&fr, 1u, 0u, &t);
    CHECK(fr.alt_m == LORA_ALT_INVALID && fr.alt_dm == -400000);
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
      CHECK(out.lat_e6 == t.lat_e6 && out.lon_e6 == t.lon_e6);
    }
    /* Just before the 100 ms deadline: no packet; the exact deadline sends. */
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
     * both target 10 Hz (100 ms). */
    CHECK(LORA_SEND_INTERVAL_MS == 100u);
    CHECK(LORA_SEND_INTERVAL_WAIT_MS == 100u);
    CHECK(lora_interval_ms((uint16_t)FLIGHT_WAIT_FOR_LAUNCH) == 100u);
    CHECK(lora_interval_ms((uint16_t)FLIGHT_ASCENT) == 100u);
    CHECK(lora_interval_ms((uint16_t)FLIGHT_FAULT) == 100u);
    CHECK(lora_interval_ms(0u) >= LORA_SEND_GAP_MS);
    CHECK(lora_interval_ms(0xFFFFu) >= LORA_SEND_GAP_MS);
    /* Clear the outstanding 100 ms deadline before switching the flight state. */
    CHECK(lora_poll(200u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 3u);
    /* 10 Hz absolute timeline: sends at 0, 100, 200; next deadline 300. */
    CHECK(lora_poll(300u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 4u);
    CHECK(lora_poll(400u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 5u);
    t.flight_state = (uint16_t)FLIGHT_WAIT_FOR_LAUNCH;
    CHECK(lora_poll(500u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 6u);
    CHECK(!lora_poll(599u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 6u);
    CHECK(lora_poll(600u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 7u);
    t.flight_state = (uint16_t)FLIGHT_ASCENT;
  }

  /* ================= 200 s soak at ~1 kHz loop rate =================
   * 100 ms cadence over 200 seconds: first packet immediately, then
   * every 100 ms through 200000 ms -> 2001 packets, zero drops. */
  {
    lora_tlm_t t = tlm();
    uint8_t tx[64];
    lora_init(0); /* fresh 0 deadline: first send at tick 0 */
    /* Dense 1 ms polling with a 100 ms cadence: packets at 1, 100, ..., 200000. */
    uint32_t ok0 = lora_ok_count();
    for (uint32_t ms = 1u; ms <= 200000u; ms++)
    {
      (void)lora_poll(ms, &t, NULL, 0u); /* NULL buffer: internal fallback */
    }
    CHECK(lora_ok_count() - ok0 == 2001u); /* 1 + 200000 / 200 */
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
    CHECK(lora_poll(1000u, &t, tx, sizeof tx)); /* deadline now 1100 */
    uint32_t ok0 = lora_ok_count();
    /* Jump across the wrap; one hundred milliseconds are required. */
    CHECK(!lora_poll(0xFFFFFFF0u, &t, tx, sizeof tx));
    CHECK(!lora_poll(0x00000000u, &t, tx, sizeof tx)); /* post-wrap: not due */
    CHECK(lora_poll(0x0000044Cu, &t, tx, sizeof tx)); /* 1100, due */
    CHECK(lora_ok_count() - ok0 == 1u);
    /* That send armed deadline 1200 (1100 + 100): 1199 is silent. */
    CHECK(!lora_poll(0x000004AFu, &t, tx, sizeof tx)); /* 1199 */
    CHECK(!lora_poll(0x0000044Bu, &t, tx, sizeof tx)); /* before 1100 */
    CHECK(lora_poll(0x000004B0u, &t, tx, sizeof tx)); /* 1200, due */
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
    CHECK(LORA_MAGIC == 0xA5u && LORA_VER == 0x03u);
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
    /* V3 air image is 41 bytes. UART feed time at 9600 8N1 is < 43 ms;
     * 100 ms leaves room for the RF transmission and receiver parsing. */
    CHECK(LORA_SEND_GAP_MS > 31u);
    CHECK(LORA_SEND_INTERVAL_MS >= LORA_SEND_GAP_MS);
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

  /* ================= backwards compatibility: V1/V2 remain readable === */
  {
    uint8_t v1[LORA_V1_FRAME_LEN];
    lora_frame_t out;
    memcpy(v1, GOLDEN_V2, sizeof v1);
    v1[1] = LORA_VER_V1;
    v1[29] = ref_crc(v1, 29u);
    CHECK(lora_verify(v1, sizeof v1));
    CHECK(lora_parse(v1, sizeof v1, &out));
    CHECK(out.alt_m == 1250 && out.alt_dm == 12500);
    CHECK(out.tilt_ddeg == LORA_TILT_INVALID);
    CHECK(!lora_verify(v1, sizeof v1 + 1u));
    CHECK(lora_verify(GOLDEN_V2, sizeof GOLDEN_V2));
    CHECK(lora_parse(GOLDEN_V2, sizeof GOLDEN_V2, &out));
    CHECK(out.alt_m == 1250 && out.alt_dm == 12500);
    CHECK(out.tilt_ddeg == 123u && out.lean_x == 100 && out.lean_y == -50);
    CHECK(!lora_verify(GOLDEN_V2, LORA_V1_FRAME_LEN));
    CHECK(!lora_parse(GOLDEN, LORA_V2_FRAME_LEN, &out));
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
    CHECK(lora_poll(0u, &t, tx, sizeof tx));   /* deadline 100 */
    CHECK(lora_ok_count() == 1u);
    CHECK(lora_poll(250u, &t, tx, sizeof tx)); /* late; missed slots skipped */
    CHECK(lora_ok_count() == 2u);
    CHECK(!lora_poll(349u, &t, tx, sizeof tx));
    CHECK(lora_poll(350u, &t, tx, sizeof tx)); /* phase reset only after long stall */
    CHECK(lora_ok_count() == 3u);
    CHECK(!lora_poll(449u, &t, tx, sizeof tx));
    CHECK(lora_poll(450u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 4u);
    /* Long stall: one packet, the missed 10 Hz slots are not replayed. */
    uint32_t ok0 = lora_ok_count();
    CHECK(lora_poll(5000u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() - ok0 == 1u);
    CHECK(lora_drop_count() == 0u);
    CHECK(!lora_poll(5099u, &t, tx, sizeof tx));
    CHECK(lora_poll(5100u, &t, tx, sizeof tx));
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
    /* Deadline is consumed: no retry until 100 ms after the skipped slot. */
    CHECK(!lora_poll(99u, &t, tx, sizeof tx));
    CHECK(lora_airbusy_count() == 1u);
    /* AUX bosalinca ayni fazdan devam: ilk basarili paket seq 0 tasir
     * (atlama seq tuketmedi). */
    lora_host_set_aux_low(false);
    CHECK(lora_poll(100u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 1u && lora_seq() == 1u);
    CHECK(lora_airbusy_count() == 1u);
    {
      lora_frame_t out;
      CHECK(lora_parse(core_of(tx), LORA_FRAME_LEN, &out));
      CHECK(out.seq == 0u);
    }
    /* Ust uste atlama: her vade-tuketimi +1, ok/seq/drop sabit. */
    lora_host_set_aux_low(true);
    CHECK(!lora_poll(200u, &t, tx, sizeof tx));
    CHECK(!lora_poll(300u, &t, tx, sizeof tx));
    CHECK(lora_airbusy_count() == 3u);
    CHECK(lora_ok_count() == 1u && lora_seq() == 1u);
    CHECK(lora_drop_count() == 0u);
    /* Resume on the 400 ms slot; the missed slots do not burst/replay. */
    lora_host_set_aux_low(false);
    CHECK(!lora_poll(399u, &t, tx, sizeof tx));
    CHECK(lora_poll(400u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 2u && lora_seq() == 2u);
    /* lora_init airbusy'yi de sifirlar, enjeksiyonu temizler. */
    lora_init(0);
    CHECK(lora_airbusy_count() == 0u);
    CHECK(lora_poll(0u, &t, tx, sizeof tx));
    CHECK(lora_ok_count() == 1u && lora_airbusy_count() == 0u);
  }

  /* ================= UART/RF rate configuration policy =================== */
  {
    CHECK(LORA_LINK_BAUD == 9600u);
    CHECK(LORA_LINK_BAUD_FALLBACK == 9600u);
    CHECK(LORA_REG0_9600 == 0x63u && LORA_REG0_57600 == 0xC3u);
    CHECK((LORA_REG0_9600 >> 5) == 0x03u); /* 011 = 9600 UART */
    CHECK((LORA_REG0_57600 >> 5) == 0x06u); /* reserved 110 = 57600 UART */
    CHECK(((LORA_REG0_9600 >> 3) & 0x03u) == 0u); /* 8N1 */
    CHECK((LORA_REG0_9600 & LORA_REG0_AIR_MASK) == LORA_REG0_AIR_4K8_BITS);
    CHECK((LORA_REG0_57600 & LORA_REG0_AIR_MASK) == LORA_REG0_AIR_4K8_BITS);
    CHECK(LORA_AIR_RATE_BPS == 4800u);
    CHECK(LORA_CFG_EXPECTED[2] == 0x63u);
    CHECK(LORA_CFG_COMMON[2] == 0x63u);
    CHECK(LORA_CFG_EXPECTED[3] == 0x00u); /* 22 dBm hardware max */
    CHECK(LORA_TX_POWER_DBM == 22u);
    CHECK(LORA_CFG_EXPECTED[4] == 0x17u); /* 873.125 MHz */
    CHECK(LORA_CFG_EXPECTED[5] == 0x00u); /* transparent mode */
    CHECK(LORA_AIR_LEN == 41u); /* 3 fixed-header bytes + 38-byte V3 */
    CHECK(LORA_SEND_INTERVAL_MS == 100u && LORA_SEND_INTERVAL_WAIT_MS == 100u);
    CHECK(LORA_SEND_INTERVAL_MS * LORA_AIR_RATE_BPS >= LORA_AIR_LEN * 8u * 1000u);
    CHECK((LORA_AIR_LEN * 10u * 1000u) / LORA_LINK_BAUD < LORA_SEND_INTERVAL_MS);
    CHECK(lora_cfg_verified_baud(LORA_CFG_EXPECTED) == 9600u);
    CHECK(lora_cfg_verified_baud(NULL) == 9600u);
  }

  /* ================= E220 C0/C2 write command framing ==================== */
  {
    const uint8_t data[8] = {0x00u,0x00u,0x63u,0x00u,0x17u,0x00u,0x00u,0x00u};
    uint8_t cmd[12];
    memset(cmd, 0xAA, sizeof cmd);
    CHECK(lora_cfg_write_frame(0xC0u, 0x00u, 8u, data, cmd, sizeof cmd) == 11u);
    CHECK(cmd[0] == 0xC0u && cmd[1] == 0x00u && cmd[2] == 0x08u);
    CHECK(memcmp(&cmd[3], data, sizeof data) == 0);
    CHECK(cmd[11] == 0xAAu); /* exact write bounds */
    memset(cmd, 0xAA, sizeof cmd);
    CHECK(lora_cfg_write_frame(0xC2u, 0x00u, 8u, data, cmd, sizeof cmd) == 11u);
    CHECK(cmd[0] == 0xC2u && cmd[1] == 0x00u && cmd[2] == 0x08u);
    CHECK(memcmp(&cmd[3], data, sizeof data) == 0);
    CHECK(cmd[11] == 0xAAu); /* exact write bounds */
    CHECK(lora_cfg_write_frame(0xC0u, 0u, 8u, data, cmd, 10u) == 0u);
    CHECK(lora_cfg_write_frame(0xC2u, 0u, 8u, data, cmd, 10u) == 0u);
    CHECK(lora_cfg_write_frame(0xC1u, 0u, 8u, data, cmd, sizeof cmd) == 0u);
    CHECK(lora_cfg_write_frame(0xC2u, 0u, 0u, data, cmd, sizeof cmd) == 0u);
    CHECK(lora_cfg_write_frame(0xC2u, 0u, 9u, data, cmd, sizeof cmd) == 0u);
    CHECK(lora_cfg_write_frame(0xC2u, 0u, 8u, NULL, cmd, sizeof cmd) == 0u);
    CHECK(lora_cfg_write_frame(0xC2u, 0u, 8u, data, NULL, sizeof cmd) == 0u);
  }

  /* ================= RF register image: full datasheet contract ========== */
  {
    const uint8_t factory[6] = {0x00u, 0x00u, 0x62u, 0x00u, 0x17u, 0x00u};
    const uint8_t target[6]  = {0x00u, 0x00u, 0x63u, 0x00u, 0x17u, 0x00u};
    CHECK(memcmp(LORA_CFG_EXPECTED, target, sizeof target) == 0);
    CHECK((factory[2] & LORA_REG0_AIR_MASK) == LORA_REG0_AIR_2K4_BITS);
    CHECK((target[2] & LORA_REG0_AIR_MASK) == LORA_REG0_AIR_4K8_BITS);
    CHECK((target[2] & 0xF8u) == (factory[2] & 0xF8u)); /* same UART/parity */
    CHECK((target[3] & LORA_REG1_POWER_MASK) == 0u); /* 22 dBm maximum */
    CHECK((target[3] & LORA_REG1_SUBPKT_MASK) == 0u); /* 200-byte subpacket */
    CHECK(target[4] == 0x17u && target[5] == 0u);
    CHECK(850125u + (uint32_t)target[4] * 1000u == 873125u);
    uint8_t cmd[8];
    CHECK(lora_cfg_common_img(cmd, true));
    CHECK(memcmp(cmd, target, 6u) == 0 && cmd[6] == 0u && cmd[7] == 0u);
    /* 41-byte air image: raw RF payload lower bound at 4.8 kbps is 68.3 ms;
     * UART feed at 9600 baud is 42.7 ms. Modem overhead must also fit 100 ms;
     * the live 10 Hz gate remains explicitly hardware-unverified until soak. */
    CHECK((LORA_AIR_LEN * 8u * 1000u) / LORA_AIR_RATE_BPS == 68u);
    CHECK((LORA_AIR_LEN * 10u * 1000u) / LORA_LINK_BAUD == 42u);
    CHECK((LORA_AIR_LEN * 8u * 1000u) / 2400u > LORA_SEND_INTERVAL_MS);
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
    CHECK(lora_cfg_found_link_baud(hi, 57600u) == 57600u);
    /* Kanonik geri-donus: LO imaj 9600'de -> 9600. */
    CHECK(lora_cfg_found_link_baud(lo, 9600u) == 9600u);
    CHECK(lora_cfg_found_link_baud(lo, 9600u) == LORA_LINK_BAUD_FALLBACK);
    /* OLAY senaryosu 2 (uyum): modul 115200'de baska imajda, onarim tutmadi.
     * Kanitlanmis degisim kazanir -> huart 115200'e uyulur. */
    CHECK(lora_cfg_found_link_baud(bozuk, 115200u) == 115200u);
    CHECK(lora_cfg_found_link_baud(fabrika, 1200u) == 1200u);
    CHECK(lora_cfg_found_link_baud(NULL, 38400u) == 38400u);
    /* Kanit yoksa imaj konusmasi gecerlidir. */
    CHECK(lora_cfg_found_link_baud(hi, 0u) == 9600u);
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

  /* Common target image: when configuration dialogue is proven, construct
   * the 8-byte 0x63 image (9600 UART + 4.8 kbps RF + zero encryption key). */
  {
    uint8_t govde[8u] = {0xAAu, 0xAAu, 0xAAu, 0xAAu,
                         0xAAu, 0xAAu, 0xAAu, 0xAAu};
    CHECK(lora_cfg_common_img(govde, true));
    CHECK(govde[0] == 0x00u && govde[1] == 0x00u && govde[2] == 0x63u);
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
   * 63-imajindan FARKLIYSA ve C2-RAM yaz+dog rula BASARILI olduktan SONRA ayni
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
    bozuk[2] ^= 0x01u; /* tek bit: ne HI ne 63-imaji */
    /* Adaylik: eslesen 63-imajina C0 YOK (flash omru). */
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