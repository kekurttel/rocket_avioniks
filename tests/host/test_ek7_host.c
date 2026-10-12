/* Host tests for the Ek-7 protocol module (ek7.c, HAL-independent). */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ek7.h"
#include "flight.h"

static int n_pass = 0;
#define CHECK(c) do { assert(c); n_pass++; } while (0)

/* feed whole frame byte-by-byte, expect single completion at end */
static bool feed_all(const uint8_t *p, size_t n, ek7_cmd_t *out) {
  ek7_rx_t r; ek7_rx_init(&r);
  ek7_cmd_t c = EK7_EV_NONE;
  bool done = false;
  for (size_t i = 0; i < n; i++) {
    bool d = ek7_rx_feed(&r, p[i], &c);
    if (i + 1 < n) CHECK(!d);
    else done = d;
  }
  if (out) *out = c;
  return done;
}

/* Dokuman-literal komutlar (Tablo 1): geriye uyumluluk icin kabul. */
static void t_cmd_legacy(void) {
  const uint8_t sit[] = {0xAA, 0x20, 0x8C, 0x0D, 0x0A};
  const uint8_t sut[] = {0xAA, 0x22, 0x8E, 0x0D, 0x0A};
  const uint8_t stop[] = {0xAA, 0x24, 0x90, 0x0D, 0x0A};
  ek7_cmd_t c = EK7_EV_NONE;
  CHECK(feed_all(sit, 5, &c) && c == EK7_EV_SIT);
  CHECK(feed_all(sut, 5, &c) && c == EK7_EV_SUT);
  CHECK(feed_all(stop, 5, &c) && c == EK7_EV_STOP);
  printf("  cmd-legacy ok\n");
}
/* Calisan test yaziliminin gonderdigi komutlar (Bolum 3 toplama kurali:
 * SUM = (0xAA + CMD) & 0xFF). run.sh karsisinda bunlar calisir. */
static void t_cmd_official(void) {
  const uint8_t sit[] = {0xAA, 0x20, 0xCA, 0x0D, 0x0A};
  const uint8_t sut[] = {0xAA, 0x22, 0xCC, 0x0D, 0x0A};
  const uint8_t stop[] = {0xAA, 0x24, 0xCE, 0x0D, 0x0A};
  ek7_cmd_t c = EK7_EV_NONE;
  CHECK(feed_all(sit, 5, &c) && c == EK7_EV_SIT);
  CHECK(feed_all(sut, 5, &c) && c == EK7_EV_SUT);
  CHECK(feed_all(stop, 5, &c) && c == EK7_EV_STOP);
  /* Toplama kurali kimligi: her resmi cerceve kurali saglar. */
  CHECK(sit[2] == ((0xAAu + 0x20u) & 0xFFu));
  CHECK(sut[2] == ((0xAAu + 0x22u) & 0xFFu));
  CHECK(stop[2] == ((0xAAu + 0x24u) & 0xFFu));
  printf("  cmd-official ok\n");
}
static void t_cmd_reject(void) {
  ek7_rx_t r; ek7_rx_init(&r); ek7_cmd_t c = EK7_EV_NONE;
  const uint8_t bad_sum[] = {0xAA, 0x20, 0x00, 0x0D, 0x0A};
  const uint8_t bad_ftr[] = {0xAA, 0x22, 0x8E, 0x00, 0x0A};
  const uint8_t bad_cmd[] = {0xAA, 0x30, 0x9C, 0x0D, 0x0A};
  CHECK(!feed_all(bad_sum, 5, &c) && c == EK7_EV_NONE);
  CHECK(!feed_all(bad_ftr, 5, &c) && c == EK7_EV_NONE);
  CHECK(!feed_all(bad_cmd, 5, &c) && c == EK7_EV_NONE);
  /* garbage resync: noise then valid frame */
  const uint8_t mix[] = {0x11, 0xAA, 0xAA, 0x24, 0x90, 0x0D, 0x0A};
  CHECK(feed_all(mix, 7, &c) && c == EK7_EV_STOP);
  ek7_rx_feed(NULL, 0xAA, &c);
  ek7_rx_init(NULL);
  printf("  cmd-reject ok\n");
}
static void t_sum(void) {
  const uint8_t d[] = {0x01, 0x14};
  CHECK(ek7_sum(d, 2) == 0x15);
  CHECK(ek7_sum(NULL, 5) == 0);
  CHECK(ek7_sum(d, 0) == 0);
}
static void t_f32(void) {
  uint8_t b[4];
  ek7_pack_f32be(1.0f, b);
  CHECK(b[0] == 0x3F && b[1] == 0x80 && b[2] == 0x00 && b[3] == 0x00);
  CHECK(fabsf(ek7_unpack_f32be(b) - 1.0f) < 1e-6f);
  ek7_pack_f32be(-123.456f, b); /* 2-decimal rounding */
  CHECK(fabsf(ek7_unpack_f32be(b) + 123.46f) < 1e-6f);
  ek7_pack_f32be(NAN, b);
  CHECK(fabsf(ek7_unpack_f32be(b)) < 1e-9f);
  CHECK(fabsf(ek7_unpack_f32be(NULL)) < 1e-9f);
  ek7_pack_f32be(1.0f, NULL);
  printf("  f32 ok\n");
}
static void t_tlm(void) {
  uint8_t p[EK7_TLM_LEN];
  CHECK(ek7_build_tlm(p, 100.5f, 900.25f, 1.0f, 2.0f, -9.81f,
                      10.0f, 20.0f, 0.0f) == 36);
  CHECK(p[0] == 0xAB && p[34] == 0x0D && p[35] == 0x0A);
  CHECK(p[33] == ek7_sum(&p[0], 33)); /* resmi: header dahil toplam */
  float a, pr, ax, ay, az, ex, ey, ez;
  CHECK(ek7_parse_tlm(p, &a, &pr, &ax, &ay, &az, &ex, &ey, &ez));
  CHECK(fabsf(a - 100.5f) < 1e-6f && fabsf(pr - 900.25f) < 1e-6f);
  CHECK(fabsf(ax - 1.0f) < 1e-6f && fabsf(az + 9.81f) < 1e-6f);
  CHECK(fabsf(ey - 20.0f) < 1e-6f && fabsf(ez) < 1e-9f);
  p[0] = 0xAA; CHECK(!ek7_parse_tlm(p, &a, &pr, &ax, &ay, &az, &ex, &ey, &ez));
  p[0] = 0xAB; p[33] ^= 0xFF;
  CHECK(!ek7_parse_tlm(p, &a, &pr, &ax, &ay, &az, &ex, &ey, &ez));
  CHECK(!ek7_parse_tlm(NULL, &a, &pr, &ax, &ay, &az, &ex, &ey, &ez));
  CHECK(ek7_build_tlm(NULL, 0, 0, 0, 0, 0, 0, 0, 0) == 0);
  printf("  tlm ok\n");
}
static void t_status(void) {
  uint8_t p[EK7_STATUS_LEN];
  CHECK(ek7_build_status(p, 0x0114) == 6);
  /* Resmi cerceve: AA D1(bit0-7) D2(bit8-15) SUM(header+D1+D2) 0D 0A.
   * 0xAA+0x14+0x01 = 191 = 0xBF. */
  CHECK(p[0] == 0xAA && p[1] == 0x14 && p[2] == 0x01 && p[3] == 0xBF &&
        p[4] == 0x0D && p[5] == 0x0A);
  CHECK(ek7_sum(&p[0], 3) == p[3]); /* resmi dogrulama araligi */
  CHECK(ek7_build_status(NULL, 0) == 0);
  printf("  status-frame ok\n");
}
static void t_stword(void) {
  flight_cfg_t c; flight_config_test(&c, 5.0f);
  flight_t f; flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_WAIT_FOR_LAUNCH);
  /* rest alt 0 < main 5 -> BELOWMAIN fact bit set even pre-launch */
  CHECK(ek7_status_word(&f, false, true, 0.0f) == EK7_ST_BELOWMAIN);
  CHECK(ek7_status_word(NULL, true, true, 0.0f) == 0);
  flight_set_state(&f, FLIGHT_ASCENT);
  f.has_max = true; f.max_alt_m = 10.0f;
  uint16_t s = ek7_status_word(&f, true, true, 4.0f);
  CHECK(s & EK7_ST_LAUNCH); CHECK(s & EK7_ST_MINALT); CHECK(s & EK7_ST_ANGLE);
  CHECK(!(s & EK7_ST_BURNOUT)); /* D5: unsupported -> 0 */
  CHECK(!(s & EK7_ST_DESCENT)); CHECK(!(s & EK7_ST_DROGUE));
  flight_set_state(&f, FLIGHT_APOGEE_CONFIRMED);
  CHECK(ek7_status_word(&f, false, true, 4.0f) & EK7_ST_DESCENT);
  f.drogue_deployed = true;
  CHECK(ek7_status_word(&f, false, true, 4.0f) & EK7_ST_DROGUE);
  f.main_deployed = true;
  CHECK(ek7_status_word(&f, false, true, 4.0f) & EK7_ST_MAIN);
  flight_set_state(&f, FLIGHT_FAULT);
  CHECK(!(ek7_status_word(&f, true, true, 4.0f) &
          (EK7_ST_LAUNCH | EK7_ST_DESCENT))); /* fault carries no flight bits */
  /* production min-alt gate: below 1250 -> bit2 clear */
  flight_config_production(&c); flight_init(&f, &c);
  flight_set_state(&f, FLIGHT_ASCENT);
  f.has_max = true; f.max_alt_m = 800.0f;
  CHECK(!(ek7_status_word(&f, false, true, 700.0f) & EK7_ST_MINALT));
  f.max_alt_m = 1300.0f;
  CHECK(ek7_status_word(&f, false, true, 1200.0f) & EK7_ST_MINALT);
  printf("  status-word ok\n");
}
static void t_map(void) {
  float x, y, z;
  ek7_map_acc(0.0f, 0.0f, 16384.0f, 16384.0f, &x, &y, &z);
  CHECK(fabsf(x) < 1e-6f && fabsf(y) < 1e-6f &&
        fabsf(z + 9.80665f) < 1e-3f); /* Z-down: rest reads -g */
  ek7_map_acc(0.0f, 0.0f, 21080.0f, 21080.0f, &x, &y, &z);
  CHECK(fabsf(z + 9.80665f) < 1e-3f); /* self-cal: measured 1 g -> -g */
  ek7_map_acc(0.0f, 0.0f, 21080.0f, 0.0f, &x, &y, &z);
  CHECK(fabsf(z + 9.80665f * 21080.0f / 16384.0f) < 1e-2f); /* nominal path */
  float ex, ey, ez;
  ek7_map_ang(0.0f, 0.0f, 16384.0f, &ex, &ey, &ez);
  CHECK(fabsf(ex) < 1e-4f && fabsf(ey) < 1e-4f && fabsf(ez) < 1e-9f);
  ek7_map_ang(16384.0f, 0.0f, 16384.0f, &ex, &ey, &ez);
  CHECK(fabsf(ex - 45.0f) < 0.5f && fabsf(ez) < 1e-9f); /* yaw always 0 */
  ek7_map_ang(0.0f, 0.0f, 0.0f, &ex, &ey, &ez); /* degenerate: no crash */
  ek7_map_acc(0, 0, 0, 16384.0f, NULL, NULL, NULL);
  ek7_map_ang(0, 0, 0, NULL, NULL, NULL);
  printf("  map ok\n");
}

int main(void) {
  t_cmd_official(); t_cmd_legacy(); t_cmd_reject(); t_sum(); t_f32(); t_tlm(); t_status();
  t_stword(); t_map();
  printf("HOST_EK7_TESTS_PASSED n=%d\n", n_pass);
  return 0;
}
