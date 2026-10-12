/* Host tests for the NMEA GPS parser (gps.c, HAL-independent).
 * Covers: valid GGA/RMC, bad checksum, malformed, incomplete, no-fix,
 * hemisphere signs, stale timeout, mixed streams, arbitrary chunking. */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gps.h"

static int n_pass = 0;
#define CHECK(c) do { assert(c); n_pass++; } while (0)

/* Feed a whole C string (includes \r\n) at timestamp t. */
static void feed_all(gps_t *g, const char *s, uint32_t t)
{
  for (size_t i = 0; s[i] != '\0'; i++)
  {
    gps_rx_byte(g, (uint8_t)s[i], t);
  }
}

/* Feed in fixed-size chunks (UART delivers arbitrary boundaries). */
static void feed_chunked(gps_t *g, const char *s, uint32_t t, size_t chunk)
{
  size_t n = strlen(s);
  for (size_t i = 0; i < n;)
  {
    size_t k = chunk;
    if (i + k > n)
    {
      k = n - i;
    }
    for (size_t j = 0; j < k; j++)
    {
      gps_rx_byte(g, (uint8_t)s[i + j], t);
    }
    i += k;
  }
}

static const char *GGA_FIX =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static const char *RMC_FIX =
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";

/* 1) valid GGA: position, altitude, time, sats. */
static void t_gga(void)
{
  gps_t g;
  gps_init(&g);
  CHECK(!g.present && !gps_has_fix(&g, 0));
  feed_all(&g, GGA_FIX, 1000);
  CHECK(g.present && g.fix && g.valid);
  CHECK(gps_has_fix(&g, 1000));
  CHECK(g.lat_e5 == 4811730);  /* 48 + 7.038/60 */
  CHECK(g.lon_e5 == 1151667);  /* 11 + 31/60 */
  CHECK(fabsf(g.alt_m - 545.4f) < 1e-3f && g.alt_valid);
  CHECK(g.hour == 12 && g.min == 35 && g.sec == 19 && g.time_valid);
  CHECK(g.sats == 8);
  CHECK(g.last_fix_ms == 1000 && g.last_update_ms == 1000);
  printf("  gga ok\n");
}

/* 2) valid RMC: position, speed (knots->m/s), course, date. */
static void t_rmc(void)
{
  gps_t g;
  gps_init(&g);
  feed_all(&g, RMC_FIX, 2000);
  CHECK(g.present && gps_has_fix(&g, 2000));
  CHECK(g.lat_e5 == 4811730 && g.lon_e5 == 1151667);
  CHECK(g.speed_valid && fabsf(g.speed_mps - 22.4f * 0.514444f) < 1e-3f);
  CHECK(g.course_valid && fabsf(g.course_deg - 84.4f) < 1e-3f);
  CHECK(g.date_valid && g.day == 23 && g.mon == 3 && g.year == 94);
  printf("  rmc ok\n");
}

/* 3) bad checksum: sentence ignored, no presence, no latching. */
static void t_badcs(void)
{
  gps_t g;
  gps_init(&g);
  feed_all(&g,
           "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n",
           3000);
  CHECK(!g.present && !g.fix && !g.valid && g.sats == 0);
  CHECK(g.lat_e5 == 0 && g.lon_e5 == 0 && !g.alt_valid);
  /* A later good sentence still works (no poisoned state). */
  feed_all(&g, GGA_FIX, 3100);
  CHECK(gps_has_fix(&g, 3100));
  printf("  bad-checksum ok\n");
}

/* 4) malformed: valid checksum but impossible coordinates/hemispheres. */
static void t_malformed(void)
{
  gps_t g;
  gps_init(&g);
  feed_all(&g,
           "$GPGGA,123519,9999.999,X,99999.999,Y,1,08,0.9,545.4,M,46.9,M,,*46\r\n",
           4000);
  CHECK(g.present);              /* framed + checksummed: module is there */
  CHECK(!g.fix && !g.valid);     /* but never a fix from garbage */
  CHECK(g.lat_e5 == 0 && g.lon_e5 == 0);
  /* Non-numeric garbage in a required field: same outcome. */
  feed_all(&g, "$GPRMC,123519,A,abcd,N,01131.000,E,022.4,084.4,230394,,*00\r\n",
           4100); /* bad checksum on purpose too */
  CHECK(!g.fix && !g.valid);
  printf("  malformed ok\n");
}

/* 5) incomplete: half a sentence changes nothing; completion works. */
static void t_incomplete(void)
{
  gps_t g;
  gps_init(&g);
  const char *half = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,54";
  feed_all(&g, half, 5000);
  CHECK(!g.present && !g.fix);
  feed_all(&g, "5.4,M,46.9,M,,*47\r\n", 5050);
  CHECK(gps_has_fix(&g, 5050) && g.lat_e5 == 4811730);
  /* A lone '$' mid-line resyncs (aborts the partial sentence). */
  feed_all(&g, "$GPGGA,000000,,,,,,0,00,99.9,,,,,,*00", 5100); /* no \n yet */
  feed_all(&g, "$GPGGA,123519,,,,,,0,00,99.9,,,,,,*50\r\n", 5150);
  CHECK(g.present && !g.fix); /* no-fix sentence parsed, no stale fix kept */
  printf("  incomplete ok\n");
}

/* 6) no-fix sentences: present, never valid, never nonzero position. */
static void t_nofix(void)
{
  gps_t g;
  gps_init(&g);
  feed_all(&g, "$GPGGA,123519,,,,,,0,00,99.9,,,,,,*50\r\n", 6000);
  CHECK(g.present && !g.fix && !g.valid && !gps_has_fix(&g, 6000));
  feed_all(&g, "$GPRMC,123519,V,,,,,,,230394,,,*1F\r\n", 6100);
  CHECK(g.present && !g.fix && !g.valid);
  CHECK(g.lat_e5 == 0 && g.lon_e5 == 0 && !g.alt_valid && !g.speed_valid);
  /* A fix followed by no-fix clears validity (stale data never served). */
  feed_all(&g, GGA_FIX, 6200);
  CHECK(gps_has_fix(&g, 6200));
  feed_all(&g, "$GPGGA,123521,,,,,,0,00,99.9,,,,,,*5B\r\n", 6300);
  CHECK(g.present && !g.fix && !g.valid && !gps_has_fix(&g, 6300));
  printf("  nofix ok\n");
}

/* 7) S/W hemispheres go negative; N/E stay positive. */
static void t_hemi(void)
{
  gps_t g;
  gps_init(&g);
  feed_all(&g,
           "$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*48\r\n",
           7000);
  CHECK(gps_has_fix(&g, 7000));
  CHECK(g.lat_e5 == -4811730 && g.lon_e5 == -1151667);
  gps_t h;
  gps_init(&h);
  feed_all(&h,
           "$GPRMC,083559,A,3351.850,S,15112.400,W,000.5,054.7,030303,011.3,E*73\r\n",
           7100);
  CHECK(gps_has_fix(&h, 7100));
  CHECK(h.lat_e5 == -3386417);  /* -(33 + 51.85/60) */
  CHECK(h.lon_e5 == -15120667); /* -(151 + 12.4/60) */
  printf("  hemispheres ok\n");
}

/* 8) stale timeout: fix expires GPS_STALE_MS after the last fix. */
static void t_stale(void)
{
  gps_t g;
  gps_init(&g);
  feed_all(&g, GGA_FIX, 0);
  CHECK(gps_has_fix(&g, 0));
  CHECK(gps_has_fix(&g, GPS_STALE_MS - 1u));
  CHECK(!gps_has_fix(&g, GPS_STALE_MS + 1u));
  CHECK(gps_is_stale(&g, GPS_STALE_MS + 1u));
  /* A fresh fix revives. */
  feed_all(&g, RMC_FIX, GPS_STALE_MS + 500u);
  CHECK(gps_has_fix(&g, GPS_STALE_MS + 500u));
  /* Non-fix traffic does NOT revive the fix (only presence). */
  feed_all(&g, "$GPGGA,123521,,,,,,0,00,99.9,,,,,,*5B\r\n",
           2u * GPS_STALE_MS);
  CHECK(g.present && !gps_has_fix(&g, 2u * GPS_STALE_MS));
  printf("  stale ok\n");
}

/* 9) mixed stream: GGA + RMC + ignored types merge into one status. */
static void t_mixed(void)
{
  gps_t g;
  gps_init(&g);
  feed_all(&g,
           "$GPGSV,3,1,11,02,25,279,41,04,73,025,42,05,21,314,43,10,70,285,44*78\r\n",
           8000);
  CHECK(g.present && !g.fix); /* alive, no position yet */
  feed_all(&g, GGA_FIX, 8100);
  feed_all(&g, RMC_FIX, 8200);
  CHECK(gps_has_fix(&g, 8200));
  CHECK(g.lat_e5 == 4811730 && g.sats == 8);
  CHECK(g.speed_valid && g.course_valid && g.date_valid);
  CHECK(g.last_fix_ms == 8200); /* RMC refreshed the fix */
  printf("  mixed ok\n");
}

/* 10) arbitrary chunking: 1..9-byte pieces give identical results. */
static void t_chunked(void)
{
  const char *stream_part1 = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,";
  const char *stream_part2 = "0.9,545.4,M,46.9,M,,*47\r\n";
  char stream[256];
  snprintf(stream, sizeof(stream), "%s%s%s", stream_part1, stream_part2,
           RMC_FIX);
  gps_t ref;
  gps_init(&ref);
  feed_all(&ref, stream, 9000);
  for (size_t chunk = 1; chunk <= 9; chunk++)
  {
    gps_t g;
    gps_init(&g);
    feed_chunked(&g, stream, 9000, chunk);
    CHECK(g.present == ref.present && g.fix == ref.fix &&
          g.valid == ref.valid);
    CHECK(g.lat_e5 == ref.lat_e5 && g.lon_e5 == ref.lon_e5 &&
          g.sats == ref.sats);
    CHECK(fabsf(g.alt_m - ref.alt_m) < 1e-6f);
    CHECK(fabsf(g.speed_mps - ref.speed_mps) < 1e-6f);
    CHECK(g.last_fix_ms == ref.last_fix_ms);
  }
  printf("  chunked ok\n");
}

/* Overflow: an over-long garbage line resyncs at the next sentence. */
static void t_overflow(void)
{
  gps_t g;
  gps_init(&g);
  for (int i = 0; i < 200; i++)
  {
    gps_rx_byte(&g, (uint8_t)'X', 9500);
  }
  gps_rx_byte(&g, (uint8_t)'\n', 9500);
  CHECK(!g.present);
  feed_all(&g, RMC_FIX, 9600);
  CHECK(gps_has_fix(&g, 9600));
  printf("  overflow ok\n");
}

/* 12) bozuk kablo fırtınası: checksumsuz/yarım/ikili çöp sessizce atılır,
 * sentinel korunur (konum/fix/sats yazılmaz), ayrıştırıcı zehirlenmez. */
static void t_garbage_storm(void)
{
  gps_t g;
  gps_init(&g);
  /* Yanlış checksum: atılır, presence bile yok. */
  feed_all(&g,
           "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n",
           10000);
  CHECK(!g.present && !g.fix && !g.valid);
  /* Checksum yok / yarım satır / satır sonu yok: hiçbir şey değişmez. */
  feed_all(&g, "$GPGGA,123519,4807.038,N,01131.000,E,1,08\r\n", 10100);
  feed_all(&g, "$GPGGA,123519,4807.038,N", 10200);
  feed_all(&g, "xyz*12\r", 10300);
  feed_all(&g, "\n", 10400);
  CHECK(!g.present && !g.fix && !g.valid);
  CHECK(g.lat_e5 == 0 && g.lon_e5 == 0 && g.sats == 0);
  CHECK(!g.alt_valid && !g.speed_valid && !g.time_valid);
  CHECK(g.ack_seq == 0);
  /* İkili çöp (kopuk kablonun ürettiği 0xB5/0xFF/0x00/NUL dahil): atılır. */
  {
    static const uint8_t bin[] = {0xB5, 0x62, 0x05, 0xFF, 0x00, 0x80, 0x01,
                                  0xB5, 0xB5, 0x62, 0x00, '\r', 0x7F};
    for (size_t i = 0; i < sizeof bin; i++)
    {
      gps_rx_byte(&g, bin[i], 10500);
    }
    gps_rx_byte(&g, (uint8_t)'\n', 10500);
  }
  CHECK(!g.present && !g.fix && !g.valid);
  CHECK(g.lat_e5 == 0 && g.lon_e5 == 0 && g.sats == 0);
  /* Fırtına sonrası sağlam cümle: ayrıştırıcı zehirlenmemiş. */
  feed_all(&g, GGA_FIX, 10600);
  CHECK(gps_has_fix(&g, 10600) && g.lat_e5 == 4811730 && g.sats == 8);
  printf("  garbage-storm ok\n");
}

/* 13) sağlam fix sonrası çöp: eski konumun üstüne ASLA yazılmaz. */
static void t_garbage_after_fix(void)
{
  gps_t g;
  gps_init(&g);
  feed_all(&g, GGA_FIX, 11000);
  CHECK(gps_has_fix(&g, 11000));
  int32_t lat = g.lat_e5, lon = g.lon_e5;
  uint8_t sats = g.sats;
  uint32_t fix_ms = g.last_fix_ms;
  /* Bozuk kablo: yarım UBX + yanlış checksum + ikili gürültü. */
  {
    static const uint8_t ubx_half[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00};
    for (size_t i = 0; i < sizeof ubx_half; i++)
    {
      gps_rx_byte(&g, ubx_half[i], 11100);
    }
  }
  feed_all(&g,
           "$GPGGA,999999,9999.999,X,99999.999,Y,1,08,0.9,545.4,M,46.9,M,,*00\r\n",
           11100);
  for (int i = 0; i < 40; i++)
  {
    gps_rx_byte(&g, (uint8_t)(0x80 + (i & 0x7F)), 11100);
  }
  gps_rx_byte(&g, (uint8_t)'\n', 11100);
  CHECK(g.present && g.fix && g.valid); /* taze fix hâlâ geçerli */
  CHECK(g.lat_e5 == lat && g.lon_e5 == lon && g.sats == sats);
  CHECK(g.last_fix_ms == fix_ms); /* çöp fix saatini oynatamaz */
  CHECK(gps_has_fix(&g, 11100));
  printf("  garbage-after-fix ok\n");
}

/* 14) yarım UBX içine '$' girerse UBX çekilir, sağlam cümle hemen işlenir. */
static void t_ubx_abort_resync(void)
{
  gps_t g;
  gps_init(&g);
  {
    static const uint8_t ubx_half[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00};
    for (size_t i = 0; i < sizeof ubx_half; i++)
    {
      gps_rx_byte(&g, ubx_half[i], 12000);
    }
  }
  CHECK(!g.present);
  feed_all(&g, GGA_FIX, 12100); /* '$' yarım UBX'i bozar, cümle işlenir */
  CHECK(gps_has_fix(&g, 12100) && g.lat_e5 == 4811730);
  /* Cümle ortasına karışan ikili baytlar atılır, checksum tutar. */
  gps_t h;
  gps_init(&h);
  {
    const char *pre = "$GPGGA,123519,4807.038,";
    const char *post = "N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    static const uint8_t mid[] = {0xB5, 0xFF, 0x00, 0x80};
    feed_all(&h, pre, 12200);
    for (size_t i = 0; i < sizeof mid; i++)
    {
      gps_rx_byte(&h, mid[i], 12200);
    }
    feed_all(&h, post, 12200);
  }
  CHECK(gps_has_fix(&h, 12200) && h.lat_e5 == 4811730 && h.sats == 8);
  printf("  ubx-abort-resync ok\n");
}

int main(void)
{
  t_gga();
  t_rmc();
  t_badcs();
  t_malformed();
  t_incomplete();
  t_nofix();
  t_hemi();
  t_stale();
  t_mixed();
  t_chunked();
  t_overflow();
  t_garbage_storm();
  t_garbage_after_fix();
  t_ubx_abort_resync();
  printf("HOST_GPS_TESTS_PASSED n=%d\n", n_pass);
  return 0;
}
