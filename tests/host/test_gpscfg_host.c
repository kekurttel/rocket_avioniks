/* Host tests for GPS configuration (UBX builders, ACK tracker, scheduler).
 * HAL-independent: compiles Src/gps.c only. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gps.h"

static int n_pass = 0;
#define CHECK(c) do { assert(c); n_pass++; } while (0)

/* Independent Fletcher (cls..payload) to cross-check the builders. */
static bool ubx_ok(const uint8_t *f, size_t n)
{
  if (n < 8u || f[0] != 0xB5u || f[1] != 0x62u)
  {
    return false;
  }
  size_t pl = (size_t)f[4] | ((size_t)f[5] << 8);
  if (n != 8u + pl)
  {
    return false;
  }
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6u + pl; i++)
  {
    a = (uint8_t)(a + f[i]);
    b = (uint8_t)(b + a);
  }
  return a == f[6u + pl] && b == f[7u + pl];
}

/* Build a UBX-ACK-ACK/NAK frame for (cls,id). */
static size_t mk_ack(uint8_t *o, bool ack, uint8_t cls, uint8_t id)
{
  o[0] = 0xB5;
  o[1] = 0x62;
  o[2] = 0x05;
  o[3] = ack ? 0x01 : 0x00;
  o[4] = 2;
  o[5] = 0;
  o[6] = cls;
  o[7] = id;
  uint8_t a = 0, b = 0;
  for (int i = 2; i < 8; i++)
  {
    a = (uint8_t)(a + o[i]);
    b = (uint8_t)(b + a);
  }
  o[8] = a;
  o[9] = b;
  return 10;
}

static void feed(gps_t *g, const uint8_t *d, size_t n, uint32_t t)
{
  for (size_t i = 0; i < n; i++)
  {
    gps_rx_byte(g, d[i], t);
  }
}

static void feed_nmea(gps_t *g, const char *body, uint32_t t)
{
  uint8_t x = 0;
  for (const char *p = body; *p; p++)
  {
    x ^= (uint8_t)*p;
  }
  char line[160];
  snprintf(line, sizeof line, "$%s*%02X\r\n", body, x);
  feed(g, (const uint8_t *)line, strlen(line), t);
}

#define GGA "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"

/* Simulated module: answers whatever the board sent last. */
typedef struct
{
  bool alive;      /* answers at all */
  bool nak;        /* answers NAK */
  int drop_first;  /* ignore this many sends before answering */
} sim_t;

int main(void)
{
  uint8_t f[64];

  /* ---- Builders: known-good vectors ---- */
  /* CFG-MSG is the 6-byte gen6 form: msgClass msgID rateDDC rateUART1
   * rateUART2 rateUSB -- there is no "current port" resolution in UBX, so the
   * rate must land in the UART1 slot (out[9]). GLL (F0 01) off => all rates 0.
   * Fletcher over 06 01 06 00 F0 01 00 00 00 00 => ckA 0xFE, ckB 0x1A. */
  static const uint8_t gll_off[] = {0xB5, 0x62, 0x06, 0x01, 0x06, 0x00,
                                    0xF0, 0x01, 0x00, 0x00, 0x00, 0x00,
                                    0xFE, 0x1A};
  CHECK(gps_ubx_cfg_msg(f, sizeof f, 0xF0, 0x01, 0) == sizeof gll_off);
  CHECK(memcmp(f, gll_off, sizeof gll_off) == 0);
  CHECK(ubx_ok(f, sizeof gll_off));
  /* Rate goes to UART1 only: DDC / UART2 / USB slots stay 0. */
  CHECK(gps_ubx_cfg_msg(f, sizeof f, 0xF0, 0x04, 1) == 14u);
  CHECK(f[9] == 1);
  CHECK(f[8] == 0 && f[10] == 0 && f[11] == 0);
  /* CFG-RATE at the gen6 floor of 1000 ms: 0x3E8 -> E8 03 LE. Fletcher over
   * 06 08 06 00 E8 03 01 00 01 00 => ckA 0x01, ckB 0x39. */
  static const uint8_t rate1000[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
                                     0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
                                     0x01, 0x39};
  CHECK(gps_ubx_cfg_rate(f, sizeof f, 1000) == sizeof rate1000);
  CHECK(memcmp(f, rate1000, sizeof rate1000) == 0);
  CHECK(ubx_ok(f, sizeof rate1000));
  size_t nn = gps_ubx_cfg_nav5_dyn(f, sizeof f, 7);
  CHECK(nn == 44u && ubx_ok(f, nn));
  CHECK(f[2] == 0x06 && f[3] == 0x24 && f[4] == 36 && f[5] == 0);
  CHECK(f[6] == 0x01 && f[7] == 0x00 && f[8] == 7); /* mask dyn, model 7 */
  for (int i = 9; i < 42; i++)
  {
    CHECK(f[i] == 0); /* nothing else touched */
  }
  /* Argument safety. */
  CHECK(gps_ubx_cfg_msg(NULL, 64, 0xF0, 0, 0) == 0);
  CHECK(gps_ubx_cfg_msg(f, 13, 0xF0, 0, 0) == 0); /* cap < 8 header + 6 */
  CHECK(gps_ubx_cfg_rate(f, 13, 1000) == 0);      /* cap < 14 */
  CHECK(gps_ubx_cfg_rate(f, 64, 999) == 0);       /* below the gen6 1000 ms floor */
  CHECK(gps_ubx_cfg_nav5_dyn(f, 43, 7) == 0);
  CHECK(gps_ubx_cfg_nav5_dyn(f, 64, 8) == 0);     /* gen7 renumbering, invalid on gen6 */
  CHECK(GPS_CFG_DYNMODEL <= 7u && GPS_CFG_RATE_MS >= 1000u);

  /* Every step builds a valid frame and reports the right ACK id. */
  for (uint8_t s = 0; s < GPS_CFG_STEPS; s++)
  {
    uint8_t ac = 0, ai = 0;
    size_t n = gps_cfg_frame(s, f, sizeof f, &ac, &ai);
    CHECK(n > 0u && ubx_ok(f, n));
    CHECK(ac == 0x06);
    CHECK(ai == f[3]); /* ACK carries the config message's own class/id */
    CHECK(n <= GPS_UBX_MAX_FRAME);
    if (s < 6u)
    {
      CHECK(ai == 0x01u);                          /* all six CFG-MSG steps */
      CHECK(f[4] == 6u && f[5] == 0u);             /* 6-byte gen6 payload */
      CHECK(f[8] == 0u && f[10] == 0u && f[11] == 0u); /* DDC / UART2 / USB unused */
      CHECK(f[9] == ((s == 0u || s == 1u) ? 1u : 0u)); /* GGA+RMC on, rest off */
    }
    else if (s == 6u)
    {
      CHECK(ai == 0x24u); /* NAV5 */
    }
    else
    {
      CHECK(ai == 0x08u); /* RATE */
    }
  }
  CHECK(gps_cfg_frame(GPS_CFG_STEPS, f, sizeof f, NULL, NULL) == 0);

  /* ---- ACK tracker ---- */
  gps_t g;
  gps_init(&g);
  uint8_t a[10];
  mk_ack(a, true, 0x06, 0x24); /* NAV5 ack: payload contains 0x24 = '$' */
  feed(&g, a, 10, 100);
  CHECK(g.ack_seq == 1 && g.ack_ok && g.ack_cls == 0x06 && g.ack_id == 0x24);
  CHECK(g.ack_count == 1 && g.nak_count == 0);
  CHECK(!g.present); /* an ACK is not NMEA: presence contract unchanged */
  mk_ack(a, false, 0x06, 0x08);
  feed(&g, a, 10, 200);
  CHECK(g.ack_seq == 2 && !g.ack_ok && g.ack_id == 0x08 && g.nak_count == 1);
  /* Bad checksum is ignored. */
  mk_ack(a, true, 0x06, 0x01);
  a[9] ^= 0xFF;
  feed(&g, a, 10, 300);
  CHECK(g.ack_seq == 2);
  /* Interleaved with NMEA, chunk by chunk: fix survives, ACK is seen. */
  gps_init(&g);
  feed_nmea(&g, GGA, 1000);
  CHECK(g.present && g.valid && g.sats == 8);
  mk_ack(a, true, 0x06, 0x24);
  feed(&g, a, 10, 1100);
  CHECK(g.ack_seq == 1);
  feed_nmea(&g, GGA, 1200);
  CHECK(g.valid && g.last_fix_ms == 1200);
  /* Binary reply split across arbitrary chunks. */
  mk_ack(a, true, 0x06, 0x08);
  for (int i = 0; i < 10; i++)
  {
    gps_rx_byte(&g, a[i], 1300);
  }
  CHECK(g.ack_seq == 2);
  /* Stray 0xB5 in the middle of an NMEA line must not hijack it. */
  gps_init(&g);
  const char *part = "$GPGGA,1235";
  feed(&g, (const uint8_t *)part, strlen(part), 10);
  gps_rx_byte(&g, 0xB5, 10);
  gps_rx_byte(&g, 0x62, 10);
  CHECK(g.ack_seq == 0);

  /* ---- Scheduler: happy path against a simulated module ---- */
  gps_t gm;
  gps_cfg_t c;
  gps_init(&gm);
  gps_cfg_init(&c);
  CHECK(c.state == GPS_CFG_IDLE);
  uint8_t buf[GPS_UBX_MAX_FRAME];
  uint32_t t = 5000;
  int sent = 0;
  uint8_t last_cls = 0, last_id = 0;
  for (; t < 5000 + 20000 && c.state != GPS_CFG_OK; t += 5)
  {
    size_t n = gps_cfg_poll(&c, &gm, t, true, false, buf, sizeof buf);
    if (n > 0u)
    {
      CHECK(ubx_ok(buf, n));
      last_cls = 0x06;
      last_id = buf[3];
      sent++;
      /* module replies ~20 ms later: emulate by feeding right away */
      uint8_t ak[10];
      mk_ack(ak, true, last_cls, last_id);
      feed(&gm, ak, 10, t);
    }
  }
  CHECK(c.state == GPS_CFG_OK && c.acked == GPS_CFG_STEPS);
  CHECK(sent == (int)GPS_CFG_STEPS); /* exactly one send per step */
  /* Terminal: nothing more is ever sent. */
  CHECK(gps_cfg_poll(&c, &gm, t + 10000, true, false, buf, sizeof buf) == 0);

  /* ---- Silent / absent GPS: NOACK after tries, then quiet ---- */
  gps_init(&gm);
  gps_cfg_init(&c);
  sent = 0;
  uint32_t first = 0, lastsend = 0;
  for (t = 1000; t < 1000 + 10000; t += 5)
  {
    size_t n = gps_cfg_poll(&c, &gm, t, true, false, buf, sizeof buf);
    if (n > 0u)
    {
      if (sent == 0)
      {
        first = t;
      }
      lastsend = t;
      sent++;
    }
  }
  CHECK(c.state == GPS_CFG_NOACK);
  CHECK(sent == (int)GPS_CFG_TRIES); /* gives up after 3 tries of step 0 */
  CHECK(lastsend - first >= 2u * GPS_CFG_ACK_TMO_MS);
  CHECK(c.acked == 0);

  /* ---- NAK stops the sequence ---- */
  gps_init(&gm);
  gps_cfg_init(&c);
  sent = 0;
  for (t = 0; t < 5000; t += 5)
  {
    size_t n = gps_cfg_poll(&c, &gm, t, true, false, buf, sizeof buf);
    if (n > 0u)
    {
      sent++;
      uint8_t ak[10];
      mk_ack(ak, sent < 3, 0x06, buf[3]); /* 3rd step refused */
      feed(&gm, ak, 10, t);
    }
  }
  CHECK(c.state == GPS_CFG_NAK && sent == 3 && c.acked == 2);

  /* ---- Retry: first send lost, second answered ---- */
  gps_init(&gm);
  gps_cfg_init(&c);
  sent = 0;
  for (t = 0; t < 30000 && c.state != GPS_CFG_OK; t += 5)
  {
    size_t n = gps_cfg_poll(&c, &gm, t, true, false, buf, sizeof buf);
    if (n > 0u)
    {
      sent++;
      if (sent != 1) /* the very first frame is "lost" */
      {
        uint8_t ak[10];
        mk_ack(ak, true, 0x06, buf[3]);
        feed(&gm, ak, 10, t);
      }
    }
  }
  CHECK(c.state == GPS_CFG_OK && sent == (int)GPS_CFG_STEPS + 1);

  /* ---- Unrelated ACK is ignored (does not advance the step) ---- */
  gps_init(&gm);
  gps_cfg_init(&c);
  CHECK(gps_cfg_poll(&c, &gm, 0, true, false, buf, sizeof buf) > 0u);
  uint8_t ak[10];
  mk_ack(ak, true, 0x06, 0x24); /* answers a different message */
  feed(&gm, ak, 10, 10);
  CHECK(gps_cfg_poll(&c, &gm, 20, true, false, buf, sizeof buf) == 0);
  CHECK(c.acked == 0 && c.step == 0 && c.state == GPS_CFG_SENDING);

  /* ---- tx_busy blocks sending ---- */
  gps_init(&gm);
  gps_cfg_init(&c);
  CHECK(gps_cfg_poll(&c, &gm, 0, true, true, buf, sizeof buf) == 0);
  CHECK(c.state == GPS_CFG_IDLE);

  /* ---- HAL tick wrap (2^32 ms): the inter-step gate must stay usable ---- */
  /* A fresh sequencer has ready_ms == 0; once uptime passes 2^31 ms the old
   * signed gate read that as "now is in the future" and froze forever. */
  gps_init(&gm);
  gps_cfg_init(&c);
  CHECK(c.ready_ms == 0);
  CHECK(gps_cfg_poll(&c, &gm, 0x80000000u, true, false, buf, sizeof buf) > 0u);
  /* A real gap whose deadline itself wrapped: 0xFFFFFFC0 + GPS_CFG_GAP_MS. */
  gps_init(&gm);
  gps_cfg_init(&c);
  c.state = GPS_CFG_SENDING;
  c.step = 1;
  c.acked = 1;
  c.ready_ms = 0x00000024u;
  CHECK(gps_cfg_poll(&c, &gm, 0xFFFFFFF0u, true, false, buf, sizeof buf) == 0);
  CHECK(gps_cfg_poll(&c, &gm, 0x00000000u, true, false, buf, sizeof buf) == 0);
  CHECK(gps_cfg_poll(&c, &gm, 0x00000010u, true, false, buf, sizeof buf) == 0);
  CHECK(gps_cfg_poll(&c, &gm, 0x00000030u, true, false, buf, sizeof buf) > 0u);

  /* ---- Flight abort: never TX once not allowed ---- */
  gps_init(&gm);
  gps_cfg_init(&c);
  CHECK(gps_cfg_poll(&c, &gm, 0, false, false, buf, sizeof buf) == 0);
  CHECK(c.state == GPS_CFG_ABORTED);
  CHECK(gps_cfg_poll(&c, &gm, 1000, true, false, buf, sizeof buf) == 0);
  /* Abort in the middle of the sequence. */
  gps_init(&gm);
  gps_cfg_init(&c);
  CHECK(gps_cfg_poll(&c, &gm, 0, true, false, buf, sizeof buf) > 0u);
  CHECK(gps_cfg_poll(&c, &gm, 600, false, false, buf, sizeof buf) == 0);
  CHECK(c.state == GPS_CFG_ABORTED);
  CHECK(gps_cfg_poll(&c, &gm, 5000, true, false, buf, sizeof buf) == 0);

  /* ---- NULL safety / names ---- */
  CHECK(gps_cfg_poll(NULL, &gm, 0, true, false, buf, sizeof buf) == 0);
  CHECK(gps_cfg_poll(&c, NULL, 0, true, false, buf, sizeof buf) == 0);
  gps_cfg_init(NULL);
  CHECK(strcmp(gps_cfg_str(GPS_CFG_OK), "OK") == 0);
  CHECK(strcmp(gps_cfg_str(GPS_CFG_NOACK), "NOACK") == 0);
  (void)sizeof(sim_t);

  /* ---- Kopuk kablo: NOACK döngüsü sınırlı, sessiz, dürüst sayaçlı ---- */
  /* Hiç cevap yok: TRIES deneme -> NOACK, sonra sonsuz sessizlik (TX yok). */
  {
    gps_init(&gm);
    gps_cfg_init(&c);
    int nsend = 0;
    for (t = 0; t < 60000; t += 5)
    {
      if (gps_cfg_poll(&c, &gm, t, true, false, buf, sizeof buf) > 0u)
      {
        nsend++;
      }
    }
    CHECK(c.state == GPS_CFG_NOACK && c.acked == 0 && nsend == (int)GPS_CFG_TRIES);
    /* Terminal sonrası poll: asla çerçeve yok, durum kıpırdamaz (fault yok). */
    for (t = 60000; t < 120000; t += 5)
    {
      CHECK(gps_cfg_poll(&c, &gm, t, true, false, buf, sizeof buf) == 0);
    }
    CHECK(c.state == GPS_CFG_NOACK && c.acked == 0);
    printf("  noack-loop ok\n");
  }

  /* ---- Kısmi NOACK: acked sayacı dürüst (TUI "NOACK n/N" bunu gösterir) ---- */
  {
    gps_init(&gm);
    gps_cfg_init(&c);
    int nsend = 0;
    for (t = 0; t < 60000 && c.state != GPS_CFG_NOACK; t += 5)
    {
      size_t n = gps_cfg_poll(&c, &gm, t, true, false, buf, sizeof buf);
      if (n > 0u)
      {
        nsend++;
        if (c.acked < 3u) /* ilk 3 adım ACK'lenir, sonra hat ölür */
        {
          uint8_t ak[10];
          mk_ack(ak, true, 0x06, buf[3]);
          feed(&gm, ak, 10, t);
        }
      }
    }
    CHECK(c.state == GPS_CFG_NOACK && c.acked == 3u && c.step == 3u);
    CHECK(c.acked <= GPS_CFG_STEPS);
    CHECK(nsend == 3 + (int)GPS_CFG_TRIES);
    printf("  partial-noack-honest ok\n");
  }

  /* ---- NAK: terminal, sessiz, asla fault yok ---- */
  {
    gps_init(&gm);
    gps_cfg_init(&c);
    int nsend = 0;
    for (t = 0; t < 10000; t += 5)
    {
      size_t n = gps_cfg_poll(&c, &gm, t, true, false, buf, sizeof buf);
      if (n > 0u)
      {
        nsend++;
        uint8_t ak[10];
        mk_ack(ak, false, 0x06, buf[3]); /* ilk adım reddedildi */
        feed(&gm, ak, 10, t);
      }
    }
    CHECK(c.state == GPS_CFG_NAK && c.acked == 0 && nsend == 1);
    /* Uçuş başladıktan sonra bile (allowed=false) NAK NAK kalır: poll TX
     * yapmaz, ABORTED'e de çevrilmez (terminal dokunulmaz). */
    for (t = 10000; t < 20000; t += 5)
    {
      CHECK(gps_cfg_poll(&c, &gm, t, false, false, buf, sizeof buf) == 0);
    }
    CHECK(c.state == GPS_CFG_NAK);
    printf("  nak-quiet ok\n");
  }

  /* ---- Çerçeve değişmezi (ayrıştırıcı resync kuralı buna dayanır):
   * adımlarımızın hiçbir ACK/NAK çerçevesinde 0x0A yoktur, yük[0] hep 0x06.
   * Bozulursa gps.c'deki '\n'/'$' abort kuralı gözden geçmelidir. ---- */
  {
    static const uint8_t ids[] = {0x01u, 0x24u, 0x08u};
    for (size_t k = 0; k < sizeof ids; k++)
    {
      for (int v = 0; v < 2; v++)
      {
        uint8_t fr[10];
        CHECK(mk_ack(fr, v != 0, 0x06, ids[k]) == 10u);
        CHECK(ubx_ok(fr, 10));
        CHECK(fr[6] == 0x06u); /* yuk[0] = cevaplanan sinif */
        for (int i = 0; i < 10; i++)
        {
          CHECK(fr[i] != 0x0Au); /* satir sonu bayti yok */
        }
      }
    }
    printf("  ack-frame-invariant ok\n");
  }

  /* ---- Yeniden deneme aralığı: üstel, tavanlı, taşmasız ---- */  {
    CHECK(gps_cfg_retry_gap_ms(0) == GPS_CFG_RETRY_BASE_MS);
    CHECK(gps_cfg_retry_gap_ms(1) == 2u * GPS_CFG_RETRY_BASE_MS);
    CHECK(gps_cfg_retry_gap_ms(2) == GPS_CFG_RETRY_MAX_MS);
    CHECK(gps_cfg_retry_gap_ms(3) == GPS_CFG_RETRY_MAX_MS);
    CHECK(gps_cfg_retry_gap_ms(255) == GPS_CFG_RETRY_MAX_MS);
    CHECK(GPS_CFG_RETRY_BASE_MS <= GPS_CFG_RETRY_MAX_MS);
    CHECK(GPS_CFG_RETRY_MAX_N > 0u && GPS_CFG_RETRY_MAX_N < 255u);
    /* Monoton (azalmaz), hep pozitif. */
    uint32_t prev = 0;
    for (uint16_t r = 0; r <= 20; r++)
    {
      uint32_t gap = gps_cfg_retry_gap_ms((uint8_t)r);
      CHECK(gap >= prev && gap > 0u && gap <= GPS_CFG_RETRY_MAX_MS);
      prev = gap;
    }
    printf("  retry-gap ok\n");
  }

  printf("HOST_GPSCFG_TESTS_PASSED n=%d\n", n_pass);
  return 0;
}
