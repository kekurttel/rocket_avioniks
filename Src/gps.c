/* NMEA byte-stream parser. See gps.h for the validity contract. Pure C,
 * no HAL, no heap, no libc float parsing (locale-independent). */
#include "gps.h"

/* ---- tiny deterministic number parsing (no strtod locale risk) ---- */

static bool gps_parse_uint(const char *s, uint32_t *out)
{
  if (s == NULL || *s == '\0')
  {
    return false;
  }
  uint32_t v = 0;
  for (; *s != '\0'; s++)
  {
    if (*s < '0' || *s > '9')
    {
      return false;
    }
    v = v * 10u + (uint32_t)(*s - '0');
    if (v > 9999999u)
    {
      return false; /* sanity cap: no NMEA field needs more */
    }
  }
  if (out != NULL)
  {
    *out = v;
  }
  return true;
}

/* Optional leading '-', digits with at most one '.', no exponent. */
static bool gps_parse_double(const char *s, double *out)
{
  if (s == NULL || *s == '\0')
  {
    return false;
  }
  bool neg = false;
  if (*s == '-')
  {
    neg = true;
    s++;
  }
  else if (*s == '+')
  {
    s++;
  }
  if (*s == '\0')
  {
    return false;
  }
  double v = 0.0;
  bool any = false;
  while (*s >= '0' && *s <= '9')
  {
    v = v * 10.0 + (*s - '0');
    s++;
    any = true;
  }
  if (*s == '.')
  {
    s++;
    double place = 0.1;
    while (*s >= '0' && *s <= '9')
    {
      v += (*s - '0') * place;
      place *= 0.1;
      s++;
      any = true;
    }
  }
  if (!any || *s != '\0')
  {
    return false;
  }
  if (out != NULL)
  {
    *out = neg ? -v : v;
  }
  return true;
}

/* ddmm.mmmm -> signed decimal degrees * 1e-5. is_lat selects the degree
 * width (2 for latitude, 3 for longitude) and range check. */
static bool gps_parse_degmin(const char *s, char hemi, bool is_lat,
                             int32_t *out_e5)
{
  double raw = 0.0;
  if (!gps_parse_double(s, &raw) || raw < 0.0)
  {
    return false;
  }
  double deg_part;
  double min_part;
  if (is_lat)
  {
    if (raw >= 9000.0)
    {
      return false; /* dd must be < 90 */
    }
    deg_part = (int)(raw / 100.0);
    min_part = raw - deg_part * 100.0;
    if (deg_part > 90.0)
    {
      return false;
    }
  }
  else
  {
    if (raw >= 18000.0)
    {
      return false;
    }
    deg_part = (int)(raw / 100.0);
    min_part = raw - deg_part * 100.0;
    if (deg_part > 180.0)
    {
      return false;
    }
  }
  if (min_part >= 60.0)
  {
    return false;
  }
  int sign = 0;
  if (is_lat)
  {
    if (hemi == 'N')
    {
      sign = 1;
    }
    else if (hemi == 'S')
    {
      sign = -1;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if (hemi == 'E')
    {
      sign = 1;
    }
    else if (hemi == 'W')
    {
      sign = -1;
    }
    else
    {
      return false;
    }
  }
  double deg = deg_part + min_part / 60.0;
  /* Round to 1e-5 deg (~1.1 m latitude: below NMEA noise, no fake
   * precision, exact integer telemetry). */
  int32_t e5 = (int32_t)(deg * 100000.0 + 0.5);
  if (out_e5 != NULL)
  {
    *out_e5 = (sign < 0) ? -e5 : e5;
  }
  return true;
}

/* hhmmss[.ss] -> h/m/s. */
static bool gps_parse_time(const char *s, uint8_t *h, uint8_t *m, uint8_t *sec)
{
  if (s == NULL)
  {
    return false;
  }
  /* Need at least 6 digits; ignore fractional part. */
  for (int i = 0; i < 6; i++)
  {
    if (s[i] < '0' || s[i] > '9')
    {
      return false;
    }
  }
  uint8_t hh = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
  uint8_t mm = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
  uint8_t ss = (uint8_t)((s[4] - '0') * 10 + (s[5] - '0'));
  if (hh > 23u || mm > 59u || ss > 60u)
  {
    return false; /* 60 admits a leap second */
  }
  if (h != NULL)
  {
    *h = hh;
  }
  if (m != NULL)
  {
    *m = mm;
  }
  if (sec != NULL)
  {
    *sec = ss;
  }
  return true;
}

/* ddmmyy -> d/m/y. */
static bool gps_parse_date(const char *s, uint8_t *d, uint8_t *mo,
                           uint8_t *y)
{
  if (s == NULL)
  {
    return false;
  }
  for (int i = 0; i < 6; i++)
  {
    if (s[i] < '0' || s[i] > '9')
    {
      return false;
    }
  }
  uint8_t dd = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
  uint8_t mm = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
  uint8_t yy = (uint8_t)((s[4] - '0') * 10 + (s[5] - '0'));
  if (dd < 1u || dd > 31u || mm < 1u || mm > 12u)
  {
    return false;
  }
  if (d != NULL)
  {
    *d = dd;
  }
  if (mo != NULL)
  {
    *mo = mm;
  }
  if (y != NULL)
  {
    *y = yy;
  }
  return true;
}

static uint8_t gps_hexval(char c)
{
  if (c >= '0' && c <= '9')
  {
    return (uint8_t)(c - '0');
  }
  if (c >= 'A' && c <= 'F')
  {
    return (uint8_t)(c - 'A' + 10);
  }
  if (c >= 'a' && c <= 'f')
  {
    return (uint8_t)(c - 'a' + 10);
  }
  return 0xFFu;
}

/* Split a complete line in place: commas and '*' become NUL, field table
 * filled. Returns field count. Needs at least the '*' + 2 hex digits. */
#define GPS_MAX_FIELDS (24u)

static uint8_t gps_split(char *line, uint8_t n, char *f[GPS_MAX_FIELDS],
                         uint8_t *xsum)
{
  uint8_t nf = 0;
  uint8_t xor = 0;
  uint8_t star = n; /* index of '*' */
  for (uint8_t i = 1; i < n; i++)
  {
    if (line[i] == '*')
    {
      star = i;
      break;
    }
    xor ^= (uint8_t)line[i];
  }
  if (star + 2u >= n)
  {
    return 0; /* no room for '*hh' */
  }
  uint8_t hi = gps_hexval(line[star + 1u]);
  uint8_t lo = gps_hexval(line[star + 2u]);
  if (hi > 15u || lo > 15u)
  {
    return 0;
  }
  if (((hi << 4) | lo) != xor)
  {
    return 0; /* checksum failure: not our sentence */
  }
  line[star] = '\0';
  if (xsum != NULL)
  {
    *xsum = xor;
  }
  f[0] = &line[0];
  nf = 1;
  for (uint8_t i = 0; i < star && nf < GPS_MAX_FIELDS; i++)
  {
    if (line[i] == ',')
    {
      line[i] = '\0';
      f[nf++] = &line[i + 1u];
    }
  }
  return nf;
}

static const char *gps_field(char *f[], uint8_t nf, uint8_t idx)
{
  if (idx >= nf || f[idx] == NULL)
  {
    return "";
  }
  return f[idx];
}

void gps_init(gps_t *g)
{
  if (g == NULL)
  {
    return;
  }
  g->present = false;
  g->fix = false;
  g->valid = false;
  g->sats = 0;
  g->lat_e5 = 0;
  g->lon_e5 = 0;
  g->alt_m = 0.0f;
  g->alt_valid = false;
  g->speed_mps = 0.0f;
  g->speed_valid = false;
  g->course_deg = 0.0f;
  g->course_valid = false;
  g->hour = 0;
  g->min = 0;
  g->sec = 0;
  g->time_valid = false;
  g->day = 0;
  g->mon = 0;
  g->year = 0;
  g->date_valid = false;
  g->last_update_ms = 0;
  g->last_fix_ms = 0;
  g->rx_bytes = 0;
  g->n = 0;
  g->overflow = false;
  g->ubx_state = 0;
  g->ubx_cls = 0;
  g->ubx_id = 0;
  g->ubx_ck_a = 0;
  g->ubx_ck_b = 0;
  g->ubx_len = 0;
  g->ubx_idx = 0;
  for (uint32_t i = 0; i < GPS_UBX_PL_MAX; i++)
  {
    g->ubx_pl[i] = 0;
  }
  g->ack_seq = 0;
  g->ack_cls = 0;
  g->ack_id = 0;
  g->ack_ok = false;
  g->ack_count = 0;
  g->nak_count = 0;
  for (uint32_t i = 0; i < GPS_LINE_MAX; i++)
  {
    g->line[i] = '\0';
  }
}

bool gps_is_stale(const gps_t *g, uint32_t now_ms)
{
  if (g == NULL || !g->present)
  {
    return true;
  }
  return (now_ms - g->last_fix_ms) > GPS_STALE_MS;
}

/* GGA: $xxGGA,time,lat,NS,lon,EW,quality,sats,...,alt,unit,... */
static void gps_parse_gga(gps_t *g, char *f[], uint8_t nf, uint32_t now_ms)
{
  if (nf < 11u)
  {
    return; /* incomplete: presence only */
  }
  const char *t = gps_field(f, nf, 1);
  const char *la = gps_field(f, nf, 2);
  const char *ns = gps_field(f, nf, 3);
  const char *lo = gps_field(f, nf, 4);
  const char *ew = gps_field(f, nf, 5);
  const char *q = gps_field(f, nf, 6);
  const char *sa = gps_field(f, nf, 7);
  const char *al = gps_field(f, nf, 9);
  g->last_update_ms = now_ms;
  g->present = true;
  uint8_t hh = 0, mm = 0, ss = 0;
  if (gps_parse_time(t, &hh, &mm, &ss))
  {
    g->hour = hh;
    g->min = mm;
    g->sec = ss;
    g->time_valid = true;
  }
  uint32_t sats = 0;
  if (gps_parse_uint(sa, &sats) && sats <= 99u)
  {
    g->sats = (uint8_t)sats;
  }
  /* Fix quality '0' = invalid. Anything else still needs coordinates. */
  if (q[0] == '\0' || q[0] == '0')
  {
    g->fix = false;
    g->valid = false;
    return;
  }
  int32_t lat = 0, lon = 0;
  if (ns[0] == '\0' || ew[0] == '\0' ||
      !gps_parse_degmin(la, ns[0], true, &lat) ||
      !gps_parse_degmin(lo, ew[0], false, &lon))
  {
    g->fix = false;
    g->valid = false;
    return;
  }
  double alt = 0.0;
  bool alt_ok = gps_parse_double(al, &alt) && alt > -1000.0 && alt < 100000.0;
  g->lat_e5 = lat;
  g->lon_e5 = lon;
  g->fix = true;
  g->valid = true;
  g->last_fix_ms = now_ms;
  if (alt_ok)
  {
    g->alt_m = (float)alt;
    g->alt_valid = true;
  }
}

/* RMC: $xxRMC,time,status,lat,NS,lon,EW,speed,course,date,... */
static void gps_parse_rmc(gps_t *g, char *f[], uint8_t nf, uint32_t now_ms)
{
  if (nf < 10u)
  {
    return; /* incomplete: presence only */
  }
  const char *t = gps_field(f, nf, 1);
  const char *st = gps_field(f, nf, 2);
  const char *la = gps_field(f, nf, 3);
  const char *ns = gps_field(f, nf, 4);
  const char *lo = gps_field(f, nf, 5);
  const char *ew = gps_field(f, nf, 6);
  const char *sp = gps_field(f, nf, 7);
  const char *co = gps_field(f, nf, 8);
  const char *da = gps_field(f, nf, 9);
  g->last_update_ms = now_ms;
  g->present = true;
  uint8_t hh = 0, mm = 0, ss = 0;
  if (gps_parse_time(t, &hh, &mm, &ss))
  {
    g->hour = hh;
    g->min = mm;
    g->sec = ss;
    g->time_valid = true;
  }
  uint8_t dd = 0, mo = 0, yy = 0;
  if (gps_parse_date(da, &dd, &mo, &yy))
  {
    g->day = dd;
    g->mon = mo;
    g->year = yy;
    g->date_valid = true;
  }
  if (st[0] != 'A')
  {
    g->fix = false;
    g->valid = false;
    return; /* 'V' = no fix: data below is void */
  }
  int32_t lat = 0, lon = 0;
  if (ns[0] == '\0' || ew[0] == '\0' ||
      !gps_parse_degmin(la, ns[0], true, &lat) ||
      !gps_parse_degmin(lo, ew[0], false, &lon))
  {
    g->fix = false;
    g->valid = false;
    return;
  }
  double spd = 0.0, crs = 0.0;
  bool spd_ok = gps_parse_double(sp, &spd) && spd >= 0.0 && spd < 5000.0;
  bool crs_ok = gps_parse_double(co, &crs) && crs >= 0.0 && crs < 360.0;
  g->lat_e5 = lat;
  g->lon_e5 = lon;
  g->fix = true;
  g->valid = true;
  g->last_fix_ms = now_ms;
  if (spd_ok)
  {
    g->speed_mps = (float)spd * GPS_KNOTS_TO_MPS;
    g->speed_valid = true;
  }
  if (crs_ok)
  {
    g->course_deg = (float)crs;
    g->course_valid = true;
  }
}

static void gps_process_line(gps_t *g, uint32_t now_ms)
{
  if (g->n < 9u || g->line[0] != '$')
  {
    return; /* too short to hold '$xxXXX*hh' */
  }
  char *f[GPS_MAX_FIELDS];
  uint8_t nf = gps_split(g->line, g->n, f, NULL);
  if (nf < 2u)
  {
    return; /* no checksum or checksum failure */
  }
  /* Talker-agnostic: bytes 3..5 select the sentence (GPGGA, GNGGA, ...). */
  if (f[0][0] != '$' || f[0][1] == '\0' || f[0][2] == '\0' || f[0][3] == '\0')
  {
    return;
  }
  bool is_gga = (f[0][3] == 'G' && f[0][4] == 'G' && f[0][5] == 'A');
  bool is_rmc = (f[0][3] == 'R' && f[0][4] == 'M' && f[0][5] == 'C');
  if (is_gga)
  {
    gps_parse_gga(g, f, nf, now_ms);
  }
  else if (is_rmc)
  {
    gps_parse_rmc(g, f, nf, now_ms);
  }
  else
  {
    /* Other checksum-valid NMEA (GSV, GLL, VTG, TXT...): the module is
     * alive and talking. Presence only, content ignored by design. */
    g->last_update_ms = now_ms;
    g->present = true;
  }
}

/* ---- UBX reply tracker (ACK/NAK to our own config frames) ----
 * Frame: B5 62 cls id lenL lenH payload ckA ckB. Only the start-of-line
 * position can begin a frame (g->n == 0: no NMEA sentence in progress), so
 * binary bytes can never leak into, or corrupt, an NMEA sentence. Frames
 * longer than GPS_UBX_PL_MAX are dropped (we never enable other UBX
 * output). Returns true when the byte was consumed by the UBX side. */
#define UBX_SYNC1 (0xB5u)
#define UBX_SYNC2 (0x62u)
#define UBX_CLS_ACK (0x05u)

static void ubx_sum(gps_t *g, uint8_t b)
{
  g->ubx_ck_a = (uint8_t)(g->ubx_ck_a + b);
  g->ubx_ck_b = (uint8_t)(g->ubx_ck_b + g->ubx_ck_a);
}

static bool gps_ubx_feed(gps_t *g, uint8_t b)
{
  /* Bozuk kablo resync: yarım kalmış bir UBX başlığının içine NMEA satır
   * sınırı girerse UBX tarafı çekilir, bayt NMEA birleştiriciye bırakılır.
   *   '\n' (0x0A): durum >= 2'de hep abort. Bizim hiçbir ACK/NAK çerçevemizde
   *     0x0A baytı yoktur (adım kimlikleri + Fletcher el hesabı, host test
   *     kilitli): sahte B5 62 eşleşmesi NMEA satır sonunu yutamaz.
   *   '$' (0x24): yalnızca başlıkta (durum 0..5) ve ilk yük baytında abort.
   *     Yük[0] bizde hep 0x06'dır (config adımlarının hepsi 0x06 sınıfı), ama
   *     yük[1] 0x24 olabilir (NAV5 cevabı) ve checksum baytı da olabilir; o
   *     yüzden yük[1]/checksum konumunda '$' tüketilir (doğrulama karar verir).
   * Kopuk hattın sahte senkronu arkadan gelen sağlam cümleyi yutamaz; gerçek
   * ACK'ler bayt bayt aynı kaldığı için tanınmaya devam eder. */
  if (b == (uint8_t)'\n' && g->ubx_state >= 2u)
  {
    g->ubx_state = 0;
    return false;
  }
  if (b == (uint8_t)'$' &&
      (g->ubx_state <= 5u || (g->ubx_state == 6u && g->ubx_idx == 0u)))
  {
    g->ubx_state = 0;
    return false;
  }
  switch (g->ubx_state)
  {
    case 0:
      if (b == UBX_SYNC1 && g->n == 0u && !g->overflow)
      {
        g->ubx_state = 1;
        return true;
      }
      return false;
    case 1:
      if (b == UBX_SYNC2)
      {
        g->ubx_state = 2;
        g->ubx_ck_a = 0;
        g->ubx_ck_b = 0;
        return true;
      }
      if (b == UBX_SYNC1)
      {
        return true; /* art arda B5: senkronu koru, NMEA tamponunu kirletme */
      }
      g->ubx_state = 0; /* not a UBX frame after all: normal data */
      return false;
    case 2:
      g->ubx_cls = b;
      ubx_sum(g, b);
      g->ubx_state = 3;
      return true;
    case 3:
      g->ubx_id = b;
      ubx_sum(g, b);
      g->ubx_state = 4;
      return true;
    case 4:
      g->ubx_len = b;
      ubx_sum(g, b);
      g->ubx_state = 5;
      return true;
    case 5:
      g->ubx_len = (uint16_t)(g->ubx_len | ((uint16_t)b << 8));
      ubx_sum(g, b);
      if (g->ubx_len > GPS_UBX_PL_MAX)
      {
        g->ubx_state = 0; /* not ours: drop */
        return true;
      }
      g->ubx_idx = 0;
      g->ubx_state = (g->ubx_len == 0u) ? 7 : 6;
      return true;
    case 6:
      g->ubx_pl[g->ubx_idx++] = b;
      ubx_sum(g, b);
      if (g->ubx_idx >= g->ubx_len)
      {
        g->ubx_state = 7;
      }
      return true;
    case 7:
      g->ubx_state = (b == g->ubx_ck_a) ? 8 : 0;
      return true;
    case 8:
      if (b == g->ubx_ck_b && g->ubx_cls == UBX_CLS_ACK &&
          g->ubx_len == 2u && (g->ubx_id == 0x01u || g->ubx_id == 0x00u))
      {
        g->ack_cls = g->ubx_pl[0];
        g->ack_id = g->ubx_pl[1];
        g->ack_ok = (g->ubx_id == 0x01u);
        g->ack_seq++;
        if (g->ack_ok)
        {
          g->ack_count++;
        }
        else
        {
          g->nak_count++;
        }
      }
      g->ubx_state = 0;
      return true;
    default:
      g->ubx_state = 0;
      return false;
  }
}

void gps_rx_byte(gps_t *g, uint8_t b, uint32_t now_ms)
{
  if (g == NULL)
  {
    return;
  }
  if (g->rx_bytes < 0xFFFFFFFFu)
  {
    g->rx_bytes++;
  }
  if (gps_ubx_feed(g, b))
  {
    return;
  }
  if (b == '\n')
  {
    if (!g->overflow && g->n > 0u)
    {
      gps_process_line(g, now_ms);
    }
    g->n = 0;
    g->overflow = false;
    return;
  }
  if (b == '\r')
  {
    return; /* NMEA CRLF: carriage return carries no data */
  }
  /* Bozuk kablo filtresi: NMEA yalnızca yazdırılabilir ASCII'dir (0x20..0x7E
   * + CR/LF). Hattın ürettiği ikili çöp (kopuk kabloda 0xB5/0xFF/0x00 gibi)
   * satır tamponuna yazılmaz: checksum'suz/yarım satırlar sessizce atılır,
   * sağlam cümleler kirlenmeden birleşir. UBX tarafının tükettiği baytlar
   * buraya zaten uğramaz; konum/fix/sats alanlarına yazma kuralı değişmez. */
  if (b < 0x20u || b > 0x7Eu)
  {
    return;
  }
  if (g->overflow)
  {
    return; /* dropping an over-long line until '\n' resyncs */
  }
  if (b == '$')
  {
    g->n = 0; /* resync: a new sentence aborts a partial one */
  }
  if (g->n < GPS_LINE_MAX - 1u)
  {
    g->line[g->n++] = (char)b;
  }
  else
  {
    g->overflow = true; /* over-long: never a valid NMEA sentence */
  }
}

/* ===================== GPS configuration (board TX -> GPS RX) ===================== */

/* UBX frame: B5 62 cls id lenL lenH payload ckA ckB; checksum = 8-bit
 * Fletcher over cls..payload. `pl_len` payload bytes must already sit at
 * out[6..]. Returns the total frame length. */
static size_t ubx_finish(uint8_t *out, uint8_t cls, uint8_t id, uint16_t pl_len)
{
  out[0] = 0xB5u;
  out[1] = 0x62u;
  out[2] = cls;
  out[3] = id;
  out[4] = (uint8_t)(pl_len & 0xFFu);
  out[5] = (uint8_t)(pl_len >> 8);
  uint8_t a = 0;
  uint8_t b = 0;
  for (uint16_t i = 0; i < (uint16_t)(4u + pl_len); i++)
  {
    a = (uint8_t)(a + out[2u + i]);
    b = (uint8_t)(b + a);
  }
  out[6u + pl_len] = a;
  out[7u + pl_len] = b;
  return (size_t)(8u + pl_len);
}

size_t gps_ubx_cfg_msg(uint8_t *out, size_t cap, uint8_t msg_cls,
                       uint8_t msg_id, uint8_t rate)
{
  if (out == NULL || cap < 14u)
  {
    return 0;
  }
  out[6] = msg_cls;
  out[7] = msg_id;
  out[8] = 0u;  /* rateDDC: unused */
  out[9] = rate; /* rate on UART1 (the port the board listens on);
                  * DDC/UART2/USB slots stay 0 */
  out[10] = 0u; /* rateUART2: unused */
  out[11] = 0u; /* rateUSB: unused */
  return ubx_finish(out, 0x06u, 0x01u, 6u);
}

size_t gps_ubx_cfg_rate(uint8_t *out, size_t cap, uint16_t meas_ms)
{
  if (out == NULL || cap < 14u || meas_ms < 1000u)
  {
    return 0; /* gen6 CFG-RATE floor = 1000 ms (NEO-6M is a 1 Hz part) */
  }
  out[6] = (uint8_t)(meas_ms & 0xFFu); /* measRate ms, LE */
  out[7] = (uint8_t)(meas_ms >> 8);
  out[8] = 1; /* navRate: one fix per measurement */
  out[9] = 0;
  out[10] = 1; /* timeRef: 1 = GPS time */
  out[11] = 0;
  return ubx_finish(out, 0x06u, 0x08u, 6u);
}

size_t gps_ubx_cfg_nav5_dyn(uint8_t *out, size_t cap, uint8_t dyn_model)
{
  if (out == NULL || cap < 44u || dyn_model > 7u)
  {
    return 0;
  }
  for (uint32_t i = 0; i < 36u; i++)
  {
    out[6u + i] = 0; /* every other NAV5 field ignored (mask below) */
  }
  out[6] = 0x01u; /* mask bit0 = apply dynModel only (U2, LE) */
  out[7] = 0x00u;
  out[8] = dyn_model; /* dynModel at payload offset 2 */
  return ubx_finish(out, 0x06u, 0x24u, 36u);
}

/* Step table: NMEA ids are class 0xF0 (GGA 00, GLL 01, GSA 02, GSV 03,
 * RMC 04, VTG 05). */
size_t gps_cfg_frame(uint8_t step, uint8_t *out, size_t cap,
                     uint8_t *ack_cls, uint8_t *ack_id)
{
  size_t n = 0;
  uint8_t ac = 0x06u;
  uint8_t ai = 0x01u; /* CFG-MSG */
  switch (step)
  {
    case 0:
      n = gps_ubx_cfg_msg(out, cap, 0xF0u, 0x00u, 1u); /* GGA on */
      break;
    case 1:
      n = gps_ubx_cfg_msg(out, cap, 0xF0u, 0x04u, 1u); /* RMC on */
      break;
    case 2:
      n = gps_ubx_cfg_msg(out, cap, 0xF0u, 0x01u, 0u); /* GLL off */
      break;
    case 3:
      n = gps_ubx_cfg_msg(out, cap, 0xF0u, 0x02u, 0u); /* GSA off */
      break;
    case 4:
      n = gps_ubx_cfg_msg(out, cap, 0xF0u, 0x03u, 0u); /* GSV off */
      break;
    case 5:
      n = gps_ubx_cfg_msg(out, cap, 0xF0u, 0x05u, 0u); /* VTG off */
      break;
    case 6:
      n = gps_ubx_cfg_nav5_dyn(out, cap, (uint8_t)GPS_CFG_DYNMODEL);
      ai = 0x24u;
      break;
    case 7:
      n = gps_ubx_cfg_rate(out, cap, (uint16_t)GPS_CFG_RATE_MS);
      ai = 0x08u;
      break;
    default:
      return 0;
  }
  if (n > 0u)
  {
    if (ack_cls != NULL)
    {
      *ack_cls = ac;
    }
    if (ack_id != NULL)
    {
      *ack_id = ai;
    }
  }
  return n;
}

void gps_cfg_init(gps_cfg_t *c)
{
  if (c == NULL)
  {
    return;
  }
  c->state = GPS_CFG_IDLE;
  c->step = 0;
  c->tries = 0;
  c->acked = 0;
  c->waiting = false;
  c->seq_at_send = 0;
  c->sent_ms = 0;
  c->ready_ms = 0;
}

/* Ölü tur aralığı: BASE << round, MAX tavanlı. Kaydırmasız katlama (8 bit
 * round doysa bile taşma yok): 0->120 sn, 1->240 sn, >=2->480 sn. */
uint32_t gps_cfg_retry_gap_ms(uint8_t round)
{
  uint32_t gap = (uint32_t)GPS_CFG_RETRY_BASE_MS;
  while (round-- > 0u)
  {
    if (gap >= (uint32_t)GPS_CFG_RETRY_MAX_MS / 2u)
    {
      gap = (uint32_t)GPS_CFG_RETRY_MAX_MS;
      break;
    }
    gap *= 2u;
  }
  return gap;
}

size_t gps_cfg_poll(gps_cfg_t *c, const gps_t *g, uint32_t now_ms,
                    bool allowed, bool tx_busy, uint8_t *out, size_t cap)
{
  if (c == NULL || g == NULL || out == NULL)
  {
    return 0;
  }
  if (c->state != GPS_CFG_IDLE && c->state != GPS_CFG_SENDING)
  {
    return 0; /* terminal: OK / NAK / NOACK / ABORTED */
  }
  if (!allowed)
  {
    c->state = GPS_CFG_ABORTED; /* never TX once the rocket left the pad */
    c->waiting = false;
    return 0;
  }
  if (tx_busy)
  {
    return 0;
  }
  if (c->state == GPS_CFG_IDLE)
  {
    c->state = GPS_CFG_SENDING;
    c->step = 0;
    c->tries = 0;
    c->waiting = false;
  }

  uint8_t ack_cls = 0;
  uint8_t ack_id = 0;

  if (c->waiting)
  {
    if (g->ack_seq != c->seq_at_send)
    {
      /* A reply arrived since the send: is it for this step? */
      (void)gps_cfg_frame(c->step, out, cap, &ack_cls, &ack_id);
      if (g->ack_cls == ack_cls && g->ack_id == ack_id)
      {
        c->waiting = false;
        if (!g->ack_ok)
        {
          /* Modül adımı reddetti: NAK terminali (adım içinde tekrar yok).
           * Yeniden deneme bir üst seviyededir: main.c retry bloğu pad
           * başında tüm diziyi tavanlı aralıkla baştan başlatır. Fault yok. */
          c->state = GPS_CFG_NAK;
          return 0;
        }
        c->acked++;
        c->step++;
        c->tries = 0;
        if (c->step >= GPS_CFG_STEPS)
        {
          c->state = GPS_CFG_OK;
          return 0;
        }
        c->ready_ms = now_ms + GPS_CFG_GAP_MS;
        return 0;
      }
      c->seq_at_send = g->ack_seq; /* unrelated reply: keep waiting */
    }
    if ((uint32_t)(now_ms - c->sent_ms) < GPS_CFG_ACK_TMO_MS)
    {
      return 0;
    }
    if (c->tries >= GPS_CFG_TRIES)
    {
      /* Adım başına üst sınır doldu: sessiz NOACK terminali. Fault değil,
       * uçuşa etki yok; pad başındaysa main.c retry bloğu (üstel aralıklı,
       * tur sayaçlı) yeniden başlatır, uçuşta/pad dışında kimse dokunmaz. */
      c->state = GPS_CFG_NOACK;
      c->waiting = false;
      return 0;
    }
    c->waiting = false; /* timed out: send the same step again */
  }

  /* Inter-step gap. wrap-safe: unsigned compare, HAL tick half-range is the
   * future/now boundary (a small delta is "now >= ready_ms", a huge one means
   * ready_ms is still ahead). `acked > 0` because ready_ms is only a real
   * deadline once a step has ACKed: the init value 0 must not freeze the
   * sequencer once uptime passes 2^31 ms (24.85 days). */
  if (c->acked > 0u && (uint32_t)(now_ms - c->ready_ms) > 0x7FFFFFFFu)
  {
    return 0; /* inter-step gap */
  }
  size_t n = gps_cfg_frame(c->step, out, cap, &ack_cls, &ack_id);
  if (n == 0u)
  {
    c->state = GPS_CFG_NAK; /* internal error (cap too small): stop safely */
    return 0;
  }
  c->tries++;
  c->waiting = true;
  c->sent_ms = now_ms;
  c->seq_at_send = g->ack_seq;
  return n;
}

const char *gps_cfg_str(gps_cfg_state_t s)
{
  switch (s)
  {
    case GPS_CFG_IDLE:
      return "IDLE";
    case GPS_CFG_SENDING:
      return "SENDING";
    case GPS_CFG_OK:
      return "OK";
    case GPS_CFG_NAK:
      return "NAK";
    case GPS_CFG_NOACK:
      return "NOACK";
    case GPS_CFG_ABORTED:
      return "ABORTED";
    default:
      return "?";
  }
}
