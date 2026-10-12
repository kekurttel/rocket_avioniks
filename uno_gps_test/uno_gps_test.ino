/*
 * uno_gps_test.ino — NEO-6M (u-blox 6) HAM teşhis. Kütüphane YOK.
 *
 * NEDEN KÜTÜPHANESİZ: Bu testin sorusu "modül mü bozuk, kablo mu bozuk?".
 * Bir kütüphane bozuk/yanlış-parse edilen veriyi sessizce yutar ve soruya
 * cevap veremezsin. Ham NMEA + checksum sonucu en güçlü kanıt: hangi cümlenin
 * geldiğini, checksum'in tuttuğunu, saniyede kaç bayt geldiğini doğrudan görürüz.
 *
 * BAĞLAMA (NEO-6M -> Uno):
 *   NEO-6M VCC -> Uno 5V
 *   NEO-6M GND -> Uno GND
 *   NEO-6M TX  -> Uno D0   (donanım RX0 — SoftwareSerial'a GEREK YOK)
 *   NEO-6M RX  -> Uno D1   (opsiyonel; tek yönlü testte kullanılmaz)
 *
 * Serial Monitor 115200.
 *
 * OKUMA KILAVUZU:
 *   [RATE] 60+ B/s      → kablo sağlam, modül konuşuyor
 *   [+] ...             → checksum DOĞRU, cümle geçerli
 *   [-] ...             → checksum BOZUK: yanlış baud veya hat gürültüsü
 *   [RATE] 0 B/s        → modül BESLEME veya TX kablosu kopuk
 */

static const uint32_t SERIAL_BAUD = 115200UL; /* Uno -> PC (Serial Monitor) */
static const uint32_t GPS_BAUD = 9600UL;     /* NEO-6M -> Uno (fabrika cikisi) */

/* Donanım UART'ı: TX=D1, RX=D0. GPS TX'ini D0'a bagladigimiz icin
 * yazilim tarafinda hicbir sey yapmaya gerek yok. */
static HardwareSerial &GPS = Serial;

static uint32_t rx_bytes = 0;   /* gecen toplam ham bayt */
static uint32_t rx_1s = 0;      /* bu saniyedeki bayt */
static uint32_t last_report_ms = 0;
static uint32_t line_bad = 0;   /* checksum bozuk cumle sayisi */
static uint32_t line_ok = 0;    /* checksum dogru cumle sayisi */
static uint8_t  last_sats = 0;  /* son GGA'daki uydu sayisi */
static bool     had_fix = false;

static char    line[96];        /* '$' haric, sadece cumle govdesi */
static uint8_t line_len = 0;

/* Tek yonlu yazdirma; ic ice cagri yok. */
static void put(const char *s)
{
  Serial.write(s);
}

static void put_u32(uint32_t v)
{
  char b[11];
  uint8_t i = 0;
  if (v == 0u)
  {
    Serial.write('0');
    return;
  }
  while ((v > 0u) && (i < (uint8_t)sizeof b))
  {
    b[i++] = (char)('0' + (v % 10u));
    v /= 10u;
  }
  while (i > 0u)
  {
    Serial.write(b[--i]);
  }
}

static uint8_t hexval(char c)
{
  if ((c >= '0') && (c <= '9')) { return (uint8_t)(c - '0'); }
  if ((c >= 'A') && (c <= 'F')) { return (uint8_t)(c - 'A' + 10); }
  if ((c >= 'a') && (c <= 'f')) { return (uint8_t)(c - 'a' + 10); }
  return 0xFFu;
}

/* line = cumle govdesi, '$' ve '*' HARIC. '*' varsa:
 *   x = XOR(line[0..star-1]);  beklenen = line[star+1], line[star+2]
 * '*' yoksa gecerli NMEA degil. */
static bool checksum_ok(const char *s, uint8_t len)
{
  uint8_t star = 0;
  bool found = false;

  for (uint8_t i = 0u; i < len; i++)
  {
    if (s[i] == '*')
    {
      star = i;
      found = true;
      break;
    }
  }
  if (!found || (star + 2u) >= len)
  {
    return false;
  }

  uint8_t x = 0;
  for (uint8_t i = 0u; i < star; i++)
  {
    x ^= (uint8_t)s[i];
  }

  uint8_t hi = hexval(s[star + 1u]);
  uint8_t lo = hexval(s[star + 2u]);
  if ((hi == 0xFFu) || (lo == 0xFFu))
  {
    return false;
  }
  return (uint8_t)((hi << 4) | lo) == x;
}

/* GGA'nin 7. alani uydu sayisidir.
 * Govde: "GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,xxx.x,M,..."
 *   alan 0 = GPGGA   alan 6 = fix quality   alan 7 = sats
 * GPGGA olarak basmak istiyoruz (saglam adayi gormek icin). */
static bool is_gga(const char *s, uint8_t len)
{
  return (len >= 6u) && (s[2] == 'G') && (s[3] == 'G') && (s[4] == 'G') && (s[5] == 'A');
}

static bool is_rmc(const char *s, uint8_t len)
{
  return (len >= 6u) && (s[2] == 'R') && (s[3] == 'M') && (s[4] == 'C') &&
         ((s[0] == 'G') || (s[0] == 'N'));
}

static uint8_t gga_sats(const char *s, uint8_t len)
{
  uint8_t field = 0;
  uint16_t val = 0;
  bool any = false;

  for (uint8_t i = 0u; i < len; i++)
  {
    if (s[i] == ',')
    {
      field++;
      if (field == 7u)
      {
        /* bu alanin rakamlarini topla */
        return (any && (val <= 99u)) ? (uint8_t)val : 0u;
      }
      val = 0;
      any = false;
    }
    else if ((field == 7u) && (s[i] >= '0') && (s[i] <= '9'))
    {
      val = (uint16_t)((val * 10u) + (uint8_t)(s[i] - '0'));
      any = true;
    }
  }
  return ((field == 7u) && any && (val <= 99u)) ? (uint8_t)val : 0u;
}

/* RMC'de fix bayragi: govde "GNRMC,hhmmss.ss,A,llll.ll,..." -> alan 2 = 'A' */
static bool rmc_valid(const char *s, uint8_t len)
{
  uint8_t field = 0;
  for (uint8_t i = 0u; i < len; i++)
  {
    if (s[i] == ',')
    {
      field++;
      if (field == 2u)
      {
        return (i + 1u < len) && (s[i + 1u] == 'A');
      }
    }
  }
  return false;
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  GPS.begin(GPS_BAUD);
  delay(400); /* power-on self-check suresi */

  put("\n=== NEO-6M HAM TEST (kutuphanesiz) ===\n");
  put("baglama: VCC->5V  GND->GND  TX->D0   GPS baud=9600\n");
  put("[+] = checksum dogru   [-] = bozuk   [RATE] 0 B/s = hat sessiz\n\n");
}

void loop()
{
  while (GPS.available())
  {
    uint8_t c = GPS.read();

    rx_bytes++;
    rx_1s++;

    if (c == (uint8_t)'$')
    {
      line_len = 0u; /* yeni cumle: eskiyi kes */
      continue;
    }

    if (line_len == 0u)
    {
      continue; /* '$' gelmeden bayt: gürültü veya başka format */
    }

    if (c == (uint8_t)'\r')
    {
      continue;
    }

    if (c == (uint8_t)'\n')
    {
      line[line_len] = '\0';
      bool ok = checksum_ok(line, line_len);

      if (ok)
      {
        line_ok++;
      }
      else
      {
        line_bad++;
      }

      /* Sadece anlamli cumleleri bas: GSV/GSA tekrarini basmak ekrani bogar. */
      if (is_gga(line, line_len))
      {
        last_sats = gga_sats(line, line_len);
        put(ok ? "[+] " : "[-] ");
        Serial.write(line, line_len);
        put("\n    sats=");
        put_u32(last_sats);
        put("\n");
      }
      else if (is_rmc(line, line_len) && rmc_valid(line, line_len))
      {
        had_fix = true;
        put(ok ? "[+] " : "[-] ");
        Serial.write(line, line_len);
        put("\n    >>> FIX ALINDI <<<\n");
      }
      else if (!ok)
      {
        /* bozuk cumleyi de goster: yanlis baud bunu uretir */
        put("[-] ");
        Serial.write(line, line_len);
        put("\n");
      }

      line_len = 0u;
      continue;
    }

    if (line_len < (uint8_t)(sizeof line) - 1u)
    {
      line[line_len++] = (char)c;
    }
    /* asiri uzun cumle: kirp, sonraki '$' ile resync */
  }

  unsigned long now = millis();
  if ((now - last_report_ms) >= 1000UL)
  {
    last_report_ms = now;
    put("[RATE] ");
    put_u32(rx_1s);
    put(" B/s  toplam=");
    put_u32(rx_bytes);
    put("  ok=");
    put_u32(line_ok);
    put(" bozuk=");
    put_u32(line_bad);
    if (rx_1s == 0u)
    {
      put("   <<< HAT SESSIZ: VCC veya TX kablosu kopuk\n");
    }
    else if (!had_fix && last_sats == 0u)
    {
      put("   (uydu bekleniyor - anteni gokyuzune cikar)\n");
    }
    else
    {
      put("\n");
    }
    rx_1s = 0;
  }
}