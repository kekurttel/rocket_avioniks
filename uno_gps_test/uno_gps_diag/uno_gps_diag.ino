/*
 * uno_gps_diag.ino — NEO-6M teşhis, D0 (donanim UART) + baud zorlamasinda
 * yazi bozulmasinin cozumu.
 *
 * SORUN: GPS.begin(9600) donanim Serial'ini da 9600'a cevirir. Bu yuzden
 * hem GPS verisi hem PC cikisi ayni hattan 9600'da gider. PC 115200'de
 * dinleyince hicbiri okunmaz (v1'de oldu).
 *
 * COZUM: (a) GPS verisini D0'dan oku ama HICBIR SEY basma — sadece say.
 *        (b) Tarama bitince Serial.end() + Serial.begin(115200) ile
 *        cikis hattini geri 115200'a al ve sonucu O ZAMAN bas.
 *
 * Tek kisit: GPS 9600'da modulen surekli konusuyorsa, arada 115200'a
 * gecmek 1-2 bayt kaybettirir — bu teşhis icin onemsizdir.
 *
 * BAĞLAMA:
 *   NEO-6M VCC -> 5V   GND -> GND   TX -> D0 (donanim RX)
 *
 * Serial Monitor 115200
 */

static const uint32_t GPS_BAUD = 9600UL; /* degistirmek istersen burasi */

static uint32_t rx = 0;
static uint32_t cnt_dollar = 0;
static uint32_t cnt_nl = 0;
static uint32_t cnt_star = 0;
static uint32_t cnt_high = 0;
static uint8_t  hist[80];

static uint32_t sample(uint32_t ms)
{
  unsigned long t0 = millis();
  uint8_t hi = 0;
  rx = 0u; cnt_dollar = 0u; cnt_nl = 0u; cnt_star = 0u; cnt_high = 0u;

  while ((millis() - t0) < ms)
  {
    while (Serial.available())
    {
      uint8_t c = Serial.read();
      rx++;
      if (c == (uint8_t)'$') { cnt_dollar++; }
      else if (c == (uint8_t)'\n') { cnt_nl++; }
      else if (c == (uint8_t)'*') { cnt_star++; }
      else if (c >= 0x80u) { cnt_high++; }
      if (hi < sizeof hist) { hist[hi++] = c; }
    }
  }
  return hi;
}

static void to_pc(void)
{
  /* GPS 9600 -> PC 115200. end() sonrasi begin() yeniden kurar. */
  Serial.end();
  Serial.begin(115200);
  delay(50);
}

static void put(const char *s) { Serial.write(s); }
static void put_u32(uint32_t v)
{
  char b[11]; uint8_t i = 0;
  if (v == 0u) { Serial.write('0'); return; }
  while ((v > 0u) && (i < (uint8_t)sizeof b))
  { b[i++] = (char)('0' + (v % 10u)); v /= 10u; }
  while (i > 0u) { Serial.write(b[--i]); }
}

void setup()
{
  /* Once birkac yazi at ki Serial'in dogru kuruldugunu gorelim. */
  Serial.begin(115200);
  delay(100);
  Serial.write("basliyor...\n");

  Serial.end();
  Serial.begin(GPS_BAUD);
  delay(250); /* modul kendi baud'unda; gecis otursun */

  /* 4 saniye dinle, hicbiri basma. */
  uint8_t n = sample(4000UL);

  to_pc(); /* artik 115200'deyiz, guvenle basabiliriz */

  put("\n=== NEO-6M D0 TEŞHİSİ ===\n");
  put("GPS baud = ");
  put_u32(GPS_BAUD);
  put("\n");
  put("rx toplam = ");
  put_u32(rx);
  put("   (");
  put_u32((rx * 1000UL) / 4000UL);
  put(" B/s)\n");
  put("'$' = ");
  put_u32(cnt_dollar);
  put("   NL = ");
  put_u32(cnt_nl);
  put("   '*' = ");
  put_u32(cnt_star);
  put("   >=0x80 = ");
  put_u32(cnt_high);
  put("\n\n");

  if (cnt_dollar > 0u)
  {
    put(">>> '$' BULUNDU — modül 9600'da, NMEA akıyor. BAŞARILI.\n");
    put("GGA/RMC cümleleri geliyor demektir. Artık CHECKSUM'a bakalım.\n\n");
  }
  else if (rx > 0u)
  {
    put(">>> Veri var ama '$' yok. Yanlış baud olabilir.\n");
    put(">>> ya da modül başka formatta (UBX-only) konuşuyor.\n");
  }
  else
  {
    put(">>> HİÇ VERİ YOK. Modül ya beslemesiz ya TX kablosu kopuk.\n");
  }

  if (n > 0u)
  {
    put("\n--- ilk baytlar (hex) ---\n");
    for (uint8_t k = 0u; k < n; k++)
    {
      Serial.write("0123456789ABCDEF"[hist[k] >> 4]);
      Serial.write("0123456789ABCDEF"[hist[k] & 0x0Fu]);
      Serial.write(' ');
      if (((k + 1u) % 16u) == 0u) { put("\n"); }
    }
    put("\n--- ascii ---\n");
    for (uint8_t k = 0u; k < n; k++)
    {
      uint8_t c = hist[k];
      Serial.write(((c >= 32u) && (c < 127u)) ? (char)c : '.');
    }
    put("\n");
  }
}

void loop()
{
  /* Tek seferlik: setup icinde bitti. */
  delay(1000);
}