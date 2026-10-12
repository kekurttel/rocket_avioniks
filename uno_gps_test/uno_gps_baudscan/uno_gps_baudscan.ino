/*
 * uno_gps_baudscan.ino — NEO-6M baud tespiti. v2
 *
 * v1 HATASI: GPS.begin(baud) donanim Serial'ini de degistirir. Tarama
 * sirasinda Serial 9600'a dustugu icin PC'ye giden banner da bozuluyordu.
 * v2: iki ayri HardwareSerial kullan. Uno'da donanim UART'i tek (Serial,
 * RX=D0 TX=D1) oldugu icin iki UART yoktur — bu yuzden tarama sirasinda
 * PC cikisini elle koruyoruz: her baud'dan sonra Serial.begin(115200)
 * geri cagrilir ve banner anlik yazilir. Gercek cozum: SoftwareSerial ile
 * D5/D4 uzerinde GPS okumak, Serial'i PC icin 115200'de birakmak.
 * v3 olarak D5/D4'e geciyoruz.
 *
 * BAĞLAMA v3:
 *   NEO-6M VCC -> 5V
 *   NEO-6M GND -> GND
 *   NEO-6M TX  -> Uno D4  (SoftwareSerial RX)
 *   (opsiyonel) RX -> D5  (SoftwareSerial TX, kullanilmiyor)
 *
 * Serial Monitor 115200
 */

#include <SoftwareSerial.h>

/* GPS'i D4(RX) uzerinden oku. Serial (D0/D1) sadece PC cikisi icin 115200'de. */
static SoftwareSerial GPS(4, 5);

static const uint32_t GPS_BAUDS[] = {9600, 38400, 115200, 4800, 19200, 57600};
static const uint8_t  NBAUD = sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0]);

static const uint32_t TEST_MS = 2200UL; /* her baud icin dinleme suresi */

static uint32_t rx = 0;
static uint32_t cnt_dollar = 0;
static uint32_t cnt_nl = 0;
static uint32_t cnt_star = 0;
static uint32_t cnt_high = 0;
static uint8_t  hist[56];

static void put(const char *s) { Serial.write(s); }

static void put_u32(uint32_t v)
{
  char b[11];
  uint8_t i = 0;
  if (v == 0u) { Serial.write('0'); return; }
  while ((v > 0u) && (i < (uint8_t)sizeof b))
  {
    b[i++] = (char)('0' + (v % 10u));
    v /= 10u;
  }
  while (i > 0u) { Serial.write(b[--i]); }
}

static void reset_stats(void)
{
  rx = 0u; cnt_dollar = 0u; cnt_nl = 0u; cnt_star = 0u; cnt_high = 0u;
}

/* Belirtilen sure boyunca GPS'i dinle, istatistik topla. */
static void drain(uint32_t ms)
{
  unsigned long t0 = millis();
  uint8_t hi = 0;
  while ((millis() - t0) < ms)
  {
    while (GPS.available())
    {
      uint8_t c = GPS.read();
      rx++;
      if (c == (uint8_t)'$') { cnt_dollar++; }
      else if (c == (uint8_t)'\n') { cnt_nl++; }
      else if (c == (uint8_t)'*') { cnt_star++; }
      else if (c >= 0x80u) { cnt_high++; }
      if (hi < sizeof hist) { hist[hi++] = c; }
    }
  }
}

static void dump_hist(uint32_t baud)
{
  put("   >>> '$' BULUNDU @ ");
  put_u32(baud);
  put("  <<<  dogru baud\n");
  put("   hex (56 bayt):\n   ");
  for (uint8_t k = 0u; k < 56u; k++)
  {
    uint8_t b = hist[k];
    Serial.write("0123456789ABCDEF"[b >> 4]);
    Serial.write("0123456789ABCDEF"[b & 0x0Fu]);
    Serial.write(' ');
    if (((k + 1u) % 14u) == 0u) { put("\n   "); }
  }
  put("\n   ascii: ");
  for (uint8_t k = 0u; k < 56u; k++)
  {
    uint8_t c = hist[k];
    Serial.write(((c >= 32u) && (c < 127u)) ? (char)c : '.');
  }
  put("\n");
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  put("\n=== NEO-6M BAUD TARAMASI v3 (SoftwareSerial D4) ===\n");
  put("'$' sayisi 0 degilse o baud'tur.\n");
  put(" baud |   B/s |    rx |   '$' |  NL |  '*' | >=0x80\n");
  put("------+-------+-------+-------+-----+------+--------\n");

  uint32_t best = 0;
  uint32_t best_dollar = 0;

  for (uint8_t i = 0u; i < NBAUD; i++)
  {
    uint32_t b = GPS_BAUDS[i];
    GPS.begin(b);
    delay(220); /* modulu yeni baud'a gecir, kararli olsun */
    reset_stats();
    drain(TEST_MS);

    put(" ");
    put_u32(b);
    put(" |");
    put_u32((rx * 1000UL) / TEST_MS);
    put(" |");
    put_u32(rx);
    put(" |");
    put_u32(cnt_dollar);
    put(" |");
    put_u32(cnt_nl);
    put(" |");
    put_u32(cnt_star);
    put(" |");
    put_u32(cnt_high);
    put("\n");

    if (cnt_dollar > 0u)
    {
      dump_hist(b);
      if (cnt_dollar > best_dollar)
      {
        best_dollar = cnt_dollar;
        best = b;
      }
    }
  }

  if (best > 0u)
  {
    put("\n=== SONUC: dogru baud = ");
    put_u32(best);
    put("  (");
    put_u32(best_dollar);
    put(" adet '$') ===\n");
    put("Bu degeri yaz: STM32 huart6 bu baud'a ayarlanmali.\n");
    put("Anten gokyuzune cikarsa burada GGA/RMC cumleleri gorunecek.\n");
  }
  else
  {
    put("\n=== SONUC: hicbir baud'da '$' YOK. ===\n");
    put("Modul VCC veya TX kablosu sorunu olabilir. Modulun guc LED'ini kontrol et.\n");
  }

  /* En iyi baud'da suregel dinleme moduna gec */
  if (best > 0u)
  {
    GPS.begin(best);
    put("\n--- canli dinleme @ ");
    put_u32(best);
    put(" (Ctrl-C ile dur) ---\n");
    for (;;)
    {
      while (GPS.available())
      {
        uint8_t c = GPS.read();
        if ((c >= 32u) && (c < 127u)) { Serial.write((char)c); }
        else if (c == (uint8_t)'\n') { Serial.write('\n'); }
        else { Serial.write('.'); }
      }
      delay(10);
    }
  }
}

void loop() {}