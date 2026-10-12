/*
 * uno_gps_scan_d0.ino — D0 (donanim UART) ile coklu baud taramasi. v5
 *
 * v4 HATASI: Serial.end() AVR'de RX'i kapatiyor -> hep 0 bayt.
 * v5: v1'in (calisan) kalibini birebir kullanir: Serial.begin(baud) dogrudan,
 *     end() YOK. Tarama sirasinda HICBIR SEY basilir; tum sonuclar global
 *     dizilerde tutulur, son en sonda Serial.begin(115200) ile tablo basilir.
 *
 * BAĞLAMA: NEO-6M VCC->5V  GND->GND  TX->D0
 * Serial Monitor 115200
 */

static const uint32_t BAUDS[] = {9600, 38400, 115200, 4800, 19200, 57600};
static const uint8_t  NBAUD = 6;
static const uint32_t TEST_MS = 2000UL;

static uint32_t res_rx[NBAUD];
static uint32_t res_d[NBAUD];
static uint32_t res_n[NBAUD];
static uint32_t res_s[NBAUD];
static uint32_t res_h[NBAUD];
static uint8_t  res_len[NBAUD];
static uint8_t  res_hist[NBAUD][40];

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

void setup()
{
  Serial.begin(115200);
  Serial.write("#basliyor\n");
  delay(120);

  for (uint8_t i = 0u; i < NBAUD; i++)
  {
    Serial.begin(BAUDS[i]); /* end() YOK — v1'in calisan kalibi */
    delay(220);            /* modulun yeni baud'a gecmesi icin */
    delay(50);

    unsigned long t0 = millis();
    uint32_t rx = 0, d = 0, n = 0, s = 0, h = 0;
    uint8_t len = 0;

    while ((millis() - t0) < TEST_MS)
    {
      while (Serial.available())
      {
        uint8_t c = Serial.read();
        rx++;
        if (c == (uint8_t)'$') { d++; }
        else if (c == (uint8_t)'\n') { n++; }
        else if (c == (uint8_t)'*') { s++; }
        else if (c >= 0x80u) { h++; }
        if (len < 40u) { res_hist[i][len++] = c; }
      }
    }

    res_rx[i] = rx;
    res_d[i] = d;
    res_n[i] = n;
    res_s[i] = s;
    res_h[i] = h;
    res_len[i] = len;
  }

  Serial.begin(115200); /* cikis hattini geri al */
  delay(60);

  put("\n=== NEO-6M D0 BAUD TARAMASI v5 ===\n");
  put(" baud |   B/s |   rx |   '$' |  NL |  '*' | >=0x80\n");
  put("------+-------+------+-------+-----+------+--------\n");

  for (uint8_t i = 0u; i < NBAUD; i++)
  {
    put(" ");
    put_u32(BAUDS[i]);
    put(" |");
    put_u32((res_rx[i] * 1000UL) / TEST_MS);
    put(" |");
    put_u32(res_rx[i]);
    put(" |");
    put_u32(res_d[i]);
    put(" |");
    put_u32(res_n[i]);
    put(" |");
    put_u32(res_s[i]);
    put(" |");
    put_u32(res_h[i]);
    put("\n");
  }

  uint32_t best = 0, best_d = 0;
  uint8_t bi = 0;
  for (uint8_t i = 0u; i < NBAUD; i++)
  {
    if (res_d[i] > best_d) { best_d = res_d[i]; best = BAUDS[i]; bi = i; }
  }

  put("\n=== SONUC ===\n");
  if (best_d > 0u)
  {
    put("DOGRU BAUD = ");
    put_u32(best);
    put("   ('$' = ");
    put_u32(best_d);
    put(")\n");
    put("hex  : ");
    for (uint8_t k = 0u; k < 40u; k++)
    {
      Serial.write("0123456789ABCDEF"[res_hist[bi][k] >> 4]);
      Serial.write("0123456789ABCDEF"[res_hist[bi][k] & 0x0Fu]);
      Serial.write(' ');
    }
    put("\nascii: ");
    for (uint8_t k = 0u; k < 40u; k++)
    {
      uint8_t c = res_hist[bi][k];
      Serial.write(((c >= 32u) && (c < 127u)) ? (char)c : '.');
    }
    put("\n");
  }
  else
  {
    put("hicbir baud'da '$' yok.\n");
    uint32_t total = 0;
    for (uint8_t i = 0u; i < NBAUD; i++) { total += res_rx[i]; }
    put("toplam bayt (tum baud'lar) = ");
    put_u32(total);
    put(total == 0u ? "  -> HAT TAMAMEN SESSIZ\n"
                    : "  -> veri var ama '$' yok\n");
  }
}

void loop()
{
  delay(1000);
}