/*
 * uno_gps_config.ino — NEO-6M UBX konfigurasyonu (gen6). GPS D2/D3'te.
 *
 * NEDEN D2/D3 (D0/D1 DEGIL):
 *   GPS'i donanim UART'ina (D0/D1) koydugumuzda iki celiski birden olusuyordu:
 *     1) Modulun TX ciktisi D0'i suruyordu. ATmega328P bootloader'i da
 *        D0/D1'i STK500 sync icin kullanir -> modul takliyken avrdude
 *        "programmer is not responding" veriyordu ve RESET butonu calismiyordu.
 *     2) GPS 9600'de konusurken ekran cikisi 115200'de olmali; donanim
 *        UART'inin TEK baud'u var, Serial.begin(9600) cagrisi tum ekran
 *        ciktisini da 9600'e dusuruyordu (banner yarida kesiliyordu).
 *   Cozum: donanim UART'i (D0/D1) SADECE ekran icin 115200'de birak,
 *   GPS'i SoftwareSerial ile D2 (RX) / D3 (TX) uzerinden 9600'da sur.
 *   Bonus: modul artik D0'i surmedigi icin, modul takliyken bile
 *   sketch yuklenebilir ve RESET butonu calisir.
 *
 * KABLOLAMA:
 *   modul VCC -> 5V        modul GND -> GND
 *   modul TX  -> D2        modul RX  -> D3
 *   (D0/D1'YE HICBIR SEY BAGLANMAZ - ekran cikisi)
 *   D3 -> modul RX arasinda 1k seri direnc onerilir (D3 5V lojik,
 *   modulun RX girisi 3.3V CMOS).
 *
 * RESET'E BASMANA GEREK YOK: loop() her 2.5 sn config'i yeniden gonderir.
 *
 * Serial Monitor 115200.
 */

#include <SoftwareSerial.h>

static SoftwareSerial GPS(2, 3);   /* D2 = RX (modulden), D3 = TX (module) */
static const uint32_t GPS_BAUD = 9600UL;
static const uint32_t CYCLE_MS = 2500UL;

/* --- Fletcher: cls..payload toplami, sync byte'lar haric --- */
static void ubx_finish(uint8_t *o, uint16_t len)
{
  uint8_t ckA = 0, ckB = 0;
  for (uint8_t i = 0u; i < 4u + len; i++)
  {
    ckA = (uint8_t)(ckA + o[2u + i]);
    ckB = (uint8_t)(ckB + ckA);
  }
  o[6u + len] = ckA;
  o[7u + len] = ckB;
}

/* CFG-MSG (0x06/0x01) gen6 payload = 6 bayt:
 *   msgClass, msgID, rateDDC, rateUART1, rateUART2, rateUSB
 *   UBX'te "current port" cozumlemesi YOK; rateUART1'e yaziyoruz. */
static uint8_t b_msg(uint8_t *o, uint8_t cls, uint8_t id, uint8_t rate)
{
  o[0]=0xB5; o[1]=0x62; o[2]=0x06; o[3]=0x01; o[4]=6u; o[5]=0u;
  o[6]=cls; o[7]=id; o[8]=0u; o[9]=rate; o[10]=0u; o[11]=0u;
  ubx_finish(o, 6u);
  return 14u;
}

/* CFG-NAV5 (0x06/0x24) payload 36: mask 0x0001 = sadece dynModel.
 * gen6'da 7 bit (0..7); 7 = Airborne <4g. 8/9/10 gen7+ ve gecersiz. */
static uint8_t b_nav5(uint8_t *o, uint8_t dyn)
{
  o[0]=0xB5; o[1]=0x62; o[2]=0x06; o[3]=0x24; o[4]=36u; o[5]=0u;
  for (uint8_t i=0;i<36u;i++){ o[6u+i]=0u; }
  o[6]=0x01u; o[7]=0x00u; o[8]=dyn;
  ubx_finish(o, 36u);
  return 44u;
}

/* CFG-RATE (0x06/0x08) payload 6: measRate, navRate, timeRef.
 * NEO-6 = 1 Hz parca; gen6 tabani 1000 ms. */
static uint8_t b_rate(uint8_t *o, uint16_t ms)
{
  o[0]=0xB5; o[1]=0x62; o[2]=0x06; o[3]=0x08; o[4]=6u; o[5]=0u;
  o[6]=(uint8_t)(ms & 0xFFu); o[7]=(uint8_t)(ms >> 8);
  o[8]=1u; o[9]=0u; o[10]=1u; o[11]=0u;
  ubx_finish(o, 6u);
  return 14u;
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

/* ACK/NAK: B5 62 05 01 <ack_id> ckA ckB ; cls 0x06 = NAK */
static int wait_ack(uint8_t want_id, uint16_t *ms)
{
  unsigned long t0 = millis();
  uint8_t st = 0;
  while ((millis() - t0) < 700UL)
  {
    while (GPS.available())
    {
      uint8_t c = GPS.read();
      if      (st == 0u) { st = (c == 0xB5u) ? 1u : 0u; }
      else if (st == 1u) { st = (c == 0x62u) ? 2u : ((c == 0xB5u) ? 1u : 0u); }
      else if (st == 2u) { st = 3u; }
      else if (st == 3u)
      {
        st = 4u;
        if ((c == 0x05u || c == 0x06u) && GPS.available())
        {
          uint8_t id = GPS.read();
          (void)GPS.read(); (void)GPS.read();
          *ms = (uint16_t)(millis() - t0);
          if (id == want_id) { return (c == 0x05u) ? 1 : -1; }
          return 0;
        }
      }
      else { st = 0u; }
    }
  }
  return 0;
}

static uint8_t f[64];
static uint32_t last_run = 0;
static bool first = true;
/* Teşhis: D2'den gelen ham bayt sayaci + son baytlar. NOACK'in neden
 * olduğunu ayirt eder: raw>0 ama ACK yok -> modul konusuyor, RX cevabi
 * gelmiyor (D3 baglantisi / seviye). raw=0 -> modul D2'ye hic ulasmiyor
 * (kablo yanlis delikte ya da modul beslenmiyor). */
static uint32_t gps_raw = 0;
static uint8_t  hist[32];
static uint8_t  hist_i = 0;

static void run_config(void)
{
  static const uint8_t ids[6] = {0x00u,0x04u,0x01u,0x02u,0x03u,0x05u};
  static const uint8_t rt[6]  = { 1u,   1u,   0u,   0u,   0u,   0u};
  static const char *nm[6] = {
    "CFG-MSG GGA  rate=1 (ac)",     "CFG-MSG RMC  rate=1 (ac)",
    "CFG-MSG GLL  rate=0 (kapat)",  "CFG-MSG GSA  rate=0 (kapat)",
    "CFG-MSG GSV  rate=0 (kapat)",  "CFG-MSG VTG  rate=0 (kapat)" };

  if (first) { put("\n=== NEO-6M UBX CONFIG (gen6) ===\n"); first = false; }
  put("\n--- turun ---\n");

  uint8_t ok = 0, fail = 0;
  for (uint8_t i = 0u; i < 8u; i++)
  {
    uint8_t n, want; const char *label;
    if (i < 6u)       { n = b_msg(f, 0xF0u, ids[i], rt[i]); want = 0x01u; label = nm[i]; }
    else if (i == 6u) { n = b_nav5(f, 7u);   want = 0x24u; label = "CFG-NAV5 dynModel=7 (Airborne<4g)"; }
    else              { n = b_rate(f, 1000u); want = 0x08u; label = "CFG-RATE 1000ms (1 Hz)"; }

    put("adim "); put_u32(i); put(" "); put(label); put(" -> ");
    uint32_t raw_before = gps_raw;
    while (GPS.available()) { GPS.read(); }      /* NMEA'yi temizle */
    GPS.write(f, n);
    delay(40);

    unsigned long t_start = millis();
    uint16_t t = 0;
    int r = wait_ack(want, &t);
    /* wait_ack zaman asimi yolunda *ms'i set etmiyor; burada olceleriz */
    t = (uint16_t)(millis() - t_start);
    if (r == 1)       { put("ACK");   ok++; }
    else if (r == -1) { put("NAK");   fail++; }
    else              { put("NOACK"); fail++; }
    put(" ("); put_u32(t); put("ms, D2'den "); put_u32(gps_raw - raw_before);
    put(" bayt)\n  hex: ");
    for (uint8_t k = 0u; k < n; k++)
    {
      char pr[3];
      pr[0] = "0123456789ABCDEF"[f[k] >> 4];
      pr[1] = "0123456789ABCDEF"[f[k] & 0x0Fu];
      pr[2] = '\0';
      Serial.write(pr); Serial.write(' ');
    }
    put("\n");
    delay(120);
  }
  put("SONUC: "); put_u32(ok); put(" ACK / "); put_u32(fail); put(" basarisiz\n");
}

void setup()
{
  Serial.begin(115200);          /* ekran: daima 115200, degismez */
  delay(150);
  put("\n=== NEO-6M UBX CONFIG (gen6) ===\n");
  put("GPS: D2(RX) <- modul TX,  D3(TX) -> modul RX, 9600\n");
  put("Ekran: D0/D1, 115200. D0/D1'e modul BAGLANMAZ.\n");
  put("RESET gerekmez - config her 2.5 sn tekrar gonderilir.\n");

  GPS.begin(GPS_BAUD);
  delay(600);                   /* modul power-on self-check */
  last_run = millis() - CYCLE_MS;
}

void loop()
{
  if ((uint32_t)(millis() - last_run) >= CYCLE_MS)
  {
    last_run = millis();
    run_config();
  }
  while (GPS.available())
  {
    uint8_t c = GPS.read();
    gps_raw++;
    hist[hist_i] = c;
    hist_i = (uint8_t)((hist_i + 1u) & 31u);
  }
  /* Teşhis satırı: D2 sessizse sorun kablonun kendisindedir. */
  if ((uint32_t)(millis() - last_run) < 300UL)
  {
    put("  [D2 son baytlar] ");
    for (uint8_t k = 0u; k < 32u; k++)
    {
      uint8_t v = hist[(uint8_t)((hist_i + k) & 31u)];
      char pr[3];
      pr[0] = "0123456789ABCDEF"[v >> 4];
      pr[1] = "0123456789ABCDEF"[v & 0x0Fu];
      pr[2] = '\0';
      Serial.write(pr); Serial.write(' ');
    }
    put("\n");
  }
}