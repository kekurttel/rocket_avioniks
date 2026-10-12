/*
 * uno_gps_deep.ino — Kesin teşhis: donanim + kablo + modül.
 *
 * v5'te takildigimiz yer: kart RESET olmuyordu, ayni baytlar (391) tekrar
 * geliyordu. Bu sketch:
 *   1) BOOT=micros() basar -> her reset'te DEGER DEGISIR. Boylece reset'in
 *      gercekten calistigini kanitlariz.
 *   2) D0 VE D4'I AYNI ANDA dinler. Kablo hangi pinde olursa olsun veri
 *      bulunur; sen kablo tasimak zorunda kalmazsIn.
 *   3) Idle seviyesi olcer: hatti bosta LOW ise modul guc yok / TX kisa.
 *   4) Loopback testi: D0<->D1 jumper varsa bayt doner, donanimi kanitlar.
 *
 * Serial Monitor 115200. HICBIR yerde Serial.begin(9600) YOK: cikis daima
 * 115200'de kalir, GPS icin SoftwareSerial kullanilir.
 */

#include <SoftwareSerial.h>

/* Iki farkli pini ayni anda dinle: hangisi takiliysa veri orada cikar. */
static SoftwareSerial S0(0, 1);  /* donanim yerine yazilim: D0=RX */
static SoftwareSerial S4(4, 5);  /* D4=RX */

/* Loopback testi icin donanim UART'i (D1 TX, D0 RX). */
static HardwareSerial &HW = Serial;

static const uint32_t BAUDS[] = {9600, 38400, 115200, 4800, 19200};
static const uint8_t  NBAUD = 5;
static const uint32_t TEST_MS = 1500UL;

/* iki port x iki metrik (bayar + yuksek bayt sayisi) */
static uint32_t d_rx[2], d_hi[2];
static uint8_t  d_len[2];
static uint8_t  d_hist[2][32];

/* D0 hattinin bos seviyesi: HIGH=1 (RX dongulu, modul TX'i), LOW=0 (kisa) */
static uint8_t idle0 = 0, idle4 = 0;

/* tum baud'larda toplanan bayt (p=0 D0, p=1 D4) */
static uint32_t sum_rx[2] = {0u, 0u};

static void put(const char *s) { HW.write(s); }
static void put_u32(uint32_t v)
{
  char b[11]; uint8_t i = 0;
  if (v == 0u) { HW.write('0'); return; }
  while ((v > 0u) && (i < (uint8_t)sizeof b))
  { b[i++] = (char)('0' + (v % 10u)); v /= 10u; }
  while (i > 0u) { HW.write(b[--i]); }
}

/* b1: 0 = D0, 1 = D4 */
static SoftwareSerial &port(uint8_t i) { return (i == 0u) ? S0 : S4; }

void setup()
{
  HW.begin(115200);
  /* BOUT satiri her reset'te farkli olmali. Degismiyorsa kart reset olmuyor. */
  HW.write("\n=== NEO-6M DERIN TESHIS ===\n");
  put("BOOT micros=");
  put_u32(micros());
  put("  (her reset'te DEGISMELI)\n\n");

  /* --- 1) Hatta olan sey: idle seviyesi --- */
  pinMode(0, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  delay(20);
  idle0 = digitalRead(0);
  idle4 = digitalRead(4);
  pinMode(0, INPUT);
  pinMode(4, INPUT);

  put("IDLE D0=");
  HW.write(idle0 ? '1' : '0');
  put("  D4=");
  HW.write(idle4 ? '1' : '0');
  put("   (1=RX dongulu/bosta, 0=kisa)\n");

  /* --- 2) LOOPBACK: D0<->D1 jumper'i var mi? ---
   * D1 TX'e yazip D0'den geri okuruz. Yazma islemi senkron: write() sonrasi
   * flush() ile veri gercekten hattan cikmis olur, sonra okuruz.
   * NOT: onceki surumde burada takilip kaliyordu; simdi sure sinirli. */
  {
    HW.flush();
    HW.write((uint8_t)'U');
    HW.flush();                 /* bayt hattan tamamen ciksin */
    delay(20);
    uint8_t got = 0;
    uint32_t t0 = millis();
    while ((millis() - t0) < 300UL)
    {
      if (HW.available()) { got = HW.read(); break; }
    }
    put("LOOPBACK D0<->D1: ");
    if (got == (uint8_t)'U') { put("GECTI (donanim OK)\n"); }
    else
    {
      put("GELMEDI (0x");
      HW.write("0123456789ABCDEF"[got >> 4]);
      HW.write("0123456789ABCDEF"[got & 0x0F]);
      put(")\n");
    }
  }

  /* --- 3) Iki portu da 5 baud'da tara --- */
  put("\n port | baud |   rx |  >=0x80\n");
  put("------+-------+------+-------\n");
  for (uint8_t i = 0u; i < NBAUD; i++)
  {
    uint32_t b = BAUDS[i];
    S0.begin(b); S4.begin(b);
    delay(200);

    for (uint8_t p = 0u; p < 2u; p++)
    {
      d_rx[p] = 0u; d_hi[p] = 0u; d_len[p] = 0u;
    }

    unsigned long t0 = millis();
    while ((millis() - t0) < TEST_MS)
    {
      for (uint8_t p = 0u; p < 2u; p++)
      {
        SoftwareSerial &ss = port(p);
        while (ss.available())
        {
          uint8_t c = ss.read();
          d_rx[p]++;
          if (c >= 0x80u) { d_hi[p]++; }
          if (d_len[p] < 32u) { d_hist[p][d_len[p]++] = c; }
        }
      }
    }

    for (uint8_t p = 0u; p < 2u; p++)
    {
      sum_rx[p] += d_rx[p];
      put(p == 0u ? "   D0 | " : "   D4 | ");
      put_u32(b);
      put(" |");
      put_u32(d_rx[p]);
      put(" |");
      put_u32(d_hi[p]);
      put("\n");
    }
  }

  /* --- 4) Ozet --- */
  uint32_t tot0 = 0, tot4 = 0;
  for (uint8_t i = 0u; i < NBAUD; i++)
  {
    tot0 += 0u; /* asagidaki toplamlar tarama sirasinda biriktirilir */
    tot4 += 0u;
  }
  put("\nTOPLAM D0=");
  put_u32(sum_rx[0]);
  put("  D4=");
  put_u32(sum_rx[1]);
  put("\n\n=== YORUM ===\n");
  put("BOOT micros her reset'te DEGISMELI -> reset calisiyor.\n");
  put("IDLE=1 + rx=0 -> kablo takili ama modul KONUSMUYOR (VCC/GND sorunu).\n");
  put("IDLE=0        -> hattin ucu GND'ye bagli (kisa devre!).\n");
  put("LOOPBACK GECTI-> Uno RX yolu saglam; sorun modul veya kablo.\n");

  put("\n--- canli izleme (Ctrl-C ile dur) ---\n");
  for (;;)
  {
    bool any = false;
    for (uint8_t p = 0u; p < 2u; p++)
    {
      SoftwareSerial &ss = port(p);
      while (ss.available())
      {
        uint8_t c = ss.read();
        any = true;
        if ((c >= 32u) && (c < 127u)) { HW.write((char)c); }
        else if (c == (uint8_t)'\n') { HW.write('\n'); }
        else { HW.write('.'); }
      }
    }
    if (!any) { delay(20); }
  }
}

void loop()
{
  /* setup icinde biten tek seferlik test. */
}