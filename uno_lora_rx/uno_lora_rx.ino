/*
 * uno_lora_rx.ino — E220 LoRa alici (NUCLEO -> Uno, tek yon).
 *
 * ALICI: modul TXD -> Uno D0. Modul 3.3V verir, Uno esigi 0.6*5V=3.0V
 * oldugu icin 4.7k pull-up OLMADAN da okunmasi gerekir. Calismazsa
 * (surekli bozuk bayt) sebebi budur; bu durumda SKETCH otomatik olarak
 * ham bayt gosterir ve hangi durumda oldugumuzu anlarsin.
 *
 * KABLOLAMA:
 *   modul VCC  -> 5V
 *   modul GND  -> GND
 *   modul M0   -> D5      (GPIO cikis; Mode 3 = HIGH/HIGH, Mode 0 = LOW/LOW)
 *   modul M1   -> D6      (GPIO cikis)
 *   modul TXD  -> D2      (TEK sinyal, D2 = SoftwareSerial RX)
 *   modul RXD  -> D3      (SADECE konfigürasyon yazma icin; D3 5V lojik,
 *                          modul girisi 3.3V CMOS: araya 1k seri direnc
 *                          veya 2k/1k bolucu onerilir)
 *   modul AUX  -> D7      (GPIO giris; LOW = mesgul/self-check; D4 bozuk
 *                          oldugu icin D7 kullaniliyor)
 *
 * MOD SEMANTIGI (datasheet §6, M1,M0):
 *   (0,0) Mode 0 = seffaf iletim (CALISMA modu)
 *   (1,1) Mode 3 = derin uyku = KONFIGÜRASYON modu (sadece 9600 8N1)
 * Boot sirasi: Mode 3 -> C1 00 06 ile kayitlari oku -> hedef imaj
 * {00,00,63,00,17,00} (9600 UART, 4.8 kbps RF) degilse E220 veri sayfasi
 * biciminde C2 RAM-yazimi dene, Mode 0'a cikip yeniden oku, sonra tek seferlik
 * C0 kalici yaz ve tekrar dogrula. 22 dBm zaten azami guc; guc bitlerini
 * artirmaya calisma.
 *
 * NEDEN D2, D0 DEĞİL:
 *   ATmega328P bootloader'ı D0/D1'i STK500 sync için kullanır. Modülün TX'i
 *   D0'a bağlıysa avrdude "not in sync: resp=0x0a / programmer is not
 *   responding" verir ve sketch yüklenemez. D2'de tutmak D0/D1'i her
 *   zaman temiz tutar: modul TAKILIYKEN bile yükleme yapılabilir.
 *   D0/D1 bu durumda ekran cikisi olarak 115200'de kalir.
 *
 * Serial Monitor 115200.
 *
 * Current frame V3: 38 bytes, little-endian. Legacy V1 (30 B) and V2 (36 B)
 * remain accepted. V3 altitude is signed decimetres (int32) to retain 0.1 m:
 *   0 A5, 1 version 03, 2 seq u16, 4 t_ms u32, 8 flight, 10 ek7 mode,
 *   12 alt_dm i32, 16 vel_cms i16, 18 sats u16, 20 lat_e6 i32, 24 lon_e6 i32,
 *   28 batt_mv u16, 30 flags, 31 tilt_ddeg u16, 33/35 lean i16, 37 CRC.
 */

/* 32-byte receive ring is sufficient for a 4.8 kbps E220 stream: the loop
 * consumes bytes continuously, and its ~10 ms status line can accumulate only
 * about ten incoming UART bytes. Keep SRAM headroom for the V3 parser. */
#ifndef _SS_MAX_RX_BUFF
#define _SS_MAX_RX_BUFF 32
#endif
#include <SoftwareSerial.h>
#include <stdint.h>

/* LoRa modulunden gelen veri: D2 = RX. D0/D1 ekran cikisi olarak
 * 115200'de kalir; modul D0'a hicbaglanmaz, boylece bootloader
 * her zaman calisir ve modul takiliyken de yukleme yapilabilir. */
static SoftwareSerial RX2(2, 3);   /* D2 = RX, D3 = TX (konfig yazma icin) */

/* E220 mod pinleri (Uno tarafi). */
static const uint8_t PIN_M0  = 5;
static const uint8_t PIN_M1  = 6;
static const uint8_t PIN_AUX = 7;

/* Both E220 modules use 9600 8N1 UART and 4.8 kbps RF. REG1=00 is the
 * hardware maximum 22 dBm, REG2=17 is 873.125 MHz, REG3=00 transparent.
 * The RF rate must match at both ends. */
static const uint8_t CFG_EXPECT[6] = {0x00,0x00,0x63,0x00,0x17,0x00};

static const uint8_t  FRAME_LEN_V1 = 30;
static const uint8_t  FRAME_LEN_V2 = 36;
static const uint8_t  FRAME_LEN_V3 = 38;
static const uint8_t  MAGIC     = 0xA5;
static const uint8_t  VERSION_V1 = 0x01;
static const uint8_t  VERSION_V2 = 0x02;
static const uint8_t  VERSION_V3 = 0x03;
static const int16_t  ALT_INVALID = (int16_t)0x8000;
static const int32_t ALT_DM_INVALID = INT32_MIN;
static const int16_t  VEL_INVALID = (int16_t)0x8000;
static const uint16_t SATS_INVALID = 0xFFFF;
static const uint16_t TILT_INVALID = 0xFFFF;

static uint8_t  buf[FRAME_LEN_V3 + 1];
static uint8_t  len = 0;
static uint32_t good = 0, bad = 0;
static uint32_t raw = 0;            /* HIC BIR SAYAC YOKSA: kablo/power sorunu */
static uint32_t raw_mark = 0;         /* hiz olcumu icin onceki RAW */
static uint32_t n_a5 = 0;             /* gorulen A5 (magic) sayisi */
static uint32_t n_00 = 0;             /* gorulen 0x00 sayisi */
static uint32_t n_hi = 0;             /* gorulen >=0x80 sayisi */
static uint32_t n_hdr = 0;            /* gorulen 00 02 17 siralamasi */
static uint8_t  hdr_hist[3] = {0,0,0};
static uint8_t  hdr_i = 0;
static uint8_t  hdr_seen = 0;
static uint8_t  rawbuf[16];         /* compact last-byte window for diagnostics */
static uint8_t  rawn = 0, raww = 0;
static uint32_t last_good_ms = 0;
static bool     reported_ok = false;

/* 4 baytlik bayt siralamasi (little-endian) */
static uint16_t rd16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static int16_t rd_i16(const uint8_t *p) { return (int16_t)rd16(p); }
static uint32_t rd32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t rd_i32(const uint8_t *p) { return (int32_t)rd32(p); }

static void put(const char *s) { Serial.write(s); }
static void put(const __FlashStringHelper *s) { Serial.print(s); }

static void put_u32(uint32_t v)
{
  char b[11]; uint8_t i = 0;
  if (v == 0u) { Serial.write('0'); return; }
  while ((v > 0u) && (i < (uint8_t)sizeof b))
  { b[i++] = (char)('0' + (v % 10u)); v /= 10u; }
  while (i > 0u) { Serial.write(b[--i]); }
}

/* Print signed decimetres as a truthful metre value (e.g. -0.4m). INT32_MIN
 * is reserved for invalid altitude. The widened negate avoids INT32_MIN UB. */
static void put_alt_dm(int32_t dm)
{
  if (dm == ALT_DM_INVALID) { put(F("--")); return; }
  int64_t signed_value = (int64_t)dm;
  uint32_t mag = (uint32_t)((signed_value < 0) ? -signed_value : signed_value);
  if (signed_value < 0) { put(F("-")); }
  put_u32(mag / 10u);
  put(F("."));
  Serial.write((char)('0' + (mag % 10u)));
  put(F("m"));
}

/* roket pozisyonunu ondalik derece yaz: e6, 6 decimals (~0.11 m) */
static void put_coord(int32_t v)
{
  uint32_t a = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
  uint32_t tamsayi = a / 1000000u;
  uint32_t kesir = a % 1000000u;
  char f[7];
  put_u32(tamsayi);
  put(F("."));
  for (uint8_t i = 0u; i < 6u; i++)
  {
    f[5u - i] = (char)('0' + (kesir % 10u));
    kesir /= 10u;
  }
  f[6] = '\0';
  put(f);
}

/* CRC: 0..n-1 bayt toplami mod 256. n = cerceve boyu - 1 (versiyona gore). */
static uint8_t crc8(const uint8_t *p, uint8_t n)
{
  uint8_t s = 0;
  for (uint8_t i = 0; i < n; i++) { s = (uint8_t)(s + p[i]); }
  return s;
}

/* decidegree -> "D.D" yaz (or: 123 -> "12.3"). */
static void put_ddeg(uint16_t dd)
{
  put_u32(dd / 10u);
  put(F("."));
  Serial.write((char)('0' + (dd % 10u)));
}

/* x1000 lean -> "-0.xxx" yaz. */
static void put_lean(int16_t v)
{
  uint16_t a;
  if (v < 0) { put(F("-")); a = (uint16_t)(-v); }
  else { a = (uint16_t)v; }
  put_u32(a / 1000u);
  put(F("."));
  uint16_t f = a % 1000u;
  Serial.write((char)('0' + (f / 100u)));
  Serial.write((char)('0' + ((f / 10u) % 10u)));
  Serial.write((char)('0' + (f % 10u)));
}

static bool handle_frame(uint8_t flen)
{
  uint8_t ver;
  int32_t alt_dm;
  int16_t vel;
  uint16_t sats, batt;
  int32_t lat, lon;
  uint16_t flight, ek7mode;
  uint8_t flags;
  if (buf[0] != MAGIC) { bad++; return false; }
  ver = buf[1];
  if (!((ver == VERSION_V1 && flen == FRAME_LEN_V1) ||
        (ver == VERSION_V2 && flen == FRAME_LEN_V2) ||
        (ver == VERSION_V3 && flen == FRAME_LEN_V3)))
  {
    bad++;
    return false;
  }
  /* Additive CRC: CRC byte is final for all versions. */
  if (crc8(buf, (uint8_t)(flen - 1u)) != buf[flen - 1u]) { bad++; return false; }
  flight = rd16(&buf[8]);
  ek7mode = rd16(&buf[10]);
  if (ver == VERSION_V3)
  {
    alt_dm = rd_i32(&buf[12]);
    vel = rd_i16(&buf[16]);
    sats = rd16(&buf[18]);
    lat = rd_i32(&buf[20]);
    lon = rd_i32(&buf[24]);
    batt = rd16(&buf[28]);
    flags = buf[30];
  }
  else
  {
    int16_t legacy_alt = rd_i16(&buf[12]);
    alt_dm = (legacy_alt == ALT_INVALID) ? ALT_DM_INVALID : (int32_t)legacy_alt * 10;
    vel = rd_i16(&buf[14]);
    sats = rd16(&buf[16]);
    lat = rd_i32(&buf[18]);
    lon = rd_i32(&buf[22]);
    batt = rd16(&buf[26]);
    flags = buf[28];
  }

  good++;
  last_good_ms = millis();

  /* TEK SATIR [LINK]: eski 6 satirlik blok paket basina ~20 Serial.print
   * (~200B @115200 = ~17ms blok) yapiyor, SoftwareSerial 9600 ile yarisip
   * cerceve kaciriyordu. Tum alanlar tek satirda (~110-130B, ~10ms): sira
   * sabit, hicbir alan dusurulmedi; sadece GPS ozetindeki tekrarlar
   * (2. sats/fix/pos) atildi. pos EN SONDA tutulur: backend POS_RE greedy
   * (pos=(.+)) oldugu icin tilt/lean'i yutmamalidir. Eksik jetonlari aynen
   * korundu: alt/vel/sats "--", batt "--" (mV'siz), pos "--",
   * v1'de "tilt=-- lean=--". Debug satirlari (MODE-PROBE, CFG) aynen. */
  put(F("[LINK] seq="));
  put_u32(rd16(&buf[2]));
  put(F(" t="));
  put_u32(rd32(&buf[4]));
  put(F("ms flight="));
  put_u32(flight);
  put(F(" ek7mode="));
  put_u32(ek7mode);

  put(F(" alt="));
  put_alt_dm(alt_dm);

  put(F(" vel="));
  if (vel != VEL_INVALID)
  {
    if (vel < 0) { put(F("-")); put_u32((uint32_t)(-vel)); }
    else { put_u32((uint32_t)vel); }
    put(F("cm/s"));
  }
  else { put(F("--")); }

  put(F(" sats="));
  if (sats == SATS_INVALID) { put(F("--")); } else { put_u32(sats); }

  put(F(" batt="));
  if (batt == 0u) { put(F("--")); } else { put_u32(batt); put(F("mV")); }

  put(F(" fix="));
  put((flags & 0x01u) ? F("Y") : F("N"));
  put(F(" LED="));
  Serial.write((char)((flags & 0x02u) ? 'B' : '-'));
  Serial.write((char)((flags & 0x04u) ? 'Y' : '-'));
  Serial.write((char)((flags & 0x08u) ? 'R' : '-'));

  /* V2: tilt 29 and lean 31/33. V3: tilt 31 and lean 33/35. */
  put(F(" tilt="));
  if (ver == VERSION_V2 || ver == VERSION_V3)
  {
    uint8_t tilt_off = (ver == VERSION_V3) ? 31u : 29u;
    uint8_t lean_x_off = (ver == VERSION_V3) ? 33u : 31u;
    uint8_t lean_y_off = (ver == VERSION_V3) ? 35u : 33u;
    uint16_t td = rd16(&buf[tilt_off]);
    int16_t lx = rd_i16(&buf[lean_x_off]);
    int16_t ly = rd_i16(&buf[lean_y_off]);
    if (td == TILT_INVALID) { put(F("-- lean=--")); }
    else { put_ddeg(td); put(F(" lean=")); put_lean(lx); put(F(",")); put_lean(ly); }
  }
  else { put(F("-- lean=--")); }

  /* Konum: fix + sifir-disi birlikte aranir (GPS ozetiyle ayni kural).
   * Sonda olmasi sart (yukaridaki nota bak). */
  bool pos_ok = ((lat != 0) || (lon != 0)) && (flags & 0x01u);
  put(F(" pos="));
  if (!pos_ok) { put(F("--")); }
  else { put_coord(lat); put(F("  ")); put_coord(lon); }

  put(F("\n"));
  reported_ok = true;
  return true;
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_AUX, INPUT);
  delay(150);
  put(F("\n=== E220 LoRa ALICI ===\n"));
  put(F("TXD->D2 RXD->D3 M0->D5 M1->D6 AUX->D7 VCC->5V\n"));

  /* D2'nin bos seviyesi (SoftwareSerial henuz baslamadi, hat bize ait):
   * 1 = bos/yuksek (normal), 0 = asagi cekili (kisa devre, yanlis delik). */
  pinMode(2, INPUT_PULLUP);
  delay(20);
  uint8_t idle = digitalRead(2);
  put(F("D2 bos seviye = "));
  Serial.write(idle ? '1' : '0');
  put(idle ? "   (normal, hat bos)\n" : "   (LOW -> kisa devre, yanlis delik)\n");

  /* SoftwareSerial icin begin() SART: pin-change interrupt'i bu cagiriyla
   * baglanir. begin() olmadan available() her zaman 0 doner, kablo dogru
   * olsa bile RAW=0 gorunur. Modul 9600'de konusuyor. */
  RX2.begin(9600);

  lora_cfg_boot();   /* Mode 3: oku, gerekirse yaz, Mode 0'a don */

  put(F("V1=30B, V2=36B legacy; V3=38B (signed 0.1m altitude) current\n"));
  put(F("gecerli cerceve bekleniyor...\n\n"));
}

/* AUX yuksek bekle (dusuk = mesgul). Donus: hazir mi. */
static bool aux_wait(unsigned long tmo_ms)
{
  unsigned long t0 = millis();
  while ((millis() - t0) < tmo_ms)
  {
    if (digitalRead(PIN_AUX) == HIGH) { delay(3); return true; }
  }
  return false;
}

static void mode_set(uint8_t m0, uint8_t m1)
{
  digitalWrite(PIN_M0, m0 ? HIGH : LOW);
  digitalWrite(PIN_M1, m1 ? HIGH : LOW);
}

/* C1 00 06 oku -> 9 bayt C1 00 06 + 6 param. Donus: okundu mu. */
static bool cfg_read(uint8_t *p6)
{
  const uint8_t cmd[3] = {0xC1, 0x00, 0x06};
  uint8_t rsp[9];
  uint8_t n = 0;
  while (RX2.available()) { RX2.read(); }
  RX2.write(cmd, 3);
  unsigned long t0 = millis();
  while (n < 9u && (millis() - t0) < 400UL)
  {
    while (RX2.available() && n < 9u) { rsp[n++] = RX2.read(); }
  }
  if (n != 9u) { return false; }
  if (rsp[0] != 0xC1 || rsp[1] != 0x00 || rsp[2] != 0x06) { return false; }
  for (uint8_t i = 0u; i < 6u; i++) { p6[i] = rsp[3u + i]; }
  return true;
}

/* C0 00 08 tam imaj yaz (flash, kalici): 00H..07H dahil CRYPT_H/_L.
 * C2 RAM yazisi bu modülde echo verip tutmuyor (dogrlama okuma eski
 * imaji gosterdi); C0 kalici yolu deneniyor. Cevap C1 00 08 echo.
 * Alinan echo echo_buf/echo_n içinde saklanir (teshis baskisi icin). */
static uint8_t echo_buf[11];
static uint8_t echo_n = 0;

static const uint8_t CFG_FULL8[8] = {0x00,0x00,0x63,0x00,0x17,0x00,0x00,0x00};

static bool cfg_write_full(void)
{
  uint8_t cmd[11] = {0xC0, 0x00, 0x08, 0,0,0,0,0,0,0,0};
  uint8_t rsp[11];
  uint8_t n = 0;
  echo_n = 0;
  for (uint8_t i = 0u; i < 8u; i++) { cmd[3u + i] = CFG_FULL8[i]; }
  while (RX2.available()) { RX2.read(); }
  RX2.write(cmd, 11);
  unsigned long t0 = millis();
  while (n < 11u && (millis() - t0) < 2500UL)
  {
    while (RX2.available() && n < 11u) { rsp[n++] = RX2.read(); }
  }
  if (n != 11u) { return false; }
  for (uint8_t i = 0u; i < 11u; i++) { echo_buf[i] = rsp[i]; }
  echo_n = n;
  if (rsp[0] != 0xC1 || rsp[1] != 0x00 || rsp[2] != 0x08) { return false; }
  for (uint8_t i = 0u; i < 8u; i++)
  {
    if (rsp[3u + i] != CFG_FULL8[i]) { return false; }
  }
  return true;
}

/* E220 manual §7.1: C2 temporary-write frame is C2 + address + length +
 * data (e.g. "C2 00 03 12 34 62"); response carries C1 + address + length
 * + 8 bytes (11 bytes). A C2 frame without the length byte is answered
 * FF FF FF and applies nothing. */
static bool cfg_write_ram(void)
{
  uint8_t cmd[11] = {0xC2, 0x00, 0x08, 0,0,0,0,0,0,0,0};
  uint8_t rsp[11];
  uint8_t n = 0;
  echo_n = 0;
  for (uint8_t i = 0u; i < 8u; i++) { cmd[3u + i] = CFG_FULL8[i]; }
  while (RX2.available()) { RX2.read(); }
  RX2.write(cmd, 11);
  unsigned long t0 = millis();
  while (n < 11u && (millis() - t0) < 800UL)
  {
    while (RX2.available() && n < 11u) { rsp[n++] = RX2.read(); }
  }
  if (n != 11u) { return false; }
  for (uint8_t i = 0u; i < 11u; i++) { echo_buf[i] = rsp[i]; }
  echo_n = n;
  if (rsp[0] != 0xC1 || rsp[1] != 0x00 || rsp[2] != 0x08) { return false; }
  for (uint8_t i = 0u; i < 8u; i++)
  {
    if (rsp[3u + i] != CFG_FULL8[i]) { return false; }
  }
  return true;
}

static bool cfg_write(const uint8_t *p6)
{
  (void)p6; /* 6 baytlik yola gerek yok: tam 8-bayt imaj yazilir */
  return cfg_write_full();
}

/* Boot konfigürasyonu: Mode 3'e gir, oku, farkliysa yaz + dogrula,
 * Mode 0'a don. Sonucu tek satir basar. Hicbir durumda takilmaz. */
static void lora_cfg_boot(void)
{
  uint8_t cur[6];
  bool match;
  mode_set(1, 1);   /* Mode 3 = konfigurasyon */
  if (!aux_wait(500UL))
  {
    put(F("LORA CFG: FAILED (AUX yok)\n"));
    mode_set(0, 0);
    return;
  }
  if (!cfg_read(cur))
  {
    put(F("LORA CFG: FAILED (cevap yok)\n"));
    mode_set(0, 0);
    (void)aux_wait(300UL);
    return;
  }
  put(F("LORA CFG read: "));
  for (uint8_t i = 0u; i < 6u; i++)
  {
    char pr[3];
    pr[0] = "0123456789ABCDEF"[cur[i] >> 4];
    pr[1] = "0123456789ABCDEF"[cur[i] & 0x0Fu];
    pr[2] = '\0';
    Serial.write(pr); Serial.write(' ');
  }
  put(F("\n"));
  match = true;
  for (uint8_t i = 0u; i < 6u; i++)
  {
    if (cur[i] != CFG_EXPECT[i]) { match = false; break; }
  }
  bool match0 = match; /* ilk okuma: C0 karari buna baglidir */
  /* Her-boot C2 RAM denemesi (flash'a yazmaz, zararsiz): log durust tutulur.
   * C0 karari SADECE 6-bayt eslesmezlige bakar; ilk okuma eslestiyse
   * C2 durumu ne olursa olsun C0 YAZILMAZ (flash omru korunur). */
  put(F("LORA CFG: C2 RAM deneniyor (C2 00 08)...\n"));
  bool c2ok = cfg_write_ram();
  /* E220 applies RAM writes on EXIT from Mode 3. Reading immediately in the
   * same configuration session can return the old image; switch to Mode 0,
   * wait for AUX-ready, then re-enter Mode 3 before verification. */
  mode_set(0, 0);
  (void)aux_wait(500UL);
  mode_set(1, 1);
  bool ready1 = aux_wait(500UL);
  bool match1 = false;
  if (ready1 && (cfg_read(cur) || cfg_read(cur)))
  {
    match1 = true;
    for (uint8_t i = 0u; i < 6u; i++)
    {
      if (cur[i] != CFG_EXPECT[i]) { match1 = false; break; }
    }
  }
  else
  {
    match1 = false; /* iki okuma da tutmadi: C2 dogrulanamadi */
  }
  /* Eslesme varsa C0 YOK (C2 durumu ne olursa olsun): flash korunur. */
  bool yazim = false;
  if (match0 && match1)
  {
    put(F("LORA CFG: OK eslesti (yazilmadi, C0 yok)\n"));
  }
  else if (match0 && !match1)
  {
    (void)c2ok;
    put(F("LORA CFG: OK eslesti (ilk imaj eslesti, C2 tutmadi, C0 yazilmadi)\n"));
    match = true; /* kayitli imaj dogruydu, C0 yok */
  }
  else if (!match0 && match1)
  {
    /* A different saved image was observed at boot. C2 + read-back confirms
     * the new 4.8 kbps image is running; commit it once with C0 so a power
     * cycle does not revert the RF rate. Next boot will match and skip flash. */
    put(F("LORA CFG: C2 dogrulandi; C0 kalici yazim deneniyor\n"));
    bool c0ok = cfg_write_full();
    if (c0ok)
    {
      (void)aux_wait(2500UL);
      mode_set(0, 0);
      (void)aux_wait(1500UL);
      mode_set(1, 1);
      if (aux_wait(500UL) && cfg_read(cur))
      {
        bool persisted_match = true;
        for (uint8_t i = 0u; i < 6u; i++)
        {
          if (cur[i] != CFG_EXPECT[i]) { persisted_match = false; break; }
        }
        put(persisted_match ? "LORA CFG: C0 kalici+DOGRULANDI\n"
                            : "LORA CFG: C0 geri-okuma farkli; RAM imaji kullaniliyor\n");
      }
      else
      {
        put(F("LORA CFG: C0 yazildi, geri-okuma yok; RAM imaji kullaniliyor\n"));
      }
    }
    else
    {
      put(F("LORA CFG: C0 kalicilik dogrulanamadi; RAM imaji kullaniliyor\n"));
    }
    match = true; /* C2 image was verified; C0 failure must not discard it */
  }
  else
  {
  /* !match0 && !match1: YALNIZ bu dalda C0 yazilir. Uzun settle'lar
   * sadece gercekten yazarken calisir (Mode0 uygula + Mode3 dogrula). */
  yazim = true;
  put(F("LORA CFG: tam imaj yaziliyor (C0 00 08)...\n"));
  /* §6.5: calisan konfigurasyon Mode 3 CIKISINDA uygulanir; ayni
   * oturumda tekrar okumak eski degerleri gosterir. Siralan: yaz ->
   * Mode 0 (uygula) -> Mode 3 (dogrula). */
  {
    bool w = cfg_write(CFG_EXPECT);
    mode_set(0, 0);
    (void)aux_wait(300UL);
    mode_set(1, 1);
    if (!aux_wait(500UL)) { w = false; }
    if (w && cfg_read(cur))
    {
      match = true;
      for (uint8_t i = 0u; i < 6u; i++)
      {
        if (cur[i] != CFG_EXPECT[i]) { match = false; break; }
      }
    }
    else
    {
      match = false;
      if (w) { (void)cfg_read(cur); } /* RD= guncel kalsin */
    }
  } /* else sonu: yazi denemesi bitti */
  } /* dis else sonu */
  mode_set(0, 0);
  (void)aux_wait(300UL);
  if (!match)
  {
    put(F("  echo ("));
    put_u32(echo_n);
    put(F(" bayt): "));
    for (uint8_t i = 0u; i < echo_n && i < 11u; i++)
    {
      char pr[3];
      pr[0] = "0123456789ABCDEF"[echo_buf[i] >> 4];
      pr[1] = "0123456789ABCDEF"[echo_buf[i] & 0x0Fu];
      pr[2] = '\0';
      Serial.write(pr); Serial.write(' ');
    }
    put(echo_n == 0u ? F(" (hic cevap yok)\n") : F("\n"));
  }
  if (yazim)
  {
  put(match ? "LORA CFG: C0 yazildi+DOGRULANDI\n" : "LORA CFG: FAILED (yazma tutmadi)\n");
  }
}

void loop()
{
  bool got = false;

  while (RX2.available())
  {
    uint8_t c = RX2.read();
    got = true;
    raw++;
    rawbuf[raww] = c;
    raww = (uint8_t)((raww + 1u) & 15u);   /* 16-byte SRAM-saving window */

    /* magic'i yakala: hizali degilsek de yeniden senkronlaniriz */
    if (len == 0u)
    {
      if (c == MAGIC) { buf[0] = c; len = 1u; n_a5++; }
      else if (c == 0x00u) { n_00++; }
      else if (c >= 0x80u) { n_hi++; }
      /* fixed-header izi: 00 02 17 siralamasi */
      hdr_hist[hdr_i] = c;
      hdr_i++;
      if (hdr_i >= 3u) { hdr_i = 0u; }
      if (hdr_seen < 3u) { hdr_seen++; }
      if (hdr_seen >= 3u)
      {
        uint8_t a = hdr_hist[hdr_i];
        uint8_t b = hdr_hist[(uint8_t)((hdr_i + 1u) % 3u)];
        uint8_t c2 = hdr_hist[(uint8_t)((hdr_i + 2u) % 3u)];
        if (a == 0x00u && b == 0x02u && c2 == 0x17u) { n_hdr++; }
      }
      continue;
    }
    buf[len++] = c;
    /* Byte 1 selects a version-specific frame length: V1=30, V2=36, V3=38. */
    uint8_t want = 0;
    if (len >= 2u)
    {
      if (buf[1] == VERSION_V1) { want = FRAME_LEN_V1; }
      else if (buf[1] == VERSION_V2) { want = FRAME_LEN_V2; }
      else if (buf[1] == VERSION_V3) { want = FRAME_LEN_V3; }
      else
      {
        bad++;
        if (c == MAGIC) { buf[0] = c; len = 1u; n_a5++; }
        else { len = 0u; }
        continue;
      }
    }
    else
    {
      want = FRAME_LEN_V3; /* version is in byte 1; buffer accommodates V3 */
    }
    if (len >= want && want != 0u)
    {
      (void)handle_frame(want);
      len = 0u;   /* sonraki magic'i bekle */
    }
  }

  /* 3 sn'dir gecerli cerceve yok: ne aldigimizi goster, sorun netlesin */
  if (millis() - last_good_ms > 3000UL)
  {
    last_good_ms = millis();
    put(F("[bekleniyor] good="));
    put_u32(good);
    put(F("  bozuk="));
    put_u32(bad);
    put(F("  RAW="));
    put_u32(raw);
    put(F(" ("));
    put_u32((raw - raw_mark + 2u) / 3u);
    put(F(" B/s)  A5="));
    put_u32(n_a5);
    put(F(" 00="));
    put_u32(n_00);
    put(F(" HI="));
    put_u32(n_hi);
    put(F(" HDR="));
    put_u32(n_hdr);
    put(F("\n"));
    raw_mark = raw;
    if (raw == 0u)
    {
      put(F("   <-- RAW=0 ise HIC BIR BAYT GELMIYOR\n"));
    }

    if (raw == 0u)
    {
      put(F("  -> Modul TXD'si D2'ye bagli mi? VCC 5V'de mi? M0->D5 M1->D6 AUX->D7 mi?\n"));
    }
    else
    {
      put(F("  -> bayt geliyor ama cerceve degil. Son 16 bayt:\n   "));
      for (uint8_t k = 0; k < 16u; k++)
      {
        uint8_t idx = (uint8_t)((raww + k) & 15u);
        uint8_t v = rawbuf[idx];
        char pair[3];
        pair[0] = "0123456789ABCDEF"[v >> 4];
        pair[1] = "0123456789ABCDEF"[v & 0x0Fu];
        pair[2] = '\0';
        Serial.write(pair);
        Serial.write(' ');
      }
      put(F("\n  A5 görünüyorsa magic dogru, CRC bozuk -> seviye sorunu.\n"));
      put(F("  A5 yoksa modul farkli modda (M0/M1) ya da yanlis baud.\n"));
    }
  }
  /* MODE-PROBE (sadece baglanti yokken, 10 sn'de bir): Mode 3'te olsaydi
   * C1 00 06'ya 9 bayt echo donerdi; Mode 0/1/2'de 3 bayt havaya gider
   * (STM32 dinlemez, zararsiz). Echo = modul Mode 3'te TAKILI (M0/M1
   * teli supheli) -> Mode 0 yeniden dayatilir. Sessizlik = Mode 3 disi.
   * Kendi sayaclarina (raw/good) dokunmaz, cerceve durumunu bozmaz. */
  {
    static uint32_t probe_ms = 0;
    if (good == 0u && (millis() - probe_ms) > 10000UL)
    {
      uint8_t got[9];
      uint8_t n = 0;
      unsigned long t0;
      const uint8_t q[3] = {0xC1u, 0x00u, 0x06u};
      probe_ms = millis();
      while (RX2.available()) { RX2.read(); }
      RX2.write(q, 3);
      t0 = millis();
      while (n < 9u && (millis() - t0) < 400UL)
      {
        while (RX2.available() && n < 9u) { got[n++] = RX2.read(); }
      }
      if (n == 9u && got[0] == 0xC1u && got[1] == 0x00u && got[2] == 0x06u)
      {
        put(F("  MODE-PROBE: Mode3 ECHO (modul Mode 3'te takili! M0/M1 supheli)\n"));
        mode_set(0, 0);
      }
      else
      {
        put(F("  MODE-PROBE: sessiz (Mode 3 disi, RF yolu supheli)\n"));
      }
    }
  }
  (void)got;
}