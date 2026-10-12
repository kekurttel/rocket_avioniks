/*
 * uno_gps_power.ino — Modulun BESLEMESINI olc. LED'e bakmadan kesin sonuc.
 *
 * NEDEN: IDLE D0=1 (hat bos yuksek) ama rx=0. Bu, modulun TX cikisinin
 * suruklemedigi anlamina gelir. Suruklemeyen bir cikis, modulun kapali
 * (VCC yok) ya da UART'i kapali oldugu anlamina gelir. Ayirt etmek icin
 * modulun VCC gerilimini A0 uzerinden olceriz.
 *
 * BAĞLAMA:
 *   modul VCC  -> Uno 5V   (zaten vardi)
 *   modul VCC  -> Uno A0   (YENI: gerilim olcumu)
 *   modul GND  -> Uno GND
 *   modul TX   -> Uno D0
 *
 * A0'a 5V dogrul girer: AVR ADC'si AVCC'yi referans alir (5V) ve giris
 * tam olarak AVCC'ye kadar sorunsuz okunur. Yine de guvenlik icin seri
 * direnc onerilir (1k) — 1k ile 5V -> ~4.98V okunur, yeterli.
 *
 * Serial Monitor 115200
 */

static void put(const char *s) { Serial.write(s); }

static void put_u32(uint32_t v)
{
  char b[11]; uint8_t i = 0;
  if (v == 0u) { Serial.write('0'); return; }
  while ((v > 0u) && (i < (uint8_t)sizeof b))
  { b[i++] = (char)('0' + (v % 10u)); v /= 10u; }
  while (i > 0u) { Serial.write(b[--i]); }
}

/* AVR ADC: 10-bit. referans = AVCC (5V). */
static uint16_t adc_read(uint8_t ch)
{
  ADMUX = (uint8_t)((1 << REFS0) | (ch & 0x07)); /* AVCC referans */
  ADCSRA = (uint8_t)((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  /* ilk okumayi at (ACP kalibrasyonu) */
  (void)ADCSRA;
  (void)ADCSRA;
  while ((ADCSRA & (1 << ADSC)) != 0) { }
  return (uint16_t)ADC;
}

static uint16_t mv_of(uint16_t adc)
{
  /* 1023 adim = AVCC. AVCC = 5000 mV (USB'den besleniyoruz). */
  return (uint16_t)(((uint32_t)adc * 5000UL) / 1023UL);
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  put("\n=== MODUL BESLEME OLCUMU ===\n");
  put("BOOT micros=");
  put_u32(micros());
  put("  (degistiyse reset calisiyor)\n");
  put("A0 modulun VCC'sine bagli. AVCC=5000mV referans.\n\n");

  /* VCC 5V mu, yoksa 0 mu? 20 kez olc, ortalama al. */
  uint32_t sum = 0;
  const uint8_t N = 20;
  for (uint8_t i = 0; i < N; i++)
  {
    sum += adc_read(0);
    delay(10);
  }
  uint16_t adc = (uint16_t)(sum / N);
  uint16_t mv = mv_of(adc);

  put("A0 ADC = ");
  put_u32(adc);
  put(" / 1023\n");
  put("A0 = ");
  put_u32(mv);
  put(" mV\n\n");

  if (mv > 4400u)
  {
    put(">>> VCC SAGLAM (~5V). Modul BESLENIYOR.\n");
    put(">>> O zaman sorun TX hattinda: modul UART'i kapali (mode/EN pini).\n");
  }
  else if (mv > 3000u)
  {
    put(">>> VCC KISMEN var (3-4.4V). Besleme zayif: kablo/breadboard sorunu.\n");
  }
  else if (mv > 200u)
  {
    put(">>> VCC YOK / kisa. Modul BESLENMIYOR (ya da VCC yanlis pinde).\n");
  }
  else
  {
    put(">>> VCC = 0. A0 ucu modul VCC'sine BAGLI DEGIL ya da modul yok.\n");
  }

  /* D0 idle seviyesi + kisa sure dinleme */
  pinMode(0, INPUT_PULLUP);
  delay(5);
  put("\nD0 bos seviye = ");
  Serial.write(digitalRead(0) ? '1' : '0');
  put("  (1=modul surukluyor/normal, 0=kisa devre)\n");

  put("D0'dan 3 sn dinleniyor...\n");
  uint32_t n = 0;
  unsigned long t0 = millis();
  while ((millis() - t0) < 3000UL)
  {
    if (Serial.available())
    {
      uint8_t c = Serial.read();
      n++;
      put("  bayt: 0x");
      Serial.write("0123456789ABCDEF"[c >> 4]);
      Serial.write("0123456789ABCDEF"[c & 0x0F]);
      put(" '");
      if ((c >= 32u) && (c < 127u)) { Serial.write((char)c); }
      else { Serial.write('.'); }
      put("'\n");
    }
  }
  put("toplam bayt = ");
  put_u32(n);
  put(n == 0u ? "  (0 = modul konusmuyor)\n" : "\n");
}

void loop()
{
  /* tek seferlik */
  delay(1000);
}