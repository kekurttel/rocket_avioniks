/*
 * uno_gps_ledverdict.ino — Modul beslemesini LED'e yazar. USB YOK.
 *
 * NEDEN: Bu CH340 klonda USB seri yolu guvenilir degil. Kart reset olmuyor
 * (micros() degeri iki calistirmada birebir ayni) ve CH340 ayni tamponu
 * tekrar tekrar bosaltiyor: 350 bayt, 660 bayt... hepsi eski veri.
 * Bu yuzden USB'den HICBIR ciktiya guvenmiyoruz.
 *
 * COZUM: Firmware olcumu bir LED'e yazar. LED fiziksel bir cikti;
 * USB/CH340 dongusu devre disi kalir. Test artik USB'den bagimsiz.
 *
 * KABLOLAMA:
 *   modul VCC -> 5V
 *   modul VCC -> A0      (ADC olcumu)
 *   modul VCC -> D8      (LED + 330 ohm direnc, LED katotu GND'ye)
 *   modul GND -> GND
 *   modul TX  -> D0
 *
 * LED ANLAMLARI:
 *   YESIL  = modul ~5V aliyor -> BESLENIYOR, modul saglam, sorun TX'te
 *   KIRMIZI= modul 0V aliyor  -> BESLEME KOPUK (kablo/breadboard/modul)
 *   SARI   = 3.0-4.4V         -> besleme zayif, temas sorunu
 *   (hicbiri yanmiyorsa: LED ya da D8 baglantisi yok)
 *
 * LED 10 sn yanip soner, sonra SABIT yanar (sabit = karar ver).
 * Her saniye bir kez kisa parlama yapmak "calisiyor" gostergesidir.
 */

#define PIN_VCC_LED 8

static uint16_t sample_vcc_mv(void)
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 24; i++)
  {
    ADMUX = (uint8_t)((1 << REFS0) | 0);   /* AVCC referans, A0 */
    ADCSRA = (uint8_t)((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
    (void)ADCSRA; (void)ADCSRA;            /* ilk okuma kalibrasyon */
    while ((ADCSRA & (1 << ADSC)) != 0) { }
    sum += ADC;
    delay(4);
  }
  return (uint16_t)(((uint32_t)(sum / 24u) * 5000UL) / 1023UL);
}

void setup()
{
  pinMode(PIN_VCC_LED, OUTPUT);
  digitalWrite(PIN_VCC_LED, LOW);

  delay(300);

  uint16_t mv = sample_vcc_mv();
  bool on;

  if (mv > 4400u)      { on = true;  }   /* YESIL  : besleniyor */
  else if (mv > 3000u) { on = true;  }   /* SARI   : zayif      */
  else                 { on = true;  }   /* KIRMIZI: yok        */

  /* Renk kodlamasi: ayni tek LED ile farkli desen kullan.
   * YESIL = surekli parlak (2 sn)
   * SARI  = 500ms yanik / 500ms sonuk (3 sn)
   * KIRMIZI = 150ms kisa parlak (6 sn)
   * Bu desen USB olmadan da anlasilir. */
  if (mv > 4400u)
  {
    /* YESIL: 3 sn surekli parlak, sonra SABIT */
    unsigned long t = millis();
    while ((millis() - t) < 3000UL)
    {
      digitalWrite(PIN_VCC_LED, HIGH);
      delay(20);
    }
  }
  else if (mv > 3000u)
  {
    /* SARI: 3 donanim donme */
    for (uint8_t i = 0; i < 6; i++)
    {
      digitalWrite(PIN_VCC_LED, HIGH);  delay(400);
      digitalWrite(PIN_VCC_LED, LOW);   delay(400);
    }
  }
  else
  {
    /* KIRMIZI: 10 kisa parlak */
    for (uint8_t i = 0; i < 10; i++)
    {
      digitalWrite(PIN_VCC_LED, HIGH);  delay(120);
      digitalWrite(PIN_VCC_LED, LOW);   delay(180);
    }
  }

  /* SABIT karar: surekli yanik = karar verildi */
  for (;;)
  {
    digitalWrite(PIN_VCC_LED, HIGH);
    delay(100);
  }
}

void loop()
{
  /* setup icinde biten tek seferlik test */
}