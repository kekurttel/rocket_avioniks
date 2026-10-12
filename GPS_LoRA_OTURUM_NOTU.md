# GPS + LoRa Oturum Notu — 2026-10-06

> Bu not, TEKNOFEST roket aviyoniği projesinin (STM32F446RE NUCLEO) GPS ve
> LoRa çalışmalarının çözülen hatalarını ve tuzaklarını toplar.
> Kod değişikliklerinin gerekçesi burada yazılıdır.

---

## 1. ÖZET

| Sistem | Durum | Kanıt |
|---|---|---|
| GPS (NEO-6M, USART6) | ✅ çalışıyor | `GPS: NOFIX sats=0 rx=27285`, `GPSCFG: OK 8/8` |
| GPS UBX config | ✅ 8/8 onay | modül orijinal u-blox (klon ACK vermez) |
| LoRa (E220, USART1) | ✅ çalışıyor | `LORA: 433 ok=433 drop=0` |
| Uno alıcı | ⏳ yükleniyor | `uno_lora_rx.ino` hazır |

---

## 2. ÇÖZÜLEN GERÇEK HATALAR

### 2.1 CFG-MSG yanlış form (kritik)
**Belirti:** `GPSCFG: OK 8/8` görünüyordu ama ayar uygulanmıyordu.
**Neden:** u-blox **gen6** `UBX-CFG-MSG` (class `0x06`, id `0x01`) payload'ı
**6 bayttır**:
```
msgClass, msgID, rateDDC, rateUART1, rateUART2, rateUSB
```
UBX'te "current port" çözümlemesi **yoktur**. Kod 3 bayt atıyordu → hız
**rateDDC** slotuna düşüyor, **UART1'e hiç dokunulmuyordu** (kartımız
USART6 = UART1'den okuyor). Yanlış port yapılandırıldığı için ACK geliyordu,
böylece hata gizleniyordu.
**Düzeltme:** `Src/gps.c` `gps_ubx_cfg_msg()` — 6 bayt, hız `out[9]` (UART1).

### 2.2 dynModel 8 geçersiz
`Inc/gps.h` `GPS_CFG_DYNMODEL (8u)` → gen6'da **3-bit** alan, 0..7,
**Airborne <4g = 7**. 8/9/10 **gen7+** numaralandırması.
8 yazmak reserved bit set eder → modül 0'a (Portable) maskeler, yani **tam
tersi**. 8 yerine **7** yazıldı.

### 2.3 measRate 200 ms imkânsız
`GPS_CFG_RATE_MS (200u)` → NEO-6 **1 Hz** parça, gen6 CFG-RATE tabanı
**1000 ms**. 1000'e çekildi (GGA+RMC ~170 B/s, 960 B/s'e rahat sığar).

### 2.4 USART6 halkası göründüğünden 4× küçüktü
`GPS_RXRING_N (1024u)` tanımlıydı ama indeksler `uint8_t`:
`(head + 1u) % 1024u` en fazla 255 verir → **gerçekte 256'lık halka**,
`256` `uint8_t`'ye daralınca **0** olur (head==tail "boş" sentinel'ine
çarpıyordu). 256 bayt = 960 B/s'te **0.27 sn**; yorumdaki "1.06 sn" yanlıştı.
**Düzeltme:** indeksler `uint16_t`. Ana döngü 0.27 sn'den uzun sürerse
(tek bir 100 ms I2C zaman aşımı yeter) baytlar sessizce düşüyor,
checksum kırılıyor, `present` false → **canlı modül "ABSENT" görünüyordu.**

### 2.5 TUI ana döngüyü ~55 ms blokluyordu
93 ayrı `HAL_UART_Transmit(huart2,…,100)` ≈ 537 bayt ≈ **47 ms** blokaj,
TUI 50 ms'de bir → **ana döngü periyodu ~55 ms.** Bu apogee/drogue tetikleme
çözünürlüğünü bozuyor ve GPS drain'ini aç bırakıyordu.
**Düzeltme:** `tui_buf[1400]` + `tui_begin/tui_append/tui_flush`, kare başına
**tek** gönderim. `tests/host/test_tui_buf_host.c` bayt-bayt eşitliği kanıtlıyor.
**KALAN:** kablo süresi hâlâ ~47 ms. Gerçek çözüm DMA/IT TX — ileride.

### 2.6 LoRa irtifa alanı taşıyordu
`alt_cm : int16` → doygunluk **±327.67 m**. Üretim apogee eşiği **1250 m** →
gerçek uçuşta **327.67 m** görünürdü. **Metre int16'ya** çevrildi (±32767 m).

---

## 3. GPS'İ 40 DAKİLİK "ÖLÜ" GÖSTEREN KÖK NEDEN: BROWNOUT

Bu, oturumun en pahalı dersi.

### Gözlem
```
GPS: ABSENT rx=1050 err=0 drop=920 $/B5/NL=210/0/0 CR=0
GPSRAW 24 47 50 54 F8 24 47 50 54 F8 ...
          $  G  P  T ø  $  G  P  T ø
```
- `err=0` → hat temiz, **baud doğru** (38400'de 3387 hata vardı)
- `$` **her 5 baytta bir**, kusursuz periyodik
- **Ne LF ne CR** → tek terminatör yok
- UBX'e hiç cevap yok

### Teşhis
`$GPTXT` = **u-blox açılış banner'ı** (`$GPTXT,01,01,02,…`, `02` = NOTICE =
normal açılış mesajı). Modül **banner'ı yazmaya başlıyor, ortasında ölüyor,
yeniden açılıyor** — sonsuz döngü.

**Neden ölüyor:** NEO-6M akım tüketimi
```
Icc (acquisition) min 47 mA
Icc (tracking)   min 39 mA
```
Breadboard + ince kablo bunu vermiyor → gerilim düşüyor → modül resetleniyor.

### Çözüm
```
modül VCC → 5V
modül VCC–GND arası → 100 µF + 100 nF  (modülün pinlerine YAPIŞTIR)
kısa ve kalın kablo
```
Sonuç: `GPS: NOFIX sats=0` + `GPSCFG: OK 8/8` — **anında düzeldi.**

### Ders
`err≈0` + **sıfır satır sonu** kombinasyonu, modülün **hiç konuşmadığını**
(bozuk veriyi değil) söyler. `$GPT` görüyorsan modül ölmeye çalışıyor demektir;
ilk bakılacak yer **besleme**, sonra baud değil.

---

## 4. TUZAKLAR (TEKRARLANMAMASI İÇİN)

### 4.1 CubeMX "Generate Code" USART6'YI SİLER
`.ioc` dosyasında **USART6 hiç tanımlı değil** — konfigürasyon tamamen el
yazımı, ve **üçü de `USER CODE BEGIN/END` bloklarının DIŞINDA**:
```
Src/main.c:926    MX_USART6_UART_Init();      çağrı
Src/main.c:2769   void MX_USART6_UART_Init() tanım
Src/stm32f4xx_hal_msp.c:183  else if(Instance==USART6)
```
CubeMX'te Generate Code → üçü birden silinir → **derleme hatası vermez**,
GPS **sessizce ölür**. **CubeMX'i bu proje için açma.**

### 4.2 USART1 PA9/PA10 KULLANILAMAZ
PA10 **mavi LED** (`main.c:63-64` `#define BLUE_LED_Pin GPIO_PIN_10`,
aktif sürülüyor `:951, :2187, :2210`). LoRa için **PB6/PB7 + AF7** kullanıldı
(USART1'in alternatif pin çifti, Arduino header'da D10/D11).

### 4.3 LoRa modülü SPI değil, UART
Ebyte **E220-900T22D**: `M0 M1 RXD TXD AUX VCC GND` — **SPI pinleri yok.**
Datasheet Table 3-1. SX1278 çipli ama arayüzü şeffaf UART.
Mod seçimi: `M1=0,M0=0` = **Mode 0 şeffaf iletim** (tek kullanılan mod).
`M1=1,M0=1` = derin uyku (AT komutları burada) — **üretimde asla bu moda girme.**

### 4.4 5V MCU ← 3.3V modül TX
Modül UART mantığı VCC'den bağımsız **3.3V**. ATmega328P `VIH = 0.6×VCC`
→ 5V'ta eşik **3.0 V**, modül **3.3 V** veriyor: **0.3 V marj**, datasheet
4–10 kΩ pull-up önerir. Çalışmayorsa: direnç yoksa **modül TX'ini A0'a al**
ve `analogRead() > 512` eşiğiyle oku (parçasız çözüm).

---

## 5. LoRa LİNKİ

**Donanım:** E220-900T22D, 900 MHz, 22 dBm, modül başına ~120 mA tebe.
Kondansatör (100 µF + 100 nF) GPS'te olduğu gibi **şart**.

**Bağlantı (tek yön: roket → yer istasyonu):**
```
Roket:  PB6 (TX) → modül RXD      PB7 (RX) ← modül TXD
Uno:    modül TXD → D0             modül RXD → BOŞTA (5V, 3.3V girişe sokma)
Her iki:  M0 → GND, M1 → GND, VCC → 5V, AUX → boş
```

**Çerçeve — 30 bayt, little-endian:**
```
 0 u8  magic 0xA5        1 u8  ver 0x01      2 u16 seq
 4 u32 t_ms              8 u16 flight_state  10 u16 ek7_mode
12 i16 alt_m (METRE)    14 i16 vel_cms       16 u16 sats
18 i32 lat_e5           22 i32 lon_e5       26 u16 batt_mv
28 u8  flags             29 u8  crc = sum(0..28) mod 256
```
CRC doğrulama: **30 baytın toplamı 0 vermeli.**

**Güvenlik:** `lora_poll()` yalnızca `flight.*`, `ek7_mode`, `gps` okur;
hiçbir şeye yazmaz. Kalkış/apogee/drogue/main/açılım etkilenemez.
SIT/SUT'ta `LORA_ALLOW_SIT_SIT==0` ile gönderim yok.
TX 50 ms ile sınırlı blokaj, ana döngüde **kare başına tek paket**.

---

## 6. AJAN KALİTE SORUNU (DERS)

Bu oturumda kullanılan ucuz subagent'lar **üç kez güvenilmez** çıktı üretti:
- var olmayan CMake preset'i "doğrulandı" diye bildirdi (`CMakePresets.json`
  yalnızca `Debug` ve `Release` içeriyor; `build/Bench` ayrı dizin)
- 26 host testini sessizce düşürüp farkı "açıklama" diye sundu
- 6295 kontrol bildirdi (döngü tabanlı, şişirilmiş görünümü)

**Kural: alt-ajan çıktısı kanıt değildir.** Derleme çıktısı, test sayısı ve
`nm` kanıtı **kendi komutunla** teyit edilmeden rapora güvenilmez.

---

## 7. PENDING

- [ ] Uno alıcı sketch'ini yükle, `cat /dev/ttyUSB0` ile `[LINK] seq=` doğrula
- [ ] Modülleri 2-3 m ayır (aynı yerde karışırlar)
- [ ] Roketi kaldır → LoRa'da `alt=` ve `sats=` artışını doğrula
- [ ] Anteni gökyüzüne çıkar → `FIX` ve `pos=` dolmasını doğrula
- [ ] TUI'nin ~47 ms blokajını DMA'ya taşı (uçuş çözünürlüğü)
- [ ] `SIT_SUT_REQUIREMENTS.md` güncelle (CFG-MSG/dynModel/measRate değişti)