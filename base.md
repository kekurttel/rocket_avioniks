# base.md — Proje Mimarisi (KALICI)

> TEKNOFEST roket aviyoniği. Bu dosya **nihai donanım/yazılım mimarisini** ve
> çalışması için bilinmesi zorunlu tuzakları tanımlar.

---

## 1. MİMARİ — İKİ KART, TEK MERKEZ

```
┌──────────────────── STM32F446RE NUCLEO ────────────────────┐
│                                                            │
│  USART2 (PA2/PA3)  → VCOM, 115200   Ek--7 SIT/SUT + TUI  │
│  USART6 (PC6/PC7)  → GPS,  9600     NEO-6M, 4 pin         │
│  USART1 (PB6/PB7)  → LoRa TX, 9600  E220-900T22D, 7 pin  │
│                                                            │
│  I2C1  (PB8/PB9)   → MPU6050 + BMP180                    │
│  GPIO              → LED'ler, B1/B2 butonları            │
│                                                            │
│  = UÇUŞUN TAMAMI BURADA. Tek muhafız, tek karar merkezi.  │
└────────────────────────────┬───────────────────────────────┘
                             │  900 MHz LoRa, 30 bayt/500 ms
                             │  (tek yönlü: roket → istasyon)
                             ▼
┌──────────────────── Arduino UNO (yer istasyonu) ──────────┐
│                                                            │
│  USART0 (D0/D1)  → LoRa RX, 9600     sadece ALIR          │
│  USART0 (D0/D1)  → ekran, 115200     sadece GOSTERIR      │
│                                                            │
│  HİÇBİR ŞEY ÜRETMEZ. Sensör yok, karar yok, uçuş yok.    │
│  Tek işi: 30 baytlık paketi almak ve ekranda göstermek.   │
└────────────────────────────────────────────────────────────┘
```

### Neden bu ayrım?
- **Uçuş güvenliği:** Rokette tek muhafız var. Yer istasyonu arızalansa/düşse bile uçuş etkilenmez.
- **GPS uçuşu etkilemez:** `gps_has_fix()` sözleşmesi — GPS yoksa uçuş normal devam eder, sadece telemetride `sats=0` görünür.
- **LoRa asla karar katmanına dokunamaz:** `lora_poll()` yalnızca okur, hiçbir şey yazmaz.

---

## 2. BAĞLAMA TABLOLARI

### 2.1 GPS — Ebyte NEO-6M (4 pin: `VCC TX RX GND`)
| Modül | STM32 | Not |
|---|---|---|
| VCC | **5V** | ⚠️ 3.3V yetmez → brownout! (Bkz. §5) |
| GND | GND | CN4 pin 2 |
| TX | **PC7 (D9)** = USART6_RX | modül bize konuşur |
| RX | **PC6** = USART6_TX | bize modüle UBX komutu (pad-only) |

**Kondansatör zorunlu:** modül VCC–GND arasına **100 µF + 100 nF**, doğrudan modülün pinlerine.

### 2.2 LoRa — Ebyte E220-900T22D (7 pin: `M0 M1 RXD TXD AUX VCC GND`)
| Modül | STM32 | Uno | Not |
|---|---|---|---|
| M0 | **PB10 (D6)** GPIO çıkış | **D5** GPIO çıkış | Mode 0 = (0,0) şeffaf; Mode 3 = (1,1) konfigürasyon |
| M1 | **PA8 (D7)** GPIO çıkış | **D6** GPIO çıkış | boot'ta Mode 3 → kontrol → Mode 0 |
| RXD | **PB6 (D10)** = USART1_TX | **D3** (sadece konfig yazma; 1k seri/2k-1k bölücü önerilir) | STM32 3.3V direkt; Uno D3 5V → 3.3V girişe dikkat |
| TXD | **PB7 (D11)** = USART1_RX | **D2** | bize istasyona |
| AUX | **PA9 (D8)** GPIO giriş | **D7** GPIO giriş (D4 bozuk) | LOW = meşgul; her mod değişiminde AUX yüksek + 2 ms beklenir |
| VCC | **5V** | 5V | ⚠️ TX anında ~110 mA |
| GND | GND | GND | |

Boot konfigürasyonu (her iki taraf): Mode 3 → `C1 00 06` ile 00H..05H
oku → beklenen `{00,00,62,00,17,00}` (adres 0000, 9600 8N1, 2.4k, 200B,
22dBm, CH23 = 873.125 MHz, şeffaf) değilse `C0 00 06` ile yaz + doğrula
→ Mode 0'a dön. Fabrika ayarı zaten bu olduğu için sağlam modül
yazılmaz. TUI: `LORA CFG: OK/FIXED/FAILED`; Uno: `LORA CFG: ...` satırı.
`FAILED` uçuşu/alıcıyı durdurmaz.

---

## 3. PROTOKOL

### 3.1 LoRa çerçevesi — 30 bayt, little-endian
```
 0 u8  magic 0xA5        1 u8  ver 0x01      2 u16 seq
 4 u32 t_ms              8 u16 flight_state  10 u16 ek7_mode
12 i16 alt_m (METRE)    14 i16 vel_cms       16 u16 sats
18 i32 lat_e5           22 i32 lon_e5       26 u16 batt_mv
28 u8  flags             29 u8  crc = sum(0..28) mod 256
```
- CRC doğrulama: **30 baytın toplamı 0 vermeli**
- `LORA_ALT_INVALID = 0x8000`, `LORA_I16_INVALID = 0x8000`
- `flags`: bit0 gps_fix, bit1 blue, bit2 yellow, bit3 red
- **Gönderim:** 500 ms (uçuşta), 1000 ms (`WAIT_FOR_LAUNCH`)
- **SIT/SUT'ta gönderim yok** (`LORA_ALLOW_SIT_SUT == 0`) — Ek-7 akışını bozmasın

### 3.2 Ek-7 (resmî SIT/SUT)
Doküman: `SIT_SUT_REQUIREMENTS.md`. **Değiştirilmedi** — resmî yazılımla
byte-uyumlu kalmalı. Modlar: `REAL` / `TEST` / `SIT` / `SUT`.

---

## 4. TUZAKLAR — BİLMEK ZORUNDA

### 4.1 🔴 CubeMX "Generate Code" USART6'YI SİLER
`.ioc` içinde USART6 **yok**, konfigürasyon el yazımı, ve **üçü de
`USER CODE` blokları DIŞINDA**:
```
Src/main.c:926    MX_USART6_UART_Init();        çağrı
Src/main.c:2769   void MX_USART6_UART_Init()   tanım
Src/stm32f4xx_hal_msp.c:183  else if(USART6)   pin config
```
→ Generate Code üçünü de siler, **derleme hatası vermez**, GPS sessizce
ölür. **CubeMX'i açma.**

### 4.2 🔴 USART1 PA9/PA10 KULLANILAMAZ
PA10 = **mavi LED** (`main.c:63-64`, aktif sürülüyor `:951, :2187, :2210`).
LoRa için **PB6/PB7 + AF7** (USART1'in alternatif çifti, Arduino D10/D11).

### 4.3 🔴 GPS BESLEME — EN SIK ÇUVDAN ÇIKAN HATA
NEO-6M `Icc acquisition min 47 mA`, `tracking min 39 mA`. Breadboard +
ince kablo yetmez → modül açılış banner'ını **yarım** yazar
(`$GPT` + bozuk bayt, hiç satır sonru yok) → sonsuz reset döngüsü.
Belirti: `err=0` + `$` var + **`NL=0, CR=0`** → baud değil, **besleme**.
Çözüm: 5V + 100 µF/100 nF.

### 4.4 🔴 Uno: MODÜL D0'ı SÜRÜYORSA HİÇBİR ŞEY ÇALIŞMAZ
ATmega328P bootloader'ı **D0/D1'i** STK500 sync için kullanır. Modülün TX'i
D0'a bağlıysa:
- `avrdude` → `not in sync: resp=0x0a`, **programmer is not responding**
- **RESET butonu yeni kodu tetiklemez** (TX hattı bozulur)
- `RAW` artar ama **hepsi `00`** (hat sürükleniyor)

**Çözüm:** LoRa/GPS modülünü Uno'da **D0/D1'e değil D2/D3'e** bağla.
`SoftwareSerial` ile sür, ekran D0/D1'de 115200'de kalsın.
Bonus: modül takılıyken bile yükleme yapılabilir.

### 4.5 🔴 Tek UART'ta iki farklı baud İMKÂNSIZ
`Serial.begin(9600)` **tüm** çıktıyı 9600'e düşürür. Ekran 115200'de
kalacaksa GPS **ayrı portta** olmalı (SoftwareSerial veya ayrı donanım UART).

### 4.6 LoRa modülü SPI DEĞİL
E220-900T22D: `M0 M1 RXD TXD AUX VCC GND` — **SCK/MISO/MOSI yok.**
LLCC68 çipli ama arayüzü şeffaf UART.

### 4.7 5V MCU ← 3.3V modül TX
ATmega328P `VIH = 0.6 × VCC` → 5V'ta eşik **3.0 V**, modül **3.3 V** verir:
0.3 V marj. Çalışmazsa: modül TX'ini **A0'a** al, `analogRead() > 512`
eşiğiyle oku (parçasız çözüm).

---

## 5. GPS UBX KONFİGÜRASYONU (gen6)

Firmware boot'ta **8 komut** gönderir, `WAIT_FOR_LAUNCH` + NORMAL modda,
yalnız pad'de. `OK 8/8` beklenir.

| # | Komut | Değer |
|---|---|---|
| 0 | CFG-MSG `F0 00` (GGA) | rate **1** |
| 1 | CFG-MSG `F0 04` (RMC) | rate **1** |
| 2 | CFG-MSG `F0 01` (GLL) | rate **0** |
| 3 | CFG-MSG `F0 02` (GSA) | rate **0** |
| 4 | CFG-MSG `F0 03` (GSV) | rate **0** |
| 5 | CFG-MSG `F0 05` (VTG) | rate **0** |
| 6 | CFG-NAV5 | dynModel **7** = Airborne<4g |
| 7 | CFG-RATE | **1000 ms** = 1 Hz |

**Düzeltilmiş hatalar (bilerek yapıldı):**
- CFG-MSG **6 bayt** (`rateUART1` slotu) — 3-byte form **yok**, "current
  port" çözümlemesi **yok**
- `dynModel 7` — **8/9/10 gen7+** numaralandırması; gen6'da 3-bit (0..7)
- `measRate 1000` — NEO-6 **1 Hz** parça, gen6 tabanı 1000 ms

---

## 6. DOĞRULAMA — HOCAYA SUNULACAK KANITLAR

```bash
# Üretim build (Debug = üretim)
cmake --build --preset Debug
# NOT: 'Bench' preset'i CMakePresets.json'da YOK; build/Bench ayrı dizindir
cmake --build build/Bench

# Flash
arm-none-eabi-objcopy -O binary build/Debug/rocket_avioniks.elf /tmp/prod.bin
st-flash --reset write /tmp/prod.bin 0x8000000

# Canlı TUI
stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0
```

**TUI satırları:**
```
MODE: REAL  LOCK: OPEN  REJ=0
FLIGHT: WAIT_FOR_LAUNCH PROD
GPS: NOFIX sats=0 rx=1533
GPSCFG: OK 8/8
LORA: 433 ok=433 drop=0
LED: BLUE=0 YEL=0 RED=0
```

**Host testleri (7 suit):**
```bash
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_flight_host.c   Src/flight.c Src/avionics.c -lm -o /tmp/tf && /tmp/tf   # 668
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_ek7_host.c     Src/ek7.c Src/flight.c Src/avionics.c -lm -o /tmp/te && /tmp/te   # 97
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_avionics_host.c Src/avionics.c -lm -o /tmp/ta && /tmp/ta                                # 183
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_runmode_host.c Src/runmode.c Src/flight.c Src/avionics.c -lm -o /tmp/tr && /tmp/tr      # 35
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_gps_host.c      Src/gps.c -lm -o /tmp/tg && /tmp/tg                                     # 92
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_gpscfg_host.c   Src/gps.c -lm -o /tmp/tc && /tmp/tc                                     # 161
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_lora_host.c     Src/lora.c -lm -o /tmp/tl && /tmp/tl                                   # 6295
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_tui_buf_host.c -lm -o /tmp/tt && /tmp/tt
```

**Kabul kanıtları:** `OK 8/8` (orijinal u-blox kanıtı — klon ACK vermez),
`drop=0`, `err=0`, 6 host suiti + lora, sıfır derleme uyarısı.

---

## 7. PENDING

- [ ] Modülleri ≥ 2 m ayır, LoRa uçtan uca doğrula (`[LINK] seq=` artmalı)
- [ ] Anteni gökyüzüne çıkar → `sats` → `FIX`, `pos=` doğrula
- [ ] Uno'da modülü **D2/D3'e** taşı (§4.4) — sonra LoRa alıcı testi
- [ ] TUI'nin ~47 ms blokajını DMA'ya taşı (uçuş çözünürlüğü)
- [ ] `SIT_SUT_REQUIREMENTS.md` güncelle (CFG-MSG/dynModel/measRate değişti)
- [ ] USB kabloları değiştir (iki kartta da port düşüyor)
