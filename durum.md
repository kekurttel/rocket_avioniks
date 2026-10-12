# DURUM — Roket Aviyoniği Proje Durum Dosyası (Türkçe, ayrıntılı)

> TEKNOFEST roket aviyoniği: STM32F446RE NUCLEO (uçuş merkezi) + Arduino Uno (yer istasyonu, yalnızca alıcı).
> Bu dosya 2026-10-09 tarihinde kodun tamamı okunarak yazıldı. Her teknik iddia `dosya:satır` ile kanıtlı.
> Okuma kapsamı: `Src/*.c`, `Inc/*.h`, `tests/host/*`, `uno_*/*.ino`, `uno_gps_test/*`,
> `CMakeLists.txt`, `CMakePresets.json`, `base.md`, `GPS_LoRA_OTURUM_NOTU.md`,
> `SIT_SUT_REQUIREMENTS.md`, `E220-900T22D_UserManual_EN_v1.2.txt`, `rocket_avioniks.ioc`,
> `pinsketch.md`, `precommands.md`, `Src/stm32f4xx_hal_msp.c`, `Src/stm32f4xx_it.c`.
> Host test sayıları bu dosya için yeniden derlenip çalıştırılarak doğrulandı (Bölüm 7).

---

## 0. ÖZET + CANLI DURUM TABLOSU

### 0.1 Tek cümlelik mimari

STM32 uçuşun tamamını yapar (sensör → karar → Ek-7 + LoRa telemetri); Uno yalnızca LoRa paketini alıp
ekrana basar, hiçbir karar vermez (`base.md:8-38`).

### 0.2 Canlı durum (bilinen son gözlenen tablo)

| Alt sistem | Durum | Kanıt / not |
|---|---|---|
| STM32 uçuş yazılımı | Çalışıyor, üretim (PROD) modunda | `FLIGHT: WAIT_FOR_LAUNCH PROD` |
| LoRa verici (STM32→hava) | `FIXED` + gönderiyor (`ok` artıyor) | `LORA CFG: FIXED ...`, `LORA: <seq> ok=N drop=0` |
| LoRa konfigürasyon kalıcılığı | C0 flash yazmaya echo yok; C2 RAM tutuyor, güç kesilince link yeniden FIX gerekiyor | `Src/lora.c:499-501`, `Src/lora.c:573-611` |
| GPS (NEO-6M) | Konuşuyor ama `NOFIX sats=0` (iç mekân) | `GPS: NOFIX sats=0 rx=...` |
| GPS UBX konfigürasyonu | `OK 8/8` | `GPSCFG: OK 8/8` |
| Uno alıcı | `RAW=0` — hiçbir bayt gelmiyor | Birincil şüphe: M1/D6 hattı açık → modül Mode 0 dışında takılı; ikincil şüphe: RF/desense/anten |
| Ek-7 SIT/SUT | Kilitli, regresyon PASS | host `test_ek7_host` 97/97 |

### 0.3 TUI satırları ve anlamları (hepsi `Src/main.c` içinde üretilir)

| TUI satırı | Üreten kod | Anlamı |
|---|---|---|
| `LORA CFG: <OK\|FIXED\|FAILED\|SKIPPED> AUX=<0/1/?> B=<baud\|--> RD=<6 hex\|--> ST=<0-4> W=<-/0/1/2>` | `Src/main.c:964-1041` | Boot LoRa konfig sonucu. `OK` = imaj zaten doğru, yazılmadı. `FIXED` = uyumsuzluk bulundu, yazıldı + doğrulandı. `FAILED` = uçuşu durdurmaz. `AUX` = çıkıştaki AUX seviyesi (`0` = düşük, `?` = bilinmiyor). `B` = Mode 3 alışverişinin yakalandığı baud (`--` = yok). `RD` = son register okuması (6 bayt hex). `ST` = aşama: 1 = hiç okuma yok, 2 = echo tutmadı, 3 = doğrulama tutmadı, 4 = tamam (`Src/main.c:1023-1024` yorumu). `W` = kazanan yazma varyantı: 0 = C2, 1 = C0, 2 = C0-8 (`Src/main.c:1024`, `Src/lora.c:565-611`) |
| `GPS: ABSENT rx=<n> err=<n> drop=<n> $/B5/NL=<a>/<b>/<c> CR=<d>` (+ arasıra `GPSRAW <hex>`) | `Src/main.c:2623-2670` | Hiç checksum-geçerli NMEA cümlesi görülmedi. `rx` = beslenen ham bayt, `err` = USART6 çizgi hatası, `drop` = halka dolu kaybı, `$`/`B5`/`NL`/`CR` = içerik parmak izi (`Src/main.c:367-380`). `err≈0` + sıfır satır sonu = besleme sorunu, baud değil (`GPS_LoRA_OTURUM_NOTU.md:68-107`) |
| `GPS: NOFIX sats=<n> rx=<n>` | `Src/main.c:2671-2678` | Modül konuşuyor ama taze geçerli fix yok |
| `GPS: FIX sats=<n> lat=<..> lon=<..> [alt=..m] [spd=..m/s]` | `Src/main.c:2679-2700` | Taze geçerli konum (`gps_has_fix`, `Inc/gps.h:96-101`) |
| `GPSCFG: <IDLE\|SENDING\|OK\|NAK\|NOACK\|ABORTED> <acked>/8` | `Src/main.c:2703-2709` | UBX pad konfigürasyon durumu; hedef `OK 8/8` |
| `LORA: <seq> ok=<n> drop=<n> auxbusy=<n>` | `Src/main.c:2713-2721` | `seq` = gelecek paket no, `ok` = UART'a verilen paket, `drop` = düşen paket, `auxbusy` = gönderim sonrası AUX-düşük örnekleri (modülün baytı havaya aldığı kanıtı; `ok` artıp bu 0 kalıyorsa bayt modüle girmiyor, `Src/lora.c:316-321`) |
| `MODE: <REAL\|TEST\|SIT\|SUT> LOCK: <OPEN\|LOCKED> REJ=<n>` | `Src/main.c:2566-2572` | Çalışma modu + gerçek-uçuş kilidi + reddedilen SIT/SUT komut sayısı |
| `FLIGHT: <durum> <PROD\|TEST>` | `Src/main.c:2559-2565` | Uçuş durumu + derleme profili |
| `LED: BLUE=<0/1> YEL=<0/1> RED=<0/1>` | `Src/main.c:2546-2557` | Mavi = bench-fall latch, sarı = parmak-düşüş/deploy latch, kırmızı = FAULT (mavi VE sarı birlikte) |
| `MAX: <..> m VEL: <..> m/s` | `Src/main.c:2573-2591` | Maksimum irtifa + düşey hız |
| `APOGEE: DROP=<0/1> VEL=<0/1> n=<n>` | `Src/main.c:2592-2597` | Apogee alt-koşulları + doğrulama sayacı |
| `MAIN: <..> m DEPLOY: D=<0/1> M=<0/1> (<d>/<m>)` | `Src/main.c:2598-2608` | Ana paraşüt irtifası + tek-atımlık deploy latch'leri |
| `SENS: MPU=<OK/FAIL> B1=<..> B2=<..> <OK/DEGRADED>` | `Src/main.c:2610-2616` | Sensör sağlığı |
| `TIM us min/avg/max: ...` (5 sn'de bir, NORMAL mod) | `Src/main.c:2728-2737` | DWT sayaçlı bölüm zamanlaması (A3) |
| `[LINK] seq=<n> t=<ms>ms` + `GPS ...` / `flight=...` (Uno) | `uno_lora_rx/uno_lora_rx.ino:156-213` | Uno alıcıda geçerli çerçeve çözümü |
| `[bekleniyor] good=<n> bozuk=<n> RAW=<n> (...) A5=<n> 00=<n> HI=<n> HDR=<n>` (Uno) | `uno_lora_rx/uno_lora_rx.ino:440-456` | 3 sn geçerli çerçeve yoksa teşhis dökümü |

---

## 1. MİMARİ ŞEMA (ASCII)

```
┌────────────────────── STM32F446RE NUCLEO (UÇUŞ MERKEZİ) ─────────────────────┐
│                                                                                │
│  USART2  PA2/PA3  115200 8N1 ──► ST-LINK VCOM ──► PC (Ek-7 SIT/SUT + TUI)      │
│      TX+RX, RXNE kesmesi + 128 B halka (Ek-7 dinleyici)                         │
│                                                                                │
│  USART6  PC6(TX)/PC7(RX)  9600 8N1 ──► GPS NEO-6M (4 pin)                      │
│      1-bayt IT + 1024 B halka (beyan) / 256 B halka (gerçek, bkz. §3.1 bug)    │
│      TX: UBX pad-konfig (kesmesiz IT, tek kare). RX: NMEA + UBX ACK/NAK.       │
│                                                                                │
│  USART1  PB6(TX)/PB7(RX) AF7  9600 8N1 ──► LoRa E220-900T22D (7 pin)           │
│      Kesme YOK; sınırlı-blokeli TX (50 ms), 500/1000 ms kadans.               │
│      M0=PB10, M1=PA8 (çıkış), AUX=PA9 (giriş). Tek yön: ROKET ──► HAVA.       │
│                                                                                │
│  I2C1  PB8(SCL)/PB9(SDA) ──► MPU6050 (0x68) + BMP280#1 (0x76) + BMP280#2 (0x77) │
│                                                                                │
│  GPIO  LD2/PA5 (yeşil) + MAVİ PA10 + SARI PB5 + KIRMIZI PB4 + B1/PC13 (buton)  │
│                                                                                │
│  = UÇUŞUN TAMAMI BURADA. Tek muhafız, tek karar merkezi.                        │
└───────────────────────────────────┬────────────────────────────────────────────┘
                                    │  900 MHz LoRa, 33 bayt hava / 500 ms
                                    │  (tek yönlü: roket → istasyon, ACK yok)
                                    ▼
┌────────────────────── Arduino UNO (YER İSTASYONU, yalnızca alıcı) ────────────┐
│                                                                                 │
│  D2 (SoftwareSerial RX) ◄── modül TXD        SADECE ALIR                        │
│  D3 (SoftwareSerial TX) ──► modül RXD        SADECE konfig yazma (dirençli)     │
│  D5/D6 (çıkış) ──► M0/M1   D7 (giriş) ◄── AUX                                   │
│  D0/D1 (donanım UART) ──► ekran, 115200     SADECE GÖSTERİR                    │
│                                                                                 │
│  HİÇBİR ŞEY ÜRETMEZ. Sensör yok, karar yok, uçuş yok.                           │
└─────────────────────────────────────────────────────────────────────────────────┘
```

Veri yönleri:

- STM32 → PC (USART2): Ek-7 ikili çerçeveler (SIT/SUT) veya metin TUI (NORMAL). Hız: SIT/SUT 10 Hz
  (`Src/main.c:226` `EK7_TX_PERIOD_MS`), TUI 50 ms periyotlu metin karesi.
- PC → STM32 (USART2): Ek-7 komutları `AA CMD SUM 0D 0A` (`Src/ek7.c:61-131`).
- STM32 → GPS (USART6 TX/PC6): 8 UBX konfig çerçevesi, pad'de, boot'ta (`Src/gps.c:768-817`).
- GPS → STM32 (USART6 RX/PC7): NMEA (GGA+RMC) + UBX ACK/NAK.
- STM32 → hava (USART1): 33 baytlık LoRa hava imajı, 500 ms (uçuş) / 1000 ms (`WAIT_FOR_LAUNCH`).
- Hava → Uno (D2): aynı imaj; Uno magic-senkronla yakalar (`uno_lora_rx/uno_lora_rx.ino:411-434`).

Tasarım sözleşmeleri (`base.md:35-38`):

- Uçuş güvenliği: tek muhafız STM32; yer istasyonu arızası uçuşu etkileyemez.
- GPS uçuşu etkilemez: `gps_has_fix()` sözleşmesi — GPS yoksa uçuş normal devam eder, telemetride `sats=0`.
- LoRa karar katmanına dokunamaz: `lora_poll()` yalnızca okur (`Src/lora.c:708-763`).

---

## 2. PİN TABLOLARI + KRİTİK TUZAKLAR

### 2.1 STM32F446RE NUCLEO pin tablosu

| İşlev | STM32 pini | Nucleo Dx | Yön / kip | Kaynak |
|---|---|---|---|---|
| VCOM TX (Ek-7/TUI) | PA2 (USART2_TX) | — | çıkış | `rocket_avioniks.ioc:58-59`, `Src/main.c:2944-2951` |
| VCOM RX (Ek-7 komut) | PA3 (USART2_RX) | — | giriş, RXNE kesmesi | `rocket_avioniks.ioc:61-64`, `Src/main.c:2959-2961` |
| GPS TX (UBX konfig) | PC6 (USART6_TX, AF8) | — | çıkış | `Src/stm32f4xx_hal_msp.c:193-201`, `Src/main.c:92-94` |
| GPS RX (NMEA) | PC7 (USART6_RX, AF8) | D9 | giriş, 1-bayt IT | `Src/stm32f4xx_hal_msp.c:193-201`, `Src/main.c:92-94` |
| LoRa TX (modül RXD'ye) | PB6 (USART1_TX, AF7) | D10 | çıkış | `Src/stm32f4xx_hal_msp.c:218-226`, `Src/main.c:96-98` |
| LoRa RX (modül TXD'den) | PB7 (USART1_RX, AF7) | D11 | giriş, kesmesiz | `Src/stm32f4xx_hal_msp.c:218-226`, `Src/main.c:96-98` |
| LoRa M0 | PB10 | D6 | GPIO çıkış, varsayılan LOW | `Inc/lora.h:325-326`, `Src/main.c:3078-3089` |
| LoRa M1 | PA8 | D7 | GPIO çıkış, varsayılan LOW | `Inc/lora.h:327-328`, `Src/main.c:3090-3091` |
| LoRa AUX | PA9 | D8 | GPIO giriş (LOW = meşgul) | `Inc/lora.h:329-330`, `Src/main.c:3092-3095` |
| Yeşil LED (LD2) | PA5 | D13 | GPIO çıkış, aktif-yüksek | `Inc/main.h:64-65`, `Src/main.c:3053-3058` |
| Mavi LED (harici) | PA10 | D2 | GPIO çıkış, aktif-yüksek | `Src/main.c:65-66`, `Src/main.c:3069-3073` |
| Sarı LED (harici) | PB5 | D4 | GPIO çıkış, aktif-yüksek | `Src/main.c:67-68`, `Src/main.c:3074-3075` |
| Kırmızı LED (harici) | PB4 | D5 | GPIO çıkış, aktif-yüksek | `Src/main.c:69-70`, `Src/main.c:3076-3077` |
| Mavi buton B1 | PC13 | — | giriş, pull-up, yoklamalı | `Inc/main.h:60-61`, `Src/main.c:3044-3051` |
| I2C1 SCL | PB8 | — | I2C | `rocket_avioniks.ioc:73-75`, `Src/stm32f4xx_hal_msp.c:97-98` |
| I2C1 SDA | PB9 | — | I2C | `rocket_avioniks.ioc:22-23` |
| SWD | PA13/PA14 | — | hata ayıklama | `Src/main.c:62-64` yorumu |

Not: Dx karşılıklarından kodda açıkça yazanlar: BLUE PA10/D2, YELLOW PB5/D4, RED PB4/D5
(`Src/main.c:54-64` yorumu), PB6/D10 + PB7/D11 (`Inc/lora.h:31`), M0 PB10/D6 + M1 PA8/D7 + AUX PA9/D8
(`Inc/lora.h:4-9`), PC7/D9 (`Src/main.c:92-93`).

### 2.2 Arduino Uno pin tablosu (alıcı)

| Modül pini | Uno pini | Yön | Not | Kaynak |
|---|---|---|---|---|
| M0 | D5 | GPIO çıkış | Mode 3 = HIGH/HIGH, Mode 0 = LOW/LOW | `uno_lora_rx/uno_lora_rx.ino:12-13,53` |
| M1 | D6 | GPIO çıkış | (RAW=0 baş şüphelisi: bu hat açıksa modül Mode 0'a dönemez) | `uno_lora_rx/uno_lora_rx.ino:13,54` |
| RXD | D3 | SoftwareSerial TX | SADECE konfig yazma; 1k seri / 2k-1k bölücü önerilir (D3 5V → modül 3.3V giriş) | `uno_lora_rx/uno_lora_rx.ino:15-17` |
| TXD | D2 | SoftwareSerial RX | TEK sinyal hattı | `uno_lora_rx/uno_lora_rx.ino:14,50` |
| AUX | D7 | GPIO giriş | LOW = meşgul; D4 bozuk olduğu için D7 | `uno_lora_rx/uno_lora_rx.ino:18-19,55` |
| VCC | 5V | besleme | TX anında ~110 mA | `base.md:62` |
| GND | GND | — | — | — |
| Ekran | D0/D1 (donanım UART) | 115200 çıkış | Modül D0/D1'e BAĞLANMAZ | `uno_lora_rx/uno_lora_rx.ino:29-34,47-50` |

GPS konfig skeçi (`uno_gps_config`) için: modül TX → D2, modül RX → D3, ekran D0/D1 115200
(`uno_gps_config/uno_gps_config.ino:17-22,31`).

### 2.3 Kritik tuzaklar

1. **CubeMX "Generate Code" USART6/USART1'i siler.** `.ioc` yalnızca USART2 + I2C1 + GPIO içerir
   (`rocket_avioniks.ioc:14` `Mcu.IP4=USART2`; fonksiyon listesinde USART6/USART1 yok,
   `rocket_avioniks.ioc:138`). `MX_USART6_UART_Init` çağrısı (`Src/main.c:952`) ve tanımı
   (`Src/main.c:2979`), `MX_USART1_UART_Init` (`Src/main.c:953`, `Src/main.c:3007`) ve MSP'deki
   USART6/USART1 dalları (`Src/stm32f4xx_hal_msp.c:183-226`) USER CODE blokları dışındadır.
   Generate → üçü de silinir, derleme hatası vermez, GPS/LoRa sessizce ölür. **CubeMX'i açma.**
   (`base.md:98-107`, `GPS_LoRA_OTURUM_NOTU.md:112-121`).
2. **PA9/PA10 USART1 için kullanılamaz.** PA10 = mavi LED (`Src/main.c:65-66`, sürüldüğü yerler
   örn. `Src/main.c:2306-2307`); LoRa USART1'in ikinci AF7 çiftine (PB6/PB7) taşındı
   (`Inc/lora.h:22-35`, `Src/stm32f4xx_hal_msp.c:204-226`).
3. **GPS beslemesi — en sık çuvallayan hata.** NEO-6M `Icc acquisition min 47 mA / tracking min 39 mA`;
   ince kablo+breadboard yetmez → modül açılış banner'ını (`$GPT...`) yarım yazıp reset döngüsüne girer.
   Belirti: `err=0` + `$` var + `NL=0, CR=0` → baud değil **besleme**. Çözüm: 5V + modül pinlerine
   **100 µF + 100 nF** (`base.md:113-118`, `GPS_LoRA_OTURUM_NOTU.md:68-107`).
4. **Uno'da modül D0'a bağlıysa hiçbir şey çalışmaz.** Bootloader D0/D1'i STK500 sync için kullanır:
   `avrdude not in sync`, RESET butonu çalışmaz, `RAW` artar ama hepsi `00`.
   Çözüm: modül D2/D3'te, `SoftwareSerial`, ekran D0/D1 115200 (`base.md:120-129`,
   `uno_lora_rx/uno_lora_rx.ino:29-34`, `uno_gps_config/uno_gps_config.ino:4-15`).
   Geçmiş düzeltmeler: `RX2.begin` eksikliği (begin olmadan `available()` hep 0 → sahte `RAW=0`,
   `uno_lora_rx/uno_lora_rx.ino:235-238`) ve D0→D2 taşıma.
5. **Tek UART'ta iki baud imkânsız.** `Serial.begin(9600)` tüm çıkışı 9600'e düşürür; ekran 115200'de
   kalacaksa GPS ayrı portta olmalı (`base.md:131-134`; Uno GPS test skeçlerinin v1 hatası,
   `uno_gps_test/uno_gps_baudscan/uno_gps_baudscan.ino:3-8`).
6. **LoRa modülü SPI değil.** E220-900T22D: `M0 M1 RXD TXD AUX VCC GND`, SCK/MISO/MOSI yok
   (`base.md:135-137`).
7. **5V MCU ← 3.3V modül TX:** ATmega328P `VIH = 0.6×VCC` → 5V'ta eşik 3.0V, modül 3.3V verir (0.3V marj).
   Çalışmazsa modül TX'ini A0'a alıp `analogRead() > 512` ile oku (`base.md:139-142`).
8. **Yakın mesafe desense:** iki modül aynı masadaysa karışabilir; ≥2 m ayır (pending iş, §8).
9. **PB4 = NJTRST yeniden kullanımı:** proje SWD-only (PA13/PA14) olduğu için PB4 GPIO olarak serbest;
   `.ioc`/`Src`/`Inc` içinde başka PB4 kullanımı yok (`Src/main.c:61-64`).

---

## 3. DOSYA DOSYA İNCELEME

### 3.1 `Src/main.c` (3136 satır — KRİTİK: tüm donanım sahipliği burada)

Amaç: donanım sahipliği (tüm UART handle'ları, GPIO, I2C, kalibrasyon, ana döngü, TUI, Ek-7 modları,
GPS/LoRa istemcileri) tek dosyada. Bakım notu: dosya CubeMX üretimi + el yazımı karışıktır; USART6/USART1
ve tüm uçuş mantığı el yazımıdır.

Ana fonksiyonlar + imzalar:

- `int main(void)` — boot: saat, GPIO, I2C1, USART2/6/1 (`Src/main.c:949-953`), `lora_init` +
  `lora_cfg_boot` + tek satır `LORA CFG:` teşhisi (`Src/main.c:954-1041`), `gps_init`/`gps_cfg_init`
  (`Src/main.c:1042-1043`), `RESET_CAUSE` + IWDG başlatma (`Src/main.c:1046-1049`), RESET bekleme kapısı
  (`Src/main.c:1052-1059`), LED darbesi (`Src/main.c:1063-1068`), kalibrasyon, ana döngü.
- `void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)` (`Src/main.c:393-419`) — USART2 (Ek-7,
  128 B halka) ve USART6 (GPS) üreticisi; her baytta `HAL_UART_Receive_IT` yeniden kurulur.
- `void HAL_UART_TxCpltCallback(...)` (`Src/main.c:340-346`) — USART6 IT gönderimi bitince
  `gps_tx_busy = false`.
- `void HAL_UART_ErrorCallback(...)` (`Src/main.c:384-391`) — USART6 çizgi hatası sayacı + alımı
  yeniden kurma (tek patlama kanalı kilitleyemez).
- `static void MX_USART2_UART_Init(void)` (`Src/main.c:2934-2964`) — 115200 8N1 + RXNE kesmesi.
- `static void MX_USART6_UART_Init(void)` (`Src/main.c:2979-2998`) — `GPS_UART_BAUD` (9600,
  `Src/main.c:2977`) 8N1 TX+RX + 1-bayt IT (`Src/main.c:2995-2997`). El yazımı, `.ioc`'ta yok.
- `static void MX_USART1_UART_Init(void)` (`Src/main.c:3007-3021`) — `LORA_UART_BAUD` (9600,
  `Src/main.c:3005`); hata bilinçli olarak `Error_Handler`'a gitmez (radyo yokluğu fault değil).
- `static void MX_GPIO_Init(void)` (`Src/main.c:3028-3097`) — LD2 + B1 (pull-up) + 3 harici LED +
  LoRa M0/M1 (çıkış, LOW) + AUX (giriş) (`Src/main.c:3078-3095`).
- `static void tui_begin / tui_append / tui_flush / diag_uart_tx` (`Src/main.c:522-562`) —
  kare başına TEK USART2 gönderimi; TUI dışı bağlamda `diag_uart_tx` doğrudan gönderir.
- `static rm_link_t rm_link_now(void)` (`Src/main.c:882`), `ek7mode_t` (`Src/main.c:184-190`:
  `EK7MODE_NORMAL=0, SIT, SUT`).
- `static void iwdg_start / iwdg_refresh` (`Src/main.c:828-848`, bare-metal, HAL modülü yok),
  `static void i2c_link_recover` (`Src/main.c:875`, boot başına maks 3, GPIO yok).

Durum makinesi/adımlar (ana döngü sırası): Ek-7 RX boşaltma → komut aktivasyonu (200 ms,
`Src/main.c:219`) → SUT zaman aşımı 1000 ms (`Src/main.c:213`) → sensör okuma → GPS halka boşaltma
(512 iterasyon üst sınırı, `Src/main.c:1735`) → `gps_cfg_poll` (pad + `!gps_tx_busy`,
`Src/main.c:1783-1802`) → I2C kurtarma → uçuş/filtre/LED → LoRa (`Src/main.c:2354-2397`) →
Ek-7 ikili TX 10 Hz (`Src/main.c:2399-2439`) → metin TUI karesi (`Src/main.c:2451-2722`) →
TIM raporu → IWDG tazeleme (`Src/main.c:2794`).

Sabitler ve anlamları: `EK7_RXRING_N=128` (`Src/main.c:297`), `GPS_RXRING_N=1024` beyan
(`Src/main.c:314`), `GPS_HIST_N=64` (`Src/main.c:364`), `TUI_BUF_N=1400` (`Src/main.c:479`),
`TUI_TX_TMO_MS = 1400*11/115+10` (`Src/main.c:483`), `EK7_RX_DRAIN_MAX=64` (`Src/main.c:212`),
`EK7_SUT_TIMEOUT_MS=1000` (`Src/main.c:213`), `EK7_ACTIVATE_MS=200` (`Src/main.c:219`),
`EK7_TX_PERIOD_MS=100` (`Src/main.c:226`), `SUT_ALT_MIN/MAX=±1000/20000 m`,
`SUT_PRESS=100..1200 mB`, `SUT_ACC_MAX=500 m/s²` (`Src/main.c:228-237`), `GPS_CFG_RETRY_MS=120000`
(`Src/main.c:328`), `KF_Q_DEFAULT=0.05` (`Src/main.c:436`), `GPS_UART_BAUD=9600`
(`Src/main.c:2977`), `LORA_UART_BAUD=9600` (`Src/main.c:3005`), `SENSOR_STALE_MS=500`
(`Src/main.c:293`).

Bağımlılıklar: tüm `Src/*.c` + `Inc/*.h` (flight, ek7, runmode, gps, lora, avionics, bmp280, mahony).

Bilinen bug/zayıflık (bu dosya için okuma sırasında BULUNDU — oturum notundaki düzeltme iddiasıyla çelişir):

- **GPS halkası gerçekte 256 bayt.** `gps_rx_head/tail` `uint16_t` bildirilmiş (`Src/main.c:316-317`)
  ve `Src/main.c:307-313` yorumu "MUST be uint16_t" diyor; ancak üretici `uint8_t nxt =
  (uint8_t)((gps_rx_head + 1u) % GPS_RXRING_N)` (`Src/main.c:407`) ve tüketici `gps_rx_tail =
  (uint8_t)((gps_rx_tail + 1u) % GPS_RXRING_N)` (`Src/main.c:1743`) ile iki güncelleme noktası da
  8 bite daraltıyor. İndeksler 0..255 aralığında sarıyor → halka 1024 değil 256 girdilik çalışıyor
  (960 B/s'te ~0.27 sn, yorumdaki "1.06 sn" yanlış). Tutarlılık bozulmadığı için (iki taraf da aynı
  aralıkta) sistem çalışır, ancak Oturum Notu §2.4'teki "düzeltildi" iddiası eksik kalmış: bildirim
  türü düzeltildi, kesme noktaları düzeltilmedi. Ek-7 halkası (128, `uint8_t`, `Src/main.c:297-300`)
  doğru. Düzeltme: iki satırdaki `(uint8_t)` daraltmasını kaldırmak.
- TUI kablosu süresi hâlâ ~47 ms blokaj (tek `HAL_UART_Transmit`); gerçek çözüm DMA/IT TX
  (`GPS_LoRA_OTURUM_NOTU.md:54-60`).

### 3.2 `Src/lora.c` (762 satır — KRİTİK) + `Inc/lora.h` (393 satır)

Amaç: tek yönlü LoRa telemetri; HAL'siz çekirdek (host'ta birebir test edilir) + hedefe özel
tek sınırlı `HAL_UART_Transmit`. Aynı metin iki kez derlenir: hedefte `LORA_WITH_HAL`
(`CMakeLists.txt:64-70`), host'ta test (`tests/host/test_lora_host.c`).

Ana fonksiyonlar + imzalar (`Src/lora.c` : `Inc/lora.h`):

- `uint8_t lora_crc(const uint8_t *d, size_t n)` (`lora.c:62-74`) — bayt toplamı mod 256 (`ek7_sum`
  ile aynı kural).
- `size_t lora_build(uint8_t *out, size_t cap, const lora_frame_t *in)` (`lora.c:76-98`) —
  30 baytlık çekirdeği AÇIK bayt yazımlarıyla kurar (`put_u8/u16/i16/u32/i32`, `lora.c:20-52`);
  asla struct `memcpy` yok (derleyici dolgusu işlemez). Dönüş `LORA_FRAME_LEN` veya 0.
- `size_t lora_build_air(...)` (`lora.c:100-125`) — `LORA_FIXED_HDR=1` iken 3 baytlık peer başlığı +
  30 baytlık çekirdek = 33 bayt (`Inc/lora.h:203-207`).
- `bool lora_verify / lora_parse` (`lora.c:127-177`) — uzunluk + magic + sürüm + CRC kalıntısı.
- `void lora_frame_from_tlm(...)` (`lora.c:233-256`) — ham telemetri → tel birimleri; uydular her
  zaman canlı geçirilir, konum fix-kapılıdır (fix yoksa 0/0).
- `uint32_t lora_interval_ms(uint16_t flight_state)` (`lora.c:258-275`) — WAIT (2) → 1000 ms, diğer → 500 ms;
  `LORA_SEND_GAP_MS=100` tabanı (aritmetik, gecikme çağrısı yok).
- `bool lora_link_allowed(uint16_t ek7_mode)` (`lora.c:278-286`) — `LORA_ALLOW_SIT_SUT=0` iken
  SIT/SUT'ta gönderim yok.
- `lora_cfg_result_t lora_cfg_boot(void)` (`lora.c:503-654`) — durum makinesi: 8'li baud taraması
  (`9600,115200,57600,38400,19200,4800,2400,1200`, `lora.c:510-512`); her baud'da Mode 3 + yerleşme +
  `C1 00 06` okuma. Eşleşme + 9600 → `OK` (`lora.c:558-564`). Uyumsuzluk → varyantlar (ucuzdan
  pahalıya): v0 `C2 00 06` RAM (2 deneme), v1 `C0 00 06` flash (2 deneme), v2 `C0 00 08` + kript
  `00 00` (2 deneme, 800 ms echo penceresi) (`lora.c:573-611`); ardından Mode 0 çıkışı + Mode 3'e
  yeniden giriş + doğrulama okuması (`lora.c:621-645`, §6.5 kuralı: çalışan konfig Mode 3 ÇIKIŞINDA
  uygulanır). Bağlantı UART'ı hep 9600'e döndürülür (`lora.c:649-650`).
- `void lora_init / bool lora_ready / lora_seq / lora_ok_count / lora_aux_busy / lora_drop_count`
  (`lora.c:673-706`).
- `bool lora_poll(uint32_t now_ms, const lora_tlm_t *t, uint8_t *tx, size_t cap)` (`lora.c:708-763`) —
  döngü başına EN FAZLA bir paket; imzasız-taşma güvenli son tarih; gönderimden ÖNCE sonraki son
  tarih kurulur (yığılma patlaması yok); hedefte sınırlı `HAL_UART_Transmit(..., LORA_TX_TMO_MS=50)`
  (`lora.c:738-739`); gönderim SONRASI AUX örneklenir (LOW = modül baytı aldı, `lora.c:744-747`).
- Yardımcılar: `lora_mode_set` (`lora.c:357-363`), `lora_mode_settle` (100 ms + AUX kaydı, AUX
  ilerlemeyi ASLA kapılamaz, `lora.c:365-373`), `lora_cfg_xfer` (`lora.c:378-416`),
  `lora_cfg_read` (`lora.c:418-443`, `FF FF FF` yanlış-format cevabı dahil),
  `lora_cfg_write_cmd` (`lora.c:449-491`, C0 yerine C0-echo kabulü: datasheet erratum),
  `lora_cfg_write` = C0 flash (`lora.c:493-501`), `lora_cfg_str`, `lora_cfg_diag` (`lora.c:334-352`).

Sabitler ve anlamları (`Inc/lora.h`): `LORA_MAGIC=0xA5`, `LORA_VER=0x01` (`lora.h:143-144`);
ofsetler 0/1/2/4/8/10/12/14/16/18/22/26/28/29 (`lora.h:146-159`); `LORA_FRAME_LEN=30`
(`lora.h:162`); `LORA_ALT_INVALID=LORA_I16_INVALID=0x8000` — gerçek okuma ±32767'ye doyurulur,
sentinel ASLA gerçek örnekten üretilmez (`lora.h:171-174`); flags bit0 fix, bit1 mavi, bit2 sarı,
bit3 kırmızı (`lora.h:180-183`); `LORA_SEND_INTERVAL_MS=500`, `WAIT=1000`, `GAP=100`, `TX_TMO=50`
(`lora.h:186-193`); `LORA_FIXED_HDR=1`, başlık `00 02 17`, `LORA_AIR_LEN=33` (`lora.h:203-207`);
`LORA_ALLOW_SIT_SUT=0` (`lora.h:210`); `LORA_EK7MODE_NORMAL/SIT/SUT = 0/1/2` (`lora.h:215-217`,
`main.c` enum'unun aynası); `LORA_SEND_IN_FAULT=1` (`lora.h:221`); M0=PB10, M1=PA8, AUX=PA9
(`lora.h:325-330`); `LORA_CFG_ADDR=0x00, LEN=6` (`lora.h:339-340`);
`LORA_CFG_EXPECTED = {00,00,62,00,17,00}` (`lora.c:303-305`); `LORA_CFG_TMO_MS=200`,
`LORA_AUX_TMO_MS=300` (`lora.c:354-355`); sonuç enum'u `SKIPPED/OK/FIXED/FAILED`
(`lora.h:344-350`).

Bağımlılıklar: hedefte `main.h` (HAL + `huart1` sahipliği `Src/main.c:100`'da, bildirim
`Src/lora.c:295`'te); uçuş/GPS'e YALNIZCA `lora_tlm_t` üzerinden okuma.

Bilinen bug/zayıflık: C0 flash yazılarda bu modüllerde echo yok → `FIXED` yerine `FAILED` + C2-RAM
geçici link (güç kesilince yeniden FIX gerekir). `ok` sayacı tek başına kanıt değil; `auxbusy`
ile birlikte okunmalı.

### 3.3 `Src/gps.c` (948 satır — KRİTİK) + `Inc/gps.h` (173 satır)

Amaç: HAL-bağımsız NMEA bayt-akış ayrıştırıcı (GGA+RMC) + UBX ACK/NAK yan-durum makinesi + 8 adımlı
UBX pad-konfig zamanlayıcı. Dinamik bellek yok, `strtod` yok (yerel-bağımsız el yazımı sayılar).

Ana fonksiyonlar + imzalar:

- `void gps_init / gps_rx_byte(g,b,now) / gps_is_stale / gps_has_fix` (`gps.c:320-381`,
  `gps.c:647-691`; `gps.h:87-101`). `gps_rx_byte` önce UBX tarafını dener (`gps.c:657`), sonra NMEA
  satır birleştirici (`\n` kapatır, `\r` yoksayılır, taşan satır atılır, `$` yeniden senkronlar).
- Sayı ayrıştırıcılar: `gps_parse_uint` (`gps.c:7-31`, üst sınır 9999999), `gps_parse_double`
  (`gps.c:34-83`), `gps_parse_degmin` (`gps.c:87-167`, ddmm.mmmm → 1e-5 derece, enlem <90/boylam <180,
  dakika <60, hemisfer zorunlu), `gps_parse_time/date` (`gps.c:171-241`), `gps_hexval`
  (`gps.c:243-258`), `gps_split` (`gps.c:264-309`, XOR checksum doğrulamalı yerinde bölme).
- `gps_parse_gga` (`gps.c:384-441`): kalite `0` = fix yok; uydu sayısı ≤99; irtifa -1000..100000 m
  aralığı.
- `gps_parse_rmc` (`gps.c:445-510`): durum `A` gerekli; hız 0..5000 knot → `GPS_KNOTS_TO_MPS`
  (`gps.h:37`); rota 0..360.
- `gps_process_line` (`gps.c:512-546`): konuşmacı-bağımsız (bayt 3..5 GGA/RMC); diğer checksum-geçerli
  cümleler yalnızca `present` işaretler.
- UBX izleyici `gps_ubx_feed` (`gps.c:564-645`): yalnızca satır başında (`n==0`) başlayabilir, NMEA'yı
  bozamaz; `GPS_UBX_PL_MAX=4` üstü çerçeveler atılır; 2 baytlık ACK (`05 01`) / NAK (`05 00`) sayaçları.
- Çerçeve kurucu `ubx_finish` (`gps.c:698-716`, 8-bit Fletcher, sync hariç).
- `gps_ubx_cfg_msg` (`gps.c:718-733`): **6 bayt** — `out[9]` = UART1 hızı (kart USART6 = modül UART1'inden
  okur; "current port" çözümlemesi yok).
- `gps_ubx_cfg_rate` (`gps.c:735-748`): `meas_ms < 1000` reddedilir (gen6 tabanı).
- `gps_ubx_cfg_nav5_dyn` (`gps.c:750-764`): `dyn_model > 7` reddedilir; maske yalnızca dynModel.
- `gps_cfg_frame(step,...)` (`gps.c:768-817`): 0 GGA-aç/1 RMC-aç/2 GLL-kapat/3 GSA-kapat/4 GSV-kapat/
  5 VTG-kapat/6 NAV5/7 RATE; ACK sınıf/id'sini de bildirir.
- `gps_cfg_poll` (`gps.c:835-927`): `allowed=false` → `ABORTED` (pad'den çıktıktan sonra asla TX);
  adım başına 3 deneme, 500 ms ACK zaman aşımı, 100 ms adım aralığı; beklenmedik ACK cevabı beklemeye
  devam eder (`gps.c:892`); hazır-ms sarma-güvenli (`gps.c:912`); terminal durumlar OK/NAK/NOACK/ABORTED.
- `gps_cfg_str` (`gps.c:929-948`).

Sabitler: `GPS_LINE_MAX=128`, `GPS_STALE_MS=3000`, `GPS_UBX_PL_MAX=4`, `GPS_CFG_RATE_MS=1000`,
`GPS_CFG_DYNMODEL=7`, `GPS_CFG_STEPS=8`, `GPS_CFG_ACK_TMO_MS=500`, `GPS_CFG_TRIES=3`,
`GPS_CFG_GAP_MS=100`, `GPS_UBX_MAX_FRAME=44` (hepsi `Inc/gps.h:33-122`).

Bağımlılıklar: yok (saf C). Çağıran: `main.c` (halka boşaltma + `gps_cfg_poll` + `gps_tx_buf`).

Bilinen bug/zayıflık: CR-only akışta ayrıştırıcı satır kapatamaz (`\n` anahtarlı, `gps.c:661`);
CR sayacı (`main.c:375-380`) bu teşhis içindir. Geçmiş düzeltmeler: 6-bayt CFG-MSG, dynModel 7,
1000 ms, `uint16_t` halka indeksleri (bildirim; kesme noktaları §3.1'deki gibi eksik).

### 3.4 `Src/flight.c` (682 satır — KRİTİK) + `Inc/flight.h` (330 satır)

Amaç: HAL-bağımsız deterministik uçuş yaşam döngüsü. Üretim/test **parametre** ayrımıdır, ikinci
algoritma değil: PROD apogee-alt-min 1250 m + main 600 m; TEST 0 m + override edilebilir main
(`flight.c:6-49`, `flight.h:36-42`).

Ana fonksiyonlar + imzalar:

- `flight_config_production / flight_config_test(main_alt)` (`flight.c:6-49`); bench tetik her iki
  config'de de `bench_enable=true` (yalnızca display+LED olduğu için el testi üretimde de çalışır,
  `flight.c:17-20`).
- `flight_init / flight_set_state` (`flight.c:51-112`); FAULT yapışkan (104-112).
- `flight_velocity` (`flight.c:114-144`): dt [5,500] ms kelepçeli, sonlu-olmayan girdi reddi.
- `flight_launch_push` (`flight.c:146-169`, `|a|>1.2·|gref|`, N=3), `flight_freefall_push`
  (`flight.c:171-194`, `|a|<0.8·|gref|`, N=3).
- `flight_vert_dyn_g` (`flight.c:196-229`): yerçekimi-referanslı izdüşüm, `dyn≈0` durgun,
  aşağı-hızlanmada NEGATİF; sabit-Z varsayımı yok.
- `flight_bench_push` (`flight.c:231-263`): `dyn < -0.30g` VE `mag < 0.95`, N=3, latch; üretimde
  `bench_enable` kapısı (not: config'ler `true` bıraktığı için pratikte her modda izler).
- `flight_bench_baro_push` (`flight.c:265-340`): gerçek fused irtifadan hız + kendi max'ı + hareket-
  canlılık kapısı (`bench_motion_ok`); hız eşiği `-0.1 m/s`, N=3, düşüş bayrağı 0.3 m.
- `flight_bench_fall` (`flight.c:342-353`): `ACCEL OR BARO` — display+LED+UART, state/deployment'a
  ASLA girmez.
- `flight_combine_pressure` (`flight.c:355-385`), `flight_alt_push` (`flight.c:387-433`,
  max takibi + hız + drop/vel bayrakları), `flight_try_drogue/main` (`flight.c:435-481`,
  tek-atımlık + durum + sensör interlock), `flight_apogee_ready` (`flight.c:483-492`,
  max+drop+vel+min-irtifa, N=3), `flight_step` (`flight.c:494-612`, tek-yön latch yaşam döngüsü +
  50-örnek geçersiz-watchdog → FAULT), `flight_health_snapshot` (`flight.c:614-651`),
  `flight_state_str` (`flight.c:653-682`), `flight_motion_seen` (`flight.h:160-170`).

Sabitler: `APOGEE_ALT_MIN_PROD=1250`, `MAIN_PROD=600`, `APOGEE_DROP=2 m`,
`DESCENT_VEL=-2 m/s`, `LAUNCH=1.20×/N=3`, `FREEFALL=0.80×/N=3`, `FAULT_INVALID_N=50`,
`BENCH_NEG_THR=-0.30g`, `BENCH_CONFIRM_N=3`, `MAG_RATIO=0.95`, `BARO_VEL=-0.1/N=3/DROP=0.3`,
`MOTION_DYN/MAG=0.10`, `MOTION_TMO=2000 ms` (hepsi `Inc/flight.h:36-155`). Durumlar 11 adet
(`flight.h:173-186`): INIT→CALIBRATION→WAIT_FOR_LAUNCH→ASCENT→APOGEE_CONFIRMED→FREE_FALL→
DROGUE_DEPLOYED→DESCENT→MAIN_DEPLOYED→COMPLETE + FAULT.

Bağımlılıklar: `avionics.h` (`avt_streak_push` bench sayaçlarında).

### 3.5 `Src/ek7.c` (297 satır — KRİTİK) + `Inc/ek7.h` (135 satır)

Amaç: Ek-7 resmi protokolü (resmi Ö-UKB yazılımıyla bayt-uyumlu; doküman değiştirilmedi,
`base.md:90-92`).

- `ek7_rx_feed` (`ek7.c:61-131`): 5 bayt `AA CMD SUM 0D 0A`; checksum İKİ kuraldan birini tutmalı —
  resmi toplama (`SUM=(AA+CMD)&FF` → SIT CA/SUT CC/STOP CE) veya doküman-literal (8C/8E/90);
  çerçeve-içi header yeniden senkronlar.
- `ek7_sum` (`ek7.c:133-145`), `ek7_pack_f32be/unpack` (`ek7.c:147-175`, 2-ondalık yuvarlamalı BE float).
- `ek7_build_tlm` (`ek7.c:177-196`): `0xAB + 8×float + SUM(header+32B) + 0D 0A` = 36 B.
- `ek7_parse_tlm` (`ek7.c:198-228`): header/checksum/footer + sonlu-float zorunluluğu.
- `ek7_build_status` (`ek7.c:230-244`): `AA D1 D2 SUM 0D 0A` = 6 B (önce D1/düşük bayt).
- `ek7_status_word` (`ek7.c:7-49`): bit0 kalkış (uçtuysa), bit1 hep 0 (D5), bit2 min-irtifa,
  bit3 açı, bit4 alçalma, bit5 drogue, bit6 ana-altı, bit7 main; FAULT hiçbir uçuş biti taşımaz.
- `ek7_map_acc` (`ek7.c:246-266`, `+ax,+ay,-az`, Z-aşağı, self-calibrating `|gref|`),
  `ek7_map_ang` (`ek7.c:268-297`, yaw=0).
- Sabitler (`Inc/ek7.h:42-75`): header'lar AA/AB/0D/0A, komutlar 20/22/24, legacy sum 8C/8E/90,
  uzunluklar 5/36/6, durum bitleri, `EK7_MPU_LSB_PER_G=16384`, `EK7_MS2_PER_G=9.80665`.
- Belgelenmiş sapmalar D1-D5 (`Inc/ek7.h:16-27`, `SIT_SUT_REQUIREMENTS.md:30-47`).

### 3.6 `Src/avionics.c` (847 satır) + `Inc/avionics.h` (402 satır)

Amaç: doğrulanmış tezgâh matematiği (flight katmanı bunun ÜSTÜNE gerçek uçuş yaşam döngüsünü ekler,
`Inc/flight.h:1-16`). Fonksiyonlar (`Src/avionics.c` satırlarıyla):

- `avt_tilt_deg` (8), `avt_tilt_mag_ok` (26), `avt_in_band` (39), `avt_vec_mag` (44) — tilt bandı
  30..45° (`avionics.h:16-21`), mag toleransı %25.
- `avt_altitude_m` (49) — barometrik bağıntı, P0'a göre.
- `avt_bmp_state` (61) — iki-sensör fazlalık durumu; `AVT_BMP_DISAGREE_PA=50 Pa` (`avionics.h:36`).
- `avt_is_degraded / avt_alt_usable` (79/84), `avt_alt_rise` (89, +0.20/+0.10 histerezis),
  `avt_alt_drop` (106, -0.15/-0.07).
- `avt_streak_push` (130) — tüm N-ardışık tetiklerin ortak sayacı (doyumlu 255).
- `avt_drop_gate_m / avt_alt_drop_n` (139/158) — gürültü-bağıl parmak göstergesi
  (`DROP_K=4, MIN=0.10, MAX=0.50`, `avionics.h:214-217`).
- `avt_ema` (177, α=0.5), `avt_p0_correct` (182, kazanç 0.5, kapak 5 Pa, ölübant 0.5 Pa),
  `avt_med3_init/push` (222/235, 3-örnek medyan ön-filtre).
- `avt_apogee_init/set_noise/update` (274/289/314) — trend apogee (ARM 0.5 m, DROP 0.3 m, N=3,
  gürültü ölçekli eşikler, `avionics.h:107-132`).
- `avt_replay_init/put` (440/572) — UART basınç tekrarı, TEST/DEBUG ONLY, varsayılan KAPALI.
- `avt_kf_init/set_noise/predict/update` (615/635/651/701) — paralel Kalman irtifa, SADECE diagnostik.
- `avt_mpu_cal_identity/apply/from_6pos/residual_ok` (754/768/798/831) — 6-poz kalibrasyon altyapısı
  (identity varsayılan → üretim davranışı bit-aynı; el prosedürü NOT EXECUTED).
- Kalibrasyon penceresi `AVT_CAL_MS=16000` (`avionics.h:198`).

### 3.7 `Src/runmode.c` (83 satır — küçük) + `Inc/runmode.h` (79 satır)

Amaç: REAL/TEST/SIT/SUT ayrımı + gerçek-uçuş kilidi. `runmode_update` (`runmode.c:14-29`):
SUT-linkte kilitlenmez (sentetik uçuş gerçek roket hakkında bir şey söylemez); pad-dışı + max ≥
`RUNMODE_LOCK_ALT_M=30 m` (`runmode.h:38`) ise latch (yalnızca reset açar). `runmode_accept`
(`runmode.c:31-51`): STOP hep kabul, SIT/SUT kilitte reddedilir + sayılır, NULL → fail-closed.
`runmode_current/str` (`runmode.c:53-83`). El sallama (2-3 m) kilidi tetikleyemez — bilinçli tasarım
(`Inc/runmode.h:14-20`).

### 3.8 `uno_lora_rx/uno_lora_rx.ino` (485 satır — KRİTİK)

Amaç: Uno yer istasyonu alıcısı. `SoftwareSerial RX2(2,3)` (`:50`); mod pinleri M0=D5, M1=D6, AUX=D7
(`:53-55`); beklenen imaj `{00,00,62,00,17,00}` (`:60`); çerçeve sabitleri (`:62-67`).

- `setup` (`:216-244`): `Serial.begin(115200)` (ekran hep 115200), D2 boş-seviye kontrolü (`:228-233`,
  LOW = kısa devre/yanlış delik), **`RX2.begin(9600)` (`:238`)** — geçmiş bug: begin olmadan
  `available()` hep 0 → sahte RAW=0; sonra `lora_cfg_boot()` (`:240`).
- `lora_cfg_boot` (`:316-397`): Mode 3 → `aux_wait` → `C1 00 06` oku → eşleşirse Mode 0 + `LORA CFG: OK`;
  farklıysa `C2` yaz (RAM!) → Mode 0 (uygula) → Mode 3 (doğrula) → `FIXED`/`FAILED` + echo dökümü.
  `cfg_write` **C2** kullanır (`:287-312`, "C0 flash yazar; düşük beslemede modül resetlenip echo
  vermeyebiliyor").
- `handle_frame` (`:132-214`): magic+sürüm + **CRC = `sum(0..28) == buf[29]`** (`:141`); yorumda açıkça
  yazar: "30 baytın toplamı == 0" kontrolü YANLIŞTIR, geçerli paketlerin ~%99'unu reddeder (`:135-140`).
  Çözüm ekranı: `[LINK]`, `GPS sats/fix/pos`, `flight/ek7mode/alt/vel/sats/batt/fix/LED/pos`.
- `loop` (`:399-486`): magic-senkron biriktirme + 3 sn çerçevesizlikte teşhis (`good/bozuk/RAW/B-s
  hızı/A5/00/HI/HDR` + son 32 bayt hex); `RAW=0` → kablo/besleme/mode şüphesi (`:459-466`).
- Doküman eskimesi: `:40` yorumu irtifa alanını `alt_cm` diye anar; kod metre okur (`:180-182`
  `put("m")`) — yorum güncellenmeli.

### 3.9 `uno_gps_config/uno_gps_config.ino` (225 satır — KRİTİK)

Amaç: NEO-6M UBX konfigürasyonu (gen6), GPS D2/D3'te. D2/D3 gerekçesi başlıkta (`:4-15`).
`SoftwareSerial GPS(2,3)`, 9600 (`:31-32`); döngü 2.5 sn (`:33`); `ubx_finish` Fletcher (`:36-46`);
`b_msg` 6-bayt CFG-MSG, hız `o[9]`=UART1 (`:51-57`); `b_nav5` dyn parametreli (`:61-68`);
`b_rate` (`:72-79`); `wait_ack` ACK/NAK ayrımı (`:92-120`); `run_config` 8 adım
(GGA/RMC aç + GLL/GSA/GSV/VTG kapat + NAV5 dyn7 + RATE 1000, `:133-181`, ham hex dökümlü);
`setup` ekranı 115200'de sabit tutar (`:185`); `loop` D2 ham bayt sayar + teşhis basar (`:197-226`).

### 3.10 Küçük/trivial dosyalar (3-8 satır özet kuralı)

- `Src/bmp280.c` (135 satır) + `Inc/bmp280.h` (~67): Bosch tamsayı kompanzasyonu (sıcaklık 32-bit,
  basınç 64-bit, Q24.8→Pa yuvarlamalı); `BMP280_CTRL_MEAS_DEMO=0x2F` (osrs_t x1/osrs_p x4/normal),
  `CONFIG_DEMO=0x04` (filtre katsayısı 2); tüm I2C zaman aşımlı, `HAL_MAX_DELAY` yok.
- `Src/mahony.c` (197 satır) + `Inc/mahony.h` (63): Mahony 6-DOF tamamlayıcı filtre (IMU-only);
  `Kp=2.0/Ki=0.01/int-kapak 0.1`, ivme-uyarlamalı kazanç (1.0/0.25/kapalı), `DT_MAX=0.1`;
  SADECE diagnostik, tilt/LED'i sürmez.
- `Src/stm32f4xx_hal_msp.c` (263 satır): USART2 PA2/PA3 AF7 (`:175` civarı); USART6 PC6/PC7 AF8
  (`:183-201`); USART1 PB6/PB7 AF7 (`:204-226`); I2C1 PB8/PB9 (`:90-98`). CubeMX tuzağının üçüncü
  ayağı (USER CODE dışı).
- `Src/stm32f4xx_it.c` (231 satır) + `Inc/stm32f4xx_it.h` (66): USART2/USART6 IRQ yönlendiriciler
  (`:217-227`); USART2 DR yoklamalı erken-önyükleme baskısı (`:95-99`).
- `Src/syscalls.c` (244) / `Src/sysmem.c` (87) / `Src/system_stm32f4xx.c` (747): CubeMX/std üretimi;
  uçuş mantığı yok.
- `Inc/main.h` (81): B1 PC13, USART_RX PA3, LD2 PA5, TMS/TCK/SWO (`:60-71`); CubeMX üretimi.
- `Inc/stm32f4xx_hal_conf.h` (495): HAL modül anahtarları; el değmedi.
- `uno_lora_getconfig/uno_lora_getconfig.ino` (70): Mischianti/Alteriom `LoRa_E220` ile register +
  modül-bilgi dökümü; kablolama `LoRa_E220(2,3,7,5,6)` = RX,TX,AUX,M0,M1 (`:11`); ekran 9600 (`:17`).
  Teşhis amaçlı, uçuş dışı.
- `uno_gps_test/uno_gps_test.ino` (285): kütüphanesiz NEO-6M ham teşhis; GPS TX → **D0** (donanım UART),
  ekran 115200; `[RATE]/[+]/[-]` kılavuzu (`:22-27`). D0 tuzağı bilerek göze alınmış eski araç.
- `uno_gps_test/uno_gps_baudscan/*` (186): v1 hatası (tarama Serial baud'unu bozuyordu), v3 D5/D4'e
  geçiş.
- `uno_gps_test/uno_gps_deep/*` (186): BOOT=micros kanıtı + D0+D4 çift-dinleme + idle-seviye + loopback.
- `uno_gps_test/uno_gps_diag/*` (144): say-sonra-bas deseni (tarama bitince 115200'e dön).
- `uno_gps_test/uno_gps_ledverdict/*` (104): CH340 güvensizliği → LED'e yazan teşhis (USB-bağımsız).
- `uno_gps_test/uno_gps_power/*` (131): A0 üzerinden modül VCC ölçümü (IDLE D0=1 + rx=0 ayrımı).
- `uno_gps_test/uno_gps_scan_d0/*` (148): v4 hatası (`Serial.end()` AVR'de RX'i kapatır), v5 global
  dizi + sonda tablo.
- `uno_gps_test/dinle_reset.py` (59) / `tek_port_test.py` (84): CH340 auto-reset güvensizliğine karşı
  tek-port-açık-tut okuma yardımcıları.
- `CMakeLists.txt` (90): C11, `Src/{bmp280,avionics,flight,ek7,gps,lora,runmode,mahony}.c` (`:48-56`),
  `LORA_WITH_HAL` (`:64-70`), `FLIGHT_TEST_MODE` opsiyonu (`:79-83`, `build/Bench` için
  `-DFLIGHT_TEST_MODE=1`).
- `CMakePresets.json` (37): `default` (gizli, Ninja + arm toolchain) → `Debug`/`Release`; **Bench
  preset'i YOK** (`build/Bench` ayrı dizindir).
- `cmake/gcc-arm-none-eabi.cmake` (47) / `cmake/starm-clang.cmake` (65) /
  `cmake/stm32cubemx/CMakeLists.txt` (91): toolchain + CubeMX kütüphane derlemesi.
- `pinsketch.md` (23): el yazısı LoRa pin sırası + Uno notları; üst seviye doküman değil, eskiz.
- `precommands.md` (4): `screen` VCOM komutu + `python tests/sit_runner.py --live`.
- `base.md` (220): KALICI mimari dokümanı (bölüm özetleri §2 tuzaklarda kullanıldı).
- `GPS_LoRA_OTURUM_NOTU.md` (190): 2026-10-06 oturum dersleri (6 gerçek hata + brownout + link).
- `SIT_SUT_REQUIREMENTS.md` (191): Ek-7 resmi gereksinim + davranış envanteri + el-testi kanıtı.
- `E220-900T22D_UserManual_EN_v1.2.pdf/.txt` (2149 satır txt): register/mod/AUX kaynağı (bkz. §6).
- `roketpdf/` (14 docx): TEKNOFEST resmi Ek dokümanları; resmi Ek-7 = `Ek-7_..._DtJW1.docx`.
- `startup_stm32f446xx.s` (538), `STM32F446xx_FLASH.ld` (250): standartvektör/link betiği.
- `UKB Taklit Yazılımı/` + `.zip`: resmi Ö-UKB test yazılımı taklidi (SIT/SUT karşı tarafı); venv
  dahil, bu dosyanın kapsamı dışında tutuldu.

---

## 4. LoRa PROTOKOL BÖLÜMÜ

### 4.1 30 baytlık çekirdek düzeni (little-endian, `Inc/lora.h:47-65`)

| Ofset | Uzun | Alan | Tür | Anlam |
|---|---|---|---|---|
| 0 | 1 | magic | u8 | `0xA5` sabit çerçeve başı |
| 1 | 1 | ver | u8 | `0x01` protokol sürümü |
| 2 | 2 | seq | u16 LE | paket sayacı, +1/paket, sarar |
| 4 | 4 | t_ms | u32 LE | kurulum anında `HAL_GetTick()` |
| 8 | 2 | flight_state | u16 LE | `flight_state_t` değeri |
| 10 | 2 | ek7_mode | u16 LE | 0 NORMAL / 1 SIT / 2 SUT |
| 12 | 2 | alt_m | i16 LE | irtifa, **TAM METRE**; geçersiz = `0x8000` |
| 14 | 2 | vel_cms | i16 LE | düşey hız, cm/s; geçersiz = `0x8000` |
| 16 | 2 | sats | u16 LE | kullanılan uydu, HER ZAMAN canlı (fix şart değil); 0xFFFF rezerve |
| 18 | 4 | lat_e5 | i32 LE | enlem ×1e-5; 0 = geçersiz |
| 22 | 4 | lon_e5 | i32 LE | boylam ×1e-5; 0 = geçersiz |
| 26 | 2 | batt_mv | u16 LE | batarya mV; 0 = bilinmiyor (bu derlemede ADC yok, hep 0, `Src/main.c:2376`) |
| 28 | 1 | flags | u8 | bit0 gps_fix, bit1 mavi, bit2 sarı, bit3 kırmızı |
| 29 | 1 | crc | u8 | `sum(bayt[0..28]) mod 256` |

### 4.2 33 baytlık hava imajı

`LORA_FIXED_HDR=1` iken (`Inc/lora.h:203-207`) çekirdeğin önüne peer başlığı gelir:

| Hava ofseti | Değer | Anlam |
|---|---|---|
| 0 | `0x00` | peer ADDH |
| 1 | `0x02` | peer ADDL (adres 0x0002) |
| 2 | `0x17` | peer CH (kanal 0x17) |
| 3..32 | 30 baytlık çekirdek | CRC dahil aynen |

Alıcı magic-senkronu 3 başlık baytını otomatik atlar (`tests/host/test_lora_host.c:18-25`
`core_of`). Peer modüller fixed-TX modunda adres 0002 kanal 0x17'dedir.

### 4.3 CRC kuralı

- Yazma: `crc = sum(0..28) mod 256` → bayt 29 (`Src/lora.c:95-96`).
- Doğrulama: `lora_crc(f,29) == f[29]`, yani kalıntı sıfır (`Src/lora.c:137-138`); `lora_verify`
  uzunluk+magic+sürüm+CRC'yi birlikte kontrol eder (`Src/lora.c:127-139`).
- Uno tarafı aynı kuralı uygular (`uno_lora_rx/uno_lora_rx.ino:135-141`) ve "30 bayt toplamı == 0"
  kısayolunun YANLIŞ olduğunu belgeler (geçerli paketlerin ~%99'unu reddeder).
- Altın vektör: `seq=0x1234, t=0x11223344, ...` → CRC `0x3F` (`tests/host/test_lora_host.c:102-131`
  elle-türetilmiş tablo).

### 4.4 Kadans

- Normal uçuş: 500 ms; `WAIT_FOR_LAUNCH`: 1000 ms (`Inc/lora.h:186-187`, `Src/lora.c:258-275`).
- 9600 8N1'de 30 bayt = 31.25 ms kablo süresi; 500 ms kadans ≈16× hava süresi, 400 baytlık modül
  tamponu asla dolmaz (`Inc/lora.h:87-98`).
- Döngü başına EN FAZLA bir paket (`Src/lora.c:100-101` yorumu, `708-763`); gönderimde gecikme çağrısı
  yok; tek HAL TX üst sınırı 50 ms (`Inc/lora.h:193`).
- FAULT'ta da gönderim sürer (`LORA_SEND_IN_FAULT=1`, `Inc/lora.h:221`) — fault paketi en değerli pakettir.

### 4.5 SIT/SUT kilidi

- `LORA_ALLOW_SIT_SUT=0` (`Inc/lora.h:210`): ek7_mode SIT/SUT iken `lora_poll` sessizce göndermez
  (`Src/lora.c:278-286`, `714-717`); Ek-7 ikili çerçeveler USART2'yi bozmaz.
- Bench derlemesinde LoRa telemetrisi istenirse bilinçli olarak 1 yapılır.

### 4.6 Uçuş-güvenlik kuralları

- `lora_poll` uçuş durum makinesine HİÇBİR ŞEY yazmaz, konuşlandırma/LED işleyicisi çağırmaz
  (`Inc/lora.h:109-117`, `Src/main.c:2354-2358`).
- Kesme kapatmaz, aktif-beklemez, `Error_Handler`'a girmez (`Inc/lora.h:118`).
- LoRa yokluğu fault değildir: `lora_init` bağlantıyı kapalı bırakır, `main.c` tek satır basar
  (`Inc/lora.h:119-120`, `Src/main.c:954-958`).
- Ölçülen değerler: uçuşun o iterasyondaki fused-P0 irtifası + `flight.vel_mps` + GPS + LED latch'leri
  (`Src/main.c:2359-2396`).

---

## 5. GPS UBX BÖLÜMÜ

### 5.1 8 adım tablosu (`Src/gps.c:768-817`, sabitler `Inc/gps.h:116-123`)

| # | Çerçeve | İçerik | ACK sınıf/id |
|---|---|---|---|
| 0 | CFG-MSG | `F0 00` (GGA) hız **1** | 06/01 |
| 1 | CFG-MSG | `F0 04` (RMC) hız **1** | 06/01 |
| 2 | CFG-MSG | `F0 01` (GLL) hız **0** | 06/01 |
| 3 | CFG-MSG | `F0 02` (GSA) hız **0** | 06/01 |
| 4 | CFG-MSG | `F0 03` (GSV) hız **0** | 06/01 |
| 5 | CFG-MSG | `F0 05` (VTG) hız **0** | 06/01 |
| 6 | CFG-NAV5 | dynModel **7** (maske yalnızca dynModel) | 06/24 |
| 7 | CFG-RATE | **1000 ms** (navRate 1, timeRef GPS) | 06/08 |

Zamanlayıcı: adım başına 3 deneme, 500 ms ACK zaman aşımı, 100 ms adım aralığı
(`Inc/gps.h:119-121`); hız SONDA (önceki ACK'ler eski hızda gelsin, `Inc/gps.h:110-114` yorumu);
ayarlar modül flash'ına yazılmaz, her boot yeniden uygulanır (durumsuz); pad-dışı `ABORTED`
(`Src/gps.c:846-851`); 120 sn'de bir pad'de sessiz yeniden-deneme (`Src/main.c:328`).
Hedef TUI: `GPSCFG: OK 8/8` (`Src/main.c:2703-2709`). Orijinal u-blox kanıtı: klonlar ACK vermez
(`base.md:208`).

### 5.2 gen6'ya özgü noktalar

- **CFG-MSG 6 bayt** (`msgClass,msgID,rateDDC,rateUART1,rateUART2,rateUSB`; hız `out[9]`=UART1,
  `Src/gps.c:718-733`). 3-bayt form hızı DDC slotuna düşürür, UART1'e dokunmaz ama ACK yine gelir →
  hata gizlenir (`GPS_LoRA_OTURUM_NOTU.md:22-33`).
- **dynModel 7** = Airborne<4g; alan 3-bit (0..7). 8/9/10 gen7+ numaralandırmasıdır; 8 yazmak
  rezerve bit kurar → modül 0'a (Portable) maskeler, tam tersi (`GPS_LoRA_OTURUM_NOTU.md:35-39`,
  `Src/gps.c:750-764` `dyn>7` reddi).
- **measRate 1000 ms** = 1 Hz; NEO-6 1 Hz parçadır, gen6 tabanı 1000 ms (`Src/gps.c:735-748`
  `<1000` reddi). GGA+RMC ~170 B/s, 960 B/s hatta rahat sığar.
- Uno tarafı aynı üç kuralı bağımsız uygular (`uno_gps_config/uno_gps_config.ino:48-79`).

---

## 6. E220 MOD/KONFİG BÖLÜMÜ

### 6.1 Mode tablosu (M1,M0) — datasheet §6 (`E220-900T22D_UserManual_EN_v1.2.txt:665-680` satırları civarı)

| M1 | M0 | Mode | Açıklama |
|---|---|---|---|
| 0 | 0 | Mode 0 Normal | UART + kablosuz açık, şeffaf iletim — ÇALIŞMA modu |
| 0 | 1 | Mode 1 WOR verici | WOR Transmitter |
| 1 | 0 | Mode 2 WOR alıcı | havadan-uyandırmalı alıcı |
| 1 | 1 | Mode 3 Derin uyku | KONFİGÜRASYON modu (yalnızca 9600 8N1) |

Üretimde yalnızca Mode 0 ↔ Mode 3 (boot) kullanılır. Mode değişimi AUX=1 iken geçerlidir;
AUX yükselen-kenarından sonra 2 ms beklenir (Tablo 5-1, `E220...txt: Table 5-1 Notes for AUX`;
kodda 100 ms yerleşme + AUX kaydı, `Src/lora.c:365-373`; Uno'da `aux_wait`, `uno_lora_rx:247-255`).

### 6.2 AUX kuralları

- LOW = meşgul/öz-kontrol; tampon dolu (400 bayt dahili) veya iletim sürüyor
  (`E220...txt: 5.6 AUX Description`, `5.6.2`).
- Öncelik düşük-seviyededir; Mode 3'ten çıkışta/parametre resetlenirken AUX düşer (Tablo 5-1).
- Kod AUX'u asla kapı yapmaz (bağlı olmayan AUX hep düşük okur ve konfigi kilitlememelidir,
  `Src/lora.c:365-373`); yalnızca teşhis için örnekler (`s_aux_exit`, `s_aux_busy`).

### 6.3 C0/C1/C2 formatları (Tablo 7-1)

| Komut | Çerçeve | Cevap | Etki |
|---|---|---|---|
| C1 + adres + uzunluk | örn. `C1 00 06` | `C1 + adres + uzunluk + parametreler` | okuma |
| C0 + adres + uzunluk + parametreler | örn. `C0 00 03 12 34 62` | `C1 + ...` echo | flash'a yaz (kalıcı) |
| C2 + adres + uzunluk + parametreler | örn. `C2 00 03 12 34 62` | `C1 + ...` echo | RAM'e yaz (geçici, güçte uçar) |
| yanlış format | — | `FF FF FF` | reddedildi |

Konfig modu yalnızca 9600 8N1'dedir. Bazı modüller C1 yerine C0 echo verir (datasheet erratum,
sahada doğrulandı) — kod ikisini de kabul eder ama adres+uzunluk tutmalı (`Src/lora.c:474-482`).

### 6.4 06H/07H/08H notları

- 06H `CRYPT_H` + 07H `CRYPT_L`: **yalnızca-yazılır**, okuma 0 döner; hava şifreleme katsayısı
  (varsayılan 0) (`E220...txt: 06H/07H CRYPT` bölümü).
- v2 yazma varyantı `C0 00 08` + kript `00 00` ile tam-aralık yazar (`Src/lora.c:573-576,601-608`).
- 08H: bu modül ailesinde ek aralık/rezerve alanı; beklenti imajı 00H..05H'i kapsar.

### 6.5 Tablo 6-3 yazım hatası notu

Datasheet §6.3 "Table 6-3 WOR sending mode" başlığı altında "When M0 = 1, M1 = 1, the module works
in mode 1" yazar (`E220...txt: §6.3 civarı`) — bu, §6 mode tablosuyla çelişir (M1=1,M0=1 = Mode 3).
Doğrusu Mode 1 = M1=0,M0=1'dir (tablonun WOR-Transmitter satırı). Metin başlığındaki M0/M1 yer
değiştirmiş; tablo esas alınır. Kod mode'ları sembolik sürer (`lora_mode_set(1,1)` = Mode 3,
`Src/lora.c:357-363`; Uno `mode_set(1,1)`, `uno_lora_rx:257-261`), metne değil.

### 6.6 Fabrika imajı

`c0 00 00 62 00 17` (Tablo 7-3): adres 0x0000, kanal 0x17, 2.4 kbps hava, 9600 8N1, 22 dBm
(`E220...txt: 7.3 Factory Default Parameter`).

### 6.7 Bizim EXPECTED imaj + nedenleri

`LORA_CFG_EXPECTED = {00,00,62,00,17,00}` (`Src/lora.c:303-305`):

| Bayt | Değer | Açılım | Neden |
|---|---|---|---|
| ADDH/ADDL | 00 00 | adres 0x0000 | şeffaf modda filtre yok; fabrika ile aynı → sağlam modül yazılmaz |
| REG0 | 0x62 | 9600 8N1 + 2.4k hava | bitler: UART `011`=9600, parite `00`=8N1, hava `010`=2.4k (Tablo 7-2) |
| REG1 | 0x00 | 200 B alt-paket, RSSI kapalı, 22 dBm | 30 B çerçeve tek alt-pakete sığar; RSSI baytı çerçeveyi bozmaz |
| REG2 | 0x17 | CH 23 → 850.125+23 = **873.125 MHz** | fabrika varsayılanı |
| REG3 | 0x00 | şeffaf iletim, RSSI baytı yok, LBT yok | header'sız 30 B çalışır; fixed-TX peer'e 3 baytlık başlıkla ulaşılır |

---

## 7. DOĞRULAMA BÖLÜMÜ

### 7.1 Derleme / flash / yükleme komutları

```bash
# Üretim derlemesi (Debug = üretim)
cmake --preset Debug && cmake --build --preset Debug
# NOT: 'Bench' preset'i CMakePresets.json'da YOK (yalnızca Debug/Release,
# doğrulanmış CMakePresets.json:13-27); bench ayrı dizindir:
cmake -B build/Bench -DCMAKE_BUILD_TYPE=Debug -DFLIGHT_TEST_MODE=1 && cmake --build build/Bench

# Flash
arm-none-eabi-objcopy -O binary build/Debug/rocket_avioniks.elf /tmp/prod.bin
st-flash --reset write /tmp/prod.bin 0x8000000

# Canlı TUI
stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0
```
(Komutlar `base.md:172-184`'ten; preset yokluğu `oturum notu §6` dersiyle çapraz doğrulandı.)

### 7.2 Host testleri — bu dosya için yeniden koşuldu (tümü PASS, sıfır uyarı)

```bash
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_flight_host.c   Src/flight.c Src/avionics.c -lm -o /tmp/tf && /tmp/tf
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_ek7_host.c     Src/ek7.c Src/flight.c Src/avionics.c -lm -o /tmp/te && /tmp/te
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_avionics_host.c Src/avionics.c -lm -o /tmp/ta && /tmp/ta
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_runmode_host.c Src/runmode.c Src/flight.c Src/avionics.c -lm -o /tmp/tr && /tmp/tr
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_gps_host.c      Src/gps.c -lm -o /tmp/tg && /tmp/tg
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_gpscfg_host.c   Src/gps.c -lm -o /tmp/tc && /tmp/tc
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_lora_host.c     Src/lora.c -lm -o /tmp/tl && /tmp/tl
gcc -std=c11 -Wall -Wextra -I Inc tests/host/test_tui_buf_host.c -lm -o /tmp/tt && /tmp/tt
```

| Suit | Ölçülen sonuç | Kapsam özeti |
|---|---|---|
| flight | `HOST_FLIGHT_TESTS_PASSED n=668` | yaşam döngüsü, kapılar, bench kanalları, P0 füzyonu |
| ek7 | `HOST_EK7_TESTS_PASSED n=97` | komut/checksum, telemetri/durum, eksen eşleme |
| avionics | `HOST_TESTS_PASSED n=183` | tilt/irtifa/histerezis/apogee/EMA/Kalman/MPU-cal; karar: A(EMA) üretimde kalır |
| runmode | `HOST_RUNMODE_TESTS_PASSED n=35` | kilit, kabul/ret, display modu |
| gps | `HOST_GPS_TESTS_PASSED n=92` | NMEA ayrıştırma, fix/stale, taşma |
| gpscfg | `HOST_GPSCFG_TESTS_PASSED n=161` | UBX kurucular, 8 adım, zamanlayıcı |
| lora | `HOST_LORA_TESTS_PASSED n=6307` | altın vektör, CRC, seq sarma, sentinel turları, kadans/kapı (döngü-tabanlı çoklu kontrol) |
| tui_buf | `ALL TUI BUFFER TESTS PASS` (25 PASS satırı, özet dahil) | bayt-bayt TUI eşliği, taşma=tek-gönderim, `tui_buf=1400 > en-kötü 954, marj 446` |

### 7.3 TUI kanıt satırları (beklenen çıktılar)

```
MODE: REAL  LOCK: OPEN  REJ=0
FLIGHT: WAIT_FOR_LAUNCH PROD
GPS: NOFIX sats=0 rx=<artan>
GPSCFG: OK 8/8
LORA: <seq> ok=<artan> drop=0 auxbusy=<...>
LED: BLUE=0 YEL=0 RED=0
```

Kabul kanıtları: `OK 8/8` (orijinal u-blox kanıtı — klon ACK vermez), `drop=0`, `err=0`,
sıfır derleme uyarısı (`base.md:208-209`). SIT/SUT kart koşusu özeti `test-results/summary.txt`:
PASS=8 FAIL=2 (SIT-06 quiet-soak, SIT-10 SIT CMD+TLM) NOT_EXECUTED=3 (el gerektiren 07/08/09).

---

## 8. AÇIK İŞLER (PENDİNG)

- [ ] **Uno RAW=0 (M1 şüphesi + ölçüm).** Birincil şüphe: M1/D6 hattı açık → modül Mode 0 dışında
  takılı. Ölçüm planı: `lora_cfg_boot` çıktısına bak (`LORA CFG: FAILED (AUX yok/cevap yok/... )`,
  `uno_lora_rx.ino:322-332`); D5/D6'da Mode 0 (LOW/LOW) doğrula; D2 boş-seviye satırını oku
  (`uno_lora_rx.ino:228-233`); AUX D7 seviyesini kontrol et. İkincil şüphe: RF/desense/antenler.
- [ ] **C0 kalıcılık.** C0 flash yazılarda echo yok; C2 RAM geçici. Güç kesilince link yeniden FIX
  gerektirir — uzun-vadeli strateji (periyodik yeniden-doğrulama vs. kabul) kararlaştırılmadı.
- [ ] **Mesafe testi.** Modülleri ≥2 m ayır, `[LINK] seq=` artışını doğrula.
- [ ] **FIX testi.** Anteni gökyüzüne çıkar → `sats` → `FIX`, `pos=` dolmasını doğrula.
- [ ] **SIT/SUT regresyonu.** `test-results/summary.txt` FAIL=2 (SIT-06, SIT-10) kök-neden analizi;
  el-testleri (07/08/09) insan eli gerektirir.
- [ ] **GPS halka kesme-noktası düzeltmesi (§3.1 bug).** `Src/main.c:407` + `Src/main.c:1743`
  `(uint8_t)` daraltmalarını kaldır → gerçek 1024 B halka; yorumu ve TUI marjını güncelle.
- [ ] **TUI blokajını DMA/IT TX'e taşıma** (~47 ms kablo süresi, uçuş çözünürlüğü).
- [ ] **`SIT_SUT_REQUIREMENTS.md` güncellemesi** (CFG-MSG/dynModel/measRate değişti).
- [ ] **USB kablolarını değiştirme** (iki kartta da port düşüyor).
- [ ] **`uno_lora_rx.ino:40` yorum eskimesi** (`alt_cm` → metre) + Uno CRC notunun korunması.
- [ ] **Uno verici tarafı sabit-header eşleşmesi:** peer fixed-TX'te kalırsa hava imajı 33 B
  (`00 02 17` + çekirdek) olmalı — alıcı teşhisindeki `HDR=` sayacı bunu izler
  (`uno_lora_rx.ino:417-425,454`).

---

## 9. AJAN KALİTE NOTU (KISA)

Doğrulanmamış alt-ajan çıktısı kanıt değildir (`GPS_LoRA_OTURUM_NOTU.md:171-180`): bu oturumda ucuz
subagent'lar var olmayan CMake preset'ini "doğrulandı" diye bildirdi, test sayısını sessizce düşürdü
ve döngü-tabanlı şişirilmiş kontrol sayısını raporladı. Kural: derleme çıktısı, test sayısı ve donanım
kanıtı **kendi komutunla** teyit edilmeden rapora yazılmaz. Bu dosyanın test sayıları (§7.2) ve
`Src/main.c:407` + `Src/main.c:1743` halka bulgusu (§3.1) doğrudan okuma/derleme ile doğrulandı;
oturum notundaki "düzeltildi" iddiasına körü körüne güvenilmedi.

---

## EK. DOSYA ENVANTERİ (yol + tek satır)

- `Src/main.c` — Tüm donanım sahipliği: UART'lar, GPIO, kalibrasyon, ana döngü, TUI, Ek-7 modları.
- `Src/lora.c` — Tek-yön LoRa telemetri + boot konfig motoru (Mode 3 → doğrula → Mode 0).
- `Src/gps.c` — NMEA ayrıştırıcı + UBX ACK izleyici + 8-adımlı pad konfig zamanlayıcı.
- `Src/flight.c` — Uçuş yaşam döngüsü: kalkış/apogee/free-fall/tek-atımlık konuşlandırma.
- `Src/ek7.c` — Ek-7 protokolü: komut çözücü, telemetri/durum çerçeveleri, eksen eşleme.
- `Src/avionics.c` — Tezgâh matematiği: tilt, baro, histerezis, apogee trendi, EMA, Kalman, MPU-cal.
- `Src/runmode.c` — REAL/TEST/SIT/SUT ayrımı + 30 m gerçek-uçuş kilidi.
- `Src/bmp280.c` — Bosch tamsayı basınç/sıcaklık kompanzasyonu (I2C).
- `Src/mahony.c` — Mahony 6-DOF tamamlayıcı filtre (yalnızca diagnostik).
- `Src/stm32f4xx_hal_msp.c` — MSP pin konfigleri (USART2/6/1, I2C1).
- `Src/stm32f4xx_it.c` — Kesme yönlendiriciler (USART2/6 RX, erken-önyükleme baskısı).
- `Src/syscalls.c` — Newlib sistem çağrı iskeleti (CubeMX).
- `Src/sysmem.c` — Heap iskeleti (CubeMX).
- `Src/system_stm32f4xx.c` — Saat/sistem başlatma (CubeMX).
- `Inc/lora.h` — LoRa tel formatı, sabitler, pinler, beklenen register imajı, API.
- `Inc/gps.h` — GPS ayrıştırıcı + UBX konfig sözleşmesi ve sabitleri.
- `Inc/flight.h` — Uçuş eşikleri, durumlar, bench kapıları, API.
- `Inc/ek7.h` — Ek-7 sabitleri, checksum kuralları, D1-D5 sapma notları, API.
- `Inc/avionics.h` — Tüm tezgâh eşikleri (tek yer), matematik API'si.
- `Inc/runmode.h` — Kilit kuralı (30 m), mod enum'ları, API.
- `Inc/bmp280.h` — BMP280 register haritası + demo konfig değerleri.
- `Inc/mahony.h` — Filtre kazançları + API.
- `Inc/main.h` — B1/LD2/UART pin tanımları (CubeMX).
- `Inc/stm32f4xx_hal_conf.h` — HAL modül anahtarları.
- `Inc/stm32f4xx_it.h` — Kesme prototipleri.
- `tests/host/test_flight_host.c` — Uçuş yaşam döngüsü host testleri (668).
- `tests/host/test_ek7_host.c` — Ek-7 protokol host testleri (97).
- `tests/host/test_avionics_host.c` — Tezgâh matematiği host testleri (183).
- `tests/host/test_runmode_host.c` — Çalışma-modu host testleri (35).
- `tests/host/test_gps_host.c` — NMEA ayrıştırıcı host testleri (92).
- `tests/host/test_gpscfg_host.c` — UBX konfig host testleri (161).
- `tests/host/test_lora_host.c` — LoRa çerçeve/kadans/kapı host testleri (6307).
- `tests/host/test_tui_buf_host.c` — TUI tek-gönderim tampon host testleri (~26).
- `uno_lora_rx/uno_lora_rx.ino` — Uno LoRa alıcısı (D2/D3 + Mode 3 boot konfig + teşhis).
- `uno_gps_config/uno_gps_config.ino` — Uno NEO-6M UBX konfigüratörü (D2/D3, 8 adım).
- `uno_lora_getconfig/uno_lora_getconfig.ino` — Kütüphaneli E220 register dökümü (teşhis).
- `uno_gps_test/uno_gps_test.ino` — Kütüphanesiz NEO-6M ham teşhis (D0).
- `uno_gps_test/uno_gps_baudscan/uno_gps_baudscan.ino` — Baud taraması (v3 D5/D4).
- `uno_gps_test/uno_gps_deep/uno_gps_deep.ino` — Çift-pin dinleme + reset kanıtı.
- `uno_gps_test/uno_gps_diag/uno_gps_diag.ino` — Say-sonra-bas teşhis deseni.
- `uno_gps_test/uno_gps_ledverdict/uno_gps_ledverdict.ino` — USB-bağımsız LED teşhis.
- `uno_gps_test/uno_gps_power/uno_gps_power.ino` — A0 üzerinden VCC ölçümü.
- `uno_gps_test/uno_gps_scan_d0/uno_gps_scan_d0.ino` — D0 çoklu-baud taraması (v5).
- `uno_gps_test/dinle_reset.py` — Tek-port-açık-tut reset+okuma yardımcısı.
- `uno_gps_test/tek_port_test.py` — Tek-port reset+okuma yardımcısı.
- `CMakeLists.txt` — C11 hedefi, kaynak listesi, LORA_WITH_HAL, FLIGHT_TEST_MODE opsiyonu.
- `CMakePresets.json` — Debug/Release presetleri (Bench YOK).
- `cmake/gcc-arm-none-eabi.cmake` — ARM toolchain dosyası.
- `cmake/starm-clang.cmake` — Clang toolchain dosyası.
- `cmake/stm32cubemx/CMakeLists.txt` — CubeMX kütüphane derlemesi.
- `base.md` — KALICI mimari dokümanı (iki kart, pinler, protokol, tuzaklar, doğrulama).
- `GPS_LoRA_OTURUM_NOTU.md` — 2026-10-06 oturum dersleri (hatalar, brownout, link, ajan kalitesi).
- `SIT_SUT_REQUIREMENTS.md` — Ek-7 resmi gereksinimler + davranış envanteri + el-testi kanıtı.
- `E220-900T22D_UserManual_EN_v1.2.txt/.pdf` — E220 datasheet (mod/register/AUX/fabrika).
- `rocket_avioniks.ioc` — CubeMX projesi (yalnızca USART2+I2C1+GPIO; USART6/1 YOK).
- `pinsketch.md` — El yazısı pin eskizi.
- `precommands.md` — VCOM screen + SIT canlı-koşu komutları.
- `test-results/summary.txt` — Kart SIT/SUT koşu özeti (8 PASS / 2 FAIL / 3 NOT_EXECUTED).
- `roketpdf/` — 14 resmi TEKNOFEST Ek docx'i (resmi Ek-7 dahil).
- `startup_stm32f446xx.s` — Vektör tablosu/başlatma (standart).
- `STM32F446xx_FLASH.ld` — Bağlayıcı betiği (standart).
- `UKB Taklit Yazılımı/` — Resmi Ö-UKB test yazılımı taklidi (SIT/SUT karşı tarafı).

---

## 10. SAHA RUNBOOK (LoRa)

Mimari ve pin sözleşmesi `base.md`'de sabittir; aşağıdakiler yalnızca sahada uygulanacak sıra ve kabul ölçütüdür.

### 10.1 Hedef register imajı (her iki modülde aynı olmalı)

| Bayt | Değer | Açılım |
|---|---|---|
| ADDH / ADDL | `00 00` | adres 0x0000, şeffaf modda filtre yok |
| REG0 | `0x65` | 9600 8N1 + 19,2k hava hızı |
| REG1 | `0x00` | 200 B alt-paket, RSSI kapalı, 22 dBm |
| REG2 | `0x17` | CH 23, 873.125 MHz |
| REG3 | `0x00` | şeffaf iletim, LBT yok |

Tam imaj: `00 00 65 00 17 00`. Mode 0 (M0=0/M1=0) = çalışma, Mode 3 (M0=1/M1=1) = konfig.

### 10.2 Boot kanıt satırları (örnek format, değerleri sahada okunur)

STM32 VCOM (115200), reset sonrası ilk satırlardan:

```
LORA CFG: OK AUX=1 B=9600 RD=00 00 65 00 17 00 ST=4 W=-
```

Uno seri monitör (115200), reset sonrası:

```
LORA CFG read: 00 00 65 00 17 00
LORA CFG: OK
```

`OK` = imaj zaten doğru, yazılmadı. STM32 uyumsuzlukta C2-RAM yazar
(`FIXED`, güç kesilince uçar, her boot yeniden yazılır); Uno C2 dener,
yalnız eşleşmezlikte C0 yazar. `FAILED` uçuşu/alıcıyı durdurmaz.

### 10.3 30 saniye kabul kriteri (uçtan uca link)

1. Her iki kartı besle, 30 sn dokunmadan bekle.
2. KABUL = Uno'da `[LINK] seq=<n>` düzenli artar + STM32'de `LORA: <seq> ok=<artan> drop=0`.
3. CRC geçen çerçeve esastır; `ok` artıp `[LINK]` yoksa hava yolu yok demektir.
4. KABUL DEĞİL = 30 sn sonunda `[LINK]` hiç yoksa ya da `good=0, RAW=0` ise Bölüm 10.4-10.6'ya geç.

### 10.4 Desense testi (modüller 3-5 m ayrılacak)

1. İki modülü aynı masada 1 m'den yakın tutma (LNA doyumu/desense riski).
2. STM32 + anten bir uçta, Uno + anten diğer uçta, aralık 3-5 m, antenler dikey.
3. 30 sn izle: `[LINK] seq` artıyorsa mesafe yeterlidir.
4. Artmıyorsa mesafeyi 5 m'ye çıkar ve anten yönünü düzelt, sonra Bölüm 10.5'e geç.

### 10.5 Besleme ölçümü (22 dBm tepe ~110 mA)

1. Şüphe belirtisi: Uno'da `RAW=0`, STM32'de `B=-- RD=-- ST=1/2` veya GPS tarafında bilinen `NL=0, CR=0` deseni.
2. Ölçüm: LoRa VCC-GND arası multimetre ile TX anında gerilimi izle; 5V hattında 4.7V altına düşme olmamalı.
3. İnce kablo/breadboard ile besleme yapma; kalın kısa kablo + modül pinlerine yakın 100 uF + 100 nF kullan.
4. Besleme düzeldikten sonra kartlara reset at ve Bölüm 10.2 satırlarını yeniden oku.

### 10.6 Modül takası sırası (kod değiştirmeden önce)

1. Sıra: antenleri değiştir (kırık/kötü temas) -> kabloları kısalt/değiştir -> STM32 ve Uno modüllerini birbiriyle takas et.
2. Her adımdan sonra reset + 30 sn kabul testi (Bölüm 10.3) uygula.
3. Takas sonrası `LORA CFG` satırı düzeldi ama `[LINK]` yoksa RF tarafı şüphelidir; tersi durumda kart UART/pin tarafı şüphelidir.
4. MPU sağlamlığı ayrıca `SENS: MPU=OK` satırı ile doğrulanır; LoRa sorununda MPU'ya dokunma.

### 10.7 Ne zaman kodcuya dönülür (hangi log satırıyla)

1. `LORA CFG: FAILED ... B=-- RD=-- ST=1/2` her iki kartta ve tüm takas/besleme adımlarından sonra sürüyorsa konfig motoruna dönülür.
2. STM32'de `ok` artıp `auxbusy=0` sabit kalıyorsa bayt modüle girmiyor demektir, sürücüye dönülür.
3. Uno'da `RAW` artıyor ama `bozuk` artıp `good=0` ise çerçeve/CRC tarafına dönülür (log satırını olduğu gibi ilet).
4. `[LINK] seq` artıyor ama telemetri alanları donuksa paket-içerik tarafına dönülür (seq + ham satırı ilet).
