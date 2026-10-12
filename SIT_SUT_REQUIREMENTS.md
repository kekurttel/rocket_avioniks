# SİT/SUT — Ek-7 Resmi Gereksinimler + USB-UART Bench Akışı

Board: NUCLEO-F446RE. Transport: ST-LINK VCOM 115200 8N1 (Ek-7 Tablo 7;
görev kararı: RS232 seviyesi/MAX3232 YOK, ST-LINK VCOM kullanılır).
Resmi kaynak: `roketpdf/Ek-7_..._DtJW1.docx` (TEKNOFEST Ö-UKB testleri).

## Resmi Ek-7 özeti

- Komut seti (Tablo 1, Big Endian): `AA CMD SUM 0D 0A`. Çalışan resmi
  test yazılımı §3 toplama kuralını gönderir: SİT başlat `AA 20 CA`,
  SUT başlat `AA 22 CC`, Durdur `AA 24 CE`. Doküman-literal Tablo 1
  değerleri (`AA 20 8C` / `AA 22 8E` / `AA 24 90`) geriye uyumluluk
  için ayrıca kabul edilir. Komut +1 sn aktivasyon; Durdur → bayraklar
  pasif + normal mod.
- SİT (§2.1.1): 10 Hz telemetri — Tablo 2 alanları (irtifa m, basınç mBar,
  ivme xyz m/s², açı xyz °; hepsi FLOAT32 BE, 2 ondalık) + checksum +
  footer. Eksenler §1.2 gövde ekseni (X burun, Y sağ, **Z aşağı**).
- SUT (§2.1.3): aynı 36 B yapı (header 0xAB) sentetik giriş; Ö-UKB kendi
  sensörünü görmezden gelir, veriyi filtre+algoritmadan geçirir; 10 Hz
  2-byte durum paketi (Tablo 5 bitleri, Tablo 6: `AA D1 D2 SUM 0D 0A`).
- Durum bitleri: 0 kalkış, 1 motor-süre, 2 min-irtifa, 3 açı,
  4 alçalma, 5 sürüklenme emri, 6 ana-altı, 7 ana emri, 8-15 rezerve.
- Haberleşme (Tablo 7): 115200 8N1 full-duplex, frame timeout 100 ms,
  komut latency ≤10 ms, SUT timeout 1000 ms → güvenli sonlandırma.
- SİT planı: 2-3 m el hareketiyle veri trafiği/filtre takibi. SUT: normal +
  anormal (ani ivme/basınç) profiller, hata yönetimi beklenir.
- Donanım: MAX3232/D-SUB9/pyro klemensleri **bu tezgahta YOK**
  (NOT EXECUTED + sınırlama olarak raporlanır).

## Doküman-kod çelişkileri (tespit, uydurma yok)

- D1 komut checksum: §3 toplama kuralı (CA/CC/CE) Tablo 1 literal
  değerlerini (8C/8E/90) vermiyor → ÇÖZÜLDÜ: resmi yazılımın
  gönderdiği toplama kuralı birincil, Tablo 1 literalleri ikincil
  olarak ikisi de kabul edilir (host testleri ikisini de kilitler).
- D2 telemetri boyutu: Tablo 3 "34 byte" vs Tablo 2 alan yapısı (8 float =
  36 byte) → alan yapısı esas (36 B).
- D3 Tablo 6 örnek checksum (0x1A) toplama kuralıyla tutarsız (header'lı
  toplam 0xBE, headersiz 0x14; ikisi de 0x1A değil) → örnek değeri
  yoksayılır; çalışan yazılımın doğruladığı kural uygulanır: telemetri
  SUM = header+32 bayt, durum SUM = header+D1+D2 (mod 256). Durum
  bayt sırası da düzeltildi: önce D1(bit0-7), sonra D2(bit8-15).
- D4 irtifa "MSL" istenir; bizde yer-referanslı bağıl irtifa (MSL yok).
- D5 bit1 (motor yanma süresi) tasarımda yok → her zaman 0.
- D6 canlı sensör ~21080 LSB/g ölçtü (nominal 16384'ün +%28 üstü) →
  telemetri |g_ref| ile self-calibrate; kararlar oran-tabanlı (etkisiz).
- D7 Tablo 5'te fault biti yok → fault RED LED + UART ile yerelde.

## Eşleşme (özet)

launch→bit0/state, apogee→bit4, drogue→bit5, main→bit7, min-alt→bit2,
below-main→bit6, tilt-band→bit3, fault→(yok, RED), one-shot→korunur,
P0/MS-fallback→korunur, bench→display-only (SUT'u etkilemez).

## Ek-7 durumu

`Ek-7` dokümanı workspace'te BULUNAMADI (`*.md/*.pdf/*.txt` tarandı;
`readme.md` boş; tek doküman `rocket_avionics_wiring.md`). Aşağıdaki
test ID'leri (`SIT-001`…) bu dosyadan türetilmiş RUNNER ID'leridir, resmi
Ek-7 numarası değildir. Ek-7 geldiğinde ID eşleştirmesi yapılmalıdır.
Doküman drift notu: `rocket_avionics_wiring.md` ikinci cihazı BMP280#2
(@0x77) gösteriyor, kod 0x77'yi MS5611 sayıyor; ayrıca "External LEDs:
not connected" artık geçersiz (PA10/PB5/PB4 bağlı).

## Davranış envanteri (koddan)

- States: INIT→CALIBRATION→WAIT_FOR_LAUNCH→ASCENT→APOGEE_CONFIRMED→
  FREE_FALL→DROGUE_DEPLOYED→DESCENT→MAIN_DEPLOYED→COMPLETE + FAULT
  (tek yön, latch; FAULT sticky; COMPLETE terminal).
- Launch: |a|>1.2·|g_ref|, N=3. Apogee: max+drop 2 m+vel<−2 m/s
  (+1250 m PROD gate / TEST disabled). Free-fall: |a|<0.8·|g_ref|, N=3.
- Drogue/Main: logical event + LED + UART, one-shot, state+sensor interlock.
  Fiziksel pyro GPIO YOK.
- Altitude: BMP280+MS5611 → median3/adet → combine → EMA → baro.
  Velocity: (cur−prev)/dT. Accel: MPU6050 filtreli vektör.
- Tilt (`TILT_30_45`, Mahony): SADECE diagnostik (BLUE LED + TUI).
  Hiçbir deployment/descent kararına girmez (grep ile doğrulandı).
- TEST_MODE: sadece parametre değiştirir (apogee-min, main-alt, bench arm).

## Negatif düşey ivme trigger (TEST bench, accel kanalı)

```
u      = g_ref / |g_ref|            (kalibrasyon board-up birimi)
vert_g = dot(a_filt, u) / |g_ref|   (≈+1.0 rest, g birimi)
dyn_g  = vert_g − 1.0               (≈0 rest, −1.0 free-fall)
```

- İşaret (özgül kuvvet fiziği): kart AŞAĞI hızlanınca ölçülen özgül kuvvet
  azalır → dyn NEGATİF. Yukarı → POZİTİF. Sabit Z varsayımı YOK.
- Statik g_ref işareti kodla tutarlı: flat table → g_ref≈[0,0,+16384].
- Bench onayı: `dyn < −0.30 g` VE `|a|/|gref| < 0.95`, N=5 consecutive
  (~500 ms @ ölçülen 100 ms loop). N=3→5 gerekçesi: masa titreşimi 3'lü
  seriyi sağlayabiliyordu (flight.h yorumu). `0.95` mag kapısı quasi-statik rotasyonu
  (dyn=cos−1) translasyondan ayırır: salt tilt asla tetiklemez.
- Eşik marjı (canlı ölçü): VACC quiet max |6| mg → eşiğe ~50×.
- Her modda aktif (display + BLUE LED; state/deployment'a bagli DEGIL).
  Uretim dahil el testi icin acik tutulur (kullanici karari).

## Barometrik düşüş kanalı (TEST bench, bağımsız ikinci sinyal)

- Kaynak: GERÇEK fused altitude (mevcut pipeline, yeni smoother YOK).
- Tetik: `baro_vel < −1.0 m/s`, N=4 consecutive (~400 ms), latch.
  N=4 gerekçesi (canlı ölçü): quiet vel gürültüsü tek örnekte −1.5 m/s'ye
  ulaşabiliyor; 4'lü seri gözlenmedi. El-hareketi hızı henüz ölçülmedi —
  kullanıcı testinde doğrulanacak (bölüm 13 prosedürü).
- `BARO_DROP` (alt < bench_max − 0.3 m) SADECE display flag'idir; tetiğe
  girmez. Oda drift'i ile quiet'ta kırpışabilir — normal, tetik etkilenmez.
- Birleşik sinyal: `BENCH_FALL = ACCEL_FALL OR BARO_FALL` (latch).
  State machine'e ve deployment'a bağlı DEĞİL:
  `BENCH_FALL_DETECTED != DROGUE_DEPLOY_EVENT`. SUT'ta mavi, bench
  yerine sentetik apogee'den yanar (dususu gosterir).

## Eşik tablosu

| Parametre | PROD | TEST |
|---|---|---|
| apogee_alt_min | 1250 m | disabled (0) |
| main_alt | 600 m | 5 m (override: `-DFLIGHT_TEST_MAIN_ALT_M`) |
| bench_neg_thr | — (inert) | −0.30 g |
| bench_confirm_n | — | 5 |
| bench_mag_ratio | — | 0.95 |
| bench_baro_vel / n / drop | — | −0.1 m/s / 3 / 0.3 m + hareket kapısı |
| launch / free-fall | 1.2×/0.8×, N=3 | aynı |

## Telemetri (50 ms TUI + edge mesajları)

`SYSTEM INIT, CALIBRATION START/DONE, SENSORS OK, WAITING FOR LAUNCH,
LAUNCH DETECTED, APOGEE CONFIRMED, FREE FALL DETECTED, DUSUS ALGILANDI,
DROGUE/MAIN DEPLOY EVENT, FLIGHT COMPLETE/FAULT, * ERROR` ve periyodik:
`VERT_ACC (mg), NEG_ACCEL ON/OFF, NEG_ACCEL_COUNT, BARO_VVEL (m/s),
BARO_DROP ON/OFF, BENCH_FALL YES/NO, DESCENT_DETECTED YES/NO (accel koldan),
BENCH ARMED/OFF, TILT_30_45 ON/OFF (diagnostik, BLUE'dan bağımsız),
MAX ALTITUDE, VERTICAL VELOCITY, DEPLOY n=x/y, P0/FUSED`.

## LED haritası

Yeşil PA5: flight latch. Mavi PA10: BENCH_FALL (accel OR baro, her mod;
tilt BLUE'u sürmez — fiziksel doğrulandı, aşağıda). Sarı PB5: drop +
deploy latch. Kırmızı PB4: FAULT-only (normalde OFF, FAULT'ta 2 Hz blink;
düşüşte/drogue/main'de kullanılmaz).

## Değişken sözlüğü (A18)

- P0 = yerdeki başlangıç basıncı (kalibrasyonda bir kez yazılır; `bmp1_p0`,
  `ms1_p0`, `fused_p0` — `main.c`).
- H = mevcut bağıl irtifa, m (`alt_m`, baro formül `avt_altitude_m`).
- Hmax = maksimum irtifa (`flight.max_alt_m`, `bench_baro_max_m` display).
- V = düşey hız, m/s (`flight.vel_mps`, `(cur-prev)/dT`; `bench_baro_vel_mps`).
- Adyn = gravity'den ayrılmış dinamik ivme, g (`flight.vert_dyn_g` =
  `dot(a,gref)/|gref|²`-ölçekli `- 1`; free-fall → −1).
- state = uçuş durumu (`flight.state`, 11 durum, tek yön + latch).
- health = `flight_health_t` (present/valid/stale + fused_src + disagree).

## V2 altyapı izleri (gereksinim → değişken → kod → test → kanıt)

- TIMING (A3): `tim_n/sum/min/max` (`main.c`) → `TIM ...` TUI → kart ölçümü.
- SELFTEST (A6): `mpu_probe_ok/cal_mpu_ok` → `SELFTEST:` satırı → kart boot log.
- RESET_CAUSE (A11): `boot_reset_cause` → `RESET_CAUSE:` → kart boot log.
- IWDG (A11): `iwdg_start/refresh` (bare-metal, HAL modülü yok) → 60 s+ soak.
- HEALTH (A7): `flight_health_snapshot` → `SENSOR_HEALTH:` → host `tH_snap`.
- CAL (A5): `avt_mpu_cal_*` (identity default, live path'a bağlı DEĞİL) →
  host `tCAL_*`; 6-poz prosedürü NOT EXECUTED (el yok).
- BARO-DISAGREE (A8): `AVT_BMP_DISAGREE_PA` 50 Pa → FUSED `AVG-DISAGREE` →
  host `tF9` + kart.
- FAULT-INJ (A9): host `tF1/F2/F4/F5/F6/F7/F11` (donmuş/spike/eksik/stale).
- I2C-REC (A10): `i2c_link_recover` (max 3/boot, GPIO yok) → MS hattı gözlemi.
- EST A/B (A12): host `tAB_table` → karar: A(EMA) üretimde kalır.
- HIL (A13): `tests/traces/ascent.csv` + `tests/hil_send.py` → SUT akışı.
- RATE A/B (A4): benchmark, değişiklik YOK (gerekçe aşağıda).

## El-testi kanıtı (HAND-002/004, TEST fw, gerçek donanım)

- Fresh boot → WAIT, BENCH NO, BLUE sönük (UART + göz).
- Tilt anında (TILT_30_45 ON @2.71 s) BENCH NO / BLUE sönük → açı tetiklemez.
- Aşağı harekette VERT −353→−478 mg, NEG count 1→2→3, t=3.05 s'te
  DESCENT YES + BENCH_FALL YES (o günkü N=3 tasarım değeriyle; güncel
  kod N=5, bu kanıt eski eşikle alındı), DUSUS
  ALGILANDI, BLUE yandı + latch (gözle doğrulandı).
- VERT min −878 mg (güçlü el hareketi). Baro VVEL quiet max tekil −1.5 m/s
  (o günkü N=4 eşiğiyle seri oluşmadı → tetik yok). State WAIT'ta kaldı, deployment/fault
  sıfır (decouple kanıtı).

## Test koşumu

```
python3 tests/sit_runner.py [--port PATH] [--capture SEC] [--out DIR]
```

`--static-only`: el gerektiren testleri atlar (NOT EXECUTED işaretler).
Hareketli testler (SIT-007/008/009) insan eli gerektirir; otonom koşuda
her zaman NOT EXECUTED + gerekçe ile kaydedilir, asla PASS uydurulmaz.
