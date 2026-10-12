/* LoRa telemetry uplink: Ebyte E220-900T22D (LLCC68, 850.125-930.125 MHz,
 * default 873.125 MHz / CH 0x17, 22 dBm).
 *
 * THE MODULE IS WIRED AS A TRANSPARENT UART, NOT SPI. Datasheet Table 3-1:
 *   1 M0  input, weak pull-up  -> STM32 PB10 (Arduino D6), GPIO output.
 *   2 M1  input, weak pull-up  -> STM32 PA8 (Arduino D7), GPIO output.
 *   3 RXD input  (TTL UART)      <- board TX (PB6)
 *   4 TXD output (TTL UART)      -> board RX (PB7)
 *   5 AUX output                 -> STM32 PA9 (Arduino D8), GPIO input.
 *   6 VCC 2.3-5.5 V             -> 5 V (>=3.3 V for full TX power)
 *   7 GND
 * Section 6 mode table (M1,M0): (0,0) = Mode 0 TRANSMISSION = "UART and
 * wireless channel open, transparent transmission on" - the RUN mode;
 * (1,1) = Mode 3 Deep Sleep = the CONFIGURATION mode (registers readable/
 * writable, only at 9600 8N1). The firmware owns M0/M1: boot enters Mode 3,
 * registers are scanned (C1, en fazla 3 tur; tur arasi Mode-0 + 100 ms),
 * her boot 65-imaji (9600 8N1 + 19,2k hava) C2-RAM ile kosulsuz yazilir ve
 * geri okumayla dogrulanir; SADECE ilk tarama okumasi 65-imajindan farkliysa
 * (modul irasamis) ve C2-dogrulama tutmussa ayni imaj bir kez C0 ile
 * kalici yazilip tekrar okunur (eslesen imaja C0 YAZILMAZ: flash omru).
 * Sonra Mode 0'a donulup ucusa gecilir. AUX her C1 oncesi en fazla 300 ms
 * beklenir, gelmezse YINE DENENIR (bagli AUX sarti YOK: takilma yok).
 * huart1 hep 9600'de kalir (57600/C5 dali YOKTUR).
 * AUX modul-mesgul isaretidir (dusuk = mesgul/oz-denetim; yuksek = hazir);
 * her kip degisimi sonrasi 100 ms oturulur, her C1 oncesi AUX-high en fazla
 * 300 ms beklenir ama beklenememesi YOLU KESMEZ (bagli AUX sarti YOK).
 * C2-RAM her boot yazilir; C0 yalniz yukaridaki kosulda bir kez yazilir.
 *
 * ---------------- WHY PB6/PB7 (and not PA9/PA10) ----------------
 * USART1's primary AF7 pair is PA9 (TX) / PA10 (RX). PA10 is the NUCLEO-F446
 * BLUE LED and is ACTIVELY driven by the flight logic (Src/main.c:
 * `#define BLUE_LED_Pin GPIO_PIN_10`, toggled in the bench-fall latch and the
 * FAULT blink path). Reusing it for a UART would silently break the blue LED
 * that the bench fall trigger and the FAULT indicator depend on, so USART1
 * moves to its second AF7 pair on this part:
 *     PB6 = USART1_TX (AF7) -> module RXD
 *     PB7 = USART1_RX (AF7) <- module TXD
 * PB6/PB7 are Arduino D10/D11 on the NUCLEO headers and are unused anywhere in
 * Src/ (PB4 red, PB5 yellow, PB8/PB9 I2C1; PB10/PB11 left free). The pin
 * configuration lives in the USART1 branch of HAL_UART_MspInit()
 * (Src/stm32f4xx_hal_msp.c).
 * ------------------------------------------------------------------
 *
 * Link direction is ONE-WAY: rocket -> ground station. There is no ACK in v1,
 * so the frame is self-validating: a receiver rejects anything whose magic,
 * version or additive CRC does not match, and any field whose value equals its
 * documented sentinel is treated as "no data" rather than as a real reading.
 *
 * ---------------- WIRE FORMAT: exactly 36 bytes, little-endian (v2) ----------
 * Built by EXPLICIT byte writes into a uint8_t buffer (never memcpy of a
 * struct), so the wire image cannot be perturbed by compiler padding,
 * alignment or bitfield layout. _Static_assert below pins the length.
 *
 *  off  len  field        type    meaning
 *  ---  ---  -----------  ------  ----------------------------------------
 *    0    1  magic        u8      0xA5, fixed frame start
 *    1    1  ver          u8      0x02, protocol version
 *    2    2  seq          u16 LE  packet counter, +1 per packet, wraps
 *    4    4  t_ms         u32 LE  HAL_GetTick() at build time
 *    8    2  flight_state u16 LE  flight_state_t value
 *   10    2  ek7_mode     u16 LE  0 NORMAL / 1 SIT / 2 SUT (mirrors main.c)
 *   12    2  alt_m        i16 LE  irtifa, TAM METRE; gecersiz/NaN/inf/
 *                             aralik-disi okuma LORA_ALT_INVALID (0x8000)
 *                             verir (doyurma YOK)
 *   14    2  vel_cms      i16 LE  dikey hiz, cm/s; gecersiz/NaN/inf/
 *                             aralik-disi okuma LORA_I16_INVALID verir
 *                             (doyurma YOK)
 *   16    2  sats         u16 LE  satellites in use, ALWAYS live (fix or not); 0xFFFF reserved
 *   18    4  lat_e5       i32 LE  latitude, degrees x 1e-5; 0 = invalid
 *   22    4  lon_e5       i32 LE  longitude, degrees x 1e-5; 0 = invalid
 *   26    2  batt_mv      u16 LE  battery millivolts; 0 = unknown
 *   28    1  flags        u8      bit0 gps_fix, bit1 blue_led,
 *                                 bit2 yellow_led, bit3 red_led
 *   29    2  tilt_ddeg    u16 LE  egiklik decidegree 0..1800 (0 = dikey);
 *                             0xFFFF = gecersiz. Filtreli ivme vektorunun
 *                             kalkis referansina gore acisi.
 *   31    2  lean_x       i16 LE  govde X egilme yonu x1000 (-1000..1000);
 *                             gecersizken 0. Yaw referansi YOK.
 *   33    2  lean_y       i16 LE  govde Y egilme yonu x1000; gecersizken 0.
 *   35    1  crc          u8      sum(bytes[0..34]) mod 256
 *
 * The CRC is a plain additive checksum of bytes 0..34, matching the repo's
 * existing convention (ek7_sum, UART command sum). Verification on the
 * receiving side is therefore "lora_crc(f, LORA_CRC_OFF) == f[LORA_CRC_OFF]",
 * i.e. the residual is zero; lora_verify() wraps exactly that.
 * v1 (30 bayt, ver 0x01) cerceveler v2 ayristirici tarafindan REDDEDILIR
 * (versiyon kapisi); Uno alici her iki surumu de kabul eder.
 *
 * ---------------- ALTITUDE UNIT: WHOLE METRES (v1) ----------------
 * alt_m is int16 WHOLE METRES: range -32767..+32767 m, i.e. +/-32.7 km. The
 * production apogee threshold is 1250 m and the bench ceiling (SUT envelope)
 * is 20000 m, so both are representable with >15x margin and no scaling can
 * ever be needed for any high-power flight this airframe flies. Sub-metre
 * resolution is not telemetry data anyone acts on; it is thrown away to keep
 * the field at i16 and the frame at exactly 30 bytes.
 *
 * This replaces the original int16 CENTIMETRE field, which saturated at
 * +/-327.67 m and therefore reported every real apogee as 327.67 m - a
 * flight-critical data bug, since the ground station would have seen the
 * rocket peak far below the 1250 m production gate. tests/host/
 * test_lora_host.c pins 1250 m and 3000 m through a build/parse round trip
 * so the regression cannot come back.
 *
 * ---------------- CADENCE AND THE 400-BYTE MODULE BUFFER ----------------
 * The module buffers 400 bytes internally. The pacing is the send cadence
 * itself: one 39-byte air packet every LORA_SEND_INTERVAL_MS (200 ms =
 * 5 Hz), same on pad and in flight. Kılavuz Tablo 7-2'deki 19,2 kbps hava
 * hızıyla 39 baytlık hava imajı 39*8/19200 = 16,25 ms seri-eşdeğer süre tutar;
 * LoRa preambul/başlık yüküyle ~25-35 ms olur. 200 ms kadans demek %8-18 hava
 * doluluğu demektir (~165-175 ms marj): sarkma/AUX-meşgul anında kayıp
 * KADANS-TAŞMA kökenlidir (tampon birikimi değil, vade-tüketimi). 9600
 * geri-dönüşte kablo süresi 39*10/9600 ~= 41 ms, 57600'de ~= 7 ms; ikisi de
 * 39*10/9600 ~= 41 ms, 57600'de ~= 7 ms; ikisi de 200 ms'in içindedir.
 * LORA_SEND_GAP_MS is a compile-time FLOOR applied to the interval, not a
 * cadence by request (livelier ground-station display); flight safety
 * unchanged (bounded 50 ms TX, one packet per poll, never blocking).
 * LORA_SEND_GAP_MS is a compile-time FLOOR applied to the interval, not a
 * busy-wait after the transmit: a packet is due when
 * (now - s_next_ms) >= 0, and s_next_ms advances by one interval per send
 * (BIRIKIMLI vade: geciken tur faz kaydirmaz; hala gerideyse vade simdiki
 * zamana kurulur, kacirilan kadans atlanir, patlama gonderilmez).
 * There is therefore no delay call on the transmit path at all.
 *
 * At most ONE packet per lora_poll() call, and lora_poll() is called at most
 * once per main-loop iteration: the main loop is the only caller.
 *
 * ---------------- SIT/SUT GATE ----------------
 * LORA_ALLOW_SIT_SUT is 0: while ek7_mode is SIT or SUT the Ek-7 binary frames
 * own USART2 and must not be perturbed, and a LoRa transmit here would add a
 * bounded blocking call inside those loops. Set it to 1 only for a bench build
 * where LoRa telemetry is explicitly wanted during SIT/SUT.
 *
 * ---------------- FLIGHT SAFETY (non-negotiable) ----------------
 * - lora_poll() reads flight state, ek7_mode and GPS values. It WRITES nothing
 *   to the flight state machine and calls no deployment/LED handler.
 * - It cannot launch, apogee, drogue, main or deploy anything. The only
 *   direction of influence is rocket -> radio.
 * - The single HAL_UART_Transmit is bounded by LORA_TX_TMO_MS (50 ms); on
 *   timeout the packet is dropped, the TX-error counter (lora_txerr_count)
 *   advances and the loop carries on. K = LORA_TXERR_REINIT_K consecutive
 *   TX errors re-init huart1 (abort + DeInit/Init, register writes only,
 *   < 5 ms, no waits) at the verified link baud (57600, fallback 9600);
 *   the re-init count is lora_reinit_count().
 * - AUX pre-TX skip: vade dolmus bir paketten ÖNCE AUX örneklenir; AUX
 *   düsükse (modül hâlâ havaya veriyor) paket gönderilmez, bekleme YOK,
 *   vade aynen tüketilir (kadans fazi korunur, patlama yok), seq/ok/drop/
 *   txerr degismez. Sayac lora_airbusy_count() ile ayri okunur; drop toplami
 *   (build + TX) bozulmaz.
 *   A dead/absent radio costs at most 50 ms once per cadence tick
 *   and never wedges the loop.
 * - No interrupt is disabled, no spin-wait, no Error_Handler, no Fault_Handler.
 * - LoRa absence is NOT a fault: lora_init() just leaves the link not-ready and
 *   main.c prints one "LORA: INIT FAIL" line.
 */
#ifndef LORA_H
#define LORA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(LORA_WITH_HAL)
/* Target build: HAL_StatusTypeDef / HAL_OK come from here. */
#include "main.h"
typedef HAL_StatusTypeDef lora_status_t;
#else
/* Host build (tests/host/test_lora_host.c): the same signature, no HAL. */
typedef int lora_status_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- Protokol sabitleri (v2) ---------------- */
#define LORA_MAGIC (0xA5u)
#define LORA_VER (0x02u)

#define LORA_OFF_MAGIC (0u)
#define LORA_OFF_VER (1u)
#define LORA_OFF_SEQ (2u)
#define LORA_OFF_T_MS (4u)
#define LORA_OFF_FLIGHT_STATE (8u)
#define LORA_OFF_EK7_MODE (10u)
#define LORA_OFF_ALT_M (12u)
#define LORA_OFF_VEL_CMS (14u)
#define LORA_OFF_SATS (16u)
#define LORA_OFF_LAT_E5 (18u)
#define LORA_OFF_LON_E5 (22u)
#define LORA_OFF_BATT_MV (26u)
#define LORA_OFF_FLAGS (28u)
/* v2 ek alanlar (LE): tilt decidegree u16, lean_x/lean_y i16 x1000.
 * tilt = filtreli ivme vektorunun kalkis yercekimi referansina gore acisi
 * (0 = dikey). lean = normalize filtreli-ivme X/Y bilesenleri x1000
 * (govde ekseninde egilme yonu; yaw referansi YOK).
 * NOT: bayt 29 bosta kalamaz (30+2+2+2=36): tilt bayt 29'dan baslar,
 * CRC 35'te, cerceve 36 bayt. */
#define LORA_OFF_TILT (29u)
#define LORA_OFF_LEANX (31u)
#define LORA_OFF_LEANY (33u)
#define LORA_OFF_CRC (35u)

/* 36 bayt toplam: yukaridaki tum alanlar CRC baytina kadar bitisik. */
#define LORA_FRAME_LEN (36u)
/* CRC, [0, LORA_OFF_CRC) = [0, 35) = 0..34 dahil baytlari kapsar. */
#define LORA_CRC_OFF (LORA_OFF_CRC)

/* ---------------- Sentinels ---------------- */
/* irtifa "veri yok": 0x8000 int16 olarak -32768'dir, yani gecerli
 * -32767..+32767 m araliginin DISINDADIR. lora_frame_from_tlm() gecerli
 * aralikta yarim-uzakliktan yuvarlar; gecersiz/NaN/inf/aralik-disi her
 * okuma bu gozcuyu verir (doyurma YOK: 32767'ye kelepcelenmis sahte irtifa
 * gercek okumadan ayirt edilemezdi). */
#define LORA_ALT_INVALID ((int16_t)0x8000)
/* hiz (cm/s) "veri yok": gecerli aralik -32767..+32767 cm/s; aralikta KESME
 * (eski davranis aynen), disinda/NaN/inf'de gozcu (doyurma YOK). */
#define LORA_I16_INVALID ((int16_t)0x8000)
#define LORA_U16_NOSATS (0xFFFFu) /* reserved; live counts always pass */
#define LORA_I32_NOPOS (0)       /* lat_e5 / lon_e5 == 0 means invalid */
#define LORA_U16_NOBATT (0u)     /* battery millivolts unknown */
/* Egiklik (tilt) "veri yok": decidegree araligi 0..1800 disinda. */
#define LORA_TILT_INVALID (0xFFFFu)
/* Egilme yonu gecersizken lean_x/lean_y 0 tasir (yon yok). */

/* ---------------- flags byte ---------------- */
#define LORA_F_GPS_FIX (1u << 0)
#define LORA_F_BLUE_LED (1u << 1)
#define LORA_F_YELLOW_LED (1u << 2)
#define LORA_F_RED_LED (1u << 3)

/* ---------------- Timing ---------------- */
#define LORA_SEND_INTERVAL_MS (200u)   /* flight cadence: 5 Hz */
#define LORA_SEND_INTERVAL_WAIT_MS (200u) /* pad cadence: 5 Hz */
#define LORA_SEND_GAP_MS (100u) /* aritmetik alt sinir, gecikme cagrisi DEGIL.
                                  * NOT: 39 baytlik hava imaji 19,2 kbps'te
                                  * ~16 ms seri-esdeger + preambul (~25-35 ms havada) tutar; bu
                                  * 100 ms taban yalniz aritmetik alt sinirdir.
                                  * Gercek koruma 200 ms kadansin
                                  * kendisidir (vade aritmetigi, patlama yok). */
#define LORA_TX_TMO_MS (50u) /* hard cap on HAL_UART_Transmit */

/* Ardisik TX-hatasinda huart1 re-init esigi (K): HAL_UART_Transmit K kez
 * ust uste basarisiz olursa (cikmis/kopmus E220 kablosu gibi kalici ariza)
 * UART abort + DeInit/Init ile dogrulanmis link baud'unda yeniden
 * baslatilir (57600, geri-donuste 9600). Sayac her basarili gonderimde
 * sifirlanir; re-init sayisi lora_reinit_count() ile okunur. K=10: tek
 * seferlik parazitler (1-2) tetiklemez, kalici ariza (~2 sn = 10 kadans)
 * cabuk toparlar. */
#define LORA_TXERR_REINIT_K (10u)

/* ---------------- Link UART baud (STM32 <-> E220, Mode 0 seffaf trafik) --
 * Hedef 57600: 39 baytlik paket 9600'de ~41 ms, 57600'de ~7 ms kablo
 * suresiyle module girer. Hava hizi 19,2 kbps: IKI modul de ayni havada
 * OLMALIDIR (Uno modulu 65-imajinda = 19,2k; STM32 HI/C5 ya da urtak
 * 65-imajina yakinsar). Tek-yanli hava degisikligi linki oldurur (sahada
 * goruldu). Yazi dogrulanmadan baud degismez: C2 yazi + 57600 readback OK
 * ise link 57600'ye gecer, aksi halde 9600 geri-donuste kalir
 * (lora_cfg_boot). FALLBACK ayni zamanda lora_init() sonrasi varsayilandir. */
#define LORA_LINK_BAUD (57600u)
#define LORA_LINK_BAUD_FALLBACK (9600u)
/* REG0 UART bitleri (kullanim kilavuzu Tablo 7-2): bit7-5 baud, bit4-3
 * 00=8N1, bit2-0 101=19,2k hava. 0x65=011_00_101 (9600), 0xC5=110_00_101
 * (57600): UART disinda yalniz hava bitleri degisebilir; parite/guc/kanal
 * bitleri aynen korunur. */
#define LORA_REG0_9600 (0x65u)
#define LORA_REG0_57600 (0xC5u)

/* ---------------- RF tek-nokta: guc / hava hizi / kanal ----------------
 * Kullanim kilavuzu Tablo 7-2 + §10.1'e gore COZUM (mevcut imaj
 * 00 00 65 00 17 00; fabrika varsayilani 00 00 62 00 17 00 idi):
 *   REG0 = 0x65 = 011_00_101 -> 9600 8N1, hava 101 = 19,2 kbps.
 *   (Fabrika 0x62 = 011_00_010 idi: ayni UART/parite, hava 010 = 2,4 kbps.)
 *   REG1 = 0x00 -> alt-paket 00 = 200 B, ortam-RSSI kapali, guc 00 = 22 dBm
 *     (EN YUKSEK = donanim tavani; kilavuz §7.2: 00=22, 01=17, 10=13,
 *     11=10 dBm. Guc ARTIRILAMAZ, zaten tavandadir).
 *   REG2 = 0x17 = CH23 -> 850.125 + 23 = 873.125 MHz.
 *   REG3 = 0x00 -> seffaf iletim, RSSI bayti yok, LBT yok.
 * Asagidaki bit sabitleri bu imajin TEK NOKTASIDIR: LORA_CFG_EXPECTED /
 * LORA_CFG_EXPECTED_HI Src/lora.c'de bunlardan kurulur, derleme-zamani
 * kilitler (asagida) bit-bit esitligi kanitlar. Davranis degisikligi YOK:
 * iki imaj yalniz REG0 UART bitlerinde ayrilir; hava hizi/guc/kanal
 * bit-bit aynidir.
 * DURUST NOT (19,2k, OLCULDU + ONARILDI): 2,4 kbps'te 39B hava suresi
 * ~150-170 ms + 200 ms kadans = ~30 ms marjdi; TX-oncesi AUX atlama
 * iskalayip gondericiyi ~1,5 Hz'ye dusuruyordu. 19,2 kbps'te 39B imaj
 * ~25-35 ms olur; marj ~165-175 ms, 5 Hz surdurulebilir. Sart: IKI
 * modul de ayni havada OLMALIDIR — tek-yanli gecis linki oldurur (sahada
 * goruldu: STM32 C5'e, Uno 65'e dagilinca hava uyusmadi). Kurtulus:
 * W=2/W=3 kolemeleri her boot'ta imaji okuyup ya uyarlar ya urtak
 * 65-imajina dondurur; iki sonuc da 19,2k havalidir (Uno ile uyumlu).
 * Hiz artirmak duyarliligi dusurur (menzil saha olcumuyle dogrulanacak). */
#define LORA_REG0_UART_9600_BITS (0x60u)   /* bit7-5 = 011 */
#define LORA_REG0_UART_57600_BITS (0xC0u)  /* bit7-5 = 110 */
#define LORA_REG0_UART_MASK (0xE0u)
#define LORA_REG0_PARITY_8N1_BITS (0x00u)  /* bit4-3 = 00 */
#define LORA_REG0_AIR_2K4_BITS (0x02u)     /* bit2-0 = 010 = 2,4 kbps (Tablo 7-2, fabrika) */
#define LORA_REG0_AIR_19K2_BITS (0x05u)    /* bit2-0 = 101 = 19,2 kbps (Tablo 7-2, mevcut) */
#define LORA_REG0_AIR_MASK (0x07u)
#define LORA_REG1_SUBPKT_200B_BITS (0x00u) /* bit7-6 = 00 */
#define LORA_REG1_SUBPKT_MASK (0xC0u)
#define LORA_REG1_POWER_22DBM_BITS (0x00u) /* bit1-0 = 00 = tavan */
#define LORA_REG1_POWER_MASK (0x03u)
#define LORA_REG1_IMAGE (LORA_REG1_SUBPKT_200B_BITS | LORA_REG1_POWER_22DBM_BITS)
#define LORA_REG2_CH (0x17u)               /* CH23 = 873.125 MHz */
#define LORA_REG3_TRANSPARENT_BITS (0x00u)
/* Bilgi tek-noktasi (karar/olcum icin, kod uretmez): */
#define LORA_AIR_RATE_BPS (19200u)
#define LORA_TX_POWER_DBM (22u) /* donanim tavani; artirilamaz */
_Static_assert((LORA_REG0_UART_9600_BITS | LORA_REG0_PARITY_8N1_BITS |
                LORA_REG0_AIR_19K2_BITS) == LORA_REG0_9600,
               "REG0 9600 imaji bit kurulumuyla ayni olmali");
_Static_assert((LORA_REG0_UART_57600_BITS | LORA_REG0_PARITY_8N1_BITS |
                LORA_REG0_AIR_19K2_BITS) == LORA_REG0_57600,
               "REG0 57600 imaji bit kurulumuyla ayni olmali");
_Static_assert(LORA_REG1_IMAGE == 0x00u,
               "REG1 imaji 200B + 22dBm ile ayni olmali");

/* ---------------- Fixed-header fallback (datasheet §5.1) ----------------
 * When the peer module is stuck in fixed-transmission mode (REG3 bit6=1)
 * and register writes fail, its address filter drops every headerless
 * packet. Setting LORA_FIXED_HDR prepends the peer's {ADDH, ADDL, CH}
 * so its filter accepts our packets. The 36-byte core frame (and its CRC)
 * is unchanged; the air image is 39 bytes. The receiver's magic sync
 * skips the 3 header bytes automatically. Values below match a peer at
 * address 0x0002, channel 0x17. Set to 0 for pure transparent operation. */
#define LORA_FIXED_HDR (1u)
#define LORA_FIXED_HDR_0 (0x00u)
#define LORA_FIXED_HDR_1 (0x02u)
#define LORA_FIXED_HDR_2 (0x17u)
#define LORA_AIR_LEN (LORA_FRAME_LEN + 3u)

/* ---------------- Link-mode gate ---------------- */
#define LORA_ALLOW_SIT_SUT (0u)

/* Keystroke mirror of the private `ek7mode_t` enum in Src/main.c. main.c
 * cannot share the enum with this translation unit, so the numeric values are
 * restated here and cross-checked in tests/host/test_lora_host.c. */
#define LORA_EK7MODE_NORMAL (0u)
#define LORA_EK7MODE_SIT (1u)
#define LORA_EK7MODE_SUT (2u)

/* Flight is in FLIGHT_FAULT: keep transmitting. LoRa is strictly a reader, and
 * the fault packet is the single most useful one in the whole flight. */
#define LORA_SEND_IN_FAULT (1u)

/* ---------------- Packed wire length ---------------- */
_Static_assert(LORA_FRAME_LEN == 36u, "LoRa frame must be exactly 36 bytes");
_Static_assert(LORA_OFF_CRC + 1u == LORA_FRAME_LEN,
               "CRC byte must be the last byte of the frame");
_Static_assert(LORA_OFF_TILT == 29u, "tilt offset must be 29");
_Static_assert(LORA_OFF_LEANX == 31u, "lean_x offset must be 31");
_Static_assert(LORA_OFF_LEANY == 33u, "lean_y offset must be 33");
_Static_assert(LORA_OFF_FLAGS + 1u == LORA_OFF_TILT,
               "flags byte must sit immediately before tilt");
/* Every field is contiguous with no gaps: each field's offset equals the
 * previous field's offset + width. */
_Static_assert(LORA_OFF_VER + 1u == LORA_OFF_SEQ, "ver/seq gap");
_Static_assert(LORA_OFF_SEQ + 2u == LORA_OFF_T_MS, "seq/t_ms gap");
_Static_assert(LORA_OFF_T_MS + 4u == LORA_OFF_FLIGHT_STATE, "t_ms/state gap");
_Static_assert(LORA_OFF_FLIGHT_STATE + 2u == LORA_OFF_EK7_MODE, "state/mode gap");
_Static_assert(LORA_OFF_EK7_MODE + 2u == LORA_OFF_ALT_M, "mode/alt gap");
_Static_assert(LORA_OFF_ALT_M + 2u == LORA_OFF_VEL_CMS, "alt/vel gap");
_Static_assert(LORA_OFF_VEL_CMS + 2u == LORA_OFF_SATS, "vel/sats gap");
_Static_assert(LORA_OFF_SATS + 2u == LORA_OFF_LAT_E5, "sats/lat gap");
_Static_assert(LORA_OFF_LAT_E5 + 4u == LORA_OFF_LON_E5, "lat/lon gap");
_Static_assert(LORA_OFF_LON_E5 + 4u == LORA_OFF_BATT_MV, "lon/batt gap");
_Static_assert(LORA_OFF_BATT_MV + 2u == LORA_OFF_FLAGS, "batt/flags gap");
/* The altitude sentinel must be outside the representable real range so a
 * real reading can never be mistaken for "no altitude". */
_Static_assert(LORA_ALT_INVALID == (int16_t)0x8000 &&
                   LORA_ALT_INVALID < (int16_t)-32767,
               "altitude sentinel must be outside the real range");

/* ---------------- Plain, HAL-free frame description ----------------
 * Every field is already in its WIRE UNIT (metres, cm/s, 1e-5 degrees,
 * millivolts) and every "no data" case is already a sentinel, so this struct
 * can be filled by a host test and by the target alike. No packing attributes:
 * lora_build() writes bytes explicitly, so compiler layout is irrelevant. */
typedef struct
{
  uint16_t seq;
  uint32_t t_ms;
  uint16_t flight_state;
  uint16_t ek7_mode;
  int16_t alt_m; /* WHOLE metres, -32767..+32767 */
  int16_t vel_cms;
  uint16_t sats;
  int32_t lat_e5;
  int32_t lon_e5;
  uint16_t batt_mv;
  uint8_t flags;
  uint16_t tilt_ddeg; /* decidegree 0..1800; 0xFFFF = gecersiz */
  int16_t lean_x;     /* govde X x1000 (-1000..1000); gecersizken 0 */
  int16_t lean_y;     /* govde Y x1000 (-1000..1000); gecersizken 0 */
} lora_frame_t;

/* Raw telemetry as the caller has it (metres, m/s, validity). Converted by
 * lora_frame_from_tlm(); keeping the two apart is what lets the encoder be
 * unit-tested on the host with no HAL and no float in the byte layout. */
typedef struct
{
  uint16_t flight_state;
  uint16_t ek7_mode;
  bool alt_valid;
  float alt_m;
  bool vel_valid;
  float vel_mps;
  bool gps_fix;
  uint16_t sats;
  int32_t lat_e5;
  int32_t lon_e5;
  uint16_t batt_mv;
  uint8_t flags;
  /* Egiklik: derece cinsinden aci (0 = dikey), tilt_valid false ise yok
   * sayilir. lean_x/y: normalize filtreli-ivme X/Y x1000 (govde ekseni,
   * yaw YOK — arayuz "yaklasik yonelim (ivmeolcer, yaw yok)" yazmalidir). */
  bool tilt_valid;
  float tilt_deg;
  float lean_x;
  float lean_y;
} lora_tlm_t;

/* =================== HAL-free core (host-testable) =================== */

/* Additive checksum, mod 256. Same rule as ek7_sum(). */
uint8_t lora_crc(const uint8_t *d, size_t n);

/* Encode in -> out. Returns LORA_FRAME_LEN, or 0 if out/in is NULL or
 * cap < LORA_FRAME_LEN (nothing is written in that case). Writes EXACTLY
 * out[0..35]; never touches out[36..cap). */
size_t lora_build(uint8_t *out, size_t cap, const lora_frame_t *in);

/* Encode in -> out as the 39-byte air image (fixed header + 36-byte
 * core). Returns LORA_AIR_LEN, or 0 if out/in NULL or cap too small.
 * Used when LORA_FIXED_HDR is 1; lora_build() stays the 36-byte core. */
size_t lora_build_air(uint8_t *out, size_t cap, const lora_frame_t *in);

/* Decode + validate out of f. Returns true only when n == LORA_FRAME_LEN,
 * magic and version match and the CRC residual is zero. */
bool lora_parse(const uint8_t *f, size_t n, lora_frame_t *out);

/* True when f is LORA_FRAME_LEN bytes with correct magic, version and CRC. */
bool lora_verify(const uint8_t *f, size_t n);

/* Map raw telemetry + tick + seq onto the wire fields (metres -> whole
 * metres, m/s -> cm/s, validity -> sentinels). HAL-free. */
void lora_frame_from_tlm(lora_frame_t *out, uint16_t seq, uint32_t t_ms,
                         const lora_tlm_t *in);

/* Cadence period for a flight state, in ms (pure). */
uint32_t lora_interval_ms(uint16_t flight_state);

/* True when the link-mode gate permits a packet for this ek7_mode (pure). */
bool lora_link_allowed(uint16_t ek7_mode);

/* ---------------- Mode / config pins (target only) ----------------
 * PB10 (D6) = M0, PA8 (D7) = M1, PA9 (D8) = AUX. All three are free in
 * Src/ (PB10/PB11 untouched, PA8/PA9 unused; PA10 is the blue LED and is
 * NOT used). MX_GPIO_Init() in Src/main.c owns the init; the config
 * engine below only drives/reads them. */
#define LORA_M0_GPIO_Port GPIOB
#define LORA_M0_Pin GPIO_PIN_10
#define LORA_M1_GPIO_Port GPIOA
#define LORA_M1_Pin GPIO_PIN_8
#define LORA_AUX_GPIO_Port GPIOA
#define LORA_AUX_Pin GPIO_PIN_9

/* ---------------- Module register image (transparent mode) ----------------
 * Read with C1 00 06, written with C2 00 08 + 8 bytes every boot
 * (RAM-only, see lora_cfg_boot). ADDH/ADDL = 0x0000, REG1 = 0x00
 * (200 B sub-packet, no ambient RSSI, 22 dBm), REG2 = 0x17 (CH 23 =
 * 850.125 + 23 = 873.125 MHz factory default), REG3 = 0x00
 * (transparent, no RSSI byte, no LBT): guc/kanal DEGILMEZ; hava hizi
 * 19,2 kbps'tir (REG0 bit2-0 = 101, Tablo 7-2).
 * Yalniz REG0 UART bitleri hedeflenir: HI imaj (0xC5 = 57600 8N1, 19,2k
 * hava) yazilir ve 57600'de geri okunur; dogrulama tutmazsa EXPECTED
 * (0x65 = 9600 8N1, 19,2k hava) geri-donus imaji yazilir. Iki imaj da
 * yalniz REG0'da ayrilir (diger 5 bayt bit-bit ayni).
 * Kor urtak donus: geri-donus yazisi/dogrulamasi da tutmazsa (FAILED)
 * ve Mode-3 diyalogu kanitliysa urtak imaj (0x65: 9600 + 19,2k, karsi
 * modulle ayni hava) tek seferlik C2 ile kor yazilir (W=2); HI okunursa
 * huart yazisiz o baud'a alinir (W=3). Imza/alan sirasi/stage anlami
 * degismez. */
#define LORA_CFG_ADDR (0x00u)
#define LORA_CFG_LEN (6u)
extern const uint8_t LORA_CFG_EXPECTED[LORA_CFG_LEN];    /* 9600 geri-donus */
extern const uint8_t LORA_CFG_EXPECTED_HI[LORA_CFG_LEN]; /* 57600 hedef */

/* Dogrulanmis 6 baytlik imajdan link baud karari (saf, HAL'siz, hedef+host
 * ortak): HI imajina tam eslesme -> LORA_LINK_BAUD (57600); her baska durum
 * (9600 imaji, bozuk imaj, NULL) -> LORA_LINK_BAUD_FALLBACK (9600). Hedef cfg
 * motoru da bu karari kullanir (yazi dogrulanmadan huart1 degismez), host
 * regresyon testi de; donanima dokunmaz, bekleme yok. */
uint32_t lora_cfg_verified_baud(const uint8_t *v6);

/* Kor urtak donus imaji (hedef + host ortak veri, HAL'siz): 00 00 65 00
 * 17 00 = 9600 8N1 + 19,2 kbps hava + 22 dBm + 873,125 MHz + seffaf.
 * Karsi modulle (Uno tarafi) ayni HAVA'da olan TEK guvenli imaj budur.
 * Boot onarimi, dogrulama imkansizken (RX olu) ve Mode-3 diyalogu
 * ayni boot'ta kanitliyken bunu kor yazar (C2, best-effort); yoksa yazmaz.
 * Saf tasiyici: dialog true ise out8'e 8 baytlik yazma govdesini kurup
 * true doner (06H/07H=0 korunur), false ise tampona dokunmaz. */
extern const uint8_t LORA_CFG_COMMON[LORA_CFG_LEN];
bool lora_cfg_common_img(uint8_t *out8, bool dialog);

/* ---------------- Kesif-onarim siniflandirmasi (saf, HAL'siz) ----------------
 * OLAY: yankisi gorulmeyen bir C0/C2 yazisi modulde TUTMUS olabilir (RX teli
 * bozukken echo kaybolur ama modul yeni imaja gecer). Motor "eski imajda
 * kaldi" varsayip 9600'den yazmaya devam ederse modul (orn. 57600'de) sagir
 * kalir: link olu. Bu uclu, yazma-dogrulama tutmadiginda calisan kesif
 * kararinin HAL'siz cekirdegidir (hedef cfg motoru + host regresyonu ayni
 * karari kullanir; donanima dokunmaz, bekleme yok):
 *   lora_cfg_classify: imaji HI(0)/LO(1)/OTHER(2)/NONE(3: NULL) dizer.
 *   lora_cfg_baud_known: 8'li tarama listesindeki baud mu (kanitlanmis
 *     degisim baud'u mu).
 *   lora_cfg_needs_repair: HI disi her sey (LO/OTHER/NULL) onarim ister.
 *   lora_cfg_found_link_baud: durust link baud'u. KANITLANMIS degisim
 *     (found_baud) her zaman kazanir (modulun gercekten dinledigi baud
 *     odur); tarama-disi/sifir baud'da imaj konusmasi gecerlidir (HI->57600,
 *     LO->9600), hicbiri yoksa 9600 geri-donus. */
#define LORA_CFG_IMG_HI (0u)
#define LORA_CFG_IMG_LO (1u)
#define LORA_CFG_IMG_OTHER (2u)
#define LORA_CFG_IMG_NONE (3u)
uint8_t lora_cfg_classify(const uint8_t *v6);
bool lora_cfg_baud_known(uint32_t b);
bool lora_cfg_needs_repair(const uint8_t *v6);
uint32_t lora_cfg_found_link_baud(const uint8_t *img6, uint32_t found_baud);

/* ---------------- Boot-dayaniklilik karar yuvalari (saf, HAL'siz) --------
 * Hedef cfg motoru + host regresyonu AYNI karari kullanir; donanima dokunmaz,
 * bekleme yok, hepsi boot-fazi. 65-imaji = LORA_CFG_EXPECTED (9600 + 19,2k).
 *   LORA_CFG_SCAN_ROUNDS: C1-taramasi en fazla bu kadar tur dener (tur arasi
 *     Mode-0 + 100 ms). Ilk tutan kazanir; hedef dongu bu sayiyi kullanir.
 *   lora_cfg_scan_retry: tur-karari. hit=true ise false (dur, kazandin);
 *     hit=false ve done<ROUNDS ise true (bir tur daha dene), yoksa false.
 *     done = bitmis tur sayisi (0..ROUNDS).
 *   lora_cfg_needs_persist: ilk tarama okumasi 65-imajindan FARKLIYSA true
 *     (modul irasamis, C0 adayi); esitse/NULL ise false (flash omru: eslesen
 *     imaja C0 YAZILMAZ; okuma yoksa yazilacak kanit yok).
 *   lora_cfg_persist_now: C0 kapisi. mismatch && c2_ok ise true (C2-RAM
 *     yaz+dog rula BASARILI olduktan SONRA ayni imaj bir kez C0 ile yazilir
 *     ve tekrar okunur); yoksa false (sessizce C2-RAM'de kal, link yolu ayni).
 * 57600/C5 dali YOKTUR; huart hep 9600. */
#define LORA_CFG_SCAN_ROUNDS (3u)
bool lora_cfg_scan_retry(bool hit, uint8_t done);
bool lora_cfg_needs_persist(const uint8_t *first6);
bool lora_cfg_persist_now(bool mismatch, bool c2_ok);

/* Boot-time config result for the TUI (target fills, main.c prints). */
typedef enum
{
  LORA_CFG_SKIPPED = 0, /* link not ready: UART init failed */
  LORA_CFG_OK,          /* read-back already matched, nothing written */
  LORA_CFG_FIXED,       /* mismatch found, rewrite verified */
  LORA_CFG_FAILED       /* AUX timeout / no answer / verify failed */
} lora_cfg_result_t;

lora_cfg_result_t lora_cfg_boot(void);
const char *lora_cfg_str(lora_cfg_result_t r);
/* Boot-config diagnosis: last read-back (valid if *have), AUX level at
 * exit (0 = low, 1 = high, 2 = unknown), baud of the successful Mode 3
 * exchange (0 = none, else 9600/57600), stage (0 = none, 1 = read fail,
 * 2 = echo fail, 3 = verify fail, 4 = ok), variant (0 = 57600 tam imaj
 * C2 00 08, 1 = 9600 geri-donus tam imaj (65-imaji), 2/3 = kaldirilan
 * kesif kollari (uretilmez; TUI geriye uyumu icin lejantta durur),
 * FF = none). Imza ve alan sirasi sabittir (boot TUI formati bozulmaz).
 * Target-only, boot phase. */
void lora_cfg_diag(uint8_t *out6, bool *have, uint8_t *aux_level,
                   uint32_t *baud_used, uint8_t *stage, uint8_t *variant);

/* =================== Target-only link driver =================== */

/* Record the MX_USART1_UART_Init() result. Anything other than success simply
 * leaves the link not-ready; the flight logic is untouched and nothing here
 * can raise a fault. */
void lora_init(lora_status_t status);

/* True when the UART came up and the link may transmit. */
bool lora_ready(void);

/* Next sequence number that will be used (0 before the first packet). */
uint16_t lora_seq(void);

/* Packets successfully handed to the UART since lora_init(). */
uint32_t lora_ok_count(void);

/* AUX-low samples right after due transmits (see s_aux_busy). */
uint32_t lora_aux_busy(void);

/* TX-oncesi AUX-atlama sayaci (hedef + host ortak): vade dolmusken AUX
 * dusuk gorunup paket havaya verilmeden atlandiysa +1. drop'tan (build) ve
 * txerr'den (UART hatasi) AYRI tutulur; lora_drop_count() geriye uyumlu
 * TOPLAM dondurmeye devam eder (s_drop + s_txerr), bu sayac toplama
 * KATILMAZ. main.c'ye dokunulamadigi icin TUI "LORA:" satirina henuz
 * eklenemez; yalniz bu getter + belge vardir. */
uint32_t lora_airbusy_count(void);

/* Su anki link baud'u (hedef + host ortak): lora_init() sonrasi FALLBACK
 * (9600); hedefte cfg 57600 imaji dogrulanirsa 57600. Kesif-onarim modul
 * baska bir tarama baud'unda bulup onarimi tutmazsa huart bulunan baud'a
 * uydurulur ve bu da bulunan baud'u dondurur (durust rapor: diag B= ile
 * ayni). Re-init her zaman bu baud'u kullanir; yazisiz degismez. */
uint32_t lora_link_baud(void);

/* Kayip-paket sayaclari (hedef + host ortak):
 *  - s_drop  : SADECE build reddi (lora_build_air tampona sigmazsa).
 *    Tampon hep LORA_AIR_LEN oldugu icin pratikte sifir kalir, ama tani
 *    (build) ile hat (TX) ayrimini durust tutmak icin ayri sayilir.
 *  - s_txerr : HAL_UART_Transmit timeout/hata sayisi (kablo/modul/kip
 *    arizasi). Hedefte s_ok=0 + txerr tirmanisi = "baytlar modulu
 *    gormuyor" demektir; auxbusy de 0 ise tel/kip/suphelenilir.
 * lora_drop_count() GERIYE UYUM icin TOPLAM dondurur (s_drop + s_txerr):
 * main.c'deki "LORA: ... drop=" satiri eski anlamiyla (kaybolan paket)
 * calismaya devam eder, TUI formati bozulmaz. Ayristirilmis degerler
 * icin lora_txerr_count() kullanilir.
 * NOT (TUI kisiti): main.c'ye dokunulamiyor; "LORA:" satirina
 * txerr=/reinit= alani main.c degismeden eklenemez. Yeni sayaclar
 * TUI'da gorunene kadar yalniz bu getter'lar + belge vardir. */
uint32_t lora_drop_count(void);

/* TX timeout/hata sayaci (hedefte kablo/modul gostergesi, hostta
 * lora_host_inject_txerr ile surulen ayni muhasebe). */
uint32_t lora_txerr_count(void);

/* Ardisik-K re-init sayaci: huart1 kac kez abort + DeInit/Init ile
 * dogrulanmis link baud'unda yeniden baslatildi. */
uint32_t lora_reinit_count(void);

/* One cadence tick. Builds at most ONE packet and transmits it with a bounded
 * LORA_TX_TMO_MS timeout. Returns true if a packet was transmitted.
 * `tx`/`cap` are an optional caller buffer (NULL = use the internal one);
 * main.c passes its own static buffer so the stack cost stays at zero.
 * Never busy-waits, never disables interrupts, never writes flight state. */
bool lora_poll(uint32_t now_ms, const lora_tlm_t *t, uint8_t *tx, size_t cap);

#if !defined(LORA_WITH_HAL)
/* Yalniz host testleri icin ariza-enjeksiyonu (hedefte YOK): hedefteki
 * HAL_UART_Transmit-hatasiyla AYNI muhasebeyi surer (s_txerr +1, ardisik
 * sayac, esikte re-init sayaci +1) ama donanima dokunmaz, kadans/vade/seq
 * degistirmez. Amaci: re-init esigini (K) HAL'siz regresyon testiyle
 * kilitlemek. Uretim yolunda cagrilmaz; sahte veri uretmez. */
void lora_host_inject_txerr(void);

/* Yalniz host testleri icin AUX-seviye enjeksiyonu (hedefte YOK): hedefteki
 * gercek AUX pini yerine poll-oncesi atlama kararini surer. low=true iken
 * vade dolmus poll paketi gondermez, s_airbusy +1 artar (hedefle ayni
 * muhasebe); low=false iken normal gonderir. Kadans/vade karari degistirmez,
 * sahte veri uretmez; varsayilan false (AUX yuksek = modul bos). */
void lora_host_set_aux_low(bool low);
#endif

#ifdef __cplusplus
}
#endif

#endif /* LORA_H */