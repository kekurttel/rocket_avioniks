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
 * registers are scanned (C1, at most three rounds, with Mode-0 recovery).
 * The common 00 00 63 00 17 00 image (9600 UART, 4.8 kbps RF) is applied via
 * C2 (C2 addr len params, manual §7.1), switched through Mode 0 and verified.
 * A differing saved image is persisted once with C0 after successful
 * verification. The live telemetry UART remains at 9600 8N1.
 * AUX is low when busy; use bounded waits before mode changes and config reads.
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
 * ---------------- WIRE FORMAT: V3, 38 bytes, little-endian ----------------
 * V1 (30 bytes) and V2 (36 bytes) remain readable; all new transmissions use
 * V3. V3 preserves the common telemetry but carries altitude as signed i32
 * decimetres (0.1 m), removing the old whole-metre precision loss.
 *
 * V3 offsets: 0 magic A5; 1 version 03; 2 seq u16; 4 tick_ms u32;
 * 8 flight_state u16; 10 ek7_mode u16; 12 alt_dm i32; 16 vel_cms i16;
 * 18 sats u16; 20 lat_e5 i32; 24 lon_e5 i32; 28 batt_mv u16; 30 flags u8;
 * 31 tilt_ddeg u16; 33 lean_x i16; 35 lean_y i16; 37 CRC u8 (sum 0..36).
 * All fields are written explicitly; no packed C struct reaches the wire.
 * Altitude is relative to the calibrated launch/reference pressure, NOT sea
 * level. INT32_MIN is the V3 altitude-invalid sentinel; V1/V2 retain i16.
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

/* ---------------- Protocol and current V3 wire offsets ---------------- */
#define LORA_MAGIC (0xA5u)
#define LORA_VER (0x03u)
#define LORA_VER_V1 (0x01u)
#define LORA_VER_V2 (0x02u)
#define LORA_VER_V3 (0x03u)
#define LORA_V1_FRAME_LEN (30u)
#define LORA_V2_FRAME_LEN (36u)
#define LORA_V3_FRAME_LEN (38u)
#define LORA_FRAME_LEN LORA_V3_FRAME_LEN

#define LORA_OFF_MAGIC (0u)
#define LORA_OFF_VER (1u)
#define LORA_OFF_SEQ (2u)
#define LORA_OFF_T_MS (4u)
#define LORA_OFF_FLIGHT_STATE (8u)
#define LORA_OFF_EK7_MODE (10u)
#define LORA_OFF_ALT_DM (12u)
#define LORA_OFF_ALT_M LORA_OFF_ALT_DM /* current V3 field offset; legacy parsing is versioned */
#define LORA_OFF_VEL_CMS (16u)
#define LORA_OFF_SATS (18u)
#define LORA_OFF_LAT_E5 (20u)
#define LORA_OFF_LON_E5 (24u)
#define LORA_OFF_BATT_MV (28u)
#define LORA_OFF_FLAGS (30u)
#define LORA_OFF_TILT (31u)
#define LORA_OFF_LEANX (33u)
#define LORA_OFF_LEANY (35u)
#define LORA_OFF_CRC (37u)
#define LORA_CRC_OFF LORA_OFF_CRC

/* ---------------- Sentinels ---------------- */
/* irtifa "veri yok": 0x8000 int16 olarak -32768'dir, yani gecerli
 * -32767..+32767 m araliginin DISINDADIR. lora_frame_from_tlm() gecerli
 * aralikta yarim-uzakliktan yuvarlar; gecersiz/NaN/inf/aralik-disi her
 * okuma bu gozcuyu verir (doyurma YOK: 32767'ye kelepcelenmis sahte irtifa
 * gercek okumadan ayirt edilemezdi). */
#define LORA_ALT_INVALID ((int16_t)0x8000) /* legacy V1/V2 whole metres */
#define LORA_ALT_DM_INVALID INT32_MIN /* V3 signed decimetres */
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
#define LORA_SEND_INTERVAL_MS (100u)   /* target cadence: 10 Hz */
#define LORA_SEND_INTERVAL_WAIT_MS (100u) /* pad cadence: 10 Hz */
#define LORA_SEND_GAP_MS (100u) /* one packet maximum per 100 ms deadline */
#define LORA_TX_TMO_MS (50u) /* bounded UART transfer at 9600 8N1 */
#define LORA_TXERR_REINIT_K (10u)

/* ---------------- Link UART baud (STM32 <-> E220) --------------------------
 * Both modules use 9600 8N1 UART. UART baud and RF air rate are separate
 * settings; changing one does not make the other side's RF rate compatible.
 */
#define LORA_LINK_BAUD (9600u)
#define LORA_LINK_BAUD_FALLBACK (9600u)
#define LORA_REG0_9600 (0x63u)   /* 9600 8N1 + 4.8 kbps RF */
#define LORA_REG0_57600 (0xC3u)  /* legacy alternate UART, same 4.8 kbps RF */

/* ---------------- RF settings: datasheet Table 7-2 ------------------------
 * Current common module image: 00 00 63 00 17 00.
 * REG0=0x63: UART 9600 8N1 and RF air rate 4.8 kbps (air-rate bits 011).
 * Both radios MUST use the same RF air rate. Ebyte explicitly states that a
 * higher air rate shortens transmission distance, so 4.8 kbps is selected as
 * a range/speed compromise for 10 Hz. 2.4 kbps is too slow for a 100 ms period.
 * REG1=0x00: 200-byte subpacket, RSSI ambient read disabled, 22 dBm MAX TX.
 * The hardware is already at its maximum output power; software cannot raise it.
 * REG2=0x17: channel 23 / 873.125 MHz. REG3=0x00: transparent mode, no RSSI
 * trailer, no LBT. Non-rate RF settings are intentionally unchanged.
 */
#define LORA_REG0_UART_9600_BITS (0x60u)
#define LORA_REG0_UART_57600_BITS (0xC0u)
#define LORA_REG0_UART_MASK (0xE0u)
#define LORA_REG0_PARITY_8N1_BITS (0x00u)
#define LORA_REG0_AIR_2K4_BITS (0x02u)
#define LORA_REG0_AIR_4K8_BITS (0x03u)
#define LORA_REG0_AIR_MASK (0x07u)
#define LORA_REG1_SUBPKT_200B_BITS (0x00u)
#define LORA_REG1_SUBPKT_MASK (0xC0u)
#define LORA_REG1_POWER_22DBM_BITS (0x00u)
#define LORA_REG1_POWER_MASK (0x03u)
#define LORA_REG1_IMAGE (LORA_REG1_SUBPKT_200B_BITS | LORA_REG1_POWER_22DBM_BITS)
#define LORA_REG2_CH (0x17u)
#define LORA_REG3_TRANSPARENT_BITS (0x00u)
#define LORA_AIR_RATE_BPS (4800u)
#define LORA_TX_POWER_DBM (22u)
_Static_assert((LORA_REG0_UART_9600_BITS | LORA_REG0_PARITY_8N1_BITS |
                LORA_REG0_AIR_4K8_BITS) == LORA_REG0_9600,
               "REG0 9600/4.8kbps image mismatch");
_Static_assert((LORA_REG0_UART_57600_BITS | LORA_REG0_PARITY_8N1_BITS |
                LORA_REG0_AIR_4K8_BITS) == LORA_REG0_57600,
               "REG0 57600/4.8kbps image mismatch");
_Static_assert(LORA_REG1_IMAGE == 0x00u,
               "REG1 must remain 200B + maximum 22dBm");

/* ---------------- Fixed-header fallback (datasheet §5.1) ----------------
 * When the peer module is stuck in fixed-transmission mode (REG3 bit6=1)
 * and register writes fail, its address filter drops every headerless
 * packet. Setting LORA_FIXED_HDR prepends the peer's {ADDH, ADDL, CH}
 * so its filter accepts our packets. The current V3 core frame is 38 bytes
 * plus the 3-byte header (41 bytes on air). The receiver magic sync skips the
 * header automatically. Values below match a peer at
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

/* ---------------- Wire layout static assertions --------------------------- */
_Static_assert(LORA_V1_FRAME_LEN == 30u, "V1 compatibility length changed");
_Static_assert(LORA_V2_FRAME_LEN == 36u, "V2 compatibility length changed");
_Static_assert(LORA_FRAME_LEN == 38u, "V3 frame must be exactly 38 bytes");
_Static_assert(LORA_OFF_CRC + 1u == LORA_FRAME_LEN, "V3 CRC must be final");
_Static_assert(LORA_OFF_VER + 1u == LORA_OFF_SEQ, "ver/seq gap");
_Static_assert(LORA_OFF_SEQ + 2u == LORA_OFF_T_MS, "seq/t_ms gap");
_Static_assert(LORA_OFF_T_MS + 4u == LORA_OFF_FLIGHT_STATE, "t_ms/state gap");
_Static_assert(LORA_OFF_FLIGHT_STATE + 2u == LORA_OFF_EK7_MODE, "state/mode gap");
_Static_assert(LORA_OFF_EK7_MODE + 2u == LORA_OFF_ALT_DM, "mode/alt gap");
_Static_assert(LORA_OFF_ALT_DM + 4u == LORA_OFF_VEL_CMS, "alt/vel gap");
_Static_assert(LORA_OFF_VEL_CMS + 2u == LORA_OFF_SATS, "vel/sats gap");
_Static_assert(LORA_OFF_SATS + 2u == LORA_OFF_LAT_E5, "sats/lat gap");
_Static_assert(LORA_OFF_LAT_E5 + 4u == LORA_OFF_LON_E5, "lat/lon gap");
_Static_assert(LORA_OFF_LON_E5 + 4u == LORA_OFF_BATT_MV, "lon/batt gap");
_Static_assert(LORA_OFF_BATT_MV + 2u == LORA_OFF_FLAGS, "batt/flags gap");
_Static_assert(LORA_OFF_FLAGS + 1u == LORA_OFF_TILT, "flags/tilt gap");
_Static_assert(LORA_OFF_TILT + 2u == LORA_OFF_LEANX, "tilt/leanx gap");
_Static_assert(LORA_OFF_LEANX + 2u == LORA_OFF_LEANY, "leanx/leany gap");
_Static_assert(LORA_ALT_DM_INVALID == INT32_MIN, "V3 altitude sentinel changed");

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
  int16_t alt_m; /* compatibility: rounded whole metres or legacy sentinel */
  int32_t alt_dm; /* V3 signed decimetres (0.1 m), INT32_MIN = invalid */
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

/* Raw telemetry as the caller has it (metres, m/s, validity); V3 altitude
 * retains 0.1 m resolution in the wire image. */
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

/* Encode current V3. Returns LORA_FRAME_LEN, or 0 if out/in is NULL or
 * cap < LORA_FRAME_LEN (nothing is written). Writes exactly out[0..37]. */
size_t lora_build(uint8_t *out, size_t cap, const lora_frame_t *in);

/* Encode the fixed-header air image (3-byte header + 38-byte V3 core = 41B).
 * Returns LORA_AIR_LEN, or 0 if out/in NULL or cap too small. */
size_t lora_build_air(uint8_t *out, size_t cap, const lora_frame_t *in);

/* Decode/validate V1 (30B), V2 (36B), or current V3 (38B). New transmissions
 * are V3; historical frames remain readable. CRC is the final byte per version. */
bool lora_parse(const uint8_t *f, size_t n, lora_frame_t *out);

/* True when a supported version has its exact frame length, magic and CRC. */
bool lora_verify(const uint8_t *f, size_t n);

/* Map raw telemetry + tick + seq to the wire fields (metres -> 0.1 m V3
 * altitude, m/s -> cm/s, validity -> sentinels). HAL-free. */
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
 * Read with C1 00 06. Current desired image is 00 00 63 00 17 00:
 * REG0=0x63 (9600 8N1 UART + 4.8 kbps RF), REG1=00 (200-byte subpacket,
 * 22 dBm maximum), REG2=17 (873.125 MHz), REG3=00 (transparent mode).
 * Persistent C0 frame and temporary C2 frame both carry a length byte
 * (C0/C2 addr len params, E220 manual §7.1). C2 is
 * verified after switching through Mode 0; C0 is written once on a verified
 * migration from a different saved image, not on every matching boot.
 */
#define LORA_CFG_ADDR (0x00u)
#define LORA_CFG_LEN (6u)
extern const uint8_t LORA_CFG_EXPECTED[LORA_CFG_LEN];    /* 9600 geri-donus */
extern const uint8_t LORA_CFG_EXPECTED_HI[LORA_CFG_LEN]; /* 57600 hedef */

/* Retained host-tested helper for legacy configuration-image diagnostics.
 * Production currently fixes the radio UART to 9600 8N1 and shares one RF
 * image, so this helper does not enable an unverified baud switch. */
uint32_t lora_cfg_verified_baud(const uint8_t *v6);

/* Kor urtak donus imaji (hedef + host ortak veri, HAL'siz): 00 00 63 00
 * 17 00 = 9600 8N1 + 4.8 kbps hava + 22 dBm + 873,125 MHz + seffaf.
 * Karsi modulle (Uno tarafi) ayni HAVA'da olan TEK guvenli imaj budur.
 * Boot onarimi, dogrulama imkansizken (RX olu) ve Mode-3 diyalogu
 * ayni boot'ta kanitliyken bunu kor yazar (C2, best-effort); yoksa yazmaz.
 * Saf tasiyici: dialog true ise out8'e 8 baytlik yazma govdesini kurup
 * true doner (06H/07H=0 korunur), false ise tampona dokunmaz. */
extern const uint8_t LORA_CFG_COMMON[LORA_CFG_LEN];
bool lora_cfg_common_img(uint8_t *out8, bool dialog);
/* Datasheet-accurate E220 write-frame builder, shared with host tests:
 * C0 = command + address + length + data; C2 = command + address + length
 * + data (manual §7.1, e.g. "C2 00 03 12 34 62").
 * Returns transmitted byte count, or 0 for invalid arguments/capacity. */
size_t lora_cfg_write_frame(uint8_t command, uint8_t addr, uint8_t len,
                            const uint8_t *data, uint8_t *out, size_t cap);

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
 * bekleme yok, hepsi boot-fazi. 63-imaji = LORA_CFG_EXPECTED (9600 + 4.8k).
 *   LORA_CFG_SCAN_ROUNDS: C1-taramasi en fazla bu kadar tur dener (tur arasi
 *     Mode-0 + 100 ms). Ilk tutan kazanir; hedef dongu bu sayiyi kullanir.
 *   lora_cfg_scan_retry: tur-karari. hit=true ise false (dur, kazandin);
 *     hit=false ve done<ROUNDS ise true (bir tur daha dene), yoksa false.
 *     done = bitmis tur sayisi (0..ROUNDS).
 *   lora_cfg_needs_persist: ilk tarama okumasi 63-imajindan FARKLIYSA true
 *     (modul irasamis, C0 adayi); esitse/NULL ise false (flash omru: eslesen
 *     imaja C0 YAZILMAZ; okuma yoksa yazilacak kanit yok).
 *   lora_cfg_persist_now: C0 kapisi. mismatch && c2_ok ise true (C2-RAM
 *     yaz+dog rula BASARILI olduktan SONRA ayni imaj bir kez C0 ile yazilir
 *     ve tekrar okunur); yoksa false (sessizce C2-RAM'de kal, link yolu ayni).
 * production never enables an alternate UART baud; huart stays 9600. */
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
 * C2 00 08, 1 = 9600 geri-donus tam imaj (63-imaji), 2/3 = kaldirilan
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