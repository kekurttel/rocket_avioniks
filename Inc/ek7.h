/* Ek-7 (TEKNOFEST O-UKB testleri) haberlesme protokolu: komut cozucu,
 * telemetri/sentetik paket insasi, durum bitleri, eksen esleme.
 *
 * HAL-independent, deterministic, host-testable. Kapsanan Ek-7 ogeleri:
 * - Tablo 1 komut seti: 0xAA CMD SUM 0x0D 0x0A (Big Endian sira).
 *   Calisan test yazilimi Bolum 3 toplama kuralini kullanir:
 *   SIT=0x20/0xCA, SUT=0x22/0xCC, STOP=0x24/0xCE. Dokuman-literal
 *   Tablo 1 degerleri (0x8C/0x8E/0x90) geriye uyumluluk icin kabul edilir.
 * - Tablo 2/3/4 telemetri: 0xAB + 8xFLOAT32(BE) + SUM + 0x0D 0x0A (36 B),
 *   SUM = header + 32 bayt (mod 256).
 * - Tablo 5/6 durum paketi: 0xAA D1(bit0-7) D2(bit8-15) SUM 0x0D 0x0A (6 B),
 *   SUM = header + D1 + D2 (mod 256), 10 Hz.
 * - Bolum 3 checksum: toplam mod 256.
 * - Bolum 1.2 govde ekseni: X burun, Y sag, Z ASAGI (yeryuzune dogru).
 *
 * Belgelenmis sapmalar (rapor + SIT_SUT_REQUIREMENTS.md):
 * D1 (COZULDU): Tablo 1 literal checksum (8C/8E/90) ile calisan yazilimin
 *     Bolum-3 toplama checksum'u (CA/CC/CE) celisiyordu. Cozum: ikisi de
 *     kabul edilir; resmi trafik toplama kuraliyla uretilir/dogrulanir.
 * D2: telemetri Tablo 3 "Byte 34" sayimi vs Tablo 2 alan yapisi (8 float);
 *     alan yapisi esas alinir -> 36 byte cerceve.
 * D3: Tablo 6 ornek checksum (0x1A) toplama kuraliyla tutarsiz (ne header'li
 *     ne headersiz toplami veriyor); ornek degeri yoksayilir, calisan
 *     yazilimin dogruladigi kural (header dahil toplam) uygulanir.
 * D4: irtifa "deniz seviyesine gore" istenir; elimizde yer-referansli
 *     bagil irtifa var (MSL referansi yok) -> bagil irtifa + not.
 * D5: Bit1 (motor yanma suresi) tasarimda yok -> her zaman 0 + not.
 */
#ifndef EK7_H
#define EK7_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flight.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EK7_CMD_HDR (0xAAu)
#define EK7_SYN_HDR (0xABu)
#define EK7_FTR1 (0x0Du)
#define EK7_FTR2 (0x0Au)

#define EK7_CMD_SIT (0x20u)
#define EK7_CMD_SUT (0x22u)
#define EK7_CMD_STOP (0x24u)

#define EK7_SUM_SIT (0x8Cu)
#define EK7_SUM_SUT (0x8Eu)
#define EK7_SUM_STOP (0x90u)

#define EK7_CMD_LEN (5u)
#define EK7_TLM_LEN (36u)    /* 1 + 8*4 + 1 + 2 (Tablo 2 alan yapisi) */
#define EK7_STATUS_LEN (6u)  /* AA D1 D2 SUM 0D 0A */
#define EK7_NFIELD (8u)      /* alt, press, ax, ay, az, ex, ey, ez */

/* Tablo 5 durum bitleri (bit 8-15 rezerve = 0). */
#define EK7_ST_LAUNCH (1u << 0)
#define EK7_ST_BURNOUT (1u << 1) /* desteklenmiyor -> 0 (D5) */
#define EK7_ST_MINALT (1u << 2)
#define EK7_ST_ANGLE (1u << 3)
#define EK7_ST_DESCENT (1u << 4)
#define EK7_ST_DROGUE (1u << 5)
#define EK7_ST_BELOWMAIN (1u << 6)
#define EK7_ST_MAIN (1u << 7)

/* MPU6050 reset-default +-2g nominal olcegi (ACCEL_CONFIG yazilmiyor).
 * Canli olcumde sensor ~21080 LSB/g verdi (+%28); bu yuzden telemetri,
 * startup kalibrasyonundaki |g_ref|'i 1 g sayar (self-calibrating).
 * Karar mantigi zaten oran tabanli oldugundan etkilenmez. */
#define EK7_MPU_LSB_PER_G (16384.0f)
#define EK7_MS2_PER_G (9.80665f)

typedef enum
{
  EK7_EV_NONE = 0,
  EK7_EV_SIT,
  EK7_EV_SUT,
  EK7_EV_STOP
} ek7_cmd_t;

typedef struct
{
  uint8_t buf[EK7_CMD_LEN];
  uint8_t n; /* biriken bayt sayisi */
} ek7_rx_t;

void ek7_rx_init(ek7_rx_t *r);
/* Bir bayt besle; 5 bayt tamamlaninca true + *out dolar (NONE: gecersiz).
 * Senkron kaybi durumunda header'dan yeniden baslar. */
bool ek7_rx_feed(ek7_rx_t *r, uint8_t b, ek7_cmd_t *out);

/* Bolum 3: bayt toplami mod 256. */
uint8_t ek7_sum(const uint8_t *d, size_t n);

/* FLOAT32 -> 2-ondalik yuvarlanmis big-endian 4 bayt; tersi. */
void ek7_pack_f32be(float v, uint8_t out[4]);
float ek7_unpack_f32be(const uint8_t in[4]);

/* 36 bayt telemetri/sentetik cerceve kur (0xAB ... SUM 0D 0A). Donus: 36.
 * press_mbar biriminde basinç. */
size_t ek7_build_tlm(uint8_t out[EK7_TLM_LEN], float alt_m, float press_mbar,
                     float ax, float ay, float az, float ex, float ey, float ez);
/* 36 bayt cerceveyi coz (header/checksum/footer dogrulanir). */
bool ek7_parse_tlm(const uint8_t in[EK7_TLM_LEN], float *alt_m,
                   float *press_mbar, float *ax, float *ay, float *az,
                   float *ex, float *ey, float *ez);

/* 6 bayt durum cercevesi kur (0xAA D1 D2 SUM 0D 0A). Donus: 6. */
size_t ek7_build_status(uint8_t out[EK7_STATUS_LEN], uint16_t st);

/* Tablo 5 durum kelimesi: flight state + tilt bandi + irtifa.
 * bit0: state>=ASCENT | bit1: 0 (D5) | bit2: max>=apogee_min |
 * bit3: tilt_band | bit4: state>=APOGEE_CONFIRMED | bit5: drogue |
 * bit6: alt_valid && alt<main_alt | bit7: main. NULL-safe (0 doner). */
uint16_t ek7_status_word(const flight_t *f, bool tilt_band, bool alt_valid,
                         float alt_m);

/* Eksen esleme (Bolum 1.2): bench varsayimi = kart duz, MPU +Z yukari.
 * acc_ek7 = (+ax, +ay, -az) m/s2; olcek = startup |g_ref| (1 g karsiligi),
 * gref_mag<=0 ise nominal 16384 LSB/g. Aci = yercekimi-referansli eksen
 * egiklikleri (derece), yaw = 0 (manyetometre yok). Saf, test edilebilir. */
void ek7_map_acc(float ax_lsb, float ay_lsb, float az_lsb, float gref_mag,
                 float *ox, float *oy, float *oz);
void ek7_map_ang(float ax, float ay, float az,
                 float *ex, float *ey, float *ez);

#ifdef __cplusplus
}
#endif

#endif /* EK7_H */
