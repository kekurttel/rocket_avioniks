/* Minimal real-data NMEA GPS driver (NEO-6M @ 9600 8N1).
 *
 * HAL-independent byte-stream parser: the caller feeds raw UART bytes with a
 * millisecond timestamp (HAL_GetTick on target); one NMEA sentence may
 * arrive in arbitrarily many chunks. No dynamic allocation, fixed buffers,
 * no heavyweight library. Supports GGA + RMC (position/time/speed/course);
 * any other checksum-valid NMEA sentence only marks the module present.
 *
 * Data-validity contract (never treat garbage as flight data):
 * - present: at least one checksum-valid NMEA sentence seen this boot.
 * - fix/valid: last fix came from a checksum-valid GGA (quality != '0') or
 *   RMC (status == 'A') with in-range coordinates, AND is not stale.
 * - stale: (now_ms - last_fix_ms) > GPS_STALE_MS. The last fix is NEVER
 *   kept forever; callers must use gps_has_fix(), not raw fields.
 * - Malformed sentences, bad checksums, incomplete lines, empty required
 *   fields and no-fix sentences update presence bookkeeping only.
 *
 * Coordinates are exposed as int32_t 1e-5 degrees (UBX-style): exact,
 * printf-free, no float formatting needed for telemetry.
 */
#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NMEA sentences are <= 82 chars; 128 leaves margin for line noise. */
#define GPS_LINE_MAX (128u)
/* NEO-6M emits at 1 Hz: 3 missed fixes = stale. Single explicit place. */
#define GPS_STALE_MS (3000u)
/* Knots -> m/s for RMC ground speed. */
#define GPS_KNOTS_TO_MPS (0.514444f)
/* UBX reply payload kept for ACK/NAK (2 bytes used; longer frames skipped). */
#define GPS_UBX_PL_MAX (4u)

typedef struct
{
  /* Presence / validity (see contract above). */
  bool present;      /* any checksum-valid NMEA sentence seen this boot */
  bool fix;          /* last fix sentence reported a valid fix */
  bool valid;        /* fix && coordinates parsed && in range */
  uint8_t sats;      /* satellites in use (GGA), 0 when unknown */
  /* Position: signed decimal degrees * 1e-5. Meaningful only if valid. */
  int32_t lat_e5;
  int32_t lon_e5;
  /* Altitude (GGA field 9, metres). */
  float alt_m;
  bool alt_valid;
  /* Ground speed (RMC, m/s) and course (RMC, degrees). */
  float speed_mps;
  bool speed_valid;
  float course_deg;
  bool course_valid;
  /* UTC time/date when available. */
  uint8_t hour, min, sec;
  bool time_valid;
  uint8_t day, mon, year; /* RMC date DDMMYY, year = 00-99 */
  bool date_valid;
  /* Bookkeeping. */
  uint32_t last_update_ms; /* last checksum-valid sentence (any type) */
  uint32_t last_fix_ms;    /* last valid fix (GGA/RMC) */
  uint32_t rx_bytes;       /* raw bytes fed (diagnostic) */
  /* UBX reply tracker: the module answers each of OUR config frames with
   * UBX-ACK-ACK / UBX-ACK-NAK (binary, shares the NMEA line). Parsed by a
   * tiny side state machine so it never disturbs the NMEA assembler.
   * ack_seq increments on every valid ACK/NAK; ack_cls/ack_id say which
   * config frame it answers, ack_ok is true for ACK, false for NAK. */
  uint8_t ubx_state;
  uint8_t ubx_cls, ubx_id, ubx_ck_a, ubx_ck_b;
  uint16_t ubx_len, ubx_idx;
  uint8_t ubx_pl[GPS_UBX_PL_MAX];
  uint8_t ack_seq;
  uint8_t ack_cls, ack_id;
  bool ack_ok;
  uint32_t ack_count, nak_count;
  /* Line assembler state (private, but kept inline: no heap). */
  char line[GPS_LINE_MAX];
  uint8_t n;
  bool overflow; /* line exceeded buffer: dropping until '\n' */
} gps_t;

void gps_init(gps_t *g);

/* Feed one UART byte with its arrival timestamp. Never blocks, never fails
 * visibly (bad input is silently dropped per the validity contract). */
void gps_rx_byte(gps_t *g, uint8_t b, uint32_t now_ms);

/* True when (now_ms - last_fix_ms) exceeds GPS_STALE_MS. Unsigned-safe. */
bool gps_is_stale(const gps_t *g, uint32_t now_ms);

/* Usable position fix right now: parsed, in range, and fresh. */
static inline bool gps_has_fix(const gps_t *g, uint32_t now_ms)
{
  return (g != NULL) && g->present && g->valid && g->fix &&
         !gps_is_stale(g, now_ms);
}

/* ===================== GPS configuration (board TX -> GPS RX) =====================
 * One-time, boot-time, PAD-ONLY setup of the NEO-6M over UBX. Never part of
 * flight: the scheduler refuses to start/continue once `allowed` is false
 * (caller passes false after launch), and GPS data never feeds launch/apogee/
 * deployment (unchanged). A GPS that is absent or does not answer is NOT a
 * fault: the sequence just ends in NOACK and the board carries on.
 *
 * Steps (in order): GGA on, RMC on, GLL/GSA/GSV/VTG off (bandwidth at
 * 9600 baud), dynamic model Airborne <4g (default "Portable" is wrong for a
 * rocket), update rate GPS_CFG_RATE_MS. Rate last, so every earlier ACK
 * arrives at the old, slow rate. Settings are NOT saved to the module's
 * flash/BBR: they are re-applied on every boot (stateless, always known).
 * ===================================================================== */
#define GPS_CFG_RATE_MS (1000u)     /* 1 Hz: NEO-6 is a 1 Hz part (gen6 CFG-RATE floor = 1000 ms); GGA+RMC ~170 B/s of 960 B/s */
#define GPS_CFG_DYNMODEL (7u)       /* gen6: 7 = Airborne <4g (3-bit field, 0..7) */
#define GPS_CFG_STEPS (8u)
#define GPS_CFG_ACK_TMO_MS (500u)   /* wait for ACK/NAK per try */
#define GPS_CFG_TRIES (3u)          /* sends per step before giving up */
#define GPS_CFG_GAP_MS (100u)       /* pause between steps */
#define GPS_UBX_MAX_FRAME (44u)     /* biggest frame we build (NAV5) */

/* Ölü NOACK/NAK dizisinin pad başında özerk yeniden denenmesi (main.c retry
 * bloğu): üstel geri çekilme, tavan sınırlı, tur sayacı üst sınırlı. Kopuk
 * kablo hattı veri yolunu şişirmez (tur başına en fazla STEPS*TRIES çerçeve),
 * sayaç dolunca sessizce NOACK/NAK'da kalınır: asla fault üretilmez, uçuş
 * mantığı etkilenmez. TUI'daki "GPSCFG: NOACK n/N" sayacı acked/STEPS'ten
 * gelir; her tur gps_cfg_init ile dürüstçe sıfırlanır (format değişmez). */
#define GPS_CFG_RETRY_BASE_MS (120000u) /* ilk yeniden deneme: 120 sn */
#define GPS_CFG_RETRY_MAX_MS (480000u)  /* aralık tavanı: 480 sn */
#define GPS_CFG_RETRY_MAX_N (15u)       /* en fazla 15 ölü tur (~110 dk pad) */

/* Yeniden deneme aralığı: BASE << round, MAX'ta tavanlı. Kaydırmasız hesap
 * (taşma yok); round doymuş olsa da sonuç hep MAX'tır. */
uint32_t gps_cfg_retry_gap_ms(uint8_t round);

typedef enum
{
  GPS_CFG_IDLE = 0, /* not started yet */
  GPS_CFG_SENDING,  /* in progress */
  GPS_CFG_OK,       /* every step ACKed */
  GPS_CFG_NAK,      /* module refused a step */
  GPS_CFG_NOACK,    /* no answer (GPS absent / RX wire not connected) */
  GPS_CFG_ABORTED   /* flight started (or not allowed) before it finished */
} gps_cfg_state_t;

typedef struct
{
  gps_cfg_state_t state;
  uint8_t step;         /* current step index */
  uint8_t tries;        /* sends done for the current step */
  uint8_t acked;        /* steps ACKed so far */
  bool waiting;         /* frame sent, waiting for ACK/NAK */
  uint8_t seq_at_send;  /* gps_t.ack_seq when the frame went out */
  uint32_t sent_ms;     /* time of the last send */
  uint32_t ready_ms;    /* do not send before this time (gap) */
} gps_cfg_t;

void gps_cfg_init(gps_cfg_t *c);

/* Raw UBX frame builders (return frame length, 0 if cap too small / bad
 * args). Frame = B5 62 cls id lenL lenH payload ckA ckB. */
size_t gps_ubx_cfg_msg(uint8_t *out, size_t cap, uint8_t msg_cls,
                       uint8_t msg_id, uint8_t rate);
size_t gps_ubx_cfg_rate(uint8_t *out, size_t cap, uint16_t meas_ms);
size_t gps_ubx_cfg_nav5_dyn(uint8_t *out, size_t cap, uint8_t dyn_model);

/* Frame for configuration step `step` (0..GPS_CFG_STEPS-1); 0 otherwise.
 * Also reports the UBX class/id the matching ACK will carry. */
size_t gps_cfg_frame(uint8_t step, uint8_t *out, size_t cap,
                     uint8_t *ack_cls, uint8_t *ack_id);

/* One scheduler tick (call every loop). Returns the frame length when the
 * caller must transmit out[] NOW (non-blocking), else 0.
 *   allowed: false once the rocket is no longer on the pad -> ABORTED.
 *   tx_busy: caller is still shifting out the previous frame. */
size_t gps_cfg_poll(gps_cfg_t *c, const gps_t *g, uint32_t now_ms,
                    bool allowed, bool tx_busy, uint8_t *out, size_t cap);

const char *gps_cfg_str(gps_cfg_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
