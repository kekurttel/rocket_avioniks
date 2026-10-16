/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "avionics.h"
#include "flight.h"
#include "ek7.h"
#include "gps.h"
#include "lora.h"
#include "runmode.h"
#include <string.h> /* memset for the LoRa telemetry snapshot */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR (0x68 << 1)
/* NOTE: no synthetic [0,0,16384] accel bias. Tilt uses the startup pose
   itself as reference (see gravity_ref); absolute sensor bias and the
   orientation reference are deliberately separate concepts. MPU stays at
   reset-default +-2g (ACCEL_CONFIG never written) but no code depends on
   the exact LSB/g value anymore. */
/* Tilt low-pass: exponential moving average over ~N samples (5 Hz loop).
   N=4 settles in ~1 s: fast enough for hand-held demo, still smooths tremor. */
#define TILT_EMA_N (4)
/* Pressure low-pass for ALT display/condition (P stays raw in STATUS).
   Single implementation: avt_ema() in avionics.c (alpha = 1/4). */
/* LEDs: onboard GREEN LD2/PA5 = flight/descent (bench apogee latched,
   extended to flight APOGEE+ latched), BLUE external PA10/D2 =
   BENCH_FALL_DETECTED (TEST bench fall, accel OR baro; tilt removed),
   YELLOW external PB5/D4 = ALT_DROP (finger-press, immediate) + deployment
   latch, RED external PB4/D5 = FAULT indicator (status/fault only).
   Active high (LD2: PA5 HIGH = ON; externals: anodes via series resistor,
   cathodes on GND; PB4 HIGH = red ON, LOW = OFF).
   PB3 left at reset default (SWO). PB4 = NJTRST pin reused as GPIO:
   project uses SWD only (PA13/PA14), so PB4 is free (verified: no other
   PB4 use in Src/Inc/.ioc). Pyro outputs are LOGICAL events only
   (one-shot flags + LEDs + UART); no physical pyro GPIO is driven. */
#define BLUE_LED_GPIO_Port GPIOA
#define BLUE_LED_Pin GPIO_PIN_10
#define YELLOW_LED_GPIO_Port GPIOB
#define YELLOW_LED_Pin GPIO_PIN_5
#define RED_LED_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_4
/* Flight build select: production by default (PROD thresholds).
   SIT/SUT bench builds pass -DFLIGHT_TEST_MODE=1 (and optionally
   -DFLIGHT_TEST_MAIN_ALT_M=<m>) to disable the 1250 m gate and override
   the main altitude. Algorithm is identical; only parameters change. */
#ifndef FLIGHT_TEST_MODE
#define FLIGHT_TEST_MODE (0)
#endif
#ifndef FLIGHT_TEST_MAIN_ALT_M
#define FLIGHT_TEST_MAIN_ALT_M FLIGHT_MAIN_ALT_TEST_M
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
/* USART6 = GPS (NEO-6M, 9600 8N1): PC7/D9 = USART6_RX (from GPS TX),
 * PC6 = USART6_TX (to GPS RX, UBX pad config; needs a physical wire).
 * USART2/VCOM is untouched. See MX_USART6_UART_Init + MspInit. */
UART_HandleTypeDef huart6;
/* USART1 = LoRa radio (Ebyte E220-900T22D, transparent UART, 9600 8N1):
 * PB6/D10 = TX (AF7) -> module RXD, PB7/D11 = RX (AF7) <- module TXD.
 * NOT the PA9/PA10 primary pair: PA10 is the BLUE LED. Src/lora.c only
 * declares this handle; main.c owns and initialises every UART handle. */
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Startup-relative orientation reference: raw accel mean of the cal window.
   This IS the user's held starting pose (flat table -> ~[0,0,+1g]).
   Gyro bias only; no accel bias subtraction anywhere in the tilt path. */
/* Gyro bias from the startup calibration. Retained for the boot log and the
   Ek-7 status frame; the Mahony estimator that consumed it is gone. */
static int32_t gbias_x = 0, gbias_y = 0, gbias_z = 0;

/* BLUE latch (bench fall detected): set once, never cleared. Drives the BLUE
 * LED + the RED LED (BLUE AND YELLOW together). Lives at file scope so the
 * TUI can display it next to the other LED states. */
static bool blue_latch = false;
/* Motion-liveness latch for the baro fall channel: last HAL_GetTick at which
 * the vert channel showed handling-level disturbance (see motion block). */
static uint32_t motion_ms = 0;
static bool motion_have = false;
static int32_t gravity_ref_x = 0, gravity_ref_y = 0, gravity_ref_z = 0;
/* Tilt low-pass state (filtered raw accel, same frame/scale as ref). */
static int32_t tilt_fx = 0, tilt_fy = 0, tilt_fz = 0;
static bool tilt_finit = false;
/* BMP280 runtime reference (ground pressure, single sensor @0x76).
   P0 is written once during calibration and never rewritten at runtime. */
static bmp280_t bmp1;
static float bmp1_p0 = 0.0f;
static bool bmp1_has_p0 = false;
/* BMP2 second pressure sensor (@0x77, SDO->3V3; BMP1 SDO->GND @0x76).
   Same shared bmp280 driver (no duplicate driver); same P0 contract:
   written once during calibration, never at runtime. */
static bmp280_t bmp2;
static float bmp2_p0 = 0.0f;
static bool bmp2_has_p0 = false;
/* Fused ground reference: mean of available sensor P0s (dual when both
   healthy, single on fallback, invalid when none). */
static float fused_p0 = 0.0f;
static bool fused_has_p0 = false;
/* LED + console pacing state. */
static bool prev_ld2 = false, prev_blue = false, prev_yel = false;
static bool prev_red = false;
static bool prev_alt_rise = false;
static bool prev_alt_drop = false;
/* Calibration noise floor for the adaptive DROP gate (set once at cal
 * end, alongside the apogee noise). One float, no extra mechanism. */
static float drop_sigma = AVT_DROP_SIGMA_DEFAULT_M;
/* Slow baseline for the DROP gate (see AVT_DROP_REF_ALPHA). Seeded from the
 * first valid altitude at runtime (NOT 0.0): alt_m is referenced to the fused
 * P0, so the first sample is generally non-zero (a few tenths of a metre).
 * Starting the ref at 0 made (alt - ref) begin life already past the gate
 * threshold, so the yellow LED self-fired for the first ~25 s while the EMA
 * walked down to the true rest altitude. Frozen on invalid samples. */
static float drop_ref = 0.0f;
static bool drop_ref_seeded = false;
/* Telemetry for the finger (DROP) gate: live (alt - ref) and the ON
 * threshold it is compared against. */
static float drop_delta_m = 0.0f;
static float drop_gate_m = 0.0f;
static uint32_t last_tui_ms = 0;
/* Ek-7 cerceve zamanlayicisi (SIT telemetri + SUT durum). TUI'dan ayri:
 * TUI 100 ms, Ek-7 Tablo 2-6 100 ms (10 Hz). */
static uint32_t last_ek7_tx_ms = 0;
static uint32_t last_dbg_ms = 0;
/* Filtered pressure for altitude (float, FPU present). */
static float press_f = 0.0f;
static bool press_finit = false;
/* 3-sample median pre-filter on compensated pressure (spike rejection).
 * Single shared point before the EMA/altitude/Kalman split. */
static avt_med3_t press_med;
/* Second median for BMP2: compensation -> median per sensor, then
 * combine -> single EMA -> altitude (see flight_combine_pressure). */
static avt_med3_t press_med2;
/* Central flight lifecycle (launch/apogee/free-fall/deploy).
 * Bench avt_apogee_t below is preserved untouched for the finger/vessel
 * demo path; flight_t is the real-flight state machine on top. */
static flight_t flight;
static bool launch_confirmed = false;
static bool ff_confirmed = false;
static bool prev_bench = false; /* edge detector for DUSUS ALGILANDI */
/* ---- Ek-7 test modes (official Tablo 1 commands, ST-LINK VCOM transport).
 * NORMAL: production/test bench behavior (text TUI). SIT: 10 Hz binary
 * telemetry from LIVE sensors. SUT: own sensors ignored, synthetic 0xAB
 * frames drive the SAME filters + flight algorithm, 10 Hz status out.
 * Stop command or SUT RX silence >1000 ms returns to NORMAL. Bench-only
 * signals never touch deployment (unchanged flight_step). */
typedef enum
{
  EK7MODE_NORMAL = 0,
  EK7MODE_SIT,
  EK7MODE_SUT
} ek7mode_t;
static ek7mode_t ek7_mode = EK7MODE_NORMAL;
/* Run-mode separation (REAL / TEST / SIT / SUT) + real-flight lock. SIT/SUT
 * commands are refused once the rocket has really flown (see runmode.h). */
static runmode_state_t rmode;
static ek7_rx_t ek7rx;
static ek7_cmd_t ek7_pending = EK7_EV_NONE;
static uint32_t ek7_activate_ms = 0;
/* SUT synthetic sample (validated ranges = Ek-7 error management). */
static uint8_t sut_acc[EK7_TLM_LEN];
static uint8_t sut_n = 0;
static bool sut_has_sample = false;
static bool sut_sample_ok = false;
/* Fresh-frame flag: set on every validated SUT frame, consumed by the
 * lifecycle step (SUT steps the flight machine once per FRAME, never once
 * per loop: the loop runs ~2x faster than the 10 Hz frame rate, and
 * re-stepping the same sample injects zero-velocity copies that pin the
 * apogee streak at zero). NORMAL/SIT step every loop as before. */
static bool sut_frame_new = false;
static float sut_alt_m = 0.0f;
static float sut_ax = 0.0f, sut_ay = 0.0f, sut_az = 0.0f;
static float sut_ex = 0.0f, sut_ey = 0.0f;
static uint32_t sut_last_ms = 0;
#define EK7_RX_DRAIN_MAX (64u)
#define EK7_SUT_TIMEOUT_MS (1000u)
/* Komut→aktivasyon gecikmesi: resmi test cihazi komutun hemen ardindan
 * (100 ms periodla) cerceve akitmaya baslar; uzun kor pencere hem ilk
 * ornekleri cope atar hem tum olay zamanlarini kaydirir. 200 ms, hatali
 * tetiklemeye karsi kisa bir terkos tutarken kaymayi ihmal edilebilir
 * seviyeye indirir. */
#define EK7_ACTIVATE_MS (200u)
/* Ek-7 TX periodi: Tablo 2/3/4 ve Tablo 5/6 her iki cerceve icin de
 * 10 Hz (100 ms) diyor. Ek-7 dali kendi zamanlayicisini kullanir; TUI
 * 100 ms kapiyla guncellenmeye devam eder (NORMAL mod metin ekrani,
 * Ek-7 cercevesi uretmez). */
#define EK7_TX_PERIOD_MS (100u)
/* Synthetic plausibility envelope (outside -> sample invalid, no feed). */
#define SUT_ALT_MIN_M (-1000.0f)
#define SUT_ALT_MAX_M (20000.0f)
#define SUT_PRESS_MIN_MB (100.0f)
#define SUT_PRESS_MAX_MB (1200.0f)
/* Synthetic plausibility envelope (outside -> sample invalid, no feed).
 * Accel cap clears real motor-burn transients (official profiles peak at
 * ~247 m/s2): a tighter cap rejected the whole burn and starved the SUT
 * watchdog. */
#define SUT_ACC_MAX_MS2 (500.0f)
#define SUT_ANG_MAX_DEG (360.0f)
/* Ek-7 TX snapshot (set in lifecycle block, consumed by TX branch). */
static bool ek7_tilt_band = false;
static float ek7_use_alt = 0.0f;
static bool ek7_use_alt_valid = false;
/* ---- DWT cycle-counter timing (A3: diagnostic only, no behavior change).
 * Sections: IMU reads, pressure reads, processing (filters+flight+LED),
 * telemetry TX (Ek-7 binary or text TUI), whole loop work (excl. HAL_Delay idle). Running min/avg/max
 * in core cycles; TUI converts with SystemCoreClock (50 MHz -> 50 cy/us). */
typedef enum
{
  TIM_IMU = 0,
  TIM_PRS,
  TIM_PROC,
  TIM_TX,
  TIM_LOOP,
  TIM_N
} tim_sec_t;
static uint32_t tim_n[TIM_N];
static uint64_t tim_sum[TIM_N];
static uint32_t tim_min[TIM_N];
static uint32_t tim_max[TIM_N];
static uint32_t tim_start[TIM_N];
static uint32_t last_tim_ms = 0;
static inline uint32_t dwt_now(void)
{
  return DWT->CYCCNT;
}
static inline void tim_begin(tim_sec_t s)
{
  tim_start[s] = dwt_now();
}
static inline void tim_end(tim_sec_t s)
{
  uint32_t d = dwt_now() - tim_start[s]; /* unsigned: wrap-safe */
  if (tim_n[s] == 0 || d < tim_min[s])
  {
    tim_min[s] = d;
  }
  if (d > tim_max[s])
  {
    tim_max[s] = d;
  }
  tim_sum[s] += d;
  tim_n[s]++;
}
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
/* ---- Sensor freshness (A7: present/valid/stale per sensor). ---- */
static uint32_t last_mpu_ms = 0;
static uint32_t last_bmp_ms = 0;
static uint32_t last_bmp2_ms = 0;
#define SENSOR_STALE_MS (500u)
/* UART RX interrupt ring (Ek-7 listener). Single-byte HW buffer would drop
 * bursts between 100 ms polls, so bytes accumulate here via RXNE IRQ and
 * the main loop drains the ring (bounded per iteration). */
#define EK7_RXRING_N (128u)
static volatile uint8_t ek7_rxring[EK7_RXRING_N];
static volatile uint8_t ek7_rx_head = 0;
static volatile uint8_t ek7_rx_tail = 0;
static uint8_t ek7_rx_tmp = 0;

/* GPS RX path (USART6, NEO-6M 9600): same 1-byte-IT + ring shape
 * Ek-7 listener. NMEA arrives at ~400 B/s, so one byte per IRQ is cheap.
 * The main loop drains the ring with a per-iteration bound; the parser
 * (gps.c) tolerates arbitrary chunking, so IRQ/loop timing never matters. */
/* 1024 bytes holds ~1.06 s of a saturated 9600-baud line (960 B/s). The
 * 256-byte version held only ~0.27 s, which a single 100 ms I2C timeout plus
 * the TUI frame used to overrun -> dropped bytes -> broken NMEA checksums ->
 * a live module reported as ABSENT. Head/tail MUST be uint16_t: with uint8_t
 * the index `nxt = (head + 1u) % 1024u` can only reach 0..255, so the array
 * silently behaved as a 256-entry ring (and 256 narrowed to 0, colliding with
 * the head==tail "empty" sentinel). Every wrap is still `% GPS_RXRING_N`. */
#define GPS_RXRING_N (1024u)
static volatile uint8_t gps_rxring[GPS_RXRING_N];
static volatile uint16_t gps_rx_head = 0;
static volatile uint16_t gps_rx_tail = 0;
static uint8_t gps_rx_tmp = 0;
static gps_t gps;
/* GPS config sequencer (board TX -> GPS RX, UBX, pad-only): polled once per
 * loop, transmits at most one frame via non-blocking IT. gps_cfg_poll
 * decides; gps_tx_busy tracks the in-flight IT transfer. */
static gps_cfg_t gps_cfg;
static uint8_t gps_tx_buf[GPS_UBX_MAX_FRAME];
static volatile bool gps_tx_busy = false;
/* Son özerk config yeniden denemesi + aralık (üstel geri çekilme, tavanlı)
 * ve ölü tur sayacı (üst sınırlı). Üçü de tanı amaçlıdır: kopuk GPS hattı
 * NOACK/NAK'da sessizce biter, asla fault üretilmez, uçuş etkilenmez. */
static uint32_t gps_cfg_retry_ms = 0;
static uint32_t gps_cfg_retry_gap = GPS_CFG_RETRY_BASE_MS;
static uint8_t gps_cfg_retry_n = 0;
/* LoRa telemetry uplink (Ebyte E220-900T22D, transparent UART on USART1
 * PB6/PB7 AF7 - NOT PA9/PA10, because PA10 is the BLUE LED). One bounded
 * 41-byte air packet (3-byte fixed header + 38-byte V3 core) per lora_poll()
 * on the 100 ms cadence. Strictly a
 * reader: it cannot touch flight state.
 * NOTE: `huart1` is DEFINED above next to huart2/huart6 (CubeMX convention),
 * not declared extern here - a second definition would be a duplicate
 * symbol. lora_uart_status starts as HAL_ERROR so an init path that never
 * runs MX_USART1_UART_Init still leaves the link closed. */
static uint8_t lora_tx_buf[LORA_AIR_LEN];
static HAL_StatusTypeDef lora_uart_status = HAL_ERROR;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != NULL && huart->Instance == USART6)
  {
    gps_tx_busy = false;
  }
}
/* USART6 line-error counter (framing/overrun/noise). Diagnostic only:
 * rx=0 + err=0   -> silent line (unwired/unpowered GPS).
 * rx=0 + err>0   -> something transmits but framing fails (wrong baud or
 *                   noisy line). Re-arms reception so one bad burst cannot
 *                   wedge the channel. */
static volatile uint32_t gps_rx_err = 0;
/* USART6 ring-full drops: bytes the producer refused to store because the
 * ring was full. A byte lost mid-sentence breaks the NMEA checksum, so
 * gps.present stays false and the TUI reports ABSENT even though the module
 * is alive. This counter is the only way to tell a silent line from an
 * overrun ring. Diagnostic only, same as gps_rx_err. USART2/Ek-7 is
 * deliberately NOT counted here: it has its own CRC counters and is a
 * contract surface. */
static volatile uint32_t gps_rx_drop = 0;
/* GPS raw-byte history for the B1 on-demand hexdump (diagnostic only).
 * Lets the bench see WHAT arrives on USART6 without streaming NMEA at
 * 10 Hz: press the blue B1 button once, one snapshot prints on VCOM. */
#define GPS_HIST_N (64u)
static uint8_t gps_hist[GPS_HIST_N];
static uint8_t gps_hist_n = 0;
/* Content fingerprint of the raw USART6 stream (diagnostic only): counts
 * '$' (NMEA sentences), 0xB5 (UBX sync) and '\n' (line ends). Read with
 * rx/err: NMEA-shaped but ABSENT = corruption on the wire (reseat!);
 * B5-heavy, no NL = UBX-only module; none of the three but rx grows =
 * unknown binary (clone chip?). Untouched by flight logic. */
static uint32_t gps_cnt_dollar = 0;
static uint32_t gps_cnt_b5 = 0;
static uint32_t gps_cnt_nl = 0;
/* CR counter: NMEA is CRLF, but some module/clone firmware emits CR only.
 * The parser keys off '\n' (gps.c), so a CR-only stream would look like "dollar
 * signs but never a single line end". Counting CR separates "module terminates
 * with CR" (parser fix) from "module sends no terminator at all" (hardware/
 * mode). Diagnostic only. */
static uint32_t gps_cnt_cr = 0;
static bool b1_prev = false;
static uint32_t b1_ms = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != NULL && huart->Instance == USART6)
  {
    gps_rx_err++;
    (void)HAL_UART_Receive_IT(huart, &gps_rx_tmp, 1);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint8_t nxt = (uint8_t)((ek7_rx_head + 1u) % EK7_RXRING_N);
    if (nxt != ek7_rx_tail)
    {
      ek7_rxring[ek7_rx_head] = ek7_rx_tmp;
      ek7_rx_head = nxt;
    }
    (void)HAL_UART_Receive_IT(huart, &ek7_rx_tmp, 1);
  }
  else if (huart->Instance == USART6)
  {
    uint16_t nxt = (uint16_t)((gps_rx_head + 1u) % GPS_RXRING_N);
    if (nxt != gps_rx_tail)
    {
      gps_rxring[gps_rx_head] = gps_rx_tmp;
      gps_rx_head = nxt;
    }
    else
    {
      gps_rx_drop++; /* full ring: byte lost, count it (never silent) */
    }
    (void)HAL_UART_Receive_IT(huart, &gps_rx_tmp, 1);
  }
}
static uint32_t last_flight_ms = 0;
static uint32_t drogue_events = 0;
static uint32_t main_events = 0;
/* Mahony 6-DOF diagnostic estimator state (never drives tilt/LEDs). */

/* Apogee/descent detector state (drives LD2+YELLOW, latched). */
static avt_apogee_t apogee;
/* Parallel Kalman altitude (DIAGNOSTIC ONLY, never drives apogee/LEDs).
 * Input is median-filtered pressure -> relative altitude (never EMA). */
static avt_kf_t kf;
static bool kf_ready = false;
static uint32_t last_kf_ms = 0;
static float kalt_m = 0.0f, kalt_v = 0.0f;
static bool kalt_valid = false;
/* Q default from host-test sweep (see tests/host Q table): best
 * RMS/latency tradeoff on the 10 Pa vessel set. R per-flight from cal. */
#define KF_Q_DEFAULT (0.05f)
/* Calibration pressure-noise estimate (Welford M2, Pa^2) feeding the
 * noise-relative apogee trend gates. P0 math itself is untouched. */
static double cal_p1_m2 = 0.0;
static uint32_t cal_p1_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void); /* GPS (NEO-6M 9600, TX+RX) */
static void MX_USART1_UART_Init(void); /* LoRa radio (E220-900T22D, 9600 8N1) */
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- TUI staging buffer: non-blocking in COUNT, not in wire time -----------
 * The text TUI is a contract. The official Ek-7 tool and a human on the
 * terminal both read these EXACT bytes, so no label, space, number format or
 * CRLF may change. The only thing that changes is how many blocking
 * HAL_UART_Transmit calls carry them: today the ~93 per-frame tokens each
 * block until TC, which pins the main loop at ~55 ms per frame and starves
 * the USART6 drain. Tokens now APPEND here and ONE transmit per TUI block
 * pushes the frame out.
 *
 * Byte-for-byte equivalence: a UART emits the concatenated stream exactly as
 * it would emit the same bytes one call at a time. USART2 has no TX
 * interrupts enabled (RXNE IT only) and HAL_UART_Transmit blocks on TC, so no
 * other writer can interleave into the middle of the staged frame. Nothing is
 * reordered, dropped or reformatted - only the call count falls from ~93 to
 * 1 per frame.
 *
 * Sizing: TUI_BUF_N = 1400 > the pessimistic analytic worst case of 954 B
 * (both arms of every conditional counted; widths 10 for uint32, 11 for
 * int32, 13 for tenths, 17 for coords, 17 for the longest state name). The
 * observed steady-state frame is ~537 B, so overflow cannot occur in normal
 * operation; the guard below exists so that a future longer frame can never
 * put a partial field on the wire. Reproduced by
 * tests/host/test_tui_buf_host.c. */
#define TUI_BUF_N (1400u)
/* Flush timeout must cover the whole staged frame or HAL_UART_Transmit would
 * cut it mid-line: 10 bits/byte at 115200 baud is 115 bytes/ms, so the
 * worst-case frame needs TUI_BUF_N*11/115 ms, +10 ms for the TC tail. */
#define TUI_TX_TMO_MS (((TUI_BUF_N * 11u) / 115u) + 10u)
static char tui_buf[TUI_BUF_N];
static uint16_t tui_len = 0;   /* bytes staged in tui_buf */
static uint16_t tui_line = 0;  /* offset where the last UNTERMINATED line starts */
static bool tui_win = false;   /* a TUI capture window is open */
static bool tui_trunc = false; /* overflow seen: rest of this frame is dropped */
static uint32_t tui_drop = 0;  /* tokens lost to overflow (debugger inspection) */

/* Append one token. Never writes a token partially. */
static void tui_append(const char *s, uint16_t len)
{
  if (tui_trunc)
  {
    return; /* already over budget: the whole rest of the frame is dropped */
  }
  if ((uint32_t)len > (uint32_t)(TUI_BUF_N - tui_len))
  {
    /* Cut back to the last CRLF so the flushed frame always ends on a line
     * boundary, then suppress every remaining token. */
    tui_trunc = true;
    tui_drop++;
    tui_len = tui_line;
    return;
  }
  for (uint16_t i = 0; i < len; i++)
  {
    tui_buf[tui_len] = s[i];
    tui_len++;
    if (s[i] == '\n')
    {
      tui_line = tui_len; /* tokens may embed CRLF ("\r\nGPSHEX: ") */
    }
  }
}

/* Open a capture window: everything appended until tui_flush() leaves as one
 * transmit. Outside a window diag_uart_tx keeps its original immediate
 * blocking transmit, so boot/calibration/event lines keep their current bytes
 * AND their current timing. */
static void tui_begin(void)
{
  tui_len = 0;
  tui_line = 0;
  tui_trunc = false;
  tui_win = true;
}

/* One transmit for the whole window. Resets unconditionally so a failed or
 * timed-out flush still leaves the next frame clean. */
static void tui_flush(void)
{
  uint16_t n = tui_len;
  tui_win = false;
  tui_len = 0;
  tui_line = 0;
  tui_trunc = false;
  if (n > 0u)
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)tui_buf, n, TUI_TX_TMO_MS);
  }
}

/* Targeted I2C test helper: blocking UART TX with finite timeout, no printf. */
static void diag_uart_tx(const char *s)
{
  uint16_t len = 0;
  while (s[len] != '\0' && len < 128)
  {
    len++;
  }
  if (len == 0u)
  {
    return;
  }
  if (tui_win)
  {
    tui_append(s, len);
    return;
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)s, len, 100);
}

static const char *diag_status_str(HAL_StatusTypeDef st)
{
  if (st == HAL_OK)
  {
    return "HAL_OK";
  }
  if (st == HAL_ERROR)
  {
    return "HAL_ERROR";
  }
  if (st == HAL_BUSY)
  {
    return "HAL_BUSY";
  }
  if (st == HAL_TIMEOUT)
  {
    return "HAL_TIMEOUT";
  }
  return "UNKNOWN";
}

static void diag_uart_hex32(uint32_t v)
{
  static const char hexd[] = "0123456789ABCDEF";
  char s[11];
  s[0] = '0';
  s[1] = 'x';
  for (int k = 0; k < 8; k++)
  {
    s[2 + k] = hexd[(v >> (28 - 4 * k)) & 0xFU];
  }
  s[10] = '\0';
  diag_uart_tx(s);
}

static void diag_uart_int32(int32_t v)
{
  if (v < 0)
  {
    diag_uart_tx("-");
    /* Handle INT32_MIN safely via 64-bit negation. */
    uint32_t u = (uint32_t)(-(int64_t)v);
    char tmp[11];
    int n = 0;
    while (u > 0)
    {
      tmp[n++] = (char)('0' + (u % 10));
      u /= 10;
    }
    char out[12];
    int idx = 0;
    for (int i = n - 1; i >= 0; i--)
    {
      out[idx++] = tmp[i];
    }
    out[idx] = '\0';
    diag_uart_tx(out);
  }
  else if (v == 0)
  {
    diag_uart_tx("0");
  }
  else
  {
    uint32_t u = (uint32_t)v;
    char tmp[11];
    int n = 0;
    while (u > 0)
    {
      tmp[n++] = (char)('0' + (u % 10));
      u /= 10;
    }
    char out[12];
    int idx = 0;
    for (int i = n - 1; i >= 0; i--)
    {
      out[idx++] = tmp[i];
    }
    out[idx] = '\0';
    diag_uart_tx(out);
  }
}

static void diag_uart_uint32(uint32_t v)
{
  if (v == 0)
  {
    diag_uart_tx("0");
    return;
  }
  char tmp[11];
  int n = 0;
  while (v > 0)
  {
    tmp[n++] = (char)('0' + (v % 10));
    v /= 10;
  }
  char out[12];
  int idx = 0;
  for (int i = n - 1; i >= 0; i--)
  {
    out[idx++] = tmp[i];
  }
  out[idx] = '\0';
  diag_uart_tx(out);
}

/* Signed tenths with one decimal, e.g. -123 -> "-12.3". plus forces '+'. */
static void diag_uart_tenths(int32_t t, bool plus)
{
  bool neg = t < 0;
  uint32_t a = neg ? (uint32_t)(-(int64_t)t) : (uint32_t)t;
  diag_uart_tx(neg ? "-" : (plus ? "+" : ""));
  diag_uart_uint32(a / 10U);
  diag_uart_tx(".");
  diag_uart_uint32(a % 10U);
}

/* GPS coordinate in 1e-6 deg, e.g. 48117300 -> "+48.117300". Six decimals
 * (~0.11 m) match the parser's rounding; zero-padded fraction keeps columns
 * aligned in the TUI. */
static void diag_uart_coord(int32_t e6)
{
  bool neg = e6 < 0;
  uint32_t a = neg ? (uint32_t)(-(int64_t)e6) : (uint32_t)e6;
  uint32_t ip = a / 1000000U;
  uint32_t fp = a % 1000000U;
  diag_uart_tx(neg ? "-" : "+");
  diag_uart_uint32(ip);
  diag_uart_tx(".");
  char f[7];
  for (int i = 5; i >= 0; i--)
  {
    f[i] = (char)('0' + (fp % 10U));
    fp /= 10U;
  }
  f[6] = '\0';
  diag_uart_tx(f);
}

/* Write GPIO only on change: LEDs never toggle needlessly. */
static void led_write(GPIO_TypeDef *port, uint16_t pin, bool *prev, bool on)
{
  if (*prev != on)
  {
    *prev = on;
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

/* Meaningful LED API over the status LEDs (no magic
 * GPIO writes at call sites). Preserves verified meanings:
 *   LED_System    (LD2/PA5):  sensors-ok + flight progression latch
 *   LED_Fall      (BLUE/PA10): BENCH_FALL_DETECTED (TEST bench, accel OR baro)
 *   LED_Deploy    (YELLOW/PB5): finger-drop + drogue/main latch
 *   LED_Red       (RED/PB4):  FAULT indicator only (normal: OFF)
 * Tilt/angle is DIAGNOSTIC ONLY (TUI + Mahony): it drives no LED and no
 * decision anywhere.
 * FAULT overrides all four (2 Hz blink handled by caller via fault_blink;
 * red joins the existing fault-blink mechanism, same phase as LD2/BLUE).
 * Safe init: all RESET in MX_GPIO_Init (USER CODE section), never SET at
 * boot; deployment never drives a pyro GPIO (logical events only). */
static void LED_System_On(bool *prev)
{
  led_write(LD2_GPIO_Port, LD2_Pin, prev, true);
}
static void LED_System_Off(bool *prev)
{
  led_write(LD2_GPIO_Port, LD2_Pin, prev, false);
}
static void LED_Fall_On(bool *prev)
{
  led_write(BLUE_LED_GPIO_Port, BLUE_LED_Pin, prev, true);
}
static void LED_Fall_Off(bool *prev)
{
  led_write(BLUE_LED_GPIO_Port, BLUE_LED_Pin, prev, false);
}
static void LED_Deploy_On(bool *prev)
{
  led_write(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, prev, true);
}
static void LED_Deploy_Off(bool *prev)
{
  led_write(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, prev, false);
}
static void LED_Red_On(bool *prev)
{
  led_write(RED_LED_GPIO_Port, RED_LED_Pin, prev, true);
}
static void LED_Red_Off(bool *prev)
{
  led_write(RED_LED_GPIO_Port, RED_LED_Pin, prev, false);
}

/* Reset-cause capture (A11): read all RCC reset flags BEFORE anything
 * clears them. start_demo gating is unchanged (pin && !por); the full
 * cause is reported later as RESET_CAUSE once UART is up. */
typedef enum
{
  RST_POWER_ON = 0,
  RST_PIN,
  RST_WATCHDOG,
  RST_SOFTWARE,
  RST_OTHER
} reset_cause_t;
static reset_cause_t boot_reset_cause = RST_OTHER;

static bool reset_capture(void)
{
  bool pin = (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U);
  bool por = (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U);
  bool iwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U);
  bool sftr = (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  if (iwdg)
  {
    boot_reset_cause = RST_WATCHDOG;
  }
  else if (sftr)
  {
    boot_reset_cause = RST_SOFTWARE;
  }
  else if (pin && !por)
  {
    boot_reset_cause = RST_PIN;
  }
  else if (por)
  {
    boot_reset_cause = RST_POWER_ON;
  }
  else
  {
    boot_reset_cause = RST_OTHER;
  }
  return pin && !por;
}

static const char *reset_cause_str(reset_cause_t c)
{
  switch (c)
  {
    case RST_POWER_ON:
      return "POWER_ON";
    case RST_PIN:
      return "PIN_RESET";
    case RST_WATCHDOG:
      return "WATCHDOG";
    case RST_SOFTWARE:
      return "SOFTWARE";
    default:
      return "OTHER";
  }
}

/* Independent watchdog (A11): bare-metal (HAL IWDG module not vendored).
 * LSI/64, reload 1000 -> ~2.0 s nominal. Loop worst-case is far below (all
 * HAL waits are finite ~100 ms); refresh runs every loop iteration and
 * every calibration step. One-way once started; timeout choice keeps >5x
 * margin under LSI spread. */
#define IWDG_KR_UNLOCK (0x5555u)
#define IWDG_KR_RELOAD (0xAAAAu)
#define IWDG_KR_START (0xCCCCu)
static void iwdg_refresh(void)
{
  IWDG->KR = IWDG_KR_RELOAD;
}
static void iwdg_start(void)
{
  RCC->CSR |= RCC_CSR_LSION;
  uint32_t to = 100000u;
  while (!(RCC->CSR & RCC_CSR_LSIRDY) && to > 0u)
  {
    to--;
  }
  IWDG->KR = IWDG_KR_UNLOCK;
  IWDG->PR = 4u;   /* /64: 32 kHz -> 500 Hz tick */
  IWDG->RLR = 1000u; /* 2.0 s timeout */
  uint32_t upd = 100000u;
  while (IWDG->SR != 0u && upd > 0u) /* bounded: ~125 us nominal */
  {
    upd--;
  }
  iwdg_refresh();
  IWDG->KR = IWDG_KR_START;
}

/* I2C link recovery (A10): bounded, only on failure streaks of a sensor
 * that probed present at boot. HAL DeInit/Init + sensor re-probe; max 3
 * attempts per boot (then silent). Never slows the good path (zero cost
 * when reads succeed). No GPIO bit-banging (pin risk avoided by design). */
static uint8_t bmp_recovers = 0;
static uint8_t b2_recovers = 0;
static uint8_t bmp_fails = 0;
static uint8_t b2_fails = 0;
#define I2C_RECOVER_AFTER (5u)
#define I2C_RECOVER_MAX (3u)
/* MPU accel stall counter: consecutive dead reads (NACK or all-zero
 * payload). Re-wake threshold below: ~4 s of dead accel at loop rate. */
static uint8_t mpu_fails = 0;
#define MPU_RECOVER_AFTER (50u)
#if FLIGHT_TEST_MODE
/* TEST-only watchdog-test latch (production: symbol absent, refresh
 * unconditional). Set by the AA 2E test hook; loop then skips refresh. */
static bool wdg_test_stop = false;
#endif
/* A6 self-test inputs: probe-level MPU result + any-valid-sample flag. */
static bool mpu_probe_ok = false;
static bool cal_mpu_ok = false;

static void i2c_link_recover(void)
{
  HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();
}

/* Current Ek-7 link mode as the run-mode module sees it. */
static rm_link_t rm_link_now(void)
{
  if (ek7_mode == EK7MODE_SUT)
  {
    return RM_LINK_SUT;
  }
  if (ek7_mode == EK7MODE_SIT)
  {
    return RM_LINK_SIT;
  }
  return RM_LINK_NORMAL;
}

/* Clean pad state for the flight machine. Used on SUT entry AND exit so a
 * synthetic flight never leaves a "flown" state/max altitude behind (that
 * would be indistinguishable from a real flight). Keeps the configuration
 * (production thresholds stay production). */
static void flight_reset_to_pad(void)
{
  flight_cfg_t keep = flight.cfg;
  flight_init(&flight, &keep);
  flight_set_state(&flight, FLIGHT_WAIT_FOR_LAUNCH);
  launch_confirmed = false;
  ff_confirmed = false;
}

/* Rejected SIT/SUT command: visible in TUI (REJ counter) and, in NORMAL
 * mode only, as one text line (binary SIT/SUT streams stay parseable). */
static void ek7_cmd_rejected(void)
{
  if (ek7_mode == EK7MODE_NORMAL)
  {
    diag_uart_tx("EK7 CMD REJECTED (REAL FLIGHT LOCK)\r\n");
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Read reset cause before anything can disturb RCC flags. */
  bool start_demo = reset_capture();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init(); /* GPS: RX IT + TX IT, never blocks; absence is not a fault */
  MX_USART1_UART_Init();
  lora_init(lora_uart_status);
  if (lora_uart_status != HAL_OK)
  {
    diag_uart_tx("LORA: INIT FAIL\r\n"); /* absence is never a fault */
  }
  else
  {
    /* E220 register check: Mode 3 read-back of 00H..05H, rewrite only on
     * mismatch, always back to Mode 0. Bounded (~1 s worst), never a
     * fault, never touches SIT/SUT or flight state. */
    lora_cfg_result_t lora_cfg = lora_cfg_boot();
    diag_uart_tx("LORA CFG: ");
    diag_uart_tx(lora_cfg_str(lora_cfg));
    {
      /* One-line diagnosis: AUX level at exit + last register read-back.
       * AUX=0 + no read = AUX kablosu yok/kopuk. Read var ama FAILED =
       * yazma tutmadi (besleme dalgalanmasi veya modul varyanti). */
      uint8_t d6[6];
      bool have = false;
      uint8_t aux = 2u;
      uint32_t baud = 0u;
      uint8_t stage = 0u;
      uint8_t variant = 0xFFu;
      lora_cfg_diag(d6, &have, &aux, &baud, &stage, &variant);
      diag_uart_tx(" AUX=");
      diag_uart_tx(aux == 1u ? "1" : (aux == 0u ? "0" : "?"));
      diag_uart_tx(" B=");
      if (baud > 0u)
      {
        static const char D[] = "0123456789";
        char nb[11];
        uint8_t ni = 0u;
        uint32_t v = baud;
        while ((v > 0u) && (ni < 10u))
        {
          nb[ni++] = D[v % 10u];
          v /= 10u;
        }
        while (ni > 0u)
        {
          char ch[2];
          ni--;
          ch[0] = nb[ni];
          ch[1] = '\0';
          diag_uart_tx(ch);
        }
      }
      else
      {
        diag_uart_tx("--");
      }
      diag_uart_tx(" RD=");
      if (have)
      {
        static const char HEX[] = "0123456789ABCDEF";
        for (uint8_t i = 0u; i < 6u; i++)
        {
          char pr[3];
          pr[0] = HEX[d6[i] >> 4];
          pr[1] = HEX[d6[i] & 0x0Fu];
          pr[2] = '\0';
          diag_uart_tx(pr);
          diag_uart_tx(" ");
        }
      }
      else
      {
        diag_uart_tx("--");
      }
      /* Stage: 1 = hic okuma yok, 2 = echo tutmadi, 3 = dogrulama
       * tutmadi, 4 = tamam. W = yazma varyanti (0 = C2 gecici tam imaj).
       * KEY=wo: 06H/07H anahtar yazma-ozeldir, okumayla bagimsiz
       * dogrulanamaz; kanit C2 echo karsilastirmasidir. */
      diag_uart_tx(" ST=");
      {
        char st[2];
        st[0] = (char)('0' + (stage > 9u ? 9u : stage));
        st[1] = '\0';
        diag_uart_tx(st);
      }
      diag_uart_tx(" W=");
      {
        char wv[2];
        wv[0] = (variant > 3u) ? '-' : (char)('0' + variant);
        wv[1] = '\0';
        diag_uart_tx(wv);
      }
    }
    diag_uart_tx("\r\n");
  }
  gps_init(&gps);
  gps_cfg_init(&gps_cfg); /* UBX pad setup; NOACK if GPS absent, never fault */
  diag_uart_tx("GPS UART INIT 9600 (absent/nofix is normal)\r\n");
  /* USER CODE BEGIN 2 */
  diag_uart_tx("RESET_CAUSE: ");
  diag_uart_tx(reset_cause_str(boot_reset_cause));
  diag_uart_tx("\r\n");
  iwdg_start(); /* A11: refreshed every loop + every cal step below */
  /* Power-on idle: no calibration, no sensor runtime, no demo LEDs.
     The black NRST button reboots into the demo (PIN reset). */
  if (!start_demo)
  {
    diag_uart_tx("AVIONICS WAITING FOR RESET\r\n");
    while (1)
    {
      __WFI();
    }
  }
  /* Startup LED pulse, SEQUENTIAL so each LED is verified separately:
     LD2 on 250 ms -> off, then BLUE on 250 ms -> off.
     (Simultaneous pulse could not tell which LED was seen.) */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
  /* MPU6050 raw data test: one-shot WHO_AM_I + wake, periodic reads in loop. */
  diag_uart_tx("SYSTEM INIT\r\n");
  diag_uart_tx("MPU6050 raw test start\r\n");

  {
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef wst = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                                             0x75, I2C_MEMADD_SIZE_8BIT,
                                             &who_am_i, 1, 100);
    uint32_t werr = HAL_I2C_GetError(&hi2c1);

    static const char hexd[] = "0123456789ABCDEF";
    char val_str[5];
    val_str[0] = '0';
    val_str[1] = 'x';
    val_str[2] = hexd[(who_am_i >> 4) & 0xFU];
    val_str[3] = hexd[who_am_i & 0xFU];
    val_str[4] = '\0';

    diag_uart_tx("WHO_AM_I: ");
    diag_uart_tx(diag_status_str(wst));
    diag_uart_tx(" value=");
    diag_uart_tx(val_str);
    diag_uart_tx(" err=");
    diag_uart_hex32(werr);
    diag_uart_tx("\r\n");

    uint8_t wake_data = 0x00;
    HAL_StatusTypeDef kst = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,
                                              0x6B, I2C_MEMADD_SIZE_8BIT,
                                              &wake_data, 1, 100);
    uint32_t kerr = HAL_I2C_GetError(&hi2c1);

    diag_uart_tx("MPU6050 wake: ");
    diag_uart_tx(diag_status_str(kst));
    diag_uart_tx(" err=");
    diag_uart_hex32(kerr);
    diag_uart_tx("\r\n");
    /* A6 self-test input: probe level (WHO_AM_I + wake). */
    mpu_probe_ok = (wst == HAL_OK) && (who_am_i == 0x68u) &&
                   (kst == HAL_OK);
  }

  /* BMP280 startup diagnostic: presence + chip ID + calib + normal mode. */
  {
    static const char hexd[] = "0123456789ABCDEF";
    HAL_StatusTypeDef b1st = bmp280_init(&bmp1, &hi2c1, 0x76);
    char b1id[5] = {'0', 'x', hexd[(bmp1.id >> 4) & 0xFU], hexd[bmp1.id & 0xFU], '\0'};
    diag_uart_tx("BMP1 addr=0x76 status=");
    diag_uart_tx(diag_status_str(b1st));
    diag_uart_tx(" ID=");
    diag_uart_tx(b1id);
    diag_uart_tx("\r\n");
  }

  /* BMP2 second pressure sensor (@0x77): chip ID + calib + normal mode
   * (same shared driver as BMP1). */
  {
    HAL_StatusTypeDef b2st = bmp280_init(&bmp2, &hi2c1, 0x77);
    diag_uart_tx("BMP2 addr=0x77 status=");
    diag_uart_tx(diag_status_str(b2st));
    if (b2st != HAL_OK)
    {
      diag_uart_tx(" BMP2 ERROR\r\n");
    }
    else
    {
      diag_uart_tx("\r\n");
    }
  }

  /* Startup calibration: board assumed flat & still for ~16 s (elapsed time). */
  {
    diag_uart_tx("CALIBRATION START\r\n");
    diag_uart_tx("MPU calibration start: keep board flat and still\r\n");

    int64_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int64_t p1_sum = 0;
    uint32_t n_cal = 0, n_p1 = 0;
    /* BMP2 ground window: plain mean over the same 16 s (same contract
       as BMP: P0 written once, never at runtime). No drift correction on
       the second sensor: the fused P0 below is the mean of available P0s. */
    int64_t p2_sum = 0;
    uint32_t n_p2 = 0;
    /* Startup-drift windows: first/last 25% of the calibration (bounds
       derive from AVT_CAL_MS, so the counts scale with the duration).
       Raw samples only (same stream as the mean); no extra buffering. */
    int64_t p1_early_sum = 0, p1_late_sum = 0;
    uint32_t n_early = 0, n_late = 0;
    uint32_t cal_start = HAL_GetTick();
    uint32_t last_prog = cal_start;

    while ((HAL_GetTick() - cal_start) < AVT_CAL_MS)
    {
      uint8_t abuf[6];
      uint8_t gbuf[6];
      HAL_StatusTypeDef ast = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                                               0x3B, I2C_MEMADD_SIZE_8BIT,
                                               abuf, 6, 100);
      HAL_StatusTypeDef gst = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                                               0x43, I2C_MEMADD_SIZE_8BIT,
                                               gbuf, 6, 100);
      if (ast == HAL_OK && gst == HAL_OK &&
          (abuf[0] | abuf[1] | abuf[2] | abuf[3] | abuf[4] | abuf[5]) != 0u)
      {
        /* (Zero-vector reject, same rule as the runtime read: a stalled
         * device must not poison the gravity reference.) */
        ax_sum += (int16_t)((abuf[0] << 8) | abuf[1]);
        ay_sum += (int16_t)((abuf[2] << 8) | abuf[3]);
        az_sum += (int16_t)((abuf[4] << 8) | abuf[5]);
        gx_sum += (int16_t)((gbuf[0] << 8) | gbuf[1]);
        gy_sum += (int16_t)((gbuf[2] << 8) | gbuf[3]);
        gz_sum += (int16_t)((gbuf[4] << 8) | gbuf[5]);
        n_cal++;
        cal_mpu_ok = true;
      }

      /* Shared ground window: average the single BMP's pressure.
         P0 math (plain mean) is untouched; Welford M2 runs in parallel
         to estimate the pressure noise floor for the apogee gates. */
      if (bmp1.present)
      {
        int32_t tc1 = 0;
        uint32_t pr1 = 0;
        if (bmp280_read(&bmp1, &tc1, &pr1) == HAL_OK && pr1 > 0)
        {
          if (n_p1 > 0)
          {
            double mean_prev = (double)p1_sum / (double)n_p1;
            double mean_new = ((double)p1_sum + (double)pr1) / (double)(n_p1 + 1U);
            cal_p1_m2 += ((double)pr1 - mean_prev) * ((double)pr1 - mean_new);
            cal_p1_cnt++;
          }
          else
          {
            cal_p1_m2 = 0.0;
            cal_p1_cnt = 1;
          }
          p1_sum += (int64_t)pr1;
          n_p1++;
          /* Time-based windows off the same raw sample (untouched stream). */
          {
            uint32_t elp = HAL_GetTick() - cal_start;
            if (elp < AVT_P0C_EARLY_MS)
            {
              p1_early_sum += (int64_t)pr1;
              n_early++;
            }
            else if (elp >= AVT_P0C_LATE_MS)
            {
              p1_late_sum += (int64_t)pr1;
              n_late++;
            }
          }
        }
      }

      /* BMP2 ground window (same 16 s, independent mean). */
      if (bmp2.present)
      {
        int32_t tc2 = 0;
        uint32_t pr2 = 0;
        if (bmp280_read(&bmp2, &tc2, &pr2) == HAL_OK && pr2 > 0)
        {
          p2_sum += (int64_t)pr2;
          n_p2++;
        }
      }

      uint32_t el = HAL_GetTick() - cal_start;
      if (el - last_prog >= 2000U)
      {
        last_prog = el;
        diag_uart_tx("MPU calibration: ");
        diag_uart_int32((int32_t)(el / 1000U));
        diag_uart_tx(" s\r\n");
      }
      iwdg_refresh(); /* A11: cal loop stays inside watchdog */
      HAL_Delay(20);
    }

    if (n_cal > 0)
    {
      int32_t ax_mean = (int32_t)(ax_sum / (int64_t)n_cal);
      int32_t ay_mean = (int32_t)(ay_sum / (int64_t)n_cal);
      int32_t az_mean = (int32_t)(az_sum / (int64_t)n_cal);
      gbias_x = (int32_t)(gx_sum / (int64_t)n_cal);
      gbias_y = (int32_t)(gy_sum / (int64_t)n_cal);
      gbias_z = (int32_t)(gz_sum / (int64_t)n_cal);

      /* Startup-relative reference: the raw mean IS the reference pose.
         No [0,0,16384] synthesis: from a single static pose, sensor bias
         and gravity direction cannot be separated, so we reference the
         pose itself instead of inventing an ideal one. */
      gravity_ref_x = ax_mean;
      gravity_ref_y = ay_mean;
      gravity_ref_z = az_mean;

      diag_uart_tx("MPU calibration complete\r\n");
      diag_uart_tx("GYRO bias: X=");
      diag_uart_int32(gbias_x);
      diag_uart_tx(" Y=");
      diag_uart_int32(gbias_y);
      diag_uart_tx(" Z=");
      diag_uart_int32(gbias_z);
      diag_uart_tx("\r\n");
      diag_uart_tx("GRAVITY ref: X=");
      diag_uart_int32(gravity_ref_x);
      diag_uart_tx(" Y=");
      diag_uart_int32(gravity_ref_y);
      diag_uart_tx(" Z=");
      diag_uart_int32(gravity_ref_z);
      diag_uart_tx("\r\n");
      diag_uart_tx("GRAVITY ref magnitude=");
      diag_uart_int32((int32_t)avt_vec_mag((float)gravity_ref_x,
                                           (float)gravity_ref_y,
                                           (float)gravity_ref_z));
      diag_uart_tx("\r\n");
    }
    else
    {
      diag_uart_tx("MPU calibration: no valid samples, biases kept at 0\r\n");
    }

    /* Descent detector starts clean at cal end (never fed during cal).
       Init BEFORE set_noise: init resets sigma to default, then the
       measured floor below overwrites it. */
    avt_apogee_init(&apogee);
    /* Ground pressure references (per sensor, kept separate).
       Noise floor: sigma_p from Welford M2 -> sigma_alt via d(alt)/dP.
       Only feeds the apogee trend gates; P0/EMA/altitude math untouched. */
    if (n_p1 > 0)
    {
      /* Effective baseline: calibration mean corrected toward the stable
       * end-of-window baseline by the measured startup drift (adaptive,
       * capped, deadbanded -- see avt_p0_correct). No fixed bias: rest
       * sits at ~0 m so the ARM guard and RISE indicator stay honest.
       * Mean, Welford M2/sigma and every raw sample above are untouched. */
      float p1_mean = (float)(p1_sum / (int64_t)n_p1);
      float p0_corr = 0.0f;
      float p1_early = p1_mean;
      float p1_late = p1_mean;
      if (n_early > 0u && n_late > 0u)
      {
        p1_early = (float)(p1_early_sum / (int64_t)n_early);
        p1_late = (float)(p1_late_sum / (int64_t)n_late);
        float sigma_p0 = 0.0f;
        if (cal_p1_cnt > 1U && cal_p1_m2 > 0.0)
        {
          sigma_p0 = (float)sqrt(cal_p1_m2 / (double)(cal_p1_cnt - 1U));
        }
        p0_corr = avt_p0_correct(p1_early, p1_late, sigma_p0, n_early, n_late);
      }
      bmp1_p0 = p1_mean + p0_corr;
      bmp1_has_p0 = true;
      {
        float sigma_p = 0.0f;
        if (cal_p1_cnt > 1U && cal_p1_m2 > 0.0)
        {
          sigma_p = (float)sqrt(cal_p1_m2 / (double)(cal_p1_cnt - 1U));
        }
        float sigma_alt = sigma_p / 12.0f;
        if (!(sigma_alt >= 0.0f) || !isfinite(sigma_alt))
        {
          sigma_alt = 0.0f;
        }
        if (sigma_alt > 0.5f)
        {
          sigma_alt = 0.5f;
        }
        avt_apogee_set_noise(&apogee, sigma_alt);
        drop_sigma = sigma_alt;
        /* Parallel KF: same sigma_alt -> R variance (m^2), floored inside.
         * h0 = stable-end rest altitude under effective P0 (no fake
         * climb transient). Q = host-sweep default. */
        {
          float r_var = sigma_alt * sigma_alt;
          float h0 = 0.0f;
          (void)avt_altitude_m(p1_late, bmp1_p0, &h0);
          avt_kf_init(&kf, h0, r_var);
          avt_kf_set_noise(&kf, r_var, KF_Q_DEFAULT);
          kf_ready = true;
        }
      }
      diag_uart_tx("BMP1 P0: ");
      diag_uart_uint32((uint32_t)bmp1_p0);
      diag_uart_tx("Pa n=");
      diag_uart_uint32(n_p1);
      diag_uart_tx("\r\n");
      /* Dynamic window report: values (not fixed text) so every run shows
         its own early/late/drift/correction/counts. */
      diag_uart_tx("BMP1 CAL: early=");
      diag_uart_uint32((uint32_t)p1_early);
      diag_uart_tx("Pa(n=");
      diag_uart_uint32(n_early);
      diag_uart_tx(") late=");
      diag_uart_uint32((uint32_t)p1_late);
      diag_uart_tx("Pa(n=");
      diag_uart_uint32(n_late);
      diag_uart_tx(") drift=");
      diag_uart_tenths((int32_t)((p1_late - p1_early) * 10.0f), true);
      diag_uart_tx("Pa corr=");
      diag_uart_tenths((int32_t)(p0_corr * 10.0f), true);
      diag_uart_tx("Pa\r\n");
    }
    else
    {
      diag_uart_tx("BMP1 P0: INVALID (no valid samples)\r\n");
    }

    /* BMP2 ground reference (independent mean, same window). */
    if (n_p2 > 0)
    {
      bmp2_p0 = (float)(p2_sum / (int64_t)n_p2);
      bmp2_has_p0 = true;
      diag_uart_tx("BMP2 P0: ");
      diag_uart_uint32((uint32_t)bmp2_p0);
      diag_uart_tx("Pa n=");
      diag_uart_uint32(n_p2);
      diag_uart_tx("\r\n");
    }
    else
    {
      diag_uart_tx("BMP2 P0: INVALID (no valid samples)\r\n");
      if (!bmp2.present)
      {
        diag_uart_tx("BMP2 ERROR\r\n");
      }
    }

    /* Fused ground reference: mean of available P0s. */
    if (bmp1_has_p0 && bmp2_has_p0)
    {
      fused_p0 = (bmp1_p0 + bmp2_p0) * 0.5f;
      fused_has_p0 = true;
    }
    else if (bmp1_has_p0)
    {
      fused_p0 = bmp1_p0;
      fused_has_p0 = true;
    }
    else if (bmp2_has_p0)
    {
      fused_p0 = bmp2_p0;
      fused_has_p0 = true;
    }
    else
    {
      fused_has_p0 = false;
    }
    if (fused_has_p0)
    {
      diag_uart_tx("FUSED P0: ");
      diag_uart_uint32((uint32_t)fused_p0);
      diag_uart_tx("Pa\r\n");
    }
    else
    {
      diag_uart_tx("FUSED P0: INVALID\r\n");
    }

    /* Central flight lifecycle init (production vs test parameters). */
    {
      flight_cfg_t fcfg;
#if FLIGHT_TEST_MODE
      flight_config_test(&fcfg, FLIGHT_TEST_MAIN_ALT_M);
      diag_uart_tx("FLIGHT CONFIG: TEST_MODE=1 APOGEE_MIN=DISABLED MAIN_ALT=");
      diag_uart_int32((int32_t)fcfg.main_alt_m);
      diag_uart_tx("m\r\n");
#else
      flight_config_production(&fcfg);
      diag_uart_tx("FLIGHT CONFIG: PRODUCTION APOGEE_MIN=1250m MAIN_ALT=600m\r\n");
#endif
      flight_init(&flight, &fcfg);
      flight_set_state(&flight, FLIGHT_CALIBRATION);
      launch_confirmed = false;
      ff_confirmed = false;
      ek7_rx_init(&ek7rx); /* Ek-7 command listener starts in NORMAL mode */
      runmode_init(&rmode); /* real-flight lock starts open (pad) */
      diag_uart_tx("RUN MODE: ");
      diag_uart_tx(runmode_str(runmode_current(&rmode, RM_LINK_NORMAL,
                                               FLIGHT_TEST_MODE != 0)));
      diag_uart_tx(" (SIT/SUT accepted until real-flight lock)\r\n");
      dwt_init();          /* A3 cycle-counter timing (diagnostic only) */
    }

    diag_uart_tx("CALIBRATION COMPLETE\r\n");
    diag_uart_tx("CALIBRATION DONE\r\n");
    if (bmp1.present && bmp2.present)
    {
      diag_uart_tx("SENSORS OK\r\n");
    }
    else if (!bmp1.present)
    {
      diag_uart_tx("BMP280 ERROR\r\n");
    }
    else if (!bmp2.present)
    {
      diag_uart_tx("BMP2 ERROR\r\n");
    }
    /* MPU health is reported per-cycle in the TUI (MPU6050 ERROR);
       calibration literately cannot know the loop-local mpu_ok here. */
    /* A6 startup self-test (GOOD/DEGRADED/FAIL summary over existing
     * probe results; drivers untouched). MS absent is DEGRADED by design
     * (normal); MPU+pressure both dead is FAIL -> immediate FAULT, which
     * the 50-sample watchdog would reach ~5 s later anyway. */
    {
      bool mpu_good = mpu_probe_ok && cal_mpu_ok;
      bool press_good = bmp1_has_p0 || bmp2_has_p0;
      diag_uart_tx("SELFTEST: MPU=");
      diag_uart_tx(mpu_good ? "GOOD" : "FAIL");
      diag_uart_tx(" BMP1=");
      diag_uart_tx(bmp1.present ? "GOOD" : "FAIL");
      diag_uart_tx(" BMP2=");
      diag_uart_tx(bmp2.present ? "GOOD" : "ABSENT");
      if (mpu_good && press_good && bmp2.present)
      {
        diag_uart_tx(" => GOOD\r\n");
      }
      else if (press_good || mpu_good)
      {
        diag_uart_tx(" => DEGRADED\r\n");
      }
      else
      {
        diag_uart_tx(" => FAIL\r\n");
      }
      if (!mpu_good && !press_good)
      {
        flight_set_state(&flight, FLIGHT_FAULT);
        diag_uart_tx("FLIGHT FAULT (no usable sensor)\r\n");
      }
    }
    avt_med3_init(&press_med);
    avt_med3_init(&press_med2);
    drop_ref = 0.0f;
    drop_ref_seeded = false;
    last_kf_ms = HAL_GetTick();
    last_flight_ms = HAL_GetTick();
    last_tui_ms = HAL_GetTick();
    last_ek7_tx_ms = HAL_GetTick();
    last_dbg_ms = HAL_GetTick();
    last_tim_ms = HAL_GetTick();
    last_mpu_ms = HAL_GetTick();
    last_bmp_ms = HAL_GetTick();
    last_bmp2_ms = HAL_GetTick();
    flight_set_state(&flight, FLIGHT_WAIT_FOR_LAUNCH);
    if (flight.state != FLIGHT_FAULT) /* self-test may have latched FAULT */
    {
      diag_uart_tx("WAITING FOR LAUNCH\r\n");
    }
    /* Late repeat: early boot lines (RESET_CAUSE) print <100 ms after
     * reset, faster than any host can attach; re-emit post-calibration so
     * the cause is always observable in normal captures. */
    diag_uart_tx("BOOT RESET_CAUSE: ");
    diag_uart_tx(reset_cause_str(boot_reset_cause));
    diag_uart_tx("\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    tim_begin(TIM_LOOP);
    /* --- Ek-7 command listener (official Tablo 1, polled every loop).
     * Bytes arrive via RXNE IRQ ring (single-byte HW buffer would drop
     * bursts between polls); the loop drains the ring bounded per
     * iteration into the command parser and SUT accumulator in parallel.
     * START arms a 1 s delayed activation (Ek-7: +1 s); STOP returns to
     * NORMAL immediately. --- */
    {
      for (uint32_t k = 0; k < EK7_RX_DRAIN_MAX; k++)
      {
        uint8_t rxb = 0;
        bool have = false;
        __disable_irq();
        if (ek7_rx_head != ek7_rx_tail)
        {
          rxb = ek7_rxring[ek7_rx_tail];
          ek7_rx_tail = (uint8_t)((ek7_rx_tail + 1u) % EK7_RXRING_N);
          have = true;
        }
        __enable_irq();
        if (!have)
        {
          break;
        }
        ek7_cmd_t ev = EK7_EV_NONE;
        if (ek7_rx_feed(&ek7rx, rxb, &ev) && ev != EK7_EV_NONE)
        {
          if (ev == EK7_EV_STOP)
          {
            /* Leaving SUT: wipe the synthetic flight so it can never be
             * mistaken for (or poison) a real one. SIT/NORMAL: no reset. */
            bool was_sut = (ek7_mode == EK7MODE_SUT);
            ek7_mode = EK7MODE_NORMAL;
            ek7_pending = EK7_EV_NONE;
            sut_n = 0;
            sut_has_sample = false;
            sut_frame_new = false;
            if (was_sut)
            {
              flight_reset_to_pad();
            }
          }
          else if (runmode_accept(&rmode, ev))
          {
            ek7_pending = ev;
            ek7_activate_ms = HAL_GetTick();
          }
          else
          {
            ek7_cmd_rejected();
          }
        }
#if FLIGHT_TEST_MODE
        /* TEST-only hooks (NOT Ek-7; compiled OUT in production):
         * AA 2E 2E 0D 0A = watchdog test (stop refresh -> bite -> boot
         * shows WATCHDOG). AA 2F 2F 0D 0A = forced BMP re-probe (exercises
         * the DeInit/Init/re-probe recovery path, reports result). */
        static uint8_t test_win[EK7_CMD_LEN];
        static uint8_t test_wn = 0;
        {
          for (uint8_t s = 0; s + 1 < EK7_CMD_LEN; s++)
          {
            test_win[s] = test_win[s + 1];
          }
          test_win[EK7_CMD_LEN - 1] = rxb;
          if (test_wn < EK7_CMD_LEN)
          {
            test_wn++;
          }
          if (test_wn >= EK7_CMD_LEN && test_win[0] == EK7_CMD_HDR &&
              test_win[3] == EK7_FTR1 && test_win[4] == EK7_FTR2)
          {
            if (test_win[1] == 0x2Eu && test_win[2] == 0x2Eu)
            {
              diag_uart_tx("WDG TEST: refresh stopped, expect reset\r\n");
              wdg_test_stop = true;
              test_wn = 0;
            }
            else if (test_win[1] == 0x2Fu && test_win[2] == 0x2Fu)
            {
              i2c_link_recover();
              if (bmp280_init(&bmp1, &hi2c1, 0x76) == HAL_OK)
              {
                diag_uart_tx("REPROBE B1: OK\r\n");
              }
              else
              {
                diag_uart_tx("REPROBE B1: FAIL\r\n");
              }
              if (bmp280_init(&bmp2, &hi2c1, 0x77) == HAL_OK)
              {
                diag_uart_tx("REPROBE B2: OK\r\n");
              }
              else
              {
                diag_uart_tx("REPROBE B2: FAIL (absent)\r\n");
              }
              test_wn = 0;
            }
          }
        }
#endif
        /* SUT synthetic accumulator (0xAB 36 B frames, checksum-gated). */
        if (sut_n == 0 && rxb != EK7_SYN_HDR)
        {
          /* wait for frame start */
        }
        else if (sut_n < EK7_TLM_LEN)
        {
          sut_acc[sut_n++] = rxb;
          if (sut_n >= EK7_TLM_LEN)
          {
            float a, pr, x, y, z, e1, e2, e3;
            if (ek7_parse_tlm(sut_acc, &a, &pr, &x, &y, &z, &e1, &e2, &e3) &&
                a >= SUT_ALT_MIN_M && a <= SUT_ALT_MAX_M &&
                pr >= SUT_PRESS_MIN_MB && pr <= SUT_PRESS_MAX_MB &&
                fabsf(x) <= SUT_ACC_MAX_MS2 && fabsf(y) <= SUT_ACC_MAX_MS2 &&
                fabsf(z) <= SUT_ACC_MAX_MS2 &&
                fabsf(e1) <= SUT_ANG_MAX_DEG &&
                fabsf(e2) <= SUT_ANG_MAX_DEG &&
                fabsf(e3) <= SUT_ANG_MAX_DEG)
            {
              sut_alt_m = a;
              sut_ax = x;
              sut_ay = y;
              sut_az = z;
              sut_ex = e1;
              sut_ey = e2;
              sut_sample_ok = true;
              sut_has_sample = true;
              sut_frame_new = true;
              sut_last_ms = HAL_GetTick();
            }
            else
            {
              sut_sample_ok = false; /* abnormal data: no feed (Ek-7 §SUT) */
            }
            sut_n = 0;
          }
        }
      }
      if (ek7_pending != EK7_EV_NONE &&
          (HAL_GetTick() - ek7_activate_ms) >= EK7_ACTIVATE_MS)
      {
        /* Re-check at activation time: the lock may have engaged during the
         * 200 ms activation delay (command accepted on the pad, rocket
         * already moving). */
        if (!runmode_accept(&rmode, ek7_pending))
        {
          ek7_pending = EK7_EV_NONE;
          ek7_cmd_rejected();
        }
      }
      if (ek7_pending != EK7_EV_NONE &&
          (HAL_GetTick() - ek7_activate_ms) >= EK7_ACTIVATE_MS)
      {
        ek7_mode = (ek7_pending == EK7_EV_SIT) ? EK7MODE_SIT : EK7MODE_SUT;
        ek7_pending = EK7_EV_NONE;
        /* Enter the frame grid now so the first SIT/SUT frame goes out one
         * full period after the command instead of on the very next loop
         * (which shortened the first gap and made the rate look uneven). */
        last_ek7_tx_ms = HAL_GetTick() - EK7_TX_PERIOD_MS;
        sut_n = 0;
        sut_has_sample = false;
        sut_frame_new = false;
        sut_last_ms = HAL_GetTick();
        if (ek7_mode == EK7MODE_SUT)
        {
          /* SUT determinizmi: sentetik ucus her zaman temiz bir yasam
           * dongusuyle baslar (tezgahta kendiliginden ASCENT'e gecmis bir
           * kartla resmi test tekrarlanabilir olmaz). Mevcut konfigurasyon
           * korunur (uretimde uretim esikleri); yalnizca durum/sayaclar
           * sifirlanir. Ucus gecis mantigi degismez. */
          flight_reset_to_pad();
          blue_latch = false; /* her SUT kosusu LED'leri temiz baslar */
        }
      }
      /* SUT RX watchdog (Tablo 7 note: 1000 ms -> safe termination). */
      if (ek7_mode == EK7MODE_SUT &&
          (HAL_GetTick() - sut_last_ms) > EK7_SUT_TIMEOUT_MS)
      {
        ek7_mode = EK7MODE_NORMAL;
        sut_has_sample = false;
        sut_frame_new = false;
        flight_reset_to_pad(); /* sentetik ucus iz birakmaz (STOP ile ayni) */
      }
    }
    /* --- GPS NMEA drain (USART6, NEO-6M). Same ring shape as Ek-7.
     * Bounded per iteration. Raised 64 -> 192: at 9600 baud the line can
     * deliver 960 B/s and the loop runs ~20 Hz, so 64 B/iter could only
     * absorb ~1280 B/s at best and 64/iter x ~10 Hz (the real loop rate
     * with a ~47 ms TUI frame + 50 ms idle) was only ~640 B/s, i.e. under
     * the wire rate. 192/iter x 10 Hz = 1920 B/s gives 2x headroom. The
     * parser tolerates arbitrary chunking, so this changes only latency.
     * Telemetry/status ONLY: never feeds launch/apogee/deployment. --- */
    {
      uint32_t now_ms = HAL_GetTick();
      for (uint32_t k = 0; k < 512u; k++)
      {
        uint8_t rxb = 0;
        bool have = false;
        __disable_irq();
        if (gps_rx_head != gps_rx_tail)
        {
          rxb = gps_rxring[gps_rx_tail];
          gps_rx_tail = (uint16_t)((gps_rx_tail + 1u) % GPS_RXRING_N);
          have = true;
        }
        __enable_irq();
        if (!have)
        {
          break;
        }
        gps_rx_byte(&gps, rxb, now_ms);
        gps_hist[gps_hist_n % GPS_HIST_N] = rxb;
        gps_hist_n++;
        if (rxb == (uint8_t)'$')
        {
          gps_cnt_dollar++;
        }
        else if (rxb == 0xB5u)
        {
          gps_cnt_b5++;
        }
        else if (rxb == (uint8_t)'\n')
        {
          gps_cnt_nl++;
        }
        else if (rxb == (uint8_t)'\r')
        {
          gps_cnt_cr++;
        }
      }
    }
    /* --- GPS UBX config sender (board TX -> GPS RX, pad-only). One poll
     * per loop, at most one frame, never blocking (IT transfer, ~29 ms at
     * 9600 baud for the longest frame). Allowed only on the pad in NORMAL
     * link mode: WAIT_FOR_LAUNCH before any motion, and never during
     * SIT/SUT (link streams stay clean) nor after launch (ABORTED).
     * Absent/silent GPS ends in NOACK; flight never depends on this.
     * A dead NOACK/NAK is retried on the pad with a capped exponential gap
     * (see the retry block below) while still on
     * the pad: the module may power up late, brown out and reboot (which
     * wipes our RAM-side-only settings), or get rewired mid-session. --- */
    {
      uint32_t now_ms = HAL_GetTick();
      /* Pad koşulu (sıkı): yalnızca kalkış beklenirken VE Normal linkte TX.
       * SIT/SUT'ta link akışları temiz kalır; uçuşta/sonrasında poll zaten
       * ABORTED'e çeker, bu blok terminal ABORTED/OK'ya dokunmaz. */
      bool pad = (flight.state == FLIGHT_WAIT_FOR_LAUNCH) &&
                 (ek7_mode == EK7MODE_NORMAL);
      /* Ölü dizi (NOACK/NAK) pad başında, üstel aralıkla, en fazla
       * GPS_CFG_RETRY_MAX_N tur yeniden denenir: kopuk kablo hattı şişirmez
       * (tur başına en fazla STEPS*TRIES çerçeve, aralık 120->480 sn tavanlı),
       * modül geç enerjilenirse ya da kablo oturum içinde değişirse toparlanır.
       * Sayaç dolunca sessizce NOACK/NAK'da kalınır: fault yok, uçuşa etki
       * yok. TUI'daki "GPSCFG: NOACK n/N" sayacı acked/STEPS'ten gelir ve her
       * tur gps_cfg_init ile dürüstçe sıfırlanır. */
      if (pad && (gps_cfg.state == GPS_CFG_NOACK ||
                  gps_cfg.state == GPS_CFG_NAK) &&
          (gps_cfg_retry_n < GPS_CFG_RETRY_MAX_N) &&
          (now_ms - gps_cfg_retry_ms) >= gps_cfg_retry_gap)
      {
        gps_cfg_retry_ms = now_ms;
        gps_cfg_retry_n++;
        gps_cfg_retry_gap = gps_cfg_retry_gap_ms(gps_cfg_retry_n);
        gps_cfg_init(&gps_cfg);
      }
      size_t n = gps_cfg_poll(&gps_cfg, &gps, now_ms, pad, gps_tx_busy,
                              gps_tx_buf, sizeof(gps_tx_buf));
      if (n > 0u)
      {
        /* Latch the busy flag only when the IT transfer actually started.
         * If HAL_UART_Transmit_IT returns HAL_BUSY/HAL_ERROR nothing is in
         * flight, TxCplt will never fire, and a flag set beforehand would
         * latch true forever and wedge the whole sequencer. Not touching
         * the frame count, the pad-only gate or SIT/SUT. */
        gps_tx_busy = (HAL_UART_Transmit_IT(&huart6, gps_tx_buf,
                                            (uint16_t)n) == HAL_OK);
      }
    }
    bool sut_active = (ek7_mode == EK7MODE_SUT);
    bool sit_active = (ek7_mode == EK7MODE_SIT);
    /* --- MPU6050 accel read (tilt source; gyro bias kept from cal).
     * SUT: own sensors ignored (Ek-7), synthetic frame drives the path. --- */
    tim_begin(TIM_IMU);
    uint8_t abuf[6];
    int16_t ax = 0, ay = 0, az = 0;
    bool mpu_ok = false;

    if (!sut_active)
    {
      HAL_StatusTypeDef ast = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                                               0x3B, I2C_MEMADD_SIZE_8BIT,
                                               abuf, 6, 100);
      /* Zero-vector reject: a live MPU6050 never returns six 0x00 bytes
       * (sensor noise alone guarantees nonzero LSBs). All-zero with ACK
       * means a stalled/crashed device or SDA stuck low; feeding it to
       * the tilt EMA drags the filtered vector to zero, which reads as
       * dyn=-1.0/mag=0.0 i.e. fake free-fall (bench cascade observed on
       * the bench: ACC 0, VERT -1000 mg, phantom BLUE). Count and heal. */
      bool alive = (ast == HAL_OK) && ((abuf[0] | abuf[1] | abuf[2] |
                                        abuf[3] | abuf[4] | abuf[5]) != 0u);
      if (alive)
      {
        ax = (int16_t)((abuf[0] << 8) | abuf[1]);
        ay = (int16_t)((abuf[2] << 8) | abuf[3]);
        az = (int16_t)((abuf[4] << 8) | abuf[5]);
        mpu_ok = true;
        mpu_fails = 0;
      }
      else if (++mpu_fails >= MPU_RECOVER_AFTER)
      {
        /* Sustained dead accel: re-wake the device (SLEEP bit clears on
         * brownout/ESD crash). One I2C write per burst; unbounded, the
         * sensor is flight-critical and the attempt is cheap. */
        mpu_fails = 0;
        uint8_t wake = 0x00;
        if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B,
                              I2C_MEMADD_SIZE_8BIT, &wake, 1, 100) == HAL_OK)
        {
          diag_uart_tx("I2C RECOVERED MPU\r\n");
        }
      }
    }
    /* No accel bias subtraction: tilt compares raw current accel
       against the raw startup reference (same frame, same scale). */
    int32_t acx = (int32_t)ax;
    int32_t acy = (int32_t)ay;
    int32_t acz = (int32_t)az;

    /* --- Tilt: EMA low-pass on raw accel, same frame/scale as ref.
           Validity judged on filtered accel vs startup reference. --- */
    bool tilt_valid = false;
    float tilt_deg = 0.0f;
    if (mpu_ok)
    {
      if (!tilt_finit)
      {
        tilt_fx = acx;
        tilt_fy = acy;
        tilt_fz = acz;
        tilt_finit = true;
      }
      else
      {
        tilt_fx += (acx - tilt_fx) / TILT_EMA_N;
        tilt_fy += (acy - tilt_fy) / TILT_EMA_N;
        tilt_fz += (acz - tilt_fz) / TILT_EMA_N;
      }
      if (avt_tilt_mag_ok((float)tilt_fx, (float)tilt_fy, (float)tilt_fz,
                          (float)gravity_ref_x, (float)gravity_ref_y,
                          (float)gravity_ref_z))
      {
        tilt_deg = avt_tilt_deg((float)tilt_fx, (float)tilt_fy, (float)tilt_fz,
                                (float)gravity_ref_x, (float)gravity_ref_y,
                                (float)gravity_ref_z);
        tilt_valid = true;
      }
    }
    bool tilt_cond = tilt_valid
                     && avt_in_band(tilt_deg, AVT_TILT_LO_DEG, AVT_TILT_HI_DEG);

    /* --- Tilt band (Ek-7 status bit3 source + telemetry parity). --- */
    /* --- Gyro read: calibration bias only.
           The Mahony 6-DOF estimator was removed (it was diagnostic-only and
           its angle drifted because the accelerometer channel it corrects with
           was unreliable). One 6-byte accel read per cycle instead of two
           keeps the MPU bus quiet, which is what the accel channel needs. --- */
    uint8_t gbuf[6];
    if (!sut_active)
    {
      (void)HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43,
                             I2C_MEMADD_SIZE_8BIT, gbuf, 6, 100);
    }

    tim_end(TIM_IMU);
    tim_begin(TIM_PROC);

    /* --- Dual pressure reads: BMP280 @0x76 + BMP2 @0x77 ---
     * Health is tracked per sensor; fusion never crashes on one failure
     * (fallback to the survivor, invalid only when both fail).
     * SUT: skipped (own sensors ignored, synthetic drives the path). */
    tim_end(TIM_IMU);
    tim_begin(TIM_PRS);
    bool b1_ok = false;
    uint32_t p1 = 0;
    if (!sut_active && bmp1.present)
    {
      int32_t tc = 0;
      /* Sifir-veri/NACK sertlestirmesi: HAL_OK tek basina yetmez; telafi
       * cikisi fiziksel zarftan gecer (sifir, takilmis-hat sacmaligi elenir).
       * Fallback korunur: sagkalan sensor combine'da tek basina yasar. */
      b1_ok = (bmp280_read(&bmp1, &tc, &p1) == HAL_OK) &&
              avt_press_ok((float)p1);
      if (!b1_ok && ++bmp_fails >= I2C_RECOVER_AFTER &&
          bmp_recovers < I2C_RECOVER_MAX)
      {
        bmp_recovers++;
        bmp_fails = 0;
        i2c_link_recover();
        if (bmp280_init(&bmp1, &hi2c1, 0x76) == HAL_OK)
        {
          diag_uart_tx("I2C RECOVERED BMP\r\n");
        }
      }
      else if (b1_ok)
      {
        bmp_fails = 0;
      }
    }
    bool b2_ok = false;
    uint32_t p2 = 0;
    if (!sut_active && bmp2.present)
    {
      int32_t tc2 = 0;
      /* BMP1 ile ayni sozlesme: zarf disi cop gecersiz, fallback korunur. */
      b2_ok = (bmp280_read(&bmp2, &tc2, &p2) == HAL_OK) &&
              avt_press_ok((float)p2);
      if (!b2_ok && ++b2_fails >= I2C_RECOVER_AFTER &&
          b2_recovers < I2C_RECOVER_MAX)
      {
        b2_recovers++;
        b2_fails = 0;
        i2c_link_recover();
        if (bmp280_init(&bmp2, &hi2c1, 0x77) == HAL_OK)
        {
          diag_uart_tx("I2C RECOVERED MS\r\n");
        }
      }
      else if (b2_ok)
      {
        b2_fails = 0;
      }
    }
    tim_end(TIM_PRS);
    if (mpu_ok)
    {
      last_mpu_ms = HAL_GetTick();
    }
    if (b1_ok)
    {
      last_bmp_ms = HAL_GetTick();
    }
    if (b2_ok)
    {
      last_bmp2_ms = HAL_GetTick();
    }
    avt_bmp_state_t bmp_state = avt_bmp_state(b1_ok, b2_ok,
                                              (float)p1, (float)p2);
    bool sensors_usable = avt_alt_usable(bmp_state);

    /* --- Relative altitude from fused ground reference ---
     * Chain per spec: compensation -> median3 per sensor -> combine
     * (average / single / invalid) -> shared EMA -> baro altitude.
     * Raw p1/p2 in TUI stay untouched. */
    bool alt_valid = false;
    float alt_m = 0.0f;
    float p_med = 0.0f;
    bool p_med_ok = false;
    float p1_med = 0.0f, p2_med = 0.0f;
    bool p1_med_ok = false, p2_med_ok = false;
    float p_comb = 0.0f;
    if (b1_ok)
    {
      p1_med = avt_med3_push(&press_med, (float)p1);
      p1_med_ok = true;
    }
    if (b2_ok)
    {
      p2_med = avt_med3_push(&press_med2, (float)p2);
      p2_med_ok = true;
    }
    if (flight_combine_pressure(p1_med_ok, p2_med_ok, p1_med, p2_med,
                                 &p_comb) && fused_has_p0 &&
        avt_press_ok(fused_p0))
    {
      p_med = p_comb;
      p_med_ok = true;
      /* Light low-pass on fused pressure: kills few-Pa jitter so the
         demo hysteresis does not flicker. P in STATUS stays raw. */
      if (isfinite(p_med) && avt_press_ok(p_med))
      {
        if (!press_finit)
        {
          press_f = p_med;
          press_finit = true;
        }
        else
        {
          press_f = avt_ema(press_f, p_med);
        }
      }
      /* DELIK NOTU (bulgu, Src/main.c:2026 eski): `alt_valid =
       * avt_altitude_m(press_f, fused_p0, &alt_m)` dogrudan alt_m'ye
       * yaziyordu; NaN/inf/tasma copu `true` ile gecerli cikiyor, alt_m
       * kirleniyor, TUI'daki (int32_t)(alt_m*10.0f) donusumu tanimsiz
       * davranisa giriyor (400000000 m gozlemi) ve LoRa i16 telafi
       * disi sayi tasiyordu. Artik: gecici degiskene yaz, sonlu +
       * zarf-ici dogrula, ancak o zaman alt_m'ye yaz. Gecersizde
       * alt_m'ye YAZMA YOK (eski gecerli dondurulmez, sifir uydurulmaz). */
      float alt_tmp = 0.0f;
      if (isfinite(press_f) && avt_altitude_m(press_f, fused_p0, &alt_tmp) &&
          isfinite(alt_tmp) && alt_tmp <= AVT_ALT_LIMIT_M &&
          alt_tmp >= -AVT_ALT_LIMIT_M)
      {
        alt_m = alt_tmp;
        alt_valid = true;
      }
      else
      {
        alt_valid = false;
      }
    }
    bool alt_cond = avt_alt_rise(prev_alt_rise, alt_m, alt_valid);
    prev_alt_rise = alt_cond;
    /* Finger gate: noise-relative thresholds on (alt - slow_ref).
       Fast presses outrun ref and fire; slow ambient is absorbed into ref
       so the gate neither self-fires nor latches on a drifted rest.
       Absolute ALT / apogee / Kalman paths above are untouched. */
    if (alt_valid)
    {
      if (!drop_ref_seeded)
      {
        /* First valid sample defines the rest altitude. Without this the
         * ref sat at 0 while alt_m was already ~-0.3 m, i.e. pre-armed. */
        drop_ref = alt_m;
        drop_ref_seeded = true;
      }
      else
      {
        drop_ref += (alt_m - drop_ref) * AVT_DROP_REF_ALPHA;
      }
    }
    drop_delta_m = alt_valid ? (alt_m - drop_ref) : 0.0f;
    drop_gate_m = avt_drop_gate_m(drop_sigma);
    /* Direct gate (no streak, no velocity): a light press must fire at once.
     * Hardening attempts (N-consecutive + descent-rate) are reverted: they
     * demanded too hard a press. Self-fire on room gusts is the accepted
     * cost; the TUI DROP line exposes delta vs gate so it stays visible. */
    bool drop_cond = avt_alt_drop_n(prev_alt_drop, drop_delta_m, alt_valid,
                                    drop_sigma);
    prev_alt_drop = drop_cond;

    /* --- Parallel Kalman altitude (DIAGNOSTIC ONLY): median pressure
           input, never the EMA output. ALT/EMA/apogee path above is
           untouched and apogee keeps using ALT (no Kalman trigger). --- */
    kalt_valid = false;
    if (kf_ready && p_med_ok && fused_has_p0)
    {
      float z_raw = 0.0f;
      bool z_ok = avt_altitude_m(p_med, fused_p0, &z_raw);
      uint32_t now_ms = HAL_GetTick();
      float dt = (now_ms - last_kf_ms) / 1000.0f;
      last_kf_ms = now_ms;
      avt_kf_predict(&kf, dt);
      if (avt_kf_update(&kf, z_raw, z_ok))
      {
        kalt_m = kf.h;
        kalt_v = kf.v;
        kalt_valid = true;
      }
    }

    /* --- Apogee/descent trigger (bench demo, PRESERVED): LED fires on the
           way DOWN past apogee, not on rise. Latched; same var drives LEDs. --- */
    avt_phase_t phase = avt_apogee_update(&apogee, alt_m, alt_valid);
    bool descent_cond = (phase == AVT_PHASE_DESCENT);

    /* --- Flight lifecycle (real-flight state machine on top of bench) ---
     * Launch: |a| streak vs gravity ref (debounced N=3, no single-spike).
     * Apogee: max + drop 2 m + vel < -2 m/s (+1250 m PROD gate / disabled TEST).
     * Free-fall: |a| under-streak after apogee. TEST bench: gravity-
     * referenced signed vertical-accel trigger + independent barometric
     * fall trigger (hand up-then-fast-down), display/BLUE/UART only.
     * Tilt/angle is diagnostic-only (TUI + Mahony) and feeds no LED and
     * no decision. Deployments: one-shot, state-validated, sensor-interlocked inside
     * flight_step(). */
    {
      float gref_mag = avt_vec_mag((float)gravity_ref_x,
                                   (float)gravity_ref_y,
                                   (float)gravity_ref_z);
      float acc_mag = avt_vec_mag((float)acx, (float)acy, (float)acz);
      bool acc_ok = mpu_ok && isfinite(acc_mag) && gref_mag > 0.0f;
      /* SUT substitution (Ek-7): synthetic body-frame accel (m/s2, Z down)
       * drives the SAME gates via g-unit equivalents. Board-up = -Zbody. */
      float sut_mag_ratio = 1.0f;
      float sut_dyn_g = 0.0f;
      bool sut_acc_ok = false;
      float sut_tilt_deg = 0.0f;
      if (sut_active && sut_has_sample && sut_sample_ok && gref_mag > 0.0f)
      {
        float smag = sqrtf(sut_ax * sut_ax + sut_ay * sut_ay +
                           sut_az * sut_az);
        sut_mag_ratio = smag / EK7_MS2_PER_G;
        sut_dyn_g = (-sut_az / EK7_MS2_PER_G) - 1.0f;
        sut_acc_ok = isfinite(smag) && isfinite(sut_dyn_g);
        sut_tilt_deg = sqrtf(sut_ex * sut_ex + sut_ey * sut_ey);
        if (sut_acc_ok)
        {
          acc_mag = sut_mag_ratio * gref_mag;
          acc_ok = true;
        }
      }
      bool tilt_band = tilt_cond;
      /* Vertical channel from the ALREADY-FILTERED accel (existing tilt
       * EMA, no new smoother) projected on the calibration gravity unit.
       * Diagnostic in PROD, descent trigger in TEST (bench_enable). */
      {
        float dyn_g = 0.0f, mag_ratio = 1.0f;
        bool vok = tilt_finit &&
                   flight_vert_dyn_g((float)tilt_fx, (float)tilt_fy,
                                     (float)tilt_fz, (float)gravity_ref_x,
                                     (float)gravity_ref_y,
                                     (float)gravity_ref_z, &dyn_g, &mag_ratio);
        flight.vert_dyn_g = vok ? dyn_g : 0.0f;
        flight.vert_mag_ratio = vok ? mag_ratio : 1.0f;
        flight.vert_valid = vok && mpu_ok;
        if (sut_active && sut_acc_ok)
        {
          flight.vert_dyn_g = sut_dyn_g;
          flight.vert_mag_ratio = sut_mag_ratio;
          flight.vert_valid = true;
          tilt_band = avt_in_band(sut_tilt_deg, AVT_TILT_LO_DEG,
                                  AVT_TILT_HI_DEG);
        }
        (void)flight_bench_push(&flight, flight.vert_dyn_g,
                                flight.vert_mag_ratio,
                                flight.vert_valid);
      }
      if (flight.state == FLIGHT_WAIT_FOR_LAUNCH)
      {
        if (acc_ok)
        {
          launch_confirmed = flight_launch_push(&flight, acc_mag, gref_mag);
        }
        else
        {
          launch_confirmed = false;
        }
      }
      if (flight.state == FLIGHT_APOGEE_CONFIRMED)
      {
        if (acc_ok)
        {
          ff_confirmed = flight_freefall_push(&flight, acc_mag, gref_mag);
        }
        else
        {
          ff_confirmed = false;
        }
      }
      uint32_t now_ms = HAL_GetTick();
      float dt_flight = (now_ms - last_flight_ms) / 1000.0f;
      /* SUT steps once per FRAME (fresh sample only): re-stepping the same
       * sample at loop rate injects zero-velocity copies and pins the
       * apogee streak. dt then spans step-to-step (~100 ms). NORMAL/SIT
       * step every loop exactly as before. */
      bool sut_step = !sut_active || sut_frame_new;
      sut_frame_new = false;
      if (sut_step)
      {
        last_flight_ms = now_ms;
      }
      /* Bench baro channel from the DELIVERED fused altitude (existing
       * pipeline, no new smoother). Combined BENCH_FALL = accel OR baro,
       * any mode, state-independent, display + BLUE + UART only: never
       * advances state, never deploys (BENCH_FALL != DROGUE_DEPLOY). */
      /* SUT altitude substitution (Ek-7): synthetic altitude drives the
       * SAME altitude gates; sensor path ignored. Interlocks preserved. */
      float use_alt = alt_m;
      bool use_alt_valid = alt_valid;
      bool use_sensors = sensors_usable;
      if (sut_active)
      {
        use_alt = sut_alt_m;
        use_alt_valid = sut_has_sample && sut_sample_ok;
        use_sensors = use_alt_valid;
      }
      /* Motion liveness for the baro fall channel (NORMAL only):
       * SUT scripts its own physics, so it bypasses with always-true.
       * Any real handling disturbs the vert channel far above rest noise;
       * a room pressure step does not move the board at all. */
      if (sut_active)
      {
        flight.bench_motion_ok = true;
      }
      else
      {
        bool moved = flight_motion_seen(flight.vert_dyn_g,
                                        flight.vert_mag_ratio,
                                        flight.vert_valid);
        if (moved)
        {
          motion_ms = now_ms;
          motion_have = true;
        }
        flight.bench_motion_ok =
            motion_have &&
            ((now_ms - motion_ms) < FLIGHT_BENCH_MOTION_TMO_MS);
      }
      (void)flight_bench_baro_push(&flight, use_alt, use_alt_valid,
                                   dt_flight);
      {
        bool bench_fall_now = flight_bench_fall(&flight);
        if (bench_fall_now && !prev_bench &&
            ek7_mode == EK7MODE_NORMAL) /* binary stays parseable in test */
        {
          diag_uart_tx("DUSUS ALGILANDI\r\n");
        }
        prev_bench = bench_fall_now;
      }
      flight_state_t prev = flight.state;
      if (sut_step)
      {
        (void)flight_step(&flight, launch_confirmed, ff_confirmed,
                          use_alt, use_alt_valid, dt_flight, use_sensors);
      }
      ek7_tilt_band = tilt_band; /* Ek-7 status bit3 source */
      ek7_use_alt = use_alt;
      ek7_use_alt_valid = use_alt_valid;
      /* Real-flight lock: engages (latched) once the REAL sensors have taken
       * the rocket well off the pad. SUT frames never count. */
      {
        bool was_locked = rmode.locked;
        runmode_update(&rmode, rm_link_now(), &flight);
        if (rmode.locked && !was_locked && ek7_mode == EK7MODE_NORMAL)
        {
          diag_uart_tx("REAL FLIGHT LOCK: SIT/SUT DISABLED\r\n");
        }
      }
      /* Text transition events: NORMAL mode only (binary parseable in test). */
      if (flight.state != prev && ek7_mode == EK7MODE_NORMAL)
      {
        if (prev == FLIGHT_WAIT_FOR_LAUNCH && flight.state == FLIGHT_ASCENT)
        {
          diag_uart_tx("LAUNCH DETECTED\r\n");
        }
        else if (flight.state == FLIGHT_APOGEE_CONFIRMED)
        {
          diag_uart_tx("APOGEE CONFIRMED\r\n");
        }
        else if (flight.state == FLIGHT_FREE_FALL)
        {
          diag_uart_tx("FREE FALL DETECTED\r\n");
        }
        else if (flight.state == FLIGHT_DROGUE_DEPLOYED)
        {
          diag_uart_tx("DROGUE DEPLOY EVENT\r\n");
          drogue_events++;
        }
        else if (flight.state == FLIGHT_MAIN_DEPLOYED)
        {
          diag_uart_tx("MAIN DEPLOY EVENT\r\n");
          main_events++;
        }
        else if (flight.state == FLIGHT_COMPLETE)
        {
          diag_uart_tx("FLIGHT COMPLETE\r\n");
        }
        else if (flight.state == FLIGHT_FAULT)
        {
          diag_uart_tx("FLIGHT FAULT\r\n");
        }
      }
      /* Per-cycle sensor errors are visible in the throttled TUI
         (BMP280/BMP2/MPU6050 ERROR); no UART spam here. */
    }

    /* --- LEDs via meaningful API (change-only writes).
     * LD2 (system/flight): bench DESCENT latched OR flight APOGEE+ latched;
     *   FAULT blinks at ~2 Hz (caller toggles via tick below).
     * BLUE (bench/fall): BENCH_FALL_DETECTED (accel OR baro, any mode).
     *   Tilt NEVER drives BLUE or any decision (diagnostic TUI only).
     * YELLOW (deploy/drop): finger DROP immediate OR drogue/main latched;
     *   FAULT blinks. RED: FAULT-only. --- */
    {
      bool fault = (flight.state == FLIGHT_FAULT);
      bool fault_blink = fault && ((HAL_GetTick() / 250U) % 2U == 0U);
      bool ld2_on = descent_cond ||
                    flight.state == FLIGHT_APOGEE_CONFIRMED ||
                    flight.state == FLIGHT_FREE_FALL ||
                    flight.state == FLIGHT_DROGUE_DEPLOYED ||
                    flight.state == FLIGHT_DESCENT ||
                    flight.state == FLIGHT_MAIN_DEPLOYED ||
                    flight.state == FLIGHT_COMPLETE ||
                    flight.drogue_deployed || flight.main_deployed;
      bool yel_on = drop_cond || flight.drogue_deployed || flight.main_deployed;
      /* BLUE latch, two disjoint sources (never at the same time):
       * - Bench (TEST NORMAL): real hand-drop / free-fall trigger.
       *   Bench is inert in production and in SUT, so this leg can only
       *   fire on the bench.
       * - SUT (any build): synthetic descent onset = APOGEE_CONFIRMED..
       *   COMPLETE. The raw accel bench gate is NOT used here: on smooth
       *   synthetic coast it confirms mid-ASCENT (~6 s), while blue must
       *   mean "the rocket started falling". */
      if (!sut_active && flight_bench_fall(&flight))
      {
        blue_latch = true;
      }
      if (sut_active && flight.state >= FLIGHT_APOGEE_CONFIRMED &&
          flight.state <= FLIGHT_COMPLETE)
      {
        blue_latch = true;
      }
      bool bench_fall_led = blue_latch;
      /* RED: BLUE (latched fall) AND YELLOW (finger drop/deploy) together. */
      bool red_on = bench_fall_led && yel_on;
      if (fault)
      {
        led_write(LD2_GPIO_Port, LD2_Pin, &prev_ld2, fault_blink);
        led_write(BLUE_LED_GPIO_Port, BLUE_LED_Pin, &prev_blue, fault_blink);
        led_write(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, &prev_yel, !fault_blink);
        if (fault_blink)
        {
          LED_Red_On(&prev_red);
        }
        else
        {
          LED_Red_Off(&prev_red);
        }
      }
      else
      {
        if (ld2_on)
        {
          LED_System_On(&prev_ld2);
        }
        else
        {
          LED_System_Off(&prev_ld2);
        }
        if (bench_fall_led)
        {
          LED_Fall_On(&prev_blue);
        }
        else
        {
          LED_Fall_Off(&prev_blue);
        }
        if (yel_on)
        {
          LED_Deploy_On(&prev_yel);
        }
        else
        {
          LED_Deploy_Off(&prev_yel);
        }
        if (red_on)
        {
          LED_Red_On(&prev_red);
        }
        else
        {
          LED_Red_Off(&prev_red);
        }
      }
    }

    /* --- LoRa telemetry uplink: at most ONE packet per loop iteration on the
     * cadence enforced inside lora_poll(). Reads only flight/ek7_mode/gps and
     * writes nothing back: no launch, apogee, drogue, main or deployment
     * influence in either direction. Never called from an interrupt.
     * lora_poll() refuses in SIT/SUT (LORA_ALLOW_SIT_SUT == 0). --- */
    {
      lora_tlm_t lora_lt;
      uint32_t lora_now_ms = HAL_GetTick();
      bool lora_fix = gps_has_fix(&gps, lora_now_ms);
      memset(&lora_lt, 0, sizeof lora_lt);
      lora_lt.flight_state = (uint16_t)flight.state;
      lora_lt.ek7_mode = (uint16_t)ek7_mode;
      /* alt_m is the SAME fused-P0 barometric altitude the flight machine
       * uses this iteration; V3 carries it as signed decimetres (0.1 m),
       * INT32_MIN when invalid. */
      lora_lt.alt_m = alt_m;
      lora_lt.alt_valid = alt_valid;
      lora_lt.vel_mps = flight.vel_mps;
      lora_lt.vel_valid = flight.vel_valid;
      lora_lt.gps_fix = lora_fix;
      lora_lt.sats = gps.sats;
      lora_lt.lat_e6 = gps.lat_e6;
      lora_lt.lon_e6 = gps.lon_e6;
      lora_lt.batt_mv = 0u; /* no battery ADC in this build: 0 = unknown */
      /* Egiklik v2: tilt_deg/tilt_valid yerelden; lean govde X/Y'nin
       * filtreli vektor buyuklugune bolumu (buyukluk 0 ise 0). Yaw yok. */
      lora_lt.tilt_valid = tilt_valid;
      lora_lt.tilt_deg = tilt_deg;
      {
        float _fx = (float)tilt_fx, _fy = (float)tilt_fy,
              _fz = (float)tilt_fz;
        float _m = sqrtf(_fx * _fx + _fy * _fy + _fz * _fz);
        if (tilt_valid && _m > 0.0f)
        {
          lora_lt.lean_x = _fx / _m;
          lora_lt.lean_y = _fy / _m;
        }
        else
        {
          lora_lt.lean_x = 0.0f;
          lora_lt.lean_y = 0.0f;
        }
      }
      if (lora_fix)
      {
        lora_lt.flags |= LORA_F_GPS_FIX;
      }
      if (blue_latch)
      {
        lora_lt.flags |= LORA_F_BLUE_LED;
      }
      /* SARI/KIRMIZI bayraklar fiziksel LED'lerin aynisi: asagidaki
       * ifadeler LED blogundaki yel_on/red_on ile kelimesi kelimesine ayni
       * (TUI LED satiri dahil). Eskiden SARI yalniz drop_cond'a, KIRMIZI
       * yalniz FLIGHT_FAULT'a bakiyordu; oysa SARI drop/drogue/main
       * latch'larinda, KIRMIZI MAVI latch + SARI birlikteyken yanar. */
      if (drop_cond || flight.drogue_deployed || flight.main_deployed)
      {
        lora_lt.flags |= LORA_F_YELLOW_LED;
      }
      if (blue_latch &&
          (drop_cond || flight.drogue_deployed || flight.main_deployed))
      {
        lora_lt.flags |= LORA_F_RED_LED;
      }
      (void)lora_poll(lora_now_ms, &lora_lt, lora_tx_buf,
                      sizeof(lora_tx_buf));
    }

    /* --- Ek-7 binary TX (SIT/SUT modes, 10 Hz loop rate).
     * SIT: live-sensor frame (Tablo 2/3). SUT: status word frame
     * (Tablo 5/6). Text TUI/TILTDBG/edge messages are suppressed in test
     * modes so the binary stream stays parseable. --- */
    tim_end(TIM_PROC);
    tim_begin(TIM_TX);
    /* AUX-dusuk sure olcumu (sadece TUI tanisi; ucus/sensor/LoRa-GPS/Ek-7'ye etkisi yok):
     * Her TX sonrasi AUX duser; dusuk kalis suresi hava suresini ele verir
     * (4.8k->~70ms). Her dongude tek okuma + karsilastirma, bekleme yok.
     * LORA_AUX_GPIO_Port/Pin lora.h'tan gelir (main.c zaten include eder) ve pinin
     * giris olarak kuruldugu MX_GPIO_Init'te gorulur; erisim dogrulandi.
     * Kesinlik: HAL_GetTick 1ms cozunurluklu, ~70ms olcumu icin yeterli.
     * Surekli dusuk kalan (yukselmeyen) AUX'da olcum uretilmez, sahte veri basilmaz. */
    static bool aux_init = false;
    static bool aux_prev_hi = true;
    static uint32_t aux_fall_ms = 0;
    static bool aux_in_low = false;
    static uint32_t aux_last_ms = 0;
    static bool aux_have = false;
    static uint32_t aux_min_ms = 0;
    static uint32_t aux_max_ms = 0;
    {
      GPIO_PinState aux_now = HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin);
      bool now_hi = (aux_now == GPIO_PIN_SET);
      uint32_t aux_now_ms = HAL_GetTick();
      if (!aux_init)
      {
        aux_prev_hi = now_hi;
        aux_in_low = !now_hi;
        aux_fall_ms = aux_now_ms;
        aux_init = true;
      }
      else
      {
        if (aux_prev_hi && !now_hi)
        {
          aux_fall_ms = aux_now_ms; /* dusen kenar: olcum basladi */
          aux_in_low = true;
        }
        else if (!aux_prev_hi && now_hi && aux_in_low)
        {
          uint32_t d = aux_now_ms - aux_fall_ms; /* unsigned: tick tasmasi guvenli */
          aux_last_ms = d;
          if (!aux_have || d < aux_min_ms)
          {
            aux_min_ms = d;
          }
          if (!aux_have || d > aux_max_ms)
          {
            aux_max_ms = d;
          }
          aux_have = true;
          aux_in_low = false;
        }
        aux_prev_hi = now_hi;
      }
    }
    if (sit_active || (ek7_mode == EK7MODE_SUT))
    {
      /* Ek-7 10 Hz gate (own timer; the TUI gate is a separate 100 ms tick). */
      if ((HAL_GetTick() - last_ek7_tx_ms) >= EK7_TX_PERIOD_MS)
      {
        last_ek7_tx_ms = HAL_GetTick();
        if (sit_active)
        {
          uint8_t tlm[EK7_TLM_LEN];
          float pax, pay, paz, pex, pey, pez;
          float gref_live = avt_vec_mag((float)gravity_ref_x,
                                        (float)gravity_ref_y,
                                        (float)gravity_ref_z);
          ek7_map_acc((float)tilt_fx, (float)tilt_fy, (float)tilt_fz,
                      gref_live, &pax, &pay, &paz);
          ek7_map_ang((float)tilt_fx, (float)tilt_fy, (float)tilt_fz,
                      &pex, &pey, &pez);
          /* SIT display floor: pad-relative altitude noise straddles 0
           * (±0.5 m room wander), and the official GUI counts every
           * sub-altMin frame as an error against its default 0..10000 m
           * range. True pad altitude IS 0, so negatives carry no
           * information here; floor them. SIT-TX ONLY: flight, Kalman,
           * TUI and SUT paths keep the unclamped value. */
          float talt = (alt_valid && alt_m > 0.0f) ? alt_m : 0.0f;
          float tprs = press_finit ? (press_f / 100.0f) : 0.0f;
          ek7_build_tlm(tlm, talt, tprs, pax, pay, paz, pex, pey, pez);
          HAL_UART_Transmit(&huart2, tlm, EK7_TLM_LEN, 100);
        }
        else
        {
          uint8_t stf[EK7_STATUS_LEN];
          uint16_t stw = ek7_status_word(&flight, ek7_tilt_band,
                                        ek7_use_alt_valid, ek7_use_alt);
          ek7_build_status(stf, stw);
          HAL_UART_Transmit(&huart2, stf, EK7_STATUS_LEN, 100);
        }
      }
    }
    else
    /* --- Live TUI every ~100 ms (elapsed time, not loop count).
           Clear + home (+hide cursor) redraws the same block in place;
           idempotent, so a late-attaching terminal renders cleanly.
           100 ms keeps the terminal live while leaving the 100 ms LoRa
           cadence affordable (a worst-case ~950 B frame costs ~85 ms
           of blocking UART time at 115200). No system("clear"), no shell:
           MCU-side ANSI only. --- */
    if ((HAL_GetTick() - last_tui_ms) >= 100U)
    {
      last_tui_ms = HAL_GetTick();
      tui_begin(); /* stage the whole frame; one transmit at tui_flush() */
      diag_uart_tx("\033[2J\033[H\033[?25l");
      diag_uart_tx("UP: ");
      diag_uart_uint32(HAL_GetTick() / 1000U);
      diag_uart_tx(" s\r\n");
      /* --- Altitude block: raw pressure, filtered pressure, altitude. --- */
      diag_uart_tx("P1: ");
      if (b1_ok)
      {
        diag_uart_uint32(p1);
        diag_uart_tx(" Pa  PF: ");
      }
      else
      {
        diag_uart_tx("INVALID  PF: ");
      }
      if (press_finit)
      {
        diag_uart_int32((int32_t)press_f);
      }
      else
      {
        diag_uart_tx("INVALID");
      }
      diag_uart_tx(" Pa\r\nALT: ");
      if (alt_valid)
      {
        diag_uart_tenths((int32_t)(alt_m * 10.0f), true);
        diag_uart_tx(" m");
      }
      else
      {
        diag_uart_tx("INVALID");
      }
      diag_uart_tx("  KF: ");
      if (kalt_valid)
      {
        diag_uart_tenths((int32_t)(kalt_m * 10.0f), true);
        diag_uart_tx(" m\r\n");
      }
      else
      {
        diag_uart_tx("INVALID\r\n");
      }
      /* --- Fall triggers: the two bench channels that light BLUE. --- */
      diag_uart_tx("VERT: ");
      if (flight.vert_valid)
      {
        diag_uart_int32((int32_t)(flight.vert_dyn_g * 1000.0f));
        diag_uart_tx(" mg");
      }
      else
      {
        diag_uart_tx("INVALID");
      }
      diag_uart_tx("  ACC: ");
      diag_uart_int32((int32_t)avt_vec_mag((float)tilt_fx, (float)tilt_fy,
                                           (float)tilt_fz));
      diag_uart_tx(" LSB  NEG: ");
      diag_uart_uint32(flight.bench_n);
      diag_uart_tx("  MOT: ");
      diag_uart_tx(flight.bench_motion_ok ? "1" : "0");
      diag_uart_tx("\r\n");
      /* --- Latched fall + LED mirror. --- */
      diag_uart_tx("FALL: ");
      diag_uart_tx(flight.bench_confirmed ? "ACC" : "--");
      diag_uart_tx(flight.bench_baro_confirmed ? "BARO" : "--");
      diag_uart_tx("  BARO_V: ");
      if (flight.bench_baro_vel_valid)
      {
        diag_uart_tenths((int32_t)(flight.bench_baro_vel_mps * 10.0f), true);
        diag_uart_tx(" m/s\r\n");
      }
      else
      {
        diag_uart_tx("INVALID\r\n");
      }
      /* Finger gate: live offset vs the ON threshold it is tested against.
       * Fires when DELTA <= -GATE; both in millimetres so a light tap is
       * visible instead of hidden behind the 0.1 m display resolution. */
      diag_uart_tx("DROP: ");
      if (alt_valid)
      {
        diag_uart_int32((int32_t)(drop_delta_m * 1000.0f));
        diag_uart_tx(" mm");
      }
      else
      {
        diag_uart_tx("INVALID");
      }
      diag_uart_tx(" / ");
      diag_uart_int32((int32_t)(drop_gate_m * 1000.0f));
      diag_uart_tx(" mm  DREF_M: ");
      diag_uart_tenths((int32_t)(drop_ref * 10.0f), true);
      diag_uart_tx(" m\r\n");
      diag_uart_tx("LED: BLUE=");
      diag_uart_tx(blue_latch ? "1" : "0");
      diag_uart_tx(" YEL=");
      diag_uart_tx((drop_cond || flight.drogue_deployed || flight.main_deployed)
                       ? "1"
                       : "0");
      diag_uart_tx(" RED=");
      diag_uart_tx((blue_latch && (drop_cond || flight.drogue_deployed ||
                                  flight.main_deployed))
                       ? "1"
                       : "0");
      diag_uart_tx("\r\n");
      /* --- Flight lifecycle. --- */
      diag_uart_tx("FLIGHT: ");
      diag_uart_tx(flight_state_str(flight.state));
#if FLIGHT_TEST_MODE
      diag_uart_tx(" TEST\r\n");
#else
      diag_uart_tx(" PROD\r\n");
#endif
      diag_uart_tx("MODE: ");
      diag_uart_tx(runmode_str(runmode_current(&rmode, rm_link_now(),
                                               FLIGHT_TEST_MODE != 0)));
      diag_uart_tx(rmode.locked ? "  LOCK: LOCKED" : "  LOCK: OPEN");
      diag_uart_tx("  REJ=");
      diag_uart_uint32(rmode.rejected);
      diag_uart_tx("\r\n");
      diag_uart_tx("MAX: ");
      if (flight.has_max)
      {
        diag_uart_tenths((int32_t)(flight.max_alt_m * 10.0f), true);
        diag_uart_tx(" m  VEL: ");
      }
      else
      {
        diag_uart_tx("INVALID  VEL: ");
      }
      if (flight.vel_valid)
      {
        diag_uart_tenths((int32_t)(flight.vel_mps * 10.0f), true);
        diag_uart_tx(" m/s\r\n");
      }
      else
      {
        diag_uart_tx("INVALID\r\n");
      }
      diag_uart_tx("APOGEE: DROP=");
      diag_uart_tx(flight.drop_flag ? "1" : "0");
      diag_uart_tx(" VEL=");
      diag_uart_tx(flight.vel_flag ? "1" : "0");
      diag_uart_tx(" n=");
      diag_uart_uint32(flight.apogee_n);
      diag_uart_tx("\r\nMAIN: ");
      diag_uart_tenths((int32_t)(flight.cfg.main_alt_m * 10.0f), false);
      diag_uart_tx(" m  DEPLOY: D=");
      diag_uart_tx(flight.drogue_deployed ? "1" : "0");
      diag_uart_tx(" M=");
      diag_uart_tx(flight.main_deployed ? "1" : "0");
      diag_uart_tx(" ("/*,*/ "");
      diag_uart_uint32(drogue_events);
      diag_uart_tx("/");
      diag_uart_uint32(main_events);
      diag_uart_tx(")\r\n");
      /* --- Sensor health, one line. --- */
      diag_uart_tx("SENS: MPU=");
      diag_uart_tx(mpu_ok ? "OK" : "FAIL");
      diag_uart_tx(" B1=");
      diag_uart_tx(!bmp1.present ? "ABSENT" : (b1_ok ? "OK" : "FAIL"));
      diag_uart_tx(" B2=");
      diag_uart_tx(!bmp2.present ? "ABSENT" : (b2_ok ? "OK" : "FAIL"));
      diag_uart_tx(bmp_state == AVT_BOTH_OK ? " OK\r\n" : " DEGRADED\r\n");
      /* --- GPS status (USART6/NEO-6M, NORMAL text TUI only: the SIT/SUT
       * binary frames above are byte-exact and know nothing of GPS).
       * ABSENT = no valid NMEA frame yet; NOFIX = talking but no/stale
       * fix; FIX = fresh valid position. Telemetry only, never flight. */
      {
        uint32_t now_ms = HAL_GetTick();
        diag_uart_tx("GPS: ");
        if (!gps.present)
        {
          diag_uart_tx("ABSENT rx=");
          diag_uart_uint32(gps.rx_bytes);
          diag_uart_tx(" err=");
          diag_uart_uint32(gps_rx_err);
          /* Ring-full drops: non-zero means the module IS talking and we
           * lost bytes, which is why present=0 (ABSENT) above. */
          diag_uart_tx(" drop=");
          diag_uart_uint32(gps_rx_drop);
          diag_uart_tx(" $/B5/NL=");
          diag_uart_uint32(gps_cnt_dollar);
          diag_uart_tx("/");
          diag_uart_uint32(gps_cnt_b5);
          diag_uart_tx("/");
          diag_uart_uint32(gps_cnt_nl);
          diag_uart_tx(" CR=");
          diag_uart_uint32(gps_cnt_cr);
          /* Auto raw dump (diagnostic, one-shot): the finger counts alone cannot
           * tell "module sends NMEA without line terminators" from "module sends
           * something that is not NMEA at all" — a '$' with no '\n' is
           * physically impossible for real NMEA. Print the first 64 bytes once so
           * the bench can decide by eye. Purely diagnostic; flight never reads
           * gps_hist. */
          {
            static bool gps_dump_done = false;
            (void)gps_dump_done;
            static uint8_t gps_dump_ct = 0;
            if (((gps_dump_ct++) % 16u) == 0u && gps_hist_n > 4u)
            {
              diag_uart_tx("GPSRAW ");
              uint32_t cnt = gps_hist_n;
              if (cnt > 32u) { cnt = 32u; }
              for (uint32_t i = 0; i < cnt; i++)
              {
                uint8_t v = gps_hist[i];
                char pair[3];
                pair[0] = "0123456789ABCDEF"[v >> 4];
                pair[1] = "0123456789ABCDEF"[v & 0x0Fu];
                pair[2] = '\0';
                diag_uart_tx(pair);
              }
              diag_uart_tx("\r\n");
            }
          }
          diag_uart_tx("\r\n");
        }
        else if (!gps_has_fix(&gps, now_ms))
        {
          diag_uart_tx("NOFIX sats=");
          diag_uart_uint32(gps.sats);
          diag_uart_tx(" rx=");
          diag_uart_uint32(gps.rx_bytes);
          diag_uart_tx("\r\n");
        }
        else
        {
          diag_uart_tx("FIX sats=");
          diag_uart_uint32(gps.sats);
          diag_uart_tx(" lat=");
          diag_uart_coord(gps.lat_e6);
          diag_uart_tx(" lon=");
          diag_uart_coord(gps.lon_e6);
          if (gps.alt_valid)
          {
            diag_uart_tx(" alt=");
            diag_uart_tenths((int32_t)(gps.alt_m * 10.0f), true);
            diag_uart_tx("m");
          }
          if (gps.speed_valid)
          {
            diag_uart_tx(" spd=");
            diag_uart_tenths((int32_t)(gps.speed_mps * 10.0f), false);
            diag_uart_tx("m/s");
          }
          diag_uart_tx("\r\n");
        }
      }
      /* --- GPS UBX config state (pad-only setup, see sender above). --- */
      diag_uart_tx("GPSCFG: ");
      diag_uart_tx(gps_cfg_str(gps_cfg.state));
      diag_uart_tx(" ");
      diag_uart_uint32(gps_cfg.acked);
      diag_uart_tx("/");
      diag_uart_uint32(GPS_CFG_STEPS);
      diag_uart_tx("\r\n");
      /* LoRa uplink counters, staged into the same frame (one transmit).
       * auxbusy = AUX-low samples right after due transmits: climbs iff the
       * module takes bytes toward air (ok alone cannot prove that). */
      diag_uart_tx("LORA: ");
      diag_uart_uint32(lora_seq());
      diag_uart_tx(" ok=");
      diag_uart_uint32(lora_ok_count());
      diag_uart_tx(" drop=");
      diag_uart_uint32(lora_drop_count());
      diag_uart_tx(" auxbusy=");
      diag_uart_uint32(lora_aux_busy());
      diag_uart_tx(" txerr=");
      diag_uart_uint32(lora_txerr_count());
      diag_uart_tx(" reinit=");
      diag_uart_uint32(lora_reinit_count());
      diag_uart_tx(" airskip=");
      diag_uart_uint32(lora_airbusy_count());
      /* Yeni tani alani (mevcut LORA alanlari aynen korunur): son AUX-dusuk suresi
       * + akan min/max (ms). Gozlem yoksa "--" basilir, sifir uydurulmaz. */
      diag_uart_tx(" auxlo=");
      if (aux_have)
      {
        diag_uart_uint32(aux_last_ms);
        diag_uart_tx("ms min/max=");
        diag_uart_uint32(aux_min_ms);
        diag_uart_tx("/");
        diag_uart_uint32(aux_max_ms);
        diag_uart_tx("ms");
      }
      else
      {
        diag_uart_tx("--");
      }
      diag_uart_tx("\r\n");
      tui_flush(); /* ONE transmit for the whole staged frame */
    }

    /* A3 timing report every 5 s (NORMAL mode only): per-section
     * min/avg/max microseconds + sample count. Worst-case (max) is monotonic
     * since boot; loop work excludes the HAL_Delay(50) idle. */
    if (ek7_mode == EK7MODE_NORMAL &&
        (HAL_GetTick() - last_tim_ms) >= 5000U)
    {
      last_tim_ms = HAL_GetTick();
      uint32_t mhz = SystemCoreClock / 1000000U;
      if (mhz == 0)
      {
        mhz = 1;
      }
      static const char *tn[TIM_N] = {"IMU", "PRS", "PROC", "TX", "LOOP"};
      tui_begin();
      diag_uart_tx("TIM us min/avg/max: ");
      for (int s = 0; s < TIM_N; s++)
      {
        uint32_t avg = (tim_n[s] > 0)
                           ? (uint32_t)(tim_sum[s] / tim_n[s] / mhz)
                           : 0;
        diag_uart_tx(tn[s]);
        diag_uart_tx(" ");
        diag_uart_uint32(tim_n[s] > 0 ? tim_min[s] / mhz : 0);
        diag_uart_tx("/");
        diag_uart_uint32(avg);
        diag_uart_tx("/");
        diag_uart_uint32(tim_max[s] / mhz);
        diag_uart_tx(" ");
      }
      diag_uart_tx("n=");
      diag_uart_uint32(tim_n[TIM_LOOP]);
      diag_uart_tx("\r\n");
      tui_flush();
    }

    /* Fault visibility without TUI spam: TILTDBG at 2 s, only while the
       tilt decision is invalid (filtered vector, magnitudes, guard).
       NORMAL mode only (binary stream stays parseable in test modes). */
    if (ek7_mode == EK7MODE_NORMAL &&
        !tilt_valid && (HAL_GetTick() - last_dbg_ms) >= 2000U)
    {
      last_dbg_ms = HAL_GetTick();
      float amag = avt_vec_mag((float)tilt_fx, (float)tilt_fy, (float)tilt_fz);
      float gmag = avt_vec_mag((float)gravity_ref_x, (float)gravity_ref_y,
                               (float)gravity_ref_z);
      tui_begin();
      diag_uart_tx("TILTDBG | amag=");
      diag_uart_int32((int32_t)amag);
      diag_uart_tx(" | gmag=");
      diag_uart_int32((int32_t)gmag);
      diag_uart_tx(" | err=");
      if (gmag > 0.0f)
      {
        float dev = (amag > gmag) ? (amag - gmag) : (gmag - amag);
        diag_uart_tenths((int32_t)(1000.0f * dev / gmag), false);
      }
      else
      {
        diag_uart_tx("?");
      }
      /* Gecerlilik bayragi: bu satir yalniz tilt gecersizken basilir
       * (o yuzden normalde tv=N), ama ifade gercektir: kosul genislerse
       * Y de basar. Son gecerli deger dondurulmez (sahte olur); pakette
       * tilt=0xFFFF + lean=0 zaten vardir, burasi yalniz ayirt eder. */
      diag_uart_tx("% | tv=");
      diag_uart_tx(tilt_valid ? "Y" : "N");
      diag_uart_tx("\r\n");
      tui_flush();
    }
    tim_end(TIM_TX);
    tim_end(TIM_LOOP);
#if FLIGHT_TEST_MODE
    if (!wdg_test_stop)
#endif
    {
      iwdg_refresh(); /* A11: every loop iteration */
    }

    /* B1 (blue button, PC13, active low): one-shot GPS line diagnostic.
     * Prints the last 64 raw USART6 bytes as hex + the parser's current
     * line buffer as text. Edge-triggered, 300 ms debounce. Answers
     * "what is actually arriving" without continuous NMEA streaming. */
    {
      bool pressed = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
      uint32_t now_ms = HAL_GetTick();
      if (pressed && !b1_prev && (now_ms - b1_ms) > 300u)
      {
        b1_ms = now_ms;
        static const char hexd[] = "0123456789ABCDEF";
        /* B1 snapshot goes through the same staging buffer: the hex dump is
         * 64 one-shot 2-byte tokens, which alone was ~200 blocking calls. */
        tui_begin();
        diag_uart_tx("GPSRAW n=");
        diag_uart_uint32(gps_hist_n);
        diag_uart_tx(" rx=");
        diag_uart_uint32(gps.rx_bytes);
        diag_uart_tx(" err=");
        diag_uart_uint32(gps_rx_err);
        diag_uart_tx(" present=");
        diag_uart_tx(gps.present ? "1" : "0");
        diag_uart_tx("\r\nGPSHEX: ");
        for (uint8_t i = 0; i < GPS_HIST_N; i++)
        {
          uint8_t b = gps_hist[(gps_hist_n + i) % GPS_HIST_N];
          char h[3] = {hexd[(b >> 4) & 0xFu], hexd[b & 0xFu], '\0'};
          diag_uart_tx(h);
        }
        diag_uart_tx("\r\nGPSLINE: ");
        for (uint8_t i = 0; i < gps.n; i++)
        {
          char c[2] = {gps.line[i], '\0'};
          diag_uart_tx((gps.line[i] >= 32 && gps.line[i] < 127) ? c : ".");
        }
        diag_uart_tx("\r\n");
        tui_flush();
      }
      b1_prev = pressed;
    }

    /* Idle pacing. NORMAL: 10 ms loop so the 100 ms LoRa deadline is met
     * with margin: a transmitting iteration costs ~10 (idle) + ~15 (work,
     * amortised TUI at the 100 ms gate below) + ~43 ms (41 B @9600 UART),
     * i.e. ~70-90 ms, so the 10 Hz deadline fires every tick. TUI refreshes
     * at its own 100 ms gate (still live). SIT/SUT: 5 ms, so the Ek-7
     * 100 ms gate can actually be hit on time (a 50 ms idle quantises a
     * 100 ms period into 100/150 ms steps and measured 8.9 Hz instead of
     * 10 Hz). Frame TIMING only: the sensor path itself is untouched, and
     * SUT still steps once per frame (see sut_step), never per loop.
     * Flight uses measured dt (dt_flight), launch/free-fall streaks span
     * seconds, and the DROP baseline EMA (AVT_DROP_REF_ALPHA) has a
     * tens-of-seconds time constant (tau = loop period / alpha) against
     * descent timescales of seconds: the faster loop does not change
     * flight decisions, only their latency (shorter). */
    HAL_Delay((ek7_mode == EK7MODE_NORMAL) ? 10 : 5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* Ek-7 listener: RXNE interrupt + 1-byte IT reception into the ring.
     NVIC group is set in HAL_MspInit (priority 0 grouping). */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  (void)HAL_UART_Receive_IT(&huart2, &ek7_rx_tmp, 1);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function (GPS, hand-written: NOT
  * in .ioc. On the next CubeMX regeneration add USART6 async TX+RX on
  * PC7/AF8, or this function is lost. USART2/VCOM is untouched.)
  * No TX pin is configured: the GPS RX wire is physically disconnected.
  * @param None
  * @retval None
  */
/* GPS UART baud. PROBE: the Uno test saw 86 B/s at 9600 with no valid
 * sentence — the classic signature of a module actually running at 38400.
 * Flip this one value to probe. Restore 9600 if 38400/115200 stay silent. */
#define GPS_UART_BAUD (9600u)

static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = GPS_UART_BAUD;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* Same pattern as the Ek-7 listener: low-rate NMEA needs only 1-byte IT.
   * GPS absence (no bytes ever) is normal and never a fault. */
  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  (void)HAL_UART_Receive_IT(&huart6, &gps_rx_tmp, 1);
}

/* LoRa radio UART: E220-900T22D in Mode 0 (M0/M1 tied low on the board),
 * transparent transmission, 9600 8N1, no flow control. PB6/PB7 AF7 are
 * configured in the USART1 branch of HAL_UART_MspInit. No RX interrupt and
 * no AUX: the send cadence is the pacing. A failure here is reported as one
 * TUI line and then ignored - it is never a fault. */
#define LORA_UART_BAUD (9600u)

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = LORA_UART_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  lora_uart_status = HAL_UART_Init(&huart1);
  /* Deliberately no Error_Handler: a missing or failed radio must never
   * stop the flight. USART1 gets no NVIC line: the transmit is
   * bounded-blocking (LORA_TX_TMO_MS) from the main loop. */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin (blue button, active low).
   * Pull-up (not CubeMX-default NOPULL): without it the pin floats and a
   * press is unreadable; the EXTI falling edge is unused by this firmware
   * (polled in the main loop), so plain input is correct and quiet. */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* External LEDs, active high (anodes via series resistor, cathodes on
     common GND). BLUE PA10/D2 = tilt, YELLOW PB5/D4 = drop, RED PB4/D5 =
     FAULT-only (new). PB3 untouched (SWO). PB4 is the NJTRST pin reused
     as GPIO (SWD-only project, PA13/PA14); verified free in .ioc/Src/Inc.
     Safe default: all LOW/OFF before mode init, red never pulses at boot. */
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = YELLOW_LED_Pin;
  HAL_GPIO_Init(YELLOW_LED_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = RED_LED_Pin;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);
  /* LoRa E220 mode/config pins (datasheet §6/§7): M0 = PB10/D6 and
   * M1 = PA8/D7 are push-pull outputs, default Mode 0 (both LOW =
   * transparent). AUX = PA9/D8 is the module-busy input (low = busy).
   * lora_cfg_boot() drives M0/M1 into Mode 3 for the register check and
   * returns them to Mode 0 before the flight loop starts. */
  HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LORA_M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_M0_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LORA_M1_Pin;
  HAL_GPIO_Init(LORA_M1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LORA_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
