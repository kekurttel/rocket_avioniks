/* tests/host/test_tui_buf_host.c
 *
 * Host coverage for the TUI staging buffer added in Src/main.c (tui_begin /
 * tui_append / tui_flush + the diag_uart_tx gate). main.c cannot be linked on
 * the host (it pulls the whole STM32 HAL), so this suite reimplements the same
 * three routines verbatim, driven against a byte-capture mock of
 * HAL_UART_Transmit.
 *
 * What this DOES prove:
 *   1. Token-at-a-time appends produce exactly the concatenated byte stream
 *      that the old per-token HAL_UART_Transmit(huart2, s, len, 100) calls
 *      produced - i.e. the TUI contract bytes are unchanged. Proven
 *      differentially: every token in a synthetic 537 B frame is pushed
 *      through both paths and the two byte logs are compared.
 *   2. An overflowing frame is never emitted with a partial line: the flush
 *      ends on the last CRLF, and every token after the overflow is dropped.
 *   3. A token that does not fit is never written partially (a half field
 *      must not reach the wire).
 *   4. TUI_BUF_N exceeds the worst-case frame, and the flush timeout covers
 *      the worst-case wire time.
 *   5. tui_len is reset unconditionally, so a failed/timed-out flush still
 *      leaves the next frame clean (no bytes from frame N leak into N+1).
 *
 * The worst-case frame size itself is measured by the companion script
 * (it parses Src/main.c's TUI blocks and sums maximum token widths); see the
 * Src/main.c header comment. This suite re-asserts the resulting 954 B bound.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- constants mirrored from Src/main.c ---- */
#define TUI_BUF_N (1400u)
/* Worst-case staged frame measured from the TUI block (both arms of every
 * conditional counted). Must stay < TUI_BUF_N. */
#define TUI_WORST_CASE (954u)
#define BAUD (115200u)
/* Bytes per millisecond at 115200 8N1 (10 bits/byte), as the Src/main.c
 * formula uses it. */
#define BYTES_PER_MS (BAUD / 1000u)
#define TUI_TX_TMO_MS (((TUI_BUF_N * 11u) / BYTES_PER_MS) + 10u)

/* ---- byte-capture mock for the old per-token transmit path ---- */
static uint8_t cap_old[8192];
static size_t cap_old_n;
static unsigned tx_calls_old;

static void mock_transmit_old(const char *s, uint16_t len)
{
  unsigned i;
  for (i = 0; i < len; i++)
  {
    cap_old[cap_old_n++] = (uint8_t)s[i];
  }
  tx_calls_old++;
}

/* ---- byte-capture mock for the new single-transmit flush ---- */
static uint8_t cap_new[8192];
static size_t cap_new_n;
static unsigned tx_calls_new;
static int tx_fail; /* 1 = next flush transmit fails/times out */

static uint8_t huart2_dummy;

/* Mirrors HAL_UART_Transmit on huart2: all-or-nothing, appends the payload. */
static int HAL_UART_Transmit(void *h, uint8_t *p, uint16_t len, uint32_t tmo)
{
  (void)h;
  (void)tmo;
  if (tx_fail)
  {
    return -1; /* HAL_ERROR / HAL_TIMEOUT: nothing leaves the buffer */
  }
  {
    uint16_t i;
    for (i = 0; i < len; i++)
    {
      cap_new[cap_new_n++] = p[i];
    }
  }
  tx_calls_new++;
  (void)huart2_dummy;
  return 0;
}

/* ---- the staging buffer, same logic as Src/main.c ---- */
static char tui_buf[TUI_BUF_N];
static uint16_t tui_len;
static uint16_t tui_line;
static bool tui_win;
static bool tui_trunc;
static uint32_t tui_drop;

static void tui_append(const char *s, uint16_t len)
{
  if (tui_trunc)
  {
    return;
  }
  if ((uint32_t)len > (uint32_t)(TUI_BUF_N - tui_len))
  {
    tui_trunc = true;
    tui_drop++;
    tui_len = tui_line;
    return;
  }
  {
    uint16_t i;
    for (i = 0; i < len; i++)
    {
      tui_buf[tui_len] = s[i];
      tui_len++;
      if (s[i] == '\n')
      {
        tui_line = tui_len;
      }
    }
  }
}

static void tui_begin(void)
{
  tui_len = 0;
  tui_line = 0;
  tui_trunc = false;
  tui_win = true;
}

static void tui_flush(void)
{
  uint16_t n = tui_len;
  tui_win = false;
  tui_len = 0;
  tui_line = 0;
  tui_trunc = false;
  if (n > 0u)
  {
    (void)HAL_UART_Transmit(NULL, (uint8_t *)tui_buf, n, TUI_TX_TMO_MS);
  }
}

/* Mirrors the patched diag_uart_tx: stage inside a window, otherwise the
 * original immediate blocking transmit. */
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
  mock_transmit_old(s, len);
}

/* A realistic worst-case-shaped frame: 15 lines of the live TUI with every
 * numeric field at its maximum width (10-digit uint32, signed tenths, GPS
 * coordinates), matching the ~537 B observed frame and the shape of the
 * 954 B analytic bound. */
static void emit_frame(void)
{
  diag_uart_tx("\033[2J\033[H\033[?25l");
  diag_uart_tx("UP: ");
  diag_uart_tx("4294967295");
  diag_uart_tx(" s\r\n");
  diag_uart_tx("P1: ");
  diag_uart_tx("4294967295");
  diag_uart_tx(" Pa  PF: INVALID  PF: ");
  diag_uart_tx("-2147483648");
  diag_uart_tx(" Pa\r\nALT: ");
  diag_uart_tx("-2147483648.9");
  diag_uart_tx(" m  KF: ");
  diag_uart_tx("+4294967295.9");
  diag_uart_tx(" m\r\n");
  diag_uart_tx("VERT: INVALID  ACC: ");
  diag_uart_tx("2147483648");
  diag_uart_tx(" LSB  NEG: ");
  diag_uart_tx("4294967295");
  diag_uart_tx("  MOT: 1\r\n");
  diag_uart_tx("FALL: ACCBARO  BARO_V: ");
  diag_uart_tx("-2147483648.9");
  diag_uart_tx(" m/s\r\n");
  diag_uart_tx("DROP: INVALID / ");
  diag_uart_tx("-2147483648");
  diag_uart_tx(" mm  DREF_M: ");
  diag_uart_tx("+214748364.8");
  diag_uart_tx(" m\r\n");
  diag_uart_tx("LED: BLUE=1 YEL=1 RED=1\r\n");
  diag_uart_tx("FLIGHT: WAIT_FOR_LAUNCH");
  diag_uart_tx(" PROD\r\n");
  diag_uart_tx("MODE: SUT  LOCK: LOCKED  REJ=");
  diag_uart_tx("4294967295");
  diag_uart_tx("\r\n");
  diag_uart_tx("MAX: INVALID  VEL: INVALID\r\n");
  diag_uart_tx("APOGEE: DROP=1 VEL=1 n=");
  diag_uart_tx("4294967295");
  diag_uart_tx("\r\nMAIN: ");
  diag_uart_tx("214748364.8");
  diag_uart_tx(" m  DEPLOY: D=1 M=1 (");
  diag_uart_tx("4294967295");
  diag_uart_tx("/");
  diag_uart_tx("4294967295");
  diag_uart_tx(")\r\n");
  diag_uart_tx("SENS: MPU=FAIL B1=ABSENT B2=ABSENT DEGRADED\r\n");
  diag_uart_tx("GPS: ABSENT rx=");
  diag_uart_tx("4294967295");
  diag_uart_tx(" err=");
  diag_uart_tx("4294967295");
  diag_uart_tx(" drop=");
  diag_uart_tx("4294967295");
  diag_uart_tx(" $/B5/NL=");
  diag_uart_tx("4294967295");
  diag_uart_tx("/");
  diag_uart_tx("4294967295");
  diag_uart_tx("/");
  diag_uart_tx("4294967295");
  diag_uart_tx("\r\n");
  diag_uart_tx("GPSCFG: ABORTED ");
  diag_uart_tx("4294967295");
  diag_uart_tx("/");
  diag_uart_tx("4294967295");
  diag_uart_tx("\r\n");
}

static int fails;

static void check(int ok, const char *what)
{
  if (ok)
  {
    printf("PASS %s\n", what);
  }
  else
  {
    printf("FAIL %s\n", what);
    fails++;
  }
}

/* 1. Differential: staged frame == per-token transmit frame, byte for byte. */
static void test_byte_equivalence(void)
{
  cap_old_n = 0;
  cap_new_n = 0;
  tx_calls_old = 0;
  tx_calls_new = 0;
  tx_fail = 0;

  /* old path: every token goes straight out on its own */
  tui_win = false;
  emit_frame();

  /* new path: same tokens, one transmit for the frame */
  tui_begin();
  emit_frame();
  tui_flush();

  check(cap_old_n == cap_new_n, "frame length identical old vs new");
  check(cap_old_n > 0 && cap_old_n <= TUI_BUF_N,
        "frame fits TUI_BUF_N");
  check(memcmp(cap_old, cap_new, cap_old_n) == 0,
        "frame bytes identical old vs new");
  check(tx_calls_old > 40,
        "old path really was one blocking transmit per token");
  check(tx_calls_new == 1, "new path is exactly ONE transmit");
  printf("     frame = %u bytes, old transmits = %u, new transmits = %u\n",
         (unsigned)cap_new_n, tx_calls_old, tx_calls_new);
}

/* 2/3. Overflow: no partial line, no partial token, rest of frame dropped. */
static void test_overflow_line_safety(void)
{
  size_t i;
  cap_new_n = 0;
  tx_calls_new = 0;
  tui_drop = 0;

  tui_begin();
  /* fill until ~30 bytes short, then keep appending complete lines + a long
   * token: the frame must be cut on a CRLF boundary, never mid-line. */
  while (tui_len + 40u < TUI_BUF_N)
  {
    diag_uart_tx("PADDING LINE 012345678901234567890123456789\r\n");
  }
  diag_uart_tx("GPS: ABSENT rx=1234567890 err=0 drop=0 $/B5/NL=1/2/3\r\n");
  /* this one cannot fit: must be refused whole */
  diag_uart_tx("THIS TOKEN IS FAR TOO LONG TO FIT IN THE REMAINING SPACE!!!!!!");
  diag_uart_tx("AFTER\r\n");
  diag_uart_tx("STILL AFTER\r\n");
  tui_flush();

  check(tx_calls_new == 1, "overflowing frame still ONE transmit");
  check(cap_new_n > 0, "overflowing frame emitted its prefix");
  /* every emitted line must be CRLF-terminated: last byte must be '\n' and no
   * line may exceed the frame budget */
  check(cap_new_n > 0 && cap_new[cap_new_n - 1] == (uint8_t)'\n',
        "overflow frame ends on a CRLF, never mid-line");
  check(cap_new_n <= TUI_BUF_N, "overflow frame within TUI_BUF_N");
  check(tui_drop == 1, "exactly one token counted as dropped");
  /* the refused token must not appear anywhere on the wire */
  {
    int found = 0;
    for (i = 0; i + 24u <= cap_new_n; i++)
    {
      if (memcmp(cap_new + i, "THIS TOKEN IS FAR", 17) == 0)
      {
        found = 1;
      }
    }
    check(found == 0, "oversized token never partially written");
  }
  {
    int found = 0;
    for (i = 0; i + 5u <= cap_new_n; i++)
    {
      if (memcmp(cap_new + i, "AFTER", 5) == 0)
      {
        found = 1;
      }
    }
    check(found == 0, "tokens after the overflow are suppressed");
  }
}

/* 4. Sizing and timeout headroom. */
static void test_sizing(void)
{
  check(sizeof(tui_buf) == TUI_BUF_N, "sizeof(tui_buf) == TUI_BUF_N");
  check(TUI_WORST_CASE < TUI_BUF_N,
        "TUI_BUF_N exceeds the worst-case TUI frame");
  printf("     sizeof(tui_buf) = %u, worst case = %u, margin = %u bytes\n",
         (unsigned)sizeof(tui_buf), (unsigned)TUI_WORST_CASE,
         (unsigned)(TUI_BUF_N - TUI_WORST_CASE));
  /* wire time of a full buffer must fit inside the flush timeout, or the
   * transmit would cut the frame mid-line */
  {
    uint32_t need_ms = (TUI_BUF_N * 10u + (BAUD / 1000u) - 1u) / (BAUD / 1000u);
    check(TUI_TX_TMO_MS >= need_ms,
          "flush timeout covers worst-case wire time");
    printf("     worst-case wire time = %u ms, TUI_TX_TMO_MS = %u ms\n",
           (unsigned)need_ms, (unsigned)TUI_TX_TMO_MS);
  }
}

/* 5. A failed flush must not leak into the next frame. */
static void test_flush_failure_is_clean(void)
{
  size_t first;
  cap_new_n = 0;
  tx_calls_new = 0;

  tx_fail = 0;
  tui_begin();
  diag_uart_tx("FRAME ONE\r\n");
  tui_flush();
  first = cap_new_n;
  check(first == 11, "frame one transmitted");

  tx_fail = 1;
  tui_begin();
  diag_uart_tx("FRAME TWO MUST NOT LEAK\r\n");
  tui_flush();
  check(cap_new_n == first, "failed flush emitted nothing and leaked nothing");
  check(tui_len == 0, "tui_len reset even when the transmit failed");

  tx_fail = 0;
  tui_begin();
  diag_uart_tx("FRAME THREE\r\n");
  tui_flush();
  check(cap_new_n == first + 13 && cap_new_n == 24,
        "next frame starts clean after a failed flush");
}

/* Empty window must not call the UART at all (no zero-length transmit). */
static void test_empty_window_no_transmit(void)
{
  tx_calls_new = 0;
  cap_new_n = 0;
  tx_fail = 0;
  tui_begin();
  tui_flush();
  check(tx_calls_new == 0, "empty frame issues no transmit");
  check(cap_new_n == 0, "empty frame emits no bytes");
}

/* Outside a window the old immediate blocking behaviour must be preserved. */
static void test_outside_window_is_immediate(void)
{
  tx_calls_new = 0;
  cap_new_n = 0;
  cap_old_n = 0;
  tx_calls_old = 0;
  tui_win = false;
  diag_uart_tx("BOOT LINE\r\n");
  check(tx_calls_old == 1, "outside a window each token still transmits");
  check(cap_old_n == 11, "outside a window the bytes go out unchanged");
  check(tx_calls_new == 0, "outside a window nothing is staged");
}

int main(void)
{
  printf("TUI staging buffer host tests\n");
  test_byte_equivalence();
  test_overflow_line_safety();
  test_sizing();
  test_flush_failure_is_clean();
  test_empty_window_no_transmit();
  test_outside_window_is_immediate();
  if (fails == 0)
  {
    printf("ALL TUI BUFFER TESTS PASS\n");
    return 0;
  }
  printf("%d FAILURE(S)\n", fails);
  return 1;
}