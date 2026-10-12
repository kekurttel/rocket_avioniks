/* Run-mode separation: REAL flight vs TEST build vs SIT vs SUT.
 *
 * HAL-independent, deterministic, host-testable.
 *
 * Why this exists: SIT/SUT commands arrive over the same UART that is
 * always listening. SUT makes the board ignore its own sensors and trust
 * synthetic frames, so it must never be possible to enter it once the
 * rocket has really left the pad.
 *
 * Rule set (single place):
 * - STOP is always accepted (it only returns to NORMAL, harmless).
 * - SIT/SUT are accepted until the REAL-FLIGHT LOCK engages.
 * - The lock engages (latched, only a reset clears it) when the board is
 *   NOT in SUT (so the flight state is driven by real sensors) and the
 *   flight state is past the pad (ASCENT or later, incl. FAULT) AND the
 *   max altitude has reached RUNMODE_LOCK_ALT_M.
 *   The altitude condition is deliberate: hand shaking on the bench can
 *   trip the launch detector (ASCENT) but never climbs tens of metres,
 *   so bench SIT/SUT runs keep working; a real flight always does.
 * - Leaving SUT (STOP or timeout) is expected to reset the flight state
 *   (done by the caller) so synthetic flights never look like real ones.
 */
#ifndef RUNMODE_H
#define RUNMODE_H

#include <stdbool.h>
#include <stdint.h>

#include "ek7.h"
#include "flight.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Max altitude (m, pad-relative) above which a non-SUT ASCENT counts as a
 * real flight. 10x the biggest hand test (2-3 m), far below any apogee. */
#define RUNMODE_LOCK_ALT_M (30.0f)

typedef enum
{
  RUNMODE_REAL = 0, /* production build, normal operation */
  RUNMODE_TEST,     /* bench/test build, normal operation */
  RUNMODE_SIT,      /* Ek-7 SIT active (live sensors, binary telemetry) */
  RUNMODE_SUT       /* Ek-7 SUT active (synthetic data) */
} runmode_t;

typedef enum
{
  RM_LINK_NORMAL = 0,
  RM_LINK_SIT,
  RM_LINK_SUT
} rm_link_t;

typedef struct
{
  bool locked;       /* real-flight lock (latched) */
  uint32_t rejected; /* number of rejected SIT/SUT commands */
} runmode_state_t;

void runmode_init(runmode_state_t *rm);

/* Call once per loop with the current link mode and flight state. */
void runmode_update(runmode_state_t *rm, rm_link_t link, const flight_t *f);

/* May this command be executed now? Counts a rejection when not.
 * NULL-safe: a NULL state refuses SIT/SUT (fail closed), accepts STOP. */
bool runmode_accept(runmode_state_t *rm, ek7_cmd_t cmd);

/* Display mode for logs/TUI. */
runmode_t runmode_current(const runmode_state_t *rm, rm_link_t link,
                          bool test_build);
const char *runmode_str(runmode_t m);

#ifdef __cplusplus
}
#endif

#endif /* RUNMODE_H */
