/* Host tests for run-mode separation (runmode.c, HAL-independent). */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

#include "runmode.h"

static int n_pass = 0;
#define CHECK(c) do { assert(c); n_pass++; } while (0)

static void mk(flight_t *f, flight_state_t st, bool has_max, float max_m)
{
  flight_cfg_t c;
  flight_config_production(&c);
  flight_init(f, &c);
  f->state = st;
  f->has_max = has_max;
  f->max_alt_m = max_m;
}

int main(void)
{
  runmode_state_t rm;
  flight_t f;

  /* Fresh state: everything accepted, nothing locked. */
  runmode_init(&rm);
  CHECK(!rm.locked && rm.rejected == 0u);
  CHECK(runmode_accept(&rm, EK7_EV_SIT));
  CHECK(runmode_accept(&rm, EK7_EV_SUT));
  CHECK(runmode_accept(&rm, EK7_EV_STOP));
  CHECK(!runmode_accept(&rm, EK7_EV_NONE));

  /* On the pad / hand-test: ASCENT from shaking but only a few metres. */
  mk(&f, FLIGHT_WAIT_FOR_LAUNCH, false, 0.0f);
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(!rm.locked);
  mk(&f, FLIGHT_ASCENT, true, 3.0f);
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(!rm.locked);
  runmode_update(&rm, RM_LINK_SIT, &f);
  CHECK(!rm.locked);
  CHECK(runmode_accept(&rm, EK7_EV_SUT)); /* bench SUT still works */

  /* Just under the threshold: still open. */
  mk(&f, FLIGHT_ASCENT, true, RUNMODE_LOCK_ALT_M - 0.1f);
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(!rm.locked);

  /* Synthetic flight (SUT) never locks, even at thousands of metres. */
  mk(&f, FLIGHT_COMPLETE, true, 1300.0f);
  runmode_update(&rm, RM_LINK_SUT, &f);
  CHECK(!rm.locked);

  /* Max altitude without the state being past the pad: no lock. */
  mk(&f, FLIGHT_WAIT_FOR_LAUNCH, true, 500.0f);
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(!rm.locked);
  /* ASCENT but no valid max yet: no lock. */
  mk(&f, FLIGHT_ASCENT, false, 500.0f);
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(!rm.locked);

  /* Real flight: ASCENT above the threshold, normal link -> LOCK. */
  mk(&f, FLIGHT_ASCENT, true, RUNMODE_LOCK_ALT_M);
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(rm.locked);
  CHECK(!runmode_accept(&rm, EK7_EV_SUT));
  CHECK(!runmode_accept(&rm, EK7_EV_SIT));
  CHECK(rm.rejected == 2u);
  CHECK(runmode_accept(&rm, EK7_EV_STOP)); /* STOP stays harmless */
  CHECK(rm.rejected == 2u);

  /* Latched: state going back to pad values does not unlock. */
  mk(&f, FLIGHT_WAIT_FOR_LAUNCH, false, 0.0f);
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(rm.locked);
  CHECK(!runmode_accept(&rm, EK7_EV_SUT));

  /* Lock also engages from SIT link (live sensors) and from FAULT/COMPLETE. */
  runmode_init(&rm);
  mk(&f, FLIGHT_DESCENT, true, 1400.0f);
  runmode_update(&rm, RM_LINK_SIT, &f);
  CHECK(rm.locked);
  runmode_init(&rm);
  mk(&f, FLIGHT_FAULT, true, 800.0f);
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(rm.locked);
  runmode_init(&rm);
  mk(&f, FLIGHT_FAULT, false, 0.0f); /* pad fault, never flew */
  runmode_update(&rm, RM_LINK_NORMAL, &f);
  CHECK(!rm.locked);

  /* Fail-closed / NULL safety. */
  CHECK(!runmode_accept(NULL, EK7_EV_SUT));
  CHECK(!runmode_accept(NULL, EK7_EV_SIT));
  CHECK(runmode_accept(NULL, EK7_EV_STOP));
  runmode_init(NULL);
  runmode_update(NULL, RM_LINK_NORMAL, &f);
  runmode_update(&rm, RM_LINK_NORMAL, NULL);

  /* Display mode. */
  runmode_init(&rm);
  CHECK(runmode_current(&rm, RM_LINK_NORMAL, false) == RUNMODE_REAL);
  CHECK(runmode_current(&rm, RM_LINK_NORMAL, true) == RUNMODE_TEST);
  CHECK(runmode_current(&rm, RM_LINK_SIT, false) == RUNMODE_SIT);
  CHECK(runmode_current(&rm, RM_LINK_SUT, true) == RUNMODE_SUT);
  CHECK(runmode_str(RUNMODE_REAL)[0] == 'R');
  CHECK(runmode_str(RUNMODE_TEST)[0] == 'T');
  CHECK(runmode_str(RUNMODE_SIT)[1] == 'I');
  CHECK(runmode_str(RUNMODE_SUT)[1] == 'U');

  printf("HOST_RUNMODE_TESTS_PASSED n=%d\n", n_pass);
  return 0;
}
