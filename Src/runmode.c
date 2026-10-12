/* Run-mode separation (see runmode.h for the rule set). */
#include "runmode.h"

void runmode_init(runmode_state_t *rm)
{
  if (rm == NULL)
  {
    return;
  }
  rm->locked = false;
  rm->rejected = 0u;
}

void runmode_update(runmode_state_t *rm, rm_link_t link, const flight_t *f)
{
  if (rm == NULL || f == NULL || rm->locked)
  {
    return; /* latched: only a reset clears it */
  }
  if (link == RM_LINK_SUT)
  {
    return; /* synthetic flight: says nothing about the real rocket */
  }
  if (f->state >= FLIGHT_ASCENT && f->has_max &&
      f->max_alt_m >= RUNMODE_LOCK_ALT_M)
  {
    rm->locked = true;
  }
}

bool runmode_accept(runmode_state_t *rm, ek7_cmd_t cmd)
{
  if (cmd == EK7_EV_STOP)
  {
    return true;
  }
  if (cmd != EK7_EV_SIT && cmd != EK7_EV_SUT)
  {
    return false;
  }
  if (rm == NULL)
  {
    return false; /* fail closed */
  }
  if (rm->locked)
  {
    rm->rejected++;
    return false;
  }
  return true;
}

runmode_t runmode_current(const runmode_state_t *rm, rm_link_t link,
                          bool test_build)
{
  (void)rm;
  if (link == RM_LINK_SUT)
  {
    return RUNMODE_SUT;
  }
  if (link == RM_LINK_SIT)
  {
    return RUNMODE_SIT;
  }
  return test_build ? RUNMODE_TEST : RUNMODE_REAL;
}

const char *runmode_str(runmode_t m)
{
  switch (m)
  {
    case RUNMODE_REAL:
      return "REAL";
    case RUNMODE_TEST:
      return "TEST";
    case RUNMODE_SIT:
      return "SIT";
    case RUNMODE_SUT:
      return "SUT";
    default:
      return "?";
  }
}
