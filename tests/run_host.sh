#!/bin/bash
# Builds and runs every host test suite (no board needed).
# Usage: tests/run_host.sh
set -e
cd "$(dirname "$0")/.."
OUT=/tmp/rocket_host_tests
mkdir -p "$OUT"
CFLAGS="-std=c11 -Wall -Wextra -O1 -IInc"

run() {
  name="$1"; shift
  gcc $CFLAGS "$@" -lm -o "$OUT/$name"
  echo "== $name"
  "$OUT/$name" | tail -2
}

run gps       tests/host/test_gps_host.c       Src/gps.c
run avionics  tests/host/test_avionics_host.c  Src/avionics.c
run flight    tests/host/test_flight_host.c    Src/flight.c Src/avionics.c
run ek7       tests/host/test_ek7_host.c       Src/ek7.c Src/flight.c Src/avionics.c
if [ -f tests/host/test_runmode_host.c ]; then
  run runmode tests/host/test_runmode_host.c   Src/runmode.c Src/flight.c Src/avionics.c
fi
echo "ALL HOST SUITES OK"
