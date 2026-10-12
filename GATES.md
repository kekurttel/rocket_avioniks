# Acceptance Gates — BOZOK Ground Station / LoRa / Altitude

This ledger is the acceptance contract for this change. Commands are local, deterministic checks only; none opens a serial device, resets a board, uploads firmware, or flashes the STM32. E220/STM32/Uno production hardware is not to be touched by this software task.

## Gate 0 — scope and non-destructive invariants

- `CHECK: git diff --check` — no whitespace errors in the change.
- `CHECK: git status --short --branch` — only intended source, tests, docs, and this ledger may be changed before commit.
- `CHECK: secret scan of staged paths` — no high-confidence API keys, personal access tokens, private keys, or embedded URL credentials.
- `CHECK: ` ground-station/backend remains the only serial-port owner; no test command uses `cat`, `stty`, `screen`, `picocom`, or `arduino-cli upload`.
- Scheduler note (justified exception to "no sampling-schedule change"): NORMAL idle 50 ms → 10 ms and TUI gate 50 ms → 100 ms are REQUIRED for a 10 Hz average (a transmitting iteration costs ~43 ms UART + ~30 ms amortised TUI + delay; 50 ms idle caps the mean at ~9.2 Hz). Flight safety is preserved: `flight_step` uses measured `dt_flight`, launch/free-fall streaks span seconds, and the DROP baseline EMA (tens-of-seconds time constant vs seconds-scale descents) keeps its tuned alpha — faster polling only shortens detection latency. Host flight/avionics suites re-verify the gates unchanged.

## Gate 1 — flight/control regression

- `CHECK: bash tests/run_host.sh` — expected token `ALL HOST SUITES OK`.
- Intent: altitude physics and deployment/flight state transitions stay unchanged; test-only helpers remain test-only.

## Gate 2 — LoRa wire contract and 10 Hz schedule

- `CHECK: gcc -std=c11 -Wall -Wextra -O1 -IInc tests/host/test_lora_host.c Src/lora.c Src/flight.c Src/avionics.c Src/runmode.c -lm -o /tmp/bozok-lora-host && /tmp/bozok-lora-host` — expected token `HOST_LORA_TESTS_PASSED`.
- Verify 100 ms cadence in pad and flight, sequence monotonicity/wrap, CRC, all previous wire versions, signed/decimal altitude boundaries, and invalid-value sentinels.
- `CHECK: Datasheet configuration assertions` — 9600 8N1 module UART; same 4.8 kbps RF rate on both radios; REG1 remains 0x00 / 22 dBm; channel remains 0x17; REG3 remains transparent (0x00); encryption key remains 0x0000.

## Gate 3 — receiver and telemetry backend

- `CHECK: arduino-cli compile --fqbn arduino:avr:uno --build-path /tmp/bozok-uno-build uno_lora_rx` — expected successful compile; no upload.
- `CHECK: PYTHONPATH=groundstation/backend python3 tests/host/test_groundstation_protocol.py` — expected token `GROUNDSTATION_PROTOCOL_TESTS_OK`.
- Backend tests must cover negative and fractional altitude strings, legacy V1/V2 packets, V3 decimal-altitude packets, invalid sentinel handling, and sequence/gap accounting.

## Gate 4 — ground-station integration

- `CHECK: npm run build --prefix groundstation/frontend` — successful TypeScript + Vite build.
- `CHECK: cmake --build --preset Debug -j2` — successful STM32 firmware build, with no programming step.
- `CHECK: all wire decoders agree` — Uno serial output, FastAPI state, UI readouts, graphs, and session recordings expose the same signed, launch-relative altitude precision and correct units.

## Gate 5 — release review

- `CHECK: git diff --check` and full relevant test suite after integration; reread changed files and this ledger.
- `CHECK: hardware soak` — live 30 s run must average approximately 10 Hz, show increasing contiguous `[LINK] seq`, CRC-valid packets, no unexpected gaps, and `stale=False`; STM32 TUI remains responsive. This hardware gate is **ABANDONED FOR THIS SOFTWARE SESSION** because the live serial owner is explicitly protected and this task must not open/reopen serial ports, reset the Uno, or flash either board. Software-only build success is not proof of the live 10 Hz link.
- Operator steps (user runs hardware): flash BOTH boards from this tree (STM32 Debug ELF + Uno sketch) — V3 frames require the V3 parser, and mixed versions silently reject (Uno `bozuk` climbs, no `[LINK]`). Then run `durum.md` §10 runbook: boot proof lines (`RD=00 00 63 ...`, `FIXED`/`OK`), 30 s acceptance (~10 Hz, contiguous seq, `stale=False`), desense/power/swap ladder on failure. `RAW=0` with no port errors means wiring/power/mode, never a software retry loop.

## Safety constraints

No deployment/flight-state logic, barometric formula, baseline calibration, sensor sampling schedule, or flight thresholds may be changed. Never run any acceptance command that opens `/dev/ttyUSB0` or `/dev/ttyACM0`. Do not upload/flash firmware in this task. RF power must not be represented as software-increasable beyond the module's datasheet maximum.
