# BOZOK Rocket Avionics

STM32F446RE flight avionics firmware, Arduino-based GPS/LoRa utilities, host-side tests, and the BOZOK ground station.

## Project layout

- `Src/`, `Inc/`, `Drivers/`, `cmake/`: STM32 firmware and HAL/CMSIS integration.
- `uno_lora_rx/`, `uno_lora_getconfig/`, `uno_gps_test/`, `uno_gps_config/`: Arduino sketches for radio/GPS setup and receiver testing.
- `groundstation/backend/`: FastAPI serial/telemetry backend.
- `groundstation/frontend/`: React + TypeScript UI.
- `groundstation/remotion/`: Remotion intro animation source.
- `tests/`: host-side firmware tests and SIT/SUT utilities.
- `base.md`, `durum.md`, `GPS_LoRA_OTURUM_NOTU.md`, `SIT_SUT_REQUIREMENTS.md`: architecture and verification notes.
- `roketpdf/`, `E220-900T22D_UserManual_EN_v1.2.pdf`: reference documents.

## Host tests

```bash
bash tests/run_host.sh
```

## Ground station

See [`groundstation/README.md`](groundstation/README.md) for local startup instructions. Keep the backend as the sole owner of the serial port; do not open the same device independently with `cat`, `stty`, or another serial monitor while the UI/backend is connected.

## Repository hygiene

Generated build outputs, package installations, virtual environments, captured telemetry/logs, local recovery archives, photo collections, credentials, and downloaded simulator installers are intentionally excluded. Dependency lockfiles and the source files needed to build the project are retained.
