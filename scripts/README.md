# scripts/

Build, setup and utility scripts.

## Setup

| Script | Purpose |
|--------|---------|
| `setup_environment.sh` | Detect / install ARM GCC, nrfutil, nrfjprog, CMake. Generates `build_config.sh` |

## Build

| Script | Output | Comm |
|--------|--------|------|
| `build_linkitv4_kim.sh`  | `ports/nrf52840/build/LINKIT/`      | KIM2 (default) |
| `build_linkitv4_smd.sh`  | `ports/nrf52840/build/LINKIT_SMD/`  | SMD (`-DARGOS_SMD=ON`) |
| `build_linkitv4_lora.sh` | `ports/nrf52840/build/LINKIT_LORA/` | LoRa (`-DLORA_RAK3172=ON`) |
| `build_rspb.sh`          | `ports/nrf52840/build/RSPB/`        | SMD on RSPB board |
| `build_with_bootloader.sh <target> [--clean] [--recover]` | Merged hex (app + bootloader + SoftDevice) | any |
| `build_unit_tests.sh`    | `tests/build/TrackerTests`          | host |

All build scripts source `build_config.sh` (created by `setup_environment.sh`) and use `git describe --dirty` to embed the firmware version.

## Other

| Script | Purpose |
|--------|---------|
| `run_tests.sh` | Build and run the host test suite |
| `log_stack_dump.py` | Decode a hex stack-dump captured from the device using `addr2line` |

## Adding a new build target

Copy an existing `build_*.sh`, adjust the build dir / CMake flags, and document it in [`../README.md`](../README.md).
