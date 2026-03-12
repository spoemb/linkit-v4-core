# Linkit V4 Core

Wildlife satellite/LoRa tracker firmware for the nRF52840 platform. Next generation evolution of the [CLS-Argos-Linkit-CORE](https://github.com/arribada/CLS-Argos-Linkit-CORE) platform.

Hardware repository: [linkit-v4-hw](https://github.com/arribada/linkit-v4-hw)

## Supported Boards

| Board | Communication | Interface | Use Case | Documentation |
|-------|--------------|-----------|----------|---------------|
| **LinkIt V4 KIM** | Argos (CLS KIM2) | UART | Open ocean, remote areas | [docs/boards/LinkIt-V4-KIM.md](docs/boards/LinkIt-V4-KIM.md) |
| **LinkIt V4 SMD** | Argos (Arribada SMD) | SPI | Open ocean, remote areas | [docs/boards/LinkIt-V4-SMD.md](docs/boards/LinkIt-V4-SMD.md) |
| **LinkIt V4 LoRa** | LoRaWAN (RAK3172-SiP) | UART | Coastal, urban, farm | [docs/boards/LinkIt-V4-LoRa.md](docs/boards/LinkIt-V4-LoRa.md) |
| **RSPB** | Argos (SMD) | SPI | Bird tracking (TPL5111 duty cycle) | [docs/boards/RSPB.md](docs/boards/RSPB.md) |

## Platform

Recommended: **Ubuntu/Debian** or **WSL2** (Windows). A **Docker** image is also provided with all tools pre-installed:

```bash
docker build -t linkit-v4-build .
docker run --rm -v $(pwd):/workspace linkit-v4-build ./scripts/build_core.sh
```

Windows native is possible with adapted tools but not officially supported. Use WSL2 for the best experience.

## Quick Start

### 1. Setup Environment

The setup script installs all required tools (ARM GCC 10.3, nrfutil, nrfjprog, CMake, etc.):

```bash
./scripts/setup_environment.sh          # interactive
./scripts/setup_environment.sh --auto   # auto-install everything
```

This generates `build_config.sh` with detected tool paths and configures VSCode tasks.

### 2. Build

Use the build scripts (they source `build_config.sh` automatically):

| Script | Board | Output |
|--------|-------|--------|
| `./scripts/build_core.sh` | LinkIt V4 KIM | `build/LINKIT/` |
| `./scripts/build_linkitv4_smd.sh` | LinkIt V4 SMD | `build/LINKIT_SMD/` |
| `./scripts/build_linkitv4_lora.sh` | LinkIt V4 LoRa | `build/LINKIT_LORA/` |
| `./scripts/build_rspb.sh` | RSPB | `build/RSPB/` |

Or build manually with CMake:

```bash
source build_config.sh  # load toolchain paths
mkdir -p ports/nrf52840/build/LINKIT && cd ports/nrf52840/build/LINKIT
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
  -DBOARD=LINKIT -DENABLE_AXL_SENSOR=ON \
  -DCMAKE_BUILD_TYPE=Debug -DDEBUG_LEVEL=4 ../..
make -j$(nproc)
```

See [docs/2-Building.md](docs/2-Building.md) for all CMake options and per-board examples.

### 3. Flash (Debug, SoftDevice + App)

```bash
nrfjprog --recover
nrfjprog -f nrf52 --program ports/nrf52840/drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --sectorerase
nrfjprog -f nrf52 --program ports/nrf52840/build/<BUILD_DIR>/<TARGET>.hex --sectorerase
nrfjprog -f nrf52 --reset
```

### Build Documentation (Doxygen, optional)

```bash
cd docs && doxygen Doxyfile
# Open: docs/doxygen_output/html/index.html
```

Requires [Doxygen](https://www.doxygen.nl/) installed. The generated documentation includes all source code API reference and the wiki pages (boards, DTE commands, parameters, architecture).

### 4. Unit Tests

```bash
cd tests && mkdir -p build && cd build
cmake .. && make -j$(nproc)
cd .. && ln -sf data build/ && ./build/TrackerTests -v
```

## CMake Build Options

### Board Selection

| Option | Values | Default | Description |
|--------|--------|---------|-------------|
| `BOARD` | `LINKIT`, `RSPB` | `LINKIT` | Target board |

### Communication Module (mutually exclusive)

| Option | Description |
|--------|-------------|
| *(none)* | KIM2 module (CLS, UART, legacy Argos) |
| `ARGOS_SMD=ON` | SMD module (Arribada, SPI, A+ protocol) |
| `LORA_RAK3172=ON` | RAK3172-SiP (LoRaWAN 1.0.3) |

### Sensor Flags

| Flag | Default | Sensor |
|------|---------|--------|
| `ENABLE_AXL_SENSOR` | 1 | BMA400 accelerometer |
| `ENABLE_PRESSURE_SENSOR` | 0 | LPS28DFW pressure/temperature/altitude |
| `ENABLE_THERMISTOR_SENSOR` | 0 (1 for RSPB) | NTC thermistor |
| `ENABLE_PH_SENSOR` | 0 | OEM pH sensor |
| `ENABLE_SEA_TEMP_SENSOR` | 0 | RTD / TSYS01 sea temperature |
| `ENABLE_ALS_SENSOR` | 0 | LTR-303 ambient light |
| `ENABLE_CDT_SENSOR` | 0 | Conductivity-Depth-Temperature |
| `CAM_ENABLE` | 0 | RunCam camera trigger |

Note: `ENABLE_THERMISTOR_SENSOR` and `ENABLE_SEA_TEMP_SENSOR` are mutually exclusive (shared Argos TX slot).

### Other Options

| Option | Default | Description |
|--------|---------|-------------|
| `DEBUG_LEVEL` | 3 | Debug verbosity (0=off, 4=trace) |
| `CMAKE_BUILD_TYPE` | Debug | `Debug` or `Release` |
| `BATTERY_MONITOR_TYPE` | ANALOG (STC3117 for RSPB) | `ANALOG`, `FAKE`, `STC3117` |
| `GPS_FAKE_POSITION` | 0 | Simulate GPS fix at Saint-Paul, Reunion |

## Key Parameters (DTE)

Parameters are read/written via the DTE protocol (USB, BLE, or UART) using `PARMR`/`PARMW` commands with 5-character keys.

| Group | Prefix | Examples |
|-------|--------|----------|
| Argos TX | `ARP` | `ARP01` (mode), `ARP05` (repetition), `ARP16` (depth pile) |
| GNSS | `GNP` | `GNP01` (enable), `GNP05` (timeout), `GNP25` (trigger on surfaced) |
| Underwater | `UNP` | `UNP01` (enable), `UNP10` (detect source), `UNP20-23` (SWS analog) |
| Low Battery | `LBP` | `LBP01` (enable), `LBP02` (threshold), `LBP12` (critical) |
| Zone | `ZOP` | `ZOP01` (type), `ZOP04` (out-of-zone mode) |
| LoRa | `LRP` | `LRP01-06` (credentials), `LRP07-15` (radio config) |
| Power/Wakeup | `PWP` | `PWP01` (shutdown timer), `PWP03` (boot modulo) |
| Identity | `IDP`/`IDT` | `IDP12` (Argos ID), `IDP13` (SMD seckey) |

See [docs/4-DTE-Commands/Parameter-Reference.md](docs/4-DTE-Commands/Parameter-Reference.md) for the complete parameter reference.

## Documentation

| Document | Description |
|----------|-------------|
| [Quick Start](docs/1-Quick-Start.md) | First-time setup guide |
| [Building](docs/2-Building.md) | Detailed build instructions |
| [Programming](docs/3-Programming.md) | Flashing and debugging |
| [DTE Commands](docs/4-DTE-Commands/README.md) | DTE protocol and commands reference |
| [Parameter Reference](docs/4-DTE-Commands/Parameter-Reference.md) | All configurable parameters |
| [Architecture](docs/5-Architecture/README.md) | System architecture overview |
| [Board: LinkIt V4 KIM](docs/boards/LinkIt-V4-KIM.md) | KIM module board details |
| [Board: LinkIt V4 SMD](docs/boards/LinkIt-V4-SMD.md) | SMD module board details |
| [Board: LinkIt V4 LoRa](docs/boards/LinkIt-V4-LoRa.md) | LoRa RAK3172 board details |
| [Board: RSPB](docs/boards/RSPB.md) | RSPB board details |

Compatible with:
- [PyLinkit](https://github.com/arribada/CLS-Argos-Linkit-pylinkit) - Python configuration tool
- LinkIt App - Mobile app

## Hardware Compatibility

### Linkit V4
* LinkIt V4 with KIM module (CLS) - [HW: kim-only](https://github.com/arribada/linkit-v4-hw/tree/kim-only)
* LinkIt V4 with SMD module (Arribada) - [HW: smd-only](https://github.com/arribada/linkit-v4-hw/tree/smd-only)
* LinkIt V4 with LoRa RAK3172-SiP
* RSPB Tracker

### Legacy (use [CLS-Argos-Linkit-CORE](https://github.com/arribada/CLS-Argos-Linkit-CORE))
* Horizon Board (Artic R2)
* Linkit V3 CORE (Artic R2)

## How to Contribute

If you'd like to contribute, start by searching through the issues and pull requests to see whether someone else has raised a similar idea or question.

* Change as little as possible. Break up into multiple PRs if necessary.
* If you've added a new feature, document it with a simple example.
* Please submit your PR using the release_candidate branch.

## Owners

<p align="center">
  <img src="https://www.cls-telemetry.com/wp-content/uploads/2021/06/logo-cls.png" alt="CLS" width="50%">
</p>
<p align="center">
Collaboration with
</p>
<p align="center">
  <img src="https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg" alt="Arribada" width="50%">
</p>
