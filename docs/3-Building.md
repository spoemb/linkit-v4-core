# Environment Setup

Before building for the first time, run the setup script to install and configure all required tools:

```bash
./scripts/setup_environment.sh          # interactive
./scripts/setup_environment.sh --auto   # auto-install everything
```

This installs:
- **ARM GCC 10.3** (downloaded from ARM, not the apt version which has SDK compatibility issues)
- **Build tools** (make, cmake, ninja, crc32, xxd)
- **nrfutil** + nrf5sdk-tools plugin (for DFU package generation)
- **nRF Command Line Tools** (nrfjprog, mergehex, J-Link)

And generates `build_config.sh` in the project root with detected tool paths. All build scripts (`scripts/build_*.sh`) source this file automatically.

Alternatively, use the Docker image (all tools pre-installed):

```bash
docker build -t linkit-v4-build .
docker run --rm -v $(pwd):/workspace linkit-v4-build ./scripts/build_core.sh
```

# Build Scripts

For convenience, each board variant has a dedicated build script:

| Script | Board | Comm Module | Build Dir |
|--------|-------|------------|-----------|
| `scripts/build_core.sh` | LinkIt V4 | KIM | `build/LINKIT/` |
| `scripts/build_linkitv4_smd.sh` | LinkIt V4 | SMD | `build/LINKIT_SMD/` |
| `scripts/build_linkitv4_lora.sh` | LinkIt V4 | LoRa | `build/LINKIT_LORA/` |
| `scripts/build_rspb.sh` | RSPB | SMD | `build/RSPB/` |
| `scripts/build_unit_tests.sh` | - | - | `tests/build/` |

# Directory Structure

```
linkit-v4-core/
├── core/                    # Portable firmware code (no hardware dependencies)
│   ├── configuration/       # ConfigStore, calibration, filesystem
│   ├── hardware/            # Abstract sensor/timer interfaces
│   ├── logging/             # Logger, LogFormatter, messages
│   ├── protocol/            # DTE protocol, encoder/decoder, params
│   └── services/            # All service implementations
├── ports/
│   └── nrf52840/            # nRF52840 hardware implementation
│       ├── bsp/             # Board Support Packages
│       │   ├── linkitv4_v1.0/      # LinkIt V4 board definitions
│       │   └── rspbtracker_v1.0/  # RSPB board definitions
│       ├── core/hardware/   # Hardware drivers (I2C, SPI, GPIO, sensors)
│       └── CMakeLists.txt   # Main firmware build
├── tests/                   # Unit tests (CppUTest)
│   ├── src/                 # Test source files
│   └── CMakeLists.txt       # Test build
└── docs/                    # Configuration files (.cfg)
```

# CMake Configuration

The firmware is built from `ports/nrf52840/`:

```bash
cd ports/nrf52840
mkdir build && cd build
cmake [OPTIONS] ..
make -j$(nproc)
```

## Board Selection (`BOARD`)

| Value | Description | BSP Directory |
|-------|-------------|---------------|
| `LINKIT` (default) | LinkIt V4 v1.0 | `bsp/linkitv4_v1.0/` |
| `RSPB` | RSPB Tracker v1.0 | `bsp/rspbtracker_v1.0/` |

```bash
cmake -DBOARD=RSPB ..
```

## Sensor Enable Flags

These flags control conditional compilation via `#if ENABLE_*` guards throughout the codebase. When disabled, the corresponding ParamIDs, DTE keys, services, and drivers are excluded from the build.

| Flag | Default | Effect |
|------|---------|--------|
| `ENABLE_AXL_SENSOR` | 1 | BMA400 accelerometer (wakeup, activity) |
| `ENABLE_PRESSURE_SENSOR` | 0 | LPS28DFW pressure/temperature/altitude |
| `ENABLE_THERMISTOR_SENSOR` | 0 (LINKIT), 1 (RSPB) | NTC thermistor temperature |
| `ENABLE_PH_SENSOR` | 0 | OEM pH sensor |
| `ENABLE_SEA_TEMP_SENSOR` | 0 | RTD or TSYS01 sea temperature |
| `ENABLE_ALS_SENSOR` | 0 | LTR-303 ambient light sensor |
| `ENABLE_CDT_SENSOR` | 0 | Conductivity-Depth-Temperature |
| `ENABLE_CAM_SENSOR` | 0 | Camera trigger |
| `ENABLE_MORTALITY_SENSOR` | 0 (LINKIT), 1 (RSPB) | Bird mortality detection service |

```bash
cmake -DENABLE_PRESSURE_SENSOR=1 -DENABLE_SEA_TEMP_SENSOR=1 ..
```

## Other Build Options

| Variable | Default | Description |
|----------|---------|-------------|
| `ARGOS_SMD` | OFF | Enable SMD satellite module (A+ protocol) |
| `BATTERY_MONITOR_TYPE` | ANALOG (LINKIT), STC3117 (RSPB) | Battery monitor: ANALOG, FAKE, STC3117 |
| `GPS_FAKE_POSITION` | 0 | Simulate GPS fix at Saint-Paul, Reunion |
| `EXT_GPIO5_DEVICE` | CAM | External GPIO5 device: CAM or BUZZER |
| `DEBUG_LEVEL` | 3 | Debug verbosity (0=off, 3=trace) |
| `CMAKE_BUILD_TYPE` | Debug | Debug or Release |

## RSPB-Specific Options

When `BOARD=RSPB`, these are automatically set:

| Variable | Value | Description |
|----------|-------|-------------|
| `EXTERNAL_WAKEUP` | 1 | TPL5111 periodic wakeup timer |
| `WAKEUP_PERIOD` | 6300 | Wakeup interval in seconds (1h45) |
| `ENABLE_THERMISTOR_SENSOR` | 1 | NTC thermistor enabled |
| `ENABLE_MORTALITY_SENSOR` | 1 | Bird mortality detection |
| `BATTERY_MONITOR_TYPE` | STC3117 | I2C fuel gauge |

## Satellite / Communication Module

| Option | Description |
|--------|-------------|
| `ARGOS_SMD=ON` | Use Arribada SMD satellite module (SPI by default, A+ protocol) |
| `ARGOS_SMD=ON` + `SMD_UART=ON` | Use Arribada SMD satellite module (AT/UART commands) |
| `LORA_RAK3172=ON` | Use RAK3172-SiP LoRa module (UART, LoRaWAN 1.0.3) |
| *(neither)* | Default: KIM2 module (CLS, UART, legacy Argos) |

Only one communication module can be active at a time. `ARGOS_SMD` and `LORA_RAK3172` are mutually exclusive.

### SMD Transport: SPI vs AT/UART

The SMD module supports two transport layers, selected at **compile time**:

| | SPI (default) | AT/UART (`SMD_UART=ON`) |
|---|---|---|
| **Protocol** | Binary Protocol A+ (64-byte full-duplex frames) | Text-based AT commands (`AT+CMD=<params>\r\n`) |
| **Interface** | SPI Master (125kHz, Mode 0) | Async UART |
| **CMake flag** | Default (no extra flag needed) | `-DSMD_UART=ON` |
| **Compile define** | `SMD_SPI=1` | `SMD_UART=1` |

Both transports implement the same abstract `SmdSatCmd` interface (`ports/nrf52840/core/hardware/smd_sat/smd_sat_cmd.hpp`), so the rest of the firmware is transport-agnostic.

```bash
 SPI mode (default)
cmake -DBOARD=LINKIT -DARGOS_SMD=ON ...

 AT/UART mode
cmake -DBOARD=LINKIT -DARGOS_SMD=ON -DSMD_UART=ON ...
```

# Build Examples per Board

## LinkIt V4 with KIM (default)

```bash
mkdir -p ports/nrf52840/build/LINKIT && cd ports/nrf52840/build/LINKIT
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
  -DBOARD=LINKIT -DENABLE_AXL_SENSOR=ON \
  -DCMAKE_BUILD_TYPE=Debug -DDEBUG_LEVEL=4 ../..
make -j$(nproc)
```

## LinkIt V4 with SMD

```bash
mkdir -p ports/nrf52840/build/LINKIT_SMD && cd ports/nrf52840/build/LINKIT_SMD
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
  -DBOARD=LINKIT -DARGOS_SMD=ON -DENABLE_AXL_SENSOR=ON \
  -DCMAKE_BUILD_TYPE=Debug -DDEBUG_LEVEL=4 ../..
make -j$(nproc)
```

## LinkIt V4 with LoRa

```bash
mkdir -p ports/nrf52840/build/LINKIT_LORA && cd ports/nrf52840/build/LINKIT_LORA
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
  -DBOARD=LINKIT -DLORA_RAK3172=ON -DENABLE_AXL_SENSOR=ON \
  -DCMAKE_BUILD_TYPE=Debug -DDEBUG_LEVEL=4 ../..
make -j$(nproc)
```

## RSPB with SMD

```bash
mkdir -p ports/nrf52840/build/RSPB && cd ports/nrf52840/build/RSPB
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
  -DBOARD=RSPB -DARGOS_SMD=ON \
  -DENABLE_PRESSURE_SENSOR=ON -DENABLE_AXL_SENSOR=ON \
  -DCMAKE_BUILD_TYPE=Debug -DDEBUG_LEVEL=4 ../..
make -j$(nproc)
```

## Additional Examples

```bash
 LinkIt V4 with pressure sensor (KIM)
cmake ... -DENABLE_PRESSURE_SENSOR=1 ..

 LinkIt V4 with all sensors
cmake ... \
  -DENABLE_PRESSURE_SENSOR=1 \
  -DENABLE_SEA_TEMP_SENSOR=1 \
  -DENABLE_PH_SENSOR=1 \
  -DENABLE_ALS_SENSOR=1 \
  -DENABLE_CDT_SENSOR=1 ..

 Release build for production
cmake ... -DCMAKE_BUILD_TYPE=Release ..
```

For detailed board-specific documentation see [Board Variants](https://github.com/arribada/linkit-v4-core/wiki/5-%E2%80%90-Boards).

# Building Unit Tests

Tests are built separately with CppUTest:

```bash
cd tests
mkdir build && cd build
cmake ..
make -j$(nproc)

 Run all tests
ctest --output-on-failure

 Run specific test
./TrackerTests -g PressureSensorTest
```

The test build defines all `ENABLE_*` flags to 1 so all code paths are compiled and tested regardless of the target board configuration.

# Impact of ENABLE_* Flags on Code

The `ENABLE_*` flags have a cascading effect through the codebase:

1. **`core/protocol/base_types.hpp`** - ParamID enum members are `#if` guarded
2. **`core/protocol/dte_params.cpp`** - `param_map[]` entries use the flag as `is_implemented`
3. **`core/configuration/config_store.hpp`** - Default values are always present (slots reserved)
4. **`core/services/`** - Service classes are conditionally included
5. **`ports/nrf52840/`** - Hardware drivers are conditionally compiled

Slots in the ParamID enum and `config_store.hpp` defaults array are **always reserved** even when a sensor is disabled. This ensures stable parameter indexing across builds with different sensor configurations.
