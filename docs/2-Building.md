# 2 - Building the Project

## Directory Structure

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
│       │   ├── gentracker_v1.0/   # GenTracker board definitions
│       │   └── rspbtracker_v1.0/  # RSPB board definitions
│       ├── core/hardware/   # Hardware drivers (I2C, SPI, GPIO, sensors)
│       └── CMakeLists.txt   # Main firmware build
├── tests/                   # Unit tests (CppUTest)
│   ├── src/                 # Test source files
│   └── CMakeLists.txt       # Test build
└── docs/                    # Configuration files (.cfg)
```

## CMake Configuration

The firmware is built from `ports/nrf52840/`:

```bash
cd ports/nrf52840
mkdir build && cd build
cmake [OPTIONS] ..
make -j$(nproc)
```

### Board Selection (`BOARD`)

| Value | Description | BSP Directory |
|-------|-------------|---------------|
| `LINKIT` (default) | GenTracker v1.0 | `bsp/gentracker_v1.0/` |
| `RSPB` | RSPB Tracker v1.0 | `bsp/rspbtracker_v1.0/` |

```bash
cmake -DBOARD=RSPB ..
```

### Device Model (`MODEL`)

| Value | Description | Key Defaults |
|-------|-------------|-------------|
| `SB` | Surface Bird | Pass prediction mode, 6 retries, 1h GNSS interval |
| `UW` | Underwater | Legacy mode, underwater detection enabled, 240s acq timeout |
| `CORE` | Core/Generic | Legacy mode, wireless charging enabled |

The MODEL sets compile-time flags `MODEL_SB`, `MODEL_UW`, `MODEL_CORE` (one = 1, others = 0) and affects many default parameter values in `config_store.hpp`.

```bash
cmake -DMODEL=UW ..
```

### Sensor Enable Flags

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

```bash
cmake -DMODEL=UW -DENABLE_PRESSURE_SENSOR=1 -DENABLE_SEA_TEMP_SENSOR=1 ..
```

### Other Build Options

| Variable | Default | Description |
|----------|---------|-------------|
| `ARGOS_SMD` | OFF | Enable SMD satellite module (A+ protocol) |
| `BATTERY_MONITOR_TYPE` | ANALOG (LINKIT), STC3117 (RSPB) | Battery monitor: ANALOG, FAKE, STC3117 |
| `GPS_FAKE_POSITION` | 0 | Simulate GPS fix at Saint-Paul, Reunion |
| `EXT_GPIO5_DEVICE` | CAM | External GPIO5 device: CAM or BUZZER |
| `DEBUG_LEVEL` | 3 | Debug verbosity (0=off, 3=trace) |
| `CMAKE_BUILD_TYPE` | Debug | Debug or Release |

### RSPB-Specific Options

When `BOARD=RSPB`, these are automatically set:

| Variable | Value | Description |
|----------|-------|-------------|
| `EXTERNAL_WAKEUP` | 1 | TPL5111 periodic wakeup timer |
| `WAKEUP_PERIOD` | 6300 | Wakeup interval in seconds (1h45) |
| `ENABLE_THERMISTOR_SENSOR` | 1 | NTC thermistor enabled |
| `BATTERY_MONITOR_TYPE` | STC3117 | I2C fuel gauge |

## Build Examples

```bash
# GenTracker UW with pressure sensor
cmake -DMODEL=UW -DENABLE_PRESSURE_SENSOR=1 ..

# RSPB SB with SMD satellite module
cmake -DBOARD=RSPB -DMODEL=SB -DARGOS_SMD=ON ..

# GenTracker with all sensors
cmake -DMODEL=UW \
  -DENABLE_PRESSURE_SENSOR=1 \
  -DENABLE_SEA_TEMP_SENSOR=1 \
  -DENABLE_PH_SENSOR=1 \
  -DENABLE_ALS_SENSOR=1 \
  -DENABLE_CDT_SENSOR=1 ..

# Release build for production
cmake -DMODEL=SB -DCMAKE_BUILD_TYPE=Release ..
```

## Building Unit Tests

Tests are built separately with CppUTest:

```bash
cd tests
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run all tests
ctest --output-on-failure

# Run specific test
./CLSGenTrackerTests -g PressureSensorTest
```

The test build defines all `ENABLE_*` flags to 1 so all code paths are compiled and tested regardless of the target board configuration.

## Impact of ENABLE_* Flags on Code

The `ENABLE_*` flags have a cascading effect through the codebase:

1. **`core/protocol/base_types.hpp`** - ParamID enum members are `#if` guarded
2. **`core/protocol/dte_params.cpp`** - `param_map[]` entries use the flag as `is_implemented`
3. **`core/configuration/config_store.hpp`** - Default values are always present (slots reserved)
4. **`core/services/`** - Service classes are conditionally included
5. **`ports/nrf52840/`** - Hardware drivers are conditionally compiled

Slots in the ParamID enum and `config_store.hpp` defaults array are **always reserved** even when a sensor is disabled. This ensures stable parameter indexing across builds with different sensor configurations.
