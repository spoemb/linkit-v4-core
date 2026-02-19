# 1 - Quick Start Guide

## Prerequisites

- **ARM GCC Toolchain** (gcc-arm-none-eabi 10.3 or later)
- **nRF5 SDK 17.1.0** (Nordic Semiconductor)
- **CMake 3.15+**
- **nrfjprog** (Nordic command-line tools) or J-Link Commander
- **Python 3** (for build scripts)

## Clone and Build

```bash
git clone <repository-url> linkit-v4-core
cd linkit-v4-core

# Build for GenTracker board, UW model with pressure sensor
cd ports/nrf52840
mkdir build && cd build
cmake -DMODEL=UW -DENABLE_PRESSURE_SENSOR=1 ..
make -j$(nproc)
```

The output firmware is generated as a `.hex` file in the build directory.

## Flash the Firmware

```bash
# Flash SoftDevice first (only needed once)
nrfjprog --program s140_nrf52_7.2.0_softdevice.hex --chiperase --verify
nrfjprog --reset

# Flash application
nrfjprog --program linkit_v4.hex --sectorerase --verify
nrfjprog --reset
```

## First Connection

Connect to the device via UART (115200 baud, 8N1) or USB CDC:

```
# List all parameters
$PARML#000;

# Read all parameters
$PARMR#000;

# Read specific parameters
$PARMR#00A;GNP01,ARP01

# Write a parameter
$PARMW#010;GNP01=1,ARP01=2
```

See [AT Commands Reference](4-AT-Commands/README.md) for the full command set.

## Build Tests

```bash
cd tests
mkdir build && cd build
cmake ..
make -j$(nproc)
ctest --output-on-failure
```

## Next Steps

- [Building the Project](2-Building.md) - Full build configuration reference
- [AT Commands Reference](4-AT-Commands/README.md) - Complete command documentation
- [Architecture Overview](5-Architecture/README.md) - Understand the firmware design
