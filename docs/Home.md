# LinKit v4 Core - Firmware Documentation

LinKit v4 Core is a multi-sensor wildlife tracker firmware running on nRF52840 (Nordic Semiconductor) with Argos satellite communication. It is designed for tracking birds and marine animals with configurable sensors, underwater detection, geofencing zones, and low-battery management.

## Features

- **GNSS** - u-blox M8 GPS with AssistNow Online/Offline, configurable dynamic models, HDOP/HACC filtering
- **Argos Satellite TX/RX** - Pass prediction, legacy, duty cycle, and Doppler modes. Supports SMD satellite module (A+ protocol)
- **Pressure Sensor** - LPS28DFW barometric pressure with altitude computation and configurable full scale (1260/4060 hPa)
- **Accelerometer** - BMA400 3-axis with wakeup detection, activity monitoring, configurable measurement range
- **Thermistor** - NTC temperature sensor with wakeup threshold
- **Underwater Detection** - Saltwater switch (digital/analog with auto-calibration), pressure-based, GNSS-based
- **Geofencing Zones** - Circular zone with out-of-zone detection and alternate Argos configuration
- **Low Battery Management** - Configurable threshold with alternate GNSS/Argos parameters
- **Camera** - Optional camera trigger on surfaced/accelerometer wakeup events
- **Dive Mode** - Dive state machine with reed switch pause/resume
- **178 Configurable Parameters** - Read/write via DTE AT commands over UART/USB/BLE
- **Sensor Data Logging** - Per-sensor CSV-formatted flash logs with DUMPD retrieval
- **Sensor Data in TX** - Configurable sensor data aggregation (oneshot/mean/median) appended to Argos packets

## Supported Boards

| Board | Description | Key Differences |
|-------|-------------|-----------------|
| **GenTracker (LINKIT)** | General-purpose tracker v1.0 | Analog battery monitor, external I2C bus for optional sensors |
| **RSPB Tracker** | RSPB bird tracker v1.0 | STC3117 fuel gauge, TPL5111 external wakeup, thermistor, SMD satellite module support |

## Hardware

- **MCU**: nRF52840 (ARM Cortex-M4F, 1MB Flash, 256KB RAM)
- **SoftDevice**: S140 v7.2.0 (BLE 5.0)
- **Build System**: CMake + ARM GCC Toolchain
- **Test Framework**: CppUTest

## Wiki Structure

1. [Quick Start Guide](1-Quick-Start.md) - Get up and running
2. [Building the Project](2-Building.md) - CMake configuration and build options
3. [Programming the Module](3-Programming.md) - Flashing, debugging, DFU
4. [AT Commands Reference](4-AT-Commands/README.md) - DTE protocol and commands
5. [Firmware Architecture](5-Architecture/README.md) - Code structure and design patterns
6. [Development Guide](6-Development-Guide/README.md) - Adding parameters, services, sensors
