# LinKit v4 Core - Firmware Documentation

LinKit v4 Core is a multi-sensor wildlife tracker firmware running on nRF52840 (Nordic Semiconductor). It supports Argos satellite communication (KIM/SMD modules) and LoRaWAN (RAK3172-SiP) for tracking birds and marine animals with configurable sensors, underwater detection, geofencing zones, and low-battery management.

## Features

- **GNSS** - u-blox M10Q GPS with AssistNow Online/Offline, configurable dynamic models, HDOP/HACC filtering
- **Argos Satellite TX/RX** - Pass prediction, legacy, duty cycle, and Doppler modes. Supports KIM2 (CLS) and SMD (Arribada, A+ protocol) modules
- **LoRaWAN** - RAK3172-SiP module (Semtech SX1262), OTAA/ABP, EU868 (+ other bands), Class A/B/C
- **Pressure Sensor** - LPS28DFW barometric pressure with altitude computation and configurable full scale (1260/4060 hPa)
- **Accelerometer** - BMA400 3-axis with wakeup detection, activity monitoring, configurable measurement range
- **Thermistor** - NTC temperature sensor with wakeup threshold
- **Underwater Detection** - Saltwater switch (analog with auto-calibration, 5-level detection), pressure-based, GNSS-based
- **Geofencing Zones** - Circular zone with out-of-zone detection and alternate Argos configuration
- **Low Battery Management** - Configurable threshold with alternate GNSS/Argos parameters and critical shutdown
- **Camera** - Optional camera trigger on surfaced/accelerometer wakeup events
- **Dive Mode** - Dive state machine with reed switch pause/resume
- **Bird Mortality Detection** (RSPB) - Automatic mortality confidence from activity, temperature, and GPS stationarity
- **206 Configurable Parameters** - Read/write via DTE commands over USB/BLE/UART
- **Sensor Data Logging** - Per-sensor CSV-formatted flash logs with DUMPD retrieval
- **Sensor Data in TX** - Configurable sensor data aggregation (oneshot/mean/median) appended to Argos/LoRa packets

## Supported Boards

| Board | Communication | Description |
|-------|--------------|-------------|
| **LinkIt V4 KIM** | Argos (KIM2, UART) | Default Argos tracker with CLS module |
| **LinkIt V4 SMD** | Argos (SMD, SPI) | Arribada SMD module with A+ protocol and security |
| **LinkIt V4 LoRa** | LoRaWAN (RAK3172) | Terrestrial LPWAN for gateway-covered areas |
| **RSPB Tracker** | Argos (SMD, SPI) | Bird tracker with TPL5111 duty cycle wakeup |

See [Board Variants](https://github.com/arribada/linkit-v4-core/wiki/5-%E2%80%90-Boards) for detailed hardware specs, parameters, and configuration per board.

## Hardware

- **MCU**: nRF52840 (ARM Cortex-M4F, 1MB Flash, 256KB RAM)
- **SoftDevice**: S140 v7.2.0 (BLE 5.0)
- **GPS**: u-blox M10Q (UART)
- **Build System**: CMake + ARM GCC Toolchain
- **Test Framework**: CppUTest

## Wiki Structure

1. [User Guide](https://github.com/arribada/linkit-v4-core/wiki/1-%E2%80%90-User-Guide) - Activate, configure, and deploy your tracker
2. [Developer Quick Start](https://github.com/arribada/linkit-v4-core/wiki/2-%E2%80%90-Developer-quick-start) - Build firmware from source
3. [Building the Project](https://github.com/arribada/linkit-v4-core/wiki/3-%E2%80%90-Building) - CMake configuration and build options
4. [Programming the Module](https://github.com/arribada/linkit-v4-core/wiki/4-%E2%80%90-Programming) - Flashing, debugging, DFU
5. [Board Variants](https://github.com/arribada/linkit-v4-core/wiki/5-%E2%80%90-Boards) - Hardware specs and configuration per board
6. [DTE Commands Reference](https://github.com/arribada/linkit-v4-core/wiki/6-%E2%80%90-DTE-commands) - DTE protocol and commands
7. [Firmware Architecture](https://github.com/arribada/linkit-v4-core/wiki/7-%E2%80%90-Architecture) - Code structure and design patterns
8. [Development Guide](https://github.com/arribada/linkit-v4-core/wiki/8-%E2%80%90-Development-guide) - Adding parameters, services, sensors
9. [Parameters Definition](https://github.com/arribada/linkit-v4-core/wiki/9-%E2%80%90-Parameters-Definition) - All configurable parameters and their purpose
10. [Satellite Message Format](https://github.com/arribada/linkit-v4-core/wiki/10-%E2%80%90-Satellite-Message-Format) - Argos & LoRa packet encoding, bit layouts, decoding
11. [LinkIt UW Behavior](https://github.com/arribada/linkit-v4-core/wiki/11-%E2%80%90-LinkIt-UW-Behavior) - Underwater mode, SWS algorithm, Argos TX modes
12. [RSPB Bird Tracker](https://github.com/arribada/linkit-v4-core/wiki/12-%E2%80%90-RSPB-Mortality-Tracker) - TPL5111 duty cycling, battery modes, mortality detection
13. [SWS Algorithm](https://github.com/arribada/linkit-v4-core/wiki/13-%E2%80%90-SWS-Algorithm) - SWS analog detection algorithm deep-dive
