# Board Porting

Guide for porting the firmware to a new hardware platform.

## Architecture

The firmware separates portable code (`core/`) from hardware-specific code (`ports/`):

```
core/                           # Portable: no hardware dependencies
├── hardware/sensor.hpp         # Abstract Sensor interface
├── hardware/calibration.hpp    # Abstract Calibratable interface
├── services/                   # All service logic
├── protocol/                   # DTE protocol
├── configuration/              # Config store interface
└── logging/                    # Logger interface

ports/nrf52840/                 # Hardware-specific
├── bsp/                        # Board Support Packages
│   ├── gentracker_v1.0/bsp.hpp
│   └── rspbtracker_v1.0/bsp.hpp
├── core/hardware/              # Sensor/peripheral drivers
│   ├── bma400/                 # Accelerometer
│   ├── lps28dfw/               # Pressure sensor
│   ├── m8/                     # GPS module
│   ├── pressure_sensor/        # Pressure sensor wrapper
│   └── ...
└── CMakeLists.txt              # Build configuration
```

## Step 1: Create BSP

Create a new BSP directory: `ports/<mcu>/bsp/<board_name>/bsp.hpp`

Define all hardware mappings:

```cpp
#pragma once

// Peripherals
#define RTC_DATE_TIME    RTC_1
#define RTC_TIMER        RTC_2
#define SPI_SATELLITE    SPI_2
#define UART_GPS         UART_0

// I2C buses
#define ONBOARD_I2C_BUS  I2C_0
#define EXTERNAL_I2C_BUS I2C_1  // Optional

// I2C addresses
#define BMA400_I2C_ADDR  0x14
#define LPS28DFW_I2C_ADDR 0x5C

// GPIO pin assignments
#define GPS_POWER        GPIO_GPS_PWR_EN
#define GPS_RST          GPIO_GPS_RST
#define SAT_PWR_EN       GPIO_SAT_EN
#define SWS_ENABLE_PIN   GPIO_SWS_SEND
#define SWS_SAMPLE_PIN   GPIO_SWS

// Battery ADC
#define BATTERY_ADC      ADC_CHANNEL_0
#define ADC_GAIN         (1.0/5.0)
#define V_DIV_GAIN       1.443

// Feature flags
#define POWER_ON_RESET_REQUIRES_REED_SWITCH  1
#define NO_ARGOS_PA_GAIN_CTRL               1
```

## Step 2: Implement Hardware Abstractions

Implement these abstract interfaces from `core/`:

### Required

| Interface | Location | Methods |
|-----------|----------|---------|
| Sensor | `core/hardware/sensor.hpp` | `read(port)` |
| Logger | `core/logging/logger.hpp` | `create()`, `write()`, `read()`, `num_entries()`, `truncate()` |
| ConfigurationStore | `core/configuration/config_store.hpp` | `init()`, `factory_reset()`, `serialize_config()`, etc. |
| Timer | (platform-specific) | Scheduling and RTC |
| PMU | (platform-specific) | `delay_ms()`, `hardware_version()`, `device_identifier()` |

### I2C / SPI

Implement platform-specific communication:

```cpp
class NrfI2C {
public:
    static void write(unsigned int bus, unsigned char addr,
                      const uint8_t* data, uint16_t len, bool no_stop);
    static void read(unsigned int bus, unsigned char addr,
                     uint8_t* data, uint16_t len);
};
```

### GPIO

```cpp
class GPIOPins {
public:
    static void set(unsigned int pin);
    static void clear(unsigned int pin);
    static bool read(unsigned int pin);
    static void acquire_sensors_pwr();  // VSENSORS ON
    static void release_sensors_pwr();  // VSENSORS OFF
};
```

## Step 3: Create CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.15)

# Board selection
set(BOARD "MY_BOARD" CACHE STRING "Target board")

# Model selection
set(MODEL "SB" CACHE STRING "Device model")

# Sensor flags
option(ENABLE_PRESSURE_SENSOR "Enable pressure sensor" OFF)
option(ENABLE_AXL_SENSOR "Enable accelerometer" ON)
# ...

# Include core sources
file(GLOB_RECURSE CORE_SOURCES "${CMAKE_SOURCE_DIR}/../../core/**/*.cpp")

# Include port-specific sources
file(GLOB_RECURSE PORT_SOURCES "${CMAKE_SOURCE_DIR}/core/hardware/**/*.cpp")

# Compile definitions
add_definitions(
    -DBOARD_${BOARD}=1
    -DMODEL_${MODEL}=1
    -DENABLE_PRESSURE_SENSOR=${ENABLE_PRESSURE_SENSOR}
    -DENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR}
)
```

## Step 4: Implement SensorsPowerGuard

Critical RAII pattern for managing sensor power:

```cpp
class SensorsPowerGuard {
public:
    SensorsPowerGuard() { GPIOPins::acquire_sensors_pwr(); }
    ~SensorsPowerGuard() { GPIOPins::release_sensors_pwr(); }
};
```

All sensor drivers must use this guard and re-apply their configuration after power-on, since sensor registers are volatile.

## Existing Board References

Study these for reference:

| Board | BSP | Key Differences |
|-------|-----|-----------------|
| GenTracker v1.0 | `bsp/gentracker_v1.0/bsp.hpp` | Analog battery, 2 I2C buses, reed switch boot |
| RSPB v1.0 | `bsp/rspbtracker_v1.0/bsp.hpp` | STC3117 fuel gauge, TPL5111, SMD satellite, thermistor ADC |
