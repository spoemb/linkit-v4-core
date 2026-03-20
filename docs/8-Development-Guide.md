This guide covers common development tasks for extending the LinKit v4 firmware.

# Code Conventions

- C++20 standard
- Tabs for indentation
- `#pragma once` for header guards
- Enum classes for type safety
- `DEBUG_TRACE()`, `DEBUG_INFO()`, `DEBUG_WARN()`, `DEBUG_ERROR()` for debug output
- Packed structs (`__attribute__((packed))`) for log entries and wire formats
- `ErrorCode` exceptions for error handling (not std::exception)
- Static registries (SensorManager, ServiceManager, LoggerManager, CalibratableManager) for component lookup

# Key Source Files

| File | Purpose |
|------|---------|
| `core/protocol/base_types.hpp` | ParamID enum, BaseEncoding, BaseType variant, enums |
| `core/protocol/dte_params.cpp` | Parameter map (name, key, type, range, flags) |
| `core/protocol/dte_protocol.hpp` | Encoder/decoder for DTE protocol |
| `core/configuration/config_store.hpp` | Default values, config version, read/write API |
| `core/configuration/config_store_fs.hpp` | Flash serializer/deserializer |
| `core/services/service.hpp` | Service base class |
| `core/services/sensor_service.hpp` | SensorService base class |
| `core/hardware/sensor.hpp` | Sensor abstract interface |
| `core/logging/messages.hpp` | Log entry structures |
| `tests/src/dte_handler_test.cpp` | DTE protocol tests |

---

# Adding Parameters

Adding a parameter touches 5-6 files in a specific order:

```
1. base_types.hpp     → ParamID enum + optional new enum type
2. dte_params.cpp     → Parameter map entry
3. dte_protocol.hpp   → Encode/decode (only if new enum type)
4. config_store.hpp   → Default value + version bump
5. config_store_fs.hpp → Serializer/deserializer (only if new variant type)
6. tests              → Update PARML/PARMR test strings
```

## Step 1: Add ParamID (`core/protocol/base_types.hpp`)

```cpp
enum class ParamID {
    // ... existing params ...
if ENABLE_PRESSURE_SENSOR
    PRESSURE_SENSOR_FULL_SCALE = 177,
endif
    __PARAM_SIZE = 206,  // Increment this!
    __NULL_PARAM = 0xFFFF
};
```

**Rules:**
- Slots are always reserved even when `#if` guard is false (stable indexing)
- Use `#if ENABLE_*` guard if parameter is sensor-specific
- Increment `__PARAM_SIZE` by 1

If you need a new enum type, add the enum class, add it to `BaseEncoding`, and add it to the `BaseType` variant (at the end).

## Step 2: Add Parameter Map Entry (`core/protocol/dte_params.cpp`)

```cpp
// [177] Pressure sensor full scale
{ "PRESSURE_SENSOR_FULL_SCALE", "PRP07", BaseEncoding::PRESSURESENSORFULLSCALE,
  0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
```

Fields: name, key (5-char: 3-letter prefix + 2-digit number), encoding, min/max (0,0 = no range check), permitted_values (empty = use range), is_implemented (ENABLE_* macro), is_writable.

## Step 3: Add Encode/Decode (`core/protocol/dte_protocol.hpp`)

Only needed for new enum types. Standard types (UINT, FLOAT, BOOLEAN, TEXT) are already handled. Add encoder (`encode()`) and decoder function, then wire them into the main encode/decode switch.

## Step 4: Add Default Value (`core/configuration/config_store.hpp`)

Add the default to `default_params[]` at the correct position. **Bump the config version** to force factory reset on existing devices:

```cpp
static inline const unsigned int m_config_version_code = 0x1c07e800 | 0x15;
//                                                   was 0x14 ────────────^
```

## Step 5: Add Serializer/Deserializer (`core/configuration/config_store_fs.hpp`)

Only needed if you added a new type to the `BaseType` variant. Add `operator()` overload in serializer and switch case in deserializer.

## Step 6: Update Tests (`tests/src/dte_handler_test.cpp`)

Update the PARML expected response string (add new key) and PARMR expected response string (add key=default). Recalculate `#LEN` hex values.

## Step 7: Use the Parameter

```cpp
void sensor_init() override {
    unsigned int fs = (unsigned int)service_read_param<BasePressureSensorFullScale>(
        ParamID::PRESSURE_SENSOR_FULL_SCALE);
    m_sensor.set_full_scale(fs);
}
```

## Checklist

- [ ] ParamID added with correct slot, `__PARAM_SIZE` incremented
- [ ] New enum type added to BaseEncoding + BaseType variant (if needed)
- [ ] `param_map[]` entry added at correct position
- [ ] Encode/decode added (if new enum type)
- [ ] Default value in `default_params[]`, config version bumped
- [ ] Serializer/deserializer updated (if new variant type)
- [ ] PARML and PARMR test strings updated, all tests pass

---

# Adding Sensors

## Sensor Hierarchy

```
Sensor (core/hardware/sensor.hpp)          ← Abstract interface
  └── MyDeviceDriver (ports/nrf52840/...)  ← Hardware-specific driver

MySensorWrapper (ports/nrf52840/...)       ← Wraps device, implements Sensor
  └── Registers with SensorManager("NAME")
```

## Step 1: Create the Device Driver

In `ports/nrf52840/core/hardware/my_sensor/my_device.cpp`:

```cpp
void MyDevice::read(double& value1, double& value2) {
    SensorsPowerGuard power_guard;  // VSENSORS ON

    // IMPORTANT: Re-apply configuration after power cycle!
    // Sensor registers are volatile and lost when VSENSORS is off.
    NrfI2C::write(m_bus, m_addr, &reg, 1, true);
    NrfI2C::read(m_bus, m_addr, buf, len);

    value1 = /* ... */;
    value2 = /* ... */;
}  // VSENSORS OFF (guard destroyed)
```

**Key pattern**: Always use `SensorsPowerGuard` for I2C access and re-apply sensor configuration in `read()`.

## Step 2: Create the Sensor Wrapper

```cpp
class MySensor : public Sensor {
public:
    MySensor(MyDevice& device)
        : Sensor("MY_SENSOR"), m_device(device) {}

    double read(unsigned int port = 0) override {
        double v1, v2;
        m_device.read(v1, v2);
        switch (port) {
            case 0: return v1;
            case 1: return v2;
            default: return 0.0;
        }
    }
private:
    MyDevice& m_device;
};
```

The Sensor constructor registers with SensorManager using the provided name.

## Step 3: Add Calibration Support (Optional)

Override `calibration_write/read/save` from `Calibratable`. Add SCALW/SCALR mapping in DTEHandler for the new sensor type.

## Step 4: Instantiate in Board Application

```cpp
MyDevice my_device(ONBOARD_I2C_BUS, MY_DEVICE_I2C_ADDR);
MySensor my_sensor(my_device);
MySensorService my_service(my_sensor, &my_logger);
```

## Step 5: BSP & Build

Add I2C address and pin definitions in the board's `bsp.hpp`. Add `ENABLE_MY_SENSOR` flag in CMakeLists.txt.

## Checklist

- [ ] Device driver with SensorsPowerGuard and config re-apply in read()
- [ ] Sensor wrapper implementing `Sensor::read(port)`, registered with SensorManager
- [ ] BSP definitions (I2C address, bus, pins)
- [ ] CMake `ENABLE_*` flag
- [ ] Service, parameters, and calibration support (if needed)

---

# Adding Services

## Service Types

| Base Class | Use Case |
|-----------|----------|
| `Service` | Non-sensor services (Argos TX, dive mode, memory monitor) |
| `SensorService` | Sensor-based services with periodic sampling and logging |
| `UWDetectorService` | Underwater detection services |

## Creating a SensorService

**Step 1 — Define log entry structure:**

```cpp
struct __attribute__((packed)) MySensorLogEntry {
    LogHeader header;
    union {
        struct {
            double value1;
            double value2;
        };
        uint8_t data[MAX_LOG_PAYLOAD];
    };
};
```

**Step 2 — Define log formatter:**

```cpp
class MySensorLogFormatter : public LogFormatter {
public:
    const std::string header() override {
        return "log_datetime,value1,value2\r\n";
    }
    const std::string log_entry(const LogEntry& e) override {
        const MySensorLogEntry *log = (const MySensorLogEntry *)&e;
        // Format timestamp + values as CSV
    }
};
```

**Step 3 — Implement the service:**

```cpp
class MySensorService : public SensorService {
public:
    MySensorService(Sensor& sensor, Logger *logger = nullptr)
        : SensorService(sensor, ServiceIdentifier::MY_SENSOR, "MY_SENSOR", logger) {}

private:
    void sensor_populate_log_entry(LogEntry *e, ServiceSensorData& data) override {
        MySensorLogEntry *log = (MySensorLogEntry *)e;
        log->value1 = data.port[0];
        log->value2 = data.port[1];
        service_set_log_header_time(log->header, service_current_time());
    }
    unsigned int sensor_max_samples() override { /* from config */ }
    unsigned int sensor_num_channels() override { return 2U; }
    bool sensor_is_enabled() override { /* from config */ }
    unsigned int sensor_periodic() override { /* from config, *1000 */ }
    unsigned int sensor_tx_periodic() override { /* from config */ }
    BaseSensorEnableTxMode sensor_enable_tx_mode() override { /* from config */ }
};
```

**Step 4 — Register:** The service auto-registers with ServiceManager in its constructor.

**Step 5 — Add parameters:** MY_SENSOR_ENABLE, MY_SENSOR_PERIODIC, MY_SENSOR_ENABLE_TX_MODE, etc. (see Adding Parameters).

**Step 6 — Add ServiceIdentifier** to the enum.

**Step 7 — Add log type mapping** in DTEHandler for DUMPD/ERASE commands.

## Creating a Non-Sensor Service

```cpp
class MyService : public Service {
public:
    MyService() : Service(ServiceIdentifier::MY_SERVICE, "MY_SERVICE") {}
private:
    void service_init() override { /* setup */ }
    void service_term() override { /* cleanup */ }
    bool service_is_enabled() override { return true; }
    unsigned int service_next_schedule_in_ms() override { return 60000; }
    void service_initiate() override {
        // Do work
        service_complete();
    }
};
```

---

# Board Porting

## Architecture

```
core/                           # Portable: no hardware dependencies
├── hardware/sensor.hpp         # Abstract Sensor interface
├── services/                   # All service logic
├── protocol/                   # DTE protocol
├── configuration/              # Config store interface
└── logging/                    # Logger interface

ports/nrf52840/                 # Hardware-specific
├── bsp/                        # Board Support Packages
│   ├── linkitv4_v1.0/bsp.hpp
│   └── rspbtracker_v1.0/bsp.hpp
├── core/hardware/              # Sensor/peripheral drivers
└── CMakeLists.txt
```

## Step 1: Create BSP

Create `ports/<mcu>/bsp/<board_name>/bsp.hpp` with all hardware mappings:

```cpp
pragma once

// Peripherals
define RTC_DATE_TIME    RTC_1
define SPI_SATELLITE    SPI_2
define UART_GPS         UART_0

// I2C
define ONBOARD_I2C_BUS  I2C_0
define BMA400_I2C_ADDR  0x14
define LPS28DFW_I2C_ADDR 0x5C

// GPIO
define GPS_POWER        GPIO_GPS_PWR_EN
define SAT_PWR_EN       GPIO_SAT_EN
define SWS_ENABLE_PIN   GPIO_SWS_SEND

// Feature flags
define POWER_ON_RESET_REQUIRES_REED_SWITCH  1
```

## Step 2: Implement Hardware Abstractions

Required interfaces from `core/`:

| Interface | Methods |
|-----------|---------|
| Sensor | `read(port)` |
| Logger | `create()`, `write()`, `read()`, `num_entries()`, `truncate()` |
| ConfigurationStore | `init()`, `factory_reset()`, `serialize_config()`, etc. |
| Timer | Scheduling and RTC |
| PMU | `delay_ms()`, `hardware_version()`, `device_identifier()` |

Plus platform-specific I2C, SPI, GPIO implementations.

## Step 3: Implement SensorsPowerGuard

Critical RAII pattern:

```cpp
class SensorsPowerGuard {
public:
    SensorsPowerGuard() { GPIOPins::acquire_sensors_pwr(); }
    ~SensorsPowerGuard() { GPIOPins::release_sensors_pwr(); }
};
```

All sensor drivers must use this guard and re-apply configuration after power-on.

## Existing Board References

| Board | BSP | Key Differences |
|-------|-----|-----------------|
| LinkIt V4 v1.0 | `bsp/linkitv4_v1.0/bsp.hpp` | Analog battery, 2 I2C buses, reed switch boot, SMD INT |
| RSPB v1.0 | `bsp/rspbtracker_v1.0/bsp.hpp` | STC3117 fuel gauge, TPL5111, SMD satellite, thermistor ADC |

---

# Testing

## Framework

The project uses **CppUTest** for unit testing. Tests mock hardware dependencies so they run on the host machine (x86/x64).

## Building and Running

```bash
cd tests && mkdir build && cd build
cmake .. && make -j$(nproc)

ctest --output-on-failure         # Run all tests
./TrackerTests -g PressureSensorTest  # Specific test group
./TrackerTests -v                     # Verbose output
```

## Test Build Configuration

The test CMake defines all `ENABLE_*` flags to 1, ensuring all code paths are compiled and tested regardless of the target board.

## Mocks and Fakes

| Mock/Fake | Purpose |
|-----------|---------|
| mock_config_store | In-memory ConfigurationStore |
| fake_timer | Controllable timer for scheduling tests |
| fake_sensor | Programmable sensor returning preset values |
| fake_logger | In-memory logger capturing log entries |
| fake_battery_monitor | Configurable battery level/voltage |

## Writing Tests

```cpp
include "CppUTest/TestHarness.h"

TEST_GROUP(MySensorTest) {
    FakeSensor sensor;
    FakeLogger logger;
    MySensorService *service;

    void setup() { service = new MySensorService(sensor, &logger); }
    void teardown() { delete service; SensorManager::clear(); }
};

TEST(MySensorTest, BasicReading) {
    sensor.set_value(0, 1.013);
    // Exercise the service...
    DOUBLES_EQUAL(1.013, result, 0.001);
}
```

## DTE Handler Tests

When adding a parameter, update PARML and PARMR expected strings in `dte_handler_test.cpp`. Recalculate `#LEN` (hex byte count of payload between `;` and `\r`).

## Common Assertions

```cpp
CHECK(condition);
CHECK_EQUAL(expected, actual);
STRCMP_EQUAL(expected, actual);
DOUBLES_EQUAL(expected, actual, tolerance);
LONGS_EQUAL(expected, actual);
CHECK_THROWS(ExceptionType, expression);
```
