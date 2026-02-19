# Testing

## Framework

The project uses **CppUTest** for unit testing. Tests mock hardware dependencies so they run on the host machine (x86/x64), not on the target nRF52840.

## Building Tests

```bash
cd tests
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Running Tests

```bash
# Run all tests
ctest --output-on-failure

# Run all tests via executable directly
./CLSGenTrackerTests

# Run a specific test group
./CLSGenTrackerTests -g PressureSensorTest

# Run a specific test
./CLSGenTrackerTests -g PressureSensorTest -n AltitudeCalculation

# Verbose output
./CLSGenTrackerTests -v
```

## Test Directory Structure

```
tests/
├── CMakeLists.txt           # Test build configuration
└── src/
    ├── dte_handler_test.cpp  # DTE protocol commands
    ├── encoder_test.cpp      # Argos data encoding
    ├── decoder_test.cpp      # Argos data decoding
    ├── pressure_sensor_test.cpp
    ├── axl_sensor_test.cpp
    ├── thermistor_test.cpp
    ├── als_sensor_test.cpp
    ├── ph_sensor_test.cpp
    ├── sea_temp_sensor_test.cpp
    ├── cdt_sensor_test.cpp
    ├── sws_test.cpp          # Digital saltwater switch
    ├── sws_analog_test.cpp   # Analog SWS with calibration
    ├── gnss_detector_test.cpp
    ├── dive_mode_test.cpp
    ├── gps_test.cpp
    ├── argos_test.cpp
    ├── argos_tx_test.cpp
    ├── config_store_test.cpp
    ├── calibration_test.cpp
    ├── logger_test.cpp
    ├── timer_test.cpp
    ├── scheduler_test.cpp
    ├── filesystem_test.cpp
    ├── memory_monitor_test.cpp
    ├── haversine_test.cpp
    ├── timeutils_test.cpp
    ├── bitpack_test.cpp
    ├── binascii_test.cpp
    ├── crc8_test.cpp
    ├── crc16_test.cpp
    ├── crc32_test.cpp
    ├── bch_test.cpp
    └── ...
```

## Test Build Configuration

The test CMake defines **all** `ENABLE_*` flags to 1, ensuring all code paths are compiled and tested regardless of the target board:

```cmake
ENABLE_PRESSURE_SENSOR=1
ENABLE_AXL_SENSOR=1
ENABLE_THERMISTOR_SENSOR=1
ENABLE_PH_SENSOR=1
ENABLE_SEA_TEMP_SENSOR=1
ENABLE_ALS_SENSOR=1
ENABLE_CDT_SENSOR=1
ENABLE_CAM_SENSOR=1
EXTERNAL_WAKEUP=1
```

## Mocks and Fakes

Tests use mock/fake implementations for hardware dependencies:

- **mock_config_store** - In-memory ConfigurationStore with no flash
- **fake_timer** - Controllable timer for scheduling tests
- **fake_sensor** - Programmable sensor returning preset values
- **fake_logger** - In-memory logger capturing log entries
- **fake_battery_monitor** - Configurable battery level/voltage

### Mock Sensor Pattern

```cpp
class FakeSensor : public Sensor {
public:
    FakeSensor() : Sensor("TEST_SENSOR") {}
    double read(unsigned int port = 0) override {
        return m_values[port];
    }
    void set_value(unsigned int port, double value) {
        m_values[port] = value;
    }
private:
    double m_values[6] = {};
};
```

## Writing Tests

### Basic Test Structure

```cpp
#include "CppUTest/TestHarness.h"
#include "my_service.hpp"

TEST_GROUP(MySensorTest) {
    FakeSensor sensor;
    FakeLogger logger;
    MySensorService *service;

    void setup() {
        // Called before each test
        service = new MySensorService(sensor, &logger);
    }
    void teardown() {
        // Called after each test
        delete service;
        SensorManager::clear();
    }
};

TEST(MySensorTest, BasicReading) {
    sensor.set_value(0, 1.013);
    sensor.set_value(1, 22.5);
    // Exercise the service...
    DOUBLES_EQUAL(1.013, result, 0.001);
}
```

### DTE Handler Tests

When adding a parameter, update the PARML and PARMR test strings:

```cpp
TEST(DTEHandler, PARML_REQ) {
    // The expected string contains ALL parameter DTE keys, comma-separated
    // Update #LEN (hex) when adding keys
    std::string expected = "$O;PARML#3B3;IDP12,IDT06,...,PRP07\r";
    // ...
}

TEST(DTEHandler, PARMR_REQ_CheckEmptyRequest) {
    // The expected string contains ALL parameter key=value pairs
    // Update #LEN (hex) when adding params
    std::string expected = "$O;PARMR#546;IDP12=0,...,PRP07=0\r";
    // ...
}
```

**Tip**: To recalculate `#LEN`, count the bytes in the payload (everything after `;` and before `\r`), then convert to 3-digit hex.

## Common Assertions

```cpp
CHECK(condition);                    // Boolean check
CHECK_EQUAL(expected, actual);       // Equality check
STRCMP_EQUAL(expected, actual);       // String comparison
DOUBLES_EQUAL(expected, actual, tolerance); // Float comparison
LONGS_EQUAL(expected, actual);       // Integer comparison
CHECK_THROWS(ExceptionType, expression); // Exception check
```
