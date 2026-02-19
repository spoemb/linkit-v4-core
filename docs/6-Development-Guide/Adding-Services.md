# Adding Services

Guide for creating a new service in the firmware.

## Service Types

Choose the appropriate base class:

| Base Class | Use Case |
|-----------|----------|
| `Service` | Non-sensor services (Argos TX, dive mode, memory monitor) |
| `SensorService` | Sensor-based services with periodic sampling and logging |
| `UWDetectorService` | Underwater detection services |

## Creating a SensorService

### Step 1: Define Log Entry Structure

In a new header file (e.g., `core/services/my_sensor_service.hpp`):

```cpp
#pragma once
#include "sensor_service.hpp"
#include "messages.hpp"
#include "logger.hpp"
#include "timeutils.hpp"

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

### Step 2: Define Log Formatter

```cpp
class MySensorLogFormatter : public LogFormatter {
public:
    const std::string header() override {
        return "log_datetime,value1,value2\r\n";
    }
    const std::string log_entry(const LogEntry& e) override {
        char entry[512], d1[128];
        const MySensorLogEntry *log = (const MySensorLogEntry *)&e;
        std::time_t t = convert_epochtime(log->header.year, log->header.month,
            log->header.day, log->header.hours, log->header.minutes, log->header.seconds);
        std::tm *tm = std::gmtime(&t);
        std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);
        snprintf(entry, sizeof(entry), "%s,%f,%f\r\n", d1, log->value1, log->value2);
        return std::string(entry);
    }
};
```

### Step 3: Implement the Service Class

```cpp
class MySensorService : public SensorService {
public:
    MySensorService(Sensor& sensor, Logger *logger = nullptr)
        : SensorService(sensor, ServiceIdentifier::MY_SENSOR, "MY_SENSOR", logger) {}

private:
    // Fill log entry from sensor data
    void sensor_populate_log_entry(LogEntry *e, ServiceSensorData& data) override {
        MySensorLogEntry *log = (MySensorLogEntry *)e;
        log->value1 = data.port[0];
        log->value2 = data.port[1];
        service_set_log_header_time(log->header, service_current_time());
    }

    // Max samples per TX event
    unsigned int sensor_max_samples() override {
        return service_read_param<unsigned int>(ParamID::MY_SENSOR_ENABLE_TX_MAX_SAMPLES);
    }

    // Number of sensor channels to read
    unsigned int sensor_num_channels() override { return 2U; }

    // Is sensor enabled in config?
    bool sensor_is_enabled() override {
        return service_read_param<bool>(ParamID::MY_SENSOR_ENABLE);
    }

    // Sampling period in ms (0 = disabled)
    unsigned int sensor_periodic() override {
        unsigned int schedule = 1000 * service_read_param<unsigned int>(ParamID::MY_SENSOR_PERIODIC);
        return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
    }

    // TX sampling period in ms
    unsigned int sensor_tx_periodic() override {
        return service_read_param<unsigned int>(ParamID::MY_SENSOR_ENABLE_TX_SAMPLE_PERIOD);
    }

    // Can this sensor operate underwater?
    bool sensor_is_usable_underwater() override { return false; }

    // TX aggregation mode
    BaseSensorEnableTxMode sensor_enable_tx_mode() override {
        return service_read_param<BaseSensorEnableTxMode>(ParamID::MY_SENSOR_ENABLE_TX_MODE);
    }

    // Optional: sensor-specific initialization
    void sensor_init() override {
        // Configure sensor hardware
    }
};
```

### Step 4: Register the Service

In the board-specific application code (e.g., `ports/nrf52840/`):

```cpp
// Create sensor, logger, and service
MySensor my_sensor(I2C_BUS, ADDRESS);
Logger my_logger("MY_SENSOR");
MySensorLogFormatter my_formatter;
MySensorService my_service(my_sensor, &my_logger);

// Logger setup
my_logger.set_log_formatter(&my_formatter);
```

The service auto-registers with ServiceManager in its constructor via the Service base class. ServiceManager::startall() will start it.

### Step 5: Add Parameters

Follow [Adding Parameters](Adding-Parameters.md) to add:
- `MY_SENSOR_ENABLE` (BOOLEAN)
- `MY_SENSOR_PERIODIC` (UINT, seconds)
- `MY_SENSOR_ENABLE_TX_MODE` (SENSORENABLETXMODE)
- `MY_SENSOR_ENABLE_TX_MAX_SAMPLES` (UINT)
- `MY_SENSOR_ENABLE_TX_SAMPLE_PERIOD` (UINT, ms)

### Step 6: Add ServiceIdentifier

Add to the `ServiceIdentifier` enum in `core/services/service.hpp` (or wherever it's defined).

### Step 7: Add Log Type Mapping

Register the logger name in DTEHandler's log dump/erase maps so DUMPD and ERASE commands work with the new sensor type.

## Creating a Non-Sensor Service

For services that don't follow the sensor sampling pattern, inherit directly from `Service`:

```cpp
class MyService : public Service {
public:
    MyService() : Service(ServiceIdentifier::MY_SERVICE, "MY_SERVICE") {}

private:
    void service_init() override { /* setup */ }
    void service_term() override { /* cleanup */ }

    bool service_is_enabled() override { return true; }

    unsigned int service_next_schedule_in_ms() override {
        return 60000; // Run every 60 seconds
    }

    void service_initiate() override {
        // Do work here
        service_complete(); // Signal completion and reschedule
    }
};
```
