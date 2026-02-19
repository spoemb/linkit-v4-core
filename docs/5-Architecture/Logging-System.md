# Logging System

## Overview

The logging system provides per-service flash-based data logging with CSV-formatted retrieval via the DUMPD command.

## Components

### LogEntry Structure
Location: `core/logging/messages.hpp`

```cpp
struct LogHeader {           // 9 bytes
    uint8_t  day, month;
    uint16_t year;
    uint8_t  hours, minutes, seconds;
    LogType  log_type;
    uint8_t  payload_size;
};

struct LogEntry {            // 128 bytes total
    LogHeader header;
    uint8_t data[MAX_LOG_PAYLOAD];  // 119 bytes payload
};
```

### Service-Specific Log Entries

Each service defines a packed struct overlaying LogEntry:

| Service | Log Struct | Fields |
|---------|-----------|--------|
| GPS | GPSLogEntry | event_type, batt_voltage, iTOW, fix data (lat, lon, height, etc.) |
| Pressure | PressureLogEntry | pressure (bar), temperature (C), altitude (m) |
| AXL | AXLLogEntry | x, y, z (g), activity, temperature |
| Thermistor | ThermistorLogEntry | temperature (C) |
| Camera | CAMLogEntry | event_type, batt_voltage, counter |
| Underwater | UnderwaterLogEntry | DRY or WET event |
| Battery | BatteryLogEntry | event, voltage |
| System | SystemStartupLogEntry | cause (BROWNOUT, WATCHDOG, etc.) |

### Logger Abstract Class
Location: `core/logging/logger.hpp`

```cpp
class Logger {
    virtual void create() = 0;        // Create log file
    virtual void write(void *) = 0;   // Write entry
    virtual void read(void *, int) = 0; // Read entry by index
    virtual unsigned int num_entries() = 0;
    virtual void truncate() = 0;      // Clear all entries
};
```

### LogFormatter
Converts raw log entries to CSV strings:

```cpp
class LogFormatter {
    virtual const std::string header() = 0;        // CSV header line
    virtual const std::string log_entry(const LogEntry& e) = 0; // Format one entry
};
```

Example (PressureLogFormatter):
- Header: `log_datetime,pressure,temperature,altitude\r\n`
- Entry: `18/02/2026 10:30:00,1.013250,22.500000,10.52\r\n`

### LoggerManager
Static registry:
- add(Logger&) / remove(Logger&)
- create() - Create all log files
- truncate() - Clear all logs
- find_by_name(name) - Lookup logger

## Log Retrieval

Logs are retrieved via the DUMPD command, which paginates entries and returns base64-encoded CSV data. The first packet includes the CSV header line.

## Log Types

| Type ID | Name | Description |
|---------|------|-------------|
| 0 | GPS | GNSS fixes |
| 1 | CAM | Camera events |
| 2 | AXL | Accelerometer data |
| 3 | STARTUP | System startup events |
| 4 | UNDERWATER | Wet/dry transitions |
| 5 | BATTERY | Battery events |
| 6 | STATE | State changes |
| 7 | ZONE | Geofence events |
| 8 | OTA_UPDATE | Firmware update events |
| 9 | BLE | BLE connection events |
