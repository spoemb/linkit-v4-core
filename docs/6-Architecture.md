# 6 - Firmware Architecture

## Layered Architecture

```
┌─────────────────────────────────┐
│         Application             │  main.cpp, state machine
├─────────────────────────────────┤
│      Services Framework         │  ServiceManager, 15+ services
├─────────────────────────────────┤
│     DTE Protocol Layer          │  DTEHandler, encoder/decoder
├─────────────────────────────────┤
│    Configuration Store          │  178 params, flash persistence
├─────────────────────────────────┤
│   Hardware Abstraction Layer    │  Sensor, Timer, PMU, GPIO
├─────────────────────────────────┤
│    Board Support Package        │  linkitv4_v1.0, rspbtracker_v1.0
├─────────────────────────────────┤
│     nRF52840 + SoftDevice       │  Nordic SDK 17.1.0
└─────────────────────────────────┘
```

## Directory Organization

- `core/` — Portable code, no hardware dependencies
  - `core/services/` — Service implementations
  - `core/protocol/` — DTE protocol, encoder, decoder, params
  - `core/configuration/` — ConfigStore, calibration
  - `core/hardware/` — Abstract interfaces (Sensor, Timer)
  - `core/logging/` — Logger, messages
- `ports/nrf52840/` — Hardware-specific implementations
  - `ports/nrf52840/bsp/` — Board support packages
  - `ports/nrf52840/core/hardware/` — Sensor drivers (BMA400, LPS28DFW, M8, etc.)

## Key Design Patterns

- **Dependency Inversion**: `core/` defines abstract interfaces, `ports/` provides implementations
- **RAII**: SensorsPowerGuard for VSENSORS power management
- **Static Registry**: SensorManager, LoggerManager, CalibratableManager, ServiceManager
- **Variant-based Configuration**: BaseType = std::variant<...> for type-safe parameter storage
- **Event-driven Services**: ServiceEvent for inter-service communication

---

## Service Framework

The Service Framework is the core scheduling and lifecycle engine. All firmware functionality (GNSS, Argos TX, sensors, underwater detection) is implemented as services managed by ServiceManager.

### Service Base Class

Location: `core/services/service.hpp`

**Lifecycle:**

1. **Construction** — Service registers with ServiceManager
2. **start()** — Calls service_init(), enables scheduling
3. **Scheduling loop** — ServiceManager calls service_initiate() based on service_next_schedule_in_ms()
4. **stop()** — Calls service_term(), disables scheduling

**Virtual methods to implement:**

| Method | Required | Description |
|--------|----------|-------------|
| service_init() | No | Initialize resources |
| service_term() | No | Release resources |
| service_is_enabled() | Yes | Is service enabled in config? |
| service_next_schedule_in_ms() | Yes | Interval until next execution |
| service_initiate() | Yes | Execute the service task |
| service_cancel() | No | Cancel pending operation |
| service_is_usable_underwater() | No | Can run underwater? (default: false) |
| service_is_triggered_on_surfaced() | No | Trigger when surface detected? |
| service_is_triggered_on_event() | No | Trigger on inter-service event? |

**Utility methods:**

- service_complete() — Mark task complete, optionally reschedule
- service_log() — Write entry to logger
- service_read_param\<T\>(ParamID) — Read configuration parameter
- service_write_param\<T\>(ParamID, value) — Write configuration parameter
- service_current_time() — Get system time
- service_is_battery_level_low() — Check low battery
- service_reschedule() — Force reschedule

### ServiceManager

Static class managing all services:
- add(Service&) / remove(Service&)
- startall() / stopall()
- notify_underwater_state(bool) — Broadcast underwater state
- notify_peer_event(ServiceEvent&) — Distribute events
- inject_event(ServiceEvent&) — Inject custom events

### Scheduling

Services return SCHEDULE_DISABLED (0xFFFFFFFF) to disable automatic scheduling. Otherwise return milliseconds until next execution. Services multiply seconds config values by 1000.

### SensorService

Location: `core/services/sensor_service.hpp`

Extends Service for sensor-based services. Handles periodic sensor reading, sample aggregation, log entry creation, and TX data collection.

| Method | Required | Description |
|--------|----------|-------------|
| sensor_populate_log_entry() | Yes | Fill log entry from sensor data |
| sensor_init() | No | Sensor-specific init (e.g., set full scale) |
| sensor_is_enabled() | Yes | Is sensor enabled? |
| sensor_periodic() | Yes | Sampling period in ms |
| sensor_tx_periodic() | Yes | TX sampling period in ms |
| sensor_max_samples() | Yes | Max samples per TX event |
| sensor_num_channels() | Yes | Number of channels (1-6) |
| sensor_enable_tx_mode() | Yes | TX aggregation mode |
| sensor_is_usable_underwater() | No | Usable underwater? |

**Aggregation modes:**

| Mode | Behavior |
|------|----------|
| OFF | Sensor data not included in TX |
| ONESHOT | First sample only |
| MEAN | Average of all samples |
| MEDIAN | Median of all samples |

---

## Services Reference

### Core Services

| Service | File | Purpose | Key Params |
|---------|------|---------|------------|
| ARGOSTxService | core/services/argos_tx_service.hpp | Argos satellite TX with pass prediction. Modes: OFF, PASS_PREDICTION, LEGACY, DUTY_CYCLE, DOPPLER | ARGOS_MODE, TR_NOM, NTRY_PER_MESSAGE, DUTY_CYCLE, ARGOS_DEPTH_PILE |
| ARGOSRxService | core/services/argos_rx_service.hpp | Receives AOP updates from Argos downlink | ARGOS_RX_EN, ARGOS_RX_MAX_WINDOW, ARGOS_RX_AOP_UPDATE_PERIOD |
| GPSService | core/services/gps_service.hpp | GNSS acquisition with filtering and dynamic models. Cold start retry, AssistNow, HDOP/HACC filtering | GNSS_EN, GNSS_ACQ_TIMEOUT, GNSS_FIX_MODE, GNSS_DYN_MODEL |

### Sensor Services

| Service | File | Channels | Key Params |
|---------|------|----------|------------|
| PressureSensorService | core/services/pressure_sensor_service.hpp | 2 (pressure bar, temperature C) + computed altitude | PRP01-PRP07 |
| AXLSensorService | core/services/axl_sensor_service.hpp | 6 (X, Y, Z, activity, temperature, wakeup) | AXP01-AXP09 |
| ThermistorSensorService | core/services/thermistor_sensor_service.hpp | 1 (temperature C) | THP01-THP08 |
| ALSSensorService | core/services/als_sensor_service.hpp | 1 (lumens) | LTP01-LTP06 |
| PHSensorService | core/services/ph_sensor_service.hpp | 1 (pH) | PHP01-PHP06 |
| SeaTempSensorService | core/services/sea_temp_sensor_service.hpp | 1 (temperature C, RTD or TSYS01) | STP01-STP06 |
| CDTSensorService | core/services/cdt_sensor_service.hpp | 3 (conductivity, depth, temperature) | CDP01-CDP05 |
| CAMService | core/services/cam_service.hpp | Camera trigger on surfaced/AXL wakeup | CAP01-CAP05 |

### Underwater Detection Services

| Service | File | Source | Description |
|---------|------|--------|-------------|
| SWSService | core/services/sws_service.hpp | SWS (0) | Digital saltwater switch |
| SWSAnalogService | core/services/sws_analog_service.hpp | SWS (0) | Analog SWS with auto-calibration, 5-level detection, biofouling adaptation. Test mode via `SWSTST` DTE command |
| PressureDetectorService | core/services/pressure_detector_service.hpp | PRESSURE_SENSOR (1) | Pressure threshold detection |
| GNSSDetectorService | core/services/gnss_detector_service.hpp | GNSS (2) / SWS_GNSS (3) | GNSS signal quality detection |

### Utility Services

| Service | File | Purpose |
|---------|------|---------|
| DiveModeService | core/services/dive_mode_service.hpp | Dive state machine with reed switch pause/resume |
| MemoryMonitorService | core/services/memory_monitor_service.hpp | Logs heap/stack statistics every 12 hours |

---

## Configuration System

The configuration system manages 178 typed parameters stored in flash memory. Parameters are accessed by ParamID enum and exposed to pylinkit via 5-character DTE keys.

### ConfigurationStore

Location: `core/configuration/config_store.hpp`

Abstract base class with pure virtual methods:
- init() — Load config from flash
- is_valid() — Check if config is loaded
- factory_reset() — Reset all params to defaults
- read_pass_predict() / write_pass_predict() — AOP satellite data
- serialize_config() — Save to flash

**Parameter access:**

```cpp
// Read parameter (type-safe)
bool gnss_en = configuration_store->read_param<bool>(ParamID::GNSS_EN);
unsigned int timeout = configuration_store->read_param<unsigned int>(ParamID::GNSS_ACQ_TIMEOUT);

// Write parameter
configuration_store->write_param(ParamID::GNSS_EN, true);
configuration_store->save_params();  // Persist to flash
```

### BaseType Variant

Parameters are stored as `std::variant`:
```cpp
using BaseType = std::variant<
    std::string, unsigned int, int, double, std::time_t, BaseRawData,
    BaseArgosMode, BaseArgosPower, BaseArgosDepthPile, bool,
    BaseGNSSFixMode, BaseGNSSDynModel, BaseLEDMode, BaseZoneType,
    BaseArgosModulation, BaseUnderwaterDetectSource, BaseDebugMode,
    BasePressureSensorLoggingMode, BasePressureSensorFullScale,
    BaseSensorEnableTxMode
>;
```

### Config Version Code

```cpp
static inline const unsigned int m_config_version_code = 0x1c07e800 | 0x15;
```

When the version code is bumped, existing devices perform a factory reset on next boot to load new defaults. This is necessary when adding new parameters or changing default values.

### Configuration Modes

The firmware dynamically selects parameter sets based on state:
- **NORMAL** — Default parameters
- **LOW_BATTERY** — LB_* parameters (when LB_EN=true and battery below LB_THRESHOLD)
- **OUT_OF_ZONE** — ZONE_* parameters (when outside geofencing zone)

### ConfigStoreFS

Location: `core/configuration/config_store_fs.hpp`

Flash-backed implementation using LittleFS. Serializes/deserializes BaseType variants to/from binary via operator() overloads (serializer) and switch cases (deserializer). Stores AOP pass prediction data separately.

### Calibration System

Location: `core/configuration/calibration.hpp`

```cpp
class Calibratable {
    virtual void calibration_write(const double value, const unsigned int offset) {}
    virtual void calibration_read(double& value, const unsigned int offset) {}
    virtual void calibration_save(bool force) {}
};
```

Devices inherit from Calibratable and register with CalibratableManager. Used for sea level pressure reference, accelerometer axis offsets, and SWS analog baselines.

DTE interface: SCALW (write calibration) and SCALR (read calibration). The Calibration helper class persists values to LittleFS files, with a map\<unsigned int, double\> keyed by offset.

---

## DTE Protocol

The DTE (Data Terminal Equipment) protocol handles communication between the firmware and external tools (pylinkit, LinkIt GUI). It provides parameter read/write, sensor access, log retrieval, and device management.

### Protocol Stack

```
pylinkit / LinkIt GUI (BLE)
        |
    DTE Framing ($CMD#LEN;PAYLOAD\r)
        |
    DTEDecoder (parse + validate)
        |
    DTEHandler (dispatch to command handler)
        |
    ConfigStore / SensorManager / LoggerManager
        |
    DTEEncoder (format response)
        |
    DTE Framing (response)
```

### Key Components

**DTEDecoder** (`core/protocol/dte_protocol.hpp`): Parses incoming DTE messages — extracts command name, payload length, payload. Validates length, decodes arguments based on command prototype (BaseEncoding). For PARMR/PARMW: resolves DTE keys to ParamIDs.

**DTEEncoder** (`core/protocol/dte_protocol.hpp`): Formats outgoing DTE responses — OK: `$O;CMD#LEN;PAYLOAD\r`, Error: `$N;CMD#LEN;ERROR_CODE\r`. Encodes BaseType values to string using BaseEncoding rules.

**DTEHandler** (`core/protocol/dte_handler.cpp`): Central dispatcher. Calls DTEDecoder::decode(), switches on DTECommand enum, calls the appropriate handler, returns DTEAction for post-command processing (RESET, FACTR, CONFIG_UPDATED, etc.).

### BaseEncoding Types

| Encoding | Wire Format | C++ Type |
|----------|-------------|----------|
| UINT | Decimal string | unsigned int |
| FLOAT | Decimal with decimals | double |
| BOOLEAN | "0" or "1" | bool |
| HEXADECIMAL | Hex string | unsigned int |
| TEXT | Raw string | std::string |
| DATESTRING | DD/MM/YYYY HH:MM:SS | std::time_t |
| BASE64 | Base64-encoded binary | BaseRawData |
| ARGOSMODE | "0"-"4" | BaseArgosMode |
| GNSSFIXMODE | "1"-"3" | BaseGNSSFixMode |
| GNSSDYNMODEL | "0"-"10" | BaseGNSSDynModel |
| LEDMODE | "0","1","3" | BaseLEDMode |
| DEPTHPILE | Decimal | BaseArgosDepthPile |
| MODULATION | "0"-"2" | BaseArgosModulation |
| UWDETECTSOURCE | "0"-"3" | BaseUnderwaterDetectSource |
| DEBUGMODE | "0"-"2" | BaseDebugMode |
| PRESSURESENSORLOGGINGMODE | "0"-"1" | BasePressureSensorLoggingMode |
| PRESSURESENSORFULLSCALE | "0"-"1" | BasePressureSensorFullScale |
| SENSORENABLETXMODE | "0"-"3" | BaseSensorEnableTxMode |
| KEY_LIST | Comma-separated keys | vector\<ParamID\> |
| KEY_VALUE_LIST | key=value pairs | vector\<ParamValue\> |

### Argos Encoder/Decoder

For satellite transmission, a separate binary encoder/decoder packs GPS fixes and sensor data into compact Argos packets. DTEEncoder bit-packs GPS fixes into depth-piled payloads, DTEDecoder decodes received satellite data.

### Parameter Map

Location: `core/protocol/dte_params.cpp`

Static array mapping ParamID to DTE metadata:
```cpp
struct BaseMap {
    BaseName name;           // Human-readable name
    BaseKey key;             // 5-char DTE key (e.g., "GNP01")
    BaseEncoding encoding;   // Type encoding
    BaseConstraint min_value; // Minimum value
    BaseConstraint max_value; // Maximum value
    vector<BaseConstraint> permitted_values; // Enum values
    bool is_implemented;     // Available in this build?
    bool is_writable;        // Can be written via PARMW?
};
```

The is_implemented flag uses ENABLE_* macros, so disabled sensors have their keys hidden from PARML.

---

## Hardware Abstraction Layer

The firmware uses abstract C++ interfaces in `core/hardware/` that are implemented in `ports/nrf52840/core/hardware/`. This allows the core firmware to be portable across different MCUs.

### Sensor Abstraction

Location: `core/hardware/sensor.hpp`

```cpp
class Sensor : public Calibratable {
public:
    Sensor(const char *name = "Sensor");
    virtual double read(unsigned int port = 0) = 0;
    virtual void install_event_handler(unsigned int, std::function<void()>) {}
    virtual void remove_event_handler(unsigned int) {}
    virtual void set_full_scale(unsigned int mode) {}
};
```

**SensorManager**: Static registry — add(Sensor&, name), find_by_name(name), clear().

### Sensor Implementations

| Name | Driver | Bus | File |
|------|--------|-----|------|
| "PRS" | LPS28DFW (pressure) | I2C | ports/nrf52840/core/hardware/lps28dfw/ |
| "AXL" | BMA400 (accelerometer) | I2C | ports/nrf52840/core/hardware/bma400/ |
| "RTD" | OEM RTD (temperature) | I2C | ports/nrf52840/core/hardware/ |
| "TSYS01" | TSYS01 (temperature) | I2C | ports/nrf52840/core/hardware/ |
| "ALS" | LTR-303 (ambient light) | I2C | ports/nrf52840/core/hardware/ |
| "PH" | OEM pH | I2C | ports/nrf52840/core/hardware/ |
| "CDT" | AD5933 + ADS1115 | I2C | ports/nrf52840/core/hardware/ |
| "THERM" | NTC Thermistor | ADC | ports/nrf52840/core/hardware/ |

### Power Management

**SensorsPowerGuard (RAII):** Acquires VSENSORS power rail on construction, releases on destruction. Sensor I2C registers are volatile and lost when VSENSORS is powered off — drivers must re-apply configuration in every read() call.

```cpp
void LPS28DFW::read(double& temperature, double& pressure) {
    SensorsPowerGuard power_guard;  // VSENSORS ON
    lps28dfw_init_set(&m_ctx, LPS28DFW_DRV_RDY);
    lps28dfw_mode_set(&m_ctx, &m_mode);
    // ... read sensor ...
}  // VSENSORS OFF (guard destroyed)
```

**PMU (Power Management Unit):** Abstract interface — delay_ms(), hardware_version(), device_identifier().

### Communication Interfaces

- **I2C**: NrfI2C::write/read, two buses (ONBOARD_I2C_BUS, EXTERNAL_I2C_BUS)
- **SPI**: Satellite module communication (SMD A+ protocol), SPI_SATELLITE bus
- **UART**: UART_GPS for u-blox GPS, debug output via UART or USB CDC
- **GPIO**: GPIOPins::set/clear/read, acquire_sensors_pwr/release_sensors_pwr, named pins in BSP

### LED Abstraction

**Single-Color LED** (`core/hardware/led.hpp`): on(), off(), get_state(), flash(period_ms), is_flashing(). Implementations: GPIOLed, NrfLed.

**RGB LED** (`core/hardware/rgb_led.hpp`): set(color), off(), flash(color, period_ms), flash_alternate(color1, color2, period_ms). Implementation: NrfRGBLed (active-low, timer-based).

| Pin | GPIO | Function |
|-----|------|----------|
| GPIO_LED_RED | P1.07 | Red channel (active low) |
| GPIO_LED_GREEN | P1.10 | Green channel (active low) |
| GPIO_LED_BLUE | P1.04 | Blue channel (active low) |

Global instances: `status_led` (RGB, main), `ext_status_led` (optional single-color). The LED state machine (`core/sm/ledsm.hpp`) controls `status_led` during normal operation.

---

## Logging System

The logging system provides per-service flash-based data logging with CSV-formatted retrieval via the DUMPD command.

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

### Logger and LogFormatter

**Logger** (`core/logging/logger.hpp`): create(), write(void*), read(void*, int), num_entries(), truncate().

**LogFormatter**: Converts raw log entries to CSV. Each service implements header() and log_entry(). Example: `log_datetime,pressure,temperature,altitude\r\n`.

**LoggerManager**: Static registry — add/remove, create(), truncate(), find_by_name().

### Log Types

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

Logs are retrieved via the DUMPD command, which paginates entries and returns base64-encoded CSV data.

---

## Message & Event System

Services communicate through a typed event system. Events are broadcast by ServiceManager to all registered services.

### ServiceEvent

Services emit and receive ServiceEvent objects containing:
- **event_source**: ServiceIdentifier (which service sent the event)
- **event_type**: Type of event
- **event_data**: std::variant with event-specific payload

### Event Types

| Event | Source | Description |
|-------|--------|-------------|
| SERVICE_INIT | Any | Service initialized |
| SERVICE_ACTIVE | Any | Service started processing |
| SERVICE_INACTIVE | Any | Service completed processing |
| SERVICE_LOG_UPDATED | Sensors | New log entry written |
| GNSS_ON | GPS | GNSS acquisition started |
| GNSS_OFF | GPS | GNSS acquisition completed |

### Event Flow

1. Service calls service_complete() or directly emits event
2. ServiceManager::notify_peer_event() broadcasts to all services
3. Each service's notify_peer_event() checks if event is relevant
4. Service may trigger its own action in response

### ServiceIdentifier Enum

| Value | Service |
|-------|---------|
| SERVICE_MANAGER | ServiceManager itself |
| GNSS_SENSOR | GPSService |
| UW_SENSOR | Underwater detection |
| ALS_SENSOR | Ambient light |
| AXL_SENSOR | Accelerometer |
| CDT_SENSOR | Conductivity-Depth-Temp |
| PH_SENSOR | pH |
| PRESSURE_SENSOR | Pressure |
| SEA_TEMP_SENSOR | Sea temperature |
| THERMISTOR_SENSOR | Thermistor |
| CAM_SENSOR | Camera |
| ARGOS_TX_SERVICE | Argos TX |
| ARGOS_RX_SERVICE | Argos RX |
| DIVE_MODE | Dive mode |
| MEMORY_MONITOR | Memory monitor |

### Underwater State Notification

A special broadcast path exists for underwater state changes:
```cpp
ServiceManager::notify_underwater_state(bool is_underwater);
```
This notifies all services simultaneously, allowing them to defer scheduling when submerged, trigger GNSS acquisition on surfacing, or start/stop dive mode tracking.
