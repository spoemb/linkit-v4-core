# Service Framework

## Overview

The Service Framework is the core scheduling and lifecycle engine. All firmware functionality (GNSS, Argos TX, sensors, underwater detection) is implemented as services managed by ServiceManager.

## Service Base Class

Location: `core/services/service.hpp`

### Lifecycle

1. **Construction** - Service registers with ServiceManager
2. **start()** - Calls service_init(), enables scheduling
3. **Scheduling loop** - ServiceManager calls service_initiate() based on service_next_schedule_in_ms()
4. **stop()** - Calls service_term(), disables scheduling

### Virtual Methods to Implement

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

### Utility Methods

- service_complete() - Mark task complete, optionally reschedule
- service_log() - Write entry to logger
- service_read_param<T>(ParamID) - Read configuration parameter
- service_write_param<T>(ParamID, value) - Write configuration parameter
- service_current_time() - Get system time
- service_is_battery_level_low() - Check low battery
- service_reschedule() - Force reschedule

### ServiceManager

Static class managing all services:
- add(Service&) / remove(Service&)
- startall() / stopall()
- notify_underwater_state(bool) - Broadcast underwater state
- notify_peer_event(ServiceEvent&) - Distribute events
- inject_event(ServiceEvent&) - Inject custom events

### Scheduling

Services return SCHEDULE_DISABLED (0xFFFFFFFF) to disable automatic scheduling. Otherwise return milliseconds until next execution. Services multiply seconds config values by 1000.

## SensorService

Location: `core/services/sensor_service.hpp`

Extends Service for sensor-based services. Handles:
- Periodic sensor reading
- Sample aggregation (ONESHOT, MEAN, MEDIAN)
- Log entry creation
- TX data collection

### Additional Virtual Methods

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

### Aggregation Modes

| Mode | Behavior |
|------|----------|
| OFF | Sensor data not included in TX |
| ONESHOT | First sample only |
| MEAN | Average of all samples |
| MEDIAN | Median of all samples |
