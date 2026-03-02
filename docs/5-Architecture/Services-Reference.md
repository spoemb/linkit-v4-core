# Services Reference

## Service Identifiers

Defined in ServiceIdentifier enum (core/logging/messages.hpp via service.hpp).

## Core Services

### ARGOSTxService
- **File**: core/services/argos_tx_service.hpp
- **Purpose**: Manages Argos satellite transmissions with pass prediction
- **Modes**: OFF, PASS_PREDICTION, LEGACY, DUTY_CYCLE, DOPPLER
- **Features**: Depth piling, sensor data appending, certification TX mode
- **Key Params**: ARGOS_MODE, TR_NOM, NTRY_PER_MESSAGE, DUTY_CYCLE, ARGOS_DEPTH_PILE

### ARGOSRxService
- **File**: core/services/argos_rx_service.hpp
- **Purpose**: Receives AOP updates from Argos downlink
- **Key Params**: ARGOS_RX_EN, ARGOS_RX_MAX_WINDOW, ARGOS_RX_AOP_UPDATE_PERIOD

### GPSService
- **File**: core/services/gps_service.hpp
- **Purpose**: GNSS acquisition with configurable filtering and dynamic models
- **Features**: Cold start retry, AssistNow, HDOP/HACC filtering, surface/AXL triggers
- **Key Params**: GNSS_EN, GNSS_ACQ_TIMEOUT, GNSS_FIX_MODE, GNSS_DYN_MODEL

## Sensor Services

### PressureSensorService
- **File**: core/services/pressure_sensor_service.hpp
- **Channels**: 2 (pressure in bar, temperature in C)
- **Computed**: Barometric altitude (m) using hypsometric formula
- **Logging**: CSV: log_datetime, pressure, temperature, altitude
- **Modes**: ALWAYS (log every sample) or UW_THRESHOLD (log on threshold crossing)
- **Key Params**: PRP01-PRP07

### AXLSensorService
- **File**: core/services/axl_sensor_service.hpp
- **Channels**: 6 (X, Y, Z, activity, temperature, wakeup_triggered)
- **Features**: Wakeup event detection, activity monitoring
- **Key Params**: AXP01-AXP09

### ThermistorSensorService
- **File**: core/services/thermistor_sensor_service.hpp
- **Channels**: 1 (temperature in C)
- **Features**: Wakeup threshold detection
- **Key Params**: THP01-THP08

### ALSSensorService
- **File**: core/services/als_sensor_service.hpp
- **Channels**: 1 (lumens)
- **Key Params**: LTP01-LTP06

### PHSensorService
- **File**: core/services/ph_sensor_service.hpp
- **Channels**: 1 (pH value)
- **Key Params**: PHP01-PHP06

### SeaTempSensorService
- **File**: core/services/sea_temp_sensor_service.hpp
- **Channels**: 1 (temperature in C)
- **Supports**: RTD or TSYS01 sensor
- **Key Params**: STP01-STP06

### CDTSensorService
- **File**: core/services/cdt_sensor_service.hpp
- **Channels**: 3 (conductivity, depth, temperature)
- **Key Params**: CDP01-CDP05

### CAMService
- **File**: core/services/cam_service.hpp
- **Purpose**: Camera trigger on surfaced/AXL wakeup events
- **Key Params**: CAP01-CAP05

## Underwater Detection Services

### UWDetectorService (Abstract Base)
- **File**: core/services/uwdetector_service.hpp
- **Purpose**: Abstract base for underwater detection with temporal filtering
- **Features**: Configurable dry/wet sampling, min samples before state change

### SWSService
- **File**: core/services/sws_service.hpp
- **Purpose**: Digital saltwater switch detection
- **Source**: UNDERWATER_DETECT_SOURCE = SWS

### SWSAnalogService
- **File**: core/services/sws_analog_service.hpp
- **Purpose**: Analog SWS with auto-calibration, hysteresis, trend detection
- **Features**: Persistent calibration in noinit RAM, automatic air/water baseline learning, 3-tier rapid transition detection, biofouling adaptation
- **Test Mode**: `SWSTST` DTE command enables standalone testing with RGB LED feedback (BLUE=underwater, YELLOW=surface). See [SWS Analog Implementation](../sws_analog_implementation.md#test-mode)
- **Key Params**: UNP20-UNP23

### PressureDetectorService
- **File**: core/services/pressure_detector_service.hpp
- **Purpose**: Underwater detection using pressure threshold
- **Source**: UNDERWATER_DETECT_SOURCE = PRESSURE_SENSOR

### GNSSDetectorService
- **File**: core/services/gnss_detector_service.hpp
- **Purpose**: Underwater detection using GNSS signal quality
- **Source**: UNDERWATER_DETECT_SOURCE = GNSS or SWS_GNSS

## Utility Services

### DiveModeService
- **File**: core/services/dive_mode_service.hpp
- **Purpose**: Dive state machine with reed switch pause/resume
- **Key Params**: UNP12, UNP13

### MemoryMonitorService
- **File**: core/services/memory_monitor_service.hpp
- **Purpose**: Logs heap/stack statistics every 12 hours
