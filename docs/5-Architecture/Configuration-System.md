# Configuration System

## Overview

The configuration system manages 178 typed parameters stored in flash memory. Parameters are accessed by ParamID enum and exposed to pylinkit via 5-character DTE keys.

## ConfigurationStore

Location: `core/configuration/config_store.hpp`

Abstract base class with pure virtual methods:
- init() - Load config from flash
- is_valid() - Check if config is loaded
- factory_reset() - Reset all params to defaults
- read_pass_predict() / write_pass_predict() - AOP satellite data
- serialize_config() - Save to flash

### Parameter Access

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
- **NORMAL** - Default parameters
- **LOW_BATTERY** - LB_* parameters (when LB_EN=true and battery below LB_THRESHOLD)
- **OUT_OF_ZONE** - ZONE_* parameters (when outside geofencing zone)

## ConfigStoreFS

Location: `core/configuration/config_store_fs.hpp`

Flash-backed implementation using LittleFS:
- Serializes/deserializes BaseType variants to/from binary
- Handles each variant type via operator() overloads (serializer) and switch cases (deserializer)
- Stores AOP pass prediction data separately

## Calibration System

Location: `core/configuration/calibration.hpp`

### Calibratable Interface

```cpp
class Calibratable {
    virtual void calibration_write(const double value, const unsigned int offset) {}
    virtual void calibration_read(double& value, const unsigned int offset) {}
    virtual void calibration_save(bool force) {}
};
```

Devices inherit from Calibratable and register with CalibratableManager. Used for:
- Sea level pressure reference (pressure sensor offset 0)
- Accelerometer axis offsets
- SWS analog baselines

### DTE Interface

- **SCALW** - Write calibration: `$SCALW#00D;1,0,1013.25\r` (pressure, offset 0, value 1013.25)
- **SCALR** - Read calibration: `$SCALR#005;1,0\r` (pressure, offset 0)

### Calibration Storage

The Calibration helper class persists calibration values to LittleFS files, with a map<unsigned int, double> keyed by offset.
