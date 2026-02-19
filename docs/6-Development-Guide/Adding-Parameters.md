# Adding Parameters

Step-by-step guide for adding a new configurable parameter to the firmware. We use the real example of `PRESSURE_SENSOR_FULL_SCALE` (ParamID 177, DTE key PRP07) added to configure the LPS28DFW pressure sensor range.

## Overview

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

Add the new parameter to the `ParamID` enum. Choose the next available slot number.

```cpp
enum class ParamID {
    // ... existing params ...
#if ENABLE_PRESSURE_SENSOR
    PRESSURE_SENSOR_FULL_SCALE = 177,
#endif
    __PARAM_SIZE = 178,  // Increment this!
    __NULL_PARAM = 0xFFFF
};
```

**Rules:**
- Slots are always reserved even when `#if` guard is false (stable indexing)
- Use `#if ENABLE_*` guard if parameter is sensor-specific
- Increment `__PARAM_SIZE` by 1

### If you need a new enum type

Add the enum class:

```cpp
enum class BasePressureSensorFullScale {
    FS_1260 = 0,
    FS_4060,
};
```

Add it to `BaseEncoding`:

```cpp
enum class BaseEncoding {
    // ... existing ...
    PRESSURESENSORFULLSCALE,
};
```

Add it to the `BaseType` variant:

```cpp
using BaseType = std::variant<
    std::string, unsigned int, /* ... existing types ... */,
    BasePressureSensorFullScale  // Add at the end
>;
```

## Step 2: Add Parameter Map Entry (`core/protocol/dte_params.cpp`)

Add an entry to `param_map[]` at the position matching the ParamID slot:

```cpp
// [177] Pressure sensor full scale (slots always reserved)
{ "PRESSURE_SENSOR_FULL_SCALE", "PRP07", BaseEncoding::PRESSURESENSORFULLSCALE,
  0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
```

**Fields:**
- `name` - Human-readable name (matches config file)
- `key` - 5-char DTE key (3-letter prefix + 2-digit number)
- `encoding` - BaseEncoding type for wire format
- `min_value` / `max_value` - Range constraints (0,0 = no range check)
- `permitted_values` - Enum values (empty = use min/max range)
- `is_implemented` - Use ENABLE_* macro for conditional params
- `is_writable` - Can be written via PARMW

The `param_map` array size is checked at compile time with `static_assert`.

## Step 3: Add Encode/Decode (`core/protocol/dte_protocol.hpp`)

Only needed if you added a new enum type in Step 1. For standard types (UINT, FLOAT, BOOLEAN, TEXT), skip this step.

Add encoder:

```cpp
static inline void encode(std::string& output, const BasePressureSensorFullScale& value) {
    output += std::to_string((unsigned int)value);
}
```

Add decoder:

```cpp
static BasePressureSensorFullScale decode_pressure_sensor_full_scale(const std::string& s) {
    unsigned int v = std::stoul(s);
    switch (v) {
        case 0: return BasePressureSensorFullScale::FS_1260;
        case 1: return BasePressureSensorFullScale::FS_4060;
        default: throw ErrorCode::DTE_PROTOCOL_VALUE_OUT_OF_RANGE;
    }
}
```

Add the decoder switch case in the main decode function and the encoding fallthrough in the encode function.

## Step 4: Add Default Value (`core/configuration/config_store.hpp`)

Add the default to the `default_params` array at the correct position:

```cpp
static inline const std::array<BaseType,MAX_CONFIG_ITEMS> default_params { {
    // ... existing defaults ...
    /* [177] PRESSURE_SENSOR_FULL_SCALE */ BasePressureSensorFullScale::FS_1260,
}};
```

**Bump the config version** to force factory reset on existing devices:

```cpp
static inline const unsigned int m_config_version_code = 0x1c07e800 | 0x15;
//                                                   was 0x14 ────────────^
```

## Step 5: Add Serializer/Deserializer (`core/configuration/config_store_fs.hpp`)

Only needed if you added a new type to the `BaseType` variant. The `std::visit` in the serializer requires an `operator()` overload for every type in the variant.

Add serializer:

```cpp
void operator()(BasePressureSensorFullScale &s) {
    std::memcpy(entry_buffer, &s, sizeof(s));
}
```

Add deserializer case:

```cpp
case BaseEncoding::PRESSURESENSORFULLSCALE: {
    BasePressureSensorFullScale value = *(BasePressureSensorFullScale *)param_value;
    m_params.at(index) = value;
    break;
}
```

## Step 6: Update Tests (`tests/src/dte_handler_test.cpp`)

Update the PARML expected response string to include the new key:

```cpp
// Add ",PRP07" to the end of the key list
// Recalculate the #LEN hex value
```

Update the PARMR expected response string:

```cpp
// Add ",PRP07=0" to the end of the value list
// Recalculate the #LEN hex value
```

## Step 7: Use the Parameter

In a service, read the parameter:

```cpp
void sensor_init() override {
    unsigned int fs = (unsigned int)service_read_param<BasePressureSensorFullScale>(
        ParamID::PRESSURE_SENSOR_FULL_SCALE);
    m_sensor.set_full_scale(fs);
}
```

## Checklist

- [ ] ParamID added with correct slot number
- [ ] `__PARAM_SIZE` incremented
- [ ] New enum type added (if needed) to BaseEncoding, BaseType variant
- [ ] `param_map[]` entry added at correct position
- [ ] Encode/decode added (if new enum type)
- [ ] Default value added to `default_params[]`
- [ ] Config version bumped
- [ ] Serializer/deserializer updated (if new variant type)
- [ ] PARML test string updated
- [ ] PARMR test string updated
- [ ] All tests pass
