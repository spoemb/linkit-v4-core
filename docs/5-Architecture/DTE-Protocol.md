# DTE Protocol

## Overview

The DTE (Data Terminal Equipment) protocol handles communication between the firmware and external tools (pylinkit, UART terminal). It provides parameter read/write, sensor access, log retrieval, and device management.

## Protocol Stack

```
pylinkit / UART Terminal
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

## Key Components

### DTEDecoder
Location: `core/protocol/dte_protocol.hpp`

Parses incoming DTE messages:
1. Extracts command name, payload length, payload
2. Validates length matches actual payload
3. Decodes arguments based on command prototype (BaseEncoding)
4. For PARMR/PARMW: resolves DTE keys to ParamIDs

### DTEEncoder
Location: `core/protocol/dte_protocol.hpp`

Formats outgoing DTE responses:
- OK responses: `$O;CMD#LEN;PAYLOAD\r`
- Error responses: `$N;CMD#LEN;ERROR_CODE\r`
- Encodes BaseType values to string using BaseEncoding rules

### DTEHandler
Location: `core/protocol/dte_handler.cpp`

Central dispatcher with handle_dte_message():
1. Calls DTEDecoder::decode() to parse the request
2. Switches on DTECommand enum to call the appropriate handler
3. Each handler returns a response string
4. Returns DTEAction for post-command processing (RESET, FACTR, CONFIG_UPDATED, etc.)

### BaseEncoding

Type system for parameter encoding/decoding:

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
| KEY_LIST | Comma-separated keys | vector<ParamID> |
| KEY_VALUE_LIST | key=value pairs | vector<ParamValue> |

## Argos Encoder/Decoder

For satellite transmission, a separate binary encoder/decoder packs GPS fixes and sensor data into compact Argos packets:

- **DTEEncoder** (for Argos TX): Bit-packs GPS fixes into depth-piled payloads
- **DTEDecoder** (for Argos RX): Decodes received satellite data

## Parameter Map

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
