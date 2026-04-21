# core/configuration/

Persistent configuration storage and parameter definitions.

## Files

| File | Role |
|------|------|
| `config_store.hpp` | `ConfigurationStore` interface + `default_params` array |
| `config_store_fs.hpp` | LittleFS-backed implementation |
| `calibration.{hpp,cpp}` | Sensor calibration data (separate from DTE params) |

The actual parameter map (DTE 5-char keys, types, min/max, default) lives in [`../protocol/dte_params.cpp`](../protocol/dte_params.cpp) — they are tightly coupled.

## Adding a new parameter

1. Add an entry to the `ParamID` enum in `config_store.hpp` (right before `__PARAM_SIZE`).
2. Add the matching `BaseMap` entry in [`../protocol/dte_params.cpp`](../protocol/dte_params.cpp) `param_map[]` (DTE key, encoding, min/max, default).
3. Bump the config version in `config_store.hpp` (`config_version_compatibility`) so old flash configs are migrated/rejected safely.
4. For conditional params, guard with `#if defined(FEATURE)` in the enum and `is_implemented=(FEATURE == 1)` in the map.

A `static_assert` on `__PARAM_SIZE == sizeof(param_map)/sizeof(BaseMap)` catches mismatches at build time.
