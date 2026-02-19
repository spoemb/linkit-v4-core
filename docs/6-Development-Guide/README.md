# 6 - Development Guide

This guide covers common development tasks for extending the LinKit v4 firmware.

## Contents

- [Adding Parameters](Adding-Parameters.md) - Add a new configurable parameter
- [Adding Services](Adding-Services.md) - Create a new service
- [Adding Sensors](Adding-Sensors.md) - Integrate a new hardware sensor
- [Testing](Testing.md) - Unit testing with CppUTest
- [Board Porting](Board-Porting.md) - Port to a new hardware platform

## Code Conventions

- C++20 standard
- Tabs for indentation
- `#pragma once` for header guards
- Enum classes for type safety
- `DEBUG_TRACE()`, `DEBUG_INFO()`, `DEBUG_WARN()`, `DEBUG_ERROR()` for debug output
- Packed structs (`__attribute__((packed))`) for log entries and wire formats
- `ErrorCode` exceptions for error handling (not std::exception)
- Static registries (SensorManager, ServiceManager, LoggerManager, CalibratableManager) for component lookup

## Key Source Files

| File | Purpose |
|------|---------|
| `core/protocol/base_types.hpp` | ParamID enum, BaseEncoding, BaseType variant, enums |
| `core/protocol/dte_params.cpp` | Parameter map (name, key, type, range, flags) |
| `core/protocol/dte_protocol.hpp` | Encoder/decoder for DTE protocol |
| `core/configuration/config_store.hpp` | Default values, config version, read/write API |
| `core/configuration/config_store_fs.hpp` | Flash serializer/deserializer |
| `core/services/service.hpp` | Service base class |
| `core/services/sensor_service.hpp` | SensorService base class |
| `core/hardware/sensor.hpp` | Sensor abstract interface |
| `core/logging/messages.hpp` | Log entry structures |
| `tests/src/dte_handler_test.cpp` | DTE protocol tests |
