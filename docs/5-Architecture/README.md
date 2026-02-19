# 5 - Firmware Architecture

## Layered Architecture

The firmware follows a layered architecture with clear separation of concerns:

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
│    Board Support Package        │  gentracker_v1.0, rspbtracker_v1.0
├─────────────────────────────────┤
│     nRF52840 + SoftDevice       │  Nordic SDK 17.1.0
└─────────────────────────────────┘
```

## Directory Organization

- `core/` - Portable code, no hardware dependencies
  - `core/services/` - Service implementations
  - `core/protocol/` - DTE protocol, encoder, decoder, params
  - `core/configuration/` - ConfigStore, calibration
  - `core/hardware/` - Abstract interfaces (Sensor, Timer)
  - `core/logging/` - Logger, messages
- `ports/nrf52840/` - Hardware-specific implementations
  - `ports/nrf52840/bsp/` - Board support packages
  - `ports/nrf52840/core/hardware/` - Sensor drivers (BMA400, LPS28DFW, M8, etc.)

## Key Design Patterns

- **Dependency Inversion**: `core/` defines abstract interfaces, `ports/` provides implementations
- **RAII**: SensorsPowerGuard for VSENSORS power management
- **Static Registry**: SensorManager, LoggerManager, CalibratableManager, ServiceManager
- **Variant-based Configuration**: BaseType = std::variant<...> for type-safe parameter storage
- **Event-driven Services**: ServiceEvent for inter-service communication

## Sections

- [Service Framework](Service-Framework.md)
- [Services Reference](Services-Reference.md)
- [Configuration System](Configuration-System.md)
- [DTE Protocol](DTE-Protocol.md)
- [Hardware Abstraction](Hardware-Abstraction.md)
- [Logging System](Logging-System.md)
- [Message System](Message-System.md)
