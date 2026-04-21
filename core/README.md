# core/

Platform-independent code. Anything in here **must not** include nRF SDK or Linux-specific headers — hardware access goes through abstract interfaces in [`core/hardware/`](hardware/), and concrete implementations live in [`../ports/`](../ports/).

## Layout

| Folder | Purpose |
|--------|---------|
| [`configuration/`](configuration/) | Persistent param store (LittleFS), DTE param map, calibration |
| [`services/`](services/) | Long-running services: GPS, Argos/LoRa TX, RX, SWS, mortality, depth pile |
| [`protocol/`](protocol/) | DTE protocol parser/serializer (PARMR/PARMW/CMDR) |
| [`sm/`](sm/) | Top-level state machines: gentracker, buzzm, ledsm (built on tinyfsm) |
| [`scheduling/`](scheduling/) | Cooperative scheduler + Argos/LoRa TX schedulers |
| [`hardware/`](hardware/) | Pure-virtual hardware interfaces (GPIO, PMU, GPS, battery, …) |
| [`ble_services/`](ble_services/) | BLE GATT service interface (impls in `ports/nrf52840/core/interface/`) |
| [`filesystem/`](filesystem/) | LittleFS-backed file abstraction + OTA file updater |
| [`logging/`](logging/) | Debug macros, console + flash logging |
| [`util/`](util/) | Pure helpers: BCH, CRC, base64, binascii, bitpack, haversine |

## Where do I add my code?

- **A new sensor or peripheral abstraction** → interface in [`hardware/`](hardware/), driver in [`../ports/nrf52840/core/hardware/`](../ports/nrf52840/core/hardware/).
- **A new long-running behavior** (GPS, scheduler-driven) → service in [`services/`](services/).
- **A new DTE command or parameter** → [`protocol/`](protocol/) and [`configuration/`](configuration/).
- **A new top-level mode/state** → state in [`sm/gentracker.cpp`](sm/gentracker.cpp).
