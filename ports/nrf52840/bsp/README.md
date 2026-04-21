# ports/nrf52840/bsp/

Board Support Packages — pin maps and Nordic SDK config per board variant.

| Variant | Folder | Use |
|---------|--------|-----|
| LinkIt V4 v1.0 | `linkitv4_v1.0/` | LinkIt V4 KIM / SMD / LoRa |
| RSPB Tracker v1.0 | `rspbtracker_v1.0/` | RSPB bird mortality tracker |

Each variant contains:
- `bsp.hpp` / `bsp.cpp` — pin definitions, peripheral instance configs (`UARTAsync_Inits`, SPI, I2C, RTC, etc.)
- `sdk_config.h` — Nordic SDK feature toggles for that board

The active BSP is selected at build time via `-DBOARD=LINKIT` or `-DBOARD=RSPB`.

## Adding a new board variant

1. Copy an existing variant folder, rename, and update pin assignments in `bsp.hpp`.
2. Add a CMake branch for the new board in [`../CMakeLists.txt`](../CMakeLists.txt).
3. Add a build script under [`../../../scripts/`](../../../scripts/).
