# ports/

Platform-specific implementations of the abstractions defined in [`../core/hardware/`](../core/hardware/).

| Folder | Target |
|--------|--------|
| [`nrf52840/`](nrf52840/) | nRF52840 embedded target (production firmware) |
| [`linux/`](linux/) | Host (Linux) target — used by the unit test suite and for development |

Choose the port via the build script ([`../scripts/`](../scripts/)) or CMake `-DPLATFORM=…` in the per-port `CMakeLists.txt`.
