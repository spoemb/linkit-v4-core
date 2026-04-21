# libraries/

Vendored third-party libraries. Treat as read-only — do not modify in place; if a patch is unavoidable, document it here so an SDK update does not silently revert it.

| Folder | What | Why |
|--------|------|-----|
| `etl-20.27.3/` | [Embedded Template Library](https://www.etlcpp.com/) | STL-like containers without dynamic allocation |
| `littlefs/` | [LittleFS](https://github.com/littlefs-project/littlefs) | Power-loss resilient flash filesystem |
| `prepass/` | CLS Prepass / Previpass package | Argos satellite pass prediction |
| `inplace_function/` | `stdext::inplace_function` | Heap-free `std::function` alternative |
| `CmBacktrace/` | [CmBacktrace](https://github.com/armink/CmBacktrace) | ARM Cortex-M crash backtrace |

The Nordic nRF5 SDK is **not** here — it lives under [`../ports/nrf52840/drivers/nRF5_SDK_17.0.2/`](../ports/nrf52840/drivers/nRF5_SDK_17.0.2/) because it is heavily entangled with the nRF port build.
