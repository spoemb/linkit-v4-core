# ports/nrf52840/core/

nRF52840-specific implementations of the abstractions in [`../../../core/`](../../../core/).

| Folder | Role |
|--------|------|
| [`hardware/`](hardware/) | Device drivers: nRF peripherals (`nrf_*`) and external sensors (`bma400`, `lps28dfw`, `m10qasync`, `lora_rak3172`, `smd_sat`, `kim2`, …) |
| [`interface/`](interface/) | External interfaces: BLE GATT, USB CDC, BLE STM OTA |
| [`memmang/`](memmang/) | Custom heap (FreeRTOS heap_4 derivative) and `new`/`delete` bindings |
| [`scheduling/`](scheduling/) | Hooks into [`../../../core/scheduling/`](../../../core/scheduling/) for the nRF target |
| [`filesystem/`](filesystem/) | LittleFS hookups (flash sectors, locks) |

All files in this tree may include nRF SDK headers (`nrf_gpio.h`, `nrf_libuarte_async.h`, `app_*.h`, …). Files in [`../../../core/`](../../../core/) must not.
