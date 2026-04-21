# core/filesystem/

File abstraction layer and OTA file updater.

## Files

| File | Role |
|------|------|
| `filesystem.hpp` | `FileSystem` interface (open/read/write/close, mkdir, list) |
| `ota_file_updater.hpp` | OTA update interface |
| `ota_flash_file_updater.{cpp,hpp}` | Flash-backed OTA file writer used during DFU |

The concrete LittleFS-backed implementation lives in [`../../ports/nrf52840/core/filesystem/`](../../ports/nrf52840/core/filesystem/).

LittleFS itself is vendored under [`../../libraries/littlefs/`](../../libraries/littlefs/).
