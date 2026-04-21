# ports/nrf52840/bootloader/

Nordic Secure Bootloader configurations for DFU (Device Firmware Update) over BLE and USB.

| Folder | Board |
|--------|-------|
| `secure_bootloader/linkitv4_v1.0/` | LinkIt V4 (KIM / SMD / LoRa) |
| `secure_bootloader/rspbtracker_v1.0/` | RSPB Tracker |

Each contains a `config/sdk_config.h` tuned for that board (DFU transports, key validation, NVMC layout). The actual bootloader source comes from the Nordic SDK at [`../drivers/nRF5_SDK_17.0.2/examples/dfu/`](../drivers/nRF5_SDK_17.0.2/examples/dfu/).

To build a flashable hex with bootloader + SoftDevice + app, use:

```bash
./scripts/build_with_bootloader.sh <target> --clean --recover
```

See the [Programming wiki page](https://github.com/arribada/linkit-v4-core/wiki/04-%E2%80%90-Programming) for DFU usage from the host side.
