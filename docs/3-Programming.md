# 3 - Programming the Device

This page covers how to program the two programmable targets on the LinkIt V4 board:

| Target | Connector | Probe | Methods |
|--------|-----------|-------|---------|
| **LinkIt Core** (nRF52840) | **J1** | TC2030 pogo-pin | J-Link (SWD) or DFU (BLE) |
| **SMD Module** (Argos/Kineis) | **J12** | TC2030 pogo-pin | J-Link (SWD) or DFU (SPI/UART) |

The SMD module is only present on boards built with `ARGOS_SMD=ON` (LinkIt V4 SMD, RSPB).

---

## 1 - Programming the LinkIt Core (nRF52840)

### 1.1 - Via J-Link Probe (J1)

Connect a J-Link debug probe to the **J1** TC2030 pogo-pin connector on the board.

#### Merged Hex (Recommended for First Flash)

The merged hex contains **bootloader + SoftDevice + application** in a single file:

```bash
nrfjprog --recover
nrfjprog -f nrf52 --program <merged_hex_file> --sectorerase
nrfjprog -f nrf52 --reset
```

| Board | Build Dir | Merged Hex File |
|-------|-----------|-----------------|
| KIM | `build/LINKIT/` | `LinkIt_board_merged-*.hex` |
| SMD | `build/LINKIT_SMD/` | `LinkIt_board_merged-*.hex` |
| LoRa | `build/LINKIT_LORA/` | `LinkIt_board_merged-*.hex` |
| RSPB | `build/RSPB/` | `LinkIt_RSPB_board_merged-*.hex` |

All paths are relative to `ports/nrf52840/`.

> **Note:** The merged hex is only generated if the bootloader has been built first. See [Quick Start - Build the Bootloader](1-Quick-Start.md#4-build-the-bootloader-first-time-only).

#### App-Only Flash (Development)

When the bootloader and SoftDevice are already on the device, flash only the application for faster iteration:

```bash
nrfjprog -f nrf52 --program <app_hex_file> --sectorerase
nrfjprog -f nrf52 --reset
```

#### Flash Without Bootloader (Debug)

For debugging without the bootloader (SoftDevice + application only):

```bash
nrfjprog --recover
nrfjprog -f nrf52 --program ports/nrf52840/drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --sectorerase
nrfjprog -f nrf52 --program <app_hex_file> --sectorerase
nrfjprog -f nrf52 --reset
```

### 1.2 - Via DFU over BLE (No J-Link Required)

The firmware can be updated over Bluetooth using **pylinkit** (CLI) or the **LinkIt GUI tool**. The bootloader must already be on the device.

#### Prerequisites

The device must be powered on and advertising via BLE. Scan for available devices first:

```bash
pylinkit --scan
```

This will list all nearby LinkIt devices with their BLE addresses (e.g., `xx:xx:xx:xx:xx:xx`).

#### Using pylinkit (CLI)

```bash
pylinkit --device xx:xx:xx:xx:xx:xx --fw firmware.zip
```

The `.zip` DFU package is generated automatically by the build scripts:

| Board | DFU Package |
|-------|-------------|
| KIM | `build/LINKIT/LinkIt_board_dfu-*.zip` |
| SMD | `build/LINKIT_SMD/LinkIt_board_dfu-*.zip` |
| LoRa | `build/LINKIT_LORA/LinkIt_board_dfu-*.zip` |
| RSPB | `build/RSPB/LinkIt_RSPB_board_dfu-*.zip` |

> **Warning:** The firmware update takes several minutes. Do not reset the device or disconnect during the transfer. Ensure the battery is not critically low.

#### Using the LinkIt GUI Tool

The graphical interface provides the same firmware update functionality with a visual progress indicator. Select the DFU package (`.zip`) and the target device from the GUI.

---

## 2 - Programming the SMD Module (Argos/Kineis)

The SMD satellite module has its own MCU and firmware. It can be updated independently from the LinkIt Core, either via the **J12** debug connector or via DFU over SPI/UART.

### 2.1 - Via J-Link Probe (J12)

Connect a J-Link debug probe to the **J12** TC2030 pogo-pin connector (dedicated to the SMD module).

**Important:** The SMD module must be powered on before flashing. The nRF52840 controls power to the satellite module, so the LinkIt Core must be running and the satellite power rail must be enabled.

Power on the satellite module first:

```bash
# Via pylinkit (BLE)
pylinkit --device xx:xx:xx:xx:xx:xx --pwron satellite

# Via DTE command (serial terminal)
$PWRON#001;3
```

Once the satellite power rail is active, flash the SMD firmware with the J-Link probe connected to J12:

```bash
# Flash SMD firmware via J12
nrfjprog --program <smd_firmware.hex> --sectorerase
nrfjprog --reset
```

> **Note:** The J12 probe targets the SMD module's MCU, not the nRF52840. Make sure the correct J-Link target is selected.

### 2.2 - Via DFU (No J-Link Required)

The SMD firmware can also be updated through the LinkIt Core, which forwards the firmware to the SMD module over SPI or UART. This does not require a J-Link probe.

#### Using pylinkit (CLI)

The device must be powered on and discoverable via BLE (see `pylinkit --scan`).

```bash
# DFU via SPI (recommended)
pylinkit --device xx:xx:xx:xx:xx:xx --smdfw <smd_firmware.bin> --smdfw_mode spi

# DFU via UART
pylinkit --device xx:xx:xx:xx:xx:xx --smdfw <smd_firmware.bin> --smdfw_mode uart
```

The SPI DFU process can take up to 3 minutes. pylinkit will wait for the device to confirm completion.

#### Using the LinkIt GUI Tool

The graphical interface supports SMD firmware updates with visual progress tracking. Select the SMD firmware binary and the DFU mode (SPI or UART).

#### Using DTE Commands (Manual)

For manual control of the SMD DFU process via a serial terminal:

| Command | Description |
|---------|-------------|
| `$SMDDFU#001;5` | Check SMD firmware version |
| `$SMDDFU#001;0` | Enter DFU mode |
| `$SMDDFU#001;2` | Check DFU status |
| `$SMDDFU#001;4` | Get bootloader info |
| `$SMDDFU#001;1` | Exit DFU mode |

---

## 3 - Debug Output (Log)

The debug output is a **read-only log stream** (trace, warnings, errors). It is separate from the DTE command interface.

### Output Modes

The log output interface is configured via `DEBUG_OUTPUT_MODE` (DTE key: `DBP01`):

| Value | Mode | Interface | Default For |
|-------|------|-----------|-------------|
| 0 | UART | **J3** connector (UART, requires USB-to-UART adapter) | RSPB |
| 1 | USB_CDC | USB connector (virtual serial port) | LinkIt V4 (KIM, SMD, LoRa) |
| 2 | BLE_NUS | Bluetooth Low Energy Nordic UART Service | - |

### Connecting to Debug Log

**LinkIt V4** (KIM, SMD, LoRa) — USB CDC (default):

Connect the USB cable and open a serial terminal at **115200 baud, 8N1**:
```bash
minicom -D /dev/ttyACM0 -b 115200
# or
screen /dev/ttyACM0 115200
```

**RSPB** — UART via J3 connector:

The RSPB has no USB interface for debug. Connect a **USB-to-UART adapter** to the **J3** header, then use the same serial terminal commands on the adapter's port (e.g., `/dev/ttyUSB0`).

**BLE NUS** (all boards):

Set `DBP01=2` first, then use the nRF Connect app (iOS/Android) or any BLE terminal supporting the Nordic UART Service.

**pylinkit log viewer** (BLE):
```bash
pylinkit --device xx:xx:xx:xx:xx:xx --log
```

### Debug Levels

Set at **compile time** via `DEBUG_LEVEL` in CMake:

| Level | Output |
|-------|--------|
| 0 | Off |
| 1 | Errors only |
| 2 | Errors + Warnings |
| 3 | Errors + Warnings + Info |
| 4 | Errors + Warnings + Info + Trace |

```bash
cmake ... -DDEBUG_LEVEL=4 ...
```

---

## 4 - DTE Communication (Commands)

The DTE (Data Terminal Equipment) protocol is used for **device configuration and control** (read/write parameters, trigger actions, firmware updates). DTE commands are sent over **BLE** using pylinkit or the LinkIt GUI tool.

### Using pylinkit (CLI)

The device must be powered on and discoverable via BLE:

```bash
# Scan for devices
pylinkit --scan

# Read all parameters to a file
pylinkit --device xx:xx:xx:xx:xx:xx --parmr params.cfg

# Write parameters from a file
pylinkit --device xx:xx:xx:xx:xx:xx --parmw params.cfg

# Factory reset
pylinkit --device xx:xx:xx:xx:xx:xx --factw

# Software reset
pylinkit --device xx:xx:xx:xx:xx:xx --rstbw
```

### Using the LinkIt GUI Tool

The graphical interface provides the same DTE functionality: parameter read/write, device control, log viewing, and firmware updates.

### DTE Protocol Reference

DTE commands follow this format:

```
$COMMAND#LEN;ARGS\r
```

Example commands:

```
$PARML#000;            # List all parameter keys
$PARMR#000;            # Read all parameters
$PARMR#00A;GNP01,ARP01 # Read specific parameters
$PARMW#010;GNP01=1    # Write a parameter
$PWRON#001;3           # Power on satellite module
```

See [DTE Commands Reference](https://github.com/arribada/linkit-v4-core/wiki/5-%E2%80%90-DTE%E2%80%90Commands) for the full protocol specification and command set.

---

## 5 - LED Modes

| Parameter | DTE Key | Description |
|-----------|---------|-------------|
| `LED_MODE` | `LDP01` | Internal LED behavior |
| `EXT_LED_MODE` | `LDP02` | External LED behavior |

| Value | Mode | Behavior |
|-------|------|----------|
| 0 | OFF | LED disabled |
| 1 | HRS_24 | LED active for the first 24 hours after boot |
| 3 | ALWAYS | LED always active |

---

## 6 - Power Management

### LinkIt V4 (KIM, SMD, LoRa)

Boot by holding a magnet against the reed switch for ~3 seconds. The device enters deep sleep when idle and wakes on accelerometer activity or scheduled events.

### RSPB

The RSPB uses a **TPL5111 external timer** for ultra-low-power duty cycling (0 current draw between cycles).

| Parameter | DTE Key | Description |
|-----------|---------|-------------|
| `SHUTDOWN_TIMER` | `PWP01` | Max awake time per cycle in seconds (0 = no limit) |
| `BOOT_COUNTER` | `PWP02` | Current boot count (read-only) |
| `BOOT_COUNTER_MODULO` | `PWP03` | Run every Nth boot (min: 2) |
| `WAKEUP_PERIOD` | `PWP04` | TPL5111 wakeup interval in seconds (read-only, hardware) |

**Effective duty cycle** = `WAKEUP_PERIOD` x `BOOT_COUNTER_MODULO`. Default: 6300s x 2 = **3.5 hours**.

The device runs when `BOOT_COUNTER % BOOT_COUNTER_MODULO == 0`, and powers down immediately on other boots. Applying a magnet during a modulo-skip overrides the skip (useful for configuration access).

---

## 7 - Troubleshooting

### Device Not Responding

1. **Check power**: Reed switch engaged (LinkIt V4) or TPL5111 cycling (RSPB)?
2. **Check BLE**: `pylinkit --scan` — if the device does not appear, it may be in deep sleep or not powered
3. **Recover via J-Link**: `nrfjprog --recover` erases and unlocks the chip

### Flash Verification Fails

```bash
nrfjprog --recover
nrfjprog -f nrf52 --program <merged_hex_file> --sectorerase
nrfjprog -f nrf52 --reset
```

`--recover` performs a full erase including UICR and flash protection bits.

### DFU Not Working

- Ensure the bootloader is on the device (flash a merged hex first via J-Link)
- Check that the DFU package (`.zip`) matches the board variant
- Verify the device is discoverable: `pylinkit --scan`

### SMD Module Not Responding

- Ensure satellite power is on: `pylinkit --device xx:xx:xx:xx:xx:xx --pwron satellite` or `$PWRON#001;3`
- Run SMD SPI test: `pylinkit --device xx:xx:xx:xx:xx:xx --smdtst` or `$SMDTST#000;`
- Check SMD firmware version: `pylinkit --device xx:xx:xx:xx:xx:xx --smddfu version` or `$SMDDFU#001;5`

### No Debug Output

- Verify `DEBUG_LEVEL` > 0 at compile time
- Check `DBP01` matches your interface (1 = USB CDC for LinkIt V4, 0 = UART/J3 for RSPB)
- For RSPB: debug log is on the **J3** UART connector, not USB — requires a USB-to-UART adapter

---

## Next Steps

- [Board Variants](https://github.com/arribada/linkit-v4-core/wiki/4-%E2%80%90-Boards) — Hardware specs, parameters, and configuration per board
- [DTE Commands Reference](https://github.com/arribada/linkit-v4-core/wiki/5-%E2%80%90-DTE%E2%80%90Commands) — Full protocol and command documentation
- [Architecture Overview](https://github.com/arribada/linkit-v4-core/wiki/6-%E2%80%90-Architecture) — Firmware design and service model
- [Application Behavior](https://github.com/arribada/linkit-v4-core/wiki/7-%E2%80%90-Behavior) — Underwater mode, RSPB duty cycling, SWS algorithm