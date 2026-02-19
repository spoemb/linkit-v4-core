# 3 - Programming the Module

## Flashing with J-Link

### Prerequisites

- J-Link debug probe connected to the target SWD port
- `nrfjprog` (Nordic command-line tools) installed
- SoftDevice S140 v7.2.0 hex file

### First-Time Setup

The SoftDevice must be flashed once before the application:

```bash
# Erase chip and flash SoftDevice
nrfjprog --program s140_nrf52_7.2.0_softdevice.hex --chiperase --verify
nrfjprog --reset
```

### Flashing the Application

```bash
# Flash application (preserves SoftDevice)
nrfjprog --program linkit_v4.hex --sectorerase --verify
nrfjprog --reset
```

### Full Erase and Reflash

```bash
nrfjprog --eraseall
nrfjprog --program s140_nrf52_7.2.0_softdevice.hex --verify
nrfjprog --program linkit_v4.hex --verify
nrfjprog --reset
```

## Debug Output

Debug output is configured via the `DEBUG_OUTPUT_MODE` parameter (DTE key: `DBP01`):

| Value | Mode | Description |
|-------|------|-------------|
| 0 | `UART` | UART debug output on SWO pin (P0.11 for RSPB) |
| 1 | `USB_CDC` | USB CDC debug output (default for LinKit v4) |
| 2 | `BLE_NUS` | Bluetooth Low Energy Nordic UART Service |

To change the debug output mode:

```
$PARMW#00A;DBP01=0
```

### Debug Levels

The debug level is set at compile time via `DEBUG_LEVEL`:

| Level | Output |
|-------|--------|
| 0 | Off |
| 1 | Errors only |
| 2 | Errors + Warnings |
| 3 | Errors + Warnings + Info + Trace |

## DFU (Device Firmware Update)

### SPI DFU (SMD Module)

For RSPB boards with SMD satellite module (`ARGOS_SMD=ON`), the SMD firmware can be updated via SPI DFU:

```
# Check SMD firmware version
$SMDDFU#001;5

# Enter DFU mode
$SMDDFU#001;0

# Check DFU status
$SMDDFU#001;2

# Get bootloader info
$SMDDFU#001;4

# Exit DFU mode
$SMDDFU#001;1
```

## LED Modes

Two LED parameters control visual feedback:

| Parameter | DTE Key | Description |
|-----------|---------|-------------|
| `LED_MODE` | `LDP01` | Internal LED behavior |
| `EXT_LED_MODE` | `LDP02` | External LED behavior |

| Value | Mode | Behavior |
|-------|------|----------|
| 0 | OFF | LED disabled |
| 1 | HRS_24 | LED active for first 24 hours after boot |
| 3 | ALWAYS | LED always active |

## Power Management

### GenTracker

The GenTracker uses a pseudo power-off mode. It requires reed switch engagement to boot (`POWER_ON_RESET_REQUIRES_REED_SWITCH`).

### RSPB Tracker

The RSPB uses a TPL5111 external wakeup timer for periodic power cycling:

| Parameter | DTE Key | Description |
|-----------|---------|-------------|
| `SHUTDOWN_TIMER` | `PWP01` | Shutdown duration in seconds |
| `BOOT_COUNTER` | `PWP02` | Current boot count (read-only) |
| `BOOT_COUNTER_MODULO` | `PWP03` | Run every N boots (min: 2) |
| `WAKEUP_PERIOD` | `PWP04` | Wakeup period in seconds (read-only, set by hardware) |

The device runs on `BOOT_COUNTER % BOOT_COUNTER_MODULO == 0` and goes back to sleep otherwise. This allows running every Nth wakeup to save power.

## UART Communication

Default UART settings: **115200 baud, 8N1**

The DTE protocol uses framed messages. See [DTE Commands](4-AT-Commands/README.md) for the full protocol specification.
