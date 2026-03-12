# 4 - Board Variants

All boards share the same nRF52840 MCU, u-blox M10Q GNSS, and SoftDevice S140 v7.2.0. They differ in their communication module, power management, and peripherals.

## Hardware Comparison

| | **LinkIt V4 KIM** | **LinkIt V4 SMD** | **LinkIt V4 LoRa** | **RSPB** |
|---|---|---|---|---|
| **Comm Module** | KIM2 (CLS) | SMD (Arribada) | RAK3172-SiP | SMD (Arribada) |
| **Comm Interface** | UART | SPI | UART | SPI |
| **Protocol** | Argos Legacy | Argos A+ | LoRaWAN 1.0.3 | Argos A+ |
| **Coverage** | Global (satellite) | Global (satellite) | Terrestrial (gateway) | Global (satellite) |
| **Battery Monitor** | Analog ADC | Analog ADC | Analog ADC | STC3117 fuel gauge |
| **Power-on** | Reed switch (3s) | Reed switch (3s) | Reed switch (3s) | TPL5111 timer |
| **Debug Log** | USB CDC | USB CDC | USB CDC | UART (J3) |
| **Thermistor** | Optional | Optional | Optional | Always enabled |
| **Wireless Charging** | Yes | Yes | Yes | No |
| **BSP** | `linkitv4_v1.0/` | `linkitv4_v1.0/` | `linkitv4_v1.0/` | `rspbtracker_v1.0/` |
| **Build Script** | `build_core.sh` | `build_linkitv4_smd.sh` | `build_linkitv4_lora.sh` | `build_rspb.sh` |
| **Build Dir** | `build/LINKIT/` | `build/LINKIT_SMD/` | `build/LINKIT_LORA/` | `build/RSPB/` |
| **CMake Key Flags** | `BOARD=LINKIT` | `BOARD=LINKIT ARGOS_SMD=ON` | `BOARD=LINKIT LORA_RAK3172=ON` | `BOARD=RSPB ARGOS_SMD=ON` |
| **Hardware Repo** | [kim-only](https://github.com/arribada/linkit-v4-hw/tree/kim-only) | [smd-only](https://github.com/arribada/linkit-v4-hw/tree/smd-only) | - | - |

For build and flash instructions, see [Building](https://github.com/arribada/linkit-v4-core/wiki/2-%E2%80%90-Building) and [Programming](https://github.com/arribada/linkit-v4-core/wiki/3-%E2%80%90-Programming).

---

## LinkIt V4 KIM

The default variant. Uses the CLS KIM2 module for Argos satellite communication over UART.

### Argos Packet Sizes

| Packet Type | Size |
|-------------|------|
| SHORT_PACKET | 12 bytes |
| LONG_PACKET | 31 bytes |

### Supported Argos Modes

| Mode | Value | Description |
|------|-------|-------------|
| OFF | 0 | Argos TX disabled |
| PASS_PREDICTION | 1 | TX only during predicted satellite passes (recommended) |
| LEGACY | 2 | Periodic TX regardless of satellite position |
| DUTY_CYCLE | 3 | TX according to 24-bit duty cycle bitmask |
| DOPPLER | 4 | Doppler-based location mode |

### KIM-Specific DTE Commands

The KIM module does not require credentials setup (unlike SMD).

| Command | Description |
|---------|-------------|
| SATTX | Manual Argos satellite transmission |
| SATDP | Start periodic Doppler TX calibration |
| PASPW | Write pass prediction (AOP) data |

### Bootloader

```bash
# Build
cd ports/nrf52840/bootloader/secure_bootloader/linkitv4_v1.0/armgcc
make mergehex
```

### Typical Configuration

```
PARMW,ARP01,2          # ARGOS_MODE = Legacy
PARMW,ARP05,60         # TR_NOM = 60s
PARMW,GNP01,1          # GNSS enabled
PARMW,GNP05,240        # GNSS timeout = 240s
PARMW,UNP01,1          # Underwater detection enabled
PARMW,UNP10,0          # Detection source = SWS
PARMW,AXP01,1          # Accelerometer enabled
```

---

## LinkIt V4 SMD

Uses the Arribada SMD module for Argos communication via SPI with the modern A+ protocol and built-in security.

### Argos Packet Sizes

| Packet Type | Size |
|-------------|------|
| SHORT_PACKET | 12 bytes |
| LONG_PACKET | 24 bytes (vs 31 for KIM) |

The SMD module uses shorter long packets than KIM and does not include a CRC field in the sensor packet.

### SMD Protocol (A+)

The SMD module uses a framed protocol with:
- Magic bytes for frame identification
- Sequence numbers for reliable delivery
- CRC for data integrity
- Built-in security key management (KMAC)
- Radio configuration commands

### SMD Credentials

Before the SMD module can transmit, credentials must be loaded:

| Parameter | DTE Key | Description |
|-----------|---------|-------------|
| ARGOS_SECKEY | IDP13 | Security key (KMAC) for A+ protocol |
| ARGOS_RADIOCONF | IDP14 | Radio configuration data |

```bash
# Via pylinkit
pylinkit --device xx:xx:xx:xx:xx:xx --smdcd --smdid <id> --smdaddr <addr> --smdseckey <key> --smdradioconf <conf>
```

### SMD-Specific DTE Commands

| Command | Description |
|---------|-------------|
| SATTX | Manual Argos satellite transmission |
| SATDP | Start periodic Doppler TX calibration |
| PASPW | Write pass prediction (AOP) data |
| SMDDFU | SMD module firmware update (DFU over SPI) |
| SMDTST | SMD SPI connectivity test |
| SMDCD | Write SMD credentials (security key + radio config) |

### SMD vs KIM

| Feature | SMD | KIM |
|---------|-----|-----|
| Interface | SPI | UART |
| Protocol | A+ (modern, with security) | Legacy |
| Long packet size | 24 bytes | 31 bytes |
| Security | Built-in KMAC | None |
| DFU support | Yes (SMDDFU) | No |
| Credentials | Required (SMDCD) | Not required |

### Bootloader

Same as KIM (shared `linkitv4_v1.0` BSP):
```bash
cd ports/nrf52840/bootloader/secure_bootloader/linkitv4_v1.0/armgcc
make mergehex
```

### Typical Configuration

```
PARMW,ARP01,2          # ARGOS_MODE = Legacy
PARMW,ARP05,60         # TR_NOM = 60s
PARMW,GNP01,1          # GNSS enabled
PARMW,GNP05,240        # GNSS timeout = 240s
PARMW,UNP01,1          # Underwater detection enabled
PARMW,UNP10,0          # Detection source = SWS
PARMW,AXP01,1          # Accelerometer enabled

# Set SMD credentials
PARMW,IDP13,<security_key>
PARMW,IDP14,<radio_config>
SMDCD
```

---

## LinkIt V4 LoRa

Replaces the Argos satellite module with a RAK3172-SiP LoRaWAN module (Semtech SX1262) for terrestrial LPWAN communication.

### LoRaWAN Specifications

| Feature | Details |
|---------|---------|
| **LoRaWAN Version** | 1.0.3 |
| **Join Modes** | OTAA (recommended) and ABP |
| **Default Band** | EU868 (configurable) |
| **Device Class** | A (default), B, C supported |
| **Low Power** | Standby (~1.7uA, ~10ms wake) or shutdown (0uA, ~2.5s wake) |

### Packet Types

| Type | Content | Max Size |
|------|---------|----------|
| GPS_SINGLE | Single GPS fix + battery | 51 bytes |
| GPS_MULTI | Multiple GPS fixes with delta-time compression | Up to 222 bytes |
| SENSOR | Sensor data (temperature, pressure, etc.) | Variable |
| STATUS | Device status, battery, counters | Variable |

Payload size depends on the data rate (DR). Higher DR = more bandwidth = larger payloads.

### LoRa Parameters

#### Network Configuration (OTAA - recommended)

| Parameter | DTE Key | Default | Description |
|-----------|---------|---------|-------------|
| LORA_DEVEUI | LRP01 | (from module) | Device EUI - read from RAK3172, read-only |
| LORA_APPEUI | LRP02 | "" | Application EUI (JoinEUI) - 16 hex chars |
| LORA_APPKEY | LRP03 | "" | Application Key - 32 hex chars |
| LORA_NJM | LRP07 | 1 | Network Join Mode: 0=ABP, 1=OTAA |

#### Network Configuration (ABP)

| Parameter | DTE Key | Default | Description |
|-----------|---------|---------|-------------|
| LORA_DEVADDR | LRP04 | "" | Device Address - 8 hex chars |
| LORA_APPSKEY | LRP05 | "" | Application Session Key - 32 hex chars |
| LORA_NWKSKEY | LRP06 | "" | Network Session Key - 32 hex chars |

#### Radio Configuration

| Parameter | DTE Key | Default | Description |
|-----------|---------|---------|-------------|
| LORA_BAND | LRP08 | 4 | Frequency band: 0=EU433, 1=CN470, 2=RU864, 3=IN865, 4=EU868, 5=US915, 6=AU915, 7=KR920, 8=AS923-1/2/3/4 |
| LORA_CLASS | LRP09 | 0 | Device class: 0=A, 1=B, 2=C |
| LORA_DR | LRP10 | 3 | Data rate (EU868: 0=SF12, 1=SF11, 2=SF10, 3=SF9, 4=SF8, 5=SF7) |
| LORA_ADR | LRP11 | false | Adaptive Data Rate (recommended OFF for mobile) |
| LORA_TXP | LRP12 | 0 | TX power index (0 = max) |
| LORA_CFM | LRP13 | false | Confirmed messages (ACK required) |
| LORA_FPORT | LRP14 | 2 | FPort for uplink (1-223) |
| LORA_LP_MODE | LRP15 | 1 | Low-power between TX: 0=shutdown, 1=standby |

### LoRa-Specific DTE Commands

| Command | Description |
|---------|-------------|
| LORATX | Manual LoRa TX (for testing) |
| GNSSBR | GNSS bridge mode (USB <-> u-blox UART passthrough) |

### Bridge Modes

**GNSS bridge** — direct communication with the u-blox M10Q for configuration via u-center or AssistNow loading:
```
GNSSBR    # Enter GNSS bridge mode
```

**LoRa bridge** — direct AT command access to the RAK3172 (RUI3 AT commands):
```
LORATX    # Enter LoRa bridge mode (send +++ to exit)
```

### LoRa vs Argos

| Feature | LoRa (RAK3172) | Argos (KIM/SMD) |
|---------|----------------|-----------------|
| Coverage | Terrestrial (gateway required) | Global (satellite) |
| Range | Up to 15km (line of sight) | Global |
| Data rate | 300bps - 11kbps | Fixed (low) |
| Power | Very low (~1.7uA standby) | Higher (satellite TX power) |
| Cost | Free (LoRaWAN) | Argos subscription |
| Latency | Near real-time | Hours (satellite pass) |
| Bidirectional | Yes (Class A/B/C) | Limited (RX windows) |
| Best for | Coastal, urban, farm | Open ocean, remote areas |

### Bootloader

Same as KIM/SMD (shared `linkitv4_v1.0` BSP):
```bash
cd ports/nrf52840/bootloader/secure_bootloader/linkitv4_v1.0/armgcc
make mergehex
```

### Typical Configuration

```
# OTAA Configuration
PARMW,LRP02,<appeui_16hex>     # Application EUI
PARMW,LRP03,<appkey_32hex>     # Application Key
PARMW,LRP07,1                  # OTAA mode
PARMW,LRP08,4                  # EU868 band
PARMW,LRP10,3                  # DR3 (SF9)
PARMW,LRP15,1                  # Standby low-power mode

# General tracker config
PARMW,GNP01,1                  # GNSS enabled
PARMW,GNP05,240                # GNSS timeout = 240s
PARMW,UNP01,1                  # Underwater detection enabled
PARMW,AXP01,1                  # Accelerometer enabled
```

---

## RSPB

The RSPB (Royal Society for the Protection of Birds) board is designed for bird tracking. It uses the SMD module (like LinkIt V4 SMD) but with a completely different power management approach.

### Key Differences vs LinkIt V4

| Feature | RSPB | LinkIt V4 |
|---------|------|-----------|
| Power-on | TPL5111 external timer | Reed switch (magnet 3s hold) |
| Debug log | UART via **J3** connector | USB CDC |
| Battery monitor | STC3117 fuel gauge | Analog ADC |
| Thermistor | Always enabled | Optional |
| Wireless charging | No | Yes |
| Boot logic | Counter modulo filtering | Reed switch gate |
| LoRa support | No | Yes (with `LORA_RAK3172=ON`) |

### Boot Behavior

The RSPB uses a TPL5111 external timer for ultra-low-power duty cycling:

1. TPL5111 wakes the device periodically (every `WAKEUP_PERIOD` seconds, default 6300s = 1h45)
2. A **boot counter** increments at each wakeup (`PWP02`)
3. The firmware checks `boot_counter % BOOT_COUNTER_MODULO` (`PWP03`)
   - Result **!= 0**: device powers down immediately (not its turn)
   - Result **== 0**: device runs its full operational cycle
4. Applying a magnet during a modulo-skip overrides the skip (allows configuration access)

**Effective duty cycle** = `WAKEUP_PERIOD` x `BOOT_COUNTER_MODULO` (default: 6300s x 2 = **3.5 hours**).

### Pseudo-RTC

The RSPB implements a pseudo-RTC since it may lack a battery-backed RTC:
- At each boot, `LAST_KNOWN_RTC` (`PWP06`) is advanced by `WAKEUP_PERIOD`
- This gives an approximate time immediately, before a GNSS fix corrects it
- The value is persisted to flash

### Shutdown Timer

`SHUTDOWN_TIMER` (`PWP01`) controls max awake time per cycle. After this time, the device powers down. Set to 0 to disable (stays awake until task completion).

### RSPB-Specific Parameters

| Parameter | DTE Key | Default | Description |
|-----------|---------|---------|-------------|
| SHUTDOWN_TIMER | PWP01 | 0 | Max awake time per cycle in seconds (0 = no limit) |
| BOOT_COUNTER | PWP02 | 0 | Current boot counter (read-only) |
| BOOT_COUNTER_MODULO | PWP03 | 2 | Run every Nth boot |
| WAKEUP_PERIOD | PWP04 | 6300 | TPL5111 wakeup interval in seconds (read-only, hardware) |
| SHUTDOWN_NTIME_SAT | PWP05 | 0 | Number of Argos TX before auto-shutdown (0 = disabled) |
| LAST_KNOWN_RTC | PWP06 | 0 | Persisted RTC for pseudo-RTC chain (read-only) |

### SMD Credentials

Same as LinkIt V4 SMD — the SMD module requires credentials before it can transmit. See [SMD Credentials](#smd-credentials) above.

### Bootloader

The RSPB uses its own bootloader:
```bash
cd ports/nrf52840/bootloader/secure_bootloader/rspbtracker_v1.0/armgcc
make mergehex
```

### Typical Configuration

```
PARMW,ARP01,1          # ARGOS_MODE = Pass Prediction
PARMW,ARP05,60         # TR_NOM = 60s repetition
PARMW,GNP01,1          # GNSS enabled
PARMW,GNP05,120        # GNSS acquisition timeout = 120s
PARMW,PWP03,2          # Run every 2nd boot (effective period = 3.5h)
PARMW,PWP01,3600       # Stay awake max 1 hour per cycle
```
