How to activate, configure, and deploy a LinkIt V4 / RSPB tracker.

---

# What the Tracker Does

The LinkIt V4 is a GPS/satellite wildlife tracker. Once deployed, it autonomously:

1. Acquires GPS positions at configurable intervals
2. Transmits positions via Argos satellite (or LoRaWAN) — no cellular coverage needed
3. Detects underwater/surface transitions (marine models) and adapts behavior
4. Manages battery life with automatic low-power modes

Data is received through the Argos/Kineis satellite network and made available on the CLS/Kineis web platform.

---

# Board Variants at a Glance

| Tracker | Use Case | Communication | Power Control |
|---------|----------|---------------|---------------|
| **LinkIt V4 SMD** | Marine (turtles, seals) | Argos satellite (SMD) | Reed switch on/off |
| **LinkIt V4 KIM** | Marine / terrestrial | Argos satellite (KIM2) | Reed switch on/off |
| **LinkIt V4 LoRa** | Terrestrial (gateway range) | LoRaWAN | Reed switch on/off |
| **RSPB** | Birds | Argos satellite (SMD) | TPL5111 auto-wakeup |

---

# Power On / Off (LinkIt V4)

The tracker uses a **reed switch** activated by a magnet. All actions require a **confirmation gesture**: hold, release, then re-engage the magnet within 2 seconds.

## Power On (from Off state)

1. Place the magnet on the reed switch area
2. Hold for **3 seconds** — LED blinks **blue rapidly**
3. Release the magnet
4. Re-engage the magnet **within 2 seconds** to confirm
5. The tracker boots (white flashing LED), then enters operational mode

## Enter Configuration Mode (from Operational state)

1. Place the magnet — LED turns white (magnet detected)
2. Hold for **3 seconds** — LED blinks **blue rapidly** (config confirmation)
3. Release the magnet
4. Re-engage **within 2 seconds** to confirm
5. LED turns blue — BLE advertising starts, you can connect

## Exit Configuration Mode

1. Place the magnet while in configuration
2. Hold for **3 seconds** — LED blinks **green rapidly** (exit config confirmation)
3. Release the magnet
4. Re-engage **within 2 seconds** to confirm
5. Tracker returns to operational mode

## Power Off

1. Place the magnet
2. Hold for **6 seconds** — LED blinks **red rapidly** (power off confirmation)
3. Release the magnet
4. Re-engage **within 2 seconds** to confirm
5. Tracker powers down

> **Timeout:** If you don't re-engage within 2 seconds, the action is cancelled and the tracker returns to its previous state.

## Timing Reference

| Parameter | Value |
|-----------|-------|
| Short hold (enter/exit config) | 3 seconds |
| Long hold (power off) | 6 seconds |
| Confirmation window | 2 seconds |

## RSPB Tracker

The RSPB board uses an external TPL5111 timer — it powers on/off automatically. If a magnet is present at boot, the tracker enters configuration mode directly.

---

# LED Indicators

The RGB LED provides visual feedback on the tracker state:

| LED Pattern | Color | Meaning |
|-------------|-------|---------|
| Flashing fast | White | Booting / powering down |
| Solid | White | Magnet detected (any state) |
| Flashing | Green | PreOperational — battery OK |
| Flashing | Yellow | PreOperational — battery low |
| Flashing | Red | Error / PreOperational error |
| Solid | Blue | Configuration mode — waiting for BLE connection |
| Flashing | Blue | Configuration mode — BLE not connected |
| Flashing slow | Cyan | GPS acquisition in progress |
| Solid (3s) | Green | GPS fix acquired |
| Solid (3s) | Red | GPS acquisition failed (no fix) |
| Solid | Magenta | Argos satellite transmission in progress |
| Solid | Yellow | Battery critical |
| Alternating | Blue/White | DFU firmware update in progress |

> **LED Mode:** LEDs can be configured to be always on (`ALWAYS`), active for the first 24 hours only (`HRS_24`), or off (`OFF`) via the `LED_MODE` parameter. This saves battery on long deployments.

---

# Connecting to the Tracker

## Via BLE (Configuration Mode)

1. Put the tracker in configuration mode (magnet gesture above)
2. Use one of these tools to connect via Bluetooth Low Energy:
   - **LinkIt GUI** — graphical configuration tool
   - **pylinkit** — Python command-line tool

## Via USB (LinkIt V4 only)

LinkIt V4 boards expose a USB CDC serial port when connected via USB. You can use any serial terminal (115200 baud) to send DTE commands directly.

---

# Basic Configuration

Parameters are read and written using DTE commands. The most common parameters:

## GPS

| Parameter | Key | Description | Typical Value |
|-----------|-----|-------------|---------------|
| GNSS Enable | GNP01 | Enable/disable GPS | 1 (enabled) |
| GNSS Acquisition Timeout | GNP05 | Max seconds to wait for fix | 240 |
| GNSS Dynamic Model | GNP11 | Motion model (0=Portable, 5=Sea, 6=Airborne) | 5 (marine) or 6 (birds) |
| Trigger on Surfaced | GNP25 | Start GPS immediately on surface detection | 1 |

## Argos Satellite TX

| Parameter | Key | Description | Typical Value |
|-----------|-----|-------------|---------------|
| Argos Mode | ARP01 | TX mode (0=Off, 1=PassPrediction, 2=Legacy, 5=SurfacingBurst) | 5 (marine) or 1 (birds) |
| TX Interval | ARP05 | Seconds between transmissions (TR_NOM) | 60 |
| Dry Time Before TX | UNP02 | Wait N seconds at surface before first TX | 0 |

## Underwater Detection (marine models)

| Parameter | Key | Description | Typical Value |
|-----------|-----|-------------|---------------|
| Underwater Enable | UNP01 | Enable surface/dive detection | 1 |
| Detection Source | UNP10 | 0=SWS, 1=Pressure, 2=GNSS, 3=SWS+GNSS | 0 (SWS) |
| Sampling Freq (surface) | UNP04 | Seconds between SWS samples at surface | 10 |
| Sampling Freq (underwater) | UNP03 | Seconds between SWS samples underwater | 2 |

## Battery Management

| Parameter | Key | Description | Typical Value |
|-----------|-----|-------------|---------------|
| Low Battery Threshold | PPP01 | Voltage (mV) to enter low-battery mode | 3400 |

## Example: Configure via DTE Commands

```
 Enable GPS with sea model, 240s timeout
$PARMW#01C;GNP01=1,GNP05=240,GNP11=5

 Argos surfacing burst mode, 60s interval
$PARMW#018;ARP01=5,ARP05=60

 Enable underwater detection (SWS), fast sampling
$PARMW#018;UNP01=1,UNP10=0,UNP03=2

 Read back all parameters to verify
$PARMR#000;
```

---

# Typical Deployment Scenarios

## Marine Turtle

- **Argos Mode:** Surfacing Burst (`ARP01=5`) — Doppler bursts on surface, then GPS positions
- **Underwater Detection:** SWS enabled (`UNP01=1, UNP10=0`)
- **GPS Model:** Sea (`GNP11=5`)
- **GPS Trigger on Surfaced:** Yes (`GNP25=1`)

See the [GUI Parameter Reference](GUI-Parameter-Reference.md) "Scenario A" for a detailed walkthrough.

## Bird Tracking (RSPB)

- **Argos Mode:** Pass Prediction (`ARP01=1`) — TX during satellite passes
- **Underwater Detection:** Disabled (`UNP01=0`)
- **GPS Model:** Airborne (`GNP11=6`)
- **Duty Cycling:** TPL5111 auto-wakeup, controlled by `BOOT_COUNTER_MODULO`

See [RSPB Behavior](12-RSPB-Mortality-Tracker.md) for details.

---

# Retrieving Data

## Satellite Data (Argos/Kineis)

Transmitted data is available on the **CLS/Kineis ArgosWeb** platform. The tracker ID is linked to your Argos program number.

## On-Device Logs

Sensor data is also logged to on-board flash memory. To retrieve logs:

```
 List available log files
$DUMPD#006;GNSS01
$DUMPD#006;AXL001
$DUMPD#006;PRESS1

 Dump all logs
$DUMPD#000;
```

Logs are CSV-formatted and include timestamps.

---

# Firmware Update (DFU)

The tracker supports over-the-air firmware updates via BLE:

1. Put the tracker in configuration mode
2. Connect with the LinkIt GUI or `nrfutil`
3. Upload the DFU package (`.zip` file)
4. LED alternates blue/white during update
5. Tracker reboots automatically on success

See [Programming](3-Programming.md) for details.

---

# Troubleshooting

| Symptom | Possible Cause | Action |
|---------|---------------|--------|
| No LED response to magnet | Battery dead or tracker off | Charge battery, try magnet gesture again |
| Red flashing LED | Error state | Check debug log via USB/UART |
| Yellow solid LED | Battery critical | Charge immediately |
| No GPS fix (red flash after cyan) | Poor sky view, antenna issue | Ensure clear sky view, check antenna connection |
| No satellite TX | Argos mode off, or underwater | Check `ARP01` is not 0, check `UNP01` |
| BLE not visible | Not in config mode | Perform 3s magnet gesture to enter config |

---

# Next Steps

- [GUI Parameter Reference](GUI-Parameter-Reference.md) — Full parameter list and deployment scenarios
- [DTE Commands Reference](6-DTE-Commands.md) — Complete command protocol
- [Board Variants](5-Boards.md) — Hardware differences between boards
- [LinkIt UW Behavior](11-LinkIt-UW-Behavior.md) — Underwater mode, SWS, Argos TX modes
- [RSPB Tracker](12-RSPB-Mortality-Tracker.md) — Bird tracker, duty cycling, mortality
- [Developer Quick Start](2-Developer-Quick-Start.md) — Building firmware from source
