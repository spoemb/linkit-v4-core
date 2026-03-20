# GUI Update Prompt — Comprehensive Firmware Update

This prompt describes ALL changes needed in the LinkIt V4 GUI application to support the latest firmware features. It includes: new mortality detection parameters, updated satellite packet formats, magnet gesture behavior, RSPB bird tracker behavior, and content for info/help pages.

---

# Part 1 — New Parameters: Mortality Detection (MTP01-MTP07)

## New DTE Parameter Group

Add a new parameter group **"Mortality Detection"** in the GUI. These params exist only on RSPB firmware builds (`ENABLE_MORTALITY_SENSOR=1`).

| DTE Key | Display Name | Type | Range | Default | Unit | RW | Description |
|---------|-------------|------|-------|---------|------|----|-------------|
| MTP01 | Mortality Enable | Boolean | - | false | - | RW | Master enable for mortality detection |
| MTP02 | Activity Threshold | Integer | 0-255 | 10 | - | RW | BMA400 activity score below which bird is considered immobile |
| MTP03 | Temperature Threshold | Float | 0.0-60.0 | 25.0 | °C | RW | Body temperature below which bird is considered hypothermic |
| MTP04 | GPS Distance Threshold | Integer | 0-10000 | 50 | m | RW | Position movement below which bird is considered stationary |
| MTP05 | Confirm Days | Integer | 1-30 | 3 | days | RW | Consecutive days of high confidence before CONFIRMED status |
| MTP06 | Duty Cycle Modulo | Integer | 0-100 | 0 | - | RW | New BOOT_COUNTER_MODULO when mortality confirmed. **0 = disabled (never modify duty cycle)** |
| MTP07 | Original Modulo (backup) | Integer | 0-100 | 0 | - | RO | Auto-saved backup of original BOOT_COUNTER_MODULO on first confirmation |

## Visibility Rules

- Entire "Mortality Detection" group: **hidden** when device model is NOT "RSPB"
- MTP02-MTP07: **hidden** when MTP01 = false
- MTP07: always **read-only** (greyed out)
- MTP06: tooltip "Set to 0 to never modify duty cycle. When >0, replaces BOOT_COUNTER_MODULO on mortality confirmation."

## Recommended Layout

```
┌─ Mortality Detection ──────────────────────────────────┐
│                                                         │
│  [x] Enable Mortality Detection               (MTP01)  │
│                                                         │
│  Activity Threshold:      [  10  ] /255        (MTP02)  │
│  Temperature Threshold:   [ 25.0 ] °C          (MTP03)  │
│  GPS Distance Threshold:  [  50  ] meters      (MTP04)  │
│  Confirm Days:            [   3  ] days        (MTP05)  │
│                                                         │
│  [ ] Adapt Duty Cycle When Confirmed                    │
│      New Modulo:          [   6  ]             (MTP06)  │
│      Original (backup):   4  (read-only)       (MTP07)  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

The "Adapt Duty Cycle" checkbox maps to MTP06: unchecked = 0, checked = editable value.

---

# Part 2 — New Log Type: Mortality

## DUMPD Access

- **d_type = C (hex 0x0C)** in `$DUMPD` command (BaseLogDType::MORTALITY = 12)
- **erase_type = E (hex 0x0E)** in `$ERASE` command (BaseEraseType::MORTALITY = 14)
- Log file name on device: `MORTALITY`
- LogType enum value: `LOG_MORTALITY = 14`

## Log Entry Fields (MortalityLogEntry)

| Field | Type | Size | Description |
|-------|------|------|-------------|
| header | LogHeader | 9 bytes | Standard header: day, month, year, hours, minutes, seconds, log_type, payload_size |
| confidence | uint8 | 1 | Mortality confidence percentage (0-100%) |
| consecutive_days | uint8 | 1 | Number of consecutive days with high confidence |
| status | uint8 | 1 | 0=ALIVE, 1=SUSPECTED, 2=CONFIRMED |
| last_activity | uint8 | 1 | Last BMA400 activity reading (0-255) |
| last_body_temp | uint16 | 2 | Last thermistor reading (raw ADC units) |
| last_lat | double | 8 | Last GPS latitude (degrees) |
| last_lon | double | 8 | Last GPS longitude (degrees) |
| last_eval_epoch | uint32 | 4 | Unix epoch timestamp of last daily evaluation |

## Mortality Log Viewer

Add a **"Mortality"** tab in the log viewer. Display as a timeline chart:
- **Primary axis**: Confidence % over time (line chart, 0-100%)
- **Color bands** behind the chart: green (ALIVE), yellow (SUSPECTED), red (CONFIRMED)
- **Secondary axis**: Activity readings (0-255) and body temperature (°C)
- **Table view**: All fields per entry with timestamp

---

# Part 3 — Satellite Packet Formats (All Types)

The tracker transmits several packet types over Argos satellite. The GUI must decode each format correctly.

## Encoding Constants

```
BATTERY_REF_MV      = 2700      // Base voltage offset
BATTERY_MV_PER_UNIT = 20        // mV per LSB
LAT_LON_RESOLUTION  = 10000     // Lat/lon scaling factor
SPEED_METRES_PER_UNIT = 40      // Speed: value × 40 m/s
ALTITUDE_METRES_PER_UNIT = 40   // Altitude: value × 40 m
HEADING_DEGREES_PER_UNIT = 1/1.42 // ~0.7° per unit
MAX_GPS_ENTRIES    = 4          // Max GPS fixes in long packet
```

## 3.1 — Doppler Packet (24 bits / 3 bytes)

Smallest packet. Contains only battery info. Argos satellite computes position from Doppler frequency shift.

```
[Last position 8b][Battery voltage 7b][Low battery 1b]
```

| Field | Bits | Width | Encoding |
|-------|------|-------|----------|
| Last known position | 0-7 | 8 | Reserved (0) |
| Battery voltage | 8-14 | 7 | `(raw × 20) + 2700` = mV |
| Low battery flag | 15 | 1 | Boolean |

**When used:** ARGOS_MODE=4 (DOPPLER), Low Battery mode, Surfacing Burst Doppler phase.

## 3.2 — Short GNSS Packet (96 bits / 12 bytes)

Single GPS fix with full detail (heading, altitude).

```
[Header 3b][Time 16b][GPS 51b][Heading 8b][Altitude 8b][Battery 7b][LB 1b]
 ← padding to 96 bits if needed →
```

| Field | Bits | Width | Encoding |
|-------|------|-------|----------|
| Header | 0-2 | 3 | `0b000` (short packet) |
| Day | 3-7 | 5 | 1-31 |
| Hour | 8-12 | 5 | 0-23 |
| Minute | 13-18 | 6 | 0-59 |
| Latitude | 19-39 | 21 | Signed: `(raw - 900000) / 10000.0` degrees |
| Longitude | 40-61 | 22 | Signed: `(raw - 1800000) / 10000.0` degrees |
| Speed | 62-68 | 7 | `raw × 40` m/s. Max = 127 × 40 = 5080 m/s |
| Out-of-zone flag | 69 | 1 | 1 = outside geofence |
| Heading | 70-77 | 8 | `raw / 1.42` degrees (0-360°). 0xFF = invalid |
| Altitude | 78-85 | 8 | `raw × 40` meters. 0xFF = invalid, 0x00 = 0m |
| Battery voltage | 86-92 | 7 | `(raw × 20) + 2700` mV |
| Low battery | 93 | 1 | Boolean |

**No GPS fix:** Latitude, Longitude, Speed = all 1s (0x1FFFFF, 0x3FFFFF, 0x7F). Heading = 0xFF. Altitude = 0xFF.

**When used:** ARGOS_MODE=2 (LEGACY), single GPS fix available.

## 3.3 — Long GNSS Packet (224 bits / 28 bytes)

Multiple GPS fixes (2-4) with delta-time encoding for subsequent fixes.

```
[Time 16b][GPS#1 51b][Battery 8b][DeltaTimeLoc 4b][GPS#2 43b][GPS#3 43b][GPS#4 43b]
```

| Field | Bits | Width | Encoding |
|-------|------|-------|----------|
| Day | 0-4 | 5 | 1-31 |
| Hour | 5-9 | 5 | 0-23 |
| Minute | 10-15 | 6 | 0-59 |
| Latitude #1 | 16-36 | 21 | Signed (same as short packet) |
| Longitude #1 | 37-58 | 22 | Signed |
| Speed #1 | 59-65 | 7 | `raw × 40` m/s |
| Out-of-zone | 66 | 1 | Boolean |
| Battery voltage | 67-73 | 7 | `(raw × 20) + 2700` mV |
| Low battery | 74 | 1 | Boolean |
| Delta time loc | 75-78 | 4 | Time resolution between fixes: 0=10min, 1=30min, 2=1h, 3=2h |
| GPS #2-4 | 79+ | 43 each | Lat (21b) + Lon (22b) per entry. 0xFFFFFF = empty slot |

**Ordering:** Fixes are packed newest-first (entry 0 = most recent, entry 3 = oldest).

**When used:** ARGOS_MODE=2 (LEGACY), multiple GPS fixes in depth pile.

## 3.4 — Sensor Packet (variable, max 192 bits / 24 bytes)

GPS fix + sensor data. Two formats: **standard** (all boards) and **RSPB compact**.

### 3.4a — Standard Sensor Packet (non-RSPB)

```
[Time 16b][GPS 51b][Battery 8b][ALS? 17b][PH? 14b][Pressure? 29b][SeaTemp? 21b][AXL? 67b]
```

| Field | Bits | Width | Present | Encoding |
|-------|------|-------|---------|----------|
| Day | 0-4 | 5 | Always | 1-31 |
| Hour | 5-9 | 5 | Always | 0-23 |
| Minute | 10-15 | 6 | Always | 0-59 |
| Latitude | 16-36 | 21 | Always | Signed (no fix = all 1s) |
| Longitude | 37-58 | 22 | Always | Signed (no fix = all 1s) |
| Speed | 59-65 | 7 | Always | `raw × 40` m/s |
| Out-of-zone | 66 | 1 | Always | Boolean |
| Battery voltage | 67-73 | 7 | Always | `(raw × 20) + 2700` mV |
| Low battery | 74 | 1 | Always | Boolean |
| ALS (light) | 75-91 | 17 | If ALS enabled + TX_MODE != OFF | Lux (raw) |
| pH | varies | 14 | If PH enabled + TX_MODE != OFF | pH (raw) |
| Pressure | varies | 15 | If Pressure enabled + TX_MODE != OFF | hPa (encoded) |
| Pressure temp | varies | 14 | If Pressure enabled + TX_MODE != OFF | °C (encoded) |
| Sea temperature | varies | 21 | If SeaTemp enabled + TX_MODE != OFF | °C (encoded, high precision) |
| AXL temperature | varies | 14 | If AXL enabled + TX_MODE != OFF | BMA400 die temp (raw) |
| AXL X axis | varies | 15 | If AXL enabled + TX_MODE != OFF | Acceleration (raw) |
| AXL Y axis | varies | 15 | If AXL enabled + TX_MODE != OFF | Acceleration (raw) |
| AXL Z axis | varies | 15 | If AXL enabled + TX_MODE != OFF | Acceleration (raw) |
| AXL activity | varies | 8 | If AXL enabled + TX_MODE != OFF | 0-255 activity score |

**Sensors are packed sequentially.** Only enabled sensors with TX_MODE != OFF are included. If total > 192 bits, the packet is **truncated** to 192 bits (last sensors dropped).

### 3.4b — RSPB Compact Sensor Packet (133 bits)

For RSPB bird trackers. AXL X/Y/Z axes and AXL temperature are **removed** (useless for birds). Only the activity score (8 bits) is kept. Mortality confidence (7 bits) is appended.

```
[Time 16b][GPS 51b][Battery 8b][Pressure 29b][Thermistor 14b][Activity 8b][Mortality% 7b]
  Total: 133 bits / 192 max = 59 bits FREE
```

| Field | Bit Position | Width | Encoding |
|-------|-------------|-------|----------|
| Day | 0-4 | 5 | 1-31 |
| Hour | 5-9 | 5 | 0-23 |
| Minute | 10-15 | 6 | 0-59 |
| Latitude | 16-36 | 21 | Signed: `(raw - 900000) / 10000.0` degrees. No fix = 0x1FFFFF |
| Longitude | 37-58 | 22 | Signed: `(raw - 1800000) / 10000.0` degrees. No fix = 0x3FFFFF |
| Speed | 59-65 | 7 | `raw × 40` m/s. No fix = 0x7F |
| Out-of-zone | 66 | 1 | Boolean (geofence) |
| Battery voltage | 67-73 | 7 | `(raw × 20) + 2700` mV |
| Low battery | 74 | 1 | Boolean |
| Pressure | 75-89 | 15 | Barometric pressure (hPa encoded) |
| Pressure temperature | 90-103 | 14 | LPS28DFW temperature (°C encoded) |
| Body temperature | 104-117 | 14 | Thermistor NTC (raw ADC units) |
| Activity score | 118-125 | 8 | BMA400 activity (0-255). 0=immobile, 255=max motion |
| Mortality confidence | 126-132 | 7 | Percentage 0-100%. Only present when MORTALITY_ENABLE=true |

**Format detection:** Check device model = "RSPB" (from IDT02 or IDP11). RSPB always uses compact format.

## 3.5 — Certification Packet (96 or 224 bits)

User-defined payload for hardware certification testing. Size depends on payload length.

- Short (≤12 bytes): 96 bits
- Long (>12 bytes): 224 bits
- Payload from CTP02 (CERT_TX_PAYLOAD), hex-encoded

**When used:** CTP01 (CERT_TX_ENABLE) = true.

---

# Part 4 — Magnet (Reed Switch) Gestures

The tracker is controlled by a magnet that activates a reed switch. All actions require a **confirmation gesture** to prevent accidental activation.

## Timing Constants

| Constant | Value | Description |
|----------|-------|-------------|
| SHORT_HOLD | 3000 ms | Hold duration for BLE config toggle |
| LONG_HOLD | 6000 ms | Hold duration for power off |
| CONFIRMATION_TIMEOUT_MS | 2000 ms | Window to re-engage magnet after release |
| OFF_LED_PERIOD_MS | 5000 ms | LED display time before actual power down |

## Gesture Flow

```
   Place magnet                                     Remove magnet     Replace magnet
       |                                                 |                 |
       v                                                 v                 v
    ENGAGE                                            RELEASE          RE-ENGAGE
    (solid white LED)                                 (2s window)      (CONFIRMED!)
       |                                                 |
       | Hold 3 seconds...                               |
       v                                                 |
    SHORT_HOLD                                           |
    (rapid BLUE blink = enter BLE config)                |
    (rapid GREEN blink = exit BLE config,                |
     if already in config mode)                          |
       |                                                 |
       | Continue holding to 6 seconds...                |
       v                                                 |
    LONG_HOLD                                            |
    (rapid RED blink = power off)                        |
```

## Available Actions

| Current State | Gesture | LED Feedback | Confirmed Action |
|--------------|---------|-------------|-----------------|
| Any (except Config) | Hold 3s, release, re-engage within 2s | Rapid BLUE blink → release → re-engage | Enter BLE Configuration mode |
| Configuration | Hold 3s, release, re-engage within 2s | Rapid GREEN blink → release → re-engage | Exit Configuration → PreOperational → Operational |
| Any state | Hold 6s, release, re-engage within 2s | Rapid RED blink → release → re-engage | Power off (OffState → PMU::powerdown) |

## LED Color Reference

| LED State | Color | Pattern | Meaning |
|-----------|-------|---------|---------|
| Magnet engaged | White | Solid | Magnet detected, waiting for gesture |
| Confirm enter BLE | Blue | Rapid blink | Release + re-engage to confirm BLE config entry |
| Confirm exit BLE | Green | Rapid blink | Release + re-engage to confirm exit config |
| Confirm power off | Red | Rapid blink | Release + re-engage to confirm power off |
| GNSS searching | Blue | Slow blink | GPS acquisition in progress |
| GNSS fix acquired | Green | 1 flash | GPS fix obtained |
| GNSS no fix | Red | 1 flash | GPS timeout, no fix |
| Argos TX active | Cyan | Flash | Satellite transmission in progress |
| Argos TX complete | Green | 1 flash | Transmission done |
| Pre-operational OK | Green | 2 flashes | Battery nominal, transitioning to operational |
| Pre-operational LB | Yellow | 2 flashes | Battery low but operational |
| Battery critical | Red | Rapid blink | Critical battery, powering off |
| Boot | White | Solid (1s) | System initializing |
| Power down | Red | Fade out (5s) | Powering off |
| Error | Red | Rapid blink (5s) | Fatal error, powering off |

## RSPB Specifics

- **TRANSIT_PERIOD_MS = 100ms** (vs 5000ms on LinkIt V4): RSPB transitions from PreOp → Operational almost instantly (no time to waste with TPL5111)
- **BatteryCriticalState**: If magnet is present → enters ConfigurationState (allows BLE config of a depleted device). If no magnet → immediate powerdown
- LED mode should be OFF for bird deployments (LDP01=0, LDP02=0)

---

# Part 5 — RSPB Bird Tracker Behavior (Help Page Content)

## Overview

The RSPB tracker is a nano-power GPS/Satellite tracker designed for long-term bird monitoring. It uses a TPL5111 external timer chip that physically cuts power between sessions. The device wakes periodically, acquires GPS + sensor data, transmits via Argos satellite, then powers down completely.

## Duty Cycling with TPL5111

```
  TPL5111 Timer (every ~1h45 = 6300 seconds)
         |
         v
  ┌──────────────┐     ┌────────────────┐     ┌──────────────────┐     ┌───────────┐
  │    BOOT      │ --> │ PRE-OPERATIONAL│ --> │   OPERATIONAL    │ --> │ POWER OFF │
  │ Check modulo │     │ Check battery  │     │ GNSS + TX cycle  │     │ Wait next │
  │ counter      │     │ level          │     │ Sensor sampling  │     │ TPL5111   │
  └──────┬───────┘     └───────┬────────┘     │ Mortality eval   │     │ wakeup    │
         |                     |              └────────┬─────────┘     └───────────┘
         v                     v                       v
   Not our turn?        Battery critical?       TX limit or timer?
   → POWER OFF          → POWER OFF            → POWER OFF
```

**BOOT_COUNTER_MODULO** controls how many wakeups are skipped:
- Counter increments on each boot
- When `counter % modulo == 0` → active cycle runs, counter resets to 0
- Example: modulo=4 with 1h45 interval → active cycle every ~7 hours

**Effective cycle period = WAKEUP_PERIOD × BOOT_COUNTER_MODULO**

## Battery Modes

```
  100% ├──────────────────────────────────────┤
       │           NORMAL MODE                │  GPS + Sensors + Satellite TX
  LB%  ├──── LB_THRESHOLD (default: 10%) ────┤
       │         LOW BATTERY MODE             │  Doppler-only TX (no GPS, no sensors)
  LBP12├──── LB_CRITICAL (default: 5%) ──────┤
       │           CRITICAL                   │  Immediate power off
   0%  └──────────────────────────────────────┘
```

**Normal mode:** GNSS on → first fix → GNSS off → N satellite TX messages (with GPS + sensors) → power off.

**Low battery mode:** No GNSS, no sensors → N Doppler TX (3 bytes: battery only) → power off. Argos satellites estimate position from Doppler frequency shift.

**Critical mode:** Immediate power off. Exception: if magnet detected → BLE configuration mode (allows reconfiguring a depleted device).

## Session Timeline (Normal)

```
  T+0s     TPL5111 wakes device, modulo check → active cycle
  T+0.1s   Pre-operational: battery OK → Operational
  T+1s     GNSS on, sensors start sampling
  T+45s    First GPS fix → GNSS off (single fix mode)
  T+46s    Mortality algorithm evaluates (if enabled)
  T+47s    Satellite TX #1 (GPS + pressure + thermistor + activity + mortality%)
  T+107s   Satellite TX #2
  T+167s   Satellite TX #3 → SHUTDOWN_NTIME_SAT reached → POWER OFF
           Total awake: ~3 minutes, next wakeup in ~7 hours
```

## Pseudo RTC (Timekeeping)

The TPL5111 architecture means the MCU loses all state at power-off, including the real-time clock. The firmware maintains approximate time via a flash-persisted parameter chain:

```
  Each boot: LAST_KNOWN_RTC += WAKEUP_PERIOD (approximate)
  Each GPS fix: LAST_KNOWN_RTC = exact satellite time (corrects drift)
```

## Mortality Detection

When enabled (MTP01=1), the firmware monitors three indicators at each active session:

| Indicator | Weight | Condition | Live Bird | Dead Bird |
|-----------|--------|-----------|-----------|-----------|
| Activity (BMA400) | 40 pts | Activity < MTP02 | 20-200 (moving) | 0-5 (immobile) |
| Body temperature | 30 pts | Temp < MTP03 | ~38-42°C | Converges to ambient (10-25°C) |
| GPS stationarity | 30 pts | Distance < MTP04 AND speed < 0.1 m/s | Moves between sessions | Same position |

**Session score** = sum of triggered indicators (0-100).

**Confidence** = exponential moving average: `new = (old × 7 + score × 3) / 10`

**Status transitions:**

| Status | Icon | Condition |
|--------|------|-----------|
| ALIVE | 🟢 | Confidence < 50% |
| SUSPECTED | 🟡 | Confidence >= 50% |
| CONFIRMED | 🔴 | Confidence >= 80% for MTP05 consecutive days |

**Duty cycle adaptation** (opt-in, MTP06 > 0): On CONFIRMED, the firmware increases BOOT_COUNTER_MODULO to reduce wakeup frequency (saves battery). Original value is backed up in MTP07 and restored if bird "recovers".

**The mortality confidence (0-100%) is transmitted in every satellite sensor packet** so ground stations can see the bird's status in real-time.

## Energy Budget

| Phase | Duration | Current | Energy |
|-------|----------|---------|--------|
| Boot + modulo skip | ~50ms | 5 mA | ~0.07 µAh |
| Boot + pre-operational | ~0.2s | 5 mA | ~0.3 µAh |
| GNSS acquisition (typical) | ~45s | 25 mA | ~312 µAh |
| GNSS acquisition (cold start) | 180s | 25 mA | ~1250 µAh |
| Satellite TX (×3, 60s interval) | ~180s | 15 mA avg | ~750 µAh |
| **Total per active cycle (typical)** | **~3 min** | - | **~1.1 mAh** |

With modulo=4 (~7h cycle): ~3-8 mAh/day. NCR18650 (3400 mAh): **~400-1000 days** without solar.

---

# Part 6 — RSPB Configuration Scenario for GUI

Add a pre-configured deployment scenario **"RSPB Bird Mortality"** that applies all parameters at once:

```
# Argos TX
ARP01=2          # LEGACY mode (fixed interval)
ARP05=120        # TX every 2 minutes
ARP32=0          # No satellite RX (save power)

# Power management (TPL5111)
PWP01=900        # Safety shutdown timer: 15 min
PWP03=5          # Active every 5th wakeup (~8h45)
PWP05=5          # 5 TX messages per session

# GNSS
GNP01=1          # GPS enabled
GNP11=6          # Dynamic model: Airborne 1G (birds)
GNP30=1          # Stop GPS after first fix

# Sensors
AXP01=1          # Accelerometer enabled (required for mortality)
AXP05=1          # AXL TX mode = MEAN
AXP06=10         # 10 samples averaged per TX
THP01=1          # Thermistor enabled (required for mortality)
THP06=2          # Thermistor TX mode = ONESHOT
PRP01=1          # Pressure enabled
PRP04=2          # Pressure TX mode = ONESHOT

# Mortality detection
MTP01=1          # Enable mortality detection
MTP02=10         # Activity threshold (immobile if < 10/255)
MTP03=25.0       # Temperature threshold (hypothermic if < 25°C)
MTP04=50         # GPS distance threshold (stationary if < 50m)
MTP05=3          # Confirm after 3 consecutive days
MTP06=6          # Adapt duty cycle to modulo 6 when confirmed

# Disable unused features
UNP01=0          # No underwater detection
LDP01=0          # LEDs off
LDP02=0          # External LED off

# Low battery
LBP01=1          # Enable low battery mode
LBP02=30         # Enter LB at 30% SOC
LBP06=0          # No GPS in LB (Doppler only)
LBP14=2          # 2 Doppler TX per LB session
```

---

# Part 7 — Dashboard Widgets

## Mortality Status Widget (RSPB only)

```
┌─────────────────────────────────────┐
│  Bird Status: ● ALIVE               │
│  Confidence:  ████░░░░░░░░  38%     │
│  Consecutive Days: 0 / 3            │
│                                     │
│  Last Activity:     42 / 255        │
│  Body Temperature:  39.2°C          │
│  Position Change:   1.2 km          │
│  Last Evaluation:   2026-03-19 14:22│
└─────────────────────────────────────┘
```

**Status indicator colors:**
- 🟢 Green dot + "ALIVE": confidence < 50%
- 🟡 Yellow dot + "SUSPECTED": confidence 50-79%
- 🔴 Red dot + "CONFIRMED": confidence >= 80% for N+ days

## RSPB Session Info Widget

```
┌─────────────────────────────────────┐
│  Duty Cycle                         │
│  Wakeup Period:    1h 45m (6300s)   │
│  Boot Modulo:      5                │
│  Effective Cycle:  ~8h 45m          │
│  Boot Counter:     3 / 5            │
│                                     │
│  Last Session                       │
│  TX Count:         3 / 5            │
│  GPS Fix:          ✓ (TTFF: 32s)    │
│  Battery:          78% (3.92V)      │
│  Shutdown Cause:   NTIME_SAT        │
└─────────────────────────────────────┘
```

---

# Part 8 — Help Page: Underwater Detection (LinkIt V4)

For non-RSPB devices (marine trackers), the underwater detection system controls when the device can transmit:

```
     SURFACE                          UNDERWATER
┌──────────────────┐            ┌──────────────────┐
│  GNSS active     │  SWS wet   │  GNSS suspended  │
│  Argos TX OK     │ ────────>  │  Argos TX OFF    │
│  Sensors active  │            │  Sensors active  │
│                  │  SWS dry   │  (reduced rate)  │
│                  │ <────────  │                  │
└──────────────────┘            └──────────────────┘
```

**Detection sources:**
- SWS (Saltwater Switch): Analog conductivity electrode with auto-calibration
- Pressure sensor: Depth > threshold = underwater
- GNSS: Satellite count below threshold = submerged
- SWS + GNSS: Combined (highest reliability)

**ARGOS_MODE=5 (Surfacing Burst):** Optimal for marine animals with brief surfacings. Progressive Doppler messages on surface detection, then switches to GNSS TX when GPS fix acquired. Interval formula: `Interval(N) = min(INIT_S + (N-1) × STEP_S, MAX_S)`

---

# Part 9 — ARGOS_MODE Reference (Help Page)

| Value | Name | Description | Use Case |
|-------|------|-------------|----------|
| 0 | OFF | No satellite TX | Debugging / BLE-only config |
| 1 | PASS_PREDICTION | TX when satellite overhead (needs AOP data) | Optimal power efficiency |
| 2 | LEGACY | Fixed interval TX, all hours | Simple, reliable. Default for RSPB |
| 3 | DUTY_CYCLE | Fixed interval within hourly windows (24-bit bitmask) | Targeted TX hours |
| 4 | DOPPLER | Doppler-only (3 bytes, no GNSS) | Ultra-low power, low accuracy |
| 5 | SURFACING_BURST | Progressive Doppler then GNSS | Marine animals (turtles, seals) |

---

# Part 10 — GNSS Dynamic Models (Help Page)

| Value | Model | Use Case |
|-------|-------|----------|
| 0 | PORTABLE | General / default |
| 3 | PEDESTRIAN | Slow-moving land animals |
| 5 | SEA | Marine surface animals (turtles, seals) |
| 6 | AIRBORNE_1G | **Birds** (accepts high speed, low acceleration) |
| 7 | AIRBORNE_2G | Fast-diving birds |

**Wrong model = missed fixes.** PORTABLE rejects bird-speed motion. SEA is tuned for surface-level marine navigation only.
