# Overview

The RSPB tracker is a wildlife GPS/Satellite tracker designed for bird monitoring.
It uses a **nano-power architecture**: the device spends most of its time completely
powered off. An external timer chip (TPL5111) periodically wakes the tracker to
perform a short operational cycle, then the device powers down again.

```
  TPL5111 Timer (every ~1h45)
         |
         v
  +-------------+     +----------------+     +------------------+     +-----------+
  |    BOOT     | --> | PRE-OPERATIONAL| --> |   OPERATIONAL    | --> | POWER OFF |
  | Check modulo|     | Check battery  |     | GNSS + TX cycle  |     | Wait next |
  | counter     |     | level          |     | (limited by      |     | TPL5111   |
  +------+------+     +-------+--------+     |  NTIME_SAT)      |     | wakeup    |
         |                    |               +--------+---------+     +-----------+
         |                    |                        |
         v                    v                        v
   Not our turn?       Battery critical?        TX limit reached?
   --> POWER OFF       --> POWER OFF            --> POWER OFF
       immediately         immediately              (session done)
```

## Key Concept: Duty Cycling with TPL5111

The TPL5111 is an external nano-power timer that physically controls the power
supply. When the tracker powers down, the TPL5111 keeps counting and will
re-power the device after its configured interval (~1h45min).

To further reduce energy usage, not every wakeup results in an active cycle.
The `BOOT_COUNTER_MODULO` parameter controls this:

```
 Wakeup 1     Wakeup 2     Wakeup 3     Wakeup 4     Wakeup 5
  (skip)       (skip)       (skip)      ACTIVE!        (skip)  ...
    |            |            |            |              |
    v            v            v            v              v
 increment    increment    increment    modulo=4       increment
 counter +    counter +    counter +    --> RUN!       counter +
 power off    power off    power off    GPS + TX       power off

                    With modulo=4 and 1h45 interval:
                    Active cycle every ~7 hours
```


---

# Battery Modes

The tracker operates in three distinct modes depending on battery voltage.
Each mode has different behavior to maximize device lifetime.

```
  Battery Level
  ============

  100% |########################################|
       |             NORMAL MODE                 |  Full operation:
       |          GPS fix + Sensor data          |  GNSS, Sensors, Satellite TX
       |         + Satellite transmission        |
       |                                         |
  LB%  |---- LB_THRESHOLD (default: 10%) -------|
       |           LOW BATTERY MODE              |  Reduced operation:
       |         Doppler TX only (no GPS)        |  Battery voltage only
       |         Fewer messages per session       |
       |                                         |
  LBP12|---- LB_CRITICAL_THRESH (default: 5%)   |
       |          CRITICAL MODE                  |  No operation:
       |       Immediate power off               |  Preserve battery for
       |       No TX, no GPS, nothing            |  solar recharge
   0%  |_________________________________________|
```

## Mode 1: Normal Operation

When battery is above `LB_THRESHOLD`:

```
  WAKEUP
    |
    v
  +--------+     +-------+     +--------+     +--------+     +--------+
  | GNSS   | --> | Got   | --> | TX #1  | --> | TX #2  | --> | TX #3  | --> POWER OFF
  | acquire |     | fix!  |     | (GPS+  |     | (GPS+  |     | (GPS+  |
  | position|     | Stop  |     | sensor)|     | sensor)|     | sensor)|
  | (1 fix) |     | GNSS  |     |  60s   |     |  60s   |     |        |
  +----+----+     +-------+     +--------+     +--------+     +--------+
       |
       | No fix after timeout?
       v
    TX with "NO FIX" status --> POWER OFF
```

**What happens:**
1. GNSS module powers on, acquires one satellite fix (position)
2. With `GNSS_SESSION_SINGLE_FIX=1`, GNSS stops after the first fix (saves power)
3. The tracker sends `SHUTDOWN_NTIME_SAT` satellite messages (default: 3)
4. Each message contains GPS position + sensor data
5. Messages are spaced by `TR_NOM` interval (default: 60 seconds)
6. After the last message, the device powers down

**Parameters controlling Normal mode:**

| Parameter | DTE Key | Default | RSPB | Description |
|-----------|---------|---------|------|-------------|
| GNSS_SESSION_SINGLE_FIX | GNP30 | 0 (off) | **1** | Stop GNSS after first fix |
| SHUTDOWN_NTIME_SAT | PWP05 | 0 (off) | **3** | Number of TX before powerdown |
| TR_NOM | ARP05 | 60s | 60s | Interval between TX messages |
| GNSS_COLD_ACQ_TIMEOUT | GNP09 | 530s | **180s** | Max time for cold GPS fix |
| GNSS_ACQ_TIMEOUT | GNP05 | 120s | **90s** | Max time for warm GPS fix |
| GNSS_DYN_MODEL | GNP11 | 0 (portable) | **6** (airborne) | GPS motion model for birds |
| GNSS_HACCFILT_THR | GNP21 | 5m | **10m** | Accept fix with this accuracy |
| NTRY_PER_MESSAGE | ARP19 | 6 | **3** | TX repetitions per message |
| ARGOS_RX_EN | ARP32 | 1 (on) | **0** | Satellite downlink (disabled to save power) |

## Mode 2: Low Battery

When battery drops below `LB_THRESHOLD` and `LB_EN=1`:

```
  WAKEUP
    |
    v
  +----------+     +----------+     +----------+
  | Doppler  | --> | Doppler  | --> | POWER    |
  | TX #1    |     | TX #2    |     | OFF      |
  | (battery |     | (battery |     |          |
  |  voltage)|     |  voltage)|     |          |
  |   90s    |     |          |     |          |
  +----------+     +----------+     +----------+

  NO GPS, NO SENSORS
  Just battery voltage in a small Doppler packet (3 bytes)
  Satellites use Doppler shift to estimate position
```

**What happens:**
1. No GNSS module powered (saves significant energy)
2. No sensor acquisition
3. The tracker sends `LB_SHUTDOWN_NTIME_SAT` Doppler messages (default: 2)
4. Each Doppler message contains only battery voltage (24 bits / 3 bytes)
5. The Argos satellite system uses Doppler frequency shift to estimate position
6. Messages are spaced by `TR_LB` interval (default: 90 seconds)
7. After the last message, the device powers down

**Parameters controlling Low Battery mode:**

| Parameter | DTE Key | Default | RSPB | Description |
|-----------|---------|---------|------|-------------|
| LB_EN | LBP01 | 0 (off) | **1** | Enable low battery mode |
| LB_THRESHOLD | LBP02 | 10% | 10% | Battery % to enter LB mode |
| LB_GNSS_EN | LBP06 | 1 (on) | **0** | Disable GNSS in LB mode |
| LB_SHUTDOWN_NTIME_SAT | LBP14 | 0 (off) | **2** | Number of Doppler TX before powerdown |
| TR_LB | ARP06 | 240s | **90s** | Interval between Doppler TX |

## Mode 3: Critical Battery

When battery SOC drops below `LB_CRITICAL_THRESH`:

```
  WAKEUP
    |
    v
  IMMEDIATE POWER OFF
  (no GPS, no TX, nothing)
  Wait for solar recharge
```

**What happens:**
1. The device detects critically low battery at boot
2. Immediately powers down without any operation
3. The TPL5111 will keep waking the device periodically
4. Once solar panels recharge the battery above critical threshold,
   the device will resume Low Battery or Normal mode

**Exception:** If a magnet is detected (reed switch engaged) while in critical battery state, the device enters configuration mode instead of powering off. This allows reconfiguring a depleted device via BLE without waiting for solar recharge.

**Parameters controlling Critical mode:**

| Parameter | DTE Key | Default | Description |
|-----------|---------|---------|-------------|
| LB_CRITICAL_THRESH | LBP12 | 5% | SOC percentage threshold for critical mode |


---

# Safety Net: Shutdown Timer

The `SHUTDOWN_TIMER` is an independent safety mechanism that runs in parallel
with the NTIME_SAT counter. It prevents the device from staying awake
indefinitely if something goes wrong (e.g., satellite TX keeps failing).

```
  WAKEUP
    |
    +---> Start SHUTDOWN_TIMER (600s = 10 minutes)
    |
    +---> Start normal operation (GPS + TX cycle)
    |
    |     Which triggers first?
    |     =======================
    |
    +---> NTIME_SAT reached (e.g., 3 TX done)  --> POWER OFF  (normal case)
    |
    +---> SHUTDOWN_TIMER expired (600s)          --> POWER OFF  (safety net)
```

| Parameter | DTE Key | Default | RSPB | Description |
|-----------|---------|---------|------|-------------|
| SHUTDOWN_TIMER | PWP01 | 0 (off) | **600s** | Max seconds awake per cycle |


---

# Pseudo RTC (Timekeeping Without Battery Backup)

The TPL5111 architecture means the MCU loses all RAM state at each power-off, including the
real-time clock. There is no battery-backed RTC crystal on the RSPB board. To maintain
approximate time between wakeups, the firmware implements a **pseudo RTC chain** using a
flash-persisted parameter.

## How It Works

```
  Boot N                                       Boot N+1
  ======                                       ========

  Read LAST_KNOWN_RTC from flash               Read LAST_KNOWN_RTC from flash
  (e.g., 1708444800)                           (e.g., 1708451100)
       |                                            |
       v                                            v
  Add WAKEUP_PERIOD                            Add WAKEUP_PERIOD
  (+ 6300 seconds)                             (+ 6300 seconds)
       |                                            |
       v                                            v
  rtc->settime(1708451100)                     rtc->settime(1708457400)
  "Approximate time set"                       "Approximate time set"
       |                                            |
       v                                            v
  ... GNSS fix acquired ...                    ... GNSS fix acquired ...
  Exact time from satellite!                   Exact time from satellite!
       |                                            |
       v                                            v
  Save LAST_KNOWN_RTC to flash                 Save LAST_KNOWN_RTC to flash
  (real timestamp from GNSS)                   (real timestamp from GNSS)
```

## Chain Initialization

The pseudo RTC chain needs a starting value. There are two ways to seed it:

1. **First GNSS fix:** When the device gets its first GPS fix, `LAST_KNOWN_RTC` is
   automatically saved to flash during the power-off phase. Subsequent boots use this
   as the base.

2. **Manual seed via RTCW command:** Before deployment, send the current Unix timestamp
   via DTE:
   ```
   $RTCW#00A;1708444800\r
   ```
   This sets the RTC immediately and also saves `LAST_KNOWN_RTC` to flash, seeding
   the chain without needing a GNSS fix first.

## Accuracy

The pseudo RTC drifts by the difference between the configured `WAKEUP_PERIOD` and
the actual TPL5111 timer interval. Typical drift is a few seconds per cycle. Each
successful GNSS fix corrects the drift completely.

If the device fails to get a GNSS fix for several cycles, the time will gradually
drift but remain usable for pass prediction and timestamping.

## Related Parameters

| Parameter | DTE Key | Description |
|-----------|---------|-------------|
| LAST_KNOWN_RTC | PWP06 | Flash-persisted Unix timestamp. Updated on GNSS fix and RTCW. |
| WAKEUP_PERIOD | PWP04 | TPL5111 period in seconds (~6300s). Added to LAST_KNOWN_RTC at boot. |
| RTC_CURRENT_TIME | SYT01 | Live RTC value readable via STATR. |

## Reading Current RTC

To check the current RTC time:
```
$STATR#005;SYT01\r
$O;STATR#010;SYT01=1708451100\r
```


---

# Complete Session Timeline

## Normal mode example (RSPB deployment config):

```
  Time
   |
   |  T+0s     TPL5111 wakes device
   |            Boot, check modulo counter (is it our turn?)
   |            --> YES (counter % 4 == 0)
   |
   |  T+0.1s   Pre-operational: check battery
   |            --> Battery OK (Normal mode)
   |
   |  T+1s     GNSS module powered on
   |            Searching for satellites...
   |
   |  T+45s    First GPS fix acquired! (typical TTFF)
   |            GNSS module powered off (GNSS_SESSION_SINGLE_FIX=1)
   |
   |  T+46s    Satellite TX #1: GPS position + sensor data
   |
   |  T+106s   Satellite TX #2: GPS position + sensor data  (+60s TR_NOM)
   |
   |  T+166s   Satellite TX #3: GPS position + sensor data  (+60s TR_NOM)
   |            SHUTDOWN_NTIME_SAT reached (3/3)
   |            --> POWER OFF
   |
   |  T+166s   Total awake time: ~2.8 minutes
   |            Device sleeps until next TPL5111 wakeup (~7h later)
   |
   v
```

## Low battery mode example:

```
  Time
   |
   |  T+0s     TPL5111 wakes device
   |            Boot, check modulo counter --> YES
   |
   |  T+0.1s   Pre-operational: check battery
   |            --> Battery LOW (Low Battery mode)
   |
   |  T+1s     Doppler TX #1: battery voltage (3 bytes)
   |            No GPS, no sensors
   |
   |  T+91s    Doppler TX #2: battery voltage    (+90s TR_LB)
   |            LB_SHUTDOWN_NTIME_SAT reached (2/2)
   |            --> POWER OFF
   |
   |  T+91s    Total awake time: ~1.5 minutes
   |
   v
```


---

# All RSPB Parameters Reference

## Power Management (TPL5111)

| Parameter | DTE Key | Type | Range | Default | RSPB | Description |
|-----------|---------|------|-------|---------|------|-------------|
| SHUTDOWN_TIMER | PWP01 | UINT | 0-86400 | 0 | **600** | Safety timeout (seconds). Forces powerdown if session exceeds this. 0=disabled. |
| BOOT_COUNTER | PWP02 | UINT | - | 0 | - | Read-only. Current boot count since last active cycle. |
| BOOT_COUNTER_MODULO | PWP03 | UINT | 2-1000 | 2 | **4** | Run active cycle every N wakeups. Higher = less frequent = more battery savings. |
| WAKEUP_PERIOD | PWP04 | UINT | - | 6300 | - | Read-only. TPL5111 wakeup period in seconds (~1h45 = 6300s). |
| SHUTDOWN_NTIME_SAT | PWP05 | UINT | 0-65535 | 0 | **3** | Number of satellite TX messages per normal session before powerdown. 0=disabled. |

## GNSS (GPS)

| Parameter | DTE Key | Type | Range | Default | RSPB | Description |
|-----------|---------|------|-------|---------|------|-------------|
| GNSS_EN | GNP01 | BOOL | 0/1 | 1 | 1 | Enable GNSS module. |
| GNSS_ACQ_TIMEOUT | GNP05 | UINT | 10-600 | 120 | **90** | Warm/hot start timeout (seconds). |
| GNSS_COLD_ACQ_TIMEOUT | GNP09 | UINT | 10-600 | 530 | **180** | Cold start timeout (seconds). First fix or no backup battery. |
| GNSS_DYN_MODEL | GNP11 | ENUM | 0-10 | 0 | **6** | Motion dynamics model. 0=Portable, 6=Airborne 1G (birds). |
| GNSS_HACCFILT_THR | GNP21 | UINT | 0+ | 5 | **10** | Horizontal accuracy filter (meters). Accept fix if hAcc < this value. |
| GNSS_MIN_NUM_FIXES | GNP22 | UINT | 1+ | 1 | 1 | Consecutive valid fixes required before accepting. |
| GNSS_SESSION_SINGLE_FIX | GNP30 | BOOL | 0/1 | 0 | **1** | Stop GNSS after first successful fix in session. Saves battery. |

## Argos Satellite TX

| Parameter | DTE Key | Type | Range | Default | RSPB | Description |
|-----------|---------|------|-------|---------|------|-------------|
| TR_NOM | ARP05 | UINT | 30-1200 | 60 | 60 | Normal mode: interval between TX messages (seconds). |
| NTRY_PER_MESSAGE | ARP19 | UINT | 0-86400 | 6 | **3** | Number of times each message is repeated on air. |
| ARGOS_RX_EN | ARP32 | BOOL | 0/1 | 1 | **0** | Enable satellite downlink (RX). Disable to save ~3.75 mAh/cycle. |

## Low Battery Mode

| Parameter | DTE Key | Type | Range | Default | RSPB | Description |
|-----------|---------|------|-------|---------|------|-------------|
| LB_EN | LBP01 | BOOL | 0/1 | 0 | **1** | Enable low battery mode. When battery < threshold, switch to LB behavior. |
| LB_THRESHOLD | LBP02 | UINT | 0-100 | 10 | 10 | Battery percentage to enter Low Battery mode. |
| LB_GNSS_EN | LBP06 | BOOL | 0/1 | 1 | **0** | Enable GNSS in LB mode. 0 = Doppler-only (no GPS position). |
| LB_SHUTDOWN_NTIME_SAT | LBP14 | UINT | 0-65535 | 0 | **2** | Number of Doppler TX per LB session before powerdown. 0=disabled. |
| TR_LB | ARP06 | UINT | 30-1200 | 240 | **90** | LB mode: interval between Doppler TX (seconds). |
| LB_CRITICAL_THRESH | LBP12 | UINT | 0-100 | 5 | 5 | Battery SOC (%) for critical mode. Below this = immediate poweroff. |

## Underwater / Surface Detection

| Parameter | DTE Key | Type | Range | Default | RSPB | Description |
|-----------|---------|------|-------|---------|------|-------------|
| UNDERWATER_EN | UNP01 | BOOL | 0/1 | 0 | **0** | Underwater detection. Disabled for aerial birds. |
| DRY_TIME_BEFORE_TX | UNP02 | UINT | 0+ | 1 | **0** | Seconds at surface before TX allowed. 0 = TX immediately. |

## LEDs

| Parameter | DTE Key | Type | Range | Default | RSPB | Description |
|-----------|---------|------|-------|---------|------|-------------|
| LED_MODE | LDP01 | ENUM | 0-3 | 1 | **0** | Status LED. 0=OFF, 1=24h mode. Disabled on birds (invisible + wastes energy). |
| EXT_LED_MODE | LDP02 | ENUM | 0-3 | 3 | **0** | External LED. 0=OFF. Disabled on birds. |

## Mortality Detection

Requires `ENABLE_MORTALITY_SENSOR=1` (auto-enabled on RSPB). Combines AXL activity, body temperature, and GPS stationarity to compute a mortality confidence percentage (0-100%).

**WARNING:** Mortality detection requires AXL (`AXP01=1`), Thermistor (`THP01=1`), and GNSS (`GNP01=1`) to be enabled. If any sensor is disabled, the algorithm works with partial data only (biased toward ALIVE).

| Parameter | DTE Key | Type | Range | Default | RSPB | Description |
|-----------|---------|------|-------|---------|------|-------------|
| MORTALITY_ENABLE | MTP01 | BOOL | 0/1 | 0 | **1** | Enable mortality detection. |
| MORTALITY_ACTIVITY_THRESH | MTP02 | UINT | 0-255 | 10 | 10 | Activity score below which = immobile. |
| MORTALITY_TEMP_THRESH | MTP03 | FLOAT | 0-60 | 25.0 | 25.0 | Body temp below which = hypothermic (°C). Live bird ~40°C. |
| MORTALITY_GPS_DISTANCE_THRESH | MTP04 | UINT | 0-10000 | 50 | 50 | GPS movement below which = stationary (meters). |
| MORTALITY_CONFIRM_DAYS | MTP05 | UINT | 1-30 | 3 | 3 | Days of high confidence before CONFIRMED. |
| MORTALITY_DUTY_CYCLE_MODULO | MTP06 | UINT | 0-100 | 0 | **6** | New BOOT_COUNTER_MODULO when confirmed. 0=never modify. |
| MORTALITY_ORIGINAL_MODULO | MTP07 | UINT | - | 0 | - | Backup of original modulo (auto, read-only). |

### Scoring Algorithm

Each active session computes a **session score** (0-100) from three independent indicators:

| Indicator | Points | Condition |
|-----------|--------|-----------|
| Immobility | 40 pts | AXL activity < `MORTALITY_ACTIVITY_THRESH` (default 10) |
| Hypothermia | 30 pts | Thermistor temp < `MORTALITY_TEMP_THRESH` (default 25°C) |
| Stationarity | 30 pts | GPS moved < `MORTALITY_GPS_DISTANCE_THRESH` (default 50m) AND speed < 0.1 m/s |

**Confidence** is computed as an exponential moving average:
```
confidence = (old_confidence * 7 + session_score * 3) / 10
```
70% weight on history, 30% on current session. This smooths out single bad readings.

### Status State Machine

```
                    confidence >= 50%
    ALIVE ─────────────────────────────> SUSPECTED
      ^                                      |
      |           confidence < 50%           |
      +──────────────────────────────────────+
      |                                      |
      |     consecutive_days >= CONFIRM_DAYS |
      |                                      v
      +──────────────────────────── CONFIRMED
              bird shows life again
```

**Daily evaluation** (once per UTC calendar day):
- If `confidence >= 80%` → increment `consecutive_days`
- Otherwise → decrement `consecutive_days` (floor at 0)
- If `consecutive_days >= MORTALITY_CONFIRM_DAYS` (default 3) → status = **CONFIRMED**

### Duty Cycle Adaptation on Mortality

When mortality is **CONFIRMED** and `MORTALITY_DUTY_CYCLE_MODULO > 0` (RSPB only):
1. Save current `BOOT_COUNTER_MODULO` to `MORTALITY_ORIGINAL_MODULO`
2. Set `BOOT_COUNTER_MODULO = MORTALITY_DUTY_CYCLE_MODULO` (e.g., 6)
3. Device wakes less frequently → saves battery on dead bird

If the bird **recovers** (status returns to ALIVE):
1. Restore `BOOT_COUNTER_MODULO` from `MORTALITY_ORIGINAL_MODULO`
2. Clear `MORTALITY_ORIGINAL_MODULO` to 0

### State Persistence (FsLog)

Mortality state (confidence, consecutive_days, status, last positions) is **persisted to flash**
via a dedicated FsLog partition (`"MORTALITY"`, 64KB). On each evaluation, a `MortalityLogEntry`
is written. On boot, the last entry is read to restore state across TPL5111 power cycles.

This ensures the confirmation counter survives the complete power-off between wakeups.

### Sensor Input Requirements

The service listens to peer events from:
- **AXL sensor** → caches activity level (port[4])
- **Thermistor** → caches body temperature
- **GNSS** → caches position + speed

Evaluation triggers when GPS data is available AND at least one other sensor has reported.
If a sensor is disabled, its indicator scores 0 (biased toward ALIVE). A dead bird will
eventually be detected across subsequent sessions even with partial data.

### Mortality Log

The service writes a `MortalityLogEntry` to `mortality.log` on each evaluation, readable via `$DUMPD`:

| Field | Type | Description |
|-------|------|-------------|
| confidence | uint8 | Current confidence (0-100%) |
| consecutive_days | uint8 | Days with high confidence |
| status | uint8 | 0=ALIVE, 1=SUSPECTED, 2=CONFIRMED |
| last_activity | uint8 | Last AXL activity reading (0-255) |
| last_body_temp | uint16 | Last thermistor reading (raw) |
| last_lat | double | Last GPS latitude |
| last_lon | double | Last GPS longitude |
| last_eval_epoch | uint32 | Epoch of last daily evaluation |

### Safety Guarantees

1. **Compile-time**: `ENABLE_MORTALITY_SENSOR=OFF` (default) → zero code on non-RSPB boards
2. **Runtime**: `MORTALITY_ENABLE=false` (default) → service disabled, zero impact
3. **Duty cycle**: `MORTALITY_DUTY_CYCLE_MODULO=0` (default) → never touches duty cycle

**RSPB Compact Sensor Packet (133/192 bits):**
```
[Time 16b][GPS 51b][Battery 8b][Pressure 29b][Thermistor 14b][Activity 8b][Mortality% 7b]
```
Mortality confidence (7 bits, 0-100%) is **always transmitted** when the sensor is enabled,
regardless of status — this allows satellite operators to observe the confidence trend.

Note: AXL X/Y/Z axes and AXL temperature are dropped for RSPB (useless for birds). Only the activity score (8 bits) is kept.

## Device Identity

| Parameter | DTE Key | Type | Default | RSPB | Description |
|-----------|---------|------|---------|------|-------------|
| PROFILE_NAME | IDP11 | TEXT | FACTORY | RSPB_DEPLOY_V1 | Human-readable deployment profile name. |


---

# How to Configure a Device

Parameters are sent via BLE (Bluetooth) or USB using DTE commands.
Each command starts with `$PARMW#` followed by the hex-encoded length of the
payload, then a semicolon, then key=value pairs separated by commas.

## RSPB Deployment Commands

Send these commands one by one via BLE (LinkIt app) or USB terminal:

```
 GNSS timeouts (faster, save battery on GPS failure)
$PARMW#00E;GNP09=180,GNP05=90\r

 Safety shutdown timer (10 minutes max per cycle)
$PARMW#00A;PWP01=600\r

 Duty cycle: active every 4th wakeup (~7 hours)
$PARMW#009;PWP03=4\r

 GNSS for bird flight (airborne model, relaxed accuracy)
$PARMW#015;GNP11=6,GNP21=10,GNP22=1\r

 Argos TX: no RX, 3 repetitions, no surface wait
$PARMW#016;ARP32=0,ARP19=3,UNP02=0\r

 Disable underwater detection
$PARMW#009;UNP01=0\r

 LEDs off (invisible on bird, save energy)
$PARMW#00E;LDP01=0,LDP02=0\r

 Session shutdown: 3 TX then powerdown, no GNSS re-acquisition
$PARMW#013;PWP05=3,GNP30=1\r

 Low battery: enable, no GPS, 2 Doppler TX, 90s interval
$PARMW#01C;LBP01=1,LBP06=0,LBP14=2,ARP06=90\r

 Profile name
$PARMW#015;IDP11=RSPB_DEPLOY_V1\r
```

## Reading Parameters

To verify a parameter was set correctly:
```
$PARMR#005;PWP05\r
```
Response: `$O;PARMR#007;PWP05=3\r`

To read all parameters at once:
```
$PARMR#000;\r
```


---

# Estimated Energy Budget (RSPB Deployment)

| Phase | Duration | Current | Energy per cycle |
|-------|----------|---------|-----------------|
| Boot + modulo check (skip) | ~50ms | 5 mA | ~0.07 uAh |
| Boot + pre-operational | ~0.2s | 5 mA | ~0.3 uAh |
| GNSS acquisition (typical) | ~45s | 25 mA | ~312 uAh |
| GNSS acquisition (worst: cold) | 180s | 25 mA | ~1250 uAh |
| Satellite TX (x3, 60s interval) | ~180s | 15 mA avg | ~750 uAh |
| **Total per active cycle (typical)** | **~3 min** | - | **~1.1 mAh** |
| **Total per active cycle (worst)** | **~6 min** | - | **~2.3 mAh** |

With modulo=4 (one active cycle per ~7h): **~3-8 mAh per day**
NCR18650 battery (3400 mAh): theoretical **~400-1000 days** without solar.
With solar panels: effectively indefinite in good conditions.


---

# Troubleshooting

## Device not transmitting
- Check `SHUTDOWN_NTIME_SAT` (PWP05) is not 0 — if 0, only SHUTDOWN_TIMER controls session end
- Check `GNSS_EN` (GNP01) is 1
- Check `ARGOS_RX_EN` (ARP32) — if 1, device spends 15min listening before TX

## Device transmitting but no GPS position
- Check GNSS timeouts: `GNP09` (cold) and `GNP05` (warm) may be too short
- Check `GNSS_HACCFILT_THR` (GNP21) — if too low, fixes are rejected
- Check `GNSS_DYN_MODEL` (GNP11) — wrong model can prevent lock

## Battery draining too fast
- Increase `BOOT_COUNTER_MODULO` (PWP03) for less frequent cycles
- Reduce `SHUTDOWN_NTIME_SAT` (PWP05) for fewer TX per session
- Ensure `ARGOS_RX_EN` (ARP32) is 0
- Ensure LEDs are off: `LDP01=0`, `LDP02=0`
- Reduce `GNSS_COLD_ACQ_TIMEOUT` (GNP09) to fail faster on bad GPS conditions

## Low battery mode not activating
- Check `LB_EN` (LBP01) is 1
- Check `LB_THRESHOLD` (LBP02) — battery % must be below this value

## Device stuck (never powers down)
- `SHUTDOWN_TIMER` (PWP01) should be > 0 as a safety net
- Recommended: 600s (10 minutes)
