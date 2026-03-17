# 7 - Application Behavior

This page describes the runtime behavior of each application mode. The firmware adapts its behavior based on the board variant (LinkIt V4 vs RSPB) and the deployment scenario (underwater marine tracking vs aerial bird tracking).

---

## 1 - LinkIt V4 Underwater Mode

### Overview

When `UNDERWATER_EN=1`, the tracker detects whether the animal is submerged or at the surface. This is critical for marine deployments (turtles, seals, diving birds): satellite TX is impossible underwater, so the device suspends all transmissions while submerged and resumes only after the animal surfaces.

```
                         SURFACE                      UNDERWATER
                    +-----------------+            +-----------------+
                    |  GNSS active    |            |  GNSS suspended |
                    |  Argos TX OK    |  SWS wet   |  Argos TX OFF   |
                    |  Sensors active | ---------> |  Sensors active |
                    |                 |            |  (reduced rate)  |
                    |                 |  SWS dry   |                 |
                    |                 | <--------- |                 |
                    +-----------------+            +-----------------+
                           |                              |
                           v                              v
                     After DRY_TIME_BEFORE_TX       deschedule() all
                     --> reschedule Argos TX         Argos & GNSS tasks
```

### How It Works

1. The **UW Detector Service** samples the saltwater switch at a configurable interval (`SAMPLING_SURF_FREQ` at surface, `SAMPLING_UNDER_FREQ` underwater)
2. When a state change is confirmed, the scheduler is notified via `notify_underwater_state()`
3. **Submerged**: Argos TX and GNSS are descheduled
4. **Surfaced**: After `DRY_TIME_BEFORE_TX` seconds, Argos TX is rescheduled and GNSS can acquire a new fix

### Scheduler Integration

**ArgosScheduler** (`notify_underwater_state`):
- **Submerged** (`state=true`): calls `deschedule()` — all pending Argos TX are cancelled
- **Surfaced** (`state=false`): sets `m_earliest_tx = now + dry_time_before_tx`, then `reschedule()`

**GPSScheduler** (`notify_underwater_state`):
- **Submerged**: cancels ongoing GNSS acquisition, suspends scheduling
- **Surfaced**: resumes GNSS scheduling, allows new fix acquisition

### Detection Sources

| Source | ID | Description |
|--------|----|-------------|
| SWS | 0 | Saltwater switch (analog conductivity electrode) |
| PRESSURE_SENSOR | 1 | Pressure sensor (depth > threshold) |
| GNSS | 2 | GNSS signal quality (poor signal = submerged) |
| SWS_GNSS | 3 | Hybrid: SWS for transitions, GNSS for surface confirmation |

For marine turtle deployments, **SWS** is the recommended and most robust method.

### Interaction with Battery Modes

- **Normal mode**: underwater suspends both GNSS and Argos TX
- **Low Battery mode**: underwater suspends Doppler TX
- **Critical mode**: device is off regardless

### Underwater Detection Parameters

| Parameter | DTE Key | Type | Default | Description |
|-----------|---------|------|---------|-------------|
| UNDERWATER_EN | UNP01 | BOOL | 0 | Enable underwater detection |
| DRY_TIME_BEFORE_TX | UNP02 | UINT | 0 | Seconds at surface before TX allowed |
| SAMPLING_UNDER_FREQ | UNP03 | UINT | 10 | Sampling interval underwater (seconds) |
| SAMPLING_SURF_FREQ | UNP04 | UINT | 10 | Sampling interval at surface (seconds) |
| UW_MAX_SAMPLES | UNP05 | UINT | 1 | Samples per detection cycle |
| UW_MIN_DRY_SAMPLES | UNP06 | UINT | 1 | Consecutive dry samples to confirm surface |
| UW_SAMPLE_GAP | UNP07 | UINT | 1000 | Gap between batch sub-samples (ms) |
| UW_PIN_SAMPLE_DELAY | UNP08 | UINT | 1 | RC charge time before ADC read (ms). Keep at 1. |
| UNDERWATER_DETECT_SOURCE | UNP10 | ENUM | 0 | 0=SWS, 1=Pressure, 2=GNSS, 3=SWS+GNSS |
| UNDERWATER_DETECT_THRESH | UNP11 | FLOAT | 1.1 | Threshold for pressure/GNSS methods |
| UW_MAX_DIVE_TIME | UNP24 | UINT | 7200 | Force surface after this duration (seconds) |
| UW_MIN_SURFACE_TIME | UNP25 | UINT | 2 | Surface lockout after detection (seconds) |

### SWS Analog Calibration Parameters

| Parameter | DTE Key | Type | Default | Description |
|-----------|---------|------|---------|-------------|
| SWS_ANALOG_THRESHOLD_MIN | UNP20 | UINT | 0 | Minimum valid ADC reading |
| SWS_ANALOG_THRESHOLD_MAX | UNP21 | UINT | 8000 | Max valid ADC (legacy, superseded by observed peak) |
| SWS_ANALOG_HYSTERESIS | UNP22 | UINT | 4 | Hysteresis as % of threshold |
| SWS_ANALOG_CALIB_INTERVAL | UNP23 | UINT | 3600 | Air baseline recalibration interval (seconds) |

---

## 2 - SWS Analog Algorithm

The SWS analog algorithm (`SWSAnalogService`) is the primary underwater detection method for marine deployments. It uses RC time constant discrimination to measure the electrical conductivity between two electrodes.

### Hardware: RC Discrimination

A 100nF capacitor charges through the medium when the electrode is enabled. The charge rate depends on the medium's resistance:

```
     VDD
      |
   [Electrode TX] ──SWS_ENABLE_PIN──> [salt water] ──> [Electrode RX]
                                                             |
                                                          100nF cap
                                                             |
                                                          ADC 14-bit
                                                             |
                                                          1M pull-down
                                                             |
                                                            GND
```

```
  Enable SWS pin -> wait 1ms -> read 14-bit ADC -> disable SWS pin

  Water (R~10k, tau=1ms):   at 1ms = 63% charged -> ADC ~2500-3000
  Wet film (R~50k, tau=5ms): at 1ms = 18% charged -> ADC ~500-1000
  Air (R=inf):               at 1ms = 0% charged  -> ADC ~0-300
```

The 1ms sampling point maximizes discrimination between water and wet electrode film. **The pin_sample_delay must remain fixed at 1ms** — it is a physical constant of the circuit.

### Auto-Calibration

**Initial calibration (at boot):**

```
  Boot -> Take 10 ADC samples, 100ms apart
  avg > 2500? --> Started in water: water=avg, air=avg/3
  avg <= 2500? --> Started in air: air=avg, water=min(air*3, observed_peak)
  -> Calculate threshold and store in noinit RAM with CRC16
```

**Dynamic water baseline (EMA):** When confirmed underwater, the water baseline adapts:
```
new_water = 0.19 * current_reading + 0.81 * old_water
```
Protected by strict conditions: value > threshold + hysteresis, value >= 2000, value >= air*3 (bypassed if air >= 1000), value >= 85% of water baseline, capped at observed peak ADC.

**Air baseline adaptation (at surface > 10s):**

Uses a 10-sample ring buffer of surface readings. Adaptation requires at least 5 readings before activating. Downward adaptation is intentionally faster (20%) because an inflated air baseline (from wet electrode calibration) is more harmful than a slightly low one.

| Direction | Trigger | Formula | Rate |
|-----------|---------|---------|------|
| Upward (biofouling) | avg > air * 1.3 | air = 0.9\*air + 0.1\*avg | 10%/update |
| Downward (wet calib fix) | avg < air * 0.7 | air = 0.8\*air + 0.2\*avg | 20%/update |
| Timed recalibration | elapsed > 3600s | air = avg | 100% |

Calibration data (air, water, threshold, hysteresis, timestamp) is stored in noinit RAM with CRC16 (survives MCU soft reset). The observed peak ADC has a separate CRC16 and noinit storage — both are validated independently at boot.

### Dynamic Threshold

The threshold is positioned as a percentage between air and water baselines:

```
  threshold = air + (water - air) * ratio
  hysteresis = threshold * 4%

  Clean (contrast >= 8x):  ratio = 35%  -> threshold close to air
  Moderate (contrast >= 4x): ratio = 50% -> midpoint
  Low contrast (< 4x):    ratio = 40%  -> closer to air for detection
```

The observed peak ADC (highest value ever read) caps water baseline estimates and threshold+hysteresis to prevent the threshold from exceeding values the ADC can actually produce.

### 5-Level Surface Detection

Five independent methods detect surfacing, ordered from fastest to slowest. All require >= 1s underwater and proximity guard OK (filtered < 95% of peak).

| Level | Speed | Condition | Use Case |
|-------|-------|-----------|----------|
| **L1** | 1 sample | Drop > 5% from recent peak | Sharp water exit |
| **L2** | 2 samples | 2 consecutive drops, cumulative > 3% | Gradual exit |
| **L3** | 3+ samples | MA3 trend down 3x, total > 5% | Noisy signal |
| **L4** | Variable | filtered < water * 85% | Absolute reference |
| **L5** | >10s | Drop > 15% from dive peak | Safety net |

**Recent peak tracking (L1):** The recent peak decays toward the current reading at 5% per sample: `recent_peak = 0.95 * recent_peak + 0.05 * reading`. This prevents L1 from becoming stale if the signal drifts slowly while underwater.

**MA3 noise tolerance (L3):** The trend counter only decrements by 1 on a MA3 increase (not full reset). This allows one noise bounce without breaking the trend detection — 3 consecutive decreases are still required for trigger.

When any level triggers: state forced to surface, air baseline recalibrated (if `filtered > 2x air` AND `filtered < 80% water`), surface lockout activated.

### Detection Algorithm Flow

Each call to `detector_state()`:

1. Read ADC (enable SWS pin, wait 1ms, read 14-bit ADC, disable)
2. First-sample coherence check (once after boot)
3. Update observed peak ADC
4. Filtering (moving average, window=2)
5. Time tracking + max dive timeout check
6. 5-level surface detection (L1-L5 with proximity guard)
7. Recent peak & dive peak tracking
8. Surface baseline tracking (air adaptation)
9. State determination (hysteresis threshold)
10. Apply surface override (L1-L5 forced surface + recalibrate)
11. Max dive timeout (force surface + 30s lockout)
12. Surface lockout enforcement
13. State change logging

### Safety Mechanisms

- **Max Dive Time** (default 7200s = 2h): Forces SURFACE, activates 30s lockout. Does NOT recalibrate air baseline.
- **Min Surface Time** (default 2s): Ignores underwater for 2s after surface detection. Prevents oscillation.
- **Surface Lockout** (30s): After max dive timeout. Water baseline NOT updated during lockout.
- **Proximity Guard** (95%): L1-L5 overrides blocked if filtered >= 95% of max(water, dive_peak).

### Session Timeline Examples

**Normal dive cycle (clean electrode):**

```
  T+0s     SWS reads ADC = 180 (air) -> SURFACE
  T+10s    Turtle dives, ADC = 2900 > threshold_high -> UNDERWATER
           ArgosScheduler: deschedule(), GPSScheduler: suspend
  T+70s    ADC = 2850, water baseline EMA: 2900 -> 2890
  T+130s   Turtle surfaces, ADC drops 2900 -> 1500
           L1: 48% drop > 5% -> SURFACE! Air recalibrated to 1500
           ArgosScheduler: reschedule(), GPSScheduler: resume
  T+145s   ADC = 300 (dried), air adapts down: 1500 -> 1260
```

**Dive with biofouling:**

```
  T+0s     Air ~1000 (biofilm), Water ~2800, contrast 2.8x, ratio=40%
           threshold = 1000 + 1800*0.40 = 1720
  T+10s    Turtle dives, ADC = 2750 > 1789 -> UNDERWATER
  T+70s    Surfaces, ADC drops 2780 -> 2100
           L1: 24% > 5% -> SURFACE in 1s
  T+80s    Upward air adaptation: 1000 -> 1010
```

### DTE Test Commands

- `$SWSST` — Read SWS status (air, water, threshold, hysteresis, raw_adc, filtered_adc, calibrated, underwater, time_in_state, surface_level, contrast_x10, observed_peak)
- `$SWSTST,1` — Start test mode (continuous sampling with async SWSST push and RGB LED feedback: BLUE=underwater, YELLOW=surface)
- `$SWSTST,0` — Stop test mode

### Internal Constants Reference

| Constant | Value | Description |
|----------|-------|-------------|
| ADC_HISTORY_SIZE | 2 | Moving average filter window |
| DEFAULT_THRESHOLD_RATIO_PERCENT | 35% | Threshold position (clean electrode) |
| DEFAULT_ALPHA_PERCENT | 19% | EMA alpha for water baseline |
| ABSOLUTE_MIN_WATER_ADC | 2000 | Reject water readings below this |
| MIN_WATER_AIR_RATIO | 3 | Water must be >= 3x air |
| L1_DROP_PERCENT | 5% | Instant surface trigger |
| L2_DROP_PERCENT | 3% | Cumulative 2-sample trigger |
| L3_DROP_PERCENT | 5% | MA3 trend trigger |
| L4_DROP_PERCENT | 15% | Absolute water baseline trigger |
| L5_DROP_PERCENT | 15% | Dive peak trigger (>10s gate) |
| PROXIMITY_GUARD_PERCENT | 95% | L-override block threshold |
| RECENT_PEAK_DECAY | 5% | L1 recent peak decay per sample |
| SURFACE_BUFFER_SIZE | 10 | Ring buffer for surface readings |
| SURFACE_BUFFER_MIN_COUNT | 5 | Minimum readings before air adaptation |
| SURFACE_OVERRIDE_AIR_MIN | 2x air | Guard: override must exceed 2x air baseline |
| SURFACE_OVERRIDE_WATER_MAX | 80% water | Guard: override must be below 80% water baseline |
| SURFACE_LOCKOUT_DURATION_SEC | 30 | Lockout after max dive timeout |
| WATER_DETECT_HEURISTIC | 2500 | Initial calibration water/air threshold |

Source files: `core/services/sws_analog_service.hpp/.cpp`, `core/services/uwdetector_service.hpp/.cpp`

### Troubleshooting (SWS)

| Symptom | Cause | Fix |
|---------|-------|-----|
| Never detects underwater | threshold_high > water ADC | Check air baseline via `$SWSST`. Inflated air = wet calibration, will auto-correct. |
| Always thinks underwater | Biofouling or wet calibration | Reduce `UW_MAX_DIVE_TIME`. Downward air adaptation should auto-correct. |
| Never detects dives | threshold too high, or peak=0 | Do one immersion to learn peak. Verify `UW_PIN_SAMPLE_DELAY`=1. |
| Slow surface detection | Low contrast or proximity guard | Lower `SAMPLING_UNDER_FREQ` to 1s. L1 should trigger for >5% drop. |
| Threshold exceeds water readings | observed_peak_adc caps not working | Check `$SWSST` observed_peak. First immersion needed if 0. |
| Not transmitting after surfacing | DRY_TIME_BEFORE_TX too high | Check param UNP02. Verify `UNDERWATER_EN` is enabled. |

---

## 3 - Surfacing Burst Mode (ARGOS_MODE=5)

### Overview

Surfacing Burst mode (`ARGOS_MODE=5`, `SURFACING_BURST`) combines Doppler and GNSS transmissions for marine animals that surface briefly (turtles, seals). On surfacing, the device immediately sends rapid Doppler messages with progressive intervals, allowing Kineis/Argos satellites to compute a Doppler-based position while the GPS acquires a fix. Once a GNSS fix is available, the service switches to standard GNSS transmissions.

This mode is specifically designed for deployments where:
- Animals surface for short periods (minutes)
- GPS cold start may take 30-120 seconds
- Doppler localization requires multiple spaced messages
- Every surfacing event should produce at least some position data

### Behavior

```
SURFACE DETECTED
  |
  +-- Phase 1: DOPPLER BURST (progressive intervals)
  |     T=0s                         -> TX Doppler #1 (immediate, no TCXO warmup)
  |     T=INIT_S                     -> TX Doppler #2
  |     T+=INIT_S + STEP_S           -> TX Doppler #3
  |     T+=INIT_S + 2*STEP_S         -> TX Doppler #4
  |     ...                          -> capped at MAX_S
  |
  +-- GNSS FIX ACQUIRED (GPS service notifies valid position)
  |     |
  |     v
  +-- Phase 2: GNSS TX (standard interval)
  |     Uses TR_NOM interval with GNSS/sensor packets
  |     Continues until next dive
  |
  +-- DIVE DETECTED -> reset, ready for next surfacing
```

### Progressive Interval Formula

```
Interval(N) = min(INIT_S + (N-1) * STEP_S, MAX_S)

Example with INIT_S=5, STEP_S=1, MAX_S=30:
  MSG#1:  0s  (immediate)
  MSG#2:  5s  after MSG#1
  MSG#3:  6s  after MSG#2
  MSG#4:  7s  after MSG#3
  ...
  MSG#27: 30s after MSG#26 (cap reached)
  MSG#28: 30s after MSG#27 (stays at cap)
```

### Parameters

| Parameter | DTE Key | Default | Range | Description |
|-----------|---------|---------|-------|-------------|
| ARGOS_MODE | ARP01 | 2 | 0-5 | Set to 5 for Surfacing Burst mode |
| SURFACING_BURST_INIT_S | ARP40 | 5 | 5-120 | Initial Doppler interval (seconds) |
| SURFACING_BURST_STEP_S | ARP41 | 1 | 0-30 | Interval increment per message (seconds) |
| SURFACING_BURST_MAX_S | ARP42 | 30 | 5-300 | Maximum Doppler interval cap (seconds) |
| TR_NOM | ARP05 | 60 | 30-1200 | GNSS TX interval after fix (seconds) |
| DRY_TIME_BEFORE_TX | UNP02 | 0 | 0-max | Delay before first TX after surfacing |

### Key Behaviors

- **TCXO warmup** is automatically skipped on the first TX after surfacing (saves ~5s)
- **time_sync_burst** is ignored in Surfacing Burst mode (Doppler first, not GNSS)
- **DRY_TIME_BEFORE_TX** still applies to the first Doppler message if configured
- **Duty cycle** is NOT enforced during Doppler phase (goal is rapid TX), only in GNSS phase
- **Low Battery / Zone modes** can also use SURFACING_BURST (LBP04=5, ZOP11=5)
- If no GNSS fix is acquired before the next dive, only Doppler data is available

### Session Timeline Example

**Typical turtle surfacing (GPS fix in 45s):**

```
  T+0s     SWS detects surface -> SURFACING_BURST activated
  T+0s     Doppler TX #1 (immediate, no TCXO warmup, 3 bytes)
  T+5s     Doppler TX #2
  T+11s    Doppler TX #3
  T+18s    Doppler TX #4
  T+26s    Doppler TX #5
  T+35s    Doppler TX #6
  T+45s    GPS fix acquired! -> switch to GNSS phase
  T+105s   GNSS TX #1 (96 bits short packet with position, +60s TR_NOM)
  T+165s   GNSS TX #2
  T+200s   Turtle dives -> all TX suspended, burst state reset
           Total: 6 Doppler + 2 GNSS messages in 200s surfacing
```

**Short surfacing (no GPS fix):**

```
  T+0s     Surface -> SURFACING_BURST activated
  T+0s     Doppler TX #1
  T+5s     Doppler TX #2
  T+11s    Doppler TX #3
  T+15s    Turtle dives -> burst reset
           Total: 3 Doppler messages (satellite computes Doppler position)
```

### DTE Configuration Example

```
# Enable Surfacing Burst mode
$PARMW#007;ARP01=5\r

# Progressive Doppler: 5s initial, +1s step, max 30s
$PARMW#015;ARP40=5,ARP41=1,ARP42=30\r

# GNSS phase: 60s interval
$PARMW#008;ARP05=60\r

# Enable underwater detection with SWS
$PARMW#007;UNP01=1\r

# No dry time delay (immediate TX on surfacing)
$PARMW#007;UNP02=0\r
```

---

## 4 - RSPB Bird Tracker Behavior

### Overview

The RSPB tracker is a wildlife GPS/Satellite tracker designed for bird monitoring. It uses a **nano-power architecture**: the device spends most of its time completely powered off. An external timer chip (TPL5111) periodically wakes the tracker to perform a short operational cycle, then the device powers down again.

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
         v                    v                        v
   Not our turn?       Battery critical?        TX limit reached?
   --> POWER OFF       --> POWER OFF            --> POWER OFF
```

### Duty Cycling with TPL5111

The TPL5111 is an external nano-power timer that physically controls the power supply. When the tracker powers down, the TPL5111 keeps counting and re-powers the device after its configured interval (~1h45min).

`BOOT_COUNTER_MODULO` controls how many wakeups are skipped. The counter increments on each boot, and when `counter % modulo == 0`, an active cycle runs. The counter is automatically cleared to 0 after a successful modulo check. Protection: if counter exceeds `modulo + 1` (corrupted flash), it is reset. Minimum modulo value is 2.

```
 Wakeup 1     Wakeup 2     Wakeup 3     Wakeup 4
  (skip)       (skip)       (skip)      ACTIVE!   ...
    |            |            |            |
 increment    increment    increment    modulo=4
 counter +    counter +    counter +    --> RUN!
 power off    power off    power off    counter=0
                                        GPS + TX

 With modulo=4 and 1h45 interval: Active cycle every ~7 hours
```

### Battery Modes

```
  100% |########################################|
       |             NORMAL MODE                 |  GNSS, Sensors, Satellite TX
  LB%  |---- LB_THRESHOLD (default: 10%) -------|
       |           LOW BATTERY MODE              |  Doppler TX only (no GPS)
  LBP12|---- LB_CRITICAL_THRESH (default: 5%)   |
       |          CRITICAL MODE                  |  Immediate power off
   0%  |_________________________________________|
```

**Mode 1 — Normal Operation** (battery > LB_THRESHOLD):

1. GNSS module powers on, acquires one fix (`GNSS_SESSION_SINGLE_FIX=1`)
2. GNSS stops after first fix (saves power)
3. Sends `SHUTDOWN_NTIME_SAT` satellite messages (default: 3)
4. Each message contains GPS position + sensor data, spaced by `TR_NOM` (default: 60s)
5. After last message, device powers down

**Mode 2 — Low Battery** (battery < LB_THRESHOLD, `LB_EN=1`):

1. No GNSS, no sensors
2. Sends `LB_SHUTDOWN_NTIME_SAT` Doppler messages (default: 2)
3. Each Doppler message contains only battery voltage (24 bits / 3 bytes)
4. Argos satellites use Doppler frequency shift to estimate position
5. Messages spaced by `TR_LB` (default: 90s)

**Mode 3 — Critical Battery** (battery < LB_CRITICAL_THRESH):

Immediate power off. No GPS, no TX. TPL5111 keeps waking the device — once solar recharges above critical threshold, operation resumes. **Exception:** if a magnet is detected (reed switch) while in critical battery state, the device enters configuration mode instead of powering off. This allows reconfiguring a depleted device via BLE without waiting for recharge.

### Safety Net: Shutdown Timer

`SHUTDOWN_TIMER` runs in parallel with the NTIME_SAT counter. Prevents the device from staying awake indefinitely if something goes wrong. Whichever triggers first (NTIME_SAT reached or timer expired) causes power down.

### Pseudo RTC (Timekeeping Without Battery Backup)

The TPL5111 architecture means the MCU loses all RAM state at each power-off, including the real-time clock. The firmware implements a **pseudo RTC chain** using a flash-persisted parameter.

```
  Boot N                                       Boot N+1
  Read LAST_KNOWN_RTC from flash               Read LAST_KNOWN_RTC from flash
  Add WAKEUP_PERIOD (+6300s)                   Add WAKEUP_PERIOD (+6300s)
  rtc->settime(approximate)                    rtc->settime(approximate)
  ... GNSS fix acquired ...                    ... GNSS fix acquired ...
  Save exact time to flash                     Save exact time to flash
```

**Chain initialization:** Either the first GNSS fix seeds LAST_KNOWN_RTC automatically, or manually via `$RTCW#00A;1708444800\r` before deployment.

**Accuracy:** Drifts by the difference between configured WAKEUP_PERIOD and actual TPL5111 interval. Each GNSS fix corrects drift completely.

### RSPB Session Timeline

**Normal mode (typical):**

```
  T+0s     TPL5111 wakes device, modulo check -> active cycle
  T+0.1s   Pre-operational: battery OK -> Normal mode
  T+1s     GNSS module on, searching...
  T+45s    First GPS fix! GNSS off (GNSS_SESSION_SINGLE_FIX=1)
  T+46s    Satellite TX #1 (GPS + sensor data)
  T+106s   Satellite TX #2 (+60s TR_NOM)
  T+166s   Satellite TX #3 -> SHUTDOWN_NTIME_SAT reached -> POWER OFF
           Total awake: ~2.8 minutes, next wakeup in ~7h
```

**Low battery mode:**

```
  T+0s     TPL5111 wakes, modulo check -> active
  T+0.1s   Battery LOW -> LB mode
  T+1s     Doppler TX #1 (battery voltage, 3 bytes)
  T+91s    Doppler TX #2 -> LB_SHUTDOWN_NTIME_SAT reached -> POWER OFF
           Total awake: ~1.5 minutes
```

### Energy Budget

| Phase | Duration | Current | Energy per cycle |
|-------|----------|---------|-----------------|
| Boot + modulo check (skip) | ~50ms | 5 mA | ~0.07 uAh |
| Boot + pre-operational | ~0.2s | 5 mA | ~0.3 uAh |
| GNSS acquisition (typical) | ~45s | 25 mA | ~312 uAh |
| GNSS acquisition (worst: cold) | 180s | 25 mA | ~1250 uAh |
| Satellite TX (x3, 60s interval) | ~180s | 15 mA avg | ~750 uAh |
| **Total per active cycle (typical)** | **~3 min** | - | **~1.1 mAh** |
| **Total per active cycle (worst)** | **~6 min** | - | **~2.3 mAh** |

With modulo=4 (~7h cycle): ~3-8 mAh/day. NCR18650 (3400 mAh): ~400-1000 days without solar.

### RSPB Parameters

**Power Management:**

| Parameter | DTE Key | Default | RSPB | Description |
|-----------|---------|---------|------|-------------|
| SHUTDOWN_TIMER | PWP01 | 0 | **900** | Safety timeout (seconds) |
| BOOT_COUNTER | PWP02 | 0 | - | Read-only boot count |
| BOOT_COUNTER_MODULO | PWP03 | 2 | **4** | Active cycle every N wakeups |
| WAKEUP_PERIOD | PWP04 | 6300 | - | Read-only TPL5111 period (seconds) |
| SHUTDOWN_NTIME_SAT | PWP05 | 0 | **3** | TX messages per normal session |
| LAST_KNOWN_RTC | PWP06 | - | - | Flash-persisted Unix timestamp |

**GNSS:**

| Parameter | DTE Key | Default | RSPB | Description |
|-----------|---------|---------|------|-------------|
| GNSS_ACQ_TIMEOUT | GNP05 | 120 | **90** | Warm start timeout (seconds) |
| GNSS_COLD_ACQ_TIMEOUT | GNP09 | 530 | **180** | Cold start timeout (seconds) |
| GNSS_DYN_MODEL | GNP11 | 0 | **6** | 6=Airborne 1G (birds) |
| GNSS_HACCFILT_THR | GNP21 | 5 | **10** | Accept fix accuracy (meters) |
| GNSS_SESSION_SINGLE_FIX | GNP30 | 0 | **1** | Stop GNSS after first fix |

**Low Battery:**

| Parameter | DTE Key | Default | RSPB | Description |
|-----------|---------|---------|------|-------------|
| LB_EN | LBP01 | 0 | **1** | Enable low battery mode |
| LB_THRESHOLD | LBP02 | 10% | 10% | Battery % to enter LB mode |
| LB_GNSS_EN | LBP06 | 1 | **0** | GNSS in LB (0=Doppler only) |
| LB_SHUTDOWN_NTIME_SAT | LBP14 | 0 | **2** | Doppler TX per LB session |
| TR_LB | ARP06 | 240 | **90** | Interval between Doppler TX (seconds) |
| LB_CRITICAL_THRESH | LBP12 | 5% | 5% | SOC for critical mode |

### Troubleshooting (RSPB)

| Symptom | Cause | Fix |
|---------|-------|-----|
| Not transmitting | SHUTDOWN_NTIME_SAT=0 | Set PWP05 > 0 |
| TX but no GPS position | Timeout too short | Increase GNP09 (cold) and GNP05 (warm) |
| Battery draining fast | Too frequent cycles | Increase PWP03 (modulo), reduce PWP05, disable ARP32 |
| LB mode not activating | LB_EN=0 | Set LBP01=1, check LBP02 threshold |
| Device stuck (never powers down) | No shutdown mechanism | Set PWP01 > 0 (recommended: 600s) |

---

## 5 - RSPB Deployment Configuration Example

Example deployment configuration for bird mortality detection:

```
# GNSS timeouts
$PARMW#012;GNP09=180,GNP05=90\r

# Safety shutdown timer (15 min)
$PARMW#009;PWP01=900\r

# Duty cycle: every 5th wakeup (~8h45)
$PARMW#007;PWP03=5\r

# GNSS for bird flight
$PARMW#019;GNP11=6,GNP21=10,GNP22=1\r

# Argos TX: no RX, 3 repetitions, 2min interval
$PARMW#021;ARP05=120,ARP32=0,ARP19=3,UNP02=0\r

# Disable underwater detection
$PARMW#007;UNP01=0\r

# LEDs off
$PARMW#00F;LDP01=0,LDP02=0\r

# Session: 5 TX then powerdown, single GNSS fix
$PARMW#00F;PWP05=5,GNP30=1\r

# Low battery: enable, no GPS, 2 Doppler TX, 3min interval
$PARMW#02A;LBP01=1,LBP02=30,LBP06=0,LBP14=2,ARP06=180\r

# Critical threshold
$PARMW#009;LBP12=3.3\r

# Sensors for mortality detection (AXL + Pressure + Thermistor)
$PARMW#015;AXP01=1,AXP05=ONESHOT\r
$PARMW#015;PRP01=1,PRP04=ONESHOT\r
$PARMW#015;THP01=1,THP06=ONESHOT\r

# Profile name
$PARMW#017;IDP11=RSPB_MORTALITY_V1\r
```

**Mortality detection indicators:**
1. Accelerometer: zero/constant acceleration = no movement
2. Pressure: constant value = no altitude change (stays on ground)
3. Thermistor: temperature dropping toward ambient = loss of body heat
4. GPS: same fixed position across consecutive sessions
