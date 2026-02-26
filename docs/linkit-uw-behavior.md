# LinkIt V4 - Underwater Mode & Saltwater Switch Behavior Guide

## Overview

When `UNDERWATER_EN=1`, the tracker detects whether the animal is submerged
or at the surface. This is critical for marine deployments (turtles, seals, diving
birds): satellite TX is impossible underwater, so the device suspends all
transmissions while submerged and resumes only after the animal surfaces.

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


---

## How It Works

1. The **UW Detector Service** samples the saltwater switch at a configurable
   interval (`SAMPLING_SURF_FREQ` at surface, `SAMPLING_UNDER_FREQ` underwater)
2. When a state change is confirmed, the scheduler is notified via
   `notify_underwater_state()`
3. **Submerged**: Argos TX and GNSS are descheduled (no satellite communication
   possible underwater)
4. **Surfaced**: After `DRY_TIME_BEFORE_TX` seconds, Argos TX is rescheduled
   and GNSS can acquire a new fix

### Scheduler Integration

When the UW detector confirms a state change, it fires a `service_complete()`
callback with the new state. The schedulers react as follows:

**ArgosScheduler** (`notify_underwater_state`):
- **Submerged** (`state=true`): calls `deschedule()` — all pending Argos TX are cancelled
- **Surfaced** (`state=false`): sets `m_earliest_tx = now + dry_time_before_tx`, then `reschedule()` — Argos TX resumes after the configured surface delay

**GPSScheduler** (`notify_underwater_state`):
- **Submerged**: cancels ongoing GNSS acquisition, suspends scheduling
- **Surfaced**: resumes GNSS scheduling, allows new fix acquisition


---

## Detection Sources

The tracker supports multiple methods to detect underwater state:

| Source | ID | Description |
|--------|----|-------------|
| SWS | 0 | Saltwater switch (analog conductivity electrode) |
| PRESSURE_SENSOR | 1 | Pressure sensor (depth > threshold) |
| GNSS | 2 | GNSS signal quality (poor signal = submerged) |
| SWS_GNSS | 3 | Hybrid: SWS for transitions, GNSS for surface confirmation |

For marine turtle deployments, **SWS (Saltwater Switch)** is the recommended
and most robust method.

### Interaction with Battery Modes

Underwater detection operates independently of battery mode:
- In **Normal mode**: underwater suspends both GNSS and Argos TX
- In **Low Battery mode**: underwater suspends Doppler TX
- In **Critical mode**: device is off regardless


---

## Saltwater Switch (SWS) Algorithm

The SWS analog algorithm (`SWSAnalogService`) is the primary underwater detection
method for marine deployments. It measures the electrical conductivity between
two electrodes on the tracker housing. Saltwater has much higher conductivity
than air, producing a higher ADC reading when submerged.

### Principle

```
                   AIR (dry)                    SALTWATER (submerged)
              +----------------+             +---------------------+
  Electrode   |  Low conductivity |           |  High conductivity   |
  reading:    |  ADC ~ 100-300    |           |  ADC ~ 2000-8000+    |
              +----------------+             +---------------------+

              |<-- air baseline -->|         |<-- water baseline -->|
                                   |         |
                          threshold = air + 35% * (water - air)
                                   |<-hyst-->|<-hyst-->|
                                   threshold_low   threshold_high
```

A single ADC read cycle works as follows:
1. Enable the SWS electrode pin (apply voltage)
2. Wait `UW_PIN_SAMPLE_DELAY` ms for capacitance settling
3. Read the 14-bit ADC (0-16383 range)
4. Disable the electrode pin (save power)


---

## Auto-Calibration

The algorithm learns the environment and adapts over time. This is essential
because biofouling (salt deposits, biofilm growth) gradually changes the
electrode's baseline readings.

### Initial Air Calibration (at boot)

```
  Boot
    |
    v
  Take 10 ADC samples in air, 100ms apart
    |
    v
  air_baseline = average of 10 samples (e.g., 200)
  water_baseline = air_baseline * 3 (initial estimate, e.g., 600)
    |
    v
  Calculate threshold and store in noinit RAM with CRC16
```

### Dynamic Water Baseline (Exponential Moving Average)

When confirmed underwater, the water baseline is updated using EMA:
```
  new_water = alpha * current_reading + (1 - alpha) * old_water
  (alpha = 0.19 by default)
```

The update is protected by strict conditions to prevent biofouling corruption:
- Value must be above threshold + hysteresis (truly underwater)
- Value must be >= 5x air baseline (true saltwater, not biofouling)
- Value must be >= 2000 ADC (absolute minimum for seawater)
- Value must be >= 85% of expected water baseline or higher

### Periodic Recalibration

- Every `SWS_ANALOG_CALIB_INTERVAL` seconds (default 3600s = 1h), the air
  baseline is recalibrated when at surface (10 new samples)

### Calibration Persistence

- Calibration data is stored in noinit RAM (survives MCU reset)
- Protected by CRC16 checksum — corrupt data triggers fresh calibration


---

## Dynamic Threshold

The threshold is positioned as a percentage between air and water baselines:

```
  threshold = air_baseline + (water_baseline - air_baseline) * ratio
  (ratio = 35% by default — closer to air for faster surface detection)

  hysteresis = threshold * 14%

  Example with air=200, water=2500:
    range    = 2300
    threshold = 200 + 2300 * 0.35 = 1005
    hysteresis = 1005 * 0.14 = 141
    threshold_low  = 1005 - 141 = 864   (below this -> SURFACE)
    threshold_high = 1005 + 141 = 1146  (above this -> UNDERWATER)
    between 864-1146 -> maintain current state (hysteresis zone)
```


---

## State Confirmation

To prevent jitter at the threshold boundary, state changes require multiple
consecutive samples:

```
  ADC > threshold_high for N consecutive samples -> UNDERWATER
  (N = UW_MAX_SAMPLES, default 1 = immediate dive detection)

  ADC < threshold_low for M consecutive samples -> SURFACE
  (M = UW_MIN_DRY_SAMPLES, default 2 = 2-second surface confirmation)

  ADC in hysteresis zone -> maintain current state
```

The base class `UWDetectorService` manages the sampling batches:
- It calls `detector_state()` up to `UW_MAX_SAMPLES` times per batch
- Samples within a batch are spaced by `UW_SAMPLE_GAP` ms (default 1000ms)
- If `UW_MIN_DRY_SAMPLES` consecutive dry readings are reached, the batch
  terminates early (no need to keep sampling)
- A state change only fires a callback if the new state differs from the
  current state (or on first detection)


---

## Multi-Layer Surface Detection

The algorithm uses four independent methods to detect surfacing, providing
robustness against biofouling (the main challenge for marine deployments):

### 1. Threshold Crossing (simple)

- ADC drops below `threshold_low` for `UW_MIN_DRY_SAMPLES` consecutive reads
- Works well with clean electrodes

### 2. Rapid ADC Drop Detection (fast, <2s)

Detects the instant loss of water contact when the animal surfaces. Even with
heavy biofouling, the relative drop is always significant because the
conductivity path through water is broken.

```
  Three tiers (fastest to most conservative):

  TIER 1 (1s detection):  >25% drop AND >300 ADC absolute  -> immediate SURFACE
  TIER 2 (2s detection):  >12% drop AND >150 ADC + 2 consecutive decreases
  TIER 3 (backup):        >8% drop  AND >100 ADC + sustained trend

  Uses RAW ADC values (not filtered) for maximum responsiveness.
```

When rapid detection fires:
- The ADC history buffer is reset to the new raw value (prevents the moving
  average filter from retaining old underwater values)
- The air baseline is recalibrated using the raw value
- A `m_rapid_surface_confirmed` flag is set to bypass confirmation on
  subsequent samples within the same batch

### 3. Trend Analysis (biofouling-resistant)

Detects gradual drying by tracking a decreasing ADC trend over 8 samples:
- Consecutive trend: >= 3 decreasing samples + >= 10% total drop
- Cumulative trend: >= 15% drop from peak ADC since submerging (after 10s)
- Only triggers if ADC < 2000 (absolute minimum for true seawater)

### 4. Variance Detection (stability-based)

- High variance (> 10000) -> unstable readings = drying surface
- Low variance (< 2000) -> stable readings = confirmed underwater
- Calculated over the trend buffer (last 8 readings) using running mean
  and sum of squared differences


---

## Biofouling Compensation

Biofouling (salt/biofilm accumulation on the electrode) is the primary challenge
for long-duration marine deployments. It elevates air readings, potentially
causing the device to think it's underwater while at surface.

The algorithm handles this through multiple mechanisms:

### 1. Adaptive Air Baseline

When at surface for >10 seconds with elevated readings (>130% of air baseline):
```
  new_air = air_baseline * 0.9 + avg_surface_readings * 0.1
```
Gradual adaptation (10% per update) prevents overcorrection.

### 2. Extended Dive Recalibration

After 60s underwater, checks every 30s whether the minimum ADC during the dive
suggests biofouling:
- If `min_adc > air_baseline * 3` AND `min_adc < water_baseline * 0.7`:
  the air baseline has shifted significantly
- Recalibrates: `new_air = min_adc * 0.85` (only if >150% increase)

### 3. Hysteresis Zone Stuck Detection

If the ADC stays in the hysteresis zone for >30 seconds:
- The air baseline is moved upward toward the current reading
- `new_air = filtered_value * 0.75` (applied every 15s while stuck)

### 4. Max Dive Timeout Recalibration

When `UW_MAX_DIVE_TIME` forces a surface state:
- Uses the minimum ADC observed during the entire dive
- `new_air = min_adc_during_dive * 0.8`
- Activates a 30s surface lockout to prevent immediate re-submersion


---

## Safety Mechanisms

```
  Max Dive Time (default 7200s = 2h):
    If underwater > max_dive_time -> force SURFACE
    Activate surface lockout (30s) to prevent immediate re-submersion
    Recalibrate air baseline (likely biofouling)

  Min Surface Time (default 10s):
    Ignore underwater detections if at surface < min_surface_time
    Prevents brief wave splash from triggering dive state

  Surface Lockout (30s):
    After max dive timeout forces surface, prevent re-submersion for 30s
    Gives time for recalibration to stabilize
```


---

## Session Timeline Examples

### Normal dive cycle (marine turtle with SWS):

```
  Time
   |
   |  T+0s     Device is running continuously
   |            SWS sampling every 60s at surface
   |
   |  T+0s     SWS reads ADC = 180 (air)  --> SURFACE
   |            GNSS acquiring fix, Argos TX scheduled
   |
   |  T+60s    SWS reads ADC = 190 (air)  --> SURFACE (no change)
   |
   |  T+120s   Turtle dives!
   |            SWS reads ADC = 4200 (saltwater) > threshold_high (1146)
   |            1 sample >= UW_MAX_SAMPLES (1) --> UNDERWATER confirmed!
   |            |
   |            +---> notify_underwater_state(true)
   |            +---> ArgosScheduler: deschedule() all TX
   |            +---> GPSScheduler: cancel GNSS, suspend
   |            Now sampling every 60s underwater
   |
   |  T+180s   SWS reads ADC = 4150 --> still UNDERWATER
   |            Water baseline updated via EMA
   |
   |  T+300s   Turtle surfaces!
   |            SWS reads ADC = 4100 -> 350 (rapid drop)
   |            RAPID TIER 1: 91% drop, >300 absolute --> SURFACE!
   |            (detected in 1 sample = 1 second)
   |            |
   |            +---> notify_underwater_state(false)
   |            +---> m_earliest_tx = now + DRY_TIME_BEFORE_TX
   |            +---> ArgosScheduler: reschedule()
   |            +---> GPSScheduler: resume GNSS acquisition
   |
   |  T+300s   DRY_TIME_BEFORE_TX = 0 --> TX allowed immediately
   |            GNSS powers on, acquires fix
   |            Argos TX resumes with new position
   |
   v
```

### Dive with biofouling (after weeks deployed):

```
  Time
   |
   |  T+0s     Electrode has biofouling
   |            Air ADC now reads ~600 instead of ~200 (salt deposits)
   |            Adaptive air baseline has adjusted: air=500
   |            Threshold recalculated: 500 + 35% * (3000-500) = 1375
   |
   |  T+60s    Turtle surfaces
   |            ADC drops from 3200 to 1800 in 1 sample
   |            RAPID TIER 1: 43% drop (>25%), 1400 abs (>300)
   |            --> SURFACE detected in 1s despite biofouling!
   |
   |  T+70s    At surface for 10s, readings still elevated (~650)
   |            Adaptive air recalibration kicks in
   |            air_baseline = 500 * 0.9 + 650 * 0.1 = 515
   |            Threshold adjusts down for next cycle
   |
   v
```

### Max dive timeout scenario:

```
  Time
   |
   |  T+0s     Turtle dives, heavy biofouling on electrode
   |            ADC = 1500 (biofouling + water)
   |            Above threshold -> UNDERWATER
   |
   |  T+60s    ADC = 1480 (still biofouled)
   |            Extended dive recalib check: too early (<60s)
   |
   |  T+120s   ADC = 1450, min_adc_during_dive = 1450
   |            Extended dive recalib: 1450 > air*3(600)
   |            but 1450 < water*0.7(2100) -> biofouling suspected
   |            new_air = 1450 * 0.85 = 1232 (> old*1.5? yes)
   |            Recalibrate! Threshold shifts up
   |
   |  ...if still stuck after 7200s...
   |
   |  T+7200s  UW_MAX_DIVE_TIME reached -> FORCE SURFACE
   |            Recalibrate air baseline from min ADC
   |            Surface lockout 30s activated
   |
   v
```


---

## Parameters Reference

### Underwater Detection

| Parameter | DTE Key | Type | Default | Description |
|-----------|---------|------|---------|-------------|
| UNDERWATER_EN | UNP01 | BOOL | 0 | Enable underwater detection |
| DRY_TIME_BEFORE_TX | UNP02 | UINT | 0 | Seconds at surface before TX allowed |
| SAMPLING_UNDER_FREQ | UNP03 | UINT | 60 | Sampling interval underwater (seconds) |
| SAMPLING_SURF_FREQ | UNP04 | UINT | 60 | Sampling interval at surface (seconds) |
| UW_MAX_SAMPLES | UNP05 | UINT | 1 | Consecutive wet samples to confirm dive |
| UW_MIN_DRY_SAMPLES | UNP06 | UINT | 2 | Consecutive dry samples to confirm surface |
| UW_SAMPLE_GAP | UNP07 | UINT | 1000 | Gap between confirmation samples (ms) |
| UW_PIN_SAMPLE_DELAY | UNP08 | UINT | 1 | ADC electrode settling time (ms) |
| UNDERWATER_DETECT_SOURCE | UNP10 | ENUM | 0 | 0=SWS, 1=Pressure, 2=GNSS, 3=SWS+GNSS |
| UNDERWATER_DETECT_THRESH | UNP11 | FLOAT | 1.1 | Threshold for pressure/GNSS methods |

### SWS Analog Calibration

| Parameter | DTE Key | Type | Default | Description |
|-----------|---------|------|---------|-------------|
| SWS_ANALOG_THRESHOLD_MIN | UNP20 | UINT | 100 | Minimum valid ADC reading |
| SWS_ANALOG_THRESHOLD_MAX | UNP21 | UINT | 3000 | Maximum valid ADC reading |
| SWS_ANALOG_HYSTERESIS | UNP22 | UINT | 14 | Hysteresis as % of threshold |
| SWS_ANALOG_CALIB_INTERVAL | UNP23 | UINT | 3600 | Air baseline recalibration interval (seconds) |
| UW_MAX_DIVE_TIME | UNP24 | UINT | 7200 | Force surface after this duration (seconds, 0=disabled) |
| UW_MIN_SURFACE_TIME | UNP25 | UINT | 10 | Ignore dives within this time after surfacing (seconds) |

### Algorithm Constants (hardcoded)

| Constant | Value | Purpose |
|----------|-------|---------|
| ADC_INVALID_MIN | 50 | Minimum valid ADC reading |
| ADC_INVALID_MAX | 16383 | Maximum (14-bit ADC) |
| DEFAULT_THRESHOLD_RATIO | 35% | Threshold position between air & water |
| DEFAULT_ALPHA | 0.19 | EMA factor for water baseline |
| DEFAULT_HYSTERESIS | 14% | Hysteresis as % of threshold |
| ABSOLUTE_MIN_WATER_ADC | 2000 | True seawater minimum ADC |
| MIN_WATER_AIR_RATIO | 5x | Water must be >= 5x air baseline |
| RAPID_DROP_TIER1 | 25% / 300 abs | Tier 1 rapid surface detection |
| RAPID_DROP_TIER2 | 12% / 150 abs | Tier 2 rapid surface detection |
| RAPID_DROP_TIER3 | 8% / 100 abs | Tier 3 rapid surface detection |
| SURFACE_LOCKOUT | 30s | Lockout after max dive timeout |
| HYSTERESIS_STUCK_TIMEOUT | 30s | Recalibrate if stuck in hysteresis zone |
| EXTENDED_DIVE_RECALIB_START | 60s | Start biofouling check after this time underwater |
| EXTENDED_DIVE_RECALIB_INTERVAL | 30s | Check interval during extended dive |
| SURFACE_ADAPT_THRESHOLD | 1.3x | Trigger air recalib if readings 30% above baseline |


---

## Troubleshooting

### Underwater detection not working
- Check `UNDERWATER_EN` (UNP01) is 1
- Check `UNDERWATER_DETECT_SOURCE` (UNP10) matches your hardware (0=SWS for conductivity electrode)
- Verify SWS electrode wiring: enable pin and ADC channel must match BSP definition
- Read SWS diagnostic status via `$SWSST` DTE command to check calibration values

### Device thinks it's always underwater
- Likely biofouling — reduce `UW_MAX_DIVE_TIME` (UNP24) to force periodic surface state and recalibration
- Check `SWS_ANALOG_THRESHOLD_MIN` (UNP20) — if too low, noise may look like water
- Check the air baseline via `$SWSST` — if it's very high, biofouling has shifted it

### Device never detects dives
- Check that `SWS_ANALOG_THRESHOLD_MAX` (UNP21) is high enough for the ADC range
- The water baseline may not have been learned yet — first dive after boot uses the initial estimate (air * 3)
- Verify electrode is making contact with water when submerged

### Surface detection too slow
- The algorithm has <2s surface detection via rapid drop tiers
- Check `UW_MIN_DRY_SAMPLES` (UNP06) — lower values = faster (default 2)
- Heavy biofouling may require time for adaptive recalibration — ensure `SWS_ANALOG_CALIB_INTERVAL` (UNP23) is not too high

### Device not transmitting after surfacing
- Check `DRY_TIME_BEFORE_TX` (UNP02) — if set too high, TX is delayed after surfacing
- Check that the Argos scheduler receives the surface notification (enable DEBUG logging)
- Verify `UNDERWATER_EN` is enabled — if disabled, `notify_underwater_state` is ignored by the scheduler
