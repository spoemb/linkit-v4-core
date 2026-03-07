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
method for marine deployments. It uses RC time constant discrimination to measure
the electrical conductivity between two electrodes.

### Hardware Principle

A 100nF capacitor charges through the medium when the electrode is enabled.
The charge rate depends on the medium's resistance:

```
  Enable SWS pin -> wait 1ms -> read 14-bit ADC -> disable SWS pin

  Water (R~10k, tau=1ms):   at 1ms = 63% charged -> ADC ~2500-3000
  Wet film (R~50k, tau=5ms): at 1ms = 18% charged -> ADC ~500-1000
  Air (R=inf):               at 1ms = 0% charged  -> ADC ~0-300
```

The 1ms sampling point is the critical design choice: it maximizes the
discrimination between water and wet electrode film.

Between samples (1s apart), the 1M pull-down fully discharges the cap
(tau=100ms, 5*tau=500ms < 1000ms).


---

## Auto-Calibration

### Initial Calibration (at boot)

```
  Boot
    |
    v
  Take 10 ADC samples, 100ms apart
    |
    v
  avg > 2500? --> Started in water: water=avg, air=avg/3
  avg <= 2500? --> Started in air: air=avg, water=min(air*3, observed_peak)
    |
    v
  Calculate threshold and store in noinit RAM with CRC16
```

### Dynamic Water Baseline (Exponential Moving Average)

When confirmed underwater (filtered > threshold_high), the water baseline adapts:
```
  new_water = 0.19 * current_reading + 0.81 * old_water
```

Protected by strict conditions:
- Value > threshold_current + hysteresis (truly underwater)
- Value >= 2000 (absolute minimum for seawater)
- Value >= air * 3 (bypassed if air >= 1000 to allow recovery)
- Value within expected range (>= 85% of water baseline or higher)
- Capped at observed peak ADC (prevents inflation above actual readings)

### Air Baseline Adaptation (at surface > 10s)

| Direction | Trigger | Formula | Rate |
|-----------|---------|---------|------|
| Upward (biofouling) | avg > air * 1.3 | air = 0.9*air + 0.1*avg | 10%/update |
| Downward (wet calib fix) | avg < air * 0.7 | air = 0.8*air + 0.2*avg | 20%/update |
| Timed recalibration | elapsed > 3600s | air = avg | 100% |

### Calibration Persistence

- Stored in noinit RAM (survives MCU soft reset)
- Protected by CRC16 checksum — corrupt data triggers fresh calibration
- Observed peak ADC stored separately with its own CRC


---

## Dynamic Threshold

The threshold is positioned as a percentage between air and water baselines.
The ratio adapts to the electrode contrast:

```
  threshold = air + (water - air) * ratio
  hysteresis = threshold * 4%

  Clean (contrast >= 8x):  ratio = 35%  -> threshold close to air
  Moderate (contrast >= 4x): ratio = 50% -> midpoint
  Low contrast (< 4x):    ratio = 40%  -> closer to air for detection

  Example: air=200, water=3000, contrast=15x, ratio=35%
    threshold = 200 + 2800 * 0.35 = 1180
    hysteresis = 1180 * 4% = 47
    threshold_high = 1227 (above this -> UNDERWATER)
    threshold_low  = 1133 (below this -> SURFACE)
```

### Dynamic Peak ADC Capping

The observed peak ADC (highest value ever read) caps:
- Water baseline estimates (air*3 never exceeds actual peak)
- threshold + hysteresis (never exceeds what the ADC produces)

This prevents the case where calibrating with wet electrodes (air=1200)
would estimate water=3600 and push threshold above actual readings (~3000).


---

## 5-Level Surface Detection

Five independent methods detect surfacing, ordered from fastest to slowest.
All require >= 1s underwater and proximity guard OK (filtered < 95% of peak).

| Level | Speed | Condition | Use Case |
|-------|-------|-----------|----------|
| **L1** | 1 sample | Drop > 5% from recent peak | Sharp water exit |
| **L2** | 2 samples | 2 consecutive drops, cumulative > 3% | Gradual exit |
| **L3** | 3+ samples | MA3 trend down 3x, total > 5% | Noisy signal |
| **L4** | Variable | filtered < water * 85% | Absolute reference |
| **L5** | >10s | Drop > 15% from dive peak | Safety net |

When any level triggers:
1. State forced to surface
2. Air baseline recalibrated to current value (if contrast allows)
3. Surface lockout activated (UW_MIN_SURFACE_TIME)

For full algorithm details, see [sws_analog_implementation.md](sws_analog_implementation.md).


---

## Safety Mechanisms

```
  Max Dive Time (default 7200s = 2h):
    If underwater > max_dive_time -> force SURFACE
    Activate surface lockout (30s) to prevent immediate re-submersion
    Does NOT recalibrate (would corrupt baselines with underwater values)

  Min Surface Time (default 2s):
    After any surface detection, ignore underwater for 2s
    Prevents oscillation from wet electrode at surface

  Surface Lockout (30s):
    After max dive timeout forces surface, prevent re-submersion for 30s
    Water baseline NOT updated during lockout (readings are surface values)

  Proximity Guard (95%):
    L1-L5 overrides blocked if filtered >= 95% of max(water, dive_peak)
    Prevents false surface detection from normal underwater ADC noise
```


---

## Session Timeline Examples

### Normal dive cycle (clean electrode):

```
  T+0s     SWS reads ADC = 180 (air) -> SURFACE
           GNSS active, Argos TX scheduled

  T+10s    Turtle dives
           SWS reads ADC = 2900 > threshold_high (1227) -> UNDERWATER
           -> ArgosScheduler: deschedule()
           -> GPSScheduler: suspend

  T+70s    SWS reads ADC = 2850 -> still UNDERWATER
           Water baseline updated via EMA: 2900 -> 2890

  T+130s   Turtle surfaces
           SWS reads ADC = 2900 -> 1500 (wet electrode drying)
           L1: drop from recent_peak 2900 to 1500 = 48% > 5% -> SURFACE!
           Air recalibrated to 1500 (will adapt down with dry readings)
           -> ArgosScheduler: reschedule()
           -> GPSScheduler: resume

  T+145s   SWS reads ADC = 300 (dried) -> still SURFACE
           Air baseline adapts down: 1500 -> 0.8*1500 + 0.2*300 = 1260
```

### Dive with biofouling (after months deployed):

```
  T+0s     Electrode has biofouling
           Air reads ~1000 (biofilm), Water reads ~2800
           Contrast = 2.8x -> ratio = 40%
           threshold = 1000 + 1800*0.40 = 1720, hysteresis = 69
           threshold_high = 1789

  T+10s    Turtle dives
           ADC = 2750 > 1789 -> UNDERWATER

  T+70s    Turtle surfaces
           ADC drops from 2780 to 2100 in 1 sample
           L1: (2780-2100)/2780 = 24% > 5% -> SURFACE in 1s!

  T+80s    At surface for 10s, readings settling ~1100
           Upward air adaptation: 1000*0.9 + 1100*0.1 = 1010
           Threshold adjusts for next cycle
```


---

## Parameters Reference

### Underwater Detection

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

### SWS Analog Calibration

| Parameter | DTE Key | Type | Default | Description |
|-----------|---------|------|---------|-------------|
| SWS_ANALOG_THRESHOLD_MIN | UNP20 | UINT | 0 | Minimum valid ADC reading |
| SWS_ANALOG_THRESHOLD_MAX | UNP21 | UINT | 8000 | Max valid ADC (legacy, superseded by observed peak) |
| SWS_ANALOG_HYSTERESIS | UNP22 | UINT | 4 | Hysteresis as % of threshold |
| SWS_ANALOG_CALIB_INTERVAL | UNP23 | UINT | 3600 | Air baseline recalibration interval (seconds) |
| UW_MAX_DIVE_TIME | UNP24 | UINT | 7200 | Force surface after this duration (seconds, 0=disabled) |
| UW_MIN_SURFACE_TIME | UNP25 | UINT | 2 | Surface lockout after detection (seconds) |


---

## Troubleshooting

### Underwater detection not working
- Check `UNDERWATER_EN` (UNP01) is 1
- Check `UNDERWATER_DETECT_SOURCE` (UNP10) is 0 (SWS)
- Verify SWS electrode wiring matches BSP definition
- Read `$SWSST` -> check that threshold_high < actual water ADC readings

### Device thinks it's always underwater
- Likely biofouling — reduce `UW_MAX_DIVE_TIME` to force periodic surface
- Check air baseline via `$SWSST` — if very high, biofouling or wet calibration
- The downward air adaptation (avg < air * 0.7) should auto-correct over time

### Device never detects dives
- Check `$SWSST`: if threshold_high > water readings, threshold is too high
- Check observed_peak: if 0, no peak learned yet (first boot needs one immersion)
- Verify `UW_PIN_SAMPLE_DELAY` = 1 (higher values lose discrimination)

### Surface detection too slow
- L1 should detect in 1 sample for any drop > 5% from recent peak
- If readings drop slowly (biofouling), L2/L3 catch it in 2-5 samples
- Lower `SAMPLING_UNDER_FREQ` to 1s for faster sampling
- Check proximity guard: readings must drop below 95% of peak to allow L-overrides

### Threshold exceeds actual water readings
- The observed_peak_adc caps threshold+hysteresis at the max ADC ever seen
- If peak=0 (first boot), do one immersion to learn it
- Check `$SWSST` for the observed_peak field (last value in response)

### Device not transmitting after surfacing
- Check `DRY_TIME_BEFORE_TX` — if set high, TX is delayed
- Verify Argos scheduler receives surface notification (enable DEBUG logging)
- Check `UNDERWATER_EN` is enabled

---

*Last updated: 2026-03-07*
