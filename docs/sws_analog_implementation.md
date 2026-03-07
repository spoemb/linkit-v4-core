# SWS Analog - Surface/Underwater Detection Algorithm

## Table of Contents

1. [Context and Problem Statement](#context-and-problem-statement)
2. [Hardware: RC Discrimination](#hardware-rc-discrimination)
3. [Architecture](#architecture)
4. [Detection Algorithm](#detection-algorithm)
5. [5-Level Surface Detection](#5-level-surface-detection)
6. [Calibration System](#calibration-system)
7. [Dynamic Peak ADC Tracking](#dynamic-peak-adc-tracking)
8. [Safety Mechanisms](#safety-mechanisms)
9. [Parameters Reference](#parameters-reference)
10. [Troubleshooting](#troubleshooting)

---

## Context and Problem Statement

### SWS Function

The Salt Water Switch (SWS) detects whether a sea turtle tracker is underwater or at the surface.
Two electrodes measure the conductivity of the medium via a 14-bit ADC (0-16383) on nRF52840.

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

### The Two Problems

**1. Biofouling** — After months at sea, salt/biofilm accumulate on electrodes:

```
Month 0  (clean)    : Air ~200,   Water ~3000  -> contrast 15x
Month 6  (moderate) : Air ~1000,  Water ~2500  -> contrast 2.5x
Month 12 (severe)   : Air ~1700,  Water ~2100  -> contrast 1.2x
```

A fixed threshold inevitably fails. The algorithm must adapt dynamically.

**2. Wet electrode after exit** — When the tracker exits water, the electrode stays
wet. Without RC discrimination, a wet electrode reads the same as submerged water,
making surface detection impossible.

### Fundamental Insight

The algorithm exploits **RC time constant discrimination**: water and wet film have
different resistances, so a capacitor charges at different rates through each medium.
By reading the ADC at a precise time (1ms), we can distinguish them.

---

## Hardware: RC Discrimination

A 100nF capacitor at the ADC pin charges through the medium's resistance when
the SWS electrode is enabled. The 1M pull-down resistor discharges it between reads.

### Time Constants

```
Medium          | Resistance  | tau = RC         | @1ms charge | @2ms charge
----------------|-------------|------------------|-------------|------------
Seawater        | ~10k ohm    | 1ms              | 63%         | 86%
Wet film        | ~50k ohm    | 5ms              | 18%         | 33%
Light biofouling| ~100k ohm   | 10ms             | 10%         | 18%
Air             | infinity    | infinity         | 0%          | 0%
```

### Why 1ms Pin Sample Delay

At 1ms, the ratio between water (63%) and wet film (18%) is **3.5x** — excellent
discrimination. This is the KEY parameter that makes the system work.

Increasing this delay loses discrimination:
- At 5ms: water=99%, film=63% -> ratio 1.6x (poor)
- At 15ms: water=100%, film=95% -> ratio 1.05x (useless)

### Capacitor Discharge Between Samples

The 1M pull-down discharges the 100nF cap: tau = 100ms.
Full discharge (5*tau) = 500ms. With 1s sampling interval, the cap is fully
discharged before each reading.

**The pin_sample_delay must remain fixed at 1ms** — it is a physical constant of the
circuit, not a tunable parameter. Changing it changes what is being measured.

---

## Architecture

### Class Hierarchy

```
UWDetectorService (parent)
        |
        |  inherits
        v
SWSAnalogService (child)
```

**Files**:
- `core/services/uwdetector_service.hpp/.cpp` — Scheduling and batch management
- `core/services/sws_analog_service.hpp/.cpp` — Detection algorithm

### Batch Operation

The parent manages sampling cycles:

```
service_next_schedule_in_ms() -> SAMPLING_SURF_FREQ (surface) or SAMPLING_UNDER_FREQ (underwater)
        |
        v
service_initiate() -> detector_state() called UW_MAX_SAMPLES times
        |               spaced by UW_SAMPLE_GAP ms
        v
If all samples = underwater -> confirmed underwater
If UW_MIN_DRY_SAMPLES = surface -> confirmed surface (early termination)
```

With `UW_MAX_SAMPLES=1` (default), each cycle is a single call to `detector_state()`.

---

## Detection Algorithm

### Overview: detector_state() Flow

Each call to `detector_state()` executes these steps in order:

```
 1. READ ADC
    raw_value = read_analog_sws()
    Enable SWS pin -> wait 1ms -> read 14-bit ADC -> disable SWS pin

 2. FIRST-SAMPLE COHERENCE CHECK (once after boot)
    If stored calibration is incoherent with first reading -> recalibrate

 3. UPDATE OBSERVED PEAK ADC
    Track highest ADC ever seen (persistent in noinit RAM)

 4. FILTERING
    filtered_value = moving_average(raw, history_size=2)

 5. TIME TRACKING
    Update time_in_current_state, check max dive timeout

 6. 5-LEVEL SURFACE DETECTION (see next section)
    Evaluate L1-L5, with proximity guard

 7. RECENT PEAK & DIVE PEAK TRACKING
    Update m_recent_peak (decaying), m_peak_adc_since_underwater

 8. SURFACE BASELINE TRACKING
    If at surface > 10s: collect readings, adapt air baseline up or down

 9. STATE DETERMINATION (hysteresis threshold)
    filtered > threshold_high -> underwater (+ update water baseline via EMA)
    filtered < threshold_low  -> surface (after min_dry_samples)
    in between -> maintain current state

10. APPLY SURFACE OVERRIDE (L1-L5)
    If any level triggered: force surface, recalibrate air, apply lockout

11. MAX DIVE TIMEOUT
    If underwater > max_dive_time -> force surface + lockout

12. SURFACE LOCKOUT
    If lockout active -> force surface regardless

13. STATE CHANGE
    If state changed: reset all tracking variables, log transition
```

### Dynamic Threshold

```
ADC
 ^
 |  +--- threshold_water (EMA, alpha=19%)
 |  |
 |  |  -- threshold_high = threshold_current + hysteresis
 |  |
 |  |  -- threshold_current = air + ratio * (water - air)
 |  |
 |  |  -- threshold_low = threshold_current - hysteresis
 |  |
 |  +--- threshold_air (calibrated, adaptive)
 |
 0 ----------------------------------------------------> time
```

**Ratio** varies with contrast (water/air):

| Contrast | Ratio | Meaning |
|----------|-------|---------|
| >= 8x    | 35%   | Clean electrode: threshold close to air for fast surface detection |
| >= 4x    | 50%   | Moderate biofouling: midpoint balance |
| < 4x     | 40%   | Low contrast (wet electrodes or severe biofouling): closer to air to ensure underwater detection |

**Hysteresis** = 4% of threshold_current (minimum 10 ADC counts).

Both threshold+hysteresis are capped at the observed peak ADC to prevent
the threshold from exceeding values the ADC can actually produce.

---

## 5-Level Surface Detection

All levels require:
- Currently underwater (`m_current_state = true`)
- At least 1 second underwater (`OVERRIDE_MIN_TIME_SEC`)
- Proximity guard OK: `filtered_value < max(water_baseline, peak_during_dive) * 95%`

The proximity guard prevents false surface detection from normal underwater ADC noise.

### Level 1 — INSTANT (1 sample)

```
Condition: raw_value < recent_peak * (1 - 5%)
Latency:   1 sample (~1s)
Use case:  Clean/moderate electrode, sharp water exit
```

`m_recent_peak` decays 5% per sample toward the current reading. This makes L1
track drift without false triggers: if readings slowly decrease (drift), the peak
follows. Only a SUDDEN drop exceeds the 5% gap.

Example: water at 3000, turtle exits -> reading drops to 2700.
Drop = (3000-2700)/3000 = 10% > 5% -> **L1 triggers immediately**.

### Level 2 — FAST (2 samples)

```
Condition: 2 consecutive raw drops AND cumulative drop > 3% from first drop
Latency:   2 samples (~2s)
Use case:  Gradual exit where individual drops are < 5% (no L1 trigger)
```

Example: 3000 -> 2960 -> 2900. Individual drops are 1.3% and 2% (no L1).
Cumulative from 2960: (2960-2900)/2960 = 2% (no L2 yet).
Next: 2960 -> 2900 -> 2850. Cumulative = (2960-2850)/2960 = 3.7% -> **L2 triggers**.

### Level 3 — TREND (3+ MA3 decreases)

```
Condition: MA3 decreased 3+ consecutive times AND total MA3 drop > 5%
Latency:   ~5-6 samples
Use case:  Noisy signal where bounces break L2's consecutive requirement
```

Computes a 3-sample moving average of filtered values. If each new MA3 is lower
than the previous, the trend counter increments. A single increase only decrements
the counter by 1 (noise tolerance), preventing a complete reset.

Useful when the signal bounces: 3000, 2950, 2980, 2920, 2970, 2880...
L2 resets on each bounce, but the MA3 trend still decreases overall.

### Level 4 — ABSOLUTE (variable latency)

```
Condition: filtered_value < water_baseline * 85%
Latency:   Variable (depends on how fast readings drop)
Use case:  Reliable absolute reference when water baseline is well-calibrated
```

Independent of recent history. Directly compares to the calibrated water baseline.
With water=3000: triggers when filtered < 2550.

### Level 5 — SAFETY NET (>10s underwater)

```
Condition: (peak_during_dive - filtered) / peak_during_dive > 15%
           AND time underwater > 10s
Latency:   >10s (intentionally slow, last resort)
Use case:  All baselines have drifted, nothing else triggered
```

Uses the peak ADC observed since dive start (not the calibrated water baseline).
The 10s minimum prevents early false triggers from initial stabilization.

### Detection Hierarchy

```
Speed:    L1 (1s) > L2 (2s) > L3 (~5s) > L4 (variable) > L5 (>10s)

L1: instant drop from recent peak      -> catches sharp exits
L2: cumulative 2-sample drop            -> catches gradual exits
L3: MA3 trend                           -> catches noisy gradual exits
L4: absolute vs water baseline          -> catches any exit when baseline is good
L5: peak-relative with time gate        -> last resort safety net
```

---

## Calibration System

### Initial Air Calibration (at boot)

Takes 10 ADC samples, 100ms apart. If average > 2500: assumes device booted
in water and swaps air/water estimates. Otherwise, `air = average`,
`water = air * 3` (capped at observed peak ADC if available).

### Water Baseline (EMA, continuous)

When confirmed underwater (filtered > threshold_high), the water baseline adapts:

```
new_water = 0.19 * value + 0.81 * old_water
```

**Protection conditions** (all must be true):
- Value > threshold_current + hysteresis
- Value >= 2000 (absolute minimum for seawater)
- Value >= air * 3 (or air >= 1000, to allow recovery from corrupted calibration)
- Value >= 85% of current water_baseline OR > water_baseline
- Not in surface lockout period

Water baseline is also capped at observed peak ADC.

### Air Baseline Adaptation

Three modes, checked when at surface > 10 seconds:

| Mode | Condition | Action | Speed |
|------|-----------|--------|-------|
| Timed recalibration | Elapsed > CALIB_INTERVAL (3600s) | air = average of surface readings | Immediate |
| Upward adaptation | avg > air * 1.3 AND avg < threshold | air = 0.9*air + 0.1*avg | Slow (10%) |
| Downward adaptation | avg < air * 0.7 | air = 0.8*air + 0.2*avg | Fast (20%) |

Downward adaptation is faster because an inflated air baseline (from wet electrode
calibration) is more damaging than a low one.

### Surface Override Air Recalibration

When a L1-L5 surface override triggers, if the current filtered value is > 2x air
baseline AND < 80% of water baseline, it immediately sets air = filtered_value.
This handles the wet electrode at exit: the reading is high but dropping, so
recalibrating air to this value prevents immediate re-trigger on the next sample.

### First-Sample Coherence Check

On the first ADC read after boot, if the stored calibration is wildly inconsistent
with the actual reading (e.g., stored air=1000 but reading=4800, or stored air=3000
but reading=500), the calibration is invalidated and rebuilt from the current reading.

### Calibration Persistence

All calibration data (air, water, threshold, hysteresis, timestamp) is stored in
noinit RAM with CRC16 validation. Survives MCU soft resets but not power cycles
(unless backed by retained RAM).

---

## Dynamic Peak ADC Tracking

### Problem

When calibrating in air with wet electrodes:
- Air reads ~1200 (wet film, not true air)
- Water estimated as air * 3 = 3600
- But actual water ADC is only ~3000
- Threshold + hysteresis = 3647 > actual water readings -> never detects underwater

### Solution: m_observed_peak_adc

A persistent tracker (noinit RAM + CRC) records the highest ADC value actually
observed during operation. This value caps:

1. **Water baseline estimation**: `air * 3` capped at observed peak
2. **EMA water updates**: new_water capped at observed peak
3. **Threshold + hysteresis**: if threshold_current + hysteresis > observed peak,
   both are reduced proportionally

The peak updates continuously from raw readings. On first boot (peak=0), no capping
is applied — the peak is learned from the first immersion.

---

## Safety Mechanisms

### Max Dive Timeout (default: 7200s = 2h)

If underwater longer than `UW_MAX_DIVE_TIME`:
- Forces surface state
- Does NOT recalibrate air baseline (would corrupt it with underwater values)
- Activates 30s surface lockout

### Surface Lockout

After a surface override (L1-L5) or max dive timeout:
- `UW_MIN_SURFACE_TIME` seconds of forced surface (default 2s)
- After max dive timeout: 30s lockout (`SURFACE_LOCKOUT_DURATION_SEC`)
- During lockout, water baseline is NOT updated (readings are surface, not water)

### Proximity Guard

All L1-L5 surface overrides are blocked if:
```
filtered_value >= max(water_baseline, peak_during_dive) * 95%
```

This prevents false surface detection from normal underwater ADC noise/drift.
Using `max(water_baseline, peak_during_dive)` handles the case where water EMA
has drifted below actual readings.

---

## Parameters Reference

### Configurable Parameters (DTE / Config Store)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `UW_PIN_SAMPLE_DELAY` | **1 ms** | RC charge time before ADC read. Fixed at 1ms for optimal water/film discrimination. Do not change. |
| `UW_MAX_SAMPLES` | **1** | Samples per detection cycle. 1 = immediate dive detection. |
| `UW_MIN_DRY_SAMPLES` | **1** | Consecutive dry samples to confirm surface via threshold. |
| `UW_SAMPLE_GAP` | **1000 ms** | Interval between sub-samples within a batch (when MAX_SAMPLES > 1). |
| `SWS_ANALOG_THRESHOLD_MIN` | **0** | Minimum valid ADC reading. |
| `SWS_ANALOG_THRESHOLD_MAX` | **8000** | Maximum valid ADC (legacy, superseded by observed peak). |
| `SWS_ANALOG_HYSTERESIS` | **4%** | Hysteresis as % of threshold. Higher = more stable but slower detection. |
| `SWS_ANALOG_CALIB_INTERVAL` | **3600 s** | Air baseline timed recalibration interval (1h). |
| `UW_MAX_DIVE_TIME` | **7200 s** | Force surface after this time underwater (2h). 0 = disabled. |
| `UW_MIN_SURFACE_TIME` | **2 s** | Lockout after surface override. Prevents oscillation. |
| `SAMPLING_UNDER_FREQ` | **10 s** | Sampling period when underwater. |
| `SAMPLING_SURF_FREQ` | **10 s** | Sampling period when at surface. |

### Internal Constants (sws_analog_service.cpp)

#### ADC

| Constant | Value | Description |
|----------|-------|-------------|
| `ADC_INVALID_MAX` | 16383 | 14-bit ADC maximum |
| `ADC_HISTORY_SIZE` | 2 | Moving average filter window (small = fast response) |

#### Detection Tuning

| Constant | Value | Description |
|----------|-------|-------------|
| `DEFAULT_THRESHOLD_RATIO_PERCENT` | 35% | Threshold position between air and water (clean electrode) |
| `DEFAULT_ALPHA_PERCENT` | 19% | EMA alpha for water baseline adaptation |
| `DEFAULT_HYSTERESIS_PERCENT` | 4% | Fallback hysteresis if config is invalid |
| `DEFAULT_MAX_SAMPLES` | 1 | Default samples per cycle |
| `DEFAULT_MIN_DRY_SAMPLES` | 1 | Default dry samples to confirm surface |

#### Water Baseline Protection

| Constant | Value | Description |
|----------|-------|-------------|
| `ABSOLUTE_MIN_WATER_ADC` | 2000 | Reject water readings below this (not seawater) |
| `MIN_WATER_AIR_RATIO` | 3 | Water must be >= 3x air (bypassed if air >= 1000) |

#### Surface Baseline Adaptation

| Constant | Value | Description |
|----------|-------|-------------|
| `SURFACE_ADAPT_THRESHOLD` | 1.3x | Upward air adaptation trigger (avg > air * 1.3) |
| `MIN_SURFACE_TIME_FOR_ADAPT` | 10 s | Minimum surface time before air adaptation starts |
| Downward threshold | 0.7x | Downward air adaptation trigger (avg < air * 0.7) |

#### 5-Level Surface Detection

| Constant | Value | Description |
|----------|-------|-------------|
| `L1_DROP_PERCENT` | 5% | Drop from recent peak to trigger L1 (instant) |
| `L2_DROP_PERCENT` | 3% | Cumulative 2-sample drop to trigger L2 |
| `L2_MIN_CONSECUTIVE` | 2 | Minimum consecutive raw drops for L2 |
| `L3_DROP_PERCENT` | 5% | Total MA3 trend drop to trigger L3 |
| `L3_MIN_CONSECUTIVE` | 3 | Consecutive MA3 decreases for L3 |
| `L4_DROP_PERCENT` | 15% | Drop below water baseline for L4 |
| `L5_DROP_PERCENT` | 15% | Drop from dive peak for L5 |
| `L5_MIN_TIME_SEC` | 10 s | Minimum underwater time before L5 activates |

#### Safety

| Constant | Value | Description |
|----------|-------|-------------|
| `OVERRIDE_MIN_TIME_SEC` | 1 s | Minimum underwater time before any L-override |
| `PROXIMITY_GUARD_PERCENT` | 95% | L-overrides blocked if filtered >= 95% of peak |
| `SURFACE_LOCKOUT_DURATION_SEC` | 30 s | Lockout after max dive timeout |
| `WATER_DETECT_HEURISTIC` | 2500 | Air calibration: if avg > this, assume in water |

#### Dynamic Threshold Ratio

| Contrast (water/air) | Ratio | Effect |
|----------------------|-------|--------|
| >= 8x (clean) | 35% | Threshold close to air, fast surface detection |
| >= 4x (moderate biofouling) | 50% | Balanced midpoint |
| < 4x (wet electrodes / severe) | 40% | Closer to air to ensure underwater detection |

#### Peak ADC Capping (in update_dynamic_threshold)

When `threshold_current + hysteresis > observed_peak_adc`:
- If threshold >= peak: threshold = peak * 90%, hysteresis = peak * 5%
- If threshold < peak: hysteresis = peak - threshold

---

## Troubleshooting

### Device never detects underwater (always surface)

**Check**: `$SWSST` command -> look at `threshold_high` (= threshold + hysteresis)
vs `raw_adc` when submerged.

**Common causes**:
- `threshold_high > raw_adc`: threshold too high. Check if air baseline is inflated
  (wet electrode calibration). The downward air adaptation should fix this over time.
- `observed_peak` = 0: first boot, no peak learned yet. First immersion will set it.
- Very low contrast (water/air < 2x): check electrode connections, verify conductivity.

### Device never detects surface (always underwater)

**Check**: `$SWSST` -> look at `surface_level`. If always 0, no L-override triggers.

**Common causes**:
- Pin sample delay too high: must be 1ms. Higher values lose water/film discrimination.
- Proximity guard blocking: if readings stay within 5% of peak, L-overrides are blocked.
  This is correct behavior for underwater drift, but indicates the electrode may not be
  exiting water cleanly.
- Air baseline too low: threshold is far below water, readings stay in hysteresis zone.

### Slow surface detection (>5s)

- Increase sampling frequency: lower `SAMPLING_UNDER_FREQ` to 1s
- L1 (instant) should trigger for any drop > 5% from recent peak
- If L1 doesn't trigger, readings are drifting slowly -> L2/L3 should catch it in 2-5s
- Check if proximity guard is blocking (readings very close to water baseline)

### Threshold/hysteresis exceed actual water readings

- The `observed_peak_adc` should prevent this. Check its value in `$SWSST`.
- If peak = 0 (first boot), it hasn't been learned yet. Do one immersion cycle.
- If peak is correct but threshold still too high: check air baseline — if inflated,
  the ratio calculation produces an inflated threshold.

### Calibration corrupted after reset

- Calibration is in noinit RAM with CRC16. If CRC fails, fresh calibration runs.
- Power cycle (not soft reset) clears noinit RAM -> full recalibration at boot.
- `observed_peak_adc` has its own CRC and persists independently of calibration.

---

## DTE Test Commands

### $SWSST — Read SWS Status

Returns: `air, water, threshold, hysteresis, raw_adc, filtered_adc, calibrated, underwater, time_in_state, surface_level, contrast_x10, observed_peak`

### $SWSTST — Start/Stop Test Mode

- `$SWSTST,1` — Start test mode (continuous sampling with async SWSST push)
- `$SWSTST,0` — Stop test mode

In test mode, SWSST data is pushed on every sample regardless of state change.
Useful for real-time monitoring and calibration verification.

---

*Source files*:
- `core/services/sws_analog_service.hpp` — Header (231 lines)
- `core/services/sws_analog_service.cpp` — Implementation (~865 lines)
- `core/services/uwdetector_service.hpp/.cpp` — Parent class (scheduling)

*Last updated: 2026-03-07*
