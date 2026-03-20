# Table of Contents

1. [Context and Problem Statement](#context-and-problem-statement)
2. [Hardware: RC Discrimination](#hardware-rc-discrimination)
3. [Architecture](#architecture)
4. [Detection Algorithm](#detection-algorithm)
5. [5-Level Surface Detection](#5-level-surface-detection)
6. [Calibration System](#calibration-system)
7. [Dynamic Peak ADC Tracking](#dynamic-peak-adc-tracking)
8. [Adaptive Sample Delay](#adaptive-sample-delay)
9. [Safety Mechanisms](#safety-mechanisms)
10. [SWS Logger](#sws-logger)
11. [Parameters Reference](#parameters-reference)
12. [Troubleshooting](#troubleshooting)

---

# Context and Problem Statement

## SWS Function

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

## The Two Problems

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

## Fundamental Insight

The algorithm exploits **RC time constant discrimination**: water and wet film have
different resistances, so a capacitor charges at different rates through each medium.
By reading the ADC at a short, adaptive delay (100-5000µs), we can distinguish them.

---

# Hardware: RC Discrimination

A 100nF capacitor at the ADC pin charges through the medium's resistance when
the SWS electrode is enabled. The 1M pull-down resistor discharges it between reads.

## Time Constants

```
Medium          | Resistance  | tau = RC         | @1ms charge | @2ms charge
----------------|-------------|------------------|-------------|------------
Seawater        | ~10k ohm    | 1ms              | 63%         | 86%
Wet film        | ~50k ohm    | 5ms              | 18%         | 33%
Light biofouling| ~100k ohm   | 10ms             | 10%         | 18%
Air             | infinity    | infinity         | 0%          | 0%
```

## Adaptive Delay (formerly fixed at 1ms)

The sample delay is now **adaptive** (100-5000µs) and adjusts based on electrode contrast:

- **High contrast** (clean electrode, >10x): delay increases toward 5ms for stronger signal
- **Low contrast** (biofouling, <5x): delay decreases toward 100µs to maximize water/film discrimination

At shorter delays, the ratio between water and film charge is higher, which is critical
when biofouling narrows the gap between water and air readings.

## Capacitor Discharge Between Samples

The 1M pull-down discharges the 100nF cap: tau = 100ms.
Full discharge (5*tau) = 500ms. With 1s+ sampling interval, the cap is fully
discharged before each reading.

---

# Architecture

## Class Hierarchy

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

## Batch Operation

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

# Detection Algorithm

## Overview: detector_state() Flow

Each call to `detector_state()` executes these steps in order:

```
 1. READ ADC
    raw_value = read_analog_sws()
    Enable SWS pin -> wait adaptive delay (µs) -> read 14-bit ADC -> disable SWS pin

 2. FIRST-SAMPLE COHERENCE CHECK (once after boot)
    If stored calibration is incoherent with first reading -> recalibrate

 3. UPDATE OBSERVED PEAK ADC
    Track highest ADC ever seen (persistent in noinit RAM)

 4. FILTERING
    filtered_value = moving_average(raw, history_size=2)
    Uses proper count tracking (not skipping zeros)

 5. TIME TRACKING
    Update time_in_current_state, check max dive timeout

 6. 5-LEVEL SURFACE DETECTION (see next section)
    Evaluate L1-L5, with adaptive proximity guard

 7. RECENT PEAK & DIVE PEAK TRACKING
    Update m_recent_peak (decaying 2%/sample), m_peak_adc_since_underwater

 8. SURFACE BASELINE TRACKING (blocked during lockout)
    If at surface > 10s and NOT in lockout: collect readings, adapt air baseline up or down
    Trigger adjust_sample_delay() after any air recalibration

 9. STATE DETERMINATION (hysteresis threshold)
    filtered > threshold_high -> underwater (+ update water baseline via EMA)
    filtered < threshold_low  -> surface (after min_dry_samples)
    in between -> maintain current state
    threshold_low protected against underflow (clamped above air baseline)

10. APPLY SURFACE OVERRIDE (L1-L5)
    If any level triggered: force surface, recalibrate air via EMA (15% weight),
    cap air at 70% of water, adjust sample delay, apply lockout

11. MAX DIVE TIMEOUT
    If underwater > max_dive_time -> force surface + 30s lockout

12. SURFACE LOCKOUT (time-based)
    Compared against actual time in surface state (not sample count).
    If lockout active -> force surface regardless.

13. STATE CHANGE
    If state changed: reset all tracking variables, log transition
    In test mode: set LED BLUE (underwater) or YELLOW (surface)

14. STATUS PUSH & LOG
    Update status snapshot for DTE SWSST (includes sample_delay_us)
    Write SWSLogEntry to persistent flash log (if ENABLE_SWS_LOG)
```

## Dynamic Threshold

```
ADC
 ^
 |  +--- threshold_water (EMA, alpha=19%)
 |  |
 |  |  -- threshold_high = threshold_current + hysteresis
 |  |
 |  |  -- threshold_current = air + ratio * (water - air)
 |  |
 |  |  -- threshold_low = threshold_current - hysteresis (clamped > air)
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

**Threshold underflow protection:** `threshold_low` is clamped to never go below `threshold_air + 1` when `threshold_current > threshold_air`. This prevents the threshold_low from wrapping around to 0 with large hysteresis values.

Both threshold+hysteresis are capped at the observed peak ADC to prevent
the threshold from exceeding values the ADC can actually produce.

---

# 5-Level Surface Detection

All levels require:
- Currently underwater (`m_current_state = true`)
- At least 1 second underwater (`OVERRIDE_MIN_TIME_SEC`)
- Proximity guard OK (adaptive, see below)

## Adaptive Proximity Guard

The proximity guard prevents false surface detection from normal underwater ADC noise.

```
proximity_ref = max(water_baseline, peak_during_dive)
```

**Adaptive threshold based on contrast:**
- **Normal (contrast >= 5x):** `filtered_value < proximity_ref * 95%` — must drop 5% below peak
- **Biofouling (contrast < 5x):** `filtered_value < proximity_ref * 99%` — relaxed to 1% gap

The relaxed guard for low contrast allows detection even when the water/surface gap
is small, which is critical for heavily biofouled electrodes.

## Level 1 — INSTANT (1 sample)

```
Condition: raw_value < prev_raw * (1 - 4%)
Latency:   1 sample (~1s)
Use case:  Sharp water exit, any electrode condition
```

**Changed:** Now compares against `prev_raw` (previous raw value) instead of `m_recent_peak`.
This avoids false triggers from gradual drift where the decaying peak still accumulates
a gap from slow changes.

Example: water at 3000, turtle exits -> reading drops to 2850.
Drop = (3000-2850)/3000 = 5% > 4% -> **L1 triggers immediately**.

## Level 2 — FAST (2 samples)

```
Condition: 2 consecutive raw drops, each step >= 2%, cumulative > 3% from first drop
Latency:   2 samples (~2s)
Use case:  Gradual exit where individual drops < 4% (no L1 trigger)
```

**Added:** Each individual step must be >= `L2_MIN_STEP_PERCENT` (2%). This filters out
gradual drift or salinity changes where the ADC drifts by <1% per sample, which is
not a real surface transition.

Example: 3000 -> 2940 (2% step OK) -> 2870 (2.4% step OK). Cumulative = (2940-2870)/2940 = 2.4% (not yet).
Next: 2940 -> 2870 -> 2800. Cumulative = (2940-2800)/2940 = 4.8% -> **L2 triggers**.

## Level 3 — TREND (3+ MA3 decreases)

```
Condition: MA3 decreased 3+ consecutive times AND total MA3 drop > 4%
Latency:   ~5-6 samples
Use case:  Noisy signal where bounces break L2's consecutive requirement
```

**Changed:** Total MA3 drop threshold reduced from 5% to **4%** for faster detection
during heavy biofouling where signal swings are small.

Computes a 3-sample moving average of filtered values. If each new MA3 is lower
than the previous, the trend counter increments. A single increase only decrements
the counter by 1 (noise tolerance), preventing a complete reset.

## Level 4 — ABSOLUTE (variable latency)

```
Condition: filtered_value < water_baseline * 92%
Latency:   Variable (depends on how fast readings drop)
Use case:  Moderate biofouling, well-calibrated baselines
```

**Changed:** Threshold relaxed from 85% to **92%** (8% drop instead of 15%). The original
15% was too conservative — many real surface transitions only produce a 10-12% drop
from the water baseline, especially with moderate biofouling.

## Level 5 — SAFETY NET (>10s underwater)

```
Condition: (peak_during_dive - filtered) / peak_during_dive > 10%
           AND time underwater > 10s
Latency:   >10s (intentionally slow, last resort)
Use case:  All baselines have drifted, nothing else triggered
```

**Changed:** Drop threshold reduced from 15% to **10%** for better catch rate as
a safety net.

## Detection Hierarchy

```
Speed:    L1 (1s) > L2 (2s) > L3 (~5s) > L4 (variable) > L5 (>10s)

L1: instant drop from previous raw      -> catches sharp exits
L2: cumulative 2-sample drop (min step)  -> catches gradual exits
L3: MA3 trend                            -> catches noisy gradual exits
L4: absolute vs water baseline           -> catches any exit when baseline is good
L5: peak-relative with time gate         -> last resort safety net
```

---

# Calibration System

## Initial Air Calibration (at boot)

Takes 10 ADC samples, 100ms apart. If average > 2500: assumes device booted
in water and swaps air/water estimates. Otherwise, `air = average`,
`water = air * 3` (capped at both `air + 3000` and observed peak ADC).

**Added:** The water estimate is now also capped at `air + 3000`. This prevents
absurd water estimates when air is already high from dirty pins (e.g., air=1900
-> water would be 5700, but no real electrode produces that gap).

## Water Baseline (EMA, continuous)

When confirmed underwater (filtered > threshold_high), the water baseline adapts:

```
new_water = 0.19 * value + 0.81 * old_water
```

**Protection conditions** (all must be true):
- Value > threshold_current + hysteresis
- Value >= 2000 (absolute minimum for seawater)
- Value >= air * 3 (or air >= 1000, to allow recovery from corrupted calibration)
- Value >= 85% of current water_baseline OR > water_baseline
  - **Exception:** When water baseline is still estimated (no observed peak yet),
    the guard relaxes to `ABSOLUTE_MIN_WATER_ADC` (2000) to allow full adaptation
    from a wildly wrong initial estimate
- Not in surface lockout period

Water baseline is also capped at observed peak ADC.

## Air Baseline Adaptation

Three modes, checked when at surface > 10 seconds **and NOT in surface lockout**:

| Mode | Condition | Action | Speed |
|------|-----------|--------|-------|
| Timed recalibration | Elapsed > CALIB_INTERVAL (3600s) | air = average of surface readings | Immediate |
| Upward adaptation | avg > air * 1.3 AND avg < threshold | air = 0.9*air + 0.1*avg | Slow (10%) |
| Downward adaptation | avg < air * 0.7 | air = 0.8*air + 0.2*avg (min 5) | Fast (20%) |

**Changed:** Surface baseline tracking is now blocked during lockout period to prevent
transitional readings from corrupting the air baseline.

**Added:** Downward adaptation has a floor of 5 ADC counts to prevent air baseline
collapse to zero over very long deployments.

**Added:** `adjust_sample_delay()` is called after any air recalibration to adapt
the RC charge time to the new contrast ratio.

## Surface Override Air Recalibration

When a L1-L5 surface override triggers, the air baseline is updated using a
**conservative EMA** (15% weight) instead of direct replacement:

```
new_air = old_air * 0.85 + filtered_value * 0.15
```

**Changed from:** Direct `air = filtered_value` replacement. The old approach
caused air ratcheting upward on repeated rapid transitions, because the filtered
value at water exit is still near the water baseline (MA2 filter lag). The EMA
approach prevents corruption from transitional readings.

**Hard safety cap:** `new_air` is capped at `water * 0.70` (70%). This guarantees
that `threshold_high` never exceeds ~86% of the water baseline, so actual water
readings (even 10-15% below baseline) still cross the threshold.

The recalibration only applies if `new_air > old_air` (upward only) and `new_air < water`.

## First-Sample Coherence Check

On the first ADC read after boot, if the stored calibration is wildly inconsistent
with the actual reading (e.g., stored air=1000 but reading=4800, or stored air=3000
but reading=500), the calibration is invalidated and rebuilt from the current reading.

## Calibration Persistence

All calibration data (air, water, threshold, hysteresis, timestamp, CRC) is stored in
a single noinit RAM structure with an embedded CRC16 field. Survives MCU soft resets
but not power cycles (unless backed by retained RAM).

**Changed:** The separate `m_calib_crc` static member has been removed. The CRC is
now embedded directly in the `CalibrationData` struct as `m_calib.crc`.

---

# Dynamic Peak ADC Tracking

## Problem

When calibrating in air with wet electrodes:
- Air reads ~1200 (wet film, not true air)
- Water estimated as air * 3 = 3600
- But actual water ADC is only ~3000
- Threshold + hysteresis = 3647 > actual water readings -> never detects underwater

## Solution: m_observed_peak_adc

A persistent tracker (noinit RAM + CRC) records the highest ADC value actually
observed during operation. This value caps:

1. **Water baseline estimation**: `air * 3` capped at observed peak (and at `air + 3000`)
2. **EMA water updates**: new_water capped at observed peak
3. **Threshold + hysteresis**: if threshold_current + hysteresis > observed peak,
   both are reduced proportionally

The peak updates continuously from raw readings. On first boot (peak=0), no capping
is applied — the peak is learned from the first immersion.

---

# Adaptive Sample Delay

## Problem

The fixed 1ms sample delay works well for clean electrodes, but degrades with
biofouling. When contrast drops below 5x, the 1ms reading may not provide enough
discrimination between water and wet film.

## Solution: adjust_sample_delay()

The sample delay adapts continuously based on the current water/air contrast ratio:

| Contrast (×10) | Direction | Adjustment | Reasoning |
|----------------|-----------|------------|-----------|
| < 5.0x (`contrast_x10 < 50`) | Decrease | -25% per step | Shorter delay → better film/water discrimination when gap is small |
| > 10.0x (`contrast_x10 > 100`) | Increase | +10% per step | Longer delay → stronger signal when electrode is clean |
| 5.0x - 10.0x | No change | Stable | Good operating range |

**Bounds:** 100µs (floor) to 5000µs (ceiling). Default: 1000µs (1ms).

The delay is initialized from the `UW_PIN_SAMPLE_DELAY` config parameter (converted from ms to µs) and adjusts after every air baseline recalibration event.

**Implementation:** `PMU::delay_us()` is used instead of `PMU::delay_ms()` for
microsecond-precision timing of the RC charge period.

---

# Safety Mechanisms

## Max Dive Timeout (default: 7200s = 2h)

If underwater longer than `UW_MAX_DIVE_TIME`:
- Forces surface state
- Does NOT recalibrate air baseline (would corrupt it with underwater values)
- Activates 30s surface lockout

## Surface Lockout (time-based)

After a surface override (L1-L5) or max dive timeout:
- Lockout duration compared against **actual time in surface state** (not sample count)
- `UW_MIN_SURFACE_TIME` seconds of forced surface (default 2s) after L-override
- After max dive timeout: 30s lockout (`SURFACE_LOCKOUT_DURATION_SEC`)
- During lockout:
  - Water baseline is NOT updated
  - Air baseline adaptation is blocked
  - Underwater readings are suppressed (forced surface)

**Changed from:** Sample-count-based lockout (decremented per sample). The time-based
approach is more accurate when sampling intervals vary between surface and underwater.

## Proximity Guard (adaptive)

L1-L5 surface overrides are blocked when readings are too close to water peak:

| Contrast | Guard | Min gap to allow override |
|----------|-------|--------------------------|
| >= 5.0x | 95% | 5% below peak |
| < 5.0x (biofouling) | 99% | 1% below peak |

Using `max(water_baseline, peak_during_dive)` handles the case where water EMA
has drifted below actual readings.

## Recent Peak Decay

The `m_recent_peak` decays **2% per sample** toward the current reading (changed from 5%).
The slower decay preserves sensitivity with longer sampling periods (e.g., 10s), where
5%/sample decay would cause the peak to collapse too quickly between samples.

---

# SWS Logger

## Persistent Flash Log (ENABLE_SWS_LOG)

When `ENABLE_SWS_LOG` is defined at build time, every `detector_state()` call writes
a `SWSLogEntry` to flash via the Logger framework. This provides post-deployment analysis
of the SWS algorithm behavior.

## Log Entry Format (CSV)

```
log_datetime,raw_adc,filtered_adc,threshold,hysteresis,air,water,calibrated,underwater,time_in_state,surface_level,contrast_x10,observed_peak,sample_delay_us
```

| Field | Type | Description |
|-------|------|-------------|
| log_datetime | DD/MM/YYYY HH:MM:SS | Timestamp of sample |
| raw_adc | uint16 | Raw 14-bit ADC reading |
| filtered_adc | uint16 | MA2 filtered value |
| threshold | uint16 | Active threshold (ADC counts) |
| hysteresis | uint16 | Hysteresis value (ADC counts) |
| air | uint16 | Air baseline |
| water | uint16 | Water baseline |
| calibrated | uint8 | 1 = calibration valid |
| underwater | uint8 | 1 = underwater, 0 = surface |
| time_in_state | uint16 | Seconds in current state (max 65535) |
| surface_level | uint8 | Detection level (0=none, 1-5=L1-L5) |
| contrast_x10 | uint16 | Water/air ratio × 10 (e.g., 47 = 4.7x) |
| observed_peak | uint16 | Highest ADC value observed |
| sample_delay_us | uint16 | Current adaptive RC delay (µs) |

Retrieve via DTE: `$DUMPD` with the SWS log type.

## Test Mode (DTE SWSTST)

- `$SWSTST,1` — Start test mode: forces SWS service on, LED feedback active
- `$SWSTST,0` — Stop test mode: restores normal LED behavior via `m_on_test_stop` callback

**LED feedback during test mode:**
- **BLUE** = underwater state
- **YELLOW** = surface state

LED updates on every state transition. When test mode stops, the `m_on_test_stop`
callback restores the LED to its normal FSM-driven state.

---

# Parameters Reference

## Configurable Parameters (DTE / Config Store)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `UW_PIN_SAMPLE_DELAY` | **1 ms** | Initial RC charge time (converted to µs internally). Auto-adapts during operation. |
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

## Internal Constants (sws_analog_service.cpp)

### ADC

| Constant | Value | Description |
|----------|-------|-------------|
| `ADC_INVALID_MAX` | 16383 | 14-bit ADC maximum |
| `ADC_HISTORY_SIZE` | 2 | Moving average filter window (small = fast response) |

### Detection Tuning

| Constant | Value | Description |
|----------|-------|-------------|
| `DEFAULT_THRESHOLD_RATIO_PERCENT` | 35% | Threshold position between air and water (clean electrode) |
| `DEFAULT_ALPHA_PERCENT` | 19% | EMA alpha for water baseline adaptation |
| `DEFAULT_MIN_DRY_SAMPLES` | 1 | Immediate surface on threshold crossing |

### Adaptive Sample Delay

| Constant | Value | Description |
|----------|-------|-------------|
| `SAMPLE_DELAY_MIN_US` | 100 µs | Floor: below this, ADC values approach noise |
| `SAMPLE_DELAY_MAX_US` | 5000 µs | Ceiling: above this, biofouled signals converge |
| `SAMPLE_DELAY_DEFAULT_US` | 1000 µs | Default: 1ms (good balance for clean electrode) |
| `CONTRAST_LOW_THRESHOLD` | 50 | contrast_x10 < 5.0x → reduce delay (-25%) |
| `CONTRAST_HIGH_THRESHOLD` | 100 | contrast_x10 > 10.0x → increase delay (+10%) |

### Water Baseline Protection

| Constant | Value | Description |
|----------|-------|-------------|
| `ABSOLUTE_MIN_WATER_ADC` | 2000 | Reject water readings below this (not seawater) |
| `MIN_WATER_AIR_RATIO` | 3 | Water must be >= 3x air (bypassed if air >= 1000) |

### Surface Baseline Adaptation

| Constant | Value | Description |
|----------|-------|-------------|
| `SURFACE_ADAPT_THRESHOLD` | 1.3x | Upward air adaptation trigger (avg > air * 1.3) |
| `MIN_SURFACE_TIME_FOR_ADAPT` | 10 s | Minimum surface time before air adaptation starts |
| Downward threshold | 0.7x | Downward air adaptation trigger (avg < air * 0.7) |
| Downward minimum | 5 | Air baseline cannot go below 5 ADC counts |

### Air Recalibration on L-Override

| Constant | Value | Description |
|----------|-------|-------------|
| `AIR_RECALIB_EMA_WEIGHT` | 0.15 | EMA weight for filtered value (conservative, prevents ratcheting) |
| `AIR_RECALIB_MAX_RATIO` | 0.70 | Hard cap: air must never exceed 70% of water |

### 5-Level Surface Detection

| Constant | Value | Description |
|----------|-------|-------------|
| `L1_DROP_PERCENT` | 4% | Drop from previous raw to trigger L1 (instant) |
| `L2_DROP_PERCENT` | 3% | Cumulative 2-sample drop to trigger L2 |
| `L2_MIN_CONSECUTIVE` | 2 | Minimum consecutive raw drops for L2 |
| `L2_MIN_STEP_PERCENT` | 2% | Each individual L2 step must exceed this (filters drift) |
| `L3_DROP_PERCENT` | 4% | Total MA3 trend drop to trigger L3 |
| `L3_MIN_CONSECUTIVE` | 3 | Consecutive MA3 decreases for L3 |
| `L4_DROP_PERCENT` | 8% | Drop below water baseline for L4 |
| `L5_DROP_PERCENT` | 10% | Drop from dive peak for L5 |
| `L5_MIN_TIME_SEC` | 10 s | Minimum underwater time before L5 activates |

### Safety

| Constant | Value | Description |
|----------|-------|-------------|
| `OVERRIDE_MIN_TIME_SEC` | 1 s | Minimum underwater time before any L-override |
| `PROXIMITY_GUARD_PERCENT` | 95% | Default: L-overrides blocked if filtered >= 95% of peak |
| `PROXIMITY_GUARD_BIOFOULING` | 99% | Relaxed: when contrast < 5x, allow 1% gap |
| `SURFACE_LOCKOUT_DURATION_SEC` | 30 s | Lockout after max dive timeout |
| `WATER_DETECT_HEURISTIC` | 2500 | Air calibration: if avg > this, assume in water |

### Dynamic Threshold Ratio

| Contrast (water/air) | Ratio | Effect |
|----------------------|-------|--------|
| >= 8x (clean) | 35% | Threshold close to air, fast surface detection |
| >= 4x (moderate biofouling) | 50% | Balanced midpoint |
| < 4x (wet electrodes / severe) | 40% | Closer to air to ensure underwater detection |

### Peak ADC Capping (in update_dynamic_threshold)

When `threshold_current + hysteresis > observed_peak_adc`:
- If threshold >= peak: threshold = peak * 90%, hysteresis = peak * 5%
- If threshold < peak: hysteresis = peak - threshold

---

# Troubleshooting

## Device never detects underwater (always surface)

**Check**: `$SWSST` command -> look at `threshold_high` (= threshold + hysteresis)
vs `raw_adc` when submerged.

**Common causes**:
- `threshold_high > raw_adc`: threshold too high. Check if air baseline is inflated
  (wet electrode calibration). The downward air adaptation should fix this over time.
- `observed_peak` = 0: first boot, no peak learned yet. First immersion will set it.
- Very low contrast (water/air < 2x): check electrode connections, verify conductivity.

## Device never detects surface (always underwater)

**Check**: `$SWSST` -> look at `surface_level`. If always 0, no L-override triggers.

**Common causes**:
- Sample delay too high: check `sample_delay_us` in SWSST output. If stuck at max (5000µs),
  the delay may have ratcheted up. A recalibration cycle will adjust it.
- Proximity guard blocking: with contrast >= 5x, readings must drop 5% below peak.
  With contrast < 5x (biofouling), the guard relaxes to 1%.
- Air baseline too low: threshold is far below water, readings stay in hysteresis zone.

## Slow surface detection (>5s)

- Increase sampling frequency: lower `SAMPLING_UNDER_FREQ` to 1-2s
- L1 (instant) should trigger for any drop > 4% from previous raw
- If L1 doesn't trigger, readings are drifting slowly -> L2/L3 should catch it in 2-5s
- Check if proximity guard is blocking (readings very close to water baseline)
- Check `sample_delay_us`: if too high, water/film discrimination may be poor

## Threshold/hysteresis exceed actual water readings

- The `observed_peak_adc` should prevent this. Check its value in `$SWSST`.
- If peak = 0 (first boot), it hasn't been learned yet. Do one immersion cycle.
- If peak is correct but threshold still too high: check air baseline — if inflated,
  the ratio calculation produces an inflated threshold.

## Calibration corrupted after reset

- Calibration is in noinit RAM with embedded CRC16. If CRC fails, fresh calibration runs.
- Power cycle (not soft reset) clears noinit RAM -> full recalibration at boot.
- `observed_peak_adc` has its own CRC and persists independently of calibration.

---

# DTE Test Commands

## $SWSST — Read SWS Status

Returns: `air, water, threshold, hysteresis, raw_adc, filtered_adc, calibrated, underwater, time_in_state, surface_level, contrast_x10, observed_peak, sample_delay_us`

## $SWSTST — Start/Stop Test Mode

- `$SWSTST,1` — Start test mode (continuous sampling with async SWSST push, LED feedback)
- `$SWSTST,0` — Stop test mode (restores normal LED via on_test_stop callback)

In test mode:
- SWSST data is pushed on every sample regardless of state change
- LED shows current state: **BLUE** = underwater, **YELLOW** = surface
- LED updates on state transitions

---

*Source files*:
- `core/services/sws_analog_service.hpp` — Header with SWSLogEntry, SWSLogFormatter
- `core/services/sws_analog_service.cpp` — Implementation (~950 lines)
- `core/services/uwdetector_service.hpp/.cpp` — Parent class (scheduling)

*Last updated: 2026-03-20*
