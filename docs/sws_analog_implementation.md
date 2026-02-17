# SWS Analog - Surface/Underwater Detection Algorithm with Biofouling

## Table of Contents

1. [Context and Problem Statement](#context-and-problem-statement)
2. [Parent-Child Architecture](#parent-child-architecture)
3. [Detection Algorithm](#detection-algorithm)
4. [Rapid Transition Detection (3 Tiers)](#rapid-transition-detection-3-tiers)
5. [Dynamic Biofouling Adaptation](#dynamic-biofouling-adaptation)
6. [Safeties and Timeouts](#safeties-and-timeouts)
7. [Parameters and Constants](#parameters-and-constants)
8. [Unit Tests](#unit-tests)
9. [Lifecycle Performance](#lifecycle-performance)

---

## Context and Problem Statement

### SWS Function

The Salt Water Switch (SWS) detects whether a sea turtle tracker is underwater or at the surface.
Two electrodes measure the conductivity of the medium via a 14-bit ADC (0-16383) on nRF52840.

```
     VDD
      |
   [Electrode TX] ---> [salt water = conductor] ---> [Electrode RX]
                                                          |
                                                       ADC 14-bit
                                                     (0 = air, ~3000 = water)
```

- **Dry air**: ADC ~ 100-300 (low conductivity)
- **Salt water**: ADC ~ 2500-4000 (high conductivity)

### The Problem: Biofouling

After months of deployment at sea, salt and biofilm accumulate on the electrodes:

```
Month 0  (clean)    : Air = 200,  Water = 3000  → gap = 2800  (93% drop)
Month 3  (light)    : Air = 600,  Water = 2800  → gap = 2200  (78% drop)
Month 6  (moderate) : Air = 1000, Water = 2500  → gap = 1500  (60% drop)
Month 9  (heavy)    : Air = 1400, Water = 2300  → gap = 900   (39% drop)
Month 12 (severe)   : Air = 1700, Water = 2100  → gap = 400   (19% drop)
```

A fixed threshold inevitably fails. The algorithm must be **adaptive** and detect the surface
**in under 2 seconds** even with extreme biofouling.

### Fundamental Insight

Even with extreme biofouling, when the turtle exits the water, the electrode **loses contact
with the water immediately**. The salt/biofilm layer remains conductive, but the break in the
conductivity path through the water always causes a **significant relative drop** in the ADC.

It is this **rate of change** (not the absolute value) that the algorithm exploits.

---

## Parent-Child Architecture

### Class Hierarchy

```
UWDetectorService (parent)
        |
        |  inherits
        v
SWSAnalogService (child)
```

**Files**:
- `core/services/uwdetector_service.hpp/.cpp` - Batch management and scheduling
- `core/services/sws_analog_service.hpp/.cpp` - Detection algorithm

### Batch Operation

The parent (`UWDetectorService`) manages a batch system:

```
                    Batch (3 samples)
         ┌─────────────┼──────────────┐
         v             v              v
    detector_state() detector_state() detector_state()
         |             |              |
         v             v              v
    sample 1        sample 2       sample 3
         |             |              |
         └─────────────┼──────────────┘
                       v
              m_current_state updated
              (ONLY time state changes)
```

**Critical detail**: `m_current_state` is updated **only at the end of the batch**.
During a batch, `detector_state()` is called 3 times (`UW_MAX_SAMPLES`), but all
3 calls see the SAME `m_current_state` (the value from the end of the previous batch).

This has a major consequence for rapid detection: if the first call detects
the surface (rapid drop), the next two calls still see `m_current_state = true`
(underwater). Without special handling, they could return `true` (underwater)
and cancel the rapid detection.

**Implemented solution**: the `m_rapid_surface_confirmed` flag (see next section).

---

## Detection Algorithm

### Overview

Each call to `detector_state()` executes these steps in order:

```
┌──────────────────────────────────────────────────────────┐
│ 1. ADC READ                                              │
│    raw_value = read_analog_sws()                         │
│    Validation: 50 <= raw <= threshold_max                │
├──────────────────────────────────────────────────────────┤
│ 2. FILTERING                                             │
│    filtered_value = moving_average(raw, history_size=2)  │
├──────────────────────────────────────────────────────────┤
│ 3. TREND DETECTION                                       │
│    - Circular buffer (8 samples)                         │
│    - Consecutive decrease counter                        │
│    - Total drop calculation (max - min of buffer)        │
│    - Cumulative drop calculation (from peak underwater)  │
├──────────────────────────────────────────────────────────┤
│ 4. VARIANCE DETECTION                                    │
│    - Mean and sum of squared deviations                  │
│    - High variance = surface (unstable drying)           │
│    - Low variance = underwater (stable)                  │
├──────────────────────────────────────────────────────────┤
│ 5. ADAPTIVE AIR RECALIBRATION (if at surface > 10s)      │
│    - Surface readings buffer (10 samples)                │
│    - If mean > 1.3x air_baseline → recalibrate           │
│    - Gradual adaptation (10% per update)                 │
├──────────────────────────────────────────────────────────┤
│ 6. RAPID TRANSITION DETECTION (TIER 1/2/3)               │
│    - Drop % calculated on raw_value (not filtered!)      │
│    - 3 sensitivity levels (see next section)             │
├──────────────────────────────────────────────────────────┤
│ 7. BIOFOULING SURFACE OVERRIDE                           │
│    - If rapid drop OR (trend + variance) → force surface │
│    - Reset ADC buffer after rapid detection              │
│    - Automatic air baseline recalibration                │
├──────────────────────────────────────────────────────────┤
│ 8. HYSTERESIS DECISION                                   │
│    - If biofouling_override → surface                    │
│    - If rapid_surface_confirmed → surface (bypass)       │
│    - If filtered > threshold_high → underwater           │
│    - If filtered < threshold_low → surface               │
│    - Otherwise → maintain previous state                 │
├──────────────────────────────────────────────────────────┤
│ 9. SAFETIES                                              │
│    - Max dive time → force surface                       │
│    - Surface lockout → prevent re-submersion             │
│    - Extended dive recalibration → adjust baselines      │
└──────────────────────────────────────────────────────────┘
```

### Dynamic Threshold

The threshold is positioned between the air baseline and the water baseline:

```
ADC
 ^
 |
 |  ┌─── threshold_water (EMA, alpha=0.19)
 |  |
 |  |  ──── threshold_high = threshold_current + hysteresis
 |  |
 |  |  ──── threshold_current = air + ratio% × (water - air)
 |  |                           ratio = 35%
 |  |  ──── threshold_low  = threshold_current - hysteresis
 |  |                         hysteresis = 10% of threshold
 |  |
 |  └─── threshold_air (calibrated in air, adapts to biofouling)
 |
 0 ──────────────────────────────────────────> time
```

The 35% ratio places the threshold closer to air than to water.
This favors **rapid surface detection** at the expense of slightly slower
submersion detection (acceptable for the turtle use case).

---

## Rapid Transition Detection (3 Tiers)

### Principle

Instead of waiting for the filtered ADC to drop below the threshold (slow with biofouling),
we detect the **instantaneous rate of change** of the raw ADC (unfiltered).

```
ADC
 ^
 |  3000 ●─────────── underwater ────────────────●
 |                                             │
 |                                             │ DROP 60%
 |                                             │ (1 sample)
 |                                             │
 |  1200                                       ●─── surface (biofouling)
 |
 |   200 ─────────── threshold (with high biofouling)
 |
 0 ──────────────────────────────────────────> time
          ^                                ^
          |        classic detection       |
          |        would take 15+ sec      |
          |                                |
          |   RAPID T1 detects in 1 sample |
```

### The 3 Tiers

The algorithm uses the **raw_value** (not filtered) to calculate the drop:

```
drop_percent = (prev_trend_value - raw_value) * 100 / prev_trend_value
absolute_drop = prev_trend_value - raw_value
```

| Tier | Min drop % | Min abs drop | Additional condition | Latency |
|------|-----------|-------------|-------------------------|---------|
| **T1** | >= 25% | >= 300 | None | **1 sample** |
| **T2** | >= 12% | >= 150 | trend_count >= 1 AND below_water_baseline | **2 samples** |
| **T3** | >= 8% | >= 100 | trend_count >= 2 | **3+ samples** |

**Conditions for rapid detection to activate**:
- `m_current_state == true` (currently "underwater")
- `prev_trend_value > 0` (we have a previous sample)
- `raw_value < prev_trend_value` (ADC has decreased)

### Post-Detection: ADC Buffer Reset

After rapid detection, the ADC history buffer is **reset** with
the current raw_value:

```cpp
// CRITICAL: without this reset, the moving average filter retains
// the old underwater values, producing a high filtered_value
// on the next call. Since m_current_state is not yet updated
// (end of batch), the next detector_state() would see
// filtered > threshold → return "underwater", cancelling the detection.
for (int i = 0; i < ADC_HISTORY_SIZE; i++) {
    m_adc_history[i] = raw_value;
}
```

### Post-Detection: `m_rapid_surface_confirmed` Flag

When the biofouling_surface_override triggers, the `m_rapid_surface_confirmed`
flag is set to `true`. Subsequent calls to `detector_state()` within the SAME batch
check this flag:

```cpp
if (biofouling_surface_override) {
    new_state = false;                    // Surface!
    m_rapid_surface_confirmed = true;     // Flag for remaining batch samples
} else if (m_rapid_surface_confirmed && filtered_value < threshold_high) {
    new_state = false;                    // Confirm surface without re-verification
} else if (filtered_value > threshold_high) {
    m_rapid_surface_confirmed = false;    // Clear if back underwater
    // ... classic detection
}
```

This mechanism solves the batch problem:
1. Batch sample 1: rapid drop detected → `m_rapid_surface_confirmed = true`, returns `false`
2. Batch sample 2: `m_current_state` is still `true` (not yet updated), but the flag
   bypasses verification → returns `false`
3. Batch sample 3: same
4. End of batch: parent sees 3x `false` → `m_current_state = false`

---

## Dynamic Biofouling Adaptation

### 5 Adaptation Mechanisms

The algorithm has 5 mechanisms that progressively adapt to biofouling:

#### 1. EMA Water Baseline (continuous)

The water baseline adapts continuously via an exponential moving average:

```
threshold_water = alpha * value + (1 - alpha) * threshold_water
alpha = 0.19 (19%)
```

**Strict conditions** to avoid corruption by biofouling:
- Value > threshold_current + hysteresis (clearly underwater)
- Value >= 2000 (absolute minimum for sea water)
- Value >= 5 x air_baseline (minimum water/air ratio)
- Value within expected range (+-15% of water_baseline)

#### 2. Adaptive Air Recalibration (at surface > 10s)

When at the surface for > 10 seconds, ADC readings are stored in a buffer.
If the average exceeds 1.3x the current air baseline → biofouling detected:

```
new_air = 0.9 * old_air + 0.1 * avg_surface_reading
```

Gradual adaptation (10% per update) to avoid false adjustments.

#### 3. Extended Dive Recalibration (underwater > 60s)

Every 30 seconds after 60s underwater, checks whether minimum readings
suggest biofouling (air baseline too low for reality):

```
Condition: min_adc_during_dive > 3 * air_baseline
           AND min_adc < 0.7 * water_baseline
           AND shift > 50%

Action: air_baseline = min_adc * 0.85
```

#### 4. Rapid Detection Air Recalibration (immediate)

When a rapid drop is detected and the raw_value is above the air baseline:

```
air_baseline = raw_value * 0.8
```

This allows the threshold to adapt immediately after the first rapid detection.

#### 5. Max Dive Timeout Recalibration (safety)

If max dive time is reached, it's likely a biofouling issue.
Forces recalibration:

```
Condition: min_adc_during_dive > 2 * air_baseline
Action: air_baseline = min_adc * 0.8
```

### Threshold Evolution Over 12 Months

With adaptive mechanisms active, here is the measured evolution from tests:

```
Stage              | air_baseline | water_baseline | threshold | detection
───────────────────┼──────────────┼────────────────┼───────────┼──────────
Month 0  (clean)   |     200      |     3000       |    499    | T1, 1 sample
Month 3  (light)   |     480*     |     2800       |    628    | T1, 1 sample
Month 6  (moderate)|     800*     |     2500       |    733    | T1, 1 sample
Month 9  (heavy)   |    1120*     |     2300       |   1077    | T1, 1 sample
Month 12 (severe)  |    1360*     |     2100       |   1617    | T2, 2 samples
```

(*) Baselines automatically recalibrated by adaptive mechanisms.

---

## Safeties and Timeouts

### Max Dive Time (default: 7200s = 2h)

If the tracker stays "underwater" longer than `UW_MAX_DIVE_TIME`:
1. Forces surface state
2. Recalibrates air baseline (biofouling suspected)
3. Activates a **surface lockout** of 30s (prevents immediate return to underwater)

### Min Surface Time (default: 10s)

Prevents false returns to underwater if just surfaced.

### Surface Lockout (30s after max dive timeout)

After a force-surface by timeout, prevents underwater re-detection for 30s.
Gives the electrodes time to stabilize at the surface.

### Hysteresis Stuck Recalibration (30s in hysteresis zone)

If the ADC stays in the hysteresis zone for > 30s, progressively recalibrates
the air baseline toward the current value.

### ADC Validation

Every ADC reading is validated:
- `value >= 50` (not zero/noise)
- `value <= threshold_max` (not saturated)

If invalid, the previous state is maintained.

---

## Parameters and Constants

### Configurable Parameters (DTE / Config Store)

| Parameter | ParamID | Default | Description |
|-----------|---------|---------|-------------|
| `SWS_ANALOG_THRESHOLD_MIN` | UNP20 | 100 | Minimum valid ADC |
| `SWS_ANALOG_THRESHOLD_MAX` | UNP21 | 3000 | Maximum valid ADC |
| `SWS_ANALOG_HYSTERESIS` | UNP22 | 10% | Hysteresis in % |
| `SWS_ANALOG_CALIB_INTERVAL` | UNP23 | 3600s | Recalibration interval |
| `UW_MAX_DIVE_TIME` | UNP24 | 7200s | Max dive time (0=disabled) |
| `UW_MIN_SURFACE_TIME` | UNP25 | 10s | Min surface time |

### Internal Constants (defined in sws_analog_service.cpp)

#### Detection

| Constant | Value | Role |
|----------|-------|------|
| `ADC_HISTORY_SIZE` | 2 | Moving average buffer size (small = fast) |
| `DEFAULT_THRESHOLD_RATIO_PERCENT` | 35% | Threshold position between air and water |
| `DEFAULT_ALPHA_PERCENT` | 19% | EMA water baseline adaptation speed |
| `DEFAULT_MAX_SAMPLES` | 1 | Submersion confirmation (1 = immediate) |
| `DEFAULT_MIN_DRY_SAMPLES` | 2 | Surface confirmation (2 samples) |

#### Rapid Transition

| Constant | Value | Role |
|----------|-------|------|
| `RAPID_DROP_TIER1_PERCENT` | 25% | T1 threshold: single-sample drop |
| `RAPID_DROP_TIER1_ABSOLUTE` | 300 | T1: minimum absolute drop |
| `RAPID_DROP_TIER2_PERCENT` | 12% | T2 threshold: 2x confirmed drop |
| `RAPID_DROP_TIER2_ABSOLUTE` | 150 | T2: minimum absolute drop |
| `RAPID_DROP_TIER3_PERCENT` | 8% | T3 threshold: trend-supported drop |
| `RAPID_DROP_TIER3_ABSOLUTE` | 100 | T3: minimum absolute drop |
| `BIOFOULING_OVERRIDE_MIN_TIME_SEC` | 3s | Min time underwater before override |

#### Biofouling Adaptation

| Constant | Value | Role |
|----------|-------|------|
| `ABSOLUTE_MIN_WATER_ADC` | 2000 | Absolute minimum for sea water |
| `MIN_WATER_AIR_RATIO` | 5 | Minimum water/air ratio |
| `SURFACE_ADAPT_THRESHOLD` | 1.3 | Air recalibration trigger threshold at surface |
| `MIN_SURFACE_TIME_FOR_ADAPT` | 10s | Min surface time before adaptation |
| `EXTENDED_DIVE_RECALIB_START_SEC` | 60s | Start biofouling check underwater |
| `EXTENDED_DIVE_RECALIB_INTERVAL_SEC` | 30s | Biofouling check frequency |

#### Trend and Variance

| Constant | Value | Role |
|----------|-------|------|
| `TREND_BUFFER_SIZE` | 8 | Trend buffer size |
| `TREND_DECREASE_THRESHOLD_PERCENT` | 2% | Threshold to count a decrease |
| `TREND_DECREASE_ABSOLUTE_MIN` | 8 | Absolute minimum to count decrease |
| `TREND_CONSECUTIVE_DECREASE_MIN` | 3 | Consecutive decreases for trend |
| `TREND_TOTAL_DROP_PERCENT` | 10% | Min total drop for surface trend |
| `CUMULATIVE_DROP_PERCENT` | 15% | Cumulative drop for surface |
| `VARIANCE_HIGH_THRESHOLD` | 10000 | High variance = surface |

---

## Unit Tests

### Test Suite (14/14 passing)

**File**: `tests/src/sws_analog_test.cpp`

#### Basic Functional Tests

| # | Test | ADC | Validation |
|---|------|-----|------------|
| 1 | `InitialAirCalibration` | 200 (air) | Initial calibration correct |
| 2 | `SurfaceDetection` | 150 (surface) | Surface detection OK |
| 3 | `UnderwaterDetection` | 200→2500 | Submersion detection OK |
| 4 | `HysteresisPreventOscillation` | 260 (hysteresis zone) | No oscillation |
| 5 | `SalinityAdaptation` | 2500→2800 | Salinity adaptation OK |
| 6 | `MaxDiveTimeSafety` | 2500 for >5s | Force surface after timeout |
| 7 | `InvalidADCValuesHandling` | 3500 (invalid) | Out-of-range value rejected |

#### Rapid Detection Tests

| # | Test | ADC water→air | Drop | Tier | Samples |
|---|------|-------------|------|------|---------|
| 8 | `RapidSurfaceDetection_CleanSensor` | 3000→150 | 95% | T1 | <= 3 |
| 9 | `RapidSurfaceDetection_ModerateBiofouling` | 3000→1200 | 60% | T1 | <= 3 |
| 10 | `RapidSurfaceDetection_HeavyBiofouling` | 2800→1800 | 35% | T1 | <= 3 |

#### False Positive Prevention Tests

| # | Test | Scenario | Validation |
|---|------|----------|------------|
| 11 | `NoFalsePositive_UnderwaterFluctuation` | ADC +-10% underwater | Stays underwater |
| 12 | `NoFalsePositive_GradualSalinityChange` | 3000→2500 gradual | Stays underwater |

#### Progressive Degradation Tests

| # | Test | Scenario | Validation |
|---|------|----------|------------|
| 13 | `ProgressiveBiofouling_MultiCycleDegradation` | 5 biofouling stages | Detection <= 3 samples at EVERY stage |
| 14 | `ExtremeBiofouling_SmallGap` | 1800→1500 (16% drop) | Detection <= 5 samples |

### Execution

```bash
cd tests/build
cmake .. -DFETCHCONTENT_SOURCE_DIR_CPPUTEST=/tmp/cpputest_cache
make -j$(nproc)

# All SWS tests
./CLSGenTrackerTests -g SWSAnalog

# A specific test
./CLSGenTrackerTests -g SWSAnalog -n ProgressiveBiofouling_MultiCycleDegradation -v
```

---

## Lifecycle Performance

### Test Results (actual measurements)

```
Output from ProgressiveBiofouling_MultiCycleDegradation test:

Stage              | water→air   | Drop % | Tier | Latency  | Adapted threshold
───────────────────┼─────────────┼────────┼──────┼──────────┼──────────────────
Month 0  (clean)   | 3000→200    |  93%   | T1   | 1 sample | 499 → 628
Month 3  (light)   | 2800→600    |  78%   | T1   | 1 sample | 1083 → 1286
Month 6  (moderate)| 2500→1000   |  60%   | T1   | 1 sample | 1267 → 1409
Month 9  (heavy)   | 2300→1400   |  39%   | T1   | 1 sample | 1409 → 1617
Month 12 (severe)  | 2100→1700   |  19%   | T2   | 2 samples| 1617 → 1773

Output from ExtremeBiofouling_SmallGap test:
Extreme            | 1800→1500   |  16%   | T2   | 2 samples| 1573
```

### Theoretical Limits

| Drop % | Abs drop | Tier | Latency | Equivalent biofouling |
|--------|----------|------|---------|-----------------------|
| >= 25% | >= 300 | T1 | 1s | Up to ~9 months |
| >= 12% | >= 150 | T2 | 2s | Up to ~15 months |
| >= 8% | >= 100 | T3 | 3s+ | Extreme case |
| < 8% | < 100 | Trend/timeout | 15-30s | Nearly dead electrodes |

Beyond 15 months (drop < 8%), the algorithm falls back to:
- Decreasing trend detection (TREND_CONSECUTIVE_DECREASE_MIN = 3)
- Variance detection (VARIANCE_HIGH_THRESHOLD = 10000)
- Max dive timeout as last resort (default 2h)

### Complete Decision Diagram

```
                    detector_state() called
                           |
                    ┌──────┴──────┐
                    │ Read ADC    │
                    │ raw_value   │
                    └──────┬──────┘
                           |
                    ┌──────┴──────┐
                    │ Valid?      │──── No ──→ return m_current_state
                    └──────┬──────┘
                           | Yes
                    ┌──────┴──────┐
                    │ Filter      │
                    │ (MA size=2) │
                    └──────┬──────┘
                           |
                    ┌──────┴──────┐
                    │ Trend +     │
                    │ Variance    │
                    └──────┬──────┘
                           |
              ┌────────────┴────────────┐
              │ Currently underwater    │
              │ (m_current_state=true)  │
              └────────────┬────────────┘
                      Yes  |  No
                    ┌──────┴──────┐      ┌──────────────┐
                    │ RAW drop?   │      │ Adaptation   │
                    └──┬──────┬───┘      │ air baseline │
                 Yes   |      | No       └──────┬───────┘
           ┌───────────┘      └──────┐          |
    ┌──────┴──────┐          ┌───────┴────┐     |
    │ T1/T2/T3?   │          │ Trend/Var? │     |
    └──────┬──────┘          └───────┬────┘     |
           | Yes                     | Yes      |
    ┌──────┴──────────┐      ┌───────┴────┐     |
    │ RAPID OVERRIDE  │      │ BIOFOULING │     |
    │ Reset ADC buf   │      │ OVERRIDE   │     |
    │ Recalib air     │      └───────┬────┘     |
    │ Set flag        │              |           |
    └──────┬──────────┘              |           |
           └──────────┬──────────────┘           |
                      v                          |
              ┌───────────────┐                  |
              │ new_state =   │                  |
              │ false (SURFACE)│                 |
              └───────┬───────┘                  |
                      |                          |
                      └──────────┬───────────────┘
                                 v
                      ┌──────────────────┐
                      │ Hysteresis check │
                      │ + Safety timeout │
                      └────────┬─────────┘
                               v
                        return new_state
```

---

*Source files*:
- `core/services/sws_analog_service.hpp` - Header (187 lines)
- `core/services/sws_analog_service.cpp` - Implementation (826 lines)
- `tests/src/sws_analog_test.cpp` - Unit tests (746 lines, 14 tests)

*Last updated: 2026-02-17*
