# Overview

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

# How It Works

1. The **UW Detector Service** samples the saltwater switch at a configurable
   interval (`SAMPLING_SURF_FREQ` at surface, `SAMPLING_UNDER_FREQ` underwater)
2. When a state change is confirmed, the scheduler is notified via
   `notify_underwater_state()`
3. **Submerged**: Argos TX and GNSS are descheduled (no satellite communication
   possible underwater)
4. **Surfaced**: After `DRY_TIME_BEFORE_TX` seconds, Argos TX is rescheduled
   and GNSS can acquire a new fix

## Scheduler Integration

When the UW detector confirms a state change, it fires a `service_complete()`
callback with the new state. The schedulers react as follows:

**ArgosScheduler** (`notify_underwater_state`):
- **Submerged** (`state=true`): calls `deschedule()` — all pending Argos TX are cancelled
- **Surfaced** (`state=false`): sets `m_earliest_tx = now + dry_time_before_tx`, then `reschedule()` — Argos TX resumes after the configured surface delay

**GPSScheduler** (`notify_underwater_state`):
- **Submerged**: cancels ongoing GNSS acquisition, suspends scheduling
- **Surfaced**: resumes GNSS scheduling, allows new fix acquisition


---

# Detection Sources

The tracker supports multiple methods to detect underwater state:

| Source | ID | Description |
|--------|----|-------------|
| SWS | 0 | Saltwater switch (analog conductivity electrode) |
| PRESSURE_SENSOR | 1 | Pressure sensor (depth > threshold) |
| GNSS | 2 | GNSS signal quality (poor signal = submerged) |
| SWS_GNSS | 3 | Hybrid: SWS for transitions, GNSS for surface confirmation |

For marine turtle deployments, **SWS (Saltwater Switch)** is the recommended
and most robust method.

## Interaction with Battery Modes

Underwater detection operates independently of battery mode:
- In **Normal mode**: underwater suspends both GNSS and Argos TX
- In **Low Battery mode**: underwater suspends Doppler TX
- In **Critical mode**: device is off regardless


---

# Saltwater Switch (SWS) Algorithm

The SWS analog algorithm (`SWSAnalogService`) is the primary underwater detection
method for marine deployments. It uses RC time constant discrimination to measure
the electrical conductivity between two electrodes.

## Hardware Principle

A 100nF capacitor charges through the medium when the electrode is enabled.
The charge rate depends on the medium's resistance:

```
  Enable SWS pin -> wait adaptive delay (100-5000µs) -> read 14-bit ADC -> disable SWS pin

  Water (R~10k, tau=1ms):   at 1ms = 63% charged -> ADC ~2500-3000
  Wet film (R~50k, tau=5ms): at 1ms = 18% charged -> ADC ~500-1000
  Air (R=inf):               at 1ms = 0% charged  -> ADC ~0-300
```

The sample delay is now **adaptive** (100-5000µs, default 1ms) and adjusts based
on electrode contrast. Shorter delay improves discrimination when biofouling
narrows the water/air gap; longer delay gives stronger signal on clean electrodes.

Between samples (1s apart), the 1M pull-down fully discharges the cap
(tau=100ms, 5*tau=500ms < 1000ms).


---

# Auto-Calibration

## Initial Calibration (at boot)

```
  Boot
    |
    v
  Take 10 ADC samples, 100ms apart
    |
    v
  avg > 2500? --> Started in water: water=avg, air=avg/3
  avg <= 2500? --> Started in air: air=avg, water=min(air*3, air+3000, observed_peak)
    |
    v
  Calculate threshold and store in noinit RAM with CRC16
```

## Dynamic Water Baseline (Exponential Moving Average)

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

## Air Baseline Adaptation (at surface > 10s)

| Direction | Trigger | Formula | Rate |
|-----------|---------|---------|------|
| Upward (biofouling) | avg > air * 1.3 | air = 0.9*air + 0.1*avg | 10%/update |
| Downward (wet calib fix) | avg < air * 0.7 | air = 0.8*air + 0.2*avg | 20%/update |
| Timed recalibration | elapsed > 3600s | air = avg | 100% |

## Calibration Persistence

- Stored in noinit RAM (survives MCU soft reset)
- Protected by CRC16 checksum — corrupt data triggers fresh calibration
- Observed peak ADC stored separately with its own CRC


---

# Dynamic Threshold

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

## Dynamic Peak ADC Capping

The observed peak ADC (highest value ever read) caps:
- Water baseline estimates (air*3 never exceeds actual peak)
- threshold + hysteresis (never exceeds what the ADC produces)

This prevents the case where calibrating with wet electrodes (air=1200)
would estimate water=3600 and push threshold above actual readings (~3000).


---

# 5-Level Surface Detection

Five independent methods detect surfacing, ordered from fastest to slowest.
All require >= 1s underwater and adaptive proximity guard OK
(filtered < 95% of peak, relaxed to 99% when contrast < 5x for biofouled electrodes).

| Level | Speed | Condition | Use Case |
|-------|-------|-----------|----------|
| **L1** | 1 sample | Drop > 4% from previous raw | Sharp water exit |
| **L2** | 2 samples | 2 consecutive drops, cumulative > 3% | Gradual exit |
| **L3** | 3+ samples | MA3 trend down 3x, total > 4% | Noisy signal |
| **L4** | Variable | filtered < water * 92% | Absolute reference |
| **L5** | >10s | Drop > 10% from dive peak | Safety net |

When any level triggers:
1. State forced to surface
2. Air baseline updated via conservative EMA (15% weight, capped at 70% of water)
3. Surface lockout activated (UW_MIN_SURFACE_TIME, time-based)
4. Adaptive sample delay recalculated

For full algorithm details, see [13-SWS-Algorithm.md](13-SWS-Algorithm.md).


---

# Safety Mechanisms

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

  Proximity Guard (adaptive):
    Normal (contrast >= 5x): blocked if filtered >= 95% of max(water, dive_peak)
    Biofouling (contrast < 5x): relaxed to 99% — allows 1% gap detection
    Prevents false surface detection from normal underwater ADC noise
```


---

# Session Timeline Examples

## Normal dive cycle (clean electrode):

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
           L1: drop from prev_raw 2900 to 1500 = 48% > 4% -> SURFACE!
           Air EMA: 15% toward 1500, capped at 70% of water
           -> ArgosScheduler: reschedule()
           -> GPSScheduler: resume

  T+145s   SWS reads ADC = 300 (dried) -> still SURFACE
           Air baseline adapts down: 1500 -> 0.8*1500 + 0.2*300 = 1260
```

## Dive with biofouling (after months deployed):

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
           L1: (2780-2100)/2780 = 24% > 4% -> SURFACE in 1s!

  T+80s    At surface for 10s, readings settling ~1100
           Upward air adaptation: 1000*0.9 + 1100*0.1 = 1010
           Threshold adjusts for next cycle
```


---

# Argos TX Modes for Underwater Trackers

The Argos mode (`ARP01`) determines how satellite transmissions are scheduled.
For underwater trackers, the mode interacts with the dive/surface cycle.

| Mode | Value | Description |
|------|-------|-------------|
| OFF | 0 | No TX |
| PASS_PREDICTION | 1 | TX only during predicted satellite passes (requires AOP data) |
| LEGACY | 2 | Periodic TX at TR_NOM interval, regardless of satellite position |
| DUTY_CYCLE | 3 | TX only during enabled hours (24-bit bitmask, 1 bit per UTC hour) |
| DOPPLER | 4 | Doppler-only packets (3 bytes, no GPS) |
| SURFACING_BURST | 5 | Doppler burst on surface, then GNSS when fix available |

## Duty Cycle Mode (ARP01=3)

The 24-bit `DUTY_CYCLE` parameter (`ARP18`) controls which hours of the day TX is allowed:

```
Bit:   23 22 21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
Hour:   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23
```

Examples:
- `0xFFFFFF` — TX allowed 24/7 (equivalent to legacy mode)
- `0xFF0000` — TX allowed hours 0-7 UTC only
- `0x0F0F0F` — TX allowed hours 4-7, 12-15, 20-23 UTC

The scheduler scans forward from the current time to find the next valid TX slot within
the duty cycle mask. TX within permitted hours follows `TR_NOM` interval.

## Surfacing Burst Mode (ARP01=5)

Combines Doppler and GNSS transmissions for marine animals that surface briefly (turtles, seals). On surfacing, the device immediately sends rapid Doppler messages with progressive intervals, allowing satellites to compute a Doppler-based position while the GPS acquires a fix.

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
  +-- GNSS FIX ACQUIRED
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
- **Duty cycle** is NOT enforced during Doppler phase, only in GNSS phase
- **Low Battery / Zone modes** can also use SURFACING_BURST (LBP04=5, ZOP11=5)
- If no GNSS fix is acquired before next dive, only Doppler data is available

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

### DTE Configuration Example

```
 Enable Surfacing Burst mode
$PARMW#007;ARP01=5\r

 Progressive Doppler: 5s initial, +1s step, max 30s
$PARMW#015;ARP40=5,ARP41=1,ARP42=30\r

 GNSS phase: 60s interval
$PARMW#008;ARP05=60\r

 Enable underwater detection with SWS, no dry time delay
$PARMW#00E;UNP01=1,UNP02=0\r
```

## Interaction with Underwater Detection

All Argos modes interact with the UW detector:
- **Submerged:** Argos TX descheduled regardless of mode
- **Surfaced:** TX resumes after `DRY_TIME_BEFORE_TX` seconds
- **Surfacing Burst:** First Doppler sent immediately (DRY_TIME_BEFORE_TX=0 recommended)


---

# Parameters Reference

## Underwater Detection

| Parameter | DTE Key | Type | Default | Description |
|-----------|---------|------|---------|-------------|
| UNDERWATER_EN | UNP01 | BOOL | 0 | Enable underwater detection |
| DRY_TIME_BEFORE_TX | UNP02 | UINT | 0 | Seconds at surface before TX allowed |
| SAMPLING_UNDER_FREQ | UNP03 | UINT | 10 | Sampling interval underwater (seconds) |
| SAMPLING_SURF_FREQ | UNP04 | UINT | 10 | Sampling interval at surface (seconds) |
| UW_MAX_SAMPLES | UNP05 | UINT | 1 | Samples per detection cycle |
| UW_MIN_DRY_SAMPLES | UNP06 | UINT | 1 | Consecutive dry samples to confirm surface |
| UW_SAMPLE_GAP | UNP07 | UINT | 1000 | Gap between batch sub-samples (ms) |
| UW_PIN_SAMPLE_DELAY | UNP08 | UINT | 1 | Initial RC charge time (ms). Auto-adapts 100-5000µs based on contrast. |
| UNDERWATER_DETECT_SOURCE | UNP10 | ENUM | 0 | 0=SWS, 1=Pressure, 2=GNSS, 3=SWS+GNSS |
| UNDERWATER_DETECT_THRESH | UNP11 | FLOAT | 1.1 | Threshold for pressure/GNSS methods |

## SWS Analog Calibration

| Parameter | DTE Key | Type | Default | Description |
|-----------|---------|------|---------|-------------|
| SWS_ANALOG_THRESHOLD_MIN | UNP20 | UINT | 0 | Minimum valid ADC reading |
| SWS_ANALOG_THRESHOLD_MAX | UNP21 | UINT | 8000 | Max valid ADC (legacy, superseded by observed peak) |
| SWS_ANALOG_HYSTERESIS | UNP22 | UINT | 4 | Hysteresis as % of threshold |
| SWS_ANALOG_CALIB_INTERVAL | UNP23 | UINT | 3600 | Air baseline recalibration interval (seconds) |
| UW_MAX_DIVE_TIME | UNP24 | UINT | 7200 | Force surface after this duration (seconds, 0=disabled) |
| UW_MIN_SURFACE_TIME | UNP25 | UINT | 2 | Surface lockout after detection (seconds) |


---

# Troubleshooting

## Underwater detection not working
- Check `UNDERWATER_EN` (UNP01) is 1
- Check `UNDERWATER_DETECT_SOURCE` (UNP10) is 0 (SWS)
- Verify SWS electrode wiring matches BSP definition
- Read `$SWSST` -> check that threshold_high < actual water ADC readings

## Device thinks it's always underwater
- Likely biofouling — reduce `UW_MAX_DIVE_TIME` to force periodic surface
- Check air baseline via `$SWSST` — if very high, biofouling or wet calibration
- The downward air adaptation (avg < air * 0.7) should auto-correct over time

## Device never detects dives
- Check `$SWSST`: if threshold_high > water readings, threshold is too high
- Check observed_peak: if 0, no peak learned yet (first boot needs one immersion)
- Check `sample_delay_us` in `$SWSST` output — delay auto-adapts

## Surface detection too slow
- L1 should detect in 1 sample for any drop > 4% from previous raw
- If readings drop slowly (biofouling), L2/L3 catch it in 2-5 samples
- Lower `SAMPLING_UNDER_FREQ` to 1s for faster sampling
- Check proximity guard: readings must drop below 95% of peak to allow L-overrides

## Threshold exceeds actual water readings
- The observed_peak_adc caps threshold+hysteresis at the max ADC ever seen
- If peak=0 (first boot), do one immersion to learn it
- Check `$SWSST` for the observed_peak field (last value in response)

## Device not transmitting after surfacing
- Check `DRY_TIME_BEFORE_TX` — if set high, TX is delayed
- Verify Argos scheduler receives surface notification (enable DEBUG logging)
- Check `UNDERWATER_EN` is enabled

---

*Last updated: 2026-03-20*
