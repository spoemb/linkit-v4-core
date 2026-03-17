# GUI Parameter Reference & Configuration Guide

Reference for the LinkIt V4 GUI: parameter definitions, visibility rules, and **configuration scenarios** showing how parameters interact for real deployments.

---

# Part 1 — Configuration Scenarios

## Scenario A: Turtle Underwater — Surfacing Burst (recommended)

**Goal:** Maximize position data during short surface windows. Rapid Doppler for immediate satellite localization, then GNSS if time permits.

```
Surface detected → Doppler #1 (0s) → #2 (5s) → #3 (15s) → #4 (25s)...
                   ↓ GPS fix acquired
                   → GNSS TX #1 (60s) → #2 (120s)...
                   ↓ Dive detected
                   → All TX stops, burst state reset
```

### Key parameters and WHY

| Parameter | Value | Reason |
|-----------|-------|--------|
| **ARP01** ARGOS_MODE | 5 (SURFACING_BURST) | Combines Doppler + GNSS automatically |
| **ARP40** SURFACING_BURST_INIT_S | 5 | First Doppler gap — Kineis needs ≥5s between messages for Doppler computation |
| **ARP41** SURFACING_BURST_STEP_S | 10 | Progressive spacing — reduces channel saturation while keeping messages flowing |
| **ARP42** SURFACING_BURST_MAX_S | 60 | Cap — beyond 60s, a satellite pass may end |
| **ARP05** TR_NOM | 60 | GNSS phase interval — once GPS fix acquired, standard 60s spacing |
| **ARP30** TIME_SYNC_BURST_EN | 0 | Ignored in SURFACING_BURST — first message is always Doppler, not GNSS |
| **ARP35** ARGOS_TCXO_WARMUP_TIME | 5 | Normal warmup, but auto-skipped on first TX after surfacing (firmware handles this) |
| **UNP01** UNDERWATER_EN | 1 | Essential — drives the entire surface/dive cycle |
| **UNP02** DRY_TIME_BEFORE_TX | 0 | No delay — Doppler starts immediately on surface |
| **UNP03** SAMPLING_UNDER_FREQ | 2 | Fast underwater sampling (2s) — detect surface quickly |
| **GNP01** GNSS_EN | 1 | GPS runs in parallel with Doppler burst |
| **GNP25** GNSS_TRIGGER_ON_SURFACED | 1 | GPS starts immediately on surface — no time to waste |
| **GNP02** GNSS_HDOPFILT_EN | 0 | **Disabled** — accept any fix, speed is priority over precision |
| **GNP20** GNSS_HACCFILT_EN | 0 | **Disabled** — same reason, first fix wins |
| **GNP11** GNSS_DYN_MODEL | 5 (SEA) | u-blox model optimized for marine surface motion |

### What happens at each phase

**Doppler phase (no GPS fix yet):**
- Firmware sends 3-byte Doppler packets (battery voltage only)
- Kineis satellite computes position from frequency shift (~5-10km accuracy)
- Multiple messages = better Doppler solution
- Progressive interval: `0, 5, 15, 25, 35, 45, 55, 60, 60, 60...`

**GNSS phase (GPS fix acquired):**
- Firmware switches to GNSS packets (96-248 bits with lat/lon/speed/heading)
- Uses TR_NOM (60s) interval
- Depth pile packs multiple fixes per packet
- Continues until dive

**If turtle dives before GPS fix:**
- Only Doppler data available — satellite estimates position
- Burst state resets, ready for next surfacing

---

## Scenario B: Turtle Underwater — Legacy Mode (simple)

**Goal:** Simple fixed-interval TX. No progressive Doppler. Works but less optimal for short surfacings.

| Parameter | Value | Reason |
|-----------|-------|--------|
| **ARP01** ARGOS_MODE | 2 (LEGACY) | Fixed interval TX |
| **ARP05** TR_NOM | 60 | TX every 60s |
| **GNP01** GNSS_EN | 1 | GPS active |
| **ARP30** TIME_SYNC_BURST_EN | 1 | First TX sends GNSS packet immediately (if GPS has fix) |
| **UNP01** UNDERWATER_EN | 1 | SWS detection |
| **UNP02** DRY_TIME_BEFORE_TX | 0 | Immediate TX |

**Limitation:** First TX waits for GPS fix. If GPS is slow (cold start), first messages are missed. No Doppler fallback.

---

## Scenario C: Bird Tracker — RSPB (TPL5111 wakeup)

**Goal:** Ultra-low power. Device sleeps most of the time. Wakes periodically, gets one GPS fix, sends N messages, powers off.

| Parameter | Value | Reason |
|-----------|-------|--------|
| **ARP01** ARGOS_MODE | 2 (LEGACY) | Simple interval |
| **ARP05** TR_NOM | 120 | 2 min between TX |
| **PWP05** SHUTDOWN_NTIME_SAT | 5 | Power off after 5 TX messages |
| **PWP03** BOOT_COUNTER_MODULO | 5 | Active cycle every 5th wakeup (~8h45) |
| **GNP30** GNSS_SESSION_SINGLE_FIX | 1 | Stop GPS after first fix (save power) |
| **GNP11** GNSS_DYN_MODEL | 6 (AIRBORNE_1G) | Bird flight dynamics |
| **GNP21** GNSS_HACCFILT_THR | 10 | Relaxed accuracy (faster fix) |
| **ARP32** ARGOS_RX_EN | 0 | No RX (save power, AOP uploaded via BLE) |
| **UNP01** UNDERWATER_EN | 0 | No water detection |

**Session timeline:**
```
TPL5111 wakeup → boot counter check → GPS on → first fix (45s) → GPS off
→ TX#1 → wait 120s → TX#2 → ... → TX#5 → POWER OFF
Total awake: ~12 min, next wakeup in ~8h45
```

---

## Scenario D: Duty Cycle — Targeted TX Hours

**Goal:** Save power by only transmitting during hours when satellites are most likely overhead.

| Parameter | Value | Reason |
|-----------|-------|--------|
| **ARP01** ARGOS_MODE | 3 (DUTY_CYCLE) | Hourly window control |
| **ARP18** DUTY_CYCLE | 0x0F0F0F | TX during hours 0-3, 8-11, 16-19 UTC |
| **ARP05** TR_NOM | 90 | TX every 90s within active hours |

**Duty cycle bitmask:** 24 bits, bit 23 = hour 0 UTC, bit 0 = hour 23 UTC.
```
0x0F0F0F = 0000 1111 0000 1111 0000 1111
           h0-3      h8-11     h16-19
```

---

## Scenario E: Low Battery Behavior

**Goal:** When battery is low, reduce power consumption. Two strategies:

### E1: Doppler-only in LB (simplest)

| Parameter | Value | Reason |
|-----------|-------|--------|
| **LBP01** LB_EN | 1 | Enable LB mode |
| **LBP02** LB_THRESHOLD | 15 | Enter LB below 15% SOC |
| **LBP04** LB_ARGOS_MODE | 4 (DOPPLER) | Doppler-only (no GPS = huge power saving) |
| **LBP06** LB_GNSS_EN | 0 | GPS off |
| **ARP06** TR_LB | 120 | TX every 2 min |
| **LBP12** LB_CRITICAL_THRESH | 5 | Below 5% → immediate power off |

### E2: Surfacing Burst in LB (turtle, maximum data)

| Parameter | Value | Reason |
|-----------|-------|--------|
| **LBP04** LB_ARGOS_MODE | 5 (SURFACING_BURST) | Keep Doppler burst behavior |
| **LBP06** LB_GNSS_EN | 0 | No GPS in LB — Doppler only |
| **ARP06** TR_LB | 120 | Fallback interval |

When LB + SURFACING_BURST + GNSS_EN=0: the burst sends Doppler messages progressively, but never switches to GNSS phase (no GPS). Pure Doppler positioning.

---

## Scenario F: GNSS Configuration Strategies

### F1: Fastest fix (precision secondary)

For animals with short surface windows. Accept any fix immediately.

| Parameter | Value | Effect |
|-----------|-------|--------|
| **GNP02** GNSS_HDOPFILT_EN | 0 | No geometry filter |
| **GNP20** GNSS_HACCFILT_EN | 0 | No accuracy filter |
| **GNP05** GNSS_ACQ_TIMEOUT | 90 | Short timeout (don't waste battery) |
| **GNP09** GNSS_COLD_ACQ_TIMEOUT | 180 | Reasonable cold start |
| **GNP22** GNSS_MIN_NUM_FIXES | 1 | Accept first fix |
| **GNP24** GNSS_ASSISTNOW_EN | 1 | AssistNow for faster TTFF |
| **GNP25** GNSS_TRIGGER_ON_SURFACED | 1 | Start GPS immediately |

**Expected TTFF:** ~10-30s warm, ~60-120s cold.

### F2: High precision (stationary or long surface)

For animals that stay at surface for long periods. Get the best fix possible.

| Parameter | Value | Effect |
|-----------|-------|--------|
| **GNP02** GNSS_HDOPFILT_EN | 1 | Require good geometry |
| **GNP03** GNSS_HDOPFILT_THR | 2 | Strict HDOP |
| **GNP20** GNSS_HACCFILT_EN | 1 | Require accuracy |
| **GNP21** GNSS_HACCFILT_THR | 5 | 5m accuracy |
| **GNP05** GNSS_ACQ_TIMEOUT | 240 | Long timeout (more chance of good fix) |
| **GNP22** GNSS_MIN_NUM_FIXES | 3 | 3 consecutive valid fixes |

### F3: Dynamic model selection

| Value | Model | Use case |
|-------|-------|----------|
| 0 | PORTABLE | General/default |
| 3 | PEDESTRIAN | Slow-moving land animals |
| 5 | **SEA** | Marine animals (turtles, seals) |
| 6 | **AIRBORNE_1G** | Birds (accepts high speed, low accel) |
| 7 | AIRBORNE_2G | Fast-diving birds |

**Wrong model = missed fixes.** PORTABLE mode rejects bird-speed motion. SEA mode is specifically tuned for surface-level marine navigation.

---

## Scenario G: Underwater Detection Strategies

### G1: SWS (saltwater switch) — recommended for marine

| Parameter | Value | Effect |
|-----------|-------|--------|
| **UNP10** UNDERWATER_DETECT_SOURCE | 0 (SWS) | Analog conductivity electrode |
| **UNP03** SAMPLING_UNDER_FREQ | 2 | Fast sampling UW (detect surface in ~2s) |
| **UNP04** SAMPLING_SURF_FREQ | 10 | Slower at surface (save power) |
| **UNP24** UW_MAX_DIVE_TIME | 7200 | Force surface after 2h (safety) |
| **UNP25** UW_MIN_SURFACE_TIME | 5 | 5s anti-bounce lockout |
| **UNP08** UW_PIN_SAMPLE_DELAY | 1 | **Must stay at 1ms** (RC circuit constant) |

**Auto-calibration:** Air/water baselines adapt automatically. No manual threshold configuration needed. SWS_ANALOG_* params are for edge cases only.

### G2: Pressure sensor — for depth-logging deployments

| Parameter | Value | Effect |
|-----------|-------|--------|
| **UNP10** UNDERWATER_DETECT_SOURCE | 1 (PRESSURE) | Depth > threshold = underwater |
| **UNP11** UNDERWATER_DETECT_THRESH | 1.1 | 1.1 meters depth threshold |

### G3: Hybrid SWS+GNSS — maximum reliability

| Parameter | Value | Effect |
|-----------|-------|--------|
| **UNP10** UNDERWATER_DETECT_SOURCE | 3 (SWS_GNSS) | SWS for transitions, GNSS confirms surface |

---

## Scenario H: Sensor Data in TX Packets

Sensors can inject data into GNSS satellite packets. Each sensor adds bits to the packet:

| Sensor | TX bits | Content |
|--------|---------|---------|
| AXL (BMA400) | 67 bits | Temperature + X/Y/Z accel + activity |
| Pressure | 29 bits | Depth value |
| Sea Temp | 21 bits | Temperature |
| ALS (Light) | 17 bits | Ambient light |
| pH | 14 bits | pH value |
| Thermistor | 14 bits | Temperature |

**Max TX payload:** 192 bits (24 bytes LDA2). GPS alone = 75 bits. So ~117 bits available for sensors.

### Configuration pattern (per sensor):

```
*_ENABLE = 1              → Power on the sensor hardware
*_PERIODIC = 0            → No periodic local readings (0 = disabled)
*_ENABLE_TX_MODE = ONESHOT → Read once at TX time, include in satellite packet
```

**TX modes:**
- **OFF** — Sensor data NOT included in satellite packet
- **ONESHOT** — Single reading at TX time
- **MEAN** — Average of N samples over period
- **MEDIAN** — Median of N samples over period

**Important:** `PERIODIC` and `TX_MODE` are independent. PERIODIC logs locally. TX_MODE includes in satellite packet. You can have TX_MODE=ONESHOT with PERIODIC=0 (no local logging, just TX).

---

# Part 2 — Complete Parameter Reference

## ARGOS_MODE Values & Visibility Matrix

| Value | Name | Description |
|-------|------|-------------|
| 0 | OFF | No satellite TX |
| 1 | PASS_PREDICTION | TX when satellite overhead (needs AOP) |
| 2 | LEGACY | Fixed interval TX, all hours |
| 3 | DUTY_CYCLE | Fixed interval TX within hourly windows |
| 4 | DOPPLER | Doppler-only (3-byte, no GNSS) |
| 5 | SURFACING_BURST | Progressive Doppler then GNSS (marine) |

**GUI Visibility Rules:**

| Parameter | OFF | LEGACY | DUTY_CYCLE | DOPPLER | SURFACING_BURST | PASS_PRED |
|-----------|-----|--------|------------|---------|-----------------|-----------|
| TR_NOM (ARP05) | H | A | A | A | A | A |
| DUTY_CYCLE (ARP18) | H | H | **A** | H | H | H |
| NTRY_PER_MESSAGE (ARP19) | H | A | A | A | G | A |
| ARGOS_DEPTH_PILE (ARP16) | H | A | A | H | A | A |
| DLOC_ARG_NOM (ARP11) | H | A | A | H | A | A |
| TIME_SYNC_BURST_EN (ARP30) | H | A | A | H | G (ignored) | A |
| TX_JITTER_EN (ARP31) | H | A | A | A | A | A |
| SURFACING_BURST_INIT_S (ARP40) | H | H | H | H | **A** | H |
| SURFACING_BURST_STEP_S (ARP41) | H | H | H | H | **A** | H |
| SURFACING_BURST_MAX_S (ARP42) | H | H | H | H | **A** | H |
| GNSS_EN (GNP01) | H | A | A | G | A | A |
| UNDERWATER_EN (UNP01) | A | A | A | A | **A** | A |

**Cascading visibility:**
- All GNSS params → hidden if GNSS_EN=false
- All UW params → hidden if UNDERWATER_EN=false
- All LB params → hidden if LB_EN=false
- SWS params → hidden if UNDERWATER_DETECT_SOURCE ≠ SWS or SWS_GNSS
- GNSS UW params → hidden if UNDERWATER_DETECT_SOURCE ≠ GNSS or SWS_GNSS
- Sensor TX params → hidden if sensor *_ENABLE=false
- Sensor TX options → hidden if *_ENABLE_TX_MODE=OFF

---

## Device Identification

| DTE Key | Name | Type | Default | RW | Description |
|---------|------|------|---------|-----|-------------|
| IDP12 | ARGOS_DECID | UINT | 0 | RW | Argos decimal platform ID |
| IDT06 | ARGOS_HEXID | HEX | 0 | RW | Argos hex platform ID |
| IDP11 | PROFILE_NAME | TEXT | FACTORY | RW | Deployment profile name |
| IDP13 | ARGOS_SECKEY | TEXT | "" | RW | A+ security key (SMD) |
| IDP14 | ARGOS_RADIOCONF | TEXT | "" | RW | Radio calibration (SMD) |
| IDT02 | DEVICE_MODEL | TEXT | - | RO | Board name |
| IDT03 | FW_APP_VERSION | TEXT | - | RO | Firmware version |
| IDT04 | HW_VERSION | TEXT | - | RO | Hardware version |
| IDT10 | DEVICE_DECID | UINT | - | RO | Unique device ID |

## Argos Satellite TX

| DTE Key | Name | Type | Range | Default | Description |
|---------|------|------|-------|---------|-------------|
| ARP01 | ARGOS_MODE | ENUM | 0-5 | 2 | TX scheduling mode |
| ARP05 | TR_NOM | UINT | 30-1200 s | 60 | TX interval (seconds) |
| ARP18 | DUTY_CYCLE | UINT | 0-0xFFFFFF | 0 | 24-bit hourly TX mask |
| ARP19 | NTRY_PER_MESSAGE | UINT | 0-86400 | 0 | TX repetitions per message |
| ARP16 | ARGOS_DEPTH_PILE | ENUM | 1-24 | 16 | GPS fixes per TX packet |
| ARP11 | DLOC_ARG_NOM | ENUM | 10min-24h | 600s | GNSS acquisition period |
| ARP35 | ARGOS_TCXO_WARMUP_TIME | UINT | 0-30 s | 5 | TCXO warmup (auto-skipped after surfacing) |
| ARP30 | ARGOS_TIME_SYNC_BURST_EN | BOOL | - | true | First TX = GNSS burst (ignored in mode 5) |
| ARP31 | ARGOS_TX_JITTER_EN | BOOL | - | true | ±5s random jitter |
| ARP40 | SURFACING_BURST_INIT_S | UINT | 5-120 | 5 | Doppler burst: initial interval |
| ARP41 | SURFACING_BURST_STEP_S | UINT | 0-30 | 1 | Doppler burst: step increment |
| ARP42 | SURFACING_BURST_MAX_S | UINT | 5-300 | 30 | Doppler burst: max interval cap |
| PWP05 | SHUTDOWN_NTIME_SAT | UINT | 0-65535 | 0 | Power off after N TX (0=unlimited) |

## Argos Satellite RX

| DTE Key | Name | Type | Range | Default | Description |
|---------|------|------|-------|---------|-------------|
| ARP32 | ARGOS_RX_EN | BOOL | - | true | Enable RX for AOP updates |
| ARP33 | ARGOS_RX_MAX_WINDOW | UINT | 1-max s | 900 | Max RX duration |
| ARP34 | ARGOS_RX_AOP_UPDATE_PERIOD | UINT | 0-max s | 90 | Min interval between AOP updates |

**Telemetry (read-only):** ART01 LAST_TX, ART02 TX_COUNTER, ART03 ARGOS_AOP_DATE, ART10 RX_COUNTER, ART11 RX_TIME

## GNSS (GPS)

| DTE Key | Name | Type | Range | Default | Description |
|---------|------|------|-------|---------|-------------|
| GNP01 | GNSS_EN | BOOL | - | true | Enable GNSS |
| GNP05 | GNSS_ACQ_TIMEOUT | UINT | 10-600 s | 120 | Warm start timeout |
| GNP09 | GNSS_COLD_ACQ_TIMEOUT | UINT | 10-600 s | 530 | Cold start timeout |
| GNP23 | GNSS_COLD_START_RETRY_PERIOD | UINT | 1-max s | 60 | Cold start retry interval |
| GNP10 | GNSS_FIX_MODE | ENUM | 2D/3D/AUTO | AUTO | Fix mode |
| GNP11 | GNSS_DYN_MODEL | ENUM | 0-10 | 0 | Dynamic model (0=Portable, 5=Sea, 6=Airborne) |
| GNP02 | GNSS_HDOPFILT_EN | BOOL | - | true | HDOP filter enable |
| GNP03 | GNSS_HDOPFILT_THR | UINT | 2-15 | 2 | HDOP threshold |
| GNP20 | GNSS_HACCFILT_EN | BOOL | - | true | Accuracy filter enable |
| GNP21 | GNSS_HACCFILT_THR | UINT | 0-max m | 5 | Accuracy threshold (meters) |
| GNP22 | GNSS_MIN_NUM_FIXES | UINT | 1-max | 1 | Consecutive fixes required |
| GNP25 | GNSS_TRIGGER_ON_SURFACED | BOOL | - | true | Start GPS on surface detection |
| GNP26 | GNSS_TRIGGER_ON_AXL_WAKEUP | BOOL | - | false | Start GPS on motion |
| GNP28 | GNSS_TRIGGER_COLD_START_ON_SURFACED | BOOL | - | false | Force cold start on surfacing |
| GNP30 | GNSS_SESSION_SINGLE_FIX | BOOL | - | false | Stop after first fix |
| GNP24 | GNSS_ASSISTNOW_EN | BOOL | - | true | AssistNow Online |
| GNP27 | GNSS_ASSISTNOW_OFFLINE_EN | BOOL | - | false | AssistNow Offline |
| GNP31 | GNSS_TOKEN | TEXT | - | "" | u-blox auth token |

## Underwater Detection

| DTE Key | Name | Type | Range | Default | Description |
|---------|------|------|-------|---------|-------------|
| UNP01 | UNDERWATER_EN | BOOL | - | false | Enable UW detection |
| UNP10 | UNDERWATER_DETECT_SOURCE | ENUM | 0-3 | 0 (SWS) | 0=SWS, 1=Pressure, 2=GNSS, 3=SWS+GNSS |
| UNP02 | DRY_TIME_BEFORE_TX | UINT | 0-max s | 0 | Surface delay before TX |
| UNP03 | SAMPLING_UNDER_FREQ | UINT | 1-max s | 2 | Sample interval underwater |
| UNP04 | SAMPLING_SURF_FREQ | UINT | 1-max s | 10 | Sample interval at surface |
| UNP05 | UW_MAX_SAMPLES | UINT | 1-max | 1 | Sub-samples per cycle |
| UNP06 | UW_MIN_DRY_SAMPLES | UINT | 1-max | 1 | Dry samples to confirm surface |
| UNP07 | UW_SAMPLE_GAP | UINT | 1-max ms | 1000 | Gap between sub-samples |
| UNP08 | UW_PIN_SAMPLE_DELAY | UINT | 1-max ms | 1 | RC charge time (**keep at 1**) |
| UNP24 | UW_MAX_DIVE_TIME | UINT | 0-max s | 7200 | Force surface timeout |
| UNP25 | UW_MIN_SURFACE_TIME | UINT | 0-max s | 5 | Surface lockout |
| UNP11 | UNDERWATER_DETECT_THRESH | FLOAT | - | 1.1 | Pressure/GNSS threshold |
| UNP20 | SWS_ANALOG_THRESHOLD_MIN | UINT | 0-16383 | 0 | Min ADC |
| UNP21 | SWS_ANALOG_THRESHOLD_MAX | UINT | 50-16383 | 8000 | Max ADC |
| UNP22 | SWS_ANALOG_HYSTERESIS | UINT | 0-50 % | 6 | Hysteresis |
| UNP23 | SWS_ANALOG_CALIB_INTERVAL | UINT | 60-max s | 3600 | Air recalibration interval |
| UNP12 | UW_DIVE_MODE_ENABLE | BOOL | - | false | Dive mode FSM |
| UNP13 | UW_DIVE_MODE_START_TIME | UINT | - | 0 | Dive mode timestamp |
| UNP14 | UW_GNSS_DRY_SAMPLING | UINT | 1-max s | 14400 | GNSS surface sample (GNSS detect) |
| UNP15 | UW_GNSS_WET_SAMPLING | UINT | 1-max s | 14400 | GNSS UW sample (GNSS detect) |
| UNP16 | UW_GNSS_MAX_SAMPLES | UINT | 1-max | 10 | GNSS detect max samples |
| UNP17 | UW_GNSS_MIN_DRY_SAMPLES | UINT | 1-max | 1 | GNSS detect dry confirm |
| UNP18 | UW_GNSS_DETECT_THRESH | UINT | 1-7 | 1 | Satellite count threshold |

## Low Battery Mode

| DTE Key | Name | Type | Range | Default | Description |
|---------|------|------|-------|---------|-------------|
| LBP01 | LB_EN | BOOL | - | false | Enable LB mode |
| LBP02 | LB_THRESHOLD | UINT | 0-100 % | 10 | SOC to enter LB |
| LBP12 | LB_CRITICAL_THRESH | UINT | 0-100 % | 5 | SOC for power off |
| LBP04 | LB_ARGOS_MODE | ENUM | 0-5 | 2 | TX mode in LB |
| ARP06 | TR_LB | UINT | 30-1200 s | 240 | TX interval in LB |
| LBP11 | LB_NTRY_PER_MESSAGE | UINT | 0-86400 | 4 | TX reps in LB |
| LBP08 | LB_ARGOS_DEPTH_PILE | ENUM | 1-24 | 1 | Depth pile in LB |
| LBP05 | LB_ARGOS_DUTY_CYCLE | UINT | 0-0xFFFFFF | 0 | Duty cycle in LB |
| LBP06 | LB_GNSS_EN | BOOL | - | true | GNSS in LB |
| LBP09 | LB_GNSS_ACQ_TIMEOUT | UINT | 10-600 s | 120 | GNSS timeout in LB |
| LBP07 | LB_GNSS_HDOPFILT_THR | UINT | 2-15 | 2 | HDOP in LB |
| LBP10 | LB_GNSS_HACCFILT_THR | UINT | 0-max m | 5 | Accuracy in LB |
| ARP12 | DLOC_ARG_LB | ENUM | 10min-24h | 3600s | GNSS period in LB |
| LBP14 | LB_SHUTDOWN_NTIME_SAT | UINT | 0-65535 | 0 | TX count before shutdown in LB |

## Geofencing Zone

| DTE Key | Name | Type | Default | Description |
|---------|------|------|---------|-------------|
| ZOP04 | ZONE_ENABLE | BOOL | false | Enable geofencing |
| ZOP01 | ZONE_TYPE | ENUM | CIRCLE | Zone shape |
| ZOP19 | ZONE_CENTER_LATITUDE | FLOAT | -48.8752 | Center lat |
| ZOP18 | ZONE_CENTER_LONGITUDE | FLOAT | -123.3925 | Center lon |
| ZOP20 | ZONE_RADIUS | UINT m | 1000 | Radius |
| ZOP05 | ZONE_ENABLE_ACTIVATION_DATE | BOOL | true | Use activation date |
| ZOP06 | ZONE_ACTIVATION_DATE | DATE | 01/01/2020 | Start date |
| ZOP11 | ZONE_ARGOS_MODE | ENUM 0-5 | LEGACY | Mode out of zone |
| ZOP10 | ZONE_ARGOS_REPETITION_SECONDS | UINT | 240 | TX interval out of zone |
| ZOP08 | ZONE_ARGOS_DEPTH_PILE | ENUM | 1 | Depth pile out of zone |
| ZOP12 | ZONE_ARGOS_DUTY_CYCLE | UINT | 0xFFFFFF | Duty cycle out of zone |
| ZOP13 | ZONE_ARGOS_NTRY_PER_MESSAGE | UINT | 0 | TX reps out of zone |
| ZOP14 | ZONE_GNSS_DELTA_ARG | ENUM | 3600 | GNSS period out of zone |
| ZOP15 | ZONE_GNSS_HDOPFILT_THR | UINT | 2 | HDOP out of zone |
| ZOP16 | ZONE_GNSS_HACCFILT_THR | UINT | 5 | Accuracy out of zone |
| ZOP17 | ZONE_GNSS_ACQ_TIMEOUT | UINT s | 240 | GNSS timeout out of zone |

## Sensors

Pattern for each sensor: `*_ENABLE`, `*_PERIODIC` (local log interval, 0=off), `*_ENABLE_TX_MODE` (OFF/ONESHOT/MEAN/MEDIAN), `*_ENABLE_TX_MAX_SAMPLES`, `*_ENABLE_TX_SAMPLE_PERIOD`

### BMA400 Accelerometer (AXL)
AXP01 enable, AXP02 periodic, AXP03 wakeup_thresh (0-8g), AXP04 wakeup_samples (0-50), AXP08 range (0-4), AXP09 power_mode (0-2), AXP05 tx_mode, AXP06 tx_max_samples, AXP07 tx_sample_period

### Pressure Sensor
PRP01 enable, PRP02 periodic, PRP03 logging_mode (ALWAYS/UW_THRESHOLD), PRP07 full_scale (1260/4060), PRP04 tx_mode, PRP05 tx_max_samples, PRP06 tx_sample_period

### Sea Temperature (RTD/TSYS01)
STP01 enable, STP02 periodic, STP04 tx_mode, STP05 tx_max_samples, STP06 tx_sample_period

### Thermistor
THP01 enable, THP02 periodic, THP04 wakeup_thresh, THP05 wakeup_samples, THP06 tx_mode, THP07 tx_max_samples, THP08 tx_sample_period

### ALS (Light)
LTP01 enable, LTP02 periodic, LTP04 tx_mode, LTP05 tx_max_samples, LTP06 tx_sample_period

### pH Sensor
PHP01 enable, PHP02 periodic, PHP04 tx_mode, PHP05 tx_max_samples, PHP06 tx_sample_period

### CDT (Conductivity/Depth/Temp)
CDP01 enable, CDP02 periodic

## Camera (ENABLE_CAM_SENSOR)
CAP01 enable, CAP02 trigger_on_surfaced, CAP03 trigger_on_axl, CAP04 period_on, CAP05 period_off, LBP13 lb_cam_en

## LED & Debug
LDP01 LED_MODE (OFF=0, HRS_24=1, ALWAYS=3), LDP02 EXT_LED_MODE, DBP01 DEBUG_OUTPUT_MODE (UART=0, USB_CDC=1, BLE_NUS=2)

## Power Management (EXTERNAL_WAKEUP only)
PWP01 SHUTDOWN_TIMER, PWP02 BOOT_COUNTER (RO), PWP03 BOOT_COUNTER_MODULO, PWP04 WAKEUP_PERIOD, PWP06 LAST_KNOWN_RTC (RO)

## LoRa RAK3172 (LORA_RAK3172 only)
LRP01 DEVEUI (RO), LRP02 APPEUI, LRP03 APPKEY, LRP04 DEVADDR, LRP05 APPSKEY, LRP06 NWKSKEY, LRP07 NJM (0=ABP/1=OTAA), LRP08 BAND (0-12), LRP09 CLASS (0-2), LRP10 DR (0-15), LRP11 ADR, LRP12 TXP (0-14), LRP13 CFM, LRP14 FPORT (1-223), LRP15 LP_MODE (0=shutdown/1=standby)

## Certification Mode
CTP01 CERT_TX_ENABLE, CTP02 CERT_TX_PAYLOAD, CTP03 CERT_TX_MODULATION (LDK/LDA2/VLDA4), CTP04 CERT_TX_REPETITION

## Battery Telemetry (read-only)
POT03 BATT_SOC (%), POT05 LAST_FULL_CHARGE_DATE, POT06 BATT_VOLTAGE (V), SYT01 RTC_CURRENT_TIME
