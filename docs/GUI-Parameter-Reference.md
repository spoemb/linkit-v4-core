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
| AXL (BMA400) | 67 bits (8 bits RSPB) | Full: Temp + X/Y/Z + activity. **RSPB compact: activity only** |
| Pressure | 29 bits | Pressure + temperature |
| Sea Temp | 21 bits | Temperature |
| ALS (Light) | 17 bits | Ambient light |
| pH | 14 bits | pH value |
| Thermistor | 14 bits | Body temperature |
| Mortality (RSPB) | 7 bits | Confidence percentage (0-100%) |

**Max TX payload:** 192 bits (24 bytes LDA2). GPS alone = 75 bits. RSPB total with all sensors: 133 bits (59 bits free).

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

## Scenario I: RSPB Mortality Detection

**Goal:** Detect when a tracked bird has died by combining accelerometer activity, body temperature, and GPS stationarity. Automatically reduce wakeup frequency when mortality is confirmed.

**Prerequisites:** `ENABLE_MORTALITY_SENSOR=1` (auto for RSPB), plus AXL, Thermistor, and GNSS must all be enabled and in TX mode.

| Parameter | Value | Reason |
|-----------|-------|--------|
| **MTP01** MORTALITY_ENABLE | 1 | Enable mortality detection algorithm |
| **MTP02** MORTALITY_ACTIVITY_THRESH | 10 | Activity < 10/255 = immobile (BMA400 activity score) |
| **MTP03** MORTALITY_TEMP_THRESH | 25.0 | Body temp < 25°C = hypothermic (live bird ~40°C) |
| **MTP04** MORTALITY_GPS_DISTANCE_THRESH | 50 | < 50m between sessions = stationary |
| **MTP05** MORTALITY_CONFIRM_DAYS | 3 | 3 consecutive days of high confidence → CONFIRMED |
| **MTP06** MORTALITY_DUTY_CYCLE_MODULO | 6 | Reduce wakeups when confirmed (0=never modify) |
| **AXP01** AXL_SENSOR_ENABLE | 1 | **Required** for activity measurement |
| **AXP05** AXL_SENSOR_ENABLE_TX_MODE | MEAN | Average activity over TX period |
| **THP01** THERMISTOR_SENSOR_ENABLE | 1 | **Required** for body temperature |
| **THP06** THERMISTOR_SENSOR_ENABLE_TX_MODE | ONESHOT | Single reading per TX |
| **GNP01** GNSS_EN | 1 | **Required** for position/stationarity |

**RSPB satellite packet (compact, 133/192 bits):**
```
[Time 16b][GPS 51b][Battery 8b][Pressure 29b][Thermistor 14b][Activity 8b][Mortality% 7b]
```

**WARNING:** Mortality detection requires AXL, Thermistor, and GNSS to be enabled. If any of these sensors is disabled, the algorithm will work with partial data only (biased toward ALIVE).

---

# Part 2 — Complete Parameter Reference

## GUI Mode: Standard vs Advanced

The GUI has two display modes controlled by an "Advanced Mode" toggle. **Standard mode** shows only the parameters a wildlife researcher needs for deployment. **Advanced mode** reveals low-level tuning parameters that can break the device if misconfigured.

**Rule of thumb:** If a user can brick the device, corrupt data, or waste battery by changing it without understanding the firmware internals → Advanced.

### Build Availability Legend

Some parameters are only available on specific firmware builds. The GUI should detect the device model (via `IDT02 DEVICE_MODEL`) and hide parameters that don't exist on the connected device. The firmware returns error code 6 (`PARAM_KEY_UNRECOGNISED`) for parameters not compiled in.

| Tag | Meaning | How to detect |
|-----|---------|---------------|
| **All** | Available on all builds | Always present |
| **RSPB only** | Only on RSPB builds (`BOARD=RSPB`) | `DEVICE_MODEL == "RSPB"` |
| **LinkIt only** | Only on LinkIt V4 builds (not RSPB) | `DEVICE_MODEL != "RSPB"` |
| **SMD only** | Only when `ARGOS_SMD=ON` (RSPB + LinkIt SMD) | Try reading `IDP13`; if error 6 → not SMD |
| **LoRa only** | Only when `LORA_RAK3172=ON` (LinkIt LoRa) | Try reading `LRP01`; if error 6 → not LoRa |

**Build → Parameter availability matrix:**

| Parameter Group | LinkIt KIM | LinkIt SMD | LinkIt LoRa | RSPB |
|----------------|------------|------------|-------------|------|
| Core (Argos TX, GNSS, UW, LB, Zone, Sensors) | Yes | Yes | Yes | Yes |
| Power Management (PWP01-PWP06) | No | No | No | **Yes** |
| Thermistor (THP01-THP08) | Optional | Optional | Optional | **Always** |
| Sea Temperature (STP01-STP06) | Optional | Optional | Optional | **Never** |
| Mortality (MTP01-MTP07) | No | No | No | **Yes** |
| SMD Credentials (IDP13, IDP14) | No | **Yes** | No | **Yes** |
| LoRa (LRP01-LRP15) | No | No | **Yes** | No |
| Camera (CAP01-CAP05, LBP13) | Optional | Optional | Optional | Optional |

**Note:** "Optional" means the parameter exists only if the sensor was enabled at compile time. The GUI should gracefully handle missing optional parameters (error 6 on read = hide the group).

### Device Identification

| DTE Key | Name | Mode | Build | Reason |
|---------|------|------|-------|--------|
| IDP12 | ARGOS_DECID | Standard | All | Must configure for each device |
| IDT06 | ARGOS_HEXID | Standard | All | Must configure for each device |
| IDP11 | PROFILE_NAME | Standard | All | Deployment label |
| IDP13 | ARGOS_SECKEY | **Advanced** | **SMD only** | Raw crypto key, set via SMDCD or pylinkit |
| IDP14 | ARGOS_RADIOCONF | **Advanced** | All | Raw radio calibration blob |
| IDT02 | DEVICE_MODEL | Standard (RO) | All | Info display |
| IDT03 | FW_APP_VERSION | Standard (RO) | All | Info display |
| IDT04 | HW_VERSION | Standard (RO) | All | Info display |
| IDT10 | DEVICE_DECID | Standard (RO) | All | Info display |

### Argos Satellite TX

| DTE Key | Name | Mode | Reason |
|---------|------|------|--------|
| ARP01 | ARGOS_MODE | Standard | Core: how the tracker transmits |
| ARP05 | TR_NOM | Standard | Core: TX interval |
| ARP18 | DUTY_CYCLE | Standard | Core: when DUTY_CYCLE mode selected |
| ARP19 | NTRY_PER_MESSAGE | **Advanced** | Protocol detail, wrong value = channel saturation |
| ARP16 | ARGOS_DEPTH_PILE | **Advanced** | Packet packing strategy, needs protocol knowledge |
| ARP11 | DLOC_ARG_NOM | Standard | GPS acquisition period (user-friendly) |
| ARP35 | ARGOS_TCXO_WARMUP_TIME | **Advanced** | Hardware timing, firmware auto-manages |
| ARP30 | TIME_SYNC_BURST_EN | **Advanced** | Protocol optimization detail |
| ARP31 | ARGOS_TX_JITTER_EN | **Advanced** | Anti-collision, should stay ON |
| ARP40 | SURFACING_BURST_INIT_S | **Advanced** | Burst tuning, defaults are optimized |
| ARP41 | SURFACING_BURST_STEP_S | **Advanced** | Burst tuning, defaults are optimized |
| ARP42 | SURFACING_BURST_MAX_S | **Advanced** | Burst tuning, defaults are optimized |
| PWP05 | SHUTDOWN_NTIME_SAT | Standard | Core: how many TX per session (RSPB) |

### Argos Satellite RX

| DTE Key | Name | Mode | Reason |
|---------|------|------|--------|
| ARP32 | ARGOS_RX_EN | Standard | Enable/disable RX (power impact) |
| ARP33 | ARGOS_RX_MAX_WINDOW | **Advanced** | Protocol detail |
| ARP34 | ARGOS_RX_AOP_UPDATE_PERIOD | **Advanced** | Protocol detail |

### GNSS (GPS)

| DTE Key | Name | Mode | Reason |
|---------|------|------|--------|
| GNP01 | GNSS_EN | Standard | Core: enable GPS |
| GNP05 | GNSS_ACQ_TIMEOUT | Standard | How long to search for GPS fix |
| GNP09 | GNSS_COLD_ACQ_TIMEOUT | Standard | First-boot or backup-lost timeout |
| GNP23 | GNSS_COLD_START_RETRY_PERIOD | **Advanced** | Retry timing detail |
| GNP10 | GNSS_FIX_MODE | **Advanced** | Most users should leave AUTO |
| GNP11 | GNSS_DYN_MODEL | Standard | Important: must match animal type |
| GNP02 | GNSS_HDOPFILT_EN | **Advanced** | Quality filter, needs GPS knowledge |
| GNP03 | GNSS_HDOPFILT_THR | **Advanced** | Quality filter threshold |
| GNP20 | GNSS_HACCFILT_EN | Standard | Enable accuracy filter (simple on/off) |
| GNP21 | GNSS_HACCFILT_THR | Standard | Accuracy in meters (intuitive) |
| GNP22 | GNSS_MIN_NUM_FIXES | **Advanced** | Multi-fix averaging, niche use |
| GNP25 | GNSS_TRIGGER_ON_SURFACED | **Advanced** | Auto-managed by UW mode |
| GNP26 | GNSS_TRIGGER_ON_AXL_WAKEUP | **Advanced** | Niche: motion-triggered GPS |
| GNP28 | GNSS_TRIGGER_COLD_START_ON_SURFACED | **Advanced** | Debug/recovery use |
| GNP30 | GNSS_SESSION_SINGLE_FIX | Standard | Important for RSPB power saving |
| GNP24 | GNSS_ASSISTNOW_EN | Standard | Faster GPS fix (user understands) |
| GNP27 | GNSS_ASSISTNOW_OFFLINE_EN | **Advanced** | Needs offline data preload |
| GNP31 | GNSS_TOKEN | **Advanced** | u-blox auth token, set once |

### Underwater Detection

| DTE Key | Name | Mode | Reason |
|---------|------|------|--------|
| UNP01 | UNDERWATER_EN | Standard | Core: enable/disable UW detection |
| UNP10 | UNDERWATER_DETECT_SOURCE | Standard | Which sensor detects water |
| UNP02 | DRY_TIME_BEFORE_TX | Standard | Surface delay (intuitive) |
| UNP03 | SAMPLING_UNDER_FREQ | **Advanced** | Sampling rate tuning |
| UNP04 | SAMPLING_SURF_FREQ | **Advanced** | Sampling rate tuning |
| UNP05 | UW_MAX_SAMPLES | **Advanced** | Sub-sampling detail |
| UNP06 | UW_MIN_DRY_SAMPLES | **Advanced** | Detection algorithm tuning |
| UNP07 | UW_SAMPLE_GAP | **Advanced** | Hardware timing |
| UNP08 | UW_PIN_SAMPLE_DELAY | **Advanced** | RC circuit constant, **must stay at 1** |
| UNP24 | UW_MAX_DIVE_TIME | Standard | Safety: max dive before forced surface |
| UNP25 | UW_MIN_SURFACE_TIME | **Advanced** | Anti-bounce lockout |
| UNP11 | UNDERWATER_DETECT_THRESH | **Advanced** | Pressure/GNSS threshold tuning |
| UNP20 | SWS_ANALOG_THRESHOLD_MIN | **Advanced** | SWS auto-calibrates, only for edge cases |
| UNP21 | SWS_ANALOG_THRESHOLD_MAX | **Advanced** | SWS auto-calibrates |
| UNP22 | SWS_ANALOG_HYSTERESIS | **Advanced** | SWS tuning |
| UNP23 | SWS_ANALOG_CALIB_INTERVAL | **Advanced** | SWS tuning |
| UNP12 | UW_DIVE_MODE_ENABLE | **Advanced** | Reed switch pause during dive |
| UNP13 | UW_DIVE_MODE_START_TIME | **Advanced** | Dive mode detail |
| UNP14-18 | UW_GNSS_* | **Advanced** | GNSS-based UW detection tuning (all 5 params) |

### Low Battery Mode

| DTE Key | Name | Mode | Reason |
|---------|------|------|--------|
| LBP01 | LB_EN | Standard | Core: enable low battery mode |
| LBP02 | LB_THRESHOLD | Standard | Battery % to enter LB |
| LBP12 | LB_CRITICAL_THRESH | Standard | Battery % for shutdown |
| LBP04 | LB_ARGOS_MODE | **Advanced** | LB TX strategy, needs protocol knowledge |
| ARP06 | TR_LB | Standard | LB TX interval (intuitive) |
| LBP11 | LB_NTRY_PER_MESSAGE | **Advanced** | Protocol detail |
| LBP08 | LB_ARGOS_DEPTH_PILE | **Advanced** | Protocol detail |
| LBP05 | LB_ARGOS_DUTY_CYCLE | **Advanced** | Protocol detail |
| LBP06 | LB_GNSS_EN | Standard | GPS on/off in LB (power impact) |
| LBP09 | LB_GNSS_ACQ_TIMEOUT | **Advanced** | LB GPS tuning |
| LBP07 | LB_GNSS_HDOPFILT_THR | **Advanced** | LB GPS quality filter |
| LBP10 | LB_GNSS_HACCFILT_THR | **Advanced** | LB GPS accuracy filter |
| ARP12 | DLOC_ARG_LB | **Advanced** | LB GPS period |
| LBP14 | LB_SHUTDOWN_NTIME_SAT | Standard | LB TX count before shutdown (RSPB) |

### Geofencing Zone

| DTE Key | Name | Mode | Reason |
|---------|------|------|--------|
| ZOP04 | ZONE_ENABLE | Standard | Core: enable/disable geofence |
| ZOP01 | ZONE_TYPE | Standard | Zone shape |
| ZOP19 | ZONE_CENTER_LATITUDE | Standard | Center position |
| ZOP18 | ZONE_CENTER_LONGITUDE | Standard | Center position |
| ZOP20 | ZONE_RADIUS | Standard | Zone size |
| ZOP05 | ZONE_ENABLE_ACTIVATION_DATE | Standard | Time-based activation |
| ZOP06 | ZONE_ACTIVATION_DATE | Standard | When zone becomes active |
| ZOP11 | ZONE_ARGOS_MODE | **Advanced** | Out-of-zone TX strategy |
| ZOP10 | ZONE_ARGOS_REPETITION_SECONDS | **Advanced** | Out-of-zone TX interval |
| ZOP08 | ZONE_ARGOS_DEPTH_PILE | **Advanced** | Out-of-zone packing |
| ZOP12 | ZONE_ARGOS_DUTY_CYCLE | **Advanced** | Out-of-zone duty cycle |
| ZOP13 | ZONE_ARGOS_NTRY_PER_MESSAGE | **Advanced** | Out-of-zone repetitions |
| ZOP14-17 | ZONE_GNSS_* | **Advanced** | Out-of-zone GPS tuning (all 4 params) |

### Sensors

Generic pattern (applies to all sensors):

| DTE Key | Name | Mode | Build | Reason |
|---------|------|------|-------|--------|
| *P01 | *_ENABLE | Standard | Depends on sensor | Enable/disable sensor |
| *P02 | *_PERIODIC | **Advanced** | Depends on sensor | Local logging interval (most users only need TX) |
| *_TX_MODE | *_ENABLE_TX_MODE | Standard | Depends on sensor | Include in satellite packet |
| *_TX_MAX_SAMPLES | *_ENABLE_TX_MAX_SAMPLES | **Advanced** | Depends on sensor | Aggregation tuning |
| *_TX_SAMPLE_PERIOD | *_ENABLE_TX_SAMPLE_PERIOD | **Advanced** | Depends on sensor | Aggregation timing |

Sensor-specific params:

| DTE Key | Name | Mode | Build | Reason |
|---------|------|------|-------|--------|
| AXP01-09 | AXL_* | Mixed | All (default ON) | BMA400 accelerometer |
| AXP03 | AXL_WAKEUP_THRESH | **Advanced** | All | Motion detection threshold |
| AXP04 | AXL_WAKEUP_SAMPLES | **Advanced** | All | Motion detection debounce |
| AXP08 | AXL_MEASUREMENT_RANGE | **Advanced** | All | Accelerometer G-range |
| AXP09 | AXL_POWER_MODE | **Advanced** | All | Low-power vs normal mode |
| PRP01-07 | PRESSURE_* | Mixed | Optional (all boards) | LPS28DFW pressure sensor |
| PRP03 | PRESSURE_LOGGING_MODE | **Advanced** | Optional | Always vs UW-only logging |
| PRP07 | PRESSURE_FULL_SCALE | **Advanced** | Optional | Pressure range (shallow vs deep) |
| THP01-08 | THERMISTOR_* | Mixed | **RSPB always, LinkIt optional** | NTC thermistor (body temp) |
| THP04 | THERMISTOR_WAKEUP_THRESH | **Advanced** | RSPB / optional | Temperature event threshold |
| THP05 | THERMISTOR_WAKEUP_SAMPLES | **Advanced** | RSPB / optional | Temperature event debounce |
| STP01-06 | SEA_TEMP_* | Mixed | **LinkIt only** (never RSPB) | RTD/TSYS01 sea temperature |
| LTP01-06 | ALS_* | Mixed | Optional (all boards) | LTR-303 ambient light |
| PHP01-06 | PH_* | Mixed | Optional (all boards) | OEM pH sensor |
| CDP01-05 | CDT_* | Mixed | Optional (all boards) | Conductivity/Depth/Temp |
| CAP01-05 | CAM_* | Mixed | Optional (all boards) | Camera trigger |

**Mutual exclusion:** `THERMISTOR` and `SEA_TEMP` cannot both be enabled (share same TX packet slot). RSPB always uses Thermistor. LinkIt marine trackers typically use Sea Temp. The GUI should enforce this: enabling one disables the other.

### Mortality Detection (RSPB)

| DTE Key | Name | Mode | Build | Reason |
|---------|------|------|-------|--------|
| MTP01 | MORTALITY_ENABLE | Standard | **RSPB only** | Core: enable mortality detection |
| MTP02 | MORTALITY_ACTIVITY_THRESH | Standard | **RSPB only** | Intuitive: "how still = dead?" |
| MTP03 | MORTALITY_TEMP_THRESH | Standard | **RSPB only** | Intuitive: "how cold = dead?" |
| MTP04 | MORTALITY_GPS_DISTANCE_THRESH | Standard | **RSPB only** | Intuitive: "how far = alive?" |
| MTP05 | MORTALITY_CONFIRM_DAYS | Standard | **RSPB only** | Intuitive: "how long before confirmed?" |
| MTP06 | MORTALITY_DUTY_CYCLE_MODULO | **Advanced** | **RSPB only** | Modifies boot modulo, can affect battery life |
| MTP07 | MORTALITY_ORIGINAL_MODULO | **Advanced** (RO) | **RSPB only** | Internal state backup |

### Power Management (RSPB / TPL5111)

| DTE Key | Name | Mode | Build | Reason |
|---------|------|------|-------|--------|
| PWP01 | SHUTDOWN_TIMER | Standard | **RSPB only** | Safety timeout (intuitive) |
| PWP02 | BOOT_COUNTER | **Advanced** (RO) | **RSPB only** | Internal counter |
| PWP03 | BOOT_COUNTER_MODULO | Standard | **RSPB only** | Core: wakeup frequency |
| PWP04 | WAKEUP_PERIOD | **Advanced** (RO) | **RSPB only** | Hardware constant |
| PWP06 | LAST_KNOWN_RTC | **Advanced** (RO) | **RSPB only** | Internal pseudo-RTC state |

### LED & Debug

| DTE Key | Name | Mode | Reason |
|---------|------|------|--------|
| LDP01 | LED_MODE | Standard | LED on/off |
| LDP02 | EXT_LED_MODE | **Advanced** | External LED, niche |
| DBP01 | DEBUG_OUTPUT_MODE | **Advanced** | Developer-only |

### LoRa RAK3172

| DTE Key | Name | Mode | Build | Reason |
|---------|------|------|-------|--------|
| LRP01 | LORA_DEVEUI | Standard (RO) | **LoRa only** | Device identifier |
| LRP02 | LORA_APPEUI | Standard | **LoRa only** | Network join credential |
| LRP03 | LORA_APPKEY | Standard | **LoRa only** | Network join credential |
| LRP04 | LORA_DEVADDR | **Advanced** | **LoRa only** | ABP mode only |
| LRP05 | LORA_APPSKEY | **Advanced** | **LoRa only** | ABP mode only |
| LRP06 | LORA_NWKSKEY | **Advanced** | **LoRa only** | ABP mode only |
| LRP07 | LORA_NJM | Standard | **LoRa only** | OTAA vs ABP (important choice) |
| LRP08 | LORA_BAND | Standard | **LoRa only** | Frequency band (region-dependent) |
| LRP09 | LORA_CLASS | **Advanced** | **LoRa only** | Device class tuning |
| LRP10 | LORA_DR | **Advanced** | **LoRa only** | Data rate / spreading factor |
| LRP11 | LORA_ADR | **Advanced** | **LoRa only** | Adaptive data rate |
| LRP12 | LORA_TXP | **Advanced** | **LoRa only** | TX power tuning |
| LRP13 | LORA_CFM | **Advanced** | **LoRa only** | Confirmed messages |
| LRP14 | LORA_FPORT | **Advanced** | **LoRa only** | Application port routing |
| LRP15 | LORA_LP_MODE | **Advanced** | **LoRa only** | Standby vs shutdown |

### Certification Mode

| DTE Key | Name | Mode | Build | Reason |
|---------|------|------|-------|--------|
| CTP01-04 | CERT_TX_* | **Advanced** | All | Hardware certification only, never for deployment |

### Summary

| Category | Count | Description |
|----------|-------|-------------|
| **Standard** | ~40 params | What a researcher needs to deploy a tracker |
| **Advanced** | ~70 params | Low-level tuning, protocol details, hardware constants |
| **Read-only** | ~12 params | Status/telemetry, always visible in info panel |
| **RSPB only** | ~12 params | PWP01-06 + MTP01-07 (hidden on LinkIt) |
| **LoRa only** | ~15 params | LRP01-15 (hidden on non-LoRa builds) |
| **SMD only** | 1 param | IDP13 ARGOS_SECKEY (hidden on KIM/LoRa) |
| **LinkIt only** | ~6 params | STP01-06 Sea Temp (never on RSPB) |

---

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

## Mortality Detection (ENABLE_MORTALITY_SENSOR only, RSPB)

| DTE Key | Name | Type | Range | Default | Description |
|---------|------|------|-------|---------|-------------|
| MTP01 | MORTALITY_ENABLE | BOOL | - | false | Enable mortality detection |
| MTP02 | MORTALITY_ACTIVITY_THRESH | UINT | 0-255 | 10 | Activity below which = immobile |
| MTP03 | MORTALITY_TEMP_THRESH | FLOAT | 0-60 °C | 25.0 | Body temp below which = hypothermic |
| MTP04 | MORTALITY_GPS_DISTANCE_THRESH | UINT | 0-10000 m | 50 | Distance below which = stationary |
| MTP05 | MORTALITY_CONFIRM_DAYS | UINT | 1-30 | 3 | Consecutive days before CONFIRMED |
| MTP06 | MORTALITY_DUTY_CYCLE_MODULO | UINT | 0-100 | 0 | Modulo when confirmed (0=disabled) |
| MTP07 | MORTALITY_ORIGINAL_MODULO | UINT | - | 0 | Backup of original modulo (auto, RO) |

**GUI Visibility:** All MTP params → hidden when MTP01=false. MTP06 → greyed with tooltip "0=never modify duty cycle" when value is 0.

## Power Management (EXTERNAL_WAKEUP only)
PWP01 SHUTDOWN_TIMER, PWP02 BOOT_COUNTER (RO), PWP03 BOOT_COUNTER_MODULO, PWP04 WAKEUP_PERIOD, PWP06 LAST_KNOWN_RTC (RO)

## LoRa RAK3172 (LORA_RAK3172 only)
LRP01 DEVEUI (RO), LRP02 APPEUI, LRP03 APPKEY, LRP04 DEVADDR, LRP05 APPSKEY, LRP06 NWKSKEY, LRP07 NJM (0=ABP/1=OTAA), LRP08 BAND (0-12), LRP09 CLASS (0-2), LRP10 DR (0-15), LRP11 ADR, LRP12 TXP (0-14), LRP13 CFM, LRP14 FPORT (1-223), LRP15 LP_MODE (0=shutdown/1=standby)

## Certification Mode
CTP01 CERT_TX_ENABLE, CTP02 CERT_TX_PAYLOAD, CTP03 CERT_TX_MODULATION (LDK/LDA2/VLDA4), CTP04 CERT_TX_REPETITION

## Battery Telemetry (read-only)
POT03 BATT_SOC (%), POT05 LAST_FULL_CHARGE_DATE, POT06 BATT_VOLTAGE (V), SYT01 RTC_CURRENT_TIME

---

# Part 3 — Enum Values & Log Type Reference

All enum types used in DTE parameters. The GUI must use these exact integer values for encoding/decoding.

## BaseArgosMode (ARP01, LBP04, ZOP11)

| Value | Name | GUI Label | Description |
|-------|------|-----------|-------------|
| 0 | OFF | Off | No satellite TX |
| 1 | PASS_PREDICTION | Pass Prediction | TX when satellite overhead (needs AOP) |
| 2 | LEGACY | Legacy | Fixed interval TX, all hours |
| 3 | DUTY_CYCLE | Duty Cycle | Fixed interval within hourly bitmask windows |
| 4 | DOPPLER | Doppler Only | Doppler-only 3-byte packets (no GNSS) |
| 5 | SURFACING_BURST | Surfacing Burst | Progressive Doppler then GNSS (marine) |

## BaseDepthPile (ARP16, LBP08, ZOP08)

Allowed values only — not a contiguous range:

| Value | GUI Label |
|-------|-----------|
| 1 | 1 fix |
| 2 | 2 fixes |
| 3 | 3 fixes |
| 4 | 4 fixes |
| 8 | 8 fixes |
| 12 | 12 fixes |
| 16 | 16 fixes |
| 20 | 20 fixes |
| 24 | 24 fixes |

## BaseDeltaTimeLoc (ARP11, ARP12, ZOP14)

GNSS acquisition period — enum maps to seconds:

| Value | Seconds | GUI Label |
|-------|---------|-----------|
| 1 | 600 | 10 min |
| 2 | 900 | 15 min |
| 3 | 1800 | 30 min |
| 4 | 3600 | 1 hour |
| 5 | 7200 | 2 hours |
| 6 | 10800 | 3 hours |
| 7 | 14400 | 4 hours |
| 8 | 21600 | 6 hours |
| 9 | 43200 | 12 hours |
| 10 | 86400 | 24 hours |

## BaseGNSSDynModel (GNP11)

| Value | Name | GUI Label | Use Case |
|-------|------|-----------|----------|
| 0 | PORTABLE | Portable | General / default |
| 2 | STATIONARY | Stationary | Fixed installations |
| 3 | PEDESTRIAN | Pedestrian | Slow land animals |
| 4 | AUTOMOTIVE | Automotive | Vehicle-mounted |
| 5 | SEA | Sea | Marine animals (turtles, seals) |
| 6 | AIRBORNE_1G | Airborne 1G | **Birds** (high speed, low accel) |
| 7 | AIRBORNE_2G | Airborne 2G | Fast-diving birds |
| 8 | AIRBORNE_4G | Airborne 4G | High-G maneuvers |
| 9 | WRIST_WORN_WATCH | Wrist | Wrist-worn |
| 10 | BIKE | Bike | Bicycle-mounted |

## BaseGNSSFixMode (GNP10)

| Value | Name | GUI Label |
|-------|------|-----------|
| 1 | FIX_2D | 2D Only |
| 2 | FIX_3D | 3D Only |
| 3 | AUTO | Auto (2D or 3D) |

## BaseSensorEnableTxMode (AXP05, THP06, PRP04, STP04, PHP04, LTP04)

| Value | Name | GUI Label | Description |
|-------|------|-----------|-------------|
| 0 | OFF | Off | Sensor data NOT included in satellite packet |
| 1 | ONESHOT | Oneshot | Single reading at TX time |
| 2 | MEAN | Mean | Average of N samples over period |
| 3 | MEDIAN | Median | Median of N samples over period |

## BaseUnderwaterDetectSource (UNP10)

| Value | Name | GUI Label | Description |
|-------|------|-----------|-------------|
| 0 | SWS | Saltwater Switch | Analog conductivity electrode |
| 1 | PRESSURE_SENSOR | Pressure | Depth > threshold = underwater |
| 2 | GNSS | GNSS Signal | Satellite count < threshold = submerged |
| 3 | SWS_GNSS | SWS + GNSS | Combined (highest reliability) |

## BaseLEDMode (LDP01, LDP02)

| Value | Name | GUI Label |
|-------|------|-----------|
| 0 | OFF | Off |
| 1 | HRS_24 | First 24 hours |
| 3 | ALWAYS | Always on |

Note: value 2 is unused (gap in enum).

## BaseDebugMode (DBP01)

| Value | Name | GUI Label |
|-------|------|-----------|
| 0 | UART | UART (SWO pin) |
| 1 | USB_CDC | USB CDC |
| 2 | BLE_NUS | BLE NUS |

## BaseZoneType (ZOP01)

| Value | Name | GUI Label |
|-------|------|-----------|
| 1 | CIRCLE | Circle |

## BasePressureSensorLoggingMode (PRP03)

| Value | Name | GUI Label |
|-------|------|-----------|
| 0 | ALWAYS | Always log |
| 1 | UW_THRESHOLD | Only when underwater |

## BasePressureSensorFullScale (PRP07)

| Value | Name | GUI Label |
|-------|------|-----------|
| 0 | FS_1260 | 1260 hPa (shallow) |
| 1 | FS_4060 | 4060 hPa (deep) |

## BaseArgosModulation (CTP03)

| Value | Name | GUI Label | Packet Size |
|-------|------|-----------|-------------|
| 0 | LDK | LDK | 128 bits (16 bytes) |
| 1 | A2 | LDA2 | 192 bits (24 bytes) |
| 2 | A4 | VLDA4 | 24 bits (3 bytes) |

## MortalityStatus (in Mortality Log entries)

| Value | Name | GUI Label | Color |
|-------|------|-----------|-------|
| 0 | ALIVE | Alive | Green |
| 1 | SUSPECTED | Suspected | Yellow |
| 2 | CONFIRMED | Confirmed | Red |

---

# Part 4 — Log Types (DUMPD / ERASE)

## DUMPD d_type (BaseLogDType)

Used in `$DUMPD#001;X\r` where X is the hex d_type value.

| Hex | Dec | Enum Name | Log File | Description |
|-----|-----|-----------|----------|-------------|
| 0 | 0 | INTERNAL | system.log | System events (boot, state changes, errors) |
| 1 | 1 | GNSS_SENSOR | sensor.log | GPS fixes |
| 2 | 2 | ALS_SENSOR | ALS | Ambient light readings |
| 3 | 3 | PH_SENSOR | PH | pH readings |
| 4 | 4 | RTD_SENSOR | RTD | RTD temperature readings |
| 5 | 5 | CDT_SENSOR | CDT | Conductivity/Depth/Temperature |
| 6 | 6 | CAM_SENSOR | CAM | Camera trigger events |
| 7 | 7 | AXL_SENSOR | AXL | Accelerometer data |
| 8 | 8 | PRESSURE_SENSOR | PRESSURE | Pressure/depth readings |
| 9 | 9 | THERMISTOR_SENSOR | THERMISTOR | NTC temperature readings |
| A | 10 | TSYS01_SENSOR | TSYS01 | TSYS01 sea temperature |
| B | 11 | SWS_LOG | SWS | Saltwater switch transitions |
| **C** | **12** | **MORTALITY** | **MORTALITY** | **Mortality detection state (RSPB)** |

## ERASE log_type (BaseEraseType)

Used in `$ERASE#001;X\r` where X is the decimal log_type value.

| Dec | Enum Name | Target | Description |
|-----|-----------|--------|-------------|
| 1 | GNSS_SENSOR | sensor.log | Erase GPS log |
| 2 | SYSTEM | system.log | Erase system log |
| 3 | ALL | * | Erase ALL logs |
| 4 | ALS_SENSOR | ALS | Erase light log |
| 5 | PH_SENSOR | PH | Erase pH log |
| 6 | RTD_SENSOR | RTD | Erase RTD log |
| 7 | CDT_SENSOR | CDT | Erase CDT log |
| 8 | CAM_SENSOR | CAM | Erase camera log |
| 9 | AXL_SENSOR | AXL | Erase accelerometer log |
| 10 | PRESSURE_SENSOR | PRESSURE | Erase pressure log |
| 11 | THERMISTOR_SENSOR | THERMISTOR | Erase thermistor log |
| 12 | TSYS01_SENSOR | TSYS01 | Erase sea temp log |
| 13 | SWS_LOG | SWS | Erase SWS log |
| **14** | **MORTALITY** | **MORTALITY** | **Erase mortality log (RSPB)** |

## Sensor Calibration Types (SCALW/SCALR)

Used in `$SCALW#...;type,...\r` and `$SCALR#...;type\r`.

| Value | Enum Name | Sensor |
|-------|-----------|--------|
| 0 | AXL | BMA400 Accelerometer |
| 1 | PRESSURE | LPS28DFW Pressure |
| 2 | ALS | LTR-303 Light |
| 3 | PH | pH Sensor |
| 4 | RTD | RTD Temperature |
| 5 | CDT | Conductivity/Depth/Temp |
| 6 | MCP47X6 | DAC (calibration reference) |
| 7 | THERMISTOR | NTC Thermistor |
