This page describes every configurable parameter in detail: what it controls, how it affects device behavior, and recommended values for common deployment scenarios.

Parameters are read/written via the DTE protocol using 5-character keys (see [DTE Commands](https://github.com/arribada/linkit-v4-core/wiki/6-%E2%80%90-DTE-commands)). Default values can be overridden at deployment time using DTE configuration templates.

---

# Identity & Device Info

These parameters identify the device and its Argos credentials.

| Key | Name | Type | Default | Description |
|-----|------|------|---------|-------------|
| IDP12 | ARGOS_DECID | UINT | 0 | **Argos decimal platform ID.** Assigned by CLS when registering the device on the Argos system. Must match the Argos account. Without a valid ID, transmitted messages will not be processed by the Argos ground segment. |
| IDT06 | ARGOS_HEXID | HEX | 0 | **Argos hexadecimal platform ID.** The hex representation of the Argos ID, embedded in every transmitted packet header. Set automatically when writing ARGOS_DECID on SMD builds. |
| IDP11 | PROFILE_NAME | TEXT | "FACTORY" | **Human-readable deployment profile name.** Used to identify which configuration is loaded on the device (e.g., "RSPB_MORTALITY_V1", "TURTLE_UW_V2"). No functional effect, purely organizational. Max 128 characters. |
| IDP13 | ARGOS_SECKEY | TEXT | "" | **A+ security key** (SMD builds only). 128-bit AES key used for KMAC authentication in A+ protocol. Written via `SMDCD` command along with other credentials. |
| IDP14 | ARGOS_RADIOCONF | TEXT | "" | **Argos radio configuration** (SMD builds only). TX power and frequency calibration data specific to the SMD module. Written via `SMDCD`. |

**Read-only identity:**

| Key | Name | Description |
|-----|------|-------------|
| IDT02 | DEVICE_MODEL | Board name (e.g., "LinkIt V4", "RSPB") |
| IDT03 | FW_APP_VERSION | Firmware version string (e.g., "v4.0.1-abc1234") |
| IDT04 | HW_VERSION | Hardware version from BSP (e.g., "LINKIT V4 V1.0") |
| IDT10 | DEVICE_DECID | Unique device ID derived from nRF52840 hardware identifier |

---

# Argos Satellite TX

These parameters control how and when the device transmits to Argos satellites.

## Operating Mode

| Key | Name | Values | Description |
|-----|------|--------|-------------|
| ARP01 | ARGOS_MODE | OFF=0, PP=1, LEGACY=2, DUTY=3, DOPPLER=4 | **Argos transmission mode.** Determines how the device schedules satellite transmissions. **PASS_PREDICTION (1)**: Uses AOP satellite pass data to transmit only when a satellite is overhead — most power-efficient but requires valid AOP data. **LEGACY (2)**: Transmits at fixed intervals regardless of satellite positions — simple but wastes power on transmissions with no satellite overhead. **DUTY_CYCLE (3)**: Transmits during configurable time windows (24-bit mask, one bit per hour). **DOPPLER (4)**: Minimal 3-byte packets for Doppler-based positioning — no GPS data, satellites estimate position from frequency shift. Used in low battery mode. |

## Transmission Timing

| Key | Name | Range | Default | Description |
|-----|------|-------|---------|-------------|
| ARP05 | TR_NOM | 30-1200s | 60 | **Nominal TX repetition period.** Time in seconds between consecutive satellite transmissions. Lower = more messages but higher power consumption. Argos requires minimum 30s between TX. For RSPB with SHUTDOWN_NTIME_SAT=3 and TR_NOM=60: 3 messages over ~3 minutes per session. |
| ARP19 | NTRY_PER_MESSAGE | 0-86400 | 0 (SB: 6) | **TX repetitions per message.** Number of times each message is repeated on-air within a single TX slot. Higher = better chance of satellite reception but more power. 0 means transmit once. Typical: 3-6 for field deployments. |
| ARP35 | ARGOS_TCXO_WARMUP_TIME | 0-30s | 5 | **TCXO warmup time before TX.** The temperature-compensated crystal oscillator needs time to stabilize for accurate frequency. 5s is safe for most conditions. Reduce only if power is critical and temperature is stable. |
| ARP30 | ARGOS_TIME_SYNC_BURST_EN | BOOL | true | **Time-synchronized burst mode.** When enabled, the device aligns TX to Argos time slots for better satellite detection probability. Should be enabled for production deployments. |
| ARP31 | ARGOS_TX_JITTER_EN | BOOL | true | **TX jitter.** Adds small random timing offset to prevent collisions when multiple devices transmit simultaneously. Should be enabled when deploying multiple trackers. |

## Data Packing

| Key | Name | Values | Default | Description |
|-----|------|--------|---------|-------------|
| ARP16 | ARGOS_DEPTH_PILE | 1,2,3,4,8,12,16,20,24 | 16 (UW: 1) | **Depth pile (number of GPS fixes per message).** Controls how many GPS positions are packed into a single Argos packet. Higher = more data per message but larger packets. **1**: Send only the latest fix (used with frequent TX). **16-24**: Accumulate many fixes before sending (used with pass prediction to send a full history when a satellite passes). UW model defaults to 1 because surface time is limited. |
| ARP11 | DLOC_ARG_NOM | AQPERIOD | 600 (SB: 3600) | **GNSS acquisition period.** Time between consecutive GNSS fix attempts in seconds. Controls how often the device acquires a new GPS position. SB model uses 3600s (1h) for birds with long surface periods. |

## Duty Cycle

| Key | Name | Range | Default | Description |
|-----|------|-------|---------|-------------|
| ARP18 | DUTY_CYCLE | 0-16777215 | 0 | **24-bit duty cycle mask** (one bit per hour, bit 0 = midnight UTC). Only used when ARGOS_MODE=DUTY_CYCLE. Each set bit enables TX during that hour. Example: `0x0F0F0F` = TX during hours 0-3, 8-11, 16-19. |

---

# Argos Satellite RX

The device can receive downlink data from Argos satellites to update orbital prediction data (AOP).

| Key | Name | Range | Default | Description |
|-----|------|-------|---------|-------------|
| ARP32 | ARGOS_RX_EN | BOOL | true | **Enable Argos downlink reception.** When enabled, the device listens for AOP updates after each TX session. Receiving AOP data keeps pass predictions accurate. **Disable to save power** (~3.75 mAh per RX window) — recommended for RSPB and battery-constrained deployments where AOP can be uploaded manually via pylinkit. |
| ARP33 | ARGOS_RX_MAX_WINDOW | 1-MAX | 900 | **Maximum RX window duration (seconds).** How long the device listens for a satellite downlink after TX. 900s = 15 minutes. Longer windows increase AOP update probability but consume more power. |
| ARP34 | ARGOS_RX_AOP_UPDATE_PERIOD | 0-MAX | 90 | **Minimum time between AOP updates (seconds).** Prevents redundant AOP processing if multiple downlinks are received in quick succession. |

**Read-only telemetry:**

| Key | Name | Description |
|-----|------|-------------|
| ART01 | LAST_TX | Timestamp of last successful transmission |
| ART02 | TX_COUNTER | Total number of transmissions since last reset |
| ART03 | ARGOS_AOP_DATE | Date of the loaded AOP bulletin (pass prediction data) |
| ART10 | ARGOS_RX_COUNTER | Number of successful RX receptions |
| ART11 | ARGOS_RX_TIME | Total time spent in RX mode (seconds) |

---

# GNSS (GPS)

These parameters control the u-blox M10Q GNSS module: when it runs, how it filters fixes, and how it interacts with underwater detection.

## Core Settings

| Key | Name | Range | Default | Description |
|-----|------|-------|---------|-------------|
| GNP01 | GNSS_EN | BOOL | true | **Enable GNSS module.** When disabled, no GPS fixes are acquired. The device can still transmit Doppler-only packets (ARGOS_MODE=DOPPLER). |
| GNP05 | GNSS_ACQ_TIMEOUT | 10-600s | 120 (UW: 240) | **Warm/hot start acquisition timeout.** Maximum time the GNSS module runs when it has recent ephemeris data (warm start). If no valid fix within this time, GNSS powers off. UW model uses longer timeout (240s) because surface windows are precious. For RSPB: reduce to 90s to save battery on failed fixes. |
| GNP09 | GNSS_COLD_ACQ_TIMEOUT | 10-600s | 530 | **Cold start acquisition timeout.** Maximum time for first fix after power-on or when no almanac/ephemeris is available. Cold starts can take 1-10 minutes depending on sky visibility. For RSPB: reduce to 180s (3 min) as a battery-saving tradeoff. |
| GNP10 | GNSS_FIX_MODE | 2D=1, 3D=2, AUTO=3 | AUTO | **Fix mode.** AUTO lets the receiver choose 2D or 3D based on satellite geometry. 3D requires 4+ satellites but provides altitude. 2D works with 3 satellites but assumes fixed altitude. |
| GNP11 | GNSS_DYN_MODEL | 0-10 | PORTABLE (0) | **u-blox dynamic model.** Tells the GNSS receiver what kind of motion to expect, improving fix quality. **0 (PORTABLE)**: General use. **6 (AIRBORNE_1G)**: For birds — accepts high-speed, low-acceleration motion. **7 (AIRBORNE_2G)**: High-dynamics flight. Wrong model can prevent lock (e.g., PORTABLE rejects bird-speed motion). |

## Fix Filtering

| Key | Name | Range | Default | Description |
|-----|------|-------|---------|-------------|
| GNP02 | GNSS_HDOPFILT_EN | BOOL | true | **Enable HDOP filter.** Reject fixes with poor geometric dilution of precision. |
| GNP03 | GNSS_HDOPFILT_THR | 2-15 | 2 | **HDOP filter threshold.** Fixes with HDOP > this value are rejected. Lower = stricter. 2 is very strict (requires good satellite geometry). |
| GNP20 | GNSS_HACCFILT_EN | BOOL | true | **Enable horizontal accuracy filter.** Reject fixes with estimated accuracy worse than threshold. |
| GNP21 | GNSS_HACCFILT_THR | 0-MAX | 5m | **Horizontal accuracy threshold (meters).** Fixes with hAcc > this are rejected. 5m is strict. For RSPB: relax to 10m to get faster fixes (shorter TTFF = less battery). |
| GNP22 | GNSS_MIN_NUM_FIXES | 1-MAX | 1 | **Consecutive valid fixes required.** Number of successive fixes passing all filters before accepting. Higher = more reliable but longer acquisition. 1 is sufficient for most deployments. |

## Triggers & Scheduling

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| GNP25 | GNSS_TRIGGER_ON_SURFACED | true | **Trigger GNSS when animal surfaces.** When the UW detector confirms surface state, immediately start GNSS acquisition. Essential for marine trackers — maximizes the chance of getting a fix during limited surface time. |
| GNP26 | GNSS_TRIGGER_ON_AXL_WAKEUP | false | **Trigger GNSS on accelerometer wakeup.** Start GNSS when the BMA400 detects motion. Useful for stationary deployments (e.g., nests) where the animal may not move for hours. |
| GNP28 | GNSS_TRIGGER_COLD_START_ON_SURFACED | false | **Force cold start on surfacing.** Discard cached data and do a full cold start when surfacing. Useful if the device has been underwater for so long that ephemeris is stale. |
| GNP23 | GNSS_COLD_START_RETRY_PERIOD | 1-MAX | 60s | **Cold start retry interval.** After a failed cold start, wait this many seconds before trying again. Prevents continuous GNSS drain after repeated failures. |
| GNP30 | GNSS_SESSION_SINGLE_FIX | BOOL | false | **Stop GNSS after first valid fix.** For RSPB: set to 1. Saves significant power by powering off GNSS immediately after one fix instead of continuing to acquire fixes for the depth pile. |

## AssistNow

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| GNP24 | GNSS_ASSISTNOW_EN | true | **Enable AssistNow Online.** Uses cached satellite data to speed up TTFF (time to first fix). Data is obtained from u-blox servers and loaded via `GNSSBR` bridge mode or pylinkit. |
| GNP27 | GNSS_ASSISTNOW_OFFLINE_EN | false | **Enable AssistNow Offline.** Uses pre-loaded almanac data valid for days/weeks. More useful for deployments without frequent BLE connections. Check status with `$GNSSA`. |
| GNP31 | GNSS_TOKEN | TEXT | "" | **AssistNow authentication token.** Required by u-blox servers for AssistNow data downloads. Obtain from u-blox Thingstream portal. |

---

# Underwater Detection

These parameters control how the device detects whether the animal is submerged. When underwater, GNSS and Argos TX are suspended to save power (satellite communication is impossible underwater). See [Behavior - Underwater Mode](https://github.com/arribada/linkit-v4-core/wiki/11-%E2%80%90-LinkIt-UW-Behavior) for the full algorithm description.

## Core Settings

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| UNP01 | UNDERWATER_EN | false (UW: true) | **Enable underwater detection.** Must be enabled for marine deployments. When disabled, the device assumes it is always at the surface. |
| UNP10 | UNDERWATER_DETECT_SOURCE | SWS (0) | **Detection method.** **SWS (0)**: Saltwater switch — analog conductivity electrode, recommended for turtles. **PRESSURE (1)**: Depth > threshold. **GNSS (2)**: Poor GNSS signal = submerged. **SWS_GNSS (3)**: Hybrid — SWS for dive detection, GNSS for surface confirmation. |
| UNP02 | DRY_TIME_BEFORE_TX | 0s | **Surface delay before TX.** Seconds the device must be at the surface before Argos TX is allowed. 0 = TX immediately on surfacing. Increase if false surface detections cause wasted TX attempts. |

## Sampling

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| UNP03 | SAMPLING_UNDER_FREQ | 10s | **Sampling interval underwater.** How often the SWS electrode is read while submerged. Lower = faster surface detection but more power. 1s gives fastest response. |
| UNP04 | SAMPLING_SURF_FREQ | 10s | **Sampling interval at surface.** How often the SWS electrode is read while at the surface. Can be slower than underwater sampling since detecting a dive is less time-critical. |
| UNP05 | UW_MAX_SAMPLES | 1 | **Samples per detection cycle.** Number of sub-samples taken per cycle. With 1 (default), each cycle is a single read. Higher values provide averaging but slower response. |
| UNP06 | UW_MIN_DRY_SAMPLES | 1 | **Consecutive dry samples to confirm surface.** Via threshold-based detection (not L1-L5 overrides). Higher = more stable but slower surface confirmation. |
| UNP07 | UW_SAMPLE_GAP | 1000ms | **Gap between sub-samples.** When UW_MAX_SAMPLES > 1, the interval between consecutive sub-samples within a single cycle. |
| UNP08 | UW_PIN_SAMPLE_DELAY | 1ms | **RC charge time before ADC read.** The time the SWS electrode pin is enabled before reading the ADC. **Must remain at 1ms** — this is a physical constant of the RC circuit that maximizes water/film discrimination. Changing it degrades detection. |

## Safety

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| UNP24 | UW_MAX_DIVE_TIME | 7200s (2h) | **Maximum dive duration.** Force surface state after this time underwater. Prevents the device from being stuck in underwater mode indefinitely (e.g., due to biofouling). 0 = disabled. |
| UNP25 | UW_MIN_SURFACE_TIME | 2s | **Surface lockout after detection.** After detecting surface (via L1-L5 or threshold), ignore underwater readings for this duration. Prevents oscillation from wet electrode at surface. |
| UNP11 | UNDERWATER_DETECT_THRESH | 1.1 | **Threshold for pressure/GNSS detection methods.** For PRESSURE source: depth in meters. For GNSS source: signal quality threshold. Not used with SWS source. |

## SWS Analog Calibration

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| UNP20 | SWS_ANALOG_THRESHOLD_MIN | 0 | **Minimum valid ADC reading.** Readings below this are treated as noise/invalid. |
| UNP21 | SWS_ANALOG_THRESHOLD_MAX | 8000 | **Maximum valid ADC.** Legacy parameter, superseded by dynamic observed peak tracking in the algorithm. |
| UNP22 | SWS_ANALOG_HYSTERESIS | 4% | **Hysteresis as percentage of threshold.** Higher = more stable transitions but slower detection. 4% is a good balance. |
| UNP23 | SWS_ANALOG_CALIB_INTERVAL | 3600s | **Air baseline recalibration interval.** After this time at surface, the air baseline is fully recalibrated to current readings. Handles slow biofouling drift. |

## Dive Mode

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| UNP12 | UW_DIVE_MODE_ENABLE | false | **Enable dive mode state machine.** Adds reed switch pause/resume for dive tracking. Specialized for dive-logging deployments. |
| UNP13 | UW_DIVE_MODE_START_TIME | 0 | **Dive mode start time.** Unix timestamp when dive mode was activated. |

## GNSS-Based Detection (UNP10=2 or 3)

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| UNP14 | UW_GNSS_DRY_SAMPLING | 14400s | **GNSS sampling interval at surface (GNSS detection).** How often to run a GNSS acquisition to confirm surface state. |
| UNP15 | UW_GNSS_WET_SAMPLING | 14400s | **GNSS sampling interval underwater (GNSS detection).** |
| UNP16 | UW_GNSS_MAX_SAMPLES | 10 | **GNSS detection max samples per cycle.** |
| UNP17 | UW_GNSS_MIN_DRY_SAMPLES | 1 | **GNSS detection dry samples to confirm surface.** |
| UNP18 | UW_GNSS_DETECT_THRESH | 1 (UW: 2) | **GNSS detection signal quality threshold.** Number of satellites below which the device is considered submerged. |

---

# Low Battery Mode

When the battery drops below a threshold, the device switches to a reduced-power mode with different GNSS and Argos parameters. See [RSPB Battery Modes](https://github.com/arribada/linkit-v4-core/wiki/12-%E2%80%90-RSPB-Mortality-Tracker) for the full behavior description.

## Mode Control

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| LBP01 | LB_EN | false | **Enable low battery mode.** When battery SOC drops below LB_THRESHOLD, the device switches to LB parameters. Essential for RSPB deployments to maximize lifetime. |
| LBP02 | LB_THRESHOLD | 10% | **Battery percentage to enter LB mode.** When SOC < this value and LB_EN=1, the device uses LB parameters instead of normal ones. RSPB typical: 10-30%. |
| LBP12 | LB_CRITICAL_THRESH | 5% | **Critical battery threshold.** Below this SOC, the device powers off immediately without any operation. Protects the battery from deep discharge. On RSPB, the TPL5111 keeps waking the device — once solar recharges above this threshold, operation resumes. |

## LB GNSS Settings

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| LBP06 | LB_GNSS_EN | true | **GNSS in LB mode.** Set to 0 for Doppler-only operation (no GPS). Saves significant power. Argos satellites estimate position from Doppler shift (~5-10km accuracy). |
| LBP09 | LB_GNSS_ACQ_TIMEOUT | 120s | **GNSS timeout in LB mode.** Shorter timeout to fail faster and save battery. |
| LBP07 | LB_GNSS_HDOPFILT_THR | 2 | **HDOP filter in LB mode.** Can be relaxed to accept lower-quality fixes. |
| LBP10 | LB_GNSS_HACCFILT_THR | 5m | **Accuracy filter in LB mode.** |

## LB Argos Settings

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| LBP04 | LB_ARGOS_MODE | LEGACY | **Argos mode in LB.** Typically LEGACY or DOPPLER. |
| ARP06 | TR_LB | 240s | **TX interval in LB mode.** Time between Doppler/LB transmissions. RSPB typical: 90s. |
| LBP11 | LB_NTRY_PER_MESSAGE | 4 | **TX repetitions in LB mode.** |
| LBP08 | LB_ARGOS_DEPTH_PILE | 1 | **Depth pile in LB mode.** Usually 1 (Doppler packets are small). |
| LBP05 | LB_ARGOS_DUTY_CYCLE | 0 | **Duty cycle mask in LB mode.** |
| ARP12 | DLOC_ARG_LB | 3600s | **GNSS period in LB mode.** |
| LBP14 | LB_SHUTDOWN_NTIME_SAT | 0 | **Number of TX before powerdown in LB mode** (RSPB only). 0 = use SHUTDOWN_TIMER only. Set to 2 for RSPB to send 2 Doppler messages then power off. |

---

# Geofencing Zone

Define a circular zone around a reference point. When the device detects it is outside this zone, it switches to alternate Argos parameters (typically more aggressive TX).

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| ZOP04 | ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE | false | **Enable geofencing.** When enabled, the device compares each GPS fix against the zone boundary. |
| ZOP01 | ZONE_TYPE | CIRCLE | **Zone shape.** Currently only CIRCLE is supported. |
| ZOP19 | ZONE_CENTER_LATITUDE | -48.8752 | **Zone center latitude** (decimal degrees, -90 to 90). |
| ZOP18 | ZONE_CENTER_LONGITUDE | -123.3925 | **Zone center longitude** (decimal degrees, -180 to 180). |
| ZOP20 | ZONE_RADIUS | 1000m | **Zone radius in meters.** |
| ZOP05 | ZONE_ENABLE_ACTIVATION_DATE | true | **Enable zone activation date.** Zone checking only starts after ZONE_ACTIVATION_DATE. Useful for delayed deployment. |
| ZOP06 | ZONE_ACTIVATION_DATE | 01/01/2020 | **Zone activation date** (DD/MM/YYYY HH:MM:SS). |

**Out-of-zone Argos overrides** (used when device is outside the zone):

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| ZOP11 | ZONE_ARGOS_MODE | LEGACY | Argos mode when out of zone |
| ZOP10 | ZONE_ARGOS_REPETITION_SECONDS | 240s | TX interval when out of zone |
| ZOP08 | ZONE_ARGOS_DEPTH_PILE | 1 | Depth pile when out of zone |
| ZOP12 | ZONE_ARGOS_DUTY_CYCLE | 0xFFFFFF | Duty cycle when out of zone |
| ZOP13 | ZONE_ARGOS_NTRY_PER_MESSAGE | 0 | TX repetitions when out of zone |
| ZOP14 | ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS | 3600s | GNSS period when out of zone |
| ZOP15 | ZONE_GNSS_HDOPFILT_THR | 2 | HDOP filter when out of zone |
| ZOP16 | ZONE_GNSS_HACCFILT_THR | 5m | Accuracy filter when out of zone |
| ZOP17 | ZONE_GNSS_ACQ_TIMEOUT | 240s | GNSS timeout when out of zone |

---

# Pass Prediction

These parameters tune the Prepass algorithm used in PASS_PREDICTION mode (ARP01=1) to compute when Argos satellites are overhead.

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| PPP01 | PP_MIN_ELEVATION | 15.0° | **Minimum satellite elevation.** Ignore passes where the satellite is below this angle above the horizon. Higher = better signal but fewer passes. |
| PPP02 | PP_MAX_ELEVATION | 90.0° | **Maximum satellite elevation.** Usually 90° (zenith). |
| PPP03 | PP_MIN_DURATION | 30s | **Minimum pass duration.** Ignore passes shorter than this. Very short passes have poor reception. |
| PPP04 | PP_MAX_PASSES | 1000 | **Maximum passes to compute.** Limits computation time and memory. |
| PPP05 | PP_LINEAR_MARGIN | 300s | **Linear margin (seconds).** Time buffer added before/after computed pass window to account for prediction errors. |
| PPP06 | PP_COMP_STEP | 10s | **Computation step size.** Resolution of the pass prediction search in seconds. Smaller = more precise but slower computation. |

---

# Sensors

All sensors follow the same parameter pattern. Each sensor has:
- **ENABLE** — Turn the sensor on/off
- **PERIODIC** — Sampling period in seconds (0 = sample only when triggered)
- **ENABLE_TX_MODE** — How sensor data is included in satellite packets
- **ENABLE_TX_MAX_SAMPLES** — Maximum samples aggregated per TX event
- **ENABLE_TX_SAMPLE_PERIOD** — Sampling interval for TX data collection (ms)

## TX Aggregation Modes

| Value | Mode | Description |
|-------|------|-------------|
| 0 | OFF | Sensor data is not included in satellite packets |
| 1 | ONESHOT | Include the most recent single sample |
| 2 | MEAN | Include the average of all samples since last TX |
| 3 | MEDIAN | Include the median of all samples since last TX |

## Pressure Sensor (LPS28DFW)

Requires `ENABLE_PRESSURE_SENSOR` at build time. Measures barometric pressure, temperature, and computes altitude.

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| PRP01 | PRESSURE_SENSOR_ENABLE | false | Enable pressure sensor |
| PRP02 | PRESSURE_SENSOR_PERIODIC | 0 | Sampling period (seconds). 0 = triggered only. |
| PRP03 | PRESSURE_SENSOR_LOGGING_MODE | ALWAYS (0) | **ALWAYS (0)**: Log every sample. **UW_THRESHOLD (1)**: Log only when depth crosses a threshold (reduces flash writes for UW deployments). |
| PRP04 | PRESSURE_SENSOR_ENABLE_TX_MODE | OFF | TX aggregation mode |
| PRP05 | PRESSURE_SENSOR_ENABLE_TX_MAX_SAMPLES | 1 | Max samples per TX |
| PRP06 | PRESSURE_SENSOR_ENABLE_TX_SAMPLE_PERIOD | 1000ms | TX sampling interval |
| PRP07 | PRESSURE_SENSOR_FULL_SCALE | 1260hPa (0) | **Full scale range.** 0 = 1260 hPa (standard atmospheric). 1 = 4060 hPa (for deep underwater pressure measurement up to ~30m depth). |

## Accelerometer (BMA400)

Requires `ENABLE_AXL_SENSOR`. 3-axis accelerometer with wakeup detection and activity monitoring.

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| AXP01 | AXL_SENSOR_ENABLE | false | Enable accelerometer |
| AXP02 | AXL_SENSOR_PERIODIC | 0 | Sampling period (seconds) |
| AXP03 | AXL_SENSOR_WAKEUP_THRESH | 0.0g | **Wakeup threshold (g).** The acceleration level that triggers a wakeup interrupt. 0 = wakeup disabled. For mortality detection: any non-zero value detects movement. |
| AXP04 | AXL_SENSOR_WAKEUP_SAMPLES | 5 | **Wakeup samples.** Number of consecutive samples above threshold before triggering wakeup. Higher = fewer false triggers. |
| AXP08 | AXL_SENSOR_MEASUREMENT_RANGE | 0 | **Measurement range.** 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g. Lower range = better resolution. ±2g is sufficient for most wildlife tracking. |
| AXP09 | AXL_SENSOR_POWER_MODE | 0 | **Power mode.** 0 = Low Power, 1 = Normal, 2 = Sleep. Low Power is recommended for battery-operated deployments. |
| AXP05 | AXL_SENSOR_ENABLE_TX_MODE | OFF | TX aggregation mode |
| AXP06 | AXL_SENSOR_ENABLE_TX_MAX_SAMPLES | 1 | Max samples per TX |
| AXP07 | AXL_SENSOR_ENABLE_TX_SAMPLE_PERIOD | 1000ms | TX sampling interval |

## Thermistor (NTC)

Requires `ENABLE_THERMISTOR_SENSOR`. Reads temperature from an NTC thermistor via ADC.

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| THP01 | THERMISTOR_SENSOR_ENABLE | false | Enable thermistor |
| THP02 | THERMISTOR_SENSOR_PERIODIC | 0 | Sampling period (seconds) |
| THP03 | THERMISTOR_SENSOR_VALUE | 0.0 | **Last reading (read-only).** Current temperature value in °C. Read via STATR. |
| THP04 | THERMISTOR_SENSOR_WAKEUP_THRESH | 0.0°C | **Wakeup threshold.** Temperature above which a wakeup event is generated. 0 = disabled. |
| THP05 | THERMISTOR_SENSOR_WAKEUP_SAMPLES | 0 | **Wakeup samples.** Consecutive readings above threshold before triggering. 0 = disabled. |
| THP06 | THERMISTOR_SENSOR_ENABLE_TX_MODE | OFF | TX aggregation mode |
| THP07 | THERMISTOR_SENSOR_ENABLE_TX_MAX_SAMPLES | 1 | Max samples per TX |
| THP08 | THERMISTOR_SENSOR_ENABLE_TX_SAMPLE_PERIOD | 1000ms | TX sampling interval |

## Other Sensors

These sensors follow the same ENABLE / PERIODIC / TX_MODE pattern. All require their respective `ENABLE_*` build flag.

**Ambient Light (LTR-303)** — Keys: LTP01-LTP06. Measures luminosity in lumens.

**pH Sensor (OEM)** — Keys: PHP01-PHP06. Measures pH value.

**Sea Temperature (RTD or TSYS01)** — Keys: STP01-STP06. High-precision ocean temperature.

**Conductivity-Depth-Temperature (CDT)** — Keys: CDP01-CDP05. Multi-parameter oceanographic sensor.

**Camera** — Keys: CAP01-CAP05. Triggers an external camera on surfaced or accelerometer wakeup events.

## Mortality Detection (RSPB)

Requires `ENABLE_MORTALITY_SENSOR` at build time (auto-enabled on RSPB). Combines accelerometer activity, body temperature, and GPS stationarity to compute a mortality confidence percentage (0-100%) transmitted in every satellite sensor packet.

**WARNING:** Mortality detection requires AXL (`AXP01=1`), Thermistor (`THP01=1`), and GNSS (`GNP01=1`) to be enabled and active. If any sensor is disabled, the algorithm works with partial data only (biased toward ALIVE). The RSPB build script enables all three by default.

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| MTP01 | MORTALITY_ENABLE | false | Enable mortality detection. When false, service is disabled, zero CPU/flash impact. |
| MTP02 | MORTALITY_ACTIVITY_THRESH | 10 | **Activity threshold (0-255).** BMA400 activity score below which the bird is considered immobile. At rest: ~0-5, walking: ~20-50, flying: ~100+. |
| MTP03 | MORTALITY_TEMP_THRESH | 25.0 | **Body temperature threshold (°C).** Below this = hypothermic. Live bird body temp ~38-42°C. Dead bird converges to ambient (~10-25°C). |
| MTP04 | MORTALITY_GPS_DISTANCE_THRESH | 50 | **Stationarity threshold (meters).** If GPS position moved less than this since last session AND speed < 0.1 m/s, bird is considered stationary. |
| MTP05 | MORTALITY_CONFIRM_DAYS | 3 | **Confirmation period (days).** Number of consecutive days with confidence >= 80% before status transitions to CONFIRMED. |
| MTP06 | MORTALITY_DUTY_CYCLE_MODULO | 0 | **Duty cycle when confirmed.** Replaces BOOT_COUNTER_MODULO when mortality is confirmed. **0 = disabled** (never modify duty cycle, just report confidence). Set > 0 to opt in. |
| MTP07 | MORTALITY_ORIGINAL_MODULO | 0 | **Backup modulo (read-only).** Auto-saved when mortality first confirmed. Used to restore original duty cycle on recovery. |

---

# Power Management (RSPB / TPL5111)

These parameters are only active on `EXTERNAL_WAKEUP` builds (RSPB). They control the duty cycling behavior. See [RSPB Behavior](https://github.com/arribada/linkit-v4-core/wiki/12-%E2%80%90-RSPB-Mortality-Tracker) for the full description.

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| PWP01 | SHUTDOWN_TIMER | 0 | **Safety shutdown timer (seconds).** Maximum time the device stays awake per cycle. When this timer expires, the device powers down regardless of what it's doing. 0 = disabled. **RSPB recommended: 600s (10 min).** Prevents stuck sessions from draining the battery. |
| PWP03 | BOOT_COUNTER_MODULO | 2 | **Active cycle frequency.** The device runs an active cycle (GNSS + TX) every Nth TPL5111 wakeup. Intermediate wakeups just increment the counter and power off (~50ms). **Effective cycle period = WAKEUP_PERIOD × BOOT_COUNTER_MODULO.** With default 6300s × 2 = 3.5h. RSPB typical: 4 (~7h) or 5 (~8.75h). |
| PWP05 | SHUTDOWN_NTIME_SAT | 0 | **TX count before powerdown.** After sending this many satellite messages, the device powers down. 0 = disabled (only SHUTDOWN_TIMER controls session end). RSPB typical: 3-5. |
| PWP06 | LAST_KNOWN_RTC | 0 | **Flash-persisted RTC (read-only).** Updated on every GNSS fix and RTCW command. Used by the pseudo-RTC chain to maintain approximate time between power cycles. |

**Read-only:**

| Key | Name | Description |
|-----|------|-------------|
| PWP02 | BOOT_COUNTER | Current boot counter value since last active cycle |
| PWP04 | WAKEUP_PERIOD | TPL5111 hardware wakeup period (~6300s = 1h45m) |

---

# LED

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| LDP01 | LED_MODE | HRS_24 (1) | **Internal RGB LED behavior.** **OFF (0)**: LED always off — recommended for bird trackers (invisible, saves energy). **HRS_24 (1)**: LED active for the first 24 hours after boot — useful for initial deployment verification, then auto-disables. **ALWAYS (3)**: LED always active — useful for development and testing. |
| LDP02 | EXT_LED_MODE | ALWAYS (3) | **External single-color LED.** Same modes as LED_MODE. Controls the optional external status LED connector. |

---

# Debug

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| DBP01 | DEBUG_OUTPUT_MODE | USB_CDC (1) | **Debug log output interface.** **UART (0)**: Debug output on the J3 UART connector. Default for RSPB (no USB). Requires a USB-to-UART adapter. **USB_CDC (1)**: Debug output on the USB connector as a virtual serial port. Default for LinkIt V4 (KIM, SMD, LoRa). Connect at 115200 baud 8N1. **BLE_NUS (2)**: Debug output over Bluetooth (Nordic UART Service). Available on all boards. Use nRF Connect app or `pylinkit --log`. |

---

# Certification Test

These parameters are used for Argos TX certification and testing. Not used in normal operation.

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| CTP01 | CERT_TX_ENABLE | false | **Enable certification TX mode.** When enabled, the device continuously transmits a fixed payload for TX power and frequency certification. Overrides normal Argos behavior. |
| CTP02 | CERT_TX_PAYLOAD | 27 bytes 0xFF | **Certification TX payload.** Fixed hex-encoded data transmitted in certification mode. |
| CTP03 | CERT_TX_MODULATION | LDA2 (1) | **Certification modulation.** 0 = LDK, 1 = LDA2, 2 = VLDA4. |
| CTP04 | CERT_TX_REPETITION | 60s | **Certification TX interval.** Time between certification transmissions. |

---

# LoRa RAK3172 Configuration

These parameters are only available on `LORA_RAK3172=ON` builds (LinkIt V4 LoRa). They configure the RAK3172-SiP LoRaWAN module.

## Network Credentials

| Key | Name | Description |
|-----|------|-------------|
| LRP01 | LORA_DEVEUI | **Device EUI (read-only).** 16 hex chars. Unique identifier burned into the RAK3172 module. Cannot be changed. |
| LRP02 | LORA_APPEUI | **Application EUI.** 16 hex chars. Provided by the LoRaWAN network server (e.g., TTN, Chirpstack). |
| LRP03 | LORA_APPKEY | **Application key.** 32 hex chars. Secret key for OTAA join. Must match the network server configuration. |
| LRP04 | LORA_DEVADDR | **Device address** (ABP mode). 8 hex chars. |
| LRP05 | LORA_APPSKEY | **Application session key** (ABP mode). 32 hex chars. |
| LRP06 | LORA_NWKSKEY | **Network session key** (ABP mode). 32 hex chars. |

## Radio Configuration

| Key | Name | Default | Description |
|-----|------|---------|-------------|
| LRP07 | LORA_NJM | 1 (OTAA) | **Network join mode.** 0 = ABP (pre-provisioned keys), 1 = OTAA (over-the-air activation). OTAA is recommended for most deployments. |
| LRP08 | LORA_BAND | 4 (EU868) | **Frequency band.** 0=EU433, 1=CN470, 2=RU864, 3=IN865, 4=EU868, 5=US915, 6=AU915, 7=KR920, 8=AS923-1, 9=AS923-2, 10=AS923-3, 11=AS923-4. Must match your regional regulations. |
| LRP09 | LORA_CLASS | 0 (A) | **LoRaWAN class.** 0=A (uplink-initiated, lowest power), 1=B (beacon-synchronized), 2=C (continuous RX). Class A is recommended for battery-operated wildlife trackers. |
| LRP10 | LORA_DR | 3 (SF9) | **Data rate.** 0=SF12 (longest range, lowest throughput), 5=SF7 (shortest range, highest throughput). DR3 (SF9) is a good compromise for wildlife tracking payloads. |
| LRP11 | LORA_ADR | false | **Adaptive data rate.** Let the network server optimize the data rate based on signal quality. Useful in fixed deployments, less predictable for mobile wildlife trackers. |
| LRP12 | LORA_TXP | 0 | **TX power index.** 0 = maximum power (14 dBm for EU868). Higher index = lower power. |
| LRP13 | LORA_CFM | false | **Confirmed uplinks.** Request server acknowledgment for each message. Increases reliability but uses more airtime and battery (downlink RX windows). |
| LRP14 | LORA_FPORT | 2 | **Application FPort.** LoRaWAN port number (1-223). Used by the server to route messages to the correct application handler. |
| LRP15 | LORA_LP_MODE | 1 (standby) | **RAK3172 low-power mode.** 0 = stop mode (deepest sleep, slower wake), 1 = standby mode (faster wake, slightly higher idle current). |

---

# Battery & System Status (Read-only)

These values are read via `$STATR`:

| Key | Name | Description |
|-----|------|-------------|
| POT03 | BATT_SOC | Battery state of charge (%). Fuel gauge on RSPB (STC3117), estimated from voltage on LinkIt V4. |
| POT06 | BATT_VOLTAGE | Battery voltage in volts (e.g., 3.85). |
| POT05 | LAST_FULL_CHARGE_DATE | Timestamp of last detected full charge (wireless charging on LinkIt V4). |
| SYT01 | RTC_CURRENT_TIME | Current RTC value as Unix timestamp. On RSPB, this is approximate between GNSS fixes. |
