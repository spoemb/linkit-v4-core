# Parameter Reference

Complete reference of all 178 configurable parameters. Parameters are grouped by function.

## Key Format

Each parameter has a 5-character DTE key (3-letter prefix + 2-digit number) used in PARMR/PARMW commands.

## Parameter Table

### Identity & Device Info

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 0 | ARGOS_DECID | IDP12 | UINT | 0-4294967295 | 0 | RW | |
| 1 | ARGOS_HEXID | IDT06 | HEX | 0-FFFFFFFF | 0 | RW | |
| 2 | DEVICE_MODEL | IDT02 | TEXT | - | board name | R | |
| 3 | FW_APP_VERSION | IDT03 | TEXT | - | auto | R | |
| 86 | HW_VERSION | IDT04 | TEXT | - | auto | R | |
| 93 | DEVICE_DECID | IDT10 | UINT | - | auto | R | |
| 8 | PROFILE_NAME | IDP11 | TEXT | - | "FACTORY" | RW | |

### Argos TX Configuration

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 14 | ARGOS_MODE | ARP01 | ARGOSMODE | {OFF=0, PP=1, LEGACY=2, DUTY=3, DOPPLER=4} | LEGACY (SB: PP) | RW | |
| 13 | TR_NOM | ARP05 | UINT | 30-1200 | 60 | RW | |
| 15 | NTRY_PER_MESSAGE | ARP19 | UINT | 0-86400 | 0 (SB: 6) | RW | |
| 16 | DUTY_CYCLE | ARP18 | UINT | 0-16777215 | 0 | RW | |
| 19 | ARGOS_DEPTH_PILE | ARP16 | DEPTHPILE | {1, 2, 3, 4, 8, 12, 16, 20, 24} | 16 (UW: 1) | RW | |
| 18 | DLOC_ARG_NOM | ARP11 | AQPERIOD | {0-8} | 600 (SB: 3600) | RW | |
| 54 | ARGOS_TIME_SYNC_BURST_EN | ARP30 | BOOL | - | true | RW | |
| 56 | ARGOS_TX_JITTER_EN | ARP31 | BOOL | - | true | RW | |
| 92 | ARGOS_TCXO_WARMUP_TIME | ARP35 | UINT | 0-30 | 5 | RW | |

### Argos RX Configuration

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 57 | ARGOS_RX_EN | ARP32 | BOOL | - | true | RW | |
| 58 | ARGOS_RX_MAX_WINDOW | ARP33 | UINT | 1-MAX | 900 | RW | |
| 59 | ARGOS_RX_AOP_UPDATE_PERIOD | ARP34 | UINT | 0-MAX | 90 | RW | |

### Argos Telemetry (Read-only)

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 10 | ARGOS_AOP_DATE | ART03 | DATE | - | auto | R | |
| 4 | LAST_TX | ART01 | DATE | - | auto | R | |
| 5 | TX_COUNTER | ART02 | UINT | - | auto | R | |
| 60 | ARGOS_RX_COUNTER | ART10 | UINT | - | auto | R | |
| 61 | ARGOS_RX_TIME | ART11 | UINT | - | auto | R | |

### GNSS Configuration

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 17 | GNSS_EN | GNP01 | BOOL | - | true | RW | |
| 22 | GNSS_HDOPFILT_EN | GNP02 | BOOL | - | true | RW | |
| 23 | GNSS_HDOPFILT_THR | GNP03 | UINT | 2-15 | 2 | RW | |
| 24 | GNSS_ACQ_TIMEOUT | GNP05 | UINT | 10-600 | 120 (UW: 240) | RW | |
| 47 | GNSS_COLD_ACQ_TIMEOUT | GNP09 | UINT | 10-600 | 530 | RW | |
| 48 | GNSS_FIX_MODE | GNP10 | GNSSFIXMODE | {2D=1, 3D=2, AUTO=3} | AUTO | RW | |
| 49 | GNSS_DYN_MODEL | GNP11 | GNSSDYNMODEL | {0, 2-10} | PORTABLE | RW | |
| 50 | GNSS_HACCFILT_EN | GNP20 | BOOL | - | true | RW | |
| 51 | GNSS_HACCFILT_THR | GNP21 | UINT | 0-MAX | 5 | RW | |
| 52 | GNSS_MIN_NUM_FIXES | GNP22 | UINT | 1-MAX | 1 | RW | |
| 53 | GNSS_COLD_START_RETRY_PERIOD | GNP23 | UINT | 1-MAX | 60 | RW | |
| 62 | GNSS_ASSISTNOW_EN | GNP24 | BOOL | - | true | RW | |
| 94 | GNSS_TRIGGER_ON_SURFACED | GNP25 | BOOL | - | true | RW | |
| 95 | GNSS_TRIGGER_ON_AXL_WAKEUP | GNP26 | BOOL | - | false | RW | |
| 127 | GNSS_ASSISTNOW_OFFLINE_EN | GNP27 | BOOL | - | false | RW | |
| 147 | GNSS_TRIGGER_COLD_START_ON_SURFACED | GNP28 | BOOL | - | false | RW | |
| 176 | GNSS_SESSION_SINGLE_FIX | GNP30 | BOOL | - | false | RW | |

### Underwater Detection

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 26 | UNDERWATER_EN | UNP01 | BOOL | - | false (UW: true) | RW | |
| 27 | DRY_TIME_BEFORE_TX | UNP02 | UINT | 0-MAX | 0 | RW | Seconds at surface before TX. 0 = immediate |
| 28 | SAMPLING_UNDER_FREQ | UNP03 | UINT | 1-MAX | 60 | RW | |
| 40 | SAMPLING_SURF_FREQ | UNP04 | UINT | 1-MAX | 60 | RW | |
| 128 | UW_MAX_SAMPLES | UNP05 | UINT | 1-MAX | 5 (UW: 10) | RW | |
| 129 | UW_MIN_DRY_SAMPLES | UNP06 | UINT | 1-MAX | 1 (UW: 3) | RW | |
| 130 | UW_SAMPLE_GAP | UNP07 | UINT | 1-MAX | 1000 | RW | |
| 131 | UW_PIN_SAMPLE_DELAY | UNP08 | UINT | 1-MAX | 1 (UW: 10) | RW | |
| 96 | UNDERWATER_DETECT_SOURCE | UNP10 | UWDETECTSOURCE | {SWS=0, PRESSURE=1, GNSS=2, SWS_GNSS=3} | SWS (UW: SWS_GNSS) | RW | |
| 97 | UNDERWATER_DETECT_THRESH | UNP11 | FLOAT | 0-MAX | 1.1 | RW | |
| 138 | UW_DIVE_MODE_ENABLE | UNP12 | BOOL | - | false | RW | |
| 139 | UW_DIVE_MODE_START_TIME | UNP13 | UINT | 0-MAX | 0 | RW | |
| 140 | UW_GNSS_DRY_SAMPLING | UNP14 | UINT | 1-MAX | 14400 | RW | |
| 141 | UW_GNSS_WET_SAMPLING | UNP15 | UINT | 1-MAX | 14400 | RW | |
| 142 | UW_GNSS_MAX_SAMPLES | UNP16 | UINT | 1-MAX | 10 | RW | |
| 143 | UW_GNSS_MIN_DRY_SAMPLES | UNP17 | UINT | 1-MAX | 1 | RW | |
| 144 | UW_GNSS_DETECT_THRESH | UNP18 | UINT | 1-7 | 1 (UW: 2) | RW | |
| 136 | UW_MAX_DIVE_TIME | UNP24 | UINT | 0-MAX | 7200 | RW | |
| 137 | UW_MIN_SURFACE_TIME | UNP25 | UINT | 0-MAX | 10 | RW | |

### SWS Analog

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 132 | SWS_ANALOG_THRESHOLD_MIN | UNP20 | UINT | 50-4095 | 100 | RW | |
| 133 | SWS_ANALOG_THRESHOLD_MAX | UNP21 | UINT | 50-4095 | 3000 | RW | |
| 134 | SWS_ANALOG_HYSTERESIS | UNP22 | UINT | 0-50 | 10 | RW | |
| 135 | SWS_ANALOG_CALIB_INTERVAL | UNP23 | UINT | 60-MAX | 3600 | RW | |

### Low Battery Mode

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 29 | LB_EN | LBP01 | BOOL | - | false | RW | |
| 30 | LB_THRESHOLD | LBP02 | UINT | 0-100 | 10 | RW | |
| 145 | LB_CRITICAL_THRESH | LBP12 | UINT | 0-100 | 5 | RW | Battery SOC (%) below which device enters critical poweroff |
| 32 | TR_LB | ARP06 | UINT | 30-1200 | 240 | RW | |
| 33 | LB_ARGOS_MODE | LBP04 | ARGOSMODE | {0-4} | LEGACY | RW | |
| 34 | LB_ARGOS_DUTY_CYCLE | LBP05 | UINT | 0-16777215 | 0 | RW | |
| 35 | LB_GNSS_EN | LBP06 | BOOL | - | true | RW | |
| 36 | DLOC_ARG_LB | ARP12 | AQPERIOD | {0-8} | 3600 | RW | |
| 37 | LB_GNSS_HDOPFILT_THR | LBP07 | UINT | 2-15 | 2 | RW | |
| 38 | LB_ARGOS_DEPTH_PILE | LBP08 | DEPTHPILE | {1-4, 8-12, 16, 20, 24} | 1 | RW | |
| 39 | LB_GNSS_ACQ_TIMEOUT | LBP09 | UINT | 10-600 | 120 | RW | |
| 63 | LB_GNSS_HACCFILT_THR | LBP10 | UINT | 0-MAX | 5 | RW | |
| 64 | LB_NTRY_PER_MESSAGE | LBP11 | UINT | 0-86400 | 4 | RW | |
| 175 | LB_SHUTDOWN_NTIME_SAT | LBP14 | UINT | 0-65535 | 0 | RW | |

### Zone (Geofencing)

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 65 | ZONE_TYPE | ZOP01 | ZONETYPE | {CIRCLE=1} | CIRCLE | RW | |
| 66 | ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE | ZOP04 | BOOL | - | false | RW | |
| 67 | ZONE_ENABLE_ACTIVATION_DATE | ZOP05 | BOOL | - | true | RW | |
| 68 | ZONE_ACTIVATION_DATE | ZOP06 | DATE | - | 01/01/2020 | RW | |
| 69 | ZONE_ARGOS_DEPTH_PILE | ZOP08 | DEPTHPILE | {1-4, 8-12, 16, 20, 24} | 1 | RW | |
| 71 | ZONE_ARGOS_REPETITION_SECONDS | ZOP10 | UINT | 30-1200 | 240 (SB: 60) | RW | |
| 72 | ZONE_ARGOS_MODE | ZOP11 | ARGOSMODE | {0-4} | LEGACY (SB: PP) | RW | |
| 73 | ZONE_ARGOS_DUTY_CYCLE | ZOP12 | UINT | 0-16777215 | 16777215 | RW | |
| 74 | ZONE_ARGOS_NTRY_PER_MESSAGE | ZOP13 | UINT | 0-86400 | 0 | RW | |
| 75 | ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS | ZOP14 | AQPERIOD | {0-8} | 3600 | RW | |
| 76 | ZONE_GNSS_HDOPFILT_THR | ZOP15 | UINT | 2-15 | 2 | RW | |
| 77 | ZONE_GNSS_HACCFILT_THR | ZOP16 | UINT | 0-MAX | 5 | RW | |
| 78 | ZONE_GNSS_ACQ_TIMEOUT | ZOP17 | UINT | 10-600 | 240 | RW | |
| 79 | ZONE_CENTER_LONGITUDE | ZOP18 | FLOAT | -180 to 180 | -123.3925 | RW | |
| 80 | ZONE_CENTER_LATITUDE | ZOP19 | FLOAT | -90 to 90 | -48.8752 | RW | |
| 81 | ZONE_RADIUS | ZOP20 | UINT | 1-MAX | 1000 | RW | |

### Pass Prediction

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 41 | PP_MIN_ELEVATION | PPP01 | FLOAT | 0-90 | 15.0 | RW | |
| 42 | PP_MAX_ELEVATION | PPP02 | FLOAT | 0-90 | 90.0 | RW | |
| 43 | PP_MIN_DURATION | PPP03 | UINT | 20-3600 | 30 | RW | |
| 44 | PP_MAX_PASSES | PPP04 | UINT | 1-10000 | 1000 | RW | |
| 45 | PP_LINEAR_MARGIN | PPP05 | UINT | 1-3600 | 300 | RW | |
| 46 | PP_COMP_STEP | PPP06 | UINT | 1-1000 | 10 | RW | |

### Pressure Sensor

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 124 | PRESSURE_SENSOR_ENABLE | PRP01 | BOOL | - | false | RW | ENABLE_PRESSURE_SENSOR |
| 125 | PRESSURE_SENSOR_PERIODIC | PRP02 | UINT | 0-MAX | 0 | RW | ENABLE_PRESSURE_SENSOR |
| 146 | PRESSURE_SENSOR_LOGGING_MODE | PRP03 | PRESSURESENSORLOGGINGMODE | {ALWAYS=0, UW_THRESHOLD=1} | ALWAYS | RW | ENABLE_PRESSURE_SENSOR |
| 157 | PRESSURE_SENSOR_ENABLE_TX_MODE | PRP04 | SENSORENABLETXMODE | {OFF=0, ONESHOT=1, MEAN=2, MEDIAN=3} | OFF | RW | ENABLE_PRESSURE_SENSOR |
| 158 | PRESSURE_SENSOR_ENABLE_TX_MAX_SAMPLES | PRP05 | UINT | 1-MAX | 1 | RW | ENABLE_PRESSURE_SENSOR |
| 159 | PRESSURE_SENSOR_ENABLE_TX_SAMPLE_PERIOD | PRP06 | UINT | 1-MAX | 1000 | RW | ENABLE_PRESSURE_SENSOR |
| 177 | PRESSURE_SENSOR_FULL_SCALE | PRP07 | PRESSURESENSORFULLSCALE | {1260hPa=0, 4060hPa=1} | 0 (1260hPa) | RW | ENABLE_PRESSURE_SENSOR |

### Accelerometer

**Note:** The Argos TX encoding uses `AXL_SENSOR_MEASUREMENT_RANGE` as the offset for X/Y/Z axis values. The encoding formula is `(value_g + g_range) * 1000` where g_range is derived from this parameter (0=2g, 1=4g, 2=8g, 3=16g). The decoder (pylinkiot) must use the same g_range to decode: `value_g = (raw / 1000.0) - g_range`.

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 118 | AXL_SENSOR_ENABLE | AXP01 | BOOL | - | false | RW | ENABLE_AXL_SENSOR |
| 119 | AXL_SENSOR_PERIODIC | AXP02 | UINT | 0-MAX | 0 | RW | ENABLE_AXL_SENSOR |
| 120 | AXL_SENSOR_WAKEUP_THRESH | AXP03 | FLOAT | 0-8.0 | 0.0 | RW | ENABLE_AXL_SENSOR |
| 121 | AXL_SENSOR_WAKEUP_SAMPLES | AXP04 | UINT | 1-5 | 5 | RW | ENABLE_AXL_SENSOR |
| 122 | AXL_SENSOR_MEASUREMENT_RANGE | AXP08 | UINT | 0-4 | 0 | RW | ENABLE_AXL_SENSOR |
| 123 | AXL_SENSOR_POWER_MODE | AXP09 | UINT | 0-2 | 0 | RW | ENABLE_AXL_SENSOR |
| 160 | AXL_SENSOR_ENABLE_TX_MODE | AXP05 | SENSORENABLETXMODE | {0-3} | OFF | RW | ENABLE_AXL_SENSOR |
| 161 | AXL_SENSOR_ENABLE_TX_MAX_SAMPLES | AXP06 | UINT | 1-MAX | 1 | RW | ENABLE_AXL_SENSOR |
| 162 | AXL_SENSOR_ENABLE_TX_SAMPLE_PERIOD | AXP07 | UINT | 1-MAX | 1000 | RW | ENABLE_AXL_SENSOR |

### Thermistor

**Note:** The thermistor calibration offset (set via `CALCW,THERMISTOR,0,<offset>`) is loaded once at boot. A device reboot is required after changing the calibration offset for the new value to take effect.

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 112 | THERMISTOR_SENSOR_ENABLE | THP01 | BOOL | - | false | RW | ENABLE_THERMISTOR_SENSOR |
| 113 | THERMISTOR_SENSOR_PERIODIC | THP02 | UINT | 0-MAX | 0 | RW | ENABLE_THERMISTOR_SENSOR |
| 114 | THERMISTOR_SENSOR_VALUE | THP03 | FLOAT | - | 0.0 | R | ENABLE_THERMISTOR_SENSOR |
| 115 | THERMISTOR_SENSOR_WAKEUP_THRESH | THP04 | FLOAT | 0-MAX | 0.0 | RW | ENABLE_THERMISTOR_SENSOR |
| 116 | THERMISTOR_SENSOR_WAKEUP_SAMPLES | THP05 | UINT | 0-MAX | 0 | RW | ENABLE_THERMISTOR_SENSOR |
| 163 | THERMISTOR_SENSOR_ENABLE_TX_MODE | THP06 | SENSORENABLETXMODE | {0-3} | OFF | RW | ENABLE_THERMISTOR_SENSOR |
| 164 | THERMISTOR_SENSOR_ENABLE_TX_MAX_SAMPLES | THP07 | UINT | 1-MAX | 1 | RW | ENABLE_THERMISTOR_SENSOR |
| 165 | THERMISTOR_SENSOR_ENABLE_TX_SAMPLE_PERIOD | THP08 | UINT | 1-MAX | 1000 | RW | ENABLE_THERMISTOR_SENSOR |

### Battery & Power (Read-only)

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 6 | BATT_SOC | POT03 | UINT | 0-100 | auto | R | |
| 87 | BATT_VOLTAGE | POT06 | FLOAT | 0-12 | auto | R | |
| 7 | LAST_FULL_CHARGE_DATE | POT05 | DATE | - | auto | R | |

### Power Management

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 88 | SHUTDOWN_TIMER | PWP01 | UINT | 0-86400 | 0 | RW | EXTERNAL_WAKEUP |
| 89 | BOOT_COUNTER | PWP02 | UINT | 0-MAX | 0 | R | EXTERNAL_WAKEUP |
| 90 | BOOT_COUNTER_MODULO | PWP03 | UINT | 2-1000 | 2 | RW | EXTERNAL_WAKEUP |
| 91 | WAKEUP_PERIOD | PWP04 | UINT | 0-86400 | 6300 | R | EXTERNAL_WAKEUP |
| 174 | SHUTDOWN_NTIME_SAT | PWP05 | UINT | 0-65535 | 0 | RW | EXTERNAL_WAKEUP |

### LED

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 55 | LED_MODE | LDP01 | LEDMODE | {OFF=0, HRS_24=1, ALWAYS=3} | HRS_24 | RW | |
| 117 | EXT_LED_MODE | LDP02 | LEDMODE | {OFF=0, HRS_24=1, ALWAYS=3} | ALWAYS | RW | |

### Certification Test

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 82 | CERT_TX_ENABLE | CTP01 | BOOL | - | false | RW | |
| 83 | CERT_TX_PAYLOAD | CTP02 | TEXT | - | 27 bytes FF | RW | |
| 84 | CERT_TX_MODULATION | CTP03 | MODULATION | {LDK=0, LDA2=1, VLDA4=2} | LDA2 | RW | |
| 85 | CERT_TX_REPETITION | CTP04 | UINT | 2-MAX | 60 | RW | |

### Debug

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 126 | DEBUG_OUTPUT_MODE | DBP01 | DEBUGMODE | {UART=0, USB_CDC=1, BLE_NUS=2} | USB_CDC | RW | |

### SMD Credentials

| ID | Name | DTE Key | Type | Range | Default | RW | Enable Flag |
|----|------|---------|------|-------|---------|----|-------------|
| 172 | ARGOS_SECKEY | IDP13 | TEXT | - | "" | RW | ARGOS_SMD |
| 173 | ARGOS_RADIOCONF | IDP14 | TEXT | - | "" | RW | ARGOS_SMD |
