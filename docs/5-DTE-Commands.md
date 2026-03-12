# 5 - DTE Commands Reference

The DTE (Data Terminal Equipment) protocol is the primary interface for configuring and querying the tracker. It is used by **pylinkit**, the **LinkIt GUI tool**, and any BLE/UART terminal to communicate with the firmware.

## Protocol Framing

### Request Format

```
$COMMAND#LEN;PAYLOAD\r
```

| Field | Description |
|-------|-------------|
| `$` | Start delimiter |
| `COMMAND` | Command name (4-6 uppercase characters) |
| `#` | Length separator |
| `LEN` | Payload length in hexadecimal (3 digits) |
| `;` | Payload separator |
| `PAYLOAD` | Command-specific payload (comma-separated) |
| `\r` | Carriage return terminator |

### Response Format

**Success:**
```
$O;COMMAND#LEN;PAYLOAD\r
```

**Error:**
```
$N;COMMAND#LEN;ERROR_CODE\r
```

### Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | OK | Success |
| 1 | INCORRECT_COMMAND | Unknown command |
| 2 | INCORRECT_DATA | Invalid data |
| 3 | MISSING_ARGUMENT | Required argument missing |
| 4 | VALUE_OUT_OF_RANGE | Value exceeds allowed range |
| 5 | MESSAGE_TOO_LARGE | Payload exceeds max length (0xFFF) |
| 6 | PARAM_KEY_UNRECOGNISED | Unknown parameter key |
| 7 | BAD_FORMAT | Malformed request |
| 8 | DATA_LENGTH_MISMATCH | Payload length doesn't match #LEN |
| 9 | UNEXPECTED_ARGUMENT | Too many arguments |
| 10 | INVALID_ACCESS_CODE | Wrong SECUR access code |

---

## Command Summary

| Command | Description |
|---------|-------------|
| **Parameter Commands** | |
| [PARML](#parml----list-all-parameter-keys) | List all parameter keys |
| [PARMR](#parmr----read-parameters) | Read parameters by key |
| [PARMW](#parmw----write-parameters) | Write parameters by key |
| [STATR](#statr----read-statustelemetry) | Read status/telemetry parameters |
| **Device Commands** | |
| [PROFR](#profr----read-profile-name) | Read profile name |
| [PROFW](#profw----write-profile-name) | Write profile name |
| [FACTW](#factw----factory-reset) | Factory reset |
| [RSTBW](#rstbw----reboot-device) | Reboot device |
| [RSTVW](#rstvw----reset-variable) | Reset variable (TX/RX counters) |
| [SECUR](#secur----security-access) | Security access authentication |
| [RTCW](#rtcw----set-rtc) | Set device RTC |
| [DUMPM](#dumpm----dump-memory) | Dump raw memory region |
| [DUMPD](#dumpd----dump-log-data) | Dump sensor log data (CSV) |
| [ERASE](#erase----erase-log-data) | Erase sensor log data |
| **Argos/Satellite Commands** | |
| [PASPW](#paspw----write-pass-prediction-data) | Write pass prediction (AOP) data |
| [SATTX](#sattx----manual-argos-transmit) | Manual Argos satellite transmission |
| [SATDP](#satdp----satellite-doppler-calibration) | Start periodic Doppler TX calibration |
| [SMDDFU](#smddfu----smd-satellite-dfu) | SMD module firmware update |
| [SMDTST](#smdtst----smd-spi-test) | SMD SPI connectivity test |
| [SMDCD](#smdcd----smd-credentials-write) | Write SMD credentials |
| **Sensor Commands** | |
| [SENSR](#sensr----read-sensor-values) | Read live sensor values |
| [SCALW](#scalw----write-sensor-calibration) | Write sensor calibration |
| [SCALR](#scalr----read-sensor-calibration) | Read sensor calibration |
| [PWRON](#pwron----power-onoff-components) | Power on/off components |
| [SWSST](#swsst----sws-calibration-status) | Read SWS calibration status |
| [SWSTST](#swstst----sws-test-mode) | Start/stop SWS test mode |
| [GNSSI](#gnssi----gnss-device-info) | Read GNSS module info |
| [GNSSA](#gnssa----gnss-almanac-status) | Read GNSS almanac status |
| **LoRa/Bridge Commands** | |
| [LORATX](#loratx----lora-transmission--bridge-mode) | Manual LoRa TX / bridge mode |
| [GNSSBR](#gnssbr----gnss-bridge-mode) | GNSS bridge mode (USB passthrough) |

---

## Parameter Key Naming Convention

Parameter keys are 5 characters: 3-letter prefix + 2-digit number. The third character indicates the type:

- **P** -- Configurable parameter (read/write). Returned by `PARMR` with no arguments.
- **T** -- Telemetry / status value (read-only). Returned by `STATR` with no arguments.

| Prefix | Group | Description |
|--------|-------|-------------|
| `ARP` | Argos Params | Argos TX configuration |
| `ART` | Argos Telemetry | Argos counters and timestamps |
| `GNP` | GNSS Params | GNSS acquisition and filtering |
| `UNP` | Underwater Params | Underwater detection and sampling |
| `LBP` | Low Battery Params | Low battery mode configuration |
| `ZOP` | Zone Params | Geofencing zone configuration |
| `PPP` | Pass Predict Params | Prepass algorithm settings |
| `PRP` | Pressure Params | Pressure sensor configuration |
| `AXP` | Accelerometer Params | Accelerometer configuration |
| `THP` | Thermistor Params | Thermistor sensor configuration |
| `STP` | Sea Temp Params | Sea temperature sensor |
| `PHP` | pH Params | pH sensor configuration |
| `LTP` | Light Params | Ambient light sensor |
| `CDP` | CDT Params | Conductivity-Depth-Temperature |
| `CAP` | Camera Params | Camera trigger configuration |
| `CTP` | Certification Params | TX certification test mode |
| `LDP` | LED Params | LED mode configuration |
| `PWP` | Power Params | TPL5111 wakeup management |
| `DBP` | Debug Params | Debug output configuration |
| `IDT` | Identity Telemetry | Device model, HW/FW version |
| `IDP` | Identity Params | Device IDs, profile name |
| `POT` | Power Telemetry | Battery SOC, voltage |
| `SYT` | System Telemetry | System status (RTC, etc.) |
| `LRP` | LoRa Params | LoRa RAK3172 configuration |
| `LRT` | LoRa Telemetry | LoRa counters and status |

---

## Parameter Commands

### PARML -- List All Parameter Keys

```
$PARML#000;\r
→ $O;PARML#LEN;IDP12,IDT06,IDT02,IDT03,ART01,ART02,...\r
```

Returns all implemented DTE keys. The exact set depends on the build configuration.

### PARMR -- Read Parameters

Read one or more values by key, or all "P"-type parameters when called empty.

```
$PARMR#000;\r                      # Read all configurable parameters
$PARMR#00F;GNP01,ARP05,ARP01\r     # Read specific parameters
→ $O;PARMR#01B;GNP01=1,ARP05=60,ARP01=2\r
```

### PARMW -- Write Parameters

Write one or more key=value pairs. Read-only parameters are silently skipped. Triggers `CONFIG_UPDATED` after saving.

```
$PARMW#011;GNP01=1,ARP05=120\r
→ $O;PARMW#000;\r
```

Error 4 (VALUE_OUT_OF_RANGE) if a value exceeds the parameter's allowed range.

#### Commonly Used Parameters

| Key | Name | Type | Range | Description |
|-----|------|------|-------|-------------|
| IDP12 | ARGOS_DECID | UINT | 0-4294967295 | Argos decimal ID |
| IDT06 | ARGOS_HEXID | HEX | 0-FFFFFFFF | Argos hex ID |
| IDP11 | PROFILE_NAME | TEXT | - | Profile name |
| ARP05 | TR_NOM | UINT | 30-1200 | TX repetition (seconds) |
| ARP01 | ARGOS_MODE | ENUM | 0-4 | Argos operating mode |
| GNP01 | GNSS_EN | BOOL | 0/1 | GNSS enable |
| GNP05 | GNSS_ACQ_TIMEOUT | UINT | 10-600 | GNSS acquisition timeout (s) |
| UNP01 | UNDERWATER_EN | BOOL | 0/1 | Underwater detection enable |
| LBP01 | LB_EN | BOOL | 0/1 | Low battery mode enable |
| LBP02 | LB_THRESHOLD | UINT | 0-100 | Low battery threshold (%) |
| DBP01 | DEBUG_OUTPUT_MODE | ENUM | 0-2 | 0=UART, 1=USB_CDC, 2=BLE_NUS |

### STATR -- Read Status/Telemetry

Identical to PARMR but filters for "T"-type keys when called empty.

```
$STATR#000;\r                  # Read all telemetry
$STATR#00A;ART02,IDT03\r       # Read specific keys
→ $O;STATR#013;ART02=0,IDT03=V0.1\r
```

#### Default Telemetry Keys

| Key | Name | Description |
|-----|------|-------------|
| IDT02 | DEVICE_MODEL | Device model |
| IDT03 | FW_APP_VERSION | Firmware version |
| IDT04 | HW_VERSION | Hardware version |
| IDT10 | DEVICE_DECID | Device decimal ID |
| ART01 | LAST_TX | Last transmission timestamp |
| ART02 | TX_COUNTER | Transmit counter |
| ART03 | ARGOS_AOP_DATE | AOP bulletin date |
| POT03 | BATT_SOC | Battery state of charge (%) |
| POT06 | BATT_VOLTAGE | Battery voltage (V) |
| SYT01 | RTC_CURRENT_TIME | Live RTC value (Unix timestamp) |

---

## Device Commands

### PROFR -- Read Profile Name

```
$PROFR#000;\r
→ $O;PROFR#018;Profile Name For Tracker\r
```

### PROFW -- Write Profile Name

Max 128 characters.

```
$PROFW#018;Profile Name For Tracker\r
→ $O;PROFW#000;\r
```

### FACTW -- Factory Reset

Restore all parameters to factory defaults. Device reboots after response.

```
$FACTW#000;\r
→ $O;FACTW#000;\r
```

### RSTBW -- Reboot Device

```
$RSTBW#000;\r
→ $O;RSTBW#000;\r
```

### RSTVW -- Reset Variable

Reset a runtime counter to zero.

| Index | Variable |
|-------|----------|
| 1 | TX_COUNTER |
| 2 | BOOT_COUNTER (EXTERNAL_WAKEUP builds only) |
| 3 | ARGOS_RX_COUNTER |
| 4 | ARGOS_RX_TIME |

```
$RSTVW#001;1\r
→ $O;RSTVW#000;\r
```

### SECUR -- Security Access

Authenticate for privileged operations (OTA, etc.). Accepts code `0x12345678` or the device's `ARGOS_DECID`.

```
$SECUR#008;12345678\r
→ $O;SECUR#000;\r
```

### RTCW -- Set RTC

Set the device RTC to a Unix timestamp. On RSPB builds, also updates `LAST_KNOWN_RTC` (PWP06) for the pseudo-RTC chain.

```
$RTCW#00A;1708444800\r
→ $O;RTCW#000;\r
```

Read back via: `$STATR#005;SYT01\r`

### DUMPM -- Dump Memory

Read raw memory as base64. Max length: `0x500`.

```
$DUMPM#007;100,200\r
→ $O;DUMPM#LEN;<base64_data>\r
```

### DUMPD -- Dump Log Data

Paginated CSV dump of sensor logs. Call repeatedly until `mmm == MMM`.

| d_type | Log Type |
|--------|----------|
| 0 | System (system.log) |
| 1 | GNSS |
| 2 | ALS |
| 3 | PH |
| 4 | RTD |
| 5 | CDT |
| 6 | Camera |
| 7 | AXL (Accelerometer) |
| 8 | Pressure |
| 9 | Thermistor |
| A | TSYS01 |

```
$DUMPD#001;1\r
→ $O;DUMPD#LEN;000,002,<base64_csv_data>\r
```

Response: `mmm` (page index), `MMM` (max page index), `data` (base64 CSV). Max 8 entries per page.

### ERASE -- Erase Log Data

| log_type | Target |
|----------|--------|
| 1 | GNSS |
| 2 | System |
| 3 | ALL |
| 4-12 | ALS, PH, RTD, CDT, Camera, AXL, Pressure, Thermistor, TSYS01 |

```
$ERASE#001;3\r
→ $O;ERASE#000;\r
```

---

## Argos/Satellite Commands

### PASPW -- Write Pass Prediction Data

Upload base64-encoded AOP satellite pass prediction data. Updates `ARGOS_AOP_DATE` (ART03).

```
$PASPW#LEN;<base64_encoded_prepass_data>\r
→ $O;PASPW#000;\r
```

Verify: `$PARMR#005;ART03\r`

### SATTX -- Manual Argos Transmit

Trigger a test transmission (async response after TX completes).

Args: `modulation` (0=LDK, 1=LDA2, 2=VLDA4), `power` (mW), `freq` (MHz), `size` (bytes), `tcxo_warmup` (seconds, optional).

```
$SATTX#01A;1,500,401.630000,4,5\r
→ $O;SATTX#000;\r
```

### SATDP -- Satellite Doppler Calibration

Start periodic Doppler TX at `TR_NOM` interval. Runs until device reset.

```
$SATDP#000;\r
→ $O;SATDP#000;\r
```

### SMDDFU -- SMD Satellite DFU

Manage SMD module firmware updates over SPI. Only on `ARGOS_SMD=ON` builds.

| Action | Value | Description |
|--------|-------|-------------|
| ENTER | 0 | Enter DFU (bootloader) mode |
| EXIT | 1 | Exit DFU mode |
| STATUS | 2 | Query DFU mode state |
| UPDATE | 3 | Trigger firmware update |
| INFO | 4 | Get bootloader info (DFU mode only) |
| VERSION | 5 | Get firmware version (app mode only) |

```
$SMDDFU#001;5\r
→ $O;SMDDFU#LEN;0,0,0,v1.2.3\r
```

Response: `status` (0=OK), `dfu_mode` (0/1), `progress` (0-100%), `info` (text).

### SMDTST -- SMD SPI Test

Run full SPI diagnostic against the SMD module.

```
$SMDTST#000;\r
→ $O;SMDTST#LEN;<test_results_text>\r
```

### SMDCD -- SMD Credentials Write

Write credentials to the SMD module and save to config store.

Args: `id` (uint), `addr` (hex), `seckey` (text), `radioconf` (text).

```
$SMDCD#LEN;12345,ABCDEF01,0123456789ABCDEF,<radioconf>\r
→ $O;SMDCD#000;\r
```

Updates: `ARGOS_DECID` (IDP12), `ARGOS_HEXID` (IDT06), `ARGOS_SECKEY` (IDP13), `ARGOS_RADIOCONF` (IDP14).

---

## Sensor Commands

### SENSR -- Read Sensor Values

Live sensor reading. Bitmask selects sensors.

| Bit | Value | Sensor |
|-----|-------|--------|
| 0 | 1 | Battery (voltage + SOC) |
| 1 | 2 | Pressure (pressure, temperature, altitude) |
| 2 | 4 | GNSS (lat, lon, HDOP, satellites) |
| 3 | 8 | Accelerometer (X, Y, Z, temperature, activity) |
| 4 | 16 | Thermistor (temperature) |

Use 31 for all sensors.

```
$SENSR#004;1,10\r
→ $O;SENSR#LEN;3700,80,0.0,0.0,0.0,0.0,0.0,99.9,0,0.0,0.0,0.0,0.0,0\r
```

Response fields (15): batt_mv, batt_soc, pressure, temperature, altitude, lat, lon, hdop, num_sv, accel_x, accel_y, accel_z, accel_temp, activity, thermistor_temp.

### SCALW -- Write Sensor Calibration

Args: `sensor_id` (0-7), `offset`, `value` (float).

| ID | Sensor |
|----|--------|
| 0 | AXL (Accelerometer) |
| 1 | PRS (Pressure) |
| 2 | ALS (Ambient Light) |
| 3 | PH |
| 4 | RTD (Sea Temperature) |
| 5 | CDT |
| 6 | MCP47X6 (DAC) |
| 7 | THERMISTOR |

```
$SCALW#00D;1,0,1013.250000\r
→ $O;SCALW#000;\r
```

### SCALR -- Read Sensor Calibration

Args: `sensor_id` (0-7), `offset`.

```
$SCALR#003;1,0\r
→ $O;SCALR#00A;1013.250000\r
```

### PWRON -- Power On/Off Components

| Value | Component |
|-------|-----------|
| 0 | ALL (GNSS + sensors + satellite) |
| 1 | GNSS (also enables sensor power rail) |
| 2 | SENSORS (power rail only) |
| 3 | SATELLITE (also enables sensor power rail) |
| 4 | OFF (all components) |

```
$PWRON#001;3\r
→ $O;PWRON#000;\r
```

### SWSST -- SWS Calibration Status

Read the Salt Water Switch analog state. Only on builds with `SWS_ADC`.

```
$SWSST#000;\r
→ $O;SWSST#LEN;2048,3500,2800,50,2750,2760,1,0,300\r
```

Response fields (9): air, water, threshold, hysteresis, raw_adc, filtered_adc, calibrated, underwater, time_in_state.

### SWSTST -- SWS Test Mode

Start (1) or stop (0) SWS test with LED feedback (Blue=underwater, Yellow=surface).

```
$SWSTST#001;1\r
→ $O;SWSTST#001;1\r
```

### GNSSI -- GNSS Device Info

Query u-blox M10Q hardware ID and firmware version. Powers on GNSS automatically if not cached.

```
$GNSSI#000;\r
→ $O;GNSSI#02B;01A2B3C4D5,SPG 4.04 (7b202e),00190000\r
```

Response: unique_id, sw_version, hw_version.

### GNSSA -- GNSS Almanac Status

Check AssistNow Offline almanac file status.

```
$GNSSA#000;\r
→ $O;GNSSA#00D;1,32768,35,28,0\r
```

Response: present, file_size, total_records, valid_records, stale.

---

## LoRa/Bridge Commands

### LORATX -- LoRa Transmission / Bridge Mode

Manual LoRa TX or USB-to-RAK3172 AT command passthrough. Only on `LORA_RAK3172=ON` builds.

```
$LORATX#000;\r
```

In bridge mode, RAK3172 RUI3 AT commands can be sent directly (e.g., `AT+DEVEUI=?`, `AT+JOIN`).

### GNSSBR -- GNSS Bridge Mode

USB-to-GNSS UART passthrough for u-blox configuration via u-center or manual AssistNow loading.

```
$GNSSBR#000;\r
```

Reset the device to exit bridge mode.

---

## Parameter Reference

Complete table of all configurable parameters, grouped by function.

### Identity & Device Info

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| ARGOS_DECID | IDP12 | UINT | 0-4294967295 | 0 | RW |
| ARGOS_HEXID | IDT06 | HEX | 0-FFFFFFFF | 0 | RW |
| DEVICE_MODEL | IDT02 | TEXT | - | board name | R |
| FW_APP_VERSION | IDT03 | TEXT | - | auto | R |
| HW_VERSION | IDT04 | TEXT | - | auto | R |
| DEVICE_DECID | IDT10 | UINT | - | auto | R |
| PROFILE_NAME | IDP11 | TEXT | - | "FACTORY" | RW |

### Argos TX Configuration

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| ARGOS_MODE | ARP01 | ARGOSMODE | OFF=0, PP=1, LEGACY=2, DUTY=3, DOPPLER=4 | LEGACY (SB: PP) | RW |
| TR_NOM | ARP05 | UINT | 30-1200 | 60 | RW |
| NTRY_PER_MESSAGE | ARP19 | UINT | 0-86400 | 0 (SB: 6) | RW |
| DUTY_CYCLE | ARP18 | UINT | 0-16777215 | 0 | RW |
| ARGOS_DEPTH_PILE | ARP16 | DEPTHPILE | 1,2,3,4,8,12,16,20,24 | 16 (UW: 1) | RW |
| DLOC_ARG_NOM | ARP11 | AQPERIOD | 0-8 | 600 (SB: 3600) | RW |
| ARGOS_TIME_SYNC_BURST_EN | ARP30 | BOOL | - | true | RW |
| ARGOS_TX_JITTER_EN | ARP31 | BOOL | - | true | RW |
| ARGOS_TCXO_WARMUP_TIME | ARP35 | UINT | 0-30 | 5 | RW |

### Argos RX Configuration

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| ARGOS_RX_EN | ARP32 | BOOL | - | true | RW |
| ARGOS_RX_MAX_WINDOW | ARP33 | UINT | 1-MAX | 900 | RW |
| ARGOS_RX_AOP_UPDATE_PERIOD | ARP34 | UINT | 0-MAX | 90 | RW |

### Argos Telemetry (Read-only)

| Name | Key | Type | Description |
|------|-----|------|-------------|
| ARGOS_AOP_DATE | ART03 | DATE | AOP bulletin date |
| LAST_TX | ART01 | DATE | Last transmission timestamp |
| TX_COUNTER | ART02 | UINT | Transmit counter |
| ARGOS_RX_COUNTER | ART10 | UINT | Receive counter |
| ARGOS_RX_TIME | ART11 | UINT | Receive time |

### GNSS Configuration

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| GNSS_EN | GNP01 | BOOL | - | true | RW |
| GNSS_HDOPFILT_EN | GNP02 | BOOL | - | true | RW |
| GNSS_HDOPFILT_THR | GNP03 | UINT | 2-15 | 2 | RW |
| GNSS_ACQ_TIMEOUT | GNP05 | UINT | 10-600 | 120 (UW: 240) | RW |
| GNSS_COLD_ACQ_TIMEOUT | GNP09 | UINT | 10-600 | 530 | RW |
| GNSS_FIX_MODE | GNP10 | GNSSFIXMODE | 2D=1, 3D=2, AUTO=3 | AUTO | RW |
| GNSS_DYN_MODEL | GNP11 | GNSSDYNMODEL | 0, 2-10 | PORTABLE | RW |
| GNSS_HACCFILT_EN | GNP20 | BOOL | - | true | RW |
| GNSS_HACCFILT_THR | GNP21 | UINT | 0-MAX | 5 | RW |
| GNSS_MIN_NUM_FIXES | GNP22 | UINT | 1-MAX | 1 | RW |
| GNSS_COLD_START_RETRY_PERIOD | GNP23 | UINT | 1-MAX | 60 | RW |
| GNSS_ASSISTNOW_EN | GNP24 | BOOL | - | true | RW |
| GNSS_TRIGGER_ON_SURFACED | GNP25 | BOOL | - | true | RW |
| GNSS_TRIGGER_ON_AXL_WAKEUP | GNP26 | BOOL | - | false | RW |
| GNSS_ASSISTNOW_OFFLINE_EN | GNP27 | BOOL | - | false | RW |
| GNSS_TRIGGER_COLD_START_ON_SURFACED | GNP28 | BOOL | - | false | RW |
| GNSS_SESSION_SINGLE_FIX | GNP30 | BOOL | - | false | RW |
| GNSS_TOKEN | GNP31 | TEXT | - | "" | RW |

### Underwater Detection

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| UNDERWATER_EN | UNP01 | BOOL | - | false (UW: true) | RW |
| DRY_TIME_BEFORE_TX | UNP02 | UINT | 0-MAX | 0 | RW |
| SAMPLING_UNDER_FREQ | UNP03 | UINT | 1-MAX | 60 | RW |
| SAMPLING_SURF_FREQ | UNP04 | UINT | 1-MAX | 60 | RW |
| UW_MAX_SAMPLES | UNP05 | UINT | 1-MAX | 5 (UW: 10) | RW |
| UW_MIN_DRY_SAMPLES | UNP06 | UINT | 1-MAX | 1 (UW: 3) | RW |
| UW_SAMPLE_GAP | UNP07 | UINT | 1-MAX | 1000 | RW |
| UW_PIN_SAMPLE_DELAY | UNP08 | UINT | 1-MAX | 1 (UW: 10) | RW |
| UNDERWATER_DETECT_SOURCE | UNP10 | ENUM | SWS=0, PRESSURE=1, GNSS=2, SWS_GNSS=3 | SWS | RW |
| UNDERWATER_DETECT_THRESH | UNP11 | FLOAT | 0-MAX | 1.1 | RW |
| UW_DIVE_MODE_ENABLE | UNP12 | BOOL | - | false | RW |
| UW_DIVE_MODE_START_TIME | UNP13 | UINT | 0-MAX | 0 | RW |
| UW_GNSS_DRY_SAMPLING | UNP14 | UINT | 1-MAX | 14400 | RW |
| UW_GNSS_WET_SAMPLING | UNP15 | UINT | 1-MAX | 14400 | RW |
| UW_GNSS_MAX_SAMPLES | UNP16 | UINT | 1-MAX | 10 | RW |
| UW_GNSS_MIN_DRY_SAMPLES | UNP17 | UINT | 1-MAX | 1 | RW |
| UW_GNSS_DETECT_THRESH | UNP18 | UINT | 1-7 | 1 (UW: 2) | RW |
| UW_MAX_DIVE_TIME | UNP24 | UINT | 0-MAX | 7200 | RW |
| UW_MIN_SURFACE_TIME | UNP25 | UINT | 0-MAX | 10 | RW |

### SWS Analog

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| SWS_ANALOG_THRESHOLD_MIN | UNP20 | UINT | 50-4095 | 100 | RW |
| SWS_ANALOG_THRESHOLD_MAX | UNP21 | UINT | 50-4095 | 3000 | RW |
| SWS_ANALOG_HYSTERESIS | UNP22 | UINT | 0-50 | 10 | RW |
| SWS_ANALOG_CALIB_INTERVAL | UNP23 | UINT | 60-MAX | 3600 | RW |

### Low Battery Mode

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| LB_EN | LBP01 | BOOL | - | false | RW |
| LB_THRESHOLD | LBP02 | UINT | 0-100 | 10 | RW |
| LB_CRITICAL_THRESH | LBP12 | UINT | 0-100 | 5 | RW |
| TR_LB | ARP06 | UINT | 30-1200 | 240 | RW |
| LB_ARGOS_MODE | LBP04 | ARGOSMODE | 0-4 | LEGACY | RW |
| LB_ARGOS_DUTY_CYCLE | LBP05 | UINT | 0-16777215 | 0 | RW |
| LB_GNSS_EN | LBP06 | BOOL | - | true | RW |
| DLOC_ARG_LB | ARP12 | AQPERIOD | 0-8 | 3600 | RW |
| LB_GNSS_HDOPFILT_THR | LBP07 | UINT | 2-15 | 2 | RW |
| LB_ARGOS_DEPTH_PILE | LBP08 | DEPTHPILE | 1-4, 8-24 | 1 | RW |
| LB_GNSS_ACQ_TIMEOUT | LBP09 | UINT | 10-600 | 120 | RW |
| LB_GNSS_HACCFILT_THR | LBP10 | UINT | 0-MAX | 5 | RW |
| LB_NTRY_PER_MESSAGE | LBP11 | UINT | 0-86400 | 4 | RW |
| LB_SHUTDOWN_NTIME_SAT | LBP14 | UINT | 0-65535 | 0 | RW |

### Zone (Geofencing)

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| ZONE_TYPE | ZOP01 | ZONETYPE | CIRCLE=1 | CIRCLE | RW |
| ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE | ZOP04 | BOOL | - | false | RW |
| ZONE_ENABLE_ACTIVATION_DATE | ZOP05 | BOOL | - | true | RW |
| ZONE_ACTIVATION_DATE | ZOP06 | DATE | - | 01/01/2020 | RW |
| ZONE_ARGOS_DEPTH_PILE | ZOP08 | DEPTHPILE | 1-4, 8-24 | 1 | RW |
| ZONE_ARGOS_REPETITION_SECONDS | ZOP10 | UINT | 30-1200 | 240 (SB: 60) | RW |
| ZONE_ARGOS_MODE | ZOP11 | ARGOSMODE | 0-4 | LEGACY (SB: PP) | RW |
| ZONE_ARGOS_DUTY_CYCLE | ZOP12 | UINT | 0-16777215 | 16777215 | RW |
| ZONE_ARGOS_NTRY_PER_MESSAGE | ZOP13 | UINT | 0-86400 | 0 | RW |
| ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS | ZOP14 | AQPERIOD | 0-8 | 3600 | RW |
| ZONE_GNSS_HDOPFILT_THR | ZOP15 | UINT | 2-15 | 2 | RW |
| ZONE_GNSS_HACCFILT_THR | ZOP16 | UINT | 0-MAX | 5 | RW |
| ZONE_GNSS_ACQ_TIMEOUT | ZOP17 | UINT | 10-600 | 240 | RW |
| ZONE_CENTER_LONGITUDE | ZOP18 | FLOAT | -180 to 180 | -123.3925 | RW |
| ZONE_CENTER_LATITUDE | ZOP19 | FLOAT | -90 to 90 | -48.8752 | RW |
| ZONE_RADIUS | ZOP20 | UINT | 1-MAX | 1000 | RW |

### Pass Prediction

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| PP_MIN_ELEVATION | PPP01 | FLOAT | 0-90 | 15.0 | RW |
| PP_MAX_ELEVATION | PPP02 | FLOAT | 0-90 | 90.0 | RW |
| PP_MIN_DURATION | PPP03 | UINT | 20-3600 | 30 | RW |
| PP_MAX_PASSES | PPP04 | UINT | 1-10000 | 1000 | RW |
| PP_LINEAR_MARGIN | PPP05 | UINT | 1-3600 | 300 | RW |
| PP_COMP_STEP | PPP06 | UINT | 1-1000 | 10 | RW |

### Sensor Configuration

**Pressure** (requires `ENABLE_PRESSURE_SENSOR`):

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| PRESSURE_SENSOR_ENABLE | PRP01 | BOOL | - | false | RW |
| PRESSURE_SENSOR_PERIODIC | PRP02 | UINT | 0-MAX | 0 | RW |
| PRESSURE_SENSOR_LOGGING_MODE | PRP03 | ENUM | ALWAYS=0, UW_THRESHOLD=1 | ALWAYS | RW |
| PRESSURE_SENSOR_ENABLE_TX_MODE | PRP04 | ENUM | OFF=0, ONESHOT=1, MEAN=2, MEDIAN=3 | OFF | RW |
| PRESSURE_SENSOR_ENABLE_TX_MAX_SAMPLES | PRP05 | UINT | 1-MAX | 1 | RW |
| PRESSURE_SENSOR_ENABLE_TX_SAMPLE_PERIOD | PRP06 | UINT | 1-MAX | 1000 | RW |
| PRESSURE_SENSOR_FULL_SCALE | PRP07 | ENUM | 1260hPa=0, 4060hPa=1 | 0 | RW |

**Accelerometer** (requires `ENABLE_AXL_SENSOR`):

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| AXL_SENSOR_ENABLE | AXP01 | BOOL | - | false | RW |
| AXL_SENSOR_PERIODIC | AXP02 | UINT | 0-MAX | 0 | RW |
| AXL_SENSOR_WAKEUP_THRESH | AXP03 | FLOAT | 0-8.0 | 0.0 | RW |
| AXL_SENSOR_WAKEUP_SAMPLES | AXP04 | UINT | 1-5 | 5 | RW |
| AXL_SENSOR_MEASUREMENT_RANGE | AXP08 | UINT | 0-4 | 0 | RW |
| AXL_SENSOR_POWER_MODE | AXP09 | UINT | 0-2 | 0 | RW |
| AXL_SENSOR_ENABLE_TX_MODE | AXP05 | ENUM | OFF=0, ONESHOT=1, MEAN=2, MEDIAN=3 | OFF | RW |
| AXL_SENSOR_ENABLE_TX_MAX_SAMPLES | AXP06 | UINT | 1-MAX | 1 | RW |
| AXL_SENSOR_ENABLE_TX_SAMPLE_PERIOD | AXP07 | UINT | 1-MAX | 1000 | RW |

**Thermistor** (requires `ENABLE_THERMISTOR_SENSOR`):

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| THERMISTOR_SENSOR_ENABLE | THP01 | BOOL | - | false | RW |
| THERMISTOR_SENSOR_PERIODIC | THP02 | UINT | 0-MAX | 0 | RW |
| THERMISTOR_SENSOR_VALUE | THP03 | FLOAT | - | 0.0 | R |
| THERMISTOR_SENSOR_WAKEUP_THRESH | THP04 | FLOAT | 0-MAX | 0.0 | RW |
| THERMISTOR_SENSOR_WAKEUP_SAMPLES | THP05 | UINT | 0-MAX | 0 | RW |
| THERMISTOR_SENSOR_ENABLE_TX_MODE | THP06 | ENUM | OFF=0, ONESHOT=1, MEAN=2, MEDIAN=3 | OFF | RW |
| THERMISTOR_SENSOR_ENABLE_TX_MAX_SAMPLES | THP07 | UINT | 1-MAX | 1 | RW |
| THERMISTOR_SENSOR_ENABLE_TX_SAMPLE_PERIOD | THP08 | UINT | 1-MAX | 1000 | RW |

### Battery & Power (Read-only)

| Name | Key | Type | Description |
|------|-----|------|-------------|
| BATT_SOC | POT03 | UINT | Battery state of charge (%) |
| BATT_VOLTAGE | POT06 | FLOAT | Battery voltage (V) |
| LAST_FULL_CHARGE_DATE | POT05 | DATE | Last full charge timestamp |

### Power Management (RSPB / EXTERNAL_WAKEUP)

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| SHUTDOWN_TIMER | PWP01 | UINT | 0-86400 | 0 | RW |
| BOOT_COUNTER | PWP02 | UINT | 0-MAX | 0 | R |
| BOOT_COUNTER_MODULO | PWP03 | UINT | 2-1000 | 2 | RW |
| WAKEUP_PERIOD | PWP04 | UINT | 0-86400 | 6300 | R |
| SHUTDOWN_NTIME_SAT | PWP05 | UINT | 0-65535 | 0 | RW |
| LAST_KNOWN_RTC | PWP06 | UINT | 0-MAX | 0 | R |

### LED

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| LED_MODE | LDP01 | LEDMODE | OFF=0, HRS_24=1, ALWAYS=3 | HRS_24 | RW |
| EXT_LED_MODE | LDP02 | LEDMODE | OFF=0, HRS_24=1, ALWAYS=3 | ALWAYS | RW |

### Certification Test

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| CERT_TX_ENABLE | CTP01 | BOOL | - | false | RW |
| CERT_TX_PAYLOAD | CTP02 | TEXT | - | 27 bytes FF | RW |
| CERT_TX_MODULATION | CTP03 | ENUM | LDK=0, LDA2=1, VLDA4=2 | LDA2 | RW |
| CERT_TX_REPETITION | CTP04 | UINT | 2-MAX | 60 | RW |

### Debug

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| DEBUG_OUTPUT_MODE | DBP01 | DEBUGMODE | UART=0, USB_CDC=1, BLE_NUS=2 | USB_CDC | RW |

### SMD Credentials (ARGOS_SMD builds)

| Name | Key | Type | Default | RW |
|------|-----|------|---------|----|
| ARGOS_SECKEY | IDP13 | TEXT | "" | RW |
| ARGOS_RADIOCONF | IDP14 | TEXT | "" | RW |

### LoRa RAK3172 Configuration (LORA_RAK3172 builds)

| Name | Key | Type | Range | Default | RW |
|------|-----|------|-------|---------|----|
| LORA_DEVEUI | LRP01 | TEXT | 16 hex | "" | R |
| LORA_APPEUI | LRP02 | TEXT | 16 hex | "" | RW |
| LORA_APPKEY | LRP03 | TEXT | 32 hex | "" | RW |
| LORA_DEVADDR | LRP04 | TEXT | 8 hex | "" | RW |
| LORA_APPSKEY | LRP05 | TEXT | 32 hex | "" | RW |
| LORA_NWKSKEY | LRP06 | TEXT | 32 hex | "" | RW |
| LORA_NJM | LRP07 | UINT | 0-1 | 1 | RW |
| LORA_BAND | LRP08 | UINT | 0-12 | 4 (EU868) | RW |
| LORA_CLASS | LRP09 | UINT | 0-2 | 0 (A) | RW |
| LORA_DR | LRP10 | UINT | 0-15 | 3 (SF9) | RW |
| LORA_ADR | LRP11 | BOOL | - | false | RW |
| LORA_TXP | LRP12 | UINT | 0-14 | 0 | RW |
| LORA_CFM | LRP13 | BOOL | - | false | RW |
| LORA_FPORT | LRP14 | UINT | 1-223 | 2 | RW |
| LORA_LP_MODE | LRP15 | UINT | 0-1 | 1 (standby) | RW |

### System Status (Read-only via STATR)

| Name | Key | Type | Description |
|------|-----|------|-------------|
| RTC_CURRENT_TIME | SYT01 | UINT | Live RTC value (Unix timestamp) |
