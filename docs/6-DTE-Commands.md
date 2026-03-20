The DTE (Data Terminal Equipment) protocol is the primary interface for configuring and querying the tracker. It is used by **pylinkit**, the **LinkIt GUI tool**, and any BLE/UART terminal to communicate with the firmware.

# Protocol Framing

## Request Format

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

## Response Format

**Success:**
```
$O;COMMAND#LEN;PAYLOAD\r
```

**Error:**
```
$N;COMMAND#LEN;ERROR_CODE\r
```

## Error Codes

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

# Command Summary

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

# Parameter Key Naming Convention

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

# Parameter Commands

## PARML -- List All Parameter Keys

```
$PARML#000;\r
→ $O;PARML#LEN;IDP12,IDT06,IDT02,IDT03,ART01,ART02,...\r
```

Returns all implemented DTE keys. The exact set depends on the build configuration.

## PARMR -- Read Parameters

Read one or more values by key, or all "P"-type parameters when called empty.

```
$PARMR#000;\r                      # Read all configurable parameters
$PARMR#00F;GNP01,ARP05,ARP01\r     # Read specific parameters
→ $O;PARMR#01B;GNP01=1,ARP05=60,ARP01=2\r
```

## PARMW -- Write Parameters

Write one or more key=value pairs. Read-only parameters are silently skipped. Triggers `CONFIG_UPDATED` after saving.

```
$PARMW#011;GNP01=1,ARP05=120\r
→ $O;PARMW#000;\r
```

Error 4 (VALUE_OUT_OF_RANGE) if a value exceeds the parameter's allowed range.

### Commonly Used Parameters

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

## STATR -- Read Status/Telemetry

Identical to PARMR but filters for "T"-type keys when called empty.

```
$STATR#000;\r                  # Read all telemetry
$STATR#00A;ART02,IDT03\r       # Read specific keys
→ $O;STATR#013;ART02=0,IDT03=V0.1\r
```

### Default Telemetry Keys

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

# Device Commands

## PROFR -- Read Profile Name

```
$PROFR#000;\r
→ $O;PROFR#018;Profile Name For Tracker\r
```

## PROFW -- Write Profile Name

Max 128 characters.

```
$PROFW#018;Profile Name For Tracker\r
→ $O;PROFW#000;\r
```

## FACTW -- Factory Reset

Restore all parameters to factory defaults. Device reboots after response.

```
$FACTW#000;\r
→ $O;FACTW#000;\r
```

## RSTBW -- Reboot Device

```
$RSTBW#000;\r
→ $O;RSTBW#000;\r
```

## RSTVW -- Reset Variable

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

## SECUR -- Security Access

Authenticate for privileged operations (OTA, etc.). Accepts code `0x12345678` or the device's `ARGOS_DECID`.

```
$SECUR#008;12345678\r
→ $O;SECUR#000;\r
```

## RTCW -- Set RTC

Set the device RTC to a Unix timestamp. On RSPB builds, also updates `LAST_KNOWN_RTC` (PWP06) for the pseudo-RTC chain.

```
$RTCW#00A;1708444800\r
→ $O;RTCW#000;\r
```

Read back via: `$STATR#005;SYT01\r`

## DUMPM -- Dump Memory

Read raw memory as base64. Max length: `0x500`.

```
$DUMPM#007;100,200\r
→ $O;DUMPM#LEN;<base64_data>\r
```

## DUMPD -- Dump Log Data

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
| B | SWS (saltwater switch log) |
| C | Mortality (RSPB) |

```
$DUMPD#001;1\r
→ $O;DUMPD#LEN;000,002,<base64_csv_data>\r
```

Response: `mmm` (page index), `MMM` (max page index), `data` (base64 CSV). Max 8 entries per page.

## ERASE -- Erase Log Data

| log_type | Target |
|----------|--------|
| 1 | GNSS |
| 2 | System |
| 3 | ALL |
| 4-12 | ALS, PH, RTD, CDT, Camera, AXL, Pressure, Thermistor, TSYS01 |
| 13 | SWS |
| 14 | Mortality |

```
$ERASE#001;3\r
→ $O;ERASE#000;\r
```

---

# Argos/Satellite Commands

## PASPW -- Write Pass Prediction Data

Upload base64-encoded AOP satellite pass prediction data. Updates `ARGOS_AOP_DATE` (ART03).

```
$PASPW#LEN;<base64_encoded_prepass_data>\r
→ $O;PASPW#000;\r
```

Verify: `$PARMR#005;ART03\r`

## SATTX -- Manual Argos Transmit

Trigger a test transmission (async response after TX completes).

Args: `modulation` (0=LDK, 1=LDA2, 2=VLDA4), `power` (mW), `freq` (MHz), `size` (bytes), `tcxo_warmup` (seconds, optional).

```
$SATTX#01A;1,500,401.630000,4,5\r
→ $O;SATTX#000;\r
```

## SATDP -- Satellite Doppler Calibration

Start periodic Doppler TX at `TR_NOM` interval. Runs until device reset.

```
$SATDP#000;\r
→ $O;SATDP#000;\r
```

## SMDDFU -- SMD Satellite DFU

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

## SMDTST -- SMD SPI Test

Run full SPI diagnostic against the SMD module.

```
$SMDTST#000;\r
→ $O;SMDTST#LEN;<test_results_text>\r
```

## SMDCD -- SMD Credentials Write

Write credentials to the SMD module and save to config store.

Args: `id` (uint), `addr` (hex), `seckey` (text), `radioconf` (text).

```
$SMDCD#LEN;12345,ABCDEF01,0123456789ABCDEF,<radioconf>\r
→ $O;SMDCD#000;\r
```

Updates: `ARGOS_DECID` (IDP12), `ARGOS_HEXID` (IDT06), `ARGOS_SECKEY` (IDP13), `ARGOS_RADIOCONF` (IDP14).

---

# Sensor Commands

## SENSR -- Read Sensor Values

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

## SCALW -- Write Sensor Calibration

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

## SCALR -- Read Sensor Calibration

Args: `sensor_id` (0-7), `offset`.

```
$SCALR#003;1,0\r
→ $O;SCALR#00A;1013.250000\r
```

## PWRON -- Power On/Off Components

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

## SWSST -- SWS Calibration Status

Read the Salt Water Switch analog state. Only on builds with `SWS_ADC`.

```
$SWSST#000;\r
→ $O;SWSST#LEN;2048,3500,2800,50,2750,2760,1,0,300\r
```

Response fields (9): air, water, threshold, hysteresis, raw_adc, filtered_adc, calibrated, underwater, time_in_state.

## SWSTST -- SWS Test Mode

Start (1) or stop (0) SWS test with LED feedback (Blue=underwater, Yellow=surface).

```
$SWSTST#001;1\r
→ $O;SWSTST#001;1\r
```

## GNSSI -- GNSS Device Info

Query u-blox M10Q hardware ID and firmware version. Powers on GNSS automatically if not cached.

```
$GNSSI#000;\r
→ $O;GNSSI#02B;01A2B3C4D5,SPG 4.04 (7b202e),00190000\r
```

Response: unique_id, sw_version, hw_version.

## GNSSA -- GNSS Almanac Status

Check AssistNow Offline almanac file status.

```
$GNSSA#000;\r
→ $O;GNSSA#00D;1,32768,35,28,0\r
```

Response: present, file_size, total_records, valid_records, stale.

---

# LoRa/Bridge Commands

## LORATX -- LoRa Transmission / Bridge Mode

Manual LoRa TX or USB-to-RAK3172 AT command passthrough. Only on `LORA_RAK3172=ON` builds.

```
$LORATX#000;\r
```

In bridge mode, RAK3172 RUI3 AT commands can be sent directly (e.g., `AT+DEVEUI=?`, `AT+JOIN`).

## GNSSBR -- GNSS Bridge Mode

USB-to-GNSS UART passthrough for u-blox configuration via u-center or manual AssistNow loading.

```
$GNSSBR#000;\r
```

Reset the device to exit bridge mode.

---

# Parameter Reference

For the complete table of all configurable parameters (keys, types, ranges, defaults, and detailed descriptions), see [Parameters Definition](9-Parameters.md).

