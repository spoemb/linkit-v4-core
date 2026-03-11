# Sensor Commands

DTE commands for reading live sensor data, calibrating sensors, controlling power rails, and querying SWS analog status.

## Protocol Format

```
Request:  $CMD#LEN;PAYLOAD\r
Success:  $O;CMD#LEN;PAYLOAD\r
Error:    $N;CMD#LEN;ERROR_CODE\r
```

`LEN` is a 3-digit hex value representing the payload byte length.

---

## SENSR -- Read Sensor Values

Perform a live sensor reading. The bitmask argument selects which sensor subsystems to sample.

**Request:** `sensors_bitmask` (uint, 1--15), `timeout` (uint, 5--300 seconds).

### Sensor Bitmask

| Bit | Value | Sensor |
|-----|-------|--------|
| 0 | 1 | Battery (voltage + state of charge) |
| 1 | 2 | Pressure (pressure, temperature, altitude) |
| 2 | 4 | GNSS (latitude, longitude, HDOP, satellite count) |
| 3 | 8 | Accelerometer (X, Y, Z axes, temperature, activity) |
| 4 | 16 | Thermistor (temperature) |

Combine bits to read multiple sensors. Use 31 (0x1F) for all sensors.

```
$SENSR#004;1,10\r
```

The `timeout` parameter is reserved for future GNSS acquisition use. Currently the GNSS reading returns the last cached position.

**Response:** 15 comma-separated fields:

| Field | Name | Type | Unit | Description |
|-------|------|------|------|-------------|
| 1 | batt_mv | uint | mV | Battery voltage in millivolts |
| 2 | batt_soc | uint | % | Battery state of charge (0--100) |
| 3 | pressure | float | bar | Barometric pressure |
| 4 | temperature | float | C | Pressure sensor temperature |
| 5 | altitude | float | m | Barometric altitude (computed from pressure) |
| 6 | lat | float | deg | GNSS latitude |
| 7 | lon | float | deg | GNSS longitude |
| 8 | hdop | float | -- | GNSS horizontal dilution of precision (99.9 = no fix) |
| 9 | num_sv | uint | -- | Number of GNSS satellites in view |
| 10 | accel_x | float | g | Accelerometer X-axis |
| 11 | accel_y | float | g | Accelerometer Y-axis |
| 12 | accel_z | float | g | Accelerometer Z-axis |
| 13 | accel_temp | float | C | Accelerometer temperature |
| 14 | activity | uint | -- | Activity level (0--255) |
| 15 | thermistor_temp | float | C | Thermistor temperature |

Fields for unselected sensors return default values (0 for integers, 0.0 for floats, 99.9 for hdop).

**Example -- battery only (mask=1):**

```
Request:  $SENSR#004;1,10\r
Response: $O;SENSR#LEN;3700,80,0.000000,0.000000,0.000000,0.000000,0.000000,99.900000,0,0.000000,0.000000,0.000000,0.000000,0\r
```

**Example -- all sensors (mask=15):**

```
Request:  $SENSR#005;15,60\r
Response: $O;SENSR#LEN;3700,80,1013.250000,22.500000,0.003906,48.856614,2.352222,1.200000,12,0.050000,-0.020000,0.980000,23.500000,42\r
```

**Altitude computation:** `altitude = 44330 * (1 - (P_hpa / P0_hpa)^(1/5.255))` where `P0_hpa` is the sea-level reference pressure read from the pressure sensor calibration (offset 0).

---

## SCALW -- Write Sensor Calibration

Write a calibration value to a specific sensor at a given offset.

**Request:** `sensor` (uint, 0--7), `offset` (uint), `value` (float).

### Sensor IDs

| ID | Sensor |
|----|--------|
| 0 | AXL (Accelerometer) |
| 1 | PRS (Pressure) |
| 2 | ALS (Ambient Light) |
| 3 | PH |
| 4 | RTD (Sea Temperature) |
| 5 | CDT (Conductivity/Depth/Temp) |
| 6 | MCP47X6 (DAC) |
| 7 | THERMISTOR |

```
$SCALW#00D;1,0,1013.250000\r
```

This writes the value `1013.25` to the pressure sensor (ID=1) at calibration offset 0 (sea-level reference pressure).

**Response:** OK (no payload).

```
$O;SCALW#000;\r
```

**Error:** Returns error 5 (INCORRECT_DATA) if the sensor is not registered in the `CalibratableManager`, or error 8 (MISSING_ARGUMENT) if fewer than 3 arguments are provided.

---

## SCALR -- Read Sensor Calibration

Read a calibration value from a specific sensor at a given offset.

**Request:** `sensor` (uint, 0--7), `offset` (uint).

```
$SCALR#003;1,0\r
```

**Response:** The calibration value (float).

```
$O;SCALR#00A;1013.250000\r
```

**Error:** Returns error 5 (INCORRECT_DATA) if the sensor is not registered, or error 8 (MISSING_ARGUMENT) if fewer than 2 arguments are provided.

---

## PWRON -- Power On/Off Components

Manually control power to hardware subsystems. Useful for testing and diagnostics.

**Request:** `component` (uint, 0--4).

| Value | Component | Description |
|-------|-----------|-------------|
| 0 | ALL | Power on GNSS, sensors, and satellite module |
| 1 | GNSS | Power on GNSS receiver (also enables sensor power rail) |
| 2 | SENSORS | Power on sensor power rail only |
| 3 | SATELLITE | Power on satellite module (also enables sensor power rail) |
| 4 | OFF | Power off all components |

**Example -- power on all:**

```
$PWRON#001;0\r
```

**Response:** OK (no payload).

```
$O;PWRON#000;\r
```

**Example -- power off:**

```
$PWRON#001;4\r
```

**Response:**

```
$O;PWRON#000;\r
```

**Error:** Returns error 7 (VALUE_OUT_OF_RANGE) for invalid component values.

---

## SWSST -- SWS Analog Calibration Status

Read the current state of the Salt Water Switch (SWS) analog underwater detection system. Only available on builds with `SWS_ADC` defined.

**Request:** No arguments.

```
$SWSST#000;\r
```

**Response:** 9 comma-separated fields:

| Field | Name | Type | Description |
|-------|------|------|-------------|
| 1 | air | uint | Air threshold (ADC counts) |
| 2 | water | uint | Water threshold (ADC counts) |
| 3 | threshold | uint | Current active threshold (ADC counts) |
| 4 | hysteresis | uint | Hysteresis value (ADC counts) |
| 5 | raw_adc | uint | Last raw ADC sample |
| 6 | filtered_adc | uint | Last filtered ADC value |
| 7 | calibrated | bool | 1 if calibration is complete, 0 otherwise |
| 8 | underwater | bool | 1 if currently detected as underwater, 0 otherwise |
| 9 | time_in_state | uint | Seconds spent in current state (air/water) |

```
$O;SWSST#LEN;2048,3500,2800,50,2750,2760,1,0,300\r
```

**Error:** Returns error 6 (PARAM_KEY_UNRECOGNISED) on builds without SWS analog support.

---

## SWSTST -- SWS Test Mode

Start or stop the Salt Water Switch (SWS) analog test mode. When active, the SWS service runs independently of the device configuration (`UNDERWATER_EN`, `UNDERWATER_DETECT_SOURCE`) and provides real-time LED feedback on state transitions.

**Request:** `action` (uint, 0 or 1).

| Value | Action |
|-------|--------|
| 0 | Stop test mode |
| 1 | Start test mode |

```
$SWSTST#001;1\r
```

**Response:** `running` (uint) -- 1 if test mode is active, 0 if stopped.

```
$O;SWSTST#001;1\r
```

### LED Feedback

During test mode, the RGB status LED indicates state transitions:

| State | LED Color |
|-------|-----------|
| Underwater detected | Blue |
| Surface detected | Yellow |
| Test mode stopped | Off |

The LED updates only on state changes, not on every sample. When test mode is stopped, the LED is turned off.

### Typical Workflow

```
$SWSTST#001;1\r       # Start SWS test
                       # → LED turns YELLOW (surface) or BLUE (underwater)
                       # Dip electrodes in salt water → LED turns BLUE
                       # Remove from water → LED turns YELLOW
$SWSST#000;\r          # Read live calibration values
$SWSTST#001;0\r        # Stop SWS test → LED off
```

---

## GNSSI -- GNSS Device Info

Query the GNSS module (u-blox M10Q) for its unique hardware ID and firmware/hardware version strings. The information is cached after the first GNSS power-on cycle.

**Request:** No arguments.

```
$GNSSI#000;\r
```

**Behavior:**

- If the GNSS module has been powered on at least once (via `PWRON 1` or a normal operational cycle), the cached info is returned immediately.
- If no cached info is available, the handler autonomously powers on the GNSS module, waits for the device info callback, sends the response asynchronously, then powers off the GNSS.

**Response:** 3 comma-separated TEXT fields:

| Field | Name | Type | Description |
|-------|------|------|-------------|
| 1 | unique_id | text | 5-byte hardware unique ID as hex string (10 chars) |
| 2 | sw_version | text | GNSS firmware version (e.g., "SPG 4.04 (7b202e)") |
| 3 | hw_version | text | GNSS hardware version (e.g., "00190000") |

**Example:**

```
Request:  $GNSSI#000;\r
Response: $O;GNSSI#02B;01A2B3C4D5,SPG 4.04 (7b202e),00190000\r
```

**Typical workflow:**

```
$PWRON#001;1\r          # Power on GNSS
(wait ~2 seconds for configure phase)
$GNSSI#000;\r           # Read device info (cached)
$PWRON#001;4\r          # Power off
```

**Error:** Returns error 5 (INCORRECT_DATA) if no GPS device is available or if device info could not be retrieved.

---

## GNSSA -- GNSS Almanac Status

Query the status of the GNSS AssistNow Offline almanac file stored on the device. This is useful to check whether the almanac needs to be refreshed before deployment.

**Request:** No arguments.

```
$GNSSA#000;\r
```

**Response:** 5 comma-separated fields:

| Field | Name | Type | Description |
|-------|------|------|-------------|
| 1 | present | uint | 1 if almanac file exists, 0 otherwise |
| 2 | file_size | uint | File size in bytes (0 if not present) |
| 3 | total_records | uint | Total number of records in the file |
| 4 | valid_records | uint | Number of records still valid (not expired) |
| 5 | stale | uint | 1 if almanac is expired/stale, 0 if fresh |

**Example:**

```
Request:  $GNSSA#000;\r
Response: $O;GNSSA#00D;1,32768,35,28,0\r
```

**Error:** Returns error 5 (INCORRECT_DATA) if no GPS device is available.
