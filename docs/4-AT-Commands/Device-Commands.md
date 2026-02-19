# Device Management Commands

DTE commands for device identification, reset, security, and data dump operations.

## Protocol Format

```
Request:  $CMD#LEN;PAYLOAD\r
Success:  $O;CMD#LEN;PAYLOAD\r
Error:    $N;CMD#LEN;ERROR_CODE\r
```

`LEN` is a 3-digit hex value representing the payload byte length. Commands with no payload use `#000;`.

## Error Codes

| Code | Meaning |
|------|---------|
| 0 | OK |
| 1 | Incorrect command |
| 4 | Data length mismatch |
| 5 | Incorrect data |
| 6 | Parameter key unrecognised |
| 7 | Value out of range |
| 8 | Missing argument |
| 9 | Bad format |
| 10 | Message too large |
| 12 | Invalid access code |

---

## PROFR -- Read Profile Name

Read the device's profile name string.

**Request:** No arguments.

```
$PROFR#000;\r
```

**Response:** The profile name (TEXT).

```
$O;PROFR#018;Profile Name For Tracker\r
```

---

## PROFW -- Write Profile Name

Set the device's profile name. Maximum 128 characters.

**Request:** `profile_name` (TEXT, 1--128 chars).

```
$PROFW#018;Profile Name For Tracker\r
```

**Response:** OK (no payload).

```
$O;PROFW#000;\r
```

---

## FACTW -- Factory Reset

Restore all parameters to factory defaults and increment the configuration version. The device sends an OK response and then performs a full reset.

**Request:** No arguments.

```
$FACTW#000;\r
```

**Response:** OK, then the device resets.

```
$O;FACTW#000;\r
```

**Side effects:** Triggers `DTEAction::FACTR`. The DTE service sends the response before performing the reset, so the caller will receive the OK before the connection drops.

---

## RSTBW -- Reboot Device

Reboot the device. The device sends an OK response and then performs a software reset.

**Request:** No arguments.

```
$RSTBW#000;\r
```

**Response:** OK, then the device reboots.

```
$O;RSTBW#000;\r
```

**Side effects:** Triggers `DTEAction::RESET`.

---

## RSTVW -- Reset Variable

Reset a specific runtime counter variable to zero.

**Request:** `index` (hex). Permitted values:

| Index | Variable |
|-------|----------|
| 1 | TX_COUNTER |
| 3 | ARGOS_RX_COUNTER |
| 4 | ARGOS_RX_TIME |

```
$RSTVW#001;1\r
```

**Response:** OK (no payload).

```
$O;RSTVW#000;\r
```

**Error:** Returns error code 5 (INCORRECT_DATA) if the index is not one of the permitted values.

---

## SECUR -- Security Access

Authenticate for privileged operations (e.g., OTA firmware update). The firmware accepts one of two valid access codes:

- Static code: `0x12345678`
- Device-specific code: the device's `ARGOS_DECID` value

**Request:** `accesscode` (hex).

```
$SECUR#008;12345678\r
```

**Response (success):**

```
$O;SECUR#000;\r
```

**Response (failure):** Error code 12 (INVALID_ACCESS_CODE).

```
$N;SECUR#001;C\r
```

**Side effects:** On success, triggers `DTEAction::SECUR` to grant elevated DTE privileges.

---

## DUMPM -- Dump Memory

Read raw memory contents from the device. Returns the data as a base64-encoded string.

**Request:** `start_address` (hex), `length` (hex, max `0x500`).

```
$DUMPM#007;100,200\r
```

**Response:** Base64-encoded memory contents.

```
$O;DUMPM#LEN;<base64_data>\r
```

**Error:** Returns error 5 (INCORRECT_DATA) if the memory access interface is not available, or error 8 (MISSING_ARGUMENT) if fewer than 2 arguments are provided.

---

## DUMPD -- Dump Log Data

Read logged sensor data in a paginated CSV format. Each call returns one page of results. The response includes a page index (`mmm`) and the maximum page index (`MMM`). Call repeatedly until `mmm` equals `MMM` to retrieve all data.

The first page (mmm=0) includes the CSV header row.

**Request:** `d_type` (hex) -- log type identifier.

| d_type (hex) | Log Type |
|--------------|----------|
| 0 | Internal (system.log) |
| 1 | GNSS Sensor |
| 2 | ALS Sensor |
| 3 | PH Sensor |
| 4 | RTD Sensor |
| 5 | CDT Sensor |
| 6 | Camera Sensor |
| 7 | AXL (Accelerometer) |
| 8 | Pressure Sensor |
| 9 | Thermistor Sensor |
| A | TSYS01 Sensor |

```
$DUMPD#001;1\r
```

**Response:** `mmm` (hex page index), `MMM` (hex max page index), `data` (base64-encoded CSV).

```
$O;DUMPD#LEN;000,002,<base64_csv_data>\r
```

**Pagination:** When `mmm < MMM`, the handler signals `DTEAction::AGAIN` to indicate more pages remain. The caller should re-issue the same DUMPD request to fetch the next page. State resets automatically after the last page or if the `d_type` changes between calls.

**Max entries per page:** 8 log entries.

---

## ERASE -- Erase Log Data

Delete logged data for a specific sensor type, or all logs at once.

**Request:** `log_type` (uint, range 1--12).

| log_type | Target |
|----------|--------|
| 1 | GNSS Sensor (sensor.log) |
| 2 | System (system.log) |
| 3 | ALL (truncates all loggers) |
| 4 | ALS Sensor |
| 5 | PH Sensor |
| 6 | RTD Sensor |
| 7 | CDT Sensor |
| 8 | Camera Sensor |
| 9 | AXL (Accelerometer) |
| 10 | Pressure Sensor |
| 11 | Thermistor Sensor |
| 12 | TSYS01 Sensor |

```
$ERASE#001;3\r
```

**Response:** OK (no payload).

```
$O;ERASE#000;\r
```

**Error:** Returns error 7 (VALUE_OUT_OF_RANGE) for invalid log_type values (e.g., 0).

```
$N;ERASE#001;7\r
```
