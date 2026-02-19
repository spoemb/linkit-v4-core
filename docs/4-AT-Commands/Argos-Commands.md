# Argos Satellite Commands

DTE commands for Argos satellite operations: pass prediction upload, manual transmit, and SMD module management.

## Protocol Format

```
Request:  $CMD#LEN;PAYLOAD\r
Success:  $O;CMD#LEN;PAYLOAD\r
Error:    $N;CMD#LEN;ERROR_CODE\r
```

`LEN` is a 3-digit hex value representing the payload byte length.

---

## PASPW -- Write Pass Prediction Data (AOP)

Upload Argos satellite pass prediction (AOP) data to the device. The firmware decodes the prepass file, extracts satellite orbital records, and updates the `ARGOS_AOP_DATE` parameter to the most recent bulletin timestamp found in the data.

**Request:** `prepass_file` (base64-encoded binary prepass data).

```
$PASPW#LEN;<base64_encoded_prepass_data>\r
```

**Response:** OK (no payload).

```
$O;PASPW#000;\r
```

**Processing details:**

1. The base64 payload is decoded and parsed as an AOP satellite record set (up to 40 satellite entries).
2. The firmware scans all records for the most recent bulletin date.
3. If valid records exist, they are written to the configuration store via `write_pass_predict()`.
4. `ARGOS_AOP_DATE` (key `ART03`) is set to the most recent bulletin date and saved.

**Verification:** After a successful PASPW, the AOP date can be confirmed with PARMR:

```
$PARMR#005;ART03\r
$O;PARMR#019;ART03=18/09/2021 23:09:10\r
```

**Error:** Returns error 5 (INCORRECT_DATA) if:
- The base64 data cannot be decoded as a valid prepass file
- The decoded data contains zero satellite records
- No records have a valid bulletin date

---

## SATTX -- Manual Argos Transmit

Trigger a manual Argos satellite transmission for testing or certification purposes. The command is asynchronous: the OK response is sent only after the transmission completes (or an error occurs).

**Request:** `modulation` (uint), `power` (uint, mW), `freq` (float, MHz), `size` (uint, bytes), `tcxo_warmup` (uint, seconds, optional).

### Modulation Values

| Value | Modulation |
|-------|------------|
| 0 | LDK |
| 1 | LDA2 |
| 2 | VLDA4 |

```
$SATTX#01A;1,500,401.630000,4,5\r
```

This sends a 4-byte LDA2 transmission at 500 mW on 401.630 MHz with 5 seconds of TCXO warmup.

**Response (async, on success):**

```
$O;SATTX#000;\r
```

**Response (async, on error):**

```
$N;SATTX#001;5\r
```

**Behavior:**
- On SMD builds (`ARGOS_SMD=1`), the command activates the satellite device, subscribes to TX events, configures power/frequency/modulation, and transmits a test packet filled with `0xFF` bytes.
- The response is delivered asynchronously via `KineisEventTxComplete` (success) or `KineisEventDeviceError` (failure) callbacks.
- The satellite device auto-powers off after a 30-second idle timeout.
- On non-SMD builds, returns error 5 (INCORRECT_DATA) immediately.

**Error:** Returns error 8 (MISSING_ARGUMENT) if fewer than 4 arguments are provided. The `tcxo_warmup` parameter defaults to 0 if not supplied.

---

## SMDDFU -- SMD Satellite DFU

Manage firmware updates on the SMD (Satellite Module for ARGOS) via SPI. Only available on RSPB+SMD builds (`ARGOS_SMD=1`).

**Request:** `action` (uint, 0--5).

### Actions

| Value | Action | Description |
|-------|--------|-------------|
| 0 | ENTER | Enter DFU (bootloader) mode |
| 1 | EXIT | Exit DFU mode, return to application |
| 2 | STATUS | Query current DFU mode state |
| 3 | UPDATE | Trigger firmware update (requires prior OTA file upload) |
| 4 | INFO | Get bootloader information (only valid in DFU mode) |
| 5 | VERSION | Get application firmware version (only valid outside DFU mode) |

**Example -- check DFU status:**

```
$SMDDFU#001;2\r
```

**Example -- get firmware version:**

```
$SMDDFU#001;5\r
```

**Response:** `status` (uint), `dfu_mode` (bool), `progress` (uint, 0--100%), `info` (text).

```
$O;SMDDFU#LEN;0,0,0,v1.2.3\r
```

Response field details:

| Field | Type | Description |
|-------|------|-------------|
| status | uint | 0=OK, 1=error |
| dfu_mode | bool | 1 if in DFU/bootloader mode, 0 if in application mode |
| progress | uint | Firmware update progress percentage (0--100) |
| info | text | Additional info (firmware version, bootloader version, error message) |

**Example responses:**

```
Version query (app mode):    $O;SMDDFU#LEN;0,0,0,v1.2.3\r
Status query (DFU mode):     $O;SMDDFU#LEN;0,1,0,\r
Bootloader info (DFU mode):  $O;SMDDFU#LEN;0,1,0,BL v2.1\r
Version query (DFU mode):    $O;SMDDFU#LEN;1,1,0,SMD in DFU mode - exit first\r
```

**Error:** Returns error 5 (INCORRECT_DATA) if the SMD satellite instance is not available or if an invalid action is provided.

---

## SMDTST -- SMD SPI Applicative Test

Run a full SPI diagnostic test against the SMD satellite module. Executes all A+ protocol read commands and returns the results. Only available on RSPB+SMD builds (`ARGOS_SMD=1`).

**Request:** No arguments.

```
$SMDTST#000;\r
```

**Response:** Test results as a text string.

```
$O;SMDTST#LEN;<test_results_text>\r
```

The test result string contains the output of all A+ protocol read commands executed over SPI.

**Error:** Returns error 5 (INCORRECT_DATA) if the SMD satellite instance is not available or if the SPI test returns no data.

---

## SMDCD -- SMD Credentials Write

Write Argos satellite credentials to the SMD module and update the device configuration store. After writing, the credentials are read back from the SMD for verification and then stored persistently. Only available on RSPB+SMD builds (`ARGOS_SMD=1`).

**Request:** `id` (uint, 0--999999), `addr` (hex, 0--0xFFFFFFFF), `seckey` (text), `radioconf` (text).

| Parameter | Encoding | Description |
|-----------|----------|-------------|
| id | uint | Argos decimal ID |
| addr | hex | Argos hex address |
| seckey | text | Security key string |
| radioconf | text | Radio configuration string |

```
$SMDCD#LEN;12345,ABCDEF01,0123456789ABCDEF,<radioconf_string>\r
```

**Response:** OK (no payload).

```
$O;SMDCD#000;\r
```

**Processing details:**

1. Credentials are written to the SMD module via `set_credentials()`.
2. Credentials are read back from the SMD via `read_credentials()` to confirm.
3. The confirmed values are stored in the configuration store:
   - `ARGOS_DECID` (key `IDP12`)
   - `ARGOS_HEXID` (key `IDT06`)
   - `ARGOS_SECKEY` (key `IDP13`)
   - `ARGOS_RADIOCONF` (key `IDP14`)
4. Configuration is saved to persistent storage.

**Error:** Returns error 5 (INCORRECT_DATA) if:
- The SMD satellite instance is not available
- The credential write or read-back operation fails
