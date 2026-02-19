# Parameter Management Commands

DTE commands for listing, reading, and writing device configuration parameters and telemetry values.

## Protocol Format

```
Request:  $CMD#LEN;PAYLOAD\r
Success:  $O;CMD#LEN;PAYLOAD\r
Error:    $N;CMD#LEN;ERROR_CODE\r
```

`LEN` is a 3-digit hex value representing the payload byte length. Commands with no payload use `#000;`.

## Parameter Key Naming Convention

Each parameter has a 5-character DTE key (e.g., `ARP05`, `GNP01`). The third character indicates the key type:

- **P** -- Configurable parameter (read/write). Returned by `PARMR` when called with no arguments.
- **T** -- Telemetry / status value (read-only). Returned by `STATR` when called with no arguments.

Examples:
- `ARP05` (TR_NOM) -- "P" key, configurable Argos parameter
- `ART02` (TX_COUNTER) -- "T" key, telemetry read-only counter
- `IDT03` (FW_APP_VERSION) -- "T" key, read-only identification

---

## PARML -- List All Parameter Keys

List all implemented parameter keys on the device.

**Request:** No arguments.

```
$PARML#000;\r
```

**Response:** Comma-separated list of all implemented DTE keys.

```
$O;PARML#LEN;IDP12,IDT06,IDT02,IDT03,ART01,ART02,POT03,POT05,IDP11,ART03,ARP05,ARP01,...\r
```

The list includes every parameter from the `param_map` where `is_implemented` is true, regardless of whether it is a "P" or "T" key. The exact set of keys depends on the build configuration (which sensors and features are enabled).

---

## PARMR -- Read Parameters

Read one or more parameter values by key. If no keys are provided, all configurable "P"-type parameters are returned.

**Request:** Comma-separated list of DTE keys (KEY_LIST encoding), or empty for all "P" keys.

Read specific parameters:

```
$PARMR#00F;GNP01,ARP05,ARP01\r
```

Read all configurable parameters (empty key list):

```
$PARMR#000;\r
```

**Response:** Comma-separated key=value pairs.

```
$O;PARMR#01B;GNP01=1,ARP05=60,ARP01=2\r
```

Full example reading a broad set of parameters:

```
Request:
$PARMR#0C5;IDT06,IDP12,IDT02,IDT03,ART01,ART02,POT03,POT05,IDP11,ART03,ARP05,ARP01,ARP19,ARP18,GNP01,ARP11,ARP16,GNP02,GNP03,GNP05,UNP01,UNP02,UNP03,LBP01,LBP02,ARP06,LBP04,LBP05,LBP06,ARP12,LBP07,LBP08,LBP09\r

Response:
$O;PARMR#159;IDT06=0,IDP12=0,IDT02=SURFACEBOX,IDT03=V0.1,ART01=01/01/1970 00:00:00,ART02=0,POT03=0,POT05=01/01/1970 00:00:00,IDP11=FACTORY,ART03=07/10/2021 22:41:14,ARP05=60,ARP01=2,ARP19=0,ARP18=0,GNP01=1,ARP11=1,ARP16=10,GNP02=1,GNP03=2,GNP05=120,UNP01=0,UNP02=1,UNP03=60,LBP01=0,LBP02=10,ARP06=240,LBP04=2,LBP05=0,LBP06=1,ARP12=4,LBP07=2,LBP08=1,LBP09=120\r
```

**Error:** Returns error 6 (PARAM_KEY_UNRECOGNISED) if any key in the list is not recognised.

---

## PARMW -- Write Parameters

Write one or more parameter values. Read-only parameters are silently skipped (no error). After writing, all parameters are saved to persistent storage and the `CONFIG_UPDATED` action is triggered.

**Request:** Comma-separated key=value pairs (KEY_VALUE_LIST encoding).

```
$PARMW#008;ARP05=90\r
```

Write multiple parameters at once:

```
$PARMW#011;GNP01=1,ARP05=120\r
```

**Response:** OK (no payload).

```
$O;PARMW#000;\r
```

**Error (out of range):** Returns error 7 (VALUE_OUT_OF_RANGE) if a value exceeds the parameter's allowed range.

```
Request:  $PARMW#009;ARP05=29\r
Response: $N;PARMW#001;7\r
```

**Side effects:** Triggers `DTEAction::CONFIG_UPDATED`, which notifies the system to reload active configuration.

### Commonly Used Parameters

| Key | Name | Type | Range | RW | Description |
|-----|------|------|-------|----|-------------|
| IDP12 | ARGOS_DECID | UINT | 0--0xFFFFFFFF | RW | Argos decimal ID |
| IDT06 | ARGOS_HEXID | HEX | 0--0xFFFFFFFF | RW | Argos hex ID |
| IDT02 | DEVICE_MODEL | TEXT | -- | RW | Device model name |
| IDT03 | FW_APP_VERSION | TEXT | -- | RO | Firmware version |
| IDP11 | PROFILE_NAME | TEXT | -- | RW | Profile name |
| ARP05 | TR_NOM | UINT | 30--1200 | RW | TX repetition (seconds) |
| ARP01 | ARGOS_MODE | ENUM | 0--4 | RW | Argos operating mode |
| GNP01 | GNSS_EN | BOOL | 0/1 | RW | GNSS enable |
| GNP05 | GNSS_ACQ_TIMEOUT | UINT | 10--600 | RW | GNSS acquisition timeout (seconds) |
| UNP01 | UNDERWATER_EN | BOOL | 0/1 | RW | Underwater detection enable |
| LBP01 | LB_EN | BOOL | 0/1 | RW | Low battery mode enable |
| LBP02 | LB_THRESHOLD | UINT | 0--100 | RW | Low battery threshold (%) |
| ARP16 | ARGOS_DEPTH_PILE | ENUM | 1,2,3,4,8,9,10,11,12 | RW | Message depth (repetitions) |

---

## STATR -- Read Status/Telemetry Parameters

Read telemetry and status values. Functionally identical to `PARMR` but filters for "T"-type (telemetry) keys when called with an empty argument. When specific keys are provided, it reads those keys regardless of type.

**Request:** Comma-separated list of DTE keys, or empty for all "T" keys.

Read all telemetry parameters:

```
$STATR#000;\r
```

**Response:** Comma-separated key=value pairs (only "T"-type keys).

```
$O;STATR#0B9;IDT06=0,IDT02=SURFACEBOX,IDT03=V0.1,ART01=01/01/1970 00:00:00,ART02=0,POT03=0,POT05=01/01/1970 00:00:00,ART03=07/10/2021 22:41:14,ART10=0,ART11=0,IDT04=SIMULATOR,POT06=0,IDT10=305419896\r
```

Read specific keys (can include both P and T keys):

```
$STATR#00A;ART02,IDT03\r
```

**Response:**

```
$O;STATR#013;ART02=0,IDT03=V0.1\r
```

### Default Telemetry Keys

| Key | Name | Description |
|-----|------|-------------|
| IDT06 | ARGOS_HEXID | Argos hex ID |
| IDT02 | DEVICE_MODEL | Device model |
| IDT03 | FW_APP_VERSION | Firmware version |
| IDT04 | HW_VERSION | Hardware version |
| IDT10 | DEVICE_DECID | Device decimal ID |
| ART01 | LAST_TX | Last transmission timestamp |
| ART02 | TX_COUNTER | Transmit counter |
| ART03 | ARGOS_AOP_DATE | AOP bulletin date |
| ART10 | ARGOS_RX_COUNTER | Argos receive counter |
| ART11 | ARGOS_RX_TIME | Argos receive time |
| POT03 | BATT_SOC | Battery state of charge (%) |
| POT05 | LAST_FULL_CHARGE_DATE | Last full charge timestamp |
| POT06 | BATT_VOLTAGE | Battery voltage (V) |
