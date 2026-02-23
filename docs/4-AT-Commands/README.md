# 4 - DTE Commands Reference

The DTE (Data Terminal Equipment) protocol is the primary interface for configuring and querying the tracker. It is used by **pylinkit** and any UART/USB/BLE terminal to communicate with the firmware.

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

## Command Summary

| Command | Direction | Description | Details |
|---------|-----------|-------------|---------|
| PARML | Request | List all parameter keys | [Parameter Commands](Parameter-Commands.md) |
| PARMR | Request | Read parameters by key | [Parameter Commands](Parameter-Commands.md) |
| PARMW | Request | Write parameters by key | [Parameter Commands](Parameter-Commands.md) |
| STATR | Request | Read status parameters (xxxTxx keys) | [Parameter Commands](Parameter-Commands.md) |
| PROFR | Request | Read profile name | [Device Commands](Device-Commands.md) |
| PROFW | Request | Write profile name | [Device Commands](Device-Commands.md) |
| FACTW | Request | Factory reset | [Device Commands](Device-Commands.md) |
| RSTBW | Request | Reboot device | [Device Commands](Device-Commands.md) |
| RSTVW | Request | Reset variable (TX/RX counters) | [Device Commands](Device-Commands.md) |
| SECUR | Request | Security access authentication | [Device Commands](Device-Commands.md) |
| DUMPM | Request | Dump raw memory region | [Device Commands](Device-Commands.md) |
| DUMPD | Request | Dump sensor log data (CSV) | [Device Commands](Device-Commands.md) |
| ERASE | Request | Erase sensor log data | [Device Commands](Device-Commands.md) |
| PASPW | Request | Write pass prediction (AOP) data | [Argos Commands](Argos-Commands.md) |
| SATTX | Request | Manual Argos satellite transmission | [Argos Commands](Argos-Commands.md) |
| SENSR | Request | Read sensor values | [Sensor Commands](Sensor-Commands.md) |
| SCALW | Request | Write sensor calibration value | [Sensor Commands](Sensor-Commands.md) |
| SCALR | Request | Read sensor calibration value | [Sensor Commands](Sensor-Commands.md) |
| PWRON | Request | Power on/off components | [Sensor Commands](Sensor-Commands.md) |
| SWSST | Request | Read SWS analog calibration status | [Sensor Commands](Sensor-Commands.md) |
| GNSSI | Request | Read GNSS module device info | [Sensor Commands](Sensor-Commands.md) |
| GNSSA | Request | Read GNSS almanac file status | [Sensor Commands](Sensor-Commands.md) |
| RTCW | Request | Set device RTC (unix timestamp) | [Device Commands](Device-Commands.md) |
| SATDP | Request | Start periodic Doppler TX calibration | [Argos Commands](Argos-Commands.md) |
| SMDDFU | Request | SMD satellite module DFU | [Argos Commands](Argos-Commands.md) |
| SMDTST | Request | SMD SPI applicative test | [Argos Commands](Argos-Commands.md) |
| SMDCD | Request | SMD credentials write | [Argos Commands](Argos-Commands.md) |

## Parameter Key Naming Convention

Parameter keys are 5 characters: 3-letter prefix + 2-digit number.

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

See [Parameter Reference](Parameter-Reference.md) for the complete table.
