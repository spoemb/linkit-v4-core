This page describes every message type transmitted by the tracker over Argos satellite and LoRaWAN, including the bit-level encoding and how to decode received payloads.

All packets are packed **MSB-first** (big endian) using variable-length bit fields. The firmware uses `PACK_BITS()` from `core/util/bitpack.hpp` to build packets sequentially.

---

# 1 - Argos Message Types

The tracker produces 4 Argos packet types depending on the operating mode and available data:

| Type | Header | Size | When Used |
|------|--------|------|-----------|
| [Short Packet](#11---short-packet-gnss-single-fix) | `0b000` (3 bits) | 96 bits (12 bytes) | Single GPS fix |
| [Long Packet](#12---long-packet-gnss-multi-fix) | *(none)* | 224 bits (28 bytes) | Multiple GPS fixes (depth pile) |
| [Sensor Packet](#13---sensor-packet-gnss--sensors) | *(none)* | Variable, max 192 bits (24 bytes) | GPS + environmental sensors |
| [Doppler Packet](#14---doppler-packet-no-gnss) | *(none)* | 24 bits (3 bytes) | No GPS, battery only |

CRC8 and BCH error correction are added by the satellite module (SMD or KIM2), not by the firmware.

---

## 1.1 - Short Packet (GNSS Single Fix)

**96 bits / 12 bytes.** Used when `ARGOS_DEPTH_PILE=1` or only one fix is available.

### Bit Layout

| Bit Offset | Width | Field | Encoding |
|------------|-------|-------|----------|
| 0 | 3 | Header | `0b000` |
| 3 | 5 | Day | Day of month (1-31) |
| 8 | 5 | Hour | Hour (0-23) |
| 13 | 6 | Minute | Minute (0-59) |
| 19 | 21 | Latitude | See [GPS Encoding](#16---gps-coordinate-encoding) |
| 40 | 22 | Longitude | See [GPS Encoding](#16---gps-coordinate-encoding) |
| 62 | 7 | Speed | `(ground_speed_m_s * 3600) / 2000000` |
| 69 | 1 | Out-of-zone | 1 = device is outside geofence zone |
| 70 | 8 | Heading | `heading_degrees / 1.42` |
| 78 | 8 | Altitude | `altitude_mm / (1000 * 40)`. 255 = no 3D fix |
| 86 | 7 | Battery | `(voltage_mV - 2700) / 20` (0-127) |
| 93 | 1 | Low battery | 1 = battery below threshold |
| 94 | 2 | *(padding)* | Unused (zeros) |

**No valid GPS fix:** Latitude, longitude, and speed fields are filled with all 1s (`0x1FFFFF`, `0x3FFFFF`, `0x7F`).

### Decoding Example (Python)

```python
def decode_short_packet(data: bytes):
    """Decode a 12-byte Argos short packet."""
    bits = int.from_bytes(data, 'big')
    total = 96

    def extract(offset, width):
        return (bits >> (total - offset - width)) & ((1 << width) - 1)

    header   = extract(0, 3)    # Should be 0b000
    day      = extract(3, 5)
    hour     = extract(8, 5)
    minute   = extract(13, 6)
    lat_raw  = extract(19, 21)
    lon_raw  = extract(40, 22)
    speed_r  = extract(62, 7)
    ooz      = extract(69, 1)
    heading_r= extract(70, 8)
    alt_raw  = extract(78, 8)
    batt_raw = extract(86, 7)
    low_batt = extract(93, 1)

    # Decode latitude (21 bits, bit 20 = sign)
    if lat_raw & (1 << 20):
        lat = -((lat_raw & 0xFFFFF) / 10000.0)
    else:
        lat = lat_raw / 10000.0

    # Decode longitude (22 bits, bit 21 = sign)
    if lon_raw & (1 << 21):
        lon = -((lon_raw & 0x1FFFFF) / 10000.0)
    else:
        lon = lon_raw / 10000.0

    speed_kmh   = speed_r * 2000000 / 3600  # Back to mm/s, then /1000 for m/s
    heading_deg = heading_r * 1.42
    altitude_m  = alt_raw * 40 if alt_raw != 255 else None
    voltage_mV  = batt_raw * 20 + 2700

    # Check for invalid fix (all 1s)
    valid = not (lat_raw == 0x1FFFFF and lon_raw == 0x3FFFFF)

    return {
        'day': day, 'hour': hour, 'minute': minute,
        'latitude': lat if valid else None,
        'longitude': lon if valid else None,
        'speed_m_s': speed_kmh / 1000 if valid else None,
        'heading_deg': heading_deg,
        'altitude_m': altitude_m,
        'battery_mV': voltage_mV,
        'low_battery': bool(low_batt),
        'out_of_zone': bool(ooz),
        'valid': valid,
    }
```

---

## 1.2 - Long Packet (GNSS Multi-Fix)

**224 bits / 28 bytes.** Used when `ARGOS_DEPTH_PILE > 1` and multiple fixes are available. Packs up to **4 GPS entries** per message.

### Bit Layout

| Bit Offset | Width | Field | Notes |
|------------|-------|-------|-------|
| 0 | 5 | Day | Day of most recent fix |
| 5 | 5 | Hour | |
| 10 | 6 | Minute | |
| 16 | 21 | GPS[0] Latitude | Most recent fix |
| 37 | 22 | GPS[0] Longitude | |
| 59 | 7 | GPS[0] Speed | |
| 66 | 1 | Out-of-zone | |
| 67 | 7 | Battery voltage | |
| 74 | 1 | Low battery | |
| 75 | 4 | Delta-time-loc | Time interval code between GPS entries |
| 79 | 21 | GPS[1] Latitude | |
| 100 | 22 | GPS[1] Longitude | |
| 122 | 21 | GPS[2] Latitude | |
| 143 | 22 | GPS[2] Longitude | |
| 165 | 21 | GPS[3] Latitude | |
| 186 | 22 | GPS[3] Longitude | |

**No header field** — the long packet is identified by its size (224 bits vs 96 for short).

GPS entries are stored **most recent first**. The time of GPS[N] = timestamp - (N * delta_time_loc).

### Delta-Time-Loc Codes

| Code | Interval | Code | Interval |
|------|----------|------|----------|
| 1 | 10 min | 6 | 3 hours |
| 2 | 15 min | 7 | 4 hours |
| 3 | 30 min | 8 | 6 hours |
| 4 | 1 hour | 9 | 12 hours |
| 5 | 2 hours | 10 | 24 hours |

Controlled by parameter `ARP11` (DLOC_ARG_NOM).

---

## 1.3 - Sensor Packet (GNSS + Sensors)

**Variable length, max 192 bits (24 bytes).** Combines one GPS fix with environmental sensor data. Used when any sensor has `ENABLE_TX_MODE != OFF`.

### Base Fields (always present)

| Bit Offset | Width | Field | Encoding |
|------------|-------|-------|----------|
| 0 | 5 | Day | *(no header — sensor packet has no 3-bit header on Argos)* |
| 5 | 5 | Hour | |
| 10 | 6 | Minute | |
| 16 | 21 | Latitude | |
| 37 | 22 | Longitude | |
| 59 | 7 | Speed | |
| 66 | 1 | Out-of-zone | |
| 67 | 7 | Battery voltage | |
| 74 | 1 | Low battery | |

**= 75 bits base**

### Sensor Fields (appended in order, only if sensor is enabled with TX mode)

| Sensor | Width | Pre-encoding | Decode Formula |
|--------|-------|-------------|----------------|
| ALS (ambient light) | 17 bits | Raw ADC value | `value` (lumens raw) |
| pH | 14 bits | `pH * 1000` | `value / 1000.0` |
| Pressure (bar) | 15 bits | `pressure_hPa * 1000` | `value / 1000.0` (hPa) |
| Pressure (temp) | 14 bits | `(temp_C + 40) * 100` | `value / 100.0 - 40.0` (°C) |
| Sea temperature | 21 bits | `(temp_C + 126) * 1000` | `value / 1000.0 - 126.0` (°C) |
| Thermistor (RSPB) | 14 bits | `(temp_C + 40) * 100` | `value / 100.0 - 40.0` (°C) |
| AXL temp | 14 bits | `(temp_C + 40) * 100` | `value / 100.0 - 40.0` (°C) |
| AXL X-axis | 15 bits | `(accel_g + g_range) * 1000` | `value / 1000.0 - g_range` (g) |
| AXL Y-axis | 15 bits | *(same)* | *(same)* |
| AXL Z-axis | 15 bits | *(same)* | *(same)* |
| AXL activity | 8 bits | Raw (0-255) | `value` |
| Mortality confidence | 7 bits | Raw (0-100%) | `value` (%) |

**g_range** depends on `AXP08` (AXL_SENSOR_MEASUREMENT_RANGE): 0=2.0g, 1=4.0g, 2=8.0g, 3=16.0g.

### Board-Specific Variants

**RSPB (bird tracker):**
- Thermistor uses 14 bits (offset +40, scale x100) instead of sea temp's 21 bits
- Accelerometer: **activity only** (8 bits) — X/Y/Z axes and temp are dropped
- Mortality confidence appended (7 bits) if `ENABLE_MORTALITY_SENSOR`

**LinkIt V4 (marine):**
- Sea temperature uses 21 bits (offset +126, scale x1000) — wider range for ocean
- Accelerometer: full format = temp(14) + X(15) + Y(15) + Z(15) + activity(8) = **67 bits**
- No mortality field

### Size Examples

| Configuration | Bits | Bytes |
|---------------|------|-------|
| GPS only | 75 | 10 |
| GPS + Pressure | 75 + 29 = 104 | 13 |
| GPS + Pressure + Thermistor + AXL(RSPB) | 75 + 29 + 14 + 8 = 126 | 16 |
| GPS + Pressure + Thermistor + AXL(RSPB) + Mortality | 75 + 29 + 14 + 8 + 7 = 133 | 17 |
| GPS + ALS + pH + Pressure + Sea Temp + AXL(full) | 75 + 17 + 14 + 29 + 21 + 67 = 223 | **28 (truncated to 24)** |

If the total exceeds 192 bits (24 bytes), the packet is **truncated** with a warning. Choose sensor combinations carefully.

### RSPB Sensor Packet (Fixed Layout)

The RSPB bird tracker uses a **fixed sensor combination**: Pressure + Thermistor + AXL (compact) + Mortality. This produces a predictable 133-bit packet on every transmission.

#### Bit Layout

| Bit Offset | Width | Field | Encoding | Decode |
|------------|-------|-------|----------|--------|
| 0 | 5 | Day | Day of month (1-31) | |
| 5 | 5 | Hour | Hour (0-23) | |
| 10 | 6 | Minute | Minute (0-59) | |
| 16 | 21 | Latitude | See [GPS Encoding](#16---gps-coordinate-encoding) | |
| 37 | 22 | Longitude | See [GPS Encoding](#16---gps-coordinate-encoding) | |
| 59 | 7 | Speed | `(gSpeed_mm_s * 3600) / 2000000` | `raw * 2000000 / 3600` mm/s |
| 66 | 1 | Out-of-zone | 1 = outside geofence | |
| 67 | 7 | Battery voltage | `(mV - 2700) / 20` | `raw * 20 + 2700` mV |
| 74 | 1 | Low battery | 1 = below LB threshold | |
| **75** | **15** | **Pressure (hPa)** | `pressure_hPa * 1000` | `raw / 1000.0` hPa |
| **90** | **14** | **Pressure temp** | `(temp_C + 40) * 100` | `raw / 100.0 - 40.0` °C |
| **104** | **14** | **Body temperature** | `(temp_C + 40) * 100` | `raw / 100.0 - 40.0` °C |
| **118** | **8** | **AXL activity** | Raw (0-255) | `raw` |
| **126** | **7** | **Mortality confidence** | Raw (0-100) | `raw` % |

**Total: 133 bits / 17 bytes** — 59 bits free within the 192-bit LDA2 limit.

#### Field Details

**Body temperature (thermistor):** NTC sensor measuring skin/body temperature. Live bird: ~38-42°C. Dead bird converges to ambient (~10-25°C). This is the primary mortality indicator — a reading below `MORTALITY_TEMP_THRESH` (default 25°C) scores 30 points toward mortality.

**AXL activity (compact):** BMA400 activity score (0-255). On RSPB, X/Y/Z axes and AXL die temperature are dropped (useless for bird tracking — only movement detection matters). At rest: ~0-5, walking: ~20-50, flying: ~100+. Below `MORTALITY_ACTIVITY_THRESH` (default 10) scores 40 points toward mortality.

**Mortality confidence:** EMA-smoothed confidence percentage (0-100%). Combines activity (40pts), temperature (30pts), and GPS stationarity (30pts). Values:
- 0-49%: ALIVE
- 50-79%: SUSPECTED
- 80-100% sustained over `MORTALITY_CONFIRM_DAYS` (default 3): CONFIRMED

**Pressure:** Barometric pressure from LPS28DFW. Used for altitude estimation and dive depth on marine models. On RSPB, primarily useful for altitude context.

#### Decoding Example (Python)

```python
def decode_rspb_sensor_packet(data: bytes):
    """Decode a 17-byte RSPB sensor packet (133 bits)."""
    bits = int.from_bytes(data, 'big')
    total = len(data) * 8
    pos = 0

    def extract(width):
        nonlocal pos
        val = (bits >> (total - pos - width)) & ((1 << width) - 1)
        pos += width
        return val

    result = {}

    # Time
    result['day']    = extract(5)
    result['hour']   = extract(5)
    result['minute'] = extract(6)

    # GPS
    lat_raw = extract(21)
    lon_raw = extract(22)
    result['latitude']  = decode_latitude(lat_raw)
    result['longitude'] = decode_longitude(lon_raw)
    speed_raw = extract(7)
    result['speed_m_s'] = speed_raw * 2000000 / 3600 / 1000

    # Flags
    result['out_of_zone'] = bool(extract(1))

    # Battery
    result['battery_mV']  = extract(7) * 20 + 2700
    result['low_battery']  = bool(extract(1))

    # Pressure sensor
    result['pressure_hPa']    = extract(15) / 1000.0
    result['pressure_temp_C'] = extract(14) / 100.0 - 40.0

    # Body temperature (thermistor)
    result['body_temp_C'] = extract(14) / 100.0 - 40.0

    # Accelerometer (compact: activity only)
    result['activity'] = extract(8)

    # Mortality confidence
    result['mortality_pct'] = extract(7)

    # GPS validity check
    valid = not (lat_raw == 0x1FFFFF and lon_raw == 0x3FFFFF)
    result['gps_valid'] = valid
    if not valid:
        result['latitude'] = None
        result['longitude'] = None
        result['speed_m_s'] = None

    return result
```

#### Example Decoded Output

```
Input (hex):  0A 8C 3B 2E 91 A4 00 1C 0F 83 E8 1A 08 32 19 80 00
Output:
  day=5, hour=6, minute=12
  latitude=-48.8752, longitude=-123.3925
  speed_m_s=0.0
  battery_mV=3260, low_battery=False
  pressure_hPa=1013.250, pressure_temp_C=21.5
  body_temp_C=12.3        ← below 25°C threshold → hypothermic
  activity=2              ← below 10 threshold → immobile
  mortality_pct=87        ← SUSPECTED/CONFIRMED range
  gps_valid=True
```

---

### Decoding Example (Python) — Generic Sensor Packet

```python
def decode_sensor_packet(data: bytes, sensors: dict):
    """
    Decode an Argos sensor packet.
    sensors: dict of enabled sensors, e.g.:
        {'pressure': True, 'thermistor': True, 'axl_rspb': True, 'mortality': True}
    """
    bits = int.from_bytes(data, 'big')
    total = len(data) * 8
    pos = 0

    def extract(width):
        nonlocal pos
        val = (bits >> (total - pos - width)) & ((1 << width) - 1)
        pos += width
        return val

    result = {}
    result['day']       = extract(5)
    result['hour']      = extract(5)
    result['minute']    = extract(6)

    lat_raw = extract(21)
    lon_raw = extract(22)
    result['latitude']  = decode_latitude(lat_raw)
    result['longitude'] = decode_longitude(lon_raw)
    result['speed']     = extract(7) * 2000000 / 3600 / 1000  # m/s
    result['out_of_zone'] = bool(extract(1))
    result['battery_mV'] = extract(7) * 20 + 2700
    result['low_battery'] = bool(extract(1))

    if sensors.get('als'):
        result['als_raw'] = extract(17)
    if sensors.get('ph'):
        result['ph'] = extract(14) / 1000.0
    if sensors.get('pressure'):
        result['pressure_hPa'] = extract(15) / 1000.0
        result['pressure_temp_C'] = extract(14) / 100.0 - 40.0
    if sensors.get('sea_temp'):
        result['sea_temp_C'] = extract(21) / 1000.0 - 126.0
    if sensors.get('thermistor'):
        result['thermistor_C'] = extract(14) / 100.0 - 40.0
    if sensors.get('axl_rspb'):
        result['axl_activity'] = extract(8)
    elif sensors.get('axl_full'):
        g_range = sensors.get('g_range', 2.0)
        result['axl_temp_C']  = extract(14) / 100.0 - 40.0
        result['axl_x_g']     = extract(15) / 1000.0 - g_range
        result['axl_y_g']     = extract(15) / 1000.0 - g_range
        result['axl_z_g']     = extract(15) / 1000.0 - g_range
        result['axl_activity'] = extract(8)
    if sensors.get('mortality'):
        result['mortality_confidence_pct'] = extract(7)

    return result


def decode_latitude(raw):
    if raw & (1 << 20):
        return -((raw & 0xFFFFF) / 10000.0)
    return raw / 10000.0

def decode_longitude(raw):
    if raw & (1 << 21):
        return -((raw & 0x1FFFFF) / 10000.0)
    return raw / 10000.0
```

---

## 1.4 - Doppler Packet (No GNSS)

**24 bits / 3 bytes.** Minimal packet with battery info only. The Argos satellite estimates position from the Doppler frequency shift of the received signal (~5-10 km accuracy).

Used in:
- **SURFACING_BURST** mode before GPS fix is acquired
- **DOPPLER** mode (`ARP01=4`)
- **Low battery** mode with `LB_GNSS_EN=0`

### Bit Layout

| Bit Offset | Width | Field | Encoding |
|------------|-------|-------|----------|
| 0 | 8 | Last position index | Usually 0 |
| 8 | 7 | Battery voltage | `(voltage_mV - 2700) / 20` |
| 15 | 1 | Low battery | 1 = low battery |
| 16 | 8 | CRC8 | Added by satellite module |

---

## 1.5 - Certification Packet

Custom payload for Argos TX power and frequency certification. Not used in production. Controlled by `CTP01-CTP04` parameters.

---

## 1.6 - GPS Coordinate Encoding

Both Argos and LoRa use the same coordinate encoding:

**Latitude (21 bits):**
- Positive (North): `encoded = latitude * 10000`
- Negative (South): `encoded = (|latitude| * 10000) | (1 << 20)`
- Resolution: 0.0001° (~11.1 meters)

**Longitude (22 bits):**
- Positive (East): `encoded = longitude * 10000`
- Negative (West): `encoded = (|longitude| * 10000) | (1 << 21)`
- Resolution: 0.0001° (~11.1 meters at equator)

**Decode:**
```
if (bit 20 set for lat / bit 21 set for lon):
    value = -(raw & mask) / 10000.0
else:
    value = raw / 10000.0
```

---

## 1.7 - Common Field Encodings

| Field | Bits | Encode | Decode | Unit | Range |
|-------|------|--------|--------|------|-------|
| Battery voltage | 7 | `(mV - 2700) / 20` | `raw * 20 + 2700` | mV | 2700-5240 |
| Speed | 7 | `(m/s * 3600) / 2000000` | `raw * 2000000 / 3600` | mm/s | 0-127 units |
| Heading | 8 | `degrees / 1.42` | `raw * 1.42` | degrees | 0-360 |
| Altitude | 8 | `mm / (1000 * 40)` | `raw * 40` | meters | 0-10160m, 255=invalid |
| Latitude | 21 | See above | See above | degrees | ±90 |
| Longitude | 22 | See above | See above | degrees | ±180 |

---

# 2 - LoRa Message Types

LoRa packets follow a similar structure but with differences to leverage the higher bandwidth and bidirectional capability of LoRaWAN.

All LoRa packets share a **12-bit common header**:

| Bit Offset | Width | Field | Values |
|------------|-------|-------|--------|
| 0 | 2 | Packet type | `00`=GPS Single, `01`=GPS Multi, `10`=Sensor, `11`=Status |
| 2 | 3 | Flags | bit2=out_of_zone, bit1=low_battery, bit0=valid |
| 5 | 7 | Battery voltage | `(mV - 2700) / 20` |

## 2.1 - GPS Single (`0b00`)

Single GPS fix with full metadata.

| Bit Offset | Width | Field |
|------------|-------|-------|
| 0 | 12 | Header (type=00 + flags + voltage) |
| 12 | 4 | GPS count (=1) |
| 16 | 4 | Delta-time-loc code |
| 20 | 5 | Day |
| 25 | 5 | Hour |
| 30 | 6 | Minute |
| 36 | 21 | Latitude |
| 57 | 22 | Longitude |
| 79 | 7 | Speed |
| 86 | 8 | Heading |
| 94 | 8 | Altitude (255=invalid) |
| 102 | 4 | Num satellites (0-15) |

**Total: 106 bits (14 bytes)**

## 2.2 - GPS Multi (`0b01`)

Multiple GPS entries. First entry is full (86 bits), subsequent entries are delta (50 bits each: lat + lon + speed only).

| Bit Offset | Width | Field |
|------------|-------|-------|
| 0 | 12 | Header (type=01 + flags + voltage) |
| 12 | 4 | GPS count (N) |
| 16 | 4 | Delta-time-loc code |
| 20 | 86 | GPS[0] — full entry (day/hour/min/lat/lon/speed/heading/alt/numSV) |
| 106 | 50 | GPS[1] — delta (lat/lon/speed) |
| 156 | 50 | GPS[2] — delta |
| ... | 50 | GPS[N-1] — delta |

**Size: 106 + (N-1) * 50 bits.** Max entries depends on LoRa data rate (DR0: ~6 entries, DR3: ~38 entries).

## 2.3 - Sensor (`0b10`)

GPS + environmental sensors with a **presence bitmask** indicating which sensors are included.

| Bit Offset | Width | Field |
|------------|-------|-------|
| 0 | 12 | Header (type=10 + flags + voltage) |
| 12 | 6 | Sensor mask (6 bits, see below) |
| 18 | 86 | GPS entry (if mask bit 5 set) |
| ... | var | Sensor fields (in mask order) |

**Sensor mask bits:**

| Bit | Sensor | Field Width |
|-----|--------|-------------|
| 5 | GPS | 86 bits (full entry) |
| 4 | ALS | 17 bits |
| 3 | pH | 14 bits |
| 2 | Pressure | 29 bits (15 pressure + 14 temp) |
| 1 | Sea temp / Thermistor | 14 bits |
| 0 | Accelerometer | 67 bits (temp + X + Y + Z + activity) |

Sensors are packed in order from bit 5 down to bit 0 (GPS first, AXL last).

## 2.4 - Status (`0b11`)

Minimal heartbeat packet: header only (12 bits / 2 bytes). Sent when no GPS or sensor data is available.

| Bit Offset | Width | Field |
|------------|-------|-------|
| 0 | 2 | Type = `0b11` |
| 2 | 3 | Flags |
| 5 | 7 | Battery voltage |

---

# 3 - Depth Pile: Multi-Fix Aggregation

The **depth pile** is a FIFO queue (max 24 entries) that accumulates GPS fixes between transmissions. The `ARGOS_DEPTH_PILE` parameter controls how many fixes are packed per message.

## How It Works

1. Each GPS fix is pushed into the depth pile with a **burst counter** (number of times to retransmit)
2. At TX time, the depth pile is split into segments of up to 4 entries
3. Each segment becomes one Argos long packet
4. The retrieval cycles through segments round-robin

## Example

With `ARGOS_DEPTH_PILE=16` and 16 accumulated fixes:
- 4 segments of 4 fixes each
- Message 1: fixes [12-15] (most recent)
- Message 2: fixes [8-11]
- Message 3: fixes [4-7]
- Message 4: fixes [0-3] (oldest)

With `ARGOS_DEPTH_PILE=1` (underwater model):
- Each TX sends the latest fix only (short packet)
- Maximizes freshness during limited surface time

## Sensor Depth Pile

Sensor data uses separate depth piles per sensor type. At TX time:
- `retrieve_sensor_single()` returns the most recent cached value
- Sensor values are pre-encoded when received (offset + scale applied)
- If a sensor has no data, its field is omitted from the packet

---

# 4 - Surfacing Burst Mode (ARP01=5)

This mode combines Doppler and GNSS packets automatically during surface windows:

```
Surface detected
  ├── Phase 1: Doppler packets (3 bytes each)
  │   ├── TX at t=0s
  │   ├── TX at t=5s    (SURFACING_BURST_INIT_S)
  │   ├── TX at t=15s   (+10s step)
  │   ├── TX at t=25s   (+10s step)
  │   └── TX at t=35s... (capped at SURFACING_BURST_MAX_S)
  │
  ├── GPS fix acquired → switch to Phase 2
  │
  └── Phase 2: GNSS packets (short or sensor)
      ├── TX at TR_NOM interval (e.g., every 60s)
      └── Until dive detected → all TX stops
```

The receiver sees a mix of 3-byte Doppler packets and 12+ byte GNSS packets from the same device during a single surfacing event.

---

# 5 - Quick Reference: Decode All Sensor Values

| Sensor | Bits | Encoded As | Decode to Physical | Range |
|--------|------|------------|-------------------|-------|
| ALS | 17 | Raw ADC | `raw` lumens | 0-131071 |
| pH | 14 | `pH * 1000` | `raw / 1000` | 0.000-16.383 |
| Pressure | 15 | `hPa * 1000` | `raw / 1000` hPa | 0-32.767 hPa |
| Pressure temp | 14 | `(°C + 40) * 100` | `raw / 100 - 40` °C | -40 to +123.83 °C |
| Sea temperature | 21 | `(°C + 126) * 1000` | `raw / 1000 - 126` °C | -126 to +1972 °C |
| Thermistor | 14 | `(°C + 40) * 100` | `raw / 100 - 40` °C | -40 to +123.83 °C |
| AXL temperature | 14 | `(°C + 40) * 100` | `raw / 100 - 40` °C | -40 to +123.83 °C |
| AXL axis (X/Y/Z) | 15 | `(g + g_range) * 1000` | `raw / 1000 - g_range` g | ±g_range |
| AXL activity | 8 | Raw | `raw` | 0-255 |
| Mortality | 7 | Raw percentage | `raw` % | 0-100 |
| Battery voltage | 7 | `(mV - 2700) / 20` | `raw * 20 + 2700` mV | 2700-5240 mV |

---

# 6 - Implementation Reference

| Purpose | File |
|---------|------|
| Argos packet builder | `core/services/argos_tx_service.hpp` / `.cpp` |
| LoRa packet builder | `core/services/lora_tx_service.hpp` / `.cpp` |
| Bit packing utility | `core/util/bitpack.hpp` |
| Depth pile template | `core/services/depth_pile.hpp` |
| Coordinate/value conversion | `argos_tx_service.cpp:478-506` |
| Sensor pre-encoding | `argos_tx_service.cpp:1146-1193` (DepthPileManager) |
| Type enums | `core/protocol/base_types.hpp` |
