# core/util/

Pure helper functions — no I/O, no global state, easily unit-testable.

| File | Purpose |
|------|---------|
| `bch.hpp` | BCH error-correction code (Argos packet) |
| `crc8.hpp`, `crc16.hpp`, `crc32.hpp` | CRC helpers |
| `binascii.hpp` | Hex ↔ binary conversion |
| `base64.hpp` | (also in `protocol/`, kept for util-only consumers) |
| `bitpack.hpp` | Big-endian bit packing for satellite payloads |
| `haversine.{cpp,hpp}` | Great-circle distance |
| `time_utils.hpp` | Date/time helpers (epoch ↔ calendar) |
| `nrf_assert.h` | Assert macro shim for the SDK |

If you need a small, self-contained utility, add it here. Anything that touches hardware or a service belongs elsewhere.
