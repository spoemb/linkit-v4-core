# ports/linux/

Host (Linux) port. Used by the unit test suite ([`../../tests/`](../../tests/)) and for prototyping platform-independent code without flashing hardware.

| File | Role |
|------|------|
| `main.cpp` | Host entry point |
| `pmu.hpp` | Linux-side PMU stub (timestamps via `clock_gettime`, no real low-power) |
| `linux_timer.hpp` | Posix timer implementation of the timer abstraction |
| `interrupt_lock.cpp` | No-op lock (single-threaded host) |

Most host-side stubs live alongside the tests in [`../../tests/fakes/`](../../tests/fakes/) — this folder only contains the bits that are not test-specific.
