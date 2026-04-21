# ports/nrf52840/core/memmang/

Custom heap and dynamic-memory bindings for the embedded target.

| File | Role |
|------|------|
| `heap_4.c`, `heap.h` | FreeRTOS-derived `heap_4` allocator (best-fit + coalesce) |
| `heap_mem.c` | Heap region declaration (linker symbols) |
| `bindings.c`, `bindings.cpp` | `malloc`/`free` and `new`/`delete` overrides routed to `heap_4` |
| `memmang.{cpp,hpp}` | Memory-management facade |

The default newlib allocator is replaced because:
- `heap_4` is deterministic and matches the firmware's allocation patterns.
- We need accurate heap usage tracking for the watchdog / diagnostics.

If you suspect a leak or fragmentation issue, the `MemoryAccess` interface in [`../../../../core/hardware/`](../../../../core/hardware/) exposes free-bytes / largest-block stats.
