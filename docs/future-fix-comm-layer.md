# Future Fix: Communication Layer ISR Safety

## Status: Deferred
**Date**: 2026-03-03
**Affects**: LoRa RAK3172, KIM2, GPS (all UART comm layers)
**Severity**: LOW probability, HIGH impact
**Priority**: Fix if unexplained crashes are observed in field

---

## Issue 1: send_AT() Busy-Wait Polling

### Description

`send_AT()` in `lora_rak3172.cpp` (and the equivalent in `kim2.cpp`) uses a
polling loop with `PMU::delay_ms(1)` to wait for the module response. This
busy-waits on the DWT cycle counter, blocking the cooperative scheduler.

### Call Chain

```
send_AT(cmd)
  └→ m_lora_comm.send(cmd, params)
  └→ while (!m_cmd_is_ok && !m_is_error && timeout--)
       PMU::delay_ms(1)
         └→ nrf_delay_ms(1)
             └→ nrf_delay_us(1000)
                 └→ nrfx_coredep_delay_us()  // DWT->CYCCNT busy-wait
```

### Why It Works

Interrupts are NOT disabled during the busy-wait. The UART RX IRQ fires
normally, `handle_rx_buffer()` processes the response, sets `m_cmd_is_ok=true`,
and the polling loop exits. Typical latency: ~200ms per AT command.

### Impact

- Scheduler blocked ~200ms per AT command
- Full configure sequence: ~2.6s total (13 commands × 200ms)
- GPS, BLE, sensor services starved during this time
- BUT: chunks are 200ms with 50-100ms scheduler gaps between commands

### Proper Fix

Replace polling with an event-driven async pattern:

```cpp
// Instead of blocking:
bool send_AT(ATCmd cmd) {
    m_pending_cmd = cmd;
    m_lora_comm.send(cmd, params);
    initiate_timeout(1000);
    // Return immediately - response handled in react() callbacks
}

void react(const LoRaCommEventRespOk&) {
    cancel_timeout();
    m_cmd_is_ok = true;
    run_state_machine(0);  // Continue state machine in scheduler
}
```

This requires refactoring the entire state machine to be fully event-driven
instead of synchronous. The configure sequence would become a chain of
send → react(OK) → send next → react(OK) → ... without ever blocking.

### Estimated Effort

- LoRa: ~2 days (state_configure has 13 sequential steps)
- KIM2: ~3 days (more complex state machine)
- Testing: ~2 days (regression + timing validation)

---

## Issue 2: Heap Allocation in UART ISR Context

### Description

`handle_rx_buffer()` is called directly from the UART hardware IRQ (no deferral).
It uses `std::string` operations (`append`, `substr`, `erase`, copy constructor)
which may trigger heap allocation via `pvPortMalloc`.

### Call Chain

```
UART hardware IRQ (UARTE1_IRQHandler)
  → irq_handler()                          [nrf_libuarte_drv.c]
  → uart_evt_handler()                     [nrf_libuarte_async.c]
  → lora_nrf_libuarte_async_evt_handler()  [lora_rak3172_comm.cpp]
  → LoRaComm::handle_rx_buffer()           ← ISR CONTEXT
    → std::string::append()                ← potential heap alloc
    → std::string::substr()                ← heap alloc (new string)
    → std::string::erase()                 ← memmove, no alloc
    → parse_rx_line()
      → std::string trimmed = line;        ← heap alloc (copy)
      → m_last_value = trimmed;            ← potential heap alloc
```

### Locking Mechanism

`pvPortMalloc()` (heap_4.c) uses `vTaskSuspendAll()` / `xTaskResumeAll()` for
synchronization. This prevents task switching but **does NOT disable interrupts**.

### Reentrancy Risk

```
Main thread: std::string operation → pvPortMalloc()
  → vTaskSuspendAll()
  → traversing free list...        ←── UART IRQ fires here
    │
    │ ISR: handle_rx_buffer()
    │   → std::string::append()
    │     → pvPortMalloc()          ←── REENTRANCY: free list corrupted
    │
  → free list now inconsistent
  → crash or silent memory corruption
```

### Mitigating Factors

1. **Critical window is ~µs**: pvPortMalloc manipulates the free list for only
   a few microseconds. UART IRQ fires ~10-50 times/sec. Collision probability
   is extremely low.

2. **SSO (Small String Optimization)**: std::string on ARM GCC keeps strings
   ≤15 chars on the stack. "OK", "AT_ERROR", "+EVT:JOINED" etc. fit in SSO —
   no heap allocation for most responses.

3. **Pre-existing pattern**: KIM2 (`kim2_comm.cpp`) and GPS have used the same
   pattern for years without observed heap corruption in the field.

4. **Partial mitigation applied**: Buffers are now pre-reserved in `init()` and
   string copies minimized in the ISR path (see hardening commit).

### Proper Fix

Replace std::string in ISR with a static ring buffer:

```cpp
// ISR-safe: zero allocation
class LoRaComm {
    // Fixed-size ring buffer for ISR use
    uint8_t  m_ring_buf[256];
    volatile uint16_t m_ring_head;  // Written by ISR
    volatile uint16_t m_ring_tail;  // Read by main thread

    // Called from UART ISR - no heap allocation
    void handle_rx_buffer(uint8_t* buffer, uint8_t length) {
        for (uint8_t i = 0; i < length; i++) {
            uint16_t next = (m_ring_head + 1) % sizeof(m_ring_buf);
            if (next == m_ring_tail) break;  // Full - drop byte
            m_ring_buf[m_ring_head] = buffer[i];
            m_ring_head = next;
        }
        nrf_libuarte_async_rx_free(...);
    }

    // Called from scheduler - safe to use std::string
    void process_rx() {
        while (m_ring_tail != m_ring_head) {
            char c = m_ring_buf[m_ring_tail];
            m_ring_tail = (m_ring_tail + 1) % sizeof(m_ring_buf);
            m_rx_buffer += c;
            if (c == '\n') {
                parse_and_dispatch(m_rx_buffer);
                m_rx_buffer.clear();
            }
        }
    }
};
```

This completely eliminates heap allocation in ISR context. `process_rx()` would
be called from the state machine or a periodic scheduler task.

### Estimated Effort

- LoRa: ~1 day (straightforward, new code)
- KIM2: ~2 days (different response format, more complex parsing)
- GPS: ~2 days (binary protocol, high data rate at 460800 baud)
- Testing: ~2 days

---

## Decision Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-03-03 | Defer full fix, apply hardening only | Risk is theoretical, same pattern runs in KIM2 for years. Hardening (pre-reserve buffers, minimize copies) reduces probability further without major refactoring. |

## Hardening Applied (2026-03-03)

The following low-risk changes were applied to reduce heap allocation in ISR:

1. **`constexpr std::string_view` for response constants** — eliminates
   per-TU heap-allocated static `std::string` objects
2. **`reserve()` on all string buffers in `init()`** — ensures `append()`
   reuses existing capacity instead of reallocating
3. **Reuse member `m_line_buffer`** instead of allocating a new `std::string`
   per line in handle_rx_buffer
4. **In-place trimming** in `parse_rx_line()` — no copy of the line string
5. **Direct `substr` assignment to `m_last_value`** — avoids intermediate
   string for RX payload extraction
