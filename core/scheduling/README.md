# core/scheduling/

Cooperative scheduler and TX timing.

## Files

| File | Role |
|------|------|
| `scheduler.hpp`, `service_scheduler.hpp` | Generic cooperative scheduler primitives |
| `argos_tx_scheduler.{cpp,hpp}` | Argos TX timing: prepass + duty cycle + cooldown |
| `lora_tx_scheduler.{cpp,hpp}` | LoRa TX timing: 4 trigger modes (parity with Argos) |
| `interrupt_lock.hpp` | RAII lock used by services that touch shared state from ISR context |

## Notes

- The scheduler is single-threaded and cooperative — services must not block.
- Long operations should yield via `run_state_machine(delay_ms)` or break work across ticks.
- TX schedulers compute the **next allowed TX time**; the calling service decides whether to actually transmit (e.g. only when surfaced).
