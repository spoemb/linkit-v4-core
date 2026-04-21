# core/sm/

Top-level state machines, built on [tinyfsm](tinyfsm.hpp).

## Files

| File | Role |
|------|------|
| `gentracker.{cpp,hpp}` | Main FSM: boot → idle → operational → off, plus DFU and bridge sub-states |
| `buzzm.{cpp,hpp}` | Buzzer FSM (deployment/error patterns) |
| `ledsm.{cpp,hpp}` | RGB LED FSM (status indication, GPS fix, TX in progress) |
| `error.hpp` | Error codes raised across the codebase |
| `tinyfsm.hpp` | Header-only FSM library (do not modify) |

## Adding a new top-level state

Modify `gentracker.cpp`:
1. Define the new state class (inherit from `GenTracker`).
2. Implement `entry()` / `exit()` / `react()` for the events you handle.
3. Wire transitions from existing states with `transit<NewState>()`.

Be careful with services: start them in `entry()`, stop them in `exit()`, and never let two states own the same service simultaneously.
