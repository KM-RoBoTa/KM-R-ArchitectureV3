# Contract: how the Dynamixel adapter answers the HAL

The contract is `kmr_core/src/hal.rs` (human-only, the authority). This page says what the
Dynamixel adapter does on each call, so the core's `run_on` and `Powered` guard
(`specs/002-kmr-core/design-first-motor.md`) can rely on it. FR-001 (SRS FR-5.1, section 7 HAL).

2026-10-29 (D17, plan E11): after core KS.1 the adapter is one member of the robot's driver
list. `check` then returns its `Claim` (its table's indices, groups per (bus, series), cadence,
costs), not a whole-N layout, and every row below touches only owned indices (FR-036).
See `specs/002-kmr-core/design-hal-set.md`.

## Per call

| Call | Phase | Dynamixel does | Errors (`DynamixelError`) | I/O |
|---|---|---|---|---|
| `check(overrides)` | `run()` finalization | builds groups per (bus, series); cadence = the bus's `read_every`, else `DEFAULT_READ_EVERY`; `overrides` is ignored (core K3.8 removes `Overrides::read_every`; it carries nothing); warns once per Protocol 1 actuator that `Qd`, `Tau` are never read; declares costs | `TooManyBuses` | none |
| `open` | Init | resolves every port (`ById` under `/dev/serial/by-id`, `ByDev` as given), pre-flights each, refuses a shared device, then flushes and sets the latency timer on each, then opens all at once | `PortNotFound`, `PortAccess`, `SharedPort`, `PortFlush`, `LatencyTimer`, `PortOpen`, `Sdk` | ports only |
| `zero` | Init | nothing: absolute encoders; the zero is a software offset (D8) | none | none |
| `ping` | PostInit | up to 3 pings per actuator, no sleep | `Missing`, `Actuator` | yes |
| `health` | PostInit | Protocol 2: error status, voltage, temperature against the model; Protocol 1: one warning, skipped (step E2) | `Health`, `Actuator` | yes |
| `configure` | PostInit | Protocol 2: operating mode must be position; a powered actuator is warned, never an error | `Setup`, `Actuator` | yes |
| `read_all(sample)` | PostInit | every group whatever the cadence; seed positions must be inside the register range | `Actuator`, `Backend`, `Health` | yes |
| `torque_on` | PostInit | `Torque::On`: torque on; `Torque::ReadOnly`: `force_torque_off` | `Actuator` | yes |
| `read(sample, faults)` | tick, before `First` | groups whose bus is due; fills only what it read; returns the refreshed mask | — (bits) | yes |
| `write(command, batch, faults)` | tick, after `Last` | per run: the mode's field when its `written` bit is set; one sync write per run | — (bits) | yes |
| `hold(last)` | shutdown | `last.q` to every powered position actuator, one sync write per group; `last.written` ignored | — (logged) | yes |
| `torque_off` | every exit after `open` | `force_torque_off` on every actuator of every opened bus, read-only ones included; a failure is logged and the loop goes on | — (logged) | yes |
| `close` | last | flushes every resolved port, closes all | — (logged) | ports only |

Velocity and current modes never reach `write`: `Table::check` refuses them (FR-018 (SRS §8)).
When they open, `hold` sends `Qd(0)` or `Tau(0)` for them (FR-016 (SRS FR-4.1)).

## Guarantees the core relies on

1. **Never panics** (FR-033 (SRS NFR-2)). No `unwrap`, no unchecked index, no overflowing
   arithmetic outside tests. Required because `torque_off` and `close` run inside `Drop`, during
   unwinding too.
2. **`torque_off` before or after a failed `open` is a no-op on the bus.** It checks `open`.
3. **`close` is always safe**, opened or not, and may follow `torque_off` in any state.
4. **`torque_off` and `close` are idempotent.** A second call sends nothing harmful.
5. **No retry, sleep, allocation, lock or `dyn` on `read`, `write`** outside SDK call sites
   (FR-013 (SRS NFR-1, FR-5.10)). Retries only in `ping` (FR-007 (SRS FR-5.10)).
6. **The hold command is right with either hold target** of the core (latest sample, or sample
   overlaid with this tick's writes): position mode sends `q`.
7. **The error type** names the actuator index, bus label, firmware id and port where known
   (FR-007 (SRS FR-6.4)). It is `Copy`, has no heap, and implements `core::error::Error`.

## Exit paths (as the adapter sees them)

| Exit | Calls after the failure point |
|---|---|
| `bind` error (before `run_on`) | none: the adapter does not exist yet |
| `check` error | none |
| `open` error | `torque_off` (no-op), `close` |
| `zero` .. `torque_on` error | `torque_off`, `close` |
| shutdown | `hold`, `torque_off`, `close` |
| emergency stop | `torque_off`, `close` |
| panic in a controller | `torque_off`, `close`, during unwind |
| SIGKILL, SIGQUIT, abort | nothing: torque stays as it was (SRS NFR-2; bus watchdog is an SRS gap) |
