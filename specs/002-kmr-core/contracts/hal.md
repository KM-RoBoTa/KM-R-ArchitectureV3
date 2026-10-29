# Contract: the hardware contract, engine side

The trait is `kmr_core::hal::Hal` (human-only file). This page says what the engine promises an
adapter and what it expects back. The adapter side is
`specs/001-kmr-adapters/contracts/hal-conformance.md`. FR-039 to FR-047 (SRS FR-5.1 to FR-5.8,
section 7 HAL).

**2026-10-29 (D17, plan KS.1).** After M1 the trait below describes one **member** of the robot's
hardware, not all of it. `check` returns the member's `Claim` (owned indices, their cadence and
local groups, costs) instead of a whole-N `Layout`; the engine merges the claims, checks that they
cover `0..N` exactly once (FR-063 (SRS FR-5.1, FR-6.4)) and fans every call below out to every
member, `torque_off` and `close` in reverse order. See [design-hal-set.md](../design-hal-set.md).

## Shape

```rust
pub trait Hal<const N: usize>: seal::Adapter {          // `Hal` with N = 4 until K2.5
    type Error: core::error::Error + 'static;
    fn check(&mut self, overrides: Overrides) -> Result<Layout<N>, Self::Error>;
    fn open(&mut self) -> Result<(), Self::Error>;
    fn zero(&mut self) -> Result<(), Self::Error>;
    fn ping(&mut self) -> Result<(), Self::Error>;
    fn health(&mut self) -> Result<(), Self::Error>;
    fn configure(&mut self) -> Result<(), Self::Error>;
    fn read_all(&mut self, sample: Sample<'_, N>) -> Result<(), Self::Error>;
    fn torque_on(&mut self) -> Result<(), Self::Error>;
    fn read(&mut self, sample: Sample<'_, N>, faults: &mut Faults<N>) -> Mask<N>;
    fn write(&mut self, command: Command<'_, N>, batch: &Batch<'_, N>, faults: &mut Faults<N>);
    fn hold(&mut self, target: Command<'_, N>);
    fn torque_off(&mut self);
    fn close(&mut self);
}
impl<A: Hal<N>, const N: usize> Hal<N> for &mut A { /* forwards; K1.6 */ }
```

`seal::Adapter` is `#[doc(hidden)] pub`: adapter crates implement it, `controllers` does not by
convention and review (FR-040 (SRS FR-5.1)). Static dispatch only (FR-010 (SRS FR-11.3)).

## Call order the engine guarantees

| Moment | Calls | On `Err` |
|---|---|---|
| finalization | `check(Overrides::default())` | `run_on` returns `RunError::Hal`, no other call |
| Init, before user `Init` | guard built, `open`, `zero` | `torque_off`, `close`, return |
| PostInit, before user `PostInit` | `ping`, `health`, `configure`, `read_all` (into a pushed slot), estop check (K1.9), `torque_on` | `torque_off`, `close`, return |
| tick, after `begin_tick` | slot pushed (copy of `prev(0)`), `read` | faults only |
| tick, after `Last` and the limit stage | `write` if the batch is not empty | faults only |
| shutdown | `hold(target)`, wait if `hold` duration set, then guard | – |
| emergency stop | guard | – |
| guard drop (any exit after `open`, unwind included) | `torque_off`, `close` | – |

Promises:

1. `check` is the only call before `open`, and does no I/O.
2. After `open` was called, even if it failed, `torque_off` then `close` are called exactly once
   each, last.
3. `read` gets a slot already holding `prev(0)`'s values: an adapter writes only what it read.
4. `write` gets only actuators with a set bit in some field's mask, ordered by `Layout` groups.
   A field whose bit is clear carries a stale value the adapter must not send.
5. `hold` gets the latest sample overlaid with this tick's writes; `written` marks the overlaid
   values (OC1). Never cleared Desired (no zeros).
6. No call happens inside a controller, and no call is concurrent with another.
7. A `Some(hold)` wait is cut by an emergency stop at the next `dt` slice.

## What the engine expects

1. `torque_off` and `close` never panic, are idempotent, and are safe before or after a failed
   `open`. They run inside `Drop`, during unwinding too.
2. `read` and `write` do not allocate, lock, sleep or retry (SRS NFR-1, FR-5.10).
3. `read` returns the mask of actuators refreshed this tick (age, `ever_read`, K3.5).
4. A failed read leaves the value as it was and sets `stale` (FR-022 (SRS FR-2.6)).
5. A written target it cannot send (torque off by choice, field not taken by the mode, out of
   range, non-finite) is not sent and sets `rejected` (FR-047 (SRS FR-5.8), research R16).
6. `Layout` holds the per-actuator cadence as a whole number of ticks (FR-045 (SRS FR-5.3)) and
   the (bus, series) group of each actuator (FR-044).
7. At most 64 actuators (D4).

## `Overrides`

Empty and `#[non_exhaustive]` after K3.8 (research R9). It carries no cadence and no baud. Kept
so a run-time override can be added without a breaking change.

## `NoHardware`

`impl<const N: usize> Hal<N>`, `Error = Infallible`. `check` returns one group, cadence 1.
`open` prints "no bus plugin: simulating" once, the only such warning (FR-042 (SRS FR-6.3));
lands with K1.6.
`read` refreshes nothing, so History reads return `NoSample` (research R6, pending human).
