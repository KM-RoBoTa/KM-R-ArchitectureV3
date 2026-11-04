# 0008 — Empty robot: a compile error by type-state; one `RunError` for `run()`

Status: **proposed**, drafted by an agent under the maintainer's goal of
2026-11-03, waiting for the maintainer's sign-off at review. Built by core
T034 (`RunError`) and T035 (type-state), with kmr_api T040. The research is
[`specs/002-kmr-core/research.md`](../../specs/002-kmr-core/research.md) R8
and R9; the contract this sits next to is
[0002](0002-hal-contract.md).

## The question

Two things `run()` must say before any hardware is touched:

1. **A robot with no controller.** It would power the actuators and command
   nothing. AGENTS.md defended a run-time check (`L::LEN == 0` →
   `ApiError::NoControllers`), because a trait bound leaked
   `frunk_core::...` paths into the user's E0277. SRS NFR-4 and section 8
   decide for a compile error.
2. **What `run()` returns on failure.** Today `run_on` returns the driver
   set's `HardwareError`, and a bad `dt` is caught only by an interim check
   in `kmr_api`. Configuration faults (a `dt` out of range, a missing
   joint-limit policy) are not hardware errors and need a home of their own.

## Current state

| Item | Today |
|---|---|
| Empty robot | not checked at all: `ApiError::NoControllers` exists and is never returned |
| `run_on` | `Result<(), HardwareError<D::Error>>` |
| `dt` range | zero or unset falls back to 1 ms with a warning in the core; `kmr_api` rejects outside [10 µs, 1 s] (`UserError::DtOutOfRange`) |
| `Overrides` | still carries `read_every` |

## Decision

**Empty robot, by type-state.** `Robot<const N, CS = Scheduled, E = Empty>`.
Every `add_*` returns `Robot<N, _, Filled>`. `run()` and `run_on()` require
`E: HasController`, a sealed trait carrying
`#[diagnostic::on_unimplemented]`. The user's `Robot::new().run()` then
fails under `cargo check` with the core's own words:

```text
error[E0277]: kmr: this robot has no controller
  |     Robot::new().run().ok();
  |                  ^^^ `run()` needs at least one controller
  = note: add one with `.add_controller(EachTick, f)` before `.run()` (SRS NFR-4)
```

`schedule.rs` is not touched: the marker is one parameter beside the
schedules, not a bound over six HList buckets. `ApiError::NoControllers`
goes.

**One error for `run()`.**

```rust
pub enum RunError<E> {
    Config(ConfigError),
    Hal(E),
}
pub enum ConfigError {
    DtOutOfRange { dt: Duration },
    NoPolicy { actuator: usize },   // with the joint-limit policy, core T051 (D2)
}
```

`run_on` returns `Result<(), RunError<HardwareError<D::Error>>>`; on
`NoHardware` the error is `RunError<Infallible>`: only `Config` can happen. The `dt`
range check moves into the core, before `claim`, so nothing is opened for a
bad `dt`. `ApiError` goes. `ZeroReadCadence` is not added: the cadence is in
the hardware table, and `Overrides` ends empty
([0002](0002-hal-contract.md)).

## Defence

### Real-time performance

No cost. The marker is a zero-sized type parameter resolved at compile
time; the configuration checks run once, at finalization, before any I/O.
Nothing is added to the tick.

### Quality

- **Wrong programs do not build.** An empty robot is a mistake the compiler
  can see, so it is a compile error, not a run-time branch every binary
  carries.
- **The message is the core's.** `on_unimplemented` puts a readable line
  first; the internal type paths that justified the run-time check stay
  below it, as notes.
- **Configuration and hardware are told apart.** A user matching on the
  error knows whether to fix `main.rs` or the wiring. The adapter's error
  stays typed inside `Hal`.
- **Proved** in a scratch prototype (core, api and user crates, rustc
  1.97.1): the empty user robot fails under `cargo check` with the message
  above, through a `kmr_api`-style wrapper, next to a const generic N; the
  non-empty build has no warning.

### Developer experience

Users never name `E`: `robot!` emits `Robot<N>`, and the builder chain
infers the rest. The one visible change is the error type of `run()`, which
`kmr_api` maps into its own `UserError` with a fix in each message (SRS
NFR-4).

### Alternatives rejected

- **The run-time `L::LEN == 0` check.** The SRS overrules it, and it fails
  only when the binary runs, possibly on the robot.
- **A bound over the schedule buckets.** Needs a type-level OR across six
  HLists and an edit of the HList machinery, for the same result.
- **Keep `HardwareError` as `run()`'s error and add configuration variants
  to it.** Mixes two causes in a type owned by the driver set.

## Consequences

- `kmr_api` threads one more parameter through its `Robot` wrapper and the
  `robot!` aliases, drops its `NoControllers` path, and maps `RunError` into
  `UserError` (kmr_api T040).
- A compile-fail doctest pins the empty-robot message (core T058).
- AGENTS.md's paragraph on the runtime `L::LEN == 0` check is rewritten when
  T035 lands (core T056).
