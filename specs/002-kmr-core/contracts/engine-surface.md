# Contract: the engine surface other crates may name

Everything else in `kmr_core` is `pub(crate)` (FR-056 (SRS NFR-3)). Controllers see none of this
directly; they go through `kmr_api` (SRS Definitions "API"). "M" is the milestone that lands it.

## Builder (robot.rs)

```rust
pub struct Robot<const N: usize, CS = Scheduled, E = Empty>;              // N: M2, E: M3
impl<const N: usize> Robot<N> { pub fn new() -> Self; }
impl<const N: usize, CS, E> Robot<N, CS, E> {
    pub fn set_dt(self, dt: Duration) -> Self;                             // range checked at run (M3)
    pub fn set_hold(self, hold: Option<Duration>) -> Self;                 // M4
    pub fn set_limits(self, limits: [Limit; N], policy: [Option<Policy>; N]) -> Self; // M5
    pub fn set_envelopes(self, envelopes: [Envelope; N]) -> Self;           // M5
    pub fn add_control_fn<S, C>(self, s: S, c: C) -> Robot<N, CS::Output, Filled>;
    pub fn add_control_fn_with<S, T, C>(self, s: S, ctx: T, c: C) -> Robot<N, CS::Output, Filled>;
    pub fn add_control_fn_as_thread<S, T, C>(self, s: S, ctx: T, c: C) -> Robot<N, CS::Output, Filled>;
    pub fn run(self) -> Result<(), RunError<Infallible>> where E: HasController, CS: Drive<N>;
    pub fn run_on<A: Hal<N>>(self, hal: A) -> Result<(), RunError<A::Error>>
        where E: HasController, CS: Drive<N>;                              // M1 (A::Error), M3 (RunError)
    pub fn run_on_staged<A: Hal<N>, St: Stages<N>>(self, hal: A, st: St)
        -> Result<(), RunError<A::Error>> where E: HasController, CS: Drive<N>; // M5
}
```

- `run()` is `run_on(NoHardware)`. `run_on(&mut adapter)` hands the adapter back (FR-050
  (SRS FR-5.9)). Both signals return `Ok(())` (FR-038).
- 2026-10-29 (D17, plan KS.1): after M1, `run_on` and `run_on_staged` take the list of drivers
  (`D: Drivers`, an HList of `Hal<N>` members) instead of `A: Hal<N>`, and `run()` is
  `run_on(hlist![NoHardware])`. See [design-hal-set.md](../design-hal-set.md).
- `Limit { lower: Option<Q>, upper: Option<Q>, speed: Qd, effort: Tau }`: typed, never plain
  `f32`; `None` = no position bound (continuous joint), never clamped to 0. `Envelope { q:
  Option<(Q, Q)>, qd: Option<(Qd, Qd)>, tau: Option<(Tau, Tau)> }`. `Policy { Clamp, Warn,
  Abort }`. A limited actuator without a policy is `RunError::Config(NoPolicy)` (D2 B, assumed).
- An empty robot fails to compile: `error[E0277]: kmr: this robot has no controller`
  (FR-009 (SRS NFR-4); proved, research R8). `HasController` is sealed.
- Until M2, the signatures read `Robot<CS>` with N = 4; until M3, `Result<(), A::Error>`.

## State handle (state/robot_state.rs)

Generic accessors only; `kmr_api` adds the plain ones (FR-024 (SRS Definitions "State")).

```rust
pub fn set_at<T: StateField>(&mut self, value: T, index: usize) -> Result<(), StateError>;
pub fn set_all<T: StateField>(&mut self, values: [T; N]);
pub fn current<T: StateField>(&self) -> Result<&[T; N], StateError>;
pub fn prev<T: StateField>(&self, depth: usize) -> Result<&[T; N], StateError>;
pub fn age(&self, index: usize) -> Result<Option<u32>, StateError>;          // M3
pub fn sample_age(&self) -> Option<u32>;                                       // M3, oldest actuator
pub fn faults(&self, depth: usize) -> Result<&Faults<N>, StateError>;        // M4
pub fn tick_flags(&self, depth: usize) -> Result<TickFlags, StateError>;      // M4
pub fn cadence(&self, index: usize) -> Result<NonZeroU32, StateError>;        // M3, from Layout
pub fn telemetry(&self) -> &Telemetry<N>;                                      // SRS FR-5.11
```

## Telemetry (telemetry.rs, SRS FR-5.11, 2026-11-05)

```rust
pub struct Telemetry<const N: usize>;                  // private fields; Clone, Copy, Debug, PartialEq, Default
impl<const N: usize> Telemetry<N> {
    pub fn temperature_c(&self, index: usize) -> Result<Option<f32>, StateError>;
    pub fn voltage_v(&self, index: usize) -> Result<Option<f32>, StateError>;
    pub fn hardware_error(&self, index: usize) -> Result<Option<u8>, StateError>;
    pub fn age(&self, index: usize) -> Result<Option<u64>, StateError>;        // ticks
    pub fn set_temperature_c(&mut self, index: usize, celsius: f32);           // adapter side;
    pub fn set_voltage_v(&mut self, index: usize, volts: f32);                 // index past N
    pub fn set_hardware_error(&mut self, index: usize, bits: u8);              // ignored
}
```

Not a `StateField`, never in Desired or History. Controllers get `&Telemetry` only, so the
setters are the adapters' (`Hal::telemetry`).

No getter on Desired, no setter on History, no field access (SC-008 (SRS NFR-3)).

## Values and fields (state/)

`Q`, `Qd`, `Tau`, `Kp`, `Kd` (M3 for the last two): `repr(transparent)` `f32`, `From` both ways,
the arithmetic of `derive_more`. `StateField` (sealed by `sealed::Slot` in **`state/field.rs`,
invariant**), re-exported by `kmr_api` as `Field`. `HISTORY_DEPTH`.

## Time (clock.rs, M1)

`elapsed() -> Duration`, `dt() -> Duration`, `tick() -> u64`, `overrun_count() -> u64`,
`last_overrun() -> Option<Duration>`, `accumulated_overrun() -> Duration`. Read-only, by value
(FR-029 (SRS FR-3.3)).

## Signals (signal.rs, invariant, unchanged)

`shutdown!(reason)`, `emergency_stop!(reason)`: any depth, any thread, no heap, no lock,
async-signal-safe (FR-036 (SRS FR-4.4)). Foreign code reaches them through a Rust forwarder in
`kmr_ffi` (D13 A); no new entry.

## Hidden entries for `kmr_api` (`#[doc(hidden)] pub`, never re-exported)

- `ExternalFlags::raise(bits)`: one `fetch_or` (FR-052 (SRS FR-13.4)), M4.
- `Stages<const N>`: `inbound(&mut self, &mut RobotState<N>, &Time)` after the read and before
  `First`; `outbound(&mut self, &RobotState<N>, &Time)` after the write, so depth 0, its flags
  and the per-actuator age are all reachable through the read-only accessors. `impl for ()`
  (FR-053 (SRS §4, FR-7.1)), M5.
- `Host` (M6): `now`, `sleep_until`, `poll`; public and sealed, for a `no_std` target port only
  (FR-058 (SRS NFR-7)).

## Errors (error.rs)

`RunError<E> { Config(ConfigError), Hal(E) }`; `ConfigError { DtOutOfRange { dt }, NoPolicy
{ actuator } }`; `StateError { OutOfRange { len }, NoSample { depth }, Undecided(&'static str) }`.
Human-readable `Display`; release builds hide internal causes (FR-055 (SRS NFR-2)).

## For `kmr_adapters`

The whole of [hal.md](hal.md): `Hal<N>`, `seal::Adapter`, `Mask<N>`, `Written<N>`, `Faults<N>`,
`Layout<N>`, `Group`, `Overrides`, `Command`, `Sample`, `Batch`, `Runs`, `NoHardware`,
`telemetry::Telemetry<N>`.

## For `kmr_ffi`

Value types as `f32`, `Mask<N>` words and `TickFlags` as plain integers, `Time` readers as
`Duration`, age as `Option<u32>` per actuator; all through `kmr_api`. `FieldId` (M7, K7.1):
`pub enum FieldId { Q, Qd, Tau }` (+ `Kp`, `Kd` after M3), or `const ID: FieldId` on the
sealed `Slot`, in **`state/field.rs` (invariant)**.
