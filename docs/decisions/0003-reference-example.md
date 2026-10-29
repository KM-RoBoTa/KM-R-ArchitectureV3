# 0003 — The reference example: what `controllers/` ships

Status: **proposed**, waiting for the maintainer's decision. One point rests on
a pending decision (D15, see [Pending](#pending)).

The research behind each point is in
[`specs/005-controllers/research.md`](../../specs/005-controllers/research.md)
(R1, R3, R4, R8).

## The question

`controllers/src/main.rs` is the file a new user copies, and the controller of
delivery step 1 (D1 A). Four choices shape it:

- whether the crate pulls the vendor SDK by default;
- what it moves, and how it ends;
- whether it ships a bus line;
- whether it holds tests.

## Current state

| Piece | State |
|---|---|
| `kmr_api::robot!(XmBench)` in `main.rs` | written |
| `hardware` feature in `controllers/Cargo.toml` | written, off by default |
| The swing controller | not written: waits on `Time::elapsed()` (core K1.1, human) |
| Bus line | none |
| Test module | none |

Until the swing lands, `main.rs` nudges actuator 0 every tick and runs until
Ctrl-C.

## Decision proposed

1. **Feature off by default.** `[features] hardware = ["kmr_api/dynamixel"]`.
   The bench command is `cargo run -p controllers --features hardware --release`.
2. **A bounded swing that ends itself.** Per actuator,
   `target = start + Q::from(Deg(5.0 · sin(2π · s / 4 s)))`, with
   `s = elapsed() − t0` and `t0` the `elapsed()` at which `start` was read.
   `AMPLITUDE = Deg(5.0)`, `PERIOD = 4 s`, `DEMO_LENGTH = 12 s`, then
   `shutdown!`.
3. **No bus line as shipped.** The walkthrough adds one line,
   `.add_plugins(Bus::by_id("<serial>"))`.
4. **No test module in `main.rs`.**

## Defence

### Real-time performance

- The swing costs one `sin` per actuator per tick, in `f32`, with no heap,
  lock, sleep or I/O. Its context is fixed in size and owned by the engine.
- It is a function of time, not of the call count. After an overrun the
  target moves along the same curve; a 100 ms stall moves it 0.8° at most.
- The feature, the bus line and the test choice cost nothing in the tick.
  None of this was measured; it follows from the code shape.

### Quality

- **Bounded by construction.** `|sin| ≤ 1`, so the target never leaves ±5° of
  where the actuator stood (SRS FR-4.5 open, FR-13.1). At `s = 0` the target
  is the start: no jump at the first command. Peak speed is about 7.9 °/s,
  far from any XmBench limit.
- **No silent simulation.** With the feature off, a bus plugin is a `run()`
  error naming the missing feature (`UserError::FeatureOff`), never a quiet
  fallback (kmr_adapters FR-031, SRS FR-6.3).
- **Builds anywhere.** On by default, `cargo build --workspace` and every
  cross-build would need `dxl_rs` and libclang, which CI does not have.
  `cargo tree -p controllers` finds `dxl_rs` 0 times by default.
- **Runs anywhere.** A shipped serial would fail bring-up on every machine but
  one. Without a bus line the binary simulates, CI's smoke run included.
- **Checked without tests in the file.** A binary's functions are not
  reachable from `tests/`. The crate is checked by build, lint, text search
  and a simulation run, and on the bench by the History record (SC-009).

### Developer experience

- One flag on one command drives the bench; no `Cargo.toml` edit.
- Three swings in 12 s: long enough to see, short enough for the onboarding
  trial. The run ends on its own, without Ctrl-C.
- A new machine edits one line, the bus line.
- The file the user copies holds no test math.

### Alternatives rejected

- **Feature on by default.** Breaks the workspace build and the cross-builds
  without the SDK.
- **The user enables the SDK in `Cargo.toml`.** One more edited line in the
  onboarding trial.
- **A ramp** is unbounded. **A square step** jumps by the full amplitude every
  half period. **`tick() × dt()` as time** is wrong after skipped slots.
- **Ship the bench adapter's serial.** Moves the bench with no edit, fails
  everywhere else. **Two binaries** double the file count the trial measures.
- **A test module at the bottom of `main.rs`.** Lengthens the onboarding file
  and teaches nothing about writing a controller.

## Pending

- **D15** (no bus line as shipped) is assumed, option A, pending the
  maintainer. Another answer changes the builder chain in `main.rs` and the
  walkthrough step only.
- **Core open choice 6** (origin of `elapsed()`): `DEMO_LENGTH` is the one
  place that changes.
