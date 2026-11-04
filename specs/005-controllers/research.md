# Research: controllers, the reference example

Phase 0 of [plan.md](plan.md). Each entry: decision, rationale, alternatives. "Assumed,
pending human: Dn" marks a choice made with the recommended option of the coordination page.

## R1. Shipped bus line (spec Q1). Assumed, pending human: D15 A

- **Decision**: the shipped `main.rs` binds no bus. It simulates anywhere, in CI included. The
  walkthrough adds one line, `.add_plugins(Bus::by_id("<serial>"))`, before `.add_controller_with`.
- **Rationale**: the port is per machine (FR-006 (SRS FR-6.3)), so every user but one edits the
  line anyway. A shipped serial would make every other machine fail at bring-up.
- **Alternatives**: B, the bench adapter's serial: moves the bench with no edit, fails
  everywhere else. C, two binaries: twice the file count the trial measures.
- **One place to change**: the builder chain in `main.rs`, plus the walkthrough step.

## R2. Bus label. Assumed, pending human: D6

- **Decision**: XmBench has one bus, so the walkthrough line has no `.bus("label")`.
- **If D6 changes** (label mandatory): the walkthrough line gains `.bus("<label>")`. Nothing
  else changes.

## R3. Demo motion: numbers and shape

- **Decision**: per actuator, `target = start + Q::from(Deg(5.0 · sin(2π · s / 4 s)))`, with
  `s = elapsed() − t0` and `t0` the `elapsed()` at which `start` was read. Constants:
  `AMPLITUDE = Deg(5.0)`, `PERIOD = 4 s`, `DEMO_LENGTH = 12 s`.
- **Rationale**:
  - Bounded by construction: `|sin| ≤ 1`, so the target never leaves ±5° of the start
    (FR-022 (SRS FR-4.5 open, FR-13.1)). Proved over 0 to 50 000 s (R10).
  - No jump at the first command: `s = 0` gives `target = start`.
  - Slow: peak speed is 5° · 2π / 4 s ≈ 7.9 °/s.
  - A function of time, not of call count (FR-016 (SRS FR-1.5, FR-3.3)). After an overrun the
    target moves along the same curve; a 100 ms stall moves it at most 0.8°.
  - Three full swings in 12 s: long enough to see, short enough for a trial.
  - Never near a joint limit on XmBench. The policy line of R6 is there because `run()`
    requires it under D2 B, not because the swing relies on it.
- **Alternatives**: a ramp (unbounded, rejected by FR-022); a square step (a jump of the full
  amplitude each half period); `tick()` times `dt()` (equal on the grid, wrong after skipped
  slots).

## R4. Hardware feature in the user crate

- **Decision**: `controllers/Cargo.toml` gets `[features] hardware = ["kmr_api/dynamixel"]`,
  off by default. The bench command is `cargo run -p controllers --features hardware`.
- **Rationale**: kmr_adapters R9 gates the C SDK behind `dynamixel`, and "the robot binary
  enables it". On by default, `cargo build --workspace` and every cross-build would need
  `dxl_rs` and libclang, which CI does not have yet (critical path step 1). Off by default, a
  bus plugin without the feature is a `run()` error naming the feature (kmr_adapters R9), never
  a silent simulation (kmr_adapters FR-031 (SRS FR-6.3)).
- **Proof** (R10): `cargo tree` finds `dxl_rs` 0 times by default, for the crate and for the
  workspace, and finds it with `--features hardware`.
- **Alternatives**: feature on by default (breaks CI today); the user edits `Cargo.toml`
  (one more edited line in the trial).
- **Cost to the metric**: one flag on one command. The walkthrough lists it (FR-027).

## R5. Duration counted from `elapsed()`. Pending human: core open choice 6

- **Decision**: the end check is `time.elapsed() >= DEMO_LENGTH`, independent of the start
  sample. The controller writes first, then raises, so the shutdown tick still writes and the
  hold keeps that tick's target (design-first-motor §2, hold target).
- **Rationale**: the demo must end even when no sample ever comes (spec Edge Cases). Today
  `elapsed()` counts from `Robot::new()`, so the motion lasts 12 s minus bring-up. If the core
  moves the origin to `arm`, the motion lasts the full 12 s. SC-010 holds under both: it is
  stated against `elapsed()`.
- **One place to change**: `DEMO_LENGTH`.

## R6. Joint-limit policy. Assumed, pending human: D2 B

- **Decision**: every plan assumes D2 B: `run()` refuses, before any I/O, unless every limited
  actuator has a policy (core K5.3). The model makes a limit mandatory per actuator, so the
  example sets one for all four in one builder line, `.set_limit_policy_all(Policy::Clamp)`
  (kmr_api C4). Plan C9, gated on kmr_api C4 and core K5.3. The walkthrough lists the line.
- **Why `Clamp`**: it keeps a command inside the limit and never cuts torque mid-swing. `Abort`
  would end the demo with an emergency stop; `Warn` would send the command past the limit. The
  swing never reaches a limit (R3), so the choice is not exercised on XmBench.
- **Until C9 lands**: the example has no policy line. Before core K5.3 nothing refuses it.
- **If D2 A or C wins**: C9 is dropped (A) or becomes optional (C). Nothing else changes. The
  swing does not rely on the policy (FR-022).

## R7. ROS2 baseline (spec Q2). Assumed, pending human: D14

- **Decision**: the baseline is a minimal node made with `ros2 pkg create`, C++ and Python, on
  the current ROS2 LTS, driving the same four actuators through the vendor's ROS2 SDK package.
  Only files the user writes or edits count, for both sides. kmr must beat both variants on
  time and on file count (SC-001, SC-002 (SRS FR-10.4, NFR-4)).
- **Where**: a "Baseline" section of `controllers/README.md`. One place to change.
- **Alternatives**: count every generated file (inflates the baseline); a full `ros2_control`
  setup (not "the equivalent node").

## R8. No test module in `main.rs`

- **Decision**: `main.rs` holds no `#[cfg(test)]` block. The bound proof stays in the scratch
  prototype and in the bench History check (SC-009). CI checks the crate by build, lint, text
  search and a simulation run.
- **Rationale**: the file is what the user copies. Twenty lines of test math there teach
  nothing about writing a controller, and a binary crate's functions are not reachable from
  `tests/`.
- **Alternative**: a test module at the bottom of `main.rs`. Rejected for onboarding length.

## R9. What the example does not depend on

- **Zero position (D8)**: the swing is relative to the start sample, so it is the same under
  every D8 option.
- **Read cadence (D9)**: the demo needs one sample, the start. Later targets do not read.
- **N cap (D4)**: N = 4.
- **Const generic N** (design-const-generic-n): the example names only the `robot!` aliases,
  so it is unchanged when `State` becomes `State<'_, N>` (that design's prototype `muser`).
- **Prismatic units (SRS FR-2.2)**: XmBench is all revolute. The README says the amplitude is
  in degrees and fits revolute actuators only.

## R10. Prototype

Path: `/tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/controllers-plan/`.
A copy of the critical-path prototype (`scratchpad/critpath/`: the core design applied). Added
in the copy only: `kmr_api::Time::{elapsed, dt, tick}` forwarders, `kmr_api::Deg` with
`From<Deg> for Q`, the planned `controllers/Cargo.toml` and `main.rs`, two scratch tests.
rustc 1.97.1, `--offline`.

| Check | Result |
|---|---|
| `cargo build -p controllers` | ok, 0 warnings from `controllers` |
| `cargo clippy -p controllers --all-targets`, root `clippy.toml` | 0 warnings from `controllers`; 0 with `-W clippy::pedantic` too |
| `cargo fmt -p controllers --check` | clean |
| `From<Deg> for Q` in `kmr_api` | compiles: the orphan rule allows it (`Deg` is local) |
| Simulation run, no bus plugin | exit 0 after 12.005 s; shutdown logged with its reason and `main.rs:37` |
| Warnings in that run | **2**, not 1: kmr_api's `eprintln!` "kmr: no bus plugin: simulating, no hardware is driven" and the core's `WARN` "no hardware adapter compiled in: nothing is read or written" (R11) |
| `prev_q(0)` at tick 0, simulation | `Ok(all zeros)`, not "no sample" (R11) |
| Swing bound, 0 to 50 000 s in 37 ms steps | worst 0.08726646 rad, bound 5° = 0.08726654 rad; `offset(0) = 0` |
| `cargo tree`, `dxl_rs` count | 0 (crate), 0 (workspace); present with `--features hardware` |
| `cargo build --target aarch64-unknown-linux-gnu` | links with `aarch64-linux-gnu-gcc`; fails with the default `cc` |
| `cargo check --target armv7-unknown-linux-gnueabihf` | ok; link not tried, no armv7 linker installed |
| A `run()` error returned from `main` | prints `Error: DuplicateBus { port: "X" }`, exit 1: the `Debug` form, not the message (R12) |

Not proved here: the bring-up error with a bus plugin and no robot (needs critical path step
9), and motion on hardware (HUMAN, step 11).

## R11. Simulation: two documents disagree, and two warnings

- **Finding 1**: kmr_core FR-042 says the no-hardware adapter seeds nothing and History reads
  return "no sample". The human design (`design-first-motor.md` #3, `push_slot`) pushes a copy
  of `prev(0)`, "zeros when empty", every tick. The prototype shows the design wins in code:
  `prev_q(0)` is `Ok` with zeros.
- **Decision for this module**: the demo is right under both. With zeros it swings around 0 rad
  and writes to an adapter that drops writes. With "no sample" it writes nothing. Both end on
  `DEMO_LENGTH`. Quickstart V2 accepts either.
- **Finding 2**: the simulation run prints two warnings. SC-011 and core FR-042 want exactly
  one, the core's, worded "no bus plugin: simulating". kmr_api must drop its `eprintln!` and the
  core warning must use that text. This module prints none (FR-007 (SRS FR-6.3)).

## R12. Readable error from `main`

- **Finding**: `fn main() -> Result<(), UserError>` prints the error's `Debug`. With a derived
  `Debug` that is `DuplicateBus { port: "X" }`, not the `Display` text.
- **Decision**: keep `main` returning `run()` (FR-005 (SRS FR-10.1)). Ask `kmr_api` for a
  `Debug` on `UserError` that prints the `Display` message, so SC-012's "readable bring-up
  error" holds with no code in the example.
- **Alternative**: the example matches the error, prints `{e}` and returns `ExitCode`. Five
  more lines in the file every user copies. Rejected.
- **Outcome, 2026-10-29**: `kmr_api` prints the `Display` text in release builds; debug builds
  derive `Debug`, so the variant and the typed adapter error show. Quickstart V4 runs
  `--release`.

## R13. Hardware safety assumption on the start sample

- **Finding**: the demo trusts `prev_q(0)` as where the actuator stands. Under the design,
  History seeds from `read_all` in PostInit. If a slot could ever hold "zeros when empty" on
  hardware, the first command would drive every actuator to 0 rad: a jump, not a swing.
- **Need**: on hardware, `read_all` fills every actuator or bring-up fails (kmr_core,
  kmr_adapters). The example cannot check this itself until sample age and stale flags reach
  the API (kmr_api FR-031, off the critical path).

## R14. Known conflicts, recorded and not fixed

- Constitution Principle 0 (agent trailers) against AGENTS.md rule (b) and the human's
  no-trailer decision.
- D1, delivery gate: SRS section 8 gives this module no step (plan.md, Constitution Check).
- SRS Definitions "Context" allows `Arc<Mutex<T>>`; the root `clippy.toml` bans `Arc` in this
  crate. The example uses a plain struct.

## R15. Coordination log

- 2026-10-28 coordination: plan step C0 added. It only switches `main.rs` to
  `robot!(XmBench)`, is unblocked, and lands before kmr_api A4, which makes bare
  `Robot::new()` stop compiling (kmr_api research R5).
- 2026-10-28 coordination: D2 aligned across all plans on option B (assumed, pending human:
  D2). R6 rewritten; plan step C9 adds one `set_limit_policy_all` line, gated on kmr_api C4 and
  core K5.3. Data model, API needs and walkthrough follow.
