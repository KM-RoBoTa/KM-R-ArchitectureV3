# Data model: controllers, the reference example

Phase 1 of [plan.md](plan.md). Everything here lives in `controllers/src/main.rs` unless said
otherwise. The `robot!` aliases `N`, `State`, `Robot` come from `kmr_api::robot!(XmBench)`.

## Robot line

| Item | Value | Rule |
|---|---|---|
| `kmr_api::robot!(XmBench);` | module level, once | FR-003 (SRS FR-6.1) |
| `N` | 4 for XmBench | from the preset; never written by the user |
| `State`, `Robot` | aliases carrying `N` | signatures stay `&mut State` |

## Constants

| Name | Type | Value | Rule |
|---|---|---|---|
| `AMPLITUDE` | `Deg` | `Deg(5.0)` | half the swing; revolute only (R9) |
| `PERIOD` | `Duration` | 4 s | one full swing |
| `DEMO_LENGTH` | `Duration` | 12 s | compared with `Time::elapsed()` (R5) |
| `dt` | `Duration` | 1 ms, set in the builder | FR-021 (SRS FR-3.1) |

## Context `Swing`

The controller's own data (FR-014 (SRS Definitions "Context")). Built in `main` with
`Swing::default()`, moved in at registration, lent back as `&mut Swing` each tick. Never shared,
never locked.

| Field | Type | Meaning |
|---|---|---|
| `start` | `Option<([Q; N], Duration)>` | start position and the `elapsed()` at which it was read; `None` until the first sample |

Size is fixed at compile time, about 32 bytes for N = 4. No heap.

## Target function

`offset(s: Duration) -> Q = Q::from(Deg(AMPLITUDE.0 · sin(2π · s / PERIOD)))`.

| Property | Value |
|---|---|
| Range | `|offset| ≤ 5°` = 0.0873 rad for every `s` (R10) |
| At `s = 0` | exactly `Q(0.0)`: the first command equals the start |
| Peak speed | ≈ 7.9 °/s |
| Input | time since the start was read, never a call count (FR-016 (SRS FR-1.5)) |

## Controller `swing` on `EachTick`

`fn swing(ctx: &mut Swing, time: &Time, robot: &mut State, _: &Sensors)`, registered with
`add_controller_with(EachTick, Swing::default(), swing)` (FR-013, FR-015 (SRS FR-1.1, FR-1.4)).

Per tick, in this order:

1. If `start` is `None`: take `robot.prev_q(0)`. On `Ok(q)`, store `(q, time.elapsed())`. On
   `Err`, store nothing (FR-019 (SRS FR-2.6)).
2. If `start` is `Some((q0, t0))`: `robot.set_all_q(q0.map(|q| q + offset(elapsed − t0)))`.
   Every tick, never assumed to persist (FR-018 (SRS FR-2.5, FR-3.4)).
3. If `time.elapsed() ≥ DEMO_LENGTH`: `shutdown!("reference demo: time is up")` (FR-023
   (SRS FR-4.1, FR-4.4)).

`elapsed − t0` uses `saturating_sub`. No index, no `unwrap`, no allocation.

## States

| State | Entered when | Writes | Leaves when |
|---|---|---|---|
| Waiting | run starts | nothing | a sample is read |
| Swinging | `start` stored | all `Q`, every tick | `elapsed() ≥ DEMO_LENGTH` |
| Ending | `shutdown!` raised | this tick's `Q` | the engine finishes the tick |
| (engine) Hold, torque off | after the tick | hold = latest sample with this tick's writes | `run()` returns `Ok(())` |

Waiting can go straight to Ending: with no sample, the demo still ends on time. Ctrl-C,
SIGTERM or SIGHUP from any state is a shutdown handled by the engine (FR-024 (SRS FR-4.3)).
An emergency stop from anywhere skips the write and the hold (design-first-motor §3).

## Builder chain

```text
Robot::new()
    .set_dt(1 ms)
    .set_limit_policy_all(Policy::Clamp)     // plan C9; assumed, pending human: D2 B
    [.add_plugins(Bus::by_id("<serial>"))   // added by the walkthrough only, D15 A]
    .add_controller_with(EachTick, Swing::default(), swing)
    .run()                                   // main returns this Result
```

One chained expression, no `build()` (FR-005 (SRS FR-10.1)). `Bus::by_dev` is not used
(FR-006 (SRS FR-6.3)).

## Walkthrough (`controllers/README.md`)

| Part | Content |
|---|---|
| Prerequisites | toolchain, a fresh checkout, XmBench wired and powered, the adapter's serial |
| Files written | none for XmBench; `Cargo.toml` and `main.rs` for a copied crate |
| Lines edited | the bus line (added); the preset name (only for another robot) |
| Commands | the build and run commands, `--features hardware` for the bench |
| Stated numbers | preset, amplitude, period, length, torque off right after the hold |
| Warnings | revolute only; position mode only; a robot that would fall needs a hold duration |
| Baseline | the ROS2 node and the count rule (R7) |

## Trial record (SRS FR-10.4)

One row per trial, kept by the human with the README.

| Field | Type |
|---|---|
| side | kmr, ROS2 C++, ROS2 Python |
| robot | XmBench |
| elapsed | minutes, fresh checkout to first motion |
| files written | count |
| lines edited | count |
| commands run | count |
