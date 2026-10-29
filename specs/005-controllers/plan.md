# Implementation Plan: controllers, the reference example

**Branch**: none (spec dir `specs/005-controllers`; work sits on `agent/robot-macro`) |
**Date**: 2026-10-28 | **Spec**: [spec.md](spec.md)

**Input**: `specs/005-controllers/spec.md` (coordinated 2026-10-28). Authority above it:
`docs/srs/SRS.md`, `## Module: controllers`. Designs it must fit:
`specs/002-kmr-core/design-first-motor.md`, `specs/002-kmr-core/design-const-generic-n.md`,
`specs/001-kmr-adapters/plan.md`. Coordination: `specs/spec-coordination-2026-10-28.md`
(critical path step 10).

## Summary

One file a new user copies to move a first motor. `controllers/src/main.rs` names the robot
with `kmr_api::robot!(XmBench)`, registers one `EachTick` controller with a context, and runs
one builder chain. The controller reads the start position from `prev_q(0)` once, then writes
`start + 5° · sin(2π · s / 4 s)` to every actuator each tick, where `s` is the time since the
start was read. After 12 s of `Time::elapsed()` it raises `shutdown!`. The engine finishes the
tick, holds, and cuts torque.

As shipped, `main.rs` binds no bus and simulates (D15 A). The walkthrough adds one
`Bus::by_id` line and one Cargo feature flag on the command line. The plan adds a
`controllers/README.md` walkthrough for the SRS FR-10.4 metric.

The code is small. Its blockers are upstream: the public `Time` getters (core #14, then
kmr_api), `Deg` and `From<Deg> for Q` (kmr_api FR-033), and `run()` driving the Dynamixel
adapter (critical path steps 3 to 9). A scratch prototype proves the file builds, lints clean,
runs, and stays inside its amplitude ([research.md](research.md) R10).

## Technical Context

**Language/Version**: Rust 1.97, edition 2024, stable

**Primary Dependencies**: `kmr_api` only (FR-002 (SRS Definitions "API"; SRS NFR-3)). `std`
for `Duration` and `f32::sin`.

**Storage**: N/A

**Testing**: `cargo build`, `cargo clippy --all-targets`, `cargo fmt --check`; a text search of
`controllers/`; a timed simulation run (exit code, one warning); cross-builds; bench runs and
the onboarding trial by the human. No test module in `main.rs` (research R8).

**Target Platform**: Linux x86-64, aarch64, armv7, std (FR-025 (SRS FR-5.5, NFR-5))

**Project Type**: binary crate in the Cargo workspace; user-facing reference

**Performance Goals**: 1 kHz tick (`dt` = 1 ms, FR-021 (SRS FR-3.1)). The controller costs a
few `f32` operations per actuator per tick.

**Constraints**: no lock, heap, sleep, block or I/O in the tick (FR-009 (SRS module
controllers; SRS NFR-1)); no `unwrap`, `expect`, `panic!`, markers (FR-008, FR-011 (SRS NFR-2));
typed values only (FR-017 (SRS FR-2.2)); bounded motion (FR-022 (SRS FR-4.5 open, FR-13.1)).

**Scale/Scope**: one file of about 50 lines, one manifest, one README. N = 4 (XmBench).

No NEEDS CLARIFICATION is left. Every open point is planned with the recommended option and
isolated to one place ([research.md](research.md)):

| Open point | Assumed | One place to change |
|---|---|---|
| D15 / spec Q1: shipped bus line | A: none, simulates as shipped | the builder chain in `main.rs` |
| D14 / spec Q2: ROS2 baseline | minimal `ros2 pkg create` node, user-edited files counted | the baseline section of `controllers/README.md` |
| D6: bus label | optional for a one-bus robot, so `Bus::by_id("…")` has no label | the walkthrough's bus line |
| D2: joint-limit policy | B: `run()` refuses unless every limited actuator has a policy (decided 2026-10-28 (for now): D2 B), so the example sets one for all actuators | the policy line of the builder chain (C9) |
| Core open choice 6: `elapsed()` origin | `Robot::new()` today; the demo is right under either | `DEMO_LENGTH` |
| Simulation History (core FR-042 vs core design) | either; the demo is right under both | none needed |

## Constitution Check

*GATE: checked before Phase 0, re-checked after Phase 1. Passes, with two recorded conflicts
for the human.*

| Principle | Status | Note |
|---|---|---|
| 0. ov policy | Pass, one conflict recorded | This plan edits `specs/005-controllers/` only. Conflict: Principle 0 asks agent trailers; AGENTS.md rule (b) and the human's standing decision forbid them. Not resolved here |
| I. SRS authority | Pass, one conflict recorded | Conflicts reported in research.md. Marker rule: the module SRS forbids `todo!()`, so the example avoids every open path (FR-012 (SRS FR-1.2, FR-2.7, FR-8.4)) instead of marking it. Delivery order: D1, below |
| II. Hot path | Pass | One controller: array `map`, `sin`, one setter. No heap, `dyn` or lock. `shutdown!` is one compare-and-swap |
| III. No panics | Pass | `main` returns `run()`'s `Result`. A missing sample writes nothing (FR-019 (SRS FR-2.6)) |
| IV. Hand-engineered core | Pass | No `kmr_core` edit by this module. Its blockers are HUMAN core items; their one invariant file is `state/field.rs` (core #2, write mask) |
| V. Measured | Pass | FR-10.4 time and file count are measured by the trial. Amplitude, period and length are picked for safety, not measured, and stated in the README |
| VI. Secure by default | N/A | No network path |
| Technical constraints | Pass, one gap | `f32` only. armv7 links only where an armv7 linker is installed (research R10) |

**D1, delivery gate (decided 2026-10-28: D1 A).** SRS section 8 lists no step for this
module. The critical path puts it at step 10, after every step that moves a motor. Under D1 A
the example is written once steps 3 to 9 land; nothing here starts early except this plan.

Post-design re-check: unchanged. Phase 1 adds no dependency, no heap type and no core edit.

## Project Structure

### Documentation (this feature)

```text
specs/005-controllers/
├── plan.md              # this file
├── research.md          # Phase 0
├── data-model.md        # Phase 1
├── quickstart.md        # Phase 1
├── contracts/
│   ├── run-behaviour.md # what the binary prints and returns, per scenario
│   └── api-needs.md     # the kmr_api surface the example calls
└── tasks.md             # Phase 2 (/speckit-tasks), not created here
```

### Source Code (repository root)

```text
controllers/
├── Cargo.toml           # kmr_api only; feature `hardware = ["kmr_api/dynamixel"]` (R4)
├── README.md            # new: the FR-10.4 walkthrough and the ROS2 baseline (R7)
└── src/
    └── main.rs          # rewritten: robot!(XmBench), Swing context, one builder chain
```

**Structure Decision**: the crate keeps its one binary. No `build.rs`, no config file, no
`tests/` (FR-004 (SRS FR-6.1)). The README is documentation, not a side file.

## Work outline

For `/speckit-tasks`. Every row is in `controllers/**` unless it says otherwise.

C0 is unblocked and must land before kmr_api plan A4 (`Robot<H>`): once `H` is generic, a bare
`kmr_api::Robot::new()` stops compiling (kmr_api research R5), so `main.rs` must already name
`robot!(XmBench)`. `robot!` exists today; the `N == JOINTS` assert holds for XmBench (N = 4).

| # | Who | Work | Blocked on | FR |
|---|---|---|---|---|
| C0 | Agent | `main.rs`: add `kmr_api::robot!(XmBench);` at module level and drop `Robot`, `State` from the `use kmr_api::{..}` line. Nothing else changes (the controller body stays until C2). Before kmr_api A4 | nothing | FR-003 (SRS FR-6.1) |
| C1 | Agent | `Cargo.toml`: add `[features] hardware = ["kmr_api/dynamixel"]` | nothing | FR-002, FR-025 (SRS NFR-3, NFR-5) |
| C2 | Agent | `main.rs`: `robot!(XmBench)`, constants, `Swing`, `swing`, builder chain, as in [data-model.md](data-model.md) | kmr_api `Time` getters and `Deg` ([contracts/api-needs.md](contracts/api-needs.md)) | FR-003, FR-005, FR-013 to FR-019, FR-021 to FR-024 |
| C3 | Agent | `README.md`: walkthrough, numbers, end-of-demo torque off, ROS2 baseline and count rule | C2 | FR-027, FR-028 (SRS FR-10.4, NFR-4) |
| C4 | Agent | Run [quickstart.md](quickstart.md) V1 to V3; report exact results | C2 | SC-003, SC-004, SC-007, SC-008, SC-011 |
| C5 | Agent | V4: bus plugin, no robot, readable error | critical path step 9 | FR-026, SC-012 (SRS FR-6.4) |
| C6 | HUMAN | V5, V6: bench runs, History bounds, Ctrl-C, `Torque_Enable` = 0 | critical path steps 1 to 9 | SC-009, SC-010 (SRS FR-4.1, FR-4.3) |
| C7 | HUMAN | V7: onboarding trial against the ROS2 baseline | C3, C6 | SC-001, SC-002 (SRS FR-10.4) |
| C8 | HUMAN | CI smoke run of V2 (`.github/workflows/ci.yml` is outside this module) | C2 | SC-011 |
| C9 | Agent | `main.rs`: one builder line, `.set_limit_policy_all(Policy::Clamp)`, before `.add_controller_with`. Assumed, pending human: D2 (option B). The swing never reaches a limit (R3); the line is there because `run()` requires it. Walkthrough lists it | kmr_api C4, core K5.3 | FR-022 (SRS FR-4.5 open, FR-6.4) |

## Complexity Tracking

No constitution violation needs a justification.
