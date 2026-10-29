# Implementation Plan: kmr_model, the robot model from one descriptor

**Branch**: `agent/robot-macro` (spec dir `003-kmr-model`) | **Date**: 2026-10-28 |
**Spec**: [spec.md](spec.md)

**Input**: `specs/003-kmr-model/spec.md`, SRS module `kmr_model`, delivery step 2.

## Summary

`kmr_api::robot!` in `main.rs` names a preset or a descriptor. A proc macro in `kmr_model`
reads it at compile time and emits only `N`, a `model` module of `const` arrays (names, limits,
kind bit sets, links) and an `actuator!` name macro. kmr_api's `macro_rules!` front door around
it emits the `State`/`Robot` aliases (`Robot = kmr_api::Robot<__Spec>`), the hidden `__Spec`,
the checks, and splits a `hardware = ..` argument off before the proc macro (D7 A, assumed,
pending human: D7). No item is emitted twice. The binary gets constants only. N reaches
kmr_core as a const generic, which is human work; until then a compile-time assert in the
front door ties N to `kmr_core::state::JOINTS`. Design: [research.md](research.md).

## Technical Context

**Language/Version**: Rust 2024, rustc 1.97.1 stable

**Primary Dependencies**: build time only: `roxmltree`, `ron`, `serde`, `serde_json`,
`proc-macro2`, `quote`, `syn` (parsing only). Emitted code: `kmr_core` value types through
`kmr_api`.

**Storage**: descriptor files: user crate, or `kmr_model/presets/` for presets

**Testing**: `cargo test`; `trybuild` for compile-fail cases (R10); unit tests on the pure parse
functions

**Target Platform**: build host any; emitted code: x86-64, aarch64, armv7 Linux and the `no_std`
bare-metal CI target

**Project Type**: library (proc-macro crate + one `macro_rules!` front door in kmr_api)

**Performance Goals**: 0 ns per tick (SC-004); 100-joint URDF adds < 1 s to a clean build
(SC-005, starting guess)

**Constraints**: emitted code has no heap, no `dyn`, no lock, no `std` (FR-019); heap allowed
inside the proc macro only (`kmr_model/clippy.toml`); kmr_core untouched (Constitution IV)

**Scale/Scope**: 1 to a few hundred actuators per descriptor; one descriptor per binary

## Constitution Check

*Before Phase 0, and again after Phase 1.*

| Principle | Status | Note |
| --- | --- | --- |
| 0 ov policy: `.aiignore` paths untouched | Pass | No edit to kmr_core, SRS.md, ov files |
| 0 ov policy: agent trailer on commits | **Conflict** | Human decided "no trailers" (2026-10-27, ov `none` mode). Constitution needs a human amendment. Not this plan's call. |
| I SRS authority | Pass | Spec markers resolved from the SRS; conflicts in research R11 |
| I Delivery order | **Deviation** | Step 1 has not moved a motor. This is design only. Implementing it before step 1 needs the human's waiver. |
| II Hot path | Pass | Emitted facts are consts; nothing runs per tick |
| II Heap ban | Pass, justified | Heap inside the proc macro only, gone after the build (Complexity Tracking) |
| III No panics | Pass | Errors are `compile_error!`; no run-time lookup that can panic (R6) |
| III Errors | Pass | Bad descriptor = compile error (FR-008, SRS FR-6.4) |
| IV Hand-engineered core | Pass | Const generic N is a HUMAN item; the temporary assert keeps the core untouched |
| IV YAGNI | Pass | Emitted code adds no type (core types, `u64`, `f32`, `&str`); one private `Reader` trait at build time (FR-025) |
| V Measured | Pass | SC-004, SC-005 measured by the bench module |
| VI Secure by default | N/A | No network path |

Re-check after Phase 1: unchanged. The design added no type, no dependency of kmr_core, no
run-time path.

## Project Structure

### Documentation (this feature)

```text
specs/003-kmr-model/
├── spec.md
├── plan.md              # this file
├── research.md          # R1–R11
├── data-model.md        # build-time model, emitted facts
├── quickstart.md        # validation steps
├── contracts/
│   ├── robot-macro.md   # robot! forms, emitted items, error messages
│   └── descriptor-ron.md
└── tasks.md             # /speckit-tasks, not created here
```

### Source Code

```text
kmr_model/                      # proc-macro crate (exists on agent/robot-macro)
├── Cargo.toml
├── clippy.toml                 # disallowed-types = [] (build time only)
├── presets/
│   └── xm_bench.urdf           # HUMAN: real bench URDF (R7)
└── src/
    ├── lib.rs                  # #[proc_macro] model!: input → Model → tokens
    ├── input.rs                # `preset = X` | `model = "path"` (syn)
    ├── model.rs                # Model, Actuator, Link, ModelError; validation 1–10
    ├── urdf.rs                 # roxmltree → Model
    ├── serde_format.rs         # .ron / JSON → Model (one schema)
    ├── presets.rs              # name → embedded descriptor text
    └── emit.rs                 # Model → N, model::*, actuator!

kmr_api/
├── src/robot_macro.rs          # robot! front door: aliases, __Spec, hardware arms, checks (kmr_api plan A4, B1 to B3)
└── tests/
    ├── robot_macro.rs          # N, order, limits, names, links
    ├── formats.rs              # URDF = .ron = JSON
    ├── ui.rs + ui/*.rs/.stderr # trybuild compile-fail, one per edge case
    └── fixtures/               # four, six_plus_two, continuous, prismatic, hundred, bad_*

kmr_adapters/src/
├── presets/xm_bench.rs         # XmBench::HARDWARE: Table<1, 4> (kmr_adapters plan B2); Preset::N removed once N comes from here
└── (robot.rs)                  # deleted by kmr_adapters plan B5
```

**Structure Decision**: no new crate. `kmr_model` stays the proc-macro crate the SRS module maps
to; `kmr_api` keeps the only public door. kmr_core is not touched.

## Phase 0 and Phase 1 output

- [research.md](research.md): every question decided; R11 lists conflicts for the human.
- [data-model.md](data-model.md), [contracts/](contracts/), [quickstart.md](quickstart.md).

## Hand-offs

**HUMAN (kmr_core)**: N as a const generic (core plan M2); then delete the temporary
`N == JOINTS` assert in `robot!`. The limits consumer is core K5.2: typed `[Limit; N]`, with
`lower`/`upper` `None` where `CONTINUOUS` is set (kmr_api B3 does that mapping). D2 assumed B.

**HUMAN (data)**: the real URDF of the XM430 bench chain for `presets/xm_bench.urdf` (R7).

**HUMAN (decisions)**: waive the delivery order for step 2, or wait for step 1; amend the
constitution's trailer rule; reword SRS NFR-10's "kmr_model is a dependency of kmr_core" (R11).

**kmr_adapters**: drop `Preset::N`; make the hardware table a fixed-size array, so `robot!`'s
array-type check (kmr_api FR-007) catches a length other than N (kmr_adapters plan B1, B4).
`Table::check_kinds(self, [u64; MASK_WORDS]) -> Self` takes `model::PRISMATIC` (E6).

**kmr_ffi** (open, pending human: D12): its build helper needs N in a user `build.rs` from a
preset name or descriptor path (kmr_ffi plan B2). A proc-macro crate exports only macros, so
that library entry needs kmr_model split into a lib crate and a macro crate. Not planned here;
research R12.

**bench**: SC-004 overhead and SC-005 build time (SRS NFR-8 (d), NFR-11).

## Complexity Tracking

| Violation | Why needed | Simpler alternative rejected because |
| --- | --- | --- |
| Heap types in `kmr_model` (root clippy bans them) | Parsing XML/RON/JSON needs owned strings and vectors | A `no_std` parser would cost weeks and buy nothing: the heap dies with the build, the binary gets consts only |
| Temporary `N == JOINTS` assert | kmr_core's N is still a hand-set const | Editing kmr_core is human work; without the assert a mismatch would build and index out of range |
| Planning step 2 before step 1 moved a motor | User asked for a plan now | Plan only; implementation waits for the human's call |
