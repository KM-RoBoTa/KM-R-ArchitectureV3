# Implementation Plan: kmr_ffi, foreign-language controller math

**Branch**: `006-kmr-ffi` (spec dir; no branch created, the branch is the human's) | **Date**: 2026-10-28 |
**Spec**: [spec.md](spec.md)

**Input**: `specs/006-kmr-ffi/spec.md` (coordinated 2026-10-28). Authority above it:
`docs/srs/SRS.md` (patched 2026-10-27), module `kmr_ffi`. Core designs it must fit:
`specs/002-kmr-core/design-const-generic-n.md`, `specs/002-kmr-core/design-first-motor.md`.
Coordination: `specs/spec-coordination-2026-10-28.md`. No `kmr_ffi/` code exists yet.

## Summary

SRS delivery step 8, Could. A Rust controller hands its math to C, C++, Python or Lua. The
Rust controller stays the one registered on a phase. The builder never crosses (FR-002 (SRS
FR-10.1)).

Design in five lines:

1. Three `#[repr(C)]` types cross: `ReadView<N, D>` (every History slot: `f32` arrays and flag
   masks; per-actuator age for the latest slot), `WriteBuf<N>` (one `f32` array and one written mask per field), `TimeView`
   (`u64` ticks and nanoseconds). Generic over N on the Rust side; plain on the C side.
2. Copy, not share. kmr_api's glue fills the view from `State`, calls the foreign function
   once, and applies only the flagged entries through the normal setters. The seal holds.
3. One field list (`for_each_field!`) drives the Rust types, the C and C++ headers and both
   bridges. A core-side exhaustive field id makes a missing field a build error.
4. C/C++ get N from a build helper in the user's `build.rs` (D12 A). It writes the headers and
   an ABI file; `foreign!` in `main.rs` const-asserts that N equals `robot!`'s N.
5. Foreign code ends a call with a status. The Rust forwarder raises `shutdown!` or
   `emergency_stop!` (D13 A). No foreign path touches `signal.rs`.

The riskiest claim is proved in a scratch crate ([research.md](research.md) R1): the Rust and
the generated C layouts are identical for N = 1, 3, 4, 10, 64; generic `repr(C)` structs pass
`improper_ctypes` cleanly; today's `Q` (no `repr(transparent)`) does not; a wrong N fails
`cargo check`. 6 tests pass in debug and release, clippy clean.

## Technical Context

**Language/Version**: Rust 1.97, edition 2024, stable; C11 and C++17 for the user side

**Primary Dependencies**: none from kmr in `kmr_ffi` itself (R9). Opt-in: `cc` (feature
`build`, build scripts only), `pyo3` (feature `python`), `mlua` with vendored Lua 5.4
(feature `lua`). Versions chosen at implementation; neither is in the offline cache today

**Storage**: N/A

**Testing**: `cargo test -p kmr_ffi` (Rust `extern "C" fn` doubles, header text); the C example
crate's tests (C/C++ compiled by `cc`, layout probes for several N); `kmr_api` glue tests on a
scripted History (feature `ffi`, core `test-util`); compile-fail for a wrong N

**Target Platform**: Linux x86-64, aarch64, armv7, std (FR-001 (SRS FR-10.1), SRS NFR-5).
`no_std` out of this version (spec Scope)

**Project Type**: library crate in the Cargo workspace, reached through `kmr_api`'s `ffi` feature

**Performance Goals**: C/C++ path per call: one fill of `ReadView` (440 bytes at N = 4, 3560 at
N = 64, measured layout, research R1), one indirect call, one apply loop over N. Cost over an empty Rust
controller measured on the NFR-1 bench (SC-003 (SRS NFR-1, NFR-11))

**Constraints**: C/C++ path heap-free, lock-free, `dyn`-free in kmr code (FR-021 (SRS NFR-1));
no panic or exception crosses (FR-022 (SRS NFR-2)); heap allowed for bridge set-up, before
the first tick (FR-019 (SRS FR-10.3)); N ≤ 64 from `u64` masks (D4 A)

**Scale/Scope**: N in 1..=64, `HISTORY_DEPTH` slots; three fields today (`Q`, `Qd`, `Tau`),
`Kp`/`Kd` next; four languages; four examples

No NEEDS CLARIFICATION is left. The spec's open points are planned with the recommended option,
each isolated to one place ([research.md](research.md)):

| Point | Assumed | One place to change |
|---|---|---|
| Q1 time and flag types (no D id; SRS problem FR-10.2 vs FR-3.3) | integers: `u64` ticks, `u64` ns, `u64` masks | `TimeView`, `ActuatorFlags` in `view.rs` |
| D12 how C/C++ gets N | A: build helper in user `build.rs`, robot named again, const check | `kmr_ffi::build` and `foreign!` |
| D13 foreign shutdown/e-stop | A: return status, forwarder raises the macro | `Status` and the glue's `match` in kmr_api |
| D4 N cap | A: 64, one `u64` per mask | `ffi::Mask = u64` and the const assert in `ReadView::default` |
| D1 delivery gate | A: plan now, build after step 7 moves a motor | task order only |

## Constitution Check

*GATE: checked before Phase 0, re-checked after Phase 1. Passes with three items for the human.*

| Principle | Status | Note |
|---|---|---|
| 0. ov policy: `.aiignore` read-only | Pass | Nothing in `kmr_core/` is edited. Core needs are HUMAN tasks below, invariant file named |
| 0. ov policy: agent trailer | **Conflict, for the human** | Constitution Principle 0 requires an agent trailer; AGENTS.md rule (b) and the human's final decision forbid any AI or co-author trailer. This plan makes no commit. Agent commits follow the human's decision until one side is amended |
| I. SRS authority | Pass | Every FR cites its SRS id. SRS problems are reported in research.md R14, not patched |
| I. Delivery order (SRS §8) | **Conditional on D1** | Step 8 starts after steps 1 to 7 move a motor. D1 A assumed. Planning now is allowed by the spec's Gate |
| II. Real-time hot path | **Conflict, for the human** | Principle II says "only `kmr_api` may use" heap; SRS FR-10.3 lets kmr_ffi use it like kmr_api. The SRS wins (Principle I). Needs a constitution amendment and a `kmr_ffi/clippy.toml` override, both HUMAN. The C/C++ per-call path stays heap-free (FR-021) |
| III. No panics, errors returned | Pass | `forbid(clippy::unwrap_used)`, no indexing outside `get`/iterators, foreign errors returned (FR-022). Bridge load errors returned in `main.rs` before `run()` (FR-020 (SRS FR-6.4)). No SRS markers needed: D13 closes FR-023 |
| IV. Hand-engineered core, small surface | Pass | `pub` only what kmr_api re-exports. `kmr_ffi` has no kmr dependency, so it cannot reach core internals. The one core addition (field id, R7) is a HUMAN task in `state/field.rs` (invariant) |
| V. Measured, not guessed | Pass | Layout proved, not assumed. Per-tick overhead measured by bench (SC-003). Decision record `docs/decisions/0004-ffi-boundary.md` (FR-026 (SRS NFR-6)) |
| VI. Secure by default | N/A | No network path |
| Technical constraints | Pass | `f32` state values; integer time is the justified exception of R3, not `f64` |
| Workflow | Pass | Agentic work only on `agent/<topic>`, maintainer authorization on the PR, `agentic` label |

Post-design re-check: unchanged. The design adds no `dyn`, no heap on the C/C++ per-call path, no
core edit, one new crate (`kmr_ffi`, mapped one-to-one to the SRS module).

## Order of work

Nothing here starts before delivery step 7 is done (D1 A). "Blocked" names what lands first.

### A. Crate and types (no kmr dependency)

| # | Who | Work | FR |
|---|---|---|---|
| A1 | HUMAN | Root `Cargo.toml` members `kmr_ffi` and its four example crates; `kmr_ffi/clippy.toml` heap override; constitution II amendment | FR-019 (SRS FR-10.3) |
| A2 | Agent | `fields.rs`: `for_each_field!` (q, qd, tau) and `for_each_flag!` (stale, bus_error, offline, rejected, joint_limit, envelope) | FR-014 (SRS FR-10.2) |
| A3 | Agent | `view.rs`: `Slot<N>`, `ReadView<N, D>` (with `age: [u32; N]`, latest slot only), `WriteBuf<N>`, `Written`, `ActuatorFlags`, `TimeView`; N ≤ 64 const assert; rerun the R1 layout probes on the new shape | FR-006 to FR-010 (SRS FR-10.2, FR-2.1, FR-2.6, FR-3.3) |
| A4 | Agent | `call.rs`: `CFn<N, D>`, `call`, `Status`, `ForeignError`, `foreign!` (extern block with the fixed signature and the ABI check). Tests with Rust `extern "C" fn` doubles: error clears the buffer, status mapping | FR-003, FR-011, FR-012, FR-022 (SRS FR-10.1, FR-2.5, NFR-2) |

### B. C and C++ (feature `build`)

| # | Who | Work | FR |
|---|---|---|---|
| B1 | Agent | `build.rs` module: C header and C++ header from the field list; `kmr_ffi_abi.rs`; `Build::robot(..).compile(&[..])` over `cc` | FR-015, FR-016 (SRS FR-10.1, FR-6.1) |
| B2 | Agent, blocked on kmr_model lib entry | N in the build script from a preset name or descriptor path, through kmr_model's library (R4) | FR-015 (SRS FR-6.1) |
| B3 | Agent | `examples/c` crate: `swing.c`, layout probes N = 1, 3, 4, 10, 64, C error path; compile-fail for a wrong N (SC-005) | FR-024, SC-005 (SRS FR-10.4, FR-10.2) |
| B4 | Agent | `examples/cpp` crate: `KMR_CPP_CONTROLLER`, typed `Q`/`Qd`/`Tau`, a throwing body returns an error | FR-016, SC-007 (SRS NFR-2) |

### C. Glue in kmr_api (blocked on HUMAN core readers)

| # | Who | Work | FR |
|---|---|---|---|
| C1 | HUMAN | Core: Time getters (core #14), per-field write mask (core #2), History slot per tick (core #3), per-actuator age `Option<u32>` (core FR-018, K3.5), fault flags per slot with envelope per actuator (core #13, K4.1), `FieldId` on `Slot` in **`state/field.rs` (invariant)** (core K7.1) | FR-009, FR-010, FR-011, FR-014 |
| C2 | Agent | kmr_api feature `ffi` = `dep:kmr_ffi`; `kmr_api::ffi` re-exports; `ffi::call(f, time, state, ctx)` fills, calls, applies, maps `Status` to `shutdown!` / `emergency_stop!` | FR-001, FR-003, FR-011, FR-023 (SRS FR-10.1, FR-4.1, FR-4.2) |
| C3 | Agent | Exhaustive `match` on `FieldId` in the apply step; compile-fail when a field lacks its FFI line (SC-004) | FR-014 (SRS FR-10.2) |
| C4 | Agent | Glue tests: SC-001 (C twin, inline and inline-with-context, 1000 ticks), SC-006 (10^8 ticks, scripted clock), stale actuator, "no sample" (`samples == 0`) | SC-001, SC-006 (SRS FR-2.5, FR-3.3) |

### D. Python and Lua (features `python`, `lua`)

| # | Who | Work | FR |
|---|---|---|---|
| D1 | Agent | `python.rs`: `Bridge::load(path, function)`, per-call refill of persistent lists, `PyErr` → `ForeignError` | FR-017, FR-019, FR-020 (SRS FR-10.1, FR-10.3) |
| D2 | Agent | `lua.rs`: same over `mlua`, 1-based tables, `send` feature for the threaded form | FR-018, FR-004 (SRS FR-10.3, FR-1.2) |
| D3 | Agent | `examples/python`, `examples/lua`; SC-002 against the Rust twin | FR-024, SC-002 |

### E. Docs and measures

| # | Who | Work |
|---|---|---|
| E1 | Agent | `docs/decisions/0004-ffi-boundary.md`: copy vs share, header production, integer time, status signals, bridge state (FR-026 (SRS NFR-6)) |
| E2 | Agent | Crate docs: crash/abort leaves torque as it was (FR-025 (SRS NFR-2, FR-5.10)); each bridge's cost (FR-005 (SRS FR-10.3)) |
| E3 | Agent (bench) | Empty C forwarder for NFR-8 (d) (SC-003) |
| E4 | HUMAN | FR-10.4 run against the ROS2 baseline (D14) (SC-008) |
| E5 | Agent | `/speckit-ov-c4-code` for kmr_ffi |

## Needs from other modules

| From | What | Step |
|---|---|---|
| HUMAN, `kmr_core` | Readers for every History depth, per-actuator age, stale and fault flags per slot; Time getters; setters that set the per-field mask | C1 |
| HUMAN, `kmr_core` | `FieldId` (or an associated const on `Slot`) in `state/field.rs` (invariant), so the FFI field list is checked exhaustively; core plan K7.1 | C1, C3 |
| HUMAN, `kmr_core` | Not needed: `#[repr(transparent)]` on `Q`/`Qd`/`Tau` (core FR-016). The copy design works without it; it stays core's own requirement | — |
| Agent, `kmr_api` | Feature `ffi` (kmr_api FR-072), `ffi-build` for build scripts, `ffi-python`, `ffi-lua` (kmr_api plan G1, G2); glue `ffi::call`; History, age and flag readers on the `State` wrapper | C2, D1, D2 |
| Agent, `kmr_model` | A library entry (not a proc macro) that gives N for a preset name or descriptor path, the same N `robot!` gets. Needs the kmr_model lib + macro crate split; kmr_model research R12, open, pending D12 | B2 |
| Agent, `bench` | The NFR-8 (d) comparison run (SC-003) | E3 |
| `controllers` | The FR-10.4 baseline protocol (D14) | E4 |
| Repo, HUMAN | Workspace member, clippy override, constitution amendment | A1 |

## Project Structure

### Documentation (this feature)

```text
specs/006-kmr-ffi/
├── spec.md              # authority for this plan
├── plan.md              # this file
├── research.md          # Phase 0: R1..R14
├── data-model.md        # Phase 1: types that cross, lifecycle of one call, bridges
├── quickstart.md        # Phase 1: validation per step
├── contracts/
│   ├── c-abi.md         # generated C/C++ headers, function signature, status codes
│   ├── rust-surface.md  # kmr_api::ffi: foreign!, call, build helper, errors
│   └── bridges.md       # Python and Lua script contract
└── checklists/          # untouched
```

### Source Code (repository root)

```text
kmr_ffi/                         # new crate, SRS module kmr_ffi/**
├── Cargo.toml                   # features: build (cc), python (pyo3), lua (mlua)
├── clippy.toml                  # HUMAN: heap override like kmr_api's
├── src/
│   ├── lib.rs                   # crate doc, re-exports
│   ├── fields.rs                # for_each_field!, for_each_flag!: the one declaration
│   ├── view.rs                  # Slot, ReadView, WriteBuf, Written, ActuatorFlags, TimeView
│   ├── call.rs                  # CFn, call, Status, ForeignError, foreign!
│   ├── build.rs                 # feature build: headers, ABI file, cc wrapper
│   ├── python.rs                # feature python: Bridge
│   └── lua.rs                   # feature lua: Bridge
└── examples/
    ├── c/                       # crate: Cargo.toml, build.rs, src/main.rs, swing.c, probes
    ├── cpp/                     # crate: Cargo.toml, build.rs, src/main.rs, swing.cpp
    ├── python/                  # crate: Cargo.toml, src/main.rs, swing.py
    └── lua/                     # crate: Cargo.toml, src/main.rs, swing.lua

kmr_api/                         # agent, existing crate
├── Cargo.toml                   # features ffi, ffi-build, ffi-python, ffi-lua
└── src/ffi.rs                   # re-exports, fill/apply glue, Status → signal macros
```

**Structure Decision**: one new crate, `kmr_ffi`, with no kmr dependency (R9), so kmr_api can
depend on it optionally without a cycle and the glue that needs `State` internals stays in
kmr_api (kmr_api FR-072). Each example is its own small crate because a C example needs its own
`build.rs`.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| Heap in a crate other than kmr_api (Principle II) | SRS FR-10.3 allows it; an interpreter allocates | No heap: an embedded interpreter cannot run without it |
| A `build.rs` in the user's crate (SRS FR-6.1 says none) | C/C++ sources must be compiled against this build's N (D12 A) | Pointer + length (D12 C) loses the compile-time N check and puts a length test in every C function |
| The robot is named twice (`build.rs` and `robot!`) | Build scripts run before `robot!` expands | A const assert makes a mismatch a `cargo check` error (R1, proved) |
