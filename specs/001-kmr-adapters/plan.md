# Implementation Plan: kmr_adapters, hardware adapters (Dynamixel first)

**Branch**: `001-kmr-adapters` (spec dir; work sits on `agent/robot-macro`) | **Date**: 2026-10-28 |
**Spec**: [spec.md](spec.md)

**Input**: `specs/001-kmr-adapters/spec.md` (re-specified 2026-10-28). Authority above it:
`docs/srs/SRS.md`. Core designs it must fit: `specs/002-kmr-core/design-first-motor.md`,
`specs/002-kmr-core/design-const-generic-n.md`. Contract: `kmr_core/src/hal.rs` (read-only).

Replaces the plan of 2026-10-26. `tasks.md` was regenerated on 2026-10-28; `fix-plan.md` is
stale. Old task ids: [Old tasks](#old-tasks-obsolete-or-reopened).

## Summary

SRS delivery step 1, adapter half. The Dynamixel adapter exists (86 tests green, scripted
backend). Three gaps stand between it and the first motor:

1. Torque can stay on. `torque_off` skips read-only actuators, and the real backend cannot cut
   an actuator this process did not switch on. The fake backend hides both.
2. The robot's hardware is hard-coded in `kmr_adapters/src/robot.rs`, with `PORTS` as a const.
   It must become the preset's hardware half, a table generic over N, overridable in `main.rs`.
3. Ports must come from bus plugins (`by_id`, `by_dev`), resolved at bring-up.

The plan fixes them in that order, then does the rest. Within B and C, what the first motor
needs comes first: B1, B2, B5, then C1 to C4 (kmr_api A3 to A5 need `Binding` and
`Dynamixel::bind`), then B3, B4 (overrides and count-check hook, needed by kmr_api B1 at
delivery step 2). No `kmr_core` edit is needed for steps
A to C. They fit the human core design unchanged: `run_on<A: Hal>`, the `Powered` guard
(`torque_off` then `close` on every exit after `open`), the hold target built from the latest
sample plus this tick's writes, and `Command` with its per-field write mask. The set of drivers
(D17, 2026-10-29, `specs/002-kmr-core/design-hal-set.md`) changes one thing here, after core
KS.1: the adapter is one member, and `check` returns its claim, not a whole-N layout (E11).

## Technical Context

**Language/Version**: Rust 1.97, edition 2024, stable

**Primary Dependencies**: `kmr_core` (HAL contract); `dxl_rs` (KM-RoBoTa, path dependency today,
optional behind the `dynamixel` feature); `rustix` (termios flush, pty in tests); `thiserror`,
`tracing`

**Storage**: N/A. Hardware data is Rust compile-time data (FR-028 (SRS FR-6.1)). The USB latency
timer is a sysfs file.

**Testing**: `cargo test -p kmr_adapters` on the strict scripted backend, no hardware, no C SDK;
compile-fail doctests for bad tables; pty tests for port resolution; bench runs by the human

**Target Platform**: Linux x86-64, aarch64, armv7, std (SRS NFR-5)

**Project Type**: library crate in the Cargo workspace

**Performance Goals**: 1 kHz tick; one write transaction per (bus, series) (SC-002 (SRS FR-5.3));
a write tick's I/O fits the tick on the NFR-1 bench (SC-003 (SRS FR-5.10, NFR-11))

**Constraints**: per-tick path without allocation, lock, sleep, retry or `dyn` outside SDK call
sites (FR-013 (SRS NFR-1, FR-5.10)); no panic (FR-033 (SRS NFR-2)); `f32` in SRS units (FR-021
(SRS FR-2.2))

**Scale/Scope**: N = 4 (XmBench) on the critical path; N ≤ 64 after core's const-generic N; up to
4 buses per process (`MAX_PORTS`); six Dynamixel models

No NEEDS CLARIFICATION is left. The three open spec markers are planned with the recommended
option, each isolated to one place ([research.md](research.md) R5, R7, R8):

| Marker | Assumed | One place to change |
|---|---|---|
| D8 / Q3 zero position | C: per-actuator `zero`, default model centre | `Zero::DEFAULT` and `units::origin` |
| D9 / Q2 default cadence | A: per-brand default, bench-confirmed | `dynamixel::config::DEFAULT_READ_EVERY` |
| D6 / Q1 bus binding | B: plugin names its bus label, optional for one bus | `binding::match_plugins` |
| D7 hardware without a preset | A: `robot!` `hardware = ..` argument over this crate's `Table` and `const fn` overrides | kmr_api's `hardware` arms; here only the table types (research R16) |
| D15 shipped bus line | A: the shipped `controllers/src/main.rs` has no bus line and simulates | quickstart § 5 and the bench's local line (research R17) |

## Constitution Check

*GATE: checked before Phase 0, re-checked after Phase 1. Passes with two items for the human.*

| Principle | Status | Note |
|---|---|---|
| 0. ov policy: `.aiignore` read-only | Pass | Steps A to C touch `kmr_adapters/` only. Core needs are listed for the human (below) |
| 0. ov policy: agent trailer | **Conflict, for the human** | The constitution requires an agent trailer on every agent commit. AGENTS.md rule (b) and the human's final decision forbid any AI or co-author trailer (`76bd454` allowed them; the working-tree AGENTS.md forbids them again). This plan makes no commit. Until the human amends one side, agent commits follow the human's decision |
| I. SRS authority | Pass | Every FR cites its SRS id. SRS problems are reported in research.md R15, not patched |
| I. Delivery order (SRS §8) | **Conditional on D1** | "Nothing below a line starts before the line above moves a motor" is circular for step 1 (D1). This plan assumes D1 option A: step 1 = contract + adapter + minimal state transition + exit paths. All adapter work here is inside step 1 either way |
| II. Real-time hot path | Pass | Static dispatch, fixed arrays, bitsets. SDK allocations stay at marked call sites (SDK boundary exemption) |
| III. No panics, errors returned | Pass | Bad tables fail const evaluation (a compile error, not a run-time panic). Bind errors are `run()` errors before I/O. After `open`, nothing panics. The strict fake returns errors instead of asserting, so a test unwinding through `Powered` cannot double-panic |
| IV. Hand-engineered core | Pass | No core edit. `impl Hal for &mut A` (core open choice 3) is the human's; only step D needs it |
| V. Measured, not guessed | Pass, one placeholder | `DEFAULT_READ_EVERY` is a placeholder until the NFR-1 bench (SC-003). `docs/decisions/0002-hal-contract.md` needs an update (step E) |
| VI. Secure by default | N/A | No network path |
| Technical constraints | Pass | Cross-builds without hardware: the `dynamixel` feature only compiles the SDK, never picks simulation (FR-031 (SRS FR-6.3)) |
| Workflow | Pass | Agentic work only on `agent/<topic>`, maintainer authorization on the PR, PR into the human branch, `agentic` label |

Post-design re-check: unchanged. The design adds no `dyn`, no heap outside SDK call sites, no
core edit, no new crate.

## Order of work

HUMAN steps are the human's. "Blocked" names what must land first. Critical-path numbers refer to
`specs/spec-coordination-2026-10-28.md` §3.

### A. Torque off everywhere (critical path step 2)

| # | Who | Work | FR |
|---|---|---|---|
| A1 | Agent | Strict fake: the scripted backend refuses what `DxlRs` refuses ([contracts/backend.md](contracts/backend.md) § Strictness). Existing tests that relied on leniency are rewritten, not deleted | FR-033 (SRS NFR-2) |
| A2 | Agent | Torque tests, red first: every id starts powered by an earlier run, one is read-only, and every exit after `open` (shutdown, estop, bring-up error, panic) ends with no id powered. PostInit writes torque off on a read-only actuator | FR-017 (SRS FR-4.2, FR-5.6, FR-5.8), FR-006 (SRS FR-5.2, FR-5.8) |
| A3 | Agent | `Backend::force_torque_off`. `torque_off` writes it to every actuator of every opened bus, read-only ones included. `torque_on` writes it to read-only actuators. `configure` no longer fails on a read-only actuator left powered; it warns. A2 goes green | FR-017, FR-006, FR-033 |
| A4 | Agent, blocked on HUMAN critical step 1 | `DxlRs::force_torque_off` over the new `dxl_rs` primitive, under the `dynamixel` feature. Until then A3 ships it as today's `unpower` fallback, so the feature build stays green | FR-017 |

### B. Preset hardware half (first motor: B1, B2, B5)

| # | Who | Work | FR |
|---|---|---|---|
| B1 | Agent | `dynamixel::Table<NB, N>`: buses (label, baud, `Option` cadence) and actuators (bus, id, model, mode, torque, zero). `Table::check` replaces `validate`; `PORTS` leaves the table. `DEFAULT_READ_EVERY` (D9) and `Zero::DEFAULT` (D8) are defined here | FR-023, FR-027, FR-028 (SRS FR-6.1, FR-6.4) |
| B2 | Agent | `presets::XmBench::HARDWARE: Table<1, 4>` replaces `robot.rs`. `Preset::N` stays, derived from the table, until `robot!` takes N from `kmr_model` | FR-023 (SRS FR-6.1) |
| B5 | Agent | `Dynamixel::new(backend, table)` takes the table by value. `robot.rs`, the `Hardware` alias and its `Default` go. `DxlRs::new()` public under `dynamixel` | FR-022 (SRS FR-6.1) |

B3 and B4 follow step C, below.

### C. Plugin port resolution (first motor)

| # | Who | Work | FR |
|---|---|---|---|
| C1 | Agent | `Binding { port: Port, baud, bus }`, `Port::{ById, ByDev}`. `kmr_api::Bus` wraps it | FR-029 (SRS FR-6.3) |
| C2 | Agent | `Dynamixel::bind(backend, table, &[Binding])`: label matching (D6, in `match_plugins` only), plugin baud checked against every model on its bus, naming the actuator index. No I/O | FR-030, FR-027 (SRS FR-6.3, FR-6.4) |
| C3 | Agent | `open` resolves `ById` under `/dev/serial/by-id`, takes `ByDev` as given, pre-flights both, refuses two buses on one device | FR-004, FR-008, FR-029, FR-030 (SRS FR-5.10, FR-6.3) |
| C4 | Agent | Tests: unknown label, unbound bus, extra plugin, two buses without labels, one bus without a label, bad baud, `by_dev` on a pty, shared device | SC-004 (SRS FR-13.4) |

### B, rest. Overrides and count-check hook (after C; kmr_api B1, delivery step 2)

| # | Who | Work | FR |
|---|---|---|---|
| B3 | Agent | `const fn` overrides on `Table`: `baud`, `id`, `mode`, `read_only`, `read_every`, `zero`, and the group forms over an index array `&[usize; K]` (`read_only_group`, `mode_group`, `zero_group`; kmr_api FR-020 (SRS FR-5.8)). Each re-runs `check` | FR-026 (SRS FR-6.1) |
| B4 | Agent | Count-check hook: `Table::actuators` is a public `[Actuator; N]`, so `robot!` writes `const _: &[Actuator; N] = &__TABLE.actuators;`. Compile-fail doctest for E0308 in `config.rs`; the `robot!` line is kmr_api's | FR-024 (SRS FR-6.1, FR-6.4) |

### D. Through the engine (after HUMAN core steps 3 to 8)

| # | Who | Work |
|---|---|---|
| D1 | Agent, blocked on core #8 and `impl Hal for &mut A` | The four end-to-end tests of the core prototype, through `run_on(&mut adapter)` on a pty: shutdown order, estop skips write and hold, missing actuator, panicking controller |
| D2 | kmr_api, not this crate (its plan A5, critical path step 9) | No kmr_api edit here, `kmr_api/src/robot.rs` included. kmr_api A5 wires `run()` through the `__Spec` that `robot!` emits (A4): plugins → `Dynamixel::bind` → `run_on`; no plugin → `NoHardware`; plugins without the `dynamixel` feature → `run()` error, never simulation (FR-031). This crate's part: B5, C2 |
| D3 | HUMAN (critical path step 11) | Bench run; `Torque_Enable` = 0 on every id after shutdown and after estop |

### E. The rest

| # | Who | Work | FR |
|---|---|---|---|
| E1 | Agent | Zero offset in `units` (D8) | FR-005 (SRS FR-5.2) |
| E2 | Agent | Protocol 1 health under the SDK boundary exemption: one warning, voltage and temperature skipped | FR-006 (SRS FR-5.2) |
| E3 | HUMAN, then Agent | `dxl_rs` block sync read (10 bytes at 126); one read per group; costs follow | FR-010, FR-035 (SRS FR-5.3, NFR-11) |
| E4 | HUMAN, then Agent | `dxl_rs` pushed; root `Cargo.toml` git dependency; CI installs libclang and cross-builds | SC-007 (SRS NFR-5) |
| E5 | Agent, after core const-generic N | `Dynamixel<B, NB, N>`, `impl Hal<N>`, `DxlRs` capacity from N (design-const-generic-n step 9) | FR-022 (SRS FR-6.1) |
| E6 | Agent | Kind check: `pub const fn check_kinds(self, prismatic: [u64; MASK_WORDS]) -> Self` rejects a prismatic actuator, naming its index. kmr_api's `robot!` calls it as `const _: Table<__NB, __M> = __TABLE.check_kinds(model::PRISMATIC);` (a `const _: () = ..` form is E0308) | FR-021 (SRS FR-2.2) |
| E7 | Agent | SC-008 allocation count over 10 000 ticks on the scripted bus | SC-008 (SRS NFR-1) |
| E8 | Agent | Decision record 0002, ARCHITECTURE, ROADMAP, then `/speckit-ov-c4-code` | constitution V |
| E9 | HUMAN | Bench: SC-001, SC-003 (and the `DEFAULT_READ_EVERY` value), SC-005 | |
| E10 | Agent, after B2 | Declared costs as public consts: `XmBench::READ_COST`, `XmBench::WRITE_COST` (`Duration`), computed in const evaluation from `HARDWARE` and the backends' read plan; a test pins them to the `Layout` costs `check` declares. kmr_api C7 re-exports them; the bench records them (T067) | FR-035 (SRS NFR-11, FR-5.3) |
| E11 | Agent, after core KS.1 | The adapter as one member: `check` returns its `Claim` (owned indices, local groups, cadence, costs); `read`, `read_all`, `write` touch only owned slots; a multi-brand table names each actuator's index (T069) | FR-036, FR-002 (SRS FR-5.1, FR-5.4) |

## Needs from other modules

| From | What | Step |
|---|---|---|
| HUMAN, `dxl_rs` | Push `feat/battery-status`; add `force_torque_off` (write `Torque_Enable` = 0 without a `TorqueOn` handle) | A4, E4 |
| HUMAN, `dxl_rs` | One 10-byte block sync read at address 126 | E3 |
| HUMAN, `kmr_core` | Core plan #2, #3, #8, #9, #10, #11 per design-first-motor; optional `impl Hal for &mut A` | D1 |
| HUMAN, `kmr_core` | Const-generic N | E5 |
| HUMAN, `kmr_core` | Not needed by this adapter: the plugin-baud run-time override of kmr_core FR-039. The baud enters at `bind` (research R6) | — |
| HUMAN, `kmr_core` | K3.8 removes `Overrides::read_every`; `check` reads the cadence from the table only | B1 (T020) |
| HUMAN, `kmr_core` | KS.1: the hardware as a list of drivers, the coverage check, `Claim` (design-hal-set.md) | E11 (T069) |
| Agent, `kmr_api` | `robot!` count check on `Table::actuators` (kmr_api B1); `Bus::bus(label)` (A3); `run()` wiring through `__Spec` (A4, A5). None of it is edited by this plan | B4, C1, D2 |

## Project Structure

### Documentation (this feature)

```text
specs/001-kmr-adapters/
├── spec.md              # authority for this plan
├── plan.md              # this file
├── research.md          # Phase 0: R1..R15
├── data-model.md        # Phase 1: table, binding, adapter state, lifecycle
├── quickstart.md        # Phase 1: validation per step
├── contracts/
│   ├── hal-conformance.md   # how the adapter answers each Hal call, every exit
│   ├── hardware-table.md    # public table types, presets, overrides, count-check hook
│   ├── port-binding.md      # Binding, bind(), errors, D6 isolation
│   └── backend.md           # private SDK seam, force_torque_off, strict fake
├── fix-plan.md          # stale, retired later (not touched)
└── tasks.md             # regenerated 2026-10-28
```

### Source Code (repository root)

```text
kmr_adapters/
├── Cargo.toml                 # feature dynamixel = ["dep:dxl_rs"]: compiles the SDK only
└── src/
    ├── lib.rs                 # pub mod dynamixel, presets, binding; Hardware alias removed (B5)
    ├── binding.rs             # new (C1, C2): Binding, Port, match_plugins (D6)
    ├── presets/
    │   ├── mod.rs             # Preset trait (N until kmr_model owns it)
    │   └── xm_bench.rs        # new (B2): XmBench::HARDWARE; READ_COST, WRITE_COST (E10)
    ├── robot.rs               # removed (B5)
    └── dynamixel/
        ├── mod.rs             # Dynamixel: Hal; bind (C2); torque_off/torque_on (A3)
        ├── config.rs          # Table, Bus, Actuator, Zero, DEFAULT_READ_EVERY, check (B1, B3)
        ├── units.rs           # origin(actuator) (E1)
        ├── port.rs            # resolve by_id, by_dev pre-flight (C3)
        ├── io.rs              # unchanged on the critical path
        ├── backend.rs         # + force_torque_off (A3)
        ├── dxl_rs.rs          # + force_torque_off (A4, blocked)
        ├── scripted.rs        # strict (A1)
        └── tests.rs           # torque (A2), table (B), binding (C4)
```

**Structure Decision**: no new crate. `binding.rs` and `presets/` are new modules in
`kmr_adapters`. Dependency direction stays `kmr_core` ← `kmr_adapters` ← `kmr_api` ←
`controllers`.

## Old tasks: obsolete or reopened

From `tasks.md` of 2026-10-26 (not edited here; `/speckit-tasks` regenerates it).

**Obsolete** (contradict the 2026-10-28 spec or the core designs):

| Task | Why |
|---|---|
| T008 | Superseded by core plan #2 (design-first-motor) |
| T009, T022, T039 | `set_read_every` builder knob: cadence is a table field (FR-026), not a builder setting |
| T010, T011 | Not adapter tasks. `ZeroReadCadence` is obsolete: K = 0 cannot be written (`NonZeroU32`). `RunError<E>` is not obsolete: core K3.3 (HUMAN) adds `RunError::{Config, Hal}`; until then `run_on` returns `A::Error` |
| T012, T013 | Superseded by core #8 to #10 (`Powered` guard, hold target, five bus checks) |
| T017 | `robot.rs` with `PORTS`: replaced by the XmBench hardware half (B2) and plugins (C) |
| T021 | `Hardware` alias chosen by a feature: plugins decide, never a feature (FR-031) |
| T023 (`CadenceMissing` part) | A bus without a cadence takes the brand default (D9) |
| T029 | Bench run through `robot.rs` and a feature flag: now `robot!(XmBench)` and `Bus::by_id` (D3) |
| T035 ("read-only never torqued") | Read-only actuators now get torque written off at PostInit (FR-006) |
| T046 (example-frozen check) | FR-019 retired; the example names `robot!(XmBench)` |

**Marked done, not done (reopened)**: T001 (git dependency: still a path dependency), T028 (one
10-byte sync read: three reads today), T032 (Protocol 1 health fails), T033 (torque off skips
read-only actuators), T041 (cross job not green without `dxl_rs`).

**Partly kept**: T015 (validation, without `ports`), T020 (errors, minus `CadenceMissing`), T030
(port resolution, `PORTS` gone), T044 (decision record needs the new design).

**Still valid**: T036, T040, T042, T043, T047.

## Complexity Tracking

| Deviation | Why needed | Simpler alternative rejected because |
|---|---|---|
| Private `Backend` trait, two impls | The strict fake is how A2, SC-002 and SC-004 are tested without hardware | Calling `dxl_rs` directly: no fault injection, no transaction count |
| SDK allocates and blocks ~34 ms on a missing servo | Only SDK driving both protocols now; SDK boundary exemption | Waiting for `dxl_rs_v2` blocks the first motor |
| `dynamixel` Cargo feature | `cargo test --workspace` must not need libclang or the C SDK | Always compiling the SDK breaks CI on machines without it |
| Adapter fixed at N = `JOINTS` until core's N lands | `Hal` is not generic yet | The table is already generic over N, so only the adapter waits (E5) |
| Software zero offset, no Homing Offset write | `dxl_rs` cannot write that register | Register write needs an SDK change and is not on the critical path |
