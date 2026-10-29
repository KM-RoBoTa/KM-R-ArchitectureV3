# Implementation Plan: kmr_api, the public user API

**Branch**: none created (spec dir `specs/004-kmr-api`; work sits on `agent/robot-macro`) |
**Date**: 2026-10-28 | **Spec**: [spec.md](spec.md)

**Input**: `specs/004-kmr-api/spec.md` (coordinated 2026-10-28). Authority above it:
`docs/srs/SRS.md`. Designs it must fit: `specs/002-kmr-core/design-first-motor.md`,
`specs/002-kmr-core/design-const-generic-n.md`, `specs/003-kmr-model/` (plan, contract
`robot-macro.md`), `specs/001-kmr-adapters/` (plan, contracts). Review of the uncommitted code:
`specs/003-kmr-model/review-2026-10-28.md`.

## Summary

kmr_api is the one crate user code imports. Three gaps matter first, in this order:

1. **Critical path step 9.** `run()` simulates even with bus plugins. It must hand the
   Dynamixel adapter, bound to the `Bus::by_id` serial, to the core's `run_on`, forward the
   `Time` getters, add `Deg`/`Cm`/`M`, and drop both of its own simulation messages.
2. **`robot!`** must carry the hardware, not only N: the `Robot` alias names a hidden `Spec`
   type that builds the adapter, so `run()` picks hardware without a Cargo switch. It gains the
   `hardware = ..` argument (D7 A), the count check and the N ≤ 64 assert.
3. **The rest**, in SRS delivery order: faults and age (C), Zenoh (Z), fallbacks and envelopes,
   security, the `ffi` feature.

The risky claims were built on a scratch copy and pass (research R1, R2, R5, R7, R10, R15).

## Technical Context

**Language/Version**: Rust 2024, rustc 1.97.1 stable

**Primary Dependencies**: `kmr_core` (engine, HAL contract), `kmr_adapters` (presets, tables,
`Binding`, `Dynamixel::bind`), `kmr_model` (proc macro, through `robot!` only), `thiserror`.
Opt-in: `zenoh` and a triple-buffer slot (feature `zenoh`), `kmr_ffi` (features `ffi`,
`ffi-build`, `ffi-python`, `ffi-lua`), `presort-off` (forwarded to kmr_core, D11 B).
Dev: `trybuild`, `color-eyre`.

**Storage**: N/A. Hardware data is compile-time Rust in `main.rs`; Zenoh identities are
per-machine files named in the builder.

**Testing**: `cargo test -p kmr_api`, with and without `--features dynamixel`; a test-only
`Spec` over a recording adapter (the HAL seal is a public module); `trybuild` compile-fail
snapshots for `robot!` and the type-state; Zenoh tests on a local peer; allocation counting in
the tick.

**Target Platform**: Linux x86-64, aarch64, armv7, `std` (SRS section 4). `no_std` user path
out of this version (spec Scope).

**Project Type**: library crate in the Cargo workspace, plus its example

**Performance Goals**: 0 allocations and 0 locks from kmr_api inside the tick over 1 000 000
ticks at 1 kHz, gateway on (SC-003 (SRS NFR-1)); each slot side one atomic swap at most.

**Constraints**: no `dyn`, no heap per controller (FR-067 (SRS NFR-1)); heap only outside the
tick (`kmr_api/clippy.toml`); kmr_core untouched by agents (Constitution IV); errors before any
hardware I/O where they can be (FR-014 (SRS FR-6.4)).

**Scale/Scope**: N = 4 (XmBench) on the critical path; N from 1 to 64 after the const generic;
one robot per binary.

No NEEDS CLARIFICATION is left. Open spec markers and pending decisions are planned with the
recommended option and isolated (research.md):

| Decision | Assumed | One place to change |
| --- | --- | --- |
| D1 delivery gate | A: step 1 includes kmr_api's wiring | phase order of this plan (R21) |
| D4 N cap | A: 64 | one assert in `robot!` `@items` (R9) |
| D5 markers after open | A: compile-time marker for home state | `home.rs` + 2 example lines (R10) |
| D6 bus binding | B: `.bus("label")`, optional for one bus | `Bus::bus`; kmr_adapters `match_plugins` (R8) |
| D7 hardware without preset (spec Q1) | A: `robot!` `hardware = ..` argument | `robot!` `hardware` arms and `@table` (R7) |
| D3 envelope breach (spec Q3) | A: clamp + flag, in the core's stage | core stage; API passes data only (R17) |
| D2 joint-limit default | B: `run()` refuses unless every limited actuator has a policy (decided 2026-10-28 (for now): D2 B) | core `limits::require_policies` (K5.3); here only `set_limit_policy_all` (C4, R17) |
| D12, D13 FFI | A, A | `ffi.rs`, two feature lines (R19) |
| D16 `try_lock` in the example | replace | the example + `pub` on `Latest` (R20) |

## Constitution Check

*Before Phase 0, and again after Phase 1.*

| Principle | Status | Note |
| --- | --- | --- |
| 0. `.aiignore` read-only | Pass | Plan writes `specs/004-kmr-api/` only. Core needs are HUMAN tasks below |
| 0. Agent trailer | **Conflict, for the human** | Constitution requires one; AGENTS.md rule (b) and the human's final decision forbid it. No commit here. Agent commits follow the human until one side is amended (R22) |
| I. SRS authority | Pass | Every FR cites its SRS id; SRS problems in R23, not patched |
| I. Delivery order | **Conditional on D1** | Circular for step 1. Planned with D1 A: phase A is step 1, phase B waits for the first motor (R21) |
| I. Markers | Pass | Markers only where no torque is on: compile-time for home state (D5 A), the center-of-mass one unreachable (R10, R11) |
| II. Hot path | Pass | Static dispatch; `Guarded` is a named struct; slots are triple buffers (proved 0 allocations); Zenoh on its own threads |
| II. Heap | Pass | Heap in builder data and gateway threads only; `kmr_api/clippy.toml` allows it |
| III. No panics, errors returned | Pass | All finalization errors are `UserError`; bring-up errors come back after the core's guard cut torque |
| IV. Hand-engineered core | Pass | kmr_core items are HUMAN; invariant files named where touched (below) |
| IV. Small surface, no trait objects | Pass | One hidden trait (`Spec`), one sealed `Plugin`; no `dyn`; `BringUp` holds a `String`, not a boxed error |
| IV. `unsafe` | Pass, watch | Prefer a slot crate; a hand-written slot is flagged for human review (R15, R22) |
| V. Measured | Pass | SC-003, SC-010 on the NFR-1 bench (bench module); compile time with `--timings` after `Robot<H>` |
| VI. Secure by default | Pass | Default deny; identities per machine; crypto off the tick; FR-059 by construction (R18) |
| Technical constraints | Pass | Cross-builds need no hardware; the feature only compiles the SDK |
| Workflow | Pass | Agentic work only on `agent/<topic>`, maintainer authorization on the PR, PR into the human branch, `agentic` label |

Post-design re-check: unchanged. The design adds no `dyn`, no lock in the tick, no core edit by
an agent, no new crate.

## Order of work

"Blocked" names what must land first. HUMAN items are the maintainer's. Critical-path numbers
refer to `specs/spec-coordination-2026-10-28.md` §3.

### A. First motor (delivery step 1; critical path step 9)

| # | Who | Work | Blocked on | FR |
| --- | --- | --- | --- | --- |
| A0 | Agent | Drop the "no bus plugin: simulating" `eprintln!` from `run()`; keep the "not driven yet" one | core K1.6 (`NoHardware` prints "no bus plugin: simulating" at `open`); never before it, or no warning prints at all | FR-026 (SRS FR-6.3) |
| A1 | Agent | `Time`: private named field; six getters forwarded | core #14 (design commit 1) | FR-038 (SRS FR-3.3) |
| A2 | Agent | `units.rs`: `Deg`, `Cm`, `M` and per-second forms; `From` into `Q`, `Qd` | nothing | FR-033 (SRS FR-2.2) |
| A3 | Agent | `Bus` wraps `Binding`; `.bus(label)` (D6) | kmr_adapters C1 | FR-022 to FR-024 (SRS FR-6.3) |
| A4 | Agent | `spec.rs`: hidden `Spec`; `__DxlAdapter<NB>`, `__bind_dynamixel`, both cfg'd on kmr_api's `dynamixel`; `robot!` preset form emits `__Spec` and `Robot = kmr_api::Robot<__Spec>` | kmr_adapters B1, B2, B5, C2; controllers plan C0 (`main.rs` on `robot!(XmBench)`, R5) | FR-002, FR-005 (SRS FR-6.1, FR-5.1) |
| A5 | Agent | `Robot<H, C>`; `run()`: `finalize()`, then `run_on(NoHardware)` or `run_on(H::bind(..)?)`; remove the "not driven yet" message | core design commits 6 to 8; A3, A4 | FR-014, FR-026, FR-027 (SRS FR-6.3, FR-6.4) |
| A6 | Agent | `UserError`: `FeatureOff`, `NoHardwareDeclared`, `DtOutOfRange`, `BringUp { cause }`; `Debug` = `Display`; interim `dt` range check in `finalize()` | A5 | FR-015, FR-021 (SRS FR-3.1, FR-6.4) |
| A7 | Agent | Tests: port of the prototype's six cases on a recording adapter; kmr_api's own tests on `robot!(XmBench)` | A5, A6 | SC-005, SC-007 |
| A8 | Agent | Example: `Bus::by_id` line under `cfg!(feature = "dynamixel")` (R6) | A5 | FR-069, SC-001 (SRS FR-8.4) |
| A9 | HUMAN | Bench run: controllers demo with `--features hardware`; `Torque_Enable` = 0 on every id | everything above, kmr_adapters A, D | critical path step 11 |

### B. `robot!` and payloads (delivery step 2, after the first motor under D1 A)

Aligned with `specs/003-kmr-model/plan.md`; that plan owns the proc macro, the model facts, the
descriptor errors and the trybuild suite. This plan owns only the rows below.

| # | Who | Work | Blocked on | FR |
| --- | --- | --- | --- | --- |
| B1 | Agent | `hardware = \|hw\| ..` and `hardware = Table { .. }` arms; `@table` (`__NB`, `__M`, `__TABLE`, count check, `__Spec`); descriptor without hardware gives `NoHardwareDeclared` | A4; kmr_adapters B3 (with the group-slice overrides), B4 | FR-006, FR-007, FR-020 (SRS FR-6.1, FR-5.8) |
| B2 | Agent | `@items` behind a hidden internal macro (review #7); N ≤ 64 assert (D4) | 003 model! | FR-007; kmr_core FR-012 |
| B3 | Agent | `Spec` model consts from `model::*`. Kinds: `const _: Table<__NB, __M> = __TABLE.check_kinds(model::PRISMATIC);`, sets typed `[u64; MASK_WORDS]` (R9). Limits: `[Limit; N]` for the core's `set_limits`, typed `Q`/`Qd`/`Tau`; where `model::CONTINUOUS` has the bit, `lower` and `upper` are `None`, never the emitted `Q(0.0)` placeholder (a continuous joint is never clamped to 0) | 003 emit; kmr_adapters E6; core K5.2 for the limits consumer | FR-002 (SRS FR-6.1, FR-4.5) |
| B4 | Agent | `home.rs`: compile-time FR-2.7 marker (D5 A); remove the example's two calls | nothing | FR-036 (SRS FR-2.7) |
| B5 | Agent | `payload.rs`: `Payload::medium`, `heavy`; `register_payload`; link check in `finalize()`; COM marker unreachable (R11). Example registers its payload through the builder and loses the "spawns a thread" comment | 003 `LINK_NAMES` | FR-062 to FR-066 (SRS FR-8.1 to FR-8.4) |
| B6 | Agent | `Latest<T>` public (D16); example drops `try_lock` | Z1 (move it forward) | FR-069 (SRS NFR-1) |
| B7 | Agent, after HUMAN core const-generic steps 1 to 7 | `State<'a, N>`, `GroupView<'a, N, K>`, `Robot<N, H, C>` (gains `E` in C6), `Spec<N>`; drop `kmr_api::N` and the `N == JOINTS` assert; `three.urdf` becomes a passing test | core steps 1 to 7 (invariant files: `state/field.rs`, `controllers.rs`, `schedule.rs`, `schedule/macros.rs`) | FR-004 (SRS FR-6.1) |

### C. Faults, age, settings (API side of delivery steps 3 and 4)

| # | Who | Work | Blocked on | FR |
| --- | --- | --- | --- | --- |
| C1 | Agent | `State::age_at` (`Result<Option<u32>, StateError>`), `sample_age` (`Option<u32>`), `stale_at`; `None` = never read | core #4 (K3.5) | FR-031 (SRS FR-2.1, FR-2.6) |
| C2 | Agent | `Faults` view; `State::faults`, `prev_faults` | core #13 | FR-039 (SRS FR-13.4) |
| C3 | Agent | `State::read_every(i)` | core: Layout cadence readable through the handle | FR-042 (SRS FR-5.3) |
| C4 | Agent | `set_shutdown_hold`, `set_limit_policy[_group]` forwarders, and one call `set_limit_policy_all(Policy)` that sets the policy of every actuator (the bench and the controllers example need it under D2 B, decided 2026-10-28 (for now): D2 B) | core K4.3 (hold duration); core K5.2, K5.3 (#20) | FR-018, FR-019 (SRS FR-4.1, FR-4.5) |
| C5 | Agent | `bind()`, `Bound::adapter`, `Bound::run` | core `impl Hal for &mut A` or `run_on` returning the adapter | FR-041 (SRS FR-5.9) |
| C6 | Agent | `Robot<N, H, C, E>`: thread the core's `E` type-state (`Empty`, `Filled`; core K3.4) and forward the no-controller bound; drop `NoControllers`; drop the interim `dt` check; map the core's `RunError::Config` (`DtOutOfRange`, `NoPolicy`) to `UserError` and `RunError::Hal` to `BringUp` | core #15 (K3.4), #6 (K3.2), #7 (K3.3), K5.3 | FR-015, FR-016 (SRS FR-3.1, NFR-4) |
| C7 | Agent | Bench hooks: feature `presort-off = ["kmr_core/presort-off"]` (D11 B, assumed, pending human); re-export each preset's declared read and write costs (kmr_adapters FR-035) under `kmr_api::presets` | core K6.3; kmr_adapters: costs as public consts | bench FR-016 (SRS FR-5.3); kmr_adapters FR-035 (SRS NFR-11, FR-5.3) |

### Z. Zenoh (delivery step 5)

| # | Who | Work | Blocked on | FR |
| --- | --- | --- | --- | --- |
| Z1 | Agent | `slot.rs` (`Latest<T>`): triple buffer, superseded count; allocation-count test | nothing | FR-044 (SRS section 4) |
| Z2 | Agent | Feature `zenoh`; sealed `Plugin`; `Gateway` plugin, session thread, keys, messages, ring, counters | Z1 | FR-043, FR-045, FR-048 (SRS FR-7.1) |
| Z3 | Agent | Inbound and outbound stages over the core's stage points | core FR-053 | FR-044, FR-046 (SRS FR-7.1) |
| Z4 | Agent | Tests on a local peer; feature-off `cargo tree` check | Z2, Z3 | SC-004, SC-010 |

### E. Fallbacks and envelopes (delivery step 6)

| # | Who | Work | Blocked on | FR |
| --- | --- | --- | --- | --- |
| E1 | Agent | `fallback.rs`: `Guarded<F, G, P>`, `add_controller_or`, predicates | C2 | FR-049, FR-050 (SRS FR-13.5) |
| E2 | Agent | `envelope.rs`: `set_envelope[_group]`, handed to the core's stage (D3 A) | core #20, envelope flag | FR-051, FR-052 (SRS FR-13.5) |

### F. Security (delivery step 7)

| # | Who | Work | Blocked on | FR |
| --- | --- | --- | --- | --- |
| F1 | Agent | mTLS or QUIC config, ACL default deny, identity files | Z2 | FR-054, FR-060 (SRS FR-13.2, NFR-12) |
| F2 | Agent | Desired rule, seq check, counters, unauthorized flag | F1; core FR-052 pending word | FR-055 to FR-057, FR-061 |
| F3 | Agent | `add_remote_source` with required fallback; watchdog | E1, F2 | FR-058, FR-059 (SRS FR-13.3) |

### G. FFI feature (delivery step 8)

| # | Who | Work | Blocked on | FR |
| --- | --- | --- | --- | --- |
| G1 | Agent | Features `ffi`, `ffi-build`; `kmr_api::ffi` re-exports; glue `ffi::call` (D12 A, D13 A) | kmr_ffi plan A, C1 | FR-072 (SRS FR-10) |
| G2 | Agent | Features `ffi-python = ["ffi", "kmr_ffi/python"]`, `ffi-lua = ["ffi", "kmr_ffi/lua"]`; re-export `kmr_api::ffi::python::Bridge`, `kmr_api::ffi::lua::Bridge` (kmr_ffi contracts/rust-surface.md) | G1; kmr_ffi plan D1, D2 | FR-072 (SRS FR-10.1, FR-10.3) |

Last, per the constitution workflow: `docs/decisions/` entry for `Robot<H>` and the hardware
argument (Constitution V), then `/speckit-ov-c4-code` for kmr_api.

## Needs from other modules

| From | What | For |
| --- | --- | --- |
| HUMAN, kmr_core | design-first-motor commits 1 to 8: Time getters, write mask, History slot, `run_on<A: Hal>`, HAL call points, drop guard, SIGTERM/SIGHUP. Invariant file: `state/field.rs` (commit 3) | A1, A5 |
| HUMAN, kmr_core | K1.6: `NoHardware` prints "no bus plugin: simulating" once at `open` (kmr_core FR-042; the prototype core still says "no hardware adapter compiled in") | A0, SC-007 |
| HUMAN, kmr_core | `impl<A: Hal> Hal for &mut A` (open choice 3) or `run_on` returning the adapter | C5 |
| HUMAN, kmr_core | #4 age (`Option<u32>` per actuator), #13 fault flags (envelope per actuator) + hidden lock-free setter (FR-052), #15 no-controller bound (`E`), #6/#7 `dt` check and `RunError::{Config, Hal}`, #20 limit and envelope stage (`set_limits` with `Limit { lower: Option<Q>, upper: Option<Q>, .. }`, `set_envelopes`), K5.3 `require_policies`, a hold-duration setter, Layout cadence through the handle, K6.3 `presort-off` | C1 to C7, E2 |
| HUMAN, kmr_core | Two stage points per tick (FR-053); `outbound` takes `&RobotState<N>` so age and flags are reachable | Z3 |
| HUMAN, kmr_core | Const-generic N, steps 1 to 7. Invariant files: `state/field.rs`, `controllers.rs`, `schedule.rs`, `schedule/macros.rs` | B7 |
| Agent, kmr_adapters | A: torque off everywhere. B1, B2, B5 then C1, C2 (first motor): public `Table<NB, N>` with `pub actuators`, presets as `HARDWARE` consts, `Binding`, `Port`, `Dynamixel::bind`. B3, B4 after C (step 2): `const fn` overrides, count-check hook. E6: `check_kinds(self, [u64; MASK_WORDS]) -> Self`. Declared read and write costs as public consts | A3 to A5, B1, B3, C7 |
| Agent, kmr_adapters | Group-slice `const fn` overrides (`read_only_group(&[usize; K])` and the other per-actuator fields), now in kmr_adapters T021 | B1, FR-020 |
| Agent, kmr_adapters | `DxlRs::new()` public under its `dynamixel` feature; the `Hardware` alias and `robot.rs` removed (B5 of that plan) | A4 |
| Agent, kmr_model | The proc macro emits only N, `model::*` (incl. `LINK_NAMES`; kinds as `[u64; MASK_WORDS]`) and `actuator!`; preset model half with the preset's N. kmr_api's front door emits the `State`/`Robot` aliases, `__Spec` and the checks. 003's `robot-macro.md` lists the `hardware = ..` argument and `Robot<__Spec>` (kmr_api splits `hardware` off before the proc macro) | B1 to B3, B5 |
| Agent, controllers | Plan C0 (unblocked): switch `main.rs` to `robot!(XmBench)` before A4. Later C1: feature `hardware = ["kmr_api/dynamixel"]` | A4 (R5) |
| kmr_ffi | Crate, types sized by N, `Status`, build helper; `python`, `lua` features | G1, G2 |
| bench | NFR-1 bench for SC-003 and SC-010 | Z4 |

## Project Structure

### Documentation (this feature)

```text
specs/004-kmr-api/
├── spec.md
├── plan.md              # this file
├── research.md          # R1–R24
├── data-model.md        # builder, tick and Zenoh data; finalization order
├── quickstart.md        # validation per phase
├── contracts/
│   ├── builder.md       # Robot builder, run(), errors
│   ├── robot-hardware.md# robot! hardware argument and Spec (extends 003's robot-macro.md)
│   ├── handles.md       # State, Time, Sensors, units, faults
│   └── zenoh.md         # gateway, keys, timing, security
└── tasks.md             # /speckit-tasks, not created here
```

### Source Code (repository root)

```text
kmr_api/
├── Cargo.toml                   # features: dynamixel, zenoh, presort-off, ffi, ffi-build, ffi-python, ffi-lua
├── src/
│   ├── lib.rs                   # re-exports; drops the crate-wide allow(dead_code) (review #9)
│   ├── robot.rs                 # Robot<H, C> (then <N, H, C, E>), settings, registrations, finalize(), run(), bind()
│   ├── spec.rs                  # hidden Spec, __DxlAdapter, __bind_dynamixel (A4)
│   ├── robot_macro.rs           # robot! front door, hardware arms, @table, @items (A4, B1, B2)
│   ├── bus.rs                   # Bus over Binding (A3)
│   ├── error.rs                 # UserError, StateError (A6)
│   ├── time.rs                  # Time getters (A1)
│   ├── units.rs                 # Deg, Cm, M (A2)
│   ├── state.rs, group.rs       # exist; age, faults, cadence (C1 to C3)
│   ├── home.rs                  # FR-2.7 compile-time marker (B4)
│   ├── payload.rs               # rewritten (B5)
│   ├── fallback.rs, envelope.rs # E1, E2
│   ├── slot.rs                  # Latest<T> (Z1)
│   ├── zenoh/                   # gateway, keys, msg, security, watchdog (Z2, Z3, F1 to F3)
│   └── ffi.rs                   # G1, G2
├── tests/
│   ├── run.rs                   # A7
│   ├── robot_macro.rs, ui/      # B1, B2 (with 003's trybuild suite)
│   └── fixtures/
└── examples/complete_api_example.rs
```

**Structure Decision**: no new crate. kmr_api keeps the only public door; Zenoh and FFI stay
behind features inside it. kmr_core is not touched by this plan's agent tasks.

## Complexity Tracking

| Violation | Why needed | Simpler alternative rejected because |
| --- | --- | --- |
| A type parameter `H` on the public builder | FR-005: the adapter follows the robot, with no feature switch | A feature or a run-time lookup is the second switch FR-005 forbids |
| Interim `dt` check in kmr_api (owner: core) | FR-015 must hold from the first motor; `run_on` has no API error channel until core #7 | Waiting leaves a zero `dt` silently turned into 1 ms |
| Planning steps 2 to 8 before step 1 moved a motor | Task asks for the whole module plan | Plan only; implementation follows D1 |
