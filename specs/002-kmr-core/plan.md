# Implementation Plan: kmr_core, the engine

**Branch**: `002-kmr-core` (spec dir; work sits on `agent/robot-macro`) | **Date**: 2026-10-28 |
**Spec**: [spec.md](spec.md)

**Input**: `specs/002-kmr-core/spec.md` (coordinated 2026-10-28). Authority above it:
`docs/srs/SRS.md` (patched 2026-10-27), module `kmr_core`. Human designs folded in:
[design-first-motor.md](design-first-motor.md) and [design-const-generic-n.md](design-const-generic-n.md).
Order: `specs/spec-coordination-2026-10-28.md` §3 (critical path). Earlier plan:
`specs/kmr-core-completion-plan.md`, kept where valid ([research.md](research.md) R20).

**Owner of every task: HUMAN.** `kmr_core/` is in `.aiignore`. This plan designs; it edits
nothing in the core. Invariant files are named on every step that touches them.

## Summary

The engine already has the builder, the six-phase HList schedule, the absolute-deadline clock
and the signal bus. The runtime calls no hardware: History is never filled, Desired is never
applied, `JOINTS = 4` is a literal.

The plan has seven milestones, in critical-path order:

1. **M1 First motor** (critical path steps 3 to 8, SRS delivery step 1 under D1 A).
   `design-first-motor.md` commits 1 to 8, plus one estop check before `torque_on`. N stays 4.
2. **M2 N from the robot line** (SRS delivery step 2, core half). `design-const-generic-n.md`
   steps 0 to 10, with the bridge `N = JOINTS`.
3. **M3 State transition complete** (delivery step 3). `dt` range, `RunError`, type-state empty
   check, sample age, `Kp`/`Kd`, layout, no hot-path logging, real-time sleep.
4. **M4 Signals, Time, faults** (delivery step 4). Fault flags per slot, the external pending
   word, the hold duration.
5. **M5 API hooks** (supports delivery steps 5 and 6). Stage points, limit and envelope stage.
6. **M6 Gates**. `no_std` build in CI, dynamic-scheduler review, pre-sort switch.
7. **M7 FFI hook** (supports delivery step 8). A public `FieldId` for kmr_ffi's exhaustive match.

Each milestone leaves the workspace green. Nothing below M1 starts before M1 moves a motor
(Constitution I, SRS §8, assuming D1 A).

## Technical Context

**Language/Version**: Rust 1.97.1, edition 2024, stable only

**Primary Dependencies**: `frunk` (HList; `frunk_core` without default features for `no_std`),
`circular-buffer`, `thiserror`, `derive_more`, `tracing` (std only), `ctrlc` with the
`termination` feature (std only), `rstest` (tests)

**Storage**: N/A. Fixed arrays and `'static`s; no heap (root `clippy.toml`)

**Testing**: `cargo test -p kmr_core` with the scripted `Host` and a recording fake `Hal`
(`Recorder`); own-process tests in `kmr_core/tests/` for the global bus and signals; compile-fail
doctests; `cargo build --target thumbv7em-none-eabihf --no-default-features` for `no_std`

**Target Platform**: Linux x86-64, aarch64, armv7 with `std` (SRS NFR-5); one bare-metal target
with CAS for the `no_std` CI build (assumed `thumbv7em-none-eabihf`, research R13)

**Project Type**: library crate (the engine) in one Cargo workspace

**Performance Goals**: 1 kHz tick on the NFR-1 bench (RP4); per tick one atomic load per bus
check (five checks), one pass over N for the write batch, zero allocation, zero lock
(SC-005 (SRS NFR-1))

**Constraints**: no heap, no `dyn`, no lock, no hot-path logging (FR-054 (SRS NFR-1, NFR-10));
no `unwrap`/`panic`/unchecked index (FR-055 (SRS NFR-2)); `panic = "unwind"` in release
(FR-043 (SRS NFR-2)); N in 1..=64 (FR-012, D4)

**Scale/Scope**: N = 4 (XmBench) for M1; N from 1 to 64 after M2; `HISTORY_DEPTH` = 4, one
capacity constant (FR-013 (SRS §8 decision 0001)); 25 source files today, about 3 600 lines

No NEEDS CLARIFICATION is left. Every open point is planned with the recommended option and
isolated to one place ([research.md](research.md), "assumed, pending human"):

| Pending | Assumed | One place to change |
|---|---|---|
| D1 delivery gate | A: M1 is delivery step 1 | milestone order in this plan |
| D2 joint limit, no policy | B: `run()` refuses, before I/O, unless every limited actuator has a policy (same assumption in every plan) | `limits::require_policies` (finalization) |
| D3 envelope breach | A: clamp + envelope flag | `Breach::Envelope` arm of the pre-write stage |
| D4 N cap | A: 64, `MASK_WORDS = 1` | `hal::MASK_WORDS` and the `Mask::<N>::CHECK` assert |
| D5 markers after `open` | A: typed error, no marker | `RobotState::home_state` returns `StateError::Undecided` |
| D11 pre-sort gain | B: opt-in `presort-off` feature | `Batch::new` |
| D13 foreign stop | A: no core change | none (`signal.rs` untouched) |
| FR-042 no-hardware seed | no sample until first read | `History::ever_read` mask gate |
| first-motor open choices 1 to 7 | the design's recommendation, OC4 added | see research R4 |

## Constitution Check

*GATE: checked before Phase 0, re-checked after Phase 1. Passes, with three items for the human.*

| Principle | Status | Note |
|---|---|---|
| 0. `.aiignore` read-only | Pass | Plan files only, in `specs/002-kmr-core/`. Every task is HUMAN |
| 0. Agent trailer | **Conflict, for the human** | The constitution requires an agent trailer. AGENTS.md rule (b) and the human's final decision forbid any AI or co-author trailer (`76bd454` allowed them; the working-tree AGENTS.md forbids them again). Recorded, not fixed. Core commits are human commits, so no trailer question arises for this module |
| I. SRS authority | Pass | Every FR cites its SRS id. 12 SRS problems reported in research R19, not patched |
| I. Delivery order | **Conditional on D1** | §8 gate is circular for step 1. Assumed D1 A: M1 = contract + minimal state transition + exit paths. If D1 B, M1 keeps its content and only its label changes |
| I. Markers | Pass with D5 | No `unimplemented!` after `open` (D5 A). Two `todo!()` today (`state/history.rs:24`, `state/robot_state.rs:59`) become a typed error in M3 |
| II. Real-time hot path | Pass | Static dispatch, fixed arrays, one atomic load per check. M3 removes per-tick `trace!`/`debug!` |
| II. `no_std` without `alloc` | Pass, M6 | Prototype proved const N and the type-state on `thumbv7em`; `thumbv6m` has no CAS (proved, research R13) |
| III. No panics | Pass | Exit paths end in `Powered`'s `Drop`; adapters guarantee no panic there (adapters contract). Release keeps `panic = "unwind"` |
| IV. Hand-engineered core | Pass | Every task HUMAN; invariant files named per step |
| IV. YAGNI | Pass | No threads, no sensor contract, no dynamic scheduler in this version |
| V. Measured | Pass, bench-bound | Overhead, jitter, pre-sort cost measured on the NFR-1 bench (bench FR-016, D11). Decision record 0003 for N (M2 step 0) |
| VI. Secure by default | N/A | No network path in the core; external flags come through `kmr_api` |
| Workflow | Pass | Core commits are human; plan makes no commit |

**AGENTS.md drift (for the human, not fixed):** AGENTS.md defends a runtime `L::LEN == 0` check;
SRS NFR-4 and §8 decide a type-state bound. M3 implements the bound (research R8). AGENTS.md also
says `Robot::run()` checks nothing yet, which stays true until M3.

Re-check after Phase 1: unchanged. The design adds no dependency, no `dyn`, no heap. It touches
no invariant file beyond those the two human designs already name, plus `state/field.rs` for
`Kp`/`Kd` (M3) and `FieldId` (M7), and `signal.rs` imports for `no_std` (M6). `schedule.rs` is
not touched for the empty check.

## Project Structure

### Documentation (this feature)

```text
specs/002-kmr-core/
├── spec.md                      # coordinated 2026-10-28
├── design-first-motor.md        # human design, prototyped (M1)
├── design-const-generic-n.md    # human design, prototyped (M2)
├── plan.md                      # this file
├── research.md                  # decisions, assumptions, SRS problems, prototype results
├── data-model.md                # entities, masks, flags, run lifecycle
├── quickstart.md                # validation per milestone
└── contracts/
    ├── hal.md                   # the hardware contract as the engine drives it
    └── engine-surface.md        # what kmr_api, adapters and kmr_ffi may name
```

### Source Code (repository root)

```text
kmr_core/                         # HUMAN-only (.aiignore)
├── Cargo.toml                    # M1: ctrlc "termination"; M6: std / no_std features
├── src/
│   ├── lib.rs                    # M2: drop JOINTS re-export; M6: #![no_std] gate
│   ├── clock.rs                  # M1: Time getters; M3: dt range; M4: hold wait
│   ├── error.rs                  # M3: RunError<E>, DtOutOfRange, StateError::Undecided
│   ├── hal.rs                    # M1: impl Hal for &mut A; M2: <const N>; M4: Faults<N>
│   ├── robot.rs                  # M1: run_on; M2: Robot<N, CS>; M3: E type-state
│   ├── runtime.rs                # M1: guard, call points, 5 bus checks; M5: stages, limits
│   ├── controllers.rs            # INVARIANT, M2 only: Env<'a, N>, ControlFn<N>
│   ├── schedule.rs               # INVARIANT, M2 only: RunPhase<N>, Drive<N>
│   ├── schedule/macros.rs        # INVARIANT (schedule), M2 only
│   ├── signal.rs                 # INVARIANT, M6 only: std:: imports become core:: (research R13)
│   ├── sealed.rs                 # INVARIANT, untouched
│   ├── sensors.rs                # untouched (FR-049 placeholder)
│   ├── config/user_config.rs     # M4: hold duration; M5: limit policies
│   └── state/
│       ├── field.rs              # INVARIANT: M1 Slot::mask; M2 generic methods; M3 Kp, Kd; M7 FieldId
│       ├── desired.rs            # M1 Written; M2 Desired<N>; M3 Kp, Kd
│       ├── history.rs            # M1 Copy slot; M2 State<N>; M3 age, ever_read; M4 flags
│       ├── joint_state.rs        # M3 Kp, Kd, f32 both ways, repr(transparent)
│       ├── robot_state.rs        # M1 engine block; M2 RobotState<N>; M3 age reads
│       └── test.rs
└── tests/
    ├── run_signals.rs            # M2: Robot::<4>::new()
    ├── foreign_ctrlc.rs
    └── termination.rs            # M1: SIGTERM, SIGHUP (new)
```

**Structure Decision**: one crate, unchanged layout. No new module except an optional
`limits.rs` in M5 for the pre-write stage. Dependency direction stays
`kmr_core` ← `kmr_adapters` ← `kmr_api` ← `controllers`; the core depends on no workspace crate.

## Work order

Every row is HUMAN. "Inv" names the invariant file touched. "CP" is the critical-path step of the
coordination page. "Done when" is the test that proves it ([quickstart.md](quickstart.md)).

### M1 First motor (CP 3 to 8; SRS delivery step 1 under D1 A)

Source: `design-first-motor.md` §5, unchanged except K1.9. N stays `JOINTS = 4`.

| # | Commit | FR | Inv | CP | Done when |
|---|---|---|---|---|---|
| K1.1 | `feat(core): read-only Time getters` (#14) | FR-029 (SRS FR-3.3) | – | 8 | `tick_and_overrun_count_follow_end_tick` |
| K1.2 | `feat(core): stop on SIGTERM and SIGHUP` (#11 half) | FR-035 (SRS FR-4.3) | – | 7 | `tests/termination.rs` |
| K1.3 | `feat(core): per-field write mask` (#2) | FR-021 (SRS Def. "Write mask") | **`state/field.rs`** (`Slot::mask`) | 3 | `a_write_sets_the_bit_of_its_field_only` and 2 more |
| K1.4 | `feat(core): command and clear after apply` (#2) | FR-019 (SRS FR-2.5) | – | 3 | `the_command_borrows_desired_and_clear_resets_values_and_masks` |
| K1.5 | `feat(core): one History slot per push, hold target` (#3) | FR-017, FR-022 (SRS FR-2.3, FR-2.6), FR-032 (SRS FR-4.1) | – | 4 | `each_pushed_slot_starts_as_a_copy_of_the_last`, `the_hold_target_is_...` |
| K1.6 | `feat(core): run_on over any adapter` (#8) + `impl Hal for &mut A`. `NoHardware` prints "no bus plugin: simulating" once, at `open`: the only simulation warning (the prototype's "no hardware adapter compiled in" goes); kmr_api A0 waits on this | FR-041, FR-050 (SRS FR-5.1, FR-5.9), FR-042 (SRS FR-6.3) | – | 5 | `a_check_error_is_returned_before_any_io`; a `NoHardware` run logs that text exactly once |
| K1.7 | `feat(core): bring-up calls behind a torque-off guard` (#9 startup, #10, #11) | FR-039, FR-043, FR-048, FR-020 (SRS FR-5.2, FR-5.6, NFR-2) | – | 6, 7 | 8 exit-path tests of the design |
| K1.8 | `feat(core): read before First, write after Last` (#9 tick, #10) | FR-030, FR-034 (SRS FR-3.4, FR-4.2) | – | 6, 7 | 7 tick tests of the design |
| K1.9 | `fix(core): estop check before torque_on` (design OC4) | FR-034 (SRS FR-4.2) | – | 7 | `an_estop_in_post_init_never_powers` (new) |
| K1.10 | Root `Cargo.toml`: `[profile.release] panic = "unwind"` stated | FR-043 (SRS NFR-2) | – | 7 | `cargo metadata` shows it; outside `kmr_core/`, an agent may do it if asked |

Then, outside the core: adapters D1 (4 end-to-end tests), `kmr_api` `run()` wiring (CP 9),
`controllers` `robot!(XmBench)` (CP 10), HUMAN bench run with `Torque_Enable` = 0 on every id
(CP 11). **M1 is done when the bench motor moves and stops with torque off.**

### Driver set (D17, 2026-10-29)

Source: [design-hal-set.md](design-hal-set.md). After M1 (its open choice 2, pending human):
XmBench has one member, so M1 is unchanged. Placement A, in the core, assumed, pending human.

| # | Commit | FR | Inv | Done when |
|---|---|---|---|---|
| KS.1 | `feat(core): the hardware as a list of drivers`: `Claim` from `check`, the sealed walk over `HNil`/`HCons`, merge and group renumbering, coverage check, per-member `Batch` view, `run_on` over the list, `Powered` over every member | FR-039, FR-041, FR-043, FR-044, FR-063 (SRS FR-5.1, FR-5.3, FR-5.6, FR-6.4) | – | a gap and an overlap each give the named `run()` error before any I/O; a two-member recorder sees every call on both, exits in reverse order |

### M2 N from the robot line (SRS delivery step 2, core half)

Source: `design-const-generic-n.md` §5. Starts after M1 (D1 A). The bridge default `N = JOINTS`
keeps every commit green. The M1 items generalise as the design §2.4 says (`Slot::mask<const N>`,
`Written<N>`, `HoldTarget<N>`, `Powered<'_, A: Hal<N>>`).

| # | Step | FR | Inv |
|---|---|---|---|
| K2.0 | `docs/decisions/0003-actuator-count-const-generic.md`, supersedes the N half of 0001 | FR-011 (SRS FR-6.1, §8) | – |
| K2.1 | `impl<const N> State<N>` and `Default` | FR-011 | – |
| K2.2 | `Desired<N>`, generic `Slot` methods (`slot`, `field`, `mask`) | FR-011, FR-021 | **`state/field.rs`** |
| K2.3 | Drop `DEPTH` from `RobotState`; rewrite `state/test.rs:451` on `History` | FR-013 (SRS §8 decision 0001) | – |
| K2.4 | `RobotState<const N = JOINTS>` | FR-011 | – |
| K2.5 | `hal.rs` generic: `Mask<N>([u64; MASK_WORDS])`, `Hal<const N = JOINTS>`, all contract types | FR-011, FR-012 (D4) | – |
| K2.6 | One commit: `Env<'a, N>`, `ControlFn<N>`, `ControlFnWith<T, N>`, `RunPhase<N>`, `Drive<N>`, `Robot<const N = JOINTS, CS>` | FR-011, FR-006 | **`controllers.rs`, `schedule.rs`, `schedule/macros.rs`** |
| K2.7 | `impl<const N> Robot<N> { new }` with the `Mask::<N>::CHECK` backstop; tests name N | FR-012 | – |
| K2.8 | (Agent, `kmr_api`) `State<'a, N>`, `Robot<N, C>`, `robot!` emits aliases + N ≤ 64 assert | FR-011, FR-012 | – |
| K2.9 | (Agent, `kmr_adapters`) `Dynamixel<B, NB, N>`, `impl Hal<N>` (adapters E5) | FR-011 | – |
| K2.10 | Remove every `= JOINTS` default and `JOINTS`; SC-009 builds N = 1, 4, 10, 64; 65 fails | FR-011, SC-009 (SRS FR-6.1) | – |

K2.8 and K2.9 are not core rows; they sit here for order only. **M2 is done when SC-009 passes.**

### M3 State transition complete (SRS delivery step 3)

| # | Item | FR | Inv | Note |
|---|---|---|---|---|
| K3.1 | `todo!()` in `home_state` (`history.rs:24`, `robot_state.rs:59`) → `Err(StateError::Undecided("SRS FR-2.7"))` | FR-026 (SRS FR-2.7), FR-055 | – | D5 A; completion plan #1 changed |
| K3.2 | `dt` range [10 µs, 1 s]; unset told apart from zero (`Option<Duration>`) | FR-027 (SRS FR-3.1) | – | #6 |
| K3.3 | `RunError<E> { Config(ConfigError), Hal(E) }`; `run_on` returns it; `run()` keeps `Result<(), RunError<Infallible>>` | FR-027, FR-041 (SRS FR-6.4) | – | #7 minus `ZeroReadCadence` |
| K3.4 | Type-state empty check: `Robot<N, CS, E = Empty>`, sealed `HasController` with `#[diagnostic::on_unimplemented]` | FR-009 (SRS NFR-4, §8) | – (no `schedule.rs`) | #15; proved, research R8 |
| K3.5 | Per-actuator age in ticks; sample age = oldest; `ever_read` mask; "no sample" until first read | FR-018 (SRS FR-2.1), FR-042 (SRS FR-6.3) | – | #4; research R6 |
| K3.6 | `Kp`, `Kd` fields; `From` both ways with `f32` for all five | FR-014, FR-016 (SRS FR-2.2, FR-10.2) | **`state/field.rs`** | #21 |
| K3.7 | `#[repr(transparent)]` on value types; SoA slot aligned to 64 bytes; layout documented | FR-016, FR-059, FR-060 (SRS FR-10.2, NFR-9, NFR-10) | – | #26 |
| K3.8 | Cadence exposed read-only from `Layout` (per actuator); no builder knob; `Overrides::read_every` removed, `Overrides` kept empty and `#[non_exhaustive]` | FR-045, FR-039 (SRS FR-5.3) | – | supersedes #5; research R9 |
| K3.9 | No per-tick `trace!`/`debug!`; startup and exit logs only | FR-054 (SRS NFR-1) | – | #17 |
| K3.10 | Real-time sleep: absolute `clock_nanosleep` behind `RealHost`, optional spin tail, measured | FR-028, FR-031 (SRS FR-3.2, NFR-1) | – | #16 |

### M4 Signals, Time, faults (SRS delivery step 4)

| # | Item | FR | Inv | Note |
|---|---|---|---|---|
| K4.1 | Fault flags per History slot: `Faults<N>`, per actuator (stale, bus error, offline, rejected, joint limit, envelope) + `TickFlags`, per tick (overrun, sensor missing, remote timeout, unauthorized) | FR-051 (SRS FR-13.4, NFR-12) | – | #13; envelope bit per actuator from D3 A, as kmr_api's `Faults` view and kmr_ffi's `kmr_actuator_flags` |
| K4.2 | Hidden external entry: `ExternalFlags`, one `AtomicU32`, `fetch_or` from any thread, `swap(0)` at the inbound stage | FR-052 (SRS FR-13.4) | – | proved, research R10 |
| K4.3 | Hold duration `Option<Duration>`, `None` default; wait in `dt` slices, estop poll per slice | FR-033 (SRS FR-4.1) | – | Host unchanged |
| K4.4 | Overrun flag set in the slot of the late tick | FR-051, FR-029 | – | |
| K4.5 | `run()` logs the ending signal; `Ok` for both | FR-038 (SRS §8 open) | – | |

### M5 API hooks (support for delivery steps 5 and 6)

| # | Item | FR | Inv | Note |
|---|---|---|---|---|
| K5.1 | Stage points: `Stages<N>` trait, `()` no-op impl, `run_on_staged`; inbound after read, outbound after write; `outbound(&mut self, &RobotState<N>, &Time)`, so age and flags are reachable | FR-053 (SRS §4, FR-7.1) | – | contract in `engine-surface.md` |
| K5.2 | Limit and envelope stage before the batch: `set_limits([Limit; N], [Option<Policy>; N])`, `set_envelopes([Envelope; N])`; `Limit { lower: Option<Q>, upper: Option<Q>, speed: Qd, effort: Tau }`, typed, from `robot!` data; `None` = no position bound (continuous joint), never clamped to 0; per-actuator `Policy { Clamp, Warn, Abort }` | FR-037 (SRS FR-4.5, FR-13.5) | – | #20 changed by D2 B, D3 A |
| K5.3 | `require_policies` at finalization: a limit without a policy is a `run()` error | FR-037 (D2 B) | – | one function |

### M6 Gates

| # | Item | FR | Inv | Note |
|---|---|---|---|---|
| K6.1 | `std` default feature; `no_std` without `alloc`: `frunk_core` no default features, `Host` public and sealed, own `Instant`, tracing no-op, `ctrlc` gated | FR-058 (SRS NFR-7) | **`signal.rs`** (`std::sync::atomic`, `std::{fmt, ptr}` at :44-45 become `core::`; imports only, ordering untouched) | #24; target assumed |
| K6.2 | CI job: `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf` | SC-007 | – | CI file is not core; human |
| K6.3 | Opt-in `presort-off` feature in `Batch::new` | FR-044 (SRS FR-5.3) | – | D11 B |
| K6.4 | Review only: builder, phases, signatures vs a dynamic mode | FR-008, SC-013 (SRS FR-1.7) | – | #25 reduced |
| K6.5 | Fix `lib.rs` doc ("not meant to be no_std") and AGENTS.md empty-check text | FR-058, FR-009 | – | AGENTS.md is human-owned |

### M7 FFI hook (supports SRS delivery step 8)

| # | Item | FR | Inv | Note |
|---|---|---|---|---|
| K7.1 | Public `FieldId` enum (`Q`, `Qd`, `Tau`; `Kp`, `Kd` after K3.6), or `const ID: FieldId` on `Slot`, so kmr_api's FFI apply step matches every field exhaustively | FR-062 (SRS FR-10.2); kmr_ffi FR-014 (SRS FR-10.2) | **`state/field.rs`** | HUMAN; for kmr_ffi plan C1, C3; starts with delivery step 8 |

Out of this version (SRS §8 open): threaded spawn (#22), Sensors read (#23), home source
(FR-2.7), multi-actuator joint, running-loop hold (FR-13.3).

## Cross-module needs

| Module | Must honour |
|---|---|
| kmr_adapters | `Hal<N>` after K2.5; `torque_off`/`close` never panic, idempotent, safe before `open` (they run in `Drop`); fill only what was read (FR-022); return the refreshed mask (age, K3.5); flag `rejected` for non-finite or unencodable targets (research R16); drop the `overrides.read_every` branch of `check`: `Overrides` carries nothing after K3.8 (research R9); after KS.1, `check` returns the member's `Claim` and the member fills only its owned slots (design-hal-set.md) |
| kmr_api | Thread `E` (type-state) and N through its wrapper; emit the N ≤ 64 assert in `robot!`; hand the driver list to `run_on` and take it back (one member until KS.1); never re-export `ExternalFlags`; implement `Stages<N>` for its Zenoh slots; pass limits (`[Limit; N]`, `None` bounds where `CONTINUOUS` is set) and envelopes as per-actuator data; offer one call that sets a policy for every actuator (D2 B); forward `RunError::{Config, Hal}`; forward `presort-off` |
| kmr_model | Emit per-actuator limits in actuator order as typed arrays (`LOWER`, `UPPER`: `[Q; N]`; `SPEED`: `[Qd; N]`; `EFFORT`: `[Tau; N]`) and the `CONTINUOUS` set (`[u64; MASK_WORDS]`), which kmr_api turns into `None` bounds; no dependency of the core on `kmr_model` |
| kmr_ffi | Value types are `repr(transparent)` over `f32` (K3.7); flags are plain integers, envelope per actuator (K4.1); age `Option<u32>` per actuator, latest slot only; `FieldId` (K7.1); non-finite writes follow the adapter's `rejected` rule; foreign stop goes through a Rust forwarder (D13 A) |
| bench | `presort-off` feature (K6.3); the 5-check tick for overhead; the NoHardware run prints one warning |
| controllers | Nothing directly |

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| Bridge default `N = JOINTS` for 10 commits | Each commit stays green; invariant files change in one reviewed commit | One large commit hides the invariant change in a 1 000-line diff |
| Third type parameter `E` on `Robot` | SRS NFR-4 wants a compile error for an empty robot | A bound over six HList buckets needs a type-level OR; touches `schedule.rs` (invariant) |
| Startup-write warning collects phases | SC-011 wants exactly one warning | The prototype warns per phase (two warnings for two phases) |
