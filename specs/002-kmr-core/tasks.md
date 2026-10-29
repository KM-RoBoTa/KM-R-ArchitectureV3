---
description: "Tasks for kmr_core, the engine (all HUMAN), generated 2026-10-28"
---

# Tasks: kmr_core, the engine

**Input**: `specs/002-kmr-core/`: [plan.md](plan.md) (2026-10-28), [spec.md](spec.md) (coordinated
2026-10-28), [research.md](research.md), [data-model.md](data-model.md),
[quickstart.md](quickstart.md), [contracts/](contracts/), and the two human designs
[design-first-motor.md](design-first-motor.md) and [design-const-generic-n.md](design-const-generic-n.md).
Order: `specs/spec-coordination-2026-10-28.md` §3 (critical path, "CP"). Authority:
`docs/srs/SRS.md`.

**HUMAN**: every core task. `kmr_core/` is in `.aiignore`; an agent never edits it. A task that
touches an invariant file names it in **bold**: `state/field.rs`, `sealed.rs`, `signal.rs`,
`schedule.rs`, `schedule/macros.rs`, `controllers.rs`, and anything `unsafe`.

**Hand-off**: a task marked "Hand-off" belongs to another module. It keeps an id here for order
only. No core edit, and its owner does it.

**Tests**: included. Each commit carries the test named in design-first-motor §5 or in
quickstart.md. Write the test first, see it fail, then the code. Every commit leaves the
workspace green.

**Assumed, pending human**: D1 (A: M1 is SRS delivery step 1), D2 (B: `run()` refuses a limited
actuator without a policy), D3 (A: envelope breach clamps and flags), D4 (A: N ≤ 64,
`MASK_WORDS = 1`), D5 (A: typed error, no marker after `open`), D11 (B: `presort-off` feature),
D13 (A: no core change). Also the first-motor open choices OC1 to OC7 (research R4), the FR-042
"no sample" recommendation (research R6) and the `no_std` target (research R13). Each sits in the
one place its task names. Nothing here decides them.

**Ids**: `FR-0NN` / `SC-0NN` are spec.md's, written with their SRS source. `Kn.m` are plan.md's
work-order rows. User stories are spec.md's: US1 tick on time (P1), US2 stop safely (P1), US3 state
(P1), US4 N from the robot line (P1), US5 hardware adapter (P2), US6 faults (P2), US7 `no_std`
(P3), US8 dynamic-scheduler door (P3).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an unfinished task)
- **[Story]**: US1..US8 from spec.md

Paths are from the repository root.

---

## Phase 1: Setup

**Purpose**: a green baseline before the first core commit.

- [x] T001 HUMAN: Run quickstart.md "Always" from the repo root (`cargo fmt --all --check`, `cargo clippy --workspace --all-targets`, `cargo test --workspace --no-fail-fast`). Expect 191 passed, 0 failed, 7 ignored (design-first-motor §4 baseline; other modules may have moved the count). Note kmr_core's clippy warnings (6 today) so later commits add none

---

## Phase 2: Foundational: the design notes match the plan

**Purpose**: the human builds M1 and M2 from the two design notes. Coordination §5 lists where
they lag the coordinated plan. Fix the notes first, so no commit follows stale text.

- [x] T002 [P] HUMAN: Update specs/002-kmr-core/design-first-motor.md: (1) `push_slot` copies zeros into empty slots, while FR-042 (SRS FR-6.3) says History reads return "no sample" on a no-hardware run: state that the push stays and the read side gets an `ever_read` gate and `Option<u32>` age in K3.5 (research R6); (2) drop "`Overrides::default()` until #5 adds the cadence knob": `Overrides` stays empty (K3.8, research R9); cadence comes from the hardware table; (3) the per-tick `Faults` local becomes per-slot `Faults<N>`, envelope bit per actuator (K4.1); (4) §5 "`run_on(kmr_adapters::Hardware::default())`" becomes the `__Spec` that `robot!` emits and `Dynamixel::bind` (kmr_api A4, A5); (5) state the K1.6 text "no bus plugin: simulating", printed once at `open`. Found by analysis, same note: (6) `elapsed()` counts from `arm`, not `startup` (OC6, data-model `Time`); (7) one startup-write warning per run listing every phase that wrote, not one per phase (OC7, SC-011 (SRS FR-5.2, FR-2.5))
- [x] T003 [P] HUMAN: Gates M1 with T002 (Phase 2 checkpoint), though it describes M2. Update specs/002-kmr-core/design-const-generic-n.md §2.2 and §5 step 8: (1) the alias reads `Robot<N, __Spec>` (later with `E`) and comes from kmr_api's front door, not the proc macro; (2) controllers moves to `robot!` earlier, in controllers plan C0; (3) kind bit sets are `[u64; MASK_WORDS]` too (kmr_adapters T058, kmr_api B3)

**Checkpoint**: both notes match plan.md. M1 can start.

---

## Phase 3: M1 First motor (US5, US2, US1, US3), CP 3 to 11, SRS delivery step 1 🎯 MVP

**Goal**: the engine reads before `First`, writes after `Last`, and every exit after `open` ends
with `torque_off` then `close`. N stays `JOINTS = 4`. M1 is done when the bench motor moves and
stops with torque off. Assumed, pending human: D1 (option A).

**Independent Test**: quickstart.md § M1: `cargo test -p kmr_core exit_paths`, `state::test`,
`--test termination`, `clock`; then the bench.

### Core commits (design-first-motor §5 order, then K1.9, K1.10)

- [x] T004 [US1] HUMAN: K1.1 `feat(core): read-only Time getters` (#14, CP 8) in kmr_core/src/clock.rs and kmr_core/src/runtime.rs: fields `tick`, `overrun_count`; one `pub` block `elapsed()`, `dt()`, `tick()`, `overrun_count()`, `last_overrun()`, `accumulated_overrun()`, values by copy; the `pub(crate)` `elapsed()` and `accumulated_overrun()` go. `last_overrun()` = the previous tick's lateness, `None` if on time (OC5). `elapsed()` counts from `arm`, not from `startup` as the design's code comment says (OC6). Test `tick_and_overrun_count_follow_end_tick`. FR-029 (SRS FR-3.3). Same as kmr_adapters T035. Unblocks kmr_api A1 and bench B-1
- [x] T005 [US2] HUMAN: K1.2 `feat(core): stop on SIGTERM and SIGHUP` (#11 half, CP 7): `ctrlc = { workspace = true, features = ["termination"] }` in kmr_core/Cargo.toml; the handler reason in kmr_core/src/runtime.rs names all three signals; new own-process test kmr_core/tests/termination.rs (`kill -TERM` then `kill -HUP` from a controller). FR-035 (SRS FR-4.3). Same as kmr_adapters T036
- [x] T006 [US3] HUMAN: K1.3 `feat(core): per-field write mask` (#2, CP 3): `written: Written` in kmr_core/src/state/desired.rs; **kmr_core/src/state/field.rs (invariant: `Slot` gains `fn mask(written: &mut Written) -> &mut Mask`, one line per impl)**; kmr_core/src/state/robot_state.rs: `set_at` sets the bit after the store (an out-of-range index returns before it), `set_all` sets `Mask::ALL`. Tests `a_write_sets_the_bit_of_its_field_only`, `an_out_of_range_write_sets_no_bit`, `set_all_marks_every_actuator_of_its_field`. FR-021 (SRS Definitions "Write mask"). Same as kmr_adapters T037 (part)
- [x] T007 [US3] HUMAN: K1.4 `feat(core): command and clear after apply` (#2, CP 3): engine block in kmr_core/src/state/robot_state.rs, `pub(crate)` `written()`, `command()`, `clear_desired()` (values and masks). Test `the_command_borrows_desired_and_clear_resets_values_and_masks`. FR-019 (SRS FR-2.5). Same as kmr_adapters T037 (part)
- [x] T008 [US3] HUMAN: K1.5 `feat(core): one History slot per push, hold target` (#3, CP 4): `State<N>` derives `Clone, Copy` in kmr_core/src/state/history.rs; `push_slot()` (a copy of `prev(0)`, lent as a `Sample`) and `hold_target()` (latest sample overlaid with this tick's writes, `written` = this tick's masks; OC1) in kmr_core/src/state/robot_state.rs. Interim: an empty History still copies zeros until K3.5 (T036) gates reads. Tests `each_pushed_slot_starts_as_a_copy_of_the_last`, `the_hold_target_is_the_sample_overlaid_with_this_ticks_writes`. FR-017, FR-022 (SRS FR-2.3, FR-2.6), FR-032 (SRS FR-4.1). Same as kmr_adapters T038
- [x] T009 [US5] HUMAN: K1.6 `feat(core): run_on over any adapter` (#8, CP 5): `run_on<A: Hal>(self, hal: A) -> Result<(), A::Error>` and `run()` = `run_on(NoHardware)` in kmr_core/src/robot.rs; `runtime()` and `run_with()` take the adapter, finalization calls `check(Overrides::default())` only, in kmr_core/src/runtime.rs; `impl<A: Hal> Hal for &mut A` and its seal impl in kmr_core/src/hal.rs (OC3, FR-050 option A); `NoHardware` prints "no bus plugin: simulating" once, at `open`, and the old "no hardware adapter compiled in" text goes. Both signals return `Ok` (FR-038). The `Recorder` fake in the runtime tests. Tests `a_check_error_is_returned_before_any_io`; a `NoHardware` run logs that text exactly once. FR-041, FR-050 (SRS FR-5.1, FR-5.9), FR-042 (SRS FR-6.3). Same as kmr_adapters T039. Unblocks kmr_api A0, C5
- [x] T010 [US2] HUMAN: K1.7 `feat(core): bring-up calls behind a torque-off guard` (#9 startup, #10, #11, CP 6, 7) in kmr_core/src/runtime.rs: `Powered` drop guard built just before `open` (`Drop` = `torque_off` then `close`); Init: `open`, `zero` before user `Init`; PostInit: `ping`, `health`, `configure`, `push_slot` + `read_all`, `torque_on` before user `PostInit`; the two TODO blocks go; clock armed after bring-up (FR-031 (SRS FR-5.10)); `discard_startup_writes` gives one warning per run naming every phase that wrote, not one per phase as the prototype did (OC7, SC-011). Tests: the 8 of design §5 commit 7. FR-039, FR-043, FR-048, FR-020 (SRS FR-5.2, FR-5.6, NFR-2). `sealed.rs`, `signal.rs`, `schedule.rs`, `controllers.rs` untouched. Same as kmr_adapters T040 (part)
- [x] T011 [US1] HUMAN: K1.8 `feat(core): read before First, write after Last` (#9 tick, #10, CP 6, 7) in kmr_core/src/runtime.rs: `push_slot` + `read` after `begin_tick`; a new estop check after `Last` (five bus checks per tick); `Batch` from the masks, `write` skipped on an empty batch (OC2); `clear_desired()` before the sleep; shutdown: `hold(hold_target)` then the guard; test constants `AFTER_LAST = 5`, `TICK_END = 6`, `PER_TICK = 5`. Tests: the 7 of design §5 commit 8. FR-030, FR-034 (SRS FR-3.4, FR-4.2), FR-019 (SRS FR-2.5), FR-032 (SRS FR-4.1). Same as kmr_adapters T040 (part). With T009 and T010, unblocks kmr_adapters T041 and kmr_api A5
- [x] T012 [US2] HUMAN: K1.9 `fix(core): estop check before torque_on` (OC4, CP 7) in kmr_core/src/runtime.rs: one `estop(host.poll())` after `read_all`; a pending estop never powers. New test `an_estop_in_post_init_never_powers`. FR-034 (SRS FR-4.2)
- [x] T013 [P] [US2] HUMAN: K1.10 root Cargo.toml: `[profile.release] panic = "unwind"`, stated, so the `Powered` guard runs on a panic (CP 7). Outside `kmr_core/`: an agent may do it if asked (plan K1.10). Check: the line is in Cargo.toml (`cargo metadata` does not print profiles). FR-043 (SRS NFR-2)
- [x] T014 [US1] HUMAN: Run quickstart.md § M1 (`cargo test -p kmr_core exit_paths`, `state::test`, `--test termination`, `clock`) and "Always": the 24 core tests of design §4 and T012's pass, 0 failed, no new clippy warning in kmr_core; exit-path records match contracts/hal.md "Call order"

### Outside the core (hand-offs, CP 9 to 11)

- [ ] T015 [US5] Hand-off, owner kmr_adapters T041 (plan D1); blocked on T009, T010, T011: four `engine` tests through `run_on(&mut adapter)` on a pty in kmr_adapters/src/dynamixel/tests.rs. No core edit
- [ ] T016 [US5] Hand-off, owner kmr_api A0, A1, A5 (CP 9): A0 (drop kmr_api's own simulation `eprintln!`) waits on T009, never before it; A1 (forward the six Time getters) waits on T004; A5 (`run()` hands the adapter to `run_on`) waits on T009 to T011 and on kmr_adapters T024, T030 through kmr_api A3, A4. Files kmr_api/src/robot.rs (A0, A5), kmr_api/src/time.rs (A1). No core edit
- [ ] T017 [US1] Hand-off, owner controllers C0 and C2 (CP 10): C0 (`kmr_api::robot!(XmBench)` in controllers/src/main.rs) is unblocked and goes before kmr_api A4; C2 (bounded swing from `prev(0)`, `shutdown!` after a fixed time) waits on kmr_api A1, A2. No bus line in the shipped file (assumed, pending human: D15 A). No core edit
- [ ] T018 [US2] HUMAN: Bench, CP 11 (quickstart.md § M1 "Bench"); one session with kmr_adapters T045, kmr_api T017 (A9) and controllers T014, T015 (C6). Blocked on T004–T013, kmr_adapters T017 (which waits on dxl_rs, kmr_adapters T002), T041, kmr_api T011 (A5) and controllers T012 (C2). XmBench, `cargo run -p controllers --features hardware` with a local `Bus::by_id` line: the swing runs, `shutdown!` ends it, `Torque_Enable` = 0 on ids 1 to 4. Repeat with an emergency stop, raised by the local `emergency_stop!` line of controllers T015, then Ctrl-C, `kill -TERM`, `kill -HUP`. SC-003 (SRS FR-4.1, FR-4.2, FR-5.6, NFR-2)

**Checkpoint**: delivery step 1 (D1 A): a motor moves and every exit cuts torque. Nothing below
starts before this (Constitution I, SRS §8).

---

## Phase 4: M2 N from the robot line (US4), SRS delivery step 2, core half

**Goal**: N is a const generic; no `JOINTS` in the core. The bridge default `N = JOINTS` keeps
every step green until T030 removes it. M1 items generalise as design-const-generic-n §2.4 says
(`Slot::mask<const N>`, `Written<N>`, `HoldTarget<N>`, `Powered<'_, A: Hal<N>>`).

**Independent Test**: quickstart.md § M2: SC-009 builds N = 1, 4, 10, 64; 65 fails.

**Gate**: after T018 (D1 A). Same work as kmr_adapters T056 (steps 0 to 7).

- [ ] T019 [P] [US4] HUMAN: Before T021, record a clean `cargo build --timings -p controllers` (design-const-generic-n §4: hal.rs and the adapter move their codegen into the user crate); repeat after T030 and keep both reports (target/cargo-timings/) with the PR. FR-061 (SRS NFR-11). Added by analysis: the design asks it, no plan row
- [ ] T020 [P] [US4] HUMAN: K2.0 docs/decisions/NNNN-actuator-count-const-generic.md, NNNN the next free number in docs/decisions/ at commit time (0001 and 0002 exist): supersedes the N half of 0001; `HISTORY_DEPTH` stays a constant. No code. FR-011 (SRS FR-6.1, §8 decision 0001)
- [ ] T021 [US4] HUMAN: K2.1 `impl<const N: usize> State<N>` and its `Default` in kmr_core/src/state/history.rs; callers unchanged. FR-011 (SRS FR-6.1)
- [ ] T022 [US4] HUMAN: K2.2 `Desired<const N>` in kmr_core/src/state/desired.rs; **kmr_core/src/state/field.rs (invariant)**: `Slot` methods `slot`, `field`, `mask` generic over N, the trait itself not, `StateField` unchanged (design §2.4); `RobotState` holds `Desired<JOINTS>`. FR-011, FR-021 (SRS FR-6.1, Definitions "Write mask")
- [ ] T023 [US4] HUMAN: K2.3 drop `DEPTH` from `RobotState` in kmr_core/src/state/robot_state.rs; rewrite the `RobotState::<DEPTH>` test at kmr_core/src/state/test.rs:451 on `History`. Must land before T024: once N goes first, `RobotState::<X>` means N = X. FR-013 (SRS §8 decision 0001)
- [ ] T024 [US4] HUMAN: K2.4 `RobotState<const N: usize = JOINTS>` in kmr_core/src/state/robot_state.rs; every `JOINTS` becomes N; `OutOfRange { len: N }`. FR-011 (SRS FR-6.1)
- [ ] T025 [US4] HUMAN: K2.5 kmr_core/src/hal.rs generic: `Mask<const N>([u64; MASK_WORDS])` with `pub const MASK_WORDS: usize = 1` (assumed, pending human: D4 A; the one place), `trait Hal<const N: usize = JOINTS>`, `Written<N>`, `Faults<N>`, `Layout<N>`, `Command<'a, N>`, `Sample<'a, N>`, `Batch<'a, N>`, `Runs<'a, N>`, `HoldTarget<N>`, `impl<const N> Hal<N> for NoHardware`, the `&mut A` impl. Adapters compile unchanged. `MASK_WORDS` is what kmr_adapters T058, kmr_api T066 (hidden re-export, then B3) and kmr_model T032 name; this task unblocks all three. FR-011, FR-012 (SRS FR-6.1, Definitions "Write mask")
- [ ] T026 [US4] HUMAN: K2.6, one commit: **kmr_core/src/controllers.rs (invariant)** `Env<'a, N>`, `ControlFn<N>`, `ControlFnWith<T, N>`; **kmr_core/src/schedule.rs and kmr_core/src/schedule/macros.rs (invariant)** `RunPhase<N>`, `Drive<N>` (`Insert`, `HAppend`, `Schedule` unchanged); kmr_core/src/robot.rs `Robot<const N: usize = JOINTS, CS = Scheduled>`, `impl Robot<JOINTS, Scheduled> { new }` kept concrete; kmr_core/src/runtime.rs `<const N, CS: Drive<N>>`, `run_on<A: Hal<N>>`, `Powered<'_, A: Hal<N>>` (research R3). One commit, since every `Robot<CS>` breaks with E0747. kmr_api names `ControlFn<JOINTS>` in the same commit (design §5 step 6). FR-011, FR-006 (SRS FR-6.1, FR-1.6)
- [ ] T027 [US4] HUMAN: K2.7 `impl<const N: usize> Robot<N> { new }` in kmr_core/src/robot.rs, naming the backstop `Mask::<N>::CHECK` ("SRS write mask: more than 64 actuators"; fires under `cargo build` only); core tests, doctests, kmr_core/tests/run_signals.rs, kmr_core/tests/foreign_ctrlc.rs, kmr_core/tests/termination.rs name N (`Robot::<4>::new()`). FR-012 (SRS Definitions "Write mask"). Unblocks kmr_api B7 and kmr_adapters T057
- [ ] T028 [US4] Hand-off, owner kmr_api B7 and B2 (K2.8); blocked on T027: `State<'a, N>`, `GroupView<'a, N, K>`, `Robot<N, ..>`; `robot!` emits `N`, `State<'a>`, `Robot<N, __Spec>` from kmr_api's front door, and the `const _` assert "SRS write mask: more than 64 actuators" that fires under `cargo check` (FR-012); the temporary `N == JOINTS` assert goes (kmr_model plan); the `three.urdf` test passes. Files kmr_api/src/state.rs, kmr_api/src/robot.rs, kmr_api/src/robot_macro.rs. No core edit
- [ ] T029 [US4] Hand-off, owner kmr_adapters T057 (plan E5, K2.9); blocked on T027: `Dynamixel<B, NB, N>`, `impl Hal<N>` in kmr_adapters/src/dynamixel/mod.rs. No core edit
- [ ] T030 [US4] HUMAN: K2.10, after T028 and T029: remove every `= JOINTS` default and `JOINTS` itself from kmr_core/src/state.rs, kmr_core/src/lib.rs and the files of T021 to T027; `grep -rn "JOINTS" kmr_core/src` returns nothing; SC-009 builds N = 1, 4, 10, 64 on `NoHardware` (`cargo test -p kmr_api robot_macro`; the robots and fixtures are kmr_model T062's); a 65-actuator `robot!` fails under `cargo check` with "SRS write mask: more than 64 actuators"; `Robot::<65>::new()` fails under `cargo build`. FR-011, SC-009 (SRS FR-6.1)

**Checkpoint**: SC-009 passes; delivery step 2, core half.

---

## Phase 5: M3 State transition complete (US1, US3, US5), SRS delivery step 3

**Goal**: `dt` range, `RunError`, type-state empty check, sample age, `Kp`/`Kd`, layout, no
hot-path logging, real-time sleep.

**Independent Test**: quickstart.md § M3.

**Gate**: after T030.

- [ ] T031 [US3] HUMAN: K3.1 replace the two `todo!()` home calls (kmr_core/src/state/history.rs:24, kmr_core/src/state/robot_state.rs:59) with `Err(StateError::Undecided("SRS FR-2.7: home state source"))`; add `Undecided(&'static str)` to `StateError` in kmr_core/src/error.rs; fix the comment at kmr_core/src/state/test.rs:10. Assumed, pending human: D5 A (the one place). With D5 B, keep spec FR-026's `unimplemented!("SRS FR-2.7: home state source")` instead. Check: `grep -rn "todo!\|unimplemented!" kmr_core/src` is empty. FR-026 (SRS FR-2.7), FR-055 (SRS NFR-2)
- [ ] T032 [US5] HUMAN: In kmr_core/src/runtime.rs PostInit bring-up, one warning "out-of-place check skipped: SRS FR-2.7 home state open", never a marker. Test: one line per run. FR-048 (SRS FR-5.2). Added by analysis: research R7 states it, no plan row
- [ ] T033 [US1] HUMAN: K3.2 `dt` as `Option<Duration>` in kmr_core/src/config/user_config.rs and kmr_core/src/robot.rs: unset → 1 ms with one warning; `Some` outside [10 µs, 1 s], zero included, is an error at finalization (kmr_core/src/clock.rs keeps `set_delta`). FR-027 (SRS FR-3.1)
- [ ] T034 [US1] HUMAN: K3.3 `RunError<E> { Config(ConfigError), Hal(E) }` and `ConfigError { DtOutOfRange { dt } }` in kmr_core/src/error.rs (`NoPolicy` lands in T051); `run_on` returns `RunError<A::Error>`, `run()` returns `Result<(), RunError<Infallible>>` in kmr_core/src/robot.rs; `ApiError` goes; no `ZeroReadCadence`. Test SC-004: `dt` of 0, 5 µs, 2 s → `RunError::Config`, the recorder logs 0 I/O calls. FR-027, FR-041 (SRS FR-3.1, FR-6.4). Unblocks kmr_api T040 (C6 first part; drops its interim `dt` check, A6) and bench B-2. Breaks kmr_api's build (`ApiError`, `run_on`'s return) until kmr_api T040: land them together
- [ ] T035 [US1] HUMAN: K3.4 type-state empty check in kmr_core/src/robot.rs: `Robot<const N, CS = Scheduled, E = Empty>`; every add returns `E = Filled`; `run`, `run_on` require `E: HasController`, a trait sealed by a private module in robot.rs, with `#[diagnostic::on_unimplemented]` "kmr: this robot has no controller". **schedule.rs and sealed.rs stay untouched** (research R8; plan Project Structure). Compile-fail doctest expecting that message. FR-009 (SRS NFR-4, §8 "Type-state over runtime checks"). Unblocks kmr_api T040 (C6 first part); breaks kmr_api's build until it lands: land them together
- [ ] T036 [US3] HUMAN: K3.5 per-actuator age in kmr_core/src/state/history.rs and kmr_core/src/state/robot_state.rs: `ever_read: Mask<N>`; `read`'s returned mask resets each refreshed actuator's age (consumed in kmr_core/src/runtime.rs); `age(index) -> Result<Option<u32>, StateError>`, `None` until first read; `sample_age() -> Option<u32>` = oldest; a History read of an actuator never read returns `StateError::NoSample { depth }`, never zeros. Storage (`[Option<u32>; N]` or `[u32; N]` + `ever_read`) is the human's pick (data-model). Assumed, pending human: FR-042 recommendation (research R6). Tests: cadence 3 reads age 0, 1, 2, 0; a `NoHardware` run returns `NoSample` on every History read. FR-018 (SRS FR-2.1), FR-042 (SRS FR-6.3), FR-025 (SRS NFR-2). Unblocks kmr_api C1, kmr_ffi C1 (part), bench C-2
- [ ] T037 [US3] HUMAN: K3.6 `Kp`, `Kd` value types in kmr_core/src/state/joint_state.rs; **kmr_core/src/state/field.rs (invariant)**: one `Slot` impl and one `StateField` impl each; arrays and masks in kmr_core/src/state/desired.rs and kmr_core/src/state/history.rs; `Command` and `Sample` carry them in kmr_core/src/hal.rs; `From` both ways with `f32` for all five types. Units: N·m/rad, N·m·s/rad revolute; N/mm, N·s/mm prismatic (spec assumption). FR-014, FR-016 (SRS FR-2.2, FR-10.2). kmr_adapters must flag `rejected` on a `Kp`/`Kd` write it cannot send (contracts/hal.md "What the engine expects" 5)
- [ ] T038 [US3] HUMAN: K3.7 `#[repr(transparent)]` on `Q`, `Qd`, `Tau`, `Kp`, `Kd` in kmr_core/src/state/joint_state.rs; the SoA slot aligned to 64 bytes in kmr_core/src/state/history.rs; each layout choice documented there. Test: `size_of::<Q>() == 4`, `align_of` of a 4-actuator slot is 64. FR-016, FR-059, FR-060 (SRS FR-10.2, NFR-9, NFR-10). The layout benchmark is the bench module's
- [ ] T039 [US5] HUMAN: K3.8 after kmr_adapters T020 (it drops the adapter's `overrides.read_every` branch; landing first breaks the adapter build): remove `Overrides::read_every`, keep `Overrides` empty and `#[non_exhaustive]` in kmr_core/src/hal.rs (and `NoHardware::check` cadence 1); expose the cadence read-only, `cadence(index) -> Result<NonZeroU32, StateError>` from `Layout`, in kmr_core/src/state/robot_state.rs; no builder knob. FR-045, FR-039 (SRS FR-5.3). Unblocks kmr_api C3, bench C-2
- [ ] T040 [US1] HUMAN: K3.9 remove every per-tick `trace!`/`debug!` from kmr_core/src/runtime.rs (the `First`, `EachTick`, `Last` walks today); startup and exit logs only. FR-054 (SRS NFR-1). The bench checks it at every `RUST_LOG` level
- [ ] T041 [US1] HUMAN: K3.10 real-time sleep in kmr_core/src/runtime.rs (`RealHost`): absolute `clock_nanosleep` to the deadline, optional spin tail, measured on the NFR-1 bench (bench A-10). FR-028, FR-031 (SRS FR-3.2, NFR-1)
- [ ] T042 [US1] HUMAN: SC-005 test in a new own-process kmr_core/tests/no_alloc.rs: a counting `#[global_allocator]` (**`unsafe impl GlobalAlloc`: invariant per AGENTS.md**) over 10 000 scripted ticks with the recording adapter: 0 allocations and 0 log lines per tick; plus a review that the tick path names no `Mutex`, `RwLock` or `Once`. FR-054 (SRS NFR-1). Added by analysis: quickstart § M3 lists it, no plan row
- [ ] T043 [US1] HUMAN: SC-001 and SC-002 tests in kmr_core/src/runtime.rs tests: 10 000 scripted ticks record read, `First`, `EachTick`, `Last`, write, sleep on every tick, controllers in declaration order; with injected overruns every tick starts on `origin + k*dt` and no two start in one slot. FR-030, FR-028 (SRS FR-3.4, FR-3.2). Added by analysis: the design tests run a few ticks only
- [ ] T065 [US1] HUMAN: `test-util` feature, off by default, in kmr_core/Cargo.toml: the scripted `Host` (instants, sleeps and bus scripted; no real sleep, no global bus) and the recording adapter of the runtime tests (kmr_core/src/runtime.rs), `#[doc(hidden)] pub` under the feature only, so kmr_api and kmr_ffi tests script ticks, reads, faults and overruns from outside the core. The feature name is kmr_api plan § C's and kmr_ffi plan C4's; no core plan row (found by cross-module analysis). `signal.rs` untouched; a user still cannot implement `Host` (T052 makes it public and sealed later, for `no_std`). Test: a kmr_core/tests/ file built with `--features test-util` drives 3 scripted ticks with one injected overrun through the public items only. FR-056 (SRS NFR-3): nothing new is public without the feature. Unblocks kmr_api T037, kmr_ffi T016, T024–T026

**Checkpoint**: quickstart.md § M3 passes; delivery step 3.

---

## Phase 6: M4 Signals, Time, faults (US6, US2), SRS delivery step 4

**Goal**: every fault is a typed flag in its tick's slot; hold duration; the ending signal is
logged.

**Independent Test**: quickstart.md § M4.

**Gate**: after Phase 5.

- [ ] T044 [US6] HUMAN: K4.1 per-slot flags in kmr_core/src/hal.rs and kmr_core/src/state/history.rs: `Faults<N>` per actuator (`stale`, `bus_error`, `offline`, `rejected`, `joint_limit`, `envelope`; `envelope` assumed, pending human: D3 A), each a `Mask<N>`; `TickFlags` per tick, a `u32` (`OVERRUN`, `SENSOR_MISSING`, `REMOTE_TIMEOUT`, `UNAUTHORIZED`); `read` and `write` fill the current slot's `Faults<N>`, no longer a dropped local (kmr_core/src/runtime.rs); readers `faults(depth)`, `tick_flags(depth)` in kmr_core/src/state/robot_state.rs; no setter for controllers. Test SC-006: each injected fault shows in its tick's slot at depth 0, then 1. FR-051 (SRS FR-13.4, NFR-12). Unblocks kmr_api C2, kmr_ffi C1 (part)
- [ ] T045 [US6] HUMAN: K4.2 `ExternalFlags(AtomicU32)`, `#[doc(hidden)] pub`, in kmr_core/src/hal.rs: `raise(bits)` = `fetch_or(Release)` from any thread; `take()` = `swap(0, Acquire)`, called by the loop after `read` and before `First` (where T049's inbound stage will sit), folded into the slot's `TickFlags`. **signal.rs untouched.** Test: a raise from another thread during tick k shows in slot k or k+1, never lost. FR-052 (SRS FR-13.4). Unblocks kmr_api F2
- [ ] T046 [US2] HUMAN: K4.3 hold duration `Option<Duration>`, `None` default, `set_hold` in kmr_core/src/config/user_config.rs and kmr_core/src/robot.rs; `Some(d)`: the runtime sleeps to `min(hold_end, now + dt)` and polls the bus after each slice, an estop ends the wait (kmr_core/src/runtime.rs, kmr_core/src/clock.rs); `Host` unchanged (research R14). Test SC-010: `torque_off` follows `hold` after `d` within one `dt`; an estop at 1 s of 2 s ends the wait. FR-033 (SRS FR-4.1). Unblocks kmr_api C4
- [ ] T047 [US6] HUMAN: K4.4 set `TickFlags::OVERRUN` in the slot of the late tick (kmr_core/src/runtime.rs, kmr_core/src/clock.rs). Test: an overrun on tick 7 reads set at depth 1 on tick 8 (US6 scenario 1). FR-051, FR-029 (SRS FR-13.4, FR-3.3)
- [ ] T048 [US2] HUMAN: K4.5 `run()` logs the signal that ended it, once, at exit, in kmr_core/src/runtime.rs; `Ok(())` for both signals. FR-038 (SRS §8 open "run() after an emergency stop")

**Checkpoint**: quickstart.md § M4 passes; delivery step 4.

---

## Phase 7: M5 API hooks (US6, US2), supports SRS delivery steps 5 and 6

**Goal**: kmr_api gets stage points and a limit and envelope stage before the write.

**Independent Test**: quickstart.md § M5.

**Gate**: after Phase 6.

- [ ] T049 [US6] HUMAN: K5.1 `Stages<const N>`, `#[doc(hidden)] pub`, with `impl Stages<N> for ()`, in kmr_core/src/robot.rs: `inbound(&mut self, &mut RobotState<N>, &Time)` after `read` and the T045 fold, before `First`; `outbound(&mut self, &RobotState<N>, &Time)` after the write, so depth 0, its flags and the age are reachable; `run_on_staged(hal, stages)`; `run_on` uses `()` (kmr_core/src/runtime.rs). Static dispatch, no heap. Test: a `Stages` recorder sees `inbound` after `read` and before `First`, `outbound` after `write`. FR-053 (SRS §4 Constraints, FR-7.1). Unblocks kmr_api Z3
- [ ] T050 [US2] HUMAN: K5.2 limit and envelope stage before the batch, in a new kmr_core/src/limits.rs, called from kmr_core/src/runtime.rs: `set_limits([Limit; N], [Option<Policy>; N])` and `set_envelopes([Envelope; N])` on the builder (kmr_core/src/robot.rs, kmr_core/src/config/user_config.rs); `Limit { lower: Option<Q>, upper: Option<Q>, speed: Qd, effort: Tau }`, typed; `None` = no position bound (continuous joint), never clamped to 0; `Policy { Clamp, Warn, Abort }` per actuator, `Abort` raises `emergency_stop!` with reason `JointLimit` and skips that tick's write; a limit breach sets `joint_limit`; an envelope breach clamps and sets `envelope` (assumed, pending human: D3 A; the `Breach::Envelope` arm); a non-finite value is a breach (research R16). Tests: the M5 cases of quickstart.md. FR-037 (SRS FR-4.5, FR-13.5). Unblocks kmr_api B3 (limits consumer), C4, E2
- [ ] T051 [US2] HUMAN: K5.3 `limits::require_policies` in kmr_core/src/limits.rs, called at finalization: a limited actuator without a policy is `RunError::Config(ConfigError::NoPolicy { actuator })` before any I/O; add `NoPolicy` to kmr_core/src/error.rs. Assumed, pending human: D2 B (the one place). Test: `run_on` returns `NoPolicy`, 0 I/O calls. FR-037 (SRS FR-4.5, FR-6.4). Unblocks kmr_api C4 (T038), the `NoPolicy` part of C6 (kmr_api T068, which lands with this task) and controllers C9

**Checkpoint**: quickstart.md § M5 passes.

---

## Phase 8: M6 Gates (US7, US5, US8)

**Goal**: an opt-in `no_std` build in CI, the pre-sort switch, the dynamic-scheduler review.

**Independent Test**: quickstart.md § M6.

**Gate**: after Phase 7.

- [ ] T052 [US7] HUMAN: K6.1 `std` default feature and a `no_std` build without `alloc`: kmr_core/Cargo.toml (`frunk_core` without default features; `ctrlc`, `tracing-subscriber` behind `std`); `#![no_std]` gate in kmr_core/src/lib.rs; `Host` public and sealed like `hal::seal::Adapter` (not in sealed.rs), an own monotonic `Instant`, tracing a no-op, no threaded registration without `std` (FR-002, found by analysis) in kmr_core/src/runtime.rs, kmr_core/src/clock.rs, kmr_core/src/robot.rs; **kmr_core/src/signal.rs (invariant): `std::sync::atomic`, `std::{fmt, ptr}` at :44-45 become `core::`, imports only, ordering untouched**. Target assumed `thumbv7em-none-eabihf` (SRS §8 reference MCU open, research R13). A `no_std` user path stays out of this version (spec issue 9, pending human). FR-058 (SRS NFR-7)
- [ ] T053 [US7] HUMAN: K6.2 CI job in .github/workflows/ci.yml: `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf`, after T052. CI is not core; the plan keeps it human. SC-007 (SRS NFR-7)
- [ ] T054 [US5] HUMAN: K6.3 opt-in `presort-off` feature in kmr_core/Cargo.toml; `Batch::new` keeps declaration order under it (kmr_core/src/hal.rs). Assumed, pending human: D11 B. Test: `cargo build -p kmr_core --features presort-off`. FR-044 (SRS FR-5.3). Unblocks kmr_api C7, bench D-1
- [ ] T055 [P] [US8] HUMAN: K6.4 review only: builder methods (kmr_core/src/robot.rs), phase rules, controller signatures (kmr_core/src/controllers.rs, read only) against a later heap-gated dynamic mode; record the finding in the PR, or a decision record if something must change. FR-008, SC-013 (SRS FR-1.7)
- [ ] T056 [US7] HUMAN: K6.5 fix the kmr_core/src/lib.rs crate doc ("not meant to be no_std", SRS NFR-7); update AGENTS.md (human-owned): the runtime `L::LEN == 0` paragraph becomes the type-state bound (T035, research R8). Found by analysis, same file: "The hardware layer does not exist yet ..." (after M1), `JOINTS` as a fixed const (after M2), "`Robot::run()` does not check for an empty controller list yet" (after T035). FR-058, FR-009 (SRS NFR-7, NFR-4)

---

## Phase 9: M7 FFI hook (US3), supports SRS delivery step 8

**Goal**: kmr_api can match every field exhaustively, so a field without its FFI line fails the
build.

**Gate**: starts with SRS delivery step 8 (steps 1 to 7 done, D1 A); after T037.

- [ ] T057 [US3] HUMAN: K7.1 public `FieldId` (`Q`, `Qd`, `Tau`, `Kp`, `Kd`), or `const ID: FieldId` on `Slot`, in **kmr_core/src/state/field.rs (invariant)**; users cannot add one; only kmr_api names it, not re-exported to users. Test: an exhaustive `match` with no wildcard arm compiles. FR-062 (SRS FR-10.2, FR-2.2). Unblocks kmr_ffi C1 (part), C3

---

## Phase 10: Polish and cross-cutting

- [ ] T058 [P] HUMAN: SC-008 compile-fail doctests in kmr_core/src/lib.rs (not in state/field.rs, so the invariant file stays as is): no Desired read, no new field type (`impl StateField` outside the core), no new phase (`impl Schedule` outside); the direct field write (kmr_core/src/state/robot_state.rs:29) and the empty robot (T035) exist already. "No adapter or Host impl from controllers" cannot be a compile error under a `#[doc(hidden)] pub` seal (see analysis); review it instead. SC-008 (SRS NFR-2, NFR-3, FR-2.2). Added by analysis
- [ ] T059 [P] HUMAN: CI step in .github/workflows/ci.yml counting SRS markers (`unimplemented!("SRS`, `Undecided("SRS`) in kmr_core/src and failing on any `unwrap`, `expect` or `panic!` outside them; a marker on the control path blocks a release. FR-055, SC-008 (SRS NFR-2, §5 marker rule). Added by analysis: no plan row
- [ ] T060 [P] HUMAN: docs/decisions/0002-hal-contract.md, its one owner (the contract is kmr_core/src/hal.rs; kmr_adapters T061 hands off here): call points, `Powered` guard, five bus checks, hold target (OC1), `impl Hal for &mut A`, empty `Overrides`, `Stages`; the adapter items of kmr_adapters T061 (no "port binding as a const"; `force_torque_off` and the strict fake, the table generic over N, binding by label (D6), plugin baud at `bind`, zero offset (D8), default cadence (D9) with the number of kmr_adapters T064, Protocol 1 health exemption); and one new decision record, the next free number in docs/decisions/, for the type-state empty check and `RunError` (research R8, R9). Constitution V. Added by analysis
- [ ] T061 [P] HUMAN: docs/ARCHITECTURE.md and docs/ROADMAP.md: runtime, state and hardware-contract sections after M1 to M5 (one clock, five bus checks, exit table, const N, faults)
- [ ] T062 HUMAN: FR-056 visibility review over kmr_core/src: every new `pub` item is one that contracts/engine-surface.md names for kmr_api, kmr_adapters or kmr_ffi; the rest is `pub(crate)`. FR-056 (SRS NFR-3)
- [ ] T063 HUMAN: Run specs/002-kmr-core/quickstart.md in full ("Always", § M1 to § M6); 0 failed; record the counts in the PR
- [ ] T064 HUMAN: `/speckit-ov-c4-code` for kmr_core: docs/c4/kmr_core.dsl (Constitution, Development Workflow). An agent may run it with the human

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (T001)** first.
- **Foundational (T002, T003)** before M1: both gate it (Phase 2 checkpoint), T003 though it
  describes M2. Both [P].
- **M1 (T004–T018)**: core commits T004 → T012 in order; T013 any time. Hand-offs: T015 after
  T009–T011; T016 (A0 after T009, A1 after T004, A5 after T009–T011); T017 C0 any time, C2 after
  kmr_api A1, A2; T018 last, the one bench session (blockers: T004–T013, kmr_adapters T017,
  T041, kmr_api T011, controllers T012).
- **M2 (T019–T030)**: gate T018 (D1 A). T020 any time; T021 → T022 → T023 → T024 → T025 → T026
  → T027 → (T028 ‖ T029) → T030.
- **M3 (T031–T043, T065)**: gate T030. T034, T035 land with kmr_api T040. T039 after
  kmr_adapters T020. T042, T043 after T040. T065 any time in M3.
- **M4 (T044–T048)**: after M3. T045 before T049.
- **M5 (T049–T051)**: after M4. T051 after T050; it lands with kmr_api T068.
- **M6 (T052–T056)**: after M5. T053 after T052.
- **M7 (T057)**: after T037 and SRS delivery steps 1 to 7.
- **Polish (T058–T064)**: after the milestones they describe; T063, T064 last.

### Critical path to the first motor (coordination §3)

| CP | Owner | Here |
|---|---|---|
| 1 | HUMAN dxl_rs | kmr_adapters T002, T003 |
| 2 | Agent kmr_adapters | kmr_adapters T004–T017 |
| 3 | HUMAN core #2 | T006, T007 |
| 4 | HUMAN core #3 | T008 |
| 5 | HUMAN core #8 | T009 |
| 6 | HUMAN core #9 | T010, T011 |
| 7 | HUMAN core #10, #11 | T005, T010, T011, T012, T013 |
| 8 | HUMAN core #14 | T004 |
| 9 | Agent kmr_api | T016 (kmr_api A0, A1, A5) |
| 10 | Agent controllers | T017 (controllers C0, C2) |
| 11 | HUMAN bench | T018 (= kmr_adapters T045, kmr_api A9, controllers C6) |

Not needed first: core plan #1, #5, #6, #7, #13, #15, #18+, all of kmr_model.

### Hand-offs this module unblocks

| Core task | Unblocks |
|---|---|
| T004 (K1.1) | kmr_api A1, bench B-1 |
| T009 (K1.6) | kmr_api A0, C5; kmr_adapters T041 (with T010, T011) |
| T009–T011 (K1.6–K1.8) | kmr_api A5, kmr_adapters T041 |
| T025 (K2.5) | kmr_adapters T058, kmr_api T066, kmr_model T032 |
| T027 (K2.7) | kmr_api B7, kmr_adapters T057 |
| T034 (K3.3), T033 (K3.2) | kmr_api T040 (C6 first part; its A6 interim check goes), bench B-2 |
| T035 (K3.4) | kmr_api T040 (C6 first part) |
| T036 (K3.5) | kmr_api C1, kmr_ffi C1, bench C-2 |
| T065 (`test-util`) | kmr_api T037; kmr_ffi T016, T024–T026 |
| T039 (K3.8) | kmr_api C3, bench C-2 |
| T044 (K4.1) | kmr_api C2 (then E1), kmr_ffi C1 |
| T045 (K4.2) | kmr_api F2 |
| T046 (K4.3) | kmr_api C4 |
| T049 (K5.1) | kmr_api Z3 |
| T050 (K5.2) | kmr_api B3, C4, E2 |
| T051 (K5.3) | kmr_api C4 (T038), C6 `NoPolicy` part (T068); controllers C9 |
| T054 (K6.3) | kmr_api C7, bench D-1 |
| T057 (K7.1) | kmr_ffi C1, C3 |

### Waits on other modules

- T018 (one bench session with kmr_adapters T045, kmr_api T017, controllers T014, T015) on
  kmr_adapters T017 (dxl_rs, T002), T041; kmr_api T011 (A5); controllers T012 (C2).
- T030 on kmr_api B7, B2 (T028), kmr_adapters T057 (T029) and kmr_model T062 (the SC-009
  robots).
- T034, T035 with kmr_api T040; T051 with kmr_api T068.
- T039 on kmr_adapters T020.

### Hand-offs to these ids in other tasks files

One-line hand-offs, no duplicate text: kmr_adapters T035 → T004; T036 → T005; T037 → T006, T007;
T038 → T008; T039 → T009; T040 → T010, T011; T056 → T020–T027. kmr_model T046 → T020–T027.
kmr_ffi T009 → T004, T006–T008, T036, T044, T057. T012, T013 have no mirror.

## Parallel opportunities

- T002 ‖ T003.
- T013 (root Cargo.toml) beside any M1 commit.
- T019, T020 beside T021.
- T028 ‖ T029 (other modules).
- T055, T058–T061 beside other work once their milestone landed.

```text
M1 core:   T004 → T005 → T006 → T007 → T008 → T009 → T010 → T011 → T012   (T013 any time)
M1 rest:   T014 → T015 → T016, T017 → T018
M2:        T021 → T022 → T023 → T024 → T025 → T026 → T027 → (T028 ‖ T029) → T030
```

## Implementation strategy

### MVP: delivery step 1 (D1 A)

1. T001 baseline; T002, T003 design notes.
2. T004–T013, each commit green; T014.
3. Hand-offs T015–T017 land in their modules.
4. T018 bench. A motor moves and every exit cuts torque. Stop and validate.

### Incremental

5. M2 (N from the robot line), then M3, M4, M5, M6, each with its quickstart section.
6. M7 when delivery step 8 opens.
7. Polish; then the human PR.

## Pending decisions and the task that holds each

| Decision | Assumed | Task |
|---|---|---|
| D1 delivery gate | A | phase gates (T018 before M2) |
| D2 limit without policy | B | T051 |
| D3 envelope breach | A | T044 (`envelope` bit), T050 (`Breach::Envelope`) |
| D4 N cap | A, 64 | T025 (`MASK_WORDS`), T027 (backstop), T028 (kmr_api assert) |
| D5 markers after `open` | A | T031 |
| D11 pre-sort gain | B | T054 |
| D13 foreign stop | A | none (`signal.rs` untouched) |
| OC1–OC7 (research R4) | the design's, OC4 added | T004 (OC5, OC6), T008 (OC1), T009 (OC3), T010 (OC7), T011 (OC2), T012 (OC4) |
| FR-042 "no sample" | recommended | T036 |
| `no_std` target | `thumbv7em-none-eabihf` | T052, T053 |

## Notes

- Core commits are human commits: no agent trailer question arises here. The constitution's
  Principle 0 trailer rule and AGENTS.md rule (b) still conflict; that is the human's to settle.
- Commit per task, Conventional Commits, scope `core`.
- An agent that reaches a HUMAN task stops and hands it over.
- Name each invariant file in the PR that touches it: T006, T022, T037, T057 (`state/field.rs`),
  T026 (`controllers.rs`, `schedule.rs`, `schedule/macros.rs`), T052 (`signal.rs`), T042
  (`unsafe`).

## Analysis 2026-10-28 (cross-module)

- 2026-10-28: T003 gates M1; T018 takes the one bench blocker list (T004–T013, kmr_adapters T017, T041, kmr_api T011, controllers T012), names the `emergency_stop!` line of controllers T015 and the `hardware` feature; T020 takes the next free record number; T025 names kmr_api T066, kmr_model T032; T030 points at kmr_model T062 for the N = 1, 4, 10, 64 robots; T034, T035 land with kmr_api T040, T051 with kmr_api T068; T060 is the one owner of 0002 (kmr_adapters T061 hands off); new T065 (`test-util`, M3) for kmr_api T037 and kmr_ffi T016, T024–T026; mirror section rewritten as hand-offs.

## Added 2026-10-29 (D17, set of drivers)

- [ ] T069 [2026-10-29] HUMAN: the robot's hardware as a list of drivers, placement A (assumed, pending human). In kmr_core/src/hal.rs: `Claim`, `check` returning it, the sealed walk trait over `HNil`/`HCons<H: Hal, T>`, the nested member error, the group renumbering, the coverage check at finalization (FR-063 (SRS FR-5.1, FR-6.4)), the per-member `Batch` view and `Sample` reborrow; `run_on` takes the list, `run()` is `run_on(hlist![NoHardware])`; the `Powered` guard walks every member (`torque_off` on all, then `close` on all, reverse order). No invariant file. After M1 unless the human picks otherwise (design open choice 2). Plan KS.1. Per specs/002-kmr-core/design-hal-set.md.
