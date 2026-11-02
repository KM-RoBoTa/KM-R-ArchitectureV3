---
description: "Tasks for kmr_adapters, hardware adapters (Dynamixel first), regenerated 2026-10-28"
---

# Tasks: kmr_adapters, hardware adapters (Dynamixel first)

**Input**: `specs/001-kmr-adapters/`: [plan.md](plan.md) (2026-10-28), [spec.md](spec.md)
(re-specified 2026-10-28), [research.md](research.md), [data-model.md](data-model.md),
[quickstart.md](quickstart.md), [contracts/](contracts/). Authority: `docs/srs/SRS.md`.
Core designs: `specs/002-kmr-core/design-first-motor.md`, `design-const-generic-n.md`.

Replaces the tasks of 2026-10-26. Old ids map to new ones in [Old tasks](#old-tasks-mapped).

**Tests**: included. The plan orders the torque tests red first (step A2), and SC-002, SC-004,
SC-006, SC-008 are counted on the scripted backend. No test needs hardware or the C SDK, except
the HUMAN bench tasks.

**HUMAN**: tasks in `kmr_core/` (`.aiignore`) and in `../dxl_rs` are the human's, and so are
edits to `.github/` workflows (T053) and anything `unsafe` (T059; AGENTS.md). An agent never
does them. A task that touches an invariant file names it in **bold**.

**Hand-off**: this plan edits nothing in `kmr_core/`, `kmr_api/` or `controllers/`. A task that
names one of them says "Hand-off" and points at the owning plan's step or task; it keeps its id
for order only.

**Blocked**: an agent task that waits on a HUMAN task says "Blocked on Tnnn".

**Assumed, pending human**: D1 (step 1 scope, option A), D6 (bus label, B), D7 (hardware without a
preset, A: `robot!` takes the table types), D8 (zero, C), D9 (default cadence, A), D15 (shipped
`main.rs` has no bus line, A). Each is isolated to the place the task names; another answer
changes that place only.

**Ids**: `FR-0NN` / `SC-0NN` are this spec's, written with their SRS source: `FR-017 (SRS FR-4.2)`.
User stories are spec.md's: US1 move a preset (P1), US2 safe bring-up and shutdown (P1), US3
custom robot and overrides (P2), US4 clear faults (P2), US5 build and simulate without hardware
(P3).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an unfinished task)
- **[Story]**: US1..US5 from spec.md

Paths are from the repository root. `../dxl_rs` is the sibling SDK repository.

---

## Phase 1: Setup

**Purpose**: a green baseline, and the two SDK changes the critical path starts with.

- [x] T001 Run quickstart.md § 0 from the repo root (`cargo fmt --all --check`, `cargo clippy -p kmr_adapters --all-targets`, `cargo test -p kmr_adapters --offline`, `cargo test --workspace --no-fail-fast`); expect 86 unit tests and 3 doctests green in kmr_adapters (count of 2026-10-28); stop and report if not
- [ ] T002 HUMAN (dxl_rs, critical path step 1): add `force_torque_off` on the `TorqueOff` type-state, or a free function by port, id and protocol, in ../dxl_rs/build/actuator_codegen.rs; it writes `Torque_Enable` = 0 with `write1ByteTxRx1` / `write1ByteTxRx2` at the model's address (24 on AX, 64 on X), whatever the torque state (research R1, contracts/backend.md). Unblocks T017
- [ ] T003 HUMAN (dxl_rs): commit or drop the `.ron` edits in ../dxl_rs and push `feat/battery-status` to KM-RoBoTa/dxl-rs. Unblocks T052 (git dependency) and T053 (CI)

---

## Phase 2: Foundational: strict fake and the backend seam (plan A1, A3 seam part)

**Purpose**: the scripted backend refuses what `DxlRs` refuses (research R2), and the seam has
`force_torque_off`, so the torque tests of Phase 3 compile and fail for the right reason.
Blocks every story's tests.

- [x] T004 In kmr_adapters/src/dynamixel/backend.rs add `fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError>` to `Backend`, doc: "writes Torque Enable = 0 to actuator `id`, whether or not this backend switched it on; clears any handle kept for it"; keep `torque(.., false)` (contracts/backend.md § Change)
- [x] T005 [P] In kmr_adapters/src/dynamixel/dxl_rs.rs (feature `dynamixel`) implement `DxlRs::force_torque_off` as the fallback of contracts/backend.md: call today's `unpower`; it cuts an actuator this backend powered and returns `Err(Io)` with one `warn!` for any other. Build with `cargo build -p kmr_adapters --features dynamixel` where libclang is present; the real SDK call is T017
- [x] T006 [P] In kmr_adapters/src/dynamixel/scripted.rs track two states per (port, id): `powered_here` (switched on through this backend) and `torque` (the register); add `already_on(port, id)` (an earlier run left it on: sets `torque` only); `torqued(port, id)` reads `torque`; `force_torque_off` returns `Ok`, clears both, logs `ForceTorqueOff` (contracts/backend.md § Strictness)
- [x] T007 In kmr_adapters/src/dynamixel/scripted.rs refuse, with `Err(Io)` and never an `assert!`: `torque(.., false)` on an actuator never powered here; `sync_write_goal` naming one not powered here (whole write); `read_health`, `read_setting`, `sync_read_state` on Protocol 1; `sync_read_state` with `out.len() != ids.len()`; any call before `open_ports` or after `close_ports`. Replace every `assert!` / `assert_eq!` in the fake, so a test unwinding through the core's `Powered` guard cannot double-panic (FR-033 (SRS NFR-2))
- [x] T008 In kmr_adapters/src/dynamixel/scripted.rs log `STATE_READS.len()` records per group for `sync_read_state` (3 today, as `DxlRs` sends); in kmr_adapters/src/dynamixel/tests.rs mark the test that asserts one read per group `#[ignore = "dxl_rs sends 3 sync reads per group until its 10-byte block read (T050, T051)"]` (research R12)
- [x] T009 In kmr_adapters/src/dynamixel/tests.rs rewrite, not delete, every test the strict fake breaks: powered setup before goal writes; Protocol 1 bring-up tests assert today's `Health` / `Backend` error until T049; add `the_fake_refuses_a_plain_torque_off_it_never_powered` and `the_fake_refuses_protocol_one_health`. `cargo test -p kmr_adapters scripted` green, no abort (quickstart.md § 1)

**Checkpoint**: `cargo test -p kmr_adapters` green on the strict fake; nothing yet calls
`force_torque_off`.

---

## Phase 3: User Story 2, torque off everywhere (Priority: P1), plan step A, critical path step 2 🎯 MVP

**Goal**: every exit after `open` leaves no actuator powered, read-only ones and ones left on by an
earlier run included. PostInit writes torque off on read-only actuators. FR-017 (SRS FR-4.2,
FR-5.6, FR-5.8), FR-006 (SRS FR-5.2, FR-5.8), FR-033 (SRS NFR-2).

**Independent Test**: quickstart.md § 1, `cargo test -p kmr_adapters torque` and `read_only`, red
before T013 to T015, green after.

### Tests, red first

- [x] T010 [US2] In kmr_adapters/src/dynamixel/tests.rs add the exit tests of contracts/backend.md § Torque tests: `every_actuator_ends_torque_off_read_only_included` (all ids `already_on`, actuator 3 `ReadOnly`; bring-up, a few ticks, `torque_off`, `close`: no id torqued, one `ForceTorqueOff` per actuator, `ClosePorts` last), `a_bring_up_error_after_open_leaves_no_id_powered` (all `already_on`, ping fails on id 2), `torque_off_before_open_sends_nothing` (`open` fails: no `ForceTorqueOff`, `ClosePorts` once), `torque_off_twice_is_harmless` (no panic, no error), and `hold_sends_last_q_to_powered_position_actuators_only` (the read-only one gets nothing; `last.written` ignored, research R3)
- [x] T011 [US2] In kmr_adapters/src/dynamixel/tests.rs add `post_init_writes_torque_off_on_a_read_only_actuator` (actuator 3 `ReadOnly`, `already_on`: after `torque_on`, id 4 off, ids 1..=3 on) and `a_read_only_actuator_left_powered_comes_up_off`, which replaces `a_read_only_actuator_left_powered_fails_configure` (`configure` is `Ok` and warns; id off after `torque_on`)
- [x] T012 [US2] Run `cargo test -p kmr_adapters torque read_only`; confirm T010 and T011 fail because the read-only and `already_on` ids stay powered (not because of a build error or an abort); note the failing names in the PR description

### Implementation

- [x] T013 [US2] In kmr_adapters/src/dynamixel/mod.rs `torque_off`: a no-op until the backend opened its ports (`open` flag); else `force_torque_off` on every actuator of every opened bus, read-only ones included; a failure is a `warn!` naming actuator index, bus label and id, and the loop goes on; idempotent; no panic, no retry (FR-017 (SRS FR-4.2, FR-5.6, FR-5.8); contracts/hal-conformance.md guarantees 1 to 4)
- [x] T014 [US2] In kmr_adapters/src/dynamixel/mod.rs `torque_on`: `Torque::On` gets torque on; `Torque::ReadOnly` gets `force_torque_off` (FR-006 (SRS FR-5.2, FR-5.8))
- [x] T015 [US2] In kmr_adapters/src/dynamixel/mod.rs `configure`: a read-only actuator found powered gives one `warn!` and `Ok`, no longer an error; PostInit switches it off in `torque_on` (research R1)
- [x] T016 [US2] Run quickstart.md § 1 and `cargo clippy -p kmr_adapters --all-targets`: T010, T011 green, every expectation of § 1 met, no test aborts
- [x] T017 [US2] Blocked on T002 (HUMAN dxl_rs). In kmr_adapters/src/dynamixel/dxl_rs.rs replace the T005 fallback with the new `dxl_rs` primitive: `force_torque_off` writes torque off whatever the state and removes the actuator's `Powered` entry if there is one; drop the fallback warning; `cargo build -p kmr_adapters --features dynamixel` green (plan A4)

**Checkpoint**: on the scripted backend no exit after `open` leaves torque on. The real SDK side
waits on T002 and the bench (T045).

---

## Phase 4: User Story 1 and 3, preset hardware half (Priority: P1), plan steps B1, B2, B5

**Goal**: the robot's hardware is a preset table generic over N, checked in const evaluation.
Overrides and the count-check hook (B3, B4) wait for Phase 5b: the first motor does not need
them, while kmr_api A3 to A5 need Phase 5's `Binding` and `bind`. FR-023, FR-027, FR-028 (SRS
FR-6.1, FR-6.4).

**Independent Test**: quickstart.md § 2, `cargo test -p kmr_adapters table`.

- [x] T018 [US3] In kmr_adapters/src/dynamixel/config.rs define `pub struct Table<const NB: usize, const N: usize> { pub buses: [Bus; NB], pub actuators: [Actuator; N] }`; `Bus { label: &'static str, baud: u32, read_every: Option<NonZeroU32> }` (rename `name` to `label`); `Actuator` gains `zero: Zero`; `pub enum Zero { Center, Raw(i32) }` with `impl Zero { pub const DEFAULT: Self = Self::Center; }` (assumed, pending human: D8; the one place); `pub const DEFAULT_READ_EVERY: NonZeroU32` = 4, doc "placeholder until the NFR-1 bench, SC-003" (assumed, pending human: D9; the one place). All `Clone + Copy + Debug`, no heap (contracts/hardware-table.md § Types)
- [x] T019 [US3] In kmr_adapters/src/dynamixel/config.rs replace `validate` and `check_ports` with `pub const fn check(self) -> Self`, panicking in const evaluation with a `Message` built byte by byte. Rejects, per data-model.md § Checks: "bus index out of range; id above 252; id repeated on a bus" (naming both actuators); "mode not supported by the model; mode other than `Position` (names SRS section 8)"; "baud not accepted by a model on its bus" (names the actuator index and the model); "empty or repeated label; NB = 0"; "`Zero::Raw` outside the model's raw range". Every message names the actuator index or copies the bus label's bytes in (the `table` prototype printed "no bus with this label" without it). `PORTS` and the `ports` argument go; until T024 deletes kmr_adapters/src/robot.rs, its `const _` becomes a `Table` passed through `check`, so the build stays green
- [x] T020 [US1] In kmr_adapters/src/dynamixel/mod.rs `check`: cadence = the bus's `read_every`, else `DEFAULT_READ_EVERY`; `check` ignores `overrides` (core K3.8 removes `Overrides::read_every`; drop the branch that reads it); remove `DynamixelError::CadenceMissing` and its test (old T023 part, research R6, R8). K = 0 cannot be written (`NonZeroU32` in the table)
- [x] T023 [US1] Create kmr_adapters/src/presets/mod.rs (the `Preset` trait, moved out of kmr_adapters/src/lib.rs, `const N: usize` until `robot!` takes N from `kmr_model`) and kmr_adapters/src/presets/xm_bench.rs: `impl XmBench { pub const HARDWARE: dynamixel::Table<1, 4> }` = four XM430-W210, ids 1..=4, bus `"bus0"`, 1 000 000 baud, `read_every: None`, `.check()`ed; `impl Preset for XmBench { const N: usize = Self::HARDWARE.actuators.len(); }`; `pub mod presets;` in lib.rs; `kmr_api::presets` re-export keeps working (FR-023 (SRS FR-6.1))
- [x] T024 [US1] In kmr_adapters/src/dynamixel/mod.rs `Dynamixel::new(backend, table: Table<NB, JOINTS>)` takes the table by value; delete kmr_adapters/src/robot.rs, the `Hardware` alias and its `Default` in kmr_adapters/src/lib.rs; make `DxlRs::new()` public under `dynamixel` (kmr_api A4 calls it); update every construction in kmr_adapters/src/dynamixel/tests.rs (FR-022 (SRS FR-6.1); plan B5)
- [x] T026 [US1] In kmr_adapters/src/dynamixel/tests.rs add `table` tests: `XmBench::HARDWARE` passes `check`, `Preset::N` is 4, a bus without a cadence reads every `DEFAULT_READ_EVERY` ticks, a full XmBench write is one sync write per (bus, series) (SC-002 (SRS FR-5.3)); run quickstart.md § 2

**Checkpoint**: no robot hardware is hard-coded outside a preset; `cargo test -p kmr_adapters`
green.

---

## Phase 5: User Story 1, plugin port resolution (Priority: P1), plan step C

**Goal**: ports come from bus plugins (`by_id`, `by_dev`), bound to table buses by label before
any I/O and resolved at Init. FR-029 to FR-031 (SRS FR-6.3, FR-6.4), FR-004, FR-008 (SRS FR-5.10).

**Independent Test**: quickstart.md § 3, `cargo test -p kmr_adapters binding` and `open`.

- [x] T027 [US1] Create kmr_adapters/src/binding.rs: `pub enum Port { ById(&'static str), ByDev(&'static str) }` and `pub struct Binding { pub port: Port, pub baud: Option<u32>, pub bus: Option<&'static str> }`, both `Clone, Copy, Debug, PartialEq, Eq`, doc comments of contracts/port-binding.md § Types; `pub mod binding;` in kmr_adapters/src/lib.rs (FR-029 (SRS FR-6.3))
- [x] T028 [US1] In kmr_adapters/src/dynamixel/mod.rs add to `DynamixelError` (`Copy`, no heap): `UnknownBus { label }`, `UnboundBus { bus }`, `ExtraPlugin { port }`, `LabelRequired { port }`, `BusBoundTwice { bus }`, `BaudRejected { actuator, bus, model, baud }`; each message names the bus label and the serial or path where known (FR-007 (SRS FR-6.4); US3-AS4 asks for the model, FR-027 for the index: both named)
- [x] T029 [US1] In kmr_adapters/src/binding.rs add `match_plugins(labels: [&'static str; NB], plugins: &[Binding]) -> Result<[Binding; NB], DynamixelError>`, the only code of D6 (assumed, pending human: D6, option B; A or C replaces this function only): one bus with or without a label binds; the error cases of contracts/port-binding.md § Matching; no I/O, no allocation (FR-030 (SRS FR-6.3, FR-6.4))
- [x] T030 [US1] In kmr_adapters/src/dynamixel/mod.rs add `pub fn bind(backend: B, table: Table<NB, JOINTS>, plugins: &[Binding]) -> Result<Self, DynamixelError>`: `match_plugins`, then the plugin baud replaces the table's for that bus and every model on it must accept it, else `BaudRejected`; zero backend calls. The public SDK form is `Dynamixel::<DxlRs, NB>::bind(DxlRs::new(), ..)` under `dynamixel` (research R6; FR-027 (SRS FR-6.4))
- [x] T031 [US1] In kmr_adapters/src/dynamixel/port.rs resolve per contracts/port-binding.md § Resolution: `ById(serial)` to the single `/dev/serial/by-id` entry holding it (none or several: `PortNotFound` naming bus label and serial); `ByDev(path)` as given (missing: `PortNotFound`); pre-flight: character device, readable and writable (`PortAccess`, FR-008 (SRS FR-5.10)); two buses on one device: `SharedPort { first, bus }`, before any flush. Bring-up only, never per tick; the `PORTS` lookup goes
- [x] T032 [US1] In kmr_adapters/src/dynamixel/mod.rs `open` over the bound ports: resolve and pre-flight every port, refuse a shared device, then flush and set the latency timer on each (read back 1 ms; skipped where the file does not exist), then `open_ports` once (FR-004 (SRS FR-5.10, FR-5.2))
- [x] T033 [US1] In kmr_adapters/src/dynamixel/tests.rs add `binding` and `open` tests, each asserting zero backend calls before I/O: unknown label, unbound bus, extra plugin, two buses with an unlabelled plugin, two plugins on one label, a plugin baud a model rejects (names the actuator index), one bus without a label binds; `ById` on a scratch `by-id` directory (one match binds, none or two is `PortNotFound`); `ByDev` on a pty opens; a regular file is `PortAccess`; a shared device is `SharedPort` before any flush (SC-004 (SRS FR-13.4))
- [ ] T034 [US1] Hand-off, owner kmr_api plan A3; blocked on T027. kmr_api's `Bus` wraps `kmr_adapters::binding::Binding` (`by_id`, `by_dev`, `.baud(..)`, `.bus("label")`) and drops its own `Port` enum (D6 assumed B). No edit here

**Checkpoint**: a table plus plugins gives a bound adapter or a `run()` error before any I/O.

---

## Phase 5b: User Story 3, overrides and count-check hook (Priority: P2), plan steps B3, B4

**Goal**: every preset default is overridable from `main.rs` in const evaluation, by index or
by group, and the count check has its hook. Needed by kmr_api B1 (delivery step 2), not by the
first motor. FR-024, FR-026 (SRS FR-6.1, FR-6.4).

**Independent Test**: quickstart.md § 2, `cargo test -p kmr_adapters --doc`.

- [x] T021 [US3] After T045 (delivery step 1, D1 A). In kmr_adapters/src/dynamixel/config.rs add the `const fn` overrides of contracts/hardware-table.md § Overrides, each returning `self.check()`: `baud(bus: &str, baud: u32)`, `read_every(bus: &str, ticks: u32)` (0 is a compile error), `id(actuator: usize, id: u8)`, `mode(actuator: usize, mode: Mode)`, `read_only(actuator: usize)`, `zero(actuator: usize, raw: i32)`, and the group forms over an index array: `read_only_group(group: &[usize; K])`, `mode_group(group: &[usize; K], mode: Mode)`, `zero_group(group: &[usize; K], raw: i32)` (no `id` group form: one id per actuator); an unknown label or an index ≥ N, in a group too, is a compile error naming it (FR-026 (SRS FR-6.1); kmr_api FR-020 (SRS FR-5.8))
- [x] T022 [US3] After T045. In kmr_adapters/src/dynamixel/config.rs add `compile_fail` doctests, one per rejected table: repeated id, id 253, unknown label in `.baud`, a baud the model rejects, `Velocity` mode, `read_every` 0, `Zero::Raw` out of range, empty label, NB = 0, a group index ≥ N; and one passing doctest: an override chain in a `const` changes only the values it names (SC-006 (SRS FR-6.4))
- [x] T025 [US3] After T045. Count-check hook (plan B4): in kmr_adapters/src/dynamixel/config.rs add a `compile_fail` doctest, `const _: &[Actuator; 3] = &XmBench::HARDWARE.actuators;` fails with E0308 "expected an array with a size of 3, found one with a size of 4" (FR-024 (SRS FR-6.1, FR-6.4)). Hand-off: the `robot!` line `const _: &[Actuator; N] = &__TABLE.actuators;` is kmr_api plan B1's (kmr_api FR-007; assumed, pending human: D7 A); no kmr_api edit here

**Checkpoint**: an override chain, by index or by group, changes only what it names; bad ones fail
`cargo check` naming the index or label.

---

## Phase 6: User Story 1 and 2, through the engine (Priority: P1), plan step D, critical path steps 3 to 11

**Goal**: the real adapter runs under `run_on` and the first motor moves and stops.

**Independent Test**: quickstart.md § 4 (`cargo test -p kmr_adapters engine`), then § 5 on the
bench.

### Hand-offs to kmr_core (HUMAN; specs/002-kmr-core/tasks.md owns them, design-first-motor.md § 5 order)

- [ ] T035 [US1] Hand-off, HUMAN, owner core T004 (K1.1, Time getters; critical path step 8). No edit here
- [ ] T036 [US2] Hand-off, HUMAN, owner core T005 (K1.2, SIGTERM and SIGHUP; step 7; carries fix-plan "Torque can stay on" item 4). No edit here
- [ ] T037 [US1] Hand-off, HUMAN, owner core T006, T007 (K1.3, K1.4, write mask and `Command`; **kmr_core/src/state/field.rs**; step 3). No edit here
- [ ] T038 [US1] Hand-off, HUMAN, owner core T008 (K1.5, one History slot per push, hold target; step 4). No edit here
- [ ] T039 [US1] Hand-off, HUMAN, owner core T009 (K1.6, `run_on`, `impl Hal for &mut A`; step 5). No edit here
- [ ] T040 [US2] Hand-off, HUMAN, owner core T010, T011 (K1.7, K1.8, call points and the `Powered` guard; steps 6, 7). No edit here

### Agent

- [x] T041 [US2] Blocked on T039 (core T009, with `impl Hal for &mut A`; else a shared log inside the fake) and T040 (core T010, T011). In kmr_adapters/src/dynamixel/tests.rs add `engine` tests through `run_on(&mut adapter)` on a pty with the strict fake: shutdown (three ticks of goal writes, one hold, `ForceTorqueOff` for ids 1 to 4, `ClosePorts` last); an estop skips that tick's write and the hold; a missing actuator returns `Missing` naming index, bus and id after torque off and close; a panicking controller still gets torque off and close, then the panic goes on (plan D1; contracts/hal-conformance.md § Exit paths)
- [ ] T042 [US1] Hand-off, owner kmr_api plan A5 (critical path step 9); blocked on kmr_api A5 (kmr_api T011), which waits on T024, T030, T039, T040 (core T009–T011). No edit here, and none in kmr_api/src/robot.rs from this plan: kmr_api wires `run()` through the `__Spec` that `robot!` emits (kmr_api A4): no plugin → the core's `NoHardware` (its one warning, kmr_core FR-042); plugins → `Dynamixel::<DxlRs, NB>::bind(DxlRs::new(), XmBench::HARDWARE, &bindings)` then `run_on`; plugins without the `dynamixel` feature → a `run()` error naming the feature, never simulation (FR-031 (SRS FR-6.3)). This crate's part is T024 and T030
- [ ] T043 [US5] Hand-off, owner kmr_api plan A7; blocked on T042. kmr_api's tests check: `run()` with no plugin opens no port and logs exactly one simulation warning; with plugins and without the `dynamixel` feature it returns the error naming the feature (quickstart.md § 4; SC-007 (SRS FR-6.3)). No edit here
- [ ] T044 [US1] Hand-off, owner controllers plan C0 and C2 (critical path step 10); C2 blocked on T042. `controllers/src/main.rs`: `robot!(XmBench)` (C0, unblocked, before kmr_api A4), a bounded swing from `prev(0)`, `shutdown!` after a fixed time (C2); no bus line in the shipped file (assumed, pending human: D15 A); the bench adds `.add_plugins(Bus::by_id("<U2D2 serial>"))` locally. No edit here
- [ ] T045 [US2] HUMAN: Bench (critical path step 11; quickstart.md § 5), one session with core T018, kmr_api T017 and controllers T014, T015. Blocked on core T004–T013, T017, T041, kmr_api T011, controllers T012. The four actuators follow the swing (SC-001); after a shutdown and after an emergency stop, raised by the local `emergency_stop!` line of controllers T015, Dynamixel Wizard reads `Torque Enable` = 0 on ids 1 to 4, also with one actuator read-only (SC-005); ten runs in a row reopen the port; without the udev rule bring-up fails with the latency-timer hint and nothing is powered (FR-004)

**Checkpoint**: delivery step 1 (D1 A): a motor moves and every exit cuts torque.

---

## Phase 7: User Story 4, clear faults (Priority: P2)

**Goal**: every fault is a named error or a bit, on the strict fake. FR-014, FR-015, FR-032 (SRS
FR-2.6, FR-13.4).

**Independent Test**: SC-004 on the scripted backend.

- [x] T046 [US4] In kmr_adapters/src/dynamixel/tests.rs re-check under the strict fake: missing actuator → `Missing`; a failed read keeps the last value and sets `stale` (with `offline` for one lost actuator, `bus_error` for a failed group); a failed write sets `bus_error` and the next tick writes again; a non-finite or out-of-range target, a `Qd` write on a position actuator, a write to a read-only actuator → not sent, `rejected`; `check` warns once per Protocol 1 actuator that `Qd` and `Tau` are never read (FR-010); no test panics (SC-004 (SRS FR-13.4, NFR-2))
- [ ] T047 [US4] HUMAN: Bench: unplug one actuator mid-run; `offline` and `stale` are set for it and the loop keeps its grid (the ~34 ms SDK stall is a known limit, research R13)

---

## Phase 8: The rest, plan step E

**Purpose**: off the critical path. Each task names its story.

- [x] T048 [US3] After T045. In kmr_adapters/src/dynamixel/units.rs add `origin(actuator)`, the only reader of `Zero`: `Center` → model centre (2048 X, 512 AX/RX), `Raw(r)` → `r`; position converts both ways through it; `read_all` checks the seed positions convert inside the register range (SRS FR-5.2 "double-check zeros", research R7); tests: round trip with a `Raw` offset (FR-005 (SRS FR-5.2); assumed, pending human: D8)
- [ ] T049 [US4] Protocol 1 health under the SDK boundary exemption: add `BackendError::Unsupported` in kmr_adapters/src/dynamixel/backend.rs; `read_health` returns it for Protocol 1 in kmr_adapters/src/dynamixel/dxl_rs.rs (at the call site) and in kmr_adapters/src/dynamixel/scripted.rs; `health` in kmr_adapters/src/dynamixel.rs warns once per Protocol 1 actuator and skips voltage and temperature; the Protocol 1 bring-up tests of T009 flip to `Ok` (FR-006 (SRS FR-5.2); research R11)
- [ ] T050 [US1] HUMAN: dxl_rs: one 10-byte block sync read at address 126 in ../dxl_rs/src/transport.rs (research R12)
- [ ] T051 [US1] Blocked on T050. In kmr_adapters/src/dynamixel/dxl_rs.rs `sync_read_state` makes one call per group; `STATE_READS` becomes `&[10]` there and in kmr_adapters/src/dynamixel/scripted.rs; remove the `#[ignore]` of T008; the declared read cost follows (FR-010, FR-035 (SRS FR-5.3, NFR-11))
- [ ] T052 [US5] Blocked on T003. In Cargo.toml (root) replace `dxl_rs = { path = "../dxl_rs" }` with the git dependency on the pushed branch or a pushed rev (`git ls-remote` checks it), then `cargo update -p dxl_rs` (SC-007 (SRS NFR-5))
- [ ] T053 [US5] HUMAN (CI workflow edit, as core T053, T059 and controllers T018, T027). Blocked on T052 and on the human's choice of dxl-rs access in CI (public repository or an ssh-agent step; fix-plan decision 1). In .github/workflows/ci.yml install libclang and the C cross toolchain, and cross-build `controllers` with `--features kmr_api/dynamixel` for aarch64-unknown-linux-gnu and armv7-unknown-linux-gnueabihf; the job is green (SC-007 (SRS NFR-5))
- [ ] T054 [US5] HUMAN: dxl_rs: put the no-op stub bindings (build/dynamixel/dynamixel_sdk.rs, "Using stub DynamixelSDK bindings") behind an explicit feature in ../dxl_rs/build.rs, so a robot binary never builds on stubs; then bump the rev of T052
- [ ] T055 [US5] Blocked on T053. Cross-build `controllers` with `--features kmr_api/dynamixel` for aarch64 and armv7 on a machine with no serial device; the build log has no "Using stub DynamixelSDK bindings" line; the toolchain steps live in .github/workflows/ci.yml (SC-007 (SRS FR-5.5, NFR-5))
- [ ] T056 [US3] Hand-off, HUMAN, owner core T020–T027 (K2.0 to K2.7, design-const-generic-n.md § 5 steps 0 to 7; **state/field.rs**, **controllers.rs, schedule.rs, schedule/macros.rs**); step 8 is kmr_api's. No edit here
- [ ] T057 [US3] Blocked on T056 (core T027, K2.7). In kmr_adapters/src/dynamixel.rs, kmr_adapters/src/binding.rs and kmr_adapters/src/dynamixel/dxl_rs.rs: `Dynamixel<B, NB, N>`, `impl Hal<N>`, `bind` takes `Table<NB, N>`, `DxlRs` sizes its handle table from N (design-const-generic-n step 9; FR-022 (SRS FR-6.1))
- [ ] T058 [US3] Blocked on core T025 (K2.5, HUMAN: `MASK_WORDS` public in `kmr_core::hal`) only; the doctests pass literal bit sets, so kmr_model's `model::PRISMATIC` (kmr_model T032) is not needed here. In kmr_adapters/src/dynamixel/config.rs add `pub const fn check_kinds(self, prismatic: [u64; MASK_WORDS]) -> Self` (`MASK_WORDS` from `kmr_core::hal`; bit i is word i / 64, bit i % 64): a set bit is a compile error "SRS FR-2.2: prismatic Dynamixel actuator has no mm conversion" naming its index; add a `compile_fail` doctest and a passing one. Not `[u64; N.div_ceil(64)]` (does not compile in kmr_api's `Spec<N>` on stable). Hand-off: kmr_api plan B3 calls it as `const _: Table<__NB, __M> = __TABLE.check_kinds(model::PRISMATIC);` (a `const _: () = ..` form is E0308); no kmr_api edit here (FR-021 (SRS FR-2.2); SC-006)
- [ ] T059 [US1] HUMAN (`unsafe impl GlobalAlloc`: AGENTS.md keeps anything `unsafe` off limits for agents). In kmr_adapters/src/dynamixel/tests.rs add a counting `#[global_allocator]` under `#[cfg(test)]` (per-thread counter) and a test of 10 000 ticks of `read` and `write` on the scripted bus asserting zero allocations (SC-008 (SRS NFR-1); FR-013)
- [ ] T060 [US1] Review kmr_adapters/src/dynamixel/io.rs and kmr_adapters/src/dynamixel.rs against FR-034 (SRS NFR-10): per-actuator data in arrays by index, read-only and unread fields as bitsets out of band, buses and groups by index, enums not trait objects, no padding in per-tick data; fix a gap in place or report it in the PR
- [x] T067 [US1] After T023 and T045 (plan E10). In kmr_adapters/src/presets/xm_bench.rs add `pub const READ_COST: Duration` and `pub const WRITE_COST: Duration` on `XmBench`, each preset later the same: computed in const evaluation from `HARDWARE` with the line-time formula of `io::Groups::costs` (bytes per (bus, series) transaction × 10 bits ÷ baud) and the backends' read plan (`STATE_READS`: three sync reads per group today, one after T051; one crate-private const both backends and the preset read, so T051 changes all three). Add a `table` test: on the scripted backend, the `Layout` that `check` declares has `read_cost == XmBench::READ_COST` and `write_cost == XmBench::WRITE_COST`, so the consts cannot drift. Hand-off: kmr_api plan C7 re-exports them under `kmr_api::presets`; the bench records them as `declared_read_ns`, `declared_write_ns` (bench contracts/results-schema.md); no kmr_api edit here (FR-035 (SRS NFR-11, FR-5.3))
- [ ] T068 [US1] After T045; blocked on kmr_api T021 (the front door calls `__model!`; today kmr_api/src/robot_macro.rs:24 reads `<Preset>::N`). This crate owns the presets' hardware half, so the removal is here (kmr_model T028 hands off to it): in kmr_adapters/src/presets.rs drop `Preset::N`, and in kmr_adapters/src/presets/xm_bench.rs its derivation from `HARDWARE.actuators.len()`; N comes from the preset's model half (kmr_model T024; kmr_model research R7, review #2). The table keeps its own length; the count check (T025, kmr_api T023) keeps table and model in step. Drop the "`Preset::N` is 4" check of the T026 test (FR-023 (SRS FR-6.1))

---

## Phase 9: Polish and cross-cutting

- [ ] T061 Hand-off, owner core T060 (HUMAN; docs/decisions/0002-hal-contract.md records the contract of kmr_core/src/hal.rs, so one owner). The adapter items go there: drop "port binding as a const"; `force_torque_off` and the strict fake, the table generic over N, binding by label (D6), plugin baud at `bind` (research R6), zero offset (D8), default cadence (D9), Protocol 1 health exemption. This plan's PR lists the SRS problems of research R15 for the human (constitution V). No edit to 0002 here
- [x] T062 [P] Update docs/ARCHITECTURE.md (hardware layer section) and docs/ROADMAP.md (hardware items, `dxl_rs` changes, velocity and current modes) to this plan
- [ ] T063 HUMAN Bench SC-003: write-tick I/O under 1 ms at a 1 ms tick, read ticks logged apart, measured costs within the declared ones; pick the `DEFAULT_READ_EVERY` value (D9)
- [ ] T064 Blocked on T063. Set `DEFAULT_READ_EVERY` in kmr_adapters/src/dynamixel/config.rs to the measured value and drop "placeholder" from its doc; hand the number to core T060 for docs/decisions/0002-hal-contract.md
- [ ] T065 Run quickstart.md §§ 0 to 3 in full (`cargo fmt --all --check`, `cargo clippy --workspace --all-targets`, `cargo test --workspace --no-fail-fast`)
- [ ] T066 Update docs/c4/kmr_adapters.dsl with `/speckit-ov-c4-code` (constitution, Development Workflow)

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (T001)** first. T002, T003 are HUMAN and start at once; they block T017, T052, T053 only.
- **Foundational (T004–T009)** after T001; blocks every story's tests.
- **Phase 3, step A (T010–T017)** after Foundational. Critical path step 2. T017 waits on T002.
- **Phase 4, steps B1, B2, B5 (T018–T020, T023, T024, T026)** after Phase 3 (plan order; files
  overlap in mod.rs, tests.rs).
- **Phase 5, step C (T027–T033; T034 hand-off)** after Phase 4 (`bind` takes the `Table`).
  kmr_api A3 to A5 need it for the first motor.
- **Phase 5b, steps B3, B4 (T021, T022, T025)** after Phase 5 and T045; needed by kmr_api B1
  (delivery step 2), not by the first motor.
- **Phase 6, step D**: T035–T040 are hand-offs to core T004–T011 (HUMAN, core order), any time.
  T041 after T039, T040 and Phase 3. T042, T043, T044 are hand-offs (kmr_api A5, A7; controllers
  C0, C2); T042 waits on kmr_api A5. T045 is one bench session with core T018, kmr_api T017,
  controllers T014, T015, blocked on core T004–T013, T017, T041, kmr_api T011, controllers T012.
- **Phase 7 (T046)** after Phase 5. T047 after T045.
- **Phase 8**: T049, T059 (HUMAN), T060 after Phase 5; T048 after Phase 5 and T045. T067 after
  T023 and T045; kmr_api C7 waits on it. T051 after T050. T052 after T003. T053 (HUMAN)
  after T052. T055 after T053 (and T054 for the no-stub log). T057 after T056 (core T027). T058
  after core T025 (K2.5) only. T068 after T045 and kmr_api T021.
- **Polish**: T061 is a hand-off to core T060; T062 after the steps it describes; T064 after
  T063; T065, T066 last.

### Within a phase

- Tests before the code that turns them green (T010–T012 before T013–T015).
- config.rs types (T018) before `check` (T019); overrides (T021) before doctests (T022), both in
  Phase 5b.
- Errors (T028) before `match_plugins` (T029) before `bind` (T030).

### Critical path to the first motor

The chain is coordination § 3. Here: T002 → T017; T001, T004–T016 (critical path step 2);
T018–T020, T023, T024, T026, T027–T033 (table and binding, needed by step 9); core T004–T013
(steps 3 to 8, HUMAN; hand-offs T035–T040); T041; kmr_api T011 (A5; T034, T042 hand-offs, step
9); controllers T012 (C0 to C2; T044 hand-off, step 10); T045, the one bench session (step 11).
Phase 5b is off this path.

## Parallel opportunities

- T005 and T006 (dxl_rs.rs, scripted.rs) after T004.
- HUMAN core T004–T013 and T020–T027 (hand-offs T035–T040, T056) alongside every agent phase.
- T062 beside the other Polish tasks.

```text
Foundational:  T004 → (T005 ‖ T006) → T007 → T008 → T009
Step C:        T027 → T028 → T029 → T030 → T031 → T032 → T033
Step B rest:   T021 → T022, T025 (after step C and T045)
HUMAN core:    core T004 → … → T013 (hand-offs T035–T040)
```

## Implementation strategy

### MVP: delivery step 1 (D1 A)

1. Setup, Foundational, then Phase 3: torque off everywhere, on the fake.
2. Phase 4 and 5: preset table, plugins. Phase 5b (overrides, count-check hook) can follow
   later.
3. HUMAN core T004–T013 (hand-offs T035–T040) and dxl_rs T002; then T041; kmr_api A5 and controllers C2 (T042, T044
   hand-offs).
4. HUMAN bench T045. A motor moves and stops: delivery step 1 is done.

### Incremental

5. Phase 7 faults, Phase 8 (zero, Protocol 1 health, one sync read, CI, const N, kinds,
   allocation count).
6. Polish, then the PR from `agent/<topic>` into the human branch, `agentic` label.

## Old tasks mapped

From the tasks of 2026-10-26.

| Old | Status | New |
|---|---|---|
| T001 (git dependency) | marked done, not done: redone | T003, T052 |
| T028 (one 10-byte sync read) | marked done, not done: redone | T050, T051 (T008 ignores the test meanwhile) |
| T032 (Protocol 1 health) | marked done, not done: redone | T049 |
| T033 (torque off skips read-only) | marked done, not done: redone | T010–T017 |
| T041 (cross job green) | marked done, not done: redone | T053, T055 |
| T015, T020, T030 | partly kept | T019, T028, T031 |
| T023 (`CadenceMissing`) | that part removed | T020 |
| T036, T040, T042, T043, T047 | still valid | T045, T047, T054, T055, T066 |
| T044 | rewritten | T061 |
| T008, T009, T010, T011, T012, T013, T017, T021, T022, T029, T035 (read-only part), T039, T046 | obsolete (plan § Old tasks) | none |

## Notes

- Commits: AGENTS.md rule (b) and the human's final decision: no AI or co-author trailer. The
  constitution's Principle 0 asks for one. This conflict is the human's to settle; until then
  commits follow the human's decision.
- Delivery order (constitution I, SRS § 8) holds under D1 A; every task here is inside step 1 or
  after it.
- An agent that reaches a HUMAN task, or a task blocked on one, stops and hands it over.
- Commit per task or per group, Conventional Commits, scope `adapters`, `api` or `controllers`.

## Analysis 2026-10-28 (cross-module)

- 2026-10-28: T035–T040, T056 one-line hand-offs to core T004–T011, T020–T027; T061 hand-off to core T060 (one owner of 0002), T064 hands its number there; T042 waits on T039, T040, not T035; T045 takes the one bench blocker list and the `emergency_stop!` line of controllers T015; T021, T022, T025, T048, T067 after T045; T053 (ci.yml) and T059 (`unsafe`) HUMAN; T058 waits on core K2.5 only (breaks the kmr_api T025 → T058 → kmr_model T032 → kmr_api T025 cycle); new T068 drops `Preset::N`.

## T012 red run (2026-10-28)

`cargo test -p kmr_adapters -- torque read_only powered`, before T013 to T015. Failed for the
right reason, no build error, no abort:

- `every_actuator_ends_torque_off_read_only_included`,
  `post_init_writes_torque_off_on_a_read_only_actuator`,
  `a_read_only_actuator_left_powered_comes_up_off`: `configure` refused the read-only id left
  powered (`Setup` "read-only but its torque is on"), so it stayed on.
- `a_bring_up_error_after_open_leaves_no_id_powered`: the ids left on by an earlier run stayed
  powered.
- `torque_off_twice_is_harmless`: no `ForceTorqueOff` sent (0 of 8).

Already green before the fix (guards): `torque_off_before_open_sends_nothing`,
`hold_sends_last_q_to_powered_position_actuators_only`. All seven green after T015.

## Added 2026-10-29 (D17, set of drivers)

- [x] T069 [2026-10-29] Agent: blocked on core T069 (HUMAN). In kmr_adapters/src/dynamixel.rs `check` returns the member's `Claim` (owned mask = its table's indices, local groups per (bus, series), cadence, costs) instead of a whole-N `Layout` (T020); `read`, `read_all` and `write` touch only owned indices; the table names each actuator's index when the robot has several brands (one-brand tables keep `0..N` in model order, T018). Plan E11. Per specs/002-kmr-core/design-hal-set.md §3, §5, §7. Done 2026-11-02 on agent/driver-set: `Table::at([index; M])` and `Table::in_order()` build a const-checked `Placement<NB, M>` (table checked, each index below `JOINTS`, none taken twice, compile errors naming the actuator); `Dynamixel::new` and `bind` take it and keep the actuators by engine index, `None` where another member owns the index. `check` claims exactly the placed indices, local (bus, series) groups from 0, and refuses more than 256 groups (`TooManyGroups`); every bring-up call, `read`, `read_all`, `write`, `hold` and `torque_off` touch only owned actuators and slots; `write` skips an index it does not own without a fault. Presets keep `0..N` through `in_order()`. Tests on the strict fake: a two-bus member at engine indices 3 and 1 (claim, bring-up, read, faults, write, hold, torque off), and a full `run_on` beside a second member on pseudo-terminals. Review fix 2026-11-02: `only_subset_called` checked only the bus and the id, so a member sending another index's target passed; it now checks each call's (bus, model, id) against the table and each goal against the target of its own engine index (distinct per index), and a test proves it refuses a foreign goal, bus and id. Checked against a deliberately broken `write` and `hold` (target of `index ^ 1`): the old check passed, the new one fails both. `TooManyGroups` could not fire (groups are per (bus, series), two series, at most `MAX_PORTS` = 4 buses: 8 groups against 256) and was checked after `self.groups` was overwritten; the variant is gone, a const assert in `check` (`SERIES * B::MAX_PORTS <= MAX_GROUPS`) keeps the bound for any backend that builds.

## Added 2026-10-29 (D18, limits through the API)

- [ ] T070 [2026-10-29] Future (D18, decided 2026-10-29, not in this version): actuator limits configurable through the API as a plugin capability. Every actuator plugin whose hardware supports it, all Dynamixel models and not only the RX, may expose "set health limits" (Min and Max Voltage Limit, Temperature Limit), written to the actuator's own EEPROM registers, torque off, and read back by `health` in kmr_adapters/src/dynamixel.rs, which already checks against them. Needs a capability shape in kmr_api first (spec Roadmap, FR-006 (SRS FR-5.2)).
