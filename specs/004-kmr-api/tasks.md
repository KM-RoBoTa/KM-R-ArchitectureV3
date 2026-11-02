---
description: "Tasks for kmr_api, the public user API, generated 2026-10-28"
---

# Tasks: kmr_api, the public user API

**Input**: `specs/004-kmr-api/`: [plan.md](plan.md) (2026-10-28), [spec.md](spec.md) (coordinated
2026-10-28), [research.md](research.md), [data-model.md](data-model.md),
[quickstart.md](quickstart.md), [contracts/](contracts/). Authority: `docs/srs/SRS.md`.
Decisions and critical path: `specs/spec-coordination-2026-10-28.md` §§ 1, 3, 5. Core designs:
`specs/002-kmr-core/design-first-motor.md`, `design-const-generic-n.md`.

**Tests**: included. The plan has test steps (A7, Z4), the quickstart lists the expected
results, and SC-005 to SC-013 are counted by tests. No test needs hardware, except the HUMAN
bench task.

**HUMAN**: `kmr_core/` and `../dxl_rs` are the human's. Their steps are not tasks here; a task
that waits on one says "Blocked on core Kx.y (HUMAN)" and names, in bold, any invariant file that
core step touches (`state/field.rs`, `sealed.rs`, `signal.rs`, `schedule.rs`, `controllers.rs`,
`schedule/macros.rs`). No task here edits an invariant file. Tasks marked HUMAN are the
maintainer's.

**Hand-off**: this list edits `kmr_api/` and `docs/` only. A task that names another module's
work says "Hand-off" and gives that module's task or plan step id: kmr_adapters `Tnnn`
(its tasks.md), core `Kx.y` (core `Tnnn` for a core task with no plan row), controllers `Cn`,
kmr_ffi `An`/`Cn`/`Dn`, bench `D-n` (their plans). kmr_model (003) has no step ids; it is cited
by its tasks.md ids (`kmr_model Tnnn`), file or research id.

**HUMAN, `unsafe`**: a task that writes `unsafe` (a counting `#[global_allocator]`, a hand-written
slot) is HUMAN: T029, T048 (AGENTS.md: anything `unsafe` stays off limits for agents).

**Blocked**: "Blocked on" names what must land first.

**Assumed, pending human**: D1 (A, step 1 includes kmr_api's wiring), D2 (B, `run()` refuses a
limited actuator without a policy), D3 (A, clamp and flag), D4 (A, N ≤ 64), D5 (A, compile-time
home marker), D6 (B, `.bus("label")`), D7 (A, `robot!` `hardware = ..`), D11 (B, `presort-off`),
D12 (A), D13 (A), D16 (replace `try_lock`). Each is isolated to the place its task names.

**Ids**: `FR-0NN` / `SC-0NN` are spec.md's, with their SRS source. Stories: US1 name the robot
and write controllers (P1), US2 plug the robot into this machine (P1), US3 time and faults (P2),
US4 Zenoh (P2), US5 fallbacks and envelopes (P3), US6 secure messaging (P3), US7 payloads (P3).
Plan rows (A0 to G2) are named in each task. Findings `U1`.. are in [Analysis](#analysis-2026-10-28).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an unfinished task)
- **[Story]**: US1..US7 from spec.md

Paths are from the repository root.

Phases follow the SRS delivery order (constitution I), not story priority: phase 3 is delivery
step 1 (critical path step 9), phase 4 is step 2, then steps 3 and 4, 5, 6, 7, 8.

---

## Phase 1: Setup

- [x] T001 Run specs/004-kmr-api/quickstart.md § "Every change" from the repo root (`cargo build --workspace`, `cargo test --workspace --no-fail-fast`, `cargo clippy --workspace --all-targets`, `cargo fmt --all --check`, `cargo run -p kmr_api --example complete_api_example`, `cargo build -p controllers`); record the test counts of kmr_api; note that `run()` prints two simulation lines today (kmr_api/src/robot.rs); stop and report if anything is red. Done 2026-10-29: all green; kmr_api 22 unit, 3 integration, 2 doctests (3 ignored); `run()` prints one of its two lines per run (none or some plugins), the core prints nothing; the example builds, then panics in `initial_state()` (`todo!`, T020)

---

## Phase 2: Foundational

**Purpose**: the error variants that `Spec::bind` (T007) and `run()` (T011) return. Plan A6
lists them after A5; A4 needs them first (U8).

- [x] T002 In kmr_api/src/error.rs grow `UserError` with `FeatureOff { feature: &'static str }`, `NoHardwareDeclared`, `BringUp { cause: String }` (heap is fine outside the tick) and `DtOutOfRange { dt: Duration }`; keep `DuplicateBus { port }` and `NoControllers` (until T040). Each `Display` is one line naming the cause and the fix (contracts/builder.md § Errors: the value and the range for `dt`, `hardware = ..` for `NoHardwareDeclared`, `dynamixel` for `FeatureOff`). Replace the derived `Debug` with one that prints the `Display` text, so `fn main() -> Result<(), UserError>` shows a readable line (research R4). Unit tests: `Debug` equals `Display` for every variant. FR-021 (SRS FR-6.4, NFR-2), FR-015 (SRS FR-3.1); plan A6, variants part

**Checkpoint**: `cargo test -p kmr_api` green; nothing returns the new variants yet.

---

## Phase 3: First motor wiring (Priority: P1), plan phase A, critical path step 9 (MVP)

**Goal**: `run()` hands the Dynamixel adapter, bound to the `Bus::by_id` serial, to the core's
`run_on`; `Time` getters and `Deg` exist for the controllers demo; one simulation warning, the
core's. Delivery step 1 under D1 A (decided 2026-10-28: D1 A).

**Independent Test**: quickstart.md § Phase A: `cargo test -p kmr_api --test run` with and
without `--features dynamixel`; controllers C2 builds on `Deg` and `elapsed()`.

- [x] T003 [P] [US1] Create kmr_api/src/units.rs (plan A2, research R12): `pub struct Deg(pub f32)`, `Cm(pub f32)`, `M(pub f32)`, `DegPerS(pub f32)`, `CmPerS(pub f32)`, `MPerS(pub f32)`; `From<Deg> for Q` (rad), `From<Cm> for Q` and `From<M> for Q` (mm), and the per-second forms into `Qd` (rad/s, mm/s); no `Tau` conversion; `mod units; pub use units::*;` in kmr_api/src/lib.rs. Doc: the type does not know an actuator's kind (SRS issue 4). Tests: `Q::from(Deg(180.0))` is π, `Q::from(M(1.0))` is 1000, `Q::from(Cm(1.0))` is 10, within f32 precision (SC-009 (SRS FR-2.2)). FR-033 (SRS FR-2.2). Unblocked; controllers C2 needs it
- [x] T004 [P] [US3] Blocked on core K1.1 (HUMAN, Time getters; no invariant file). In kmr_api/src/time.rs make `Time<'a>` a struct with a private named field `inner: &'a kmr_core::Time`, not `.0` (FR-028 (SRS section 3 State, NFR-4)), and forward six getters by value, no logic: `elapsed() -> Duration`, `dt() -> Duration`, `tick() -> u64`, `overrun_count() -> u64`, `last_overrun() -> Option<Duration>`, `accumulated_overrun() -> Duration` (contracts/handles.md § Time). Never `f32` seconds. FR-038 (SRS FR-3.3); plan A1. Controllers C2 needs `elapsed()` Done 2026-10-29 after core K1.1 (918928c): `Time { inner }`, six getters forwarded by value; read by kmr_api/tests/run.rs.
- [x] T005 [P] [US2] Blocked on kmr_adapters T027 (plan C1, `Binding`, `Port`). In kmr_api/src/bus.rs make `Bus` wrap `kmr_adapters::binding::Binding { port, baud, bus }`; keep `by_id(serial)` (preferred) and `by_dev(path)` (doc: less reliable, `/dev/tty*` numbers change across reboots), `.baud(u32)`; add `.bus(label: &'static str)`, optional for a one-bus robot (assumed, pending human: D6, option B; `Bus::bus` is its one place in kmr_api); drop the local `Port` enum; keep a crate-private port string for the duplicate check of T011. Unit test: `Bus::by_id("FT8ISZ6H").baud(3_000_000)` differs from `Bus::by_id("FT8ISZ6H")` in the baud only (US2-AS3). FR-022, FR-023, FR-024 (SRS FR-6.3, FR-6.5); plan A3. Hand-off of kmr_adapters T034 Done 2026-10-29: the duplicate check compares `Bus::port()`.
- [x] T006 [US1] Hand-off, owner controllers plan C0 (unblocked): `controllers/src/main.rs` adopts `kmr_api::robot!(XmBench);` before T008, because a bare `kmr_api::Robot::new()` stops compiling once `Robot` is generic (E0283, research R5). No edit here; T008 does not start until C0 has landed Landed: controllers/src/main.rs calls `kmr_api::robot!(XmBench);` (checked 2026-11-02).
- [x] T007 [US2] Blocked on T002, T005, kmr_adapters T018, T023 (plan B1, B2: `Table<NB, N>`, `XmBench::HARDWARE`), T024 (B5: public `DxlRs::new()`) and T030 (C2: `Dynamixel::bind`). Create kmr_api/src/spec.rs (plan A4, research R2): `#[doc(hidden)] pub trait Spec { type Adapter: kmr_core::Hal; fn bind(plugins: &[Binding]) -> Result<Self::Adapter, UserError>; }`; `#[doc(hidden)] pub type __DxlAdapter<const NB: usize>`, the Dynamixel adapter under kmr_api's `dynamixel` feature and `NoHardware` without it; `#[doc(hidden)] pub fn __bind_dynamixel<const NB: usize>(table, plugins)`: with the feature, `Dynamixel::<DxlRs, NB>::bind(DxlRs::new(), table, plugins)` with the adapter error's `Display` as `BringUp { cause }`; without it, `Err(FeatureOff { feature: "dynamixel" })`. Both `cfg`s live in kmr_api, never in the macro expansion (a `cfg` there tests the user's features). Add `mod spec;` and hidden re-exports (`Spec`, `Binding`, `Table`, `__DxlAdapter`, `__bind_dynamixel`) in kmr_api/src/lib.rs. FR-002, FR-005 (SRS FR-6.1, FR-5.1) Partly done 2026-10-29: `__DxlAdapter<NB>`, `__bind_dynamixel` and the hidden re-exports (`Spec`, `Binding`, `Table`) exist; `Spec` has no item yet. `type Adapter` and `bind` are left out: they choose one adapter over the set of drivers (D17), so they land with T069. Done 2026-10-29 for one driver, as a one-member set: `Spec` has `type Adapter: Hal` and `bind`, both documented "one driver for now; becomes the driver set in T069"; hidden `__NoHardware` and `__bind_none` (`NoHardwareDeclared`) added for the model arm. T069 widens `type Adapter` to `type Drivers`; `robot!` users see no change.
- [x] T008 [US2] Blocked on T007 and T006 (controllers C0). In kmr_api/src/robot.rs make the builder `Robot<H, C = Scheduled>` with `PhantomData<fn() -> H>`, `plugins: Vec<Binding>` and `dt: Option<Duration>` (data-model.md § Builder data); `impl<H: Spec> Robot<H> { pub fn new() }` and `Default`; every setting and registration keeps `H`. Switch the unit tests of kmr_api/src/robot.rs and kmr_api/src/group/test.rs to a test-only `Spec` with `type Adapter = NoHardware`. Plan A4 (A5 also lists `Robot<H, C>`; it lands here because T009's alias needs it, U7). FR-005 (SRS FR-6.1, FR-5.1) Done 2026-10-29, with two differences: the builder keeps `plugins: Vec<Bus>` (each `Bus` holds its `Binding`, T005) and the test `Spec` carries no adapter type (D17, T069). `set_dt` stores `dt`; `run()` hands it to the core.
- [x] T009 [US1] Blocked on T008. In kmr_api/src/robot_macro.rs (contracts/robot-hardware.md § Emits): the preset arm emits `#[doc(hidden)] pub struct __Spec;`, `const __NB: usize` and `const __M: usize` read from `<Preset>::HARDWARE`, `const __TABLE: $crate::Table<__NB, __M> = <Preset>::HARDWARE;`, `impl $crate::Spec for __Spec { type Adapter = $crate::__DxlAdapter<__NB>; fn bind(..) { $crate::__bind_dynamixel(__TABLE, ..) } }` and `pub type Robot = $crate::Robot<__Spec>;`. The `model = ..` arm emits a `__Spec` with `type Adapter = NoHardware` whose `bind` returns `Err(NoHardwareDeclared)` (research R3 row 4), else `robot!(model = ..)` stops compiling (U9). All `__` items `#[doc(hidden)]`; the `N == JOINTS` assert stays. Check that kmr_api/tests/robot_macro.rs (kmr_model 003's) and the example still build. FR-002, FR-005, FR-007 (SRS FR-6.1, FR-6.4); plan A4 Partly done 2026-10-29: both arms emit `__Spec`, `impl Spec for __Spec {}` and `Robot = $crate::Robot<__Spec>`; the preset arm emits `__NB`, `__M`, `__TABLE`. The `impl Spec` bodies (adapter type, `bind`, `NoHardwareDeclared` for the model arm) wait on T069 (D17). Done 2026-10-29 for one driver: the preset arm emits `type Adapter = __DxlAdapter<__NB>` and `bind` over `__bind_dynamixel(__TABLE, ..)`; the model arm `type Adapter = __NoHardware` and `bind` over `__bind_none` (`NoHardwareDeclared`). No `cfg` in the expansion. Tests: kmr_api/tests/spec.rs, with and without `--features dynamixel`.
- [x] T010 [US2] Blocked on core K1.6 (HUMAN: `run_on`, `impl Hal for &mut A`, `NoHardware` prints "no bus plugin: simulating" once at `open`; no invariant file). In kmr_api/src/robot.rs delete the `eprintln!` "kmr: no bus plugin: simulating, no hardware is driven"; keep the "not driven yet" line until T011. Never before K1.6, or no warning prints at all (bench plan, cross-module needs). FR-026 (SRS FR-6.3); plan A0 Done 2026-10-29 after core K1.6, K1.7 (9d169a5, 47014db): `NoHardware::open` warns "no bus plugin: simulating" once; kmr_api prints nothing.
- [x] T011 [US2] Blocked on core K1.6, K1.7, K1.8 (HUMAN; design-first-motor commits 6 to 8; no invariant file), T005, T009, T010. In kmr_api/src/robot.rs: `run()` calls a private `finalize()` (two plugins with the same port string: `DuplicateBus { port }`, FR-025 (SRS FR-6.4)), then: no plugin, `inner.run_on(NoHardware)`; plugins, `H::bind(&plugins)?` then `inner.run_on(adapter)`, with the adapter's run error as `BringUp { cause }` (its `Display`). Delete the "not driven yet" `eprintln!` in the same change (research R1, R24 #5). `Ok(())` after a shutdown and after an emergency stop. Critical path step 9; hand-off of kmr_adapters T042 (plan D2). FR-014 (SRS section 3 "run() finalization"), FR-026 (SRS FR-6.3), FR-027 (SRS FR-6.4, FR-5.5); plan A5 Done 2026-10-29 after core K1.6 to K1.8 (9d169a5, 47014db, f839660), for one driver: `run()` is `finalize()`, then no plugin: `run_on(NoHardware)`; plugins: `H::bind(&plugins)?` then `run_on(adapter)`, its error's `Display` as `BringUp { cause }`. Both `eprintln!` lines are gone. T069 swaps the adapter for the driver set.
- [x] T012 [US1] Blocked on T011. In kmr_api/src/robot.rs `finalize()` checks `dt` first, before `bind`: `Some(d)` with `d` zero or outside [10 µs, 1 s] is `DtOutOfRange { dt }`; exactly 10 µs and 1 s are accepted; unset passes and keeps the core's 1 ms warning. Interim until core K3.3; T040 removes it (research R13). FR-015 (SRS FR-3.1, FR-6.4); plan A6, check part Done 2026-10-29: `finalize()` checks `dt` first ([10 µs, 1 s], `DT_MIN`/`DT_MAX` in robot.rs), then the ports; unset keeps the core's 1 ms warning.
- [x] T013 [US2] Blocked on T004, T011, T012. Create kmr_api/tests/run.rs with `kmr_api::robot!(XmBench);` and a test-only `Spec` over a recording adapter (the HAL seal is a public module); a file-local lock for the global signal bus. Cases of quickstart.md § Phase A: (a) no plugin, a controller calls `shutdown!` at tick 3: `Ok(())`, it read `tick()` 0..=3 and `dt()` = 1 ms, the log has the core's "no bus plugin: simulating" once and nothing from kmr_api (US2-AS1, SC-007); (b) a plugin, feature off: `Err(FeatureOff { feature: "dynamixel" })`, no port opened; (c) a plugin on the recording `Spec`: one `open`, one write per tick, one `torque_off`, `Ok(())`; (d) feature on, `Bus::by_id("MISSING")`: `Err(BringUp { cause })` naming the serial, `torque_off` and `close` ran (US2-AS4); (e) two plugins on one port: `Err(DuplicateBus { port })`, no I/O (US2-AS2); (f) `dt` 5 µs and zero: `Err(DtOutOfRange)`, no I/O; 10 µs and 1 s run (US1-AS9); (g) a two-bus table with one plugin: the adapter's binding error comes back as `BringUp` unchanged (research R1); (h) `BringUp` through `Debug` prints the `Display` text. Run with and without `--features dynamixel`. SC-005 (SRS FR-3.1, FR-6.4, FR-8.4), SC-007 (SRS FR-5.5, NFR-5); plan A7. Hand-off of kmr_adapters T043 Done 2026-10-29, cases (a) to (h): the recording driver (`Records`, `FailsOpen`) checks the full call order of a 3-tick run and `check, open, torque_off, close` on a failed open; (d) also on the real adapter with an unresolvable serial (no port opened); (g) a two-bus table. Not asserted: the log line of (a), since the core's tracing subscriber is process-global. Passes with and without `--features dynamixel`.
- [ ] T014 [US1] Blocked on T011. In kmr_api/examples/complete_api_example.rs put `.add_plugins(Bus::by_id("FT8ISZ6H"))` under `if cfg!(feature = "dynamixel")` (`add_plugins` returns `Self`, both branches have one type; research R6); its comment says that without the line `run()` simulates with the core's one warning. `cargo run -p kmr_api --example complete_api_example` simulates and exits `Ok` (SC-001 (SRS FR-8.4)). FR-069 (SRS FR-8.4); plan A8 Partly done 2026-11-02: the bus line is under `if cfg!(feature = "dynamixel")`, both branches one type; the example builds with and without the feature. Open: it simulates with the core's one warning, then panics in its own `use_time` controller (`todo!()`, line 34), and after that `robot.group(..)` is `todo!("Fix the invariance issue")` (kmr_api/src/group.rs) with `R_ARM = [3, 4, 5]` past N = 4: not `Ok`. Those are the maintainer's example and group design, not edited here.
- [ ] T015 [US1] Hand-off, owner controllers plan C1 (unblocked: `hardware = ["kmr_api/dynamixel"]` in controllers/Cargo.toml) and C2 (controllers/src/main.rs; critical path step 10; blocked on T003 and T004: `Deg`, `elapsed()`). No edit here
- [ ] T016 [US2] Blocked on T011 and controllers C1. With no robot plugged in, build controllers/ with `cargo build -p controllers --features hardware` natively and with `--target aarch64-unknown-linux-gnu` and `--target armv7-unknown-linux-gnueabihf` (needs libclang and the C cross toolchain); record the result in the PR. The CI job is kmr_adapters T053, T055 (hand-off). FR-027 (SRS FR-5.5), SC-007 (SRS NFR-5) Partly done 2026-11-02, no robot plugged in: `cargo build -p controllers --features hardware` builds natively and for aarch64 (`CC_aarch64_unknown_linux_gnu` and the linker set to `aarch64-linux-gnu-gcc`; an ELF aarch64 binary). armv7 fails in dxl_rs's build script: no `arm-linux-gnueabihf-gcc` on the dev machine, so the C SDK does not compile; without `hardware`, `cargo check -p controllers --target armv7-unknown-linux-gnueabihf` is green. Open: the armv7 build on a machine with the toolchain (or kmr_adapters T053's CI job).
- [ ] T017 [US2] HUMAN: Bench, critical path step 11 (plan A9), one session with core T018, kmr_adapters T045 and controllers T014, T015 (C6). Blocked on core K1.1 to K1.10 (core T004–T013), kmr_adapters T017, T041, T011 and controllers T012 (C2). XmBench plugged in, the `Bus::by_id` line enabled locally in controllers/src/main.rs: `cargo run -p controllers --features hardware`; the motors swing, `shutdown!` ends the run, Dynamixel Wizard reads `Torque_Enable` = 0 on ids 1 to 4. T014 (the example's bus line) is off this path

**Checkpoint**: delivery step 1 (D1 A, decided 2026-10-28: D1 A): a motor moves and stops with
torque off. No task of phase 4 or later starts before T017.

---

## Phase 4: `robot!` hardware, handles, payloads (Priority: P1, P3), plan phase B, delivery step 2

**Goal**: `robot!` carries the hardware and the model data; handles are sealed and pinned by
compile-fail tests; payloads go through the builder; N becomes a const generic.

**Independent Test**: quickstart.md § Phase B; `cargo test -p kmr_api --test run --test ui`.

- [x] T018 [P] [US1] In kmr_api/src/state.rs and kmr_api/src/sensors.rs replace the tuple field `.0` with a private named field `inner` (`State<'a> { inner: &'a mut kmr_core::RobotState }`, `Sensors<'a> { inner: &'a kmr_core::Sensors }`, data-model.md § Tick data); every method forwards as before; update the doc example at state.rs:27-36. No plan row covers it (U4). FR-028 (SRS section 3 State, NFR-4) Done 2026-11-02: `State { inner }`, `Sensors { inner }`; the `compile_fail,E0616` doctest names `inner`.
- [x] T019 [US1] Blocked on T018 (the cases only). Add `trybuild` to `[dev-dependencies]` in kmr_api/Cargo.toml and the harness kmr_api/tests/ui.rs, one `#[test] fn ui()` with `compile_fail("tests/ui/*.rs")` (003 research R10). The one owner of both: kmr_model T018 hands off here, and kmr_model's ui cases wait on them, so land the harness first. Add cases with pinned `.stderr` in kmr_api/tests/ui/: `state_inner.rs` (`state.inner` and `state.0` are private), `raw_field.rs` (no path from a `State` to a core field: `robot.desired.q[0] = ..` fails), `forge_field.rs` (a user `impl Field for MyType` fails: sealed). SC-008 (SRS NFR-3); FR-028, FR-035 (SRS NFR-3, NFR-4). No plan row covers it (U5) Done 2026-11-02: `trybuild` dev-dependency, kmr_api/tests/ui.rs (`compile_fail("tests/ui/*.rs")`); cases `state_inner.rs` (E0616 on `inner`, E0609 on `.0`), `raw_field.rs` (E0609 `desired`), `forge_field.rs` (E0277, sealed `Slot`).
- [x] T020 [US1] Blocked on T018 (same file). Create kmr_api/src/home.rs (plan B4, research R10; assumed, pending human: D5, option A): a sealed `HomeDecided` trait with `#[diagnostic::on_unimplemented(message = "kmr: SRS FR-2.7: home state is open; ...")]`; move `initial_state()` and `go_home(ramp)` out of kmr_api/src/state.rs into `impl State` there, same names and argument shapes, each bounded by `HomeDecided`. Remove every call from kmr_api/examples/complete_api_example.rs (three `initial_state()`, one `go_home(90)` today; the plan says two, U18). Add kmr_api/tests/ui/home_state.rs: `s.go_home(90)` and `let _: [f32; 4] = s.initial_state()` each give E0277 with that message. FR-036 (SRS FR-2.7) Done 2026-11-02 (D5 A, assumed, pending human): kmr_api/src/home.rs, `HomeDecided` (sealed, no impl, `const OPEN: Infallible`, so a body is unreachable without a panic), `initial_state<T: Field + HomeDecided>` and `go_home<R: HomeDecided>(ramp)`; the example's `initialization` keeps no call; kmr_api/tests/ui/home_state.rs pins the E0277 message for `go_home(90)`, `[Q; N]` and `[f32; 4]` (the last also fails `Field`).
- [x] T021 [US1] Blocked on kmr_model T009 (the `model!` proc macro, kmr_model/src/lib.rs, emit.rs; 003 research R9). The `__model!` switch that kmr_model T010 hands off here: in kmr_api/src/lib.rs re-export `kmr_model::model` as the hidden `__model`; in kmr_api/src/robot_macro.rs the preset arm calls `$crate::__model!($crate; preset = $preset)` and the `model = ..` arm `$crate::__model!($crate; model = $path)` (the input of kmr_model T007), so `actuator_count` and `<Preset>::N` are no longer called (kmr_model T055, kmr_adapters T068); the front door keeps `State`, `Robot`, `__Spec` and the `N == JOINTS` assert, the proc macro emits none of them (003 research R3); remove the `three.urdf` `compile_fail` doctest once kmr_model T022 replaces it (003 review #10). Move the `@items` arm behind a hidden internal macro (`#[doc(hidden)] #[macro_export] macro_rules! __robot_items`), so `robot!(@items 4)` no longer sets N (003 review #7); add `const _: () = assert!(N >= 1 && N <= 64, "SRS write mask: more than 64 actuators");` (assumed, pending human: D4, option A; kmr_core FR-012). Its UI case waits for T033: the `N == JOINTS` assert fires first today (U13). FR-007 (SRS FR-6.1, FR-6.4); plan B2 Done 2026-11-02: `kmr_api::__model!` is the hidden re-export of `kmr_model::model`; every arm calls `$crate::__model!($crate; preset = $preset)` or `($crate; model = $path)`, then the hidden `$crate::__robot_items!()` (aliases, `__Spec`, the `N == JOINTS` assert and the D4 A `N >= 1 && N <= 64` assert); `robot!(@items ..)` no longer exists. `<Preset>::N` is no longer read: N comes from the model half, the preset table is checked to cover `0..N` (`__check_coverage`). The `three.urdf` doctest is gone, replaced by kmr_model T022's kmr_api/tests/ui/n_differs_from_core.rs. kmr_model T055 removed `actuator_count` and `kmr_api::__actuator_count`. The `n_65` ui case stays with T033.
- [ ] T022 [US1] Hand-off, owner kmr_model plan (003): N, actuator order, limits, links, the `kmr:` messages and their trybuild cases (US1-AS2, US1-AS3, SC-006 first half) in kmr_api/tests/robot_macro.rs, kmr_api/tests/formats.rs and kmr_api/tests/ui/ (003 plan § Source Code, research R10). No edit here; T019 shares the harness only
- [x] T023 [US2] Blocked on T009, kmr_adapters T021 (plan B3: `const fn` overrides, group forms included) and T025 (B4: count-check hook). In kmr_api/src/robot_macro.rs add the arms of contracts/robot-hardware.md § Forms (assumed, pending human: D7, option A; these arms and `@table` are its one place): `robot!(Preset, hardware = |hw| <expr>)` rewritten to `{ let hw = <Preset>::HARDWARE; <expr> }` in a `const`; `robot!(model = "..", hardware = Table { .. })`; an `@table` arm that reads `__NB` and `__M` from the preset (not the override, so one error, research R7), emits `__TABLE`, the count check `const _: &[Actuator; N] = &__TABLE.actuators;` and `__Spec`. Re-export `Table`, `Actuator` and the table's field types publicly in kmr_api/src/lib.rs, so the user names no other crate. kmr_api splits `hardware = ..` off before the proc macro. FR-006 (SRS FR-6.1), FR-007 (SRS FR-6.4, FR-6.2), FR-020 (SRS FR-5.8); plan B1 Done 2026-11-02 (D7 A, assumed, pending human): `robot!(Preset, hardware = |hw| <expr>)` (the bare preset arm is that form with `|hw| hw`), `__TABLE = { let hw = <Preset>::HARDWARE; <expr> }`, `__NB`/`__M` from the preset; `robot!(model = .., hardware = <table expr>)`, `__NB`/`__M` read from the table's placement. Both end in the hidden `__robot_items!(@table)`: the count check `const _: [Actuator; N] = __TABLE.actuators;` (an array, not `&[..; N]`: same E0308, no borrow) and `Spec` over one Dynamixel member in model order. The `{ brand: part }` arm of T069 is tried first. The table types are public under `kmr_api::dynamixel` (T069). contracts/robot-hardware.md no longer lists the two forms as not done.
- [x] T024 [US2] Blocked on T023. In kmr_api/tests/run.rs add `mod overrides { kmr_api::robot!(XmBench, hardware = |hw| hw.read_only(3).read_only_group(&[0, 1])); }`: the table marks actuators `[true, true, false, true]` read-only and `(__NB, __M, N)` is `(1, 4, 4)`; and `robot!(model = "tests/fixtures/four.urdf")` with a plugin: `Err(NoHardwareDeclared)` before any I/O. In kmr_api/tests/ui/ add `count.rs` (`robot!(model = four.urdf, hardware = Table { .. })` with a 3-row table, N = 4, fixture path as kmr_model T021 says: E0308 "expected an array with a size of 4, found one with a size of 3", SC-006 (SRS FR-6.4, FR-6.1); the one owner, kmr_model T027 (its US7 AS3) hands off here) and `bad_index.rs` (`hw.read_only(7)`: one E0080 naming the index). US2-AS5 on hardware (unpowered after bring-up, still read) is kmr_adapters T011, T014. Quickstart § Phase B Done 2026-11-02: kmr_api/tests/run.rs `overrides` (`[true, true, false, true]` read-only, `(1, 4, 4)`), `no_table` (`NoHardwareDeclared`, no tick ran), `table` (a four-row table in model order); kmr_api/tests/ui/count.rs (one E0308, "expected an array with a size of 4, found one with a size of 3") and bad_index.rs (one E0080, "actuator 7: past the table's 4 actuators", plus the "erroneous constant" note). The fixture path is `../../../../kmr_api/tests/fixtures/four.urdf` (kmr_model T021).
- [ ] T025 [US1] Blocked on T023, T066 (the `MASK_WORDS` re-export, after core K2.5; U6), kmr_model T032 (the emit of `model::PRISMATIC`, kmr_model/src/emit.rs; 003 data-model "Emitted facts") and kmr_adapters T058 (plan E6; it waits on core K2.5 only). In kmr_api/src/robot_macro.rs emit `const _: $crate::Table<__NB, __M> = __TABLE.check_kinds(model::PRISMATIC);` (not `const _: () = ..`, which is E0308; research R9). UI case kmr_api/tests/ui/prismatic.rs: E0080 with kmr_adapters' message naming the index and SRS FR-2.2. FR-002 (SRS FR-6.1, FR-2.2); plan B3, kinds part
- [ ] T026 [US1] Blocked on T025 and the kmr_model (003) emit of `LOWER`, `UPPER`, `SPEED`, `EFFORT`, `CONTINUOUS`, `LINK_NAMES` (kmr_model T031, T032, T043). In kmr_api/src/spec.rs add to `Spec` the consts `PRISMATIC`, `CONTINUOUS` (`[u64; MASK_WORDS]`), `LOWER`, `UPPER` (`[Q; N]`), `SPEED` (`[Qd; N]`), `EFFORT` (`[Tau; N]`) and `LINK_NAMES`, filled by `robot!` from `model::*` in kmr_api/src/robot_macro.rs. Data only, no core type: the `[Limit; N]` builder is T067 (phase 5, after core K5.2), so payloads (T027) do not wait on the core's M5. FR-002 (SRS FR-6.1, FR-4.5); plan B3, limits part (consts) Partly done 2026-11-02, the part T027 needs: `Spec::LINK_NAMES: &'static [&'static str]`, filled by every `robot!` arm from `model::LINK_NAMES` (a slice: the link count is not N). Open: `PRISMATIC`, `CONTINUOUS` (wait on T025, core K2.5, kmr_model T032) and `LOWER`, `UPPER`, `SPEED`, `EFFORT` as `[_; N]`: typed on `kmr_api::N` today, a model N other than the core's would add a second error to the n_differs_from_core ui case, so they land with T032 (`Spec<N>`).
- [x] T027 [US7] Blocked on T026 (`Spec::LINK_NAMES`). Rewrite kmr_api/src/payload.rs (plan B5, research R11): `Payload::medium(mass_kg: f32, mount: &'static str)` and `Payload::heavy(..)`, fields `tier: Medium | Heavy`, `mass_kg`, `mount`; no light tier; `pub use` it in kmr_api/src/lib.rs. In kmr_api/src/robot.rs `register_payload` stores it (replaces today's `todo!()`); `finalize()` checks each mount against `Spec::LINK_NAMES` after the port check and before `bind` (data-model.md § Finalization step 3); in kmr_api/src/error.rs add `UnknownLink { link }`, whose message names the link. The center-of-mass function carries `unimplemented!("SRS FR-8.3: center of mass")` and finalization never calls it. The rule "Mass finite and > 0" (data-model.md) has no error in the plan: do not invent one (U1). FR-062 (SRS FR-8.4), FR-063 (SRS FR-8.1), FR-064 (SRS FR-8.1, FR-6.4), FR-065 (SRS FR-8.3) Done 2026-11-02 on `Spec::LINK_NAMES` (T026, part). `Payload` and `Tier` are public, fields private with `const` getters `tier()`, `mass_kg()`, `mount()`; `finalize()` checks the mounts after the ports, so `bind()` refuses an unknown link too. `center_of_mass` carries the `unimplemented!` and an `#[expect(dead_code)]`; no mass check (U1).
- [x] T028 [US7] Blocked on T027. In kmr_api/tests/run.rs: `register_payload(Payload::medium(1.2, "nope"))` gives `Err(UnknownLink { link: "nope" })` before any I/O (US7-AS2); a known link runs. In kmr_api/examples/complete_api_example.rs register the example's payload through `.register_payload(..)` and remove the comment claiming that `add_controller` spawns a thread (SRS FR-8.4). SC-001, SC-005 (SRS FR-8.4); FR-069 (SRS FR-8.4); plan B5 Done 2026-11-02: the unknown-link test has a plugin, so the refusal comes before the binding (`FeatureOff` or `BringUp` otherwise); the example registers `Payload::medium(1.2, "link4")` (XmBench's last link) and its comment no longer says `add_controller` spawns a thread. The example still panics in its own `use_time` `todo!()` (T014).
- [ ] T029 [P] [US4] HUMAN (`unsafe`: the counting `#[global_allocator]` of its test, and the hand-written slot if no crate fits). Create kmr_api/src/slot.rs (plan Z1, moved forward for T031; research R15): `Latest<T: Copy>`, `pub(crate)` for now, one writer and one reader; writer one atomic swap per put; reader one load when nothing is new, one swap when something is; an untaken value overwritten is counted (superseded). Use the `triple_buffer` crate if its writer can tell an untaken value; else the proved hand-written version, its `unsafe` blocks marked for T030. Tests: `take()` returns a value once, then `None`; two puts before a take count one superseded; a counting `#[global_allocator]` shows 0 allocations on the reader thread over 2 000 000 takes. FR-044 (SRS section 4 Constraints)
- [ ] T030 [US4] HUMAN: review every `unsafe` of kmr_api: kmr_api/src/slot.rs if T029 hand-wrote the slot, and the counting allocators of T029 and T048 (constitution IV; research R22 #3)
- [ ] T031 [US1] Blocked on T029 (HUMAN). Make `Latest<T>` public in kmr_api/src/lib.rs; in kmr_api/examples/complete_api_example.rs the gamepad hand-off uses it (reader in the inline controller's context, writer in the threaded context) in place of `Arc<Mutex<..>>` and `try_lock`. Assumed, pending human: D16 (replace); under "keep", `Latest` stays `pub(crate)` and the example keeps `try_lock`. FR-069 (SRS NFR-1, Definitions "Context"); plan B6
- [ ] T032 [US1] Blocked on core K2.1 to K2.7 (HUMAN: const-generic N; invariant files **kmr_core/src/state/field.rs** (K2.2), **kmr_core/src/controllers.rs, kmr_core/src/schedule.rs, kmr_core/src/schedule/macros.rs** (K2.6)) and kmr_adapters T057 (plan E5). In kmr_api/src/state.rs, kmr_api/src/group.rs, kmr_api/src/robot.rs, kmr_api/src/spec.rs: `State<'a, N>`, `GroupView<'a, N, K>`, `Robot<N, H, C>`, `Spec<N>` (N a separate parameter: an associated const cannot size an array, research R2). FR-004 (SRS FR-6.1); plan B7; core K2.8
- [ ] T033 [US1] Blocked on T032. In kmr_api/src/robot_macro.rs the aliases bind N (`State<'a> = $crate::State<'a, N>`, `Robot = $crate::Robot<N, __Spec>`); drop `kmr_api::N` and the `N == JOINTS` assert; add kmr_api/tests/ui/n_65.rs: "SRS write mask: more than 64 actuators" under `cargo check` (T021). Hand-offs: the `three.urdf` case turning into a passing test is kmr_model T048 (its ui case, one owner); core K2.10 (SC-009 there: N = 1, 4, 10, 64 build, robots of kmr_model T062; 65 fails). FR-004, FR-007 (SRS FR-6.1); plan B7
- [ ] T066 [US1] Blocked on core K2.5 (HUMAN, core T025: `MASK_WORDS` public in kmr_core/src/hal.rs; no invariant file). In kmr_api/src/lib.rs re-export `MASK_WORDS` hidden (`#[doc(hidden)] pub use`), at the path kmr_model's emitted `[u64; <krate>::MASK_WORDS]` names (kmr_model T032; 003 data-model: "kmr_core's constant through kmr_api"). Nothing else. Moved out of T025: there it closed a cycle (T025 → kmr_adapters T058 → kmr_model T032 → T025). FR-002 (SRS FR-6.1, FR-2.2); plan B3, kinds part

**Checkpoint**: delivery step 2: `robot!` names the robot and its hardware; payloads are
checked before I/O; `cargo test -p kmr_api` green.

---

## Phase 5: Faults, age, settings, adapter access (Priority: P2), plan phase C, delivery steps 3 and 4

**Goal**: a controller reads age, stale flags, fault flags and cadence; the builder sets the
shutdown hold and limit policies; user code reaches the adapter outside the loop; an empty
builder does not compile.

**Independent Test**: quickstart.md § Phase C, on a scripted host and adapter (kmr_core
`test-util` feature, core T065).

- [ ] T034 [US3] Blocked on core K3.5 (HUMAN, per-actuator age, `ever_read`). In kmr_api/src/state.rs and kmr_api/src/group.rs add `age_at(i) -> Result<Option<u32>, StateError>` (ticks), `sample_age() -> Option<u32>` (the oldest actuator's), `stale_at(i) -> bool`, forwarded from kmr_core FR-018; `None` means never read (no-hardware run, or before the first read). `stale_at` past N is not specified (U2). FR-031 (SRS FR-2.1, FR-2.6); plan C1
- [ ] T035 [US3] Blocked on core K4.1, K4.4 (HUMAN, per-slot fault flags). In kmr_api/src/state.rs add the read-only `Faults` view: per actuator, bit sets `stale`, `bus_error`, `offline`, `rejected`, `joint_limit`, `envelope` (assumed, pending human: D3, option A); per tick `overrun`, `sensor_missing`, `remote_timeout`, `unauthorized`; `State::faults()`, `State::prev_faults(depth)` (depth ≥ `HISTORY_DEPTH` is an error). Controllers cannot set or clear a flag; core's `ExternalFlags` is never re-exported. FR-039 (SRS FR-13.4); plan C2
- [ ] T036 [US3] Blocked on core K3.8 (HUMAN, cadence read-only from `Layout`). In kmr_api/src/state.rs add `read_every(i) -> NonZeroU32` (ticks), read-only; no builder knob, the hardware table owns it (kmr_adapters FR-026). `read_every` past N is not specified (U2). FR-042 (SRS FR-5.3); plan C3
- [ ] T037 [US3] Blocked on T034, T035, T036 and core T065 (HUMAN, M3: the `test-util` feature, scripted `Host` and recording adapter; enabled for kmr_api's tests only). In kmr_api/tests/run.rs, on the core's scripted host and adapter: a failed read of actuator 2 sets `stale_at(2)` only on the next call (US3 independent test); a tick without a read gives `sample_age()` one more than the tick before (US3-AS3); a no-hardware run gives `age_at(i)` = `Ok(None)` and `sample_age()` = `None`; an overrun tick grows `overrun_count()` by one and `last_overrun()` is the lateness (US3-AS2); `elapsed()` and `dt()` are `Duration`s (US3-AS1). Quickstart § Phase C
- [ ] T038 [US1] Blocked on core K4.3, K5.2, K5.3 (HUMAN; no invariant file) and T067. In kmr_api/src/robot.rs add `set_shutdown_hold(Option<Duration>)`, default `None` (torque off right after the hold command); `set_limit_policy(i, Policy)`, `set_limit_policy_group(&G, Policy)`, `set_limit_policy_all(Policy)`; re-export `Policy { Clamp, Warn, Abort }` in kmr_api/src/lib.rs; no default (decided 2026-10-28 (for now): D2 B, option B); `finalize()` hands `[Limit; N]` (T067) and `[Option<Policy>; N]` to the core's `set_limits`. XmBench limits reach core K5.2 only after kmr_model T029 (real bench URDF); until then the placeholder's are not handed over (kmr_model T023). In kmr_api/src/error.rs add `NoPolicy { actuator }`, whose message names the actuator and the fix `set_limit_policy_all`. An index past N is not specified (U2). Controllers C9 and the bench need `set_limit_policy_all` (hand-off). FR-018 (SRS FR-4.1), FR-019 (SRS FR-4.5); plan C4 Note 2026-11-02: the hold part exists as `Robot::set_hold(Duration)` (forwards the core's opt-in hold; no call = torque off and close, no hold command), not `set_shutdown_hold(Option<Duration>)`; the name is the maintainer's. The policy part waits on core K5.2, K5.3.
- [x] T039 [US2] Blocked on core K1.6 (`impl Hal for &mut A`). In kmr_api/src/robot.rs add `bind(self) -> Result<Bound<H, C>, UserError>` (finalization and `Spec::bind`, no I/O), `Bound::adapter(&mut self) -> &mut H::Adapter`, `Bound::run(self) -> Result<H::Adapter, UserError>` (hands the adapter back); `Robot::run()` becomes `self.bind()?.run().map(drop)`, over one private `drive()`. Never reachable in a tick (kmr_core FR-050). Test in kmr_api/tests/run.rs: `bind()?.adapter()`, then `run()` returns the adapter. FR-041 (SRS FR-5.9); plan C5 Done 2026-11-02, after T069: the one adapter is the list of drivers (design-hal-set.md §7), so `Bound::drivers(&mut self) -> Option<&mut H::Drivers>` and `Bound::run(self) -> Result<Option<H::Drivers>, UserError>`; `None` without a bus plugin, where the run simulates and no member exists. `Bound::run` lends each member (`impl Hal for &mut A`) through the hidden `Lend` (kmr_api/src/spec.rs, the bound of `Spec::Drivers`), so a member's error there names it `&mut <type>`. `Robot::run()` is `bind()` then the private `drive()` on the drivers by value, not `bind()?.run().map(drop)`, so its errors keep the bare type name. Tests in kmr_api/tests/drivers.rs, not run.rs: they need members that bind without a serial port (the fakes there); run.rs's XmBench would open one.
- [ ] T040 [US1] Blocked on core K3.2, K3.3, K3.4 (HUMAN; K3.4 does not touch `schedule.rs`) and T032; lands with core K3.3 and K3.4, which break kmr_api's build until it does. In kmr_api/src/robot.rs thread the core's type-state: `Robot<N, H, C, E = Empty>` (`Empty`, `Filled`), so `run()` exists on `Filled` only; drop the interim `dt` check of T012; map `RunError::Config(DtOutOfRange)` to `DtOutOfRange` and `Hal(e)` to `BringUp` (research R13); the `NoPolicy` arm is T068. In kmr_api/src/error.rs drop `NoControllers` and `From<CoreApiError>`. Add kmr_api/tests/ui/no_controller.rs: `Robot::new().run()` does not compile, the diagnostic names the fix (US1-AS10). FR-015 (SRS FR-3.1), FR-016 (SRS NFR-4); plan C6, first part
- [ ] T041 [US1] Blocked on T038, T040, T068. In kmr_api/tests/run.rs: a limited actuator without a policy gives `Err(NoPolicy { actuator })` before any I/O; with `set_limit_policy_all(Policy::Clamp)` it runs; the `dt` cases of T013 (f) still pass through the core's error. Quickstart § Phase C
- [ ] T067 [US1] Blocked on T026 and core K5.2 (HUMAN, the `Limit` type; no invariant file). In kmr_api/src/spec.rs add a `const fn` that builds `[Limit; N]` from `Spec`'s consts (T026), typed `Q`/`Qd`/`Tau`, with `lower` and `upper` `None` wherever `CONTINUOUS` has bit i, never the emitted `Q(0.0)` placeholder. Data only: no policy (T038 hands it to the core). Unit test: a continuous actuator gets `None` bounds. Moved out of T026, so payloads (T027) do not wait on core K5.2. FR-002 (SRS FR-6.1, FR-4.5); plan B3, limits part (`Limit`)
- [ ] T068 [US1] Blocked on core K5.3 (HUMAN; no invariant file), T038 (`NoPolicy` in kmr_api/src/error.rs) and T040; lands with core K5.3, whose new `ConfigError::NoPolicy` the match of T040 does not cover until then. In kmr_api/src/robot.rs map `RunError::Config(NoPolicy { actuator })` to `UserError::NoPolicy { actuator }` (research R13). Split from T040, so the type-state part follows core K3.3, K3.4 without waiting on K5.3. FR-019 (SRS FR-4.5); plan C6, second part

**Checkpoint**: delivery steps 3 and 4, API side.

---

## Phase 6: Zenoh gateway (Priority: P2), plan phase Z, delivery step 5

**Goal**: a peer reads depth 0 every tick and sends Desired; the loop stays on the wired host;
the tick takes no heap and no lock from kmr_api.

**Independent Test**: quickstart.md § Phase Z.

- [ ] T042 [US4] Blocked on T029 (HUMAN). In kmr_api/Cargo.toml add the feature `zenoh = ["dep:zenoh", ..]` (plus the slot crate if T029 used one), off by default. In kmr_api/src/bus.rs, kmr_api/src/robot.rs and kmr_api/src/lib.rs add a sealed `Plugin` trait implemented by `Bus` (and by `Gateway` under the feature); `add_plugins` becomes generic over it (research R16). FR-043 (SRS FR-7.1); plan Z2, part 1
- [ ] T043 [US4] Blocked on T042. Create kmr_api/src/zenoh/mod.rs, gateway.rs, keys.rs, msg.rs: `Gateway::new(GatewayConfig { instance, identity, read_history })`, the Zenoh session on its own thread; keys `kmr/<instance>/history`, `kmr/<instance>/history/ring` (queryable), `kmr/<instance>/desired`, no per-group key (FR-048, SRS issue 6); `DesiredMsg { seq: u64, sent_ns: u64, q/qd/tau: [f32; N], written: [u64; 3] }` (one mask per field; only fields whose bit is set are applied); `SampleMsg { tick: u64, age: Option<u32>, q/qd/tau: [f32; N], faults }` (contracts/zenoh.md, data-model.md § Zenoh data). FR-043, FR-046, FR-048 (SRS FR-7.1, section 7 Zenoh); plan Z2, part 2
- [ ] T044 [US4] Blocked on T043. In kmr_api/src/zenoh/gateway.rs keep a ring of the last `HISTORY_DEPTH` `SampleMsg` the gateway took, each with its tick index so gaps show, served on `history/ring`; counters `missed_samples` and `superseded_desired` as atomics, readable outside the tick. FR-045 (SRS section 7 Zenoh, FR-13.4); plan Z2, part 3
- [ ] T045 [US4] Blocked on core K5.1 (HUMAN, `Stages<N>`, `run_on_staged`), T034, T035, T044. In kmr_api/src/zenoh/mod.rs implement the core's `Stages<N>` over the two `Latest` slots; `run()` uses `run_on_staged` when a `Gateway` is plugged. Inbound (after the read, before `First`): take the Desired slot once and write the masked fields through `State`. Outbound (after the write): put depth 0, its sample age and its flags into the History slot. Tick side: fixed-size copies only, no heap, lock, wait, I/O or crypto. FR-044 (SRS section 4 Constraints, FR-7.1), FR-046 (SRS section 7 Zenoh, FR-2.5); plan Z3
- [ ] T046 [US4] Blocked on T045. Create kmr_api/tests/zenoh.rs (feature `zenoh`), on a local peer, 10 000 ticks at 1 kHz: one sample per tick received or counted missed, the ring served on request (US4-AS1); a Desired taken in tick k applied in k, never in k+1 (US4-AS2, SC-010 (SRS FR-7.1)); no Desired, no command (US4-AS3); two Desired within one tick: the latest applied, one counted superseded; a local controller writing the same actuator later in the tick wins (spec Assumptions). Plan Z4
- [ ] T047 [P] [US4] Blocked on T042. Check the feature gate of kmr_api/Cargo.toml: `cargo tree -p kmr_api -e normal | grep -c zenoh` prints 0 and with `--features zenoh` more than 0; record both in the PR (US4-AS4, SC-004 (SRS FR-7.1)). Plan Z4
- [ ] T048 [US4] HUMAN (`unsafe impl GlobalAlloc`). Blocked on T046. In kmr_api/tests/zenoh.rs add a counting `#[global_allocator]` (per-thread counter, as T029) and assert 0 allocations on the tick thread from kmr_api over 10 000 ticks with the gateway on. SC-003's "0 locks" becomes `locks: review`: no tool counts locks (H4), so the human reads kmr_api's tick side (the `Stages` impl of T045, the `Latest` reader of T029) for `Mutex`, `RwLock`, `Once`, `try_lock` or any wait, and writes `locks: review` and the finding in the PR. The 1 000 000-tick run on the NFR-1 bench is bench plan D-3 (hand-off). FR-067 (SRS NFR-1), SC-003 (SRS NFR-1)

**Checkpoint**: delivery step 5.

---

## Phase 7: Fallbacks and envelopes (Priority: P3), plan phase E, delivery step 6

**Goal**: a registration carries a fallback; envelopes reach the core's pre-write stage.

**Independent Test**: quickstart.md § Phase E; SC-013 sweep on the recording adapter.

- [ ] T049 [US5] Blocked on T035. Create kmr_api/src/fallback.rs: `Guarded<F, G, P>`, a named struct whose call reads the tick's fault flags and calls exactly one of `F` (controller) and `G` (fallback); in kmr_api/src/robot.rs `add_controller_or(phase, f, when, fallback)` stores it in an `Inline<ApiInline<Guarded<..>>>` node; predicates are plain `Fn(&Faults) -> bool` values with helpers `stale_in(&GROUP)`, `remote_timeout(source)`, `any_of` (the per-source meaning of `remote_timeout` is open, U3). No `dyn`, no heap, no core dispatch change. No hold-policy method or argument exists (FR-050). FR-049, FR-050 (SRS FR-13.5, FR-13.3); plan E1
- [ ] T050 [US5] Blocked on core K5.2, K4.1 (HUMAN; no invariant file). Create kmr_api/src/envelope.rs: `Envelope { q: Option<(Q, Q)>, qd: Option<(Qd, Qd)>, tau: Option<(Tau, Tau)> }`, rule "lower ≤ upper, finite" (data-model.md; the plan names no error for a breach, U1); in kmr_api/src/robot.rs `set_envelope(i, Envelope)` and `set_envelope_group(&G, Envelope)` (every actuator of the slice gets it; an index past N is not specified, U2), stored as per-actuator arrays and handed to the core's `set_envelopes([Envelope; N])` at finalization. The breach action is the core's stage (assumed, pending human: D3, option A). FR-051 (SRS FR-13.5, FR-2.4), FR-052 (SRS FR-13.5, FR-13.4); plan E2
- [ ] T051 [US5] Blocked on T049, T050. In kmr_api/tests/run.rs on the recording adapter: a fallback on `stale_in(&A)`, read fails on group A: the fallback runs in place of the controller for that tick (US5-AS1); a sweep of out-of-envelope targets on actuator 0: the adapter sees only values clamped to the envelope, and actuator 0's `envelope` bit is set (SC-013 (SRS FR-13.5); D3 A, assumed, pending human); an envelope on a group slice reaches every actuator of the slice (US5-AS2). Quickstart § Phase E

**Checkpoint**: delivery step 6.

---

## Phase 8: Secure messaging (Priority: P3), plan phase F, delivery step 7

**Goal**: every network path is authenticated and encrypted; only allowed identities publish
Desired; drops are counted; a silent source triggers its fallback.

**Independent Test**: quickstart.md § Phase F.

- [ ] T052 [US6] Blocked on T043. Create kmr_api/src/zenoh/security.rs: `Identity::files(cert, key, ca)` holds paths only, never key content in source or descriptor (FR-060); mutual TLS or QUIC transport config; ACL default deny on every key, debug and visualization feeds included; `read_history` identities may read the History keys. FR-054, FR-060 (SRS FR-13.2, NFR-12); plan F1
- [ ] T053 [US6] Blocked on T052 and core K4.2 (HUMAN, `ExternalFlags`; no invariant file). In kmr_api/src/zenoh/security.rs, on the gateway thread, after the transport authenticated the peer: an identity that is not a remote source is dropped, `unauthorized += 1`, and the unauthorized bit is set through the core's hidden lock-free setter; `seq` at or below the last accepted is dropped, `out_of_order += 1`; `sent_ns` is carried and logged, not checked (deferred 2026-10-27). Authentication and decryption happen before a Desired reaches the slot; the tick does no crypto. FR-055, FR-056, FR-057 (SRS FR-13.2, FR-13.3, FR-13.4), FR-061 (SRS NFR-12); plan F2
- [ ] T054 [US6] Blocked on T049, T053. Create kmr_api/src/zenoh/watchdog.rs and in kmr_api/src/robot.rs add `add_remote_source(phase, Remote::allow(identity).timeout(d), fallback)`: the fallback is a required argument, inserted as a `Guarded` node (T049) with predicate `remote_timeout(source)`; `RemoteSource { identity, timeout, last_seq, last_seen }`; the watchdog runs on the gateway thread and sets the remote-timeout bit through core K4.2 on silence. A source without a fallback cannot be written. FR-058 (SRS FR-13.3, FR-13.5), FR-059 (SRS FR-13.3, FR-6.4); plan F3
- [ ] T055 [US6] Blocked on T054. In kmr_api/tests/zenoh.rs: 1000 Desired from an unknown identity: 0 applied, `unauthorized` = 1000, flag set (SC-011 (SRS FR-13.2)); a Desired with an old `seq` is dropped and counted (US6-AS1, SC-012 (SRS FR-13.3)); a source silent past its timeout raises the flag within timeout + 1 tick and its fallback runs (US6-AS2, SC-012). Add kmr_api/tests/ui/remote_no_fallback.rs: `add_remote_source` without a fallback does not compile (US6-AS3, FR-059). Quickstart § Phase F

**Checkpoint**: delivery step 7.

---

## Phase 9: FFI feature, plan phase G, delivery step 8

**Goal**: controllers that call foreign code still import `kmr_api` only (FR-072). No spec story
covers it; labelled US1 ("one import", U16).

- [ ] T056 [US1] Blocked on kmr_ffi plan A2 to A4 (A1 is HUMAN), B1 (the build helper that `ffi-build` re-exports; kmr_ffi T010) and C1 (HUMAN core items, with core K7.1 `FieldId`: **kmr_core/src/state/field.rs**). In kmr_api/Cargo.toml add features `ffi = ["dep:kmr_ffi"]` and `ffi-build`, off by default; create kmr_api/src/ffi.rs: `kmr_api::ffi` re-exports kmr_ffi; glue `ffi::call(f, time, state, ctx)` fills the view from the `State` and `Time` wrappers, calls, applies, and maps `Status` to `shutdown!` or `emergency_stop!` (assumed, pending human: D13, option A); `ffi-build` re-exports the build helper for a `build.rs` that only compiles foreign code (assumed, pending human: D12, option A); the N of `robot!` reaches the FFI types. Nothing in the builder (FR-070). kmr_ffi plan C2 describes the same item: one implementation, here (U12). FR-072 (SRS FR-10), FR-070 (SRS FR-10.1, FR-10.3); plan G1
- [ ] T057 [US1] Blocked on T056 and kmr_ffi plan D1, D2. In kmr_api/Cargo.toml add `ffi-python = ["ffi", "kmr_ffi/python"]` and `ffi-lua = ["ffi", "kmr_ffi/lua"]`; in kmr_api/src/ffi.rs re-export `kmr_api::ffi::python::Bridge` and `kmr_api::ffi::lua::Bridge` (kmr_ffi contracts/rust-surface.md). FR-072 (SRS FR-10.1, FR-10.3); plan G2
- [ ] T058 [US1] Blocked on T057. Check the feature gate of kmr_api/Cargo.toml: `cargo tree -p kmr_api -e normal | grep -c kmr_ffi` prints 0; `cargo test -p kmr_api --features ffi` is green; the glue tests are kmr_ffi plan C4 (hand-off). Quickstart § Phase G

**Checkpoint**: delivery step 8, API side.

---

## Phase 10: Polish and cross-cutting

- [ ] T059 [P] Blocked on core K6.3 (HUMAN) and kmr_adapters T067 (plan E10). In kmr_api/Cargo.toml add `presort-off = ["kmr_core/presort-off"]` (assumed, pending human: D11, option B); check that `kmr_api::presets::XmBench::READ_COST` and `WRITE_COST` are reachable through the existing `pub use kmr_adapters::presets`. Bench FR-016 (SRS FR-5.3), kmr_adapters FR-035 (SRS NFR-11, FR-5.3); plan C7
- [x] T060 [P] In kmr_api/src/lib.rs remove `#![allow(unused_variables)]` and `#![allow(dead_code)]` (003 review #9); fix or delete what surfaces (`FakeSensor`, `FakeModel`, unused fields). `cargo clippy -p kmr_api --all-targets` clean Done 2026-11-02: `FakeSensor`, `FakeModel` and the unused `use payload::Payload` deleted; three scoped `#[expect]`s with a reason, so each fires once its cause goes: `Sensors::inner` (the core's `Sensors` has nothing to forward yet), `State::group` (the maintainer's `todo!` on the invariance issue), and payload.rs as a whole (placeholder until T027). `cargo clippy -p kmr_api --all-targets`, with and without `dynamixel`: no kmr_api warning.
- [x] T061 [P] In kmr_api/src/*.rs docs say "actuator", never "joint", for an index in `0..N` (kmr_api/src/error.rs `StateError` says "A joint index" today). FR-037 (SRS section 3 Index space) Done 2026-11-02: the doc comments of state.rs, group.rs and error.rs say actuator; "joint" stays where it names a descriptor joint or the `joints-10` feature.
- [x] T062 [P] Add a decision record under docs/decisions/, the next free number in docs/decisions/ at commit time: the `Spec` type parameter that makes the adapter follow the robot (research R2), the `hardware = ..` argument (D7 A, assumed, pending human), the compile-time home marker (D5 A, assumed, pending human), the slot choice (R15). Constitution V Done 2026-11-02: docs/decisions/0005-api-robot-spec-hardware-home-slot.md, status proposed; it also records `bind()`/`Bound` (T039) and the several-brand `hardware = { .. }` form (T069), both pending human.
- [ ] T063 Hand-off, owner controllers plan C7 (HUMAN; walkthrough in controllers/README.md, plan C3): time to first motion against the ROS2 baseline (assumed, pending human: D14), SC-002 (SRS FR-10.4, NFR-4), FR-071 (SRS FR-10.4, NFR-4). No edit here
- [ ] T064 Run specs/004-kmr-api/quickstart.md § "Every change" and every phase section whose tasks are done; `cargo fmt --all --check`, `cargo clippy --workspace --all-targets`, `cargo test --workspace --no-fail-fast`
- [ ] T065 Update docs/c4/kmr_api.dsl with `/speckit-ov-c4-code` (constitution, Development Workflow)

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (T001)** first.
- **Foundational (T002)** after T001; blocks T007 and T011.
- **Phase 3 (T003 to T017)**, delivery step 1. T003 is unblocked. T004 waits on core K1.1.
  T005 on kmr_adapters T027. T007 on kmr_adapters T018, T023, T024, T030. T008 on controllers
  C0 (T006). T010 on core K1.6. T011 on core K1.6 to K1.8. T017 (HUMAN) closes the step.
- **Phase 4 (T018 to T033)**, delivery step 2: after T017 (D1 A, assumed, pending human).
  T021 waits on kmr_model T009; T023 on kmr_adapters T021, T025; T066 on core K2.5; T025 on
  T066, kmr_model T032, kmr_adapters T058; T026 on kmr_model T031, T032, T043, no core step;
  T029 is HUMAN; T032 on core K2.1 to K2.7 and kmr_adapters T057.
- **Phase 5 (T034 to T041, T067, T068)**, steps 3 and 4: after phase 4. Core K3.5, K4.1, K4.4,
  K3.8, K4.3, K5.2, K5.3, K3.2 to K3.4. T037 waits on core T065 (`test-util`). T040 needs T032
  (N on `Robot`) first (U14) and lands with core K3.3, K3.4. T067 waits on core K5.2; T068
  lands with core K5.3.
- **Phase 6 (T042 to T048)**, step 5: after phase 5. T045 waits on core K5.1.
- **Phase 7 (T049 to T051)**, step 6: after phase 6. T050 waits on core K5.2, K4.1.
- **Phase 8 (T052 to T055)**, step 7: after phase 7. T053 waits on core K4.2.
- **Phase 9 (T056 to T058)**, step 8: after phase 8 and kmr_ffi A, C1, D1, D2.
- **Polish**: T059 after core K6.3 and kmr_adapters T067; T060 to T062 any time after phase 3;
  T064, T065 last.

### Within a phase

- Phase 3: T002 → T007 → T008 → T009 → T011 → T012 → T013; T010 before T011; T014 after T011.
- Phase 4: T018 → T019, T020; T023 → T024; T066 → T025 → T026 → T027 → T028; T029 → T031;
  T032 → T033.
- Phase 5: T026 → T067 → T038 → T068; T040 → T068 → T041.
- Files shared across tasks: kmr_api/src/robot.rs (T008, T010 to T012, T027, T038 to T040,
  T042, T049, T050, T054, T068), kmr_api/src/robot_macro.rs (T009, T021, T023, T025, T026,
  T033), kmr_api/src/state.rs (T018, T020, T032, T034 to T036), kmr_api/src/lib.rs (T021,
  T066), kmr_api/src/spec.rs (T026, T067). Tasks on one file run in dependency order.

### Critical path to the first motor (coordination page § 3, step 9)

T002 → T005 (after kmr_adapters T027) → T007 (after kmr_adapters T018, T023, T024, T030) →
T008 (after controllers C0) → T009 → T010 (after core K1.6) → T011 (after core K1.7, K1.8) →
T012 → T013; T003 and T004 (after core K1.1) feed controllers C2 (step 10); T017 is step 11,
the one bench session. T014 (the example's bus line) is off this path.

## Parallel opportunities

- T003, T004, T005 (units.rs, time.rs, bus.rs) together, each once its blocker lands.
- T018 and T029 at the start of phase 4; T047 beside T043 to T046.
- T059 to T062 together.

```text
Phase 3:  T002 → T007 → T008 → T009 → (T010) → T011 → T012 → T013 → T017 (HUMAN)
          T003 ‖ T004 ‖ T005 (T005 before T007); T011 → T014 (off-path)
Phase 4:  T018 → T019 → T020 ; T021 ; T023 → T024 ; T066 → T025 → T026 → T027 → T028
          T029 (HUMAN) → T031 ; T032 → T033
```

## Implementation strategy

### MVP: delivery step 1 (D1 A, assumed, pending human)

1. T001, T002; T003 at once (controllers C2 needs it).
2. As kmr_adapters T018 to T030, controllers C0 and core K1.1, K1.6 to K1.8 land: T004 to T016.
3. HUMAN bench T017 with kmr_adapters T045 and controllers C6. A motor moves and stops.

### Incremental

4. Phase 4 (step 2), then phases 5 to 9 in SRS delivery order, each after the core steps it
   names.
5. Polish, then the PR from `agent/<topic>` into the human branch, `agentic` label.

## Notes

- Commits: AGENTS.md rule (b) and the human's final decision: no AI or co-author trailer. The
  constitution's Principle 0 asks for one. This conflict is the human's (research R22 #1).
- An agent that reaches a HUMAN task, or a task blocked on one, stops and hands it over.
- Commit per task or per group, Conventional Commits, scope `api`.

## Analysis 2026-10-28

`/speckit-analyze` over spec.md, plan.md and this file. Fixed here: U4 to U9, U13, U14, U18
(and the task ordering they imply). Left for the spec or plan owner (not edited):

| Id | Sev | Where | Finding |
| --- | --- | --- | --- |
| C1 | CRITICAL | constitution 0 vs AGENTS.md (b) | Agent trailer required vs forbidden. Known (research R22 #1); human |
| H1 | HIGH | spec FR-036 vs research R10 | Spec says run-time `unimplemented!` bodies; plan (D5 A) makes them compile-time (R24 #1) |
| H2 | HIGH | spec FR-065 vs research R11 | "Reached only at finalization" vs never reached in this version (R24 #2) |
| H3 | HIGH | constitution I, plan A4 | Step 1 is circular (D1); the uncommitted `robot!` code predates the gate (R21). Phase 4+ gated on T017 |
| H4 | HIGH | spec SC-003, quickstart Z | "0 locks" has no measuring method; only allocations are counted (T048) |
| U1 | MEDIUM | data-model `Payload`, `Envelope` | "Mass finite and > 0" and "lower ≤ upper, finite" name no error and no check point |
| U2 | MEDIUM | contracts/handles.md, builder.md | Index past N is unspecified for `stale_at`, `read_every`, `set_limit_policy`, `set_envelope` (FR-030 covers State setters only) |
| U3 | MEDIUM | research R17, R18 | `remote_timeout(source)` is per source; the core's remote-timeout flag is one bit per tick (K4.1) |
| U10 | MEDIUM | spec SC-007 | "Not driven yet exactly once until step 1" is moot: T011 removes the line in the change that wires the adapter (R24 #5) |
| U11 | MEDIUM | spec FR-006, FR-052 | Still `[NEEDS CLARIFICATION]` (Q1, Q3); plan assumes D7 A, D3 A |
| U12 | MEDIUM | kmr_api plan G1, kmr_ffi plan C2 | Same work in two plans; one owner needed |
| U15 | MEDIUM | plan § Source, 003 plan § Source | Both claim kmr_api/tests/robot_macro.rs and ui/; 003 has no step ids for hand-offs |
| U16 | LOW | spec user stories | FR-072 (FFI) and FR-041 (adapter access) have no story |
| U17 | LOW | spec FR-038 vs contracts/handles.md | "As durations" vs `last_overrun() -> Option<Duration>` |
| U19 | LOW | bench R7 vs kmr_adapters T059 | Counting allocator `unsafe`: HUMAN in one plan, agent in the other; here routed to T030 |

Fixed in this file: U5 SC-008 compile-fail tests had no plan row (T019). U4 State and Sensors
keep `.0` with no plan row (T018). U6 B3 misses core K2.5 (`MASK_WORDS`) and its kmr_api
re-export (T025). U7 `Robot<H, C>` sits in A5 though A4's alias needs it (T008). U8 A6's error
variants come after A5 though A4 returns them (T002). U9 A4 covers the preset form only; the
model form must emit a `__Spec` in the same change (T009). U13 the N ≤ 64 assert cannot fire
while the `N == JOINTS` assert stands (UI case in T033). U14 C6 needs B7 first (T040).
U18 the example has four home-state calls, not two (T020).

Coverage: 72 FRs, 13 SCs. Every FR with buildable work has a task, or exists today (FR-009 to
FR-012, FR-017, FR-029, FR-030, FR-032, FR-034, FR-040, FR-053), or is a constraint with no
code (FR-008, FR-013, FR-047, FR-066, FR-068, FR-070). SC-002 is controllers C7's (T063).

## Analysis 2026-10-28 (cross-module)

- 2026-10-28: new T066 re-exports `MASK_WORDS` after core K2.5 (out of T025; breaks the T025 → kmr_adapters T058 → kmr_model T032 → T025 cycle); T017 takes the one bench blocker list, drops T014 (off-path); T019 owns the ui harness and `trybuild` dev-dependency (kmr_model T018 hands off); T021 names the `__model!` switch kmr_model T010 assumed; T024 owns the count ui case (kmr_model T027 hands off); T026 keeps the consts, its `[Limit; N]` part is new T067 (phase 5), so payloads no longer wait on core K5.2; T029, T048 HUMAN (`unsafe`; resolves U19); T033 hands the `three.urdf` pass case to kmr_model T048 and SC-009's robots to kmr_model T062; T037 waits on core T065 (`test-util`); T038 notes XmBench limits wait on kmr_model T029; T040 split: type-state part lands with core K3.3, K3.4, `NoPolicy` part is new T068 (lands with K5.3); T048 `locks: review` (H4); T056 adds kmr_ffi B1; T062 drops fixed record numbers.

## Added 2026-10-29 (D17, set of drivers)

- [ ] T069 [2026-10-29] Agent: blocked on core T069 (HUMAN) and kmr_adapters T069. In kmr_api/src/spec.rs replace `Spec`'s one adapter type (T007) with `type Drivers`, the HList of `Hal` members, and make `bind` return it; in kmr_api/src/robot_macro.rs emit it from the hardware table, one member per brand, `hlist![NoHardware]` without hardware (T009, T023); print the list's nested error into `UserError::BringUp`; `Bound` (T039) hands the list back. Fix the multi-brand `hardware = ..` syntax with the human first. Per specs/002-kmr-core/design-hal-set.md §6, §7. Done 2026-11-02 on agent/driver-set: `Spec` has `type Drivers: Drivers<Error: Send + Sync>` and `bind` returns the list; `run()` without a plugin runs on `HCons { NoHardware, HNil }`; the `one(adapter)` stopgap and `__bind_dynamixel` are gone. kmr_api/src/brand.rs: hidden `Brand` (one brand's part: its labels, its bind) and `Brands` (sealed, `HNil`/`HCons`), `__bind` routes plugins by bus label (`SharedBusLabel`, `UnknownBus`, `LabelRequired`, new `UserError` variants) and names a member's binding error `member k (<type>): ..` through `HardwareError::Member`, like a bring-up error. `Placement<NB, M>` is the Dynamixel `Brand`. `robot!(Preset)` keeps its form (`HCons<__DxlAdapter<__NB>, HNil>`); `robot!(model = ..)` gives `HCons<NoHardware, HNil>`; new `robot!(model = .., hardware = { dynamixel: TABLE.at([..]) })`, one member per brand, syntax assumed, pending human (contracts/robot-hardware.md § Several brands). `Bound` (T039) does not exist yet: nothing to hand back. Tests: kmr_api/tests/drivers.rs (two fake brands: own indices only, torque_off then close on each, a gap and an overlap before any open, the error chain with position, type name and typed data; routing errors), kmr_api/tests/spec.rs (the hardware arm, feature on and off), a `compile_fail,E0428` doctest for a brand named twice. Review fixes 2026-11-02: the expansion checks in const evaluation that the parts cover `0..N` exactly once (`__check_coverage`, E0080 naming the index; the preset arm too), and `kmr_api::dynamixel` re-exports the table types so a controller can write its own table. Stays open: the human has not confirmed the `hardware = { brand: part }` syntax nor the `kmr_api::dynamixel` path; the `Bound` hand-back moves to T039, which does not exist yet. Review fixes 2026-11-02 (2): a binding error is named through the core's `MemberError::head::<M>`, not built by hand. kmr_api/tests/spec.rs claimed the same with `.at([3, 2, 1, 0])` as with `in_order()` (one bus, one model); its table now has an arm bus and a slower wrist bus, so the groups and cadence of the claim show where the placement put each actuator (checked: `in_order()` fails it). The hardware arm evaluated the part in a module whose own `SHAPE`, `Part` and `PART` shadowed the user's items of those names (a user `PART` was a cycle error); they are `__SHAPE`, `__Part`, `__PART` now, tested with user items of the three names. The `@parts`/`@list` recursion was never expanded over two brands (`__brand` has only `dynamixel`): two test-only fake brands (`__brand::fake_arm`, `fake_leg`, `cfg(test)`, kmr_api/src/spec/fake.rs) let kmr_api/src/robot_macro/test.rs build a two-brand `robot!` and check its `Drivers` type, the member order, the routing of plugins by label, each member's claim, and `UnknownBus`. The run loop over two members stays covered by kmr_api/tests/drivers.rs: a unit test running it would share the signal bus with robot.rs's tests. The `Bound` hand-back landed with T039 (2026-11-02): `Bound::run` hands the whole list back.
