---
description: "Tasks for controllers, the reference example, generated 2026-10-28"
---

# Tasks: controllers, the reference example

**Input**: `specs/005-controllers/`: [plan.md](plan.md) (2026-10-28), [spec.md](spec.md)
(re-specified 2026-10-28), [research.md](research.md), [data-model.md](data-model.md),
[quickstart.md](quickstart.md), [contracts/](contracts/). Authority: `docs/srs/SRS.md`,
`## Module: controllers`. Coordination: `specs/spec-coordination-2026-10-28.md` (critical path
steps 10 and 11).

**Tests**: no test code. `main.rs` holds no test module (research R8), and a binary's functions
are not reachable from `tests/`. Each story is checked by a quickstart.md run, V1 to V7. Only the
HUMAN tasks need XmBench.

**HUMAN**: work in `kmr_core/` (`.aiignore`) and in `../dxl_rs` is the human's. So are the bench
runs, the onboarding trial and the CI workflow edits (plan C8). An agent never does them. No task
here touches an invariant file. A task that waits on one names it in **bold**.

**Hand-off**: this plan edits `controllers/**`, plus the docs of Phase 8. A task that needs work in
another module says "Hand-off", names the owning plan step or task id, and keeps its id for order
only. It edits nothing here.

**Blocked**: an agent task that waits on another task says "Blocked on" and names it: `Tnnn` here,
or `<module> <id>` elsewhere.

**Assumed, pending human**: D1 (A: the example is part of delivery step 1), D2 (B: `run()` refuses
without a limit policy; plan C9), D6 (no bus label on a one-bus robot), D14 (ROS2 baseline), D15
(A: no bus line ships). Also core open choice 6 (origin of `elapsed()`). Each is isolated to the
place its task names; another answer changes that place only.

**Ids**: `FR-0NN` / `SC-0NN` are this spec's, written with their SRS source: `FR-019 (SRS FR-2.6)`.
User stories are spec.md's: US1 first moving controller (P1), US2 the reference stays honest (P1),
US3 measure onboarding against ROS2 (P2), US4 builds anywhere, simulates or fails cleanly (P2).
Plan steps C0 to C9 are plan.md § Work outline. Other modules: kmr_api plan steps (`A0`, `C4`),
core plan steps (`K1.1`; `#14` is the design-first-motor number), kmr_adapters tasks (`T042` in
`specs/001-kmr-adapters/tasks.md`).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an unfinished task)
- **[Story]**: US1..US4 from spec.md

Paths are from the repository root.

---

## Phase 1: Setup (plan C0, C1; critical path step 10, unblocked)

**Purpose**: move `main.rs` to `robot!(XmBench)` before kmr_api A4 makes a bare `Robot::new()`
stop compiling (kmr_api research R5), and give the crate its hardware switch.

- [x] T001 Plan C0, first and unblocked. In controllers/src/main.rs add `kmr_api::robot!(XmBench);` at module level and drop `Robot` and `State` from the `use kmr_api::{..}` line; the emitted aliases replace them. Nothing else changes: `nudge_first_joint` stays until T009. Lands before kmr_api A4 (`Robot<H>`). The `N == JOINTS` assert of `robot!` holds for XmBench, N = 4 (FR-003 (SRS FR-6.1))
- [x] T002 [P] Plan C1. In controllers/Cargo.toml add `[features] hardware = ["kmr_api/dynamixel"]`, off by default; `kmr_api` stays the only dependency (FR-002 (SRS NFR-3), FR-025 (SRS FR-5.5, NFR-5); research R4)
- [x] T003 After T001, T002. From the repo root run `cargo build --workspace`, `cargo clippy --workspace --all-targets`, `cargo fmt --all --check`: green, 0 warnings from `controllers`. `cargo tree -p controllers | grep -c dxl_rs` prints 0; with `--features hardware` it prints 1 or more (SC-007, SC-008). Say in the PR that kmr_api A4 may now land

**Checkpoint**: `main.rs` names the robot once; the workspace is green; the default build pulls
no `dxl_rs`.

---

## Phase 2: Foundational: the kmr_api surface the demo calls (hand-offs)

**Purpose**: the demo needs `Time::elapsed()` and `Deg` ([contracts/api-needs.md](contracts/api-needs.md),
rows marked missing). Neither exists today. Blocks the code of every story.

- [x] T004 Hand-off, owner kmr_api plan A1; that step waits on core K1.1 (#14, HUMAN, critical path step 8: kmr_core/src/clock.rs, kmr_core/src/runtime.rs, no invariant file). `kmr_api::Time` forwards `elapsed()` and `dt()` as `Duration`, and `tick()` (kmr_api FR-038 (SRS FR-3.3)). No edit here. Blocks T009. Landed 2026-11-02: kmr_api T004, `Time::elapsed()`, `dt()` as `Duration`, `tick()` as `u64` in kmr_api/src/time.rs
- [x] T005 Hand-off, owner kmr_api plan A2, unblocked there. `kmr_api::Deg(pub f32)` and `From<Deg> for Q`, degrees to rad (kmr_api FR-033 (SRS FR-2.2)). No edit here. Blocks T006, T008. Landed 2026-11-02: kmr_api T003, `Deg(pub f32)` and `From<Deg> for Q` in kmr_api/src/units.rs

**Checkpoint**: both missing rows of contracts/api-needs.md exist in `kmr_api`. US1 code can start.

---

## Phase 3: User Story 1, first moving controller (Priority: P1), plan C2 and C6, critical path steps 10 and 11 (MVP)

**Goal**: one short file swings every actuator ±5° around where it stood, then ends with
`shutdown!` after 12 s. FR-003, FR-005, FR-013 to FR-019, FR-021 to FR-024.

**Independent Test**: quickstart.md V2 (simulation, exit 0 after about 12 s), then V5 and V6 on
XmBench (HUMAN).

T006 to T011 edit one file. Land them as one commit: states in between warn `dead_code`.

- [ ] T006 [US1] Blocked on T005 (kmr_api A2). In controllers/src/main.rs add the constants of data-model.md § Constants: `AMPLITUDE: Deg = Deg(5.0)`, "half the swing; revolute only"; `PERIOD: Duration` = 4 s, "one full swing"; `DEMO_LENGTH: Duration` = 12 s, "compared with `Time::elapsed()`". `DEMO_LENGTH` is the one place to change if core open choice 6 moves the origin of `elapsed()` (research R5) (FR-022 (SRS FR-4.5 open, FR-13.1), FR-023 (SRS FR-4.1))
- [ ] T007 [US1] In controllers/src/main.rs add the context `#[derive(Default)] struct Swing { start: Option<([Q; N], Duration)> }`: the start position and the `elapsed()` at which it was read, "`None` until the first sample". "Size is fixed at compile time, about 32 bytes for N = 4. No heap." Built in `main`, moved in at registration, never shared, never locked (FR-013 (SRS FR-1.1, FR-1.2), FR-014 (SRS Definitions "Context"))
- [ ] T008 [US1] Blocked on T005. In controllers/src/main.rs add `fn offset(s: Duration) -> Q` = `Q::from(Deg(AMPLITUDE.0 · sin(2π · s / PERIOD)))`, `f32` only. From data-model.md § Target function: "`|offset| ≤ 5°` = 0.0873 rad for every `s`"; at `s = 0` "exactly `Q(0.0)`: the first command equals the start"; the input is "time since the start was read, never a call count" (FR-016 (SRS FR-1.5, FR-3.3), FR-017 (SRS FR-2.2), FR-022 (SRS FR-4.5 open, FR-13.1))
- [ ] T009 [US1] Blocked on T004 (kmr_api A1). In controllers/src/main.rs replace `nudge_first_joint` with `fn swing(ctx: &mut Swing, time: &Time, robot: &mut State, _: &Sensors)`, in the order of data-model.md § Controller: (1) `start` is `None`: call `robot.prev_q(0)`; on `Ok(q)` store `(q, time.elapsed())`; on `Err` store nothing and write nothing (FR-019 (SRS FR-2.1, FR-2.3, FR-2.6)); (2) `start` is `Some((q0, t0))`: `robot.set_all_q(q0.map(|q| q + offset(time.elapsed().saturating_sub(t0))))` on every tick, never assumed to persist (FR-018 (SRS FR-2.5, FR-3.4, FR-5.2)); (3) then, if `time.elapsed() >= DEMO_LENGTH`, `shutdown!("reference demo: time is up")` (FR-023 (SRS FR-4.1, FR-4.4)). Write before raise, so the shutdown tick still writes (research R5). No index, no `unwrap`, `expect`, `unwrap_or*` or `panic!`; no allocation, lock, sleep or I/O (FR-009 (SRS NFR-1), FR-011 (SRS NFR-2))
- [ ] T010 [US1] In controllers/src/main.rs make `main` one chained expression, `Robot::new().set_dt(Duration::from_millis(1)).add_controller_with(EachTick, Swing::default(), swing).run()`, returned from `fn main() -> Result<(), UserError>`. No `build()`. No bus line (assumed, pending human: D15 A; this chain is the one place). No hold duration (default `None`). No signal handling of its own. No phase but `EachTick`. None of `add_controller_as_thread`, `initial_state`, `go_home`, `register_payload`, groups or `Bus::by_dev`; with no group, FR-020 (SRS FR-2.4, FR-6.2) holds by absence. The file is the reference a new user copies (FR-001 (SRS module controllers; SRS § 8), FR-005 (SRS FR-10.1), FR-006, FR-007 (SRS FR-6.3), FR-012 (SRS FR-1.2, FR-2.7, FR-8.4), FR-015 (SRS FR-1.4), FR-021 (SRS FR-3.1), FR-024 (SRS FR-4.3))
- [ ] T011 [US1] In controllers/src/main.rs write the comments: a module doc (what the file moves, how it ends, that it simulates as shipped and controllers/README.md adds the bus line) and one short doc line per item. A body comment only where the code is really ambiguous. Every comment true against the SRS and today's `kmr_api`; none names an unbuilt feature; none restates the code (FR-010 (SRS NFR-6); CONTRIBUTING.md rule 8)
- [ ] T012 [US1] After T011. Run the quickstart.md V1 build part (`cargo build --workspace`, `cargo clippy --workspace --all-targets`, `cargo fmt --all --check`; 0 warnings from `controllers`; FR-008 (SRS module controllers, NFR-2)), then V2 once: exit 0 after about 12 s, the last line is the shutdown with reason "reference demo: time is up", no panic. History may be zeros or "no sample" (research R11). The warning count is T024's. The research R10 prototype is gone from the scratchpad; this run re-proves its build and run rows on the real crate
- [x] T013 [US1] Hand-off, owner kmr_api plan A5 (critical path step 9), with A3 and A4; kmr_adapters T042 points at the same step. `run()` binds the Dynamixel adapter to the `Bus::by_id` serial and calls core `run_on`. kmr_api A5 waits on core K1.6 to K1.8 (HUMAN) and on kmr_adapters T024, T030. No edit here. Blocks T014, T015, T026. Landed 2026-11-02: kmr_api T011, `run()` binds through `Spec` and calls `run_on` (kmr_api/src/robot.rs)
- [ ] T014 [US1] HUMAN, plan C6, quickstart.md V5 (critical path step 11), one bench session with core T018, kmr_api T017 (A9) and kmr_adapters T045. Blocked on core T004–T013 (K1.1 to K1.10; K1.3 touches **kmr_core/src/state/field.rs**, invariant), kmr_adapters T017 (its T002, HUMAN dxl_rs `force_torque_off`), T041, kmr_api T011 (A5, the T013 hand-off) and T012 here. Add locally, not committed (D15 A): `.add_plugins(Bus::by_id("<U2D2 serial>"))`, and a logger for SC-009 and SC-010, a `Last` controller registered with `add_controller_with` whose fixed-size context keeps, per actuator, the first `prev_q(0)` it reads as the start and the largest |`prev_q(0)` − start| seen since (the History recording), and the `tick()` and `elapsed()` of the last tick; it prints them once, to stderr, in the tick where `elapsed() >= DEMO_LENGTH` (the shutdown tick still runs `Last`). Run `cargo run -p controllers --features hardware --release` 10 times: every actuator swings ±5° around where it stood, period 4 s; no jump at the first command (research R13); the logged largest deviation is within 5° in 10 of 10 runs (SC-009 (SRS FR-4.5 open, FR-13.1)); the logged `elapsed()` of the end tick is within one `dt` of 12 s (SC-010 (SRS FR-4.1)); afterwards `Torque_Enable` = 0 on ids 1 to 4. Once more with actuator 4 unplugged: a readable bring-up error, non-zero exit, no panic (spec Edge Cases; SRS FR-5.5, FR-6.4)
- [ ] T015 [US1] HUMAN, plan C6, quickstart.md V6. Blocked on T014. Run T014's command and press Ctrl-C mid-swing; again with `kill -TERM` and `kill -HUP` on the process. Each time: shutdown, hold, torque off, exit 0 (FR-024, SC-010 (SRS FR-4.3)). SIGTERM and SIGHUP need core K1.2 (HUMAN). Then once with a local line in `swing`, after the write, not committed: `if time.elapsed() >= Duration::from_secs(6) { kmr_api::emergency_stop!("bench: estop check") }`; the loop stops at once, no hold, `Torque_Enable` = 0 on ids 1 to 4, exit 0 (`run()` returns `Ok` for both signals). This line is what raises the emergency stop of core T018 and kmr_adapters T045

**Checkpoint**: delivery step 1 (D1 A): a motor moves on XmBench and stops with torque off.

---

## Phase 4: User Story 2, the reference stays honest (Priority: P1), plan C4 (V1)

**Goal**: the crate builds clean, holds no marker or panic path, takes no lock in a tick and says
nothing false. FR-002, FR-004, FR-008 to FR-012, FR-029, FR-030.

**Independent Test**: quickstart.md V1 plus the hand review.

- [ ] T016 [P] [US2] After T010; the layout check after T019. Run the quickstart.md V1 searches: `grep -rnE 'todo!|unimplemented!|compile_error!|unwrap|expect\(|panic!' controllers/src` and `grep -rnE 'kmr_core|kmr_model|kmr_adapters|dxl' controllers/src controllers/Cargo.toml` print nothing (FR-008 (SRS module controllers, NFR-2), SC-003, SC-007); `grep -rnE 'add_controller_as_thread|initial_state|go_home|register_payload|by_dev' controllers/src` prints nothing (FR-012 (SRS FR-1.2, FR-2.7, FR-8.4)); `command ls controllers controllers/src` shows `Cargo.toml`, `README.md`, `src/main.rs` only, no `build.rs`, no config file (FR-004 (SRS FR-6.1)); the manifest names one kmr crate, `kmr_api` (FR-002 (SRS NFR-3)). Fix a hit in controllers/ and rerun
- [ ] T017 [P] [US2] After T011. Review controllers/src/main.rs by hand and write the result in the PR: `swing` takes no lock, heap type, sleep, blocking call or I/O (SC-005 (SRS NFR-1)); no unchecked index (FR-011 (SRS NFR-2)); no FFI, middleware or payload (FR-029 (SRS FR-10.3, FR-7.3, FR-8.1)); the context is plain and heap-free (FR-030 (SRS NFR-10)); every comment true against the SRS and `kmr_api` (SC-006 (SRS NFR-6)). The human confirms both at review
- [ ] T018 [US2] HUMAN, outside this module. The workspace job of .github/workflows/ci.yml runs clippy without `-D warnings`, so a warning in `controllers` passes CI. Add `cargo clippy -p controllers --all-targets -- -D warnings` (or deny warnings for the workspace), so SC-004 holds "on every change" (SRS module controllers "compile clean"). Same file as T027 and kmr_adapters T053: one edit after the other

**Checkpoint**: V1 is clean and CI keeps it clean.

---

## Phase 5: User Story 3, measure onboarding against ROS2 (Priority: P2), plan C3, C7

**Goal**: a walkthrough lists every file, line and command from a fresh checkout to a moving
motor, with a ROS2 baseline counted the same way. FR-027, FR-028 (SRS FR-10.4, NFR-4).

**Independent Test**: a reader follows controllers/README.md on a clean machine and records time,
files and lines (V7).

- [ ] T019 [P] [US3] After T010. Create controllers/README.md, the walkthrough of data-model.md § Walkthrough. Prerequisites: toolchain, a fresh checkout, XmBench wired and powered, the adapter's serial. Files written: "none for XmBench; `Cargo.toml` and `main.rs` for a copied crate". Lines edited: the bus line `.add_plugins(Bus::by_id("<serial>"))`, added before `.add_controller_with`, with no `.bus("label")` (assumed, pending human: D6); the preset name, only for another robot. Commands: `cargo run -p controllers` simulates; `cargo run -p controllers --features hardware --release` drives the bench. Numbers: preset XmBench, ±5°, period 4 s, 12 s of `elapsed()`, torque off right after the hold. Warnings: revolute only (`Q` is mm on a prismatic actuator); position mode only (other actuators get no command); a robot that would fall needs a hold duration; with a foreign Ctrl-C handler, Ctrl-C may not stop the loop, and the demo still ends (FR-027 (SRS FR-10.4, NFR-4))
- [ ] T020 [US3] In controllers/README.md add the "Baseline" section (assumed, pending human: D14; research R7; the one place): a minimal node made with `ros2 pkg create`, C++ and Python, on the current ROS2 LTS, driving the same four actuators through the vendor's ROS2 SDK package; "only files the user writes or edits count, for both sides"; kmr must beat both on time and file count. Add the trial record table of data-model.md § Trial record: side (kmr, ROS2 C++, ROS2 Python), robot, elapsed (minutes), files written, lines edited, commands run (FR-028 (SRS FR-10.4)). Hand-off: kmr_ffi plan E4 (HUMAN) runs its FR-10.4 trial on this protocol
- [ ] T021 [US3] HUMAN, plan C7, quickstart.md V7. Blocked on T019, T020, T014. A control engineer new to kmr follows controllers/README.md from a fresh checkout to the first motion, then builds the ROS2 baseline node. One trial row per side. Expected: at most 2 files written and 2 lines edited; kmr beats both baselines on time and on file count (SC-001, SC-002 (SRS FR-10.4, NFR-4))

**Checkpoint**: the FR-10.4 metric has a written, repeatable protocol and one measured result.

---

## Phase 6: User Story 4, builds anywhere, simulates or fails cleanly (Priority: P2), plan C4 (V2, V3), C5, C8

**Goal**: cross-builds without hardware; simulation with exactly one warning; a readable bring-up
error without the robot. FR-007, FR-025, FR-026.

**Independent Test**: quickstart.md V2, V3, V4.

- [ ] T022 [P] [US4] After T002, T010. Run quickstart.md V3: `cargo build -p controllers --target x86_64-unknown-linux-gnu`; aarch64 with `CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc`; armv7 with `CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=arm-linux-gnueabihf-gcc`; `cargo tree -p controllers | grep -c dxl_rs` prints 0. Where a cross linker is missing, run `cargo check --target ..` and report that only the compile half is proved (SC-008 (SRS FR-5.5, NFR-5)). Pre-run 2026-11-02 on the pre-T010 `main.rs`: x86_64 build ok; aarch64 build ok (ELF aarch64); armv7 `cargo check` ok, `arm-linux-gnueabihf-gcc` missing, compile half only; `dxl_rs` count 0, 2 with `--features hardware`. Rerun after T010
- [x] T023 [US4] Hand-off, owner kmr_api plan A0, which waits on core K1.6 (HUMAN). The core's `NoHardware` prints "no bus plugin: simulating" once, at `open`, and kmr_api drops its own `eprintln!` (kmr_core FR-042, kmr_api FR-026 (SRS FR-6.3); research R11 found 2 warnings). No edit here: `main.rs` prints nothing of its own (FR-007). Blocks T024. Landed 2026-11-02: kmr_api T010; the core's `warn!` in kmr_core/src/hal.rs, no `eprintln!` left in kmr_api/src
- [ ] T024 [US4] Blocked on T023 and T012. Run quickstart.md V2 10 times: exactly 1 warning, "no bus plugin: simulating"; exit 0 after about 12 s; no panic (SC-011 (SRS FR-6.3); contracts/run-behaviour.md row 1)
- [x] T025 [US4] Hand-off, owner kmr_api plan A6: the `Debug` of `UserError` prints its `Display` message, so `main` returning `run()` prints text (kmr_api FR-021 (SRS FR-6.4, NFR-2); research R12). No edit here: `main.rs` keeps returning `run()` (FR-005). Blocks T026. Landed 2026-11-02 in another shape (kmr_api, 2026-10-29): `Debug` prints `Display` in release builds only, debug builds derive it; quickstart V4 and contracts/run-behaviour.md now run `--release`
- [x] T026 [US4] Plan C5, quickstart.md V4. Blocked on T013 (kmr_api A5) and T025 (kmr_api A6). Add the walkthrough bus line locally, robot unplugged, not committed. `cargo run -p controllers --release`: a `run()` error naming the adapter feature, `kmr_api/dynamixel`, which `hardware` turns on (row 2; kmr_adapters FR-031; kmr_api cannot name a feature of the user's crate, corrected 2026-11-02 with T025). `cargo run -p controllers --features hardware --release`: a readable bring-up error naming the port (row 3). 10 runs each: non-zero exit, no panic, text and not `Debug` (FR-026, SC-012 (SRS FR-6.4)). Row 3 needs `../dxl_rs` and libclang on the machine. Revert the line after. Done 2026-11-02 on a scratch copy of the committed `main.rs` (controllers/examples/, deleted after; `main.rs` is the maintainer's), bus line `Bus::by_id("KMRNOSUCHADAPTER")`, a serial no `/dev/serial/by-id` entry holds, so no port was opened. 10 runs each, exit 1, no panic. Row 2, release: "bus plugins are declared, but kmr_api was built without its `dynamixel` feature; enable `kmr_api/dynamixel`, or remove the plugins to simulate"; it names the kmr_api feature, which `hardware` turns on. Row 3, release: "hardware bring-up failed: member 0 (kmr_adapters::dynamixel::Dynamixel<..DxlRs, 1>): bus bus0: serial KMRNOSUCHADAPTER names no single device; check the bus plugin and the cable". Debug builds print the derived `Debug`: `FeatureOff { feature: "dynamixel" }`, `BringUp { source: Member(Head { .., error: PortNotFound { bus: "bus0", port: ById("KMRNOSUCHADAPTER") } }) }`
- [ ] T027 [US4] HUMAN, plan C8, outside this module. Blocked on T024; the cross part also on kmr_adapters T053 (cross linkers in CI). In .github/workflows/ci.yml: a smoke step builds `controllers`, runs the binary under a timeout (60 s), expects exit 0 and exactly one "no bus plugin: simulating" line (SC-011); the cross job adds `-p controllers`, feature off, with the cross linkers (SC-008 on every change). Same file as T018

**Checkpoint**: the shipped file runs anywhere, and fails cleanly where it cannot.

---

## Phase 7: Later kmr_api changes (off the critical path), plan C9

**Purpose**: follow the kmr_api changes that land after the first motor.

- [ ] T028 [US1] Plan C9. Blocked on kmr_api C4, which waits on core K5.2 and K5.3 (HUMAN). Assumed, pending human: D2 B. In controllers/src/main.rs add `.set_limit_policy_all(Policy::Clamp)` after `.set_dt(..)` and before `.add_controller_with` (data-model.md § Builder chain), and import `Policy`. In controllers/README.md list it as a shipped line, not one the user edits. The swing never reaches a limit (research R3); the line is there because `run()` refuses without it under D2 B. D2 A drops this task; D2 C makes the line optional. Rerun T012 and T016 (FR-022 (SRS FR-4.5 open, FR-6.4)). If core K5.3 lands before T014, do this task before T014
- [ ] T029 [US2] After kmr_api A4 (`Robot<H>`, `__Spec`), and again after kmr_api B7 (`State<'a, N>`; waits on the HUMAN core const-generic steps 1 to 7, invariant files **kmr_core/src/state/field.rs, kmr_core/src/controllers.rs, kmr_core/src/schedule.rs, kmr_core/src/schedule/macros.rs**): rerun T003's build and lint. `main.rs` builds with no edit (research R9; contracts/api-needs.md). If it needs one, report it to kmr_api; do not work around it here. A4 half done 2026-11-02: `cargo build --workspace`, `cargo clippy --workspace --all-targets` (0 warnings from `controllers`), `cargo fmt --all --check` green, `main.rs` unedited. B7 half open

---

## Phase 8: Polish and cross-cutting

- [x] T030 [P] Add docs/decisions/NNNN-reference-example.md, NNNN the next free number in docs/decisions/ at commit time (0001 and 0002 exist). Defend, in the constitution V order (real-time, quality, developer experience): the `hardware` feature off by default (research R4), the swing shape and numbers (R3), no bus line as shipped (R1; assumed, pending human: D15 A), no test module in `main.rs` (R8) (constitution V)
- [ ] T031 [P] After T010. The root README.md shows the old `nudge_first_joint` example, with `go_home` and without `robot!`; docs/ROADMAP.md says `controllers/src/main.rs` runs "until Ctrl-C". Update both to the new reference, or point README.md at controllers/README.md (FR-010 (SRS NFR-6)). Outside `controllers/**`: the human accepts it in the PR
- [ ] T032 Update docs/c4/controllers.dsl with `/speckit-ov-c4-code` (constitution, Development Workflow)
- [ ] T033 Run quickstart.md V1 to V3 in full and `cargo test --workspace --no-fail-fast`; all green (constitution, Technical Constraints)

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (T001–T003)**: now. T001 first. Nothing blocks it.
- **Foundational (T004, T005)**: hand-offs. T005 (kmr_api A2) can land now. T004 waits on core
  K1.1 (HUMAN) then kmr_api A1.
- **US1 (T006–T015)**: T006, T008 after T005; T009 after T004; T010, T011 after T009; T012 after
  T011. T013 is kmr_api A5. T014, T015: one bench session with core T018, kmr_api T017,
  kmr_adapters T045; blocked on core T004–T013, kmr_adapters T017, T041, kmr_api T011 (T013) and
  T012. T015 after T014.
- **US2 (T016–T018)**: after T010 and T011; the layout check of T016 after T019. T018 any time.
- **US3 (T019–T021)**: T019, T020 after T010. T021 after T019, T020, T014.
- **US4 (T022–T027)**: T022 after T002, T010. T024 after T023 (kmr_api A0) and T012. T026 after
  T013 (kmr_api A5) and T025 (kmr_api A6). T027 after T024; its cross part after kmr_adapters T053.
- **Phase 7**: T028 after kmr_api C4 (core K5.2, K5.3). T029 after kmr_api A4, then after B7.
- **Polish**: T030 any time. T031 after T010. T032, T033 last.

### Hand-offs

| Direction | Other module's id | Here |
|---|---|---|
| kmr_api waits on this | A4 (`Robot<H>`) waits on plan C0 (kmr_api research R5) | T001 |
| kmr_adapters points here | T044 (controllers C0, C2) | T001, T006–T012 |
| Same bench session | core T018, kmr_api A9 (T017), kmr_adapters T045; T015's `emergency_stop!` line raises their estop | T014, T015 |
| kmr_ffi waits on this | E4 (HUMAN FR-10.4 trial, D14) | T020 |
| bench builds this | A-6 `compile-time.sh` builds `controllers` as is (D15 A keeps it hardware-free) | T001–T012 |
| This waits on kmr_api | A1 (after core K1.1, HUMAN) | T004 → T009 |
| This waits on kmr_api | A2 | T005 → T006, T008 |
| This waits on kmr_api | A5, with A3, A4 (after core K1.6–K1.8, HUMAN; kmr_adapters T024, T030, T042) | T013 → T014, T015, T026 |
| This waits on kmr_api | A0 (after core K1.6, HUMAN) | T023 → T024 |
| This waits on kmr_api | A6 | T025 → T026 |
| This waits on kmr_api | C4 (after core K5.2, K5.3, HUMAN) | T028 |
| This waits on kmr_api | A4, B7 (B7 after core const-generic steps 1 to 7, HUMAN) | T029 |
| This waits on kmr_adapters | T017 (its T002, dxl_rs `force_torque_off`), T041; T053 (CI cross linkers, HUMAN) | T014; T027 |

### Critical path to the first motor

T001 (now; unblocks kmr_api A4) → kmr_api A2 (T005) → T006, T008; core K1.1 (HUMAN) → kmr_api A1
(T004) → T009 → T010 → T011 → T012 (critical path step 10). kmr_api A5 (T013, step 9) → T014
(step 11, HUMAN bench) → T015. T002 and T003 sit beside T001. Phases 4 to 8 are off this path.

### Blocked today

Only T001, T002, T003 and T030 can start now. T005 waits on an unblocked kmr_api step (A2). Every
other task waits on a HUMAN core step, directly or through kmr_api: T004 and T009 (core K1.1),
T013 and T026 (core K1.6 to K1.8), T014, T015 (core T004–T013, K1.1 to K1.10), T023 and T024
(core K1.6), T028 (core K5.2, K5.3),
T029 (core const-generic N). T018, T021, T027 are HUMAN.

### Within a phase

- One file per story phase: T006 → T007 → T008 → T009 → T010 → T011 in `main.rs`, one commit.
- T019 before T020 (same README).
- T018, T027 and kmr_adapters T053 edit the same CI file: one at a time.

## Parallel opportunities

- T002 beside T001 (Cargo.toml, main.rs).
- After T011: T016, T017, T019, T022 together (read-only runs and one new file).
- T030 any time; T031 after T010.

```text
Setup:       T001 ‖ T002 → T003
US1 code:    T005 → T006 → T007 → T008 ; T004 → T009 → T010 → T011 → T012
After T011:  T016 ‖ T017 ‖ T019 ‖ T022
Hardware:    T013 → T014 → T015 → T021 ; T013 + T025 → T026
```

## Implementation strategy

### MVP: delivery step 1 (D1 A)

1. T001 now, before kmr_api A4. T002, T003 with it.
2. When kmr_api A1 and A2 land: T006 to T012, one commit. The file runs simulated.
3. When kmr_api A5 lands: the HUMAN bench, T014 and T015. A motor moves and stops: critical path
   step 11.

### Incremental

4. US2 checks and the CI lint gate (T016 to T018).
5. US4: cross-builds, one warning, readable error (T022 to T027).
6. US3: walkthrough and trial (T019 to T021).
7. Phase 7 when kmr_api C4, A4, B7 land. Polish last. Then the PR from `agent/<topic>` into the
   human branch, `agentic` label.

## Notes

- Commits: AGENTS.md rule (b) and the human's final decision: no AI or co-author trailer. The
  constitution's Principle 0 asks for one. The human settles this conflict; until then commits
  follow the human's decision.
- Delivery order (constitution I, SRS § 8) holds under D1 A: the example is the controller of
  delivery step 1.
- An agent that reaches a HUMAN task, or a task blocked on one, stops and hands it over.
- Commit per task or per group, Conventional Commits, scope `controllers`.

## Analysis 2026-10-28: left for spec and plan

Found by `/speckit-analyze`. Fixed in this file:

- CRITICAL, constitution gates the plan does not list: decision record (T030, constitution V),
  C4 update (T032, Development Workflow).
- HIGH: SC-004 "on every change" and SC-008 had no CI gate; plan C8 covers the V2 smoke run only.
  Now T018 and T027; plan C8 should list them.
- MEDIUM: FR-012 not in the V1 search (T016); no run for a wired robot that does not match the
  preset (T014); C9 must come before the bench if core K5.3 lands first (T028); the root README
  and ROADMAP go stale with C2 (T031).

Left for spec, plan or the human:

| Sev | Where | Issue |
|---|---|---|
| CRITICAL | constitution 0; AGENTS.md rule (b) | Agent trailer required by one, forbidden by the other. Plan records it; no artifact here can settle it |
| HIGH | spec FR-006 (Q1), FR-028 (Q2) | Two NEEDS CLARIFICATION markers stay; `/speckit-clarify` not run. Plan assumes D15 A, D14 |
| HIGH | spec SC-009; quickstart V5 | "As recorded in History": the binary prints nothing of its own, and no step says how History is recorded on the bench |
| MEDIUM | plan Constitution Check, D1 paragraph | "Written once steps 3 to 9 land; nothing starts early" contradicts the work outline: C0, C1 now, C2 after kmr_api A1, A2 |
| MEDIUM | research R13 | "`read_all` fills every actuator or bring-up fails" has no owning step in the core or kmr_adapters plan |
| MEDIUM | spec SC-010; quickstart V5 | "Within one tick of its stated duration": no measurement named |
| MEDIUM | research R10 | The prototype directory is gone; its rows cannot be rerun (T012, T022 re-prove them) |
| LOW | spec Key Entities vs data-model | Spec: the context holds the start "and nothing else"; data-model adds the start time `t0` |
| LOW | plan, contracts/api-needs.md | "core #14": the core plan id is K1.1 |
| LOW | plan C8 vs kmr_adapters T053 | Same `ci.yml`: HUMAN here, agent there |
| LOW | kmr_adapters T055 | Cross-builds `controllers` with `--features kmr_api/dynamixel`; this plan names the feature `hardware` |

## Analysis 2026-10-28 (cross-module)

- 2026-10-28: T014 takes the one bench blocker list (core T004–T013, kmr_adapters T017, T041, kmr_api T011, T012 here) and a local `Last` logger that makes SC-009 and SC-010 measurable (closes the HIGH SC-009 and MEDIUM SC-010 rows above); T015 adds the local `emergency_stop!` line that core T018 and kmr_adapters T045 name; T030 wording "next free number in docs/decisions/" (it already avoided a fixed number); dependency, hand-off and blocked-today lines follow.
