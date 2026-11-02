---
description: "Tasks for kmr_ffi, foreign-language controller math, generated 2026-10-28"
---

# Tasks: kmr_ffi, foreign-language controller math

**Input**: `specs/006-kmr-ffi/`: [plan.md](plan.md) (2026-10-28), [spec.md](spec.md)
(coordinated 2026-10-28), [research.md](research.md), [data-model.md](data-model.md),
[quickstart.md](quickstart.md), [contracts/](contracts/). Authority: `docs/srs/SRS.md`.
Coordination: `specs/spec-coordination-2026-10-28.md`. Format reference:
`specs/001-kmr-adapters/tasks.md`.

**Gate**: SRS § 8 delivery step 8, Could. Nothing here starts before steps 1 to 7 move a motor
(decided 2026-10-28: D1 A). No task is on the critical path to the first motor (coordination
§ 3). The list is kept lean.

**Tests**: included. The spec counts SC-001 to SC-007 by tests. No test needs hardware. SC-003
(bench) and SC-008 (onboarding) are HUMAN or other modules' runs.

**HUMAN**: tasks in `kmr_core/` (`.aiignore`) and policy files are the human's, and so is
anything `unsafe` (T007, T008, T012, T015; AGENTS.md). An agent never does them. A task that touches an
invariant file names it in **bold**.

**Hand-off**: kmr_api owns `kmr_api/src/ffi.rs` and its features (kmr_api plan G1, G2); bench owns
`tick_ffi` (bench plan D-4). This list edits nothing in `kmr_api/`, `bench/` or `controllers/`. A
hand-off task names the owner's step and keeps its id for order only.

**Blocked**: a task that waits on a HUMAN task or another module says "Blocked on ...".

**Assumed, pending human**: D1 (A, gate), D3 (A, `envelope` flag per actuator), D4 (A, N ≤ 64),
D12 (A, user `build.rs` compiles foreign code, robot named again, const check), D13 (A, status
return), D14 (ROS2 baseline). Q1 (time and flags as integers) has no D id: coordination § 2 lists
it as "FR-10.2 vs FR-3.3". Each is isolated to the place its task names.

**Ids**: `FR-0NN` / `SC-0NN` are this spec's, written with their SRS source. Plan steps are A1 to
E5 of plan.md. User stories: US1 C or C++ (P1), US2 Python or Lua (P2), US3 history, staleness,
faults and time (P3), US4 a new field without FFI chores (P4).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an unfinished task)
- **[Story]**: US1..US4 from spec.md

Paths are from the repository root.

---

## Phase 1: Setup

**Purpose**: the gate, a green baseline, the crate and the heap rule.

- [ ] T001 Gate check, decided 2026-10-28: D1 A. Read docs/ROADMAP.md and the coordination page: delivery steps 1 to 7 are done and a motor moved; if not, stop and hand over. Run quickstart.md § 5 on the current tree (`cargo fmt --all --check`, `cargo clippy --workspace --all-targets`, `cargo test --workspace --no-fail-fast`); stop and report if red. Note whether core K3.6 (`Kp`, `Kd`) has landed (T004, T006 depend on it) and whether core M2 gives `robot!` its N (T012 depends on it)
- [ ] T002 HUMAN (plan A1): root Cargo.toml `members` gains `kmr_ffi`, then each example crate as it lands (T014, T019, T023), and `[workspace.dependencies] kmr_ffi = { path = "kmr_ffi" }`; kmr_ffi/clippy.toml overrides the root heap ban like kmr_api/clippy.toml; .specify/memory/constitution.md Principle II amended so kmr_ffi may use the heap like kmr_api, never in kmr's own per-call work (FR-019 (SRS FR-10.3, section 4)). Blocks T007 (`ForeignError` holds `String`)
- [x] T003 Create kmr_ffi/Cargo.toml (workspace package, edition 2024; no kmr dependency, research R9; features off by default: `build` = `cc`, `python` = `pyo3`, `lua` = `mlua` with vendored Lua 5.4; versions picked here, none is in the offline cache) and kmr_ffi/src/lib.rs (module doc; `#![forbid(clippy::unwrap_used)]`; `#![deny(clippy::panic, clippy::indexing_slicing)]`; modules `fields`, `view`, `call`, and `build`, `python`, `lua` behind their features; `pub` only what kmr_api re-exports) (FR-019, FR-022 (SRS FR-10.3, NFR-2)). Builds once T002 adds the member

---

## Phase 2: Foundational: the types that cross and the one call (plan A2 to A4, C1)

**Purpose**: plain types, the field list and `call`, tested with Rust doubles and no engine.
Blocks every story.

- [x] T004 Create kmr_ffi/src/fields.rs: `for_each_field!`, the one FFI line per field. It lists every field of the core's sealed set when the task runs: `q, qd, tau`, plus `kp, kd` if core K3.6 has landed (core M3 is delivery step 3, before this step 8; data-model.md and contracts/c-abi.md show three). Add `for_each_flag!(stale, bus_error, offline, rejected, joint_limit, envelope)`, the per-actuator flags of core K4.1 (`envelope` per actuator: assumed, pending human: D3 A). Both macros drive every per-field and per-flag item below, headers and bridges included (FR-014 (SRS FR-10.2, FR-2.2))
- [x] T005 Create kmr_ffi/src/view.rs from T004's macros, all `#[repr(C)]`, `Copy`, no pointers, no lifetimes (data-model.md § Types that cross): `pub type Mask = u64` (assumed, pending human: D4 A; the one place); `ActuatorFlags` (one `u64` per flag, bit i = actuator i); `Slot<N>` (one `[f32; N]` per field in engine units, `flags`, `tick_flags: u32`, `reserved: u32` "zero; keeps the size a multiple of 8"); `ReadView<N, D>` (`samples: u32` "valid slots, 0..=D. 0 = no sample", `reserved: u32`, `age: [u32; N]` latest slot only, `AGE_NONE = u32::MAX` "never read", `slot: [Slot<N>; D]` with `slot[k]` = `prev(k)`); `Written` (one `u64` per field); `WriteBuf<N>` (one `[f32; N]` per field, `written`); `TimeView` (`tick`, `dt_ns`, `elapsed_ns`, `overrun_count`, `last_overrun_ns` "0 = on time", `accumulated_overrun_ns`, all `u64`; assumed, pending human: Q1). `ReadView::default` const-asserts `1 <= N <= 64` (FR-006 to FR-010 (SRS FR-10.2, FR-2.1, FR-2.6, FR-3.3, FR-13.4))
- [x] T006 [P] In kmr_ffi/src/view.rs add the layout tests (plan A3, research R1 rerun on the real types): size, alignment and every field offset of the six types for N = 1, 4, 64 and D = 4, through `size_of`, `align_of`, `offset_of!`. With three fields they equal R1's table (`kmr_read_view` 304 / 440 / 3560 with `age` at 8; `kmr_slot` 72 / 104 / 824; `kmr_write_buf` 40 / 72 / 792; `kmr_actuator_flags` and `kmr_time_view` 48; all aligned to 8); with `kp`, `kd` they equal a hand count by the C11 rules written in the test. Add an `extern "C"` declaration over `*const ReadView<4, 4>` under `#[deny(improper_ctypes)]`, and a `compile_fail` doctest on `ReadView::<65, 4>::default()` (quickstart.md § 1)
- [ ] T007 HUMAN (`unsafe`: the `unsafe extern "C" fn` type and the crate's only `unsafe` block). Create kmr_ffi/src/call.rs (plan A4; data-model.md § Rust-only types): `CFn<N, D>` = `unsafe extern "C" fn(*const ReadView<N, D>, *mut WriteBuf<N>, *const TimeView, *mut c_void) -> i32`; `Status { Continue, Shutdown, EmergencyStop }` from 0, 1, 2 (assumed, pending human: D13 A); `ForeignError { Code(i32), Python(String), Lua(String), Load(String) }` with a one-line `Display`; the `Foreign` trait that bounds `call` in contracts/rust-surface.md, implemented for `CFn`; `pub fn call` over `&ReadView`, `&mut WriteBuf`, `&TimeView` and `Option<&mut T>`, never a raw pointer (research R1 clippy finding). It clears the buffer, makes the one call in the crate's only `unsafe` block, and on `rc < 0 || rc > 2` clears the buffer and returns `Err(ForeignError::Code(rc))`. No heap, no lock, no `dyn` on this path (FR-003, FR-011, FR-012, FR-021, FR-022 (SRS FR-10.1, FR-2.5, NFR-1, NFR-2)). Blocked on T002
- [ ] T008 HUMAN (`unsafe impl GlobalAlloc` in kmr_ffi/tests/no_alloc.rs). Tests for T007. In kmr_ffi/src/call.rs, with Rust `extern "C" fn` doubles: write then return -7 gives `Err(Code(-7))`, a zeroed buffer and mask 0; 0, 1, 2 give the three statuses with the writes kept; 3 is an error; a context counter reaches 1000 after 1000 calls; `None` passes null. In kmr_ffi/tests/no_alloc.rs, a counting `#[global_allocator]`: 10 000 calls at N = 64 allocate nothing; kmr_ffi/src/call.rs and kmr_ffi/src/view.rs use no `std::sync` type (SC-003 kmr side, SC-007 C part (SRS NFR-1, NFR-2)). `cargo test -p kmr_ffi` and `cargo clippy -p kmr_ffi --all-targets` clean (quickstart.md § 1)
- [ ] T009 Hand-off, HUMAN (plan C1), owner core T004 (K1.1), T006, T007 (K1.3, K1.4; **state/field.rs**), T008 (K1.5), T036 (K3.5), T044 (K4.1), T057 (K7.1, new for this step; **state/field.rs**). No edit here. Unblocks T013, T028

**Checkpoint**: `cargo test -p kmr_ffi` green with no engine; the layout matches R1.

---

## Phase 3: User Story 1, C and C++ (Priority: P1), plan B1 to B4, C2 to C4 🎯 MVP

**Goal**: a Rust controller forwards to a C or C++ function that reads the views and writes the
buffer. FR-001, FR-003, FR-006 to FR-016, FR-021 to FR-024.

**Independent Test**: quickstart.md §§ 2, 3: `cargo test -p kmr_ffi_example_c` and
`-p kmr_ffi_example_cpp`; the C twin matches the Rust twin bit for bit (SC-001).

- [x] T010 [US1] Create kmr_ffi/src/build.rs (feature `build`; plan B1). From T004's macros, write into `OUT_DIR`: `kmr_ffi.h` (C11) and `kmr_ffi.hpp` (C++17) exactly as contracts/c-abi.md (`KMR_N`, `KMR_HISTORY_DEPTH`, `KMR_AGE_NONE UINT32_MAX`, the six types, `static inline kmr_set_<field>(w, i, v)` that ignores `i >= KMR_N`, the status enum; `kmr::Q`, `Qd`, `Tau` with `static_assert(sizeof(Q) == sizeof(float))`, `In`, `Out`, `KMR_CPP_CONTROLLER` as `noexcept` plus `catch (...)` returning -100), and `kmr_ffi_abi.rs` (`FFI_N`, `FFI_DEPTH`); `compile(&[..])` over `cc`; errors `UnknownPreset`, `Descriptor(..)`, `Compile(..)`. The generator takes N and D as arguments; T011 supplies them. Tests: header text for N = 1, 4, 64; it names no builder, `run()`, phase, bus or HAL item (FR-002). FR-015, FR-016 (SRS FR-10.1, FR-6.1); assumed, pending human: D12 A
- [ ] T011 [US1] Blocked on kmr_model T059 (the library entry: lib + macro split, research R12; not planned there, pending human: D12) and on where the lookup lives (research R4 reads kmr_model and the core's `HISTORY_DEPTH`; R9 gives kmr_ffi no kmr dependency; reported). In kmr_ffi/src/build.rs add `robot(name)` and `.model(path)`: N from kmr_model's library entry for a preset name or descriptor path, the same N `robot!` gets, and D = `HISTORY_DEPTH`; an unknown preset or a bad descriptor gives the same message as `robot!` (plan B2; FR-015 (SRS FR-6.1))
- [ ] T012 [US1] HUMAN (`unsafe extern "C"` declarations, edition 2024). In kmr_ffi/src/call.rs add `foreign! { fn a; fn b; }`: per name, an `unsafe extern "C"` declaration with the `CFn<N, HISTORY_DEPTH>` signature; `include!` of `OUT_DIR/kmr_ffi_abi.rs`; const asserts `FFI_N == N` and `FFI_DEPTH == HISTORY_DEPTH` with the message "kmr_ffi: C sources were built for another N"; `N` resolves from `robot!` in scope (contracts/rust-surface.md § Declaring; research R4; FR-015 (SRS FR-6.1, FR-10.2); assumed, pending human: D12 A)
- [ ] T013 [US1] Hand-off, owner kmr_api G1 (kmr_api FR-072); blocked on T005, T007, T010, T012, T009 and kmr_api C1, C2 (age, stale and fault readers on `State`). kmr_ffi plan C2 and C3 land there: features `ffi = ["dep:kmr_ffi"]` and `ffi-build`; the `kmr_api::ffi` re-exports of contracts/rust-surface.md; `ffi::call(f, time, state, ctx)` doing data-model.md § One call, steps 1 to 7 (fill every depth, flags and age; clear; call; an error applies nothing; apply each set bit through the field's setter; status 1 or 2 raises `shutdown!` or `emergency_stop!` through the exported macros, no edit to **kmr_core/src/signal.rs (invariant)**); the apply step is an exhaustive `match` on core `FieldId` (K7.1), generated from `for_each_field!` (FR-001, FR-003, FR-011, FR-013, FR-023 (SRS FR-10.1, FR-2.5, FR-4.1, FR-4.2)). No kmr_api edit here
- [ ] T014 [US1] Blocked on T011, T013. Create the C example crate `kmr_ffi_example_c` (plan B3; FR-024 (SRS FR-10.4)), four user files: kmr_ffi/examples/c/Cargo.toml (`kmr_api` with `ffi`; build-dependency `kmr_api` with `ffi-build`; no other kmr crate, FR-001), kmr_ffi/examples/c/build.rs (`robot("XmBench").compile(&["src/swing.c"])`), kmr_ffi/examples/c/src/main.rs (`robot!(XmBench)`, `foreign! { fn swing; }`, an `EachTick` inline forwarder; an `Err` raises `emergency_stop!`), kmr_ffi/examples/c/src/swing.c (an error when `samples == 0`, else `q[0]`, `q[2]` = previous `q` + 0.1). Runs with no hardware
- [ ] T015 [US1] HUMAN (`unsafe` FFI calls into the C probes). In kmr_ffi/examples/c/tests/abi.rs, with C probes compiled by the example's build.rs: for N = 1, 3, 4, 10, 64, C `sizeof`, `_Alignof`, `offsetof` on the generated header equal Rust `size_of`, `align_of`, `offset_of!`; C reads a Rust-filled view by field name bit for bit (`age[N-1]` = `KMR_AGE_NONE`, `slot[3].q[N-1]`, a `u64` time above 2^32); `kmr_set_q(w, N, ..)` changes nothing; a C body that writes then returns -7 applies nothing (FR-006, FR-008, FR-012; SC-007 C part)
- [ ] T016 [US1] Blocked on T013 and core T065 (HUMAN, M3: the `test-util` feature, scripted `Host` and recording adapter, the History seam plan C4 names). In kmr_ffi/examples/c/tests/twin.rs, over 1000 scripted ticks: the C forwarder and its Rust twin give the same Desired values and written masks, bit for bit, from an inline controller and from an inline-with-context controller whose C side counts calls in `ctx` (SC-001 (SRS FR-10.3, FR-2.5); US1 scenarios 1, 2, 3, 5). C that writes nothing sends no command. A Rust controller that writes actuator 1 before the C forwarder keeps that write when C leaves actuator 1 unwritten (spec Edge cases). One C function on two registrations runs in declaration order (US1 scenario 4). A NaN written from C and from Rust ends the same, under the core's `rejected` rule (FR-013 (SRS FR-2.2, NFR-2))
- [ ] T017 [US1] Blocked on T011 (a second robot of another N, from a fixture descriptor through `.model(..)`). In kmr_ffi/examples/c/tests/wrong_n.rs run `cargo check` on the fixture crate kmr_ffi/examples/c/tests/fixtures/wrong_n/, whose build.rs names a robot of another N than its `robot!`; expect `error[E0080]` with "kmr_ffi: C sources were built for another N" at the `foreign!` line (SC-005 (SRS FR-10.2, FR-6.1))
- [ ] T018 [US1] Blocked on T013. In kmr_ffi/examples/c/tests/status.rs (its own process, a file-local lock, as kmr_core/tests/run_signals.rs), C bodies in kmr_ffi/examples/c/tests/status.c built by the example's build.rs, a no-hardware run, and a Rust `Last` controller that counts its calls: an `EachTick` C body returning 2 at tick 3 leaves the `Last` count at 2 (the stop is immediate); returning 1 at tick 3 gives 3 (the tick completes); two C forwarders in one `EachTick` bucket returning 2 then 1 at tick 3 leave it at 2, so the later shutdown did not replace the emergency stop. Writes kept before a status are T008's (FR-023 (SRS FR-4.1, FR-4.2, FR-4.3, FR-4.4); assumed, pending human: D13 A)
- [ ] T019 [US1] Blocked on T011, T013. Create the C++ example crate `kmr_ffi_example_cpp` (plan B4): kmr_ffi/examples/cpp/Cargo.toml, build.rs, src/main.rs, src/swing.cpp with `KMR_CPP_CONTROLLER` and typed `Q`, `Qd`, `Tau`. In kmr_ffi/examples/cpp/tests/cpp.rs: a throwing body returns -100 and applies nothing; its non-throwing twin applies `q[0]`; the typed accessors read the same values as the C view (FR-016 (SRS FR-10.1, NFR-2); SC-007 C++ part; US1 scenario 6)

**Checkpoint**: a C and a C++ function move the simulated robot like a Rust controller.

---

## Phase 4: User Story 2, Python and Lua (Priority: P2), plan D1 to D3

**Goal**: a script reads the same views and writes the same buffer, from any form, inline
included. FR-004, FR-005, FR-017 to FR-020.

**Independent Test**: quickstart.md § 4: SC-002 on 1000 scripted ticks; a script that does not
load returns `ForeignError::Load` before `run()`.

- [ ] T020 [P] [US2] Create kmr_ffi/src/python.rs (feature `python`, `pyo3`; plan D1; contracts/bridges.md). First check that `pyo3` supports the host CPython (3.14, research R8). `Bridge::load(path, function)` starts the interpreter, runs the file, finds the function; every failure is `Err(ForeignError::Load(..))`, never a panic (FR-020 (SRS FR-6.4)). Per call: refill persistent sequences (`read.<field>[depth][i]` 0-based, `read.age[i]` an int or `None` for never read, flag masks, `read.samples`, `time.*` as int), call `step(read, write, time)`, take `write.set_<field>(i, v)`; `0` or `None`, 1, 2 map to `Status`, any other value is `Code(-1)`; an exception is `ForeignError::Python(text)` with nothing applied (FR-012). Implements `Foreign` (FR-017, FR-019 (SRS FR-10.1, FR-10.3))
- [ ] T021 [P] [US2] Create kmr_ffi/src/lua.rs (feature `lua`, `mlua` with vendored Lua 5.4; plan D2; contracts/bridges.md): the contract of T020 for a Lua function, with 1-based tables (depth 0 is `[1]`, actuator i is element i+1), `read.age[i]` = -1 for never read, 0-based mask bits, 64-bit integers for time; a Lua error is `ForeignError::Lua(text)`, nothing applied; `mlua`'s `send` feature, so the bridge can move into a threaded controller (FR-004 (SRS FR-1.2)). Implements `Foreign` (FR-018, FR-019, FR-020 (SRS FR-10.1, FR-10.3, FR-6.4))
- [ ] T022 [US2] Hand-off, owner kmr_api G2; blocked on T013, T020, T021. Features `ffi-python = ["ffi", "kmr_ffi/python"]`, `ffi-lua = ["ffi", "kmr_ffi/lua"]`; re-exports `kmr_api::ffi::python::Bridge`, `kmr_api::ffi::lua::Bridge`. No kmr_api edit here
- [ ] T023 [US2] Blocked on T022. Create `kmr_ffi_example_python` (kmr_ffi/examples/python/Cargo.toml, src/main.rs, swing.py) and `kmr_ffi_example_lua` (kmr_ffi/examples/lua/Cargo.toml, src/main.rs, swing.lua): `Bridge::load(..)?` in `main.rs` before `run()`; the bridge is the context of an `EachTick` `add_controller_with` forwarder; the script writes `q[i]` = previous `q[i]` + 0.1 (FR-024 (SRS FR-10.4); data-model.md § Bridge lifecycle). `cargo run -p kmr_ffi_example_python` and `-p kmr_ffi_example_lua` run with no hardware
- [ ] T024 [US2] Blocked on T023 and core T065 (`test-util`, the History seam of T016). In kmr_ffi/examples/python/tests/bridge.rs and kmr_ffi/examples/lua/tests/bridge.rs: over 1000 scripted ticks from an inline-with-context controller, the written masks equal the Rust twin's and the values equal it after rounding to `f32` (SC-002 (SRS FR-10.3)); a missing file or function returns `ForeignError::Load` from `main` before `run()`; an exception or Lua error, a wrong return value, and an index out of range on `write` each apply nothing (SC-007 bridge part; US2 scenarios 1 to 3)

**Checkpoint**: Python and Lua forwarders match the Rust twin; every script error is returned.

---

## Phase 5: User Story 3, history, staleness, faults and time (Priority: P3), plan C4

**Goal**: foreign code sees what a Rust controller sees through `State` and `Time`, nothing
more. FR-009, FR-010.

**Independent Test**: `cargo test -p kmr_ffi_example_c history time`.

- [ ] T025 [US3] Blocked on core T065 (`test-util`, the History seam of T016) and on core K3.5, K4.1 (T009). In kmr_ffi/examples/c/tests/history.rs: for every depth k < `HISTORY_DEPTH`, C reads the values `prev_q(k)`, `prev_qd(k)`, `prev_tau(k)` return; a failed read on actuator 1 sets bit 1 of `slot[0].flags.stale`, keeps the last good value (never zero) and gives `age[1] > 0`; the sample age C derives from `age` (the oldest) equals `sample_age()`; a no-hardware run gives `samples == 0` and every `age[i] == KMR_AGE_NONE`; each flag of `for_each_flag!` matches kmr_api's `Faults` view (FR-010 (SRS FR-2.1, FR-2.3, FR-2.6, FR-13.4); US3 scenarios 1 to 3)
- [ ] T026 [US3] Blocked on core T065 (`test-util`: a scripted clock). In kmr_ffi/examples/c/tests/time.rs, run with `--release`: after 10^8 scripted ticks, `time.tick` and `time.elapsed_ns` seen by C are exact, and `overrun_count`, `last_overrun_ns`, `accumulated_overrun_ns` equal the `Time` getters (SC-006 (SRS FR-3.3); FR-009; US3 scenario 4; assumed, pending human: Q1)

**Checkpoint**: depth, age, stale and fault flags and time match the Rust side.

---

## Phase 6: User Story 4, a new field without FFI chores (Priority: P4), plan C3

**Goal**: one declaration per field; a forgotten one fails the build. FR-014, SC-004.

**Independent Test**: `cargo test -p kmr_ffi fields`, then kmr_api's trybuild case.

- [ ] T027 [US4] In kmr_ffi/src/fields.rs add a test: a test-only field list with one extra field `dummy` gives it a `Slot` array, a `WriteBuf` array, a `Written` mask, a line in `kmr_ffi.h` and `kmr_ffi.hpp`, and a name in both bridges' lists, from that one line (SC-004 first half; US4 scenario 1; FR-014 (SRS FR-10.2))
- [ ] T028 [US4] Hand-off, owner kmr_api G1 (its trybuild suite, kmr_api/tests/ui/); blocked on T013 and core K7.1 (T009). A compile-fail case: the apply-step `match` generated from a field list that leaves one `FieldId` variant out fails with `E0004` naming that variant (SC-004 second half; US4 scenario 2). kmr_api G1 does not list this case yet (reported). No kmr_api edit here

**Checkpoint**: adding `Kp` or `Kd` is one line in kmr_ffi/src/fields.rs; leaving it out fails
`cargo check`.

---

## Phase 7: Polish and cross-cutting, plan E1 to E5

- [ ] T029 [P] Write docs/decisions/NNNN-ffi-boundary.md, NNNN the next free number in docs/decisions/ at commit time (plan E1 names 0004; no number is reserved): copy versus shared view (research R2), header production and the `build.rs` exception (D12 A; SRS FR-6.1 conflict, research R14), integer time, ages and flags (Q1, research R3), status signals (D13 A, research R5), the bridge as context (SRS FR-1.1), the N cap (D4 A); every "assumed, pending human" kept (FR-026 (SRS NFR-6); constitution V)
- [ ] T030 [P] Crate docs in kmr_ffi/src/lib.rs, kmr_ffi/src/python.rs, kmr_ffi/src/lua.rs and the header comments of kmr_ffi/src/build.rs: a crash or abort in foreign code leaves torque as it was, and no software guard can prevent it, next to the pre-flight lesson of SRS FR-5.10 (FR-025 (SRS NFR-2, FR-5.10)); each bridge's cost (GIL, VM allocation) is the user's (FR-005 (SRS FR-10.3)); Lua is 1-based; C memory safety is the user's, every array is N long (spec Edge cases)
- [ ] T031 Hand-off, owner bench D-4 and D-5 (HUMAN runs); plan E3; blocked on T011, T012, T013. Bench's `tick_ffi` with an empty C forwarder, its `build.rs` compiling foreign code only (D12 A), measures the per-tick cost over an empty Rust controller at 1 kHz, NFR-8 (d) (SC-003 (SRS NFR-1, NFR-11)). No bench edit here
- [ ] T032 HUMAN (plan E4): SC-008 onboarding trial. A C++ engineer and a Python engineer each reach a first moving controller from T019 and T023, timed and file-counted against the ROS2 baseline and count rule of controllers C3, with the protocol of controllers C7 (SRS FR-10.4; assumed, pending human: D14)
- [ ] T033 Run quickstart.md § 5: `cargo fmt --all --check`, `cargo clippy --workspace --all-targets`, `cargo test --workspace --no-fail-fast`; `cargo build -p kmr_api` without FFI features compiles no kmr_ffi code (`cargo tree -p kmr_api -e normal` lists no `kmr_ffi`) (FR-001; kmr_api FR-072). The example crates are workspace members, so these gates need a C and C++ compiler and the Python headers on every machine and in .github/workflows/ci.yml; report what CI lacks, and that the aarch64 and armv7 cross-builds of the Python example need a pyo3 cross set-up (plan does not cover it)
- [ ] T034 Update docs/c4/kmr_ffi.dsl with `/speckit-ov-c4-code` (plan E5; constitution, Development Workflow)

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup**: T001 first; it stops everything if the D1 gate is not met. T002 is HUMAN and blocks
  T007 and every `cargo` run on kmr_ffi. T003 after T001.
- **Foundational (T004 to T008)** after T003; T007, T008 are HUMAN (`unsafe`). T009 hands off
  to core tasks (HUMAN); most land on the critical path and in core M3, M4 long before step 8;
  K7.1 (core T057) is new.
- **US1 (T010 to T019)** after Foundational. T010 and T012 (HUMAN, `unsafe`) need only kmr_ffi.
  T011 waits on kmr_model T059 (D12). T013 waits on T009 and kmr_api C1, C2. T014 to T019 wait
  on T011 and T013; T016 also on core T065 (`test-util`).
- **US2 (T020 to T024)**: T020 and T021 after Foundational, in parallel with US1. T022 after T013.
  T023, T024 after T022; T024 also on core T065.
- **US3 (T025, T026)** after T014 and core T065. Can run beside US2.
- **US4**: T027 after T004, T010, T020, T021 (it checks headers and both bridges). T028 after T013
  and core K7.1.
- **Polish**: T029, T030 any time after the code they describe. T031 after T011, T012, T013. T032 after
  T019, T023 and controllers C3. T033, T034 last.

### Hand-offs to other modules

| Task | Owner step | Waits on |
| --- | --- | --- |
| T009 | core T004, T006–T008, T036, T044, T057 (K1.1, K1.3 to K1.5, K3.5, K4.1, K7.1; HUMAN) | none |
| T011 | kmr_model T059 (lib + macro split, research R12) | D12 |
| T016, T024–T026 | core T065 (`test-util`, HUMAN) | none |
| T013 | kmr_api G1 | T005, T007, T010, T012, T009; kmr_api C1, C2 |
| T022 | kmr_api G2 | T013, T020, T021 |
| T028 | kmr_api G1 trybuild case | T013, core K7.1 |
| T031 | bench D-4, D-5 | T011, T012, T013 |
| T032 | controllers C3 (baseline), C7 (protocol) | D14 |

### Plan steps mapped

| Plan | Tasks |
| --- | --- |
| A1 | T002, T003 |
| A2 to A4 | T004 to T008 |
| B1, B2 | T010, T011, T012 |
| B3, B4 | T014, T015, T017, T019 |
| C1 | T009 |
| C2, C3 | T013, T028 (kmr_api G1) |
| C4 | T016, T018, T025, T026 |
| D1 to D3 | T020 to T024 |
| E1 to E5 | T029 to T034 |

## Parallel opportunities

- T009 (HUMAN) beside every agent task.
- T006 (view.rs tests) beside T007 (call.rs).
- T020 and T021 (python.rs, lua.rs) beside all of US1.
- T029 and T030.

```text
Foundational:  T004 → T005 → (T006 ‖ T007) → T008
US1:           T010 → T012 (HUMAN) → T013 hand-off;  T011 blocked (kmr_model T059)
               (T011, T013) → T014 → T015 to T018;  (T011, T013) → T019
US2:           (T020 ‖ T021) → T022 hand-off → T023 → T024
```

## Implementation strategy

### MVP: US1, the C path

1. T001 gate, T002 HUMAN, T003.
2. Foundational: types, field list, `call`, tested with doubles.
3. T010, T012 (HUMAN) in kmr_ffi; T011 once kmr_model T059 lands; kmr_api G1 lands the glue (T013).
4. C example and its tests (T014 to T018). Stop and validate SC-001, SC-005, SC-007 C part.

### Incremental

5. C++ (T019), then Python and Lua (US2), history and time (US3), field checks (US4).
6. Polish: decision record, crate docs, bench and onboarding hand-offs, gates, C4.
7. The PR goes from `agent/<topic>` into the human branch, `agentic` label.

## Notes

- Threaded form (FR-004 (SRS FR-1.2)): threaded controllers are never spawned (core #22, out of
  this version). No task tests the threaded case; T021 keeps `mlua` `Send` for it.
- Commits: AGENTS.md rule (b) and the human's final decision: no AI or co-author trailer. The
  constitution's Principle 0 asks for one. This conflict is the human's to settle; until then
  commits follow the human's decision.
- An agent that reaches a HUMAN task, or a task blocked on one, stops and hands it over.
- Commit per task or per group, Conventional Commits, scope `ffi`.

## Analysis 2026-10-28 (cross-module)

- 2026-10-28: T007, T008, T012 HUMAN (each writes `unsafe`: the call block, a counting `#[global_allocator]`, `unsafe extern "C"` declarations); T009 one-line hand-off to core T004, T006–T008, T036, T044, T057; T011 waits on kmr_model T059; T016, T024–T026 wait on new core T065 (`test-util`), no longer an unowned seam; T029 drops reserved record numbers. Flagged, not changed: T015 calls C probe functions from Rust, which also needs `unsafe`.
