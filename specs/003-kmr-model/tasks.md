---
description: "Tasks for kmr_model, the robot model from one descriptor, generated 2026-10-28"
---

# Tasks: kmr_model, the robot model from one descriptor

**Input**: `specs/003-kmr-model/`: [plan.md](plan.md) (2026-10-28), [spec.md](spec.md) (re-checked
2026-10-28), [research.md](research.md) (R1 to R13), [data-model.md](data-model.md),
[quickstart.md](quickstart.md), [contracts/](contracts/) (`robot-macro.md`, `descriptor-ron.md`),
[review-2026-10-28.md](review-2026-10-28.md). Authority: `docs/srs/SRS.md`. Coordination:
`specs/spec-coordination-2026-10-28.md`.

**Off the critical path** (coordination § 3). Nothing here is needed for the first motor. Under
D1 A this module is delivery step 2: every agent task after T001 waits on T002.

**Tests**: included. Plan R10 asks for `trybuild` compile-fail cases and unit tests on the pure
parse functions; SC-002 counts failing builds. No test needs hardware.

**HUMAN**: tasks in `kmr_core/` (`.aiignore`), the bench URDF data and the bench runs are the
human's. An agent never edits them. A task that touches an invariant file names it in **bold**.

**Hand-off**: this plan edits `kmr_model/`, the test files and fixtures under `kmr_api/tests/`
(its `trybuild` cases, kmr_api plan § B; the harness `kmr_api/tests/ui.rs` and the `trybuild`
dev-dependency are kmr_api T019's, and T048 adds one `t.pass` line to the harness), the
`[[test]]` entries of `kmr_api/Cargo.toml`, the root `Cargo.toml` dependency table and the docs
named in Polish. It edits nothing in `kmr_api/src/`, `kmr_adapters/` or `controllers/`. A task that needs one of them
says "Hand-off" and names the owning step; it keeps its id for order only.

**Blocked**: an agent task that waits says "Blocked on Tnnn" or names the other module's id.

**Assumed, pending human**: D1 (A: step 1 is the first motor, this module is step 2), D4 (A:
N ≤ 64, `MASK_WORDS` = 1), D7 (A: kmr_api's front door splits `hardware = ..` off before the
proc macro), D12 (open: the library entry for kmr_ffi is not planned, T059 tracks it), D16
("replace" `try_lock`, kmr_api B6). The `no_std` user path is out of this version (spec Scope,
recommended, pending human confirmation). D2 does not reach this module: the model applies no
policy (FR-014 (SRS FR-4.5)).

**Ids**: `FR-0NN` / `SC-0NN` are this spec's, written with their SRS source: `FR-008 (SRS
FR-6.4)`. User stories are spec.md's: US1 N and order (P1), US2 bad descriptor (P1), US7 presets
(P1), US3 limits (P2), US4 other formats (P2), US5 name lookup (P3), US6 link masses (P3).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an unfinished task)
- **[Story]**: US1..US7 from spec.md

Paths are from the repository root.

---

## Phase 1: Setup

**Purpose**: a green baseline, the delivery gate, the human data, the build-time dependencies.

- [x] T001 Run quickstart.md § 1 and § 8 from the repo root: `cargo build --workspace`, `timeout -s INT 5 cargo run -p kmr_api --example complete_api_example`, `cargo clippy --workspace --all-targets`, `cargo fmt --all --check`, then `cargo test --workspace --no-fail-fast`; record the test counts; confirm the `.d` file of the `robot_macro` test under `target/debug/deps/` lists `kmr_api/tests/fixtures/four.urdf` (review: rebuild tracking works). Stop and report if anything is red
- [ ] T002 HUMAN: delivery gate (Constitution I, SRS § 8, research R11 item 2; decided 2026-10-28: D1 A). Agent work starts only after the first motor moved (coordination § 3 step 11: kmr_adapters T045, kmr_api A9, controllers C6), or after the maintainer writes a waiver for delivery step 2 on the PR. Blocks T004 and every later agent task
- [ ] T003 HUMAN (data): write kmr_model/presets/xm_bench.urdf, the real XM430-W210 bench chain (research R7): four revolute joints in the order of `XmBench::HARDWARE` ids 1 to 4 (kmr_adapters T023), each with `<limit lower upper velocity effort>` in rad, rad/s, N·m; every link with `<inertial>` mass (kg) and origin (m). Agents never invent physical limits. It replaces the T023 placeholder and unblocks T029
- [x] T004 Build-time dependencies (research R8, R9, R10). In Cargo.toml (root) `[workspace.dependencies]` add `syn` (`default-features = false`, `features = ["parsing", "proc-macro"]`), `quote`, `proc-macro2`, `ron`, `serde` (feature `derive`), `serde_json`, `trybuild`, at the versions Cargo.lock already resolves where present (syn 2.0, quote 1.0, proc-macro2 1.0, ron 0.12, serde 1.0). In kmr_model/Cargo.toml depend on all but `trybuild`; kmr_api's `[dev-dependencies]` line for `trybuild` is kmr_api T019's. None reaches a binary: kmr_model is a proc-macro crate (FR-019 (SRS NFR-1))
- [ ] T005 HUMAN (decisions, report only; nothing waits on it): three conflicts this plan cannot settle (research R11). Constitution II lets only kmr_api use heap types; kmr_model uses them at build time (`kmr_model/clippy.toml`; coordination § 2 "§4 heap rules"). Constitution Principle 0 asks for agent trailers; the maintainer decided none. SRS NFR-10 says "kmr_model is a dependency of kmr_core", false with a const-generic N

---

## Phase 2: Foundational: the proc macro skeleton

**Purpose**: input, descriptor loading, build-time model, errors, and the emission of N. Every
story extends these files. Blocks every story.

- [x] T006 [P] Create kmr_model/src/model.rs with the build-time types of data-model.md: `Format { Urdf, Ron, Json }`; `Descriptor { path, format, text }`; `Model { actuators: Vec<Actuator>, links: Vec<Link> }`; `Actuator { name, kind, lower, upper, speed, effort, pos }` with `Kind { Revolute, Continuous, Prismatic }`, limits `f64` "while parsing" (`lower`, `upper` as `Option<f64>`), `pos` (line, col); `Link { name, parent: Option<usize>, mass, com: [f64; 3], pos }`; `ModelError { file, pos: Option<(line, col)>, element, reason, marker: Option<&'static str> }` with one `message()` in the shape of contracts/robot-macro.md § Errors (`kmr:` first; `:<line>` dropped when `pos` is `None`; the marker appended when set). Add the private `trait Reader { fn read(text: &str, file: &Path) -> Result<Model, ModelError>; }`, one impl per format, no trait object (research R8b; FR-025 (SRS FR-11.3)). Add `validate(model) -> Result<Model, ModelError>` with step 10 only, "at least one actuator" (`kmr: <path>: no actuated joint`), so zero actuators never reaches the front door's count assert (review #3). Doc comment on every item (CONTRIBUTING rule 8) Done 2026-11-02, reported: `Model` also carries `joints` (every joint, fixed included, by name) for steps 6 and 9; `validate` returns a checked `Robot` (`f32` values in SRS FR-2.2 units, link parents resolved), which `emit` takes; `ModelError` keeps `line` and `col` apart, the column shown only for parser faults; `Reader::read` takes the shown path as `&str`.
- [x] T007 [P] Create kmr_model/src/input.rs: parse the input with `syn` as `<krate> ; preset = <Ident>` or `<krate> ; model = <LitStr>`. `<krate>` is every token before the first `;`, kept as a `proc_macro2::TokenStream`: the front door passes `$crate`, so emitted code names `<krate>::Q`, `<krate>::Qd`, `<krate>::Tau` (assumption of this task, reported: plan and contract do not say how emitted code reaches kmr_api's types). The path comes from `syn::LitStr::value()`: raw strings work, escapes resolve (review #4). Neither key, both keys, or any other key is an error spanned on the input; a preset's model half cannot be overridden (FR-024 (SRS FR-6.1, FR-13.5)). No `hardware` key: the front door splits it off (assumed, pending human: D7 A). Unit tests in the same file for each form and each refusal
- [x] T008 Create kmr_model/src/emit.rs: `emit(model: &Model, krate: &TokenStream2, tracked: Option<&Path>) -> TokenStream2` with `quote`: `pub const N: usize = <n>;` (FR-002, FR-004 (SRS FR-6.1)); for a file descriptor `const _: &[u8] = include_bytes!("<absolute path>");` (research R2; FR-021 (SRS NFR-11)); an empty `pub mod model { }` that the stories fill. Literals through `proc_macro2::Literal` (`usize_unsuffixed`, `f32_suffixed`, `string`), never formatted text (research R9). The emitted code names no `std` item, no heap type, no `dyn`, no lock (FR-019 (SRS NFR-1)). After T006
- [x] T009 In kmr_model/src/lib.rs add `#[proc_macro] pub fn model(input: TokenStream) -> TokenStream`: input (T007), then the descriptor, then a `Reader` chosen by extension through a `match`, then `validate`, then `emit` (T008). Loading: join the path to `CARGO_MANIFEST_DIR` (research R1); a path that is not valid UTF-8 is an error (review #4); extension `.urdf`, `.ron` or `.json`, else `kmr: <path>: expected .urdf, .ron or .json` (review #9); a read failure is `kmr: cannot read robot descriptor <path>: <io error>`. The `.ron` and `.json` arms return `kmr: <path>: .ron and JSON descriptors are not supported yet` until T037. Every error is a `syn::Error` spanned on the path literal or the preset ident, emitted with `to_compile_error()`; no `unwrap_or_default`, no tokens parsed from strings (review #5; FR-001 (SRS FR-6.1), FR-008 (SRS FR-6.4, NFR-2)). Add `clippy::expect_used` and `clippy::panic` beside the existing `#![forbid(clippy::unwrap_used)]`. Keep `actuator_count` as it is until T055: kmr_api still calls it. `cargo build -p kmr_model` and `cargo test -p kmr_model` green
- [x] T010 Hand-off, owner kmr_api T021 (plan B2); blocked on T009. kmr_api's front door calls the new proc macro, `$crate::__model!($crate; preset = $preset)` and `$crate::__model!($crate; model = $path)`, and kmr_api/src/lib.rs re-exports `kmr_model::model` as the hidden `__model`; `@items` goes behind a hidden internal macro so a user cannot set N (review #7); the N ≤ 64 assert (assumed, pending human: D4 A); the `three.urdf` `compile_fail` doctest goes after T022 replaces it (review #10). The front door keeps `State`, `Robot`, `__Spec` and the temporary `N == JOINTS` assert; the proc macro emits none of them (research R3). kmr_api T021 now names this switch. No edit here Done 2026-11-02 by kmr_api T021.

**Checkpoint**: `cargo test -p kmr_model` green; the proc macro exists; nothing calls it yet.

---

## Phase 3: User Story 1, the descriptor fixes N and the order (Priority: P1) 🎯 MVP

**Goal**: a URDF gives N and the actuator order as constants; fixed joints get no index. FR-001,
FR-002, FR-003, FR-004 (SRS FR-6.1), FR-021 (SRS NFR-11).

**Independent Test**: quickstart.md § 2: `cargo test -p kmr_model urdf` and `cargo test -p kmr_api
--test robot_macro`: four.urdf gives N = 4, six_plus_two.urdf N = 6, continuous.urdf N = 1, names
in file order.

- [x] T011 [P] [US1] Fixtures in kmr_api/tests/fixtures/, all with full limits (spec Assumptions; review #10). Fix four.urdf, the reference robot (N = 4, names `j1` to `j4` kept): `j1` revolute `<limit lower="-1.57" upper="1.57" velocity="3.0" effort="1.5"/>`; `j2` continuous `<limit velocity="3.0" effort="1.5"/>`; `j3` prismatic `<limit lower="0.0" upper="0.1" velocity="0.05" effort="20"/>`; `j4` revolute `<limit lower="-3.14" upper="3.14" velocity="4.0" effort="2.0"/>`; `mount` fixed; transmission `t1` with joint `j1` and one `<actuator name="m1"/>`; `<inertial>` on every link, `l1` with mass 0.35 at origin `0 0 0.05`. Give the three joints of three.urdf `j1`'s limit. Create six_plus_two.urdf (`a1` revolute, `f1` fixed, `a2` revolute, `a3` prismatic, `a4` continuous, `f2` fixed, `a5` revolute, `a6` revolute; one chain), continuous.urdf (one continuous joint `spin`) and prismatic.urdf (one prismatic joint `slide` with `j3`'s limit). Values are test data, not a robot
- [x] T012 [P] [US1] Create kmr_model/src/urdf.rs with only its `#[cfg(test)] mod tests`, red first, on inline URDF text: 4 revolute joints give 4 actuators in file order (US1 AS1); the six_plus_two shape gives 6 and the fixed joints get no index (AS2); one continuous joint gives 1; a joint or a link without `name` is an error naming the file (review #9); `floating` and `planar` stop with `roadmap: non-1-DOF joints` (FR-010 (SRS § 8)). SC-001 (SRS FR-6.1)
- [x] T013 [US1] In kmr_model/src/urdf.rs add `struct Urdf` and its `Reader` impl on `roxmltree` (research R8). Root must be `<robot>` (data-model validation 3). Joints in document order: `fixed` skipped; `revolute`, `continuous`, `prismatic` kept; `floating`, `planar` stop with `roadmap: non-1-DOF joints` (validation 4; FR-010 (SRS § 8)); another type, or none, is an error naming the joint. Every joint and link needs a name (review #9). Reads each joint's `<parent>`, `<child>` and `<limit>` attributes as `f64`, with `text_pos_at` positions; a limit attribute that is not a number is an error naming the joint and the attribute. Turns T012 green. FR-002, FR-003 (SRS FR-6.1)
- [x] T014 [US1] In kmr_model/src/emit.rs emit `model::NAMES: [&'static str; N]`, joint names in index order (FR-015 (SRS FR-2.4), FR-020 (SRS NFR-10)); in kmr_model/src/lib.rs route `.urdf` to `Urdf`
- [x] T015 [US1] Blocked on T010 (kmr_api B2). In kmr_api/tests/robot_macro.rs: `robot!(model = "tests/fixtures/four.urdf")` gives `N == 4` and `model::NAMES == ["j1", "j2", "j3", "j4"]`; modules `six` and `one` call `kmr_api::__model!(kmr_api; model = "tests/fixtures/six_plus_two.urdf")` and `continuous.urdf` and check N = 6, N = 1 and the names in file order. Direct calls, because the front door's `N == JOINTS` assert rejects N ≠ 4 until kmr_api B7; T048 moves them to `robot!`. Rename `urdf_gives_n_without_fixed_joints_or_transmissions` to what it checks now. Quickstart § 2; SC-001 (SRS FR-6.1) Done 2026-11-02: `urdf_gives_n_and_names_without_fixed_joints_or_transmissions`; modules `six` and `one` by `kmr_api::__model!`.
- [x] T016 [US1] Run the rebuild half of quickstart.md § 7 on four.urdf: the `.d` file of the `robot_macro` test lists the fixture; a second `--no-run` recompiles nothing; `touch kmr_api/tests/fixtures/four.urdf` recompiles the test crate only. Put the exact output lines in the PR (FR-021 (SRS NFR-11); spec edge case "the descriptor changes") Done 2026-11-02: `target/debug/deps/robot_macro-49f134c676a6f220.d` lists four.urdf (3 lines); a second `--no-run` prints only `Finished`; after `touch kmr_api/tests/fixtures/four.urdf`, `-v` prints `Fresh kmr_model`, `Fresh kmr_core`, `Fresh kmr_adapters`, `Dirty kmr_api ..: the file kmr_api/tests/fixtures/four.urdf has changed` and one `rustc --crate-name robot_macro`: the test crate only.

**Checkpoint**: N and the order come from a URDF. MVP of this module.

---

## Phase 4: User Story 2, a bad descriptor does not build (Priority: P1)

**Goal**: every bad descriptor of the spec's Edge Cases is a compile error naming the file, the
element and the reason. FR-008 (SRS FR-6.4, NFR-2), FR-009 (SRS Def. "Index space"), FR-010
(SRS § 8), FR-011 (SRS FR-6.5), FR-026 (SRS FR-2.2).

**Independent Test**: quickstart.md § 3, `cargo test -p kmr_model model` and `cargo test -p
kmr_api --test ui`.

- [x] T017 [P] [US2] Bad fixtures in kmr_api/tests/fixtures/, one per edge case of spec.md, each otherwise valid and minimal: bad_syntax.urdf (an unclosed tag on a known line), bad_root.urdf (root `<model>`), bad_unnamed_joint.urdf, bad_unnamed_link.urdf, bad_zero_actuators.urdf (fixed joints only), bad_duplicate_joint.urdf, bad_duplicate_link.urdf, bad_no_limit.urdf (revolute without `<limit>`), bad_continuous_no_effort.urdf, bad_lower_above_upper.urdf, bad_negative_velocity.urdf, bad_negative_effort.urdf, bad_not_finite.urdf (`upper="inf"`), bad_not_f32.urdf (`effort="1e39"`), bad_prismatic_overflow.urdf (prismatic `upper="3.5e35"`: fits `f32` in m, not in mm), bad_floating.urdf, bad_planar.urdf, bad_mimic.urdf, bad_transmission.urdf (one transmission, two joints), bad_two_roots.urdf, bad_undefined_link.urdf (a joint's child names no link), bad_ext.yaml
- [x] T018 [US2] Hand-off, owner kmr_api T019: the harness kmr_api/tests/ui.rs (one `#[test] fn ui()`, `trybuild::TestCases::new().compile_fail("tests/ui/*.rs")`, research R10) and the `trybuild` dev-dependency are kmr_api's crate code. Every ui case here (T021, T022, T026, T039, T041, T048) waits on it. No edit here Done 2026-11-02 by kmr_api T019.
- [x] T019 [US2] Unit tests, red first, in kmr_model/src/model.rs `#[cfg(test)] mod tests`, one per validation step 3 to 10 of data-model.md, on inline text: each error names the file and the element, carries the reason of contracts/robot-macro.md § Errors and, where one applies, the marker `SRS index space: multi-actuator joint` (FR-009) or `SRS FR-6.5: multi-robot descriptor` (FR-011). A text with two faults reports the earlier step only. Passing cases (spec defaults and edge cases): a `<transmission>` with one joint and one actuator; one URDF with a common root link for several chains builds as one robot. In kmr_model/src/lib.rs a unit test: a non-UTF-8 manifest path (`OsStr::from_bytes`) is an error; a Rust string literal cannot hold one, so this case has no ui test
- [x] T020 [US2] In kmr_model/src/model.rs complete `validate` per data-model.md "Validation, in order (first failure stops the build)", before step 10 (T006): step 6 "Unique joint and link names" (`duplicate joint <name> (first at line <l>)`); step 7 "revolute and prismatic need `<limit lower upper velocity effort>`; continuous needs `velocity effort` only. `lower <= upper`, `velocity >= 0`, `effort >= 0`"; step 8 "prismatic position and speed × 1000 (m to mm, FR-017). Every value finite and within `f32` after conversion"; step 9 "one root link; every joint's parent and child are known links" (several roots carry the FR-011 marker). In kmr_model/src/urdf.rs step 5: "`<mimic>`, or a `<transmission>` with more than one joint or actuator" stops with the FR-009 marker. No panic on any input. Turns T019 green. FR-008 (SRS FR-6.4, NFR-2), FR-009 (SRS Def. "Index space"), FR-011 (SRS FR-6.5), FR-017, FR-026 (SRS FR-2.2) Done 2026-11-02 with T006, reported: step 9 also refuses a link that is the child of two joints and a loop of joints (a tree needs both); the reader runs steps 3 to 5 in passes, so a two-fault text reports the earlier step. Fixed 2026-11-02 after review: step 5 also refuses a joint or an actuator named in two transmissions (FR-009).
- [x] T021 [US2] Blocked on T010 and kmr_api T019 (harness). One ui case per T017 fixture in kmr_api/tests/ui/ (`<case>.rs`: `kmr_api::robot!(model = "..")` at module level and an empty `fn main`). trybuild compiles each case in its own crate under `target/tests/trybuild/kmr_api/` (trybuild 1.0.120 `src/run.rs:168`), so `CARGO_MANIFEST_DIR` there is not `kmr_api/`: case paths read `../../../../kmr_api/tests/fixtures/<file>` (default target dir; CI must keep it). Check one case by hand before writing the rest; the `.stderr` paths come out as `$WORKSPACE/..`. Plus missing_file.rs and two_robot_calls.rs (two calls in one module: duplicate items, spec edge case). Write each `.stderr` with `TRYBUILD=overwrite`, then read every one: the first error is the contract message, names the file and the element, carries the marker where one applies; no `count differs from kmr_core's` line anywhere (review #3). SC-002 (SRS FR-6.4) Done 2026-11-02: kmr_api/tests/ui/model_<case>.rs, one per T017 fixture (22), plus model_missing_file.rs and two_robot_calls.rs (E0428 on `N`, `model`, `State`, `Robot`, `__Spec`; E0252 `actuator`; E0119). Every model case has one error, the contract message with the file, the element and the marker where one applies; no `count differs from kmr_core's` line outside n_differs_from_core.rs. Shown paths read `$WORKSPACE/target/tests/trybuild/kmr_api/../../../../kmr_api/tests/fixtures/<file>`: kmr_model shows the joined path, not a canonical one.
- [x] T022 [US2] Blocked on T010. Add kmr_api/tests/ui/n_differs_from_core.rs on three.urdf (limits from T011): it fails with the contract's temporary message `kmr: this robot's actuator count differs from kmr_core's; N as a const generic is pending in kmr_core`. It replaces the `compile_fail` doctest of kmr_api/src/robot_macro.rs, which passes on any error (review #10); kmr_api B2 removes the doctest (T010) Done 2026-11-02: kmr_api/tests/ui/n_differs_from_core.rs on `../../../../kmr_api/tests/fixtures/three.urdf`, one E0080 with that message; the doctest is gone.

**Checkpoint**: every edge case fails the build with a message naming file and element.

---

## Phase 5: User Story 7, a preset names the whole robot (Priority: P1)

**Goal**: `robot!(XmBench)` takes N and the order from the preset's model half, shipped in
kmr_model. FR-022, FR-023 (SRS FR-6.1), FR-024 (SRS FR-6.1, FR-13.5).

**Independent Test**: `cargo test -p kmr_model presets` and `cargo test -p kmr_api --test presets`;
the `unknown_preset` ui case.

- [x] T023 [US7] Create kmr_model/presets/xm_bench.urdf as a placeholder (research R7: "until it exists, the preset is a test fixture marked as such"). First line, a comment: `PLACEHOLDER: test fixture, not the XM430 bench; limits and masses are test data; replaced by the human URDF (tasks.md T003)`. Four revolute joints `joint1` to `joint4`, in the order of `XmBench::HARDWARE` ids 1 to 4 (kmr_adapters T023), each with four.urdf `j1`'s limit; every link with `<inertial>`. No prismatic or continuous joint, so kmr_adapters' kind check (its T058) passes. While it stands, kmr_api B3 does not hand XmBench's limits to the core's limit stage (core K5.2): T029 lifts that
- [x] T024 [US7] Create kmr_model/src/presets.rs: one table, preset name to (shown path `kmr_model/presets/xm_bench.urdf`, text by `include_str!("../presets/xm_bench.urdf")`). In kmr_model/src/lib.rs route `preset = X` through it, the reader picked by the shown path's extension, with no `include_bytes!` tracking (the text lives in kmr_model; editing it rebuilds kmr_model and its callers). An unknown name is `kmr: no preset named <X>; presets: <list>`, spanned on the ident (review #5; FR-022 (SRS FR-6.1)). Unit tests in the same file: `XmBench` gives 4 actuators; `Xmbench` names itself and lists `XmBench` (US7 AS2) Done 2026-11-02, reported: the table also holds `Rx28Chain` (kmr_model/presets/rx28_chain.urdf, already a marked test fixture), since kmr_adapters ships its hardware half; the placeholder sets every link mass to 0 (none is known) and drops the `<?xml?>` line, which XML allows only before the first-line comment.
- [x] T025 [US7] Blocked on T010. Create kmr_api/tests/presets.rs: module `a` calls `kmr_api::robot!(XmBench)`, module `b` calls `kmr_api::robot!(model = "../kmr_model/presets/xm_bench.urdf")`; `N` and every `model::*` item emitted so far are equal (US7 AS1; SC-010 (SRS FR-6.1)). Extend it when T031, T032 and T043 add items Done 2026-11-02: modules `by_name`, `by_path`; N and the ten `model::*` items emitted so far.
- [x] T026 [US7] Blocked on T010. ui cases kmr_api/tests/ui/unknown_preset.rs (`robot!(Xmbench)`: the snapshot names `Xmbench` and lists `XmBench`; US7 AS2) and kmr_api/tests/ui/preset_with_model.rs (`robot!(XmBench, model = "..")` does not build; FR-024 (SRS FR-6.1, FR-13.5)) Done 2026-11-02: unknown_preset.rs (`kmr: no preset named Xmbench; presets: XmBench, Rx28Chain`, the one error); preset_with_model.rs (`no rules expected ,`: no arm takes a preset and a model).
- [x] T027 [US7] Hand-off, owner kmr_api T024 (its ui case kmr_api/tests/ui/count.rs; kmr_api FR-007 owns the count check): `robot!(model = four.urdf, hardware = Table { .. })` with 3 actuators fails with rustc E0308 "expected an array with a size of 4, found one with a size of 3" (US7 AS3; FR-023 (SRS FR-6.1); assumed, pending human: D7 A). No edit here Done 2026-11-02 by kmr_api T024 (kmr_api/tests/ui/count.rs).
- [ ] T028 [US7] Hand-off, owner kmr_adapters T068 (it owns the presets' hardware half; its T023 keeps `Preset::N` "until `robot!` takes N from `kmr_model`"); blocked on T010. Once kmr_api's preset arm calls the proc macro (kmr_api T021), `Preset::N` and its derivation from the table go: N comes from the model half (research R7; review #2). No edit here
- [ ] T029 [US7] Blocked on T003. Check kmr_model/presets/xm_bench.urdf is the human's file (no PLACEHOLDER line, 4 actuators, order of `XmBench::HARDWARE`); rerun `cargo test -p kmr_model presets`, `cargo test -p kmr_api --test presets` and the ui suite; then tell kmr_api B3 (kmr_api T067, T038), in its PR, that XmBench's limits may reach the core's limit stage

**Checkpoint**: `robot!(XmBench)` and `robot!(model = <its file>)` give the same facts.

---

## Phase 6: User Story 3, limits per actuator (Priority: P2)

**Goal**: lowest and highest position, top speed and top effort per actuator, in SRS FR-2.2
units, plus a readable "no position limit" bit. FR-012 (SRS FR-4.5), FR-013 (SRS FR-4.5,
NFR-10), FR-014 (SRS FR-4.5), FR-017, FR-026 (SRS FR-2.2).

**Independent Test**: quickstart.md § 5, `cargo test -p kmr_model emit` and `cargo test -p kmr_api
--test robot_macro -- limits`.

- [x] T030 [P] [US3] Unit tests, red first, in kmr_model/src/emit.rs `#[cfg(test)] mod tests`, on the per-actuator `f32` values `emit` writes (a pure helper returns them): the inline four.urdf gives `j1` -1.57, 1.57, 3.0, 1.5 (US3 AS1); `j3` 0.0, 100.0, 50.0, 20.0, prismatic in mm, mm/s, N (AS2; FR-017 (SRS FR-2.2)); `j2` the 0.0 placeholder for both positions (data-model)
- [x] T031 [US3] In kmr_model/src/emit.rs emit `model::LOWER`, `model::UPPER`: `[<krate>::Q; N]`; `model::SPEED`: `[<krate>::Qd; N]`; `model::EFFORT`: `[<krate>::Tau; N]`, in index order, literals by `Literal::f32_suffixed` (FR-012 (SRS FR-4.5), FR-020 (SRS NFR-10), FR-026 (SRS FR-2.2)). A continuous actuator gets the "`Q(0.0)` placeholder where `CONTINUOUS` is set" (data-model). No policy, no clamp (FR-014 (SRS FR-4.5)). Each item's doc comment gives the units: revolute rad, rad/s, N·m; prismatic mm, mm/s, N. Turns T030 green
- [ ] T032 [US3] Blocked on core T025 (K2.5, through the T046 hand-off: `MASK_WORDS` public in `kmr_core::hal`; today `WORDS` is private, `kmr_core/src/hal.rs:23`) and kmr_api T066 (the hidden `MASK_WORDS` re-export; it lands before kmr_api T025, B3, which waits on this task). In kmr_model/src/emit.rs emit `model::PRISMATIC` and `model::CONTINUOUS` as `[u64; <krate>::MASK_WORDS]`, written `{ let mut m = [0u64; MASK_WORDS]; m[0] = ..; m }` with only the words that hold a set bit; "Bit `i` of a set is word `i / 64`, bit `i % 64`, the same layout as `kmr_core::hal::Mask`"; revolute is neither (FR-013 (SRS FR-4.5, NFR-10), FR-017 (SRS FR-2.2)). Unit test: `j2` sets bit 1 of `CONTINUOUS`, `j3` bit 2 of `PRISMATIC` (US3 AS3: readable, "not a fake infinite value")
- [ ] T033 [US3] Blocked on T010. In kmr_api/tests/robot_macro.rs add `limits` tests on four.urdf: `model::LOWER[0] == Q(-1.57)` and the other T030 values; after T032, `model::CONTINUOUS[0] & (1 << 1) != 0` and `model::PRISMATIC[0] & (1 << 2) != 0`. Run `cargo test -p kmr_api --test robot_macro -- limits` Partly done 2026-11-02: module `limits` checks the four actuators of four.urdf (T030 values, `j2`'s `Q(0.0)` placeholder, `j3` in mm; `j4`'s 3.14 under `allow(clippy::approx_constant)`). The `CONTINUOUS`/`PRISMATIC` checks wait on T032.
- [ ] T034 [US3] Hand-off, owners kmr_api B3 (kmr_api T025, T026; the `[Limit; N]` builder is kmr_api T067), kmr_adapters T058 (plan E6) and core K5.2 (HUMAN); blocked on T031 and T032. kmr_api B3 maps the limits to the core's `[Limit; N]`, and every set `CONTINUOUS` bit to `lower`/`upper` = `None`, never the `Q(0.0)` placeholder; it passes `model::PRISMATIC` to `__TABLE.check_kinds(..)`. For XmBench, only after T029. No edit here

**Checkpoint**: every actuator's limits are constants in index order; a continuous joint is a bit,
not a number.

---

## Phase 7: User Story 4, same robot, other format (Priority: P2)

**Goal**: `.ron` and JSON give the same facts as URDF. FR-001 (SRS FR-6.1), FR-005 (SRS FR-6.2),
FR-006 (SRS FR-6.1), FR-025 (SRS FR-11.3).

**Independent Test**: quickstart.md § 4, `cargo test -p kmr_api --test formats`.

- [x] T035 [P] [US4] Fixtures in kmr_api/tests/fixtures/: four.ron and four.json, the robot of four.urdf in the schema of contracts/descriptor-ron.md (same names, order, limits, links, masses, COM); bad_unknown_field.ron (a `groups` field), bad_hardware_field.ron (a `baud` field on a joint), bad_syntax.ron, bad_syntax.json, bad_continuous_lower.ron (a `Continuous` joint with `lower`)
- [x] T036 [US4] Create kmr_model/src/serde_format.rs: one serde schema for `.ron` and JSON (contracts/descriptor-ron.md; research R8): `Robot { name, links, joints }`, `#[serde(deny_unknown_fields)]` on every struct ("Unknown fields are errors"); `kind` is `Revolute | Continuous | Prismatic | Fixed`; "`limit` is required for every non-fixed joint. `lower` and `upper` are optional fields: required for `Revolute` and `Prismatic`, rejected for `Continuous`"; actuator order is the order of `joints`, `Fixed` skipped (FR-003 (SRS FR-6.1)); link parents come from the joints. `struct Ron` and `struct Json` implement `Reader` with `ron::from_str` and `serde_json::from_str`; a parse error reads `kmr: <path>:<line>:<col>: <parser message>`. Errors found after parsing have no position, so the message drops `:<line>` (data-model `pos: Option`; reported against the contract's message shapes). A `kind` outside the four is serde's unknown-variant error; whether `Floating`/`Planar` there must carry FR-010's roadmap text is reported, not decided here. No hardware or group field can be read (FR-005 (SRS FR-6.2), FR-006 (SRS FR-6.1)) Done 2026-11-02, reported: a link may leave out `mass` and `com` (0 at the origin, as a URDF link without `<inertial>`); the schema types are named `Link`, `Joint`, `Limit`, `Kind` in messages; ron's unknown-field message reads ``Unexpected field named `groups` in `Robot`, expected one of ..``, not `unknown field <f>` (T039 checks it); `Floating`/`Planar` stay serde's unknown-variant error.
- [x] T037 [US4] In kmr_model/src/lib.rs route `.ron` and `.json` to the T036 readers, replacing the "not supported yet" arm of T009; the same `validate` runs after every reader. Unit tests in kmr_model/src/serde_format.rs: inline four.ron and four.json give the same `Model` as inline four.urdf, positions aside (US4 AS1)
- [x] T038 [US4] Blocked on T010. Create kmr_api/tests/formats.rs: modules `urdf`, `ron`, `json` call `kmr_api::robot!` on four.urdf, four.ron, four.json; `N` and every `model::*` item emitted so far are equal; extend when T032 and T043 add items (SC-003 (SRS FR-6.1); quickstart § 4) Done 2026-11-02: modules `urdf`, `ron`, `json`, compared item by item by a local `same_model!`.
- [x] T039 [US4] Blocked on T010. ui cases in kmr_api/tests/ui/, one per bad `.ron`/`.json` fixture of T035: the unknown fields give `kmr: <path>:<line>:<col>: unknown field <f>`; the syntax errors give line and column; the continuous `lower` is refused (spec edge case "a field the format does not know"; FR-006 (SRS FR-6.1); SC-002 (SRS FR-6.4)) Done 2026-11-02: model_unknown_field_ron.rs, model_hardware_field_ron.rs, model_syntax_ron.rs, model_syntax_json.rs, model_continuous_lower_ron.rs. kmr_model/src/serde_format.rs now words ron's `NoSuchStructField` as the contract's `unknown field `<f>` in `<struct>``, at line and column (was ron's `Unexpected field named ..`); serde_json already says `unknown field`.

**Checkpoint**: one model, three doors.

---

## Phase 8: User Story 5, find an actuator by name (Priority: P3)

**Goal**: a compile-time name-to-index lookup; a wrong name does not build. FR-015 (SRS FR-2.4),
FR-005 (SRS FR-6.2).

**Independent Test**: `cargo test -p kmr_api --test robot_macro -- names` and the
`unknown_actuator` ui case.

- [x] T040 [US5] In kmr_model/src/emit.rs emit the lookup of research R6: `macro_rules! actuator` with one arm per name expanding to its index as a `usize` literal, and a catch-all arm `compile_error!(concat!("kmr: no actuator named ", $n))`; then `pub(crate) use actuator;`, so `crate::actuator!` reaches it from a child module. Not `#[macro_export]`: two `robot!` or `__model!` calls in one test crate would export two root macros (assumption of this task, reported). No accessor method, no group (FR-005 (SRS FR-6.2), FR-015 (SRS FR-2.4))
- [x] T041 [US5] Blocked on T010. In kmr_api/tests/robot_macro.rs add `names` tests on four.urdf: `const J3: usize = actuator!("j3");` is 2 at the root, and through `crate::actuator!` from a child module; a const group `[actuator!("j1"), actuator!("j4")]` is `[0, 3]` (US5 AS1). ui case kmr_api/tests/ui/unknown_actuator.rs: `actuator!("wrist")` fails with `kmr: no actuator named wrist` (US5 AS2; R6: the span points at the `robot!` line, accepted) Done 2026-11-02: module `names` (`J3`, `ENDS`, `crate::actuator!` from `names::child`); kmr_api/tests/ui/unknown_actuator.rs, span on the `actuator!` call.

---

## Phase 9: User Story 6, link masses for payloads (Priority: P3)

**Goal**: each link's name, parent, mass and mass center as constants, and a way to tell whether a
link exists. FR-016 (SRS FR-8.1, FR-8.3).

**Independent Test**: `cargo test -p kmr_model urdf` and `cargo test -p kmr_api --test robot_macro
-- links`.

- [x] T042 [P] [US6] Unit tests, red first, in kmr_model/src/urdf.rs tests: `l1` of the inline four.urdf has mass 0.35 kg and COM (0, 0, 0.05) m (US6 AS1); the root link has no parent, every other link's parent is its joint's parent link; a link without `<inertial>` has mass 0 and COM (0, 0, 0); a non-finite mass fails validation step 8
- [x] T043 [US6] In kmr_model/src/urdf.rs read `<inertial><mass value>` (kg) and `<inertial><origin xyz>` (m) per link; a link without `<inertial>` gets mass 0 and COM origin, the URDF default (assumption of this task, reported: spec silent). In kmr_model/src/model.rs step 8 also checks link mass and COM finite and within `f32`. In kmr_model/src/emit.rs emit `model::LINK_NAMES: [&'static str; L]`, `model::LINK_PARENT: [Option<u16>; L]` (`None` for the root; a parent index above `u16::MAX` is a compile error, never a wrap), `model::LINK_MASS: [f32; L]` (kg), `model::LINK_COM: [[f32; 3]; L]` (m, link frame), `L` a literal length (data-model). No center-of-mass computation (SRS FR-8.3, roadmap), no light payload input (FR-016 (SRS FR-8.1, FR-8.3)). Turns T042 green Done 2026-11-02, reported: the `u16` parent check sits in validation step 9, so the error names the file and the link; an `<inertial>` without `<mass value>` is an error, as in urdfdom.
- [x] T044 [US6] Blocked on T010. In kmr_api/tests/robot_macro.rs add `links` tests on four.urdf: `model::LINK_MASS` at the index of `"l1"` in `model::LINK_NAMES` is 0.35; `LINK_PARENT` of `"base"` is `None`; `"tool"` is in `LINK_NAMES`, `"gripper"` is not (FR-016 (SRS FR-8.1), link-exists lookup) Done 2026-11-02: module `links`, also `LINK_COM` of `l1` and `LINK_PARENT` of `l1` = `base`.
- [x] T045 [US6] Hand-off, owner kmr_api B5; blocked on T043. `register_payload` checks the mount link against `model::LINK_NAMES` at `finalize()`; a missing link is a `run()` error naming it (US6 AS2; SRS FR-6.4, FR-8.4). No edit here Landed 2026-11-02 with kmr_api T027, T028: `finalize()` checks each mount against `Spec::LINK_NAMES` (from `model::LINK_NAMES`); kmr_api/tests/run.rs refuses `"nope"` with `UnknownLink`, before the binding.

---

## Phase 10: After the core's const-generic N (delivery step 2, core half)

**Purpose**: the human core work this module waits on, and the test changes it allows.

- [ ] T046 Hand-off, HUMAN, owner core T020–T027 (K2.0 to K2.7; K2.10 is not needed here). Core T025 (K2.5, `MASK_WORDS` public) unblocks T032 with kmr_api T066. No edit here
- [ ] T047 Hand-off, owner kmr_api B7 (kmr_api T032, T033; design step 8); blocked on T046. kmr_api drops `kmr_api::N`, the default `State` export and the `N == JOINTS` assert; the aliases take N. No edit here
- [ ] T048 [US1] Blocked on T047. In kmr_api/tests/robot_macro.rs move modules `six` and `one` from `__model!` to `robot!`. Move kmr_api/tests/ui/n_differs_from_core.rs to kmr_api/tests/ui/pass/, add `t.pass("tests/ui/pass/*.rs")` in kmr_api/tests/ui.rs and drop its `.stderr` (design step 8: "the `three.urdf` test now passes"). Add kmr_api/tests/ui/no_robot_call.rs: a controller that names `State` without `robot!` fails (spec edge case "`main.rs` has no `robot!` call"). The one owner of the `three.urdf` pass case: kmr_api T033 hands it off here
- [ ] T062 [US1] Blocked on T047 (kmr_api B7: the `N == JOINTS` assert is gone). The robots of core SC-009 (core T030 checks them): create kmr_api/tests/fixtures/ten.urdf and sixty_four.urdf (one chain of revolute joints, `j1`'s limit on each, generated once like T052's hundred.urdf; commit the files, not the loop). In kmr_api/tests/robot_macro.rs, for N = 1, 4, 10, 64 (continuous.urdf, four.urdf, ten.urdf, sixty_four.urdf): `robot!(model = ..)` builds, and a robot with one controller that raises `shutdown!` at tick 1 returns `Ok(())` on `NoHardware` (one file-local lock for the global signal bus). N = 65 is kmr_api T033's `n_65.rs` (core SC-009 (SRS FR-6.1))

---

## Phase 11: Polish and cross-cutting

- [x] T049 [P] In kmr_model/src/emit.rs tests, walk the token trees of the four.urdf emission (no `syn` `full` feature): the top level holds only `N`, `model`, the `actuator` macro and its `use`, and unnamed `const _`; `model` holds only the items of data-model.md § Emitted facts; no `fn`, `impl`, `struct`, `trait` or group (FR-005 (SRS FR-6.2), FR-018, FR-019 (SRS NFR-1); SC-009 (SRS FR-6.2))
- [x] T050 [P] Create kmr_api/tests/no_std_caller.rs: `#![no_std]`, `kmr_api::robot!(model = "tests/fixtures/four.urdf");` and one `#[test]` reading `N`, `model::NAMES[0]` and `model::LOWER[0]` (quickstart § 6). If the test harness refuses `#![no_std]`, declare `[[test]] name = "no_std_caller"` with `harness = false` in kmr_api/Cargo.toml and do the reads in `fn main`; report if neither builds. The emitted facts need no `std` or `alloc` (FR-019 (SRS NFR-1); SC-006 (SRS NFR-7)); a `no_std` firmware path stays out of this version Done 2026-11-02, reported: the test calls `kmr_model::model!(kmr_api; model = ..)`, since `robot!` emits no `model::*` before T010 (switch it to `robot!` then). The test harness accepts `#![no_std]`, no `harness = false` needed; a probe naming `::std::string::String` or `Vec` in that file fails to build, so the crate really has no `std` in scope.
- [x] T051 Cross-builds with no hardware attached (quickstart § 6; US1 AS3): `cargo build --workspace --target aarch64-unknown-linux-gnu` and `--target armv7-unknown-linux-gnueabihf`; N and the facts are the same as on x86-64. Where a cross linker is missing, run `cargo check` for both targets and say so in the PR (FR-007 (SRS FR-6.3, NFR-5); SC-007 (SRS FR-5.5, NFR-5)). CI: kmr_adapters T053 cross-builds `controllers`, which calls `robot!(XmBench)` after controllers C0; no CI edit here Run 2026-11-02 on the old `actuator_count` path, reported: `cargo check --workspace --all-targets` green for both targets; `cargo build` links for aarch64 with `CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc`, armv7 has no cross linker on the dev machine. Rerun after kmr_api T021, when `robot!` emits `model::*`. Rerun 2026-11-02 after kmr_api T021 (`robot!` emits `model::*`): `cargo build --workspace --target aarch64-unknown-linux-gnu` links (`CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc`), and `cargo test -p kmr_api --target aarch64-unknown-linux-gnu --test robot_macro --test presets --test formats` passes under `qemu-aarch64-static -L /usr/aarch64-linux-gnu` (17, 1, 2 tests): the same N and facts as on x86-64. armv7: `cargo check --workspace --all-targets` green; `cargo build` does not link, the dev machine has no `arm-linux-gnueabihf` C toolchain or libc (only kernel headers under /usr/arm-linux-gnueabihf).
- [x] T052 Create kmr_api/tests/fixtures/hundred.urdf: 100 revolute joints in one chain, `j1`'s limit on each (generated once by a shell loop; commit the file, not the loop). Read it with `kmr_api::__model!(kmr_api; model = ..)` in a test module (direct: N = 100 is above the front door's caps). Measure the build-time cost on the x86-64 dev machine with `cargo build --timings -p kmr_api --tests`, with and without that module, and with bench A-6 `compile-time.sh` once it exists ("a labelled non-reference compile-time report", bench plan). Record the figure in the PR and in T057's record; the 1 s target is a starting guess (spec Assumptions). FR-021 (SRS NFR-11); SC-005 (SRS NFR-11) Done 2026-11-02, reported: kmr_api/tests/hundred.rs calls `kmr_model::model!(kmr_api; ..)`, since `kmr_api::__model` does not exist before T010 (rename it then); this is also the first build of the emission against kmr_api's `Q`, `Qd`, `Tau`. `cargo build --timings -p kmr_api --tests` after a touch of the test file, 9 runs each, rustc 1.97.1, i9-13900T: the `hundred` test crate takes 0.08 s median with the module and 0.07 s without it, so the model of 100 actuators costs about 0.01 s, within the run-to-run spread (0.06 to 0.12 s) and far under the 1 s guess. Bench A-6 `compile-time.sh` does not exist yet: no labelled report.
- [ ] T053 HUMAN: SC-004 on the NFR-1 bench, the one owner (bench T037 hands off here): bench `budget.sh` (bench T032) at the commit before the model lands and the one after, then `results compare` (bench plan A-5, bench T035, T036) against the reference set (A-10, bench T044); engine overhead per tick equal within the run-to-run spread (SC-004 (SRS NFR-1, NFR-8 (d)); FR-018 (SRS NFR-1, NFR-11)). Blocked on bench T036, T044 and on T010 (the front door calls the model)
- [x] T054 Hand-off, review-2026-10-28.md #8 and the `Bus::baud` part of #9; owners kmr_adapters T031 (`SharedPort` at resolution catches `by_id("X")` and `by_dev(".../X")` on one device) and kmr_api A3 (`Bus` wraps `Binding`; the string compare at kmr_api/src/robot.rs:258-262 and the crate-wide `allow(dead_code)` that hid `Bus::baud` go). No edit here Landed (checked 2026-11-02): kmr_adapters T031 refuses `by_id("X")` and `by_dev(".../X")` on one device as `SharedPort` at resolution; kmr_api T005 wraps `Binding` in `Bus`, and kmr_api T060 removed the crate-wide `allow(dead_code)`, so `Bus::baud` is read. The port-string compare in `finalize()` stays: it is kmr_api FR-025's check before any I/O (contracts/builder.md `DuplicateBus`), the resolution check is the backstop for aliases.
- [x] T055 Blocked on T010. Delete `actuator_count` and its helpers from kmr_model/src/lib.rs once kmr_api calls only `__model!`; `cargo build --workspace` green Done 2026-11-02: `actuator_count`, `count`, `compile_error` and the `kmr_api::__actuator_count` re-export are gone; kmr_api/tests/rx28_chain.rs reads the preset's N through `kmr_api::__model!(kmr_api; preset = Rx28Chain)`; docs/ARCHITECTURE.md, decision 0004's state table and docs/c4/kmr_model.dsl follow (`structurizr validate` green).
- [ ] T056 SC-008: `timeout -s INT 5 cargo run -p kmr_api --example complete_api_example` builds against the preset, warns that it simulates, and stops on the signal (quickstart § 1; SC-008 (SRS FR-6.1)). Review #6 is settled by the spec's SC-008 wording. Hand-off: replacing `try_lock` is kmr_api B6 (assumed, pending human: D16 "replace"); no example edit here Checked 2026-11-02: builds against the preset and warns `no bus plugin: simulating` (kmr_api T014 gates the bus line), then panics in the example's `use_time` `todo!()` before the signal: open, see kmr_api T014.
- [x] T057 [P] Draft docs/decisions/NNNN-robot-model-proc-macro.md, NNNN the next free number at commit time (Constitution V): the proc macro reads the descriptor (R1), `include_bytes!` tracking (R2), only `N`, `model::*`, `actuator!` emitted (R3), struct-of-arrays and bit sets (R4; SRS NFR-10), the name macro (R6), presets in two halves (R7), the T052 build-time figure. Agent drafts; HUMAN accepts (as bench A-8) Drafted 2026-11-02 as docs/decisions/0004-robot-model-proc-macro.md, status proposed; acceptance is the maintainer's. It also names the rejected alternatives of R8, R9 and the pending D4, D12 and SRS NFR-10 wording (T005).
- [x] T058 [P] Update docs/ARCHITECTURE.md (a model section: `robot!` calls `kmr_model` at compile time, the binary gets constants, no run-time code) and docs/ROADMAP.md (multi-robot `.ron`, SRS FR-6.5; multi-actuator joints; non-1-DOF joints; center of mass, SRS FR-8.3; lib + macro split, research R12)
- [ ] T059 Blocked on D12 (pending human) and a plan revision (research R12); do not start. kmr_ffi plan B2 needs N in a user `build.rs` from a preset name or a descriptor path, which needs kmr_model split into a library crate and a macro crate. Not planned here; this id is what kmr_ffi B2 waits on
- [ ] T060 Run quickstart.md §§ 1 to 8 in full, with § 5 as `cargo test -p kmr_model` and `cargo test -p kmr_api --test robot_macro -- limits names links` (one positional filter only; reported against quickstart § 5); then `cargo clippy --workspace --all-targets`, `cargo fmt --all --check`, `cargo test --workspace --no-fail-fast`; report the exact results
- [x] T061 Update docs/c4/kmr_model.dsl with `/speckit-ov-c4-code` (Constitution, Development Workflow) Done 2026-11-02: six C3 components in `kmr.control` (the macro runs in rustc while the control process builds), C4 code elements grouped under them; `structurizr validate` green on docs/workspace.dsl, workspace.json untouched. Update it after T055 removes `actuator_count` and T032 adds the bit sets.

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup**: T001 first. T002, T003, T005 are HUMAN and start at once. T002 blocks T004 and every
  later agent task. T003 blocks T029 only. T005 blocks nothing.
- **Foundational (T006 to T010)** after T004. T006 and T007 in parallel; T008 after T006; T009
  after T006 to T008; T010 (kmr_api B2) after T009.
- **US1 (T011 to T016)** after Foundational. T015 waits on T010.
- **US2 (T017 to T022)** after T013 (the reader holds validation steps 3 to 5). T018 is a
  hand-off to kmr_api T019 (harness). T021, T022 wait on T010.
- Every ui case (T021, T022, T026, T039, T041, T048) also waits on kmr_api T019.
- **US7 (T023 to T029)** after T013. T025, T026 wait on T010; T027 is a hand-off to kmr_api T024;
  T028 to kmr_adapters T068; T029 on T003.
- **US3 (T030 to T034)** after US2 (steps 7 and 8 give the checked, converted limits). T032 waits
  on core T025 (K2.5) and kmr_api T066; T033 on T010.
- **US4 (T035 to T039)** after US2 (the same `validate` runs after every reader). T038, T039 wait
  on T010.
- **US5 (T040, T041)** after T014. T041 waits on T010.
- **US6 (T042 to T045)** after US2 (step 8 checks masses). T044 waits on T010.
- **Phase 10**: T046 hands off to core T020–T027 (HUMAN), after the core's M1. T047 after T046;
  T048, T062 after T047.
- **Polish**: T049, T050 after US6; T051 after US1; T052 after US2; T053 after T010 and bench
  T036, T044; T055 after T010; T057 after T052; T059 waits on D12; T060, T061 last.

### Within a story

- Tests red before the code that turns them green: T012 before T013, T019 before T020, T030
  before T031, T042 before T043.
- Fixtures before the tests that read them: T011 before T015, T017 before T021, T035 before T038.
- `emit.rs` is touched by T008, T014, T031, T032, T040, T043, T049, one at a time.

### Ids other modules wait on

| This plan | Waited on by |
| --- | --- |
| T009 (`model!` exists) | kmr_api T021 (B2, "003 model!") |
| T031, T032 (limits, kind sets) | kmr_api T025, T026 (B3); kmr_adapters T058 no longer waits (core K2.5 only) |
| T043 (`LINK_NAMES`) | kmr_api B5 ("003 `LINK_NAMES`") |
| T029 (human URDF in place) | kmr_api T038 (C4), for XmBench's limits |
| T059 (library entry) | kmr_ffi B2 (kmr_ffi T011) |
| T062 (SC-009 robots) | core T030 (K2.10) |

## Parallel opportunities

- T006 and T007 (model.rs, input.rs).
- T011, T017, T035 (fixtures) beside the code of their phase; T012 beside T011.
- kmr_api T019 (harness) beside this plan's T017 and T019.
- HUMAN T003, T005, T053 and core T020–T027 (T046) beside every agent phase.
- T049, T050, T057, T058 together.

```text
Foundational:  (T006 ‖ T007) → T008 → T009 → T010 (kmr_api T021)
US1:           (T011 ‖ T012) → T013 → T014 → T015 → T016
US2:           (T017 ‖ T019) → T020 → T021 → T022   (kmr_api T019 before T021)
US3:           T030 → T031 → T033;  core T025 → kmr_api T066 → T032
HUMAN:         T002, T003, T005, T053; core T020–T027 (T046)
```

## Implementation strategy

### MVP: User Story 1

1. T001, then wait for T002 (D1 A: the first motor, or a waiver).
2. T004, Foundational, then US1 on URDF: N and the order are constants.
3. kmr_api B2 (T010 hand-off) wires the front door; T015 proves it end to end.

### Incremental

4. US2 (errors), US7 (presets; real data at T003, T029).
5. US3 (limits; kind sets after core K2.5), US4 (`.ron`, JSON).
6. US5 (names), US6 (links).
7. Phase 10 after the core's const-generic N; Polish; the PR from `agent/<topic>` into the human
   branch, `agentic` label.

## Review findings mapped

From [review-2026-10-28.md](review-2026-10-28.md).

| # | Sev | Tasks |
| --- | --- | --- |
| 1 | High | T013, T019, T020 (validation 1 to 10), T021 |
| 2 | High | T023, T024, T028; kmr_api B2 through T010 |
| 3 | Med | T006 (step 10 in `validate` first), T021 |
| 4 | Med | T007, T009, T019 |
| 5 | Med | T007, T009, T024 |
| 6 | Med | spec SC-008 already reworded; T056; kmr_api B6 |
| 7 | Low | T010 (kmr_api B2) |
| 8 | Low | T054 (kmr_adapters T031, kmr_api A3) |
| 9 | Low | T009 (extension message), T012, T013 (unnamed joint or link), T054 (`Bus::baud`) |
| 10 | — | T011 (fixtures with limits), T012, T019 (kmr_model tests), T018, T021 (`trybuild`), T015 (order check), T022 (doctest replaced) |

## Notes

- Commits: AGENTS.md rule (b) and the maintainer's decision: no AI or co-author trailer. The
  constitution's Principle 0 asks for one. This conflict is the human's (T005); until then
  commits follow the maintainer.
- Delivery order (Constitution I, SRS § 8) holds under D1 A: every agent task here waits on T002.
- An agent that reaches a HUMAN task, or a task blocked on one, stops and hands it over.
- Commit per task or per group, Conventional Commits, scope `model`, or `api` for the tests and
  fixtures under `kmr_api/tests/`.

## Analysis 2026-10-28 (cross-module)

- 2026-10-28: T032 waits on core T025 and kmr_api T066 (new; breaks the kmr_api T025 → kmr_adapters T058 → T032 cycle); T010 owner kmr_api T021, which now names the `__model!` switch; T018 hand-off to kmr_api T019 (one harness, one `trybuild` dev-dependency; T004 drops its kmr_api line; trybuild path note moved to T021); T027 hand-off to kmr_api T024; T028 hand-off to kmr_adapters T068; T046 one-line hand-off to core T020–T027; T048 owns the `three.urdf` pass case (kmr_api T033 hands off); T053 owns SC-004 (bench T037 hands off); new T062 for core SC-009's N = 1, 4, 10, 64 robots.
