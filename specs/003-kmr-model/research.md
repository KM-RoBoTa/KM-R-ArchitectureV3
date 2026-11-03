# Research: kmr_model

Each entry: decision, why, what else was weighed. SRS ids next to spec ids.

## R1. Who reads the descriptor

- **Decision**: a function-like proc macro in `kmr_model`, called only through
  `kmr_api::robot!` in the user's `main.rs`. The path is relative to the calling crate's
  `Cargo.toml` (`CARGO_MANIFEST_DIR`).
- **Why**: FR-004 (SRS FR-6.1): no config file, no user `build.rs`, N fixed at compile time.
  Proven on branch `agent/robot-macro` (`actuator_count`).
- **Rejected**: user `build.rs` (runs after kmr_core is built, cannot set its N; SRS FR-6.1
  forbids it); workspace config line (SRS FR-6.1: no config file); env var (per machine, breaks
  FR-007).

## R2. Rebuild when the descriptor changes

- **Decision**: the expansion carries `const _: &[u8] = include_bytes!("<absolute path>");`.
  Cargo then tracks the file through rustc's dep-info.
- **Why**: FR-021 (SRS NFR-11). `proc_macro::tracked_path` is still unstable on rustc 1.97.
- **Rejected**: a `build.rs` with `rerun-if-changed` (forbidden, R1).
- **Note**: the bytes are not linked into the binary; the const is unused and dropped.
- **Limit**: a proc macro runs on every compile of its calling crate, so any edit to `main.rs`
  re-reads the descriptor. Only a fully fresh build skips it. Read time is measured (FR-021,
  SC-005).

## R3. How N and the facts reach the engine

- **Decision**: the proc macro emits, at the call site, only `N`, a `model` module of `const`
  arrays, and an `actuator!` name macro. kmr_api's `macro_rules!` front door emits the `State`
  and `Robot` aliases (`Robot = kmr_api::Robot<__Spec>`, kmr_api research R2), `__Spec` and the
  checks; no item is emitted by both. N reaches kmr_core as a const generic. Until kmr_core has
  it (HUMAN, core plan M2), the front door asserts `N == kmr_core::state::JOINTS` at compile
  time.
- **Why**: FR-004, FR-018 (SRS FR-6.1, NFR-1): constants, zero per-tick cost. kmr_core stays
  human-only (Constitution IV).
- **Rejected**: kmr_core depending on kmr_model. A proc-macro crate exports only macros, and the
  const generic makes the dependency useless. See R11 for the SRS wording this contradicts.

## R4. Shape of the facts (NFR-10)

- **Decision**: one array per fact, indexed by actuator: `NAMES: [&str; N]`,
  `LOWER: [Q; N]`, `UPPER: [Q; N]`, `SPEED: [Qd; N]`, `EFFORT: [Tau; N]`. Kinds and "no
  position limit" are bit sets `[u64; MASK_WORDS]`, kmr_core's mask width through kmr_api
  (1 under D4 A): `PRISMATIC`, `CONTINUOUS` (revolute = neither). Each is emitted as a const
  block from `[0u64; MASK_WORDS]` that sets the words it needs, so the macro needs not know the
  constant; N above 64 · `MASK_WORDS` fails the N ≤ 64 assert first.
- **Why not `[u64; W]`, `W = N.div_ceil(64)`**: kmr_api copies the sets into `Spec<N>`
  associated consts, and `[u64; N.div_ceil(64)]` in a generic context does not compile on
  stable. `Table::check_kinds(self, [u64; MASK_WORDS]) -> Self` does (proved 2026-10-28, kmr_api
  research R9). `LOWER`/`UPPER` keep a `Q(0.0)` placeholder where `CONTINUOUS` is set; kmr_api
  maps those to `None` in the core's `Limit` (kmr_api plan B3). Links: `LINK_NAMES`, `LINK_PARENT:
  [Option<u16>; L]`, `LINK_MASS: [f32; L]` (kg), `LINK_COM: [[f32; 3]; L]` (m).
- **Why**: FR-013, FR-020 (SRS NFR-10): struct-of-arrays, booleans out-of-band, encodings
  instead of an enum. The emitted code needs no new type: only `Q`, `Qd`, `Tau` (public tuple fields,
  so const-constructible), `u64`, `f32`, `&str`.
- **Rejected**: a `Kind` enum and an `Actuator` struct per index (array-of-structs, padding, and
  a type that must live in a crate kmr_core can see).
- **Open**: link COM stays in metres; SRS FR-2.2 sets units for actuators only. Roadmap
  (SRS FR-8.3), so not blocking.

## R5. Units

- **Decision**: revolute limits as written (rad, rad/s, N·m). Prismatic: URDF metres × 1000 to
  mm and mm/s; effort stays N.
- **Why**: FR-012, FR-017 (SRS FR-2.2, decided 2026-10-27).
- **Check**: the converted value must still be finite and fit `f32` (edge case list).

## R6. Name lookup at compile time

- **Decision**: `robot!` emits `macro_rules! actuator` with one arm per name, expanding to the
  index literal; a catch-all arm calls `compile_error!(concat!("kmr: no actuator named ", $n))`.
  `const ELBOW: usize = actuator!("elbow");`
- **Why**: FR-015 (SRS FR-6.1, FR-2.4): an unknown name fails the build and names it.
- **Rejected**: a `const fn index(&str) -> Option<usize>` plus an unwrap at the call site (a
  run-time call could panic; SRS NFR-2).
- **Verified 2026-10-28** (scratch, rustc 1.97.1): the emitted macro works at the root and as
  `crate::actuator!` from a child module; `actuator!("wrist")` fails with
  `kmr: no actuator named wrist`. Weak spot: the error points at the `robot!` line, not at the
  bad lookup. Acceptable for now; the message names the actuator.

## R7. Presets

- **Decision**: a preset has two halves under one name. The model half (a URDF) ships inside
  `kmr_model` under `presets/` and is embedded with `include_str!` when the proc macro builds.
  The hardware half (ids, baud, modes) stays in `kmr_adapters::presets`. `robot!(XmBench)`
  reads the model half and checks the hardware half's count against N with an array-type check,
  `const _: &[Actuator; N] = &__TABLE.actuators;` (kmr_api FR-007 owns it; rustc's E0308 names
  both counts).
- **Why**: SRS module kmr_model ("presets"), FR-6.1 (hardware data from the preset).
- **Change**: today `kmr_adapters::presets::Preset::N` gives N. It flips: the model gives N, the
  hardware table is checked against it (spec edge case "hardware declaration … than N").
- **HUMAN input**: the real URDF of the XM430 bench chain (limits, masses). Agents do not
  invent physical limits; until it exists, the preset is a test fixture marked as such.

## R8. Parsers

- **Decision**: URDF with `roxmltree` (in use). `.ron` with `ron` + `serde`; JSON with
  `serde_json`. One serde schema for both (contracts/descriptor-ron.md). URDF first (P1), `.ron`
  and JSON second (P2, User Story 4).
- **Why**: FR-001 (SRS FR-6.1: URDF default, `.ron` recommended, JSON supported). Positions for
  errors: roxmltree `text_pos_at`, ron and serde_json report line and column.
- **Rejected**: `urdf-rs` (pulls `serde-xml-rs`, drops source positions, so FR-008 messages lose
  the line).

## R8b. Format interface

- **Decision**: a private `trait Reader { fn read(text: &str, file: &Path) -> Result<Model, ModelError>; }`
  inside `kmr_model`, one impl per format, chosen by extension through a `match`, no trait
  object. A later plugin adds an impl (FR-025, SRS FR-11.3).
- **Why**: build-time only, never public; a proc-macro crate exports only macros anyway.

## R9. Macro plumbing

- **Decision**: `proc-macro2`, `quote`, and `syn` with `default-features = false`,
  `features = ["parsing", "proc-macro"]`. Errors are `syn::Error` spanned on the path literal or
  preset ident, so the compiler underlines the user's `robot!` line.
- **Why**: FR-008 (SRS FR-6.4). Float literals through `Literal::f32_suffixed` avoid hand
  formatting.
- **Rejected**: the current string formatting (no spans, fragile float text).
- **Cost**: NFR-11 build time. `syn` without `full` is small; measured in SC-005.

## R10. Tests for compile errors

- **Decision**: `trybuild` compile-fail cases in `kmr_api/tests/ui/`, one per edge case, with
  `.stderr` snapshots. Pure parse functions get unit tests inside `kmr_model`.
- **Why**: SC-002 needs every bad descriptor to fail with the right message.
- **Rejected**: `compile_fail` doctests (no message check).

## R11. Conflicts to report (SRS and constitution win; human decides)

1. SRS NFR-10 says "kmr_model is a dependency of kmr_core". With R3 it is not: kmr_core needs
   nothing from it. Wording for the human.
2. SRS delivery order: "nothing below a line starts before the line above moves a motor". Step 1
   has not moved a motor. This plan is design only; implementation waits, or the human waives
   it. Code already sits uncommitted on `agent/robot-macro`.
3. Constitution Principle 0 requires agent trailers. The human decided on 2026-10-27: no
   trailers, ov `none` mode. The constitution needs a human amendment.
4. `kmr_adapters/src/robot.rs` sizes its table with `JOINTS`; its data moves to the preset
   hardware half (R7), checked against the model's N, and kmr_adapters plan B5 deletes the
   file.

## R12. A kmr_model entry for build scripts (decided 2026-11-03: D12 A)

- **Need**: kmr_ffi's build helper (kmr_ffi plan B2, D12 A) must get N in a user `build.rs`
  from a preset name or descriptor path, the same N `robot!` gets.
- **Blocker**: a proc-macro crate exports only macros, so a build script cannot call
  kmr_model. A library entry needs kmr_model split into a lib crate (parse, validate, `Model`)
  and a thin macro crate over it. The coordination page lists the split for SRS FR-8.3 and
  FR-11.3 too.
- **Status**: decided 2026-11-03, D12 A, taken for the maintainer under the goal "don't ask"
  ([decision 0009](../../docs/decisions/0009-kmr-model-library-split.md)). Built by T059: the
  library is `kmr_model_core` (descriptor, preset, readers, checks, `Robot`, `Robot::n()`); the
  proc-macro crate keeps the name `kmr_model`, so kmr_api's dependency and `__model!` path do
  not change. kmr_ffi's build helper (its T011) depends on `kmr_model_core`.

## R13. Coordination log

- 2026-10-28 coordination: the proc macro emits only `N`, `model::*` and `actuator!`; the
  `State`/`Robot` aliases, `__Spec` and the checks come from kmr_api's front door (R3, plan,
  data-model, contracts/robot-macro.md).
- 2026-10-28 coordination: kind sets are `[u64; MASK_WORDS]`, not `[u64; ⌈N/64⌉]` (R4).
- 2026-10-28 coordination: continuous joints: the emitted `Q(0.0)` placeholder stays, kmr_api
  maps it to `None` (R4, data-model).
- 2026-10-28 coordination: `kmr_adapters/src/robot.rs` is deleted by kmr_adapters B5, not kept
  (plan, R11).
- 2026-10-28 coordination: R12 records the lib + macro split needed by kmr_ffi B2.
