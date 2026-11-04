# 0004 — The robot model: a proc macro reads the descriptor at compile time

Status: **proposed**, drafted by an agent, waiting for the maintainer's
decision. Two points rest on pending decisions (D4, D12, see
[Pending](#pending)).

The research behind each point is in
[`specs/003-kmr-model/research.md`](../../specs/003-kmr-model/research.md)
(R1 to R9, R12).

## The question

The engine must know the robot's actuator count N, their order, names and
limits, and its links, before it builds (SRS FR-6.1, NFR-10). Four things have
to be fixed:

- who reads the descriptor, and when;
- how a change to the descriptor rebuilds the robot;
- what reaches the binary, and in which shape;
- where a preset's data lives.

## Current state

| Piece | State |
|---|---|
| `kmr_model::model!`: input, URDF / `.ron` / JSON readers, validation, errors | written |
| Emitted `N`, `model::NAMES`, `LOWER`, `UPPER`, `SPEED`, `EFFORT`, `LINK_*`, `actuator!` | written |
| Emitted `model::PRISMATIC`, `CONTINUOUS` | not written: wait on `MASK_WORDS` public (core K2.5, human) |
| Presets `XmBench`, `Rx28Chain` (model half) | placeholders, marked as test fixtures; the bench URDF is human data (T003) |
| `kmr_api::robot!` calls `model!` | written (kmr_api T021), through the hidden `kmr_api::__model!`; `actuator_count` is gone (T055) |
| `robot!(model = ..)` | URDF, `.ron` and JSON |
| `robot!(Preset)` N | from the model half; the hardware half is checked to cover `0..N` |
| `actuator!` for users | written, emitted at the `robot!` call site |
| Front door N ≤ 64 cap | written (kmr_api T021, D4 A, assumed, pending human); its ui case waits on kmr_api T033 |

Below, a point the table marks as not written is the planned design, not
the branch's behaviour.

## Decision proposed

1. **A function-like proc macro reads the descriptor.** `kmr_model::model!`,
   reached only through `kmr_api::robot!` in the user's `main.rs`. The path is
   relative to the calling crate's `Cargo.toml`. URDF, `.ron` and JSON go
   through one private `Reader` trait, one impl per format, picked by
   extension, then one `validate` (R1, R8, R8b).
2. **`include_bytes!` tracks the file.** The expansion holds
   `const _: &[u8] = include_bytes!("<absolute path>");`, so Cargo rebuilds the
   caller when the descriptor changes (R2).
3. **Only constants are emitted.** `N`, a `model` module of `const` arrays and
   an `actuator!` name macro. kmr_api's front door adds `State`, `Robot`,
   `__Spec` and the checks; no item comes from both (R3).
4. **Struct of arrays, bit sets for kinds.** One array per fact, indexed by
   actuator, in SRS FR-2.2 units: `LOWER`, `UPPER: [Q; N]`, `SPEED: [Qd; N]`,
   `EFFORT: [Tau; N]`. Kinds are to be `[u64; MASK_WORDS]` bit sets,
   `PRISMATIC` and `CONTINUOUS`, the layout of `kmr_core::hal::Mask` (not
   emitted yet). A continuous joint keeps a `Q(0.0)` position placeholder; its
   bit is to say there is no limit. Links:
   `LINK_NAMES`, `LINK_PARENT: [Option<u16>; L]`, `LINK_MASS`, `LINK_COM`
   (R4, R5).
5. **Names by a generated macro.** `actuator!("j3")` expands to the index
   literal; an unknown name is `compile_error!` naming it (R6).
6. **A preset has two halves under one name.** The model half (a URDF) ships in
   `kmr_model/presets/` and is embedded with `include_str!`; the hardware half
   (ids, baud, modes) stays in `kmr_adapters::presets`. Planned with kmr_api
   T021: the model gives N and the hardware table is checked against it (R7);
   today `robot!` still takes N from the hardware half.

## Defence

### Real-time performance

- Nothing runs at run time. The binary gets `const` arrays of `f32`, `&str`
  and `Option<u16>` (`u64` with the kind sets): no heap, no `std` item, no
  `dyn`, no lock, no parse. A token walk of the emission checks this (T049).
- Struct of arrays: a loop over one fact touches one contiguous array; the
  "no position limit" flag is one bit, not an `Option` beside every limit
  (SRS NFR-10).
- The tracking constant is unnamed and never read, so rustc has no reason to
  link the descriptor's bytes; no binary has been inspected for it.
- Build-time cost, measured (T052): 100 revolute joints in one chain cost
  about 0.01 s on the x86-64 dev machine (i9-13900T, rustc 1.97.1). The test
  crate takes 0.08 s median with the model, 0.07 s without, over 9 runs each;
  the run-to-run spread is 0.06 to 0.12 s. The 1 s target of the spec is met
  by far. Not measured on the bench: `compile-time.sh` (bench A-6) does not
  exist yet.

### Quality

- **A bad descriptor does not build.** Every edge case of the spec is a
  compile error spanned on the path literal or preset name of the `model!`
  call, naming the file, the line where one is known, the element and the
  reason (SRS FR-6.4, NFR-2); the user's `robot!` line shows it after
  kmr_api T021. The crate forbids `unwrap`, `expect` and `panic`.
- **One source of N.** The descriptor fixes N; no config file, no
  environment variable, no user `build.rs` (SRS FR-6.1). Every machine and
  every target reads the same file and gets the same constants. For a preset
  this holds after kmr_api T021 (see Current state).
- **Units are fixed once.** Prismatic metres become millimetres in the macro;
  every converted value is checked finite and within `f32`.
- **Literals, not text.** Values go through `proc_macro2::Literal`, never
  through formatted strings.

### Developer experience

- One line names the robot: `robot!(XmBench)` or `robot!(model = "arm.urdf")`.
- `actuator!("elbow")` reads as the name the URDF gives; a typo fails the
  build and names it. The name matches as a plain string literal only: a raw
  or escaped literal fails the build, and the message says so.
- The descriptor format is the user's choice: URDF as exported, or `.ron` /
  JSON in one schema that refuses unknown fields (through `robot!` after
  kmr_api T021).

### Alternatives rejected

- **A user `build.rs`.** It runs after kmr_core is built, so it cannot set N,
  and SRS FR-6.1 forbids it.
- **A config line or an environment variable.** No config file (SRS FR-6.1);
  an environment variable is per machine.
- **`kmr_core` depending on `kmr_model`.** A proc-macro crate exports only
  macros, and with N a const generic the core needs nothing from it.
- **An `Actuator` struct and a `Kind` enum per index.** Array of structs,
  padding, and a type that must live where the core can see it.
- **`[u64; N.div_ceil(64)]` kind sets.** Does not compile in a generic context
  on stable; `MASK_WORDS` does.
- **A `const fn index(&str) -> Option<usize>`.** Its unwrap at the call site
  could panic at run time.
- **`urdf-rs`.** Drops source positions, so errors lose the line.
- **`proc_macro::tracked_path`.** Unstable on rustc 1.97.

## Pending

- **D4** (N ≤ 64, `MASK_WORDS` = 1) is assumed, option A. Another answer
  is to change the front door's cap only (not written yet); the planned sets
  are sized by `MASK_WORDS`, so they fit any number of words.
- **D12** (a library entry for kmr_ffi's build helper) is open. If it is
  needed, `kmr_model` splits into a library crate and a thin macro crate
  (R12); the emitted facts do not change.
- **SRS NFR-10** says "kmr_model is a dependency of kmr_core"; with point 3
  it is not. The wording is the maintainer's (T005).
