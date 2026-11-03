# 0009 — kmr_model splits into a library and a thin proc macro (D12 A)

Status: **decided** 2026-11-03, D12 option A, taken by the orchestrating agent
for the maintainer under the goal of 2026-11-03 ("implement all remaining
tasks, don't ask me stuff"); open to the maintainer's review. Built by
kmr_model T059, used by kmr_ffi T011. Research:
[`specs/003-kmr-model/research.md`](../../specs/003-kmr-model/research.md)
R12, [`specs/006-kmr-ffi/research.md`](../../specs/006-kmr-ffi/research.md)
R4; the FFI boundary is [0004](0004-ffi-boundary.md), the macro is
[0004](0004-robot-model-proc-macro.md).

## The question

C and C++ sources are compiled in the user's `build.rs`, before `robot!`
expands, so they need the robot's N there (D12). Three options were on the
coordination page:

- **A**: the user's `build.rs` compiles foreign code only, names the robot
  again, and `foreign!` checks at compile time that both N agree.
- **B**: a separate header crate per robot.
- **C**: a pointer plus a length, checked at run time.

A needs a build script to read the descriptor. `kmr_model` was a proc-macro
crate, and a proc-macro crate exports macros only: a build script cannot call
it.

## Decision

**D12 A.** `kmr_model` is split in two crates:

| Crate | Kind | Holds |
|---|---|---|
| `kmr_model_core` | library | presets, the URDF/RON/JSON readers, validation, `Robot` (`CheckedActuator`, `CheckedLink`, `Kind`), `Robot::n()`; entries `preset(name)`, `descriptor(path)`, `parse(text, shown)` |
| `kmr_model` | proc macro | the `model!` input parser and the emitter, over `kmr_model_core` |

**Names.** The proc-macro crate keeps the name `kmr_model`: kmr_api depends
on it and re-exports `kmr_model::model` as `__model`, so nothing downstream
changes. The new library is `kmr_model_core`, after the `*_core` naming the
workspace already uses for "the crate the thin one stands on". Presets move
with the readers to `kmr_model_core/presets/`, since the library embeds them.

**Messages.** The library returns the one-line `kmr: ..` text `robot!` shows;
the macro spans it on the path or the preset name. kmr_ffi's build helper
returns the same text, so an unknown preset reads the same in `build.rs` and in
`main.rs` (kmr_ffi FR-015).

**kmr_ffi's dependency.** kmr_ffi research R9 gives kmr_ffi no kmr
dependency, to keep kmr_api → kmr_ffi free of a cycle. `kmr_model_core`
depends on no kmr crate, so kmr_ffi's `build` feature may depend on it without
a cycle; the engine stays out of kmr_ffi. The History depth is not in the
library: kmr_ffi's helper takes it as an argument and kmr_api's `ffi-build`
wrapper (kmr_api G1) passes the core's `HISTORY_DEPTH`.

## Defence

### Real-time performance

None at run time: both crates run on the build machine; the binary still
receives constants only.

### Quality

- One reader and one validation for `robot!` and for `build.rs`: the N the C
  sources are built for is computed by the same code as the N of `robot!`.
- A mismatch (`build.rs` names another robot) is still caught, by
  `foreign!`'s const check (kmr_ffi T012, T017).
- `kmr_model_core` is a plain library: its tests run without a proc-macro
  harness.

### Developer experience

Users never name either crate: `robot!` is `kmr_api`'s, and the build helper
is `kmr_api::ffi::build` under `ffi-build`.

### Alternatives rejected

- **B, a header crate per robot**: one more crate per robot for the user, and
  the helper moves, not the problem.
- **C, pointer plus length**: the N check moves to run time, and every C body
  carries a length it must check.
- **Rename the macro crate (`kmr_model_macros`) and give the library the name
  `kmr_model`**: the conventional pair, but it changes kmr_api's dependency,
  its re-export and every doc that names the macro, for no user gain.
- **Keep one crate, with a `[lib]` and a separate proc-macro target**: Cargo
  allows one library target per package.

## Consequences

- SRS FR-6.1 ("no build.rs in the user's crate") needs the exception of
  0004: a `build.rs` that compiles foreign sources only.
- kmr_ffi T011 builds on `kmr_model_core`; kmr_ffi's `build` feature pulls it
  (and its parsers) into the user's build dependencies only.
