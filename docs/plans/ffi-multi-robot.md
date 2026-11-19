# Plan: C and C++ controllers for several robots in one crate

Branch `feature/ffi-multi-robot`, stacked on `feature/geckoleg-pinocchio`.

## Problem

A crate's build script can compile C and C++ controllers for one robot only:

- `Build::compile` writes one `kmr.h` (one `KMR_N`) and one `kmr_ffi_abi.rs` into `OUT_DIR`,
  and names its static libraries `kmr_ffi_c` / `kmr_ffi_cpp`. A second call overwrites the
  first.
- `c!` reads that one ABI file and asserts its N equals `robot!`'s N.

So `controllers/` holds C/C++ controllers for the snake (N = 22) and nothing else; the GeckoLeg
example (N = 3) had to become its own crate. Rust examples have no such limit: each file calls
`robot!` for its own robot.

## Goal

`controllers/build.rs` calls the helper once per robot; each example names its robot with
`robot!` and its functions with `c!`, unchanged. A function built for another robot still fails
the build (SRS FR-10.2, FR-6.1), with a message, not undefined behaviour.

```rust
// controllers/build.rs
build::model("models/ax_12_12act_an_rx28_10act.urdf").compile(&["c/clap.c", "cpp/clap.cpp"])?;
build::model("models/geckoleg.urdf").compile(&["cpp/geckoleg.cpp"])?;
```

## Design

**One group per `compile` call.** Group `k` (0, 1, … in call order, a counter in the build
script's process) gets:

- its headers in `OUT_DIR/kmr/<k>/` (`kmr.h`, `kmr.hpp`), the only kmr include dir of its
  compile, so `#include "kmr.h"` stays as is in every source;
- its libraries `kmr_ffi_c_<k>` and `kmr_ffi_cpp_<k>`.

**The check is per function, not per build.** The safety property is "this function was
compiled for this N". With several groups linked into one binary, an N per build no longer says
that. So after compiling a group the helper lists the global symbols its libraries define, and
`kmr_ffi_abi.rs` becomes a table, rewritten whole at each call:

```rust
&[("move_on_clap", [22, 8, 4, 1, 1024]), ("walk_cpp", [3, 8, 4, 1, 1024]), …]
```

`c!(walk_cpp)` looks its own name up in the table at compile time (a `const fn`) and fails with
`error[E0080]` and one of three messages:

1. the name is in no group: build.rs compiled no such function (today: a linker error);
2. its N is not `robot!`'s N: built for another robot;
3. its sizes are not this kmr's: rebuild.

A name defined by two groups is an error of the helper (`Error::DuplicateSymbol`): the linker
would pick one silently.

**Listing symbols.** `nm -g --defined-only` (POSIX `-P` output) on each library: the `nm` next
to the archiver `cc` picked (`aarch64-linux-gnu-ar` → `aarch64-linux-gnu-nm`), `NM` overrides,
then `llvm-nm`, then `nm`. No new dependency. A missing tool is `Error::NoSymbolTool` with the
fix in its text.

**What users see change.** Nothing in `c!`, `KMR_CONTROLLER`, `kmr.h` or `kmr.hpp`. `compile`
may be called several times. Two new `Error` variants. The format of `kmr_ffi_abi.rs` and where
the headers sit in `OUT_DIR` are private to kmr_ffi and kmr_api.

## Not in this plan

- `Build::include` / `Build::flag` (row of 2026-11-19 in `docs/deferred.md`): the Pinocchio
  group still gets its include dirs through `CXXFLAGS`.
- Reading symbols with the `object` crate instead of `nm`: a dependency, the maintainer's call.
  One private function to swap. Deferred row added.

## Tasks

| id | who | what |
|----|-----|------|
| M1 | Opus | kmr_ffi `build.rs`: groups, per-group dirs and library names, symbol listing, the table, the two errors; unit tests |
| M2 | Opus | kmr_api `c!`: the lookup and its three messages; `wrong_n` fixture kept green; a new fixture with two robots (right pairs build, crossed pair fails, unknown name fails) |
| M3 | Opus | move the Pinocchio example into `controllers/` (feature `pinocchio`, `cpp/geckoleg.cpp`, `examples/geckoleg_pinocchio.rs`, tests), drop the crate `kmr_ffi/examples/pinocchio`, update `scripts/build-pi.sh` |
| M4 | Sonnet | docs: `specs/006-kmr-ffi/contracts/c-abi.md`, the module docs of `kmr_api::ffi::c`, `controllers/README.md`, root `README.md`, `docs/deferred.md` rows |

M1 and M2 share the table's format: one agent, in order. M3 and M4 follow, in parallel.

## Oracle

`cargo test --workspace --no-fail-fast`, `cargo clippy --workspace --all-targets -- -D warnings`,
`cargo fmt --all --check`, the `no_std` core build, and with Pinocchio in the environment the
`controllers` tests and both dry runs (`clap_cpp` builds, `geckoleg_pinocchio --dry-run`).
