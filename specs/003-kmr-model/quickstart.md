# Quickstart: validate kmr_model

Run from the git root. Each step names the spec criterion it proves.

## 1. The workspace still builds and the example runs (SC-008)

```sh
cargo build --workspace
cargo run -p kmr_api --example complete_api_example
```

Expected: builds; the example starts with `robot!(XmBench)`, warns that it simulates (no bus
plugin bound to hardware yet), and stops on Ctrl-C.

## 2. N and order from a descriptor (SC-001, User Story 1)

```sh
cargo test -p kmr_api --test robot_macro
```

Expected: the 4-actuator, the 6+2-fixed and the continuous fixtures give N = 4, 6, 1 and
`model::NAMES` in file order.

## 3. Every bad descriptor fails the build (SC-002, User Story 2)

```sh
cargo test -p kmr_api --test ui
```

Expected: each `tests/ui/*.rs` case fails to compile and its stderr matches the `.stderr`
snapshot (message shapes: [contracts/robot-macro.md](contracts/robot-macro.md#errors-all-compile-errors-srs-fr-64)).

## 4. Three formats, one model (SC-003, User Story 4)

```sh
cargo test -p kmr_api --test formats
```

Expected: the reference robot as `.urdf`, `.ron` and `.json` emits identical `model::*` facts.

## 5. Limits, units, names, links (User Stories 3, 5, 6)

```sh
cargo test -p kmr_model
cargo test -p kmr_api --test robot_macro limits names links
```

Expected: a prismatic limit of 0.1 m reads `Q(100.0)`; `actuator!("elbow")` equals its file
index; link masses read back.

## 6. no_std and cross-builds (SC-006, SC-007)

```sh
cargo test -p kmr_api --test no_std_caller   # a #![no_std] fixture crate calling robot! (SC-006)
cargo build --workspace --target aarch64-unknown-linux-gnu
cargo build --workspace --target armv7-unknown-linux-gnueabihf
```

Expected: builds with no hardware attached. The no_std caller proves the emitted facts need no
`std`; a no_std firmware getting N is out of this version (kmr_api is std).

## 7. Rebuild and build time (SC-005)

```sh
cargo test -p kmr_api --test robot_macro --no-run
cargo test -p kmr_api --test robot_macro --no-run -v 2>&1 | grep -c Fresh   # unchanged: nothing recompiled
touch kmr_api/tests/fixtures/four.urdf
cargo test -p kmr_api --test robot_macro --no-run -v 2>&1 | grep Compiling  # edited: the test crate recompiles
cargo build --timings -p kmr_api --tests   # with the 100-joint fixture
```

Expected: a fresh build recompiles nothing; editing it rebuilds the caller; the 100-joint
fixture adds under 1 s (starting guess, replaced by the first bench measurement).

## 8. Lints

```sh
cargo clippy --workspace --all-targets
cargo fmt --all --check
```
