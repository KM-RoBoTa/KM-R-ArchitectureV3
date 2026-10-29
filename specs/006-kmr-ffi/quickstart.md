# Quickstart: validating kmr_ffi

Run guide for each step of [plan.md](plan.md). Types in [data-model.md](data-model.md); surfaces
in [contracts/](contracts/). Commands from the git root unless said otherwise. Nothing here
needs hardware.

## 0. Prototype (exists today)

```sh
cd /tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/kmr_ffi-plan
cargo test --offline -p muser -- --nocapture       # 6 passed; prints the layout per N
KMR_PROTO_N=5 cargo check --offline -p muser       # error[E0080] ... built for another N
cargo build --offline -p mlint                     # warning: Q ... not FFI-safe
```

## 1. Types and call (steps A2 to A4)

```sh
cargo test -p kmr_ffi
cargo clippy -p kmr_ffi --all-targets
```

Expected: error status clears the buffer; statuses 0, 1, 2 map; `ReadView::<65, 4>::default()`
fails to compile; no clippy warning.

## 2. C and C++ (steps B1 to B4)

User files for the C example (FR-024 (SRS FR-10.4)): `Cargo.toml`, `build.rs`, `src/main.rs`,
`src/swing.c`.

```toml
# Cargo.toml
[dependencies]
kmr_api = { workspace = true, features = ["ffi"] }
[build-dependencies]
kmr_api = { workspace = true, features = ["ffi-build"] }
```

```sh
cargo test -p kmr_ffi_example_c          # layout probes N = 1, 3, 4, 10, 64; C error path
cargo test -p kmr_ffi_example_cpp        # typed wrappers; throwing body → error, nothing applied
```

SC-005: change the preset in `build.rs` only, then `cargo check -p kmr_ffi_example_c`.
Expected: `kmr_ffi: C sources were built for another N` at the `foreign!` line.

## 3. Glue (steps C2 to C4, after HUMAN C1)

```sh
cargo test -p kmr_api --features ffi ffi
```

| Test | Expected |
|---|---|
| SC-001 C twin, inline and inline-with-context, 1000 ticks | same Desired bits and masks as the Rust twin |
| SC-004 dummy field without its FFI line | `E0004` at the `FieldId` match |
| SC-006 10^8 scripted ticks | `time.tick` and `elapsed_ns` exact |
| stale actuator 1 | bit 1 of `slot[0].flags.stale`, last good value, `age[1] > 0` |
| no-hardware adapter | `samples == 0` |
| status 2 | writes applied, emergency stop raised, first reason kept |

## 4. Python and Lua (steps D1 to D3)

```sh
cargo test -p kmr_api --features ffi-python,ffi-lua ffi
cargo run -p kmr_ffi_example_python      # no hardware: simulation warning, then runs
cargo run -p kmr_ffi_example_lua
```

Expected: SC-002 (same masks, values equal after rounding to `f32`); a script that does not load
returns `ForeignError::Load` from `main` before `run()`; an exception applies nothing (SC-007).

## 5. Workspace gates

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets
cargo test --workspace --no-fail-fast
cargo build -p kmr_api                    # no ffi feature: kmr_ffi not compiled
```

## 6. Bench and onboarding (HUMAN)

- SC-003: the bench's empty C forwarder against an empty Rust controller at 1 kHz, NFR-8 (d).
- SC-008: FR-10.4 run, C++ and Python engineer against the ROS2 baseline (D14).
