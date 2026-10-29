# Contract: Rust surface, through `kmr_api::ffi`

Users reach all of it through `kmr_api` (FR-001 (SRS FR-10.1)). `kmr_ffi` items are re-exported
under `kmr_api::ffi` when the `ffi` feature is on; build-script items under `ffi-build`. Names
are the contract; bodies are the implementation's.

## Features

| Crate | Feature | Brings |
|---|---|---|
| `kmr_api` | `ffi` | `kmr_api::ffi::{foreign, call, Status, ForeignError, ReadView, WriteBuf, TimeView}` |
| `kmr_api` | `ffi-build` | `kmr_api::ffi::build` for `build.rs` |
| `kmr_api` | `ffi-python` | `kmr_api::ffi::python::Bridge` |
| `kmr_api` | `ffi-lua` | `kmr_api::ffi::lua::Bridge` |

All off by default. A build without them compiles no FFI code (kmr_api FR-072).

## Build script (C/C++ only)

```rust
// build.rs
fn main() -> Result<(), kmr_api::ffi::build::Error> {
    kmr_api::ffi::build::robot("XmBench")      // or .model("models/arm.urdf")
        .compile(&["src/swing.c"])            // headers into OUT_DIR, cc, ABI file
}
```

| Error | When |
|---|---|
| `UnknownPreset`, `Descriptor(..)` | robot not found or descriptor invalid; same message as `robot!` |
| `Compile(..)` | the C/C++ compiler failed |

D12 A ([research.md](../research.md) R4).

## Declaring foreign functions

```rust
kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn swing; fn other; }   // once per crate
```

Expands to an `unsafe extern "C"` block with the `CFn<N, HISTORY_DEPTH>` signature for each name,
plus the ABI check (`FFI_N == N`, `FFI_DEPTH == HISTORY_DEPTH`). Resolves `N` from `robot!` in
scope.

## Calling from a controller

```rust
pub fn call<F, C>(f: F, time: &Time, state: &mut State<'_, N>, ctx: Option<&mut C>)
    -> Result<Status, ForeignError>
where F: Foreign<N>;   // a CFn<N, HISTORY_DEPTH>, or a Bridge
```

| Rule | Source |
|---|---|
| Any controller form may call it, inline included | FR-004 (SRS FR-10.3) |
| One call fills, calls, applies; nothing else to write | FR-003 (SRS FR-10.1, NFR-4) |
| Writes go through the State setters: write mask and envelopes apply | FR-011 (SRS FR-2.5, FR-13.5) |
| `Err` applies nothing | FR-012 (SRS FR-2.5) |
| `Status::Shutdown` / `EmergencyStop` have already raised the signal when `call` returns | FR-023 (SRS FR-4.1, FR-4.2), D13 A |
| C/C++ path: no heap, no lock, no `dyn`, no panic | FR-021, FR-022 (SRS NFR-1, NFR-2) |
| The threaded form has no `State`; it cannot call this. It may run foreign code on its own data | FR-004 (SRS FR-1.2) |

`call` returns the error; the Rust controller decides (log, hold, `emergency_stop!`).

## Errors

```rust
pub enum ForeignError {
    Code(i32),          // C/C++ negative or unknown return
    Python(String),     // exception text
    Lua(String),        // error text
    Load(String),       // bridge start or script load, returned in main.rs
}
```

`Display` is one human-readable line. Release builds keep the code and drop internal causes
(SRS NFR-2).
