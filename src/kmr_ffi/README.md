# kmr_ffi — C ABI for the kmr control library

`kmr_ffi` is a thin shim that re-exports the public [`kmr_api`](../kmr_api)
surface as a flat, `extern "C"` interface. It exists so that **non-Rust hosts
(C, Python via ctypes, …) can drive the control loop** against a precompiled
binary, without a Rust toolchain at their build or run time.

Rust consumers do **not** need this crate — they link the `kmr_api` `.rlib`
directly (see [`kmr_api/examples`](../kmr_api/examples)).

```
   Rust host ──► kmr_api (.rlib)  ─┐
                                   ├─► kmr_core (hidden, private)
   C / Python ─► kmr_ffi (.so/.a) ─┘
                  └ wraps kmr_api, exposes extern "C" symbols
```

`kmr_core` is never exposed — `kmr_ffi` only wraps what `kmr_api` makes public.

## How it works

The Rust API is trait-based: you implement `Controller::main_controller` and
receive an owned `RobotState`. C has no traits, so the shim swaps the trait for
a **C function pointer** and hands it an **opaque pointer** to the robot:

| Rust API                              | C ABI (`kmr_ffi.h`)                                  |
|---------------------------------------|------------------------------------------------------|
| `trait Controller`                    | `typedef void (*KmrController)(KmrRobotState*)`      |
| `fn run(impl Controller)`             | `void kmr_run(KmrController)`                         |
| `RobotState::set_q_at(v, i) -> Result`| `KmrStatus kmr_robot_set_q_at(KmrRobotState*, f32, size_t)` |
| `Result<(), Error>`                   | `KmrStatus` enum (`0` ok, `1` out-of-range, `2` null)|

The robot handle is **borrowed**: it is valid only for the duration of the
callback. Do not store it or use it after the callback returns.

Source: [`src/lib.rs`](src/lib.rs). Header: [`include/kmr_ffi.h`](include/kmr_ffi.h).

## Compile

The crate is declared `crate-type = ["cdylib", "staticlib"]`, so one build
produces both a shared library and a static archive.

From the workspace root:

```sh
cargo build --release -p kmr_ffi
```

Outputs (Linux shown; macOS = `.dylib`, Windows = `kmr_ffi.dll` + `.lib`):

| File                          | Use                                   |
|-------------------------------|---------------------------------------|
| `target/release/libkmr_ffi.so`| shared lib — C dynamic link, Python   |
| `target/release/libkmr_ffi.a` | static archive — C static link        |

The C header is checked in at `include/kmr_ffi.h` (hand-written to match the
exported symbols; if you prefer generation, `cbindgen` produces the same).

Verify the symbols:

```sh
nm -D target/release/libkmr_ffi.so | grep kmr_
# 0000... T kmr_robot_set_q_at
# 0000... T kmr_run
```

## Use

### C — [`examples/main.c`](examples/main.c)

Dynamic link (needs the `.so` at run time):

```sh
cc src/kmr_ffi/examples/main.c \
   -I src/kmr_ffi/include \
   -L target/release -lkmr_ffi \
   -o /tmp/kmr_c_demo
LD_LIBRARY_PATH=target/release /tmp/kmr_c_demo
```

Static link (no `.so` needed at run time; pull in libs Rust's std requires):

```sh
cc src/kmr_ffi/examples/main.c \
   -I src/kmr_ffi/include \
   target/release/libkmr_ffi.a \
   -lpthread -ldl -lm \
   -o /tmp/kmr_c_static
/tmp/kmr_c_static
```

### Python — [`examples/consumer.py`](examples/consumer.py)

Pure `ctypes`, no build step — just point it at the `.so`:

```sh
KMR_FFI_LIB=target/release/libkmr_ffi.so python3 src/kmr_ffi/examples/consumer.py
```

Expected output (both examples):

```
set joint 2 = 1.5 -> status 0
set joint 99       -> status 1
```

(`0` = `KMR_OK`, `1` = `KMR_OUT_OF_RANGE`.)

> For a full integrator-style walkthrough that consumes the **precompiled,
> packaged** artifacts (no app source in the path) across Rust / C / Python,
> see the `kmr_multilang_demo/` project in the parent directory.
