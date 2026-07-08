#!/usr/bin/env python3
"""consumer.py — drive the kmr control loop from Python via ctypes.

Loads the precompiled C ABI shared library (libkmr_ffi.so) and calls the same
flat symbols a C program would. No Rust or C toolchain needed at run time —
just the .so.

Run (from the workspace root):
    KMR_FFI_LIB=target/release/libkmr_ffi.so python3 src/kmr_ffi/examples/consumer.py

If KMR_FFI_LIB is unset, a few standard locations are tried. See
src/kmr_ffi/README.md for details.
"""
import ctypes
import os
import sys
from pathlib import Path


def _find_lib() -> Path:
    """Locate libkmr_ffi.so: env override first, then common build dirs."""
    env = os.environ.get("KMR_FFI_LIB")
    if env:
        return Path(env).resolve()
    here = Path(__file__).resolve()
    candidates = [
        # workspace root: src/kmr_ffi/examples/ -> ../../../target/release
        here.parents[3] / "target" / "release" / "libkmr_ffi.so",
        here.parents[3] / "target" / "debug" / "libkmr_ffi.so",
        # packaged dist (parent demo project)
        Path.cwd() / "dist" / "lib" / "libkmr_ffi.so",
    ]
    for c in candidates:
        if c.exists():
            return c
    sys.exit(
        "libkmr_ffi.so not found. Set KMR_FFI_LIB=/path/to/libkmr_ffi.so\n"
        f"tried: {', '.join(str(c) for c in candidates)}"
    )


# Controller callback type: void(*)(KmrRobotState*)
CONTROLLER = ctypes.CFUNCTYPE(None, ctypes.c_void_p)


def main() -> None:
    lib = ctypes.CDLL(str(_find_lib()))

    # Declare signatures so ctypes marshals arguments correctly.
    lib.kmr_run.argtypes = [CONTROLLER]
    lib.kmr_run.restype = None
    lib.kmr_robot_set_q_at.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_size_t]
    lib.kmr_robot_set_q_at.restype = ctypes.c_int  # KmrStatus

    @CONTROLLER
    def my_controller(robot):
        s = lib.kmr_robot_set_q_at(robot, ctypes.c_float(1.5), 2)
        print(f"set joint 2 = 1.5 -> status {s}")  # expect 0 (KMR_OK)
        s = lib.kmr_robot_set_q_at(robot, ctypes.c_float(9.0), 99)
        print(f"set joint 99       -> status {s}")  # expect 1 (OUT_OF_RANGE)

    # Keep a reference to my_controller alive for the duration of the call.
    lib.kmr_run(my_controller)


if __name__ == "__main__":
    main()
