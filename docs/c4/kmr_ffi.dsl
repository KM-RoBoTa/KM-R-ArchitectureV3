//
// kmr_ffi holds the plain C types that cross to a foreign controller and the
// build-script helper that writes the headers and compiles the user's C and
// C++. It has no engine dependency; its build helper runs in the user's
// build.rs and reads N through kmr_model_core (decision 0009), its types, the
// call and the script bridges end up in the control process.
// kmr_api::ffi, the glue that fills the views from State and applies the
// writes, is drawn in kmr_api.dsl.

// The example crates kmr_ffi/examples/c and kmr_ffi/examples/cpp are user
// crates (controllers like the controllers crate) and are not drawn.

!element kmr.control {
    ffiFields = component "kmr_ffi: field list" "The one declaration of what crosses: the core's sealed fields and the per-actuator flags. Every per-field item, Rust type and header line, is generated from it, so a new field is one line (SRS FR-10.2)." "Rust macro_rules"
    ffiView = component "kmr_ffi: views" "The #[repr(C)], Copy, pointer-free types foreign code reads and writes: the History with ages, the clock as integers, the write buffer with its written mask. N in 1..=64, checked at compile time (SRS FR-10.1, FR-2.3, FR-3.3)." "Rust, repr(C)"
    ffiBuild = component "kmr_ffi: build helper" "Feature build, run from the user's build.rs: writes kmr_ffi.h (C11), kmr_ffi.hpp (C++17) and the ABI file for one N and History depth into OUT_DIR, then compiles the user's sources with cc. The headers expose the crossing types and nothing of the engine (SRS FR-10.1)." "Rust, cc, build time"

    ffiCall = component "kmr_ffi: call" "Runs one foreign body over the views: clears the write buffer, calls, clears it again on an error so nothing applies. The C path is the crate's only unsafe block; no heap, no lock, no dyn (SRS FR-10.1, FR-2.5, NFR-1, NFR-2)." "Rust"
    ffiScript = component "kmr_ffi: script lists" "The field, flag and clock names the bridges build their script objects from, and per-field walks over the views, both generated from the field list (SRS FR-10.2)." "Rust macro_rules"
    ffiPython = component "kmr_ffi: Python bridge" "Feature python: an embedded CPython running the user's step(read, write, time). Persistent objects refilled per call, 0-based. Load errors are returned before run(); an exception applies nothing. GIL and allocation are the user's cost (SRS FR-10.1, FR-10.3, FR-6.4)." "Rust, pyo3"
    ffiLua = component "kmr_ffi: Lua bridge" "Feature lua: a vendored Lua 5.4 state running the user's step(read, write, time). 1-based tables, 0-based mask bits, age -1 for never read; Send. Load errors are returned before run(); a Lua error applies nothing (SRS FR-10.1, FR-10.3, FR-6.4)." "Rust, mlua"

    ffiView -> ffiFields "Generates its per-field types from"
    ffiCall -> ffiView "Runs a foreign body over"
    ffiScript -> ffiFields "Generates its names and walks from"
    ffiPython -> ffiCall "Is a Foreign body for"
    ffiPython -> ffiScript "Builds its objects from"
    ffiLua -> ffiCall "Is a Foreign body for"
    ffiLua -> ffiScript "Builds its tables from"
    ffiBuild -> ffiFields "Generates the header lines from"
    ffiBuild -> ffiView "Writes C and C++ declarations matching the layout of"
    apiFfi -> ffiCall "Runs a foreign body through"
    apiFfi -> ffiBuild "Re-exports, the History depth filled in"
    apiFfi -> ffiFields "Generates its apply match on FieldId from"

    group "kmr_ffi: field list" {
        ffiForEachField = component "for_each_field!" "Calls a callback macro with q: Q, qd: Qd, tau: Tau, kp: Kp, kd: Kd; each Type names its core FieldId variant." "Rust macro_rules" {
            tags "Code"
        }
        ffiForEachFlag = component "for_each_flag!" "Calls a callback macro with stale, bus_error, offline, rejected, joint_limit, envelope." "Rust macro_rules" {
            tags "Code"
        }
    }

    group "kmr_ffi: views" {
        ffiReadView = component "view::ReadView<N, D>" "samples, age per actuator (AGE_NONE = never read) and D History slots, newest first." "Rust struct, repr(C)" {
            tags "Code"
        }
        ffiSlot = component "view::Slot<N>" "One History slot: every field as N values, the per-actuator flags and the per-tick flags. A stale actuator holds its last good value." "Rust struct, repr(C)" {
            tags "Code"
        }
        ffiTimeView = component "view::TimeView" "tick, dt_ns, elapsed_ns and the three overrun counters, all u64." "Rust struct, repr(C)" {
            tags "Code"
        }
        ffiWriteBuf = component "view::WriteBuf<N>" "Values foreign code sets and the Written masks that say which; only flagged entries are applied." "Rust struct, repr(C)" {
            tags "Code"
        }
        ffiFlags = component "view::ActuatorFlags" "One Mask per flag, bit i = actuator i." "Rust struct, repr(C)" {
            tags "Code"
        }
    }

    group "kmr_ffi: build helper" {
        ffiBuildType = component "build::Build" "One build for a robot and a History depth; compile() reads N, writes the three files and compiles the sources." "Rust struct" {
            tags "Code"
        }
        ffiBuildFns = component "build::robot, build::model" "A Build for a preset name or a descriptor path, N read through kmr_model_core as robot! reads it; the depth is an argument kmr_api fills." "Rust fns" {
            tags "Code"
        }
        ffiBuildError = component "build::Error" "Unknown preset or unreadable descriptor, N outside 1..=64, a source neither C nor C++, no OUT_DIR, a failed write or compile." "Rust enum, thiserror" {
            tags "Code"
        }
        ffiHeaders = component "build::c_header, cpp_header, abi_rs" "The generated texts, from the field and flag lists." "Rust fns" {
            tags "Code"
        }
    }

    group "kmr_ffi: call" {
        ffiForeignMacro = component "foreign!" "Includes the ABI file, const-asserts its N and depth against robot!'s, declares each C body in an unsafe extern C block of the CFn signature and binds its name as a CFn const (SRS FR-10.2, FR-6.1)." "Rust macro_rules" {
            tags "Code"
        }
        ffiCFn = component "call::CFn<N, D>" "unsafe extern C fn(*const ReadView, *mut WriteBuf, *const TimeView, *mut c_void) -> i32." "Rust type alias" {
            tags "Code"
        }
        ffiForeign = component "call::Foreign<N, D>" "Something call runs once: invoke over the views and an optional context. Implemented for CFn, the two bridges and &mut of any." "Rust trait" {
            tags "Code"
        }
        ffiCallFn = component "call::call" "Clear, invoke, clear on error." "Rust fn" {
            tags "Code"
        }
        ffiStatus = component "call::Status" "Continue, Shutdown, EmergencyStop from the codes 0, 1, 2." "Rust enum" {
            tags "Code"
        }
        ffiError = component "call::ForeignError" "Code(i32), Python(String), Lua(String), Load(String); one-line Display." "Rust enum, thiserror" {
            tags "Code"
        }
    }

    group "kmr_ffi: Python bridge" {
        ffiPyBridge = component "python::Bridge" "The script's function, the prelude's object builder and the persistent read, write, time objects, built on the first call for N and D." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_ffi: Lua bridge" {
        ffiLuaBridge = component "lua::Bridge" "The Lua state, the script's function and the persistent read, write, time tables, built on the first call for N and D." "Rust struct" {
            tags "Code"
        }
    }

    ffiCallFn -> ffiForeign "Invokes"
    ffiCallFn -> ffiError "Returns"
    ffiCallFn -> ffiStatus "Returns"
    ffiCFn -> ffiForeign "Implements"
    ffiPyBridge -> ffiForeign "Implements"
    ffiLuaBridge -> ffiForeign "Implements"
    ffiCallFn -> ffiWriteBuf "Clears"
    ffiReadView -> ffiSlot "Holds D of"
    ffiSlot -> ffiFlags "Holds"
    ffiWriteBuf -> ffiForEachField "Is generated from"
    ffiSlot -> ffiForEachField "Is generated from"
    ffiFlags -> ffiForEachFlag "Is generated from"
    ffiBuildFns -> ffiBuildType "Gives"
    ffiBuildType -> ffiHeaders "Writes"
    ffiForeignMacro -> ffiHeaders "Checks the ABI file of"
    ffiForeignMacro -> ffiCFn "Declares bodies of"
    ffiBuildType -> ffiBuildError "Fails with"
    ffiHeaders -> ffiForEachField "Reads"
    ffiHeaders -> ffiForEachFlag "Reads"
}
