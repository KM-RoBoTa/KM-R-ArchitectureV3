//
// kmr_ffi holds the C ABI of C and C++ controllers: the plain #[repr(C)]
// snapshot they are called over, the one call into them, and the build-script
// helper that writes kmr.h and kmr.hpp and compiles the user's C and C++. It
// has no engine dependency; its build helper runs in the user's build.rs and
// reads N through kmr_model_core (decision 0009), its types, the call and the
// Lua bridge end up in the control process. kmr_api::c! and the fill and apply
// around the call are drawn in kmr_api.dsl.
//
// The Lua bridge and its view (ReadView, Slot, TimeView, Foreign, call) stay
// until the Lua API that replaces them lands.
//
// New since the C/C++ redesign, not placed in the hand layout yet: ffiC,
// ffiCState, ffiCTime, ffiCSensors, ffiInvoke.

// The example crates kmr_ffi/examples/{c,cpp,lua} are user crates
// (controllers like the controllers crate) and are not drawn.

!element kmr.control {
    ffiFields = component "kmr_ffi: field list" "The one declaration of what crosses: the core's sealed fields and the per-actuator flags. Every per-field item, Rust type and header line, is generated from it, so a new field is one line (SRS FR-10.2)." "Rust macro_rules"
    ffiView = component "kmr_ffi: write buffer and Lua view" "WriteBuf, the values a foreign controller stages with one written mask per field, also the staging area of the C snapshot; the Lua bridge's read view of the History and its clock as integers. N in 1..=64, checked at compile time (SRS FR-10.1, FR-2.5)." "Rust, repr(C)"
    ffiC = component "kmr_ffi: C snapshot" "The #[repr(C)], Copy, pointer-free handles a C or C++ controller gets: kmr_time (durations as seconds and nanoseconds), kmr_state (History slots with a sampled mask and their faults, ages, cadence, telemetry, the staged writes, a stop request and its reason) and kmr_sensors (IMU and microphone slots). kmr.h implements the controller API over them, so C never calls back into Rust (SRS FR-10.1, FR-10.2, FR-2.3, FR-3.3, FR-5.7, FR-5.11)." "Rust, repr(C)"
    ffiBuild = component "kmr_ffi: build helper" "Feature build, run from the user's build.rs: writes kmr.h (C11, the controller API as static inline functions over the snapshot), kmr.hpp (C++17 methods, kmr::Result, std::optional, KMR_CONTROLLER) and the ABI file for one N and the engine's sizes into OUT_DIR, then compiles the user's sources with cc. The headers expose the controller API and nothing of the engine (SRS FR-10.1)." "Rust, cc, build time"

    ffiCall = component "kmr_ffi: call" "The C controller's one call: invoke clears the previous call's writes and request and calls the C function over the snapshot, its context as void *ctx or null. The crate's only unsafe block; no heap, no lock, no dyn (SRS FR-10.1, NFR-1, NFR-2). The Lua bridge's call, Status and ForeignError stay beside it." "Rust"
    ffiScript = component "kmr_ffi: script lists" "The field, flag and clock names the Lua bridge builds its tables from, and per-field walks over the views, both generated from the field list (SRS FR-10.2)." "Rust macro_rules"
    ffiLua = component "kmr_ffi: Lua bridge" "Feature lua: a vendored Lua 5.4 state running the user's step(read, write, time). 1-based tables, 0-based mask bits, age -1 for never read; Send. Load errors are returned before run(); a Lua error applies nothing (SRS FR-10.1, FR-10.3, FR-6.4)." "Rust, mlua"

    ffiView -> ffiFields "Generates its per-field types from"
    ffiC -> ffiFields "Generates its slot from"
    ffiC -> ffiView "Stages the writes in a"
    ffiCall -> ffiC "Calls a C controller over"
    ffiScript -> ffiFields "Generates its names and walks from"
    ffiLua -> ffiCall "Is a Foreign body for"
    ffiLua -> ffiScript "Builds its tables from"
    ffiBuild -> ffiFields "Generates the header lines from"
    ffiBuild -> ffiC "Writes C and C++ declarations matching the layout of"
    apiFfi -> ffiCall "Calls a C controller through"
    apiFfi -> ffiC "Fills, and applies the writes and the request of"
    apiFfi -> ffiBuild "Re-exports, the engine's sizes filled in"
    apiFfi -> ffiFields "Generates its apply match on FieldId from"

    group "kmr_ffi: field list" {
        ffiForEachField = component "for_each_field!" "Calls a callback macro with q: Q, qd: Qd, tau: Tau, kp: Kp, kd: Kd; each Type names its core FieldId variant." "Rust macro_rules" {
            tags "Code"
        }
        ffiForEachFlag = component "for_each_flag!" "Calls a callback macro with stale, bus_error, offline, rejected, joint_limit, envelope." "Rust macro_rules" {
            tags "Code"
        }
    }

    group "kmr_ffi: write buffer and Lua view" {
        ffiReadView = component "view::ReadView<N, D>" "The Lua bridge's: samples, age per actuator (AGE_NONE = never read) and D History slots, newest first." "Rust struct, repr(C)" {
            tags "Code"
        }
        ffiSlot = component "view::Slot<N>" "The Lua bridge's History slot: every field as N values, the per-actuator flags and the per-tick flags." "Rust struct, repr(C)" {
            tags "Code"
        }
        ffiTimeView = component "view::TimeView" "The Lua bridge's clock: tick, dt_ns, elapsed_ns and the three overrun counters, all u64." "Rust struct, repr(C)" {
            tags "Code"
        }
        ffiWriteBuf = component "view::WriteBuf<N>" "Values a foreign controller stages and the Written masks that say which; only flagged entries are applied." "Rust struct, repr(C)" {
            tags "Code"
        }
        ffiFlags = component "view::ActuatorFlags" "The Lua bridge's per-actuator faults: one Mask per flag, bit i = actuator i." "Rust struct, repr(C)" {
            tags "Code"
        }
    }

    group "kmr_ffi: C snapshot" {
        ffiCState = component "c::CState<N, D>, CSlot<N>, CTickFaults" "kmr_state: D slots of every field with a sampled mask and the tick's faults, the WriteBuf, has_* masks for ages and telemetry, cadence, filled counts, signal and reason[128]." "Rust structs, repr(C)" {
            tags "Code"
        }
        ffiCTime = component "c::CTime, CDuration" "kmr_time: tick, overrun count, elapsed, dt, last and accumulated overrun as secs and nanos, so C's double equals Duration::as_secs_f64 bit for bit." "Rust structs, repr(C)" {
            tags "Code"
        }
        ffiCSensors = component "c::CSensors<I, M, W>, CImu, CMic<W>, CAxes" "kmr_sensors: the engine's IMU and microphone slots, each with a present flag; a microphone's samples of this tick and their count." "Rust structs, repr(C)" {
            tags "Code"
        }
    }

    group "kmr_ffi: build helper" {
        ffiBuildType = component "build::Build, Sizes" "One build for a robot and the engine's sizes (History depth, IMU and microphone slots, microphone window); compile() reads N, writes the three files and compiles the sources." "Rust structs" {
            tags "Code"
        }
        ffiBuildFns = component "build::robot, build::model" "A Build for a preset name or a descriptor path, N read through kmr_model_core as robot! reads it; the sizes are an argument kmr_api fills." "Rust fns" {
            tags "Code"
        }
        ffiBuildError = component "build::Error" "Unknown preset or unreadable descriptor, N outside 1..=64, a source neither C nor C++, no OUT_DIR, a failed write or compile." "Rust enum, thiserror" {
            tags "Code"
        }
        ffiHeaders = component "build::c_header, cpp_header, abi_rs" "The generated texts, from the field and flag lists; the ABI file is the array [N, depth, IMUs, microphones, window] c! checks." "Rust fns" {
            tags "Code"
        }
    }

    group "kmr_ffi: call" {
        ffiInvoke = component "c::invoke" "Clear the writes and the request, call the C function once over references to the snapshot and the context." "Rust fn" {
            tags "Code"
        }
        ffiCFn = component "c::CFn<N, D, I, M, W>" "unsafe extern C fn(*const CTime, *mut CState, *const CSensors, *mut c_void)." "Rust type alias" {
            tags "Code"
        }
        ffiForeign = component "call::Foreign<N, D>" "The Lua bridge's body trait: invoke over the view and an optional context. Implemented for the Lua bridge and &mut of any." "Rust trait" {
            tags "Code"
        }
        ffiCallFn = component "call::call" "The Lua bridge's call: clear, invoke, clear on error." "Rust fn" {
            tags "Code"
        }
        ffiStatus = component "call::Status" "Continue, Shutdown, EmergencyStop from a script's 0, 1, 2." "Rust enum" {
            tags "Code"
        }
        ffiError = component "call::ForeignError" "Code(i32), Lua(String), Load(String); one-line Display." "Rust enum, thiserror" {
            tags "Code"
        }
    }

    group "kmr_ffi: Lua bridge" {
        ffiLuaBridge = component "lua::Bridge" "The Lua state, the script's function and the persistent read, write, time tables, built on the first call for N and D." "Rust struct" {
            tags "Code"
        }
    }

    ffiInvoke -> ffiCFn "Calls a"
    ffiInvoke -> ffiCState "Clears the writes and request of"
    ffiCState -> ffiWriteBuf "Stages in a"
    ffiCState -> ffiForEachField "Is generated from"
    ffiCState -> ffiForEachFlag "Is generated from"
    ffiCallFn -> ffiForeign "Invokes"
    ffiCallFn -> ffiError "Returns"
    ffiCallFn -> ffiStatus "Returns"
    ffiLuaBridge -> ffiForeign "Implements"
    ffiCallFn -> ffiWriteBuf "Clears"
    ffiReadView -> ffiSlot "Holds D of"
    ffiSlot -> ffiFlags "Holds"
    ffiWriteBuf -> ffiForEachField "Is generated from"
    ffiSlot -> ffiForEachField "Is generated from"
    ffiFlags -> ffiForEachFlag "Is generated from"
    ffiBuildFns -> ffiBuildType "Gives"
    ffiBuildType -> ffiHeaders "Writes"
    ffiBuildType -> ffiBuildError "Fails with"
    ffiHeaders -> ffiForEachField "Reads"
    ffiHeaders -> ffiForEachFlag "Reads"
    ffiHeaders -> ffiCState "Declares the C layout of"
}
