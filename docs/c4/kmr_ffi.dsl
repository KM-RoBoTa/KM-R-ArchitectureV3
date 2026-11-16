//
// kmr_ffi holds the C ABI of C and C++ controllers: the plain #[repr(C)]
// snapshot they are called over, the one call into them, and the build-script
// helper that writes kmr.h and kmr.hpp and compiles the user's C and C++. It
// has no engine dependency; its build helper runs in the user's build.rs and
// reads N through kmr_model_core (decision 0009), its types and the call end
// up in the control process. kmr_api::c! and the fill and apply around the
// call are drawn in kmr_api.dsl. Python and Lua controllers are kmr_api's
// (kmr_api::python!, kmr_api::lua!), drawn there too.
//
// New since the C/C++ redesign, not placed in the hand layout yet: ffiC,
// ffiCState, ffiCTime, ffiCSensors, ffiInvoke, and ffiView under its new name.

// The example crates kmr_ffi/examples/{c,cpp} are user crates (controllers
// like the controllers crate) and are not drawn.

!element kmr.control {
    ffiFields = component "kmr_ffi: field list" "The one declaration of what crosses: the core's sealed fields and the per-actuator flags. Every per-field item, Rust type and header line, is generated from it, so a new field is one line (SRS FR-10.2)." "Rust macro_rules"
    ffiView = component "kmr_ffi: write buffer" "WriteBuf, the values a C or C++ controller stages with one written mask per field, the staging area of the C snapshot. N in 1..=64, checked at compile time (SRS FR-10.1, FR-2.5)." "Rust, repr(C)"
    ffiC = component "kmr_ffi: C snapshot" "The #[repr(C)], Copy, pointer-free handles a C or C++ controller gets: kmr_time (durations as seconds and nanoseconds), kmr_state (History slots with a sampled mask and their faults, ages, cadence, telemetry, the staged writes, a stop request and its reason) and kmr_sensors (IMU and microphone slots). kmr.h implements the controller API over them, so C never calls back into Rust (SRS FR-10.1, FR-10.2, FR-2.3, FR-3.3, FR-5.7, FR-5.11)." "Rust, repr(C)"
    ffiBuild = component "kmr_ffi: build helper" "Feature build, run from the user's build.rs: writes kmr.h (C11, the controller API as static inline functions over the snapshot), kmr.hpp (C++17 methods, kmr::Result, std::optional, KMR_CONTROLLER) and the ABI file for one N and the engine's sizes into OUT_DIR, then compiles the user's sources with cc. The headers expose the controller API and nothing of the engine (SRS FR-10.1)." "Rust, cc, build time"

    ffiCall = component "kmr_ffi: call" "The C controller's one call: invoke clears the previous call's writes and request and calls the C function over the snapshot, its context as void *ctx or null. The crate's only unsafe block; no heap, no lock, no dyn (SRS FR-10.1, NFR-1, NFR-2)." "Rust"

    ffiView -> ffiFields "Generates its per-field types from"
    ffiC -> ffiFields "Generates its slot from"
    ffiC -> ffiView "Stages the writes in a"
    ffiCall -> ffiC "Calls a C controller over"
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

    group "kmr_ffi: write buffer" {
        ffiWriteBuf = component "view::WriteBuf<N>" "Values a C or C++ controller stages and the Written masks that say which; only flagged entries are applied." "Rust struct, repr(C)" {
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
    }

    ffiInvoke -> ffiCFn "Calls a"
    ffiInvoke -> ffiCState "Clears the writes and request of"
    ffiCState -> ffiWriteBuf "Stages in a"
    ffiCState -> ffiForEachField "Is generated from"
    ffiCState -> ffiForEachFlag "Is generated from"
    ffiWriteBuf -> ffiForEachField "Is generated from"
    ffiBuildFns -> ffiBuildType "Gives"
    ffiBuildType -> ffiHeaders "Writes"
    ffiBuildType -> ffiBuildError "Fails with"
    ffiHeaders -> ffiForEachField "Reads"
    ffiHeaders -> ffiForEachFlag "Reads"
    ffiHeaders -> ffiCState "Declares the C layout of"
}
