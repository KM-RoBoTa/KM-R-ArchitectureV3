//
// kmr_ffi holds the plain C types that cross to a foreign controller and the
// build-script helper that writes the headers and compiles the user's C and
// C++. It has no kmr dependency; its build helper runs in the user's build.rs,
// its types end up in the control process. Drawn as it exists: the call into
// C (call.rs), the Python and Lua bridges and kmr_api::ffi are not written
// yet, so they are named here and not drawn.

!element kmr.control {
    ffiFields = component "kmr_ffi: field list" "The one declaration of what crosses: the core's sealed fields and the per-actuator flags. Every per-field item, Rust type and header line, is generated from it, so a new field is one line (SRS FR-10.2)." "Rust macro_rules"
    ffiView = component "kmr_ffi: views" "The #[repr(C)], Copy, pointer-free types foreign code reads and writes: the History with ages, the clock as integers, the write buffer with its written mask. N in 1..=64, checked at compile time (SRS FR-10.1, FR-2.3, FR-3.3)." "Rust, repr(C)"
    ffiBuild = component "kmr_ffi: build helper" "Feature build, run from the user's build.rs: writes kmr_ffi.h (C11), kmr_ffi.hpp (C++17) and the ABI file for one N and History depth into OUT_DIR, then compiles the user's sources with cc. The headers expose the crossing types and nothing of the engine (SRS FR-10.1)." "Rust, cc, build time"

    ffiView -> ffiFields "Generates its per-field types from"
    ffiBuild -> ffiFields "Generates the header lines from"
    ffiBuild -> ffiView "Writes C and C++ declarations matching the layout of"

    group "kmr_ffi: field list" {
        ffiForEachField = component "for_each_field!" "Calls a callback macro with q: Q, qd: Qd, tau: Tau." "Rust macro_rules" {
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
        ffiBuildType = component "build::Build" "One build for N and a History depth; compile() writes the three files and compiles the sources." "Rust struct" {
            tags "Code"
        }
        ffiBuildError = component "build::Error" "Unknown preset or unreadable descriptor, N outside 1..=64, a source neither C nor C++, no OUT_DIR, a failed write or compile." "Rust enum, thiserror" {
            tags "Code"
        }
        ffiHeaders = component "build::c_header, cpp_header, abi_rs" "The generated texts, from the field and flag lists." "Rust fns" {
            tags "Code"
        }
    }

    ffiReadView -> ffiSlot "Holds D of"
    ffiSlot -> ffiFlags "Holds"
    ffiWriteBuf -> ffiForEachField "Is generated from"
    ffiSlot -> ffiForEachField "Is generated from"
    ffiFlags -> ffiForEachFlag "Is generated from"
    ffiBuildType -> ffiHeaders "Writes"
    ffiBuildType -> ffiBuildError "Fails with"
    ffiHeaders -> ffiForEachField "Reads"
    ffiHeaders -> ffiForEachFlag "Reads"
}
