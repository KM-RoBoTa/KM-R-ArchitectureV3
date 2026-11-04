//
// kmr_model is a thin proc-macro crate over the kmr_model_core library
// (decision 0009): both run inside rustc, or in a build script, while the
// control process is built, and the binary only receives constants. Their
// components sit in the control process because nothing else links them.

!element kmr.control {
    modelEntry = component "kmr_model: model!" "The proc macro kmr_api's robot! calls: input, then kmr_model_core for the checked robot, then emission. Any fault is a compile error naming the file and the element (SRS FR-6.4)." "Rust proc macro, build time"
    modelLib = component "kmr_model_core: entry" "preset(name), descriptor(path), parse(text, shown): the checked robot and its N, for the proc macro and for kmr_ffi's build helper in a user build.rs." "Rust library, build time"
    modelInput = component "kmr_model: input" "Parses `<krate>; preset = Name` or `<krate>; model = \"path\"`, the forms kmr_api's front door writes." "Rust, syn"
    modelPresets = component "kmr_model_core: presets" "The model halves of the shipped presets (XmBench, Rx28Chain), embedded by name; the hardware halves are in kmr_adapters." "Rust, include_str!"
    modelReaders = component "kmr_model_core: readers" "One reader per syntax: URDF, .ron, JSON, each giving the model as written (validation steps 2 to 5)." "Rust, roxmltree, ron, serde_json"
    modelChecks = component "kmr_model_core: checks" "Validation steps 6 to 10, the same for every syntax: unique names, limits, SRS FR-2.2 units, one link tree." "Rust"
    modelEmit = component "kmr_model: emit" "Writes N, the model module (struct-of-arrays limits and links) and actuator!, with include_bytes! rebuild tracking. No std, heap, dyn or lock in the output (SRS NFR-1, NFR-10)." "Rust, quote"

    modelEntry -> modelInput "Parses its input with"
    modelEntry -> modelLib "Gets the checked robot from"
    modelLib -> modelPresets "Looks a preset up in"
    modelLib -> kmr.model "Reads the descriptor from" "File, at build time"
    modelLib -> modelReaders "Reads the descriptor with"
    modelReaders -> modelChecks "Hands the model as written to"
    modelEntry -> modelEmit "Hands the checked robot to"

    group "kmr_model: model!" {
        modelFn = component "kmr_model::model" "The #[proc_macro] entry point; expand() gets the checked robot from kmr_model_core and emits it." "Rust fn" {
            tags "Code"
        }
    }

    group "kmr_model: input" {
        modelInputType = component "input::Input" "The krate path and the Source of one call." "Rust struct" {
            tags "Code"
        }
        modelSource = component "input::Source" "Preset(Ident) or Model(LitStr)." "Rust enum" {
            tags "Code"
        }
    }

    group "kmr_model_core: entry" {
        modelLibFns = component "kmr_model_core::{preset, descriptor, parse}" "The checked Robot of a preset name, a descriptor path or a text; errors are robot!'s one-line messages. Robot::n() is N." "Rust fns" {
            tags "Code"
        }
    }

    group "kmr_model_core: presets" {
        modelPreset = component "presets::Preset" "One preset's name, shown path and descriptor text; find() lists every name on a miss." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_model_core: readers" {
        modelReader = component "model::Reader" "Reads one syntax into a Model." "Rust trait" {
            tags "Code"
        }
        modelUrdf = component "urdf::Urdf" "The URDF reader; positions from roxmltree." "Rust struct" {
            tags "Code"
        }
        modelSerde = component "serde_format::Ron, Json" "The .ron and JSON readers over one serde schema that denies unknown fields." "Rust structs" {
            tags "Code"
        }
        modelModel = component "model::Model" "A descriptor as read: actuators, joints and links in file order, before the checks." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_model_core: checks" {
        modelValidate = component "model::validate" "Steps 6 to 10 in order; the first failure stops." "Rust fn" {
            tags "Code"
        }
        modelRobot = component "model::Robot" "The checked robot: CheckedActuator in index order, CheckedLink in file order." "Rust struct" {
            tags "Code"
        }
        modelError = component "model::ModelError" "Why a descriptor does not build: file, line, column, element, reason, SRS marker." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_model: emit" {
        modelEmitFn = component "emit::emit" "N, model::{NAMES, LOWER, UPPER, SPEED, EFFORT, LINK_NAMES, LINK_PARENT, LINK_MASS, LINK_COM} and actuator!, named through the caller's krate path." "Rust fn" {
            tags "Code"
        }
    }

    modelFn -> modelInputType "Parses"
    modelFn -> modelLibFns "Calls"
    modelLibFns -> modelPreset "Finds"
    modelLibFns -> modelReader "Reads with"
    modelLibFns -> modelValidate "Checks with"
    modelInputType -> modelSource "Holds"
    modelUrdf -> modelReader "Implements"
    modelSerde -> modelReader "Implements"
    modelReader -> modelModel "Gives"
    modelValidate -> modelModel "Checks"
    modelValidate -> modelRobot "Gives"
    modelValidate -> modelError "Fails with"
    modelEmitFn -> modelRobot "Writes out"
}
