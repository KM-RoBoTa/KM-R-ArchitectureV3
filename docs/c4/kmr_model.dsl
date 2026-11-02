// C3/C4 for kmr_model: written by /speckit-ov-c4-code
//
// kmr_model is a proc-macro crate: it runs inside rustc while the control
// process is built, and the binary only receives constants. Its components
// sit in the control process because nothing else links them.

!element kmr.control {
    modelEntry = component "kmr_model: model!" "The proc macro kmr_api's robot! calls: input, descriptor, checks, emission. Any fault is a compile error naming the file and the element (SRS FR-6.4)." "Rust proc macro, build time"
    modelInput = component "kmr_model: input" "Parses `<krate>; preset = Name` or `<krate>; model = \"path\"`, the forms kmr_api's front door writes." "Rust, syn"
    modelPresets = component "kmr_model: presets" "The model halves of the shipped presets (XmBench, Rx28Chain), embedded by name; the hardware halves are in kmr_adapters." "Rust, include_str!"
    modelReaders = component "kmr_model: readers" "One reader per syntax: URDF, .ron, JSON, each giving the model as written (validation steps 2 to 5)." "Rust, roxmltree, ron, serde_json"
    modelChecks = component "kmr_model: checks" "Validation steps 6 to 10, the same for every syntax: unique names, limits, SRS FR-2.2 units, one link tree." "Rust"
    modelEmit = component "kmr_model: emit" "Writes N, the model module (struct-of-arrays limits and links) and actuator!, with include_bytes! rebuild tracking. No std, heap, dyn or lock in the output (SRS NFR-1, NFR-10)." "Rust, quote"

    modelEntry -> modelInput "Parses its input with"
    modelEntry -> modelPresets "Looks a preset up in"
    modelEntry -> kmr.model "Reads the descriptor from" "File, at build time"
    modelEntry -> modelReaders "Reads the descriptor with"
    modelReaders -> modelChecks "Hands the model as written to"
    modelChecks -> modelEmit "Hands the checked robot to"

    group "kmr_model: model!" {
        modelFn = component "kmr_model::model" "The #[proc_macro] entry point; expand() loads, checks and emits." "Rust fn" {
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

    group "kmr_model: presets" {
        modelPreset = component "presets::Preset" "One preset's name, shown path and descriptor text; find() lists every name on a miss." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_model: readers" {
        modelReader = component "model::Reader" "Reads one syntax into a Model." "Rust trait" {
            tags "Code"
        }
        modelUrdf = component "urdf::Urdf" "The URDF reader; positions from roxmltree." "Rust struct" {
            tags "Code"
        }
        modelSerde = component "serde_format::Ron, Json" "The .ron and JSON readers over one serde schema that denies unknown fields." "Rust structs" {
            tags "Code"
        }
        modelDescriptor = component "model::Descriptor" "Path, Format and text of one descriptor." "Rust struct" {
            tags "Code"
        }
        modelModel = component "model::Model" "A descriptor as read: actuators, joints and links in file order, before the checks." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_model: checks" {
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
    modelFn -> modelPreset "Finds"
    modelFn -> modelDescriptor "Loads"
    modelInputType -> modelSource "Holds"
    modelUrdf -> modelReader "Implements"
    modelSerde -> modelReader "Implements"
    modelReader -> modelModel "Gives"
    modelValidate -> modelModel "Checks"
    modelValidate -> modelRobot "Gives"
    modelValidate -> modelError "Fails with"
    modelEmitFn -> modelRobot "Writes out"
}
