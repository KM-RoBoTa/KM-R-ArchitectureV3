//
// kmr_api is the library the control engineer writes against, linked into
// the control process. It is a convenience layer over kmr_core: the value
// types are re-exported, the handles wrapped thinly. robot! names the robot
// once at build time; the builder binds the bus plugins and hands the run to
// kmr_core's run_on. The relations into kmr_core are drawn in kmr_core.dsl,
// which is included after this file.

!element kmr.control {
    apiMacro = component "kmr_api: robot!" "Names the robot once in the user's main.rs: a preset, or a descriptor with or without its hardware. Emits N, State, Robot, the model module, actuator! and a hidden __Spec; a bad table, a count other than N or parts that do not cover 0..N do not build (SRS FR-6.1, FR-6.4)." "Rust macro_rules, build time"
    apiBuilder = component "kmr_api: builder" "Robot<H, C>: dt, hold, bus plugins, payloads and controllers on schedules, the controller set as kmr_core type-state. run() binds, then kmr_core's run_on; without a plugin it simulates on NoHardware. bind() lends the drivers before and after the run (SRS FR-5.9, FR-6.3, FR-7)." "Rust"
    apiSpec = component "kmr_api: hardware spec" "The link from the named robot to its drivers: the Spec trait robot!'s __Spec implements, the per-brand part types and the coverage check. The dynamixel cfgs live here, so the feature tested is kmr_api's own (SRS FR-5.1, FR-6.1)." "Rust traits, const fn"
    apiBrands = component "kmr_api: brands" "One member per brand, HCons of Brand parts; each plugin goes to the brand whose bus label it names, before any I/O (SRS FR-5.1, FR-6.3, FR-6.4)." "Rust traits"
    apiBus = component "kmr_api: bus plugins" "Where this machine's actuators are plugged in: by serial id or device path, optional baud and bus label (SRS FR-6.3)." "Rust"
    apiHandles = component "kmr_api: controller handles" "State, GroupView, Time, Sensors: borrowed wrappers over the core handles, private inner field, lent for one controller call. Typed getters and setters only, no raw field (SRS FR-2, FR-3.3)." "Rust"
    apiUnits = component "kmr_api: units" "Deg, Cm, M and their rates, converted into Q and Qd (SRS FR-2.2)." "Rust"
    apiPayloads = component "kmr_api: payloads" "Medium and heavy payloads by mass and mount link; the mount is checked against the model's links at finalization, before any I/O (SRS FR-8.1, FR-8.4)." "Rust"
    apiErrors = component "kmr_api: errors" "UserError of run() and bind(), each saying how to fix it, the adapter's error typed inside BringUp; StateError of the handles (SRS FR-6.4, NFR-2, NFR-4)." "Rust, thiserror"
    apiReexports = component "kmr_api: re-exports" "kmr_core's Q, Qd, Tau, Kp, Kd, Mask, Limit, Policy, Envelope, the sealed Field bound, the schedule labels, shutdown! and emergency_stop!; kmr_adapters' presets and Dynamixel table types. Defined once, never copied." "Rust pub use"
    apiFake = component "kmr_api: fake brands" "Two test-only brands of fake members, so robot! expands over two brands in kmr_api's tests. Test builds only." "Rust, cfg(test)"

    apiMacro -> apiSpec "Emits the __Spec that implements"
    apiMacro -> apiBrands "Lists one part per brand as"
    apiMacro -> adaptersPresets "Takes a preset's hardware table from"
    apiMacro -> adaptersTable "Checks the hardware against N with" "const evaluation"
    apiBuilder -> apiSpec "Binds the plugins through"
    apiBuilder -> apiBus "Collects"
    apiBuilder -> apiPayloads "Registers"
    apiBuilder -> apiHandles "Wraps the core handles in, for each controller call"
    apiBuilder -> apiErrors "Fails with"
    apiSpec -> apiBrands "Binds each brand through"
    apiSpec -> adaptersMember "Binds the Dynamixel part to" "dynamixel feature"
    apiBrands -> adaptersBinding "Routes plugins by label, as"
    apiBus -> adaptersBinding "Wraps a"
    apiHandles -> apiReexports "Speaks the value types of"
    apiHandles -> apiErrors "Fails with"
    apiUnits -> apiReexports "Converts into Q and Qd of"
    apiFake -> apiBrands "Implements"

    group "kmr_api: robot!" {
        apiRobotMacro = component "kmr_api::robot!" "Preset, model = .. and hardware = .. forms; calls kmr_model's model! for N, the model module and actuator!." "Rust macro" {
            tags "Code"
        }
        apiRobotItems = component "kmr_api::__robot_items!" "The items robot! emits: the State and Robot aliases, __Spec with its Drivers, LINK_NAMES and bind." "Rust macro" {
            tags "Code"
        }
    }

    group "kmr_api: builder" {
        apiRobot = component "kmr_api::Robot" "new, set_dt, set_hold, set_limit_policy(_group, _all), set_envelope(_group), add_plugins, register_payload, add_controller, add_controller_with, add_controller_as_thread, run, bind. Finalization hands the model's limits (none for a placeholder model), the policies and the envelopes to the core." "Rust struct" {
            tags "Code"
        }
        apiBound = component "kmr_api::Bound" "A finalized robot with its drivers: drivers() before the run, run() hands them back after it." "Rust struct" {
            tags "Code"
        }
        apiWrappers = component "kmr_api::ApiInline, ApiInlineWith, ApiThreaded" "Named wrappers of the user's fn, stored as kmr_core nodes; each call wraps the borrowed core handles. Hidden." "Rust structs" {
            tags "Code"
        }
    }

    group "kmr_api: hardware spec" {
        apiSpecTrait = component "kmr_api::Spec" "Drivers, LINK_NAMES, the model's kinds and limit columns, PLACEHOLDER, LIMITS (the [Limit; N] of the columns, no position bound where continuous) and bind(plugins). Hidden; robot! implements it." "Rust trait" {
            tags "Code"
        }
        apiLend = component "kmr_api::Lend" "A driver list that lends each member as &mut, so Bound::run hands the list back." "Rust trait" {
            tags "Code"
        }
        apiBrandModule = component "kmr_api::__brand, __DxlAdapter, __NoHardware, __bind_none, __check_coverage" "Part type and shape per brand name, the Dynamixel driver or NoHardware by feature, the coverage check of 0..N." "Rust types, const fns" {
            tags "Code"
        }
    }

    group "kmr_api: brands" {
        apiBrandTrait = component "kmr_api::Brand, Brands" "One brand's part and its member; the HCons list of them, labels and bind." "Rust traits" {
            tags "Code"
        }
        apiRoute = component "kmr_api::Route, Refused, __bind" "Plugin routing by bus label; FeatureOff or the member's binding error." "Rust struct, enum, fn" {
            tags "Code"
        }
    }

    group "kmr_api: bus plugins" {
        apiBusType = component "kmr_api::Bus" "by_id, by_dev, baud, bus over a kmr_adapters Binding." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_api: controller handles" {
        apiState = component "kmr_api::State, N" "Whole-robot and per-index getters, prev_* history reads and setters for Q, Qd, Tau, Kp, Kd; age_at, sample_age, read_every, stale_at, faults and prev_faults; initial_state and go_home do not build while SRS FR-2.7 is open." "Rust struct, const" {
            tags "Code"
        }
        apiFaults = component "kmr_api::Faults, Mask" "Read-only copy of one tick's fault masks (stale, bus_error, offline, rejected, joint_limit, envelope) and flags (overrun, sensor_missing, remote_timeout, unauthorized). No setter (SRS FR-13.4)." "Rust struct" {
            tags "Code"
        }
        apiGroupView = component "kmr_api::GroupView" "K chosen actuators by index; reads gather, set_all_* scatter, every index checked first." "Rust struct" {
            tags "Code"
        }
        apiTime = component "kmr_api::Time" "elapsed, dt, tick, overrun_count, last_overrun, accumulated_overrun, by copy." "Rust struct" {
            tags "Code"
        }
        apiSensors = component "kmr_api::Sensors" "Read-only sensor view; no reading to forward yet." "Rust struct" {
            tags "Code"
        }
        apiHomeDecided = component "kmr_api::HomeDecided" "Sealed, met by no type: stops initial_state and go_home at compile time (D5 A)." "Rust trait" {
            tags "Code"
        }
    }

    group "kmr_api: units" {
        apiUnitTypes = component "kmr_api::Deg, Cm, M, DegPerS, CmPerS, MPerS" "Unit markers with From into Q or Qd." "Rust structs" {
            tags "Code"
        }
    }

    group "kmr_api: payloads" {
        apiPayload = component "kmr_api::Payload, Tier" "medium and heavy constructors, mass and mount; no light tier." "Rust struct, enum" {
            tags "Code"
        }
    }

    group "kmr_api: errors" {
        apiUserError = component "kmr_api::UserError" "DuplicateBus, UnknownBus, SharedBusLabel, LabelRequired, FeatureOff, NoHardwareDeclared, BringUp, UnknownLink, DtOutOfRange, NoPolicy, NoActuator, InvalidEnvelope." "Rust enum" {
            tags "Code"
        }
        apiStateError = component "kmr_api::StateError, StateErrorKind" "A handle's failed read or write, from the core's state error." "Rust enums" {
            tags "Code"
        }
    }

    group "kmr_api: fake brands" {
        apiFakeTypes = component "kmr_api::spec::fake::Hardware, Fake" "A fake part (bus labels, engine indices) and the member it binds to, which keeps the ports routed to it." "Rust structs" {
            tags "Code"
        }
    }

    apiRobotMacro -> apiRobotItems "Expands into"
    apiRobotItems -> apiSpecTrait "Implements"
    apiRobot -> apiBound "Finalizes into"
    apiRobot -> apiWrappers "Stores controllers as"
    apiBound -> apiLend "Lends its drivers through"
    apiSpecTrait -> apiRoute "Binds through"
    apiRoute -> apiBrandTrait "Hands each brand its plugins"
    apiBrandModule -> apiBrandTrait "Implements for the Dynamixel part"
    apiState -> apiGroupView "Creates"
    apiState -> apiFaults "Hands out"
    apiState -> apiHomeDecided "Bounds the home calls on"
    apiFakeTypes -> apiBrandTrait "Implements"

    // The user's crate (controllers.dsl is included before this file).
    ctrlMain -> apiMacro "Names the robot with"
    ctrlMain -> apiBuilder "Builds and runs the robot with"
    ctrlMain -> apiHandles "Reads and writes the state through"
}
