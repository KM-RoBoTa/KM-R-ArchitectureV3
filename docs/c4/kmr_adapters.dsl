//
// kmr_adapters is a library linked into the control process: one member of
// the robot's hardware list per brand, Dynamixel first. Its tables and
// presets are consts checked at compile time; its bring-up and per-tick
// calls run in the loop's process, behind kmr_core's Hal contract.

!element kmr.control {
    adaptersTable = component "kmr_adapters: hardware table" "The Dynamixel hardware of a robot as a const: buses by label, baud and read cadence; per actuator bus, id, model, mode, torque choice and zero. check() and the const overrides reject a bad table at compile time, naming the actuator or the bus (SRS FR-6.1, FR-6.4)." "Rust, const fn"
    adaptersPresets = component "kmr_adapters: presets" "The hardware halves of the shipped robots (XmBench, Rx28Chain): a checked table and its declared read and write costs; the model halves are in kmr_model (SRS FR-6.1, NFR-11)." "Rust consts"
    adaptersBinding = component "kmr_adapters: binding" "Bus plugins (by serial id or device path, optional baud and label) matched to the table's buses by label, before any I/O (SRS FR-6.3)." "Rust"
    adaptersMember = component "kmr_adapters: Dynamixel member" "Implements Hal for the indices its placement owns: claim, bring-up, read, write, hold, torque off on every actuator, close. Every fault a typed error or a bit (SRS FR-5.1 to FR-5.10, FR-13.4)." "Rust"
    adaptersIo = component "kmr_adapters: per-tick path" "Groups per (bus, series), read cadence, grouped reads and goal writes, start goals before torque on, line-time costs. Stack arrays only: no allocation, lock, sleep or retry (SRS FR-5.3, NFR-1)." "Rust"
    adaptersUnits = component "kmr_adapters: units" "Raw register values to SI per model and back, position through each actuator's zero; a value with no raw counterpart is None, never clamped." "Rust"
    adaptersPort = component "kmr_adapters: port" "Resolves a plugin's port to one device, checks it is a readable and writable character device, flushes it and sets the 1 ms USB latency timer (SRS FR-5.10)." "Rust, rustix"
    adaptersBackend = component "kmr_adapters: backend seam" "The private trait between the member and a Dynamixel SDK, values raw, one error variant per cause." "Rust trait"
    adaptersDxlRs = component "kmr_adapters: DxlRs" "DxlRsV2: the Backend over dxl_rs_v2, the pure-Rust SDK, behind the dynamixel feature; every dxl_rs_v2 call and type stays in its file." "Rust, dxl_rs_v2"
    adaptersScripted = component "kmr_adapters: scripted backend" "The test Backend: refuses what DxlRsV2 refuses, logs every call, scripts answers and failures. Test builds only." "Rust, cfg(test)"
    adaptersCubeTable = component "kmr_adapters: CubeMars table" "The CubeMars hardware of a robot as a const: CAN interfaces by label; per actuator interface, CAN id and the gains sent on a tick that writes none (AK60-6 default 150 and 1). check() rejects a bad table at compile time (SRS FR-6.4)." "Rust, const fn"
    adaptersCubeMember = component "kmr_adapters: CubeMars member" "Implements Hal for AK60-6 actuators in MIT mode, one group per CAN interface, read every tick: bring-up asked limp with the exit frame, positions offset from Init (the zero frame is never sent), enter then a start command at the measured position, one frame per actuator per tick, replies drained one tick old, exit frame at torque off (SRS FR-5.1, FR-5.3, FR-5.6)." "Rust"
    adaptersMit = component "kmr_adapters: MIT frames" "p/v/kp/kd/t packed 16/12/12/12/12 bits over the AK60-6 ranges, signs negated both ways, ported from cubemars_rs; a value out of range is refused, never clipped." "Rust"
    adaptersCan = component "kmr_adapters: CAN transport seam" "The private trait between the CubeMars member and the CAN bus: open, non-blocking send and receive, a bring-up wait, close." "Rust trait"
    adaptersSocketCan = component "kmr_adapters: SocketCan" "The CAN transport on raw SocketCAN: one non-blocking CAN_RAW socket per interface, bound by name through the socketcan crate, behind the cubemars feature." "Rust, socketcan"
    adaptersScriptedCan = component "kmr_adapters: scripted CAN bus" "Simulated AK60-6 motors answering every frame with their state, a fixed-size call log, scripted failures. Test builds only." "Rust, cfg(test)"

    adaptersPresets -> adaptersTable "Declares its hardware as a checked"
    adaptersPresets -> adaptersIo "Computes its declared costs with" "const evaluation"
    adaptersMember -> adaptersTable "Is built from a placed"
    adaptersMember -> adaptersBinding "Binds its buses with"
    adaptersMember -> adaptersPort "Opens its ports through"
    adaptersMember -> adaptersIo "Reads and writes each tick through"
    adaptersMember -> adaptersBackend "Talks to the SDK through"
    adaptersIo -> adaptersUnits "Converts values with"
    adaptersIo -> adaptersBackend "Sends grouped transactions through"
    adaptersDxlRs -> adaptersBackend "Implements"
    adaptersScripted -> adaptersBackend "Implements"
    adaptersPort -> hardware "Prepares the serial device of" "Linux /dev, sysfs"
    adaptersDxlRs -> hardware "Reads current state, writes goals and torque" "Serial, Dynamixel Protocol 1 and 2"
    adaptersCubeMember -> adaptersCubeTable "Is built from a placed"
    adaptersCubeMember -> adaptersBinding "Binds its CAN interfaces with"
    adaptersCubeMember -> adaptersMit "Packs commands and unpacks replies with"
    adaptersCubeMember -> adaptersCan "Talks to the CAN bus through"
    adaptersSocketCan -> adaptersCan "Implements"
    adaptersScriptedCan -> adaptersCan "Implements"
    adaptersSocketCan -> hardware "Sends MIT commands, receives state" "SocketCAN, AK60-6 MIT mode"

    group "kmr_adapters: hardware table" {
        adaptersTableType = component "kmr_adapters::dynamixel::Table" "NB buses and N actuators; check(), the overrides (baud, read_every, id, mode, read_only, zero and group forms), at() and in_order()." "Rust struct" {
            tags "Code"
        }
        adaptersActuator = component "kmr_adapters::dynamixel::Actuator, Bus" "One actuator and one bus of a table." "Rust structs" {
            tags "Code"
        }
        adaptersEncodings = component "kmr_adapters::dynamixel::Model, Mode, Torque, Zero, Series, Protocol" "Plain enum encodings; Model knows its protocol, series, bauds, modes and datasheet ranges." "Rust enums" {
            tags "Code"
        }
        adaptersPlacement = component "kmr_adapters::dynamixel::Placement" "A checked table with the engine index of each actuator, distinct and below N." "Rust struct" {
            tags "Code"
        }
        adaptersPlaced = component "kmr_adapters::config::Placed" "The placement's actuators by engine index, None where another member owns it." "Rust struct" {
            tags "Code"
        }
        adaptersMessage = component "kmr_adapters::config::Message" "A compile-error message built byte by byte in const evaluation." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_adapters: presets" {
        adaptersXmBench = component "kmr_adapters::presets::XmBench, Rx28Chain" "HARDWARE, READ_COST and WRITE_COST of each shipped robot." "Rust structs" {
            tags "Code"
        }
    }

    group "kmr_adapters: binding" {
        adaptersPortType = component "kmr_adapters::binding::Port, Binding" "ById(serial) or ByDev(path), with an optional baud and bus label." "Rust enum, struct" {
            tags "Code"
        }
        adaptersMatch = component "kmr_adapters::binding::match_plugins" "One binding per bus by label, or the error naming the plugin or the bus (D6 B)." "Rust fn" {
            tags "Code"
        }
    }

    group "kmr_adapters: Dynamixel member" {
        adaptersDynamixel = component "kmr_adapters::dynamixel::Dynamixel" "The member over NB buses and a backend: new, bind, and the Hal calls." "Rust struct" {
            tags "Code"
        }
        adaptersError = component "kmr_adapters::dynamixel::DynamixelError" "Every bind and bring-up error, Copy, naming bus, port, actuator index and id, and the values read." "Rust enum" {
            tags "Code"
        }
    }

    group "kmr_adapters: per-tick path" {
        adaptersGroups = component "kmr_adapters::io::Groups, io::Table" "Transaction groups per (bus, series) of the owned actuators, numbered from 0, with cadence and costs." "Rust structs" {
            tags "Code"
        }
        adaptersGoals = component "kmr_adapters::io::Goals" "Up to one goal per member of a group, sent in one transaction." "Rust struct" {
            tags "Code"
        }
        adaptersTick = component "kmr_adapters::io::read_due, write_batch, write_start_goals" "The per-tick read and write and the start goals of torque on; faults as bits." "Rust fns" {
            tags "Code"
        }
        adaptersCosts = component "kmr_adapters::io::declared_costs" "The costs check declares, in const evaluation, for the presets." "Rust const fn" {
            tags "Code"
        }
    }

    group "kmr_adapters: units" {
        adaptersScale = component "kmr_adapters::units::Scale" "Register range, centre and scale of one model, from the e-Manual." "Rust struct" {
            tags "Code"
        }
        adaptersConvert = component "kmr_adapters::units::origin, position_to_si, position_to_raw" "0 rad at the actuator's zero; position both ways, velocity and current to SI." "Rust fns" {
            tags "Code"
        }
    }

    group "kmr_adapters: port" {
        adaptersDevice = component "kmr_adapters::port::DevicePath" "A resolved device path, never a by-id link, compared by device." "Rust struct" {
            tags "Code"
        }
        adaptersPortFns = component "kmr_adapters::port::locate, preflight, flush, enforce_latency_timer" "Bring-up only, never per tick." "Rust fns" {
            tags "Code"
        }
    }

    group "kmr_adapters: backend seam" {
        adaptersBackendTrait = component "kmr_adapters::backend::Backend" "open_ports, ping, reads, sync writes, torque, force_torque_off, settings, health, angle limits, close_ports; MAX_PORTS and STATE_READS." "Rust trait" {
            tags "Code"
        }
        adaptersBackendError = component "kmr_adapters::backend::BackendError" "One variant per cause with its data; Io is the wire only." "Rust enum" {
            tags "Code"
        }
    }

    group "kmr_adapters: DxlRs" {
        adaptersDxlRsType = component "kmr_adapters::dynamixel::DxlRs" "DxlRsV2: one dxl_rs_v2 bus per protocol on each port, and the start goals written since each torque switch." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_adapters: scripted backend" {
        adaptersScriptedType = component "kmr_adapters::scripted::Scripted" "Per (port, id) state: powered here, torque, goals, answers, faults; a fixed-size call log." "Rust struct" {
            tags "Code"
        }
    }

    group "kmr_adapters: CubeMars member" {
        adaptersCubeMars = component "kmr_adapters::cubemars::CubeMars" "The member over NB CAN interfaces and a transport: new, bind, and the Hal calls." "Rust struct" {
            tags "Code"
        }
        adaptersCubeError = component "kmr_adapters::cubemars::CubeMarsError" "Every bind and bring-up error, Copy, naming bus, interface, actuator index and CAN id." "Rust enum" {
            tags "Code"
        }
        adaptersCubeTableType = component "kmr_adapters::cubemars::Table, Actuator, Bus, Placement" "NB interfaces and N actuators; check(), gains(), at() and in_order()." "Rust structs" {
            tags "Code"
        }
    }
}
