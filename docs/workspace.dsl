workspace "kmr" "Real-time robot control framework by KM-RoBoTa." {

    !identifiers hierarchical

    model {
        controlEngineer = person "Control engineer" "Writes the control math against the API, without modifying the framework."
        contributor = person "Contributor" "KM-RoBoTa engineer or open-source contributor. Extends the engine and the hardware layer: fixes bugs, adds adapters for actuators and sensors."
        integrator = person "System integrator" "Integrates kmr into an existing robotics environment."

        kmr = softwareSystem "kmr" "One instance per host. Runs the real-time control loop of the robots wired to that host, through its own hardware layer." {
            control = container "Control process" "The control engineer's controllers linked with kmr_api and kmr_core into one binary. Runs the loop at the user's dt (1 ms default); owns state, clock, schedules, signal bus and the hardware layer." "Rust binary"
            viz = container "Visualizer" "Live or replayed wireframe of the robot, without hardware if needed. Optional, started on its own, never in the loop's process. Roadmap." "Dioxus app (desktop or web)" "Roadmap"
            model = container "Robot model" "URDF with an actuators side file, or a self-contained .ron. Fixes the actuator count of the control process at build time; gives the visualizer its geometry." "File" "File"
            ports = container "Port binding" "Per machine: binds buses to serial ports, resolved at bring-up." "File" "File"
            keys = container "Identities and keys" "Per machine: Zenoh identities and access rules, kept outside the descriptor." "Key store" "File"
            bench = container "Bench" "Measures the engine on the reference bench: jitter, overruns, overhead, footprint." "Rust binary"
        }

        hardware = softwareSystem "Robot hardware" "Actuators and sensors on the robot's buses." "External"
        payloads = softwareSystem "Payloads" "User-added medium and heavy payloads: own hardware interface or own services." "External"
        otherKmr = softwareSystem "Other kmr instance" "kmr running on another robot." "External"
        robotics = softwareSystem "Robotics environment" "ROS2 or any other existing robotics stack." "External"
        compute = softwareSystem "External compute" "ML model, cloud service or any process that computes desired state from current state." "External"

        # Level 1
        controlEngineer -> kmr "Writes controllers, runs and tunes them"
        contributor -> kmr "Fixes bugs, adds hardware adapters"
        integrator -> kmr "Connects to an existing robotics environment"

        kmr -> hardware "Reads current state, writes desired state" "Hardware buses"
        kmr -> payloads "Takes mass and mount point of"
        kmr -> otherKmr "Publishes History to, receives Desired from" "Zenoh, encrypted"
        kmr -> robotics "Publishes History to, receives Desired from" "Zenoh, encrypted"
        kmr -> compute "Publishes History to, receives Desired from" "Zenoh encrypted, or any middleware in a controller"

        # Level 2
        controlEngineer -> kmr.control "Writes controllers, builds and runs the binary"
        controlEngineer -> kmr.viz "Watches the robot move, live or simulated"

        kmr.control -> kmr.model "Is built against" "File, at build time"
        kmr.viz -> kmr.model "Loads the geometry from" "File"
        kmr.control -> kmr.viz "Publishes History to" "Zenoh, encrypted"
        kmr.control -> kmr.ports "Binds ports at bring-up from" "File"
        kmr.control -> kmr.keys "Authenticates Zenoh peers with" "mTLS or QUIC"
        kmr.bench -> kmr.control "Measures"
        contributor -> kmr.bench "Runs on the reference bench"

        kmr.control -> hardware "Reads current state, writes desired state" "Hardware buses"
        kmr.control -> payloads "Takes mass and mount point of"
        kmr.control -> otherKmr "Publishes History to, receives Desired from" "Zenoh, encrypted"
        kmr.control -> robotics "Publishes History to, receives Desired from" "Zenoh, encrypted"
        kmr.control -> compute "Publishes History to, receives Desired from" "Zenoh encrypted, or any middleware in a controller"

        // C3/C4 per module (docs/c4/<module>.dsl)
        // ov:includes:begin
        !include c4/bench.dsl
        !include c4/controllers.dsl
        !include c4/kmr_adapters.dsl
        !include c4/kmr_api.dsl
        !include c4/kmr_core.dsl
        !include c4/kmr_ffi.dsl
        !include c4/kmr_model.dsl
        // ov:includes:end
    }

    views {
        systemContext kmr "Context" "Level 1: kmr and what surrounds one instance." {
            include *
        }

        container kmr "Containers" "Level 2: what runs on the host and how it talks." {
            include *
        }

        styles {
            element "Person" {
                shape Person
                background #08427b
                color #ffffff
            }
            element "Software System" {
                background #1168bd
                color #ffffff
            }
            element "Container" {
                background #438dd5
                color #ffffff
            }
            element "File" {
                shape Folder
            }
            element "Roadmap" {
                background #dddddd
                color #888888
                border dashed
            }
            element "External" {
                background #999999
                color #ffffff
            }
        }
    }

}
