workspace "kmr" "Real-time robot control framework by KM-RoBoTa." {

    !identifiers hierarchical

    model {
        controlEngineer = person "Control engineer" "Writes the control math against the API, without modifying the framework."
        contributor = person "Contributor" "KM-RoBoTa engineer or open-source contributor. Fixes bugs, adds hardware adapters for actuators and sensors."
        integrator = person "System integrator" "Integrates kmr into an existing robotics environment."

        kmr = softwareSystem "kmr" "One instance per host. Runs the real-time control loop of the robots wired to that host, through its own hardware layer." {
            control = container "Control process" "The control engineer's controllers linked with kmr_api and kmr_core into one binary. Runs the 1 kHz loop; owns state, clock, schedules, signal bus and the hardware layer." "Rust binary"
            viz = container "Visualizer" "Live or simulated wireframe of the robot. Optional, started on its own, never in the loop's process." "Dioxus app (desktop or web)"
            model = container "Robot model" "URDF or alternative descriptor. Fixes the joint count of the control process at build time; gives the visualizer its geometry." "File" "File"
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
        kmr -> payloads "Reads state from, drives and exchanges services with"
        kmr -> otherKmr "Exchanges messages with" "Zenoh"
        kmr -> robotics "Exchanges state and commands with" "Zenoh"
        kmr -> compute "Sends current state to, receives desired state from" "Zenoh, or any middleware in a controller"

        # Level 2
        controlEngineer -> kmr.control "Writes controllers, builds and runs the binary"
        controlEngineer -> kmr.viz "Watches the robot move, live or simulated"

        kmr.control -> kmr.model "Is built against" "File, at build time"
        kmr.viz -> kmr.model "Loads the geometry from" "File"
        kmr.control -> kmr.viz "Publishes current state to" "Zenoh"

        kmr.control -> hardware "Reads current state, writes desired state" "Hardware buses"
        kmr.control -> payloads "Reads state from, drives and exchanges services with"
        kmr.control -> otherKmr "Exchanges messages with" "Zenoh"
        kmr.control -> robotics "Exchanges state and commands with" "Zenoh"
        kmr.control -> compute "Sends current state to, receives desired state from" "Zenoh, or any middleware in a controller"
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
            element "External" {
                background #999999
                color #ffffff
            }
        }
    }

}
