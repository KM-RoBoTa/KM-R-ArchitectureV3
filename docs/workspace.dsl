// c4rizer DSL, C1 to C3. Lint: c4rizer lint docs/workspace.dsl
workspace "kmr" "Real-time robot control framework by KM-RoBoTa." {

    !identifiers hierarchical

    model {
        controlEngineer = person "Control engineer" "Writes the control math against the API."
        contributor = person "Contributor" "Fixes the engine, adds hardware adapters."
        integrator = person "System integrator" "Plugs kmr into an existing robotics stack."

        kmr = softwareSystem "kmr" "One per host. Runs the real-time loop of its robots." {
            control = container "Control process" "User controllers and the kmr crates in one binary. Runs the loop." "Rust binary" {
                controllers = component "controllers" "The user's binary: names the robot, adds controllers, runs it" "Rust crate" "Crate,Entry"
                kmrApi = component "kmr_api" "Public API users write controllers against" "Rust crate" "Crate,Entry"
                kmrCore = component "kmr_core" "The engine: control loop, state, timing, signals, hardware contract" "Rust crate" "Crate,Loop"
                kmrAdapters = component "kmr_adapters" "Hardware drivers for Dynamixel, CubeMars and XIAO boards" "Rust crate" "Crate,Hardware"
                kmrModel = component "kmr_model" "Proc macro that turns the robot descriptor into constants" "Rust crate" "Crate,Build"
                kmrModelCore = component "kmr_model_core" "Reads and checks the robot descriptor at build time" "Rust crate" "Crate,Build"
                kmrFfi = component "kmr_ffi" "C ABI and build helper for C and C++ controllers" "Rust crate" "Crate,Bridge"

                group "kmr_api" {
                    apiRobot = component "robot" "Builder of actuators, plugins and controllers, then run" "Rust module" "kmr_api,Entry"
                    apiRobotMacro = component "robot_macro" "The robot! macro: names the robot once in main.rs" "Rust module" "kmr_api,Entry"
                    apiSpec = component "spec" "Binds each hardware brand to its bus plugins" "Rust module" "kmr_api,Hardware"
                    apiActuator = component "actuator" "Actuators declared in main.rs, lowered to the hardware table" "Rust module" "kmr_api,Hardware"
                    apiVendors = component "vendors" "Dynamixel, CubeMars and XIAO models and hardware tables" "Rust module" "kmr_api,Hardware"
                    apiPlugin = component "plugin" "Buses and other plugins the builder takes" "Rust module" "kmr_api,Hardware"
                    apiController = component "controller" "Controller wrappers: context, thread, fallback on tick faults" "Rust module" "kmr_api,Loop"
                    apiHandles = component "handles" "State, group, time, sensors and fault views lent to controllers" "Rust module" "kmr_api,State"
                    apiZenoh = component "zenoh" "Optional Zenoh gateway: History out, remote Desired in" "Rust module" "kmr_api,Bridge"
                    apiFfi = component "ffi" "Optional C, C++, Lua and Python controllers" "Rust module" "kmr_api,Bridge"
                }

                group "kmr_core" {
                    coreRobot = component "kmr_core/robot" "Builder that collects controllers, limits and settings, then runs" "Rust module" "kmr_core,Entry"
                    coreSchedule = component "schedule" "Six fixed phases, one controller list per phase" "Rust module" "kmr_core,Loop"
                    coreControllers = component "kmr_core/controllers" "Controller nodes and the per-phase view they receive" "Rust module" "kmr_core,Loop"
                    coreRuntime = component "runtime" "The control loop: startup phases, then ticks until a signal" "Rust module" "kmr_core,Loop"
                    coreClock = component "clock" "Tick count, dt, deadlines and overruns, fed instants by the loop" "Rust module" "kmr_core,Loop"
                    coreSignal = component "signal" "Lock-free global bus for shutdown and emergency stop" "Rust module" "kmr_core,Safety"
                    coreState = component "state" "Typed robot state: write-only desired values, read-only history" "Rust module" "kmr_core,State"
                    coreLimits = component "limits" "Clamps or stops on joint limit and envelope breaches each tick" "Rust module" "kmr_core,Safety"
                    coreHal = component "hal" "Hardware contract adapters implement, plus the driver and sensor lists" "Rust module" "kmr_core,Hardware"
                    coreSensors = component "sensors" "Fixed IMU and microphone slots that controllers read" "Rust module" "kmr_core,State"
                }

                adpBackend = component "dynamixel/backend" "Private seam between the Dynamixel driver and an SDK" "Rust module" "kmr_adapters,Hardware"
                adpDxlRsV2 = component "dynamixel/dxl_rs_v2" "Backend on the dxl_rs_v2 SDK, every SDK call lives here" "Rust module" "kmr_adapters,Hardware"
                group "kmr_adapters" {
                    adpBinding = component "binding" "Matches bus plugins to the hardware table's buses by label" "Rust module" "kmr_adapters,Entry"
                    adpPresets = component "presets" "Hardware tables of the shipped robots, XmBench and Rx28Chain" "Rust module" "kmr_adapters,Build"
                    adpDynamixel = component "dynamixel" "Dynamixel driver: brings actuators up, reads, writes, cuts torque" "Rust module" "kmr_adapters,Hardware"
                    adpConfig = component "dynamixel/config" "Dynamixel hardware table, checked at compile time" "Rust module" "kmr_adapters,Build"
                    adpIo = component "dynamixel/io" "Per-tick grouped reads and goal writes, no allocation" "Rust module" "kmr_adapters,Loop"
                    adpPort = component "dynamixel/port" "Finds, checks and prepares the serial device of a bus" "Rust module" "kmr_adapters,Hardware"
                    adpCubemars = component "cubemars" "CubeMars AK driver in MIT mode over SocketCAN" "Rust module" "kmr_adapters,Hardware"
                    adpXiao = component "xiao" "Reads the XIAO board's IMU and microphone over USB, UART or BLE" "Rust module" "kmr_adapters,Hardware"
                }

                group "kmr_model" {
                    mdlLib = component "lib" "The model! proc macro entry point" "Rust module" "kmr_model,Entry"
                    mdlInput = component "input" "Parses the model! input: a preset name or a file path" "Rust module" "kmr_model,Build"
                    mdlEmit = component "emit" "Writes the checked robot as const items at the call site" "Rust module" "kmr_model,Build"
                }

                group "kmr_model_core" {
                    mdcLib = component "kmr_model_core/lib" "Entry points that give the checked robot and its N" "Rust module" "kmr_model_core,Entry"
                    mdcModel = component "model" "The model as read, its checks, and the checked robot" "Rust module" "kmr_model_core,State"
                    mdcUrdf = component "urdf" "Reads URDF descriptors" "Rust module" "kmr_model_core,Build"
                    mdcSerdeFormat = component "serde_format" "Reads .ron and JSON descriptors through one serde schema" "Rust module" "kmr_model_core,Build"
                    mdcPresets = component "kmr_model_core/presets" "Model halves of the shipped robots, embedded as text" "Rust module" "kmr_model_core,Build"
                }

                group "kmr_ffi" {
                    ffiC = component "c" "The C tick snapshot and the one controller call" "Rust module" "kmr_ffi,Bridge"
                    ffiFields = component "fields" "The one list of state fields and flags that cross" "Rust module" "kmr_ffi,State"
                    ffiView = component "view" "Write buffer a C controller stages into" "Rust module" "kmr_ffi,State"
                    ffiBuild = component "build" "Build helper: writes kmr.h and kmr.hpp, compiles user sources" "Rust module" "kmr_ffi,Build"
                }
            }
            viz = container "Visualizer" "Robot wireframe, live or replayed. Planned." "Dioxus app" "Roadmap"
            model = container "Robot model" "URDF or .ron. Sets the actuator count at build time." "File" "File"
            ports = container "Port binding" "Per machine: the serial port of each bus." "File" "File"
            keys = container "Identities and keys" "Per machine: Zenoh identities and access rules." "Key store" "File"
            bench = container "Bench" "Measures jitter, overruns and footprint." "Rust binary" {
                benchTick = component "tick" "Body of the tick binaries: one measured run, one report" "Rust module" "bench,Entry"
                benchSetup = component "setup" "Builds the base robot every bench binary starts from" "Rust module" "bench,Entry"
                benchPlan = component "plan" "Reads what one run measures from the environment" "Rust module" "bench,Build"
                benchProbe = component "probe" "Probe controllers that stamp each tick into static recorders" "Rust module" "bench,Loop"
                benchHist = component "hist" "Fixed, heap-free histogram of nanosecond samples" "Rust module" "bench,State"
                benchGrid = component "grid" "Replays the engine's tick grid from the bench's own stamps" "Rust module" "bench,Loop"
                benchCalib = component "calib" "Calibrated workload for the budget method" "Rust module" "bench,Loop"
                benchAlloc = component "alloc" "Counting allocator: heap calls on the loop thread" "Rust module" "bench,Loop"
                benchReport = component "report" "Writes a report's run and figures blocks to a file" "Rust module" "bench,Build"
                benchResults = component "results" "Compares two report sets or summarizes one" "Rust module" "bench,Build"
            }
        }

        hardware = softwareSystem "Robot hardware" "Actuators and sensors on the buses." "External"
        payloads = softwareSystem "Payloads" "User-added payloads with their own interface." "External"
        otherKmr = softwareSystem "Other kmr instance" "kmr on another robot." "External" {
            tags "otherKmr"
            tags "Zenoh"
        }
        robotics = softwareSystem "Robotics environment" "ROS 2 or any other robotics stack." "External" {
            tags "robotics"
            tags "Zenoh"
        }
        compute = softwareSystem "External compute" "ML model or service that computes desired state." "External" {
            tags "compute"
            tags "Zenoh"
        }

        // C1
        controlEngineer -> kmr "Writes, runs and tunes controllers"
        contributor -> kmr "Fixes bugs, adds hardware adapters"
        integrator -> kmr "Connects to a robotics stack"

        kmr -> hardware "Reads current state, writes desired state" "Hardware buses"
        kmr -> payloads "Takes mass and mount point of"
        kmr -> otherKmr "Publishes History to, receives Desired from" "Zenoh, encrypted"
        kmr -> robotics "Publishes History to, receives Desired from" "Zenoh, encrypted"
        kmr -> compute "Publishes History to, receives Desired from" "Zenoh or any middleware"

        // C2
        controlEngineer -> kmr.control "Writes controllers, builds and runs"
        controlEngineer -> kmr.viz "Watches the robot, live or simulated"
        contributor -> kmr.bench "Runs on the reference bench"

        kmr.control -> kmr.model "Is built against" "File, build time"
        kmr.viz -> kmr.model "Loads geometry from" "File"
        kmr.control -> kmr.viz "Publishes History to" "Zenoh, encrypted"
        kmr.control -> kmr.ports "Binds ports at bring-up from" "File"
        kmr.control -> kmr.keys "Authenticates Zenoh peers with" "mTLS or QUIC"
        kmr.bench -> kmr.control "Measures"

        kmr.control -> hardware "Reads current state, writes desired state" "Hardware buses"
        kmr.control -> payloads "Takes mass and mount point of"
        kmr.control -> otherKmr "Publishes History to, receives Desired from" "Zenoh, encrypted"
        kmr.control -> robotics "Publishes History to, receives Desired from" "Zenoh, encrypted"
        kmr.control -> compute "Publishes History to, receives Desired from" "Zenoh or any middleware"

        // C3 crates
        kmr.control.controllers -> kmr.control.kmrApi "Writes controllers against" "Rust crate dependency"
        kmr.control.controllers -> kmr.model "Names its robot model file" "File path, build time"
        kmr.control.kmrApi -> kmr.control.kmrCore "Wraps the engine of" "Rust crate dependency"
        kmr.control.kmrApi -> kmr.control.kmrAdapters "Binds hardware drivers from" "Rust crate dependency"
        kmr.control.kmrApi -> kmr.control.kmrModel "Reads the robot model through" "Rust crate dependency"
        kmr.control.kmrApi -> kmr.control.kmrFfi "Runs C and C++ controllers through" "Rust crate dependency, optional, feature ffi"
        kmr.control.kmrApi -> compute "Sends History, receives Desired" "Zenoh, feature zenoh"
        kmr.control.kmrApi -> kmr.keys "Loads the gateway's TLS identity from" "File, feature zenoh"
        kmr.control.kmrAdapters -> kmr.control.kmrCore "Implements the hardware traits of" "Rust crate dependency"
        kmr.control.kmrAdapters -> hardware "Drives actuators and reads sensors of" "Serial, CAN, USB, BLE"
        kmr.control.kmrModel -> kmr.control.kmrModelCore "Gets the checked robot from" "Rust crate dependency"
        kmr.control.kmrModelCore -> kmr.model "Reads and checks" "File, at build time"
        kmr.control.kmrFfi -> kmr.control.kmrModelCore "Reads the actuator count through" "Rust crate dependency, optional, feature build"

        // C3 kmr_api
        kmr.control.apiRobotMacro -> kmr.control.apiRobot "Emits the Robot alias of" "Rust macro"
        kmr.control.apiRobotMacro -> kmr.control.apiSpec "Implements the Spec trait" "Rust macro"
        kmr.control.apiRobot -> kmr.control.apiSpec "Binds plugins to drivers through" "Rust trait"
        kmr.control.apiRobot -> kmr.control.apiActuator "Lowers declared actuators with" "Rust call"
        kmr.control.apiRobot -> kmr.control.apiPlugin "Collects plugins" "Rust trait"
        kmr.control.apiRobot -> kmr.control.apiController "Stores controllers through" "Rust trait"
        kmr.control.apiRobot -> kmr.control.apiZenoh "Starts the gateway around run" "Rust call"
        kmr.control.apiActuator -> kmr.control.apiVendors "Takes actuator models from" "Rust type"
        kmr.control.apiController -> kmr.control.apiHandles "Lends handles to each call" "Rust type"
        kmr.control.apiZenoh -> kmr.control.apiPlugin "Implements the Plugin trait" "Rust trait"
        kmr.control.apiFfi -> kmr.control.apiController "Implements IntoController for scripts" "Rust trait"
        kmr.control.apiFfi -> kmr.control.apiHandles "Snapshots and applies writes through" "Rust call"

        // C3 kmr_core
        kmr.control.coreRobot -> kmr.control.coreSchedule "Routes each controller into" "Rust trait"
        kmr.control.coreRobot -> kmr.control.coreLimits "Stores limits and policies of" "Rust type"
        kmr.control.coreRobot -> kmr.control.coreRuntime "Starts the run through" "Rust call"
        kmr.control.coreRuntime -> kmr.control.coreSchedule "Walks each phase bucket of" "Rust trait"
        kmr.control.coreRuntime -> kmr.control.coreClock "Injects instants into" "Rust call"
        kmr.control.coreRuntime -> kmr.control.coreSignal "Resets, then drains between phases" "Rust call"
        kmr.control.coreRuntime -> kmr.control.coreHal "Reads, writes and powers off through" "Rust trait"
        kmr.control.coreRuntime -> kmr.control.coreState "Records each tick's sample into" "Rust call"
        kmr.control.coreRuntime -> kmr.control.coreLimits "Checks written values with" "Rust call"
        kmr.control.coreSchedule -> kmr.control.coreControllers "Holds and calls nodes of" "Rust trait"
        kmr.control.coreControllers -> kmr.control.coreState "Lends to each controller" "Rust type"
        kmr.control.coreControllers -> kmr.control.coreClock "Lends read-only to controllers" "Rust type"
        kmr.control.coreControllers -> kmr.control.coreSensors "Lends read-only to controllers" "Rust type"
        kmr.control.coreHal -> kmr.control.coreSensors "Fills slots of" "Rust call"

        // C3 kmr_adapters
        kmr.control.adpPresets -> kmr.control.adpConfig "Declares its hardware table in" "Rust type"
        kmr.control.adpDynamixel -> kmr.control.adpConfig "Is built from" "Rust type"
        kmr.control.adpDynamixel -> kmr.control.adpBinding "Matches bus plugins with" "Rust call"
        kmr.control.adpDynamixel -> kmr.control.adpPort "Opens its serial ports through" "Rust call"
        kmr.control.adpDynamixel -> kmr.control.adpIo "Reads and writes each tick through" "Rust call"
        kmr.control.adpIo -> kmr.control.adpBackend "Sends grouped transactions through" "Rust trait"
        kmr.control.adpDxlRsV2 -> kmr.control.adpBackend "Implements" "Rust trait"
        kmr.control.adpCubemars -> kmr.control.adpBinding "Matches CAN interfaces with" "Rust call"

        // C3 kmr_model
        kmr.control.mdlLib -> kmr.control.mdlInput "Parses the macro input with" "Rust call"
        kmr.control.mdlLib -> kmr.control.mdlEmit "Emits the checked robot with" "Rust call"

        // C3 kmr_model_core
        kmr.control.mdcLib -> kmr.control.mdcPresets "Looks presets up in" "Rust call"
        kmr.control.mdcLib -> kmr.control.mdcUrdf "Reads URDF with" "Rust call"
        kmr.control.mdcLib -> kmr.control.mdcSerdeFormat "Reads .ron and JSON with" "Rust call"
        kmr.control.mdcLib -> kmr.control.mdcModel "Validates the model with" "Rust call"
        kmr.control.mdcUrdf -> kmr.control.mdcModel "Implements the reader of" "Rust trait"
        kmr.control.mdcSerdeFormat -> kmr.control.mdcModel "Implements the reader of" "Rust trait"

        // C3 kmr_ffi
        kmr.control.ffiC -> kmr.control.ffiFields "Generates slots and faults from" "Rust macro"
        kmr.control.ffiC -> kmr.control.ffiView "Stages writes in" "Rust type"
        kmr.control.ffiView -> kmr.control.ffiFields "Generates field types from" "Rust macro"
        kmr.control.ffiBuild -> kmr.control.ffiFields "Generates header lines from" "Rust macro"
        kmr.control.ffiBuild -> kmr.control.ffiC "Reads the tick flags of" "Rust call"

        // C3 bench
        kmr.bench.benchTick -> kmr.bench.benchSetup "Builds its robot with" "Rust call"
        kmr.bench.benchTick -> kmr.bench.benchPlan "Reads its run plan from" "Rust call"
        kmr.bench.benchTick -> kmr.bench.benchProbe "Registers the controllers of" "Rust call"
        kmr.bench.benchTick -> kmr.bench.benchCalib "Calibrates the workload with" "Rust call"
        kmr.bench.benchTick -> kmr.bench.benchReport "Writes its figures with" "Rust call"
        kmr.bench.benchProbe -> kmr.bench.benchHist "Records each tick into" "Rust call"
        kmr.bench.benchProbe -> kmr.bench.benchGrid "Replays the tick grid with" "Rust call"
        kmr.bench.benchProbe -> kmr.bench.benchAlloc "Counts heap calls with" "Rust call"
        kmr.bench.benchResults -> kmr.bench.benchReport "Reads the reports written by" "TOML files"
        kmr.bench.benchProbe -> kmr.control "Measures each tick of" "kmr_api"
    }

    views {
        systemContext kmr "Context" "System Context diagram for kmr" {
            subtitle "Who uses one kmr instance and what it talks to."
            include *
            technology off
            maxCrossings 0
            at controlEngineer 840 130
            at contributor 500 130
            at integrator 160 130
            at kmr 480 420
            at hardware 160 970
            at payloads 160 690
            at otherKmr 500 690
            at robotics 840 970
            at compute 840 690
        }

        container kmr "Containers" "Container diagram for kmr" {
            subtitle "What runs on the host and how it talks."
            include *
            maxCrossings 0
            at controlEngineer 1610 130
            at contributor 730 150
            at kmr.control 1200 720
            at kmr.viz 1780 410
            at kmr.model 1820 680
            at kmr.ports 740 680
            at kmr.keys 730 930
            at kmr.bench 740 450
            at hardware 1810 1130
            at payloads 2110 1130
            at otherKmr 610 1180
            at robotics 910 1180
            at compute 1210 1180
            anchor kmr.control -> hardware from bottomRight
        }

        component kmr.control "C3-crates" "Component diagram for Control process: the crates" {
            subtitle "The kmr crates and what they depend on."
            include element.tag==Crate hardware compute kmr.model kmr.keys
            labels callout
            at kmr.control.controllers 1310 140
            at kmr.control.kmrApi 530 130
            at kmr.control.kmrCore 410 520
            at kmr.control.kmrAdapters 670 630
            at kmr.control.kmrModel 950 480
            at kmr.control.kmrModelCore 1270 590
            at kmr.control.kmrFfi 1310 330
            at kmr.model 1640 420
            at kmr.keys 50 110
            at hardware 770 920
            at compute 50 450
            line kmr.control.kmrApi -> compute position 63
        }

        component kmr.control "C3-core" "Component diagram for Control process: kmr_core" {
            subtitle "The engine's modules."
            include element.tag==kmr_core
            labels callout
            at kmr.control.coreRobot 1160 100
            at kmr.control.coreSchedule 770 250
            at kmr.control.coreControllers 370 880
            at kmr.control.coreRuntime 1160 380
            at kmr.control.coreClock 800 780
            at kmr.control.coreSignal 1530 530
            at kmr.control.coreState 800 950
            at kmr.control.coreLimits 1530 240
            at kmr.control.coreHal 1140 1120
            at kmr.control.coreSensors 800 1120
            line kmr.control.coreControllers -> kmr.control.coreClock position 88
        }

        component kmr.control "C3-api" "Component diagram for Control process: kmr_api" {
            subtitle "The public API's modules."
            include element.tag==kmr_api
            labels callout
            at kmr.control.apiRobot 750 100
            at kmr.control.apiRobotMacro 1470 530
            at kmr.control.apiSpec 1480 760
            at kmr.control.apiActuator 590 540
            at kmr.control.apiVendors 590 770
            at kmr.control.apiPlugin 900 930
            at kmr.control.apiController 200 460
            at kmr.control.apiHandles 180 920
            at kmr.control.apiZenoh 960 650
            at kmr.control.apiFfi 90 690
            anchor kmr.control.apiRobotMacro -> kmr.control.apiRobot to right
            anchor kmr.control.apiRobot -> kmr.control.apiController from left
            size kmr.control.apiRobot 500 120
        }

        component kmr.control "C3-adapters" "Component diagram for Control process: kmr_adapters" {
            subtitle "The hardware drivers' modules."
            include element.tag==kmr_adapters
            at kmr.control.adpBackend 560 900
            at kmr.control.adpDxlRsV2 190 660
            at kmr.control.adpBinding 140 380
            at kmr.control.adpPresets 820 380
            at kmr.control.adpDynamixel 480 100
            at kmr.control.adpConfig 820 660
            at kmr.control.adpIo 480 660
            at kmr.control.adpPort 1160 380
            at kmr.control.adpCubemars 140 100
            at kmr.control.adpXiao 990 870
        }

        component kmr.control "C3-ffi" "Component diagram for Control process: kmr_ffi" {
            subtitle "The C bridge's modules."
            include element.tag==kmr_ffi
        }

        component kmr.control "C3-model" "Component diagram for Control process: kmr_model and kmr_model_core" {
            subtitle "The robot model's build-time modules."
            include element.tag==kmr_model
            include element.tag==kmr_model_core
            at kmr.control.mdlLib 480 100
            at kmr.control.mdlInput 480 380
            at kmr.control.mdlEmit 140 380
            at kmr.control.mdcLib 1160 380
            at kmr.control.mdcModel 820 940
            at kmr.control.mdcUrdf 820 660
            at kmr.control.mdcSerdeFormat 1170 950
            at kmr.control.mdcPresets 1480 650
        }

        component kmr.bench "C3-bench" "Component diagram for Bench: its modules" {
            subtitle "What one bench run measures and writes."
            include element.tag==bench kmr.control
            at kmr.control 160 940
            at kmr.bench.benchTick 580 660
            at kmr.bench.benchSetup 840 340
            at kmr.bench.benchPlan 1190 680
            at kmr.bench.benchProbe 580 930
            at kmr.bench.benchHist 1180 940
            at kmr.bench.benchGrid 1190 1250
            at kmr.bench.benchCalib 580 340
            at kmr.bench.benchAlloc 1180 1090
            at kmr.bench.benchReport 1190 450
            at kmr.bench.benchResults 1190 190
            line kmr.bench.benchProbe -> kmr.bench.benchAlloc position 72
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
            element "compute" {
                background #5a6292
            }
            element "otherKmr" {
                background #5a6292
            }
            element "robotics" {
                background #5a6292
            }
            element "Component" {
                background #85bbf0
                color #000000
            }
            element "Crate" {
                background #438dd5
                color #ffffff
            }
            element "Entry" {
                background #7cb342
                color #000000
            }
            element "Loop" {
                background #ef6c00
                color #000000
            }
            element "Safety" {
                background #c62828
                color #ffffff
            }
            element "State" {
                background #fdd835
                color #000000
            }
            element "Hardware" {
                background #795548
                color #ffffff
            }
            element "Bridge" {
                background #8e24aa
                color #ffffff
            }
            element "Build" {
                background #00796b
                color #ffffff
            }
        }
    }

}
