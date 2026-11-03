//
// controllers is the control engineer's crate: one binary, depending on
// kmr_api only, linked with it into the control process. src/main.rs is the
// reference a new user copies. Drawn as it is on this branch: main.rs still
// holds the placeholder nudge_first_joint; the swing reference (a Swing
// context on EachTick ending with shutdown! after 12 s) is not written yet.
// The relations into kmr_api are drawn in kmr_api.dsl, included after this
// file.

!element kmr.control {
    ctrlMain = component "controllers: main" "The user's binary: names the robot with robot!(XmBench), builds Robot with dt = 1 ms and one controller on EachTick, returns run()'s Result. No bus line as shipped, so it simulates; the bench adds Bus::by_id (controllers/README.md)." "Rust binary, kmr_api only"

    group "controllers: main" {
        ctrlRobotLine = component "robot!(XmBench)" "Emits N, State, Robot and the hardware spec of the XmBench preset." "Rust macro call" {
            tags "Code"
        }
        ctrlNudge = component "nudge_first_joint" "Placeholder controller: adds 0.01 rad to actuator 0 each tick, errors ignored." "Rust fn" {
            tags "Code"
        }
        ctrlMainFn = component "main" "Robot::new().set_dt(1 ms).add_controller(EachTick, ..).run()." "Rust fn" {
            tags "Code"
        }
    }

    ctrlMainFn -> ctrlRobotLine "Uses the Robot alias of"
    ctrlMainFn -> ctrlNudge "Registers on EachTick"
}
