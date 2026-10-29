//! Your controllers live here. This crate depends on `kmr_api` only.

use kmr_api::{EachTick, Q, Sensors, Time, UserError};
use std::time::Duration;

// RX-28 bench (ten RX-28 on the FTDI adapter AL02L1KL), two lines change:
//     kmr_api::robot!(Rx28Chain);                      // instead of the line below
//     .add_plugins(kmr_api::Bus::by_id("AL02L1KL"))   // in main(); `.baud(57_600)` at factory baud
// then `cargo run -p controllers --features hardware,rx28`. Protocol 1 reads one servo
// per round trip, ~12 ms for ten: set_dt(20 ms) or more. Ids and baud: one const line
// each in kmr_adapters/src/presets/rx28_chain.rs.
kmr_api::robot!(XmBench);

fn nudge_first_joint(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    if let Ok(q) = robot.q_at(0) {
        let _ = robot.set_q_at(q + Q(0.01), 0);
    }
}

fn main() -> Result<(), UserError> {
    Robot::new()
        .set_dt(Duration::from_millis(1))
        // Bench: uncomment with the U2D2's serial (`ls /dev/serial/by-id`), then
        // `cargo run -p controllers --features hardware`. Without it, run() simulates.
        // .add_plugins(kmr_api::Bus::by_id("<U2D2 serial>"))
        .add_controller(EachTick, nudge_first_joint)
        .run()
}
