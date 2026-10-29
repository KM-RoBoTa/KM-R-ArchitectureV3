//! Your controllers live here. This crate depends on `kmr_api` only.

use kmr_api::{EachTick, Q, Sensors, Time, UserError};
use std::time::Duration;

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
