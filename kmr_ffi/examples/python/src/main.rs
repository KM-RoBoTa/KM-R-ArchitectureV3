//! A controller whose math is Python: `step` in swing.py moves every actuator by 0.1 rad a
//! tick. Without a bus plugin it simulates; with no sample to start from, the script asks for a
//! shutdown.

use std::time::Duration;

use kmr_api::ffi::python::Bridge;
use kmr_api::{EachTick, Sensors, Time};

kmr_api::robot!(XmBench);

/// Hands the tick to the script; an error stops the robot.
fn forward(bridge: &mut Bridge, time: &Time, state: &mut State, _: &Sensors) {
    if kmr_api::ffi::call(bridge, time, state, None::<&mut ()>).is_err() {
        kmr_api::emergency_stop!("swing.py failed");
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bridge = Bridge::load(concat!(env!("CARGO_MANIFEST_DIR"), "/swing.py"), "step")?;
    Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_controller_with(EachTick, bridge, forward)
        .run()?;
    Ok(())
}
