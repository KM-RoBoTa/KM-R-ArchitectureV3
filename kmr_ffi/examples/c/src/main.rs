//! A controller whose math is C: `swing` in src/swing.c moves actuators 0 and 2 by 0.1 rad a
//! tick. Without a bus plugin it simulates; with no sample to start from, `swing` returns an
//! error and the robot stops.

use std::time::Duration;

use kmr_api::{EachTick, Sensors, Time, UserError};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn swing; }

/// Hands the tick to `swing`; an error stops the robot.
fn forward(time: &Time, state: &mut State, _: &Sensors) {
    if kmr_api::ffi::call(swing, time, state, None::<&mut ()>).is_err() {
        kmr_api::emergency_stop!("swing failed");
    }
}

fn main() -> Result<(), UserError> {
    Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_controller(EachTick, forward)
        .run()
}
