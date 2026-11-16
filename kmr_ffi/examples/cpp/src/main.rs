//! A controller written in C++: `swing` in src/swing.cpp moves actuators 0 and 2 by 0.1 rad a
//! tick, through the methods of `kmr.hpp`. Without a bus plugin it simulates; with no sample to
//! start from, `swing` asks for an emergency stop and the robot stops.

use std::time::Duration;

use kmr_api::{EachTick, UserError};

kmr_api::robot!(XmBench);

fn main() -> Result<(), UserError> {
    Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_controller(EachTick, kmr_api::c!(swing_cpp))
        .run()
}
