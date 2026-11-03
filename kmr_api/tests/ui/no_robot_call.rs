//! `main.rs` without a `robot!` call: a controller that names `State` does not build, the
//! actuator count is the descriptor's and nothing gave one.

use kmr_api::{Sensors, State, Time};

fn hold(_t: &Time, _robot: &mut State, _s: &Sensors) {}

fn main() {}
