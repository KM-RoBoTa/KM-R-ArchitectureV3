#![allow(unused_variables)]
#![allow(dead_code)]
#![deny(clippy::unwrap_used)]
// #![deny(clippy::todo)]

mod group;
pub use group::GroupView;

mod sensors;
pub use sensors::*;

mod error;
pub use error::*;

mod state;
pub use state::*;

mod time;
pub use time::Time;

mod payload;
use payload::Payload;

// Schedule labels + the sealed `Schedule` trait come straight from kmr_core: it
// owns the phase set the control loop can actually run, and the seal stops users
// inventing a phase that would silently do nothing.
pub use kmr_core::{EachTick, First, Init, Last, PostInit, PreInit, Schedule};

trait FakeSensor {}
trait FakeModel {}

mod robot;
pub use robot::Robot;

pub use kmr_core::{emergency_stop, shutdown};
