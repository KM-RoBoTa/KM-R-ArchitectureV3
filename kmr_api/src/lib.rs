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

mod units;
pub use units::*;

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

mod spec;
#[doc(hidden)]
pub use kmr_adapters::{binding::Binding, dynamixel::Table};
#[doc(hidden)]
pub use spec::{__DxlAdapter, __bind_dynamixel, Spec};

mod bus;
pub use bus::Bus;

mod robot_macro;

/// The robots KM-RoBoTa ships, for [`robot!`].
pub use kmr_adapters::presets;

#[doc(hidden)]
pub use kmr_model::actuator_count as __actuator_count;

pub use kmr_core::{emergency_stop, shutdown};
