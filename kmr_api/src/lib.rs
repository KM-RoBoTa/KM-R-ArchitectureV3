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

/// The limit of an actuator, what a breach of it does, and the envelope a controller's
/// writes are clamped to (SRS FR-4.5, FR-13.5).
pub use kmr_core::limits::{Envelope, Limit, Policy};

mod faults;
pub use faults::{Faults, Mask};

mod fallback;
#[doc(hidden)]
pub use fallback::Guarded;
pub use fallback::{any_of, remote_timeout, stale_in};

mod home;
pub use home::HomeDecided;

mod time;
pub use time::Time;

mod units;
pub use units::*;

mod payload;
pub use payload::{Payload, Tier};

mod slot;
pub use slot::{Latest, LatestReader, LatestWriter};

// Schedule labels + the sealed `Schedule` trait come straight from kmr_core: it
// owns the phase set the control loop can actually run, and the seal stops users
// inventing a phase that would silently do nothing.
pub use kmr_core::{EachTick, First, Init, Last, PostInit, PreInit, Schedule};

mod robot;
/// What every built robot's schedule is: write `impl Drive` to return a robot
/// from a helper function.
pub use kmr_core::Drive;
/// The empty schedule a new [`Robot`] starts from.
pub use kmr_core::Scheduled;
/// The controller type-state of a [`Robot`]: `Empty` until the first controller, then
/// `Filled`; `run()` needs [`HasController`]. Named only to return a robot from a helper.
#[doc(hidden)]
pub use kmr_core::{Empty, Filled, HasController};
pub use robot::{Bound, Robot};

/// Why the robot's hardware could not run: the members' claims, or one
/// member's error. Reach it from [`UserError::BringUp`] with `downcast_ref`.
pub use kmr_core::hal::HardwareError;
/// Words of a write mask and of the kind bit sets `robot!` emits
/// (`model::PRISMATIC`, `model::CONTINUOUS`): the path kmr_model's output names.
#[doc(hidden)]
pub use kmr_core::hal::MASK_WORDS;
/// One member's error inside a [`HardwareError`], by its position in the list.
pub use kmr_core::hal::MemberError;
/// The drivers of a robot without a bus plugin: it simulates, reading and
/// writing nothing.
pub use kmr_core::hal::NoHardware;

mod brand;
mod spec;
#[doc(hidden)]
pub use brand::{__bind, Brand, Brands, Refused, Route};
#[doc(hidden)]
pub use kmr_adapters::{
    binding::Binding,
    dynamixel::{Placement, Table},
};
/// A list of drivers, as [`Bound::drivers`] hands it out: `HCons { head, tail }`.
pub use kmr_core::HCons;
/// The end of a list of drivers.
pub use kmr_core::HNil;
#[doc(hidden)]
pub use spec::{
    __CubeMarsAdapter, __DxlAdapter, __NoHardware, __bind_none, __brand, __check_coverage, Spec,
};

mod bus;
pub use bus::Bus;

mod plugin;
pub use plugin::Plugin;

mod robot_macro;

/// The robots KM-RoBoTa ships, for [`robot!`].
pub use kmr_adapters::presets;

/// The Dynamixel hardware table, for `robot!(model = .., hardware = { dynamixel: .. })`.
pub mod dynamixel {
    /// The `dxl_rs` backend of the Dynamixel driver.
    #[cfg(feature = "dynamixel")]
    pub use kmr_adapters::dynamixel::DxlRs;
    /// The Dynamixel driver [`Bound::drivers`](crate::Bound::drivers) hands out.
    pub use kmr_adapters::dynamixel::Dynamixel;
    /// Why a Dynamixel bind or bring-up failed; reach it with `downcast_ref`.
    pub use kmr_adapters::dynamixel::DynamixelError;
    pub use kmr_adapters::dynamixel::{
        Actuator, Bus, DEFAULT_READ_EVERY, Mode, Model, Placement, Table, Torque, Zero,
    };
}

/// The CubeMars hardware table, for `robot!(model = .., hardware = { cubemars: .. })`.
pub mod cubemars {
    /// The CubeMars driver [`Bound::drivers`](crate::Bound::drivers) hands out.
    pub use kmr_adapters::cubemars::CubeMars;
    /// Why a CubeMars bind or bring-up failed; reach it with `downcast_ref`.
    pub use kmr_adapters::cubemars::CubeMarsError;
    /// The SocketCAN transport of the CubeMars driver.
    #[cfg(feature = "cubemars")]
    pub use kmr_adapters::cubemars::SocketCan;
    pub use kmr_adapters::cubemars::{Actuator, Bus, DEFAULT_KD, DEFAULT_KP, Placement, Table};
}

#[doc(hidden)]
pub use kmr_model::model as __model;

pub use kmr_core::{emergency_stop, shutdown};

#[cfg(feature = "ffi")]
pub mod ffi;

#[cfg(any(test, feature = "test-util"))]
pub mod test_util;
