//! The CubeMars hardware: the models, for [`Robot::add_actuator`](crate::Robot::add_actuator),
//! and the table, for `robot!(model = .., hardware = { cubemars: .. })`.
//!
//! ```ignore
//! let leg = kmr_api::Bus::by_dev("can0").build();
//! Robot::new().add_actuator(kmr_api::cubemars::Ak60::ids(1..=3).gains(150.0, 1.0).on(leg))
//! ```
//!
//! Each declared actuator runs in MIT mode (`MitAk2`) on cubemars_rs. The pose it stands in at
//! bring-up is its joint 0, unless [`start`](Ids::start) says otherwise; the motor's stored zero
//! is never used nor written. Its limits are checked by cubemars_rs on every command and every
//! reply. Declaring needs no feature; driving them needs `kmr_api/cubemars`.

use core::ops::RangeInclusive;

/// The CubeMars driver [`Bound::drivers`](crate::Bound::drivers) hands out.
#[cfg(feature = "cubemars")]
pub use kmr_adapters::cubemars::CubeMars;
/// Why a CubeMars bind or bring-up failed; reach it with `downcast_ref`.
#[cfg(feature = "cubemars")]
pub use kmr_adapters::cubemars::CubeMarsError;
/// The SocketCAN opener of the CubeMars driver.
#[cfg(feature = "cubemars")]
pub use kmr_adapters::cubemars::SocketCan;
pub use kmr_adapters::cubemars::{
    Actuator, Bus, DEFAULT_KD, DEFAULT_KP, Limits, Model, Placement, Table,
};

use crate::actuator::{Actuators, Driver, IdSet};
use crate::{BusHandle, Q, Qd, Tau};

/// Actuators of one model at a range of CAN ids, not yet on a bus.
#[derive(Clone, Debug, PartialEq)]
pub struct Ids {
    ids: IdSet,
    row: Actuator,
}

impl Ids {
    /// The actuators of `model` at `ids`, with the defaults.
    fn new(model: Model, ids: IdSet) -> Self {
        Self {
            ids,
            row: Actuator::new(model, 0, 0),
        }
    }

    /// The row, `change`d.
    fn with(mut self, change: impl FnOnce(&mut Actuator)) -> Self {
        change(&mut self.row);
        self
    }

    /// Sends these gains on a tick that writes none: `kp` in `0..=500`, `kd` in `0..=5`.
    /// Default [`DEFAULT_KP`] and [`DEFAULT_KD`]; `run()` refuses a gain outside the range.
    pub fn gains(self, kp: f32, kd: f32) -> Self {
        self.with(|row| (row.kp, row.kd) = (kp, kd))
    }

    /// The joint position of the pose these actuators stand in at bring-up. Default 0.
    pub fn start(self, q: Q) -> Self {
        self.with(|row| row.start = q.0)
    }

    /// Mounted the other way round: positions, velocities and torques are negated.
    pub fn reversed(self) -> Self {
        self.with(|row| row.reversed = true)
    }

    /// Allowed joint positions; a goal outside is refused, a reply outside cuts the torque.
    pub fn position_limit(self, range: RangeInclusive<Q>) -> Self {
        self.with(|row| row.limits.position = Some((range.start().0, range.end().0)))
    }

    /// Largest joint speed either way, on goals and replies.
    pub fn velocity_limit(self, max: Qd) -> Self {
        self.with(|row| row.limits.velocity = Some(max.0))
    }

    /// Largest torque either way, on goals and replies.
    pub fn torque_limit(self, max: Tau) -> Self {
        self.with(|row| row.limits.torque = Some(max.0))
    }

    /// Highest temperature, °C; a reply above it cuts the torque, and bring-up refuses it.
    pub fn temperature_limit(self, celsius: f32) -> Self {
        self.with(|row| row.limits.temperature = Some(celsius))
    }

    /// Largest distance between a position goal and the last measured position.
    pub fn max_goal_step(self, step: Q) -> Self {
        self.with(|row| row.limits.max_goal_step = Some(step.0))
    }

    /// These actuators on `bus`, a CAN interface: `Bus::by_dev("can0").build()`.
    pub fn on(self, bus: BusHandle) -> Actuators {
        Actuators {
            bus,
            ids: self.ids,
            driver: Driver::CubeMars(self.row),
        }
    }
}

/// Declares one model type per CubeMars [`Model`].
macro_rules! models {
    ($($(#[$doc:meta])* $name:ident => $model:ident,)*) => {$(
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub struct $name;

        impl $name {
            /// The actuator at CAN id `id`.
            pub fn id(id: u8) -> Ids {
                Ids::new(Model::$model, IdSet::Range(id..=id))
            }

            /// The actuators at CAN ids `ids`, in this order.
            pub fn ids(ids: RangeInclusive<u8>) -> Ids {
                Ids::new(Model::$model, IdSet::Range(ids))
            }

            /// One actuator per actuator of the robot model, at CAN ids `1..=N`.
            pub fn all() -> Ids {
                Ids::new(Model::$model, IdSet::All)
            }
        }
    )*};
}

models! {
    /// AK60-6, MIT mode on an AK 2.0 board.
    Ak60 => Ak60_6,
    /// AK80-8, MIT mode on an AK 2.0 board.
    Ak80 => Ak80_8,
}
