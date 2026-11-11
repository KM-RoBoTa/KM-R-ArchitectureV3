//! The Dynamixel hardware: the table, for `robot!(model = .., hardware = { dynamixel: .. })`,
//! and the models, for [`Robot::add_actuator`](crate::Robot::add_actuator).
//!
//! ```ignore
//! let spine = kmr_api::Bus::by_id("AL02L1KL").baud(1_000_000).build();
//! Robot::new().add_actuator(kmr_api::dynamixel::Rx28::ids(1..=10).on(spine))
//! // or, one per model actuator at ids 1..=N:
//! Robot::new().add_actuator(kmr_api::dynamixel::Rx28::all().on(spine))
//! ```
//!
//! Each declared actuator is powered, in position mode, zero at the model's centre, unless set
//! otherwise before [`on`](Ids::on). Declaring needs no feature; driving them needs
//! `kmr_api/dynamixel`.

use core::ops::RangeInclusive;

/// The `dxl_rs` backend of the Dynamixel driver.
#[cfg(feature = "dynamixel")]
pub use kmr_adapters::dynamixel::DxlRs;
/// The Dynamixel driver [`Bound::drivers`](crate::Bound::drivers) hands out.
pub use kmr_adapters::dynamixel::Dynamixel;
/// Why a Dynamixel bind or bring-up failed; reach it with `downcast_ref`.
pub use kmr_adapters::dynamixel::DynamixelError;
/// Health limits a placement writes to its actuators at bring-up:
/// `TABLE.at([..]).health_limits(..)` (D18).
pub use kmr_adapters::dynamixel::HealthLimits;
/// The rule of the hardware table the declared actuators break, inside
/// [`UserError::HardwareRejected`](crate::UserError::HardwareRejected).
pub use kmr_adapters::dynamixel::TableError;
pub use kmr_adapters::dynamixel::{
    Actuator, Bus, DEFAULT_READ_EVERY, Mode, Model, Placement, Table, Torque, Zero,
};

use crate::BusHandle;
use crate::actuator::{Actuators, Driver, IdSet};

/// Actuators of one model at a range of firmware ids, not yet on a bus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Ids {
    ids: IdSet,
    model: Model,
    mode: Mode,
    torque: Torque,
    zero: Zero,
}

impl Ids {
    /// The actuators of `model` at `ids`, with the defaults.
    fn new(model: Model, ids: IdSet) -> Self {
        Self {
            ids,
            model,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
        }
    }

    /// Sets the operating mode. Only `Position` runs today; `run()` refuses the others.
    pub fn mode(self, mode: Mode) -> Self {
        Self { mode, ..self }
    }

    /// Reads these actuators and never powers them.
    pub fn read_only(self) -> Self {
        Self {
            torque: Torque::ReadOnly,
            ..self
        }
    }

    /// Sets where 0 rad sits.
    pub fn zero(self, zero: Zero) -> Self {
        Self { zero, ..self }
    }

    /// These actuators on `bus`.
    pub fn on(self, bus: BusHandle) -> Actuators {
        Actuators {
            bus,
            ids: self.ids,
            driver: Driver::Dynamixel {
                model: self.model,
                mode: self.mode,
                torque: self.torque,
                zero: self.zero,
            },
        }
    }
}

/// Declares one model type per Dynamixel [`Model`].
macro_rules! models {
    ($($(#[$doc:meta])* $name:ident => $model:ident,)*) => {$(
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub struct $name;

        impl $name {
            /// The actuator at firmware id `id`.
            pub fn id(id: u8) -> Ids {
                Ids::new(Model::$model, IdSet::Range(id..=id))
            }

            /// The actuators at firmware ids `ids`, in this order.
            pub fn ids(ids: RangeInclusive<u8>) -> Ids {
                Ids::new(Model::$model, IdSet::Range(ids))
            }

            /// One actuator per actuator of the robot model, at firmware ids `1..=N`.
            pub fn all() -> Ids {
                Ids::new(Model::$model, IdSet::All)
            }
        }
    )*};
}

models! {
    /// AX-12A, Protocol 1.
    Ax12a => Ax12a,
    /// RX-28, Protocol 1.
    Rx28 => Rx28,
    /// MX-64 on its Protocol 1 firmware.
    Mx64 => Mx64,
    /// MX-64(2.0): the MX-64 on its Protocol 2.0 firmware.
    Mx64V2 => Mx64V2,
    /// XM430-W210, Protocol 2.
    Xm430W210 => Xm430W210,
    /// XW430-T200, Protocol 2.
    Xw430T200 => Xw430T200,
    /// XW540-T140, Protocol 2.
    Xw540T140 => Xw540T140,
    /// XW540-T260, Protocol 2.
    Xw540T260 => Xw540T260,
}
