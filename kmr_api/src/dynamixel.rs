//! The Dynamixel hardware: the table, for `robot!(model = .., hardware = { dynamixel: .. })`,
//! and the models, for [`Robot::add_actuator`](crate::Robot::add_actuator).
//!
//! ```ignore
//! let spine = kmr_api::Bus::by_id("AL02L1KL").baud(1_000_000).build();
//! Robot::new().add_actuator(kmr_api::dynamixel::Rx28::ids(1..=10).on(spine))
//! ```
//!
//! Each declared actuator is powered, in position mode, zero at the model's centre, its gains and
//! LED as it holds them, unless set otherwise before [`on`](Ids::on). Declaring needs no feature;
//! driving them needs `kmr_api/dynamixel`.

use core::ops::RangeInclusive;

/// The `dxl_rs_v2` backend of the Dynamixel driver.
#[cfg(feature = "dynamixel")]
pub use kmr_adapters::dynamixel::DxlRsV2;
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

use kmr_adapters::dynamixel::{Gains, Settings};

use crate::BusHandle;
use crate::actuator::{Actuators, Driver};

/// Actuators of one model at a range of firmware ids, not yet on a bus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Ids {
    ids: RangeInclusive<u8>,
    model: Model,
    mode: Mode,
    torque: Torque,
    zero: Zero,
    settings: Settings,
}

impl Ids {
    /// The actuators of `model` at `ids`, with the defaults.
    fn new(model: Model, ids: RangeInclusive<u8>) -> Self {
        Self {
            ids,
            model,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
            settings: Settings::default(),
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

    /// Writes these P, I and D gains at bring-up, before the torque goes on, as register values:
    /// 0 to 254 on the MX-64, 0 to 16383 on Protocol 2. `run()` refuses them on the AX-12A and
    /// RX-28, which have no gain register, and above the model's range.
    pub fn gains(self, p: u16, i: u16, d: u16) -> Self {
        let gains = Some(Gains { p, i, d });
        Self {
            settings: Settings {
                gains,
                ..self.settings
            },
            ..self
        }
    }

    /// Switches the LED on or off at bring-up. `run()` refuses it on the XW series, which has no
    /// LED.
    pub fn led(self, on: bool) -> Self {
        Self {
            settings: Settings {
                led: Some(on),
                ..self.settings
            },
            ..self
        }
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
                settings: self.settings,
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
                Ids::new(Model::$model, id..=id)
            }

            /// The actuators at firmware ids `ids`, in this order.
            pub fn ids(ids: RangeInclusive<u8>) -> Ids {
                Ids::new(Model::$model, ids)
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
