//! XmBench: the bench chain of four XM430-W210 on one bus.

use super::Preset;
use crate::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};

/// The bench chain: four XM430-W210 on one bus.
pub struct XmBench;

impl XmBench {
    /// Ids 1 to 4 on bus `bus0` at 1 Mbit/s, read every
    /// [`DEFAULT_READ_EVERY`](crate::dynamixel::DEFAULT_READ_EVERY) ticks.
    pub const HARDWARE: Table<1, 4> = Table {
        buses: [Bus {
            label: "bus0",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: [xm430(1), xm430(2), xm430(3), xm430(4)],
    }
    .check();
}

impl Preset for XmBench {
    const N: usize = Self::HARDWARE.actuators.len();
}

/// The XM430-W210 at firmware id `id`: powered, in position mode, zero at
/// the centre.
const fn xm430(id: u8) -> Actuator {
    Actuator {
        bus: 0,
        id,
        model: Model::Xm430W210,
        mode: Mode::Position,
        torque: Torque::On,
        zero: Zero::DEFAULT,
    }
}
