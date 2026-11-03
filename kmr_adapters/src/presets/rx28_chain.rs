//! Rx28Chain: the dev bench chain of ten RX-28 on one bus, Protocol 1.
//!
//! Protocol 1 has no sync read: a read is one round trip per servo, an
//! 8-byte instruction and an 8-byte status, 160 µs on the wire at 1 Mbit/s,
//! plus the Return Delay Time (factory 250 = 500 µs) and about 1 ms of USB
//! latency on an FTDI adapter at a 1 ms latency timer: about 1.2 ms per servo
//! at Return Delay Time 0, 12 ms for the ten, 17 ms at the factory delay. The
//! write is one Protocol 1 sync write of 38 bytes, 0.4 ms. A due bus is read
//! whole within one tick, so the chain is read every tick and wants a `dt` of
//! 20 ms or more at 1 Mbit/s; at 57 600 bit/s a read is about 40 ms, `dt`
//! 50 ms. Measured on no bench yet.

use core::num::NonZeroU32;
use core::time::Duration;

use crate::dynamixel::{self, Actuator, Bus, Mode, Model, Table, Torque, Zero};

/// Firmware ids, in engine order: actuator `i` has `IDS[i]`.
const IDS: [u8; 10] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];

/// Bus baud. The RX-28 leaves the factory at 57_600 (Baud Rate register 34);
/// 1_000_000 is register 1, set in Dynamixel Wizard.
const BAUD: u32 = 1_000_000;

/// The dev bench chain: ten RX-28 on one bus.
pub struct Rx28Chain;

impl Rx28Chain {
    /// [`IDS`] on bus `bus0` at [`BAUD`], read every tick.
    pub const HARDWARE: Table<1, 10> = Table {
        buses: [Bus {
            label: "bus0",
            baud: BAUD,
            read_every: NonZeroU32::new(1),
        }],
        actuators: chain(IDS),
    }
    .check();

    /// Line time of one full read of [`Self::HARDWARE`], as `check`
    /// declares it, return delay and USB latency not counted (FR-035 (SRS
    /// NFR-11, FR-5.3)).
    pub const READ_COST: Duration = dynamixel::declared_costs(&Self::HARDWARE).0;

    /// Line time of one goal write to every actuator of
    /// [`Self::HARDWARE`], as `check` declares it.
    pub const WRITE_COST: Duration = dynamixel::declared_costs(&Self::HARDWARE).1;
}

/// One [`rx28`] per id of `ids`, in order.
const fn chain<const N: usize>(ids: [u8; N]) -> [Actuator; N] {
    let mut actuators = [rx28(0); N];
    let mut index = 0;
    while index < N {
        actuators[index] = rx28(ids[index]);
        index += 1;
    }
    actuators
}

/// The RX-28 at firmware id `id`: powered, in position (joint) mode, zero at
/// the centre, raw 512.
const fn rx28(id: u8) -> Actuator {
    Actuator {
        bus: 0,
        id,
        model: Model::Rx28,
        mode: Mode::Position,
        torque: Torque::On,
        zero: Zero::Center,
    }
}
