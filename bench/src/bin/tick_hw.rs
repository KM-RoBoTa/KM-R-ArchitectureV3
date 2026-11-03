//! `tick_hw`: `tick` on the reference chain (FR-004, FR-005, FR-015 (SRS FR-6.1, FR-6.3, FR-5.3)).
//!
//! Assumed, pending human: D10 (XmBench, one U2D2); the one place with
//! PROCEDURE.md "Reference chain". One bus, so no bus label (D6). With the
//! `two-bus` feature, the same four XM430 over two U2D2, actuators alternating
//! (research R15; assumed, pending human: D6, D7).

use std::process::ExitCode;

use kmr_api::Bus;
use kmr_bench::alloc::Counting;
use kmr_bench::buf::Buf;
use kmr_bench::tick::{self, Bench};

#[cfg(not(feature = "two-bus"))]
kmr_api::robot!(XmBench);

#[cfg(feature = "two-bus")]
kmr_api::robot!(
    model = "../kmr_model/presets/xm_bench.urdf",
    hardware = two_bus::TABLE
);

#[global_allocator]
static GLOBAL: Counting = Counting;

/// The U2D2's serial id, from /dev/serial/by-id: the one line two benches differ in.
const SERIAL: &str = "FT8ISZ6H";

/// The second U2D2 of the two-bus chain.
///
/// DECISION: no second adapter is named anywhere yet; this placeholder fails the bind with
/// the port's not-found error until a human writes the real serial id here.
#[cfg(feature = "two-bus")]
const SERIAL_2: &str = "SECOND-U2D2-UNSET";

/// XmBench's model over two buses, actuators alternating between them.
#[cfg(feature = "two-bus")]
mod two_bus {
    use kmr_api::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};

    /// Ids 1 and 3 on `bus0`, 2 and 4 on `bus1`, both at 1 Mbit/s.
    pub const TABLE: Table<2, 4> = Table {
        buses: [
            Bus {
                label: "bus0",
                baud: 1_000_000,
                read_every: None,
            },
            Bus {
                label: "bus1",
                baud: 1_000_000,
                read_every: None,
            },
        ],
        actuators: [xm430(0, 1), xm430(1, 2), xm430(0, 3), xm430(1, 4)],
    }
    .check();

    /// An XM430-W210 as XmBench has it, on bus index `bus`.
    const fn xm430(bus: usize, id: u8) -> Actuator {
        Actuator {
            bus,
            id,
            model: Model::Xm430W210,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
        }
    }
}

fn main() -> ExitCode {
    #[cfg(not(feature = "two-bus"))]
    let plugins = [Bus::by_id(SERIAL)];
    #[cfg(feature = "two-bus")]
    let plugins = [
        Bus::by_id(SERIAL).bus("bus0"),
        Bus::by_id(SERIAL_2).bus("bus1"),
    ];
    let mut buses = Buf::<128>::new();
    #[cfg(not(feature = "two-bus"))]
    let serials = [SERIAL];
    #[cfg(feature = "two-bus")]
    let serials = [SERIAL, SERIAL_2];
    for (i, serial) in serials.into_iter().enumerate() {
        let sep = if i == 0 { "" } else { "," };
        let _ = std::fmt::Write::write_fmt(&mut buses, format_args!("{sep}by_id:{serial}"));
    }
    tick::main::<__Spec>(&Bench {
        binary: "tick_hw",
        robot: "XmBench",
        robot_n: N,
        plugins: &plugins,
        buses: buses.as_str(),
    })
}
