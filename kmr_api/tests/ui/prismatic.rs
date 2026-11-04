//! A prismatic actuator on a Dynamixel table does not build: it has no mm conversion
//! (SRS FR-2.2). four.urdf's j3, index 2, is prismatic.

use kmr_api::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};

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

kmr_api::robot!(
    model = "../../../../kmr_api/tests/fixtures/four.urdf",
    hardware = Table {
        buses: [Bus { label: "arm", baud: 1_000_000, read_every: None }],
        actuators: [xm430(1), xm430(2), xm430(3), xm430(4)],
    }
);

fn main() {}
