//! A Dynamixel part that drives a prismatic actuator does not build (SRS FR-2.2): the
//! placement puts row 0 on four.urdf's j3, engine index 2.

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
    hardware = {
        dynamixel: Table {
            buses: [Bus { label: "arm", baud: 1_000_000, read_every: None }],
            actuators: [xm430(1), xm430(2), xm430(3), xm430(4)],
        }
        .at([2, 0, 1, 3])
    }
);

fn main() {}
