//! Rx28Chain: its model half counts the actuators of its hardware half, and
//! under `joints-10` the front door builds it.

use kmr_api::presets::{Preset, Rx28Chain};

#[test]
fn the_urdf_counts_the_actuators_of_the_hardware_table() {
    const URDF: usize = kmr_api::__actuator_count!("../kmr_model/presets/rx28_chain.urdf");
    assert_eq!(URDF, <Rx28Chain as Preset>::N);
    assert_eq!(URDF, Rx28Chain::HARDWARE.actuators.len());
}

#[cfg(feature = "joints-10")]
mod front_door {
    kmr_api::robot!(Rx28Chain);

    #[test]
    fn robot_gives_ten_actuators_on_one_bus() {
        assert_eq!((N, __NB, __M), (10, 1, 10));
        assert_eq!(__TABLE.buses[0].label, "bus0");
    }
}
