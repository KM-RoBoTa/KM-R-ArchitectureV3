//! `robot!` from a URDF: N is read at compile time and the aliases work.

kmr_api::robot!(model = "tests/fixtures/four.urdf");

use kmr_api::{EachTick, Q, Sensors, Time};

fn zero(_t: &Time, robot: &mut State, _s: &Sensors) {
    robot.set_all_q([Q(0.0); N]);
}

#[test]
fn urdf_gives_n_without_fixed_joints_or_transmissions() {
    assert_eq!(N, 4);
}

#[test]
fn aliases_build_a_robot() {
    let _robot = Robot::new().add_controller(EachTick, zero);
}

mod preset {
    kmr_api::robot!(XmBench);

    #[test]
    fn preset_gives_n() {
        assert_eq!(N, 4);
    }

    #[test]
    fn preset_gives_its_hardware_table() {
        assert_eq!((__NB, __M), (1, N));
        assert_eq!(__TABLE.buses[0].label, "bus0");
    }
}
