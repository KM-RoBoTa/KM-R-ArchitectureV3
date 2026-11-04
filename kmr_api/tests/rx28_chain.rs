//! Rx28Chain: its model half counts the actuators of its hardware half, and
//! the front door builds it, N = 10.

mod model_half {
    use kmr_api::presets::Rx28Chain;

    kmr_api::__model!(kmr_api; preset = Rx28Chain);

    #[test]
    fn the_model_half_counts_the_actuators_of_the_hardware_half() {
        assert_eq!(N, 10);
        assert_eq!(N, Rx28Chain::HARDWARE.actuators.len());
    }
}

mod front_door {
    kmr_api::robot!(Rx28Chain);

    #[test]
    fn robot_gives_ten_actuators_on_one_bus() {
        assert_eq!((N, __NB, __M), (10, 1, 10));
        assert_eq!(__TABLE.buses[0].label, "bus0");
    }
}
