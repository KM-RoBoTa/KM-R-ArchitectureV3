//! A 100-actuator descriptor through the model macro: its build-time cost
//! (FR-021, SRS NFR-11). Direct call, since N = 100 is above `robot!`'s caps.

mod hundred {
    kmr_api::__model!(kmr_api; model = "tests/fixtures/hundred.urdf");

    #[test]
    fn hundred_joints_give_n_100_in_file_order() {
        assert_eq!(N, 100);
        assert_eq!(model::NAMES[0], "j1");
        assert_eq!(model::NAMES[99], "j100");
        assert_eq!(model::LINK_NAMES.len(), 101);
        assert_eq!(model::LINK_PARENT[100], Some(99));
        assert_eq!(model::LOWER[99], kmr_api::Q(-1.57));
    }
}
