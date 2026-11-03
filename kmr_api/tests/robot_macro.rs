//! `robot!` from a URDF: N, names, limits, links and `actuator!` are read at compile time.

kmr_api::robot!(model = "tests/fixtures/four.urdf");

use kmr_api::{EachTick, Q, Qd, Sensors, Tau, Time};

fn zero(_t: &Time, robot: &mut State, _s: &Sensors) {
    robot.set_all_q([Q(0.0); N]);
}

#[test]
fn urdf_gives_n_and_names_without_fixed_joints_or_transmissions() {
    assert_eq!(N, 4);
    assert_eq!(model::NAMES, ["j1", "j2", "j3", "j4"]);
}

#[test]
fn aliases_build_a_robot() {
    let _robot = Robot::new().add_controller(EachTick, zero);
}

/// `Spec` carries the model's facts, for the builder and the limits (SRS FR-4.5).
#[test]
fn spec_carries_the_model_facts() {
    use kmr_api::Spec;
    assert_eq!(<__Spec as Spec<N>>::LOWER, model::LOWER);
    assert_eq!(<__Spec as Spec<N>>::UPPER, model::UPPER);
    assert_eq!(<__Spec as Spec<N>>::SPEED, model::SPEED);
    assert_eq!(<__Spec as Spec<N>>::EFFORT, model::EFFORT);
    assert_eq!(<__Spec as Spec<N>>::CONTINUOUS, [1 << 1]);
    assert_eq!(<__Spec as Spec<N>>::PRISMATIC, [1 << 2]);
    assert_eq!(<__Spec as Spec<N>>::LINK_NAMES, model::LINK_NAMES);
}

/// The model's value types are kmr_api's: a controller names no `kmr_core` path.
#[test]
fn model_constants_are_kmr_api_types() {
    let _: [Q; N] = model::LOWER;
    let _: [Qd; N] = model::SPEED;
    let _: [Tau; N] = model::EFFORT;
}

/// Six actuators between two fixed joints. A direct call: the front door's `N == JOINTS`
/// assert refused N = 6 before kmr_api B7 (kmr_model T048 moves it to `robot!`).
mod six {
    kmr_api::__model!(kmr_api; model = "tests/fixtures/six_plus_two.urdf");

    #[test]
    fn six_actuators_in_file_order() {
        assert_eq!(N, 6);
        assert_eq!(model::NAMES, ["a1", "a2", "a3", "a4", "a5", "a6"]);
    }
}

/// One continuous actuator, by a direct call as `six`.
mod one {
    kmr_api::__model!(kmr_api; model = "tests/fixtures/continuous.urdf");

    #[test]
    fn one_actuator() {
        assert_eq!(N, 1);
        assert_eq!(model::NAMES, ["spin"]);
    }
}

mod limits {
    use kmr_api::{Q, Qd, Tau};

    use super::model;

    #[test]
    fn limits_of_a_revolute_actuator_in_rad() {
        assert_eq!(model::LOWER[0], Q(-1.57));
        assert_eq!(model::UPPER[0], Q(1.57));
        assert_eq!(model::SPEED[0], Qd(3.0));
        assert_eq!(model::EFFORT[0], Tau(1.5));
    }

    #[test]
    fn limits_of_a_continuous_actuator_have_the_position_placeholder() {
        assert_eq!((model::LOWER[1], model::UPPER[1]), (Q(0.0), Q(0.0)));
        assert_eq!(model::SPEED[1], Qd(3.0));
        assert_eq!(model::EFFORT[1], Tau(1.5));
    }

    #[test]
    fn limits_of_a_prismatic_actuator_in_mm() {
        assert_eq!(model::LOWER[2], Q(0.0));
        assert_eq!(model::UPPER[2], Q(100.0));
        assert_eq!(model::SPEED[2], Qd(50.0));
        assert_eq!(model::EFFORT[2], Tau(20.0));
    }

    #[test]
    fn kinds_are_bit_sets_readable_without_a_fake_infinite_limit() {
        assert_ne!(model::CONTINUOUS[0] & (1 << 1), 0);
        assert_ne!(model::PRISMATIC[0] & (1 << 2), 0);
        assert_eq!(model::CONTINUOUS[0] & !(1 << 1), 0);
        assert_eq!(model::PRISMATIC[0] & !(1 << 2), 0);
        let _: [u64; kmr_api::MASK_WORDS] = model::CONTINUOUS;
    }

    // 3.14 is the descriptor's value, not an approximate PI.
    #[allow(clippy::approx_constant)]
    #[test]
    fn limits_of_the_last_actuator() {
        assert_eq!(model::LOWER[3], Q(-3.14));
        assert_eq!(model::UPPER[3], Q(3.14));
        assert_eq!(model::SPEED[3], Qd(4.0));
        assert_eq!(model::EFFORT[3], Tau(2.0));
    }
}

mod names {
    /// The index of `j3`.
    const J3: usize = actuator!("j3");

    /// A const group by name.
    const ENDS: [usize; 2] = [actuator!("j1"), actuator!("j4")];

    #[test]
    fn names_give_indices() {
        assert_eq!(J3, 2);
        assert_eq!(ENDS, [0, 3]);
    }

    #[test]
    fn names_index_the_model() {
        assert_eq!(super::model::NAMES[actuator!("j4")], "j4");
    }

    mod child {
        #[test]
        fn names_give_indices_through_the_crate_path() {
            assert_eq!(crate::actuator!("j2"), 1);
        }
    }
}

mod links {
    use super::model;

    /// The index of link `name` in `LINK_NAMES`.
    fn link(name: &str) -> Option<usize> {
        model::LINK_NAMES.iter().position(|&l| l == name)
    }

    #[test]
    fn links_carry_their_mass_and_mass_center() {
        let l1 = link("l1").expect("l1 is a link of four.urdf");
        assert_eq!(model::LINK_MASS[l1], 0.35);
        assert_eq!(model::LINK_COM[l1], [0.0, 0.0, 0.05]);
    }

    #[test]
    fn links_name_their_parent_and_the_root_has_none() {
        let base = link("base").expect("base is a link of four.urdf");
        let l1 = link("l1").expect("l1 is a link of four.urdf");
        assert_eq!(model::LINK_PARENT[base], None);
        assert_eq!(model::LINK_PARENT[l1].map(usize::from), Some(base));
    }

    #[test]
    fn links_are_looked_up_by_name() {
        assert!(link("tool").is_some());
        assert!(link("gripper").is_none());
    }
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
