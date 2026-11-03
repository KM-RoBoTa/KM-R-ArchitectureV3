//! The `Spec` that `robot!` emits: the driver list and `bind`, with the feature on and off.

use kmr_adapters::binding::{Binding, Port};
use kmr_api::{HCons, HNil, Spec, UserError};

const PLUGIN: Binding = Binding {
    port: Port::ById("FT8ISZ6H"),
    baud: None,
    bus: None,
};

mod preset {
    use super::*;

    kmr_api::robot!(XmBench);

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn without_the_feature_the_driver_is_no_hardware() {
        let _: Option<HCons<kmr_api::NoHardware, HNil>> = None::<<__Spec as Spec>::Drivers>;
    }

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn without_the_feature_plugins_are_refused() {
        let res = __Spec::bind(&[PLUGIN]);
        assert!(matches!(
            res,
            Err(UserError::FeatureOff {
                feature: "dynamixel"
            })
        ));
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_the_driver_is_dynamixel() {
        use kmr_api::dynamixel::{DxlRs, Dynamixel};
        let _: Option<HCons<Dynamixel<DxlRs, 1>, HNil>> = None::<<__Spec as Spec>::Drivers>;
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_a_plugin_binds_without_io() {
        assert!(__Spec::bind(&[PLUGIN]).is_ok());
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_a_wrong_label_is_a_bring_up_error() {
        let wrong = Binding {
            bus: Some("arm"),
            ..PLUGIN
        };
        let res = __Spec::bind(&[wrong]);
        assert!(
            matches!(&res, Err(UserError::BringUp { source }) if source.to_string().contains("arm") && source.to_string().contains("FT8ISZ6H")),
            "{:?}",
            res.err()
        );
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_no_plugin_is_a_bring_up_error() {
        let res = __Spec::bind(&[]);
        assert!(
            matches!(&res, Err(UserError::BringUp { source }) if source.to_string().contains("bus0")),
            "{:?}",
            res.err()
        );
    }
}

mod model {
    use super::*;

    kmr_api::robot!(model = "tests/fixtures/four.urdf");

    #[test]
    fn without_a_table_the_driver_is_no_hardware() {
        let _: Option<HCons<kmr_api::NoHardware, HNil>> = None::<<__Spec as Spec>::Drivers>;
    }

    #[test]
    fn without_a_table_plugins_are_refused() {
        assert!(matches!(
            __Spec::bind(&[PLUGIN]),
            Err(UserError::NoHardwareDeclared)
        ));
    }
}

mod placed {
    use super::*;
    use core::num::NonZeroU32;
    use kmr_api::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};

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

    /// One XM430 on bus `arm`, read every tick, then three on bus `wrist`, read every
    /// other tick, wired in reverse engine order: `in_order()` would put the arm at 0.
    const ARM: Table<2, 4> = Table {
        buses: [
            Bus {
                label: "arm",
                baud: 1_000_000,
                read_every: NonZeroU32::new(1),
            },
            Bus {
                label: "wrist",
                baud: 57_600,
                read_every: NonZeroU32::new(2),
            },
        ],
        actuators: [xm430(0, 1), xm430(1, 1), xm430(1, 2), xm430(1, 3)],
    };

    kmr_api::robot!(
        model = "tests/fixtures/four.urdf",
        hardware = { dynamixel: ARM.at([3, 2, 1, 0]) }
    );

    const LABELLED: [Binding; 2] = [
        Binding {
            bus: Some("arm"),
            ..PLUGIN
        },
        Binding {
            port: Port::ById("FT8WRIST"),
            baud: None,
            bus: Some("wrist"),
        },
    ];

    #[test]
    fn the_part_keeps_its_placement() {
        assert_eq!(__hardware::dynamixel::__PART.indices(), &[3, 2, 1, 0]);
        assert_eq!(N, 4);
    }

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn without_the_feature_the_member_is_no_hardware_and_plugins_are_refused() {
        let _: Option<HCons<kmr_api::NoHardware, HNil>> = None::<<__Spec as Spec>::Drivers>;
        assert!(matches!(
            __Spec::bind(&LABELLED),
            Err(UserError::FeatureOff {
                feature: "dynamixel"
            })
        ));
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_the_member_is_dynamixel_and_a_plugin_binds() {
        use kmr_api::dynamixel::{DxlRs, Dynamixel};
        let _: Option<HCons<Dynamixel<DxlRs, 2>, HNil>> = None::<<__Spec as Spec>::Drivers>;
        assert!(__Spec::bind(&LABELLED).is_ok());
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_a_member_claims_its_placement() {
        use kmr_core::hal::{Group, Hal, Mask, Overrides};
        let Ok(HCons { head: mut dxl, .. }) = __Spec::bind(&LABELLED) else {
            panic!("the plugins name both buses");
        };
        let Ok(claim) = dxl.check(Overrides::default()) else {
            panic!("the table is valid");
        };
        assert_eq!(claim.owned, Mask::ALL);
        // Groups are numbered in engine order: the wrist, first seen at 0, is group 0.
        assert_eq!(claim.group, [Group(0), Group(0), Group(0), Group(1)]);
        assert_eq!(claim.read_every.map(NonZeroU32::get), [2, 2, 2, 1]);
    }
}

mod user_names {
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

    /// Named like the expansion's internals once were.
    type Part = Table<1, 4>;

    /// The user's table, under a name the expansion must not shadow.
    const SHAPE: Part = Table {
        buses: [Bus {
            label: "arm",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: [xm430(1), xm430(2), xm430(3), xm430(4)],
    };

    /// The user's placement, under a name the expansion must not shadow.
    const PART: [usize; 4] = [1, 0, 3, 2];

    kmr_api::robot!(
        model = "tests/fixtures/four.urdf",
        hardware = { dynamixel: SHAPE.at(PART) }
    );

    #[test]
    fn a_part_naming_user_items_like_the_internals_reads_the_users() {
        assert_eq!(__hardware::dynamixel::__PART.indices(), &PART);
    }
}
