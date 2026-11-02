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
        let _: Option<HCons<kmr_core::hal::NoHardware, HNil>> = None::<<__Spec as Spec>::Drivers>;
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
        use kmr_adapters::dynamixel::{DxlRs, Dynamixel};
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
        let _: Option<HCons<kmr_core::hal::NoHardware, HNil>> = None::<<__Spec as Spec>::Drivers>;
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

    /// Four XM430 on bus `arm`, wired in reverse engine order.
    const ARM: Table<1, 4> = Table {
        buses: [Bus {
            label: "arm",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: [xm430(1), xm430(2), xm430(3), xm430(4)],
    };

    kmr_api::robot!(
        model = "tests/fixtures/four.urdf",
        hardware = { dynamixel: ARM.at([3, 2, 1, 0]) }
    );

    const LABELLED: Binding = Binding {
        bus: Some("arm"),
        ..PLUGIN
    };

    #[test]
    fn the_part_keeps_its_placement() {
        assert_eq!(__hardware::dynamixel::PART.indices(), &[3, 2, 1, 0]);
        assert_eq!(N, 4);
    }

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn without_the_feature_the_member_is_no_hardware_and_plugins_are_refused() {
        let _: Option<HCons<kmr_core::hal::NoHardware, HNil>> = None::<<__Spec as Spec>::Drivers>;
        assert!(matches!(
            __Spec::bind(&[LABELLED]),
            Err(UserError::FeatureOff {
                feature: "dynamixel"
            })
        ));
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_the_member_is_dynamixel_and_a_plugin_binds() {
        use kmr_adapters::dynamixel::{DxlRs, Dynamixel};
        let _: Option<HCons<Dynamixel<DxlRs, 1>, HNil>> = None::<<__Spec as Spec>::Drivers>;
        assert!(__Spec::bind(&[LABELLED]).is_ok());
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_a_member_claims_its_placement() {
        use kmr_core::hal::{Group, Hal, Overrides};
        let Ok(HCons { head: mut dxl, .. }) = __Spec::bind(&[LABELLED]) else {
            panic!("the plugin names the one bus");
        };
        let claim = dxl.check(Overrides::default());
        assert!(claim.as_ref().is_ok_and(
            |claim| claim.owned == kmr_core::hal::Mask::ALL && claim.group == [Group(0); 4]
        ));
    }
}
