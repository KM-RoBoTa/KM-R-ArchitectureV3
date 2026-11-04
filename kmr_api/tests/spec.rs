//! The `Spec` that `robot!` emits: the driver type and `bind`, with the feature on and off.

use kmr_adapters::binding::{Binding, Port};
use kmr_api::{Spec, UserError};

const PLUGIN: Binding = Binding {
    port: Port::ById("FT8ISZ6H"),
    baud: None,
    bus: None,
};

mod preset {
    use super::*;

    kmr_api::robot!(XmBench);

    #[test]
    fn a_preset_names_itself() {
        assert_eq!(<__Spec as Spec>::PRESET, Some("XmBench"));
    }

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn without_the_feature_the_driver_is_no_hardware() {
        let _: Option<kmr_core::hal::NoHardware> = None::<<__Spec as Spec>::Adapter>;
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
        let _: Option<Dynamixel<DxlRs, 1>> = None::<<__Spec as Spec>::Adapter>;
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
    fn a_model_robot_names_no_preset() {
        assert_eq!(<__Spec as Spec>::PRESET, None);
    }

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn without_the_feature_the_driver_is_no_hardware() {
        let _: Option<kmr_core::hal::NoHardware> = None::<<__Spec as Spec>::Adapter>;
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn with_the_feature_the_driver_is_dynamixel_on_n_bus_slots() {
        use kmr_adapters::dynamixel::{DxlRs, Dynamixel};
        let _: Option<Dynamixel<DxlRs, N>> = None::<<__Spec as Spec>::Adapter>;
    }

    #[test]
    fn without_a_table_plugins_are_refused() {
        assert!(matches!(
            __Spec::bind(&[PLUGIN]),
            Err(UserError::NoHardwareDeclared)
        ));
    }
}
