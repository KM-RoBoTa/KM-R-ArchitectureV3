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
        let _: Option<HCons<kmr_api::NoHardware, HNil>> = None::<<__Spec as Spec<N>>::Drivers>;
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
        let _: Option<HCons<Dynamixel<DxlRs<N>, 1, N>, HNil>> =
            None::<<__Spec as Spec<N>>::Drivers>;
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
        let _: Option<HCons<kmr_api::NoHardware, HNil>> = None::<<__Spec as Spec<N>>::Drivers>;
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
        model = "tests/fixtures/four_dxl.urdf",
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
        let _: Option<HCons<kmr_api::NoHardware, HNil>> = None::<<__Spec as Spec<N>>::Drivers>;
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
        let _: Option<HCons<Dynamixel<DxlRs<N>, 2, N>, HNil>> =
            None::<<__Spec as Spec<N>>::Drivers>;
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
        model = "tests/fixtures/four_dxl.urdf",
        hardware = { dynamixel: SHAPE.at(PART) }
    );

    #[test]
    fn a_part_naming_user_items_like_the_internals_reads_the_users() {
        assert_eq!(__hardware::dynamixel::__PART.indices(), &PART);
    }
}

mod mixed_brands {
    use super::*;
    use kmr_api::{cubemars, dynamixel};

    /// Two XM430 on bus `arm`.
    const ARM: dynamixel::Table<1, 2> = dynamixel::Table {
        buses: [dynamixel::Bus {
            label: "arm",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: [xm430(1), xm430(2)],
    };

    const fn xm430(id: u8) -> dynamixel::Actuator {
        dynamixel::Actuator {
            bus: 0,
            id,
            model: dynamixel::Model::Xm430W210,
            mode: dynamixel::Mode::Position,
            torque: dynamixel::Torque::On,
            zero: dynamixel::Zero::DEFAULT,
        }
    }

    /// Two AK60-6 on CAN interface `leg`, the second with its own gains.
    const LEG: cubemars::Table<1, 2> = cubemars::Table {
        buses: [cubemars::Bus { label: "leg" }],
        actuators: [
            cubemars::Actuator::ak60_6(0, 1),
            cubemars::Actuator::ak60_6(0, 2),
        ],
    }
    .gains(1, 40.0, 0.5);

    kmr_api::robot!(
        model = "tests/fixtures/four_dxl.urdf",
        hardware = { dynamixel: ARM.at([0, 2]), cubemars: LEG.at([1, 3]) }
    );

    /// One plugin per brand: a serial adapter and a CAN interface no machine has.
    const PLUGINS: [Binding; 2] = [
        Binding {
            bus: Some("arm"),
            ..PLUGIN
        },
        Binding {
            port: Port::ByDev("kmr-test-can"),
            baud: None,
            bus: Some("leg"),
        },
    ];

    #[cfg(not(any(feature = "dynamixel", feature = "cubemars")))]
    #[test]
    fn without_the_features_the_first_brand_is_refused() {
        assert!(matches!(
            __Spec::bind(&PLUGINS),
            Err(UserError::FeatureOff {
                feature: "dynamixel"
            })
        ));
    }

    #[cfg(all(feature = "dynamixel", not(feature = "cubemars")))]
    #[test]
    fn without_the_cubemars_feature_its_plugins_are_refused() {
        assert!(matches!(
            __Spec::bind(&PLUGINS),
            Err(UserError::FeatureOff {
                feature: "cubemars"
            })
        ));
    }

    #[cfg(feature = "cubemars")]
    #[test]
    fn with_the_feature_the_cubemars_member_claims_its_placement() {
        use kmr_api::cubemars::{CubeMars, SocketCan};
        use kmr_core::hal::{Group, Hal, Overrides};
        let leg = LEG.at([1, 3]);
        let Ok(mut member) =
            CubeMars::<SocketCan<1>, 1, N>::bind(SocketCan::new(), leg, &PLUGINS[1..])
        else {
            panic!("the plugin names the leg's interface");
        };
        let Ok(claim) = member.check(Overrides::default()) else {
            panic!("the table is valid");
        };
        assert!(claim.owned.iter().eq([1, 3]));
        assert_eq!((claim.group[1], claim.group[3]), (Group(0), Group(0)));
    }

    #[cfg(all(feature = "dynamixel", feature = "cubemars"))]
    #[test]
    fn with_both_features_each_brand_binds_its_plugin() {
        assert!(__Spec::bind(&PLUGINS).is_ok());
    }
}

mod health_limits {
    use kmr_api::dynamixel::{Actuator, Bus, HealthLimits, Mode, Model, Table, Torque, Zero};

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

    /// Four XM430 on one bus.
    const ARM: Table<1, 4> = Table {
        buses: [Bus {
            label: "arm",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: [xm430(1), xm430(2), xm430(3), xm430(4)],
    };

    /// Limits for every actuator.
    const LIMITS: HealthLimits = HealthLimits {
        min_voltage_dv: 100,
        max_voltage_dv: 140,
        temperature_c: 70,
    };

    /// A tighter temperature for the last row.
    const COOL: HealthLimits = HealthLimits {
        temperature_c: 60,
        ..LIMITS
    };

    kmr_api::robot!(
        model = "tests/fixtures/four_dxl.urdf",
        hardware = { dynamixel: ARM.in_order().health_limits(LIMITS).health_limits_of(3, COOL) }
    );

    #[test]
    fn the_part_carries_its_health_limits_through_robot() {
        assert_eq!(
            __hardware::dynamixel::__PART.limits(),
            &[Some(LIMITS), Some(LIMITS), Some(LIMITS), Some(COOL)]
        );
    }
}
