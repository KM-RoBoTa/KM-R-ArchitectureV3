//! `robot!` over two brands: the `@parts` and `@list` recursion, on the test-only fake brands.

use kmr_adapters::binding::{Binding, Port};
use kmr_core::hal::{Hal, Mask, Overrides};

use crate::spec::fake::{Fake, Hardware};
use crate::{HCons, HNil, Spec, UserError};

crate::robot!(
    model = "tests/fixtures/four.urdf",
    hardware = {
        fake_arm: Hardware { labels: ["arm"], indices: [0, 2] },
        fake_leg: Hardware { labels: ["leg", "foot"], indices: [3, 1] },
    }
);

/// A plugin by serial id on bus `label`.
const fn plugin(port: &'static str, label: &'static str) -> Binding {
    Binding {
        port: Port::ById(port),
        baud: None,
        bus: Some(label),
    }
}

/// The actuators of `indices`.
fn mask(indices: &[usize]) -> Mask {
    let mut mask = Mask::NONE;
    indices.iter().for_each(|&index| mask.set(index));
    mask
}

#[test]
fn two_brands_give_two_members_in_declaration_order() {
    let _: Option<HCons<Fake, HCons<Fake, HNil>>> = None::<<__Spec as Spec>::Drivers>;
    assert_eq!(__hardware::fake_arm::__PART.indices, [0, 2]);
    assert_eq!(__hardware::fake_leg::__PART.indices, [3, 1]);
}

#[test]
fn each_brand_binds_the_plugins_of_its_buses_and_claims_its_part() {
    let plugins = [
        plugin("FOOT0001", "foot"),
        plugin("ARM00001", "arm"),
        plugin("LEG00001", "leg"),
    ];
    let Ok(HCons {
        head: mut arm,
        tail: HCons {
            head: mut leg,
            tail: HNil,
        },
    }) = __Spec::bind(&plugins)
    else {
        panic!("every plugin names a bus of one brand");
    };
    assert_eq!(arm.ports, ["ARM00001"]);
    assert_eq!(leg.ports, ["FOOT0001", "LEG00001"]);
    let owned = |member: &mut Fake| member.check(Overrides::default()).map(|claim| claim.owned);
    assert_eq!(owned(&mut arm), Ok(mask(&[0, 2])));
    assert_eq!(owned(&mut leg), Ok(mask(&[1, 3])));
}

#[test]
fn a_plugin_neither_brand_takes_is_refused() {
    let res = __Spec::bind(&[plugin("HAND0001", "hand")]);
    assert!(
        matches!(
            res,
            Err(UserError::UnknownBus {
                label: "hand",
                port: "HAND0001"
            })
        ),
        "{:?}",
        res.err()
    );
}
