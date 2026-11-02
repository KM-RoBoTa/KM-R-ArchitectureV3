//! The checked robot, as `const` items at the call site (data-model.md
//! § Emitted facts).
//!
//! The output names no `std` item, no heap type, no `dyn` and no lock
//! (FR-019 (SRS NFR-1)). Literals go through [`Literal`], never through
//! formatted text (research R9).

use proc_macro2::{Literal, TokenStream};
use quote::quote;

use crate::model::Robot;

/// `N`, the `model` module and, for a file, the rebuild tracking of `tracked`.
///
/// `krate` is the path the output names kmr_api's types through.
pub(crate) fn emit(robot: &Robot, krate: &TokenStream, tracked: Option<&str>) -> TokenStream {
    let n = Literal::usize_unsuffixed(robot.actuators.len());
    let track = tracked.map(|path| {
        let path = Literal::string(path);
        quote! { const _: &[u8] = ::core::include_bytes!(#path); }
    });
    let names = robot.actuators.iter().map(|a| Literal::string(&a.name));
    let [lower, upper, speed, effort] = columns(robot).map(|c| c.into_iter().map(float));
    let lookup = lookup(robot);
    let links = links(robot);
    quote! {
        /// Actuator count of this robot, from its descriptor.
        #[allow(dead_code)]
        pub const N: usize = #n;

        #track

        #lookup

        /// This robot's model, read from its descriptor at compile time.
        #[allow(dead_code)]
        pub mod model {
            /// Joint names, in actuator index order.
            pub const NAMES: [&str; super::N] = [#(#names),*];

            /// Lowest position per actuator: rad (revolute) or mm (prismatic).
            /// `Q(0.0)` where `CONTINUOUS` is set: that joint has no position
            /// limit.
            pub const LOWER: [#krate::Q; super::N] = [#(#krate::Q(#lower)),*];

            /// Highest position per actuator: rad (revolute) or mm (prismatic).
            /// `Q(0.0)` where `CONTINUOUS` is set: that joint has no position
            /// limit.
            pub const UPPER: [#krate::Q; super::N] = [#(#krate::Q(#upper)),*];

            /// Top speed per actuator: rad/s (revolute, continuous) or mm/s
            /// (prismatic).
            pub const SPEED: [#krate::Qd; super::N] = [#(#krate::Qd(#speed)),*];

            /// Top effort per actuator: N·m (revolute, continuous) or N
            /// (prismatic).
            pub const EFFORT: [#krate::Tau; super::N] = [#(#krate::Tau(#effort)),*];

            #links
        }
    }
}

/// `actuator!("name")`, the index of that actuator as a `usize` literal
/// (research R6; FR-015 (SRS FR-2.4)).
///
/// Not `#[macro_export]`: two calls in one crate would export two root
/// macros. `pub(crate) use` lets a child module write `crate::actuator!`.
fn lookup(robot: &Robot) -> TokenStream {
    let arms = robot.actuators.iter().enumerate().map(|(i, a)| {
        let name = Literal::string(&a.name);
        let index = Literal::usize_suffixed(i);
        quote! { (#name) => { #index }; }
    });
    quote! {
        /// The index of the actuator named by a string literal; any other
        /// name does not build.
        #[allow(unused_macros)]
        macro_rules! actuator {
            #(#arms)*
            ($name:literal) => {
                ::core::compile_error!(::core::concat!("kmr: no actuator named ", $name))
            };
        }

        /// [`actuator!`] for the whole crate.
        #[allow(unused_imports)]
        pub(crate) use actuator;
    }
}

/// `LINK_NAMES`, `LINK_PARENT`, `LINK_MASS`, `LINK_COM`, in file order
/// (FR-016 (SRS FR-8.1)).
fn links(robot: &Robot) -> TokenStream {
    let l = Literal::usize_unsuffixed(robot.links.len());
    let names = robot.links.iter().map(|l| Literal::string(&l.name));
    let parents = robot.links.iter().map(|l| match l.parent {
        Some(p) => {
            let p = Literal::u16_suffixed(p);
            quote!(::core::option::Option::Some(#p))
        }
        None => quote!(::core::option::Option::None),
    });
    let masses = robot.links.iter().map(|l| float(l.mass));
    let coms = robot.links.iter().map(|l| {
        let [x, y, z] = l.com.map(float);
        quote!([#x, #y, #z])
    });
    quote! {
        /// Link names, in descriptor order.
        pub const LINK_NAMES: [&str; #l] = [#(#names),*];

        /// Parent of each link, as an index into `LINK_NAMES`; `None` for
        /// the root.
        pub const LINK_PARENT: [::core::option::Option<u16>; #l] = [#(#parents),*];

        /// Mass of each link, kg.
        pub const LINK_MASS: [f32; #l] = [#(#masses),*];

        /// Mass center of each link, m, in the link frame.
        pub const LINK_COM: [[f32; 3]; #l] = [#(#coms),*];
    }
}

/// Lowest position, highest position, top speed and top effort of every
/// actuator, in index order and SRS FR-2.2 units.
fn columns(robot: &Robot) -> [Vec<f32>; 4] {
    let column =
        |f: fn(&crate::model::CheckedActuator) -> f32| robot.actuators.iter().map(f).collect();
    [
        column(|a| a.lower),
        column(|a| a.upper),
        column(|a| a.speed),
        column(|a| a.effort),
    ]
}

/// `v` as an `f32` literal, its sign a separate token.
fn float(v: f32) -> TokenStream {
    let lit = Literal::f32_suffixed(v.abs());
    if v < 0.0 { quote!(-#lit) } else { quote!(#lit) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{CheckedActuator, Kind};
    use crate::urdf::tests::four;

    fn robot(names: &[&str]) -> Robot {
        let actuators = names
            .iter()
            .map(|name| CheckedActuator {
                name: (*name).to_owned(),
                kind: Kind::Revolute,
                lower: -1.0,
                upper: 1.0,
                speed: 1.0,
                effort: 1.0,
            })
            .collect();
        Robot {
            actuators,
            links: Vec::new(),
        }
    }

    /// Lowest, highest, speed and effort of actuator `i`.
    fn limits(robot: &Robot, i: usize) -> [f32; 4] {
        columns(robot).map(|c| c[i])
    }

    #[test]
    fn n_and_names_in_index_order() {
        let out = emit(&robot(&["a", "b"]), &quote!(k), None).to_string();
        assert!(out.contains("pub const N : usize = 2 ;"), "{out}");
        assert!(out.contains("[\"a\" , \"b\"]"), "{out}");
    }

    #[test]
    fn a_file_is_tracked_a_preset_is_not() {
        let file = emit(&robot(&["a"]), &quote!(k), Some("/x/r.urdf")).to_string();
        assert!(file.contains("include_bytes ! (\"/x/r.urdf\")"), "{file}");
        let preset = emit(&robot(&["a"]), &quote!(k), None).to_string();
        assert!(!preset.contains("include_bytes"), "{preset}");
    }

    #[test]
    fn one_lookup_arm_per_name_in_index_order() {
        let out = lookup(&robot(&["a", "b"])).to_string();
        assert!(
            out.contains("(\"a\") => { 0usize } ; (\"b\") => { 1usize } ;"),
            "{out}"
        );
        assert!(
            out.contains("concat ! (\"kmr: no actuator named \" , $ name)"),
            "{out}"
        );
        assert!(out.contains("pub (crate) use actuator ;"), "{out}");
        assert!(!out.contains("macro_export"), "{out}");
    }

    #[test]
    fn links_in_descriptor_order() {
        let out = links(&four()).to_string();
        assert!(
            out.contains("LINK_NAMES : [& str ; 6] = [\"base\" , \"l1\" , \"l2\""),
            "{out}"
        );
        assert!(
            out.contains("= [:: core :: option :: Option :: None , :: core :: option :: Option :: Some (0u16) ,"),
            "{out}"
        );
        assert!(
            out.contains("LINK_MASS : [f32 ; 6] = [0.5f32 , 0.35f32"),
            "{out}"
        );
        assert!(out.contains("[0f32 , 0f32 , 0.05f32]"), "{out}");
    }

    #[test]
    fn revolute_limits_are_rad_and_newton_metres() {
        assert_eq!(limits(&four(), 0), [-1.57, 1.57, 3.0, 1.5]);
    }

    #[test]
    fn prismatic_limits_are_mm_and_newtons() {
        assert_eq!(limits(&four(), 2), [0.0, 100.0, 50.0, 20.0]);
    }

    #[test]
    fn continuous_positions_are_the_zero_placeholder() {
        assert_eq!(limits(&four(), 1)[..2], [0.0, 0.0]);
    }

    #[test]
    fn limits_are_typed_through_the_krate_path() {
        let out = emit(&four(), &quote!(k), None).to_string();
        assert!(
            out.contains(
                "pub const LOWER : [k :: Q ; super :: N] = [k :: Q (- 1.57f32) , k :: Q (0f32)"
            ),
            "{out}"
        );
        assert!(
            out.contains("pub const SPEED : [k :: Qd ; super :: N]"),
            "{out}"
        );
        assert!(out.contains("k :: Tau (20f32)"), "{out}");
    }
}
