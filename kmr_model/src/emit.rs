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
    quote! {
        /// Actuator count of this robot, from its descriptor.
        #[allow(dead_code)]
        pub const N: usize = #n;

        #track

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
        }
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
