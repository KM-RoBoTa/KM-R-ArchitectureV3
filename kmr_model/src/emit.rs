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
pub(crate) fn emit(robot: &Robot, _krate: &TokenStream, tracked: Option<&str>) -> TokenStream {
    let n = Literal::usize_unsuffixed(robot.actuators.len());
    let track = tracked.map(|path| {
        let path = Literal::string(path);
        quote! { const _: &[u8] = ::core::include_bytes!(#path); }
    });
    let names = robot.actuators.iter().map(|a| Literal::string(&a.name));
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
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{CheckedActuator, Kind};

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
}
