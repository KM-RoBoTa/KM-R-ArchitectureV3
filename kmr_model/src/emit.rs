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
        // A limit of 3.14 is the descriptor's value, not an approximate PI.
        #[allow(dead_code, clippy::approx_constant)]
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
    use proc_macro2::TokenTree;

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
    fn a_limit_near_pi_is_no_clippy_error_in_the_caller() {
        let out = emit(&four(), &quote!(k), None).to_string();
        assert!(
            out.contains("# [allow (dead_code , clippy :: approx_constant)] pub mod model"),
            "{out}"
        );
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

    /// The `(keyword, name)` of each item in `ts`, and the body of
    /// `mod model`.
    fn items(ts: TokenStream) -> (Vec<(String, String)>, Option<TokenStream>) {
        let mut out = Vec::new();
        let mut body = None;
        let mut tokens = ts.into_iter().peekable();
        while let Some(t) = tokens.next() {
            let TokenTree::Ident(word) = t else { continue };
            let word = word.to_string();
            let name = match word.as_str() {
                "const" | "mod" => tokens.next().map(|t| t.to_string()),
                "macro_rules" => tokens.nth(1).map(|t| t.to_string()),
                "use" => tokens
                    .by_ref()
                    .take_while(|t| t.to_string() != ";")
                    .last()
                    .map(|t| t.to_string()),
                _ => continue,
            };
            let name = name.unwrap_or_default();
            if word == "mod"
                && let Some(TokenTree::Group(g)) = tokens.peek()
            {
                body = Some(g.stream());
            }
            if word == "const" {
                tokens.by_ref().take_while(|t| t.to_string() != ";").count();
            }
            if word == "macro_rules" {
                tokens.next();
            }
            out.push((word, name));
        }
        (out, body)
    }

    /// Every identifier in `ts`, groups included.
    fn idents(ts: TokenStream, out: &mut Vec<String>) {
        for t in ts {
            match t {
                TokenTree::Ident(i) => out.push(i.to_string()),
                TokenTree::Group(g) => idents(g.stream(), out),
                _ => {}
            }
        }
    }

    #[test]
    fn only_n_model_and_actuator_are_emitted() {
        let out = emit(&four(), &quote!(k), Some("/x/four.urdf"));
        let (top, body) = items(out.clone());
        let pairs = |v: &[(&str, &str)]| -> Vec<(String, String)> {
            v.iter()
                .map(|(a, b)| ((*a).to_owned(), (*b).to_owned()))
                .collect()
        };
        assert_eq!(
            top,
            pairs(&[
                ("const", "N"),
                ("const", "_"),
                ("macro_rules", "actuator"),
                ("use", "actuator"),
                ("mod", "model"),
            ])
        );
        let allowed = [
            "NAMES",
            "LOWER",
            "UPPER",
            "SPEED",
            "EFFORT",
            "PRISMATIC",
            "CONTINUOUS",
            "LINK_NAMES",
            "LINK_PARENT",
            "LINK_MASS",
            "LINK_COM",
        ];
        let (inner, _) = items(body.unwrap_or_default());
        assert!(!inner.is_empty());
        for (word, name) in &inner {
            assert_eq!(word, "const", "{name}");
            assert!(allowed.contains(&name.as_str()), "{name}");
        }
        let mut all = Vec::new();
        idents(out, &mut all);
        let banned = [
            "fn", "impl", "struct", "enum", "trait", "static", "dyn", "unsafe", "std", "alloc",
            "Box", "Vec", "String", "Mutex",
        ];
        for word in banned {
            assert!(!all.iter().any(|i| i == word), "{word}");
        }
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
