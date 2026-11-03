//! The checked robot, as `const` items at the call site (data-model.md
//! § Emitted facts).
//!
//! The output names no `std` item, no heap type, no `dyn` and no lock
//! (FR-019 (SRS NFR-1)). Literals go through [`Literal`], never through
//! formatted text (research R9).

use proc_macro2::{Literal, TokenStream};
use quote::quote;

use crate::model::{Kind, Robot};

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
    let prismatic = kinds(robot, Kind::Prismatic, krate);
    let continuous = kinds(robot, Kind::Continuous, krate);
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

            /// Prismatic actuators: bit `i` (word `i / 64`, bit `i % 64`, the same
            /// layout as `kmr_core::hal::Mask`) is set for actuator `i`.
            pub const PRISMATIC: [u64; #krate::MASK_WORDS] = #prismatic;

            /// Continuous actuators, laid out as `PRISMATIC`: no position limit,
            /// so `LOWER` and `UPPER` hold no bound for them. Revolute is
            /// neither.
            pub const CONTINUOUS: [u64; #krate::MASK_WORDS] = #continuous;

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
        /// The index of the actuator named by a plain string literal; any
        /// other literal does not build, a raw or escaped one included.
        #[allow(unused_macros)]
        macro_rules! actuator {
            #(#arms)*
            ($name:literal) => {
                ::core::compile_error!(::core::concat!(
                    "kmr: no actuator named ",
                    $name,
                    "; a name matches only as a plain string literal, without raw form or escape"
                ))
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

/// The bit set of the actuators of `kind`, as a `[u64; MASK_WORDS]`
/// expression: only the words that hold a set bit are written.
fn kinds(robot: &Robot, kind: Kind, krate: &TokenStream) -> TokenStream {
    let mut words: Vec<u64> = Vec::new();
    for (i, _) in robot
        .actuators
        .iter()
        .enumerate()
        .filter(|(_, a)| a.kind == kind)
    {
        let word = i / 64;
        if words.len() <= word {
            words.resize(word + 1, 0);
        }
        if let Some(w) = words.get_mut(word) {
            *w |= 1 << (i % 64);
        }
    }
    let set = words
        .iter()
        .enumerate()
        .filter(|&(_, &bits)| bits != 0)
        .map(|(w, &bits)| {
            let w = Literal::usize_unsuffixed(w);
            let bits = Literal::u64_suffixed(bits);
            quote! { m[#w] = #bits; }
        })
        .collect::<Vec<_>>();
    if set.is_empty() {
        quote! { [0u64; #krate::MASK_WORDS] }
    } else {
        quote! { { let mut m = [0u64; #krate::MASK_WORDS]; #(#set)* m } }
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
    use std::iter::Peekable;

    use proc_macro2::{Delimiter, TokenTree, token_stream};

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
            out.contains("concat ! (\"kmr: no actuator named \" , $ name , \"; a name matches only as a plain string literal, without raw form or escape\")"),
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

    type Tokens = Peekable<token_stream::IntoIter>;

    /// The `(keyword, name)` of each item in `ts`, and the body of its
    /// `mod`. The walk ends at the first token that does not continue an
    /// item kind the emission uses, pushed as `("unexpected", rest)`.
    fn items(ts: TokenStream) -> (Vec<(String, String)>, Option<TokenStream>) {
        let mut out = Vec::new();
        let mut body = None;
        let mut tokens = ts.into_iter().peekable();
        while tokens.peek().is_some() {
            let Some(pair) = item(&mut tokens, &mut body) else {
                let rest: Vec<String> = tokens.map(|t| t.to_string()).collect();
                out.push(("unexpected".to_owned(), rest.join(" ")));
                break;
            };
            out.push(pair);
        }
        (out, body)
    }

    /// One item: attributes, an optional `pub` or `pub(..)`, then a
    /// `const`, `mod`, `macro_rules!` or `use`; `None` for anything else.
    fn item(tokens: &mut Tokens, body: &mut Option<TokenStream>) -> Option<(String, String)> {
        let mut word = tokens.next()?;
        loop {
            match &word {
                TokenTree::Punct(p) if p.as_char() == '#' => {
                    group(tokens, Delimiter::Bracket)?;
                }
                TokenTree::Ident(i) if i == "pub" => {
                    if matches!(tokens.peek(), Some(TokenTree::Group(g)) if g.delimiter() == Delimiter::Parenthesis)
                    {
                        tokens.next();
                    }
                }
                _ => break,
            }
            word = tokens.next()?;
        }
        let TokenTree::Ident(word) = word else {
            return None;
        };
        let keyword = word.to_string();
        let name = match keyword.as_str() {
            "const" => {
                let name = ident(tokens)?;
                rest_of_statement(tokens)?;
                name
            }
            "mod" => {
                let name = ident(tokens)?;
                *body = Some(group(tokens, Delimiter::Brace)?);
                name
            }
            "macro_rules" => {
                match tokens.next()? {
                    TokenTree::Punct(p) if p.as_char() == '!' => {}
                    _ => return None,
                }
                let name = ident(tokens)?;
                group(tokens, Delimiter::Brace)?;
                name
            }
            "use" => rest_of_statement(tokens)?.pop()?,
            _ => return None,
        };
        Some((keyword, name))
    }

    /// The next token, when it is an identifier.
    fn ident(tokens: &mut Tokens) -> Option<String> {
        match tokens.next()? {
            TokenTree::Ident(i) => Some(i.to_string()),
            _ => None,
        }
    }

    /// The body of the next token, when it is a group in `delimiter`.
    fn group(tokens: &mut Tokens, delimiter: Delimiter) -> Option<TokenStream> {
        match tokens.next()? {
            TokenTree::Group(g) if g.delimiter() == delimiter => Some(g.stream()),
            _ => None,
        }
    }

    /// The tokens up to the next top-level `;`, which is consumed; `None`
    /// when there is no `;`.
    fn rest_of_statement(tokens: &mut Tokens) -> Option<Vec<String>> {
        let mut seen = Vec::new();
        loop {
            match tokens.next()? {
                TokenTree::Punct(p) if p.as_char() == ';' => return Some(seen),
                t => seen.push(t.to_string()),
            }
        }
    }

    #[test]
    fn the_item_walk_refuses_other_item_kinds() {
        let (top, _) = items(quote! { const A: u8 = 0; type T = u8; });
        assert_eq!(top[0], ("const".to_owned(), "A".to_owned()));
        assert_eq!(top[1].0, "unexpected");
        for other in [
            quote! { pub type T = u8; },
            quote! { static S: u8 = 0; },
            quote! { const fn f() {} },
            quote! { extern crate core; },
            quote! { macro_rules! m ( () => {} ); },
            quote! { mod m; },
            quote! { #[doc = ""] },
        ] {
            let (top, _) = items(other.clone());
            assert_eq!(top.len(), 1, "{other}");
            assert_eq!(top[0].0, "unexpected", "{other}");
        }
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
    fn continuous_and_prismatic_are_bit_sets_revolute_is_neither() {
        let arm = four();
        let names: Vec<_> = arm.actuators.iter().map(|a| a.kind).collect();
        assert_eq!(
            names,
            [
                Kind::Revolute,
                Kind::Continuous,
                Kind::Prismatic,
                Kind::Revolute
            ]
        );
        let continuous = kinds(&arm, Kind::Continuous, &quote!(k)).to_string();
        assert_eq!(
            continuous,
            "{ let mut m = [0u64 ; k :: MASK_WORDS] ; m [0] = 2u64 ; m }"
        );
        let prismatic = kinds(&arm, Kind::Prismatic, &quote!(k)).to_string();
        assert_eq!(
            prismatic,
            "{ let mut m = [0u64 ; k :: MASK_WORDS] ; m [0] = 4u64 ; m }"
        );
        let none = kinds(&arm, Kind::Revolute, &quote!(k)).to_string();
        assert!(none.contains("m [0] = 9u64"), "{none}");
        let empty = kinds(&robot(&["a"]), Kind::Prismatic, &quote!(k));
        assert_eq!(empty.to_string(), "[0u64 ; k :: MASK_WORDS]");
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
