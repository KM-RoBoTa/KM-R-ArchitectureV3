//! The input of `model!`, as kmr_api's front door writes it.
//!
//! `<krate> ; preset = <Ident>` or `<krate> ; model = "<path>"`. The front
//! door passes `$crate` as `<krate>` and has split any `hardware = ..` off
//! (assumed, pending human: D7 A).

use proc_macro2::{Ident, TokenStream, TokenTree};
use syn::parse::{Parse, ParseStream};
use syn::{LitStr, Token};

/// Where the robot model comes from.
pub(crate) enum Source {
    /// A preset shipped in kmr_model, by name.
    Preset(Ident),
    /// A descriptor file, relative to the calling crate's `Cargo.toml`.
    Model(LitStr),
}

/// The parsed input of `model!`.
pub(crate) struct Input {
    /// The path emitted code names kmr_api's types through.
    pub(crate) krate: TokenStream,
    /// Where the model comes from.
    pub(crate) source: Source,
}

/// What the user may write after the crate path.
const FORMS: &str = "expected `PresetName` or `model = \"path/to/robot.urdf\"`";

impl Parse for Input {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut krate = TokenStream::new();
        while !input.peek(Token![;]) {
            if input.is_empty() {
                return Err(input.error("kmr: internal: model! input has no `;`"));
            }
            krate.extend([input.parse::<TokenTree>()?]);
        }
        input.parse::<Token![;]>()?;
        let key: Ident = input
            .parse()
            .map_err(|e| syn::Error::new(e.span(), format!("kmr: {FORMS}")))?;
        input.parse::<Token![=]>()?;
        let source = match key.to_string().as_str() {
            "preset" => Source::Preset(input.parse()?),
            "model" => Source::Model(input.parse()?),
            other => {
                return Err(syn::Error::new(
                    key.span(),
                    format!("kmr: unknown key `{other}`; {FORMS}"),
                ));
            }
        };
        if !input.is_empty() {
            return Err(input.error(
                "kmr: one robot model only: a preset's model half cannot be overridden, and a descriptor is the whole model",
            ));
        }
        Ok(Self { krate, source })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(text: &str) -> Result<Input, String> {
        syn::parse_str::<Input>(text).map_err(|e| e.to_string())
    }

    #[test]
    fn a_preset_by_name() {
        let input = parse("kmr_api; preset = XmBench");
        assert!(matches!(input, Ok(Input { source: Source::Preset(ref p), .. }) if p == "XmBench"));
    }

    #[test]
    fn a_descriptor_path_with_its_crate() {
        let Ok(input) = parse("::kmr_api ; model = \"robot/arm.urdf\"") else {
            unreachable!("valid input refused");
        };
        assert_eq!(input.krate.to_string(), ":: kmr_api");
        assert!(matches!(input.source, Source::Model(ref l) if l.value() == "robot/arm.urdf"));
    }

    #[test]
    fn a_raw_string_path_resolves() {
        let input = parse("k; model = r\"a\\b.urdf\"");
        assert!(
            matches!(input, Ok(Input { source: Source::Model(ref l), .. }) if l.value() == "a\\b.urdf")
        );
    }

    #[test]
    fn an_escaped_path_resolves() {
        let input = parse("k; model = \"a\\u{62}.urdf\"");
        assert!(
            matches!(input, Ok(Input { source: Source::Model(ref l), .. }) if l.value() == "ab.urdf")
        );
    }

    #[test]
    fn neither_key_is_refused() {
        assert!(parse("k;").is_err_and(|e| e.contains("expected `PresetName`")));
        assert!(parse("k; \"robot.urdf\"").is_err_and(|e| e.contains("expected `PresetName`")));
    }

    #[test]
    fn another_key_is_refused() {
        assert!(parse("k; hardware = Table").is_err_and(|e| e.contains("unknown key `hardware`")));
    }

    #[test]
    fn both_keys_are_refused() {
        let both = parse("k; preset = XmBench, model = \"a.urdf\"");
        assert!(both.is_err_and(|e| e.contains("cannot be overridden")));
    }

    #[test]
    fn a_path_must_be_a_string() {
        assert!(parse("k; model = arm").is_err());
        assert!(parse("k; preset = \"XmBench\"").is_err());
    }
}
