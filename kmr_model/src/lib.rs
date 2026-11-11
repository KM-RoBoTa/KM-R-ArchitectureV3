//! The robot descriptor, read at compile time (SRS FR-6.1).
//!
//! A thin proc macro over `kmr_model_core`, which reads and checks the
//! descriptor; this crate parses the macro input and emits the constants.
//! Runs inside the compiler on the build machine: the heap it uses is gone
//! when the build ends, and the binary only receives constants.

#![forbid(clippy::unwrap_used)]
// Test helpers may stop on a broken fixture.
#![cfg_attr(not(test), forbid(clippy::expect_used, clippy::panic))]

mod emit;
mod input;

use std::path::PathBuf;

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;

use crate::input::{Input, Source};

/// `N`, the `model` module and the rebuild tracking of one robot model.
///
/// Called by `kmr_api::robot!` only, as `model!($crate; preset = Name)`,
/// `model!($crate; model = "path")`, or `model!($crate; actuators = 12)` for N
/// without descriptor; the path is relative to the calling
/// crate's `Cargo.toml`. Any fault is a compile error naming the file, the
/// element and the reason (FR-008 (SRS FR-6.4)).
#[proc_macro]
pub fn model(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as Input);
    expand(&input)
        .unwrap_or_else(syn::Error::into_compile_error)
        .into()
}

/// The expansion of `model!`, or the error spanned on the path or the
/// preset name.
fn expand(input: &Input) -> syn::Result<TokenStream2> {
    match &input.source {
        Source::Model(lit) => {
            let span = lit.span();
            let dir = std::env::var_os("CARGO_MANIFEST_DIR").ok_or_else(|| {
                syn::Error::new(span, "kmr: CARGO_MANIFEST_DIR is not set; build with cargo")
            })?;
            let path = PathBuf::from(dir).join(lit.value());
            let robot =
                kmr_model_core::descriptor(&path).map_err(|msg| syn::Error::new(span, msg))?;
            // `descriptor` refused a path that is not UTF-8.
            let shown = path.to_str().unwrap_or_default();
            Ok(emit::emit(&robot, &input.krate, Some(shown)))
        }
        Source::Preset(name) => {
            let robot = kmr_model_core::preset(&name.to_string())
                .map_err(|msg| syn::Error::new(name.span(), msg))?;
            Ok(emit::emit(&robot, &input.krate, None))
        }
        Source::Count(lit) => {
            let n = lit.base10_parse::<usize>()?;
            let robot = kmr_model_core::count(n).map_err(|msg| syn::Error::new(lit.span(), msg))?;
            Ok(emit::emit(&robot, &input.krate, None))
        }
    }
}
