//! The robot descriptor, read at compile time (SRS FR-6.1).
//!
//! Runs inside the compiler on the build machine: the heap it uses is gone
//! when the build ends, and the binary only receives constants.

#![cfg_attr(
    not(test),
    forbid(clippy::unwrap_used, clippy::expect_used, clippy::panic)
)]

mod emit;
mod input;
mod model;
mod presets;
mod serde_format;
mod urdf;

use std::path::{Path, PathBuf};

use proc_macro::TokenStream;
use proc_macro2::{Span, TokenStream as TokenStream2};

use crate::input::{Input, Source};
use crate::model::{Descriptor, Format, ModelError, Reader, Robot, validate};
use crate::serde_format::{Json, Ron};
use crate::urdf::Urdf;

/// `N`, the `model` module and the rebuild tracking of one robot model.
///
/// Called by `kmr_api::robot!` only, as `model!($crate; preset = Name)` or
/// `model!($crate; model = "path")`; the path is relative to the calling
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
            let descriptor = load(&path).map_err(|msg| syn::Error::new(span, msg))?;
            let shown = shown(&descriptor.path).map_err(|msg| syn::Error::new(span, msg))?;
            let robot = check(&descriptor, shown, span)?;
            Ok(emit::emit(&robot, &input.krate, Some(shown)))
        }
        Source::Preset(name) => {
            let span = name.span();
            let preset =
                presets::find(&name.to_string()).map_err(|msg| syn::Error::new(span, msg))?;
            let descriptor = Descriptor {
                path: PathBuf::from(preset.shown),
                format: format(Path::new(preset.shown))
                    .map_err(|msg| syn::Error::new(span, msg))?,
                text: preset.text.to_owned(),
            };
            let robot = check(&descriptor, preset.shown, span)?;
            Ok(emit::emit(&robot, &input.krate, None))
        }
    }
}

/// `path` as the text shown in messages and given to `include_bytes!`.
fn shown(path: &Path) -> Result<&str, String> {
    path.to_str().ok_or_else(|| {
        format!(
            "kmr: the descriptor path {} is not valid UTF-8",
            path.display()
        )
    })
}

/// The syntax of the descriptor at `path`, from its extension.
fn format(path: &Path) -> Result<Format, String> {
    match path.extension().and_then(|e| e.to_str()) {
        Some("urdf") => Ok(Format::Urdf),
        Some("ron") => Ok(Format::Ron),
        Some("json") => Ok(Format::Json),
        _ => Err(format!(
            "kmr: {}: expected .urdf, .ron or .json",
            path.display()
        )),
    }
}

/// Validation step 1: the descriptor at `path`, read.
fn load(path: &Path) -> Result<Descriptor, String> {
    let shown = shown(path)?;
    let format = format(path)?;
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("kmr: cannot read robot descriptor {shown}: {e}"))?;
    Ok(Descriptor {
        path: path.to_owned(),
        format,
        text,
    })
}

/// The checked robot of `descriptor`, its errors spanned on `span`.
fn check(descriptor: &Descriptor, shown: &str, span: Span) -> syn::Result<Robot> {
    read(descriptor, shown)
        .and_then(|model| validate(model, shown))
        .map_err(|e| syn::Error::new(span, e.message()))
}

/// The model of `descriptor`, by the reader of its syntax.
fn read(descriptor: &Descriptor, shown: &str) -> Result<model::Model, ModelError> {
    match descriptor.format {
        Format::Urdf => Urdf::read(&descriptor.text, shown),
        Format::Ron => Ron::read(&descriptor.text, shown),
        Format::Json => Json::read(&descriptor.text, shown),
    }
}

/// Actuator count of a URDF descriptor: its joints that are not `fixed`,
/// as a `usize` constant expression.
///
/// The path is relative to the calling crate's `Cargo.toml`. Editing the
/// descriptor rebuilds the crate. Any error is a compile error (SRS FR-6.4).
#[proc_macro]
pub fn actuator_count(input: TokenStream) -> TokenStream {
    match count(input) {
        Ok(expr) => expr,
        Err(msg) => compile_error(&msg),
    }
}

fn count(input: TokenStream) -> Result<TokenStream, String> {
    let lit = input.to_string();
    let rel = lit
        .strip_prefix('"')
        .and_then(|s| s.strip_suffix('"'))
        .ok_or_else(|| format!("kmr: expected a descriptor path in quotes, got `{lit}`"))?;
    let dir = std::env::var("CARGO_MANIFEST_DIR")
        .map_err(|_| "kmr: CARGO_MANIFEST_DIR is not set; build with cargo".to_string())?;
    let path = PathBuf::from(dir).join(rel);
    let shown = path.display().to_string();

    match path.extension().and_then(|e| e.to_str()) {
        Some("urdf") => {}
        Some("ron") | Some("json") => {
            return Err(format!(
                "kmr: SRS FR-6.1: .ron and JSON descriptors are not supported yet ({shown})"
            ));
        }
        _ => {
            return Err(format!(
                "kmr: unknown descriptor type, expected .urdf: {shown}"
            ));
        }
    }

    let text = std::fs::read_to_string(&path)
        .map_err(|e| format!("kmr: cannot read robot descriptor {shown}: {e}"))?;
    let doc = roxmltree::Document::parse(&text)
        .map_err(|e| format!("kmr: {shown} is not valid XML: {e}"))?;
    let robot = doc.root_element();
    if !robot.has_tag_name("robot") {
        return Err(format!("kmr: {shown}: the root element must be <robot>"));
    }

    let mut n = 0usize;
    for joint in robot.children().filter(|c| c.has_tag_name("joint")) {
        let name = joint.attribute("name").unwrap_or("?");
        match joint.attribute("type") {
            Some("fixed") => {}
            Some("revolute" | "continuous" | "prismatic") => n += 1,
            Some(other) => {
                return Err(format!(
                    "kmr: {shown}: joint `{name}` has type `{other}`; non-1-DOF joints are roadmap (SRS section 8)"
                ));
            }
            None => return Err(format!("kmr: {shown}: joint `{name}` has no type")),
        }
    }

    // include_bytes! makes cargo rebuild the caller when the descriptor changes.
    format!("{{ const _: &[u8] = include_bytes!({shown:?}); {n}usize }}")
        .parse()
        .map_err(|e| format!("kmr: internal: {e:?}"))
}

fn compile_error(msg: &str) -> TokenStream {
    format!("compile_error!({msg:?})")
        .parse()
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(unix)]
    #[test]
    fn a_path_that_is_not_utf8_is_an_error() {
        use std::ffi::OsStr;
        use std::os::unix::ffi::OsStrExt;

        let path = Path::new(OsStr::from_bytes(b"/robot/\xff.urdf"));
        assert!(load(path).is_err_and(|e| e.contains("is not valid UTF-8")));
    }

    #[test]
    fn an_unknown_extension_is_an_error() {
        let err = load(Path::new("/robot/arm.yaml")).err();
        assert_eq!(
            err.as_deref(),
            Some("kmr: /robot/arm.yaml: expected .urdf, .ron or .json")
        );
    }

    #[test]
    fn a_missing_file_is_an_error() {
        let err = load(Path::new("/no/such/robot.urdf")).err();
        assert!(err.is_some_and(|e| {
            e.starts_with("kmr: cannot read robot descriptor /no/such/robot.urdf: ")
        }));
    }
}
