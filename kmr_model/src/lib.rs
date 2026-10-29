//! The robot descriptor, read at compile time (SRS FR-6.1).
//!
//! Runs inside the compiler on the build machine: the heap it uses is gone
//! when the build ends, and the binary only receives constants.

#![forbid(clippy::unwrap_used)]

use std::path::PathBuf;

use proc_macro::TokenStream;

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
