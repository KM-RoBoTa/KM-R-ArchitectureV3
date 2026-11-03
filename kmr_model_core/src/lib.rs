//! The robot descriptor, read and checked on the build machine (SRS FR-6.1).
//!
//! The library behind `kmr_model::model!` (the expansion of `kmr_api::robot!`) and behind
//! kmr_ffi's build helper, so a user `build.rs` gets the same N as `robot!` (research R12,
//! decision 0009). It runs at build time only: the heap it uses is gone when the build ends.
//!
//! Every error is one line, `kmr: <file>[:<line>[:<col>]]: ..`, the text `robot!` shows as its
//! compile error (FR-008 (SRS FR-6.4)).

#![forbid(clippy::unwrap_used)]
// Test helpers may stop on a broken fixture.
#![cfg_attr(not(test), forbid(clippy::expect_used, clippy::panic))]
#![warn(missing_docs)]

mod model;
mod presets;
mod serde_format;
mod urdf;

use std::path::Path;

pub use crate::model::{CheckedActuator, CheckedLink, Kind, Robot};

use crate::model::{Format, ModelError, Reader, validate};
use crate::serde_format::{Json, Ron};
use crate::urdf::Urdf;

impl Robot {
    /// Actuator count, the N `robot!` gives the robot.
    pub fn n(&self) -> usize {
        self.actuators.len()
    }
}

/// The checked robot of the preset named `name`.
///
/// # Errors
///
/// The `robot!` message: no such preset (listing every preset), or a fault of the preset's
/// descriptor.
pub fn preset(name: &str) -> Result<Robot, String> {
    let preset = presets::find(name)?;
    parse(preset.text, preset.shown)
}

/// The name of every preset, in the order `robot!`'s error lists them.
pub fn preset_names() -> impl Iterator<Item = &'static str> {
    presets::names()
}

/// The checked robot of the descriptor file at `path`.
///
/// `path` is taken as given; `robot!` joins it to the calling crate's `CARGO_MANIFEST_DIR`
/// first, and so must a caller that wants the same file.
///
/// # Errors
///
/// The `robot!` message: a path that is not UTF-8, an unknown extension, an unreadable file, or
/// a fault of the descriptor.
pub fn descriptor(path: &Path) -> Result<Robot, String> {
    let shown = shown(path)?;
    let format = format(path)?;
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("kmr: cannot read robot descriptor {shown}: {e}"))?;
    check(&text, format, shown)
}

/// The checked robot of `text`; the extension of `shown` picks the syntax, and `shown` names
/// the file in errors.
///
/// # Errors
///
/// The `robot!` message for an unknown extension or a fault of the descriptor.
pub fn parse(text: &str, shown: &str) -> Result<Robot, String> {
    check(text, format(Path::new(shown))?, shown)
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

/// Read by the reader of `format`, then validated.
fn check(text: &str, format: Format, shown: &str) -> Result<Robot, String> {
    let model = match format {
        Format::Urdf => Urdf::read(text, shown),
        Format::Ron => Ron::read(text, shown),
        Format::Json => Json::read(text, shown),
    };
    model
        .and_then(|model| validate(model, shown))
        .map_err(|e: ModelError| e.message())
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
        assert!(descriptor(path).is_err_and(|e| e.contains("is not valid UTF-8")));
    }

    #[test]
    fn an_unknown_extension_is_an_error() {
        let err = descriptor(Path::new("/robot/arm.yaml")).err();
        assert_eq!(
            err.as_deref(),
            Some("kmr: /robot/arm.yaml: expected .urdf, .ron or .json")
        );
    }

    #[test]
    fn a_missing_file_is_an_error() {
        let err = descriptor(Path::new("/no/such/robot.urdf")).err();
        assert!(err.is_some_and(|e| {
            e.starts_with("kmr: cannot read robot descriptor /no/such/robot.urdf: ")
        }));
    }

    #[test]
    fn a_preset_gives_its_actuator_count() {
        assert_eq!(preset("XmBench").map(|r| r.n()), Ok(4));
        assert_eq!(preset("Rx28Chain").map(|r| r.n()), Ok(10));
    }

    #[test]
    fn an_unknown_preset_is_the_robot_macro_message() {
        assert_eq!(
            preset("Nope").err().as_deref(),
            Some("kmr: no preset named Nope; presets: XmBench, Rx28Chain")
        );
    }

    #[test]
    fn a_descriptor_file_gives_the_same_robot_as_its_text() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("presets/xm_bench.urdf");
        let Some(shown) = path.to_str() else {
            unreachable!("the manifest dir is UTF-8 in this repo");
        };
        let text = include_str!("../presets/xm_bench.urdf");
        assert_eq!(descriptor(&path), parse(text, shown));
    }
}
