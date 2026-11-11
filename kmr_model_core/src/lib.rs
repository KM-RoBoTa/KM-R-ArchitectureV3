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

/// A robot of `n` revolute actuators and no descriptor: N only.
///
/// Its actuators are named `j0` to `j<n-1>`, by engine index, and it has no link. It is a
/// placeholder: it has no limit to hand the engine.
///
/// # Errors
///
/// The `robot!` message for an `n` outside `1..=64` (the SRS write mask).
pub fn count(n: usize) -> Result<Robot, String> {
    if !(1..=64).contains(&n) {
        return Err(format!(
            "kmr: actuators = {n}: expected 1 to 64 actuators (SRS write mask)"
        ));
    }
    let actuators = (0..n)
        .map(|i| CheckedActuator {
            name: format!("j{i}"),
            kind: Kind::Revolute,
            lower: 0.0,
            upper: 0.0,
            speed: 0.0,
            effort: 0.0,
        })
        .collect();
    Ok(Robot {
        actuators,
        links: Vec::new(),
        placeholder: true,
    })
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
    let mut robot = model
        .and_then(|model| validate(model, shown))
        .map_err(|e: ModelError| e.message())?;
    robot.placeholder = placeholder(text);
    Ok(robot)
}

/// Whether the descriptor's first comment, past an XML declaration, starts with `PLACEHOLDER`.
fn placeholder(text: &str) -> bool {
    let mut rest = text.trim_start();
    if let Some(after) = rest.strip_prefix("<?xml") {
        rest = after.split_once("?>").map_or("", |(_, r)| r).trim_start();
    }
    rest.strip_prefix("<!--")
        .or_else(|| rest.strip_prefix("//"))
        .is_some_and(|comment| comment.trim_start().starts_with("PLACEHOLDER"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_count_is_n_revolute_placeholder_actuators_named_by_index() {
        let robot = count(3).expect("3 is in range");
        assert_eq!(robot.n(), 3);
        assert!(robot.placeholder);
        assert!(robot.links.is_empty());
        let names: Vec<_> = robot.actuators.iter().map(|a| a.name.as_str()).collect();
        assert_eq!(names, ["j0", "j1", "j2"]);
        assert!(robot.actuators.iter().all(|a| a.kind == Kind::Revolute));
        assert!(count(64).is_ok());
    }

    #[test]
    fn a_count_outside_the_write_mask_is_refused() {
        for n in [0, 65] {
            assert!(
                count(n).is_err_and(|e| e.contains("expected 1 to 64")),
                "{n}"
            );
        }
    }

    #[test]
    fn a_placeholder_is_told_by_its_first_comment() {
        assert!(preset("XmBench").is_ok_and(|r| r.placeholder));
        assert!(preset("Rx28Chain").is_ok_and(|r| !r.placeholder));
        assert!(placeholder(
            "<?xml version=\"1.0\"?>\n<!-- PLACEHOLDER: x -->"
        ));
        assert!(placeholder("// PLACEHOLDER\n(a: 1)"));
        assert!(!placeholder("<robot><!-- PLACEHOLDER --></robot>"));
    }

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
