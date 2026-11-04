//! The model halves of the presets KM-RoBoTa ships (research R7).
//!
//! The hardware halves live in `kmr_adapters::presets`, under the same names.
//! The text is embedded here, so editing a preset rebuilds kmr_model_core and its
//! callers; the expansion needs no `include_bytes!` tracking.

/// One preset's model half.
pub(crate) struct Preset {
    /// The name the user writes in `robot!`.
    pub(crate) name: &'static str,
    /// The path shown in messages; its extension picks the reader.
    pub(crate) shown: &'static str,
    /// The descriptor.
    pub(crate) text: &'static str,
}

/// Every preset, by name.
const PRESETS: &[Preset] = &[
    Preset {
        name: "XmBench",
        shown: "kmr_model_core/presets/xm_bench.urdf",
        text: include_str!("../presets/xm_bench.urdf"),
    },
    Preset {
        name: "Rx28Chain",
        shown: "kmr_model_core/presets/rx28_chain.urdf",
        text: include_str!("../presets/rx28_chain.urdf"),
    },
];

/// The name of every preset, in table order.
pub(crate) fn names() -> impl Iterator<Item = &'static str> {
    PRESETS.iter().map(|p| p.name)
}

/// The preset named `name`, or the error listing every preset.
pub(crate) fn find(name: &str) -> Result<&'static Preset, String> {
    PRESETS.iter().find(|p| p.name == name).ok_or_else(|| {
        let all: Vec<&str> = names().collect();
        format!("kmr: no preset named {name}; presets: {}", all.join(", "))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Reader, validate};
    use crate::urdf::Urdf;

    fn names(preset: &str) -> Result<Vec<String>, String> {
        let p = find(preset)?;
        let model = Urdf::read(p.text, p.shown).map_err(|e| e.message())?;
        let robot = validate(model, p.shown).map_err(|e| e.message())?;
        Ok(robot.actuators.into_iter().map(|a| a.name).collect())
    }

    #[test]
    fn xm_bench_gives_four_actuators_in_hardware_order() {
        assert_eq!(
            names("XmBench"),
            Ok(["joint1", "joint2", "joint3", "joint4"]
                .map(String::from)
                .to_vec())
        );
    }

    #[test]
    fn rx28_chain_gives_ten_actuators() {
        assert_eq!(names("Rx28Chain").map(|n| n.len()), Ok(10));
    }

    #[test]
    fn an_unknown_name_names_itself_and_lists_the_presets() {
        assert_eq!(
            find("Xmbench").err().as_deref(),
            Some("kmr: no preset named Xmbench; presets: XmBench, Rx28Chain")
        );
    }

    #[test]
    fn every_preset_is_a_urdf() {
        assert!(PRESETS.iter().all(|p| p.shown.ends_with(".urdf")));
    }
}
