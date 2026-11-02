//! The link from the robot `robot!` names to its hardware (SRS FR-6.1, FR-5.1).
//!
//! The `cfg`s live here, never in `robot!`'s expansion: there they would test the user's
//! features, not kmr_api's.

use kmr_adapters::binding::Binding;
use kmr_adapters::dynamixel::Placement;
use kmr_core::hal::{Drivers, NoHardware};
use kmr_core::{HCons, HNil};

use crate::UserError;
use crate::brand::{Brand, Refused};

/// The robot this binary drives, as the type parameter of [`Robot`](crate::Robot).
///
/// `robot!` implements it on the `__Spec` it emits; users never implement it.
#[doc(hidden)]
pub trait Spec {
    /// The robot's hardware: the list of its drivers, one member per brand,
    /// `HCons<A, HCons<B, HNil>>`. Its error crosses threads inside [`UserError::BringUp`].
    type Drivers: Drivers<Error: Send + Sync>;

    /// Binds the bus plugins to the members, no I/O. `run()` calls it only when plugins are
    /// declared.
    ///
    /// # Errors
    ///
    /// [`UserError::FeatureOff`], [`UserError::NoHardwareDeclared`], a plugin no member takes,
    /// or a member's binding error as [`UserError::BringUp`].
    fn bind(plugins: &[Binding]) -> Result<Self::Drivers, UserError>;
}

/// The driver of a robot without a hardware table.
#[doc(hidden)]
pub type __NoHardware = NoHardware;

/// Refuses the plugins: `robot!` names no hardware table for them.
///
/// # Errors
///
/// Always [`UserError::NoHardwareDeclared`].
#[doc(hidden)]
pub fn __bind_none(_plugins: &[Binding]) -> Result<HCons<__NoHardware, HNil>, UserError> {
    Err(UserError::NoHardwareDeclared)
}

/// The Dynamixel driver of a robot with `NB` Dynamixel buses.
#[doc(hidden)]
#[cfg(feature = "dynamixel")]
pub type __DxlAdapter<const NB: usize> =
    kmr_adapters::dynamixel::Dynamixel<kmr_adapters::dynamixel::DxlRs, NB>;

/// No Dynamixel driver: kmr_api is built without its `dynamixel` feature.
#[doc(hidden)]
#[cfg(not(feature = "dynamixel"))]
pub type __DxlAdapter<const NB: usize> = kmr_core::hal::NoHardware;

/// The Dynamixel part of a robot: its buses bind to the Dynamixel SDK.
impl<const NB: usize, const M: usize> Brand for Placement<NB, M> {
    type Member = __DxlAdapter<NB>;

    fn labels(&self) -> Vec<&'static str> {
        self.table().buses.iter().map(|bus| bus.label).collect()
    }

    /// Binds `plugins` to the buses of the table, on the Dynamixel SDK; without kmr_api's
    /// `dynamixel` feature, refuses them.
    #[cfg(feature = "dynamixel")]
    fn bind(
        self,
        plugins: &[Binding],
    ) -> Result<Self::Member, Refused<kmr_adapters::dynamixel::DynamixelError>> {
        use kmr_adapters::dynamixel::{DxlRs, Dynamixel};
        Dynamixel::bind(DxlRs::new(), self, plugins).map_err(Refused::Member)
    }

    /// Binds `plugins` to the buses of the table, on the Dynamixel SDK; without kmr_api's
    /// `dynamixel` feature, refuses them.
    #[cfg(not(feature = "dynamixel"))]
    fn bind(
        self,
        _plugins: &[Binding],
    ) -> Result<Self::Member, Refused<core::convert::Infallible>> {
        Err(Refused::FeatureOff("dynamixel"))
    }
}

/// The brands `robot!` knows, by the name its `hardware = { brand: .. }` argument gives them.
///
/// Each module gives the type of the brand's part, `Hardware<NB, M>`, and `shape`, its bus and
/// actuator counts, read in const evaluation from the user's expression.
#[doc(hidden)]
pub mod __brand {
    /// The Dynamixel part: a placed hardware table, `Table { .. }.at([..])`.
    pub mod dynamixel {
        use kmr_adapters::dynamixel::Placement;

        /// The type of the Dynamixel part.
        pub type Hardware<const NB: usize, const M: usize> = Placement<NB, M>;

        /// Bus and actuator counts of `hardware`.
        pub const fn shape<const NB: usize, const M: usize>(
            _hardware: &Placement<NB, M>,
        ) -> (usize, usize) {
            (NB, M)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::brand::__bind;
    use kmr_adapters::binding::Port;
    use kmr_adapters::presets::XmBench;

    const PLUGIN: Binding = Binding {
        port: Port::ById("FT8ISZ6H"),
        baud: None,
        bus: None,
    };

    /// XmBench as the one member of its robot.
    fn xm_bench() -> HCons<Placement<1, 4>, HNil> {
        HCons {
            head: XmBench::HARDWARE.in_order(),
            tail: HNil,
        }
    }

    #[test]
    fn without_a_table_plugins_are_refused() {
        assert!(matches!(
            __bind_none(&[PLUGIN]),
            Err(UserError::NoHardwareDeclared)
        ));
    }

    #[test]
    fn a_table_names_its_bus_labels() {
        assert_eq!(XmBench::HARDWARE.in_order().labels(), ["bus0"]);
    }

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn without_the_feature_plugins_are_refused() {
        let res = __bind(xm_bench(), &[PLUGIN]);
        assert!(matches!(
            res,
            Err(UserError::FeatureOff {
                feature: "dynamixel"
            })
        ));
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn a_binding_error_is_a_bring_up_error_naming_the_member() {
        let wrong = Binding {
            bus: Some("nope"),
            ..PLUGIN
        };
        let res = __bind(xm_bench(), &[wrong]);
        assert!(
            matches!(&res, Err(UserError::BringUp { source }) if source.to_string().starts_with("member 0 (kmr_adapters::dynamixel::Dynamixel<") && source.to_string().contains("nope")),
            "{:?}",
            res.err()
        );
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn a_matching_plugin_binds() {
        assert!(__bind(xm_bench(), &[PLUGIN]).is_ok());
    }
}
