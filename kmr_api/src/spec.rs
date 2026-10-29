//! The link from the robot `robot!` names to its hardware (SRS FR-6.1, FR-5.1).
//!
//! The `cfg`s live here, never in `robot!`'s expansion: there they would test the user's
//! features, not kmr_api's.

use kmr_adapters::binding::Binding;
use kmr_adapters::dynamixel::Table;
use kmr_core::hal::{Hal, NoHardware};

use crate::{N, UserError};

/// The robot this binary drives, as the type parameter of [`Robot`](crate::Robot).
///
/// `robot!` implements it on the `__Spec` it emits; users never implement it.
#[doc(hidden)]
pub trait Spec {
    /// The robot's hardware driver. One driver for now; becomes the driver set in T069.
    type Adapter: Hal;

    /// Binds the bus plugins to the driver, no I/O. One driver for now; becomes the driver set
    /// in T069. `run()` calls it only when plugins are declared.
    ///
    /// # Errors
    ///
    /// [`UserError::FeatureOff`], [`UserError::NoHardwareDeclared`], or the driver's binding
    /// error as [`UserError::BringUp`].
    fn bind(plugins: &[Binding]) -> Result<Self::Adapter, UserError>;
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
pub fn __bind_none(_plugins: &[Binding]) -> Result<__NoHardware, UserError> {
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

/// Binds `plugins` to the buses of `table`, on the Dynamixel SDK. No I/O.
///
/// # Errors
///
/// The adapter's binding error (label, baud) as [`UserError::BringUp`].
#[doc(hidden)]
#[cfg(feature = "dynamixel")]
pub fn __bind_dynamixel<const NB: usize>(
    table: Table<NB, N>,
    plugins: &[Binding],
) -> Result<__DxlAdapter<NB>, UserError> {
    use kmr_adapters::dynamixel::{DxlRs, Dynamixel};
    Dynamixel::bind(DxlRs::new(), table, plugins).map_err(|e| UserError::BringUp {
        cause: e.to_string(),
    })
}

/// Refuses the plugins: the Dynamixel adapter is not compiled in.
///
/// # Errors
///
/// Always [`UserError::FeatureOff`].
#[doc(hidden)]
#[cfg(not(feature = "dynamixel"))]
pub fn __bind_dynamixel<const NB: usize>(
    _table: Table<NB, N>,
    _plugins: &[Binding],
) -> Result<__DxlAdapter<NB>, UserError> {
    Err(UserError::FeatureOff {
        feature: "dynamixel",
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use kmr_adapters::binding::Port;
    use kmr_adapters::presets::XmBench;

    const PLUGIN: Binding = Binding {
        port: Port::ById("FT8ISZ6H"),
        baud: None,
        bus: None,
    };

    #[test]
    fn without_a_table_plugins_are_refused() {
        assert!(matches!(
            __bind_none(&[PLUGIN]),
            Err(UserError::NoHardwareDeclared)
        ));
    }

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn without_the_feature_plugins_are_refused() {
        let res = __bind_dynamixel(XmBench::HARDWARE, &[PLUGIN]);
        assert!(matches!(
            res,
            Err(UserError::FeatureOff {
                feature: "dynamixel"
            })
        ));
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn a_binding_error_is_a_bring_up_error() {
        let wrong = Binding {
            bus: Some("nope"),
            ..PLUGIN
        };
        let res = __bind_dynamixel(XmBench::HARDWARE, &[wrong]);
        assert!(matches!(res, Err(UserError::BringUp { cause }) if cause.contains("nope")));
    }

    #[cfg(feature = "dynamixel")]
    #[test]
    fn a_matching_plugin_binds() {
        assert!(__bind_dynamixel(XmBench::HARDWARE, &[PLUGIN]).is_ok());
    }
}
