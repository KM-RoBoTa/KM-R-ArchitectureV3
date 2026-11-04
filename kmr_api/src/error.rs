use std::error::Error;
use std::time::Duration;

use kmr_adapters::dynamixel::TableError;
use kmr_core::error::{ApiError as CoreApiError, StateError as CoreStateError};

/// Public error returned by [`State`](crate::State) accessors.
///
/// This is kmr_api's OWN error type, not a re-export of `kmr_core`'s
/// `StateError`: the value types (`Q`, `Qd`, `Tau`) are shared with the core,
/// but the error vocabulary users match on stays a deliberate api choice that
/// the engine can refine without breaking them. `From<CoreStateError>` maps
/// the core cause across the boundary.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum StateError {
    /// A joint index exceeded the number of recorded entries / joints.
    #[error("index {index} is out of range: only {len} entries are available")]
    OutOfRange { index: usize, len: usize },

    /// A requested history depth exceeded the retained history.
    #[error("history depth {provided_depth} is out of range: maximum depth is {max_depth}")]
    HistoryValueOutOfRange {
        provided_depth: usize,
        max_depth: usize,
    },

    /// The requested value has not been recorded yet.
    #[error("value requested is unavailable")]
    CriticalValueMissing,
}

/// Payload-free cause of a [`PrivateStateError`].
///
/// The rich error carries the offending numbers (index, len, depth…) and
/// renders them through `Display`. When you only need to branch on *what*
/// went wrong — not the specifics — match on this instead: unit variants, no
/// `{ .. }` binding. Get it via [`PrivateStateError::kind`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateErrorKind {
    /// A joint index exceeded the number of recorded entries / joints.
    OutOfRange,
    /// A requested history depth exceeded the retained history.
    HistoryValueOutOfRange,
    /// The requested value has not been recorded yet.
    CriticalValueMissing,
}

impl StateError {
    /// The payload-free cause — match this without `{ .. }`. The specific
    /// numbers stay in `self` (its `Display`).
    pub fn kind(&self) -> StateErrorKind {
        match self {
            Self::OutOfRange { .. } => StateErrorKind::OutOfRange,
            Self::HistoryValueOutOfRange { .. } => StateErrorKind::HistoryValueOutOfRange,
            Self::CriticalValueMissing => StateErrorKind::CriticalValueMissing,
        }
    }
}

impl From<CoreStateError> for StateError {
    fn from(e: CoreStateError) -> Self {
        match e {
            CoreStateError::OutOfRange { index, len } => Self::OutOfRange { index, len },
            CoreStateError::HistoryValueOutOfRange {
                provided_depth,
                max_depth,
            } => Self::HistoryValueOutOfRange {
                provided_depth,
                max_depth,
            },
            CoreStateError::CriticalValueMissing => Self::CriticalValueMissing,
        }
    }
}

/// Error returned by [`Robot::run`](crate::Robot::run): what went wrong and how to fix it.
///
/// `Debug` is derived in debug builds, so `{e:?}` shows the variant and its fields, the
/// adapter's typed error included; release builds print the `Display` text instead, so
/// `fn main() -> Result<(), UserError>` shows one readable line and no internal cause
/// (SRS NFR-2).
#[cfg_attr(debug_assertions, derive(Debug))]
#[derive(thiserror::Error)]
pub enum UserError {
    /// The builder holds no controller.
    #[error(
        "Robot has no controllers — add at least one with `.add_controller(...)` before calling `.run()`"
    )]
    NoControllers,

    /// Two bus plugins, or two different bus handles, name the same port (SRS FR-6.4: a
    /// registration error).
    #[error(
        "port `{port}` is declared as two buses; declare each port once: one `.add_plugins(..)`, or one `Bus::..build()` handle shared by its actuators"
    )]
    DuplicateBus { port: &'static str },

    /// Hardware is declared, but the adapter it needs is not compiled in.
    #[error(
        "hardware is declared, but kmr_api was built without its `{feature}` feature; enable `kmr_api/{feature}`, or remove the bus plugins and actuators to simulate"
    )]
    FeatureOff { feature: &'static str },

    /// Bus plugins are declared, but `robot!` names no preset hardware table for them.
    #[error(
        "bus plugins are declared, but this robot has no preset hardware table; on a `robot!(model = ..)` robot, hand each actuator its bus with `.add_actuator(..on(bus))` instead of `.add_plugins(..)`, or remove the plugins to simulate"
    )]
    NoHardwareDeclared,

    /// The actuators declared with `add_actuator` are not as many as the model's.
    #[error(
        "{declared} actuators are declared with `.add_actuator(..)`, but this robot's model has {expected}; declare one per non-fixed joint, in the model's joint order"
    )]
    ActuatorCount { declared: usize, expected: usize },

    /// Actuators are declared on a preset robot, whose hardware table is the preset's.
    #[error(
        "`robot!({preset})` brings its own hardware table, so `.add_actuator(..)` is refused; bind its buses with `.add_plugins(..)`, or name the robot by `robot!(model = ..)` to declare its actuators"
    )]
    PresetHardware { preset: &'static str },

    /// A bus handle carries no baud rate: without a hardware table, nothing else gives one.
    #[error("bus `{port}` has no baud rate; give it `.baud(..)` before `.build()`")]
    BusWithoutBaud { port: &'static str },

    /// The declared actuators break a rule of the hardware table; actuator numbers are engine
    /// indices, in `add_actuator` order.
    #[error("the declared actuators are refused: {source}")]
    HardwareRejected {
        #[source]
        source: Box<TableError>,
    },

    /// The hardware failed to come up. `source` is the adapter's own error, typed: reach it
    /// with [`Error::source`] and `downcast_ref`.
    #[error("hardware bring-up failed: {source}")]
    BringUp {
        #[source]
        source: Box<dyn Error + Send + Sync>,
    },

    /// The tick period is zero or outside [10 µs, 1 s] (SRS FR-3.1).
    #[error("dt of {dt:?} is out of range; give `.set_dt(..)` a value from 10 µs to 1 s")]
    DtOutOfRange { dt: Duration },
}

/// Prints the [`Display`](core::fmt::Display) text, so `fn main() -> Result<(), UserError>`
/// shows a readable line.
#[cfg(not(debug_assertions))]
impl core::fmt::Debug for UserError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(self, f)
    }
}

impl UserError {
    /// The bring-up error of an adapter failing with `source`, kept typed.
    pub(crate) fn bring_up(source: impl Error + Send + Sync + 'static) -> Self {
        Self::BringUp {
            source: Box::new(source),
        }
    }
}

impl From<CoreApiError> for UserError {
    fn from(e: CoreApiError) -> Self {
        match e {
            CoreApiError::NoControllers => Self::NoControllers,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Stands for an adapter error.
    #[derive(Debug, thiserror::Error)]
    #[error("port FT8ISZ6H not found under /dev/serial/by-id")]
    struct NotFound;

    fn every_variant() -> [UserError; 9] {
        [
            UserError::NoControllers,
            UserError::DuplicateBus { port: "FT8ISZ6H" },
            UserError::FeatureOff {
                feature: "dynamixel",
            },
            UserError::NoHardwareDeclared,
            UserError::bring_up(NotFound),
            UserError::DtOutOfRange {
                dt: Duration::from_micros(5),
            },
            UserError::ActuatorCount {
                declared: 9,
                expected: 10,
            },
            UserError::PresetHardware { preset: "XmBench" },
            UserError::BusWithoutBaud { port: "AL02L1KL" },
        ]
    }

    #[cfg(not(debug_assertions))]
    #[test]
    fn release_debug_prints_the_display_text() {
        for e in every_variant() {
            assert_eq!(format!("{e:?}"), e.to_string());
        }
    }

    #[cfg(debug_assertions)]
    #[test]
    fn debug_shows_the_variant_and_its_fields() {
        let [_, bus, _, _, bring_up, ..] = every_variant();
        assert_eq!(format!("{bus:?}"), r#"DuplicateBus { port: "FT8ISZ6H" }"#);
        assert_eq!(format!("{bring_up:?}"), "BringUp { source: NotFound }");
    }

    #[test]
    fn a_bring_up_error_keeps_the_adapter_error() {
        let e = UserError::bring_up(NotFound);
        let source = e.source().and_then(|s| s.downcast_ref::<NotFound>());
        assert!(source.is_some());
        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: port FT8ISZ6H not found under /dev/serial/by-id"
        );
    }

    #[test]
    fn every_message_is_one_line() {
        for e in every_variant() {
            assert!(!e.to_string().contains('\n'), "{e}");
        }
    }

    #[test]
    fn messages_name_the_fix() {
        let [_, _, feature, hardware, bring_up, dt, count, preset, baud] = every_variant();
        assert!(feature.to_string().contains("`kmr_api/dynamixel`"));
        assert!(hardware.to_string().contains("`.add_actuator(..on(bus))`"));
        assert!(count.to_string().starts_with("9 actuators"));
        assert!(count.to_string().contains("has 10"));
        assert!(preset.to_string().contains("`robot!(XmBench)`"));
        assert!(baud.to_string().contains("`.baud(..)`"));
        assert!(bring_up.to_string().contains("FT8ISZ6H"));
        assert!(dt.to_string().contains("5µs"));
        assert!(dt.to_string().contains("10 µs to 1 s"));
    }
}
