use core::fmt;
use std::time::Duration;

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
#[derive(PartialEq, thiserror::Error)]
pub enum UserError {
    /// The builder holds no controller.
    #[error(
        "Robot has no controllers — add at least one with `.add_controller(...)` before calling `.run()`"
    )]
    NoControllers,

    /// Two bus plugins bind the same port (SRS FR-6.4: a registration error).
    #[error("port `{port}` is bound by two bus plugins; bind each port once")]
    DuplicateBus { port: &'static str },

    /// Bus plugins are declared, but the adapter they need is not compiled in.
    #[error(
        "bus plugins are declared, but kmr_api was built without its `{feature}` feature; enable `kmr_api/{feature}`, or remove the plugins to simulate"
    )]
    FeatureOff { feature: &'static str },

    /// Bus plugins are declared, but `robot!` names no hardware for them.
    #[error(
        "bus plugins are declared, but this robot has no hardware table; add `hardware = ..` to `robot!`, or remove the plugins to simulate"
    )]
    NoHardwareDeclared,

    /// The hardware failed to come up; `cause` is the adapter's message.
    #[error("hardware bring-up failed: {cause}")]
    BringUp { cause: String },

    /// The tick period is zero or outside [10 µs, 1 s] (SRS FR-3.1).
    #[error("dt of {dt:?} is out of range; give `.set_dt(..)` a value from 10 µs to 1 s")]
    DtOutOfRange { dt: Duration },
}

/// Prints the [`Display`](fmt::Display) text, so `fn main() -> Result<(), UserError>` shows a
/// readable line.
impl fmt::Debug for UserError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
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

    fn every_variant() -> [UserError; 6] {
        [
            UserError::NoControllers,
            UserError::DuplicateBus { port: "FT8ISZ6H" },
            UserError::FeatureOff {
                feature: "dynamixel",
            },
            UserError::NoHardwareDeclared,
            UserError::BringUp {
                cause: "port FT8ISZ6H not found under /dev/serial/by-id".to_string(),
            },
            UserError::DtOutOfRange {
                dt: Duration::from_micros(5),
            },
        ]
    }

    #[test]
    fn debug_prints_the_display_text() {
        for e in every_variant() {
            assert_eq!(format!("{e:?}"), e.to_string());
        }
    }

    #[test]
    fn every_message_is_one_line() {
        for e in every_variant() {
            assert!(!e.to_string().contains('\n'), "{e}");
        }
    }

    #[test]
    fn messages_name_the_fix() {
        let [_, _, feature, hardware, bring_up, dt] = every_variant();
        assert!(feature.to_string().contains("`kmr_api/dynamixel`"));
        assert!(hardware.to_string().contains("`hardware = ..`"));
        assert!(bring_up.to_string().contains("FT8ISZ6H"));
        assert!(dt.to_string().contains("5µs"));
        assert!(dt.to_string().contains("10 µs to 1 s"));
    }
}
