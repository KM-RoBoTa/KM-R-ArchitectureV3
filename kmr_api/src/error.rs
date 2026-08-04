use kmr_core::error::{ApiError as CoreApiError, StateError as CoreStateError};

/// Public error returned by [`State`](crate::State) accessors.
///
/// This is kmr_api's OWN error type — a re-statement of `kmr_core`'s internal
/// `StateError`, so `kmr_core` never appears in a public signature and stays
/// sealed. `From<StateError>` maps the core cause across the boundary.
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

#[derive(Debug, PartialEq, thiserror::Error)]
pub enum UserError {
    #[error(
        "Robot has no controllers — add at least one with `.add_controller(...)` before calling `.run()`"
    )]
    NoControllers,
}

impl From<CoreApiError> for UserError {
    fn from(e: CoreApiError) -> Self {
        match e {
            CoreApiError::NoControllers => Self::NoControllers,
        }
    }
}
