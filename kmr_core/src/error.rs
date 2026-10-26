//! The error module.
//!
//! All the errors in the crate are derived from [`thiserror::Error`]
//!
//! This crate as a whole defined on principle 4 types of errors with each their
//! own simple guidelines:
//! - User-side recoverable: Non-critical errors that we allow the user to take action
//!   upon.
//! - User-side unrecoverable: Critial errors that must be very detailed to
//!   allow the user to understand clearly what went wrong.
//!
//! - Internal recoverable: Non-critical errors that must be opaque for the user.
//!   This can be mostly done with [`Option<T>`].
//! - Internal unrecoverable: Critial errors that mustn't be explained to the
//!   user. Internally, these must be as detailed as possible but the user
//!   only see "An unexpected error was caught, please contact us at
//!   <our coordinates>".
//!
//! Errors must be highly descriptive to ensure the API user has all the
//! required information he ever needs when faced with an unrecoverable error.
//!
//! In the case of an unrecoverable error,

#[derive(Debug, thiserror::Error)]
pub(crate) enum CoreError {
    #[error(transparent)]
    State(#[from] StateError),
}

#[derive(Debug, PartialEq, thiserror::Error)]
/// Errors while manipulating [`crate::state::State`] and [`crate::state::History`]
pub enum StateError {
    #[error("prev({index}) is out of range: only {len} entries have been recorded so far")]
    OutOfRange { index: usize, len: usize },

    #[error("History depth {provided_depth} is out of range: maximum depth is {max_depth}")]
    HistoryValueOutOfRange {
        provided_depth: usize,
        max_depth: usize,
    },

    #[error("Value requested is unavailable.")]
    CriticalValueMissing,
}

#[derive(Debug, PartialEq, thiserror::Error)]
/// Defined as a simple mistake during the usage of the API.
pub enum ApiError {
    #[error(
        "Robot has no controllers — add at least one with `.add_controller(...)` before calling `.run()`"
    )]
    NoControllers,
}
