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

    /// The behaviour is not decided yet; the text names the open SRS item.
    #[error("not decided yet: {0}")]
    Undecided(&'static str),

    /// No sample of the actuator exists `depth` ticks back: it was not read
    /// yet then, or never (a run without hardware).
    #[error("no sample {depth} ticks back: the actuator had not been read yet")]
    NoSample { depth: usize },
}

/// A configuration fault found at finalization, before any I/O (SRS FR-3.1, FR-6.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ConfigError {
    /// The tick period is zero or outside [10 µs, 1 s].
    #[error("dt of {dt:?} is out of range: it must be from 10 µs to 1 s")]
    DtOutOfRange { dt: core::time::Duration },
    /// Actuator `actuator` has a limit and no policy for its breach (SRS
    /// FR-4.5).
    #[error("actuator {actuator} has a limit and no policy for its breach")]
    NoPolicy { actuator: usize },
}

/// Why a run failed: its configuration, found before any I/O, or its hardware (`E`).
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum RunError<E> {
    /// The builder's configuration is refused; nothing was opened.
    #[error(transparent)]
    Config(#[from] ConfigError),
    /// The hardware failed: a claim mismatch or a member's bring-up error.
    #[error(transparent)]
    Hal(E),
}
