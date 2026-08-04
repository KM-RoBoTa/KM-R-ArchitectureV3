#[derive(Debug, thiserror::Error)]
pub enum CoreError {
    #[error(transparent)]
    State(#[from] StateError),
}

#[derive(Debug, PartialEq, thiserror::Error)]
pub enum StateError {
    /// [`StateRing::prev`] was called with an index that exceeds the number of
    /// recorded entries.
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
pub enum ApiError {
    #[error(
        "Robot has no controllers — add at least one with `.add_controller(...)` before calling `.run()`"
    )]
    NoControllers,
}
