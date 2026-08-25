//! WIP

/// Plain settings bag. Non-generic: holds configuration only, never the
/// controllers (those live in the separate `L` list on `Robot`).
#[derive(Default)]
pub(crate) struct RobotConfig {
    /// The time struct. See [`crate::clock::Time`].
    pub time: crate::clock::Time,
}
