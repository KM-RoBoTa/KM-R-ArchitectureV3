//! WIP

/// Plain settings bag. Non-generic: holds configuration only, never the
/// controllers (those live in the separate `L` list on `Robot`).
#[derive(Default)]
pub(crate) struct RobotConfig {
    /// The time struct. See [`crate::clock::Time`].
    pub(crate) time: crate::clock::Time,
    /// How long a shutdown holds before torque off; `None`: no hold
    /// (SRS FR-4.1).
    pub(crate) hold: Option<core::time::Duration>,
}
