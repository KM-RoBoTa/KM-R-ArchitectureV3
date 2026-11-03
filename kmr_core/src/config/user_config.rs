//! WIP

/// Plain settings bag. Non-generic: holds configuration only, never the
/// controllers (those live in the separate `L` list on `Robot`).
#[derive(Default)]
pub(crate) struct RobotConfig {
    /// The time struct. See [`crate::clock::Time`].
    pub(crate) time: crate::clock::Time,
    /// The tick period `Robot::set_dt` gave; `None`: never set, the 1 ms
    /// fallback. Checked at finalization (SRS FR-3.1).
    pub(crate) dt: Option<core::time::Duration>,
    /// How long a shutdown holds before torque off; `None`: no hold
    /// (SRS FR-4.1).
    pub(crate) hold: Option<core::time::Duration>,
}
