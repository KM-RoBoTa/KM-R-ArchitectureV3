//! WIP

use crate::limits::Limits;

/// Plain settings bag. Holds configuration only, never the controllers
/// (those live in the separate `L` list on `Robot`).
pub(crate) struct RobotConfig<const N: usize> {
    /// The time struct. See [`crate::clock::Time`].
    pub(crate) time: crate::clock::Time,
    /// The tick period `Robot::set_dt` gave; `None`: never set, the 1 ms
    /// fallback. Checked at finalization (SRS FR-3.1).
    pub(crate) dt: Option<core::time::Duration>,
    /// How long a shutdown holds before torque off; `None`: no hold
    /// (SRS FR-4.1).
    pub(crate) hold: Option<core::time::Duration>,
    /// Limits, their policies and the envelopes (SRS FR-4.5, FR-13.5).
    pub(crate) limits: Limits<N>,
}

impl<const N: usize> Default for RobotConfig<N> {
    fn default() -> Self {
        Self {
            time: crate::clock::Time::default(),
            dt: None,
            hold: None,
            limits: Limits::default(),
        }
    }
}
