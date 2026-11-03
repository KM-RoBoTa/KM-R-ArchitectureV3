use circular_buffer::FixedCircularBuffer;

use super::joint_state::{Kd, Kp, Q, Qd, Tau};
use crate::error::StateError;

#[derive(Clone, Copy, Debug, PartialEq)]
/// The current state of the robot: one history slot.
///
/// Layout: structure of arrays, one `[_; N]` per field, so a field read walks
/// contiguous memory. `repr(C)` keeps the declared field order; `align(64)`
/// starts every slot on a cache line, so a 4-actuator slot (80 bytes) spans
/// two lines and never shares one with its neighbour in the ring.
#[repr(C, align(64))]
pub(crate) struct State<const N: usize> {
    /// Array of all joint's position (as radiants).
    /// See [`super::joint_state::Q`]
    pub(crate) q: [Q; N],
    /// Array of all joint's velocites.
    /// See [`super::joint_state::Qd`]
    pub(crate) qd: [Qd; N],
    /// Array of all joint's torque.
    /// See [`super::joint_state::Tau`]
    pub(crate) tau: [Tau; N],
    /// Position gains last applied. See [`super::joint_state::Kp`]
    pub(crate) kp: [Kp; N],
    /// Velocity gains last applied. See [`super::joint_state::Kd`]
    pub(crate) kd: [Kd; N],
}

impl<const N: usize> State<N> {
    /// The robot model's home state; its source is still open in the SRS.
    pub(crate) fn home_state() -> Result<Self, StateError> {
        Err(StateError::Undecided("SRS FR-2.7: home state source"))
    }

    /// Returns a completely zeroed-out state.
    ///
    /// It is only meant to be used ONCE at the initialization phase.
    pub(crate) fn zeroed() -> Self {
        // WARN: default must depend on the robot model
        Self {
            q: [Q(0.0); N],
            qd: [Qd(0.0); N],
            tau: [Tau(0.0); N],
            kp: [Kp(0.0); N],
            kd: [Kd(0.0); N],
        }
    }

    /// Checks if the current [`State`] is zeroed.
    fn is_zeroed(&self) -> bool {
        if *self == Self::zeroed() {
            return true;
        }
        false
    }
}

impl<const N: usize> Default for State<N> {
    fn default() -> Self {
        // WARN: default must depend on the robot model
        Self::zeroed()
    }
}

#[derive(Default, Debug, PartialEq)]
/// READ ONLY
pub(crate) struct History<const N: usize, const DEPTH: usize> {
    pub(in crate::state) buf: FixedCircularBuffer<State<N>, DEPTH>,
}
