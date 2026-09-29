use circular_buffer::FixedCircularBuffer;

use super::JOINTS;
use super::joint_state::{Q, Qd, Tau};

#[derive(Debug, PartialEq)]
/// The current state of the robot.
///
///
pub struct State<const N: usize> {
    /// Array of all joint's position (as radiants).
    /// See [`super::joint_state::Q`]
    pub(crate) q: [Q; N],
    /// Array of all joint's velocites.
    /// See [`super::joint_state::Qd`]
    pub(crate) qd: [Qd; N],
    /// Array of all joint's torque.
    /// See [`super::joint_state::Tau`]
    pub(crate) tau: [Tau; N],
}

impl State<JOINTS> {
    fn home_state() -> Self {
        todo!(
            "I must save somewhere permanant the home state. This must be the default because zeroing all actuators as default is dangerous."
        )
    }

    /// Returns a completely zeroed-out state.
    ///
    /// It is only meant to be used ONCE at the initialization phase.
    pub fn zeroed() -> Self {
        // WARN: default must depend on the robot model
        Self {
            q: [Q(0.0); JOINTS],
            qd: [Qd(0.0); JOINTS],
            tau: [Tau(0.0); JOINTS],
        }
    }

    /// Checks if the current [`State`] is zeroed.
    fn is_zeroed(&self) -> bool {
        if *self == State::<JOINTS>::zeroed() {
            return true;
        }
        false
    }
}

impl Default for State<JOINTS> {
    fn default() -> Self {
        // WARN: default must depend on the robot model
        Self::zeroed()
    }
}

#[derive(Default, Debug, PartialEq)]
/// READ ONLY
pub struct History<const N: usize, const DEPTH: usize> {
    pub(in crate::state) buf: FixedCircularBuffer<State<N>, DEPTH>,
}
