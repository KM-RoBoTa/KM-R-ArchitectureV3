use circular_buffer::FixedCircularBuffer;

use super::JOINTS;
use super::joint_state::{Q, Qd, Tau};

#[derive(Debug, PartialEq)]
pub struct State<const N: usize> {
    pub q: [Q; N],
    pub qd: [Qd; N],
    pub tau: [Tau; N],
}

impl State<JOINTS> {
    fn home_state() -> Self {
        todo!(
            "I must save somewhere permanant the home state. This must be the default because zeroing all actuators as default is dangerous."
        )
    }
}

impl Default for State<JOINTS> {
    fn default() -> Self {
        // WARN: default must depend on the robot model
        Self::home_state()
    }
}

#[derive(Default, Debug, PartialEq)]
/// READ ONY
pub struct History<const N: usize, const DEPTH: usize> {
    pub(in crate::state) buf: FixedCircularBuffer<State<N>, DEPTH>,
}
