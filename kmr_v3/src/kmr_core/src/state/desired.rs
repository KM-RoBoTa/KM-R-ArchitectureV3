use super::JOINTS;
use super::history::State;
use super::joint_state::{Q, Qd, Tau};

/// WRITE ONLY
#[derive(Debug, PartialEq)]
pub(crate) struct Desired {
    pub(in crate::state) q: [Q; JOINTS],
    pub(in crate::state) qd: [Qd; JOINTS],
    pub(in crate::state) tau: [Tau; JOINTS],
}

impl Default for Desired {
    fn default() -> Self {
        Self {
            q: State::default().q,
            qd: State::default().qd,
            tau: State::default().tau,
        }
    }
}
