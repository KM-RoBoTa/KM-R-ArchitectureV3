use super::JOINTS;
use super::history::State;
use super::joint_state::{Q, Qd, Tau};
use crate::hal::Written;

/// WRITE ONLY
#[derive(Debug, PartialEq)]
pub(crate) struct Desired {
    pub(in crate::state) q: [Q; JOINTS],
    pub(in crate::state) qd: [Qd; JOINTS],
    pub(in crate::state) tau: [Tau; JOINTS],
    /// Values written since the last apply, one mask per field, apart from
    /// the arrays (SRS FR-2.5, NFR-10).
    pub(in crate::state) written: Written,
}

impl Default for Desired {
    fn default() -> Self {
        Self {
            q: State::default().q,
            qd: State::default().qd,
            tau: State::default().tau,
            written: Written::default(),
        }
    }
}
