use super::history::State;
use super::joint_state::{Kd, Kp, Q, Qd, Tau};
use crate::hal::Written;

/// WRITE ONLY
#[derive(Debug, PartialEq)]
pub(crate) struct Desired<const N: usize> {
    pub(in crate::state) q: [Q; N],
    pub(in crate::state) qd: [Qd; N],
    pub(in crate::state) tau: [Tau; N],
    pub(in crate::state) kp: [Kp; N],
    pub(in crate::state) kd: [Kd; N],
    /// Values written since the last apply, one mask per field, apart from
    /// the arrays (SRS FR-2.5, NFR-10).
    pub(in crate::state) written: Written,
}

impl<const N: usize> Default for Desired<N> {
    fn default() -> Self {
        Self {
            q: State::<N>::default().q,
            qd: State::<N>::default().qd,
            tau: State::<N>::default().tau,
            kp: State::<N>::default().kp,
            kd: State::<N>::default().kd,
            written: Written::default(),
        }
    }
}
