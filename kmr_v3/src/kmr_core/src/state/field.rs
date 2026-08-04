use super::JOINTS;
use super::desired::Desired;
use super::history::State;
use super::joint_state::{Q, Qd, Tau};

/// Routes a field element type to its storage slot in [`Desired`].
///
/// Lives in a private module so external crates can never name or implement it
/// — that *seals* [`StateField`] (you cannot add new field types from outside)
/// and keeps the private `Desired` out of the public API, even though
/// `StateField` itself bounds public methods.
pub(crate) mod sealed {
    use super::{Desired, JOINTS, State};

    pub(crate) trait Slot: Sized + Copy + From<f32> {
        fn slot(desired: &mut Desired) -> &mut [Self; JOINTS];
        fn field(state: &State<JOINTS>) -> &[Self; JOINTS];
    }
}

/// Marks a type that can be stored in [`super::RobotState`]'s desired state.
///
/// Public so `kmr_api` can name it as a bound, but sealed: the `sealed::Slot`
/// supertrait lives in a crate-private module, so no downstream crate can add a
/// new field type.
pub trait StateField: sealed::Slot {}

impl sealed::Slot for Q {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.q
    }
    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.q
    }
}
impl StateField for Q {}
impl From<f32> for Q {
    fn from(value: f32) -> Self {
        Q(value)
    }
}

impl sealed::Slot for Qd {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.qd
    }
    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.qd
    }
}
impl StateField for Qd {}
impl From<f32> for Qd {
    fn from(value: f32) -> Self {
        Qd(value)
    }
}

impl sealed::Slot for Tau {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.tau
    }

    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.tau
    }
}
impl StateField for Tau {}
impl From<f32> for Tau {
    fn from(value: f32) -> Self {
        Tau(value)
    }
}
