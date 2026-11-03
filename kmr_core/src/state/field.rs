use super::JOINTS;
use super::desired::Desired;
use super::history::State;
use super::joint_state::{Kd, Kp, Q, Qd, Tau};
use crate::hal::{Mask, Written};

/// Routes a field element type to its storage slot in [`Desired`].
///
/// Lives in a private module so external crates can never name or implement it
/// — that *seals* [`StateField`] (you cannot add new field types from outside)
/// and keeps the private `Desired` out of the public API, even though
/// `StateField` itself bounds public methods.
pub(crate) mod sealed {
    use super::{Desired, JOINTS, State};
    use crate::hal::{Mask, Written};

    pub(crate) trait Slot: Sized + Copy + From<f32> {
        fn slot(desired: &mut Desired) -> &mut [Self; JOINTS];
        fn field(state: &State<JOINTS>) -> &[Self; JOINTS];
        /// The write mask of this field.
        fn mask(written: &mut Written) -> &mut Mask;
    }
}

/// Marks a type that can be stored in [`super::RobotState`]'s desired state.
///
/// Public because `kmr_api` re-exports it (as `Field`) for users to name as a
/// bound, but sealed: the `sealed::Slot` supertrait lives in a crate-private
/// module, so no downstream crate can add a new field type:
///
/// ```compile_fail
/// #[derive(Clone, Copy)]
/// struct Mine(f32);
/// impl From<f32> for Mine {
///     fn from(v: f32) -> Self {
///         Mine(v)
///     }
/// }
/// impl kmr_core::state::StateField for Mine {}
/// ```
// DECISION: the private supertrait IS the seal, so `private_bounds` is allowed
// here rather than widening `Slot` (its methods name the crate-private `Desired`).
#[allow(
    private_bounds,
    reason = "the crate-private supertrait seals the field set"
)]
pub trait StateField: sealed::Slot {}

impl sealed::Slot for Q {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.q
    }
    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.q
    }
    fn mask(written: &mut Written) -> &mut Mask {
        &mut written.q
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
    fn mask(written: &mut Written) -> &mut Mask {
        &mut written.qd
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

    fn mask(written: &mut Written) -> &mut Mask {
        &mut written.tau
    }
}
impl StateField for Tau {}
impl From<f32> for Tau {
    fn from(value: f32) -> Self {
        Tau(value)
    }
}

impl sealed::Slot for Kp {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.kp
    }

    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.kp
    }

    fn mask(written: &mut Written) -> &mut Mask {
        &mut written.kp
    }
}
impl StateField for Kp {}
impl From<f32> for Kp {
    fn from(value: f32) -> Self {
        Kp(value)
    }
}

impl sealed::Slot for Kd {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.kd
    }

    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.kd
    }

    fn mask(written: &mut Written) -> &mut Mask {
        &mut written.kd
    }
}
impl StateField for Kd {}
impl From<f32> for Kd {
    fn from(value: f32) -> Self {
        Kd(value)
    }
}
