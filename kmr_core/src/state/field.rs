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
    use super::{Desired, FieldId, State};
    use crate::hal::{Mask, Written};

    pub(crate) trait Slot: Sized + Copy + From<f32> {
        /// Which field this is, for the crates that dispatch on it.
        const ID: FieldId;
        /// The desired array of this field.
        fn slot<const N: usize>(desired: &mut Desired<N>) -> &mut [Self; N];
        /// The array of this field in one history slot.
        fn field<const N: usize>(state: &State<N>) -> &[Self; N];
        /// The write mask of this field.
        fn mask(written: &mut Written) -> &mut Mask;
    }
}

/// Names each field type, one variant per [`StateField`] implementor.
///
/// Plumbing for `kmr_api` and the FFI layer, not re-exported to users.
// DECISION: not `#[non_exhaustive]`: the FFI layer matches it exhaustively, so
// a new field breaks its build instead of falling into a wildcard arm.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FieldId {
    /// [`Q`]
    Q,
    /// [`Qd`]
    Qd,
    /// [`Tau`]
    Tau,
    /// [`Kp`]
    Kp,
    /// [`Kd`]
    Kd,
}

/// The [`FieldId`] of field type `T`.
#[doc(hidden)]
pub const fn field_id<T: StateField>() -> FieldId {
    <T as sealed::Slot>::ID
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
    const ID: FieldId = FieldId::Q;
    fn slot<const N: usize>(desired: &mut Desired<N>) -> &mut [Self; N] {
        &mut desired.q
    }
    fn field<const N: usize>(state: &State<N>) -> &[Self; N] {
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
impl From<Q> for f32 {
    fn from(value: Q) -> Self {
        value.0
    }
}

impl sealed::Slot for Qd {
    const ID: FieldId = FieldId::Qd;
    fn slot<const N: usize>(desired: &mut Desired<N>) -> &mut [Self; N] {
        &mut desired.qd
    }
    fn field<const N: usize>(state: &State<N>) -> &[Self; N] {
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
impl From<Qd> for f32 {
    fn from(value: Qd) -> Self {
        value.0
    }
}

impl sealed::Slot for Tau {
    const ID: FieldId = FieldId::Tau;
    fn slot<const N: usize>(desired: &mut Desired<N>) -> &mut [Self; N] {
        &mut desired.tau
    }

    fn field<const N: usize>(state: &State<N>) -> &[Self; N] {
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
impl From<Tau> for f32 {
    fn from(value: Tau) -> Self {
        value.0
    }
}

impl sealed::Slot for Kp {
    const ID: FieldId = FieldId::Kp;
    fn slot<const N: usize>(desired: &mut Desired<N>) -> &mut [Self; N] {
        &mut desired.kp
    }

    fn field<const N: usize>(state: &State<N>) -> &[Self; N] {
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
impl From<Kp> for f32 {
    fn from(value: Kp) -> Self {
        value.0
    }
}

impl sealed::Slot for Kd {
    const ID: FieldId = FieldId::Kd;
    fn slot<const N: usize>(desired: &mut Desired<N>) -> &mut [Self; N] {
        &mut desired.kd
    }

    fn field<const N: usize>(state: &State<N>) -> &[Self; N] {
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
impl From<Kd> for f32 {
    fn from(value: Kd) -> Self {
        value.0
    }
}
