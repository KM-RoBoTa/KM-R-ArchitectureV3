// These are the types users do their control math in (`kmr_api` re-exports
// them as-is), so the arithmetic and `Default` have to be derived HERE: the
// orphan rule forbids `kmr_api` from adding operator impls to a core type.
// This is the only user-facing surface the core carries — keep it to that.
use derive_more::{Add, Div, Mul, Sub};

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
/// The joint position (as radiants)
pub struct Q(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
/// The joint velocity
pub struct Qd(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
/// The joint torque
pub struct Tau(pub f32);
