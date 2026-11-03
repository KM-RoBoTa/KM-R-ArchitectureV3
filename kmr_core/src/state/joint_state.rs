// These are the types users do their control math in (`kmr_api` re-exports
// them as-is), so the arithmetic and `Default` have to be derived HERE: the
// orphan rule forbids `kmr_api` from adding operator impls to a core type.
// This is the only user-facing surface the core carries — keep it to that.
//
// Layout: each type is `#[repr(transparent)]` over its `f32`, so an array of
// them is an array of `f32` (FFI, SRS FR-10.2) and costs nothing to wrap.
use derive_more::{Add, Div, Mul, Sub};

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// The joint position (as radiants)
pub struct Q(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// The joint velocity
pub struct Qd(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// The joint torque
pub struct Tau(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// The position gain of an impedance (MIT) command. Ignored by actuators
/// without one.
pub struct Kp(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// The velocity gain of an impedance (MIT) command. Ignored by actuators
/// without one.
pub struct Kd(pub f32);
