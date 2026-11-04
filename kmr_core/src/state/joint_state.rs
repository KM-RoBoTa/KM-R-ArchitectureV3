// These are the types users do their control math in (`kmr_api` re-exports
// them as-is), so the arithmetic and `Default` have to be derived HERE: the
// orphan rule forbids `kmr_api` from adding operator impls to a core type.
// This is the only user-facing surface the core carries — keep it to that.
//
// Units (SRS FR-2.2): revolute actuators in rad, rad/s, N·m; prismatic ones
// in mm, mm/s, N. `From` converts both ways with a bare `f32`.
//
// Layout: each type is `#[repr(transparent)]` over its `f32`, so an array of
// them is an array of `f32` (FFI, SRS FR-10.2) and costs nothing to wrap.
use derive_more::{Add, Div, Mul, Sub};

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// Actuator position: rad on a revolute actuator, mm on a prismatic one.
pub struct Q(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// Actuator velocity: rad/s on a revolute actuator, mm/s on a prismatic one.
pub struct Qd(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// Actuator effort: torque in N·m on a revolute actuator, force in N on a
/// prismatic one.
pub struct Tau(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// The position gain of an impedance (MIT) command: N·m/rad on a revolute
/// actuator, N/mm on a prismatic one. Ignored by actuators without one.
pub struct Kp(pub f32);

#[derive(Debug, Default, PartialEq, Clone, Copy, Add, Sub, Mul, Div)]
#[repr(transparent)]
/// The velocity gain of an impedance (MIT) command: N·m·s/rad on a revolute
/// actuator, N·s/mm on a prismatic one. Ignored by actuators without one.
pub struct Kd(pub f32);
