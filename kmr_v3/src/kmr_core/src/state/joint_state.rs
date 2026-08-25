#[derive(Debug, PartialEq, Clone, Copy)]
/// The joint position (as radiants)
pub struct Q(pub f32);

#[derive(Debug, PartialEq, Clone, Copy)]
/// The joint velocity
pub struct Qd(pub f32);

#[derive(Debug, PartialEq, Clone, Copy)]
/// The joint torque
pub struct Tau(pub f32);
