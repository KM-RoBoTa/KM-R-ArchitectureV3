//! The robot state: value types, write-only desired values, read-only history.

use crate::error::StateError;

/// Actuators of the robot the core's own tests drive. The actuator count is
/// a const generic everywhere else (decision 0007).
#[cfg(test)]
pub(crate) const N: usize = 4;
/// Amount of ticks the history of the state is kept for. WIP
pub const HISTORY_DEPTH: usize = 4;

// ── types: joint-space value newtypes ───────────────────────────────────────
mod joint_state;
pub use joint_state::{Kd, Kp, Q, Qd, Tau};

// ── history: read-only recorded state + its ring buffer ────────────────────
mod history;
pub(crate) use history::{History, State};

// ── desired: write-only staging area for the next setpoint ─────────────────
mod desired;
pub(crate) use desired::Desired;

// ── field: sealed routing from a value type to its storage slot ────────────
mod field;
pub use field::StateField;
pub(crate) use field::sealed;
#[doc(hidden)]
pub use field::{FieldId, field_id};

// ── robot: public read/write handle combining desired + history ────────────
mod robot_state;
pub use robot_state::RobotState;

#[cfg(test)]
mod test;
