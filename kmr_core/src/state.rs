use crate::error::StateError;

// todo: build time variable from model, not implemented yet — keep here, not in a submodule

/// The compile-time generated joint const. Generated from kmr_model. WIP
pub const JOINTS: usize = 4;
/// Amount of ticks the history of the state is kept for. WIP
pub const HISTORY_DEPTH: usize = 4;

// ── types: joint-space value newtypes ───────────────────────────────────────
mod joint_state;
pub use joint_state::{Q, Qd, Tau};

// ── history: read-only recorded state + its ring buffer ────────────────────
mod history;
pub use history::{History, State};

// ── desired: write-only staging area for the next setpoint ─────────────────
mod desired;
pub(crate) use desired::Desired;

// ── field: sealed routing from a value type to its storage slot ────────────
mod field;
pub use field::StateField;
pub(crate) use field::sealed;

// ── robot: public read/write handle combining desired + history ────────────
mod robot_state;
pub use robot_state::RobotState;

#[cfg(test)]
mod test;
