//! The adapter layer: one implementation of [`kmr_core::hal::Hal`] per
//! vendor (SRS FR-5.1).
//!
//! The engine only sees actuator indices, masks and opaque groups. Buses,
//! protocols, models and raw units stay in here, and every value that crosses
//! the contract is an `f32` in SI units.

// Test code may unwrap: a panic there IS the failure report. Everywhere else
// an unwrap is a crash on the control path, so it stays forbidden.
#![cfg_attr(not(test), forbid(clippy::unwrap_used))]
#![warn(missing_docs)]

pub mod binding;
pub mod dynamixel;
pub mod presets;
