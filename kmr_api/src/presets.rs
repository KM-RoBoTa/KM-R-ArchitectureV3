//! The presets kmr ships: the robots KM-RoBoTa ships, for [`robot!`](crate::robot!) (SRS
//! FR-6.1), and [`home`], a controller preset (SRS FR-2.7).

pub use kmr_adapters::presets::*;

pub mod home;
