//! The kmr bench harness: black-box, it reaches the engine through `kmr_api` only.
//!
//! Probe controllers stamp each tick into `static` histograms; the figures
//! leave after `run()` returns, as one report per process.

#![deny(clippy::unwrap_used)]

pub mod alloc;
pub mod buf;
pub mod calib;
pub mod grid;
pub mod hist;
pub mod plan;
pub mod probe;
pub mod procfs;
pub mod report;
pub mod results;
pub mod setup;
