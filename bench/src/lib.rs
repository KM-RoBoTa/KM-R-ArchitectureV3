//! The kmr bench harness: black-box, it reaches the engine through `kmr_api` only.
//!
//! Probe controllers stamp each tick into `static` histograms; the figures
//! leave after `run()` returns, as one report per process.

#![deny(clippy::unwrap_used)]
