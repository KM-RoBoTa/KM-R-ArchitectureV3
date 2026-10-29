//! The robots KM-RoBoTa ships, named in `kmr_api::robot!` (SRS FR-6.1).
//!
//! Each preset here is the hardware half; the model half is `kmr_model`'s,
//! under the same name.

mod xm_bench;

pub use xm_bench::XmBench;

/// What a preset gives the engine at compile time.
pub trait Preset {
    /// Actuator count, until `robot!` takes it from `kmr_model`.
    const N: usize;
}
