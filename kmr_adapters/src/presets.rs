//! The robots KM-RoBoTa ships, named in `kmr_api::robot!` (SRS FR-6.1).
//!
//! Each preset here is the hardware half; the model half is `kmr_model`'s,
//! under the same name, and gives N. The table keeps its own length; the
//! count check of `robot!` keeps both in step.

mod rx28_chain;
mod xm_bench;

pub use rx28_chain::Rx28Chain;
pub use xm_bench::XmBench;
