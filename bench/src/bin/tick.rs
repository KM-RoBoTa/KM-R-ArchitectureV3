//! `tick`: the simulated 1 kHz probe run, no bus plugin (FR-004, FR-006, FR-008 (SRS FR-6.1, FR-6.3,
//! NFR-8 b)).

use std::process::ExitCode;

use kmr_bench::alloc::Counting;
use kmr_bench::tick::{self, Bench};

kmr_api::robot!(XmBench);

#[global_allocator]
static GLOBAL: Counting = Counting;

fn main() -> ExitCode {
    tick::main::<N, __Spec>(&Bench {
        binary: "tick",
        robot: "XmBench",
        robot_n: N,
        plugins: &[],
        buses: "none",
    })
}
