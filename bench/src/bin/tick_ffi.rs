//! `tick_ffi`: `tick` with an empty C function, bench/ffi/empty.c, as its second `EachTick`
//! controller in place of the Rust no-op, so `results compare` against `tick` gives the cost of
//! one foreign call per tick (FR-022 (SRS NFR-11); kmr_ffi SC-003, its plan E3).

use std::process::ExitCode;

use kmr_api::{Sensors, Time};
use kmr_bench::alloc::Counting;
use kmr_bench::tick::{self, Bench};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn empty; }

#[global_allocator]
static GLOBAL: Counting = Counting;

/// Hands the tick to `empty`; a foreign error stops the robot.
fn forward(time: &Time, state: &mut State, _: &Sensors) {
    if kmr_api::ffi::call(empty, time, state, None::<&mut ()>).is_err() {
        kmr_api::emergency_stop!("the empty C body failed");
    }
}

fn main() -> ExitCode {
    tick::main_with::<N, __Spec, _>(
        &Bench {
            binary: "tick_ffi",
            robot: "XmBench",
            robot_n: N,
            plugins: &[],
            buses: "none",
            declared: None,
        },
        "probe+ffi_empty/each",
        forward,
    )
}
