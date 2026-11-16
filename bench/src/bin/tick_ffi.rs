//! `tick_ffi`: `tick` with an empty C controller, bench/ffi/empty.c, as its second `EachTick`
//! controller in place of the Rust no-op, so `results compare` against `tick` gives the cost of
//! one C controller per tick: the snapshot refilled, the call, the writes applied (FR-022 (SRS
//! NFR-11); kmr_ffi SC-003, its plan E3).

use std::process::ExitCode;

use kmr_api::{EachTick, First, Last};
use kmr_bench::alloc::Counting;
use kmr_bench::probe;
use kmr_bench::tick::{self, Bench};

kmr_api::robot!(XmBench);

#[global_allocator]
static GLOBAL: Counting = Counting;

fn main() -> ExitCode {
    let bench = Bench {
        binary: "tick_ffi",
        robot: "XmBench",
        robot_n: N,
        plugins: &[],
        buses: "none",
        declared: None,
    };
    let (robot, run) = match tick::begin::<N, __Spec>(&bench) {
        Ok(started) => started,
        Err(code) => return code,
    };
    let result = robot
        .add_controller(First, probe::start)
        .add_controller(First, probe::noop)
        .add_controller(EachTick, probe::work)
        .add_controller(EachTick, kmr_api::c!(empty))
        .add_controller(Last, probe::noop)
        .add_controller(Last, probe::end)
        .run();
    run.finish(&bench, "probe+ffi_empty/each", result)
}
