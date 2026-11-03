//! `tick_hw`: `tick` on the reference chain, one bus plugin (FR-004, FR-005, FR-015 (SRS FR-6.1,
//! FR-6.3, FR-5.3)).
//!
//! Assumed, pending human: D10 (XmBench, one U2D2); the one place with
//! PROCEDURE.md "Reference chain". One bus, so no bus label (D6).

use std::process::ExitCode;

use kmr_api::Bus;
use kmr_bench::alloc::Counting;
use kmr_bench::tick::{self, Bench};

kmr_api::robot!(XmBench);

#[global_allocator]
static GLOBAL: Counting = Counting;

/// The U2D2's serial id, from /dev/serial/by-id: the one line two benches differ in.
const SERIAL: &str = "FT8ISZ6H";

fn main() -> ExitCode {
    let mut buses = kmr_bench::buf::Buf::<64>::new();
    let _ = std::fmt::Write::write_fmt(&mut buses, format_args!("by_id:{SERIAL}"));
    tick::main::<__Spec>(&Bench {
        binary: "tick_hw",
        robot: "XmBench",
        robot_n: N,
        plugins: &[Bus::by_id(SERIAL)],
        buses: buses.as_str(),
    })
}
