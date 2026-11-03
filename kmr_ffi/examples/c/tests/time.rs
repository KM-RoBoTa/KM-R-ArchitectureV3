//! The clock as C reads it equals the `Time` getters (kmr_ffi T026, SC-006, SRS FR-3.3): over
//! 10^8 scripted ticks (run by a `--release` test, ignored in a debug one), over 10^4, and over ticks
//! that overrun.

use core::cell::Cell;
use std::time::{Duration, Instant};

use kmr_api::ffi::{TimeView, call};
use kmr_api::test_util::{
    DT, Host, Log, PER_TICK, Recorder, Scripted, TICK_END, Ticking, run_scripted,
};
use kmr_api::{EachTick, HCons, HNil, Sensors, Time};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn copy_time; }

/// A duration in nanoseconds.
fn ns(d: Duration) -> u64 {
    u64::try_from(d.as_nanos()).unwrap_or(u64::MAX)
}

/// The clock as the `Time` getters give it.
fn getters(time: &Time) -> TimeView {
    TimeView {
        tick: time.tick(),
        dt_ns: ns(time.dt()),
        elapsed_ns: ns(time.elapsed()),
        overrun_count: time.overrun_count(),
        last_overrun_ns: time.last_overrun().map_or(0, ns),
        accumulated_overrun_ns: ns(time.accumulated_overrun()),
    }
}

/// What a run saw: ticks C copied, ticks that differed from the getters, the last copy.
#[derive(Default)]
struct Seen {
    calls: Cell<u64>,
    differ: Cell<u64>,
    last: Cell<TimeView>,
    overruns: Cell<u64>,
}

/// Runs the C copy each tick on `host` and `drivers`.
fn run<P: Host, D: kmr_api::test_util::Drivers<N, Error: Send + Sync>>(
    host: &mut P,
    drivers: &mut D,
) -> Seen {
    let seen = Seen::default();
    let robot = Robot::new().set_dt(DT).add_controller_with(
        EachTick,
        TimeView::default(),
        |out: &mut TimeView, time: &Time, state: &mut State, _: &Sensors| {
            assert!(call(copy_time, time, state, Some(out)).is_ok());
            seen.calls.set(seen.calls.get() + 1);
            if *out != getters(time) {
                seen.differ.set(seen.differ.get() + 1);
            }
            if out.overrun_count > 0 {
                seen.overruns.set(seen.overruns.get() + 1);
            }
            seen.last.set(*out);
        },
    );
    run_scripted(robot, host, drivers).expect("runs");
    seen
}

/// Runs `ticks` on-deadline ticks without hardware and checks every copy.
fn ticks(ticks: u64) {
    let mut drivers = HCons {
        head: kmr_api::NoHardware,
        tail: HNil,
    };
    let seen = run(&mut Ticking::new(Instant::now(), ticks), &mut drivers);
    assert_eq!(seen.calls.get(), ticks);
    assert_eq!(seen.differ.get(), 0);
    let last = seen.last.get();
    assert_eq!(last.tick, ticks - 1);
    assert_eq!(last.dt_ns, 1_000_000);
    // Elapsed is stamped at the tick's start, which the host places half a dt in.
    assert_eq!(last.elapsed_ns / 1_000_000, ticks - 1);
}

#[test]
fn c_reads_the_clock_exactly_over_ten_thousand_ticks() {
    ticks(10_000);
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "10^8 ticks take minutes in a debug build; about 7 s with `cargo test --release -p kmr_ffi_example_c --test time`"
)]
fn c_reads_the_clock_exactly_over_a_hundred_million_ticks() {
    ticks(100_000_000);
}

#[test]
fn c_reads_the_overrun_counters_the_getters_give() {
    // `now` at arm, then at the start and the end of each tick: tick 0 ends at 1.5 dt.
    let mut host = Scripted::new(Instant::now())
        .script(&[0, 0, 15, 20, 25, 30, 47, 50, 55])
        .shutdown_from(TICK_END + 3 * PER_TICK);
    let log = Log::new();
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    let seen = run(&mut host, &mut drivers);
    assert_eq!(seen.calls.get(), 4);
    assert_eq!(seen.differ.get(), 0);
    assert!(seen.overruns.get() > 0);
    assert!(seen.last.get().accumulated_overrun_ns > 0);
}
