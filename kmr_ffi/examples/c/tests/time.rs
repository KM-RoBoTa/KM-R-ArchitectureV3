//! The clock as C reads it equals the `Time` getters bit for bit, durations as
//! `Duration::as_secs_f64` gives them (kmr_ffi T026, SC-006, SRS FR-3.3): over 10^8 scripted
//! ticks (run by a `--release` test, ignored in a debug one), over 10^4, and over ticks that
//! overrun.

use core::cell::Cell;
use std::time::Instant;

use kmr_api::test_util::{
    DT, Host, Log, PER_TICK, Recorder, Scripted, TICK_END, Ticking, run_scripted,
};
use kmr_api::{EachTick, HCons, HNil, Sensors, Time};

kmr_api::robot!(XmBench);

/// `time_out` of tests/time.c.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct TimeOut {
    tick: u64,
    overrun_count: u64,
    elapsed: f64,
    dt: f64,
    last_overrun: f64,
    accumulated_overrun: f64,
    late: u32,
    reserved: u32,
}

impl TimeOut {
    /// The copy with each duration as its bits, so the comparison is exact.
    fn bits(&self) -> [u64; 7] {
        [
            self.tick,
            self.overrun_count,
            self.elapsed.to_bits(),
            self.dt.to_bits(),
            self.last_overrun.to_bits(),
            self.accumulated_overrun.to_bits(),
            u64::from(self.late),
        ]
    }
}

/// The clock as the `Time` getters give it.
fn getters(time: &Time) -> TimeOut {
    let last = time.last_overrun();
    TimeOut {
        tick: time.tick(),
        overrun_count: time.overrun_count(),
        elapsed: time.elapsed().as_secs_f64(),
        dt: time.dt().as_secs_f64(),
        last_overrun: last.map_or(0.0, |d| d.as_secs_f64()),
        accumulated_overrun: time.accumulated_overrun().as_secs_f64(),
        late: u32::from(last.is_some()),
        reserved: 0,
    }
}

/// What a run saw: ticks C copied, ticks that differed from the getters, the last copy.
#[derive(Default)]
struct Seen {
    calls: Cell<u64>,
    differ: Cell<u64>,
    last: Cell<TimeOut>,
    overruns: Cell<u64>,
}

/// Runs the C copy each tick on `host` and `drivers`, then checks it against the getters.
fn run<P: Host, D: kmr_api::test_util::Drivers<N, Error: Send + Sync>>(
    host: &mut P,
    drivers: &mut D,
) -> Seen {
    let seen = Seen::default();
    // C writes through the reference it is handed a pointer to: interior mutability.
    let copy = Cell::new(TimeOut::default());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(EachTick, kmr_api::c!(copy_time, with = &copy))
        .add_controller(EachTick, |time: &Time, _: &mut State, _: &Sensors| {
            let out = copy.get();
            seen.calls.set(seen.calls.get() + 1);
            if out.bits() != getters(time).bits() {
                seen.differ.set(seen.differ.get() + 1);
            }
            if out.overrun_count > 0 {
                seen.overruns.set(seen.overruns.get() + 1);
            }
            seen.last.set(out);
        });
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
    assert_eq!(last.dt, 0.001);
    // Elapsed is stamped at the tick's start, which the host places half a dt in.
    assert_eq!((last.elapsed * 1000.0) as u64, ticks - 1);
}

#[test]
fn c_reads_the_clock_exactly_over_ten_thousand_ticks() {
    ticks(10_000);
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "10^8 ticks take minutes in a debug build; seconds with `cargo test --release -p kmr_ffi_example_c --test time`"
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
    assert!(seen.last.get().accumulated_overrun > 0.0);
}
