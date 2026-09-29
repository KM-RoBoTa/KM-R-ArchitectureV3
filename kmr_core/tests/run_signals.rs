//! End-to-end runs on the REAL signal bus, real clock and real sleep, through
//! the public surface only.
//!
//! The bus is process-global. Two things keep these tests from disturbing the
//! others, and each other:
//!
//! - this file is its own test binary, hence its own process: nothing here
//!   can meet the unit tests of the crate, `bus_semantics` included;
//! - inside the binary the tests run on parallel threads, so each of them
//!   takes [`BUS`] before it raises a signal or calls `run()`.
//!
//! None of them asserts on a measured duration. None of them can hang: every
//! controller that stops the loop raises on `>=` and asserts a tick budget, so
//! a loop that would ignore the signal fails within a fraction of a second.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use kmr_core::{EachTick, Last, Robot, RobotState, Sensors, Time, emergency_stop, shutdown};

static BUS: Mutex<()> = Mutex::new(());

/// A failed test poisons the lock. The guarded value is `()`, there is nothing
/// to find corrupted, so the next test goes on instead of failing in cascade.
fn bus() -> MutexGuard<'static, ()> {
    BUS.lock().unwrap_or_else(PoisonError::into_inner)
}

const DT: Duration = Duration::from_micros(200);
const STOP_AT: u32 = 3;
const TICK_BUDGET: u32 = 500;

fn tick(counter: &AtomicU32) -> u32 {
    let n = counter.fetch_add(1, Ordering::Relaxed) + 1;
    assert!(n < TICK_BUDGET, "the loop ignored the signal");
    n
}

#[test]
fn shutdown_ends_the_run_after_the_last_bucket() {
    static TICKS: AtomicU32 = AtomicU32::new(0);
    static LASTS: AtomicU32 = AtomicU32::new(0);

    fn stop(_t: &Time, _s: &mut RobotState, _se: &Sensors) {
        if tick(&TICKS) >= STOP_AT {
            shutdown!("test: enough ticks");
        }
    }
    fn last(_t: &Time, _s: &mut RobotState, _se: &Sensors) {
        tick(&LASTS);
    }

    let _bus = bus();
    let result = Robot::new()
        .set_dt(DT)
        .add_control_fn(EachTick, stop)
        .add_control_fn(Last, last)
        .run();

    assert_eq!(result, Ok(()));
    assert_eq!(TICKS.load(Ordering::Relaxed), STOP_AT);
    // Graceful: the tick that raised the shutdown ran its `Last` bucket.
    assert_eq!(LASTS.load(Ordering::Relaxed), STOP_AT);
}

#[test]
fn emergency_stop_ends_the_run_before_the_last_bucket() {
    static TICKS: AtomicU32 = AtomicU32::new(0);
    static LASTS: AtomicU32 = AtomicU32::new(0);

    fn stop(_t: &Time, _s: &mut RobotState, _se: &Sensors) {
        if tick(&TICKS) >= STOP_AT {
            emergency_stop!("test: enough ticks");
        }
    }
    fn last(_t: &Time, _s: &mut RobotState, _se: &Sensors) {
        tick(&LASTS);
    }

    let _bus = bus();
    let result = Robot::new()
        .set_dt(DT)
        .add_control_fn(EachTick, stop)
        .add_control_fn(Last, last)
        .run();

    assert_eq!(result, Ok(()));
    assert_eq!(TICKS.load(Ordering::Relaxed), STOP_AT);
    // Immediate: the tick that raised the stop never reached `Last`.
    assert_eq!(LASTS.load(Ordering::Relaxed), STOP_AT - 1);
}

#[test]
fn a_signal_left_on_the_bus_does_not_end_the_next_run() {
    static TICKS: AtomicU32 = AtomicU32::new(0);

    fn stop(_t: &Time, _s: &mut RobotState, _se: &Sensors) {
        if tick(&TICKS) >= STOP_AT {
            shutdown!("test: enough ticks");
        }
    }

    let _bus = bus();
    emergency_stop!("test: raised before the run");
    let result = Robot::new().set_dt(DT).add_control_fn(EachTick, stop).run();

    assert_eq!(result, Ok(()));
    // Without the reset at entry the run would have ended with zero ticks.
    assert_eq!(TICKS.load(Ordering::Relaxed), STOP_AT);
}

#[test]
fn two_runs_in_a_row_both_run() {
    static TICKS: AtomicU32 = AtomicU32::new(0);

    // The counter is shared by the two runs: the second one stops on its
    // first tick, which still proves that it ticked.
    fn stop(_t: &Time, _s: &mut RobotState, _se: &Sensors) {
        if tick(&TICKS) >= STOP_AT {
            shutdown!("test: enough ticks");
        }
    }

    let _bus = bus();
    for expected in [STOP_AT, STOP_AT + 1] {
        let result = Robot::new().set_dt(DT).add_control_fn(EachTick, stop).run();

        assert_eq!(result, Ok(()));
        assert_eq!(TICKS.load(Ordering::Relaxed), expected);
    }
}
