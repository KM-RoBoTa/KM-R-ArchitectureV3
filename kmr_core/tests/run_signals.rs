//! End-to-end runs on the real bus, clock and sleep, through the public API.
//!
//! Own test binary, so the global bus is not shared with the unit tests.
//! Inside it, every test takes [`BUS`] first.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

use kmr_core::{EachTick, Last, Robot, RobotState, Sensors, Time, emergency_stop, shutdown};

static BUS: Mutex<()> = Mutex::new(());

/// Ignores poisoning: one failed test must not fail the others.
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
    assert_eq!(TICKS.load(Ordering::Relaxed), STOP_AT);
}

#[test]
fn two_runs_in_a_row_both_run() {
    static TICKS: AtomicU32 = AtomicU32::new(0);

    // Shared counter: the second run stops on its first tick.
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

#[test]
fn the_run_lasts_at_least_the_grid_points_it_slept_to() {
    static TICKS: AtomicU32 = AtomicU32::new(0);
    const SLOW_DT: Duration = Duration::from_millis(2);

    fn stop(_t: &Time, _s: &mut RobotState, _se: &Sensors) {
        if tick(&TICKS) >= STOP_AT {
            shutdown!("test: enough ticks");
        }
    }

    let _bus = bus();
    let before = Instant::now();
    let result = Robot::new()
        .set_dt(SLOW_DT)
        .add_control_fn(EachTick, stop)
        .run();
    let lasted = before.elapsed();

    assert_eq!(result, Ok(()));
    assert_eq!(TICKS.load(Ordering::Relaxed), STOP_AT);
    // Lower bound only: every tick but the exit one sleeps a slot.
    assert!(lasted >= SLOW_DT * (STOP_AT - 1), "lasted {lasted:?}");
}
