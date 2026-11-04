//! A C body's status raises the signal on the real bus (kmr_ffi T018, SRS FR-4.1 to FR-4.4):
//! its own process, a file-local lock (as kmr_core/tests/run_signals.rs), a run without
//! hardware, and a `Last` controller counting its walks. Tick 3 is the fourth tick, 0-based.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use kmr_api::ffi::call;
use kmr_api::{EachTick, Last, Sensors, Time};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn at_tick_3; }

/// The signal bus is process-global: every run takes this first.
static BUS: Mutex<()> = Mutex::new(());

fn bus() -> MutexGuard<'static, ()> {
    BUS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Forwards the tick to `at_tick_3`, which returns `code` at tick 3.
fn forward(code: &mut i32, time: &Time, state: &mut State, _: &Sensors) {
    let _ = call(at_tick_3, time, state, Some(code));
}

/// Counts the `Last` walks.
fn count(n: &mut Arc<AtomicU32>, _: &Time, _: &mut State, _: &Sensors) {
    n.fetch_add(1, Ordering::SeqCst);
}

/// The `Last` walks of a run whose two `EachTick` C forwarders return `first`, then `second`,
/// at tick 3.
fn lasts(first: i32, second: i32) -> u32 {
    let _bus = bus();
    let n = Arc::new(AtomicU32::new(0));
    Robot::new()
        .add_controller_with(EachTick, first, forward)
        .add_controller_with(EachTick, second, forward)
        .add_controller_with(Last, Arc::clone(&n), count)
        .run()
        .expect("runs without hardware");
    n.load(Ordering::SeqCst)
}

#[test]
fn returning_2_stops_at_once() {
    assert_eq!(lasts(2, 0), 3, "tick 3 never reached Last");
}

#[test]
fn returning_1_completes_the_tick() {
    assert_eq!(lasts(1, 0), 4, "tick 3 reached Last, no tick after");
}

#[test]
fn a_later_shutdown_does_not_replace_the_emergency_stop() {
    assert_eq!(lasts(2, 1), 3);
}
