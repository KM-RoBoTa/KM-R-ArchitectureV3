//! `kmr_shutdown` and `kmr_emergency_stop` raise the signal on the real bus (kmr_ffi T018, SRS
//! FR-4.1 to FR-4.4): its own process, a file-local lock (as kmr_core/tests/run_signals.rs), a
//! run without hardware, and a `Last` controller counting its walks. Tick 3 is the fourth tick,
//! 0-based.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use kmr_api::{EachTick, Last, Sensors, Time, with};

kmr_api::robot!(XmBench);

/// The signal bus is process-global: every run takes this first.
static BUS: Mutex<()> = Mutex::new(());

fn bus() -> MutexGuard<'static, ()> {
    BUS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Counts the `Last` walks.
fn count(n: &mut Arc<AtomicU32>, _: &Time, _: &mut State, _: &Sensors) {
    n.fetch_add(1, Ordering::SeqCst);
}

/// The `Last` walks of a run whose two `EachTick` C controllers ask for `first`, then
/// `second`, at tick 3 (1 shutdown, 2 emergency stop, 0 nothing).
fn lasts(first: u32, second: u32) -> u32 {
    let _bus = bus();
    let n = Arc::new(AtomicU32::new(0));
    Robot::new()
        .add_controller(EachTick, kmr_api::c!(at_tick_3, with = first))
        .add_controller(EachTick, kmr_api::c!(at_tick_3, with = second))
        .add_controller(Last, with(Arc::clone(&n), count))
        .run()
        .expect("runs without hardware");
    n.load(Ordering::SeqCst)
}

#[test]
fn an_emergency_stop_is_immediate() {
    assert_eq!(lasts(2, 0), 3);
}

#[test]
fn a_shutdown_completes_the_tick() {
    assert_eq!(lasts(1, 0), 4);
}

#[test]
fn a_later_shutdown_does_not_replace_the_emergency_stop() {
    assert_eq!(lasts(2, 1), 3);
}

#[test]
fn a_later_emergency_stop_overrides_the_shutdown() {
    assert_eq!(lasts(1, 2), 3);
}
