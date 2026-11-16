//! A C controller's request raises the signal on the real bus after its call: its own process,
//! a file-local lock (as kmr_core/tests/run_signals.rs). Rust `extern "C"` functions stand in
//! for C, writing the request as `kmr_shutdown` and `kmr_emergency_stop` do.
#![cfg(feature = "ffi")]

use core::ffi::c_void;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use kmr_api::ffi::c::{__Sensors, __State, __Time, CController};
use kmr_api::{EachTick, Last, Sensors, Time, with};

kmr_api::robot!(XmBench);

/// The signal bus is process-global: every run takes this first.
static BUS: Mutex<()> = Mutex::new(());

fn bus() -> MutexGuard<'static, ()> {
    BUS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Asks for the request in `ctx` (a `u32`: 1 shutdown, 2 emergency stop) at tick 3.
unsafe extern "C" fn at_tick_3(
    time: *const __Time,
    state: *mut __State<N>,
    _: *const __Sensors,
    ctx: *mut c_void,
) {
    // SAFETY: `invoke` passes live handles; these tests give a `u32` context.
    let (time, state, level) = unsafe { (&*time, &mut *state, *ctx.cast::<u32>()) };
    if time.tick == 3 && level > state.signal {
        state.signal = level;
        state.reason[..4].copy_from_slice(b"test");
    }
}

/// Counts the `Last` walks.
fn count(n: &mut Arc<AtomicU32>, _: &Time, _: &mut State, _: &Sensors) {
    n.fetch_add(1, Ordering::SeqCst);
}

/// The `Last` walks of a run whose two `EachTick` C controllers ask for `first`, then
/// `second`, at tick 3 (0 asks nothing).
fn lasts(first: u32, second: u32) -> u32 {
    let _bus = bus();
    let n = Arc::new(AtomicU32::new(0));
    Robot::new()
        .add_controller(
            EachTick,
            CController::__new("first", at_tick_3, Some(first)),
        )
        .add_controller(
            EachTick,
            CController::__new("second", at_tick_3, Some(second)),
        )
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
