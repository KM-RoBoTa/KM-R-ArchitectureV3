//! `ffi::call` raises the signal of a foreign status on the real bus: its own process, a
//! file-local lock (as kmr_core/tests/run_signals.rs). Rust doubles stand in for the foreign body.
#![cfg(feature = "ffi")]

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use kmr_api::ffi::{Foreign, ForeignError, ReadView, Status, TimeView, WriteBuf, call};
use kmr_api::{EachTick, HISTORY_DEPTH, Last, Sensors, Time};

kmr_api::robot!(XmBench);

/// The signal bus is process-global: every run takes this first.
static BUS: Mutex<()> = Mutex::new(());

fn bus() -> MutexGuard<'static, ()> {
    BUS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A foreign body that returns `status` at tick 3 and continues before.
struct At3(Status);

impl Foreign<N, HISTORY_DEPTH> for At3 {
    fn invoke<C>(
        &mut self,
        _read: &ReadView<N, HISTORY_DEPTH>,
        _buf: &mut WriteBuf<N>,
        time: &TimeView,
        _ctx: Option<&mut C>,
    ) -> Result<Status, ForeignError> {
        Ok(if time.tick == 3 {
            self.0
        } else {
            Status::Continue
        })
    }
}

/// Forwards the tick to `At3(status)`.
fn forward(status: &mut Status, time: &Time, state: &mut State, _: &Sensors) {
    let _ = call(At3(*status), time, state, None::<&mut ()>);
}

/// Counts the `Last` walks.
fn count(n: &mut Arc<AtomicU32>, _: &Time, _: &mut State, _: &Sensors) {
    n.fetch_add(1, Ordering::SeqCst);
}

/// The `Last` walks of a run whose `EachTick` forwarders return `first`, then `second`, at
/// tick 3.
fn lasts(first: Status, second: Status) -> u32 {
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
fn an_emergency_stop_is_immediate() {
    assert_eq!(lasts(Status::EmergencyStop, Status::Continue), 3);
}

#[test]
fn a_shutdown_completes_the_tick() {
    assert_eq!(lasts(Status::Shutdown, Status::Continue), 4);
}

#[test]
fn a_later_shutdown_does_not_replace_the_emergency_stop() {
    assert_eq!(lasts(Status::EmergencyStop, Status::Shutdown), 3);
}
