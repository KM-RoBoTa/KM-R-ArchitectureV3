//! SC-005 (SRS NFR-1): 10 000 ticks of the loop on a scripted host and a
//! counting adapter allocate nothing and log nothing. Own process: the
//! counting allocator and the counting subscriber are process-global.
#![cfg(feature = "test-util")]

use std::alloc::{GlobalAlloc, Layout, System};
use std::num::NonZeroU32;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::time::{Duration, Instant};

use kmr_core::hal::{Batch, Claim, Command, Faults, Group, Hal, Mask, Overrides, Sample, seal};
use kmr_core::state::Q;
use kmr_core::test_util::{Ticking, is_emergency_stop, run_scripted};
use kmr_core::{EachTick, First, HCons, HNil, Last, Robot, RobotState, Sensors, Time};

/// Actuators of the test robot.
const N: usize = 4;

/// Ticks of the run.
const TICKS: u64 = 10_000;

/// Whether the counted window is open: from `First` of tick 0 to `First` of the last tick.
static WINDOW: AtomicBool = AtomicBool::new(false);
/// Heap calls inside the window.
static ALLOCS: AtomicU64 = AtomicU64::new(0);
/// Log events inside the window.
static EVENTS: AtomicU64 = AtomicU64::new(0);

/// [`System`] plus a count of the calls made inside the window.
struct Counting;

impl Counting {
    /// Counts one call when the window is open.
    fn count() {
        if WINDOW.load(Relaxed) {
            ALLOCS.fetch_add(1, Relaxed);
        }
    }
}

// SAFETY: every method forwards its arguments unchanged to `System`, which upholds the
// `GlobalAlloc` contract; the count is two atomics and allocates nothing.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        Self::count();
        // SAFETY: the caller's contract for `alloc` is passed on as is.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        Self::count();
        // SAFETY: the caller's contract for `alloc_zeroed` is passed on as is.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        Self::count();
        // SAFETY: `ptr` came from this allocator, which is `System`; the caller's contract holds.
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` came from this allocator, which is `System`; the caller's contract holds.
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// A subscriber that enables every level and counts the events inside the window.
struct Events;

impl tracing::Subscriber for Events {
    fn register_callsite(
        &self,
        _metadata: &'static tracing::Metadata<'static>,
    ) -> tracing::subscriber::Interest {
        tracing::subscriber::Interest::always()
    }

    fn enabled(&self, _metadata: &tracing::Metadata<'_>) -> bool {
        true
    }

    fn event(&self, _event: &tracing::Event<'_>) {
        if WINDOW.load(Relaxed) {
            EVENTS.fetch_add(1, Relaxed);
        }
    }

    fn new_span(&self, _span: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }

    fn record(&self, _span: &tracing::span::Id, _values: &tracing::span::Record<'_>) {}

    fn record_follows_from(&self, _span: &tracing::span::Id, _follows: &tracing::span::Id) {}

    fn enter(&self, _span: &tracing::span::Id) {}

    fn exit(&self, _span: &tracing::span::Id) {}
}

/// An adapter that reads every actuator each tick and counts the writes; no log.
struct Quiet {
    writes: u64,
}

impl seal::Adapter for Quiet {}

impl Hal<N> for Quiet {
    type Error = core::convert::Infallible;

    fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, Self::Error> {
        Ok(Claim {
            owned: Mask::ALL,
            group: [Group(0); N],
            read_every: [NonZeroU32::MIN; N],
            read_cost: Duration::ZERO,
            write_cost: Duration::ZERO,
        })
    }

    fn open(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn zero(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn ping(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn health(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn configure(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read_all(&mut self, sample: Sample<'_, N>) -> Result<(), Self::Error> {
        *sample.q = [Q(0.0); N];
        Ok(())
    }

    fn torque_on(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
        for q in sample.q.iter_mut() {
            *q = *q + Q(0.001);
        }
        Mask::ALL
    }

    fn write(&mut self, _command: Command<'_, N>, _batch: &Batch<'_, N>, _faults: &mut Faults<N>) {
        self.writes += 1;
    }

    fn hold(&mut self, _target: Command<'_, N>) {}

    fn torque_off(&mut self) {}

    fn close(&mut self) {}
}

/// Opens the window on tick 0, closes it on the last tick.
fn window(time: &Time, _state: &mut RobotState<N>, _sensors: &Sensors) {
    WINDOW.store(time.tick() + 1 < TICKS, Relaxed);
}

/// Reads the history and writes every actuator, as a controller does.
fn follow(_time: &Time, state: &mut RobotState<N>, _sensors: &Sensors) {
    if let Ok(q) = state.prev::<Q>(1) {
        let q = *q;
        state.set_all(q);
    }
}

#[test]
fn ten_thousand_ticks_allocate_nothing_and_log_nothing() {
    tracing::subscriber::set_global_default(Events).expect("the first subscriber");
    let mut robot = Robot::<N>::new()
        .set_dt(kmr_core::test_util::DT)
        .add_control_fn(First, window)
        .add_control_fn(EachTick, follow)
        .add_control_fn(Last, follow);
    let mut host = Ticking::new(Instant::now(), TICKS);
    let mut drivers = HCons {
        head: Quiet { writes: 0 },
        tail: HNil,
    };

    let signal = run_scripted(&mut robot, &mut host, &mut drivers).expect("Quiet never fails");
    WINDOW.store(false, Relaxed);

    assert!(!is_emergency_stop(&signal));
    assert_eq!(host.slept, TICKS);
    assert_eq!(drivers.head.writes, TICKS);
    assert_eq!(ALLOCS.load(Relaxed), 0, "heap calls in {TICKS} ticks");
    assert_eq!(EVENTS.load(Relaxed), 0, "log events in {TICKS} ticks");
}
