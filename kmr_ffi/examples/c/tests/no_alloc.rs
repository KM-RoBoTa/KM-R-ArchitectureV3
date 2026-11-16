//! A C controller's whole path allocates nothing once running: the snapshot refilled, the C
//! call, the writes applied (SC-003, SC-007 C part (SRS NFR-1, NFR-2)). Counted on the loop's
//! thread over a thousand scripted ticks of `twin`, with a recorder and a fake microphone.

use core::cell::Cell;
use std::alloc::{GlobalAlloc, Layout, System};
use std::time::Instant;

use kmr_api::test_util::{DT, FakeSensor, Log, Reading, Recorder, Ticking, run_scripted_all};
use kmr_api::{EachTick, First, HCons, HNil, Last, Sensors, Time};

kmr_api::robot!(XmBench);

/// [`System`], counting the allocations of the current thread.
struct Counting;

thread_local! {
    /// Allocations made on this thread.
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

/// Counts one allocation on this thread; a thread being torn down is not counted.
fn bump() {
    let _ = ALLOCS.try_with(|n| n.set(n.get() + 1));
}

// SAFETY: every method forwards to `System` with the caller's arguments unchanged, so
// `System`'s guarantees hold; the counter is a const-initialised thread-local `Cell`, which
// never allocates.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        bump();
        // SAFETY: forwarded as is; the caller upholds `GlobalAlloc::alloc`'s contract.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        bump();
        // SAFETY: forwarded as is; the caller upholds `GlobalAlloc::alloc_zeroed`'s contract.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        bump();
        // SAFETY: forwarded as is; the caller upholds `GlobalAlloc::realloc`'s contract.
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: forwarded as is; the caller upholds `GlobalAlloc::dealloc`'s contract.
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

#[test]
fn a_thousand_ticks_of_a_c_controller_allocate_nothing() {
    let log = Log::new();
    let audio = [3_i16, -2000, 40];
    let readings = [Reading {
        audio: &audio,
        ..Reading::QUIET
    }; 1200];
    let at_10 = Cell::new(0);
    let at_1010 = Cell::new(0);
    let robot = Robot::new()
        .set_dt(DT)
        .add_sensor(FakeSensor {
            script: &readings,
            mics: 1,
            ..FakeSensor::new(&log, 0)
        })
        .add_controller(First, |time: &Time, _: &mut State, _: &Sensors| {
            if time.tick() == 10 {
                at_10.set(ALLOCS.with(Cell::get));
            }
        })
        .add_controller(EachTick, kmr_api::c!(twin))
        .add_controller(Last, |time: &Time, _: &mut State, _: &Sensors| {
            if time.tick() == 1010 {
                at_1010.set(ALLOCS.with(Cell::get));
            }
        });
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted_all(robot, &mut Ticking::new(Instant::now(), 1100), &mut drivers).expect("runs");
    assert!(log.write.get().is_some(), "twin wrote");
    assert!(at_1010.get() > 0, "the counter ran");
    assert_eq!(at_1010.get() - at_10.get(), 0);
}
