//! The C call allocates nothing and takes no lock (SC-003 kmr side, SC-007 C part (SRS NFR-1,
//! NFR-2)). kmr_api's fill and apply around it are measured in kmr_ffi/examples/c.

use core::cell::Cell;
use core::ffi::c_void;
use std::alloc::{GlobalAlloc, Layout, System};

use kmr_ffi::{CFn, CSensors, CState, CTime, invoke};

/// The C signature at N = 64 and the core's sizes today.
type Swing = CFn<64, 4, 4, 1, 1024>;

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

/// A controller that stages every actuator's `q` from the newest slot and counts its calls in
/// a `u64` context.
unsafe extern "C" fn swing(
    _: *const CTime,
    state: *mut CState<64, 4>,
    _: *const CSensors<4, 1, 1024>,
    ctx: *mut c_void,
) {
    // SAFETY: `invoke` passes a live exclusive state and the test an exclusive `u64` context.
    let (s, n) = unsafe { (&mut *state, &mut *ctx.cast::<u64>()) };
    for (q, prev) in s.write.q.iter_mut().zip(s.slot[0].q) {
        *q = prev + 0.1;
    }
    s.write.written.q = u64::MAX;
    *n += 1;
}

#[test]
fn ten_thousand_calls_at_n_64_allocate_nothing() {
    let time = CTime::default();
    let sensors = CSensors::default();
    let mut state = CState::<64, 4>::default();
    let mut n = 0u64;
    let f: Swing = swing;
    let before = ALLOCS.with(Cell::get);
    for _ in 0..10_000 {
        invoke(f, &time, &mut state, &sensors, Some(&mut n));
    }
    let after = ALLOCS.with(Cell::get);
    assert_eq!(n, 10_000);
    assert_eq!(after - before, 0);
}

#[test]
fn the_call_path_names_no_lock() {
    for src in [include_str!("../src/c.rs"), include_str!("../src/view.rs")] {
        assert!(!src.contains(concat!("std", "::sync")));
        assert!(!src.contains("Mutex"));
    }
}

#[test]
fn the_counter_sees_an_allocation() {
    let before = ALLOCS.with(Cell::get);
    let b = std::hint::black_box(Box::new(7u64));
    assert_eq!(*b, 7);
    assert!(ALLOCS.with(Cell::get) > before);
}
