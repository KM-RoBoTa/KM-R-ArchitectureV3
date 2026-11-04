//! Counting allocator: heap calls on the loop thread inside the window (research R7; FR-012 (SRS NFR-1)).
//!
//! A binary installs [`Counting`] as its `#[global_allocator]`. Every call goes
//! to [`System`] unchanged; an allocation or reallocation made while the
//! calling thread's window is open adds one to [`allocs`].

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

thread_local! {
    /// Whether this thread's window is open: `const`-initialized, no destructor, no allocation.
    static WINDOW: Cell<bool> = const { Cell::new(false) };
}

/// Allocations seen inside a window; only the loop thread writes it.
static ALLOCS: AtomicU64 = AtomicU64::new(0);

/// The counting allocator: [`System`] plus a count of the calls inside the window.
pub struct Counting;

impl Counting {
    /// Counts one call when this thread's window is open; a thread being torn down counts nothing.
    #[inline]
    fn count() {
        if WINDOW.try_with(Cell::get).unwrap_or(false) {
            ALLOCS.store(ALLOCS.load(Relaxed).saturating_add(1), Relaxed);
        }
    }
}

// SAFETY: every method forwards its arguments unchanged to `System`, which upholds the
// `GlobalAlloc` contract; the count touches neither the layout nor the pointers, and
// allocates nothing itself (a const thread-local `Cell` and an atomic).
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

/// Opens the window on the calling thread.
pub fn open_window() {
    let _ = WINDOW.try_with(|w| w.set(true));
}

/// Closes the window on the calling thread.
pub fn close_window() {
    let _ = WINDOW.try_with(|w| w.set(false));
}

/// Allocations counted so far.
pub fn allocs() -> u64 {
    ALLOCS.load(Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::hint::black_box;

    #[global_allocator]
    static GLOBAL: Counting = Counting;

    #[test]
    fn only_the_window_on_this_thread_counts() {
        let before = allocs();
        black_box(std::ffi::CString::new("outside").ok());
        assert_eq!(allocs(), before);

        open_window();
        black_box(std::ffi::CString::new("inside").ok());
        std::thread::spawn(|| black_box(std::ffi::CString::new("other thread").ok()))
            .join()
            .ok();
        close_window();
        let counted = allocs() - before;
        assert!(counted >= 1, "{counted}");

        let after = allocs();
        black_box(std::ffi::CString::new("closed").ok());
        assert_eq!(allocs(), after);
    }
}
