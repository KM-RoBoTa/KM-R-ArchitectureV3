use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::thread;

use super::Latest;

thread_local! {
    /// Whether this thread's allocations are counted: const-initialized, no destructor.
    static COUNTED: Cell<bool> = const { Cell::new(false) };
}

/// Allocations of counted threads.
static ALLOCS: AtomicU64 = AtomicU64::new(0);

/// [`System`], counting the allocations of the threads that opted in.
struct Counting;

// SAFETY: every method forwards its arguments unchanged to `System`, which upholds the
// `GlobalAlloc` contract; the count reads a const thread-local `Cell` and bumps an atomic,
// allocating nothing and touching neither the layout nor the pointers.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count();
        // SAFETY: the caller's contract for `alloc` is passed on as is.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count();
        // SAFETY: the caller's contract for `alloc_zeroed` is passed on as is.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count();
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

/// Counts one allocation when the calling thread opted in; a thread being torn down counts none.
fn count() {
    if COUNTED.try_with(Cell::get).unwrap_or(false) {
        ALLOCS.fetch_add(1, Relaxed);
    }
}

#[test]
fn a_value_is_taken_once() {
    let (mut writer, mut reader) = Latest::new().split();
    assert_eq!(reader.take(), None);
    writer.put(7_u32);
    assert_eq!(reader.take(), Some(7));
    assert_eq!(reader.take(), None);
    assert_eq!(writer.superseded(), 0);
}

#[test]
fn two_puts_before_a_take_count_one_superseded() {
    let (mut writer, mut reader) = Latest::new().split();
    writer.put(1_u32);
    writer.put(2);
    assert_eq!(reader.take(), Some(2));
    assert_eq!(reader.take(), None);
    assert_eq!((writer.superseded(), reader.superseded()), (1, 1));
}

#[test]
fn a_put_after_a_take_supersedes_nothing() {
    let (mut writer, mut reader) = Latest::new().split();
    for value in 0..10_u32 {
        writer.put(value);
        assert_eq!(reader.take(), Some(value));
    }
    assert_eq!(writer.superseded(), 0);
}

/// The research R15 payload: a sequence number, 64 positions, a mask.
#[derive(Clone, Copy)]
struct Message {
    seq: u64,
    q: [f32; 64],
    mask: u64,
}

#[test]
fn the_reader_never_allocates_and_never_sees_a_torn_value() {
    const PUTS: u64 = 200_000;
    const TAKES: u32 = 2_000_000;
    let (mut writer, mut reader) = Latest::new().split();
    let done = AtomicBool::new(false);

    thread::scope(|scope| {
        scope.spawn(|| {
            for seq in 1..=PUTS {
                #[allow(clippy::cast_precision_loss, reason = "a marker, compared exactly")]
                let marker = seq as f32;
                writer.put(Message {
                    seq,
                    q: [marker; 64],
                    mask: seq,
                });
            }
            done.store(true, Relaxed);
        });
        let reader = scope.spawn(move || {
            COUNTED.set(true);
            let mut last = 0;
            let mut taken = 0_u64;
            for _ in 0..TAKES {
                if let Some(message) = reader.take() {
                    #[allow(clippy::cast_precision_loss, reason = "a marker, compared exactly")]
                    let marker = message.seq as f32;
                    assert!(message.seq > last, "values strictly increase");
                    assert!(
                        message.q.iter().all(|&q| q.to_bits() == marker.to_bits())
                            && message.mask == message.seq,
                        "a torn value"
                    );
                    last = message.seq;
                    taken += 1;
                }
            }
            COUNTED.set(false);
            (taken, reader.superseded())
        });
        let (taken, _) = reader.join().expect("the reader thread");
        assert!(taken > 0, "the reader took nothing");
    });

    assert!(done.load(Relaxed));
    assert_eq!(ALLOCS.load(Relaxed), 0, "the reader thread allocated");
}
