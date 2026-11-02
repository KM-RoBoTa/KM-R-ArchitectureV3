//! The recorder: statics the probe controllers write each tick and `main` reads after `run()`.
//!
//! Every item starts all-zero, so it sits in `.bss` (research R6, R21 P5).
//! One writer per item: relaxed load and store, no read-modify-write.

use std::sync::OnceLock;
use std::sync::atomic::AtomicU64;
use std::time::Instant;

use crate::hist::Histogram;

/// The process epoch every stamp counts from; set by `main` before `run()`.
pub static EPOCH: OnceLock<Instant> = OnceLock::new();

/// Start of tick 0, the grid anchor (`anchor = "tick0_start"`).
pub static T0: AtomicU64 = AtomicU64::new(0);
/// Grid slot of the tick in progress.
pub static SLOT: AtomicU64 = AtomicU64::new(0);
/// End of the previous tick's `Last`.
pub static LAST_END: AtomicU64 = AtomicU64::new(0);
/// How late the previous tick ended against its deadline, zero when on time.
pub static LAST_LATE: AtomicU64 = AtomicU64::new(0);
/// Index of the tick in progress, counted by the probe.
pub static TICK: AtomicU64 = AtomicU64::new(0);
/// Overruns the bench judged over ticks W to K−2.
pub static BENCH_OVERRUNS: AtomicU64 = AtomicU64::new(0);
/// Ticks whose `Last` finished.
pub static TICKS_DONE: AtomicU64 = AtomicU64::new(0);

/// Start lateness against the replayed grid, ticks W and on.
pub static LATENESS: Histogram = Histogram::new();
/// Gap from the previous `end` to this `start`, after an on-time tick.
pub static GAP_ON_TIME: Histogram = Histogram::new();
/// Gap from the previous `end` to this `start`, after a late tick.
pub static GAP_AFTER_LATE: Histogram = Histogram::new();

/// Nanoseconds since [`EPOCH`]; zero before it is set.
#[inline]
pub fn now_ns() -> u64 {
    EPOCH.get().map_or(0, |epoch| {
        u64::try_from(epoch.elapsed().as_nanos()).unwrap_or(u64::MAX)
    })
}

/// Bytes of the recorder statics, for the report's `bench_storage_bytes`.
pub fn bench_storage_bytes() -> u64 {
    let counters = [
        &T0,
        &SLOT,
        &LAST_END,
        &LAST_LATE,
        &TICK,
        &BENCH_OVERRUNS,
        &TICKS_DONE,
    ];
    let total = size_of_val(&EPOCH)
        + counters.iter().map(|c| size_of_val(*c)).sum::<usize>()
        + [&LATENESS, &GAP_ON_TIME, &GAP_AFTER_LATE]
            .iter()
            .map(|h| size_of_val(*h))
            .sum::<usize>();
    u64::try_from(total).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn storage_is_dominated_by_the_three_histograms() {
        let one = u64::try_from(size_of::<Histogram>()).unwrap_or(0);
        assert!(bench_storage_bytes() > 3 * one);
        assert!(bench_storage_bytes() < 3 * one + 256);
    }
}
