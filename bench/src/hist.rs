//! Fixed histogram of nanosecond samples, `static`-ready and heap-free.
//!
//! One writer per histogram: every update is a relaxed load then a store, no
//! read-modify-write (research R6). All-zero at rest, so a `static` sits in
//! `.bss` (research R21 P5).

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering::Relaxed};

/// Number of buckets.
pub const BUCKETS: usize = 20_000;

/// Width of one bucket, in ns: the buckets cover 0 to 2 ms.
pub const BUCKET_NS: u64 = 100;

/// Samples in 100 ns buckets from 0 to 2 ms, plus overflow, exact min and max.
pub struct Histogram {
    buckets: [AtomicU32; BUCKETS],
    overflow: AtomicU64,
    count: AtomicU64,
    /// The minimum, bit-inverted: zero means no sample yet.
    min_inv: AtomicU64,
    max: AtomicU64,
}

impl Histogram {
    /// An empty histogram, all-zero.
    pub const fn new() -> Self {
        Self {
            buckets: [const { AtomicU32::new(0) }; BUCKETS],
            overflow: AtomicU64::new(0),
            count: AtomicU64::new(0),
            min_inv: AtomicU64::new(0),
            max: AtomicU64::new(0),
        }
    }

    /// Records one sample, in ns. Call from one thread only.
    #[inline]
    pub fn record(&self, ns: u64) {
        let index = ns / BUCKET_NS;
        match usize::try_from(index)
            .ok()
            .and_then(|i| self.buckets.get(i))
        {
            Some(bucket) => bucket.store(bucket.load(Relaxed).saturating_add(1), Relaxed),
            None => self
                .overflow
                .store(self.overflow.load(Relaxed).saturating_add(1), Relaxed),
        }
        self.count
            .store(self.count.load(Relaxed).saturating_add(1), Relaxed);
        if !ns > self.min_inv.load(Relaxed) {
            self.min_inv.store(!ns, Relaxed);
        }
        if ns > self.max.load(Relaxed) {
            self.max.store(ns, Relaxed);
        }
    }

    /// Number of samples recorded.
    pub fn count(&self) -> u64 {
        self.count.load(Relaxed)
    }

    /// Samples at or above 2 ms.
    pub fn overflow(&self) -> u64 {
        self.overflow.load(Relaxed)
    }

    /// Exact smallest sample, `None` when empty.
    pub fn min(&self) -> Option<u64> {
        (self.count() > 0).then(|| !self.min_inv.load(Relaxed))
    }

    /// Exact largest sample, `None` when empty.
    pub fn max(&self) -> Option<u64> {
        (self.count() > 0).then(|| self.max.load(Relaxed))
    }

    /// The `permille`-th quantile as the upper edge of its bucket, `None` when empty.
    ///
    /// A rank that falls in the overflow gives the exact max, the only bound known there.
    pub fn quantile(&self, permille: u64) -> Option<u64> {
        let count = self.count();
        if count == 0 {
            return None;
        }
        let rank = count
            .saturating_mul(permille.min(1000))
            .div_ceil(1000)
            .max(1);
        let mut seen = 0u64;
        for (edge, bucket) in (1..).map(|i: u64| i * BUCKET_NS).zip(&self.buckets) {
            seen = seen.saturating_add(u64::from(bucket.load(Relaxed)));
            if seen >= rank {
                return Some(edge);
            }
        }
        self.max()
    }

    /// Median, as an upper bucket edge.
    pub fn p50(&self) -> Option<u64> {
        self.quantile(500)
    }

    /// 99th percentile, as an upper bucket edge.
    pub fn p99(&self) -> Option<u64> {
        self.quantile(990)
    }
}

impl Default for Histogram {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_histogram_has_no_figure() {
        let h = Histogram::new();
        assert_eq!(h.count(), 0);
        assert_eq!(
            (h.min(), h.max(), h.p50(), h.p99()),
            (None, None, None, None)
        );
    }

    #[test]
    fn bucket_edges_are_half_open() {
        let h = Histogram::new();
        h.record(0);
        assert_eq!(h.p50(), Some(100));
        let h = Histogram::new();
        h.record(99);
        assert_eq!(h.p50(), Some(100));
        let h = Histogram::new();
        h.record(100);
        assert_eq!(h.p50(), Some(200));
        let h = Histogram::new();
        h.record(1_999_999);
        assert_eq!((h.p50(), h.overflow()), (Some(2_000_000), 0));
    }

    #[test]
    fn two_ms_and_above_overflow_and_report_the_max() {
        let h = Histogram::new();
        h.record(2_000_000);
        h.record(5_000_123);
        assert_eq!((h.overflow(), h.count()), (2, 2));
        assert_eq!(h.p50(), Some(5_000_123));
        assert_eq!(h.max(), Some(5_000_123));
    }

    #[test]
    fn min_and_max_are_exact() {
        let h = Histogram::new();
        for ns in [7_345, 12, 99_999, 0, 400] {
            h.record(ns);
        }
        assert_eq!((h.min(), h.max()), (Some(0), Some(99_999)));
        let h = Histogram::new();
        h.record(u64::MAX);
        assert_eq!((h.min(), h.max()), (Some(u64::MAX), Some(u64::MAX)));
    }

    #[test]
    fn p50_and_p99_of_a_known_set() {
        let h = Histogram::new();
        // 1..=100 µs, one sample each: rank 50 is 50 µs, rank 99 is 99 µs.
        for us in 1..=100u64 {
            h.record(us * 1000);
        }
        assert_eq!(h.p50(), Some(50_100));
        assert_eq!(h.p99(), Some(99_100));
        assert_eq!(h.quantile(1000), Some(100_100));
        assert_eq!(h.quantile(0), Some(1_100));
    }

    #[test]
    fn a_static_histogram_is_zero_initialized() {
        static H: Histogram = Histogram::new();
        H.record(250);
        assert_eq!((H.count(), H.min(), H.p99()), (1, Some(250), Some(300)));
    }
}
