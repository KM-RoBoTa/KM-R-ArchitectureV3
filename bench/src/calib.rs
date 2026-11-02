//! Calibrated workload for the budget method (research R8, R21 P12).
//!
//! The workload is integer arithmetic through `black_box`, timed before and
//! after `run()`; a drift above 2 % marks the run `uncalibrated`.

use std::hint::black_box;
use std::time::Instant;

use crate::hist::Histogram;

/// Iterations in one timed calibration run.
pub const ITERS: u64 = 1_000_000;

/// Timed runs per calibration; the fastest one counts.
pub const RUNS: usize = 5;

/// Drift above this, in permille, flags the run `uncalibrated`.
pub const DRIFT_LIMIT_PERMILLE: u64 = 20;

/// Scratch target of the probe-cost stores, so the real histograms stay empty.
static SCRATCH: Histogram = Histogram::new();

/// Spins `iters` iterations of the workload; the result only defeats the optimizer.
#[inline]
pub fn spin(iters: u64) -> u64 {
    let mut acc = black_box(0x9E37_79B9_7F4A_7C15_u64);
    for i in 0..black_box(iters) {
        acc = black_box(acc.rotate_left(5) ^ i).wrapping_mul(0x0000_0100_0000_01B3);
    }
    acc
}

/// Iterations that take `work_ns` at `ps_per_iter`.
pub fn iters_for(work_ns: u64, ps_per_iter: u64) -> u64 {
    work_ns.saturating_mul(1000) / ps_per_iter.max(1)
}

/// Cost of one workload iteration in ps: the best of [`RUNS`] runs of [`ITERS`].
pub fn calibrate() -> u64 {
    best_ns(|| {
        black_box(spin(ITERS));
    })
    .saturating_mul(1000)
        / ITERS
}

/// `|after − before| × 1000 / before`, `None` when `before` is zero.
pub fn drift_permille(before: u64, after: u64) -> Option<u64> {
    before
        .abs_diff(after)
        .saturating_mul(1000)
        .checked_div(before)
}

/// Whether a drift breaks the calibration.
pub fn uncalibrated(drift_permille: u64) -> bool {
    drift_permille > DRIFT_LIMIT_PERMILLE
}

/// What the probe adds per tick, in ns, rounded up: two `stamp` calls and two histogram stores.
pub fn probe_cost_ns(stamp: impl Fn() -> u64) -> u64 {
    best_ns(|| {
        for _ in 0..ITERS {
            let start = stamp();
            let end = stamp();
            SCRATCH.record(end.saturating_sub(start));
            SCRATCH.record(black_box(start) & 0xFFFF);
        }
    })
    .div_ceil(ITERS)
}

/// Bytes of the calibration statics, for the report's `bench_storage_bytes`.
pub fn storage_bytes() -> u64 {
    u64::try_from(size_of_val(&SCRATCH)).unwrap_or(u64::MAX)
}

/// Fastest of [`RUNS`] timed calls of `body`, in ns.
fn best_ns(body: impl Fn()) -> u64 {
    (0..RUNS)
        .map(|_| {
            let start = Instant::now();
            body();
            u64::try_from(start.elapsed().as_nanos()).unwrap_or(u64::MAX)
        })
        .min()
        .unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drift_is_the_change_over_before_in_permille() {
        assert_eq!(drift_permille(596, 749), Some(256));
        assert_eq!(drift_permille(1039, 1129), Some(86));
        assert_eq!(drift_permille(1000, 980), Some(20));
        assert_eq!(drift_permille(1000, 1000), Some(0));
        assert_eq!(drift_permille(0, 5), None);
    }

    #[test]
    fn twenty_permille_holds_and_twenty_one_flags() {
        assert!(!uncalibrated(20));
        assert!(uncalibrated(21));
    }

    #[test]
    fn work_converts_to_iterations() {
        assert_eq!(iters_for(1_000, 500), 2_000);
        assert_eq!(iters_for(0, 500), 0);
        assert_eq!(iters_for(1_000, 0), 1_000_000);
    }

    #[test]
    fn spin_is_deterministic_and_calibration_is_positive() {
        assert_eq!(spin(1000), spin(1000));
        assert_ne!(spin(1000), spin(1001));
        assert!(calibrate() > 0);
    }
}
