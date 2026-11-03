//! The probe controllers and the recorder statics they write each tick, read by `main` after `run()`.
//!
//! Every item starts all-zero, so it sits in `.bss` (research R6, R21 P5).
//! One writer per item: relaxed load and store, no read-modify-write. The
//! controllers are plain `fn` items: no heap, no `dyn`, no lock, no I/O
//! (FR-018 (SRS NFR-1)).

use std::hint::black_box;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::time::Instant;

use kmr_api::{Q, Sensors, State, Time};

use crate::hist::Histogram;
use crate::plan::{RunPlan, WriteMode};
use crate::{alloc, calib, grid};

/// Tick period, in ns.
pub static DT_NS: AtomicU64 = AtomicU64::new(0);
/// Warm-up ticks W: the figures start at tick W.
pub static WARMUP: AtomicU64 = AtomicU64::new(0);
/// Planned ticks K: tick K−1 raises the shutdown.
pub static PLANNED: AtomicU64 = AtomicU64::new(0);
/// Workload iterations per tick.
pub static WORK_ITERS: AtomicU64 = AtomicU64::new(0);
/// Whether `work` writes every actuator.
pub static WRITE_ALL: AtomicBool = AtomicBool::new(false);
/// Test-only: the tick at which `work` allocates once; `u64::MAX` for never. Debug builds only.
#[cfg(debug_assertions)]
pub static INJECT_AT: AtomicU64 = AtomicU64::new(u64::MAX);

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

/// The engine's grid origin as the bench sees it: tick 0's start minus the engine's `elapsed()`.
///
/// DECISION: overruns are judged against this origin, not `T0`. `T0` sits after the engine's
/// `arm`, so a tick ending in that gap past the engine's deadline was on time for the bench and
/// every 10 µs run read `inconsistent`. Lateness keeps `T0` (`anchor = "tick0_start"`). What
/// remains is the engine's path from `Last` to its own end stamp (the write batch, the bus
/// checks): the bench count stays a lower bound of the engine's.
pub static ORIGIN: AtomicU64 = AtomicU64::new(0);
/// Grid slot of the tick that ended last.
pub static LAST_SLOT: AtomicU64 = AtomicU64::new(0);
/// The engine's `overrun_count()` at the previous `First`.
pub static ENGINE_COUNT_PREV: AtomicU64 = AtomicU64::new(0);
/// The engine's `accumulated_overrun()` at the previous `First`, in ns.
pub static ENGINE_ACC_PREV: AtomicU64 = AtomicU64::new(0);
/// The engine's `overrun_count()` at `First` of tick W.
pub static ENGINE_OVERRUNS_AT_W: AtomicU64 = AtomicU64::new(0);
/// The engine's `overrun_count()` at `First` of tick K−1.
pub static ENGINE_OVERRUNS_AT_END: AtomicU64 = AtomicU64::new(0);
/// The engine's `tick()` at `First` of tick K−1.
pub static ENGINE_TICK: AtomicU64 = AtomicU64::new(0);
/// The engine's `dt()` at `First` of tick K−1, in ns.
pub static ENGINE_DT_NS: AtomicU64 = AtomicU64::new(0);
/// The engine's `elapsed()` at `First` of tick K−1, in ns.
pub static ENGINE_ELAPSED_NS: AtomicU64 = AtomicU64::new(0);
/// The engine's `last_overrun()` at `First` of tick K−1, in ns plus one; zero for `None`.
pub static ENGINE_LAST_OVERRUN_NS_PLUS_ONE: AtomicU64 = AtomicU64::new(0);

/// Start lateness against the replayed grid, ticks W and on.
pub static LATENESS: Histogram = Histogram::new();
/// Gap from the previous `end` to this `start`, after an on-time tick.
pub static GAP_ON_TIME: Histogram = Histogram::new();
/// Gap from the previous `end` to this `start`, after a late tick.
pub static GAP_AFTER_LATE: Histogram = Histogram::new();

/// Hands the plan to the probe; `ps_per_iter` is the calibration taken before `run()`.
pub fn configure(plan: &RunPlan, ps_per_iter: u64) {
    DT_NS.store(plan.dt_ns, Relaxed);
    WARMUP.store(plan.warmup, Relaxed);
    PLANNED.store(plan.ticks, Relaxed);
    WORK_ITERS.store(calib::iters_for(plan.work_ns, ps_per_iter), Relaxed);
    WRITE_ALL.store(plan.write == WriteMode::All, Relaxed);
    #[cfg(debug_assertions)]
    INJECT_AT.store(plan.inject_at.unwrap_or(u64::MAX), Relaxed);
}

/// `First`, registered first: stamps the start, follows the engine's slot, records lateness and
/// gap from tick W, opens the window at W.
///
/// When the engine's overrun count grew by one, the previous tick was late by the growth of
/// its accumulated overrun, and the slot follows from that (research R4 phase B).
pub fn start<const N: usize>(time: &Time, _: &mut State<'_, N>, _: &Sensors) {
    let now = now_ns();
    let tick = TICK.load(Relaxed);
    let count = time.overrun_count();
    let acc = ns(time.accumulated_overrun());
    if tick == 0 {
        T0.store(now, Relaxed);
        ORIGIN.store(now.saturating_sub(ns(time.elapsed())), Relaxed);
    } else {
        let grew = count == ENGINE_COUNT_PREV.load(Relaxed).saturating_add(1);
        let late = if grew {
            acc.saturating_sub(ENGINE_ACC_PREV.load(Relaxed))
        } else {
            0
        };
        if grew || count == ENGINE_COUNT_PREV.load(Relaxed) {
            SLOT.store(
                grid::slot_after(LAST_SLOT.load(Relaxed), late, DT_NS.load(Relaxed)),
                Relaxed,
            );
        }
    }
    ENGINE_COUNT_PREV.store(count, Relaxed);
    ENGINE_ACC_PREV.store(acc, Relaxed);
    let warmup = WARMUP.load(Relaxed);
    if tick == warmup {
        ENGINE_OVERRUNS_AT_W.store(count, Relaxed);
    }
    if tick.saturating_add(1) == PLANNED.load(Relaxed) {
        ENGINE_OVERRUNS_AT_END.store(count, Relaxed);
        ENGINE_TICK.store(time.tick(), Relaxed);
        ENGINE_DT_NS.store(ns(time.dt()), Relaxed);
        ENGINE_ELAPSED_NS.store(ns(time.elapsed()), Relaxed);
        ENGINE_LAST_OVERRUN_NS_PLUS_ONE.store(
            time.last_overrun().map_or(0, |d| ns(d).saturating_add(1)),
            Relaxed,
        );
    }
    if tick >= warmup {
        let late = grid::lateness(
            now,
            T0.load(Relaxed),
            SLOT.load(Relaxed),
            DT_NS.load(Relaxed),
        );
        LATENESS.record(late);
        let gap = now.saturating_sub(LAST_END.load(Relaxed));
        if tick > 0 {
            if LAST_LATE.load(Relaxed) == 0 {
                GAP_ON_TIME.record(gap);
            } else {
                GAP_AFTER_LATE.record(gap);
            }
        }
    }
    if tick == warmup {
        alloc::open_window();
    }
}

/// `EachTick`: spins the calibrated workload, then writes every actuator when the plan says `all`.
pub fn work<const N: usize>(_: &Time, state: &mut State<'_, N>, _: &Sensors) {
    black_box(calib::spin(WORK_ITERS.load(Relaxed)));
    if WRITE_ALL.load(Relaxed) {
        write_target(state);
    }
    #[cfg(debug_assertions)]
    if TICK.load(Relaxed) == INJECT_AT.load(Relaxed) {
        black_box(std::ffi::CString::new("bench").ok());
    }
}

/// `Last`, registered last: stamps the end, judges the overrun, replays the next slot; tick K−1 stops.
pub fn end<const N: usize>(_: &Time, _: &mut State<'_, N>, _: &Sensors) {
    let now = now_ns();
    let tick = TICK.load(Relaxed);
    let (origin, slot, dt) = (
        ORIGIN.load(Relaxed),
        SLOT.load(Relaxed),
        DT_NS.load(Relaxed),
    );
    let planned = PLANNED.load(Relaxed);
    let judged = tick >= WARMUP.load(Relaxed) && tick.saturating_add(2) <= planned;
    if judged && grid::overran(now, origin, slot, dt) {
        BENCH_OVERRUNS.store(BENCH_OVERRUNS.load(Relaxed).saturating_add(1), Relaxed);
    }
    LAST_LATE.store(
        now.saturating_sub(grid::deadline(origin, slot, dt)),
        Relaxed,
    );
    LAST_END.store(now, Relaxed);
    LAST_SLOT.store(slot, Relaxed);
    SLOT.store(grid::next_slot(slot, now, origin, dt), Relaxed);
    let done = tick.saturating_add(1);
    TICK.store(done, Relaxed);
    TICKS_DONE.store(done, Relaxed);
    if done == planned {
        alloc::close_window();
        kmr_api::shutdown!("bench: planned ticks done");
    }
}

/// Does nothing (spec "Empty controllers").
pub fn noop<const N: usize>(_: &Time, _: &mut State<'_, N>, _: &Sensors) {}

/// Writes the hold target to every actuator: the newest read position, else 0 rad.
///
/// It calls `prev_q(0)` either way, so a simulated and a hardware run do the
/// same work (assumed FR-6.3 meaning; research R15, R16).
pub fn write_target<const N: usize>(state: &mut State<'_, N>) {
    let target = state.prev_q(0).unwrap_or([Q(0.0); N]);
    state.set_all_q(target);
}

/// Nanoseconds since [`EPOCH`]; zero before it is set.
#[inline]
pub fn now_ns() -> u64 {
    EPOCH.get().map_or(0, |epoch| {
        u64::try_from(epoch.elapsed().as_nanos()).unwrap_or(u64::MAX)
    })
}

/// Engine overruns over ticks W..K−2: the count at `First` of K−1 minus the one at W.
pub fn engine_overruns() -> u64 {
    ENGINE_OVERRUNS_AT_END
        .load(Relaxed)
        .saturating_sub(ENGINE_OVERRUNS_AT_W.load(Relaxed))
}

/// The engine's `last_overrun()` at `First` of tick K−1, in ns.
pub fn engine_last_overrun_ns() -> Option<u64> {
    ENGINE_LAST_OVERRUN_NS_PLUS_ONE.load(Relaxed).checked_sub(1)
}

/// A duration in ns, saturated.
fn ns(d: std::time::Duration) -> u64 {
    u64::try_from(d.as_nanos()).unwrap_or(u64::MAX)
}

/// Bytes of the recorder statics, for the report's `bench_storage_bytes`.
pub fn bench_storage_bytes() -> u64 {
    let counters = [
        &DT_NS,
        &WARMUP,
        &PLANNED,
        &WORK_ITERS,
        &T0,
        &SLOT,
        &LAST_END,
        &LAST_LATE,
        &TICK,
        &BENCH_OVERRUNS,
        &TICKS_DONE,
        &ORIGIN,
        &LAST_SLOT,
        &ENGINE_COUNT_PREV,
        &ENGINE_ACC_PREV,
        &ENGINE_OVERRUNS_AT_W,
        &ENGINE_OVERRUNS_AT_END,
        &ENGINE_TICK,
        &ENGINE_DT_NS,
        &ENGINE_ELAPSED_NS,
        &ENGINE_LAST_OVERRUN_NS_PLUS_ONE,
    ];
    let total = size_of_val(&EPOCH)
        + size_of_val(&WRITE_ALL)
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
