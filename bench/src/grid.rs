//! Replay of the engine's tick grid from the bench's own stamps (research R4, R5).
//!
//! Every time is in ns since the process epoch. `t0` is tick 0's start, a few
//! hundred ns after the engine's `arm`. Tick k sits on grid slot `s_k`: it is
//! due to start at `t0 + s_k·dt` and to end by `t0 + (s_k+1)·dt`. Pure
//! functions, no state: the probe keeps the slot.

/// Start of `slot` on the grid.
pub fn slot_start(t0: u64, slot: u64, dt: u64) -> u64 {
    t0.saturating_add(slot.saturating_mul(dt))
}

/// Deadline of the tick on `slot`: the start of the next slot.
pub fn deadline(t0: u64, slot: u64, dt: u64) -> u64 {
    slot_start(t0, slot.saturating_add(1), dt)
}

/// How late a tick on `slot` started, zero for a start on or before its grid point.
pub fn lateness(start: u64, t0: u64, slot: u64, dt: u64) -> u64 {
    start.saturating_sub(slot_start(t0, slot, dt))
}

/// Whether the tick on `slot`, ending at `end`, finished after its deadline.
pub fn overran(end: u64, t0: u64, slot: u64, dt: u64) -> bool {
    end > deadline(t0, slot, dt)
}

/// Slot of the next tick, the engine's rule (`kmr_core` `Time::end_tick`).
///
/// On time: the next slot. Late: the missed slots are skipped, rounded up, so
/// a tick late by exactly `k·dt` gives the next one a whole `dt`.
pub fn next_slot(slot: u64, end: u64, t0: u64, dt: u64) -> u64 {
    slot_after(slot, end.saturating_sub(deadline(t0, slot, dt)), dt)
}

/// Slot of the tick after one on `slot` that ended `late` past its deadline, zero when on time.
///
/// Phase B takes `late` from the engine: the growth of `accumulated_overrun()`
/// when `overrun_count()` grew by one (research R4).
pub fn slot_after(slot: u64, late: u64, dt: u64) -> u64 {
    let skipped = if dt == 0 { 0 } else { late.div_ceil(dt) };
    slot.saturating_add(1).saturating_add(skipped)
}

#[cfg(test)]
mod tests {
    use super::*;

    const T0: u64 = 5_000;
    const DT: u64 = 1_000_000;

    #[test]
    fn an_on_time_tick_takes_the_next_slot() {
        assert_eq!(next_slot(0, T0 + DT / 4, T0, DT), 1);
        assert_eq!(next_slot(0, T0 + DT, T0, DT), 1);
        assert!(!overran(T0 + DT, T0, 0, DT));
    }

    #[test]
    fn a_tick_late_by_exactly_dt_skips_one_slot() {
        let end = deadline(T0, 2, DT) + DT;
        assert!(overran(end, T0, 2, DT));
        assert_eq!(next_slot(2, end, T0, DT), 4);
    }

    #[test]
    fn a_tick_late_by_three_dt_and_one_ns_skips_four_slots() {
        let end = deadline(T0, 0, DT) + 3 * DT + 1;
        assert_eq!(next_slot(0, end, T0, DT), 5);
    }

    #[test]
    fn a_start_before_its_grid_point_is_zero_late() {
        assert_eq!(lateness(T0 + 3 * DT - 400, T0, 3, DT), 0);
        assert_eq!(lateness(0, T0, 0, DT), 0);
    }

    #[test]
    fn a_scripted_run_with_skipped_slots_follows_the_grid() {
        // (start offset in its slot, duration) of each tick, from t0.
        let script = [
            (0, 200_000),
            (30_000, 400_000),
            (10_000, 2_500_000),
            (2_000, 100_000),
            (0, 1_000_000),
            (700, 50_000),
        ];
        let mut slot = 0;
        let mut seen = [(0u64, 0u64, false); 6];
        for (k, (offset, length)) in script.into_iter().enumerate() {
            let start = slot_start(T0, slot, DT) + offset;
            let end = start + length;
            seen[k] = (
                slot,
                lateness(start, T0, slot, DT),
                overran(end, T0, slot, DT),
            );
            slot = next_slot(slot, end, T0, DT);
        }
        assert_eq!(
            seen,
            [
                (0, 0, false),
                (1, 30_000, false),
                (2, 10_000, true),
                (5, 2_000, false),
                (6, 0, false),
                (7, 700, false),
            ]
        );
    }

    #[test]
    fn the_engine_growth_rule_gives_the_slot_of_the_bench_rule() {
        // (count, accumulated) the engine shows at each First, from tick 0.
        let engine = [
            (0, 0),
            (0, 0),
            (1, 2_500_000),
            (1, 2_500_000),
            (2, 3_500_000),
        ];
        let mut slot = 0;
        let mut slots = [0u64; 5];
        for k in 1..engine.len() {
            let ((count, acc), (prev_count, prev_acc)) = (engine[k], engine[k - 1]);
            slot = if count == prev_count + 1 {
                slot_after(slot, acc - prev_acc, DT)
            } else {
                slot_after(slot, 0, DT)
            };
            slots[k] = slot;
        }
        assert_eq!(slots, [0, 1, 5, 6, 8]);
        assert_eq!(
            slot_after(1, 2_500_000, DT),
            next_slot(1, deadline(T0, 1, DT) + 2_500_000, T0, DT)
        );
        assert_eq!(slot_after(3, DT, DT), 5);
    }

    #[test]
    fn a_zero_dt_never_divides() {
        assert_eq!(next_slot(4, 10, 0, 0), 5);
    }
}
