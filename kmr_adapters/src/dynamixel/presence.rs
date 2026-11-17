//! Which declared actuators are live under the missing policy: lost after a
//! few silent reads, never an error, each loss or absence logged once per
//! bus; and, under the join policy, where the join of an absent one stands.

use core::fmt;
use core::time::Duration;
use std::time::Instant;

use kmr_core::hal::{JoinPolicy, Mask, MissingPolicy};
use tracing::warn;

/// Consecutive due reads that time out before a live slot is lost.
// DECISION: 3 timed-out reads: one lost packet on a noisy line is no loss,
// and three ticks stay well under the bench's reaction time. Only a timeout
// is a miss: a reply with error bits, `NotDeclared` and bus errors are not.
pub(crate) const LOST_AFTER: u8 = 3;

/// Wall-clock time between two probes of one adapter.
// DECISION: 250 ms of wall clock, one probe per adapter, round robin: the
// adapter does not know dt, and widening `Overrides` with it was not
// decided. A silent probe costs one side-line timeout.
pub(crate) const PROBE_PERIOD: Duration = Duration::from_millis(250);

/// How long the Settle stage asks a joiner whose fault flag lags, once per
/// tick: the total of bring-up's voltage flag settle loop.
pub(crate) const SETTLE_FOR: Duration = Duration::from_millis(500);

/// Cold rejoins one slot may make in a run before it is refused.
// DECISION: 3: a servo that keeps losing its power is broken or badly
// wired, and each cold rejoin powers it again.
pub(crate) const MAX_REJOINS: u8 = 3;

/// Where the actuators of one adapter stand for the run.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Presence<const N: usize> {
    /// What a declared actuator that does not answer means.
    pub(crate) missing: MissingPolicy,
    /// Whether an absent declared actuator may join the run.
    pub(crate) join: JoinPolicy,
    /// Read, written and telemetered: what `Hal::present` reports.
    pub(crate) live: Mask<N>,
    /// Powered joiners whose first read is pending: read, not written.
    pub(crate) seed: Mask<N>,
    /// Set up whole this run, by bring-up or a join: a warm rejoin's
    /// condition.
    pub(crate) configured: Mask<N>,
    /// Never probed again this run.
    pub(crate) refused: Mask<N>,
    /// Live slots that missed at least one due read, then answered.
    pub(crate) suspect: Mask<N>,
    /// The seeded slots the last `read` read, cadence or not.
    pub(crate) seed_read: Mask<N>,
    /// Consecutive due reads each slot missed.
    misses: [u8; N],
    /// Cold rejoins each slot made this run.
    pub(crate) rejoins: [u8; N],
    /// The join in progress, one stage per tick.
    pub(crate) joining: Option<Join>,
    /// Where the next probe starts looking.
    pub(crate) cursor: usize,
    /// When the last probe went out; `None` before the first.
    pub(crate) last_probe: Option<Instant>,
    /// Time between two probes: [`PROBE_PERIOD`], which tests change.
    pub(crate) probe_period: Duration,
}

/// The join of one absent slot that answered a probe.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Join {
    /// Engine index of the joiner.
    pub(crate) index: usize,
    /// The stage the next upkeep runs.
    pub(crate) stage: Stage,
    /// When Settle first asked its fault; `None` before.
    pub(crate) since: Option<Instant>,
}

/// One step of a join, one per tick.
// DECISION: up to 6 stages, one per tick: Hold (at most 2 declares) and
// Health (at most 2 EEPROM settles) are accepted as one-off overruns; Settle
// asks once per tick for at most `SETTLE_FOR`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Stage {
    /// Re-declares the bus with the joiner, which it does not hold yet.
    Hold,
    /// Rearms a warm joiner, or cuts a cold one and writes its limits.
    Health,
    /// Waits, one fault ask per tick, for a lagging fault flag to clear.
    Settle,
    /// Checks its voltage, temperature and limits, as bring-up's `health`.
    Check,
    /// Checks its mode and limits, then writes its settings.
    Configure,
    /// Writes its start goal, then turns its torque on.
    Power,
}

impl<const N: usize> Presence<N> {
    /// Both policies off, every slot live.
    pub(crate) const fn new() -> Self {
        Self {
            missing: MissingPolicy::Fail,
            join: JoinPolicy::Never,
            live: Mask::ALL,
            seed: Mask::NONE,
            configured: Mask::NONE,
            refused: Mask::NONE,
            suspect: Mask::NONE,
            seed_read: Mask::NONE,
            misses: [0; N],
            rejoins: [0; N],
            joining: None,
            cursor: 0,
            last_probe: None,
            probe_period: PROBE_PERIOD,
        }
    }

    /// Whether absent slots are probed and may join: both policies on.
    pub(crate) fn joins(&self) -> bool {
        self.tolerates() && self.join == JoinPolicy::Declared
    }

    /// The first slot of `candidates` from `cursor` on, wrapping round; the
    /// cursor is left where it is.
    // DECISION: a peek, not a take: a tick that may not probe the slot
    // found keeps it first for the next one; the caller moves the cursor
    // past it once probed.
    pub(crate) fn next_probe(&self, candidates: Mask<N>) -> Option<usize> {
        let after = candidates.iter().find(|&index| index >= self.cursor);
        after.or_else(|| candidates.iter().next())
    }

    /// Takes slot `index` out of the run for good: not probed again.
    pub(crate) fn refuse(&mut self, index: usize) {
        self.drop_slot(index);
        self.refused.set(index);
    }

    /// Whether a silent actuator is absent rather than an error.
    pub(crate) fn tolerates(&self) -> bool {
        self.missing == MissingPolicy::Tolerate
    }

    /// Counts one due read of slot `index` that timed out; true when that
    /// makes it lost. Never under `Fail`.
    pub(crate) fn miss(&mut self, index: usize) -> bool {
        if !self.tolerates() {
            return false;
        }
        let Some(misses) = self.misses.get_mut(index) else {
            return false;
        };
        *misses = misses.saturating_add(1);
        if *misses < LOST_AFTER {
            return false;
        }
        *misses = 0;
        true
    }

    /// Slot `index` answered: its count starts over, and a live one that
    /// had missed is suspect.
    pub(crate) fn answered(&mut self, index: usize) {
        let Some(misses) = self.misses.get_mut(index) else {
            return;
        };
        if *misses > 0 && self.live.get(index) {
            self.suspect.set(index);
        }
        *misses = 0;
    }

    /// Takes slot `index` out of the run: neither live nor seeded.
    pub(crate) fn drop_slot(&mut self, index: usize) {
        self.live.clear(index);
        self.seed.clear(index);
        self.suspect.clear(index);
        if let Some(misses) = self.misses.get_mut(index) {
            *misses = 0;
        }
    }

    /// Slots a due read reads: live and seeded.
    pub(crate) fn driven(&self) -> Mask<N> {
        self.live.union(self.seed)
    }
}

/// What a presence log line reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Event {
    /// Silent at bring-up.
    Start,
    /// Stopped answering at run time.
    Lost,
    /// Taken out with their whole bus, whose setup failed.
    Dropped,
    /// Not reached by the torque off at exit.
    Unreached,
}

/// One log line: `first` to `last`, a run of consecutive ids.
struct Line {
    /// What happened to them.
    event: Event,
    /// Lowest id of the run.
    first: u8,
    /// Highest id of the run.
    last: u8,
}

impl fmt::Display for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (first, last) = (self.first, self.last);
        let one = first == last;
        if one {
            write!(f, "actuator id {first} ")?;
        } else {
            write!(f, "ids {first} to {last} ")?;
        }
        f.write_str(match (self.event, one) {
            (Event::Start, _) => "not available at start",
            (Event::Lost, _) => "not available anymore",
            (Event::Dropped, true) => "dropped: its bus setup failed",
            (Event::Dropped, false) => "dropped: their bus setup failed",
            (Event::Unreached, _) => "not reached for torque off",
        })
    }
}

/// One warning per run of consecutive ids among `ids`, all on bus `bus`,
/// which the line carries as a field: AX id 3 and RX id 3 both exist.
// DECISION: `sort_unstable`, in place and allocation-free, where the plan
// says insertion sort: the order is all that matters.
pub(crate) fn report(bus: &'static str, ids: &mut [u8], event: Event) {
    each_run(ids, event, |line| warn!(bus, "{line}"));
}

/// [`report`], each line carrying `error` as a field.
pub(crate) fn report_failed(
    bus: &'static str,
    ids: &mut [u8],
    event: Event,
    error: impl fmt::Display,
) {
    each_run(ids, event, |line| warn!(bus, %error, "{line}"));
}

/// `emit` once per run of consecutive ids among `ids`, sorted first.
fn each_run(ids: &mut [u8], event: Event, mut emit: impl FnMut(Line)) {
    ids.sort_unstable();
    let mut rest = &*ids;
    while let [first, ..] = rest {
        let run = rest
            .iter()
            .zip(rest.iter().skip(1))
            .take_while(|&(&low, &high)| high == low.wrapping_add(1))
            .count();
        let last = rest.get(run).copied().unwrap_or(*first);
        emit(Line {
            event,
            first: *first,
            last,
        });
        rest = rest.get(run + 1..).unwrap_or_default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn misses_count_only_under_tolerate() {
        let mut presence = Presence::<4>::new();
        for _ in 0..2 * LOST_AFTER {
            assert!(!presence.miss(1));
        }
        presence.missing = MissingPolicy::Tolerate;
        assert!(!presence.miss(1) && !presence.miss(1));
        presence.answered(1);
        assert!(presence.suspect.get(1));
        assert!(!presence.miss(1) && !presence.miss(1));
        assert!(presence.miss(1));
    }

    #[test]
    fn lines_name_one_id_or_a_run() {
        let line = |event, first, last| Line { event, first, last }.to_string();
        assert_eq!(
            line(Event::Lost, 3, 3),
            "actuator id 3 not available anymore"
        );
        assert_eq!(
            line(Event::Start, 2, 4),
            "ids 2 to 4 not available at start"
        );
        assert_eq!(
            line(Event::Unreached, 7, 8),
            "ids 7 to 8 not reached for torque off"
        );
        assert_eq!(
            line(Event::Dropped, 5, 5),
            "actuator id 5 dropped: its bus setup failed"
        );
        assert_eq!(
            line(Event::Dropped, 5, 6),
            "ids 5 to 6 dropped: their bus setup failed"
        );
    }

    #[test]
    fn next_probe_wraps_round_from_the_cursor() {
        let mut presence = Presence::<4>::new();
        let mut candidates = Mask::NONE;
        candidates.set(0);
        candidates.set(2);
        assert_eq!(presence.next_probe(candidates), Some(0));
        presence.cursor = 1;
        assert_eq!(presence.next_probe(candidates), Some(2));
        presence.cursor = 3;
        assert_eq!(presence.next_probe(candidates), Some(0));
        assert_eq!(presence.next_probe(Mask::NONE), None);
    }
}
