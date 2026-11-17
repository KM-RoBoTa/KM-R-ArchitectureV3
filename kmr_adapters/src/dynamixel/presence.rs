//! Which declared actuators are live under the missing policy: lost after a
//! few silent reads, never an error, each loss or absence logged once per
//! bus.

use core::fmt;

use kmr_core::hal::{Mask, MissingPolicy};
use tracing::warn;

/// Consecutive due reads that time out before a live slot is lost.
// DECISION: 3 timed-out reads: one lost packet on a noisy line is no loss,
// and three ticks stay well under the bench's reaction time. Only a timeout
// is a miss: a reply with error bits, `NotDeclared` and bus errors are not.
pub(crate) const LOST_AFTER: u8 = 3;

/// Where the actuators of one adapter stand for the run.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Presence<const N: usize> {
    /// What a declared actuator that does not answer means.
    pub(crate) missing: MissingPolicy,
    /// Read, written and telemetered: what `Hal::present` reports.
    pub(crate) live: Mask<N>,
    /// Powered joiners whose first read is pending: read, not written.
    pub(crate) seed: Mask<N>,
    /// Limits written this run, their originals saved once.
    pub(crate) configured: Mask<N>,
    /// Live slots that missed at least one due read, then answered.
    pub(crate) suspect: Mask<N>,
    /// Consecutive due reads each slot missed.
    misses: [u8; N],
}

impl<const N: usize> Presence<N> {
    /// Both policies off, every slot live.
    pub(crate) const fn new() -> Self {
        Self {
            missing: MissingPolicy::Fail,
            live: Mask::ALL,
            seed: Mask::NONE,
            configured: Mask::NONE,
            suspect: Mask::NONE,
            misses: [0; N],
        }
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
        f.write_str(match self.event {
            Event::Start => "not available at start",
            Event::Lost => "not available anymore",
            Event::Unreached => "not reached for torque off",
        })
    }
}

/// One warning per run of consecutive ids among `ids`, all on bus `bus`,
/// which the line carries as a field: AX id 3 and RX id 3 both exist.
// DECISION: `sort_unstable`, in place and allocation-free, where the plan
// says insertion sort: the order is all that matters.
pub(crate) fn report(bus: &'static str, ids: &mut [u8], event: Event) {
    ids.sort_unstable();
    let mut rest = &*ids;
    while let [first, ..] = rest {
        let run = rest
            .iter()
            .zip(rest.iter().skip(1))
            .take_while(|&(&low, &high)| high == low.wrapping_add(1))
            .count();
        let last = rest.get(run).copied().unwrap_or(*first);
        let line = Line {
            event,
            first: *first,
            last,
        };
        warn!(bus, "{line}");
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
    }
}
