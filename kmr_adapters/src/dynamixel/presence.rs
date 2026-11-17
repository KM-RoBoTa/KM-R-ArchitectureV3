//! Which declared actuators are live under the missing policy: lost after a
//! few silent reads, never an error, each loss or absence logged once per
//! bus; and, under the join policy, where the join of an absent one stands.

use core::fmt;
use core::time::Duration;
use std::time::Instant;

use kmr_core::hal::{JoinPolicy, Mask, MissingPolicy, Overrides};
use tracing::warn;

use super::Dynamixel;
use super::backend::{Backend, BackendError, Undeclared};
use super::config::Model;

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

    /// A fresh presence for a run under `overrides`. A join policy without
    /// `Tolerate` is one warning.
    // DECISION (plan, DECISION 10): a join policy without `Tolerate` is one
    // warning, no error: nothing is ever absent to join.
    pub(crate) fn run(overrides: Overrides) -> Self {
        if overrides.join == JoinPolicy::Declared && overrides.missing == MissingPolicy::Fail {
            warn!("the join policy has no effect without MissingPolicy::Tolerate");
        }
        Self {
            missing: overrides.missing,
            join: overrides.join,
            ..Self::new()
        }
    }

    /// Every actuator of `owned` live, at the end of the claim.
    pub(crate) fn claim(&mut self, owned: Mask<N>) {
        self.live = owned;
    }

    /// Read, written and telemetered: what `Hal::present` reports.
    pub(crate) fn live(&self) -> Mask<N> {
        self.live
    }

    /// At the start of a tick's read: the slots a due read reads and the
    /// seeded ones, which every read reads; the seeded ones are noted as
    /// this read's.
    pub(crate) fn begin_read(&mut self) -> (Mask<N>, Mask<N>) {
        self.seed_read = self.seed;
        (self.driven(), self.seed)
    }

    /// Slot `index` read: a seeded one is live from now on.
    pub(crate) fn promote(&mut self, index: usize) {
        if self.seed.get(index) {
            self.seed.clear(index);
            self.live.set(index);
        }
    }

    /// The seeded slots the last `read` read, cadence or not.
    pub(crate) fn seed_read(&self) -> Mask<N> {
        self.seed_read
    }

    /// The slots of `powered` passed their whole bring-up; under both
    /// policies, the first probe is one probe period from now.
    // DECISION: a slot counts as set up this run, the warm rejoin's
    // condition, only once its whole bring-up passed, not once health did:
    // one lost in configure has no settings, nor its goal range.
    pub(crate) fn powered(&mut self, powered: Mask<N>) {
        self.configured = self.configured.union(powered);
        if self.joins() {
            self.last_probe = Some(Instant::now());
        }
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

impl<B, const NB: usize, const N: usize> Dynamixel<B, NB, N> {
    /// Takes actuator `index` out of the run: one `Lost` line.
    pub(super) fn lose(&mut self, index: usize) {
        self.presence.drop_slot(index);
        let (bus, id) = self.named(index);
        report(bus, &mut [id], Event::Lost);
    }

    /// One `event` line per bus and run of consecutive ids among `slots`.
    pub(super) fn report_slots(&self, slots: Mask<N>, event: Event) {
        self.each_bus(slots, |bus, ids| report(bus, ids, event));
    }

    /// `emit` once per bus with the firmware ids of `slots` on it, when it
    /// has any.
    pub(super) fn each_bus(&self, slots: Mask<N>, mut emit: impl FnMut(&'static str, &mut [u8])) {
        if slots.is_empty() {
            return;
        }
        for bus in 0..NB {
            let (mut ids, mut len) = ([0; N], 0);
            for index in slots.iter() {
                if let (Some(actuator), Some(slot)) = (self.actuators.get(index), ids.get_mut(len))
                    && actuator.bus == bus
                {
                    *slot = actuator.id;
                    len += 1;
                }
            }
            if let Some(ids) = ids.get_mut(..len)
                && !ids.is_empty()
            {
                emit(self.label(bus), ids);
            }
        }
    }

    /// Takes the actuators of `lost` out of the run: one `Lost` line per
    /// bus.
    pub(super) fn lose_slots(&mut self, lost: Mask<N>) {
        for index in lost.iter() {
            self.presence.drop_slot(index);
        }
        self.report_slots(lost, Event::Lost);
    }
}

#[expect(
    private_bounds,
    reason = "the SDK seam stays private: these helpers serve the Hal impl only"
)]
impl<B: Backend, const NB: usize, const N: usize> Dynamixel<B, NB, N> {
    /// `answer`, the first setup of bus `bus`, as it is under `Fail`. Under
    /// `Tolerate`: while the setup of bus `bus` stops at a silent
    /// actuator, takes it out of the run and sets up the live rest again,
    /// as many attempts per pass as the first setup, at most `passes`; no
    /// actuator left, no setup, and the bus holds none. Any other failure
    /// is returned. One `Start` line names the actuators taken out.
    pub(super) fn declare_present(
        &mut self,
        bus: usize,
        passes: usize,
        mut answer: Result<(), Undeclared>,
    ) -> Result<(), Undeclared> {
        if !self.presence.tolerates() {
            return answer;
        }
        let mut absent = Mask::NONE;
        for _ in 0..passes {
            let Err(Undeclared {
                id: Some(id),
                source: BackendError::Timeout,
            }) = answer
            else {
                break;
            };
            let silent = self
                .actuators
                .iter()
                .find(|(index, actuator)| {
                    actuator.bus == bus && actuator.id == id && self.presence.live.get(*index)
                })
                .map(|(index, _)| index);
            let Some(index) = silent else {
                break;
            };
            self.presence.drop_slot(index);
            absent.set(index);
            let (mut rest, mut len) = ([(0, Model::Xm430W210); N], 0);
            for (index, actuator) in self.actuators.iter() {
                if let (true, Some(slot)) = (
                    actuator.bus == bus && self.presence.live.get(index),
                    rest.get_mut(len),
                ) {
                    *slot = (actuator.id, actuator.model);
                    len += 1;
                }
            }
            let rest = rest.get(..len).unwrap_or_default();
            // DECISION: each pass retries as the first setup does, so one
            // lost reply never leaves a present actuator out of the run;
            // the plan's one attempt per pass would.
            answer = if rest.is_empty() {
                Ok(())
            } else {
                self.declare_tries(bus, rest)
            };
        }
        self.report_slots(absent, Event::Start);
        answer
    }

    /// Under `Tolerate` at exit: one torque off for each actuator of
    /// `unheld`, whose bus holds it no more; one `Unreached` line per bus
    /// names those that did not answer. One that answered is held alone,
    /// so its limits are put back then; those of one that did not are
    /// forgotten with a warning.
    // DECISION: an unheld actuator that still answers (refused, or lost then
    // left out by a Hold) is restored after its own declare-alone torque
    // off; held ones were all restored before, so no wipe precedes theirs.
    pub(super) fn cut_unheld(&mut self, unheld: Mask<N>) {
        let mut unreached = Mask::NONE;
        for index in unheld.iter() {
            let Some(&actuator) = self.actuators.get(index) else {
                continue;
            };
            let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
            if self.backend.force_torque_off(bus, model, id).is_ok() {
                self.restore_one(index);
                continue;
            }
            unreached.set(index);
            let saved = self.saved.get_mut(index).map(core::mem::take);
            if saved.is_some_and(|saved| !saved.is_empty()) {
                warn!(
                    bus = self.label(bus),
                    "limits of actuator id {id} not restored: not reachable"
                );
            }
        }
        self.report_slots(unreached, Event::Unreached);
    }

    /// Under `Tolerate`, the owned actuators whose bus holds them no more;
    /// none under `Fail`.
    pub(super) fn unheld(&self) -> Mask<N> {
        let mut unheld = Mask::NONE;
        if !self.presence.tolerates() {
            return unheld;
        }
        for (index, actuator) in self.actuators.iter() {
            if !self
                .backend
                .holds(actuator.bus, actuator.model, actuator.id)
            {
                unheld.set(index);
            }
        }
        unheld
    }

    /// Whether the exit path leaves actuator `index` to `cut_unheld`: under
    /// `Tolerate`, one its bus holds no more. A torque off to it would set
    /// it up alone and wipe what the bus holds before the limits are put
    /// back.
    pub(super) fn left_to_close(&self, index: usize) -> bool {
        self.presence.tolerates()
            && self.actuators.get(index).is_some_and(|actuator| {
                !self
                    .backend
                    .holds(actuator.bus, actuator.model, actuator.id)
            })
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
