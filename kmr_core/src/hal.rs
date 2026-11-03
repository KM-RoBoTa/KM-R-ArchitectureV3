//! The hardware contract: what the engine calls, what an adapter implements.
//!
//! The engine knows actuator indices, masks, opaque transaction groups and a
//! read cadence in ticks. Buses, series, protocols and modes stay in the
//! adapter (SRS FR-5.1, section 7).

use core::convert::Infallible;
use core::num::NonZeroU32;
use core::ops::Range;
use core::time::Duration;

use tracing::warn;

use crate::state::{JOINTS, Kd, Kp, Q, Qd, Tau};

mod set;

pub use set::{Audit, Claim, Coverage, Drivers, HardwareError, Member, MemberError, Plan, Step};

/// Seal of [`Hal`]: open to adapter crates, closed to controllers, which
/// cannot depend on `kmr_core`.
#[doc(hidden)]
pub mod seal {
    /// Implemented next to [`super::Hal`] by every adapter.
    pub trait Adapter {}
}

const WORDS: usize = JOINTS.div_ceil(64);

/// One bit per actuator.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct Mask([u64; WORDS]);

impl Mask {
    /// No actuator.
    pub const NONE: Self = Self([0; WORDS]);

    /// Every actuator, `0..JOINTS`.
    pub const ALL: Self = {
        let mut words = [u64::MAX; WORDS];
        if !JOINTS.is_multiple_of(64) {
            words[WORDS - 1] = (1 << (JOINTS % 64)) - 1;
        }
        Self(words)
    };

    /// Sets the bit of `index`. An index past `JOINTS` is ignored.
    pub fn set(&mut self, index: usize) {
        if index >= JOINTS {
            return;
        }
        if let Some(word) = self.0.get_mut(index / 64) {
            *word |= 1 << (index % 64);
        }
    }

    /// Clears the bit of `index`. An index past `JOINTS` is ignored.
    pub fn clear(&mut self, index: usize) {
        if let Some(word) = self.0.get_mut(index / 64) {
            *word &= !(1 << (index % 64));
        }
    }

    /// Whether the bit of `index` is set. `false` past `JOINTS`.
    pub fn get(&self, index: usize) -> bool {
        self.0
            .get(index / 64)
            .is_some_and(|word| word & (1 << (index % 64)) != 0)
    }

    /// Whether no bit is set.
    pub fn is_empty(&self) -> bool {
        self.0.iter().all(|&word| word == 0)
    }

    /// Bits set in `self` or in `other`.
    pub fn union(self, other: Self) -> Self {
        let mut words = self.0;
        for (word, theirs) in words.iter_mut().zip(other.0) {
            *word |= theirs;
        }
        Self(words)
    }

    /// Bits set in both `self` and `other`.
    pub fn intersection(self, other: Self) -> Self {
        let mut words = self.0;
        for (word, theirs) in words.iter_mut().zip(other.0) {
            *word &= theirs;
        }
        Self(words)
    }

    /// Bits set in `self` and not in `other`.
    pub fn difference(self, other: Self) -> Self {
        let mut words = self.0;
        for (word, theirs) in words.iter_mut().zip(other.0) {
            *word &= !theirs;
        }
        Self(words)
    }

    /// Indices whose bit is set, ascending.
    pub fn iter(&self) -> impl Iterator<Item = usize> {
        let mask = *self;
        (0..JOINTS).filter(move |&index| mask.get(index))
    }
}

/// Actuators an adapter can command in one transaction. Opaque to the engine.
#[derive(Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Group(pub u8);

/// The whole robot's hardware, merged once at run() finalization from the
/// members' claims.
#[derive(Debug)]
pub struct Layout {
    /// Pre-sort key of each actuator, robot-wide (SRS FR-5.3).
    pub group: [Group; JOINTS],
    /// Read cadence in force for each actuator, in ticks.
    pub read_every: [NonZeroU32; JOINTS],
    /// Estimated bus time of one full read.
    pub read_cost: Duration,
    /// Estimated bus time of one full write.
    pub write_cost: Duration,
    order: [usize; JOINTS],
    /// Actuators of each member, by position.
    owned: [Mask; JOINTS],
    /// Member k has the groups `bound[k]..bound[k + 1]`.
    bound: [u16; JOINTS + 1],
    members: usize,
}

impl Layout {
    /// The layout of one member owning every actuator, sorted by group once.
    pub fn new(
        group: [Group; JOINTS],
        read_every: [NonZeroU32; JOINTS],
        read_cost: Duration,
        write_cost: Duration,
    ) -> Self {
        let groups = group.iter().map(|g| u16::from(g.0) + 1).max();
        let mut bound = [0; JOINTS + 1];
        if let Some(end) = bound.get_mut(1) {
            *end = groups.unwrap_or(0);
        }
        let mut owned = [Mask::NONE; JOINTS];
        if let Some(all) = owned.first_mut() {
            *all = Mask::ALL;
        }
        Self::merged(group, read_every, (read_cost, write_cost), owned, bound, 1)
    }

    /// The layout of `members` members, sorted by group once.
    pub(crate) fn merged(
        group: [Group; JOINTS],
        read_every: [NonZeroU32; JOINTS],
        (read_cost, write_cost): (Duration, Duration),
        owned: [Mask; JOINTS],
        bound: [u16; JOINTS + 1],
        members: usize,
    ) -> Self {
        let mut order: [usize; JOINTS] = core::array::from_fn(|index| index);
        #[cfg(not(feature = "presort-off"))]
        order.sort_unstable_by_key(|&index| (group.get(index).copied(), index));
        // DECISION: `presort-off` keeps declaration order inside each member
        // only: the walk hands each member a contiguous part of the order.
        #[cfg(feature = "presort-off")]
        order.sort_unstable_by_key(|&index| {
            let of = group.get(index).map_or(0, |g| u16::from(g.0));
            let ends = bound.get(1..=members).unwrap_or(&[]);
            (ends.partition_point(|&end| end <= of), index)
        });
        Self {
            group,
            read_every,
            read_cost,
            write_cost,
            order,
            owned,
            bound,
            members,
        }
    }

    /// Each member's actuators and groups, in declaration order.
    pub(crate) fn slots(&self) -> Slots<'_> {
        Slots {
            owned: self.owned.get(..self.members).unwrap_or(&[]),
            bound: self.bound.get(..=self.members).unwrap_or(&[]),
        }
    }
}

/// A cursor over the members' actuators and groups, as the walk takes them.
#[doc(hidden)]
#[derive(Clone, Copy, Debug)]
pub struct Slots<'a> {
    owned: &'a [Mask],
    bound: &'a [u16],
}

impl<'a> Slots<'a> {
    /// The first member's actuators and groups, and the rest.
    pub(crate) fn split(self) -> (Mask, Range<u16>, Self) {
        let (owned, rest) = match self.owned {
            [first, rest @ ..] => (*first, rest),
            [] => (Mask::NONE, self.owned),
        };
        let groups = match self.bound {
            [start, end, ..] => *start..*end,
            _ => 0..0,
        };
        let bound = self.bound.get(1..).unwrap_or(&[]);
        (owned, groups, Self { owned: rest, bound })
    }
}

/// Settings the control engineer may give through the API, forwarded untouched.
///
/// Empty: the read cadence is the adapter's, read back through
/// [`crate::RobotState::cadence`], with no builder knob (SRS FR-5.3).
#[non_exhaustive]
#[derive(Clone, Copy, Default, Debug)]
pub struct Overrides {}

/// Which desired fields were written this tick, one mask per field.
#[non_exhaustive]
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct Written {
    /// Actuators whose `Q` was written.
    pub q: Mask,
    /// Actuators whose `Qd` was written.
    pub qd: Mask,
    /// Actuators whose `Tau` was written.
    pub tau: Mask,
    /// Actuators whose `Kp` was written.
    pub kp: Mask,
    /// Actuators whose `Kd` was written.
    pub kd: Mask,
}

impl Written {
    /// Every field of every actuator.
    pub const ALL: Self = Self {
        q: Mask::ALL,
        qd: Mask::ALL,
        tau: Mask::ALL,
        kp: Mask::ALL,
        kd: Mask::ALL,
    };

    /// Actuators with at least one field written.
    pub fn any(&self) -> Mask {
        self.q
            .union(self.qd)
            .union(self.tau)
            .union(self.kp)
            .union(self.kd)
    }

    /// The writes of the actuators of `owned` only.
    pub(crate) fn within(&self, owned: Mask) -> Self {
        Self {
            q: self.q.intersection(owned),
            qd: self.qd.intersection(owned),
            tau: self.tau.intersection(owned),
            kp: self.kp.intersection(owned),
            kd: self.kd.intersection(owned),
        }
    }
}

/// Values the adapter sends: this tick's desired state, or the hold target.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
pub struct Command<'a> {
    /// Desired positions.
    pub q: &'a [Q; JOINTS],
    /// Desired velocities.
    pub qd: &'a [Qd; JOINTS],
    /// Desired torques.
    pub tau: &'a [Tau; JOINTS],
    /// Desired position gains.
    pub kp: &'a [Kp; JOINTS],
    /// Desired velocity gains.
    pub kd: &'a [Kd; JOINTS],
    /// Fields written this tick.
    pub written: &'a Written,
}

impl<'a> Command<'a> {
    /// Borrows the values to send.
    pub fn new(
        q: &'a [Q; JOINTS],
        qd: &'a [Qd; JOINTS],
        tau: &'a [Tau; JOINTS],
        kp: &'a [Kp; JOINTS],
        kd: &'a [Kd; JOINTS],
        written: &'a Written,
    ) -> Self {
        Self {
            q,
            qd,
            tau,
            kp,
            kd,
            written,
        }
    }
}

/// Owned values of the shutdown hold: the latest sample overlaid with the
/// tick's writes, which `written` marks (SRS FR-4.1).
#[derive(Clone, Copy, Debug)]
pub(crate) struct HoldTarget {
    pub(crate) q: [Q; JOINTS],
    pub(crate) qd: [Qd; JOINTS],
    pub(crate) tau: [Tau; JOINTS],
    pub(crate) kp: [Kp; JOINTS],
    pub(crate) kd: [Kd; JOINTS],
    pub(crate) written: Written,
}

impl HoldTarget {
    /// Borrows the target as the command `hold` takes.
    pub(crate) fn command(&self) -> Command<'_> {
        Command::new(
            &self.q,
            &self.qd,
            &self.tau,
            &self.kp,
            &self.kd,
            &self.written,
        )
    }
}

/// The newest history slot, a copy of the previous one; the adapter
/// overwrites what it reads.
#[non_exhaustive]
#[derive(Debug)]
pub struct Sample<'a> {
    /// Read positions.
    pub q: &'a mut [Q; JOINTS],
    /// Read velocities.
    pub qd: &'a mut [Qd; JOINTS],
    /// Read torques.
    pub tau: &'a mut [Tau; JOINTS],
    /// Position gains the adapter last applied; untouched by those without.
    pub kp: &'a mut [Kp; JOINTS],
    /// Velocity gains the adapter last applied; untouched by those without.
    pub kd: &'a mut [Kd; JOINTS],
}

impl<'a> Sample<'a> {
    /// Borrows the slot to fill.
    pub fn new(
        q: &'a mut [Q; JOINTS],
        qd: &'a mut [Qd; JOINTS],
        tau: &'a mut [Tau; JOINTS],
        kp: &'a mut [Kp; JOINTS],
        kd: &'a mut [Kd; JOINTS],
    ) -> Self {
        Self { q, qd, tau, kp, kd }
    }

    /// Lends the same slot again, to the next member.
    pub(crate) fn reborrow(&mut self) -> Sample<'_> {
        Sample::new(self.q, self.qd, self.tau, self.kp, self.kd)
    }
}

/// Written actuators, in group order: the whole robot's, or one member's
/// once the walk selected it. With the `presort-off` feature, in declaration
/// order inside each member (SRS FR-5.3, for the bench's comparison).
#[derive(Debug)]
pub struct Batch<'a> {
    layout: &'a Layout,
    index: [usize; JOINTS],
    len: usize,
    /// The selected actuators are `index[from..to]`.
    from: usize,
    to: usize,
    /// First group of the selected member: its runs yield local ids.
    base: u8,
}

impl<'a> Batch<'a> {
    /// The actuators of `written` with at least one field set, sorted by group.
    pub fn new(layout: &'a Layout, written: &Written) -> Self {
        let any = written.any();
        let mut index = [0; JOINTS];
        let mut len = 0;
        for &actuator in &layout.order {
            if any.get(actuator)
                && let Some(slot) = index.get_mut(len)
            {
                *slot = actuator;
                len += 1;
            }
        }
        Self {
            layout,
            index,
            len,
            from: 0,
            to: len,
            base: 0,
        }
    }

    /// Selects the actuators of the robot-wide `groups`, one member's.
    pub(crate) fn select(&mut self, groups: Range<u16>) {
        let all = self.index.get(..self.len).unwrap_or(&[]);
        let of = |actuator: &usize| {
            let group = self
                .layout
                .group
                .get(*actuator)
                .copied()
                .unwrap_or_default();
            u16::from(group.0)
        };
        let from = all.partition_point(|actuator| of(actuator) < groups.start);
        let to = all.partition_point(|actuator| of(actuator) < groups.end);
        self.from = from;
        self.to = to;
        self.base = u8::try_from(groups.start).unwrap_or(u8::MAX);
    }

    /// Whether no actuator was written.
    pub fn is_empty(&self) -> bool {
        self.from == self.to
    }

    /// One run per group, groups ascending, actuators ascending inside a run.
    pub fn runs(&self) -> Runs<'_> {
        Runs {
            group: &self.layout.group,
            rest: self.index.get(self.from..self.to).unwrap_or(&[]),
            base: self.base,
        }
    }
}

/// Iterator over the runs of a [`Batch`].
#[derive(Debug)]
pub struct Runs<'a> {
    group: &'a [Group; JOINTS],
    rest: &'a [usize],
    base: u8,
}

impl<'a> Iterator for Runs<'a> {
    type Item = (Group, &'a [usize]);

    fn next(&mut self) -> Option<Self::Item> {
        let group = self.group;
        let of = |actuator: usize| group.get(actuator).copied();
        let first = of(*self.rest.first()?);
        let len = self
            .rest
            .iter()
            .take_while(|&&actuator| of(actuator) == first)
            .count();
        let (run, rest) = self.rest.split_at_checked(len)?;
        self.rest = rest;
        let local = first.unwrap_or_default().0.saturating_sub(self.base);
        Some((Group(local), run))
    }
}

/// Per-tick fault bits set by the adapter (SRS FR-13.4).
#[non_exhaustive]
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct Faults {
    /// Read failed: the previous value was kept (SRS FR-2.6).
    pub stale: Mask,
    /// Transaction error on the actuator's bus.
    pub bus_error: Mask,
    /// No answer from the actuator.
    pub offline: Mask,
    /// Written but not sent: torque off, field not taken by its mode, or
    /// value outside the actuator's range.
    pub rejected: Mask,
}

/// The hardware contract of one member of the robot's hardware (SRS FR-5.1,
/// section 7): the engine calls it for the actuators it claims. Static
/// dispatch only.
///
/// The engine calls the operations in the order they are declared. Once
/// `open` was called, every exit ends with `torque_off` then `close`.
///
/// The engine restricts `write` to the member's runs, with local group ids,
/// and the `written` masks of `write` and `hold` to its actuators. `Sample`
/// and `Faults` stay whole-robot: a member touches only its own slots.
pub trait Hal: seal::Adapter {
    /// Bring-up and finalization failures. Names the actuator and the port
    /// when there is one.
    type Error: core::error::Error + 'static;

    /// run() finalization, no I/O: validates the configuration, applies the
    /// overrides, claims the member's actuators.
    fn check(&mut self, overrides: Overrides) -> Result<Claim, Self::Error>;

    /// Init: resolves every port, then flushes, sets the latency timer and
    /// opens each one. Nothing is powered.
    fn open(&mut self) -> Result<(), Self::Error>;

    /// Init: brand-specific zeroing, a no-op where the brand needs none.
    fn zero(&mut self) -> Result<(), Self::Error>;

    /// PostInit: every actuator answers.
    fn ping(&mut self) -> Result<(), Self::Error>;

    /// PostInit: error status, voltage and temperature within limits.
    fn health(&mut self) -> Result<(), Self::Error>;

    /// PostInit: operating modes from the adapter configuration.
    fn configure(&mut self) -> Result<(), Self::Error>;

    /// PostInit: one full read of every actuator, whatever the cadence.
    fn read_all(&mut self, sample: Sample<'_>) -> Result<(), Self::Error>;

    /// PostInit: torque on where the adapter configuration says so.
    fn torque_on(&mut self) -> Result<(), Self::Error>;

    /// Tick, once, before `First`: reads the actuators due this tick and
    /// returns the ones refreshed. No allocation, lock, sleep or retry.
    fn read(&mut self, sample: Sample<'_>, faults: &mut Faults) -> Mask;

    /// Tick, once, after `Last`: sends `command` for the actuators of
    /// `batch`, one transaction per run where the protocol allows it. No
    /// allocation, lock, sleep or retry.
    fn write(&mut self, command: Command<'_>, batch: &Batch<'_>, faults: &mut Faults);

    /// Shutdown: holds each actuator per its mode. `written` marks values
    /// from this tick's command; the rest is the latest sample. Best effort,
    /// never fails.
    fn hold(&mut self, target: Command<'_>);

    /// Shutdown, emergency stop, bring-up error: torque off everywhere. Best
    /// effort, never fails.
    fn torque_off(&mut self);

    /// Last call: flushes and closes every port. Best effort, never fails.
    fn close(&mut self);
}

impl<A: Hal> seal::Adapter for &mut A {}

/// Lends the adapter to a run, so the caller keeps it before and after
/// (SRS FR-5.9).
impl<A: Hal> Hal for &mut A {
    type Error = A::Error;

    fn check(&mut self, overrides: Overrides) -> Result<Claim, Self::Error> {
        (**self).check(overrides)
    }

    fn open(&mut self) -> Result<(), Self::Error> {
        (**self).open()
    }

    fn zero(&mut self) -> Result<(), Self::Error> {
        (**self).zero()
    }

    fn ping(&mut self) -> Result<(), Self::Error> {
        (**self).ping()
    }

    fn health(&mut self) -> Result<(), Self::Error> {
        (**self).health()
    }

    fn configure(&mut self) -> Result<(), Self::Error> {
        (**self).configure()
    }

    fn read_all(&mut self, sample: Sample<'_>) -> Result<(), Self::Error> {
        (**self).read_all(sample)
    }

    fn torque_on(&mut self) -> Result<(), Self::Error> {
        (**self).torque_on()
    }

    fn read(&mut self, sample: Sample<'_>, faults: &mut Faults) -> Mask {
        (**self).read(sample, faults)
    }

    fn write(&mut self, command: Command<'_>, batch: &Batch<'_>, faults: &mut Faults) {
        (**self).write(command, batch, faults);
    }

    fn hold(&mut self, target: Command<'_>) {
        (**self).hold(target);
    }

    fn torque_off(&mut self) {
        (**self).torque_off();
    }

    fn close(&mut self) {
        (**self).close();
    }
}

/// The member used when no adapter is compiled in: owns every actuator,
/// reads and writes nothing.
#[derive(Clone, Copy, Default, Debug)]
pub struct NoHardware;

impl seal::Adapter for NoHardware {}

impl Hal for NoHardware {
    type Error = Infallible;

    fn check(&mut self, _overrides: Overrides) -> Result<Claim, Infallible> {
        Ok(Claim {
            owned: Mask::ALL,
            group: [Group(0); JOINTS],
            read_every: [NonZeroU32::MIN; JOINTS],
            read_cost: Duration::ZERO,
            write_cost: Duration::ZERO,
        })
    }

    fn open(&mut self) -> Result<(), Infallible> {
        warn!("no bus plugin: simulating");
        Ok(())
    }

    fn zero(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    fn ping(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    fn health(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    fn configure(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    fn read_all(&mut self, _sample: Sample<'_>) -> Result<(), Infallible> {
        Ok(())
    }

    fn torque_on(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    fn read(&mut self, _sample: Sample<'_>, _faults: &mut Faults) -> Mask {
        Mask::NONE
    }

    fn write(&mut self, _command: Command<'_>, _batch: &Batch<'_>, _faults: &mut Faults) {}

    fn hold(&mut self, _target: Command<'_>) {}

    fn torque_off(&mut self) {}

    fn close(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layout(group: impl Fn(usize) -> u8) -> Layout {
        Layout::new(
            core::array::from_fn(|index| Group(group(index))),
            [NonZeroU32::MIN; JOINTS],
            Duration::ZERO,
            Duration::ZERO,
        )
    }

    #[test]
    fn all_sets_exactly_the_actuators() {
        assert_eq!(Mask::ALL.iter().count(), JOINTS);
        assert!(!Mask::ALL.get(JOINTS));
    }

    #[test]
    fn out_of_range_bits_are_ignored() {
        let mut mask = Mask::NONE;
        mask.set(JOINTS);
        mask.clear(JOINTS + 64);
        assert!(mask.is_empty());
    }

    #[cfg(feature = "presort-off")]
    #[test]
    fn presort_off_keeps_declaration_order() {
        let layout = layout(|index| u8::from(index % 2 == 0));
        let mut written = Written::default();
        written.q.set(0);
        written.qd.set(1);
        written.tau.set(2);

        let batch = Batch::new(&layout, &written);
        let mut runs = batch.runs();

        assert_eq!(runs.next(), Some((Group(1), &[0][..])));
        assert_eq!(runs.next(), Some((Group(0), &[1][..])));
        assert_eq!(runs.next(), Some((Group(1), &[2][..])));
        assert_eq!(runs.next(), None);
    }

    #[cfg(not(feature = "presort-off"))]
    #[test]
    fn batch_keeps_written_actuators_in_group_order() {
        let layout = layout(|index| u8::from(index % 2 == 0));
        let mut written = Written::default();
        written.q.set(0);
        written.qd.set(1);
        written.tau.set(2);

        let batch = Batch::new(&layout, &written);
        let mut runs = batch.runs();

        assert_eq!(runs.next(), Some((Group(0), &[1][..])));
        assert_eq!(runs.next(), Some((Group(1), &[0, 2][..])));
        assert_eq!(runs.next(), None);
    }

    #[test]
    fn nothing_written_is_an_empty_batch() {
        let layout = layout(|_| 0);
        let batch = Batch::new(&layout, &Written::default());
        assert!(batch.is_empty());
        assert_eq!(batch.runs().next(), None);
    }
}
