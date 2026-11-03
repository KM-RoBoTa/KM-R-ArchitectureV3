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

/// Words of a [`Mask`]: the one place the actuator limit lives, `64 *
/// MASK_WORDS` (SRS Definitions "Write mask"). Also the length of the kind
/// bit sets (`PRISMATIC`, `CONTINUOUS`) the model emits.
pub const MASK_WORDS: usize = 1;

/// One bit per actuator, `N` actuators. N above `64 * MASK_WORDS` is refused
/// by [`Mask::CHECK`].
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct Mask<const N: usize = JOINTS>([u64; MASK_WORDS]);

impl<const N: usize> Mask<N> {
    /// Fails the build when `N` does not fit the mask. Post-monomorphization:
    /// it fires under `cargo build`, once named (`Robot::new` names it).
    pub const CHECK: () = assert!(
        N <= 64 * MASK_WORDS,
        "SRS write mask: more than 64 actuators"
    );

    /// No actuator.
    pub const NONE: Self = Self([0; MASK_WORDS]);

    /// Every actuator, `0..N`.
    pub const ALL: Self = {
        let mut words = [0; MASK_WORDS];
        let mut word = 0;
        while word < MASK_WORDS {
            let start = word * 64;
            if N >= start + 64 {
                words[word] = u64::MAX;
            } else if N > start {
                words[word] = (1 << (N - start)) - 1;
            }
            word += 1;
        }
        Self(words)
    };

    /// Sets the bit of `index`. An index past `N` is ignored.
    pub fn set(&mut self, index: usize) {
        if index >= N {
            return;
        }
        if let Some(word) = self.0.get_mut(index / 64) {
            *word |= 1 << (index % 64);
        }
    }

    /// Clears the bit of `index`. An index past `N` is ignored.
    pub fn clear(&mut self, index: usize) {
        if let Some(word) = self.0.get_mut(index / 64) {
            *word &= !(1 << (index % 64));
        }
    }

    /// Whether the bit of `index` is set. `false` past `N`.
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
        (0..N).filter(move |&index| mask.get(index))
    }
}

/// Actuators an adapter can command in one transaction. Opaque to the engine.
#[derive(Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Group(pub u8);

/// The whole robot's hardware, merged once at run() finalization from the
/// members' claims.
#[derive(Debug)]
pub struct Layout<const N: usize = JOINTS> {
    /// Pre-sort key of each actuator, robot-wide (SRS FR-5.3).
    pub group: [Group; N],
    /// Read cadence in force for each actuator, in ticks.
    pub read_every: [NonZeroU32; N],
    /// Estimated bus time of one full read.
    pub read_cost: Duration,
    /// Estimated bus time of one full write.
    pub write_cost: Duration,
    order: [usize; N],
    /// Actuators of each member, by position.
    owned: [Mask<N>; N],
    /// Member k has the groups `ends[k - 1]..ends[k]`, from 0 for the first.
    ends: [u16; N],
    members: usize,
}

impl<const N: usize> Layout<N> {
    /// The layout of one member owning every actuator, sorted by group once.
    pub fn new(
        group: [Group; N],
        read_every: [NonZeroU32; N],
        read_cost: Duration,
        write_cost: Duration,
    ) -> Self {
        let groups = group.iter().map(|g| u16::from(g.0) + 1).max();
        let mut ends = [0; N];
        if let Some(end) = ends.first_mut() {
            *end = groups.unwrap_or(0);
        }
        let mut owned = [Mask::NONE; N];
        if let Some(all) = owned.first_mut() {
            *all = Mask::ALL;
        }
        Self::merged(group, read_every, (read_cost, write_cost), owned, ends, 1)
    }

    /// The layout of `members` members, sorted by group once.
    pub(crate) fn merged(
        group: [Group; N],
        read_every: [NonZeroU32; N],
        (read_cost, write_cost): (Duration, Duration),
        owned: [Mask<N>; N],
        ends: [u16; N],
        members: usize,
    ) -> Self {
        let mut order: [usize; N] = core::array::from_fn(|index| index);
        #[cfg(not(feature = "presort-off"))]
        order.sort_unstable_by_key(|&index| (group.get(index).copied(), index));
        // DECISION: `presort-off` keeps declaration order inside each member
        // only: the walk hands each member a contiguous part of the order.
        #[cfg(feature = "presort-off")]
        order.sort_unstable_by_key(|&index| {
            let of = group.get(index).map_or(0, |g| u16::from(g.0));
            let ends = ends.get(..members).unwrap_or(&[]);
            (ends.partition_point(|&end| end <= of), index)
        });
        Self {
            group,
            read_every,
            read_cost,
            write_cost,
            order,
            owned,
            ends,
            members,
        }
    }

    /// Each member's actuators and groups, in declaration order.
    pub(crate) fn slots(&self) -> Slots<'_, N> {
        Slots {
            owned: self.owned.get(..self.members).unwrap_or(&[]),
            ends: self.ends.get(..self.members).unwrap_or(&[]),
            start: 0,
        }
    }
}

/// A cursor over the members' actuators and groups, as the walk takes them.
#[doc(hidden)]
#[derive(Clone, Copy, Debug)]
pub struct Slots<'a, const N: usize = JOINTS> {
    owned: &'a [Mask<N>],
    ends: &'a [u16],
    /// First group of the next member.
    start: u16,
}

impl<'a, const N: usize> Slots<'a, N> {
    /// The first member's actuators and groups, and the rest.
    pub(crate) fn split(self) -> (Mask<N>, Range<u16>, Self) {
        let (owned, rest) = match self.owned {
            [first, rest @ ..] => (*first, rest),
            [] => (Mask::NONE, self.owned),
        };
        let (groups, ends) = match self.ends {
            [end, ends @ ..] => (self.start..*end, ends),
            [] => (self.start..self.start, self.ends),
        };
        let start = groups.end;
        let rest = Self {
            owned: rest,
            ends,
            start,
        };
        (owned, groups, rest)
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
pub struct Written<const N: usize = JOINTS> {
    /// Actuators whose `Q` was written.
    pub q: Mask<N>,
    /// Actuators whose `Qd` was written.
    pub qd: Mask<N>,
    /// Actuators whose `Tau` was written.
    pub tau: Mask<N>,
    /// Actuators whose `Kp` was written.
    pub kp: Mask<N>,
    /// Actuators whose `Kd` was written.
    pub kd: Mask<N>,
}

impl<const N: usize> Written<N> {
    /// Every field of every actuator.
    pub const ALL: Self = Self {
        q: Mask::ALL,
        qd: Mask::ALL,
        tau: Mask::ALL,
        kp: Mask::ALL,
        kd: Mask::ALL,
    };

    /// Actuators with at least one field written.
    pub fn any(&self) -> Mask<N> {
        self.q
            .union(self.qd)
            .union(self.tau)
            .union(self.kp)
            .union(self.kd)
    }

    /// The writes of the actuators of `owned` only.
    pub(crate) fn within(&self, owned: Mask<N>) -> Self {
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
pub struct Command<'a, const N: usize = JOINTS> {
    /// Desired positions.
    pub q: &'a [Q; N],
    /// Desired velocities.
    pub qd: &'a [Qd; N],
    /// Desired torques.
    pub tau: &'a [Tau; N],
    /// Desired position gains.
    pub kp: &'a [Kp; N],
    /// Desired velocity gains.
    pub kd: &'a [Kd; N],
    /// Fields written this tick.
    pub written: &'a Written<N>,
}

impl<'a, const N: usize> Command<'a, N> {
    /// Borrows the values to send.
    pub fn new(
        q: &'a [Q; N],
        qd: &'a [Qd; N],
        tau: &'a [Tau; N],
        kp: &'a [Kp; N],
        kd: &'a [Kd; N],
        written: &'a Written<N>,
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
pub(crate) struct HoldTarget<const N: usize = JOINTS> {
    pub(crate) q: [Q; N],
    pub(crate) qd: [Qd; N],
    pub(crate) tau: [Tau; N],
    pub(crate) kp: [Kp; N],
    pub(crate) kd: [Kd; N],
    pub(crate) written: Written<N>,
}

impl<const N: usize> HoldTarget<N> {
    /// Borrows the target as the command `hold` takes.
    pub(crate) fn command(&self) -> Command<'_, N> {
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
pub struct Sample<'a, const N: usize = JOINTS> {
    /// Read positions.
    pub q: &'a mut [Q; N],
    /// Read velocities.
    pub qd: &'a mut [Qd; N],
    /// Read torques.
    pub tau: &'a mut [Tau; N],
    /// Position gains the adapter last applied; untouched by those without.
    pub kp: &'a mut [Kp; N],
    /// Velocity gains the adapter last applied; untouched by those without.
    pub kd: &'a mut [Kd; N],
}

impl<'a, const N: usize> Sample<'a, N> {
    /// Borrows the slot to fill.
    pub fn new(
        q: &'a mut [Q; N],
        qd: &'a mut [Qd; N],
        tau: &'a mut [Tau; N],
        kp: &'a mut [Kp; N],
        kd: &'a mut [Kd; N],
    ) -> Self {
        Self { q, qd, tau, kp, kd }
    }

    /// Lends the same slot again, to the next member.
    pub(crate) fn reborrow(&mut self) -> Sample<'_, N> {
        Sample::new(self.q, self.qd, self.tau, self.kp, self.kd)
    }
}

/// Written actuators, in group order: the whole robot's, or one member's
/// once the walk selected it. With the `presort-off` feature, in declaration
/// order inside each member (SRS FR-5.3, for the bench's comparison).
#[derive(Debug)]
pub struct Batch<'a, const N: usize = JOINTS> {
    layout: &'a Layout<N>,
    index: [usize; N],
    len: usize,
    /// The selected actuators are `index[from..to]`.
    from: usize,
    to: usize,
    /// First group of the selected member: its runs yield local ids.
    base: u8,
}

impl<'a, const N: usize> Batch<'a, N> {
    /// The actuators of `written` with at least one field set, sorted by group.
    pub fn new(layout: &'a Layout<N>, written: &Written<N>) -> Self {
        let any = written.any();
        let mut index = [0; N];
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
    pub fn runs(&self) -> Runs<'_, N> {
        Runs {
            group: &self.layout.group,
            rest: self.index.get(self.from..self.to).unwrap_or(&[]),
            base: self.base,
        }
    }
}

/// Iterator over the runs of a [`Batch`].
#[derive(Debug)]
pub struct Runs<'a, const N: usize = JOINTS> {
    group: &'a [Group; N],
    rest: &'a [usize],
    base: u8,
}

impl<'a, const N: usize> Iterator for Runs<'a, N> {
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
pub struct Faults<const N: usize = JOINTS> {
    /// Read failed: the previous value was kept (SRS FR-2.6).
    pub stale: Mask<N>,
    /// Transaction error on the actuator's bus.
    pub bus_error: Mask<N>,
    /// No answer from the actuator.
    pub offline: Mask<N>,
    /// Written but not sent: torque off, field not taken by its mode, or
    /// value outside the actuator's range.
    pub rejected: Mask<N>,
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
pub trait Hal<const N: usize = JOINTS>: seal::Adapter {
    /// Bring-up and finalization failures. Names the actuator and the port
    /// when there is one.
    type Error: core::error::Error + 'static;

    /// run() finalization, no I/O: validates the configuration, applies the
    /// overrides, claims the member's actuators.
    fn check(&mut self, overrides: Overrides) -> Result<Claim<N>, Self::Error>;

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
    fn read_all(&mut self, sample: Sample<'_, N>) -> Result<(), Self::Error>;

    /// PostInit: torque on where the adapter configuration says so.
    fn torque_on(&mut self) -> Result<(), Self::Error>;

    /// Tick, once, before `First`: reads the actuators due this tick and
    /// returns the ones refreshed. No allocation, lock, sleep or retry.
    fn read(&mut self, sample: Sample<'_, N>, faults: &mut Faults<N>) -> Mask<N>;

    /// Tick, once, after `Last`: sends `command` for the actuators of
    /// `batch`, one transaction per run where the protocol allows it. No
    /// allocation, lock, sleep or retry.
    fn write(&mut self, command: Command<'_, N>, batch: &Batch<'_, N>, faults: &mut Faults<N>);

    /// Shutdown: holds each actuator per its mode. `written` marks values
    /// from this tick's command; the rest is the latest sample. Best effort,
    /// never fails.
    fn hold(&mut self, target: Command<'_, N>);

    /// Shutdown, emergency stop, bring-up error: torque off everywhere. Best
    /// effort, never fails.
    fn torque_off(&mut self);

    /// Last call: flushes and closes every port. Best effort, never fails.
    fn close(&mut self);
}

impl<A: seal::Adapter> seal::Adapter for &mut A {}

/// Lends the adapter to a run, so the caller keeps it before and after
/// (SRS FR-5.9).
impl<const N: usize, A: Hal<N>> Hal<N> for &mut A {
    type Error = A::Error;

    fn check(&mut self, overrides: Overrides) -> Result<Claim<N>, Self::Error> {
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

    fn read_all(&mut self, sample: Sample<'_, N>) -> Result<(), Self::Error> {
        (**self).read_all(sample)
    }

    fn torque_on(&mut self) -> Result<(), Self::Error> {
        (**self).torque_on()
    }

    fn read(&mut self, sample: Sample<'_, N>, faults: &mut Faults<N>) -> Mask<N> {
        (**self).read(sample, faults)
    }

    fn write(&mut self, command: Command<'_, N>, batch: &Batch<'_, N>, faults: &mut Faults<N>) {
        (**self).write(command, batch, faults);
    }

    fn hold(&mut self, target: Command<'_, N>) {
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

impl<const N: usize> Hal<N> for NoHardware {
    type Error = Infallible;

    fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, Infallible> {
        Ok(Claim {
            owned: Mask::ALL,
            group: [Group(0); N],
            read_every: [NonZeroU32::MIN; N],
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

    fn read_all(&mut self, _sample: Sample<'_, N>) -> Result<(), Infallible> {
        Ok(())
    }

    fn torque_on(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    fn read(&mut self, _sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
        Mask::NONE
    }

    fn write(&mut self, _command: Command<'_, N>, _batch: &Batch<'_, N>, _faults: &mut Faults<N>) {}

    fn hold(&mut self, _target: Command<'_, N>) {}

    fn torque_off(&mut self) {}

    fn close(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    const N: usize = 4;

    fn layout(group: impl Fn(usize) -> u8) -> Layout<N> {
        Layout::new(
            core::array::from_fn(|index| Group(group(index))),
            [NonZeroU32::MIN; N],
            Duration::ZERO,
            Duration::ZERO,
        )
    }

    #[test]
    fn all_sets_exactly_the_actuators() {
        assert_eq!(Mask::<N>::ALL.iter().count(), N);
        assert!(!Mask::<N>::ALL.get(N));
    }

    #[test]
    fn all_fills_one_word_from_one_to_sixty_four_actuators() {
        assert!(Mask::<1>::ALL.iter().eq([0]));
        assert_eq!(Mask::<10>::ALL.iter().count(), 10);
        assert_eq!(Mask::<64>::ALL.iter().count(), 64);
        assert!(Mask::<64>::ALL.get(63));
        assert!(Mask::<0>::ALL.is_empty());
    }

    #[test]
    fn out_of_range_bits_are_ignored() {
        let mut mask = Mask::<N>::NONE;
        mask.set(N);
        mask.clear(N + 64);
        assert!(mask.is_empty());
    }

    #[cfg(feature = "presort-off")]
    #[test]
    fn presort_off_keeps_declaration_order() {
        let layout = layout(|index| u8::from(index % 2 == 0));
        let mut written = Written::<N>::default();
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
        let mut written = Written::<N>::default();
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
        let batch = Batch::new(&layout, &Written::<N>::default());
        assert!(batch.is_empty());
        assert_eq!(batch.runs().next(), None);
    }
}
