//! The robot's hardware as a list of members, each claiming the actuators it
//! drives (SRS FR-5.1, FR-6.4). The list is a frunk `HCons`, walked head
//! first like the controller buckets: static dispatch, no heap, any length.

use core::any::type_name;
use core::convert::Infallible;
use core::fmt;
use core::num::NonZeroU32;
use core::time::Duration;

use frunk::{HCons, HNil};

use super::{Batch, Command, Faults, Group, Hal, Layout, Mask, Overrides, Sample, Slots};
use crate::state::State;

/// Transaction groups the whole robot may have, `Group` being a `u8`.
const MAX_GROUPS: u16 = 256;

/// What one member declares at run() finalization, no I/O. Entries outside
/// `owned` are ignored.
#[derive(Clone, Debug)]
pub struct Claim<const N: usize> {
    /// Actuators the member drives.
    pub owned: Mask<N>,
    /// Member-local group of each owned actuator; the engine renumbers them
    /// robot-wide.
    pub group: [Group; N],
    /// Read cadence of each owned actuator, in ticks.
    pub read_every: [NonZeroU32; N],
    /// Estimated bus time of one full read of the member.
    pub read_cost: Duration,
    /// Estimated bus time of one full write of the member.
    pub write_cost: Duration,
}

/// A member as errors name it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Member {
    /// Position in the list, from 0. A member has at least one group, so
    /// there are at most `MAX_GROUPS` of them.
    pub position: u8,
    /// Type name of the member.
    pub name: &'static str,
}

/// `member 1 (kmr_adapters::cubemars::CubeMars)`.
impl fmt::Display for Member {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "member {} ({})", self.position, self.name)
    }
}

/// Why the claims do not cover the actuators exactly once.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Coverage {
    /// No member claims the actuator.
    #[error("actuator {actuator} is owned by no member")]
    Unowned {
        /// Engine index.
        actuator: usize,
    },
    /// Two members claim the actuator.
    #[error("actuator {actuator} is owned by {first} and {second}")]
    Overlap {
        /// Engine index.
        actuator: usize,
        /// The earlier member.
        first: Member,
        /// The later member.
        second: Member,
    },
    /// The member claims no actuator.
    #[error("{member} owns no actuator")]
    Idle {
        /// The member.
        member: Member,
    },
    /// The member's groups do not fit after those of the members before it.
    #[error("{member} brings the robot past {MAX_GROUPS} transaction groups")]
    TooManyGroups {
        /// The member.
        member: Member,
    },
}

/// The claims merged so far, in declaration order.
#[doc(hidden)]
#[derive(Debug)]
pub struct Plan<const N: usize> {
    /// Actuators of each member, by position. Each owns one at least, so
    /// there are at most `N` members.
    owned: [Mask<N>; N],
    /// Type name of each member, by position.
    name: [&'static str; N],
    /// Member k has the groups `ends[k - 1]..ends[k]`, from 0 for the first.
    ends: [u16; N],
    members: usize,
    taken: Mask<N>,
    group: [Group; N],
    read_every: [NonZeroU32; N],
    read_cost: Duration,
    write_cost: Duration,
}

impl<const N: usize> Plan<N> {
    /// No claim yet.
    pub(crate) fn new() -> Self {
        Self {
            owned: [Mask::NONE; N],
            name: [""; N],
            ends: [0; N],
            members: 0,
            taken: Mask::NONE,
            group: [Group(0); N],
            read_every: [NonZeroU32::MIN; N],
            read_cost: Duration::ZERO,
            write_cost: Duration::ZERO,
        }
    }

    /// Adds the claim of the next member, of type name `name`, its groups
    /// shifted past those already taken. Costs add up: members run one after
    /// the other. On error the plan is left as it was.
    pub(crate) fn add(&mut self, name: &'static str, claim: &Claim<N>) -> Result<(), Coverage> {
        let base = self.next_group();
        let member = Member {
            position: u8::try_from(self.members).unwrap_or(u8::MAX),
            name,
        };
        let top = claim
            .owned
            .iter()
            .filter_map(|actuator| claim.group.get(actuator))
            .map(|local| local.0)
            .max()
            .ok_or(Coverage::Idle { member })?;
        if let Some(actuator) = claim.owned.intersection(self.taken).iter().next() {
            return Err(Coverage::Overlap {
                actuator,
                first: self.owner_of(actuator),
                second: member,
            });
        }
        let base = u8::try_from(base)
            .ok()
            .filter(|base| base.checked_add(top).is_some())
            .ok_or(Coverage::TooManyGroups { member })?;
        let (Some(owned), Some(slot), Some(end)) = (
            self.owned.get_mut(self.members),
            self.name.get_mut(self.members),
            self.ends.get_mut(self.members),
        ) else {
            return Err(Coverage::TooManyGroups { member });
        };

        *owned = claim.owned;
        *slot = name;
        *end = u16::from(base) + u16::from(top) + 1;
        let rows = self.group.iter_mut().zip(&mut self.read_every);
        let claimed = claim.group.iter().zip(&claim.read_every);
        for (actuator, ((group, every), (local, cadence))) in rows.zip(claimed).enumerate() {
            if claim.owned.get(actuator) {
                *group = Group(base + local.0);
                *every = *cadence;
            }
        }
        self.members += 1;
        self.taken = self.taken.union(claim.owned);
        self.read_cost = self.read_cost.saturating_add(claim.read_cost);
        self.write_cost = self.write_cost.saturating_add(claim.write_cost);
        Ok(())
    }

    /// First group of the next member.
    fn next_group(&self) -> u16 {
        match self.members.checked_sub(1) {
            None => 0,
            Some(last) => self.ends.get(last).copied().unwrap_or(MAX_GROUPS),
        }
    }

    /// The member that took `actuator`.
    fn owner_of(&self, actuator: usize) -> Member {
        let owners = self.owned.iter().zip(self.name).take(self.members);
        let (position, name) = owners
            .enumerate()
            .find_map(|(position, (owned, name))| owned.get(actuator).then_some((position, name)))
            .unwrap_or_default();
        Member {
            position: u8::try_from(position).unwrap_or(u8::MAX),
            name,
        }
    }

    /// The layout of the whole robot, once every member was added.
    pub(crate) fn finish(self) -> Result<Layout<N>, Coverage> {
        if let Some(actuator) = Mask::ALL.difference(self.taken).iter().next() {
            return Err(Coverage::Unowned { actuator });
        }
        Ok(Layout::merged(
            self.group,
            self.read_every,
            (self.read_cost, self.write_cost),
            self.owned,
            self.ends,
            self.members,
        ))
    }
}

/// A bring-up step every member takes, in this order (SRS FR-5.2).
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Init: [`Hal::open`].
    Open,
    /// Init: [`Hal::zero`].
    Zero,
    /// PostInit: [`Hal::ping`].
    Ping,
    /// PostInit: [`Hal::health`].
    Health,
    /// PostInit: [`Hal::configure`].
    Configure,
    /// PostInit: [`Hal::torque_on`].
    TorqueOn,
}

/// A member's error, by position in the list: `Head` is this member's,
/// `Tail` a later one's.
#[derive(Debug, PartialEq)]
pub enum MemberError<E, T> {
    /// The error of this member.
    Head {
        /// Type name of the member.
        name: &'static str,
        /// Its error.
        error: E,
    },
    /// The error of a later member.
    Tail(T),
}

/// `member 1 (kmr_adapters::cubemars::CubeMars): <the member's error>`.
impl<E: core::error::Error + 'static, T: seal::Nested> fmt::Display for MemberError<E, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        seal::Nested::fmt_at(self, f, 0)
    }
}

/// The source is the failed member's own error.
impl<E, T> core::error::Error for MemberError<E, T>
where
    E: core::error::Error + 'static,
    T: seal::Nested,
{
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Head { error, .. } => Some(error),
            Self::Tail(error) => error.source(),
        }
    }
}

impl<E, T> MemberError<E, T> {
    /// The error of member `M`, named by its type, the head of the list.
    pub fn head<M: super::seal::Adapter>(error: E) -> Self {
        Self::Head {
            name: type_name::<M>(),
            error,
        }
    }
}

/// Why the robot's hardware could not run: the claims, or a member.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum HardwareError<E> {
    /// The claims do not cover the actuators exactly once. Nothing was
    /// opened.
    #[error(transparent)]
    Coverage(#[from] Coverage),
    /// A member failed.
    #[error(transparent)]
    Member(E),
}

impl<E> HardwareError<E> {
    /// Maps the member error, the coverage one passing through.
    fn map_member<F>(self, op: impl FnOnce(E) -> F) -> HardwareError<F> {
        match self {
            Self::Coverage(coverage) => HardwareError::Coverage(coverage),
            Self::Member(error) => HardwareError::Member(op(error)),
        }
    }
}

mod seal {
    use core::convert::Infallible;
    use core::fmt;

    use super::MemberError;

    /// Implemented by the engine for `HNil` and `HCons` only.
    pub trait Drivers {}

    /// The list's error, which numbers the failed member by its depth.
    pub trait Nested: core::error::Error + 'static {
        /// Writes the error, this node being member `position`.
        fn fmt_at(&self, f: &mut fmt::Formatter<'_>, position: usize) -> fmt::Result;
    }

    impl Nested for Infallible {
        fn fmt_at(&self, _f: &mut fmt::Formatter<'_>, _position: usize) -> fmt::Result {
            match *self {}
        }
    }

    impl<E: core::error::Error + 'static, T: Nested> Nested for MemberError<E, T> {
        fn fmt_at(&self, f: &mut fmt::Formatter<'_>, position: usize) -> fmt::Result {
            match self {
                Self::Head { name, error } => write!(f, "member {position} ({name}): {error}"),
                Self::Tail(error) => error.fmt_at(f, position + 1),
            }
        }
    }
}

/// Debug builds: the first actuator a member changed without owning it, in
/// its sample slots or its fault bits. Always empty in release builds.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Audit {
    stray: Option<Stray>,
}

/// An actuator changed by a member that does not own it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Stray {
    /// Type name of the member.
    pub(crate) member: &'static str,
    /// Engine index.
    pub(crate) actuator: usize,
}

/// What a member could change, copied before its call. Empty in release
/// builds.
struct Before<const N: usize> {
    #[cfg(debug_assertions)]
    values: Option<State<N>>,
    #[cfg(debug_assertions)]
    faults: Option<Faults<N>>,
}

impl Audit {
    /// The stray change noted, if any.
    pub(crate) fn take(&mut self) -> Option<Stray> {
        self.stray.take()
    }

    /// Copies what the next member call may change.
    fn before<const N: usize>(
        sample: Option<&Sample<'_, N>>,
        faults: Option<&Faults<N>>,
    ) -> Before<N> {
        #[cfg(debug_assertions)]
        return Before {
            values: sample.map(|s| State {
                q: *s.q,
                qd: *s.qd,
                tau: *s.tau,
                kp: *s.kp,
                kd: *s.kd,
            }),
            faults: faults.copied(),
        };
        #[cfg(not(debug_assertions))]
        {
            let _ = (sample, faults);
            Before {}
        }
    }

    /// Notes the first actuator outside `owned` that the member of type name
    /// `member` changed since `before`. The first stray change of the walk
    /// wins.
    fn after<const N: usize>(
        &mut self,
        member: &'static str,
        before: Before<N>,
        sample: Option<&Sample<'_, N>>,
        faults: Option<&Faults<N>>,
        owned: Mask<N>,
    ) {
        #[cfg(debug_assertions)]
        {
            if self.stray.is_some() {
                return;
            }
            let values = before.values.as_ref().zip(sample);
            let fault_bits = before.faults.as_ref().zip(faults);
            let moved = |actuator: usize| {
                let value = values.is_some_and(|(was, now)| {
                    row(&was.q, &was.qd, &was.tau, &was.kp, &was.kd, actuator)
                        != row(&*now.q, &*now.qd, &*now.tau, &*now.kp, &*now.kd, actuator)
                });
                let fault = fault_bits
                    .is_some_and(|(was, now)| fault_row(was, actuator) != fault_row(now, actuator));
                value || fault
            };
            if let Some(actuator) = Mask::ALL.difference(owned).iter().find(|&a| moved(a)) {
                self.stray = Some(Stray { member, actuator });
            }
        }
        #[cfg(not(debug_assertions))]
        let _ = (self, member, before, sample, faults, owned);
    }
}

/// The bits of every field of `actuator`, NaN included.
#[cfg(debug_assertions)]
fn row<const N: usize>(
    q: &[crate::state::Q; N],
    qd: &[crate::state::Qd; N],
    tau: &[crate::state::Tau; N],
    kp: &[crate::state::Kp; N],
    kd: &[crate::state::Kd; N],
    actuator: usize,
) -> Option<[u32; 5]> {
    Some([
        q.get(actuator)?.0.to_bits(),
        qd.get(actuator)?.0.to_bits(),
        tau.get(actuator)?.0.to_bits(),
        kp.get(actuator)?.0.to_bits(),
        kd.get(actuator)?.0.to_bits(),
    ])
}

/// The fault bits of `actuator`.
#[cfg(debug_assertions)]
fn fault_row<const N: usize>(faults: &Faults<N>, actuator: usize) -> [bool; 6] {
    [
        faults.stale.get(actuator),
        faults.bus_error.get(actuator),
        faults.offline.get(actuator),
        faults.rejected.get(actuator),
        faults.joint_limit.get(actuator),
        faults.envelope.get(actuator),
    ]
}

/// The robot's hardware: a list of [`Hal`] members, `HCons<A, HCons<B,
/// HNil>>`. Implemented by the engine only; the walk calls every member,
/// head first, and cuts torque in reverse.
pub trait Drivers<const N: usize>: seal::Drivers {
    /// One variant per member, nested.
    type Error: seal::Nested;

    /// Claims of every member, merged into `plan`.
    #[doc(hidden)]
    fn claim(
        &mut self,
        overrides: Overrides,
        plan: &mut Plan<N>,
    ) -> Result<(), HardwareError<Self::Error>>;

    /// `step` on every member; the first error stops the walk.
    #[doc(hidden)]
    fn step(&mut self, step: Step) -> Result<(), Self::Error>;

    /// [`Hal::read_all`] on every member, into the same slot; the actuators
    /// of every member that does not only simulate.
    #[doc(hidden)]
    fn read_all(
        &mut self,
        sample: Sample<'_, N>,
        slots: Slots<'_, N>,
        audit: &mut Audit,
    ) -> Result<Mask<N>, Self::Error>;

    /// [`Hal::read`] on every member; the refreshed masks, each cut to its
    /// member's actuators, unioned.
    #[doc(hidden)]
    fn read(
        &mut self,
        sample: Sample<'_, N>,
        faults: &mut Faults<N>,
        slots: Slots<'_, N>,
        audit: &mut Audit,
    ) -> Mask<N>;

    /// [`Hal::write`] on every member with a run in `batch`, each given its
    /// own runs and writes.
    #[doc(hidden)]
    fn write(
        &mut self,
        command: Command<'_, N>,
        batch: &mut Batch<'_, N>,
        faults: &mut Faults<N>,
        slots: Slots<'_, N>,
        audit: &mut Audit,
    );

    /// [`Hal::hold`] on every member, each given its own writes.
    #[doc(hidden)]
    fn hold(&mut self, target: Command<'_, N>, slots: Slots<'_, N>);

    /// [`Hal::torque_off`] on every member, last first.
    #[doc(hidden)]
    fn torque_off(&mut self);

    /// [`Hal::close`] on every member, last first.
    #[doc(hidden)]
    fn close(&mut self);
}

impl seal::Drivers for HNil {}

impl<const N: usize> Drivers<N> for HNil {
    type Error = Infallible;

    fn claim(
        &mut self,
        _overrides: Overrides,
        _plan: &mut Plan<N>,
    ) -> Result<(), HardwareError<Infallible>> {
        Ok(())
    }

    fn step(&mut self, _step: Step) -> Result<(), Infallible> {
        Ok(())
    }

    fn read_all(
        &mut self,
        _sample: Sample<'_, N>,
        _slots: Slots<'_, N>,
        _audit: &mut Audit,
    ) -> Result<Mask<N>, Infallible> {
        Ok(Mask::NONE)
    }

    fn read(
        &mut self,
        _sample: Sample<'_, N>,
        _faults: &mut Faults<N>,
        _slots: Slots<'_, N>,
        _audit: &mut Audit,
    ) -> Mask<N> {
        Mask::NONE
    }

    fn write(
        &mut self,
        _command: Command<'_, N>,
        _batch: &mut Batch<'_, N>,
        _faults: &mut Faults<N>,
        _slots: Slots<'_, N>,
        _audit: &mut Audit,
    ) {
    }

    fn hold(&mut self, _target: Command<'_, N>, _slots: Slots<'_, N>) {}

    fn torque_off(&mut self) {}

    fn close(&mut self) {}
}

impl<H: super::seal::Adapter, T: seal::Drivers> seal::Drivers for HCons<H, T> {}

impl<const N: usize, H: Hal<N>, T: Drivers<N>> Drivers<N> for HCons<H, T> {
    type Error = MemberError<H::Error, T::Error>;

    fn claim(
        &mut self,
        overrides: Overrides,
        plan: &mut Plan<N>,
    ) -> Result<(), HardwareError<Self::Error>> {
        let claim = self
            .head
            .check(overrides)
            .map_err(|error| HardwareError::Member(MemberError::head::<H>(error)))?;
        plan.add(type_name::<H>(), &claim)?;
        self.tail
            .claim(overrides, plan)
            .map_err(|error| error.map_member(MemberError::Tail))
    }

    fn step(&mut self, step: Step) -> Result<(), Self::Error> {
        let member = &mut self.head;
        match step {
            Step::Open => member.open(),
            Step::Zero => member.zero(),
            Step::Ping => member.ping(),
            Step::Health => member.health(),
            Step::Configure => member.configure(),
            Step::TorqueOn => member.torque_on(),
        }
        .map_err(MemberError::head::<H>)?;
        self.tail.step(step).map_err(MemberError::Tail)
    }

    fn read_all(
        &mut self,
        mut sample: Sample<'_, N>,
        slots: Slots<'_, N>,
        audit: &mut Audit,
    ) -> Result<Mask<N>, Self::Error> {
        let (owned, _groups, rest) = slots.split();
        let before = Audit::before(Some(&sample), None);
        self.head
            .read_all(sample.reborrow())
            .map_err(MemberError::head::<H>)?;
        audit.after(type_name::<H>(), before, Some(&sample), None, owned);
        let head = if self.head.simulates() {
            Mask::NONE
        } else {
            owned
        };
        let tail = self
            .tail
            .read_all(sample, rest, audit)
            .map_err(MemberError::Tail)?;
        Ok(head.union(tail))
    }

    fn read(
        &mut self,
        mut sample: Sample<'_, N>,
        faults: &mut Faults<N>,
        slots: Slots<'_, N>,
        audit: &mut Audit,
    ) -> Mask<N> {
        let (owned, _groups, rest) = slots.split();
        let before = Audit::before(Some(&sample), Some(faults));
        let head = self
            .head
            .read(sample.reborrow(), faults)
            .intersection(owned);
        audit.after(type_name::<H>(), before, Some(&sample), Some(faults), owned);
        head.union(self.tail.read(sample, faults, rest, audit))
    }

    fn write(
        &mut self,
        command: Command<'_, N>,
        batch: &mut Batch<'_, N>,
        faults: &mut Faults<N>,
        slots: Slots<'_, N>,
        audit: &mut Audit,
    ) {
        let (owned, groups, rest) = slots.split();
        batch.select(groups);
        if !batch.is_empty() {
            let written = command.written.within(owned);
            let mine = Command {
                written: &written,
                ..command
            };
            let before = Audit::before(None, Some(faults));
            self.head.write(mine, batch, faults);
            audit.after(type_name::<H>(), before, None, Some(faults), owned);
        }
        self.tail.write(command, batch, faults, rest, audit);
    }

    fn hold(&mut self, target: Command<'_, N>, slots: Slots<'_, N>) {
        let (owned, _groups, rest) = slots.split();
        let written = target.written.within(owned);
        self.head.hold(Command {
            written: &written,
            ..target
        });
        self.tail.hold(target, rest);
    }

    fn torque_off(&mut self) {
        self.tail.torque_off();
        self.head.torque_off();
    }

    fn close(&mut self) {
        self.tail.close();
        self.head.close();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::N;

    struct Dxl;
    struct CubeMars;

    fn member<H>(position: u8) -> Member {
        Member {
            position,
            name: type_name::<H>(),
        }
    }

    fn claim(
        owned: impl IntoIterator<Item = usize>,
        group: impl Fn(usize) -> u8,
        every: u32,
    ) -> Claim<N> {
        let mut mask = Mask::NONE;
        for actuator in owned {
            mask.set(actuator);
        }
        Claim {
            owned: mask,
            group: core::array::from_fn(|index| Group(group(index))),
            read_every: [NonZeroU32::new(every).unwrap(); N],
            read_cost: Duration::from_micros(100),
            write_cost: Duration::from_micros(50),
        }
    }

    /// A call a fake member saw.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Seen {
        Open,
        Zero,
        TorqueOff,
        Close,
    }

    /// What one `write` handed a member: its runs' local groups and
    /// actuators, and the `q` mask of its command.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct Wrote {
        groups: [Option<Group>; 2],
        actuators: Mask<N>,
        written_q: Mask<N>,
    }

    /// One call: the member's id and what it saw.
    type Entry = (u8, Seen);

    /// The calls of every fake member, in order. Fixed capacity: no heap.
    struct Journal(core::cell::RefCell<([Entry; 16], usize)>);

    impl Journal {
        fn new() -> Self {
            Self(core::cell::RefCell::new(([(0, Seen::Open); 16], 0)))
        }

        fn push(&self, id: u8, seen: Seen) {
            let (calls, len) = &mut *self.0.borrow_mut();
            calls[*len] = (id, seen);
            *len += 1;
        }

        fn calls(&self) -> core::cell::Ref<'_, [Entry]> {
            core::cell::Ref::map(self.0.borrow(), |(calls, len)| &calls[..*len])
        }
    }

    #[derive(Debug, PartialEq, thiserror::Error)]
    #[error("bus {0} unplugged")]
    struct Unplugged(u8);

    /// A member owning `owned`, every actuator in local group `group`.
    struct Fake<'a> {
        id: u8,
        owned: Mask<N>,
        group: u8,
        fail_open: bool,
        /// An actuator it changes without owning it, in `read` and `write`.
        stray: Option<usize>,
        journal: &'a Journal,
        wrote: core::cell::Cell<Wrote>,
    }

    impl<'a> Fake<'a> {
        fn new(id: u8, owned: Mask<N>, group: u8, journal: &'a Journal) -> Self {
            Self {
                id,
                owned,
                group,
                fail_open: false,
                stray: None,
                journal,
                wrote: core::cell::Cell::default(),
            }
        }
    }

    impl super::super::seal::Adapter for Fake<'_> {}

    impl Hal<N> for Fake<'_> {
        type Error = Unplugged;

        fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, Unplugged> {
            Ok(Claim {
                owned: self.owned,
                group: [Group(self.group); N],
                ..claim([], |_| 0, 1)
            })
        }

        fn open(&mut self) -> Result<(), Unplugged> {
            self.journal.push(self.id, Seen::Open);
            if self.fail_open {
                return Err(Unplugged(self.id));
            }
            Ok(())
        }

        fn zero(&mut self) -> Result<(), Unplugged> {
            self.journal.push(self.id, Seen::Zero);
            Ok(())
        }

        fn ping(&mut self) -> Result<(), Unplugged> {
            Ok(())
        }

        fn health(&mut self) -> Result<(), Unplugged> {
            Ok(())
        }

        fn configure(&mut self) -> Result<(), Unplugged> {
            Ok(())
        }

        fn read_all(&mut self, _sample: Sample<'_, N>) -> Result<(), Unplugged> {
            Ok(())
        }

        fn torque_on(&mut self) -> Result<(), Unplugged> {
            Ok(())
        }

        fn read(&mut self, sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
            if let Some(actuator) = self.stray {
                sample.q[actuator] = crate::state::Q(99.0);
            }
            self.owned
        }

        fn write(&mut self, command: Command<'_, N>, batch: &Batch<'_, N>, faults: &mut Faults<N>) {
            if let Some(actuator) = self.stray {
                faults.rejected.set(actuator);
            }
            let mut wrote = Wrote {
                written_q: command.written.q,
                ..Wrote::default()
            };
            for (slot, (group, run)) in wrote.groups.iter_mut().zip(batch.runs()) {
                *slot = Some(group);
                run.iter()
                    .for_each(|&actuator| wrote.actuators.set(actuator));
            }
            self.wrote.set(wrote);
        }

        fn hold(&mut self, _target: Command<'_, N>) {}

        fn torque_off(&mut self) {
            self.journal.push(self.id, Seen::TorqueOff);
        }

        fn close(&mut self) {
            self.journal.push(self.id, Seen::Close);
        }
    }

    fn mask(actuators: impl IntoIterator<Item = usize>) -> Mask<N> {
        let mut mask = Mask::NONE;
        actuators
            .into_iter()
            .for_each(|actuator| mask.set(actuator));
        mask
    }

    /// A Dynamixel-like member on `0..2`, local group 1, then a CubeMars-like
    /// one on the rest, local group 0.
    fn two<'a>(journal: &'a Journal) -> HCons<Fake<'a>, HCons<Fake<'a>, HNil>> {
        HCons {
            head: Fake::new(0, mask(0..2), 1, journal),
            tail: HCons {
                head: Fake::new(1, mask(2..N), 0, journal),
                tail: HNil,
            },
        }
    }

    /// Every field of every actuator at zero, to lend as a command or a
    /// sample.
    fn zeroed() -> crate::state::State<N> {
        crate::state::State::default()
    }

    fn command<'a>(
        values: &'a crate::state::State<N>,
        written: &'a super::super::Written<N>,
    ) -> Command<'a, N> {
        Command::new(
            &values.q,
            &values.qd,
            &values.tau,
            &values.kp,
            &values.kd,
            written,
        )
    }

    fn sample(values: &mut crate::state::State<N>) -> Sample<'_, N> {
        Sample::new(
            &mut values.q,
            &mut values.qd,
            &mut values.tau,
            &mut values.kp,
            &mut values.kd,
        )
    }

    fn layout_of(hal: &mut impl Drivers<N>) -> Layout<N> {
        let mut plan = Plan::new();
        hal.claim(Overrides::default(), &mut plan).unwrap();
        plan.finish().unwrap()
    }

    #[test]
    fn each_member_writes_only_its_runs_with_local_groups() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        let layout = layout_of(&mut hal);
        let written = super::super::Written {
            q: mask([1, 3]),
            ..Default::default()
        };
        let values = zeroed();
        let mut batch = Batch::new(&layout, &written);

        hal.write(
            command(&values, &written),
            &mut batch,
            &mut Faults::default(),
            layout.slots(),
            &mut Audit::default(),
        );

        let dxl = hal.head.wrote.get();
        let cubemars = hal.tail.head.wrote.get();
        assert_eq!(dxl.groups, [Some(Group(1)), None]);
        assert_eq!((dxl.actuators, dxl.written_q), (mask([1]), mask([1])));
        assert_eq!(cubemars.groups, [Some(Group(0)), None]);
        assert_eq!(
            (cubemars.actuators, cubemars.written_q),
            (mask([3]), mask([3]))
        );
    }

    #[test]
    fn a_member_with_nothing_written_is_skipped() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        let layout = layout_of(&mut hal);
        let written = super::super::Written {
            q: mask([2]),
            ..Default::default()
        };
        let values = zeroed();
        let mut batch = Batch::new(&layout, &written);

        hal.write(
            command(&values, &written),
            &mut batch,
            &mut Faults::default(),
            layout.slots(),
            &mut Audit::default(),
        );

        assert_eq!(hal.head.wrote.get(), Wrote::default());
        assert_eq!(hal.tail.head.wrote.get().actuators, mask([2]));
    }

    #[test]
    fn reads_union_every_member() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        let layout = layout_of(&mut hal);
        let mut values = zeroed();
        let mut audit = Audit::default();
        let read = hal.read(
            sample(&mut values),
            &mut Faults::default(),
            layout.slots(),
            &mut audit,
        );
        assert_eq!(read, Mask::ALL);
    }

    #[test]
    fn a_refreshed_mask_is_cut_to_the_member_s_actuators() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        let layout = layout_of(&mut hal);
        // Claims 0..2 but reports every actuator refreshed.
        hal.head.owned = Mask::ALL;
        hal.tail.head.owned = Mask::NONE;
        let mut values = zeroed();
        let mut audit = Audit::default();
        let read = hal.read(
            sample(&mut values),
            &mut Faults::default(),
            layout.slots(),
            &mut audit,
        );
        assert_eq!(read, mask(0..2));
    }

    #[cfg(debug_assertions)]
    #[test]
    fn a_read_into_another_member_s_slot_is_noted() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        let layout = layout_of(&mut hal);
        hal.tail.head.stray = Some(0);
        let mut values = zeroed();
        let mut audit = Audit::default();

        hal.read(
            sample(&mut values),
            &mut Faults::default(),
            layout.slots(),
            &mut audit,
        );

        assert_eq!(
            audit.take(),
            Some(Stray {
                member: type_name::<Fake<'_>>(),
                actuator: 0
            })
        );
    }

    #[cfg(debug_assertions)]
    #[test]
    fn a_fault_on_another_member_s_actuator_is_noted() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        let layout = layout_of(&mut hal);
        hal.head.stray = Some(N - 1);
        let written = super::super::Written {
            q: Mask::ALL,
            ..Default::default()
        };
        let values = zeroed();
        let mut batch = Batch::new(&layout, &written);
        let mut audit = Audit::default();

        hal.write(
            command(&values, &written),
            &mut batch,
            &mut Faults::default(),
            layout.slots(),
            &mut audit,
        );

        let stray = audit.take().expect("a stray fault");
        assert_eq!(stray.actuator, N - 1);
    }

    #[test]
    fn members_that_stay_on_their_actuators_leave_the_audit_empty() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        let layout = layout_of(&mut hal);
        let mut values = zeroed();
        let mut audit = Audit::default();
        hal.read(
            sample(&mut values),
            &mut Faults::default(),
            layout.slots(),
            &mut audit,
        );
        assert_eq!(audit.take(), None);
    }

    #[test]
    fn torque_reaches_every_member_before_any_close_last_first() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        hal.torque_off();
        hal.close();
        assert_eq!(
            *journal.calls(),
            [
                (1, Seen::TorqueOff),
                (0, Seen::TorqueOff),
                (1, Seen::Close),
                (0, Seen::Close),
            ]
        );
    }

    #[test]
    fn a_failed_step_stops_the_walk_and_names_the_member() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        hal.tail.head.fail_open = true;

        assert!(hal.step(Step::Open).is_err());
        let err = hal.step(Step::Zero).and(hal.step(Step::Open)).unwrap_err();

        assert_eq!(
            *journal.calls(),
            [
                (0, Seen::Open),
                (1, Seen::Open),
                (0, Seen::Zero),
                (1, Seen::Zero),
                (0, Seen::Open),
                (1, Seen::Open),
            ]
        );
        let text = err.to_string();
        assert!(text.starts_with("member 1 ("), "{text}");
        assert!(
            text.contains("Fake") && text.ends_with("bus 1 unplugged"),
            "{text}"
        );
        let source = core::error::Error::source(&err).expect("the member's error");
        assert_eq!(source.downcast_ref::<Unplugged>(), Some(&Unplugged(1)));
    }

    #[test]
    fn the_walk_refuses_overlapping_claims() {
        let journal = Journal::new();
        let mut hal = two(&journal);
        hal.tail.head.owned = mask([1, 2]);
        let mut plan = Plan::new();
        let err = hal.claim(Overrides::default(), &mut plan).unwrap_err();
        assert!(
            matches!(
                err,
                HardwareError::Coverage(Coverage::Overlap { actuator: 1, .. })
            ),
            "{err:?}"
        );
    }

    #[test]
    fn two_members_split_the_robot_and_their_groups() {
        let mut plan = Plan::new();
        plan.add(type_name::<Dxl>(), &claim([0, 1], |i| i as u8, 4))
            .unwrap();
        plan.add(type_name::<CubeMars>(), &claim(2..N, |_| 0, 1))
            .unwrap();
        let layout = plan.finish().unwrap();

        assert_eq!(layout.group[..3], [Group(0), Group(1), Group(2)]);
        assert!(layout.group[2..].iter().all(|&g| g == Group(2)));
        assert_eq!([0, 1, 2].map(|i| layout.read_every[i].get()), [4, 4, 1]);
        assert_eq!(layout.read_cost, Duration::from_micros(200));
        assert_eq!(layout.write_cost, Duration::from_micros(100));

        let (dxl, dxl_groups, rest) = layout.slots().split();
        let (cubemars, cubemars_groups, _) = rest.split();
        assert!(dxl.get(1) && !dxl.get(2) && cubemars.get(2));
        assert_eq!((dxl_groups, cubemars_groups), (0..2, 2..3));
    }

    #[test]
    fn a_gap_names_the_first_unowned_actuator() {
        let mut plan = Plan::new();
        let all_but_2 = (0..N).filter(|&i| i != 2);
        plan.add(type_name::<Dxl>(), &claim(all_but_2, |_| 0, 1))
            .unwrap();
        assert_eq!(
            plan.finish().unwrap_err(),
            Coverage::Unowned { actuator: 2 }
        );
    }

    #[test]
    fn an_overlap_names_both_members() {
        let mut plan = Plan::new();
        plan.add(type_name::<Dxl>(), &claim([0, 1, 2], |_| 0, 1))
            .unwrap();
        let err = plan
            .add(type_name::<CubeMars>(), &claim([2, 3], |_| 0, 1))
            .unwrap_err();

        assert_eq!(
            err,
            Coverage::Overlap {
                actuator: 2,
                first: member::<Dxl>(0),
                second: member::<CubeMars>(1),
            }
        );
        let text = err.to_string();
        assert!(text.contains("member 0") && text.contains("Dxl"), "{text}");
        assert!(
            text.contains("member 1") && text.contains("CubeMars"),
            "{text}"
        );
    }

    #[test]
    fn a_refused_claim_leaves_the_plan_as_it_was() {
        let mut plan = Plan::new();
        plan.add(type_name::<Dxl>(), &claim([0, 1], |_| 0, 4))
            .unwrap();
        let overlapping = claim(1..N, |_| 0, 1);
        assert!(plan.add(type_name::<CubeMars>(), &overlapping).is_err());
        plan.add(type_name::<CubeMars>(), &claim(2..N, |_| 0, 1))
            .unwrap();

        let layout = plan.finish().unwrap();
        assert_eq!(layout.read_every[1].get(), 4);
        assert_eq!(layout.read_cost, Duration::from_micros(200));
    }

    #[test]
    fn a_member_without_actuators_is_refused() {
        let err = Plan::new()
            .add(type_name::<Dxl>(), &claim([], |_| 0, 1))
            .unwrap_err();
        assert_eq!(
            err,
            Coverage::Idle {
                member: member::<Dxl>(0)
            }
        );
    }

    #[test]
    fn groups_past_the_u8_range_are_refused() {
        let mut plan = Plan::new();
        plan.add(type_name::<Dxl>(), &claim([0], |_| 255, 1))
            .unwrap();
        let err = plan
            .add(type_name::<CubeMars>(), &claim([1], |_| 0, 1))
            .unwrap_err();
        assert_eq!(
            err,
            Coverage::TooManyGroups {
                member: member::<CubeMars>(1)
            }
        );
    }
}
