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
use crate::state::JOINTS;

/// Transaction groups the whole robot may have, `Group` being a `u8`.
const MAX_GROUPS: u16 = 256;

/// What one member declares at run() finalization, no I/O. Entries outside
/// `owned` are ignored.
#[derive(Clone, Debug)]
pub struct Claim {
    /// Actuators the member drives.
    pub owned: Mask,
    /// Member-local group of each owned actuator; the engine renumbers them
    /// robot-wide.
    pub group: [Group; JOINTS],
    /// Read cadence of each owned actuator, in ticks.
    pub read_every: [NonZeroU32; JOINTS],
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
pub struct Plan {
    /// Actuators of each member, by position. Each owns one at least, so
    /// there are at most `JOINTS` members.
    owned: [Mask; JOINTS],
    /// Type name of each member, by position.
    name: [&'static str; JOINTS],
    /// Member k has the groups `bound[k]..bound[k + 1]`.
    bound: [u16; JOINTS + 1],
    members: usize,
    taken: Mask,
    group: [Group; JOINTS],
    read_every: [NonZeroU32; JOINTS],
    read_cost: Duration,
    write_cost: Duration,
}

impl Plan {
    /// No claim yet.
    pub(crate) fn new() -> Self {
        Self {
            owned: [Mask::NONE; JOINTS],
            name: [""; JOINTS],
            bound: [0; JOINTS + 1],
            members: 0,
            taken: Mask::NONE,
            group: [Group(0); JOINTS],
            read_every: [NonZeroU32::MIN; JOINTS],
            read_cost: Duration::ZERO,
            write_cost: Duration::ZERO,
        }
    }

    /// Adds the claim of the next member, of type name `name`, its groups
    /// shifted past those already taken. Costs add up: members run one after
    /// the other. On error the plan is left as it was.
    pub(crate) fn add(&mut self, name: &'static str, claim: &Claim) -> Result<(), Coverage> {
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
            self.bound.get_mut(self.members + 1),
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
        self.bound.get(self.members).copied().unwrap_or(MAX_GROUPS)
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
    pub(crate) fn finish(self) -> Result<Layout, Coverage> {
        if let Some(actuator) = Mask::ALL.difference(self.taken).iter().next() {
            return Err(Coverage::Unowned { actuator });
        }
        Ok(Layout::merged(
            self.group,
            self.read_every,
            (self.read_cost, self.write_cost),
            self.owned,
            self.bound,
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
    pub fn head<M: Hal<Error = E>>(error: E) -> Self {
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

/// The robot's hardware: a list of [`Hal`] members, `HCons<A, HCons<B,
/// HNil>>`. Implemented by the engine only; the walk calls every member,
/// head first, and cuts torque in reverse.
pub trait Drivers: seal::Drivers {
    /// One variant per member, nested.
    type Error: seal::Nested;

    /// Claims of every member, merged into `plan`.
    #[doc(hidden)]
    fn claim(
        &mut self,
        overrides: Overrides,
        plan: &mut Plan,
    ) -> Result<(), HardwareError<Self::Error>>;

    /// `step` on every member; the first error stops the walk.
    #[doc(hidden)]
    fn step(&mut self, step: Step) -> Result<(), Self::Error>;

    /// [`Hal::read_all`] on every member, into the same slot.
    #[doc(hidden)]
    fn read_all(&mut self, sample: Sample<'_>) -> Result<(), Self::Error>;

    /// [`Hal::read`] on every member; the refreshed masks, each cut to its
    /// member's actuators, unioned.
    #[doc(hidden)]
    fn read(&mut self, sample: Sample<'_>, faults: &mut Faults, slots: Slots<'_>) -> Mask;

    /// [`Hal::write`] on every member with a run in `batch`, each given its
    /// own runs and writes.
    #[doc(hidden)]
    fn write(
        &mut self,
        command: Command<'_>,
        batch: &mut Batch<'_>,
        faults: &mut Faults,
        slots: Slots<'_>,
    );

    /// [`Hal::hold`] on every member, each given its own writes.
    #[doc(hidden)]
    fn hold(&mut self, target: Command<'_>, slots: Slots<'_>);

    /// [`Hal::torque_off`] on every member, last first.
    #[doc(hidden)]
    fn torque_off(&mut self);

    /// [`Hal::close`] on every member, last first.
    #[doc(hidden)]
    fn close(&mut self);
}

impl seal::Drivers for HNil {}

impl Drivers for HNil {
    type Error = Infallible;

    fn claim(
        &mut self,
        _overrides: Overrides,
        _plan: &mut Plan,
    ) -> Result<(), HardwareError<Infallible>> {
        Ok(())
    }

    fn step(&mut self, _step: Step) -> Result<(), Infallible> {
        Ok(())
    }

    fn read_all(&mut self, _sample: Sample<'_>) -> Result<(), Infallible> {
        Ok(())
    }

    fn read(&mut self, _sample: Sample<'_>, _faults: &mut Faults, _slots: Slots<'_>) -> Mask {
        Mask::NONE
    }

    fn write(
        &mut self,
        _command: Command<'_>,
        _batch: &mut Batch<'_>,
        _faults: &mut Faults,
        _slots: Slots<'_>,
    ) {
    }

    fn hold(&mut self, _target: Command<'_>, _slots: Slots<'_>) {}

    fn torque_off(&mut self) {}

    fn close(&mut self) {}
}

impl<H: Hal, T: Drivers> seal::Drivers for HCons<H, T> {}

impl<H: Hal, T: Drivers> Drivers for HCons<H, T> {
    type Error = MemberError<H::Error, T::Error>;

    fn claim(
        &mut self,
        overrides: Overrides,
        plan: &mut Plan,
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

    fn read_all(&mut self, mut sample: Sample<'_>) -> Result<(), Self::Error> {
        self.head
            .read_all(sample.reborrow())
            .map_err(MemberError::head::<H>)?;
        self.tail.read_all(sample).map_err(MemberError::Tail)
    }

    fn read(&mut self, mut sample: Sample<'_>, faults: &mut Faults, slots: Slots<'_>) -> Mask {
        let (owned, _groups, rest) = slots.split();
        let head = self
            .head
            .read(sample.reborrow(), faults)
            .intersection(owned);
        head.union(self.tail.read(sample, faults, rest))
    }

    fn write(
        &mut self,
        command: Command<'_>,
        batch: &mut Batch<'_>,
        faults: &mut Faults,
        slots: Slots<'_>,
    ) {
        let (owned, groups, rest) = slots.split();
        batch.select(groups);
        if !batch.is_empty() {
            let written = command.written.within(owned);
            let mine = Command {
                written: &written,
                ..command
            };
            self.head.write(mine, batch, faults);
        }
        self.tail.write(command, batch, faults, rest);
    }

    fn hold(&mut self, target: Command<'_>, slots: Slots<'_>) {
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
    ) -> Claim {
        let mut mask = Mask::NONE;
        for actuator in owned {
            mask.set(actuator);
        }
        Claim {
            owned: mask,
            group: core::array::from_fn(|index| Group(group(index))),
            read_every: [NonZeroU32::new(every).unwrap(); JOINTS],
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
        actuators: Mask,
        written_q: Mask,
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
        owned: Mask,
        group: u8,
        fail_open: bool,
        journal: &'a Journal,
        wrote: core::cell::Cell<Wrote>,
    }

    impl<'a> Fake<'a> {
        fn new(id: u8, owned: Mask, group: u8, journal: &'a Journal) -> Self {
            Self {
                id,
                owned,
                group,
                fail_open: false,
                journal,
                wrote: core::cell::Cell::default(),
            }
        }
    }

    impl super::super::seal::Adapter for Fake<'_> {}

    impl Hal for Fake<'_> {
        type Error = Unplugged;

        fn check(&mut self, _overrides: Overrides) -> Result<Claim, Unplugged> {
            Ok(Claim {
                owned: self.owned,
                group: [Group(self.group); JOINTS],
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

        fn read_all(&mut self, _sample: Sample<'_>) -> Result<(), Unplugged> {
            Ok(())
        }

        fn torque_on(&mut self) -> Result<(), Unplugged> {
            Ok(())
        }

        fn read(&mut self, _sample: Sample<'_>, _faults: &mut Faults) -> Mask {
            self.owned
        }

        fn write(&mut self, command: Command<'_>, batch: &Batch<'_>, _faults: &mut Faults) {
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

        fn hold(&mut self, _target: Command<'_>) {}

        fn torque_off(&mut self) {
            self.journal.push(self.id, Seen::TorqueOff);
        }

        fn close(&mut self) {
            self.journal.push(self.id, Seen::Close);
        }
    }

    fn mask(actuators: impl IntoIterator<Item = usize>) -> Mask {
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
                head: Fake::new(1, mask(2..JOINTS), 0, journal),
                tail: HNil,
            },
        }
    }

    /// Every field of every actuator at zero, to lend as a command or a
    /// sample.
    fn zeroed() -> crate::state::State<JOINTS> {
        crate::state::State::default()
    }

    fn command<'a>(
        values: &'a crate::state::State<JOINTS>,
        written: &'a super::super::Written,
    ) -> Command<'a> {
        Command::new(
            &values.q,
            &values.qd,
            &values.tau,
            &values.kp,
            &values.kd,
            written,
        )
    }

    fn sample(values: &mut crate::state::State<JOINTS>) -> Sample<'_> {
        Sample::new(
            &mut values.q,
            &mut values.qd,
            &mut values.tau,
            &mut values.kp,
            &mut values.kd,
        )
    }

    fn layout_of(hal: &mut impl Drivers) -> Layout {
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
        let read = hal.read(sample(&mut values), &mut Faults::default(), layout.slots());
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
        let read = hal.read(sample(&mut values), &mut Faults::default(), layout.slots());
        assert_eq!(read, mask(0..2));
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
        plan.add(type_name::<CubeMars>(), &claim(2..JOINTS, |_| 0, 1))
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
        let all_but_2 = (0..JOINTS).filter(|&i| i != 2);
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
        let overlapping = claim(1..JOINTS, |_| 0, 1);
        assert!(plan.add(type_name::<CubeMars>(), &overlapping).is_err());
        plan.add(type_name::<CubeMars>(), &claim(2..JOINTS, |_| 0, 1))
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
