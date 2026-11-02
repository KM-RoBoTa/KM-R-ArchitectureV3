//! The per-tick path: group tables, read cadence, grouped reads and writes;
//! and the goals `torque_on` writes before the torque goes on.
//!
//! The tables are built once, in `check`; `read` and `write` only walk them,
//! with stack arrays: no allocation, no retry, no sleep (FR-013).

use core::num::NonZeroU32;
use core::time::Duration;

use kmr_core::hal::{Batch, Command, Faults, Group, Mask, Sample};
use kmr_core::state::{JOINTS, Q, Qd, Tau};

use super::backend::{Backend, BackendError, GoalRegister, RawState};
use super::config::{Actuator, Bus, Mode, Model, Protocol, Torque};
use super::units;
use super::{Dynamixel, DynamixelError};

/// Protocol 2 sync read instruction without its ids: header, id, length,
/// instruction, address, data length, CRC.
const SYNC_READ_BYTES: u64 = 14;
/// Protocol 2 status packet without its data: header, id, length,
/// instruction, error, CRC.
const STATUS_BYTES: u64 = 11;
/// Protocol 1 read of present position: 8-byte instruction, 8-byte status.
const READ_POSITION_BYTES: u64 = 16;
/// Protocol 2 sync write instruction without its entries.
const SYNC_WRITE_2_BYTES: u64 = 14;
/// Protocol 2 sync write entry: id and a 4-byte goal.
const GOAL_2_BYTES: u64 = 5;
/// Protocol 1 sync write instruction without its entries.
const SYNC_WRITE_1_BYTES: u64 = 8;
/// Protocol 1 sync write entry: id and a 2-byte goal.
const GOAL_1_BYTES: u64 = 3;
/// UART bits per byte: start, 8 data, stop.
const BITS_PER_BYTE: u64 = 10;

/// The actuators of one series on one bus: one transaction where the
/// protocol allows it.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Table {
    /// Bus index, the backend's port.
    pub(crate) bus: usize,
    /// Model of the first member; the others share its series.
    pub(crate) model: Model,
    /// Read cadence of the bus, in ticks.
    pub(crate) every: NonZeroU32,
    /// Actuator indices, ascending, `len` of them used.
    members: [usize; JOINTS],
    /// Firmware id of each member.
    ids: [u8; JOINTS],
    /// Members in use.
    len: usize,
}

impl Table {
    /// A table with no member.
    const EMPTY: Self = Self {
        bus: 0,
        model: Model::Xm430W210,
        every: NonZeroU32::MIN,
        members: [0; JOINTS],
        ids: [0; JOINTS],
        len: 0,
    };

    /// Actuator indices, ascending.
    pub(crate) fn members(&self) -> &[usize] {
        self.members.get(..self.len).unwrap_or_default()
    }

    /// Firmware ids, in the order of [`Table::members`].
    pub(crate) fn ids(&self) -> &[u8] {
        self.ids.get(..self.len).unwrap_or_default()
    }

    /// Adds actuator `index`.
    fn push(&mut self, index: usize, id: u8) {
        if let (Some(member), Some(slot)) =
            (self.members.get_mut(self.len), self.ids.get_mut(self.len))
        {
            *member = index;
            *slot = id;
            self.len += 1;
        }
    }

    /// Bytes on the wire for one read of every member; `state_reads` are
    /// the data bytes of each Protocol 2 sync read of one state read.
    fn read_bytes(&self, state_reads: &[u16]) -> u64 {
        let members = self.len as u64;
        match self.model.protocol() {
            Protocol::Two => state_reads
                .iter()
                .map(|&data| SYNC_READ_BYTES + members * (1 + STATUS_BYTES + u64::from(data)))
                .sum(),
            Protocol::One => members * READ_POSITION_BYTES,
        }
    }

    /// Bytes on the wire for one goal write to every member.
    fn write_bytes(&self) -> u64 {
        let members = self.len as u64;
        match self.model.protocol() {
            Protocol::Two => SYNC_WRITE_2_BYTES + members * GOAL_2_BYTES,
            Protocol::One => SYNC_WRITE_1_BYTES + members * GOAL_1_BYTES,
        }
    }
}

/// Every group, numbered as in the layout.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Groups {
    /// Tables, `len` of them used; at most one per actuator.
    tables: [Table; JOINTS],
    /// Tables in use.
    len: usize,
    /// Table of each actuator.
    of: [usize; JOINTS],
}

impl Groups {
    /// No group: `check` was not called.
    pub(crate) const EMPTY: Self = Self {
        tables: [Table::EMPTY; JOINTS],
        len: 0,
        of: [0; JOINTS],
    };

    /// Numbers the distinct (bus, series) pairs from 0 in first-seen
    /// actuator order; `every` is the cadence of each bus.
    pub(crate) fn build<const NB: usize>(
        actuators: &[Actuator; JOINTS],
        every: &[NonZeroU32; NB],
    ) -> Self {
        let mut groups = Self::EMPTY;
        for (index, actuator) in actuators.iter().enumerate() {
            let series = actuator.model.series();
            let found = groups
                .tables()
                .iter()
                .position(|table| table.bus == actuator.bus && table.model.series() == series);
            let number = match found {
                Some(number) => number,
                None => {
                    let Some(table) = groups.tables.get_mut(groups.len) else {
                        continue;
                    };
                    *table = Table {
                        bus: actuator.bus,
                        model: actuator.model,
                        every: every.get(actuator.bus).copied().unwrap_or(NonZeroU32::MIN),
                        ..Table::EMPTY
                    };
                    groups.len += 1;
                    groups.len - 1
                }
            };
            if let (Some(table), Some(of)) =
                (groups.tables.get_mut(number), groups.of.get_mut(index))
            {
                table.push(index, actuator.id);
                *of = number;
            }
        }
        groups
    }

    /// Tables in group order.
    pub(crate) fn tables(&self) -> &[Table] {
        self.tables.get(..self.len).unwrap_or_default()
    }

    /// Group number of each actuator. A number past `u8::MAX` saturates:
    /// `write` looks up each actuator's own table, so a shared number never
    /// mixes two tables in one transaction.
    pub(crate) fn numbers(&self) -> [Group; JOINTS] {
        self.of
            .map(|table| Group(u8::try_from(table).unwrap_or(u8::MAX)))
    }

    /// Read cadence in force for each actuator.
    pub(crate) fn cadence(&self) -> [NonZeroU32; JOINTS] {
        self.of.map(|table| {
            self.tables
                .get(table)
                .map_or(NonZeroU32::MIN, |table| table.every)
        })
    }

    /// Line time of one full read and of one full write, from each bus's
    /// baud rate. Return delay and USB latency are not counted.
    pub(crate) fn costs<const NB: usize>(
        &self,
        buses: &[Bus; NB],
        state_reads: &[u16],
    ) -> (Duration, Duration) {
        let mut read = Duration::ZERO;
        let mut write = Duration::ZERO;
        for table in self.tables() {
            let baud = buses.get(table.bus).map_or(0, |bus| u64::from(bus.baud));
            read += line_time(table.read_bytes(state_reads), baud);
            write += line_time(table.write_bytes(), baud);
        }
        (read, write)
    }
}

/// Time `bytes` take on the wire at `baud`; zero for a zero baud.
fn line_time(bytes: u64, baud: u64) -> Duration {
    let nanos = (bytes * BITS_PER_BYTE * 1_000_000_000)
        .checked_div(baud)
        .unwrap_or(0);
    Duration::from_nanos(nanos)
}

/// What became of one actuator in a group read.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Outcome {
    /// Its sample was overwritten.
    Read,
    /// It did not answer.
    Lost(BackendError),
    /// It answered this raw position, outside the register range; nothing
    /// was stored.
    Invalid(i64),
    /// The transaction failed on its bus.
    Bus(BackendError),
}

/// Up to one goal per member of one table, sent in one backend call.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Goals {
    /// Table the goals go to.
    table: usize,
    /// `(id, raw)` entries, `len` of them used.
    goals: [(u8, u32); JOINTS],
    /// Actuator index of each entry.
    indices: [usize; JOINTS],
    /// Entries in use.
    len: usize,
}

impl Goals {
    /// No goal yet for `table`.
    pub(crate) const fn new(table: usize) -> Self {
        Self {
            table,
            goals: [(0, 0); JOINTS],
            indices: [0; JOINTS],
            len: 0,
        }
    }

    /// Queues `raw` for actuator `index`.
    pub(crate) fn push(&mut self, index: usize, id: u8, raw: u32) {
        if let (Some(goal), Some(slot)) =
            (self.goals.get_mut(self.len), self.indices.get_mut(self.len))
        {
            *goal = (id, raw);
            *slot = index;
            self.len += 1;
        }
    }

    /// Actuator indices queued.
    pub(crate) fn indices(&self) -> &[usize] {
        self.indices.get(..self.len).unwrap_or_default()
    }

    /// The table and the queued goals; `None` when none is queued.
    fn queued<'a>(&'a self, groups: &'a Groups) -> Option<(&'a Table, &'a [(u8, u32)])> {
        let goals = self.goals.get(..self.len).unwrap_or_default();
        let table = groups.tables().get(self.table)?;
        (!goals.is_empty()).then_some((table, goals))
    }

    /// Sends the queued goals in one transaction; nothing when none is
    /// queued.
    pub(crate) fn send<B: Backend>(
        &self,
        backend: &mut B,
        groups: &Groups,
    ) -> Result<(), BackendError> {
        self.queued(groups).map_or(Ok(()), |(table, goals)| {
            backend.sync_write_goal(table.bus, table.model, goals)
        })
    }

    /// Sends the queued goals to `register` in one `sync_write_start_goal`,
    /// before the torque goes on; nothing when none is queued.
    pub(crate) fn send_start<B: Backend>(
        &self,
        backend: &mut B,
        groups: &Groups,
        register: GoalRegister,
    ) -> Result<(), BackendError> {
        self.queued(groups).map_or(Ok(()), |(table, goals)| {
            backend.sync_write_start_goal(table.bus, table.model, register, goals)
        })
    }
}

/// Raw goal position of `actuator` for `q`, when it is powered and in
/// position mode. `None` also outside the register range (research R11).
pub(crate) fn position_goal(actuator: &Actuator, q: Q) -> Option<u32> {
    match (actuator.mode, actuator.torque) {
        (Mode::Position, Torque::On) => units::position_to_raw(actuator.model, q),
        _ => None,
    }
}

/// Whether `actuator` starts from its present position when its torque
/// goes on, which `torque_on` then reads.
fn starts_at_present(actuator: &Actuator) -> bool {
    matches!(
        (actuator.mode, actuator.torque),
        (Mode::Position, Torque::On)
    )
}

/// The goal `actuator` holds when its torque goes on, and its register:
/// Goal Position = `present`, Goal Velocity or Goal Current = 0. `None` for
/// a read-only actuator, and for a `present` outside the register range.
fn start_goal(actuator: &Actuator, present: Q) -> Option<(GoalRegister, u32)> {
    match (actuator.mode, actuator.torque) {
        (_, Torque::ReadOnly) => None,
        (Mode::Position, Torque::On) => {
            position_goal(actuator, present).map(|raw| (GoalRegister::Position, raw))
        }
        (Mode::Velocity, Torque::On) => Some((GoalRegister::Velocity, 0)),
        (Mode::Current, Torque::On) => Some((GoalRegister::Current, 0)),
    }
}

/// Reads every member of `table` into `sample` and reports each one.
///
/// Protocol 2 reads the group in one transaction; Protocol 1 reads each
/// position on its own and leaves `Qd` and `Tau` as copied (research R12).
pub(crate) fn read_table<B: Backend>(
    backend: &mut B,
    table: &Table,
    actuators: &[Actuator; JOINTS],
    sample: &mut Sample<'_>,
    mut report: impl FnMut(usize, Outcome),
) {
    let model_of = |index: usize| actuators.get(index).map_or(table.model, |a| a.model);
    match table.model.protocol() {
        Protocol::Two => {
            let ids = table.ids();
            let mut out = [Ok(RawState::default()); JOINTS];
            let out = out.get_mut(..ids.len()).unwrap_or_default();
            if let Err(error) = backend.sync_read_state(table.bus, table.model, ids, out) {
                for &index in table.members() {
                    report(index, Outcome::Bus(error));
                }
                return;
            }
            for (&index, result) in table.members().iter().zip(out.iter()) {
                match result {
                    Ok(raw) if store_state(sample, model_of(index), index, raw) => {
                        report(index, Outcome::Read);
                    }
                    Ok(raw) => report(index, Outcome::Invalid(i64::from(raw.position))),
                    Err(error) => report(index, Outcome::Lost(*error)),
                }
            }
        }
        Protocol::One => {
            for (&index, &id) in table.members().iter().zip(table.ids()) {
                let model = model_of(index);
                match backend.read_position(table.bus, model, id) {
                    Ok(raw) => {
                        let read = i32::try_from(raw)
                            .ok()
                            .and_then(|raw| units::position_to_si(model, raw));
                        match (read, sample.q.get_mut(index)) {
                            (Some(read), Some(q)) => {
                                *q = read;
                                report(index, Outcome::Read);
                            }
                            _ => report(index, Outcome::Invalid(i64::from(raw))),
                        }
                    }
                    Err(error) if error.is_bus() => report(index, Outcome::Bus(error)),
                    Err(error) => report(index, Outcome::Lost(error)),
                }
            }
        }
    }
}

/// Overwrites the sample of actuator `index` with `raw`. `Tau` is kept when
/// the model has no known torque constant. Stores nothing and returns
/// `false` when the position is outside the register range.
fn store_state(sample: &mut Sample<'_>, model: Model, index: usize, raw: &RawState) -> bool {
    let (Some(read), Some(q)) = (
        units::position_to_si(model, raw.position),
        sample.q.get_mut(index),
    ) else {
        return false;
    };
    *q = read;
    if let Some(qd) = sample.qd.get_mut(index) {
        *qd = units::velocity_to_si(model, raw.velocity);
    }
    if let (Some(tau), Some(value)) = (
        sample.tau.get_mut(index),
        units::current_to_si(model, raw.current),
    ) {
        *tau = value;
    }
    true
}

/// Actuators whose `Qd` and `Tau` are never read: Protocol 1 reads the
/// position only (research R12).
pub(crate) fn unread_rates(actuators: &[Actuator; JOINTS]) -> Mask {
    let mut unread = Mask::NONE;
    for (index, actuator) in actuators.iter().enumerate() {
        if actuator.model.protocol() == Protocol::One {
            unread.set(index);
        }
    }
    unread
}

/// The per-tick read: every group whose bus is due on this call, the first
/// call included (FR-009). Faults go to `faults` (FR-014, FR-015).
pub(super) fn read_due<B: Backend, const NB: usize>(
    adapter: &mut Dynamixel<B, NB>,
    mut sample: Sample<'_>,
    faults: &mut Faults,
) -> Mask {
    let call = adapter.reads;
    adapter.reads = adapter.reads.wrapping_add(1);
    let mut refreshed = Mask::NONE;
    for table in adapter.groups.tables() {
        if !call.is_multiple_of(u64::from(table.every.get())) {
            continue;
        }
        read_table(
            &mut adapter.backend,
            table,
            &adapter.table.actuators,
            &mut sample,
            |index, outcome| match outcome {
                Outcome::Read => refreshed.set(index),
                Outcome::Lost(_) => {
                    faults.stale.set(index);
                    faults.offline.set(index);
                }
                Outcome::Bus(_) => {
                    faults.stale.set(index);
                    faults.bus_error.set(index);
                }
                Outcome::Invalid(_) => faults.stale.set(index),
            },
        );
    }
    refreshed
}

/// The per-tick write: one transaction per run of one table. An actuator
/// not sent is `rejected`; a failed transaction sets `bus_error` on what it
/// carried (FR-012, FR-015).
pub(super) fn write_batch<B: Backend, const NB: usize>(
    adapter: &mut Dynamixel<B, NB>,
    command: &Command<'_>,
    batch: &Batch<'_>,
    faults: &mut Faults,
) {
    for (_, run) in batch.runs() {
        let mut goals: Option<Goals> = None;
        for &index in run {
            let (Some(&actuator), Some(&table)) = (
                adapter.table.actuators.get(index),
                adapter.groups.of.get(index),
            ) else {
                faults.rejected.set(index);
                continue;
            };
            if goals.is_some_and(|goals| goals.table != table) {
                flush_goals(adapter, goals.take(), faults);
            }
            let raw = command
                .q
                .get(index)
                .filter(|_| command.written.q.get(index))
                .and_then(|&q| adapter.goal(index, q));
            match raw {
                Some(raw) => goals
                    .get_or_insert(Goals::new(table))
                    .push(index, actuator.id, raw),
                None => faults.rejected.set(index),
            }
        }
        flush_goals(adapter, goals, faults);
    }
}

/// Writes, torque still off, the goal each powered actuator of group
/// `number` holds once powered: its present position, read now, or a zero
/// velocity or current. One `sync_write_start_goal` per goal register.
pub(super) fn write_start_goals<B: Backend, const NB: usize>(
    adapter: &mut Dynamixel<B, NB>,
    number: usize,
) -> Result<(), DynamixelError> {
    let Some(&table) = adapter.groups.tables().get(number) else {
        return Ok(());
    };
    let mut present = [Q::default(); JOINTS];
    let reads = table.members().iter().any(|&index| {
        adapter
            .table
            .actuators
            .get(index)
            .is_some_and(starts_at_present)
    });
    if reads {
        read_present(adapter, &table, &mut present)?;
    }
    let mut goals = GoalRegister::ALL.map(|_| Goals::new(number));
    for &index in table.members() {
        let Some(actuator) = adapter.table.actuators.get(index) else {
            continue;
        };
        if actuator.torque == Torque::ReadOnly {
            continue;
        }
        let q = present.get(index).copied().unwrap_or_default();
        let (register, raw) = start_goal(actuator, q).ok_or_else(|| {
            adapter.position_range(index, units::nearest_position(actuator.model, q))
        })?;
        if register == GoalRegister::Position && !adapter.within_limits(index, raw) {
            return Err(adapter.outside_limits(index, raw));
        }
        if let Some(goals) = goals.get_mut(register as usize) {
            goals.push(index, actuator.id, raw);
        }
    }
    for (register, goals) in GoalRegister::ALL.into_iter().zip(&goals) {
        goals
            .send_start(&mut adapter.backend, &adapter.groups, register)
            .map_err(|source| DynamixelError::Backend {
                bus: adapter.label(table.bus),
                source,
            })?;
    }
    Ok(())
}

/// Reads the present position of every member of `table` into `present`;
/// fails on the first one a start goal needs.
fn read_present<B: Backend, const NB: usize>(
    adapter: &mut Dynamixel<B, NB>,
    table: &Table,
    present: &mut [Q; JOINTS],
) -> Result<(), DynamixelError> {
    let (mut qd, mut tau) = ([Qd::default(); JOINTS], [Tau::default(); JOINTS]);
    let mut sample = Sample::new(present, &mut qd, &mut tau);
    let actuators = &adapter.table.actuators;
    let mut failed = None;
    read_table(
        &mut adapter.backend,
        table,
        actuators,
        &mut sample,
        |index, outcome| {
            let needed = actuators.get(index).is_some_and(starts_at_present);
            if needed && !matches!(outcome, Outcome::Read) {
                failed.get_or_insert((index, outcome));
            }
        },
    );
    adapter.read_failure(failed)
}

/// Sends `goals`, `bus_error` on what they carried if it fails.
fn flush_goals<B: Backend, const NB: usize>(
    adapter: &mut Dynamixel<B, NB>,
    goals: Option<Goals>,
    faults: &mut Faults,
) {
    let Some(goals) = goals else {
        return;
    };
    if goals.send(&mut adapter.backend, &adapter.groups).is_err() {
        for &index in goals.indices() {
            faults.bus_error.set(index);
        }
    }
}

/// Smoke tests of the per-tick path; the full suite is in `tests.rs`.
#[cfg(test)]
mod tests {
    use std::fs;
    use std::os::unix::fs::symlink;
    use std::path::{Path, PathBuf};

    use kmr_core::hal::{Hal, Layout, Overrides, Written};
    use kmr_core::state::{Qd, Tau};

    use super::super::scripted::{Call, Scripted};
    use super::super::{DynamixelError, Table, Torque, Zero};
    use super::*;
    use crate::binding::{Binding, Port};

    const PORT: Port = Port::ById("SMOKE");
    const TABLE: Table<1, JOINTS> = {
        let mut actuators = [Actuator {
            bus: 0,
            id: 0,
            model: Model::Xm430W210,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
        }; JOINTS];
        let mut index = 0;
        while index < JOINTS {
            actuators[index].id = index as u8 + 1;
            index += 1;
        }
        Table {
            buses: [Bus {
                label: "bus0",
                baud: 1_000_000,
                read_every: NonZeroU32::new(3),
            }],
            actuators,
        }
        .check()
    };

    /// The adapter checked, its ports open and its actuators powered; the
    /// backend log is cleared.
    fn adapter() -> (Dynamixel<Scripted, 1>, Layout) {
        let mut adapter = Dynamixel::new(Scripted::new(), TABLE);
        let claim = adapter.check(Overrides::default()).unwrap();
        let layout = Layout::new(
            claim.group,
            claim.read_every,
            claim.read_cost,
            claim.write_cost,
        );
        adapter.backend.open_ports(&[]).unwrap();
        adapter.open = true;
        adapter.torque_on().unwrap();
        adapter.backend.clear_log();
        (adapter, layout)
    }

    fn read(adapter: &mut Dynamixel<Scripted, 1>, faults: &mut Faults) -> Mask {
        let (mut q, mut qd, mut tau) = ([Q(9.0); JOINTS], [Qd(0.0); JOINTS], [Tau(0.0); JOINTS]);
        adapter.read(Sample::new(&mut q, &mut qd, &mut tau), faults)
    }

    #[test]
    fn one_group_reads_on_the_cadence_and_writes_once() {
        let (mut adapter, layout) = adapter();
        assert_eq!(layout.group, [Group(0); JOINTS]);
        assert!(layout.read_cost > Duration::ZERO);

        let mut faults = Faults::default();
        let read_mask: [bool; 4] =
            core::array::from_fn(|_| !read(&mut adapter, &mut faults).is_empty());
        assert_eq!(read_mask, [true, false, false, true]);
        assert_eq!(
            adapter.backend.count(Call::SyncReadState),
            2 * Scripted::STATE_READS.len()
        );

        let mut written = Written::default();
        written.q.set(1);
        written.qd.set(2);
        let q = [Q(0.0); JOINTS];
        let (qd, tau) = ([Qd(0.0); JOINTS], [Tau(0.0); JOINTS]);
        let batch = Batch::new(&layout, &written);
        adapter.write(Command::new(&q, &qd, &tau, &written), &batch, &mut faults);
        assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
        assert_eq!(adapter.backend.last_goals(), &[(2, 2048)]);
        assert!(faults.rejected.get(2) && !faults.rejected.get(1));
    }

    #[test]
    fn a_failed_group_read_is_a_bus_error() {
        let (mut adapter, _) = adapter();
        adapter.backend.fail_sync_read(0);
        let mut faults = Faults::default();
        assert!(read(&mut adapter, &mut faults).is_empty());
        assert_eq!(faults.bus_error, Mask::ALL);
        assert_eq!(faults.stale, Mask::ALL);
    }

    #[test]
    fn open_resolves_and_checks_before_flushing() {
        let root = std::env::temp_dir().join(format!("kmr_io_smoke_{}", std::process::id()));
        fs::remove_dir_all(&root).ok();
        fs::create_dir_all(&root).unwrap();
        symlink("/dev/null", root.join("usb-Smoke_SMOKE-if00-port0")).unwrap();
        let root: &'static Path = PathBuf::leak(root);

        let plugin = Binding {
            port: PORT,
            baud: None,
            bus: None,
        };
        let mut adapter = Dynamixel::bind(Scripted::new(), TABLE, &[plugin]).unwrap();
        adapter.check(Overrides::default()).unwrap();
        let mut adapter = adapter.with_roots(root, root);
        let error = adapter.open();
        fs::remove_dir_all(root).ok();

        // /dev/null passes the pre-flight but is no terminal to flush.
        assert_eq!(
            error,
            Err(DynamixelError::PortFlush {
                bus: "bus0",
                port: PORT
            })
        );
        assert_eq!(adapter.backend.count(Call::OpenPorts), 0);
    }
}
