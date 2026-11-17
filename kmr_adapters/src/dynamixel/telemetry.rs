//! Telemetry rounds: input voltage, temperature and Hardware Error Status,
//! read a little at a time so the tick never waits on them (SRS FR-5.11,
//! NFR-1).
//!
//! A round is a list of steps, each one bus transaction: per Protocol 2
//! group, one sync read per register; per Protocol 1 actuator, one read.
//! The bring-up call takes every step; a tick takes at most one, every
//! [`TELEMETRY_EVERY`] ticks. Only live actuators are read: a step with
//! none sends nothing.

use kmr_core::hal::{Faults, Mask};
use kmr_core::telemetry::Telemetry;

use super::Dynamixel;
use super::backend::{Backend, Reading, Vital};
use super::config::Protocol;
use super::io::{Groups, Table};

/// Ticks between two telemetry transactions, at the least.
// DECISION: 50 ticks, 50 ms at 1 kHz: one Protocol 2 group refreshes every
// field in 150 ms, a 20-actuator Protocol 1 bus in 1 s. The adapter does not
// know dt, so the constant is in ticks; the NFR-1 bench (D9) may move it.
pub(crate) const TELEMETRY_EVERY: u32 = 50;

/// Where the telemetry rounds stand.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Rounds {
    /// Whether the bring-up call, which reads everything, was made.
    primed: bool,
    /// Calls since the last transaction.
    since: u32,
    /// The step the next transaction takes.
    next: usize,
}

impl Rounds {
    /// No call made yet.
    pub(crate) const NEW: Self = Self {
        primed: false,
        since: 0,
        next: 0,
    };
}

/// One telemetry transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    /// One register of every member of a Protocol 2 group: one sync read.
    Field(usize, Vital),
    /// Voltage and temperature of one Protocol 1 actuator: one read.
    One(usize, usize),
}

/// Steps one round of `table` takes.
fn steps_of<const N: usize>(table: &Table<N>) -> usize {
    match table.model.protocol() {
        Protocol::Two => Vital::ALL.len(),
        Protocol::One => table.members().len(),
    }
}

/// Steps one round takes: every group's.
fn steps<const N: usize>(groups: &Groups<N>) -> usize {
    groups.tables().iter().map(steps_of).sum()
}

/// Step `step` of a round, groups in order.
fn step<const N: usize>(groups: &Groups<N>, mut step: usize) -> Option<Step> {
    for (number, table) in groups.tables().iter().enumerate() {
        let len = steps_of(table);
        if step < len {
            return match table.model.protocol() {
                Protocol::Two => Vital::ALL
                    .get(step)
                    .map(|&vital| Step::Field(number, vital)),
                Protocol::One => Some(Step::One(number, step)),
            };
        }
        step -= len;
    }
    None
}

/// The telemetry call: every step on the first call, PostInit; then one
/// step once [`TELEMETRY_EVERY`] calls have passed since the last, in the
/// first tick whose state read left the step's bus alone (FR-009).
pub(super) fn refresh<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    telemetry: &mut Telemetry<N>,
    faults: &mut Faults<N>,
) -> Mask<N> {
    let total = steps(&adapter.groups);
    if !adapter.rounds.primed {
        adapter.rounds.primed = true;
        return (0..total).fold(Mask::NONE, |refreshed, number| {
            let taken = step(&adapter.groups, number)
                .map_or(Mask::NONE, |step| take(adapter, step, telemetry, faults));
            refreshed.union(taken)
        });
    }
    adapter.rounds.since = adapter.rounds.since.saturating_add(1);
    if total == 0 || adapter.rounds.since < TELEMETRY_EVERY {
        return Mask::NONE;
    }
    let number = adapter.rounds.next % total;
    let Some(next) = step(&adapter.groups, number) else {
        return Mask::NONE;
    };
    if read_this_tick(adapter, next) {
        return Mask::NONE;
    }
    // DECISION: a step with no live actuator is passed over and `since` left
    // as it is, so the next step goes on the next tick: absent slots never
    // stretch the round.
    if picked(adapter, next).is_none() {
        adapter.rounds.next = (number + 1) % total;
        return Mask::NONE;
    }
    adapter.rounds.since = 0;
    adapter.rounds.next = (number + 1) % total;
    take(adapter, next, telemetry, faults)
}

/// Whether this tick's telemetry call sent a transaction.
pub(super) fn sent<B, const NB: usize, const N: usize>(adapter: &Dynamixel<B, NB, N>) -> bool {
    adapter.rounds.since == 0
}

/// Whether this tick's `read` read the bus of `step`. A bus read every tick
/// always was: the step goes then anyway.
fn read_this_tick<B, const NB: usize, const N: usize>(
    adapter: &Dynamixel<B, NB, N>,
    step: Step,
) -> bool {
    let (Step::Field(number, _) | Step::One(number, _)) = step;
    table_read_this_tick(adapter, number)
}

/// Whether this tick's `read` read the bus of group `number`, as
/// [`read_this_tick`] tells it.
pub(super) fn table_read_this_tick<B, const NB: usize, const N: usize>(
    adapter: &Dynamixel<B, NB, N>,
    number: usize,
) -> bool {
    let every = adapter
        .groups
        .tables()
        .get(number)
        .map_or(1, |table| u64::from(table.every.get()));
    let call = adapter.reads.checked_sub(1);
    every > 1 && call.is_some_and(|call| call.is_multiple_of(every))
}

/// The live actuators `step` reads, at most `N`.
#[derive(Clone, Copy, Debug)]
struct Picked<const N: usize> {
    /// Bus and model of their group.
    table: Table<N>,
    /// The register read.
    vital: Vital,
    /// Engine indices, `len` of them used.
    members: [usize; N],
    /// Firmware ids, in the order of `members`.
    ids: [u8; N],
    /// Entries in use.
    len: usize,
}

/// What `step` reads of the live actuators; `None` when it reads none.
fn picked<B, const NB: usize, const N: usize>(
    adapter: &Dynamixel<B, NB, N>,
    step: Step,
) -> Option<Picked<N>> {
    let (Step::Field(number, _) | Step::One(number, _)) = step;
    let table = adapter.groups.tables().get(number).copied()?;
    let (vital, range) = match step {
        Step::Field(_, vital) => (vital, 0..table.members().len()),
        Step::One(_, member) => (Vital::Voltage, member..member + 1),
    };
    let mut picked = Picked {
        table,
        vital,
        members: [0; N],
        ids: [0; N],
        len: 0,
    };
    let members = table.members().get(range.clone()).unwrap_or_default();
    let ids = table.ids().get(range).unwrap_or_default();
    for (&index, &id) in members.iter().zip(ids) {
        if let (true, Some(member), Some(slot)) = (
            adapter.presence.live.get(index),
            picked.members.get_mut(picked.len),
            picked.ids.get_mut(picked.len),
        ) {
            (*member, *slot) = (index, id);
            picked.len += 1;
        }
    }
    (picked.len > 0).then_some(picked)
}

/// Sends `step` to its live actuators, stores what it read and returns the
/// actuators it read; nothing is sent when none is live. A failed read
/// keeps the previous values and sets `bus_error` or `offline` as `read`
/// does, a failed group transaction `bus_error` on all of it; never
/// `stale`, which is the state's.
fn take<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    step: Step,
    telemetry: &mut Telemetry<N>,
    faults: &mut Faults<N>,
) -> Mask<N> {
    let Some(Picked {
        table,
        vital,
        members,
        ids,
        len,
    }) = picked(adapter, step)
    else {
        return Mask::NONE;
    };
    let members = members.get(..len).unwrap_or_default();
    let ids = ids.get(..len).unwrap_or_default();
    let mut out = [Ok(Reading::default()); N];
    let out = out.get_mut(..ids.len()).unwrap_or_default();
    let mut refreshed = Mask::NONE;
    let read = adapter
        .backend
        .read_vitals(table.bus, table.model, vital, ids, out);
    if read.is_err() {
        members
            .iter()
            .for_each(|&index| faults.bus_error.set(index));
        return refreshed;
    }
    for (&index, result) in members.iter().zip(out.iter()) {
        match result {
            Ok(reading) => {
                store(telemetry, index, reading);
                refreshed.set(index);
            }
            Err(error) if error.is_bus() => faults.bus_error.set(index),
            Err(_) => faults.offline.set(index),
        }
    }
    refreshed
}

/// Writes `reading` into actuator `index`'s telemetry, in volts and °C.
fn store<const N: usize>(telemetry: &mut Telemetry<N>, index: usize, reading: &Reading) {
    if let Some(decivolts) = reading.voltage_dv {
        telemetry.set_voltage_v(index, f32::from(decivolts) / 10.0);
    }
    if let Some(celsius) = reading.temperature_c {
        telemetry.set_temperature_c(index, f32::from(celsius));
    }
    if let Some(bits) = reading.hardware_error {
        telemetry.set_hardware_error(index, bits);
    }
}
