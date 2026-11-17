//! The join of absent declared actuators while the loop runs (policy (b)):
//! each tick, at most one bounded piece of presence work, a join stage, the
//! check of a slot that missed reads, or a quiet probe.

use std::time::Instant;

use kmr_core::hal::Mask;
use tracing::{info, warn};

use super::backend::{Backend, BackendError, Setting, Undeclared};
use super::config::{Model, Torque};
use super::presence::{self, Event, Join, MAX_REJOINS, SETTLE_FOR, Stage};
use super::{Dynamixel, DynamixelError, io, telemetry};

/// The upkeep, under both policies only: the stage of the join in progress,
/// else the check of the lowest suspect slot, else a probe when one is due.
pub(super) fn upkeep<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
) {
    if !adapter.open || !adapter.presence.joins() {
        return;
    }
    if let Some(join) = adapter.presence.joining {
        advance(adapter, join);
    } else if let Some(index) = suspect(adapter) {
        check_suspect(adapter, index);
    } else {
        probe(adapter);
    }
}

/// The lowest live slot that missed a read then answered, whose actuator is
/// powered: a reboot inside the loss window leaves its torque off.
fn suspect<B, const NB: usize, const N: usize>(adapter: &Dynamixel<B, NB, N>) -> Option<usize> {
    let suspects = adapter.presence.suspect.intersection(adapter.presence.live);
    suspects.iter().find(|&index| {
        adapter
            .actuators
            .get(index)
            .is_some_and(|actuator| actuator.torque == Torque::On)
    })
}

/// One Torque Enable read of suspect slot `index`: on, no longer suspect;
/// off, restarted or held faulted. A read that fails leaves it suspect,
/// unless its bus holds it faulted: the miss count decides a silent one.
fn check_suspect<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    index: usize,
) {
    let Some(&actuator) = adapter.actuators.get(index) else {
        return;
    };
    let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
    let faulted = |adapter: &Dynamixel<B, NB, N>| adapter.backend.faulted(bus, model, id);
    match adapter
        .backend
        .read_setting(bus, model, id, Setting::TorqueEnable)
    {
        Ok(0) if faulted(adapter) => held_faulted(adapter, index),
        Ok(0) => restarted(adapter, index),
        Ok(_) => adapter.presence.suspect.clear(index),
        Err(_) if faulted(adapter) => held_faulted(adapter, index),
        // DECISION: any other failed read keeps the bit, a timeout as the
        // plan says and a garbled reply too: the next tick asks again.
        Err(_) => {}
    }
}

/// One probe of the next absent slot, when the period is over and this
/// tick sent no telemetry nor read that slot's bus.
fn probe<B: Backend, const NB: usize, const N: usize>(adapter: &mut Dynamixel<B, NB, N>) {
    let presence = &adapter.presence;
    if presence
        .last_probe
        .is_some_and(|last| last.elapsed() < presence.probe_period)
        || telemetry::sent(adapter)
    {
        return;
    }
    let candidates = adapter
        .actuators
        .owned()
        .difference(presence.driven())
        .difference(presence.refused);
    let Some(index) = presence.next_probe(candidates) else {
        return;
    };
    let (Some(number), Some(&actuator)) =
        (adapter.groups.table_of(index), adapter.actuators.get(index))
    else {
        return;
    };
    if telemetry::table_read_this_tick(adapter, number) {
        return;
    }
    adapter.presence.cursor = index + 1;
    adapter.presence.last_probe = Some(Instant::now());
    let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
    match adapter.backend.probe(bus, model, id) {
        Ok(()) => {
            let stage = if adapter.backend.holds(bus, model, id) {
                Stage::Health
            } else {
                Stage::Hold
            };
            adapter.presence.joining = Some(Join {
                index,
                stage,
                since: None,
            });
        }
        Err(BackendError::WrongModel { found }) => {
            let failure = Undeclared {
                id: Some(id),
                source: BackendError::WrongModel { found },
            };
            let error = adapter.undeclared(bus, failure);
            refuse(adapter, index, error);
        }
        Err(_) => {}
    }
}

/// Runs stage `join.stage` of the join in progress, then keeps the next one,
/// or ends the join: done, absent again, or refused.
fn advance<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    join: Join,
) {
    let index = join.index;
    let then = |stage| Some(Join { stage, ..join });
    let next = match join.stage {
        Stage::Hold => {
            hold(adapter, index);
            return;
        }
        Stage::Health => health(adapter, join),
        Stage::Settle => settle(adapter, join),
        Stage::Check => adapter
            .check_health(index, adapter.limit_writes(index))
            .map(|()| then(Stage::Configure)),
        Stage::Configure => adapter
            .configure_one(index)
            .and_then(|()| adapter.apply_settings(index))
            .map(|()| then(Stage::Power)),
        Stage::Power => power(adapter, index).map(|()| None),
    };
    adapter.presence.joining = None;
    match next {
        Ok(next) => adapter.presence.joining = next,
        Err(DynamixelError::Missing { .. }) => cut(adapter, index),
        Err(error) => {
            cut(adapter, index);
            refuse(adapter, index, error);
        }
    }
}

/// One torque off attempt to actuator `index`, its answer ignored: its bus
/// holds it from Health on, so nothing it holds is wiped.
fn cut<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    index: usize,
) {
    if let Some(&actuator) = adapter.actuators.get(index) {
        let _ = adapter
            .backend
            .force_torque_off(actuator.bus, actuator.model, actuator.id);
    }
}

/// The Hold: the bus of joiner `index` set up again with the actuators it
/// drives and the joiner, one attempt; on a failure, once more without the
/// joiner, and without an actuator found silent. The set up ones that read
/// their torque on are switched on again; see [`rearm_held`].
// DECISION (plan, DECISION 6): a failed fallback drops the bus's slots to
// absent, not refused: they rejoin one by one through the probe. The
// emergency stop alternative is the lead's (deferred row 1).
// DECISION: a driven actuator the first declare found silent is left out of
// the fallback and lost, so the rest stays held; the plan's fallback would
// stop at it again and drop the whole bus.
fn hold<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    index: usize,
) {
    adapter.presence.joining = None;
    let Some(&joiner) = adapter.actuators.get(index) else {
        return;
    };
    let bus = joiner.bus;
    let driven = adapter.presence.driven();
    let (mut on_bus, mut faulted) = (Mask::NONE, Mask::NONE);
    for (slot, actuator) in adapter.actuators.iter() {
        if actuator.bus == bus && driven.get(slot) {
            on_bus.set(slot);
            if adapter.backend.faulted(bus, actuator.model, actuator.id) {
                faulted.set(slot);
            }
        }
    }
    let mut with_joiner = on_bus;
    with_joiner.set(index);
    let (held, mut lost) = match declare(adapter, bus, with_joiner) {
        Ok(()) => {
            adapter.presence.joining = Some(Join {
                index,
                stage: Stage::Health,
                since: None,
            });
            (on_bus, Mask::NONE)
        }
        Err(failure) => {
            if failure.id == Some(joiner.id)
                && matches!(failure.source, BackendError::WrongModel { .. })
            {
                let error = adapter.undeclared(bus, failure);
                refuse(adapter, index, error);
            }
            let silent = named(adapter, on_bus, failure, BackendError::Timeout);
            let rest = on_bus.difference(silent);
            if let Err(fallback) = declare(adapter, bus, rest) {
                for slot in on_bus.iter() {
                    adapter.presence.drop_slot(slot);
                }
                adapter.report_slots(silent, Event::Lost);
                let error = adapter.undeclared(bus, fallback);
                adapter.each_bus(rest, |bus, ids| {
                    presence::report_failed(bus, ids, Event::Dropped, error);
                });
                return;
            }
            (rest, silent)
        }
    };
    lost = lost.union(rearm_held(adapter, held, faulted));
    for slot in lost.iter() {
        adapter.presence.drop_slot(slot);
    }
    adapter.report_slots(lost, Event::Lost);
}

/// Sets up bus `bus` with the actuators of `slots` on it, one attempt; no
/// call when there is none, and the bus then holds nothing.
fn declare<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    bus: usize,
    slots: Mask<N>,
) -> Result<(), Undeclared> {
    let (mut list, mut len) = ([(0, Model::Xm430W210); N], 0);
    for (slot, actuator) in adapter.actuators.iter() {
        if let (true, Some(entry)) = (actuator.bus == bus && slots.get(slot), list.get_mut(len)) {
            *entry = (actuator.id, actuator.model);
            len += 1;
        }
    }
    let list = list.get(..len).unwrap_or_default();
    if list.is_empty() {
        return Ok(());
    }
    adapter.backend.declare(bus, list)
}

/// The slot of `slots` a failed setup stopped at with `source`, as a mask.
fn named<B, const NB: usize, const N: usize>(
    adapter: &Dynamixel<B, NB, N>,
    slots: Mask<N>,
    failure: Undeclared,
    source: BackendError,
) -> Mask<N> {
    let mut found = Mask::NONE;
    if failure.source == source {
        for slot in slots.iter() {
            if adapter
                .actuators
                .get(slot)
                .is_some_and(|actuator| Some(actuator.id) == failure.id)
            {
                found.set(slot);
            }
        }
    }
    found
}

/// After a setup held `held` Off: refuses those held faulted, before it
/// (`faulted`) or by it; switches on again each powered one whose Torque
/// Enable reads 1; one reading 0 restarted. Returns those found silent.
// DECISION (plan, DECISION 5): only a slot not faulted that reads Torque
// Enable 1 is rearmed. Any failed read or rearm other than a fault is taken
// as a loss: no proof it is safe to power, and it rejoins through the probe.
fn rearm_held<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    held: Mask<N>,
    faulted: Mask<N>,
) -> Mask<N> {
    let mut lost = Mask::NONE;
    for slot in held.iter() {
        let Some(&actuator) = adapter.actuators.get(slot) else {
            continue;
        };
        let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
        if faulted.get(slot) || adapter.backend.faulted(bus, model, id) {
            held_faulted(adapter, slot);
            continue;
        }
        if actuator.torque != Torque::On {
            continue;
        }
        let answer = adapter
            .backend
            .read_setting(bus, model, id, Setting::TorqueEnable)
            .and_then(|on| match on {
                0 => Ok(false),
                _ => adapter.backend.rearm(bus, model, id).map(|()| true),
            });
        match answer {
            Ok(true) => {}
            Ok(false) => restarted(adapter, slot),
            Err(BackendError::Faulted { .. }) => held_faulted(adapter, slot),
            Err(_) if adapter.backend.faulted(bus, model, id) => held_faulted(adapter, slot),
            Err(_) => lost.set(slot),
        }
    }
    lost
}

/// The Health stage: a warm joiner (limits written this run, RAM intact) is
/// rearmed and seeded; a cold one is switched off and its limits written.
// DECISION: a read-only joiner configured this run and reading Torque
// Enable 0 is warm too, seeded as it is; a configured one whose Torque
// Enable read answers a fault goes the cold way, where Settle waits it out.
fn health<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    join: Join,
) -> Result<Option<Join>, DynamixelError> {
    let index = join.index;
    let Some(&actuator) = adapter.actuators.get(index) else {
        return Ok(None);
    };
    let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
    if adapter.presence.configured.get(index) {
        let on = match adapter
            .backend
            .read_setting(bus, model, id, Setting::TorqueEnable)
        {
            Ok(on) => Some(on != 0),
            Err(BackendError::Fault(_)) => None,
            Err(error) => return Err(adapter.actuator_error(index, error)),
        };
        match (actuator.torque, on) {
            (Torque::On, Some(true)) => {
                adapter
                    .backend
                    .rearm(bus, model, id)
                    .map_err(|error| adapter.actuator_error(index, error))?;
                seed(adapter, index);
                return Ok(None);
            }
            (Torque::ReadOnly, Some(false)) => {
                seed(adapter, index);
                return Ok(None);
            }
            _ => {}
        }
    }
    // DECISION: a fault in the reply still switched the torque off; Settle
    // asks the fault, so it is no reason to refuse the joiner here.
    match adapter.backend.force_torque_off(bus, model, id) {
        Ok(()) | Err(BackendError::Fault(_)) => {}
        Err(error) => return Err(adapter.actuator_error(index, error)),
    }
    let writes = adapter.limit_writes(index);
    if !writes.is_empty() {
        adapter.write_limits(index, actuator, writes)?;
    }
    Ok(Some(Join {
        stage: Stage::Settle,
        ..join
    }))
}

/// The Settle stage: a joiner its bus holds faulted is asked once per tick
/// whether its fault is gone, for at most [`SETTLE_FOR`] from the first ask;
/// one not faulted goes through Check in the same tick.
fn settle<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    join: Join,
) -> Result<Option<Join>, DynamixelError> {
    let index = join.index;
    let Some(&actuator) = adapter.actuators.get(index) else {
        return Ok(None);
    };
    let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
    if !adapter.backend.faulted(bus, model, id) {
        adapter.check_health(index, adapter.limit_writes(index))?;
        return Ok(Some(Join {
            stage: Stage::Configure,
            ..join
        }));
    }
    let since = join.since.unwrap_or_else(Instant::now);
    match adapter.backend.clear_fault_once(bus, model, id) {
        Ok(()) => Ok(Some(Join {
            stage: Stage::Check,
            ..join
        })),
        Err(BackendError::Fault(_)) if since.elapsed() < SETTLE_FOR => Ok(Some(Join {
            since: Some(since),
            ..join
        })),
        Err(error) => Err(adapter.actuator_error(index, error)),
    }
}

/// The Power stage: the start goal of joiner `index` alone, its present
/// position read now, then its torque on, or off for a read-only one; then
/// it is seeded.
fn power<B: Backend, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    index: usize,
) -> Result<(), DynamixelError> {
    let (Some(number), Some(&actuator)) =
        (adapter.groups.table_of(index), adapter.actuators.get(index))
    else {
        return Ok(());
    };
    let mut only = Mask::NONE;
    only.set(index);
    io::write_start_goals(adapter, number, only)?;
    let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
    match actuator.torque {
        Torque::On => adapter.backend.torque(bus, model, id, true),
        Torque::ReadOnly => adapter.backend.force_torque_off(bus, model, id),
    }
    .map_err(|error| adapter.actuator_error(index, error))?;
    seed(adapter, index);
    Ok(())
}

/// Joiner `index` is set up: read from the next tick on, driven once it
/// answered; one line.
fn seed<B, const NB: usize, const N: usize>(adapter: &mut Dynamixel<B, NB, N>, index: usize) {
    adapter.presence.seed.set(index);
    adapter.presence.configured.set(index);
    let (bus, id) = adapter.named(index);
    info!(bus, "actuator id {id} joined");
}

/// Slot `index` answered but could not be set up: never probed again this
/// run, one warning naming `error`.
// DECISION (plan, DECISION 4): a refused slot is not retried during the run.
fn refuse<B, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    index: usize,
    error: DynamixelError,
) {
    adapter.presence.refuse(index);
    let (bus, id) = adapter.named(index);
    warn!(bus, "actuator id {id} answered but was not set up: {error}");
}

/// Slot `index` is held faulted: not driven for the rest of the run, cut at
/// exit; one warning.
fn held_faulted<B, const NB: usize, const N: usize>(
    adapter: &mut Dynamixel<B, NB, N>,
    index: usize,
) {
    adapter.presence.refuse(index);
    let (bus, id) = adapter.named(index);
    warn!(bus, "actuator id {id} held faulted: not driven anymore");
}

/// Live slot `index` answers with its torque off: it lost its power. Out of
/// the run, to rejoin cold, until it did [`MAX_REJOINS`] times.
fn restarted<B, const NB: usize, const N: usize>(adapter: &mut Dynamixel<B, NB, N>, index: usize) {
    let (bus, id) = adapter.named(index);
    let presence = &mut adapter.presence;
    let rejoins = presence.rejoins.get_mut(index).map_or(u8::MAX, |rejoins| {
        *rejoins = rejoins.saturating_add(1);
        *rejoins
    });
    if rejoins > MAX_REJOINS {
        presence.refuse(index);
        warn!(
            bus,
            "actuator id {id} restarted {MAX_REJOINS} times: not driven anymore"
        );
    } else {
        presence.drop_slot(index);
        warn!(bus, "actuator id {id} restarted: set up again");
    }
}
