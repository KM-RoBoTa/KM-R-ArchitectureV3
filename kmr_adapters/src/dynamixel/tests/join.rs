//! The join of declared actuators at run time: the quiet probe, the join
//! stages one per tick, the Hold that re-declares a bus, and the checks of
//! slots that missed reads (runtime-actuators plan, T6). Every scenario runs
//! on a strict fake, which keeps what `dxl_rs_v2` holds.

use kmr_core::hal::{JoinPolicy, MissingPolicy};
use tracing::Level;

use super::presence::{logged, powered_with, reads, strict, strict_placed, tolerate};
use super::*;
use crate::dynamixel::backend::{Fault, Reply};
use crate::dynamixel::presence::{LOST_AFTER, MAX_REJOINS, Stage};

/// An hour: a probe period no test outlives.
const HOUR: Duration = Duration::from_secs(3600);

/// `Overrides` with both policies on.
fn joining() -> Overrides {
    let mut overrides = tolerate();
    overrides.join = JoinPolicy::Declared;
    overrides
}

/// [`powered_with`] `overrides`, then the PostInit telemetry call, as the
/// engine runs them.
fn brought_up<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    overrides: Overrides,
) -> Result<Claim<N>, DynamixelError> {
    let claim = powered_with(adapter, overrides)?;
    refresh(adapter, &mut Telemetry::default());
    adapter.backend.clear_log();
    Ok(claim)
}

/// Brought up under both policies, then probing on every eligible tick.
fn joined_rig<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
) -> Result<Claim<N>, DynamixelError> {
    let claim = brought_up(adapter, joining())?;
    adapter.presence.probe_period = Duration::ZERO;
    Ok(claim)
}

/// One tick as the engine runs it: `read`, a write of every `Q`, the
/// telemetry call, then `upkeep`. The write's faults.
fn tick<const NB: usize>(adapter: &mut Dynamixel<Scripted, NB, N>, claim: &Claim<N>) -> Faults<N> {
    read(adapter, &mut Slot::filled(0.0));
    let faults = write(adapter, claim, &Desired::all_q());
    refresh(adapter, &mut Telemetry::default());
    adapter.upkeep();
    faults
}

/// `times` ticks.
fn ticks<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    claim: &Claim<N>,
    times: usize,
) {
    for _ in 0..times {
        tick(adapter, claim);
    }
}

/// Ticks until slot `index` is present, at most `limit`: how many it took.
fn until_present<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    claim: &Claim<N>,
    index: usize,
    limit: usize,
) -> Option<usize> {
    (1..=limit).find(|_| {
        tick(adapter, claim);
        adapter.present().get(index)
    })
}

/// The stage of the join in progress, if any.
fn stage<const NB: usize>(adapter: &Dynamixel<Scripted, NB, N>) -> Option<(usize, Stage)> {
    adapter
        .presence
        .joining
        .map(|join| (join.index, join.stage))
}

/// Ids probed, in call order.
fn probed(backend: &Scripted) -> impl Iterator<Item = u8> + '_ {
    calls_of(backend, Call::Probe)
}

/// First ids of the records of `call`, in call order.
fn calls_of(backend: &Scripted, call: Call) -> impl Iterator<Item = u8> + '_ {
    backend
        .calls()
        .iter()
        .filter(move |record| record.call == call)
        .map(|record| record.id)
}

/// Whether the newest `Declare` names exactly `ids`, in order.
fn last_declared<const K: usize>(backend: &Scripted, ids: [u8; K]) -> bool {
    backend
        .last(Call::Declare)
        .is_some_and(|record| backend.ids(record).eq(ids))
}

/// [`AX_TABLE`] in order, every actuator writing Min and Max Voltage Limit
/// 9.0–13.0 V, put back at `close` when `restore`.
fn voltage_limited(restore: bool) -> Placement<1, N> {
    let mut placement = AX_TABLE.in_order();
    placement.settings = [Settings {
        voltage_limits_dv: Some((90, 130)),
        restore_limits: restore,
        ..Settings::NONE
    }; N];
    placement
}

// ── Probing ─────────────────────────────────────────────────────────────────

#[test]
fn join_off_never_probes() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = brought_up(&mut adapter, tolerate())?;
    adapter.presence.probe_period = Duration::ZERO;
    adapter.backend.plug(0, id(2), true);

    ticks(&mut adapter, &claim, 20);

    assert_eq!(adapter.backend.count(Call::Probe), 0);
    assert_eq!(adapter.backend.count(Call::Declare), 0);
    assert_eq!(adapter.present(), mask(|index| index != 2));
    Ok(())
}

#[test]
fn join_without_tolerate_warns_at_check() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    let mut alone = Overrides::default();
    alone.join = JoinPolicy::Declared;
    const NO_EFFECT: &str = "the join policy has no effect without MissingPolicy::Tolerate";

    assert_eq!(
        warnings(NO_EFFECT, || assert!(adapter.check(alone).is_ok())),
        1
    );
    assert_eq!(
        warnings(NO_EFFECT, || assert!(adapter.check(joining()).is_ok())),
        0
    );

    let claim = brought_up(&mut adapter, alone)?;
    adapter.presence.probe_period = Duration::ZERO;
    ticks(&mut adapter, &claim, 5);
    assert_eq!(adapter.presence.missing, MissingPolicy::Fail);
    assert_eq!(adapter.backend.count(Call::Probe), 0);
    Ok(())
}

#[test]
fn probe_waits_for_its_period() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = brought_up(&mut adapter, joining())?;
    tick(&mut adapter, &claim);
    assert_eq!(
        adapter.backend.count(Call::Probe),
        0,
        "the first probe is one period after bring-up"
    );

    adapter.presence.probe_period = HOUR;
    adapter.presence.last_probe = None;
    ticks(&mut adapter, &claim, 10);

    assert!(probed(&adapter.backend).eq([id(2)]));
    Ok(())
}

#[test]
fn probe_is_round_robin_one_per_eligible_tick() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(1)).missing(0, id(3));
    let claim = joined_rig(&mut adapter)?;

    for tick_number in 1..=4 {
        tick(&mut adapter, &claim);
        assert_eq!(adapter.backend.count(Call::Probe), tick_number);
    }

    assert!(probed(&adapter.backend).eq([id(1), id(3), id(1), id(3)]));
    Ok(())
}

#[test]
fn probe_skips_ticks_that_read_its_bus_or_sent_telemetry() -> Result<(), DynamixelError> {
    let mut adapter = strict(every(AX_TABLE, 2));
    adapter.backend.missing(0, id(2));
    joined_rig(&mut adapter)?;
    let mut telemetry = Telemetry::default();

    let (mut probes, mut telemetered) = (0, 0);
    for tick_number in 0..2 * EVERY + 2 {
        adapter.backend.clear_log();
        read(&mut adapter, &mut Slot::filled(0.0));
        let bus_read = adapter.backend.count(Call::ReadPosition) > 0;
        refresh(&mut adapter, &mut telemetry);
        let sent = adapter.backend.count(Call::ReadVitals) > 0;
        adapter.upkeep();
        let probe = adapter.backend.count(Call::Probe) > 0;
        assert_eq!(probe, !bus_read && !sent, "tick {tick_number}");
        probes += usize::from(probe);
        telemetered += usize::from(sent);
    }
    assert!(telemetered >= 2, "telemetry took its ticks");
    assert_eq!(probes, EVERY + 1 - telemetered);
    Ok(())
}

#[test]
fn probe_of_a_silent_id_changes_nothing() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    let before = adapter.presence;

    ticks(&mut adapter, &claim, 5);

    assert_eq!(adapter.backend.count(Call::Probe), 5);
    assert_eq!(adapter.backend.count(Call::Declare), 0);
    assert_eq!(adapter.presence.live, before.live);
    assert_eq!(adapter.presence.seed, Mask::NONE);
    assert_eq!(adapter.presence.refused, Mask::NONE);
    assert_eq!(stage(&adapter), None);
    for index in [0, 1, 3] {
        assert!(adapter.backend.holds(0, Model::Ax12a, id(index)));
    }
    Ok(())
}

// ── The join stages ─────────────────────────────────────────────────────────

#[test]
fn answering_id_is_held_configured_powered_then_present_after_its_first_read()
-> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.plug(0, id(2), true);

    let expected = [
        Some((2, Stage::Hold)),
        Some((2, Stage::Health)),
        Some((2, Stage::Settle)),
        Some((2, Stage::Configure)),
        Some((2, Stage::Power)),
        None,
    ];
    for (tick_number, expected) in expected.into_iter().enumerate() {
        tick(&mut adapter, &claim);
        assert_eq!(stage(&adapter), expected, "tick {tick_number}");
        assert!(!adapter.present().get(2), "tick {tick_number}");
    }
    assert!(adapter.backend.holds(0, Model::Ax12a, id(2)));
    assert!(adapter.backend.torqued(0, id(2)));
    assert!(adapter.presence.seed.get(2) && adapter.presence.configured.get(2));
    assert!(adapter.backend.count(Call::SyncWriteStartGoal) == 1);

    let (refreshed, faults) = read(&mut adapter, &mut Slot::filled(0.0));

    assert_eq!(faults, Faults::default());
    assert_eq!(refreshed, mask(|_| true));
    assert_eq!(adapter.present(), mask(|_| true));
    assert_eq!(adapter.presence.seed, Mask::NONE);
    Ok(())
}

#[test]
fn join_redeclares_the_bus_and_rearms_the_others() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.plug(0, id(2), true);
    tick(&mut adapter, &claim);
    adapter.backend.clear_log();

    tick(&mut adapter, &claim);

    assert_eq!(adapter.backend.count(Call::Declare), 1);
    assert!(last_declared(
        &adapter.backend,
        [id(0), id(1), id(2), id(3)]
    ));
    assert!(calls_of(&adapter.backend, Call::Rearm).eq([id(0), id(1), id(3)]));
    adapter.backend.clear_log();
    let faults = tick(&mut adapter, &claim);
    assert_eq!(faults, Faults::default());
    let sent = adapter.backend.last(Call::SyncWriteGoal).unwrap();
    assert!(adapter.backend.ids(sent).eq([id(0), id(1), id(3)]));
    Ok(())
}

#[test]
fn hold_rearms_only_slots_reading_torque_on() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    // Slot 0 lost its power within one tick: back with its torque off.
    adapter.backend.plug(0, id(0), true).plug(0, id(2), true);
    tick(&mut adapter, &claim);
    adapter.backend.clear_log();

    let seen = warnings("actuator id 1 restarted: set up again", || {
        tick(&mut adapter, &claim);
    });

    assert_eq!(seen, 1);
    assert!(calls_of(&adapter.backend, Call::Rearm).eq([id(1), id(3)]));
    assert!(!adapter.backend.torqued(0, id(0)));
    assert_eq!(adapter.present(), mask(|index| index == 1 || index == 3));
    assert!(!adapter.presence.refused.get(0));
    assert!(until_present(&mut adapter, &claim, 0, 10).is_some());
    assert_eq!(adapter.presence.rejoins[0], 1, "counted once powered again");
    Ok(())
}

#[test]
fn hold_never_rearms_a_faulted_slot() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.plug(0, id(2), true);
    tick(&mut adapter, &claim);
    adapter.backend.health(0, id(0), vitals(50, 30));
    adapter.backend.clear_log();

    let seen = warnings("actuator id 1 held faulted: not driven anymore", || {
        tick(&mut adapter, &claim);
    });

    assert_eq!(seen, 1);
    assert!(calls_of(&adapter.backend, Call::Rearm).eq([id(1), id(3)]));
    assert!(adapter.presence.refused.get(0));
    assert!(!adapter.present().get(0));
    ticks(&mut adapter, &claim, 10);
    assert!(!probed(&adapter.backend).any(|probed| probed == id(0)));
    Ok(())
}

#[test]
fn hold_failure_falls_back_to_the_previous_list() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.plug(0, id(2), true);
    tick(&mut adapter, &claim);
    let corrupt = BackendError::Reply(Reply::Corrupt);
    adapter
        .backend
        .fail_times(Call::Declare, 0, id(2), corrupt, 1);
    adapter.backend.clear_log();

    tick(&mut adapter, &claim);

    let backend = &adapter.backend;
    let mut declares = backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::Declare);
    let with_joiner = declares.next().map(|record| backend.ids(record));
    assert!(with_joiner.is_some_and(|ids| ids.eq([id(0), id(1), id(2), id(3)])));
    let fallback = declares.next().map(|record| backend.ids(record));
    assert!(fallback.is_some_and(|ids| ids.eq([id(0), id(1), id(3)])));
    assert!(declares.next().is_none());
    assert_eq!(stage(&adapter), None);
    assert!(!adapter.presence.refused.get(2), "absent again, quietly");
    assert!(!adapter.backend.holds(0, Model::Ax12a, id(2)));
    assert_eq!(adapter.present(), mask(|index| index != 2));
    assert!(calls_of(&adapter.backend, Call::Rearm).eq([id(0), id(1), id(3)]));
    adapter.backend.clear_log();
    assert_eq!(tick(&mut adapter, &claim), Faults::default());
    let sent = adapter.backend.last(Call::SyncWriteGoal).unwrap();
    assert!(adapter.backend.ids(sent).eq([id(0), id(1), id(3)]));

    assert!(
        until_present(&mut adapter, &claim, 2, 10).is_some(),
        "joins later"
    );
    Ok(())
}

#[test]
fn hold_double_failure_drops_the_bus_and_exit_still_cuts_it() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.plug(0, id(2), true);
    tick(&mut adapter, &claim);
    adapter
        .backend
        .fail_port(Call::Declare, 0, BackendError::Io(Wire::Closed));

    let line = (
        "dropped",
        "ids 1 to 2 dropped: their bus setup failed",
        "ax",
    );
    let seen = logged(Level::WARN, line, || {
        tick(&mut adapter, &claim);
    });

    assert_eq!(seen, (2, 1), "ids 1 to 2, then id 4");
    assert_eq!(adapter.backend.count(Call::Declare), 2);
    assert_eq!(adapter.present(), Mask::NONE);
    assert_eq!(adapter.presence.refused, Mask::NONE);
    assert_eq!(stage(&adapter), None);

    adapter.backend.heal();
    assert!(
        until_present(&mut adapter, &claim, 0, 10).is_some(),
        "rejoins through the probe"
    );
    assert_eq!(
        adapter.backend.count(Call::ForceTorqueOff),
        0,
        "warm: rearmed, never set up again"
    );
    adapter.backend.clear_log();
    adapter.torque_off();
    adapter.close();
    for index in 0..N {
        assert!(
            forced(&adapter.backend).any(|cut| cut == id(index)),
            "actuator {index} cut at exit"
        );
    }
    Ok(())
}

#[test]
fn a_present_id_silent_at_redeclare_is_dropped_and_the_rest_held() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.plug(0, id(2), true);
    tick(&mut adapter, &claim);
    adapter.backend.missing(0, id(3));
    adapter.backend.clear_log();

    let line = ("not available", "actuator id 4 not available anymore", "ax");
    let seen = logged(Level::WARN, line, || {
        tick(&mut adapter, &claim);
    });

    assert_eq!(seen, (1, 1));
    assert!(last_declared(&adapter.backend, [id(0), id(1)]));
    assert_eq!(adapter.present(), mask(|index| index < 2));
    assert!(adapter.backend.holds(0, Model::Ax12a, id(0)));
    assert!(adapter.backend.holds(0, Model::Ax12a, id(1)));
    assert!(calls_of(&adapter.backend, Call::Rearm).eq([id(0), id(1)]));
    assert_eq!(adapter.presence.refused, Mask::NONE);
    assert_eq!(stage(&adapter), None);
    Ok(())
}

#[test]
fn whole_bus_absent_at_open_then_one_joins() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    for index in 0..N {
        adapter.backend.missing(0, id(index));
    }
    let claim = joined_rig(&mut adapter)?;
    assert_eq!(adapter.present(), Mask::NONE);
    for index in 0..N {
        assert!(!adapter.backend.holds(0, Model::Ax12a, id(index)));
    }
    adapter.backend.plug(0, id(1), true);

    tick(&mut adapter, &claim);
    tick(&mut adapter, &claim);
    assert_eq!(stage(&adapter), Some((1, Stage::Hold)));
    adapter.backend.clear_log();
    tick(&mut adapter, &claim);

    assert!(last_declared(&adapter.backend, [id(1)]));
    assert!(until_present(&mut adapter, &claim, 1, 10).is_some());
    assert_eq!(adapter.present(), mask(|index| index == 1));
    Ok(())
}

#[test]
fn rejoin_of_a_held_slot_skips_hold() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.missing(0, id(2));
    reads(&mut adapter, LOST_AFTER);
    assert!(!adapter.present().get(2));
    assert!(adapter.backend.holds(0, Model::Ax12a, id(2)));
    adapter.backend.plug(0, id(2), true);
    adapter.backend.clear_log();

    tick(&mut adapter, &claim);
    assert_eq!(stage(&adapter), Some((2, Stage::Health)));
    assert!(until_present(&mut adapter, &claim, 2, 10).is_some());

    assert_eq!(adapter.backend.count(Call::Declare), 0);
    Ok(())
}

#[test]
fn joiner_start_goal_is_its_own_present_position_alone() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.plug(0, id(2), true).position(0, id(2), 600);
    for _ in 0..10 {
        if stage(&adapter) == Some((2, Stage::Power)) {
            break;
        }
        tick(&mut adapter, &claim);
    }
    assert_eq!(stage(&adapter), Some((2, Stage::Power)));
    read(&mut adapter, &mut Slot::filled(0.0));
    write(&mut adapter, &claim, &Desired::all_q());
    adapter.backend.clear_log();

    adapter.upkeep();

    assert!(calls_of(&adapter.backend, Call::ReadPosition).eq([id(2)]));
    let start = adapter.backend.last(Call::SyncWriteStartGoal).unwrap();
    assert_eq!(adapter.backend.goals(start), &[(id(2), 600)]);
    assert_eq!(adapter.backend.count(Call::SyncWriteStartGoal), 1);
    assert!(torque_asked(&adapter.backend, 0, id(2), true));
    Ok(())
}

#[test]
fn first_join_settles_a_voltage_flag_across_ticks() -> Result<(), DynamixelError> {
    let mut adapter = strict_placed(voltage_limited(false));
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    // Bench AX-12A at 10.1 V with a Min Voltage Limit of 10.5 V: flagged
    // until 9.0 V is written, and its flag lags that write.
    let low = Health {
        voltage_dv: 101,
        min_voltage_dv: 105,
        max_voltage_dv: 140,
        ..HEALTHY
    };
    let flag = BackendError::Fault(Fault::Voltage {
        measured_dv: 101,
        min_dv: 90,
        max_dv: 130,
    });
    adapter
        .backend
        .health(0, id(2), low)
        .plug(0, id(2), true)
        .fail_times(Call::ClearFaultOnce, 0, id(2), flag, 2);

    let mut settling = 0;
    for tick_number in 0..12 {
        if adapter.present().get(2) {
            break;
        }
        adapter.backend.clear_log();
        let settles = stage(&adapter) == Some((2, Stage::Settle));
        tick(&mut adapter, &claim);
        settling += usize::from(settles);
        let eeprom = adapter.backend.count(Call::WriteVoltageLimits)
            + adapter.backend.count(Call::WriteTemperatureLimit);
        assert!(eeprom <= 2, "tick {tick_number}: {eeprom} EEPROM settles");
        assert_eq!(
            adapter.backend.count(Call::ClearFault),
            0,
            "tick {tick_number}: no settle loop"
        );
        assert!(adapter.backend.count(Call::ClearFaultOnce) <= 1);
    }

    assert!(adapter.present().get(2), "joined");
    assert_eq!(settling, 3, "two flagged asks, then a clean one");
    assert!(!adapter.presence.refused.get(2));
    assert_eq!(
        adapter
            .backend
            .limits(0, id(2))
            .map(|limits| (limits.min_voltage_dv, limits.max_voltage_dv)),
        Some((90, 130))
    );
    Ok(())
}

#[test]
fn cold_rejoin_rewrites_limits_and_keeps_the_first_originals() -> Result<(), DynamixelError> {
    let mut adapter = strict_placed(voltage_limited(true));
    let claim = joined_rig(&mut adapter)?;
    let originals = adapter.saved[2];
    assert_eq!(originals.voltage_dv, Some((60, 190)));
    adapter.backend.missing(0, id(2));
    reads(&mut adapter, LOST_AFTER);
    adapter.backend.plug(0, id(2), true);
    adapter.backend.clear_log();

    assert!(until_present(&mut adapter, &claim, 2, 10).is_some());

    assert!(calls_of(&adapter.backend, Call::WriteVoltageLimits).eq([id(2)]));
    assert_eq!(adapter.saved[2], originals);
    Ok(())
}

#[test]
fn warm_rejoin_is_rearmed_without_reconfigure() -> Result<(), DynamixelError> {
    let mut adapter = strict_placed(voltage_limited(false));
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.missing(0, id(2));
    reads(&mut adapter, LOST_AFTER);
    adapter.backend.plug(0, id(2), false);
    adapter.backend.clear_log();

    let line = ("joined", "actuator id 3 joined", "ax");
    let seen = logged(Level::INFO, line, || {
        tick(&mut adapter, &claim);
        tick(&mut adapter, &claim);
    });

    assert_eq!(seen, (1, 1));
    assert!(adapter.presence.seed.get(2));
    assert!(calls_of(&adapter.backend, Call::Rearm).eq([id(2)]));
    for call in [
        Call::WriteVoltageLimits,
        Call::ReadAngleLimits,
        Call::ReadHealth,
        Call::SyncWriteStartGoal,
        Call::ForceTorqueOff,
    ] {
        assert_eq!(adapter.backend.count(call), 0, "{call:?}");
    }
    read(&mut adapter, &mut Slot::filled(0.0));
    assert!(adapter.present().get(2));
    Ok(())
}

#[test]
fn join_failure_cuts_the_joiner_warns_once_and_is_not_retried() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter
        .backend
        .plug(0, id(2), true)
        .angle_limits(0, id(2), 0, 0);

    let seen = warnings("actuator id 3 answered but was not set up", || {
        ticks(&mut adapter, &claim, 12);
    });

    assert_eq!(seen, 1);
    assert!(adapter.presence.refused.get(2));
    assert!(!adapter.present().get(2));
    assert!(!adapter.backend.torqued(0, id(2)));
    assert!(forced(&adapter.backend).any(|cut| cut == id(2)));
    assert_eq!(adapter.backend.count(Call::Probe), 1);
    Ok(())
}

#[test]
fn wrong_model_at_probe_warns_once() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter
        .backend
        .plug(0, id(2), true)
        .model_number(0, id(2), Model::Rx28.number());

    let seen = warnings("actuator id 3 answered but was not set up", || {
        ticks(&mut adapter, &claim, 10);
    });

    assert_eq!(seen, 1);
    assert_eq!(adapter.backend.count(Call::Probe), 1);
    assert_eq!(adapter.backend.count(Call::Declare), 0);
    assert!(adapter.presence.refused.get(2));
    Ok(())
}

// ── Slots that missed a read, then answered ─────────────────────────────────

/// Slot `index`, live, misses one read then answers with its torque off, as
/// after a power loss: the tick that sees it answer checks it.
fn restart<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    claim: &Claim<N>,
    index: usize,
) {
    adapter.backend.missing(0, id(index));
    tick(adapter, claim);
    adapter.backend.plug(0, id(index), true);
    tick(adapter, claim);
}

#[test]
fn suspect_slot_with_torque_off_rejoins_cold() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    let claim = joined_rig(&mut adapter)?;

    let seen = warnings("actuator id 3 restarted: set up again", || {
        restart(&mut adapter, &claim, 2);
    });

    assert_eq!(seen, 1);
    assert!(!adapter.present().get(2));
    assert_eq!(adapter.presence.suspect, Mask::NONE);
    adapter.backend.clear_log();
    assert!(until_present(&mut adapter, &claim, 2, 10).is_some());
    assert_eq!(adapter.backend.count(Call::Declare), 0, "still held");
    assert_eq!(adapter.backend.count(Call::Rearm), 0, "cold");
    assert!(forced(&adapter.backend).any(|cut| cut == id(2)));
    assert!(torque_asked(&adapter.backend, 0, id(2), true));
    Ok(())
}

#[test]
fn a_suspect_slot_still_powered_stays_live() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    let claim = joined_rig(&mut adapter)?;
    adapter.backend.missing(0, id(2));
    tick(&mut adapter, &claim);
    adapter.backend.plug(0, id(2), false);

    tick(&mut adapter, &claim);

    assert_eq!(adapter.presence.suspect, Mask::NONE);
    assert_eq!(adapter.present(), mask(|_| true));
    Ok(())
}

#[test]
fn rejoins_stop_after_max_rejoins() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    let claim = joined_rig(&mut adapter)?;
    for rejoin in 1..=MAX_REJOINS {
        restart(&mut adapter, &claim, 2);
        assert!(!adapter.presence.refused.get(2), "rejoin {rejoin}");
        assert!(
            until_present(&mut adapter, &claim, 2, 10).is_some(),
            "rejoin {rejoin}"
        );
    }

    let line = "actuator id 3 restarted 3 times: not driven anymore";
    let seen = warnings(line, || restart(&mut adapter, &claim, 2));

    assert_eq!(seen, 1);
    assert!(adapter.presence.refused.get(2));
    adapter.backend.clear_log();
    ticks(&mut adapter, &claim, 10);
    assert_eq!(adapter.backend.count(Call::Probe), 0);
    assert!(!adapter.present().get(2));
    Ok(())
}

#[test]
fn second_run_starts_from_a_fresh_presence() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = joined_rig(&mut adapter)?;
    adapter
        .backend
        .plug(0, id(2), true)
        .model_number(0, id(2), Model::Rx28.number());
    ticks(&mut adapter, &claim, 3);
    restart(&mut adapter, &claim, 1);
    assert!(until_present(&mut adapter, &claim, 1, 10).is_some());
    assert!(adapter.presence.refused.get(2));
    assert_eq!(adapter.presence.rejoins[1], 1);
    adapter.torque_off();
    adapter.close();

    adapter.check(joining())?;

    let fresh = adapter.presence;
    assert_eq!(fresh.live, mask(|_| true));
    assert_eq!(fresh.refused, Mask::NONE);
    assert_eq!(fresh.seed, Mask::NONE);
    assert_eq!(fresh.configured, Mask::NONE);
    assert_eq!(fresh.suspect, Mask::NONE);
    assert_eq!(fresh.rejoins, [0; N]);
    assert_eq!(fresh.joining, None);
    assert_eq!(fresh.last_probe, None);
    assert_eq!(fresh.join, JoinPolicy::Declared);
    Ok(())
}

#[test]
fn long_power_losses_count_as_rejoins_too() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    let claim = joined_rig(&mut adapter)?;
    let lose = |adapter: &mut Dynamixel<Scripted, 1, N>| {
        adapter.backend.missing(0, id(2));
        ticks(adapter, &claim, usize::from(LOST_AFTER));
        assert!(!adapter.present().get(2));
        adapter.backend.plug(0, id(2), true);
    };
    for rejoin in 1..=MAX_REJOINS {
        lose(&mut adapter);
        assert!(
            until_present(&mut adapter, &claim, 2, 10).is_some(),
            "rejoin {rejoin}"
        );
        assert_eq!(adapter.presence.rejoins[2], rejoin);
    }
    lose(&mut adapter);

    let line = "actuator id 3 restarted 3 times: not driven anymore";
    let seen = warnings(line, || ticks(&mut adapter, &claim, 10));

    assert_eq!(seen, 1);
    assert!(adapter.presence.refused.get(2));
    assert!(!adapter.present().get(2));
    assert!(!adapter.backend.torqued(0, id(2)));
    Ok(())
}

#[test]
fn an_unheld_slot_still_answering_gets_its_limits_back_at_close() -> Result<(), DynamixelError> {
    let mut adapter = strict_placed(voltage_limited(true));
    adapter.backend.missing(0, id(3));
    let claim = joined_rig(&mut adapter)?;
    let originals = adapter.saved[2].voltage_dv;
    adapter.backend.missing(0, id(2));
    reads(&mut adapter, LOST_AFTER);
    adapter.backend.plug(0, id(3), true);
    assert!(until_present(&mut adapter, &claim, 3, 20).is_some());
    assert!(!adapter.backend.holds(0, Model::Ax12a, id(2)));
    adapter.backend.plug(0, id(2), false);
    adapter.backend.clear_log();

    let seen = warnings("not restored", || {
        adapter.torque_off();
        adapter.close();
    });

    assert_eq!(seen, 0);
    assert!(calls_of(&adapter.backend, Call::WriteVoltageLimits).any(|named| named == id(2)));
    assert_eq!(
        adapter
            .backend
            .limits(0, id(2))
            .map(|limits| (limits.min_voltage_dv, limits.max_voltage_dv)),
        originals
    );
    Ok(())
}
