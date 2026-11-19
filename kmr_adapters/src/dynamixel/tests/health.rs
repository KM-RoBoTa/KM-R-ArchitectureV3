//! The health policy: a failed bring-up health check fails bring-up by
//! default, and under `Warn` is logged once while the actuator is set up and
//! driven like the others; a joiner gets the same policy. Bus errors and
//! limits that do not read back still fail. Every scenario runs on a strict
//! fake, which keeps what `dxl_rs_v2` holds.

use std::time::Instant;

use kmr_core::hal::{HealthPolicy, JoinPolicy};

use super::presence::{powered_with, strict, strict_placed, tolerate};
use super::*;
use crate::dynamixel::backend::Fault;
use crate::dynamixel::presence::SETTLE_FOR;

/// Message of the warning logged per actuator whose failed health check
/// `Warn` accepts.
const ACCEPTED: &str = "health check failed, driven anyway";

/// `Overrides` with the health policy `Warn`, the presence policies off.
fn warn() -> Overrides {
    let mut overrides = Overrides::default();
    overrides.health = HealthPolicy::Warn;
    overrides
}

/// Bring-up of `adapter` under `overrides`, as [`powered_with`]; the
/// result and the warnings holding [`ACCEPTED`].
fn brought_up<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    overrides: Overrides,
) -> (Result<Claim<N>, DynamixelError>, usize) {
    let mut claim = None;
    let seen = warnings(ACCEPTED, || claim = Some(powered_with(adapter, overrides)));
    (claim.expect("brought up"), seen)
}

/// The Voltage error of the AX-12A at index 0 answering [`BELOW_ITS_LIMITS`].
const LOW: DynamixelError = DynamixelError::Voltage {
    actuator: 0,
    bus: "ax",
    id: 1,
    measured_dv: 101,
    min_dv: 105,
    max_dv: 130,
};

#[test]
fn health_fail_is_the_default_and_still_fails_bring_up() {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.health(0, id(0), BELOW_ITS_LIMITS);

    let (claim, seen) = brought_up(&mut adapter, Overrides::default());

    assert_eq!(claim.err(), Some(LOW));
    assert_eq!(seen, 0);
    assert!(!adapter.backend.torqued(0, id(1)));
}

#[test]
fn health_warn_brings_up_a_voltage_faulted_actuator_and_logs_it_once() -> Result<(), DynamixelError>
{
    let mut adapter = strict(AX_TABLE);
    adapter.backend.health(0, id(0), BELOW_ITS_LIMITS);

    let (claim, seen) = brought_up(&mut adapter, warn());
    let claim = claim?;

    assert_eq!(seen, 1);
    assert_eq!(adapter.present(), Mask::ALL);
    assert!((1..N).all(|index| adapter.backend.torqued(0, id(index))));
    let ticked = warnings(ACCEPTED, || {
        for _ in 0..5 {
            read(&mut adapter, &mut Slot::filled(0.0));
            write(&mut adapter, &claim, &Desired::all_q());
        }
    });
    assert_eq!(ticked, 0);
    assert!(adapter.backend.count(Call::SyncWriteGoal) > 0);
    Ok(())
}

#[test]
fn health_warn_asks_torque_on_for_the_accepted_actuator_too() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.health(0, id(0), BELOW_ITS_LIMITS);
    adapter.check(warn())?;
    mark_open(&mut adapter);
    adapter.zero()?;
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;

    adapter.torque_on()?;

    assert!(torque_asked(&adapter.backend, 0, id(0), true));
    assert!((0..N).all(|index| torque_asked(&adapter.backend, 0, id(index), true)));
    Ok(())
}

#[test]
fn health_warn_brings_up_a_protocol_2_hardware_error() -> Result<(), DynamixelError> {
    let mut adapter = strict(X_TABLE);
    let latched = Health {
        error_status: 0x01,
        ..HEALTHY
    };
    adapter.backend.health(0, id(1), latched);

    let (claim, seen) = brought_up(&mut adapter, warn());

    claim?;
    assert_eq!(seen, 1);
    assert_eq!(adapter.present(), Mask::ALL);
    Ok(())
}

#[test]
fn health_warn_still_fails_on_a_bus_error() {
    let mut adapter = strict(AX_TABLE);
    adapter
        .backend
        .fail(Call::ReadHealth, 0, id(1), BackendError::Io(Wire::Closed));

    let (claim, seen) = brought_up(&mut adapter, warn());

    assert_eq!(
        claim.err(),
        Some(DynamixelError::Actuator {
            actuator: 1,
            bus: "ax",
            id: id(1),
            source: BackendError::Io(Wire::Closed),
        })
    );
    assert_eq!(seen, 0);
}

#[test]
fn health_warn_still_fails_on_a_limits_mismatch() {
    let mut adapter = strict_placed(set(MX2_TABLE, 1, VOLTAGES));
    adapter.backend.stuck_limits(0, id(1));

    let (claim, seen) = brought_up(&mut adapter, warn());

    assert!(
        matches!(
            claim,
            Err(DynamixelError::LimitsMismatch { actuator: 1, .. })
        ),
        "{claim:?}"
    );
    assert_eq!(seen, 0);
}

#[test]
fn health_warn_still_fails_on_limits_for_a_powered_actuator() {
    let mut adapter = strict_placed(set(MX2_TABLE, 0, VOLTAGES));
    adapter.backend.already_on(0, id(0));

    let (claim, seen) = brought_up(&mut adapter, warn());

    assert!(
        matches!(
            claim,
            Err(DynamixelError::LimitsTorqueOn { actuator: 0, .. })
        ),
        "{claim:?}"
    );
    assert_eq!(seen, 0);
}

// ── A joiner under the health policy ───────────────────────────────────────

/// Both presence policies on, the health policy `health`.
fn joining_with(health: HealthPolicy) -> Overrides {
    let mut overrides = tolerate();
    overrides.join = JoinPolicy::Declared;
    overrides.health = health;
    overrides
}

/// [`AX_TABLE`] brought up under [`joining_with`] `health` without id 3,
/// which then answers below its voltage limits: ticks until it is present,
/// refused or the Settle wait is long over. The adapter, the warnings
/// holding [`ACCEPTED`], and whether its torque on was asked.
fn faulted_joiner(health: HealthPolicy) -> Result<Joined, DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = powered_with(&mut adapter, joining_with(health))?;
    refresh(&mut adapter, &mut Telemetry::default());
    adapter.presence.probe_period = Duration::ZERO;
    adapter
        .backend
        .health(0, id(2), BELOW_ITS_LIMITS)
        .plug(0, id(2), true);

    let deadline = Instant::now() + SETTLE_FOR * 4;
    let mut asked = false;
    let seen = warnings(ACCEPTED, || {
        while Instant::now() < deadline
            && !adapter.present().get(2)
            && !adapter.presence.refused.get(2)
        {
            read(&mut adapter, &mut Slot::filled(0.0));
            write(&mut adapter, &claim, &Desired::all_q());
            refresh(&mut adapter, &mut Telemetry::default());
            adapter.upkeep();
            asked |= torque_asked(&adapter.backend, 0, id(2), true);
            adapter.backend.clear_log();
        }
    });
    Ok((adapter, seen, asked))
}

/// What [`faulted_joiner`] gives.
type Joined = (Dynamixel<Scripted, 1, N>, usize, bool);

#[test]
fn health_fail_refuses_a_faulted_joiner() -> Result<(), DynamixelError> {
    let (adapter, seen, asked) = faulted_joiner(HealthPolicy::Fail)?;

    assert!(adapter.presence.refused.get(2));
    assert!(!adapter.present().get(2));
    assert_eq!(seen, 0);
    assert!(!asked);
    Ok(())
}

#[test]
fn health_warn_joins_a_faulted_joiner_with_one_warning() -> Result<(), DynamixelError> {
    let (adapter, seen, asked) = faulted_joiner(HealthPolicy::Warn)?;

    assert!(!adapter.presence.refused.get(2));
    assert!(adapter.present().get(2));
    assert!(adapter.presence.configured.get(2));
    assert_eq!(seen, 1);
    assert!(asked);
    Ok(())
}

#[test]
fn health_warn_powers_the_rest_when_the_accepted_actuator_reads_its_fault()
-> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    let flag = BackendError::Fault(Fault::Voltage {
        measured_dv: 101,
        min_dv: 105,
        max_dv: 130,
    });
    adapter
        .backend
        .health(0, id(0), BELOW_ITS_LIMITS)
        .fail(Call::ReadPosition, 0, id(0), flag);

    let (claim, seen) = brought_up(&mut adapter, warn());

    claim?;
    assert_eq!(seen, 1);
    assert_eq!(adapter.present(), Mask::ALL);
    assert!(!adapter.backend.torqued(0, id(0)));
    assert!((1..N).all(|index| adapter.backend.torqued(0, id(index))));
    Ok(())
}

#[test]
fn health_warn_keeps_the_accepted_actuator_when_a_joiner_redeclares_its_bus()
-> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter
        .backend
        .health(0, id(0), BELOW_ITS_LIMITS)
        .missing(0, id(2));
    let claim = powered_with(&mut adapter, joining_with(HealthPolicy::Warn))?;
    refresh(&mut adapter, &mut Telemetry::default());
    adapter.presence.probe_period = Duration::ZERO;
    adapter.backend.plug(0, id(2), true);

    for _ in 0..12 {
        if adapter.present().get(2) {
            break;
        }
        read(&mut adapter, &mut Slot::filled(0.0));
        write(&mut adapter, &claim, &Desired::all_q());
        refresh(&mut adapter, &mut Telemetry::default());
        adapter.upkeep();
    }

    assert!(adapter.present().get(2), "joined");
    assert!(adapter.present().get(0));
    assert!(!adapter.presence.refused.get(0));
    Ok(())
}
