//! A limit write the actuator did not apply: the fault it still reports
//! carries the limits it holds, and `health` names them against the ones
//! written in one warning, its error unchanged.

use super::*;

/// Message start of the warning for voltage limits not applied.
const VOLTAGE_UNAPPLIED: &str = "voltage limits not applied";

/// Message start of the warning for a Temperature Limit not applied.
const TEMPERATURE_UNAPPLIED: &str = "temperature limit not applied";

/// Settings writing Min and Max Voltage Limit `min_dv`–`max_dv`.
const fn voltages(min_dv: u16, max_dv: u16) -> Settings {
    Settings {
        voltage_limits_dv: Some((min_dv, max_dv)),
        ..Settings::NONE
    }
}

/// `health` of `adapter`; its result and the warnings holding `needle`.
fn checked<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    needle: &'static str,
) -> (Result<(), DynamixelError>, usize) {
    let mut result = Ok(());
    let seen = warnings(needle, || result = adapter.health());
    (result, seen)
}

/// The Voltage error of the AX-12A at index 0 at 10.1 V against `min_dv`–
/// `max_dv`.
const fn low(min_dv: u16, max_dv: u16) -> DynamixelError {
    DynamixelError::Voltage {
        actuator: 0,
        bus: "ax",
        id: 1,
        measured_dv: 101,
        min_dv,
        max_dv,
    }
}

#[test]
fn voltage_limits_the_actuator_did_not_store_are_named_once_its_error_unchanged()
-> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(AX_TABLE, 0, voltages(80, 130));
    ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?;
    adapter.backend.stuck_limits(0, id(0));

    let (result, seen) = checked(&mut adapter, VOLTAGE_UNAPPLIED);

    assert_eq!(result, Err(low(105, 130)));
    assert_eq!(seen, 1);
    Ok(())
}

#[test]
fn the_unapplied_warning_names_what_was_written_and_what_is_held() -> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(AX_TABLE, 0, voltages(80, 130));
    ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?;
    adapter.backend.stuck_limits(0, id(0));

    let (_, seen) = checked(
        &mut adapter,
        "voltage limits not applied: wrote 8.0–13.0 V, the actuator still holds 10.5–13.0 V",
    );

    assert_eq!(seen, 1);
    Ok(())
}

#[test]
fn voltage_limits_applied_but_still_excluding_the_supply_warn_nothing() -> Result<(), DynamixelError>
{
    let mut adapter = voltage_rig(AX_TABLE, 0, voltages(110, 130));
    ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?;

    let (result, seen) = checked(&mut adapter, VOLTAGE_UNAPPLIED);

    assert_eq!(result, Err(low(110, 130)));
    assert_eq!(seen, 0);
    Ok(())
}

#[test]
fn no_limit_written_warns_nothing() -> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(AX_TABLE, 0, Settings::NONE);
    ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?;

    let (result, seen) = checked(&mut adapter, "not applied");

    assert_eq!(result, Err(low(105, 130)));
    assert_eq!(seen, 0);
    Ok(())
}

#[test]
fn a_temperature_limit_the_actuator_did_not_store_is_named_once() -> Result<(), DynamixelError> {
    let raised = Settings {
        temperature_limit_c: Some(70),
        ..Settings::NONE
    };
    let mut adapter = voltage_rig(RX_TABLE, 0, raised);
    let hot = Health {
        temperature_c: 65,
        temperature_limit_c: 60,
        ..HEALTHY
    };
    ping_answering(&mut adapter, 0, hot)?;
    adapter.backend.stuck_limits(0, id(0));

    let mut result = Ok(());
    let seen = warnings(
        "temperature limit not applied: wrote 70 °C, the actuator still holds 60 °C",
        || result = adapter.health(),
    );

    assert_eq!(
        result,
        Err(DynamixelError::Temperature {
            actuator: 0,
            bus: "rx",
            id: id(0),
            measured_c: 65,
            limit_c: 60,
        })
    );
    assert_eq!(seen, 1);
    Ok(())
}

#[test]
fn a_temperature_limit_applied_warns_nothing() -> Result<(), DynamixelError> {
    let raised = Settings {
        temperature_limit_c: Some(70),
        ..Settings::NONE
    };
    let mut adapter = voltage_rig(RX_TABLE, 0, raised);
    let hot = Health {
        temperature_c: 65,
        temperature_limit_c: 60,
        ..HEALTHY
    };
    ping_answering(&mut adapter, 0, hot)?;

    let (result, seen) = checked(&mut adapter, TEMPERATURE_UNAPPLIED);

    assert_eq!(result, Ok(()));
    assert_eq!(seen, 0);
    Ok(())
}

/// A joiner whose voltage limit write is not stored: its join is refused
/// with the same error as before, after one warning naming the limits.
#[cfg(feature = "experimental")]
#[test]
fn a_joiner_whose_voltage_limits_were_not_stored_is_named_once() -> Result<(), DynamixelError> {
    use std::time::Instant;

    use kmr_core::hal::{JoinPolicy, MissingPolicy};

    use super::presence::{powered_with, strict_placed};
    use crate::dynamixel::presence::SETTLE_FOR;

    let mut placement = AX_TABLE.in_order();
    placement.settings = [voltages(80, 130); N];
    let mut adapter = strict_placed(placement);
    adapter.backend.missing(0, id(2));
    let mut overrides = Overrides::default();
    overrides.missing = MissingPolicy::Tolerate;
    overrides.join = JoinPolicy::Declared;
    let claim = powered_with(&mut adapter, overrides)?;
    refresh(&mut adapter, &mut Telemetry::default());
    adapter.presence.probe_period = Duration::ZERO;
    adapter
        .backend
        .health(0, id(2), BELOW_ITS_LIMITS)
        .plug(0, id(2), true)
        .stuck_limits(0, id(2));

    let deadline = Instant::now() + SETTLE_FOR * 4;
    let seen = warnings(
        "voltage limits not applied: wrote 8.0–13.0 V, the actuator still holds 10.5–13.0 V",
        || {
            while Instant::now() < deadline && !adapter.presence.refused.get(2) {
                read(&mut adapter, &mut Slot::filled(0.0));
                write(&mut adapter, &claim, &Desired::all_q());
                refresh(&mut adapter, &mut Telemetry::default());
                adapter.upkeep();
            }
        },
    );

    assert!(adapter.presence.refused.get(2));
    assert_eq!(seen, 1);
    Ok(())
}
