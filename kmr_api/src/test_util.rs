//! Scripted runs for tests (feature `test-util`): the core's scripted host and recording
//! adapter, reached through kmr_api, so a crate that depends on kmr_api only can drive ticks,
//! reads, faults and overruns without a real clock, a real sleep or the global signal bus.

use kmr_core::test_util::is_emergency_stop;

use crate::{Filled, Robot, Spec, UserError};

/// What [`run_scripted`] runs on: an `HCons` list of adapters.
pub use kmr_core::hal::Drivers;
/// The adapter-side fault bits a [`Recorder`] reports on `read` and adds on `write`.
pub use kmr_core::hal::Faults;
/// The tick flags raised from outside the loop: what [`RealTime`] takes when told to.
pub use kmr_core::hal::{EXTERNAL_FLAGS, ExternalFlags};
pub use kmr_core::test_util::{
    AFTER_EACH_TICK, AFTER_FIRST, AFTER_INIT, AFTER_LAST, AFTER_PRE_INIT, BEFORE_TORQUE_ON,
    CAPACITY, Call, DT, Failed, Host, Log, PER_TICK, RealTime, Recorder, SEED, Scripted, Sent,
    TICK_END, TICK_START, Ticking,
};

/// How a scripted run ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    /// A graceful shutdown: the tick that saw it completed.
    Shutdown,
    /// An emergency stop: the run left at the next bus check.
    EmergencyStop,
}

/// Finalizes `robot` as `run()` does, then runs it on `drivers` with `host` as its clock, sleep
/// and signal bus.
///
/// # Errors
///
/// The finalization and bring-up errors of `run()`.
pub fn run_scripted<const N: usize, H, C, P, D>(
    robot: Robot<N, H, C, Filled>,
    host: &mut P,
    drivers: &mut D,
) -> Result<Stop, UserError>
where
    H: Spec<N>,
    C: kmr_core::Drive<N>,
    P: Host,
    D: Drivers<N, Error: Send + Sync>,
{
    let signal = robot.run_scripted(host, drivers)?;
    Ok(if is_emergency_stop(&signal) {
        Stop::EmergencyStop
    } else {
        Stop::Shutdown
    })
}
