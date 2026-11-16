//! The Lua bridge's glue: [`call`] fills the read view from [`State`] and [`Time`], runs the
//! bridge, and applies the flagged writes through the setters.
// DECISION: kept apart from the C path and hidden: it serves only the Lua bridge, removed later
// with it.

use std::time::Duration;

use super::__ffi_apply;
use crate::{HISTORY_DEPTH, State, Time};
pub use kmr_ffi::{
    AGE_NONE, ActuatorFlags, Foreign, ForeignError, ReadView, Slot, Status, TimeView, WriteBuf,
    Written,
};

/// Generates the fill of one History slot's values from the field list.
macro_rules! fill_values {
    ($($field:ident: $ty:ident),*) => {
        /// Copies every field `depth` back into `slot`; `false` when an actuator has no sample
        /// there.
        fn fill_values<const N: usize>(state: &State<'_, N>, depth: usize, slot: &mut Slot<N>) -> bool {
            $(
                let Ok(values) = state.prev_field::<crate::$ty>(depth) else {
                    return false;
                };
                slot.$field = values.map(|v| v.0);
            )*
            true
        }
    };
}
kmr_ffi::for_each_field!(fill_values);

/// Generates the copy of one slot's per-actuator faults from the flag list.
macro_rules! fill_flags {
    ($($flag:ident),*) => {
        /// The per-actuator faults of `faults` as the view's masks.
        fn flags<const N: usize>(faults: &crate::Faults<N>) -> ActuatorFlags {
            ActuatorFlags {
                $($flag: super::word(faults.$flag()),)*
            }
        }
    };
}
kmr_ffi::for_each_flag!(fill_flags);

/// The view of `state`: every History depth with its faults, and each actuator's age.
pub(crate) fn read_view<const N: usize>(state: &State<'_, N>) -> ReadView<N, HISTORY_DEPTH> {
    let mut view = ReadView::default();
    for (i, age) in view.age.iter_mut().enumerate() {
        *age = state.age_at(i).ok().flatten().unwrap_or(AGE_NONE);
    }
    let mut sampled = true;
    for (depth, slot) in view.slot.iter_mut().enumerate() {
        if let Ok(faults) = state.prev_faults(depth) {
            slot.flags = flags(&faults);
            slot.tick_flags = faults.tick_bits();
        }
        sampled = sampled && fill_values(state, depth, slot);
        if sampled {
            view.samples += 1;
        }
    }
    view
}

/// A duration in nanoseconds, saturated at `u64::MAX` (584 years).
fn nanos(d: Duration) -> u64 {
    u64::try_from(d.as_nanos()).unwrap_or(u64::MAX)
}

/// The view of the clock.
pub(crate) fn time_view(time: &Time<'_>) -> TimeView {
    TimeView {
        tick: time.tick(),
        dt_ns: nanos(time.dt()),
        elapsed_ns: nanos(time.elapsed()),
        overrun_count: time.overrun_count(),
        last_overrun_ns: time.last_overrun().map_or(0, nanos),
        accumulated_overrun_ns: nanos(time.accumulated_overrun()),
    }
}

/// Runs the script bridge `f` once for this tick, from a Rust controller with a [`State`].
///
/// Fills the view from `state` (every History depth, faults, ages) and `time`, calls `f`, and
/// applies each entry it flagged through the field's setter. On [`Status::Shutdown`] or
/// [`Status::EmergencyStop`] the signal is raised before `call` returns.
///
/// # Errors
///
/// What `f` returned: a wrong code, a script error. Nothing is applied and no signal is raised;
/// the controller decides (SRS FR-2.5).
pub fn call<const N: usize, F, C>(
    f: F,
    time: &Time<'_>,
    state: &mut State<'_, N>,
    ctx: Option<&mut C>,
) -> Result<Status, ForeignError>
where
    F: Foreign<N, HISTORY_DEPTH>,
{
    let read = read_view(state);
    let clock = time_view(time);
    let mut buf = WriteBuf::default();
    let status = kmr_ffi::call(f, &read, &mut buf, &clock, ctx)?;
    __ffi_apply(&buf, state);
    match status {
        Status::Continue => {}
        Status::Shutdown => kmr_core::shutdown!("ffi: the foreign controller returned 1"),
        Status::EmergencyStop => {
            kmr_core::emergency_stop!("ffi: the foreign controller returned 2");
        }
    }
    Ok(status)
}

#[cfg(test)]
mod tests;
