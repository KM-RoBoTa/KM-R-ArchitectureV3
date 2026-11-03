//! Controller math in C, C++, Python or Lua (feature `ffi`, SRS FR-10).
//!
//! A Rust controller stays the one registered on a phase; it hands one tick's math to a
//! foreign function with [`call`]: the state and the clock are copied into a [`ReadView`] and a
//! [`TimeView`], the function fills a [`WriteBuf`], and only the entries it flagged are
//! applied, through the same setters as a Rust controller's writes.
//!
//! ```ignore
//! kmr_api::robot!(XmBench);
//! kmr_api::ffi::foreign! { fn swing; }
//!
//! fn forward(time: &Time, state: &mut State, _: &Sensors) {
//!     if kmr_api::ffi::call(swing, time, state, None::<&mut ()>).is_err() {
//!         kmr_api::emergency_stop!("swing failed");
//!     }
//! }
//! ```
//!
//! A crash or abort in foreign code leaves torque as it was: no software guard can prevent it.
//! Run the controller on the no-hardware adapter first (SRS FR-5.10).

use std::time::Duration;

use crate::{HISTORY_DEPTH, State, Time};

pub use kmr_ffi::{
    AGE_NONE, ActuatorFlags, CFn, Foreign, ForeignError, ReadView, Slot, Status, TimeView,
    WriteBuf, Written,
};

#[doc(hidden)]
pub use kmr_core::state::{FieldId as __FieldId, field_id as __field_id};
#[doc(hidden)]
pub use kmr_ffi::{for_each_field as __for_each_field, foreign as __foreign};

/// The build-script helper: writes `kmr_ffi.h`, `kmr_ffi.hpp` and the ABI file into `OUT_DIR`
/// and compiles the C and C++ sources (feature `ffi-build`, in `[build-dependencies]`).
///
/// ```ignore
/// // build.rs
/// fn main() -> Result<(), kmr_api::ffi::build::Error> {
///     kmr_api::ffi::build::robot("XmBench").compile(&["src/swing.c"])
/// }
/// ```
#[cfg(feature = "ffi-build")]
pub mod build {
    use std::path::Path;

    pub use kmr_ffi::build::{Build, Error};

    /// A build for the preset `name`, the robot `robot!(name)` gives.
    pub fn robot(name: &str) -> Build {
        kmr_ffi::build::robot(name, crate::HISTORY_DEPTH)
    }

    /// A build for the descriptor at `path`, the robot `robot!(model = path)` gives; a relative
    /// `path` is relative to the crate's `Cargo.toml`.
    pub fn model(path: impl AsRef<Path>) -> Build {
        kmr_ffi::build::model(path, crate::HISTORY_DEPTH)
    }
}

/// Declares the C and C++ controller bodies the build script compiled, once per crate, after
/// `robot!`: `kmr_api::ffi::foreign! { fn swing; fn other; }`.
///
/// Each name becomes a [`CFn`] that [`call`] takes as is. The build fails with `error[E0080]`
/// at this line when the sources were built for another robot (SRS FR-10.2, FR-6.1).
#[macro_export]
macro_rules! __ffi_foreign {
    ($(fn $name:ident;)+) => {
        $crate::ffi::__foreign! { depth = $crate::HISTORY_DEPTH; $(fn $name;)+ }
    };
}
#[doc(inline)]
pub use crate::__ffi_foreign as foreign;

/// Generates the apply step from a field list: an exhaustive `match` on the core's `FieldId`.
///
/// Plumbing of [`call`], exported only for kmr_api's compile-fail suite: a list missing a field
/// fails with `E0004` naming its variant (kmr_ffi T028).
#[doc(hidden)]
#[macro_export]
macro_rules! __ffi_apply {
    ($($field:ident: $ty:ident),* $(,)?) => {
        /// Applies every flagged entry of `buf` through the field's setter.
        fn __ffi_apply<const N: usize>(buf: &$crate::ffi::WriteBuf<N>, state: &mut $crate::State<'_, N>) {
            $(
                __ffi_apply_field(
                    $crate::ffi::__field_id::<$crate::$ty>(),
                    &buf.$field,
                    buf.written.$field,
                    state,
                );
            )*
        }

        /// Applies the entries of one field whose bit is set in `written`.
        fn __ffi_apply_field<const N: usize>(
            id: $crate::ffi::__FieldId,
            values: &[f32; N],
            written: u64,
            state: &mut $crate::State<'_, N>,
        ) {
            match id {
                $(
                    $crate::ffi::__FieldId::$ty => {
                        $crate::ffi::__apply_bits(values, written, |v, i| {
                            state.__ffi_set($crate::$ty(v), i)
                        })
                    }
                )*
            }
        }
    };
}

/// Calls `set(values[i], i)` for every bit `i` of `written` below N.
#[doc(hidden)]
pub fn __apply_bits<const N: usize>(
    values: &[f32; N],
    written: u64,
    mut set: impl FnMut(f32, usize),
) {
    let mut bits = written;
    while bits != 0 {
        let i = bits.trailing_zeros() as usize;
        bits &= bits - 1;
        if let Some(&v) = values.get(i) {
            set(v, i);
        }
    }
}

kmr_ffi::for_each_field!(__ffi_apply);

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
                $($flag: word(faults.$flag()),)*
            }
        }
    };
}
kmr_ffi::for_each_flag!(fill_flags);

/// A mask as the view's one word.
fn word<const N: usize>(mask: crate::Mask<N>) -> u64 {
    let [word] = mask.words();
    word
}

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

/// Runs the foreign body `f` once for this tick, from any controller with a [`State`].
///
/// Fills the view from `state` (every History depth, faults, ages) and `time`, calls `f` (a
/// [`CFn`] from [`foreign!`], or a script bridge), and applies each entry it flagged through
/// the field's setter, so the write mask, limits and envelopes apply as for a Rust write.
/// `ctx` reaches a C body as `void *ctx`, null for `None`. On [`Status::Shutdown`] or
/// [`Status::EmergencyStop`] the signal is raised before `call` returns. No heap, no lock on
/// the C path (SRS NFR-1).
///
/// # Errors
///
/// What `f` returned: a negative or unknown code, a script error. Nothing is applied and no
/// signal is raised; the controller decides (SRS FR-2.5).
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
