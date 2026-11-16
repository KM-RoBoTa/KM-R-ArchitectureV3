//! C and C++ controllers (feature `ffi`, SRS FR-10).
//!
//! A C or C++ function is a controller of its own, added like a Rust one:
//!
//! ```ignore
//! // build.rs, with kmr_api and its `ffi-build` feature in [build-dependencies]
//! fn main() -> Result<(), kmr_api::ffi::build::Error> {
//!     kmr_api::ffi::build::robot("XmBench").compile(&["c/swing.c"])
//! }
//!
//! // main.rs
//! kmr_api::robot!(XmBench);
//!
//! fn main() -> Result<(), kmr_api::UserError> {
//!     Robot::new()
//!         .add_controller(EachTick, kmr_api::c!(swing))
//!         .run()
//! }
//! ```
//!
//! ```c
//! /* c/swing.c */
//! #include "kmr.h"
//!
//! void swing(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
//!     float q;
//!     if (kmr_q_at(state, 0, &q) == KMR_OK) {
//!         kmr_set_q_at(state, q + 0.1f, 0);
//!     }
//!     if (kmr_elapsed(time) > 60.0) {
//!         kmr_shutdown(state, "time limit");
//!     }
//! }
//! ```
//!
//! # The C API
//!
//! `kmr.h` mirrors the controller API method for method, as `static inline` functions over a
//! snapshot kmr refills each tick: `kmr_<method>(handle, args.., &out)`. A Rust `Result` is a
//! `kmr_status` (`KMR_OK`, `KMR_OUT_OF_RANGE`, `KMR_HISTORY_OUT_OF_RANGE`, `KMR_NO_SAMPLE`, ..),
//! a Rust `Option` a `bool`, or `NULL` for a reading, and a `Result<Option<_>>` answers
//! `KMR_NONE` for no value yet. Values are bare `float`s in engine units, durations `double`
//! seconds. A group is its index array and length: `kmr_group_set_all_q(state, AX, 12, v)`.
//! `kmr_shutdown(state, reason)` and `kmr_emergency_stop(state, reason)` raise the signal after
//! the call. Writes are staged and applied after the call through the same setters as a Rust
//! write, so limits and envelopes hold. C never calls back into Rust.
//!
//! `kmr.hpp` wraps it for C++17: `kmr::Time`, `kmr::State`, `kmr::Sensors`, `kmr::Result` for a
//! Rust `Result`, `std::optional` for an `Option`, and `KMR_CONTROLLER(name, function)` to
//! define the C function from a C++ one. No exception crosses: one that leaves the body
//! discards the call's writes and shuts down.
//!
//! # Context
//!
//! `c!(name, with = ctx)` gives the robot a `T` it keeps for the whole run; the function gets
//! `void *ctx` pointing at it, `NULL` without one. Mirror `T` as a `#[repr(C)]` struct.
//!
//! # Safety
//!
//! A crash or abort in C or C++ leaves torque as it was: no software guard can prevent it.
//! Run the controller on the no-hardware adapter first (SRS FR-5.10).

use kmr_core::sensors::{MAX_IMUS, MAX_MICS, MIC_WINDOW};
use kmr_core::{ControlFnWith, Filled, InlineWith, Insert};
use kmr_ffi::{CDuration, CImu, CSlot, CTickFaults, CTime, Request};

use crate::controller::{IntoController, seal::Sealed};
use crate::{HISTORY_DEPTH, Q, Schedule, Sensors, State, StateError, Time};

mod script;

#[doc(hidden)]
pub use kmr_core::state::{FieldId as __FieldId, field_id as __field_id};
#[doc(hidden)]
pub use kmr_ffi::{for_each_field as __for_each_field, for_each_flag as __for_each_flag};
// DECISION: the Lua bridge's surface stays reachable, hidden, until the bridge is removed.
#[doc(hidden)]
pub use script::{
    AGE_NONE, ActuatorFlags, Foreign, ForeignError, ReadView, Slot, Status, TimeView, WriteBuf,
    Written, call,
};

/// The build-script helper: writes `kmr.h`, `kmr.hpp` and the ABI file into `OUT_DIR` and
/// compiles the C and C++ sources (feature `ffi-build`, in `[build-dependencies]`).
///
/// ```ignore
/// // build.rs
/// fn main() -> Result<(), kmr_api::ffi::build::Error> {
///     kmr_api::ffi::build::robot("XmBench").compile(&["c/swing.c", "cpp/arm.cpp"])
/// }
/// ```
#[cfg(feature = "ffi-build")]
pub mod build {
    use std::path::Path;

    pub use kmr_ffi::build::{Build, Error};

    /// The engine's sizes the headers are written for.
    fn sizes() -> kmr_ffi::build::Sizes {
        kmr_ffi::build::Sizes {
            history_depth: crate::HISTORY_DEPTH,
            max_imus: kmr_core::sensors::MAX_IMUS,
            max_mics: kmr_core::sensors::MAX_MICS,
            mic_window: kmr_core::sensors::MIC_WINDOW,
        }
    }

    /// A build for the preset `name`, the robot `robot!(name)` gives.
    pub fn robot(name: &str) -> Build {
        kmr_ffi::build::robot(name, sizes())
    }

    /// A build for the descriptor at `path`, the robot `robot!(model = path)` gives; a relative
    /// `path` is relative to the crate's `Cargo.toml`.
    pub fn model(path: impl AsRef<Path>) -> Build {
        kmr_ffi::build::model(path, sizes())
    }
}

/// A Lua function as the foreign body (feature `ffi-lua`): a vendored Lua 5.4, 1-based tables.
///
/// `Bridge::load(path, function)` in `main`, before `run()`; the bridge is the context of a
/// `with(bridge, forwarder)` controller that passes `&mut bridge` to `ffi::call`.
#[cfg(feature = "ffi-lua")]
pub mod lua {
    pub use kmr_ffi::lua::Bridge;
}

/// The clock as C reads it.
#[doc(hidden)]
pub type __Time = CTime;
/// The state as C reads it, at this engine's History depth.
#[doc(hidden)]
pub type __State<const N: usize> = kmr_ffi::CState<N, HISTORY_DEPTH>;
/// The sensors as C reads them, at this engine's slot counts.
#[doc(hidden)]
pub type __Sensors = kmr_ffi::CSensors<MAX_IMUS, MAX_MICS, MIC_WINDOW>;
/// A C or C++ controller of a robot of N actuators.
#[doc(hidden)]
pub type __CFn<const N: usize> = kmr_ffi::CFn<N, HISTORY_DEPTH, MAX_IMUS, MAX_MICS, MIC_WINDOW>;
/// History depth, IMU slots, microphone slots and window, as `kmr_ffi_abi.rs` lists them.
#[doc(hidden)]
pub const __ABI: [usize; 4] = [HISTORY_DEPTH, MAX_IMUS, MAX_MICS, MIC_WINDOW];

/// A C or C++ function compiled by the build script, as a controller:
/// `.add_controller(EachTick, kmr_api::c!(swing))`.
///
/// The function is `void swing(const kmr_time *, kmr_state *, const kmr_sensors *, void *ctx)`,
/// `ctx` null. `c!(swing, with = ctx)` gives the robot `ctx`, kept for the whole run, and the
/// function a pointer to it: mirror its type as a `#[repr(C)]` struct.
///
/// `N` from `robot!` must be in scope. The build fails with `error[E0080]` here when the
/// sources were built for another robot (SRS FR-10.2, FR-6.1). See [the module](crate::ffi).
// DECISION: one macro for C and C++, named after the ABI: a C++ controller is a C function
// (`KMR_CONTROLLER` defines it), and a C++ name cannot cross without `extern "C"`. The context is
// `with = ctx`, the word of `with(ctx, f)`, and the robot owns it as `with` does.
#[macro_export]
macro_rules! c {
    ($name:ident $(,)?) => {
        $crate::__c!($name, ::core::option::Option::<()>::None)
    };
    ($name:ident, with = $ctx:expr $(,)?) => {
        $crate::__c!($name, ::core::option::Option::Some($ctx))
    };
}

/// The expansion of [`c!`]: the ABI check, the declaration and the controller.
#[doc(hidden)]
#[macro_export]
macro_rules! __c {
    ($name:ident, $ctx:expr) => {{
        const FFI_ABI: [usize; 5] =
            ::core::include!(::core::concat!(::core::env!("OUT_DIR"), "/kmr_ffi_abi.rs"));
        const _: () = ::core::assert!(
            FFI_ABI[0] == N,
            "kmr: the C and C++ sources were built for another N; build.rs and robot! must name \
             the same robot"
        );
        const _: () = ::core::assert!(
            FFI_ABI[1] == $crate::ffi::__ABI[0]
                && FFI_ABI[2] == $crate::ffi::__ABI[1]
                && FFI_ABI[3] == $crate::ffi::__ABI[2]
                && FFI_ABI[4] == $crate::ffi::__ABI[3],
            "kmr: the C and C++ sources were built against another kmr; rebuild them"
        );
        // SAFETY: the build helper compiled the function against `kmr.h` of the same build,
        // which declares exactly this signature for the N and sizes the asserts above pin.
        // Calling it stays `unsafe`: kmr_ffi's `invoke` does it, over references of these types.
        unsafe extern "C" {
            fn $name(
                time: *const $crate::ffi::__Time,
                state: *mut $crate::ffi::__State<N>,
                sensors: *const $crate::ffi::__Sensors,
                ctx: *mut ::core::ffi::c_void,
            );
        }
        $crate::ffi::CController::<N, _>::__new(::core::stringify!($name), $name, $ctx)
    }};
}

/// A C or C++ controller with its optional context, what [`c!`] returns and
/// [`add_controller`](crate::Robot::add_controller) takes.
pub struct CController<const N: usize, T = ()> {
    /// The function's name, for messages.
    name: &'static str,
    /// The function.
    f: __CFn<N>,
    /// The context the function gets a pointer to; `None` passes null.
    ctx: Option<T>,
}

impl<const N: usize, T> CController<N, T> {
    /// What [`c!`] expands to.
    #[doc(hidden)]
    pub fn __new(name: &'static str, f: __CFn<N>, ctx: Option<T>) -> Self {
        Self { name, f, ctx }
    }
}

impl<const N: usize, T> core::fmt::Debug for CController<N, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CController")
            .field("name", &self.name)
            .field("ctx", &self.ctx.is_some())
            .finish_non_exhaustive()
    }
}

/// A C controller's state, owned by the robot: the snapshot it is called over, and its context.
// DECISION: the snapshot lives inline in the node, about 5 KB at N = 22: refilled in place, no heap,
// and the context next to it is `Option<T>`, `None` for a null `ctx`.
#[doc(hidden)]
pub struct __Frame<const N: usize, T> {
    /// `kmr_time`.
    time: __Time,
    /// `kmr_state`.
    state: __State<N>,
    /// `kmr_sensors`.
    sensors: __Sensors,
    /// What `void *ctx` points at.
    ctx: Option<T>,
}

/// The node a C controller is stored as: wraps the core handles, fills the snapshot, calls.
#[doc(hidden)]
pub struct __CTick<const N: usize> {
    /// The function's name, for messages.
    name: &'static str,
    /// The function.
    f: __CFn<N>,
}

impl<const N: usize, T> ControlFnWith<__Frame<N, T>, N> for __CTick<N> {
    fn call(
        &self,
        time: &kmr_core::Time,
        state: &mut kmr_core::RobotState<N>,
        sensors: &kmr_core::Sensors,
        frame: &mut __Frame<N, T>,
    ) {
        let mut state = State::new(state);
        fill_time(&mut frame.time, &Time::new(time));
        fill_state(&mut frame.state, &state);
        fill_sensors(&mut frame.sensors, &Sensors::new(sensors));
        kmr_ffi::invoke(
            self.f,
            &frame.time,
            &mut frame.state,
            &frame.sensors,
            frame.ctx.as_mut(),
        );
        // DECISION: the writes apply even with a request, as a Rust controller's writes before its
        // `shutdown!` do; the reason is printed once, as Python's `shutdown(reason)` does.
        __ffi_apply(&frame.state.write, &mut state);
        match frame.state.request() {
            Request::None => {}
            Request::Shutdown => {
                eprintln!(
                    "kmr: {} asked for a shutdown: {}",
                    self.name,
                    String::from_utf8_lossy(frame.state.reason())
                );
                kmr_core::shutdown!("c: kmr_shutdown()");
            }
            Request::EmergencyStop => {
                eprintln!(
                    "kmr: {} asked for an emergency stop: {}",
                    self.name,
                    String::from_utf8_lossy(frame.state.reason())
                );
                kmr_core::emergency_stop!("c: kmr_emergency_stop()");
            }
        }
    }
}

impl<const N: usize, T> Sealed<N> for CController<N, T> {}

impl<const N: usize, S: Schedule, C, T> IntoController<N, S, C> for CController<N, T>
where
    C: Insert<S, InlineWith<__CTick<N>, __Frame<N, T>>>,
{
    type Output = C::Output;

    fn add_to<E>(
        self,
        schedule: S,
        robot: kmr_core::Robot<N, C, E>,
    ) -> kmr_core::Robot<N, C::Output, Filled> {
        let frame = __Frame {
            time: __Time::default(),
            state: __State::default(),
            sensors: __Sensors::default(),
            ctx: self.ctx,
        };
        let tick = __CTick {
            name: self.name,
            f: self.f,
        };
        robot.add_control_fn_with(schedule, frame, tick)
    }
}

/// Sets `slots[i]`; an index past the end is ignored.
fn put<T>(slots: &mut [T], i: usize, value: T) {
    if let Some(slot) = slots.get_mut(i) {
        *slot = value;
    }
}

/// A mask as its one word.
fn word<const N: usize>(mask: crate::Mask<N>) -> u64 {
    let [word] = mask.words();
    word
}

/// Copies the clock.
fn fill_time(out: &mut CTime, time: &Time<'_>) {
    out.tick = time.tick();
    out.overrun_count = time.overrun_count();
    out.elapsed = time.elapsed().into();
    out.dt = time.dt().into();
    out.accumulated_overrun = time.accumulated_overrun().into();
    let last = time.last_overrun();
    out.late = u32::from(last.is_some());
    out.last_overrun = last.map_or(CDuration::default(), CDuration::from);
}

/// Generates the copy of one History slot from the field list.
macro_rules! fill_slot {
    ($($field:ident: $ty:ident),*) => {
        /// Copies the History slot `depth` back into `slot`; `false` when there is none yet.
        // DECISION: a sample is per actuator, not per field, so `Q` alone says which were sampled.
        fn fill_slot<const N: usize>(state: &State<'_, N>, depth: usize, slot: &mut CSlot<N>) -> bool {
            if let Err(StateError::HistoryValueOutOfRange { .. }) = state.field_at::<Q>(0, depth) {
                return false;
            }
            slot.sampled = (0..N)
                .filter(|&i| state.field_at::<Q>(i, depth).is_ok())
                .fold(0, |mask, i| mask | 1 << i);
            $(
                for (i, v) in slot.$field.iter_mut().enumerate() {
                    *v = state.field_at::<crate::$ty>(i, depth).map_or(0.0, |x| x.0);
                }
            )*
            true
        }
    };
}
kmr_ffi::for_each_field!(fill_slot);

/// Generates the copy of one tick's faults from the flag list.
macro_rules! tick_faults {
    ($($flag:ident),*) => {
        /// `faults` as C reads them.
        fn tick_faults<const N: usize>(faults: &crate::Faults<N>) -> CTickFaults {
            let flags = [
                (faults.overrun(), kmr_ffi::TICK_OVERRUN),
                (faults.sensor_missing(), kmr_ffi::TICK_SENSOR_MISSING),
                (faults.remote_timeout(), kmr_ffi::TICK_REMOTE_TIMEOUT),
                (faults.unauthorized(), kmr_ffi::TICK_UNAUTHORIZED),
            ];
            CTickFaults {
                $($flag: word(faults.$flag()),)*
                tick: flags.iter().filter(|(on, _)| *on).fold(0, |bits, (_, bit)| bits | bit),
                reserved: 0,
            }
        }
    };
}
kmr_ffi::for_each_flag!(tick_faults);

/// Takes this tick's snapshot of `state`: History, faults, ages, cadence and telemetry.
fn fill_state<const N: usize>(out: &mut __State<N>, state: &State<'_, N>) {
    out.filled = 0;
    for (depth, slot) in out.slot.iter_mut().enumerate() {
        if !fill_slot(state, depth, slot) {
            break;
        }
        out.filled += 1;
    }
    out.faults_filled = 0;
    for (depth, slot) in out.slot.iter_mut().enumerate() {
        let Ok(faults) = state.prev_faults(depth) else {
            break;
        };
        slot.faults = tick_faults(&faults);
        out.faults_filled += 1;
    }
    out.has_age = 0;
    out.has_temperature = 0;
    out.has_voltage = 0;
    out.has_hardware_error = 0;
    out.has_telemetry_age = 0;
    for i in 0..N {
        let bit = 1u64 << i;
        if let Ok(Some(age)) = state.age_at(i) {
            put(&mut out.age, i, age);
            out.has_age |= bit;
        }
        if let Ok(Some(c)) = state.temperature_at(i) {
            put(&mut out.temperature, i, c.0);
            out.has_temperature |= bit;
        }
        if let Ok(Some(v)) = state.voltage_at(i) {
            put(&mut out.voltage, i, v.0);
            out.has_voltage |= bit;
        }
        if let Ok(Some(e)) = state.hardware_error_at(i) {
            put(&mut out.hardware_error, i, e);
            out.has_hardware_error |= bit;
        }
        if let Ok(Some(age)) = state.telemetry_age_at(i) {
            put(&mut out.telemetry_age, i, age);
            out.has_telemetry_age |= bit;
        }
        put(
            &mut out.read_every,
            i,
            state.read_every(i).map_or(1, |e| e.get()),
        );
    }
    let sample_age = state.sample_age();
    out.has_sample_age = u32::from(sample_age.is_some());
    out.sample_age = sample_age.unwrap_or(0);
}

/// Copies every sensor slot.
fn fill_sensors(out: &mut __Sensors, sensors: &Sensors<'_>) {
    for (i, slot) in out.imu.iter_mut().enumerate() {
        *slot = sensors.imu(i).map_or_else(CImu::default, |imu| {
            let accel = imu.accel();
            let gyro = imu.gyro();
            CImu {
                age: imu.age(),
                dropped: imu.dropped(),
                accel: accel.map_or_else(Default::default, |a| a.0.into()),
                gyro: gyro.map_or_else(Default::default, |g| g.0.into()),
                present: 1,
                has_accel: u32::from(accel.is_some()),
                has_gyro: u32::from(gyro.is_some()),
                reserved: 0,
            }
        });
    }
    for (i, slot) in out.mic.iter_mut().enumerate() {
        let Some(mic) = sensors.mic(i) else {
            slot.present = 0;
            slot.len = 0;
            continue;
        };
        let mut len = 0u32;
        for (to, from) in slot.samples.iter_mut().zip(mic.samples()) {
            *to = *from;
            len += 1;
        }
        slot.present = 1;
        slot.len = len;
        slot.rate_hz = mic.rate_hz();
        slot.dropped = mic.dropped();
    }
}

/// Generates the apply step from a field list: an exhaustive `match` on the core's `FieldId`.
///
/// Exported only for kmr_api's compile-fail suite: a list missing a field fails with `E0004`
/// naming its variant (kmr_ffi T028).
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

#[cfg(test)]
mod tests;
