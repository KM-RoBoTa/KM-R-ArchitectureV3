//! The C controller ABI: the snapshot a C or C++ controller gets each tick, and the one call.
//!
//! kmr_api fills a [`CTime`], a [`CState`] and a [`CSensors`] in place before each call, the
//! controller reads and stages through the `static inline` functions of the generated `kmr.h`,
//! and kmr_api applies the staged writes and the signal request after it returns. C never calls
//! back into Rust. Every type is `#[repr(C)]`, `Copy`, without pointers or lifetimes; the header
//! declares the same layout for one N, History depth and sensor slot counts (SRS FR-10.2).

use core::ffi::c_void;
use core::ptr;
use core::time::Duration;

use crate::view::{Mask, MaskFits, WriteBuf};

/// Bytes of the reason `kmr_shutdown` and `kmr_emergency_stop` keep, the closing NUL included.
// DECISION: a fixed buffer the header copies the reason into, not a pointer: nothing points back
// into C after the call, and 127 bytes hold any reason worth printing.
pub const REASON_LEN: usize = 128;

// DECISION: the tick flag bits are kmr_ffi's own, filled from kmr_api's `Faults` getters, so the
// C ABI does not depend on the core's `TickFlags` layout.
/// Bit of [`CTickFaults::tick`]: the tick finished after its deadline.
pub const TICK_OVERRUN: u32 = 1;
/// Bit of [`CTickFaults::tick`]: a sensor is missing this tick.
pub const TICK_SENSOR_MISSING: u32 = 1 << 1;
/// Bit of [`CTickFaults::tick`]: a remote source fell silent past its timeout.
pub const TICK_REMOTE_TIMEOUT: u32 = 1 << 2;
/// Bit of [`CTickFaults::tick`]: a message from an unknown identity was dropped.
pub const TICK_UNAUTHORIZED: u32 = 1 << 3;

/// The per-tick flags as `(name, bit)`: the header's `kmr_faults_<name>` functions.
#[cfg(any(feature = "build", test))]
pub(crate) const TICK_FLAGS: &[(&str, u32)] = &[
    ("overrun", TICK_OVERRUN),
    ("sensor_missing", TICK_SENSOR_MISSING),
    ("remote_timeout", TICK_REMOTE_TIMEOUT),
    ("unauthorized", TICK_UNAUTHORIZED),
];

/// A duration as C reads it (`kmr_duration`): whole seconds and nanoseconds, so the header's
/// `secs + nanos / 1e9` is bit for bit `Duration::as_secs_f64`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CDuration {
    /// Whole seconds.
    pub secs: u64,
    /// Nanoseconds past them, below 10⁹.
    pub nanos: u32,
    /// Zero.
    pub reserved: u32,
}

impl From<Duration> for CDuration {
    fn from(d: Duration) -> Self {
        Self {
            secs: d.as_secs(),
            nanos: d.subsec_nanos(),
            reserved: 0,
        }
    }
}

/// The clock of the tick in progress (`kmr_time`), what kmr_api's `Time` getters return.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CTime {
    /// Index of the tick in progress, 0 first.
    pub tick: u64,
    /// Late ticks so far.
    pub overrun_count: u64,
    /// Since the grid was armed, stamped at tick start.
    pub elapsed: CDuration,
    /// The tick period.
    pub dt: CDuration,
    /// The previous tick's lateness; meaningful when `late` is 1.
    pub last_overrun: CDuration,
    /// Sum of lateness.
    pub accumulated_overrun: CDuration,
    /// 1: the previous tick finished late (Rust's `Some`), 0: on time.
    pub late: u32,
    /// Zero.
    pub reserved: u32,
}

/// Generates [`CTickFaults`] from the flag list.
macro_rules! define_tick_faults {
    ($($flag:ident),* $(,)?) => {
        /// The fault bits and flags of one tick (`kmr_tick_faults`): one mask per per-actuator
        /// flag, bit i = actuator i, and the per-tick flags as `TICK_*` bits.
        #[repr(C)]
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        pub struct CTickFaults {
            $(
                #[doc = concat!("Bit i set: `", stringify!($flag), "` on actuator i.")]
                pub $flag: Mask,
            )*
            /// `TICK_OVERRUN`, `TICK_SENSOR_MISSING`, `TICK_REMOTE_TIMEOUT`, `TICK_UNAUTHORIZED`.
            pub tick: u32,
            /// Zero.
            pub reserved: u32,
        }
    };
}

/// Generates [`CSlot`] from the field list.
macro_rules! define_slot {
    ($($field:ident: $ty:ident),* $(,)?) => {
        /// One History slot as C reads it (`kmr_slot`): every field of every actuator, which of
        /// them were sampled, and the tick's faults. `Default` checks N, as
        /// [`Mask`](crate::Mask) says.
        #[repr(C)]
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub struct CSlot<const N: usize> {
            $(
                #[doc = concat!("`", stringify!($ty), "` of each actuator; 0 where not sampled.")]
                pub $field: [f32; N],
            )*
            /// Bit i set: actuator i has a sample at this depth.
            pub sampled: $crate::view::Mask,
            /// The faults of this depth's tick.
            pub faults: $crate::c::CTickFaults,
        }

        impl<const N: usize> Default for CSlot<N> {
            fn default() -> Self {
                let () = $crate::view::MaskFits::<N>::CHECK;
                Self {
                    $($field: [0.0; N],)*
                    sampled: 0,
                    faults: $crate::c::CTickFaults::default(),
                }
            }
        }
    };
}

crate::for_each_flag!(define_tick_faults);
crate::for_each_field!(define_slot);
#[cfg(test)]
pub(crate) use define_slot;

/// How a call asked the loop to stop, from [`CState::signal`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
    /// The call asked nothing.
    None,
    /// `kmr_shutdown`: kmr_api raises `shutdown!` after the call.
    Shutdown,
    /// `kmr_emergency_stop`: kmr_api raises `emergency_stop!` after the call.
    EmergencyStop,
}

/// The robot state as C reads and writes it (`kmr_state`): a snapshot of History, faults, ages
/// and telemetry, the writes of one call, and its stop request.
///
/// kmr_api refills it in place before each call; only the entries flagged in `write.written`
/// are applied after, through the same setters as a Rust write. N is in `1..=64`, checked by
/// `Default`, as [`Mask`] says:
///
/// ```
/// let state = kmr_ffi::CState::<64, 4>::default();
/// assert_eq!(state.filled, 0);
/// ```
///
/// ```compile_fail
/// let _ = kmr_ffi::CState::<65, 4>::default();
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CState<const N: usize, const D: usize> {
    /// The History, newest first: `slot[k]` is `k` ticks back.
    pub slot: [CSlot<N>; D],
    /// The values this call staged and which of them count.
    pub write: WriteBuf<N>,
    /// Ticks since each actuator's telemetry was refreshed.
    pub telemetry_age: [u64; N],
    /// Bit i set: actuator i was read, `age[i]` holds.
    pub has_age: Mask,
    /// Bit i set: `temperature[i]` holds.
    pub has_temperature: Mask,
    /// Bit i set: `voltage[i]` holds.
    pub has_voltage: Mask,
    /// Bit i set: `hardware_error[i]` holds.
    pub has_hardware_error: Mask,
    /// Bit i set: `telemetry_age[i]` holds.
    pub has_telemetry_age: Mask,
    /// Temperature of each actuator, °C.
    pub temperature: [f32; N],
    /// Input voltage of each actuator, V.
    pub voltage: [f32; N],
    /// Ticks since each actuator was read.
    pub age: [u32; N],
    /// How often each actuator is read, in ticks.
    pub read_every: [u32; N],
    /// History slots recorded, `0..=D`.
    pub filled: u32,
    /// Fault slots recorded, `0..=D`.
    pub faults_filled: u32,
    /// Age of the oldest sample; meaningful when `has_sample_age` is 1.
    pub sample_age: u32,
    /// 1: every actuator was read once, `sample_age` holds.
    pub has_sample_age: u32,
    /// 0 none, 1 shutdown, 2 emergency stop; the highest request of the call wins.
    pub signal: u32,
    /// Raw hardware error status of each actuator.
    pub hardware_error: [u8; N],
    /// The request's reason, NUL-terminated.
    pub reason: [u8; REASON_LEN],
}

impl<const N: usize, const D: usize> Default for CState<N, D> {
    fn default() -> Self {
        let () = MaskFits::<N>::CHECK;
        Self {
            slot: [CSlot::default(); D],
            write: WriteBuf::default(),
            telemetry_age: [0; N],
            has_age: 0,
            has_temperature: 0,
            has_voltage: 0,
            has_hardware_error: 0,
            has_telemetry_age: 0,
            temperature: [0.0; N],
            voltage: [0.0; N],
            age: [0; N],
            read_every: [1; N],
            filled: 0,
            faults_filled: 0,
            sample_age: 0,
            has_sample_age: 0,
            signal: 0,
            hardware_error: [0; N],
            reason: [0; REASON_LEN],
        }
    }
}

impl<const N: usize, const D: usize> CState<N, D> {
    /// Forgets the previous call's writes and request.
    pub fn clear_requests(&mut self) {
        self.write = WriteBuf::default();
        self.signal = 0;
        self.reason = [0; REASON_LEN];
    }

    /// What the call asked the loop to do.
    pub fn request(&self) -> Request {
        match self.signal {
            0 => Request::None,
            1 => Request::Shutdown,
            _ => Request::EmergencyStop,
        }
    }

    /// The request's reason, up to its NUL.
    pub fn reason(&self) -> &[u8] {
        let end = self
            .reason
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(REASON_LEN);
        self.reason.get(..end).unwrap_or_default()
    }
}

/// Three axes (`kmr_axes`): an acceleration in m/s² or an angular rate in rad/s.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CAxes {
    /// Along or around x.
    pub x: f32,
    /// Along or around y.
    pub y: f32,
    /// Along or around z.
    pub z: f32,
}

impl From<[f32; 3]> for CAxes {
    fn from([x, y, z]: [f32; 3]) -> Self {
        Self { x, y, z }
    }
}

/// One IMU slot (`kmr_imu_reading`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CImu {
    /// Ticks since the reading arrived.
    pub age: u64,
    /// Readings lost so far.
    pub dropped: u64,
    /// m/s²; meaningful when `has_accel` is 1.
    pub accel: CAxes,
    /// rad/s; meaningful when `has_gyro` is 1.
    pub gyro: CAxes,
    /// 1: the slot has a reading (Rust's `Some`).
    pub present: u32,
    /// 1: the reading carries an acceleration.
    pub has_accel: u32,
    /// 1: the reading carries an angular rate.
    pub has_gyro: u32,
    /// Zero.
    pub reserved: u32,
}

/// One microphone slot (`kmr_mic_reading`), `W` samples long.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CMic<const W: usize> {
    /// Samples lost so far.
    pub dropped: u64,
    /// 1: the slot has a reading (Rust's `Some`).
    pub present: u32,
    /// Sample rate, Hz.
    pub rate_hz: u32,
    /// Samples that arrived since the last tick, `0..=W`.
    pub len: u32,
    /// Zero.
    pub reserved: u32,
    /// PCM, oldest first; the first `len` count.
    pub samples: [i16; W],
}

impl<const W: usize> Default for CMic<W> {
    fn default() -> Self {
        Self {
            dropped: 0,
            present: 0,
            rate_hz: 0,
            len: 0,
            reserved: 0,
            samples: [0; W],
        }
    }
}

/// The sensors as C reads them (`kmr_sensors`): `I` IMU slots and `M` microphone slots of `W`
/// samples, the engine's slot counts.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CSensors<const I: usize, const M: usize, const W: usize> {
    /// IMU slots, in the order the sensors were added.
    pub imu: [CImu; I],
    /// Microphone slots, likewise.
    pub mic: [CMic<W>; M],
}

impl<const I: usize, const M: usize, const W: usize> Default for CSensors<I, M, W> {
    fn default() -> Self {
        Self {
            imu: [CImu::default(); I],
            mic: [CMic::default(); M],
        }
    }
}

/// A C or C++ controller: `void name(const kmr_time *, kmr_state *, const kmr_sensors *, void
/// *ctx)` as `kmr.h` documents it.
pub type CFn<const N: usize, const D: usize, const I: usize, const M: usize, const W: usize> =
    unsafe extern "C" fn(*const CTime, *mut CState<N, D>, *const CSensors<I, M, W>, *mut c_void);

/// Runs the C controller `f` once over the snapshot: clears the previous call's writes and
/// request, then calls. `ctx` reaches it as `void *ctx`, null for `None`.
///
/// The crate's one `unsafe` call; no heap, no lock, no `dyn` (SRS NFR-1, NFR-2).
pub fn invoke<const N: usize, const D: usize, const I: usize, const M: usize, const W: usize, C>(
    f: CFn<N, D, I, M, W>,
    time: &CTime,
    state: &mut CState<N, D>,
    sensors: &CSensors<I, M, W>,
    ctx: Option<&mut C>,
) {
    state.clear_requests();
    let ctx = ctx.map_or(ptr::null_mut(), |c| ptr::from_mut(c).cast::<c_void>());
    // SAFETY: every pointer comes from a live reference of the exact type `kmr.h` declares for
    // the same N, depth and slot counts (`c!` asserts them at compile time), valid for the whole
    // call: `time` and `sensors` shared, `state` and `ctx` exclusive (or `ctx` null). A body that
    // keeps a pointer past the call, writes through a `const` one, past an array or through a
    // `ctx` of another type breaks C's memory safety, which is the user's (crate docs).
    unsafe { f(time, state, sensors, ctx) };
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    /// Rounds `x` up to a multiple of `a`, the C11 padding rule.
    const fn align_up(x: usize, a: usize) -> usize {
        x.div_ceil(a) * a
    }

    /// The number of names in a list.
    macro_rules! count {
        ($($name:ident $(: $ty:ident)?),*) => {
            [$(stringify!($name)),*].len()
        };
    }

    const FIELDS: usize = crate::for_each_field!(count);
    const FLAGS: usize = crate::for_each_flag!(count);

    /// Checks the size and alignment of the state types against a hand count by the C11 rules.
    fn check<const N: usize, const D: usize>() {
        let faults = 8 * FLAGS + 8;
        assert_eq!(size_of::<CTickFaults>(), faults);
        let slot_sampled = align_up(4 * N * FIELDS, 8);
        let slot = slot_sampled + 8 + faults;
        assert_eq!(size_of::<CSlot<N>>(), slot);
        assert_eq!(offset_of!(CSlot<N>, sampled), slot_sampled);
        assert_eq!(offset_of!(CSlot<N>, faults), slot_sampled + 8);
        let write = size_of::<WriteBuf<N>>();
        let mut off = D * slot;
        assert_eq!(offset_of!(CState<N, D>, write), off);
        off += write;
        assert_eq!(offset_of!(CState<N, D>, telemetry_age), off);
        off += 8 * N;
        assert_eq!(offset_of!(CState<N, D>, has_age), off);
        off += 8 * 5;
        assert_eq!(offset_of!(CState<N, D>, temperature), off);
        off += 4 * N * 4;
        assert_eq!(offset_of!(CState<N, D>, filled), off);
        off += 4 * 5;
        assert_eq!(offset_of!(CState<N, D>, hardware_error), off);
        off += N;
        assert_eq!(offset_of!(CState<N, D>, reason), off);
        off += REASON_LEN;
        assert_eq!(size_of::<CState<N, D>>(), align_up(off, 8));
        assert_eq!(align_of::<CState<N, D>>(), 8);
    }

    #[test]
    fn layout_by_hand() {
        check::<1, 4>();
        check::<4, 4>();
        check::<22, 4>();
        check::<64, 2>();
        assert_eq!(size_of::<CDuration>(), 16);
        assert_eq!(size_of::<CTime>(), 16 + 4 * 16 + 8);
        assert_eq!(offset_of!(CTime, late), 80);
        assert_eq!(size_of::<CAxes>(), 12);
        assert_eq!(size_of::<CImu>(), 56);
        assert_eq!(offset_of!(CImu, present), 40);
        assert_eq!(size_of::<CMic<1024>>(), 24 + 2048);
        assert_eq!(size_of::<CMic<3>>(), 32);
        assert_eq!(offset_of!(CSensors<4, 1, 1024>, mic), 4 * 56);
    }

    // A definition checks a type for FFI safety by value only, not behind a pointer.
    #[deny(improper_ctypes_definitions)]
    extern "C" fn by_value(
        time: CTime,
        state: CState<4, 4>,
        sensors: CSensors<4, 1, 16>,
        _faults: CTickFaults,
        _slot: CSlot<4>,
    ) -> u64 {
        time.tick + u64::from(state.filled) + sensors.imu[0].age
    }

    #[test]
    fn every_type_is_ffi_safe() {
        let n = by_value(
            CTime::default(),
            CState::default(),
            CSensors::default(),
            CTickFaults::default(),
            CSlot::default(),
        );
        assert_eq!(n, 0);
    }

    #[test]
    fn a_duration_splits_into_seconds_and_nanoseconds() {
        let d = CDuration::from(Duration::new(3, 250));
        assert_eq!((d.secs, d.nanos, d.reserved), (3, 250, 0));
    }

    /// A controller that stages `q[1]` = 5, asks for `ctx`'s signal (a `u32`) and counts it.
    unsafe extern "C" fn stage_and_ask(
        _: *const CTime,
        state: *mut CState<4, 4>,
        _: *const CSensors<4, 1, 16>,
        ctx: *mut c_void,
    ) {
        // SAFETY: `invoke` passes a live exclusive state and, in these tests, a `u32` context.
        let (state, level) = unsafe { (&mut *state, &mut *ctx.cast::<u32>()) };
        state.write.q[1] = 5.0;
        state.write.written.q |= 1 << 1;
        state.signal = *level;
        state.reason[..2].copy_from_slice(b"hi");
        *level += 1;
    }

    /// A controller that asks for a shutdown when `ctx` is null and an emergency stop otherwise.
    extern "C" fn null_ctx(
        _: *const CTime,
        state: *mut CState<4, 4>,
        _: *const CSensors<4, 1, 16>,
        ctx: *mut c_void,
    ) {
        // SAFETY: `invoke` passes a live exclusive state.
        let state = unsafe { &mut *state };
        state.signal = if ctx.is_null() { 1 } else { 2 };
    }

    #[test]
    fn a_call_starts_from_no_write_and_no_request() {
        let time = CTime::default();
        let sensors = CSensors::default();
        let mut state = CState::<4, 4>::default();
        state.write.written.qd = 1;
        let mut level = 1u32;
        invoke(stage_and_ask, &time, &mut state, &sensors, Some(&mut level));
        assert_eq!(state.write.written.q, 1 << 1);
        assert_eq!(state.write.written.qd, 0, "cleared before the call");
        assert_eq!(state.request(), Request::Shutdown);
        assert_eq!(state.reason(), b"hi");
        assert_eq!(level, 2);
        invoke(stage_and_ask, &time, &mut state, &sensors, Some(&mut level));
        assert_eq!(state.request(), Request::EmergencyStop);
        state.clear_requests();
        assert_eq!((state.request(), state.reason()), (Request::None, &b""[..]));
    }

    #[test]
    fn no_context_passes_null() {
        let time = CTime::default();
        let sensors = CSensors::default();
        let mut state = CState::<4, 4>::default();
        invoke(null_ctx, &time, &mut state, &sensors, None::<&mut u8>);
        assert_eq!(state.request(), Request::Shutdown);
        invoke(null_ctx, &time, &mut state, &sensors, Some(&mut 0u8));
        assert_eq!(state.request(), Request::EmergencyStop);
    }
}
