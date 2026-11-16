//! The language-neutral half of a script controller (features `python`, `lua`): one tick's
//! copy of the state, the clock and the sensors, the writes of one call, and the file watch.
//!
//! Each buffer is sized once and refilled in place every tick, so a tick allocates nothing.
//! The state is a snapshot of History taken before the call, and the writes are staged and
//! applied through the setters after it returns: within one call that is exactly what a Rust
//! controller sees, since no getter reads Desired back.

// The snapshot buffers are the API's, never the core's.
#![allow(clippy::disallowed_types)]

use std::path::Path;
use std::time::SystemTime;

use kmr_core::sensors::{MAX_IMUS, MAX_MICS};

use crate::{HISTORY_DEPTH, Kd, Kp, Mask, Q, Qd, Sensors, State, StateError, Tau, Time};

/// The state fields, in the order of the snapshot's rows.
pub(crate) const FIELDS: usize = 5;
/// Row of `Q`.
pub(crate) const F_Q: usize = 0;
/// Row of `Qd`.
pub(crate) const F_QD: usize = 1;
/// Row of `Tau`.
pub(crate) const F_TAU: usize = 2;
/// Row of `Kp`.
pub(crate) const F_KP: usize = 3;
/// Row of `Kd`.
pub(crate) const F_KD: usize = 4;

/// A copy of a state error, which is not `Clone`.
pub(crate) fn dup(e: &StateError) -> StateError {
    match *e {
        StateError::OutOfRange { index, len } => StateError::OutOfRange { index, len },
        StateError::HistoryValueOutOfRange {
            provided_depth,
            max_depth,
        } => StateError::HistoryValueOutOfRange {
            provided_depth,
            max_depth,
        },
        StateError::CriticalValueMissing => StateError::CriticalValueMissing,
        StateError::NoSample { depth } => StateError::NoSample { depth },
    }
}

/// One tick's fault bits: six per-actuator masks, four tick flags.
#[derive(Clone, Copy, Default)]
pub(crate) struct FaultBits {
    /// `stale`, `bus_error`, `offline`, `rejected`, `joint_limit`, `envelope`.
    pub(crate) masks: [u64; 6],
    /// `overrun`, `sensor_missing`, `remote_timeout`, `unauthorized`.
    pub(crate) tick: [bool; 4],
}

impl FaultBits {
    /// Actuators with at least one fault bit set.
    pub(crate) fn any_actuator(&self) -> u64 {
        self.masks.iter().fold(0, |a, m| a | m)
    }

    /// Whether no bit and no flag is set.
    pub(crate) fn is_clear(&self) -> bool {
        self.masks.iter().all(|&m| m == 0) && !self.tick.contains(&true)
    }
}

/// A mask as its one word.
fn word<const N: usize>(mask: Mask<N>) -> u64 {
    let [w] = mask.words();
    w
}

/// The actuator indices set in `bits`, below `n`.
pub(crate) fn indices(bits: u64, n: usize) -> impl Iterator<Item = usize> {
    (0..n.min(64)).filter(move |i| bits & (1 << i) != 0)
}

/// Sets `v[i]`; an index past the end is ignored.
fn put<T>(v: &mut [T], i: usize, x: T) {
    if let Some(s) = v.get_mut(i) {
        *s = x;
    }
}

/// History, faults, ages and telemetry of one tick, and the writes of one call.
pub(crate) struct Snapshot {
    /// The actuator count.
    pub(crate) n: usize,
    /// `(field * HISTORY_DEPTH + depth) * n + i`: what `prev_<field>_at(i, depth)` returns.
    values: Vec<Result<f32, StateError>>,
    /// History slots recorded so far.
    filled: usize,
    /// `prev_faults(depth)`.
    faults: Vec<Result<FaultBits, StateError>>,
    /// `age_at(i)`.
    pub(crate) age: Vec<Option<u32>>,
    /// `sample_age()`.
    pub(crate) sample_age: Option<u32>,
    /// `read_every(i)`.
    pub(crate) read_every: Vec<u32>,
    /// `temperature_at(i)`, °C.
    pub(crate) temperature: Vec<Option<f32>>,
    /// `voltage_at(i)`, V.
    pub(crate) voltage: Vec<Option<f32>>,
    /// `hardware_error_at(i)`.
    pub(crate) hardware_error: Vec<Option<u8>>,
    /// `telemetry_age_at(i)`.
    pub(crate) telemetry_age: Vec<Option<u64>>,
    /// `field * n + i`: the values this call staged.
    staged: Vec<f32>,
    /// One mask per field: the actuators this call staged.
    written: [u64; FIELDS],
}

impl Snapshot {
    /// An empty snapshot for `n` actuators.
    pub(crate) fn new(n: usize) -> Self {
        Self {
            n,
            values: (0..FIELDS * HISTORY_DEPTH * n)
                .map(|_| Err(StateError::CriticalValueMissing))
                .collect(),
            filled: 0,
            faults: (0..HISTORY_DEPTH)
                .map(|_| Err(StateError::CriticalValueMissing))
                .collect(),
            age: vec![None; n],
            sample_age: None,
            read_every: vec![1; n],
            temperature: vec![None; n],
            voltage: vec![None; n],
            hardware_error: vec![None; n],
            telemetry_age: vec![None; n],
            staged: vec![0.0; FIELDS * n],
            written: [0; FIELDS],
        }
    }

    /// Copies one field's History out of `state`.
    fn fill_field<T, const N: usize>(
        &mut self,
        f: usize,
        state: &State<'_, N>,
        f32_of: fn(T) -> f32,
    ) where
        T: crate::Field,
    {
        for depth in 0..HISTORY_DEPTH {
            let base = (f * HISTORY_DEPTH + depth) * self.n;
            let Some(slots) = self.values.get_mut(base..base + self.n) else {
                continue;
            };
            // DECISION: one row read per depth; only a row with an unread actuator, or past the
            // History, is read value by value for each actuator's own error.
            if let Ok(row) = state.field_row::<T>(depth) {
                for (slot, &v) in slots.iter_mut().zip(row) {
                    *slot = Ok(f32_of(v));
                }
                continue;
            }
            for (i, slot) in slots.iter_mut().enumerate() {
                *slot = state.field_at::<T>(i, depth).map(f32_of);
            }
        }
    }

    /// Takes this tick's snapshot of `state` and clears the staged writes.
    pub(crate) fn fill<const N: usize>(&mut self, state: &State<'_, N>) {
        self.fill_field::<Q, N>(F_Q, state, |v| v.0);
        self.fill_field::<Qd, N>(F_QD, state, |v| v.0);
        self.fill_field::<Tau, N>(F_TAU, state, |v| v.0);
        self.fill_field::<Kp, N>(F_KP, state, |v| v.0);
        self.fill_field::<Kd, N>(F_KD, state, |v| v.0);
        self.filled = 0;
        for (depth, slot) in self.faults.iter_mut().enumerate() {
            *slot = state.prev_faults(depth).map(|f| FaultBits {
                masks: [
                    word(f.stale()),
                    word(f.bus_error()),
                    word(f.offline()),
                    word(f.rejected()),
                    word(f.joint_limit()),
                    word(f.envelope()),
                ],
                tick: [
                    f.overrun(),
                    f.sensor_missing(),
                    f.remote_timeout(),
                    f.unauthorized(),
                ],
            });
            if !matches!(slot, Err(StateError::HistoryValueOutOfRange { .. })) {
                self.filled += 1;
            }
        }
        for i in 0..self.n {
            put(&mut self.age, i, state.age_at(i).ok().flatten());
            put(
                &mut self.temperature,
                i,
                state.temperature_at(i).ok().flatten().map(|c| c.0),
            );
            put(
                &mut self.voltage,
                i,
                state.voltage_at(i).ok().flatten().map(|v| v.0),
            );
            put(
                &mut self.read_every,
                i,
                state.read_every(i).map_or(1, |e| e.get()),
            );
            put(
                &mut self.hardware_error,
                i,
                state.hardware_error_at(i).ok().flatten(),
            );
            put(
                &mut self.telemetry_age,
                i,
                state.telemetry_age_at(i).ok().flatten(),
            );
        }
        self.sample_age = state.sample_age();
        self.written = [0; FIELDS];
    }

    /// Stages every value this call wrote, through the field's setter.
    pub(crate) fn apply<const N: usize>(&self, state: &mut State<'_, N>) {
        for (f, &mask) in self.written.iter().enumerate() {
            let mut bits = mask;
            while bits != 0 {
                let i = bits.trailing_zeros() as usize;
                bits &= bits - 1;
                let Some(&v) = self.staged.get(f * self.n + i) else {
                    continue;
                };
                match f {
                    F_Q => state.stage(Q(v), i),
                    F_QD => state.stage(Qd(v), i),
                    F_TAU => state.stage(Tau(v), i),
                    F_KP => state.stage(Kp(v), i),
                    _ => state.stage(Kd(v), i),
                }
            }
        }
    }

    /// The error of a depth past the History.
    pub(crate) fn too_deep(&self, depth: usize) -> StateError {
        StateError::HistoryValueOutOfRange {
            provided_depth: depth,
            max_depth: self.filled.saturating_sub(1),
        }
    }

    /// The error of an index past N.
    pub(crate) fn out_of_range(&self, i: usize) -> StateError {
        StateError::OutOfRange {
            index: i,
            len: self.n,
        }
    }

    /// `prev_<field>_at(i, depth)`.
    pub(crate) fn at(&self, f: usize, i: usize, depth: usize) -> Result<f32, StateError> {
        if depth >= HISTORY_DEPTH {
            return Err(self.too_deep(depth));
        }
        let base = (f * HISTORY_DEPTH + depth) * self.n;
        if i >= self.n {
            // DECISION: a missing slot is reported before a bad index, the order the core
            // checks them in.
            if let Some(Err(e @ StateError::HistoryValueOutOfRange { .. })) = self.values.get(base)
            {
                return Err(dup(e));
            }
            return Err(self.out_of_range(i));
        }
        match self.values.get(base + i) {
            Some(Ok(v)) => Ok(*v),
            Some(Err(e)) => Err(dup(e)),
            None => Err(StateError::CriticalValueMissing),
        }
    }

    /// `set_<field>_at(v, i)`.
    pub(crate) fn set_at(&mut self, f: usize, v: f32, i: usize) -> Result<(), StateError> {
        if i >= self.n {
            return Err(self.out_of_range(i));
        }
        if let Some(s) = self.staged.get_mut(f * self.n + i) {
            *s = v;
            self.written[f] |= 1 << i;
        }
        Ok(())
    }

    /// `prev_faults(depth)`.
    pub(crate) fn faults(&self, depth: usize) -> Result<FaultBits, StateError> {
        match self.faults.get(depth) {
            Some(Ok(bits)) => Ok(*bits),
            Some(Err(e)) => Err(dup(e)),
            None => Err(self.too_deep(depth)),
        }
    }

    /// `stale_at(i)`.
    pub(crate) fn is_stale(&self, i: usize) -> bool {
        matches!(self.faults.first(), Some(Ok(f)) if i < 64 && f.masks[0] & (1 << i) != 0)
    }

    /// One value of a per-actuator vector, or the index error.
    pub(crate) fn get<T: Copy>(&self, v: &[T], i: usize) -> Result<T, StateError> {
        v.get(i).copied().ok_or_else(|| self.out_of_range(i))
    }

    /// Age of the oldest sample over `indices`; `None` if one was never read.
    pub(crate) fn group_sample_age(&self, indices: &[usize]) -> Option<u32> {
        indices.iter().try_fold(0, |oldest, &i| {
            Some(oldest.max(self.age.get(i).copied().flatten()?))
        })
    }
}

/// The clock of the tick in progress; durations in seconds.
#[derive(Clone, Copy, Default)]
pub(crate) struct TimeSnap {
    /// `elapsed()`.
    pub(crate) elapsed: f64,
    /// `dt()`.
    pub(crate) dt: f64,
    /// `tick()`.
    pub(crate) tick: u64,
    /// `overrun_count()`.
    pub(crate) overrun_count: u64,
    /// `last_overrun()`.
    pub(crate) last_overrun: Option<f64>,
    /// `accumulated_overrun()`.
    pub(crate) accumulated_overrun: f64,
}

impl TimeSnap {
    /// Copies the clock.
    pub(crate) fn fill(&mut self, time: &Time<'_>) {
        self.elapsed = time.elapsed().as_secs_f64();
        self.dt = time.dt().as_secs_f64();
        self.tick = time.tick();
        self.overrun_count = time.overrun_count();
        self.last_overrun = time.last_overrun().map(|d| d.as_secs_f64());
        self.accumulated_overrun = time.accumulated_overrun().as_secs_f64();
    }
}

/// One IMU's newest reading.
#[derive(Clone, Copy, Default)]
pub(crate) struct ImuSnap {
    /// Whether the slot has a reading.
    pub(crate) present: bool,
    /// m/s².
    pub(crate) accel: Option<[f32; 3]>,
    /// rad/s.
    pub(crate) gyro: Option<[f32; 3]>,
    /// Ticks since it arrived.
    pub(crate) age: u64,
    /// Readings lost so far.
    pub(crate) dropped: u64,
}

/// One microphone's samples this tick.
#[derive(Default)]
pub(crate) struct MicSnap {
    /// Whether the slot has a reading.
    pub(crate) present: bool,
    /// PCM, oldest first; the buffer is kept across ticks.
    pub(crate) samples: Vec<i16>,
    /// Hz.
    pub(crate) rate_hz: u32,
    /// Samples lost so far.
    pub(crate) dropped: u64,
}

/// The IMUs and microphones of the tick.
pub(crate) struct SensorsSnap {
    /// `imu(i)`.
    pub(crate) imus: [ImuSnap; MAX_IMUS],
    /// `mic(i)`.
    pub(crate) mics: [MicSnap; MAX_MICS],
}

impl Default for SensorsSnap {
    fn default() -> Self {
        Self {
            imus: [ImuSnap::default(); MAX_IMUS],
            mics: core::array::from_fn(|_| MicSnap::default()),
        }
    }
}

impl SensorsSnap {
    /// Copies every sensor slot.
    pub(crate) fn fill(&mut self, sensors: &Sensors<'_>) {
        for (i, snap) in self.imus.iter_mut().enumerate() {
            *snap = sensors.imu(i).map_or_else(ImuSnap::default, |imu| ImuSnap {
                present: true,
                accel: imu.accel().map(|a| a.0),
                gyro: imu.gyro().map(|g| g.0),
                age: imu.age(),
                dropped: imu.dropped(),
            });
        }
        for (i, snap) in self.mics.iter_mut().enumerate() {
            snap.samples.clear();
            match sensors.mic(i) {
                Some(mic) => {
                    snap.present = true;
                    snap.samples.extend_from_slice(mic.samples());
                    snap.rate_hz = mic.rate_hz();
                    snap.dropped = mic.dropped();
                }
                None => snap.present = false,
            }
        }
    }

    /// Microphone `index`, when it has a reading.
    pub(crate) fn mic(&self, index: usize) -> Option<&MicSnap> {
        self.mics.get(index).filter(|m| m.present)
    }

    /// IMU `index`, when it has a reading.
    pub(crate) fn imu(&self, index: usize) -> Option<ImuSnap> {
        self.imus.get(index).filter(|s| s.present).copied()
    }
}

/// What a watcher last saw of a script file.
pub(crate) struct Seen {
    /// Its modification time.
    pub(crate) modified: Option<SystemTime>,
    /// Its text.
    pub(crate) source: String,
}

impl Seen {
    /// Rereads `path` if its mtime moved; true when the text changed, now in `source`.
    pub(crate) fn changed(&mut self, path: &Path) -> bool {
        let Ok(modified) = std::fs::metadata(path).and_then(|m| m.modified()) else {
            return false;
        };
        if self.modified == Some(modified) {
            return false;
        }
        self.modified = Some(modified);
        let Ok(source) = std::fs::read_to_string(path) else {
            return false;
        };
        if source == self.source {
            return false;
        }
        self.source = source;
        true
    }
}

/// `EachTick` as `each_tick`: the last path segment of the label's type, in snake case.
pub(crate) fn phase_name<S>() -> String {
    let full = core::any::type_name::<S>();
    let last = full.rsplit("::").next().unwrap_or(full);
    let mut out = String::with_capacity(last.len() + 4);
    for (k, c) in last.chars().enumerate() {
        if c.is_ascii_uppercase() {
            if k > 0 {
                out.push('_');
            }
            out.push(c.to_ascii_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}
