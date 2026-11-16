//! The objects a script's function gets: `time`, `state`, `sensors`. One of each per
//! controller, refilled in place every tick, so a call allocates nothing on the Rust side.
//!
//! `state` is a snapshot of History taken before the call, and the writes are staged and
//! applied through the setters after it returns: within one call that is exactly what a Rust
//! controller sees, since no getter reads Desired back.

use kmr_core::sensors::{MAX_IMUS, MAX_MICS};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::{HISTORY_DEPTH, Kd, Kp, Mask, Q, Qd, Sensors, State, StateError, Tau, Time};

mod exc {
    pyo3::create_exception!(
        kmr,
        StateError,
        pyo3::exceptions::PyException,
        "A state read or write that the Rust API answers with `Err`."
    );
}

/// The Python class of a state error, for the script's globals.
pub(crate) fn state_error_type(py: Python<'_>) -> Bound<'_, pyo3::types::PyType> {
    py.get_type::<exc::StateError>()
}

/// A Rust state error as the Python exception.
fn raise(e: &StateError) -> PyErr {
    exc::StateError::new_err(e.to_string())
}

/// The state fields, in the order of the snapshot's rows.
const FIELDS: usize = 5;
/// Row of `Q`.
const F_Q: usize = 0;
/// Row of `Qd`.
const F_QD: usize = 1;
/// Row of `Tau`.
const F_TAU: usize = 2;
/// Row of `Kp`.
const F_KP: usize = 3;
/// Row of `Kd`.
const F_KD: usize = 4;

/// One tick's fault bits: six per-actuator masks, four tick flags.
#[derive(Clone, Copy, Default)]
struct FaultBits {
    /// `stale`, `bus_error`, `offline`, `rejected`, `joint_limit`, `envelope`.
    masks: [u64; 6],
    /// `overrun`, `sensor_missing`, `remote_timeout`, `unauthorized`.
    tick: [bool; 4],
}

/// A mask as its one word.
fn word<const N: usize>(mask: Mask<N>) -> u64 {
    let [w] = mask.words();
    w
}

/// The actuator indices set in `bits`, below `n`.
fn indices(bits: u64, n: usize) -> Vec<usize> {
    (0..n.min(64)).filter(|i| bits & (1 << i) != 0).collect()
}

/// Sets `v[i]`; an index past the end is ignored.
fn put<T>(v: &mut [T], i: usize, x: T) {
    if let Some(s) = v.get_mut(i) {
        *s = x;
    }
}

/// `state`: History, faults, ages and telemetry of one tick, and the writes of one call.
#[pyclass(name = "State", module = "kmr")]
pub(crate) struct PyState {
    /// The actuator count.
    n: usize,
    /// `(field * HISTORY_DEPTH + depth) * n + i`: what `prev_<field>_at(i, depth)` returns.
    values: Vec<Result<f32, StateError>>,
    /// History slots recorded so far.
    filled: usize,
    /// `prev_faults(depth)`.
    faults: Vec<Result<FaultBits, StateError>>,
    /// `age_at(i)`.
    age: Vec<Option<u32>>,
    /// `sample_age()`.
    sample_age: Option<u32>,
    /// `read_every(i)`.
    read_every: Vec<u32>,
    /// `temperature_at(i)`, °C.
    temperature: Vec<Option<f32>>,
    /// `voltage_at(i)`, V.
    voltage: Vec<Option<f32>>,
    /// `hardware_error_at(i)`.
    hardware_error: Vec<Option<u8>>,
    /// `telemetry_age_at(i)`.
    telemetry_age: Vec<Option<u64>>,
    /// `field * n + i`: the values this call staged.
    staged: Vec<f32>,
    /// One mask per field: the actuators this call staged.
    written: [u64; FIELDS],
}

impl PyState {
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
            for i in 0..self.n {
                if let Some(slot) = self.values.get_mut(base + i) {
                    *slot = state.field_at::<T>(i, depth).map(f32_of);
                }
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
    fn too_deep(&self, depth: usize) -> PyErr {
        raise(&StateError::HistoryValueOutOfRange {
            provided_depth: depth,
            max_depth: self.filled.saturating_sub(1),
        })
    }

    /// `prev_<field>_at(i, depth)`.
    fn at(&self, f: usize, i: usize, depth: usize) -> PyResult<f32> {
        if depth >= HISTORY_DEPTH {
            return Err(self.too_deep(depth));
        }
        let base = (f * HISTORY_DEPTH + depth) * self.n;
        if i >= self.n {
            // DECISION: a missing slot is reported before a bad index, the order the core
            // checks them in.
            if let Some(Err(e @ StateError::HistoryValueOutOfRange { .. })) = self.values.get(base)
            {
                return Err(raise(e));
            }
            return Err(raise(&StateError::OutOfRange {
                index: i,
                len: self.n,
            }));
        }
        match self.values.get(base + i) {
            Some(Ok(v)) => Ok(*v),
            Some(Err(e)) => Err(raise(e)),
            None => Err(raise(&StateError::CriticalValueMissing)),
        }
    }

    /// `prev_<field>(depth)`: every actuator, or the first one's error.
    fn row(&self, f: usize, depth: usize) -> PyResult<Vec<f32>> {
        (0..self.n).map(|i| self.at(f, i, depth)).collect()
    }

    /// `set_<field>_at(v, i)`.
    fn set_at(&mut self, f: usize, v: f32, i: usize) -> PyResult<()> {
        if i >= self.n {
            return Err(raise(&StateError::OutOfRange {
                index: i,
                len: self.n,
            }));
        }
        if let Some(s) = self.staged.get_mut(f * self.n + i) {
            *s = v;
            self.written[f] |= 1 << i;
        }
        Ok(())
    }

    /// `set_all_<field>(values)`: one value per actuator.
    fn set_all(&mut self, f: usize, values: &[f32]) -> PyResult<()> {
        if values.len() != self.n {
            return Err(PyValueError::new_err(format!(
                "{} values for {} actuators",
                values.len(),
                self.n
            )));
        }
        for (i, &v) in values.iter().enumerate() {
            self.set_at(f, v, i)?;
        }
        Ok(())
    }

    /// The error of an index past N.
    fn out_of_range(&self, i: usize) -> PyErr {
        raise(&StateError::OutOfRange {
            index: i,
            len: self.n,
        })
    }

    /// `stale_at(i)`.
    fn is_stale(&self, i: usize) -> bool {
        matches!(self.faults.first(), Some(Ok(f)) if i < 64 && f.masks[0] & (1 << i) != 0)
    }

    /// One value of a per-actuator vector, or the index error.
    fn get<T: Copy>(&self, v: &[T], i: usize) -> PyResult<T> {
        v.get(i).copied().ok_or_else(|| self.out_of_range(i))
    }
}

/// Generates the field methods of `State` and of `Group`, with the rest of each class.
macro_rules! classes {
    ($(
        $f:ident: $get:ident $(| $alias:ident)*, $at:ident, $prev:ident, $prev_at:ident,
        $set_all:ident, $set_at:ident;
    )*) => {
        #[pymethods]
        impl PyState {
            $(
                /// Newest value of every actuator; `None` until each has a sample.
                fn $get(&self) -> Option<Vec<f32>> {
                    self.row($f, 0).ok()
                }
                $(
                    /// Alias of the field getter.
                    fn $alias(&self) -> Option<Vec<f32>> {
                        self.$get()
                    }
                )*
                /// Newest value at actuator `i`.
                fn $at(&self, i: usize) -> PyResult<f32> {
                    self.at($f, i, 0)
                }
                /// Every actuator, `depth` ticks back.
                fn $prev(&self, depth: usize) -> PyResult<Vec<f32>> {
                    self.row($f, depth)
                }
                /// Actuator `i`, `depth` ticks back.
                fn $prev_at(&self, i: usize, depth: usize) -> PyResult<f32> {
                    self.at($f, i, depth)
                }
                /// Stages one value per actuator.
                fn $set_all(&mut self, values: Vec<f32>) -> PyResult<()> {
                    self.set_all($f, &values)
                }
                /// Stages `value` at actuator `i`.
                fn $set_at(&mut self, value: f32, i: usize) -> PyResult<()> {
                    self.set_at($f, value, i)
                }
            )*

            /// This tick's faults; `None` before the first tick.
            fn faults(&self) -> Option<PyFaults> {
                self.prev_faults(0).ok()
            }

            /// The faults `depth` ticks back.
            fn prev_faults(&self, depth: usize) -> PyResult<PyFaults> {
                match self.faults.get(depth) {
                    Some(Ok(bits)) => Ok(PyFaults { n: self.n, bits: *bits }),
                    Some(Err(e)) => Err(raise(e)),
                    None => Err(self.too_deep(depth)),
                }
            }

            /// Whether this tick's read of actuator `i` failed.
            fn stale_at(&self, i: usize) -> bool {
                self.is_stale(i)
            }

            /// Ticks since actuator `i` was last read; `None` before its first read.
            fn age_at(&self, i: usize) -> PyResult<Option<u32>> {
                self.get(&self.age, i)
            }

            /// Age of the oldest sample; `None` while an actuator was never read.
            fn sample_age(&self) -> Option<u32> {
                self.sample_age
            }

            /// How often actuator `i` is read, in ticks.
            fn read_every(&self, i: usize) -> PyResult<u32> {
                self.get(&self.read_every, i)
            }

            /// Temperature of actuator `i`, °C; `None` without a reading.
            fn temperature_at(&self, i: usize) -> PyResult<Option<f32>> {
                self.get(&self.temperature, i)
            }

            /// Temperature of every actuator.
            fn temperatures(&self) -> Vec<Option<f32>> {
                self.temperature.clone()
            }

            /// Input voltage of actuator `i`, V; `None` without a reading.
            fn voltage_at(&self, i: usize) -> PyResult<Option<f32>> {
                self.get(&self.voltage, i)
            }

            /// Raw hardware error status of actuator `i`.
            fn hardware_error_at(&self, i: usize) -> PyResult<Option<u8>> {
                self.get(&self.hardware_error, i)
            }

            /// Ticks since the telemetry of actuator `i` was refreshed.
            fn telemetry_age_at(&self, i: usize) -> PyResult<Option<u64>> {
                self.get(&self.telemetry_age, i)
            }

            /// A view of the actuators `indices`, in that order.
            fn group(slf: Bound<'_, Self>, indices: Vec<usize>) -> PyGroup {
                PyGroup { state: slf.unbind(), indices }
            }
        }

        #[pymethods]
        impl PyGroup {
            $(
                /// Newest value of each group actuator; `None` if one has none.
                fn $get(&self, py: Python<'_>) -> Option<Vec<f32>> {
                    let s = self.state.borrow(py);
                    self.indices.iter().map(|&i| s.at($f, i, 0).ok()).collect()
                }
                $(
                    /// Alias of the field getter.
                    fn $alias(&self, py: Python<'_>) -> Option<Vec<f32>> {
                        self.$get(py)
                    }
                )*
                /// Each group actuator, `depth` ticks back.
                fn $prev(&self, py: Python<'_>, depth: usize) -> PyResult<Vec<f32>> {
                    let s = self.state.borrow(py);
                    self.indices.iter().map(|&i| s.at($f, i, depth)).collect()
                }
                /// Stages one value per group actuator; nothing on an error.
                fn $set_all(&self, py: Python<'_>, values: Vec<f32>) -> PyResult<()> {
                    self.set_all(py, $f, &values)
                }
            )*

            /// Age of the oldest sample over the group.
            fn sample_age(&self, py: Python<'_>) -> Option<u32> {
                let s = self.state.borrow(py);
                self.indices.iter().try_fold(0, |oldest, &i| {
                    Some(oldest.max(s.age.get(i).copied().flatten()?))
                })
            }

            /// Whether this tick's read failed on one of the group.
            fn stale(&self, py: Python<'_>) -> bool {
                let s = self.state.borrow(py);
                self.indices.iter().any(|&i| s.is_stale(i))
            }

            /// Temperature of each group actuator.
            fn temperatures(&self, py: Python<'_>) -> Vec<Option<f32>> {
                let s = self.state.borrow(py);
                self.indices
                    .iter()
                    .map(|&i| s.temperature.get(i).copied().flatten())
                    .collect()
            }
        }
    };
}

classes! {
    F_Q: q | position, q_at, prev_q, prev_q_at, set_all_q, set_q_at;
    F_QD: qd | velocity, qd_at, prev_qd, prev_qd_at, set_all_qd, set_qd_at;
    F_TAU: tau | torque | effort, tau_at, prev_tau, prev_tau_at, set_all_tau, set_tau_at;
    F_KP: kp, kp_at, prev_kp, prev_kp_at, set_all_kp, set_kp_at;
    F_KD: kd, kd_at, prev_kd, prev_kd_at, set_all_kd, set_kd_at;
}

/// `state.group(indices)`: the same accessors over chosen actuators.
#[pyclass(name = "Group", module = "kmr")]
pub(crate) struct PyGroup {
    /// The state it views.
    state: Py<PyState>,
    /// The actuators, in group order.
    indices: Vec<usize>,
}

impl PyGroup {
    /// Stages `values` on the group: every index is checked first, so an error stages nothing.
    fn set_all(&self, py: Python<'_>, f: usize, values: &[f32]) -> PyResult<()> {
        if values.len() != self.indices.len() {
            return Err(PyValueError::new_err(format!(
                "{} values for a group of {}",
                values.len(),
                self.indices.len()
            )));
        }
        let mut s = self.state.borrow_mut(py);
        if let Some(&bad) = self.indices.iter().find(|&&i| i >= s.n) {
            return Err(s.out_of_range(bad));
        }
        for (&i, &v) in self.indices.iter().zip(values) {
            s.set_at(f, v, i)?;
        }
        Ok(())
    }
}

/// `state.faults()`: one tick's fault bits; masks read as lists of actuator indices.
#[pyclass(name = "Faults", module = "kmr")]
pub(crate) struct PyFaults {
    /// The actuator count.
    n: usize,
    /// The bits.
    bits: FaultBits,
}

#[pymethods]
impl PyFaults {
    /// Actuators whose read failed.
    fn stale(&self) -> Vec<usize> {
        indices(self.bits.masks[0], self.n)
    }
    /// Actuators whose bus transaction failed.
    fn bus_error(&self) -> Vec<usize> {
        indices(self.bits.masks[1], self.n)
    }
    /// Actuators that did not answer.
    fn offline(&self) -> Vec<usize> {
        indices(self.bits.masks[2], self.n)
    }
    /// Actuators written but not sent.
    fn rejected(&self) -> Vec<usize> {
        indices(self.bits.masks[3], self.n)
    }
    /// Actuators whose written value breached their limit.
    fn joint_limit(&self) -> Vec<usize> {
        indices(self.bits.masks[4], self.n)
    }
    /// Actuators whose written value was clamped to their envelope.
    fn envelope(&self) -> Vec<usize> {
        indices(self.bits.masks[5], self.n)
    }
    /// The tick finished after its deadline.
    fn overrun(&self) -> bool {
        self.bits.tick[0]
    }
    /// A sensor is missing this tick.
    fn sensor_missing(&self) -> bool {
        self.bits.tick[1]
    }
    /// A remote source fell silent past its timeout.
    fn remote_timeout(&self) -> bool {
        self.bits.tick[2]
    }
    /// A message from an unknown identity was dropped.
    fn unauthorized(&self) -> bool {
        self.bits.tick[3]
    }
    /// Actuators with at least one fault bit set.
    fn any_actuator(&self) -> Vec<usize> {
        indices(self.bits.masks.iter().fold(0, |a, m| a | m), self.n)
    }
    /// Whether no bit and no flag is set.
    fn is_clear(&self) -> bool {
        self.bits.masks.iter().all(|&m| m == 0) && !self.bits.tick.contains(&true)
    }
}

/// `time`: the clock of the tick in progress; durations in seconds.
#[pyclass(name = "Time", module = "kmr")]
#[derive(Default)]
pub(crate) struct PyTime {
    /// `elapsed()`.
    elapsed: f64,
    /// `dt()`.
    dt: f64,
    /// `tick()`.
    tick: u64,
    /// `overrun_count()`.
    overrun_count: u64,
    /// `last_overrun()`.
    last_overrun: Option<f64>,
    /// `accumulated_overrun()`.
    accumulated_overrun: f64,
}

impl PyTime {
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

#[pymethods]
impl PyTime {
    /// Seconds since the tick grid was armed, stamped at the start of the tick.
    fn elapsed(&self) -> f64 {
        self.elapsed
    }
    /// The tick duration, s.
    fn dt(&self) -> f64 {
        self.dt
    }
    /// Index of the tick in progress, 0 for the first one.
    fn tick(&self) -> u64 {
        self.tick
    }
    /// How many ticks finished after their deadline.
    fn overrun_count(&self) -> u64 {
        self.overrun_count
    }
    /// How late the previous tick finished, s; `None` if on time.
    fn last_overrun(&self) -> Option<f64> {
        self.last_overrun
    }
    /// Sum of every overrun so far, s.
    fn accumulated_overrun(&self) -> f64 {
        self.accumulated_overrun
    }
}

/// One IMU's newest reading.
#[derive(Clone, Copy, Default)]
struct ImuSnap {
    /// Whether the slot has a reading.
    present: bool,
    /// m/s².
    accel: Option<[f32; 3]>,
    /// rad/s.
    gyro: Option<[f32; 3]>,
    /// Ticks since it arrived.
    age: u64,
    /// Readings lost so far.
    dropped: u64,
}

/// One microphone's samples this tick.
#[derive(Default)]
struct MicSnap {
    /// Whether the slot has a reading.
    present: bool,
    /// PCM, oldest first; the buffer is kept across ticks.
    samples: Vec<i16>,
    /// Hz.
    rate_hz: u32,
    /// Samples lost so far.
    dropped: u64,
}

/// `sensors`: the IMUs and microphones of the tick.
#[pyclass(name = "Sensors", module = "kmr")]
pub(crate) struct PySensors {
    /// `imu(i)`.
    imus: [ImuSnap; MAX_IMUS],
    /// `mic(i)`.
    mics: [MicSnap; MAX_MICS],
}

impl Default for PySensors {
    fn default() -> Self {
        Self {
            imus: [ImuSnap::default(); MAX_IMUS],
            mics: core::array::from_fn(|_| MicSnap::default()),
        }
    }
}

impl PySensors {
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
}

#[pymethods]
impl PySensors {
    /// IMU `index`, in the order the sensors were added; `None` without a reading.
    fn imu(&self, index: usize) -> Option<PyImu> {
        self.imus
            .get(index)
            .filter(|s| s.present)
            .map(|&snap| PyImu { snap })
    }

    /// Microphone `index`, in the order the sensors were added; `None` without a reading.
    fn mic(slf: Bound<'_, Self>, index: usize) -> Option<PyMic> {
        let present = slf.borrow().mics.get(index).is_some_and(|m| m.present);
        present.then(|| PyMic {
            sensors: slf.unbind(),
            index,
        })
    }
}

/// A reading on three axes: `x()`, `y()`, `z()`.
#[pyclass(name = "Axes", module = "kmr")]
pub(crate) struct PyAxes([f32; 3]);

#[pymethods]
impl PyAxes {
    /// Along or around x.
    fn x(&self) -> f32 {
        self.0[0]
    }
    /// Along or around y.
    fn y(&self) -> f32 {
        self.0[1]
    }
    /// Along or around z.
    fn z(&self) -> f32 {
        self.0[2]
    }
    /// `Axes(x, y, z)`.
    fn __repr__(&self) -> String {
        let [x, y, z] = self.0;
        format!("Axes({x}, {y}, {z})")
    }
}

/// `sensors.imu(i)`.
#[pyclass(name = "Imu", module = "kmr")]
pub(crate) struct PyImu {
    /// The reading.
    snap: ImuSnap,
}

#[pymethods]
impl PyImu {
    /// Acceleration, m/s²; `None` when that half of the sensor is off.
    fn accel(&self) -> Option<PyAxes> {
        self.snap.accel.map(PyAxes)
    }
    /// Angular rate, rad/s; `None` when that half of the sensor is off.
    fn gyro(&self) -> Option<PyAxes> {
        self.snap.gyro.map(PyAxes)
    }
    /// Ticks since the reading arrived.
    fn age(&self) -> u64 {
        self.snap.age
    }
    /// Readings lost so far.
    fn dropped(&self) -> u64 {
        self.snap.dropped
    }
}

/// `sensors.mic(i)`.
#[pyclass(name = "Mic", module = "kmr")]
pub(crate) struct PyMic {
    /// The sensors it reads from.
    sensors: Py<PySensors>,
    /// Its slot.
    index: usize,
}

#[pymethods]
impl PyMic {
    /// PCM samples that arrived since the last tick, oldest first.
    fn samples(&self, py: Python<'_>) -> Vec<i16> {
        let s = self.sensors.borrow(py);
        s.mics
            .get(self.index)
            .map(|m| m.samples.clone())
            .unwrap_or_default()
    }
    /// Sample rate, Hz.
    fn rate_hz(&self, py: Python<'_>) -> u32 {
        self.sensors
            .borrow(py)
            .mics
            .get(self.index)
            .map_or(0, |m| m.rate_hz)
    }
    /// Samples lost so far.
    fn dropped(&self, py: Python<'_>) -> u64 {
        self.sensors
            .borrow(py)
            .mics
            .get(self.index)
            .map_or(0, |m| m.dropped)
    }
}
