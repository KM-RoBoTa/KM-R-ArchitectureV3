//! The objects a script's function gets: `time`, `state`, `sensors`. One of each per
//! controller, refilled in place every tick, so a call allocates nothing on the Rust side.
//!
//! `state` is a snapshot of History taken before the call, and the writes are staged and
//! applied through the setters after it returns: within one call that is exactly what a Rust
//! controller sees, since no getter reads Desired back.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::ffi::snapshot::{
    F_KD, F_KP, F_Q, F_QD, F_TAU, FaultBits, ImuSnap, SensorsSnap, Snapshot, TimeSnap, indices,
};
use crate::{Sensors, State, StateError, Time};

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

/// `state`: History, faults, ages and telemetry of one tick, and the writes of one call.
#[pyclass(name = "State", module = "kmr")]
pub(crate) struct PyState {
    /// The snapshot.
    snap: Snapshot,
}

impl PyState {
    /// An empty snapshot for `n` actuators.
    pub(crate) fn new(n: usize) -> Self {
        Self {
            snap: Snapshot::new(n),
        }
    }

    /// Takes this tick's snapshot of `state` and clears the staged writes.
    pub(crate) fn fill<const N: usize>(&mut self, state: &State<'_, N>) {
        self.snap.fill(state);
    }

    /// Stages every value this call wrote, through the field's setter.
    pub(crate) fn apply<const N: usize>(&self, state: &mut State<'_, N>) {
        self.snap.apply(state);
    }

    /// `prev_<field>_at(i, depth)`.
    fn at(&self, f: usize, i: usize, depth: usize) -> PyResult<f32> {
        self.snap.at(f, i, depth).map_err(|e| raise(&e))
    }

    /// `prev_<field>(depth)`: every actuator, or the first one's error.
    fn row(&self, f: usize, depth: usize) -> PyResult<Vec<f32>> {
        (0..self.snap.n).map(|i| self.at(f, i, depth)).collect()
    }

    /// `set_<field>_at(v, i)`.
    fn set_at(&mut self, f: usize, v: f32, i: usize) -> PyResult<()> {
        self.snap.set_at(f, v, i).map_err(|e| raise(&e))
    }

    /// `set_all_<field>(values)`: one value per actuator.
    fn set_all(&mut self, f: usize, values: &[f32]) -> PyResult<()> {
        if values.len() != self.snap.n {
            return Err(PyValueError::new_err(format!(
                "{} values for {} actuators",
                values.len(),
                self.snap.n
            )));
        }
        for (i, &v) in values.iter().enumerate() {
            self.set_at(f, v, i)?;
        }
        Ok(())
    }

    /// One value of a per-actuator vector, or the index error.
    fn get<T: Copy>(&self, v: &[T], i: usize) -> PyResult<T> {
        self.snap.get(v, i).map_err(|e| raise(&e))
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
                self.snap
                    .faults(depth)
                    .map(|bits| PyFaults { n: self.snap.n, bits })
                    .map_err(|e| raise(&e))
            }

            /// Whether this tick's read of actuator `i` failed.
            fn stale_at(&self, i: usize) -> bool {
                self.snap.is_stale(i)
            }

            /// Ticks since actuator `i` was last read; `None` before its first read.
            fn age_at(&self, i: usize) -> PyResult<Option<u32>> {
                self.get(&self.snap.age, i)
            }

            /// Age of the oldest sample; `None` while an actuator was never read.
            fn sample_age(&self) -> Option<u32> {
                self.snap.sample_age
            }

            /// How often actuator `i` is read, in ticks.
            fn read_every(&self, i: usize) -> PyResult<u32> {
                self.get(&self.snap.read_every, i)
            }

            /// Temperature of actuator `i`, °C; `None` without a reading.
            fn temperature_at(&self, i: usize) -> PyResult<Option<f32>> {
                self.get(&self.snap.temperature, i)
            }

            /// Temperature of every actuator.
            fn temperatures(&self) -> Vec<Option<f32>> {
                self.snap.temperature.clone()
            }

            /// Input voltage of actuator `i`, V; `None` without a reading.
            fn voltage_at(&self, i: usize) -> PyResult<Option<f32>> {
                self.get(&self.snap.voltage, i)
            }

            /// Raw hardware error status of actuator `i`.
            fn hardware_error_at(&self, i: usize) -> PyResult<Option<u8>> {
                self.get(&self.snap.hardware_error, i)
            }

            /// Ticks since the telemetry of actuator `i` was refreshed.
            fn telemetry_age_at(&self, i: usize) -> PyResult<Option<u64>> {
                self.get(&self.snap.telemetry_age, i)
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
                self.state.borrow(py).snap.group_sample_age(&self.indices)
            }

            /// Whether this tick's read failed on one of the group.
            fn stale(&self, py: Python<'_>) -> bool {
                let s = self.state.borrow(py);
                self.indices.iter().any(|&i| s.snap.is_stale(i))
            }

            /// Temperature of each group actuator.
            fn temperatures(&self, py: Python<'_>) -> Vec<Option<f32>> {
                let s = self.state.borrow(py);
                self.indices
                    .iter()
                    .map(|&i| s.snap.temperature.get(i).copied().flatten())
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
        if let Some(&bad) = self.indices.iter().find(|&&i| i >= s.snap.n) {
            return Err(raise(&s.snap.out_of_range(bad)));
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
        indices(self.bits.masks[0], self.n).collect()
    }
    /// Actuators whose bus transaction failed.
    fn bus_error(&self) -> Vec<usize> {
        indices(self.bits.masks[1], self.n).collect()
    }
    /// Actuators that did not answer.
    fn offline(&self) -> Vec<usize> {
        indices(self.bits.masks[2], self.n).collect()
    }
    /// Actuators written but not sent.
    fn rejected(&self) -> Vec<usize> {
        indices(self.bits.masks[3], self.n).collect()
    }
    /// Actuators whose written value breached their limit.
    fn joint_limit(&self) -> Vec<usize> {
        indices(self.bits.masks[4], self.n).collect()
    }
    /// Actuators whose written value was clamped to their envelope.
    fn envelope(&self) -> Vec<usize> {
        indices(self.bits.masks[5], self.n).collect()
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
        indices(self.bits.any_actuator(), self.n).collect()
    }
    /// Whether no bit and no flag is set.
    fn is_clear(&self) -> bool {
        self.bits.is_clear()
    }
}

/// `time`: the clock of the tick in progress; durations in seconds.
#[pyclass(name = "Time", module = "kmr")]
#[derive(Default)]
pub(crate) struct PyTime(TimeSnap);

impl PyTime {
    /// Copies the clock.
    pub(crate) fn fill(&mut self, time: &Time<'_>) {
        self.0.fill(time);
    }
}

#[pymethods]
impl PyTime {
    /// Seconds since the tick grid was armed, stamped at the start of the tick.
    fn elapsed(&self) -> f64 {
        self.0.elapsed
    }
    /// The tick duration, s.
    fn dt(&self) -> f64 {
        self.0.dt
    }
    /// Index of the tick in progress, 0 for the first one.
    fn tick(&self) -> u64 {
        self.0.tick
    }
    /// How many ticks finished after their deadline.
    fn overrun_count(&self) -> u64 {
        self.0.overrun_count
    }
    /// How late the previous tick finished, s; `None` if on time.
    fn last_overrun(&self) -> Option<f64> {
        self.0.last_overrun
    }
    /// Sum of every overrun so far, s.
    fn accumulated_overrun(&self) -> f64 {
        self.0.accumulated_overrun
    }
}

/// `sensors`: the IMUs and microphones of the tick.
#[pyclass(name = "Sensors", module = "kmr")]
#[derive(Default)]
pub(crate) struct PySensors(SensorsSnap);

impl PySensors {
    /// Copies every sensor slot.
    pub(crate) fn fill(&mut self, sensors: &Sensors<'_>) {
        self.0.fill(sensors);
    }
}

#[pymethods]
impl PySensors {
    /// IMU `index`, in the order the sensors were added; `None` without a reading.
    fn imu(&self, index: usize) -> Option<PyImu> {
        self.0.imu(index).map(|snap| PyImu { snap })
    }

    /// Microphone `index`, in the order the sensors were added; `None` without a reading.
    fn mic(slf: Bound<'_, Self>, index: usize) -> Option<PyMic> {
        let present = slf.borrow().0.mic(index).is_some();
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
        s.0.mics
            .get(self.index)
            .map(|m| m.samples.clone())
            .unwrap_or_default()
    }
    /// Sample rate, Hz.
    fn rate_hz(&self, py: Python<'_>) -> u32 {
        self.sensors
            .borrow(py)
            .0
            .mics
            .get(self.index)
            .map_or(0, |m| m.rate_hz)
    }
    /// Samples lost so far.
    fn dropped(&self, py: Python<'_>) -> u64 {
        self.sensors
            .borrow(py)
            .0
            .mics
            .get(self.index)
            .map_or(0, |m| m.dropped)
    }
}
