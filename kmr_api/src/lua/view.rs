//! The Lua objects a script's function gets: `time`, `state`, `sensors`, over the shared
//! [snapshot](crate::snapshot). One of each per controller, refilled in place every tick.

use mlua::{
    AnyUserData, Error as LuaError, Lua, MetaMethod, Result as LuaResult, Table, UserData,
    UserDataMethods,
};

use crate::StateError;
use crate::snapshot::{
    F_KD, F_KP, F_Q, F_QD, F_TAU, FaultBits, ImuSnap, SensorsSnap, Snapshot, TimeSnap, indices,
};

/// A Rust state error as the Lua error; the `StateError` global recognises it.
pub(crate) fn raise(e: StateError) -> LuaError {
    LuaError::external(e)
}

/// Whether a Lua error value is a state error.
pub(crate) fn is_state_error(e: &LuaError) -> bool {
    e.downcast_ref::<StateError>().is_some()
}

/// The method names of one field: getter and its aliases, `_at`, `prev_`, `prev_..._at`,
/// `set_all_`, `set_..._at`.
struct Names {
    /// Snapshot row.
    f: usize,
    /// `q`, `position`.
    get: &'static [&'static str],
    /// `q_at`.
    at: &'static str,
    /// `prev_q`.
    prev: &'static str,
    /// `prev_q_at`.
    prev_at: &'static str,
    /// `set_all_q`.
    set_all: &'static str,
    /// `set_q_at`.
    set_at: &'static str,
}

/// Every field, as the Rust controller API names it.
const FIELDS: [Names; 5] = [
    Names {
        f: F_Q,
        get: &["q", "position"],
        at: "q_at",
        prev: "prev_q",
        prev_at: "prev_q_at",
        set_all: "set_all_q",
        set_at: "set_q_at",
    },
    Names {
        f: F_QD,
        get: &["qd", "velocity"],
        at: "qd_at",
        prev: "prev_qd",
        prev_at: "prev_qd_at",
        set_all: "set_all_qd",
        set_at: "set_qd_at",
    },
    Names {
        f: F_TAU,
        get: &["tau", "torque", "effort"],
        at: "tau_at",
        prev: "prev_tau",
        prev_at: "prev_tau_at",
        set_all: "set_all_tau",
        set_at: "set_tau_at",
    },
    Names {
        f: F_KP,
        get: &["kp"],
        at: "kp_at",
        prev: "prev_kp",
        prev_at: "prev_kp_at",
        set_all: "set_all_kp",
        set_at: "set_kp_at",
    },
    Names {
        f: F_KD,
        get: &["kd"],
        at: "kd_at",
        prev: "prev_kd",
        prev_at: "prev_kd_at",
        set_all: "set_all_kd",
        set_at: "set_kd_at",
    },
];

/// Field `f` at `depth` for each of `ix`, as a sequence; the first error otherwise.
fn row(
    lua: &Lua,
    s: &Snapshot,
    f: usize,
    depth: usize,
    ix: impl Iterator<Item = usize> + Clone,
) -> LuaResult<Result<Table, StateError>> {
    if let Some(e) = ix.clone().find_map(|i| s.at(f, i, depth).err()) {
        return Ok(Err(e));
    }
    lua.create_sequence_from(ix.filter_map(|i| s.at(f, i, depth).ok()))
        .map(Ok)
}

/// Stages `values` on `targets`, one each: every value and index is checked first, so an
/// error stages nothing.
fn stage_all(
    s: &mut Snapshot,
    f: usize,
    values: &Table,
    targets: impl ExactSizeIterator<Item = usize> + Clone,
    what: fn(usize) -> String,
) -> LuaResult<()> {
    let len = values.raw_len();
    if len != targets.len() {
        return Err(LuaError::runtime(format!(
            "{len} values for {}",
            what(targets.len())
        )));
    }
    if let Some(bad) = targets.clone().find(|&i| i >= s.n) {
        return Err(raise(s.out_of_range(bad)));
    }
    // DECISION: the values are read in one pass into a stack buffer of the 64 actuators a
    // mask holds; only a longer group (repeated indices) reads the table twice.
    let mut buf = [0.0_f32; 64];
    if len <= buf.len() {
        for (slot, v) in buf.iter_mut().zip(values.sequence_values::<f32>()) {
            *slot = v?;
        }
        for (&v, i) in buf.iter().zip(targets) {
            s.set_at(f, v, i).map_err(raise)?;
        }
        return Ok(());
    }
    for k in 1..=len {
        values.raw_get::<f32>(k)?;
    }
    for (k, i) in targets.enumerate() {
        s.set_at(f, values.raw_get::<f32>(k + 1)?, i)
            .map_err(raise)?;
    }
    Ok(())
}

/// A sequence of per-actuator options; a `None` leaves a nil hole.
fn holes<T: mlua::IntoLua + Copy>(
    lua: &Lua,
    values: impl ExactSizeIterator<Item = Option<T>>,
) -> LuaResult<Table> {
    let t = lua.create_table_with_capacity(values.len(), 0)?;
    for (k, v) in values.enumerate() {
        if let Some(v) = v {
            t.raw_set(k + 1, v)?;
        }
    }
    Ok(t)
}

/// `state`: History, faults, ages and telemetry of one tick, and the writes of one call.
// DECISION: actuator indices stay 0-based, as in Rust and the URDF, while the sequences
// returned are ordinary 1-based Lua tables: an actuator keeps one number in every language.
pub(crate) struct LState(pub(crate) Snapshot);

impl UserData for LState {
    fn add_methods<M: UserDataMethods<Self>>(m: &mut M) {
        for names in &FIELDS {
            let f = names.f;
            for &get in names.get {
                m.add_method(get, move |lua, this, ()| {
                    Ok(row(lua, &this.0, f, 0, 0..this.0.n)?.ok())
                });
            }
            m.add_method(names.at, move |_, this, i: usize| {
                this.0.at(f, i, 0).map_err(raise)
            });
            m.add_method(names.prev, move |lua, this, depth: usize| {
                row(lua, &this.0, f, depth, 0..this.0.n)?.map_err(raise)
            });
            m.add_method(names.prev_at, move |_, this, (i, depth): (usize, usize)| {
                this.0.at(f, i, depth).map_err(raise)
            });
            m.add_method_mut(names.set_all, move |_, this, values: Table| {
                let n = this.0.n;
                stage_all(&mut this.0, f, &values, 0..n, |n| format!("{n} actuators"))
            });
            m.add_method_mut(names.set_at, move |_, this, (v, i): (f32, usize)| {
                this.0.set_at(f, v, i).map_err(raise)
            });
        }
        m.add_method("faults", |_, this, ()| {
            Ok(this
                .0
                .faults(0)
                .ok()
                .map(|bits| LFaults { n: this.0.n, bits }))
        });
        m.add_method("prev_faults", |_, this, depth: usize| {
            this.0
                .faults(depth)
                .map(|bits| LFaults { n: this.0.n, bits })
                .map_err(raise)
        });
        m.add_method("stale_at", |_, this, i: usize| Ok(this.0.is_stale(i)));
        m.add_method("age_at", |_, this, i: usize| {
            this.0.get(&this.0.age, i).map_err(raise)
        });
        m.add_method("sample_age", |_, this, ()| Ok(this.0.sample_age));
        m.add_method("read_every", |_, this, i: usize| {
            this.0.get(&this.0.read_every, i).map_err(raise)
        });
        m.add_method("temperature_at", |_, this, i: usize| {
            this.0.get(&this.0.temperature, i).map_err(raise)
        });
        m.add_method("temperatures", |lua, this, ()| {
            holes(lua, this.0.temperature.iter().copied())
        });
        m.add_method("voltage_at", |_, this, i: usize| {
            this.0.get(&this.0.voltage, i).map_err(raise)
        });
        m.add_method("hardware_error_at", |_, this, i: usize| {
            this.0.get(&this.0.hardware_error, i).map_err(raise)
        });
        m.add_method("telemetry_age_at", |_, this, i: usize| {
            this.0.get(&this.0.telemetry_age, i).map_err(raise)
        });
        m.add_function("group", |_, (state, ix): (AnyUserData, Table)| {
            let indices = ix
                .sequence_values::<usize>()
                .collect::<LuaResult<Vec<_>>>()?;
            Ok(LGroup { state, indices })
        });
    }
}

/// `state:group(indices)`: the same accessors over chosen actuators.
pub(crate) struct LGroup {
    /// The state it views.
    state: AnyUserData,
    /// The actuators, in group order.
    indices: Vec<usize>,
}

impl UserData for LGroup {
    fn add_methods<M: UserDataMethods<Self>>(m: &mut M) {
        for names in &FIELDS {
            let f = names.f;
            for &get in names.get {
                m.add_method(get, move |lua, this, ()| {
                    let s = this.state.borrow::<LState>()?;
                    Ok(row(lua, &s.0, f, 0, this.indices.iter().copied())?.ok())
                });
            }
            m.add_method(names.prev, move |lua, this, depth: usize| {
                let s = this.state.borrow::<LState>()?;
                row(lua, &s.0, f, depth, this.indices.iter().copied())?.map_err(raise)
            });
            m.add_method(names.set_all, move |_, this, values: Table| {
                let mut s = this.state.borrow_mut::<LState>()?;
                stage_all(&mut s.0, f, &values, this.indices.iter().copied(), |n| {
                    format!("a group of {n}")
                })
            });
        }
        m.add_method("sample_age", |_, this, ()| {
            let s = this.state.borrow::<LState>()?;
            Ok(s.0.group_sample_age(&this.indices))
        });
        m.add_method("stale", |_, this, ()| {
            let s = this.state.borrow::<LState>()?;
            Ok(this.indices.iter().any(|&i| s.0.is_stale(i)))
        });
        m.add_method("temperatures", |lua, this, ()| {
            let s = this.state.borrow::<LState>()?;
            holes(
                lua,
                this.indices
                    .iter()
                    .map(|&i| s.0.temperature.get(i).copied().flatten()),
            )
        });
    }
}

/// `state:faults()`: one tick's fault bits; masks read as sequences of actuator indices.
pub(crate) struct LFaults {
    /// The actuator count.
    n: usize,
    /// The bits.
    bits: FaultBits,
}

impl UserData for LFaults {
    fn add_methods<M: UserDataMethods<Self>>(m: &mut M) {
        let masks = [
            "stale",
            "bus_error",
            "offline",
            "rejected",
            "joint_limit",
            "envelope",
        ];
        for (k, name) in masks.into_iter().enumerate() {
            m.add_method(name, move |lua, this, ()| {
                let bits = this.bits.masks.get(k).copied().unwrap_or(0);
                lua.create_sequence_from(indices(bits, this.n))
            });
        }
        let flags = [
            "overrun",
            "sensor_missing",
            "remote_timeout",
            "unauthorized",
        ];
        for (k, name) in flags.into_iter().enumerate() {
            m.add_method(name, move |_, this, ()| {
                Ok(this.bits.tick.get(k).copied().unwrap_or(false))
            });
        }
        m.add_method("any_actuator", |lua, this, ()| {
            lua.create_sequence_from(indices(this.bits.any_actuator(), this.n))
        });
        m.add_method("is_clear", |_, this, ()| Ok(this.bits.is_clear()));
    }
}

/// `time`: the clock of the tick in progress; durations in seconds.
#[derive(Default)]
pub(crate) struct LTime(pub(crate) TimeSnap);

impl UserData for LTime {
    fn add_methods<M: UserDataMethods<Self>>(m: &mut M) {
        m.add_method("elapsed", |_, this, ()| Ok(this.0.elapsed));
        m.add_method("dt", |_, this, ()| Ok(this.0.dt));
        m.add_method("tick", |_, this, ()| Ok(this.0.tick));
        m.add_method("overrun_count", |_, this, ()| Ok(this.0.overrun_count));
        m.add_method("last_overrun", |_, this, ()| Ok(this.0.last_overrun));
        m.add_method("accumulated_overrun", |_, this, ()| {
            Ok(this.0.accumulated_overrun)
        });
    }
}

/// `sensors`: the IMUs and microphones of the tick.
#[derive(Default)]
pub(crate) struct LSensors(pub(crate) SensorsSnap);

impl UserData for LSensors {
    fn add_methods<M: UserDataMethods<Self>>(m: &mut M) {
        m.add_method("imu", |_, this, index: usize| {
            Ok(this.0.imu(index).map(LImu))
        });
        m.add_function("mic", |_, (sensors, index): (AnyUserData, usize)| {
            let present = sensors.borrow::<Self>()?.0.mic(index).is_some();
            Ok(present.then_some(LMic { sensors, index }))
        });
    }
}

/// A reading on three axes: `x()`, `y()`, `z()`.
pub(crate) struct LAxes([f32; 3]);

impl UserData for LAxes {
    fn add_methods<M: UserDataMethods<Self>>(m: &mut M) {
        m.add_method("x", |_, this, ()| Ok(this.0[0]));
        m.add_method("y", |_, this, ()| Ok(this.0[1]));
        m.add_method("z", |_, this, ()| Ok(this.0[2]));
        m.add_meta_method(MetaMethod::ToString, |_, this, ()| {
            let [x, y, z] = this.0;
            Ok(format!("Axes({x}, {y}, {z})"))
        });
    }
}

/// `sensors:imu(i)`.
pub(crate) struct LImu(ImuSnap);

impl UserData for LImu {
    fn add_methods<M: UserDataMethods<Self>>(m: &mut M) {
        m.add_method("accel", |_, this, ()| Ok(this.0.accel.map(LAxes)));
        m.add_method("gyro", |_, this, ()| Ok(this.0.gyro.map(LAxes)));
        m.add_method("age", |_, this, ()| Ok(this.0.age));
        m.add_method("dropped", |_, this, ()| Ok(this.0.dropped));
    }
}

/// `sensors:mic(i)`.
pub(crate) struct LMic {
    /// The sensors it reads from.
    sensors: AnyUserData,
    /// Its slot.
    index: usize,
}

impl UserData for LMic {
    fn add_methods<M: UserDataMethods<Self>>(m: &mut M) {
        m.add_method("samples", |lua, this, ()| {
            let s = this.sensors.borrow::<LSensors>()?;
            let samples = s.0.mics.get(this.index).map_or(&[][..], |m| &m.samples);
            lua.create_sequence_from(samples.iter().copied())
        });
        m.add_method("rate_hz", |_, this, ()| {
            let s = this.sensors.borrow::<LSensors>()?;
            Ok(s.0.mics.get(this.index).map_or(0, |m| m.rate_hz))
        });
        m.add_method("dropped", |_, this, ()| {
            let s = this.sensors.borrow::<LSensors>()?;
            Ok(s.0.mics.get(this.index).map_or(0, |m| m.dropped))
        });
    }
}
