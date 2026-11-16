//! A Python file as a controller (feature `python`).
//!
//! ```ignore
//! kmr_api::robot!(XmBench);
//!
//! fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     Robot::new()
//!         .add_controller(EachTick, kmr_api::python!("python/swing.py")?)
//!         .run()?;
//!     Ok(())
//! }
//! ```
//!
//! ```python
//! # python/swing.py: no import, no class.
//! def each_tick(time, state):
//!     state.set_all_q([0.1 * time.elapsed()] * N)
//! ```
//!
//! # The script
//!
//! - `python!("path.py")` calls the function named after the phase it is added to:
//!   `each_tick`, `first`, `last`, `init`, `pre_init`, `post_init`. `python!("path.py", name)`
//!   calls `name`, checked at load. The path is relative to the crate's `Cargo.toml`.
//! - The function takes up to four arguments, matched by name, in any order: `time`, `state`
//!   (or `robot`), `sensors`, `ctx`. Declare only the ones you use. They mirror the Rust controller API, method for method; values are bare
//!   floats (rad, rad/s, N·m), durations floats in seconds, a Rust `Err` a raised
//!   `StateError`, a Rust `None` a Python `None`.
//! - `ctx` is the controller's own object, kept across ticks and reloads: set any attribute
//!   on it; one never set reads `None`.
//! - Already in the script's globals: `N`, `HISTORY_DEPTH`, `shutdown(reason)`,
//!   `emergency_stop(reason)`, `StateError`. The script's folder is on `sys.path`.
//!
//! # Reload
//!
//! The file is watched while the robot runs. An edit is compiled off the control loop and
//! swapped in between two ticks; module globals start over, `ctx` is kept. An edit that does
//! not compile, or lacks the function, is reported and the previous version keeps running.
//!
//! # Cost and safety
//!
//! - Each tick takes the GIL and runs the interpreter: that cost is the user's, kmr counts the
//!   overruns (SRS FR-10.3). Compiling a reload holds the GIL too, so the tick that meets it
//!   can overrun.
//! - An exception prints its traceback, applies nothing the call wrote, and shuts the robot
//!   down gracefully.
//! - A crash in a native extension leaves torque as it was: no software guard can prevent it
//!   (SRS NFR-2). Run new code on the no-hardware adapter first.

// The interpreter's heap and the snapshot buffers are the API's, never the core's.
#![allow(clippy::disallowed_types)]

mod view;

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::ffi::CString;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use kmr_core::{ControlFnWith, Filled, InlineWith, Insert};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};

use crate::controller::{IntoController, seal::Sealed};
use crate::ffi::snapshot::{Seen, phase_name};
use crate::{HISTORY_DEPTH, Schedule, Sensors, State, Time};
use view::{PySensors, PyState, PyTime};

/// Why a script could not be loaded.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The file could not be read.
    #[error("{path}: {source}")]
    Read {
        /// The script.
        path: String,
        /// Why.
        source: std::io::Error,
    },
    /// The file did not run: a syntax error, an exception at import.
    #[error("{path}: {message}")]
    Python {
        /// The script.
        path: String,
        /// The Python error.
        message: String,
    },
    /// The file has no such function.
    #[error("{path}: no function `{function}`")]
    NoFunction {
        /// The script.
        path: String,
        /// The missing name.
        function: String,
    },
}

/// The script's helpers, compiled once per script.
const PRELUDE: &str = r#"
import inspect as _inspect, os as _os, sys as _sys, types as _types

class Ctx(_types.SimpleNamespace):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return None

def load(source, path, name, extras):
    folder = _os.path.dirname(path)
    if folder not in _sys.path:
        _sys.path.insert(0, folder)
    module = _types.ModuleType(name)
    module.__file__ = path
    module.__dict__.update(extras)
    exec(compile(source, path, "exec"), module.__dict__)
    return module

NAMES = {"time": 0, "state": 1, "robot": 1, "sensors": 2, "ctx": 3}

def params(f):
    out = []
    for p in _inspect.signature(f).parameters.values():
        if p.kind in (p.VAR_POSITIONAL, p.VAR_KEYWORD):
            raise TypeError(f"{f.__name__}: *args and **kwargs are not taken")
        if p.default is not p.empty:
            continue
        if p.name not in NAMES:
            raise TypeError(
                f"{f.__name__}: parameter `{p.name}` is not one of time, state, sensors, ctx")
        out.append(NAMES[p.name])
    return out
"#;

/// How often the watcher looks at the file.
// DECISION: a 100 ms mtime poll on a std thread, not a file-watch crate: no new dependency,
// and an edit shows within a blink.
const POLL: Duration = Duration::from_millis(100);

/// Scripts loaded so far, for module names of their own.
static LOADS: AtomicUsize = AtomicUsize::new(0);

/// `shutdown(reason="")` in a script: the graceful stop of `shutdown!`.
#[pyfunction]
#[pyo3(signature = (reason = ""))]
fn shutdown(reason: &str) {
    eprintln!("kmr: python asked for a shutdown: {reason}");
    kmr_core::shutdown!("python: shutdown()");
}

/// `emergency_stop(reason="")` in a script: the immediate stop of `emergency_stop!`.
#[pyfunction]
#[pyo3(signature = (reason = ""))]
fn emergency_stop(reason: &str) {
    eprintln!("kmr: python asked for an emergency stop: {reason}");
    kmr_core::emergency_stop!("python: emergency_stop()");
}

/// What the loop thread and the watcher share.
struct Shared {
    /// The file.
    path: PathBuf,
    /// The path as messages show it.
    label: String,
    /// The actuator count, the script's `N`.
    n: usize,
    /// [`PRELUDE`].
    prelude: Py<PyModule>,
    /// A module the watcher compiled, not taken yet.
    fresh: Mutex<Option<Py<PyModule>>>,
    /// Whether `fresh` holds one: the loop's one atomic load a tick.
    ready: AtomicBool,
    /// Set when the controller is dropped: the watcher ends.
    stop: AtomicBool,
}

impl Shared {
    /// Runs `source` as a fresh module with the script globals.
    fn compile<'py>(&self, py: Python<'py>, source: &str) -> PyResult<Bound<'py, PyModule>> {
        let extras = PyDict::new(py);
        extras.set_item("N", self.n)?;
        extras.set_item("HISTORY_DEPTH", HISTORY_DEPTH)?;
        extras.set_item("shutdown", wrap_pyfunction!(shutdown, py)?)?;
        extras.set_item("emergency_stop", wrap_pyfunction!(emergency_stop, py)?)?;
        extras.set_item("StateError", view::state_error_type(py))?;
        let k = LOADS.fetch_add(1, Ordering::Relaxed);
        let module = self.prelude.bind(py).getattr("load")?.call1((
            source,
            self.path.to_string_lossy(),
            format!("kmr_script_{k}"),
            extras,
        ))?;
        Ok(module.cast_into::<PyModule>()?)
    }

    /// Compiles the file if it changed since `seen`, and leaves the module for the loop.
    fn poll(&self, seen: &mut Seen) {
        if !seen.changed(&self.path) {
            return;
        }
        let compiled = Python::attach(|py| match self.compile(py, &seen.source) {
            Ok(m) => Some(m.unbind()),
            Err(e) => {
                e.display(py);
                None
            }
        });
        match compiled {
            Some(module) => {
                if let Ok(mut fresh) = self.fresh.lock() {
                    *fresh = Some(module);
                    self.ready.store(true, Ordering::Release);
                }
            }
            None => eprintln!(
                "kmr: {}: the edit does not run, the previous version keeps running",
                self.label
            ),
        }
    }
}

/// Ends the watcher when dropped.
struct Watch(Arc<Shared>);

impl Drop for Watch {
    fn drop(&mut self) {
        self.0.stop.store(true, Ordering::Relaxed);
    }
}

/// A loaded Python script: what `python!` returns and
/// [`add_controller`](crate::Robot::add_controller) takes.
pub struct Script {
    /// The file and its watcher.
    watch: Watch,
    /// The module as loaded.
    module: Py<PyModule>,
    /// The function `python!` named; `None` for the phase's name.
    function: Option<&'static str>,
    /// `state`, `time`, `sensors`, `ctx`.
    objects: Objects,
}

/// The four arguments of a script's function, one set per controller.
struct Objects {
    /// `state`.
    state: Py<PyState>,
    /// `time`.
    time: Py<PyTime>,
    /// `sensors`.
    sensors: Py<PySensors>,
    /// `ctx`.
    ctx: Py<PyAny>,
}

impl core::fmt::Debug for Script {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Script")
            .field("path", &self.watch.0.label)
            .field("function", &self.function)
            .finish_non_exhaustive()
    }
}

impl Script {
    /// Loads `path`, relative to `dir`, for a robot of `n` actuators, and starts watching it.
    /// What `python!` expands to.
    #[doc(hidden)]
    pub fn __load(
        dir: &str,
        path: &str,
        function: Option<&'static str>,
        n: usize,
    ) -> Result<Self, Error> {
        let full = Path::new(dir).join(path);
        let label = path.to_owned();
        let source = std::fs::read_to_string(&full).map_err(|source| Error::Read {
            path: label.clone(),
            source,
        })?;
        let modified = std::fs::metadata(&full).and_then(|m| m.modified()).ok();
        let py_err = |e: PyErr| Error::Python {
            path: label.clone(),
            message: e.to_string(),
        };
        let (shared, module, objects) = Python::attach(|py| {
            let prelude = CString::new(PRELUDE).map_err(|_| Error::Python {
                path: label.clone(),
                message: "the prelude holds a NUL byte".into(),
            })?;
            let prelude = PyModule::from_code(py, &prelude, c"<kmr>", c"_kmr_prelude")
                .map_err(py_err)?
                .unbind();
            let shared = Shared {
                path: full,
                label: label.clone(),
                n,
                prelude,
                fresh: Mutex::new(None),
                ready: AtomicBool::new(false),
                stop: AtomicBool::new(false),
            };
            let module = shared.compile(py, &source).map_err(py_err)?;
            if let Some(name) = function
                && !module.getattr(name).is_ok_and(|f| f.is_callable())
            {
                return Err(Error::NoFunction {
                    path: label.clone(),
                    function: name.to_owned(),
                });
            }
            let objects = Objects {
                state: Py::new(py, PyState::new(n)).map_err(py_err)?,
                time: Py::new(py, PyTime::default()).map_err(py_err)?,
                sensors: Py::new(py, PySensors::default()).map_err(py_err)?,
                ctx: shared
                    .prelude
                    .bind(py)
                    .getattr("Ctx")
                    .and_then(|c| c.call0())
                    .map_err(py_err)?
                    .unbind(),
            };
            Ok((shared, module.unbind(), objects))
        })?;
        let shared = Arc::new(shared);
        let watcher = Arc::clone(&shared);
        let mut seen = Seen { modified, source };
        // DECISION: a watcher that cannot be spawned only loses the reload; the script runs.
        let _ = std::thread::Builder::new()
            .name("kmr-python-watch".into())
            .spawn(move || {
                while !watcher.stop.load(Ordering::Relaxed) {
                    std::thread::sleep(POLL);
                    watcher.poll(&mut seen);
                }
            });
        Ok(Self {
            watch: Watch(shared),
            module,
            function,
            objects,
        })
    }
}

/// Loads a Python file as a controller: `python!("python/clap.py")` calls the function named
/// after the phase, `python!("python/clap.py", move_on_clap)` calls `move_on_clap`.
///
/// The path is relative to the calling crate's `Cargo.toml`; `N` from `robot!` must be in
/// scope. Returns `Result<Script, python::Error>`. See [the module](crate::ffi::python).
#[macro_export]
macro_rules! python {
    ($path:literal $(,)?) => {
        $crate::ffi::python::Script::__load(env!("CARGO_MANIFEST_DIR"), $path, None, N)
    };
    ($path:literal, $function:ident $(,)?) => {
        $crate::ffi::python::Script::__load(
            env!("CARGO_MANIFEST_DIR"),
            $path,
            Some(stringify!($function)),
            N,
        )
    };
}

/// The resolved function and the arguments it takes, built once.
struct Call {
    /// The function.
    function: Py<PyAny>,
    /// `time`, `state`, `sensors`, `ctx`, as its parameters name them.
    args: Py<PyTuple>,
    /// Whether it takes `sensors`: they are copied only then.
    sensors: bool,
}

/// A script controller's state, owned by the robot: the context of its node.
#[doc(hidden)]
pub struct Runtime {
    /// The file and its watcher.
    watch: Watch,
    /// The function's name.
    name: String,
    /// `None` while the script lacks the function.
    call: Option<Call>,
    /// The function's arguments.
    objects: Objects,
    /// Whether the missing function was reported.
    told: bool,
}

impl Runtime {
    /// The call of `name` in `module`, or why there is none.
    fn resolve(&self, py: Python<'_>, module: &Bound<'_, PyModule>) -> Result<Call, String> {
        let function = module
            .getattr(self.name.as_str())
            .ok()
            .filter(|f| f.is_callable())
            .ok_or_else(|| format!("no function `{}`", self.name))?;
        let params = self
            .watch
            .0
            .prelude
            .bind(py)
            .getattr("params")
            .and_then(|p| p.call1((&function,)))
            .and_then(|n| n.extract::<Vec<usize>>())
            .map_err(|e| e.to_string())?;
        let all: [Bound<'_, PyAny>; 4] = [
            self.objects.time.bind(py).clone().into_any(),
            self.objects.state.bind(py).clone().into_any(),
            self.objects.sensors.bind(py).clone().into_any(),
            self.objects.ctx.bind(py).clone(),
        ];
        let picked: Vec<_> = params.iter().filter_map(|&k| all.get(k)).collect();
        let args = PyTuple::new(py, picked).map_err(|e| e.to_string())?;
        Ok(Call {
            function: function.unbind(),
            args: args.unbind(),
            sensors: params.contains(&2),
        })
    }

    /// Takes the module the watcher left, if any.
    fn reload(&mut self, py: Python<'_>) {
        let shared = &self.watch.0;
        if !shared.ready.swap(false, Ordering::AcqRel) {
            return;
        }
        let Some(module) = shared.fresh.lock().ok().and_then(|mut f| f.take()) else {
            return;
        };
        match self.resolve(py, module.bind(py)) {
            Ok(call) => {
                eprintln!("kmr: {}: reloaded", shared.label);
                self.call = Some(call);
                self.told = false;
            }
            Err(why) => eprintln!(
                "kmr: {}: {why}, the previous version keeps running",
                shared.label
            ),
        }
    }

    /// Runs the function once for this tick.
    fn tick<const N: usize>(
        &mut self,
        time: &Time<'_>,
        state: &mut State<'_, N>,
        sensors: &Sensors<'_>,
    ) {
        Python::attach(|py| {
            self.reload(py);
            let Some(call) = &self.call else {
                if !self.told {
                    eprintln!(
                        "kmr: {}: no function `{}`, the robot shuts down",
                        self.watch.0.label, self.name
                    );
                    self.told = true;
                }
                kmr_core::shutdown!("python: the script has no function for this phase");
                return;
            };
            let o = &self.objects;
            o.state.borrow_mut(py).fill(state);
            o.time.borrow_mut(py).fill(time);
            if call.sensors {
                o.sensors.borrow_mut(py).fill(sensors);
            }
            match call.function.bind(py).call1(call.args.bind(py)) {
                Ok(_) => o.state.borrow(py).apply(state),
                Err(e) => {
                    e.display(py);
                    eprintln!(
                        "kmr: {}: `{}` raised; nothing it wrote is applied, the robot shuts down",
                        self.watch.0.label, self.name
                    );
                    // DECISION: an exception is a graceful shutdown, not an emergency stop: the
                    // tick completes with the other controllers' writes, then torque goes off.
                    kmr_core::shutdown!("python: the script raised");
                }
            }
        });
    }
}

/// The node a script is stored as: wraps the core handles and runs the [`Runtime`].
#[doc(hidden)]
pub struct PyTick;

impl<const N: usize> ControlFnWith<Runtime, N> for PyTick {
    fn call(
        &self,
        time: &kmr_core::Time,
        state: &mut kmr_core::RobotState<N>,
        sensors: &kmr_core::Sensors,
        rt: &mut Runtime,
    ) {
        rt.tick(
            &Time::new(time),
            &mut State::new(state),
            &Sensors::new(sensors),
        );
    }
}

impl Script {
    /// The robot-owned state of this script on phase `S`.
    fn runtime<S>(self) -> Runtime {
        let name = self.function.map_or_else(phase_name::<S>, str::to_owned);
        let mut rt = Runtime {
            watch: self.watch,
            name,
            call: None,
            objects: self.objects,
            told: false,
        };
        Python::attach(|py| match rt.resolve(py, self.module.bind(py)) {
            Ok(call) => rt.call = Some(call),
            Err(why) => eprintln!(
                "kmr: {}: {why}: the robot will shut down on its first tick",
                rt.watch.0.label
            ),
        });
        rt
    }
}

impl<const N: usize> Sealed<N> for Script {}

impl<const N: usize, S: Schedule, C> IntoController<N, S, C> for Script
where
    C: Insert<S, InlineWith<PyTick, Runtime>>,
{
    type Output = C::Output;

    fn add_to<E>(
        self,
        schedule: S,
        robot: kmr_core::Robot<N, C, E>,
    ) -> kmr_core::Robot<N, C::Output, Filled> {
        robot.add_control_fn_with(schedule, self.runtime::<S>(), PyTick)
    }
}

#[cfg(test)]
mod tests;
