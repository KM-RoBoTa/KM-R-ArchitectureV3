//! The Python bridge (feature `python`): a Python function as a controller body.
//!
//! [`Bridge::load`] starts the embedded CPython, runs the script and finds the function; it
//! returns every failure as [`ForeignError::Load`], to be returned from `main` before `run()`
//! (FR-020 (SRS FR-6.4)). Each tick the function gets `step(read, write, time)` (contracts/
//! bridges.md):
//!
//! - `read.<field>[depth][i]` floats and `read.<flag>[depth]` int masks, 0-based, depth 0 the
//!   newest; `read.age[i]` int ticks, `None` for never read; `read.samples`; 0 = do not read.
//! - `write.set_<field>(i, v)`, 0-based; an index outside `0..N` raises `IndexError`.
//! - `time.tick`, `time.dt_ns`, `time.elapsed_ns` and the overrun counters, exact ints.
//! - It returns `0` or `None`, `1` (shutdown) or `2` (emergency stop). Any other value is
//!   `ForeignError::Code(-1)`, an exception `ForeignError::Python`; either applies nothing.
//!
//! The three objects persist across calls; the bridge refills them in place.
//!
//! # Cost and safety
//!
//! - Every call takes the GIL, boxes each value as a Python object and runs the interpreter,
//!   which allocates. That cost is the user's: kmr counts the overruns, it does not prevent them
//!   (FR-005 (SRS FR-10.3)).
//! - An exception is caught and returned. A crash or abort in a native extension leaves torque
//!   as it was, and no software guard can prevent it: run the script on the no-hardware adapter
//!   first (SRS FR-5.10, FR-025).
//! - `ctx` of [`call`](crate::call) is ignored: the script keeps its own data.
// The bridge's heap is the interpreter's and the user's, never kmr's per-call work (FR-019).
#![allow(clippy::disallowed_types)]

use core::sync::atomic::{AtomicUsize, Ordering};
use std::ffi::CString;

use pyo3::prelude::*;
use pyo3::types::{PyList, PyTuple};

use crate::call::{Foreign, ForeignError, Status};
use crate::script::{self, FIELDS, FLAGS, TIME};
use crate::view::{AGE_NONE, ReadView, TimeView, WriteBuf};

/// Builds the three script objects for N actuators and D slots from the name lists.
const PRELUDE: &str = r#"
from types import SimpleNamespace as _Ns

def _kmr_new(n, d, fields, flags, clock):
    read = _Ns(samples=0, age=[None] * n)
    for f in fields:
        setattr(read, f, [[0.0] * n for _ in range(d)])
    for f in flags:
        setattr(read, f, [0] * d)
    write = _Ns()
    def setter(f):
        def set_(i, v):
            if type(i) is not int or not 0 <= i < n:
                raise IndexError(f"set_{f}: actuator {i!r} outside 0..{n - 1}")
            getattr(write, f)[i] = float(v)
            setattr(write, "written_" + f, getattr(write, "written_" + f) | (1 << i))
        return set_
    for f in fields:
        setattr(write, f, [0.0] * n)
        setattr(write, "written_" + f, 0)
        setattr(write, "set_" + f, setter(f))
    time = _Ns(**{t: 0 for t in clock})
    return read, write, time
"#;

/// Bridges loaded so far, for a module name of their own.
static LOADS: AtomicUsize = AtomicUsize::new(0);

/// The persistent `read`, `write`, `time` objects, built for one N and D.
struct Objects {
    /// N and D they were built for.
    shape: (usize, usize),
    /// `read`.
    read: Py<PyAny>,
    /// `write`.
    write: Py<PyAny>,
    /// `time`.
    time: Py<PyAny>,
    /// `written_<field>`, in [`FIELDS`] order.
    written: Vec<String>,
}

/// A loaded Python script and its function, the context of a forwarding controller.
pub struct Bridge {
    /// The user's function.
    function: Py<PyAny>,
    /// `_kmr_new` of the prelude.
    new: Py<PyAny>,
    /// The field names the objects are built from.
    fields: &'static [&'static str],
    /// The flag names the objects are built from.
    flags: &'static [&'static str],
    /// Built on the first call, when N and D are known.
    objects: Option<Objects>,
}

impl core::fmt::Debug for Bridge {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Bridge").finish_non_exhaustive()
    }
}

/// A Python error as the error of a call: its type and message.
fn py_err(e: PyErr) -> ForeignError {
    ForeignError::Python(e.to_string())
}

impl Bridge {
    /// Starts the interpreter, runs the file at `path` and finds `function` in it.
    ///
    /// # Errors
    ///
    /// [`ForeignError::Load`] when the file cannot be read, does not run, or has no callable
    /// `function`.
    pub fn load(path: &str, function: &str) -> Result<Self, ForeignError> {
        Self::load_with(path, function, FIELDS, FLAGS)
    }

    /// [`load`](Self::load) with the script objects built from other name lists.
    pub(crate) fn load_with(
        path: &str,
        function: &str,
        fields: &'static [&'static str],
        flags: &'static [&'static str],
    ) -> Result<Self, ForeignError> {
        let load = |what: &str| ForeignError::Load(format!("{path}: {what}"));
        let code = std::fs::read_to_string(path).map_err(|e| load(&e.to_string()))?;
        let code = CString::new(code).map_err(|_| load("the file holds a NUL byte"))?;
        let file = CString::new(path).map_err(|_| load("the path holds a NUL byte"))?;
        // A module name already in `sys.modules` would reuse that module's globals.
        let k = LOADS.fetch_add(1, Ordering::Relaxed);
        let name = CString::new(format!("kmr_script_{k}")).map_err(|_| load("module name"))?;
        let prelude = CString::new(PRELUDE).map_err(|_| load("the prelude holds a NUL byte"))?;
        Python::attach(|py| {
            let py_load = |e: PyErr| load(&e.to_string());
            let new = PyModule::from_code(py, &prelude, c"<kmr_ffi>", c"_kmr_ffi")
                .and_then(|m| m.getattr("_kmr_new"))
                .map_err(py_load)?;
            let module = PyModule::from_code(py, &code, &file, &name).map_err(py_load)?;
            let f = module
                .getattr(function)
                .map_err(|_| load(&format!("no function `{function}`")))?;
            if !f.is_callable() {
                return Err(load(&format!("`{function}` is not callable")));
            }
            Ok(Self {
                function: f.unbind(),
                new: new.unbind(),
                fields,
                flags,
                objects: None,
            })
        })
    }

    /// The objects for N and D, built on first use or when the shape changes.
    fn objects(&mut self, py: Python<'_>, n: usize, d: usize) -> PyResult<&Objects> {
        if self.objects.as_ref().map(|o| o.shape) != Some((n, d)) {
            let made = self.new.bind(py).call1((
                n,
                d,
                self.fields.to_vec(),
                self.flags.to_vec(),
                TIME.to_vec(),
            ))?;
            let made = made.cast_into::<PyTuple>()?;
            self.objects = Some(Objects {
                shape: (n, d),
                read: made.get_item(0)?.unbind(),
                write: made.get_item(1)?.unbind(),
                time: made.get_item(2)?.unbind(),
                written: self.fields.iter().map(|f| format!("written_{f}")).collect(),
            });
        }
        self.objects
            .as_ref()
            .ok_or_else(|| pyo3::exceptions::PyRuntimeError::new_err("no script objects"))
    }
}

/// Copies `view` and `time` into the persistent objects and clears the written masks.
fn fill<const N: usize, const D: usize>(
    py: Python<'_>,
    o: &Objects,
    view: &ReadView<N, D>,
    time: &TimeView,
) -> PyResult<()> {
    let read = o.read.bind(py);
    read.setattr("samples", view.samples)?;
    let age = read.getattr("age")?.cast_into::<PyList>()?;
    for (i, a) in view.age.iter().enumerate() {
        if *a == AGE_NONE {
            age.set_item(i, py.None())?;
        } else {
            age.set_item(i, *a)?;
        }
    }
    for (k, slot) in view.slot.iter().enumerate() {
        script::slot_fields(slot, |name, values| {
            let row = read.getattr(name)?.get_item(k)?.cast_into::<PyList>()?;
            for (i, v) in values.iter().enumerate() {
                row.set_item(i, f64::from(*v))?;
            }
            Ok::<(), PyErr>(())
        })?;
        script::flag_masks(&slot.flags, |name, mask| {
            read.getattr(name)?.set_item(k, mask)
        })?;
    }
    let write = o.write.bind(py);
    for w in &o.written {
        write.setattr(w.as_str(), 0)?;
    }
    let t = o.time.bind(py);
    for (name, v) in TIME.iter().zip(script::time_values(time)) {
        t.setattr(*name, v)?;
    }
    Ok(())
}

/// Copies the flagged entries of the `write` object into `buf`.
fn drain<const N: usize>(py: Python<'_>, o: &Objects, buf: &mut WriteBuf<N>) -> PyResult<()> {
    let write = o.write.bind(py);
    let mut written = o.written.iter();
    script::buf_fields(buf, |name, values, mask| {
        let Some(w) = written.next() else {
            return Ok(());
        };
        let m = write.getattr(w.as_str())?.extract::<u64>()? & script::all(N);
        let list = write.getattr(name)?;
        for (i, v) in values.iter_mut().enumerate() {
            if m & (1 << i) != 0 {
                // Rounded to f32, the engine's unit (SC-002).
                *v = list.get_item(i)?.extract::<f64>()? as f32;
            }
        }
        *mask = m;
        Ok(())
    })
}

/// The status of the function's return value; `Code(-1)` for anything else.
fn status(ret: &Bound<'_, PyAny>) -> Result<Status, ForeignError> {
    if ret.is_none() {
        return Ok(Status::Continue);
    }
    ret.extract::<i32>()
        .ok()
        .and_then(Status::from_code)
        .ok_or(ForeignError::Code(-1))
}

impl<const N: usize, const D: usize> Foreign<N, D> for Bridge {
    fn invoke<C>(
        &mut self,
        read: &ReadView<N, D>,
        buf: &mut WriteBuf<N>,
        time: &TimeView,
        _ctx: Option<&mut C>,
    ) -> Result<Status, ForeignError> {
        Python::attach(|py| {
            let function = self.function.clone_ref(py);
            let o = self.objects(py, N, D).map_err(py_err)?;
            fill(py, o, read, time).map_err(py_err)?;
            let ret = function
                .bind(py)
                .call1((o.read.bind(py), o.write.bind(py), o.time.bind(py)))
                .map_err(py_err)?;
            let s = status(&ret)?;
            drain(py, o, buf).map_err(py_err)?;
            Ok(s)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::call::call;
    use crate::view::Written;

    /// Writes `body` to a fresh script file and returns its path.
    fn script(name: &str, body: &str) -> String {
        let path =
            std::env::temp_dir().join(format!("kmr_ffi_py_{name}_{}.py", std::process::id()));
        std::fs::write(&path, body).expect("temp dir is writable");
        path.to_string_lossy().into_owned()
    }

    /// One call of `bridge` on a view with one sample, `q = [1, 2, 3, 4]`.
    fn run(bridge: &mut Bridge) -> (Result<Status, ForeignError>, WriteBuf<4>) {
        let mut view = ReadView::<4, 2> {
            samples: 1,
            age: [0, 3, AGE_NONE, 0],
            ..ReadView::default()
        };
        if let Some(s) = view.slot.first_mut() {
            s.q = [1.0, 2.0, 3.0, 4.0];
            s.flags.stale = 0b10;
        }
        let time = TimeView {
            tick: (1 << 40) + 1,
            dt_ns: 1_000_000,
            ..TimeView::default()
        };
        let mut buf = WriteBuf::default();
        buf.written.qd = 1;
        let out = call(bridge, &view, &mut buf, &time, None::<&mut ()>);
        (out, buf)
    }

    #[test]
    fn a_step_reads_and_writes_through_the_objects() {
        let path = script(
            "swing",
            "def step(read, write, time):\n\
             \x20   assert read.samples == 1\n\
             \x20   assert read.age == [0, 3, None, 0]\n\
             \x20   assert read.stale == [2, 0]\n\
             \x20   assert time.tick == (1 << 40) + 1 and time.dt_ns == 1000000\n\
             \x20   for i in (0, 2):\n\
             \x20       write.set_q(i, read.q[0][i] + 0.1)\n\
             \x20   return 1\n",
        );
        let mut b = Bridge::load(&path, "step").expect("loads");
        for _ in 0..3 {
            let (out, buf) = run(&mut b);
            assert_eq!(out, Ok(Status::Shutdown));
            assert_eq!(buf.q, [1.1, 0.0, 3.1, 0.0]);
            assert_eq!(
                buf.written,
                Written {
                    q: 0b101,
                    ..Written::default()
                }
            );
        }
    }

    #[test]
    fn none_and_codes_map_to_statuses() {
        for (ret, want) in [
            ("None", Ok(Status::Continue)),
            ("0", Ok(Status::Continue)),
            ("2", Ok(Status::EmergencyStop)),
            ("3", Err(ForeignError::Code(-1))),
            ("'x'", Err(ForeignError::Code(-1))),
        ] {
            let path = script(
                "ret",
                &format!("def step(r, w, t):\n    w.set_qd(1, 5)\n    return {ret}\n"),
            );
            let mut b = Bridge::load(&path, "step").expect("loads");
            let (out, buf) = run(&mut b);
            assert_eq!(out, want, "return {ret}");
            if out.is_err() {
                assert_eq!(buf, WriteBuf::default());
            } else {
                assert_eq!(buf.written.qd, 0b10);
                assert_eq!(buf.qd[1], 5.0);
            }
        }
    }

    #[test]
    fn an_exception_applies_nothing() {
        for (name, body) in [
            ("raise", "    w.set_q(0, 1)\n    raise ValueError('boom')\n"),
            ("index", "    w.set_q(4, 1)\n"),
        ] {
            let path = script(name, &format!("def step(r, w, t):\n{body}"));
            let mut b = Bridge::load(&path, "step").expect("loads");
            let (out, buf) = run(&mut b);
            assert!(
                matches!(&out, Err(ForeignError::Python(t)) if t.contains("Error")),
                "{name}: {out:?}"
            );
            assert_eq!(buf, WriteBuf::default());
        }
    }

    #[test]
    fn two_scripts_keep_their_own_globals() {
        let one = script("one", "K = 1\ndef step(r, w, t):\n    return K\n");
        let two = script("two", "K = 2\ndef step(r, w, t):\n    return K\n");
        let mut a = Bridge::load(&one, "step").expect("loads");
        let mut b = Bridge::load(&two, "step").expect("loads");
        assert_eq!(run(&mut a).0, Ok(Status::Shutdown));
        assert_eq!(run(&mut b).0, Ok(Status::EmergencyStop));
    }

    #[test]
    fn load_failures_are_returned() {
        let missing = Bridge::load("/nonexistent/kmr.py", "step");
        assert!(matches!(missing, Err(ForeignError::Load(_))));
        let path = script("nofn", "x = 1\n");
        assert!(matches!(
            Bridge::load(&path, "step"),
            Err(ForeignError::Load(m)) if m.contains("no function `step`")
        ));
        assert!(matches!(
            Bridge::load(&path, "x"),
            Err(ForeignError::Load(m)) if m.contains("not callable")
        ));
        let path = script("syntax", "def step(:\n");
        assert!(matches!(
            Bridge::load(&path, "step"),
            Err(ForeignError::Load(_))
        ));
    }
}
