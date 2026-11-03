//! The Lua bridge (feature `lua`): a Lua 5.4 function as a controller body.
//!
//! The interpreter is vendored, so the bridge needs no system Lua. [`Bridge::load`] creates a
//! Lua state, runs the script and finds the global function; it returns every failure as
//! [`ForeignError::Load`], to be returned from `main` before `run()` (FR-020 (SRS FR-6.4)).
//! Each tick the function gets `step(read, write, time)` (contracts/bridges.md):
//!
//! - Tables are **1-based**: depth 0 is `read.<field>[1]`, actuator i is element `i + 1`, and
//!   `write.set_<field>(i, v)` takes `i` in `1..=N`; outside it is a Lua error.
//! - `read.<flag>[depth + 1]` are integer masks with **0-based** bits: bit i = actuator i.
//! - `read.age[i + 1]` is integer ticks, `-1` for never read (a table keeps no hole).
//! - `read.samples`, `time.tick`, `time.dt_ns`, ... are 64-bit integers, so they stay exact.
//! - It returns `0` or `nil`, `1` (shutdown) or `2` (emergency stop). Any other value is
//!   `ForeignError::Code(-1)`, a Lua error `ForeignError::Lua`; either applies nothing.
//!
//! The three tables persist across calls; the bridge refills them in place. With `mlua`'s
//! `send` feature the bridge is `Send`, so it can move into a threaded controller once that
//! hand-off exists (FR-004 (SRS FR-1.2)).
//!
//! # Cost and safety
//!
//! - Every call runs the VM, which allocates and collects garbage. That cost is the user's: kmr
//!   counts the overruns, it does not prevent them (FR-005 (SRS FR-10.3)).
//! - A Lua error is caught and returned. A crash or abort in a native module leaves torque as
//!   it was, and no software guard can prevent it: run the script on the no-hardware adapter
//!   first (SRS FR-5.10, FR-025).
//! - `ctx` of [`call`](crate::call) is ignored: the script keeps its own data.
// The bridge's heap is the VM's and the user's, never kmr's per-call work (FR-019).
#![allow(clippy::disallowed_types)]

use mlua::{Function, Lua, Table, Value};

use crate::call::{Foreign, ForeignError, Status};
use crate::script::{self, FIELDS, FLAGS, TIME};
use crate::view::{AGE_NONE, ReadView, TimeView, WriteBuf};

/// Builds the three script tables for N actuators and D slots from the name lists.
const PRELUDE: &str = r#"
local n, d, fields, flags, clock = ...
local read = { samples = 0, age = {} }
for i = 1, n do read.age[i] = -1 end
for _, f in ipairs(fields) do
  local t = {}
  for k = 1, d do
    local row = {}
    for i = 1, n do row[i] = 0.0 end
    t[k] = row
  end
  read[f] = t
end
for _, f in ipairs(flags) do
  local t = {}
  for k = 1, d do t[k] = 0 end
  read[f] = t
end
local write = {}
for _, f in ipairs(fields) do
  local vals = {}
  for i = 1, n do vals[i] = 0.0 end
  write[f] = vals
  write["written_" .. f] = 0
  write["set_" .. f] = function(i, v)
    if math.type(i) ~= "integer" or i < 1 or i > n then
      error(("set_%s: actuator %s outside 1..%d"):format(f, tostring(i), n), 2)
    end
    if type(v) ~= "number" then
      error(("set_%s: value %s is not a number"):format(f, tostring(v)), 2)
    end
    write[f][i] = v + 0.0
    write["written_" .. f] = write["written_" .. f] | (1 << (i - 1))
  end
end
local time = {}
for _, t in ipairs(clock) do time[t] = 0 end
return read, write, time
"#;

/// The persistent `read`, `write`, `time` tables, built for one N and D.
struct Tables {
    /// N and D they were built for.
    shape: (usize, usize),
    /// `read`.
    read: Table,
    /// `write`.
    write: Table,
    /// `time`.
    time: Table,
    /// `written_<field>`, in [`FIELDS`] order.
    written: Vec<String>,
}

/// A loaded Lua script and its function, the context of a forwarding controller.
pub struct Bridge {
    /// The Lua state; it owns the script.
    lua: Lua,
    /// The user's function.
    function: Function,
    /// The field names the tables are built from.
    fields: &'static [&'static str],
    /// The flag names the tables are built from.
    flags: &'static [&'static str],
    /// Built on the first call, when N and D are known.
    // DECISION: `load` takes no N or D (contracts/bridges.md), so the script objects are built
    // lazily; `tick_flags` is not exposed, the contract lists only the per-actuator flags.
    tables: Option<Tables>,
}

impl core::fmt::Debug for Bridge {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Bridge").finish_non_exhaustive()
    }
}

/// A Lua error as the error of a call.
fn lua_err(e: mlua::Error) -> ForeignError {
    ForeignError::Lua(e.to_string())
}

/// A `u64` as a Lua integer; past `i64::MAX` it saturates.
// DECISION: saturate, not wrap: a negative tick would read as time running backwards.
fn int(v: u64) -> i64 {
    i64::try_from(v).unwrap_or(i64::MAX)
}

impl Bridge {
    /// Creates a Lua state, runs the file at `path` and finds the global `function` in it.
    ///
    /// # Errors
    ///
    /// [`ForeignError::Load`] when the file cannot be read, does not run, or has no global
    /// function `function`.
    pub fn load(path: &str, function: &str) -> Result<Self, ForeignError> {
        Self::load_with(path, function, FIELDS, FLAGS)
    }

    /// [`load`](Self::load) with the script tables built from other name lists.
    pub(crate) fn load_with(
        path: &str,
        function: &str,
        fields: &'static [&'static str],
        flags: &'static [&'static str],
    ) -> Result<Self, ForeignError> {
        let load = |what: &str| ForeignError::Load(format!("{path}: {what}"));
        let code = std::fs::read_to_string(path).map_err(|e| load(&e.to_string()))?;
        let lua = Lua::new();
        lua.load(code)
            .set_name(format!("@{path}"))
            .exec()
            .map_err(|e| load(&e.to_string()))?;
        let function = match lua.globals().get::<Value>(function) {
            Ok(Value::Function(f)) => f,
            Ok(Value::Nil) => return Err(load(&format!("no function `{function}`"))),
            Ok(_) => return Err(load(&format!("`{function}` is not a function"))),
            Err(e) => return Err(load(&e.to_string())),
        };
        Ok(Self {
            lua,
            function,
            fields,
            flags,
            tables: None,
        })
    }

    /// The tables for N and D, built on first use or when the shape changes.
    fn tables(&mut self, n: usize, d: usize) -> mlua::Result<&Tables> {
        if self.tables.as_ref().map(|t| t.shape) != Some((n, d)) {
            let (read, write, time) = self.lua.load(PRELUDE).set_name("=kmr_ffi").call((
                n,
                d,
                self.lua.create_sequence_from(self.fields.iter().copied())?,
                self.lua.create_sequence_from(self.flags.iter().copied())?,
                self.lua.create_sequence_from(TIME.iter().copied())?,
            ))?;
            self.tables = Some(Tables {
                shape: (n, d),
                read,
                write,
                time,
                written: self.fields.iter().map(|f| format!("written_{f}")).collect(),
            });
        }
        self.tables
            .as_ref()
            .ok_or_else(|| mlua::Error::runtime("no script tables"))
    }
}

/// Copies `view` and `time` into the persistent tables and clears the written masks.
fn fill<const N: usize, const D: usize>(
    t: &Tables,
    view: &ReadView<N, D>,
    time: &TimeView,
) -> mlua::Result<()> {
    t.read.raw_set("samples", view.samples)?;
    let age: Table = t.read.raw_get("age")?;
    for (i, a) in view.age.iter().enumerate() {
        let a = if *a == AGE_NONE { -1 } else { i64::from(*a) };
        age.raw_set(i + 1, a)?;
    }
    for (k, slot) in view.slot.iter().enumerate() {
        script::slot_fields(slot, |name, values| {
            let rows: Table = t.read.raw_get(name)?;
            let row: Table = rows.raw_get(k + 1)?;
            for (i, v) in values.iter().enumerate() {
                row.raw_set(i + 1, f64::from(*v))?;
            }
            Ok::<(), mlua::Error>(())
        })?;
        script::flag_masks(&slot.flags, |name, mask| {
            let masks: Table = t.read.raw_get(name)?;
            // Bit 63 set reads as a negative integer; the bits are the same.
            masks.raw_set(k + 1, mask.cast_signed())
        })?;
    }
    for w in &t.written {
        t.write.raw_set(w.as_str(), 0)?;
    }
    for (name, v) in TIME.iter().zip(script::time_values(time)) {
        t.time.raw_set(*name, int(v))?;
    }
    Ok(())
}

/// Copies the flagged entries of the `write` table into `buf`.
fn drain<const N: usize>(t: &Tables, buf: &mut WriteBuf<N>) -> mlua::Result<()> {
    let mut written = t.written.iter();
    script::buf_fields(buf, |name, values, mask| {
        let Some(w) = written.next() else {
            return Ok(());
        };
        let m = t.write.raw_get::<i64>(w.as_str())?.cast_unsigned() & script::all(N);
        let list: Table = t.write.raw_get(name)?;
        for (i, v) in values.iter_mut().enumerate() {
            if m & (1 << i) != 0 {
                // Rounded to f32, the engine's unit (SC-002).
                *v = list.raw_get::<f64>(i + 1)? as f32;
            }
        }
        *mask = m;
        Ok(())
    })
}

/// The status of the function's return value; `Code(-1)` for anything else.
fn status(ret: &Value) -> Result<Status, ForeignError> {
    match ret {
        Value::Nil => Ok(Status::Continue),
        // DECISION: integers only, so `return 2.0` is `Code(-1)`, as a wrong Python type is.
        Value::Integer(rc) => i32::try_from(*rc)
            .ok()
            .and_then(Status::from_code)
            .ok_or(ForeignError::Code(-1)),
        _ => Err(ForeignError::Code(-1)),
    }
}

impl<const N: usize, const D: usize> Foreign<N, D> for Bridge {
    fn invoke<C>(
        &mut self,
        read: &ReadView<N, D>,
        buf: &mut WriteBuf<N>,
        time: &TimeView,
        _ctx: Option<&mut C>,
    ) -> Result<Status, ForeignError> {
        let function = self.function.clone();
        let t = self.tables(N, D).map_err(lua_err)?;
        fill(t, read, time).map_err(lua_err)?;
        let ret: Value = function
            .call((&t.read, &t.write, &t.time))
            .map_err(lua_err)?;
        let s = status(&ret)?;
        drain(t, buf).map_err(lua_err)?;
        Ok(s)
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
            std::env::temp_dir().join(format!("kmr_ffi_lua_{name}_{}.lua", std::process::id()));
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
    fn a_step_reads_and_writes_one_based_tables() {
        let path = script(
            "swing",
            "function step(read, write, time)\n\
             \x20 assert(read.samples == 1)\n\
             \x20 assert(read.age[1] == 0 and read.age[2] == 3 and read.age[3] == -1)\n\
             \x20 assert(read.stale[1] == 2 and read.stale[2] == 0)\n\
             \x20 assert(math.type(time.tick) == 'integer' and time.tick == (1 << 40) + 1)\n\
             \x20 assert(read.q[1][4] == 4.0)\n\
             \x20 for _, i in ipairs({1, 3}) do write.set_q(i, read.q[1][i] + 0.1) end\n\
             \x20 return 1\n\
             end\n",
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
    fn nil_and_codes_map_to_statuses() {
        for (ret, want) in [
            ("nil", Ok(Status::Continue)),
            ("0", Ok(Status::Continue)),
            ("2", Ok(Status::EmergencyStop)),
            ("3", Err(ForeignError::Code(-1))),
            ("2.0", Err(ForeignError::Code(-1))),
            ("'x'", Err(ForeignError::Code(-1))),
        ] {
            let path = script(
                "ret",
                &format!("function step(r, w, t)\n  w.set_qd(2, 5)\n  return {ret}\nend\n"),
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
    fn a_lua_error_applies_nothing() {
        for (name, body) in [
            ("error", "  w.set_q(1, 1)\n  error('boom')\n"),
            ("index", "  w.set_q(5, 1)\n"),
            ("zero", "  w.set_q(0, 1)\n"),
        ] {
            let path = script(name, &format!("function step(r, w, t)\n{body}end\n"));
            let mut b = Bridge::load(&path, "step").expect("loads");
            let (out, buf) = run(&mut b);
            assert!(matches!(&out, Err(ForeignError::Lua(_))), "{name}: {out:?}");
            assert_eq!(buf, WriteBuf::default());
        }
    }

    #[test]
    fn load_failures_are_returned() {
        let missing = Bridge::load("/nonexistent/kmr.lua", "step");
        assert!(matches!(missing, Err(ForeignError::Load(_))));
        let path = script("nofn", "x = 1\n");
        assert!(matches!(
            Bridge::load(&path, "step"),
            Err(ForeignError::Load(m)) if m.contains("no function `step`")
        ));
        assert!(matches!(
            Bridge::load(&path, "x"),
            Err(ForeignError::Load(m)) if m.contains("not a function")
        ));
        let path = script("syntax", "function step(\n");
        assert!(matches!(
            Bridge::load(&path, "step"),
            Err(ForeignError::Load(_))
        ));
    }

    #[test]
    fn the_bridge_can_move_to_another_thread() {
        fn is_send<T: Send>() {}
        is_send::<Bridge>();
    }
}
