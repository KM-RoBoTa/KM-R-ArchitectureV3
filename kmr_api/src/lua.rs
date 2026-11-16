//! A Lua file as a controller (feature `lua`, Lua 5.4 vendored).
//!
//! ```ignore
//! kmr_api::robot!(XmBench);
//!
//! fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     Robot::new()
//!         .add_controller(EachTick, kmr_api::lua!("lua/swing.lua")?)
//!         .run()?;
//!     Ok(())
//! }
//! ```
//!
//! ```lua
//! -- lua/swing.lua: no require, no module table.
//! function each_tick(time, state)
//!     state:set_q_at(0.1 * time:elapsed(), 0)
//! end
//! ```
//!
//! # The script
//!
//! - `lua!("path.lua")` calls the global function named after the phase it is added to:
//!   `each_tick`, `first`, `last`, `init`, `pre_init`, `post_init`. `lua!("path.lua", name)`
//!   calls `name`, checked at load. The path is relative to the crate's `Cargo.toml`.
//! - The function takes up to four parameters, matched by name, in any order: `time`, `state`
//!   (or `robot`), `sensors`, `ctx`. Declare only the ones you use. The function must be a
//!   global defined in the script itself (`function each_tick(..)` or `each_tick =
//!   function(..)`), not a `local function`, and take no `...`.
//! - Methods are called with `:` (`state:q_at(0)`) and mirror the Rust controller API, method
//!   for method; values are bare numbers (rad, rad/s, N·m), durations numbers in seconds, a
//!   Rust `Err` a Lua error (catch it with `pcall`), a Rust `None` a `nil`.
//! - **Actuator indices are 0-based**, as in Rust and in the URDF: `state:q_at(0)` is the
//!   first actuator, `state:group({0, 1, 2})` the first three. An actuator has the same number
//!   in every language. The values a method returns are ordinary Lua sequences, so the first
//!   actuator's value is `state:q()[1]`. A per-actuator list that can lack a value
//!   (`temperatures()`) leaves `nil` holes: walk it with `for i = 1, N`.
//! - `set_all_q(values)` takes one value per actuator; the group's takes one per member. An
//!   error stages nothing.
//! - `ctx` is the controller's own table, kept across ticks and reloads; a field never set
//!   reads `nil`.
//! - Already in the script's globals: `N`, `HISTORY_DEPTH`, `shutdown(reason)`,
//!   `emergency_stop(reason)`, and `StateError(err)`, true when an error caught with `pcall`
//!   is a state error (an index past N, a depth past the History, a value not read yet). The
//!   standard library is Lua's safe set (no `debug`, no C modules); `require` finds modules
//!   next to the script.
//!
//! # Reload
//!
//! The file is watched while the robot runs. An edit is read off the control loop and compiled
//! on it, between two ticks, in the same Lua state; the script's globals start over, `ctx` is
//! kept. An edit that does not compile, raises at load, or lacks the function, is reported and
//! the previous version keeps running. Modules taken with `require` are not reloaded.
//!
//! # Cost and safety
//!
//! - Each tick runs the interpreter: that cost is the user's, kmr counts the overruns (SRS
//!   FR-10.3). The tick that meets an edit also compiles it.
//! - An uncaught error prints its traceback, applies nothing the call wrote, and shuts the
//!   robot down gracefully.
//! - Each `lua!` controller has its own Lua state.

// The interpreter's heap and the snapshot buffers are the API's, never the core's.
#![allow(clippy::disallowed_types)]

mod view;

use core::sync::atomic::{AtomicBool, Ordering};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use kmr_core::{ControlFnWith, Filled, InlineWith, Insert};
use mlua::{AnyUserData, Function, Lua, MultiValue, Table, Value};

use crate::controller::{IntoController, seal::Sealed};
use crate::snapshot::{Seen, SensorsSnap, Snapshot, TimeSnap, phase_name};
use crate::{HISTORY_DEPTH, Schedule, Sensors, State, Time};
use view::{LSensors, LState, LTime};

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
    /// The file did not run: a syntax error, an error at load.
    #[error("{path}: {message}")]
    Lua {
        /// The script.
        path: String,
        /// The Lua error.
        message: String,
    },
    /// The file has no such global function.
    #[error("{path}: no function `{function}`")]
    NoFunction {
        /// The script.
        path: String,
        /// The missing name.
        function: String,
    },
}

/// How often the watcher looks at the file.
// DECISION: the 100 ms mtime poll of `python!`, on a std thread: no new dependency.
const POLL: Duration = Duration::from_millis(100);

/// The parameter names a function may take, and the object each one gets.
const PARAMS: [(&str, usize); 5] = [
    ("time", 0),
    ("state", 1),
    ("robot", 1),
    ("sensors", 2),
    ("ctx", 3),
];

/// What the loop thread and the watcher share.
struct Shared {
    /// The file.
    path: PathBuf,
    /// The path as messages show it.
    label: String,
    /// Source text the watcher read, not taken yet.
    fresh: Mutex<Option<String>>,
    /// Whether `fresh` holds one: the loop's one atomic load a tick.
    ready: AtomicBool,
    /// Set when the controller is dropped: the watcher ends.
    stop: AtomicBool,
}

impl Shared {
    /// Reads the file if it changed since `seen`, and leaves the text for the loop.
    // DECISION: the watcher only reads; the loop compiles. Lua values belong to one state,
    // `ctx` must survive the reload, and a small script compiles in tens of microseconds.
    fn poll(&self, seen: &mut Seen) {
        if !seen.changed(&self.path) {
            return;
        }
        if let Ok(mut fresh) = self.fresh.lock() {
            *fresh = Some(seen.source.clone());
            self.ready.store(true, Ordering::Release);
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

/// The four arguments of a script's function, one set per controller.
struct Objects {
    /// `time`.
    time: AnyUserData,
    /// `state`.
    state: AnyUserData,
    /// `sensors`.
    sensors: AnyUserData,
    /// `ctx`.
    ctx: Table,
}

impl Objects {
    /// The object parameter `k` of [`PARAMS`] gets.
    fn get(&self, k: usize) -> Value {
        match k {
            0 => Value::UserData(self.time.clone()),
            1 => Value::UserData(self.state.clone()),
            2 => Value::UserData(self.sensors.clone()),
            _ => Value::Table(self.ctx.clone()),
        }
    }
}

/// A Lua state with the script globals and one compiled version of a script.
struct Vm {
    /// The interpreter.
    lua: Lua,
    /// The file and its watcher.
    watch: Watch,
    /// The actuator count, the script's `N`.
    n: usize,
    /// `time`, `state`, `sensors`, `ctx`.
    objects: Objects,
}

impl Vm {
    /// A fresh interpreter for a robot of `n` actuators, `require` looking next to `path`.
    fn new(path: PathBuf, label: String, n: usize) -> mlua::Result<Self> {
        // DECISION: one Lua state per `lua!`, with mlua's safe standard library: scripts never
        // share globals, and the state lives and dies with its controller.
        let lua = Lua::new();
        if let Some(folder) = path.parent() {
            let package: Table = lua.globals().get("package")?;
            let search: String = package.get("path")?;
            package.set(
                "path",
                format!("{}/?.lua;{search}", folder.to_string_lossy()),
            )?;
        }
        let objects = Objects {
            time: lua.create_userdata(LTime(TimeSnap::default()))?,
            state: lua.create_userdata(LState(Snapshot::new(n)))?,
            sensors: lua.create_userdata(LSensors(SensorsSnap::default()))?,
            ctx: lua.create_table()?,
        };
        let shared = Shared {
            path,
            label,
            fresh: Mutex::new(None),
            ready: AtomicBool::new(false),
            stop: AtomicBool::new(false),
        };
        Ok(Self {
            lua,
            watch: Watch(Arc::new(shared)),
            n,
            objects,
        })
    }

    /// The path as messages show it.
    fn label(&self) -> &str {
        &self.watch.0.label
    }

    /// Runs `source` in a fresh environment holding the script globals; returns it.
    fn compile(&self, source: &str) -> mlua::Result<Table> {
        let lua = &self.lua;
        let env = lua.create_table()?;
        env.set("N", self.n)?;
        env.set("HISTORY_DEPTH", HISTORY_DEPTH)?;
        env.set(
            "shutdown",
            lua.create_function(|_, reason: Option<String>| {
                eprintln!(
                    "kmr: lua asked for a shutdown: {}",
                    reason.unwrap_or_default()
                );
                kmr_core::shutdown!("lua: shutdown()");
                Ok(())
            })?,
        )?;
        env.set(
            "emergency_stop",
            lua.create_function(|_, reason: Option<String>| {
                eprintln!(
                    "kmr: lua asked for an emergency stop: {}",
                    reason.unwrap_or_default()
                );
                kmr_core::emergency_stop!("lua: emergency_stop()");
                Ok(())
            })?,
        )?;
        // DECISION: `StateError` is a predicate, `StateError(err)`: a Lua error raised from
        // Rust is an opaque value, so there is no class to compare against as in Python.
        env.set(
            "StateError",
            lua.create_function(|_, e: Value| {
                Ok(matches!(&e, Value::Error(err) if view::is_state_error(err)))
            })?,
        )?;
        let base = lua.create_table()?;
        base.set("__index", lua.globals())?;
        env.set_metatable(Some(base))?;
        lua.load(source)
            .set_name(format!("@{}", self.label()))
            .set_environment(env.clone())
            .exec()?;
        Ok(env)
    }

    /// The call of global `name` in `env`, compiled from `source`, or why there is none.
    fn resolve(&self, env: &Table, source: &str, name: &str) -> Result<Call, String> {
        let Ok(Value::Function(function)) = env.raw_get::<Value>(name) else {
            return Err(format!("no global function `{name}`"));
        };
        let info = function.info();
        if info.is_vararg {
            return Err(format!("{name}: `...` is not taken"));
        }
        let ours = info.source.as_deref() == Some(&format!("@{}", self.label()));
        let names = info
            .line_defined
            .filter(|_| ours)
            .and_then(|line| param_names(source, line, name))
            .filter(|p| p.len() == usize::from(info.num_params))
            .ok_or_else(|| {
                format!("{name}: its parameter list is not found where the script defines it")
            })?;
        let mut picked = Vec::with_capacity(names.len());
        for p in &names {
            let Some(&(_, k)) = PARAMS.iter().find(|(n, _)| n == p) else {
                return Err(format!(
                    "{name}: parameter `{p}` is not one of time, state, sensors, ctx"
                ));
            };
            picked.push(k);
        }
        let args: MultiValue = picked.iter().map(|&k| self.objects.get(k)).collect();
        // DECISION: the objects are bound to the function once, at resolve, so a tick calls
        // it with no argument to marshal.
        let bound = function.bind(args).map_err(|e| e.to_string())?;
        Ok(Call {
            function: bound,
            sensors: picked.contains(&2),
        })
    }
}

/// The parameter names of `function name(..)` or `name = function(..)` on 1-based `line`.
// DECISION: Lua's `debug.getlocal` would give the names, but mlua opens the `debug` library
// only in an `unsafe` constructor; the definition is read from the source instead, and the
// count checked against the one Lua reports.
fn param_names(source: &str, line: usize, name: &str) -> Option<Vec<String>> {
    let start = if line <= 1 {
        0
    } else {
        source.match_indices('\n').nth(line - 2)?.0 + 1
    };
    let rest = source.get(start..)?;
    let ident = |c: char| c.is_alphanumeric() || c == '_';
    let open = rest.match_indices(name).find_map(|(at, _)| {
        let before = rest.get(..at)?.chars().next_back();
        if before.is_some_and(ident) {
            return None;
        }
        let after = rest.get(at + name.len()..)?;
        if after.chars().next().is_some_and(ident) {
            return None;
        }
        let after = after.trim_start();
        let paren = match after.strip_prefix('=') {
            Some(value) => value.trim_start().strip_prefix("function")?.trim_start(),
            None => after,
        };
        paren.strip_prefix('(')
    })?;
    let list = open.get(..open.find(')')?)?;
    let names: Vec<String> = list
        .split(',')
        .map(str::trim)
        .filter(|p| !p.is_empty())
        .map(str::to_owned)
        .collect();
    names.iter().all(|p| p.chars().all(ident)).then_some(names)
}

/// A loaded Lua script: what `lua!` returns and
/// [`add_controller`](crate::Robot::add_controller) takes.
pub struct Script {
    /// The interpreter, the file and the objects.
    vm: Vm,
    /// The environment the script ran in.
    env: Table,
    /// Its text, for the parameter names.
    source: String,
    /// The function `lua!` named; `None` for the phase's name.
    function: Option<&'static str>,
}

impl core::fmt::Debug for Script {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Script")
            .field("path", &self.vm.label())
            .field("function", &self.function)
            .finish_non_exhaustive()
    }
}

impl Script {
    /// Loads `path`, relative to `dir`, for a robot of `n` actuators, and starts watching it.
    /// What `lua!` expands to.
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
        let lua_err = |e: mlua::Error| Error::Lua {
            path: label.clone(),
            message: e.to_string(),
        };
        let vm = Vm::new(full, label.clone(), n).map_err(lua_err)?;
        let env = vm.compile(&source).map_err(lua_err)?;
        if let Some(name) = function
            && !matches!(env.raw_get::<Value>(name), Ok(Value::Function(_)))
        {
            return Err(Error::NoFunction {
                path: label,
                function: name.to_owned(),
            });
        }
        let watcher = Arc::clone(&vm.watch.0);
        let mut seen = Seen {
            modified,
            source: source.clone(),
        };
        // DECISION: a watcher that cannot be spawned only loses the reload; the script runs.
        let _ = std::thread::Builder::new()
            .name("kmr-lua-watch".into())
            .spawn(move || {
                while !watcher.stop.load(Ordering::Relaxed) {
                    std::thread::sleep(POLL);
                    watcher.poll(&mut seen);
                }
            });
        Ok(Self {
            vm,
            env,
            source,
            function,
        })
    }
}

/// Loads a Lua file as a controller: `lua!("lua/clap.lua")` calls the function named after
/// the phase, `lua!("lua/clap.lua", move_on_clap)` calls `move_on_clap`.
///
/// The path is relative to the calling crate's `Cargo.toml`; `N` from `robot!` must be in
/// scope. Returns `Result<Script, lua::Error>`. See [the module](crate::lua).
#[macro_export]
macro_rules! lua {
    ($path:literal $(,)?) => {
        $crate::lua::Script::__load(env!("CARGO_MANIFEST_DIR"), $path, None, N)
    };
    ($path:literal, $function:ident $(,)?) => {
        $crate::lua::Script::__load(
            env!("CARGO_MANIFEST_DIR"),
            $path,
            Some(stringify!($function)),
            N,
        )
    };
}

/// The resolved function, its arguments bound.
struct Call {
    /// The function, called with no argument.
    function: Function,
    /// Whether it takes `sensors`: they are copied only then.
    sensors: bool,
}

/// A script controller's state, owned by the robot: the context of its node.
#[doc(hidden)]
pub struct Runtime {
    /// The interpreter, the file and the objects.
    vm: Vm,
    /// The function's name.
    name: String,
    /// `None` while the script lacks the function.
    call: Option<Call>,
    /// Whether the missing function was reported.
    told: bool,
}

impl Runtime {
    /// Compiles the text the watcher left, if any, and swaps it in when it holds the function.
    fn reload(&mut self) {
        let shared = &self.vm.watch.0;
        if !shared.ready.swap(false, Ordering::AcqRel) {
            return;
        }
        let Some(source) = shared.fresh.lock().ok().and_then(|mut f| f.take()) else {
            return;
        };
        let label = self.vm.label();
        let resolved = self
            .vm
            .compile(&source)
            .map_err(|e| e.to_string())
            .and_then(|env| self.vm.resolve(&env, &source, &self.name));
        match resolved {
            Ok(call) => {
                eprintln!("kmr: {label}: reloaded");
                self.call = Some(call);
                self.told = false;
            }
            Err(why) => {
                eprintln!("kmr: {label}: {why}\nkmr: {label}: the previous version keeps running")
            }
        }
    }

    /// Runs the function once for this tick.
    fn tick<const N: usize>(
        &mut self,
        time: &Time<'_>,
        state: &mut State<'_, N>,
        sensors: &Sensors<'_>,
    ) {
        self.reload();
        let Some(call) = &self.call else {
            if !self.told {
                eprintln!(
                    "kmr: {}: no function `{}`, the robot shuts down",
                    self.vm.label(),
                    self.name
                );
                self.told = true;
            }
            kmr_core::shutdown!("lua: the script has no function for this phase");
            return;
        };
        let o = &self.vm.objects;
        // A borrow fails only while a script holds the object, which no script can across ticks.
        let _ = o.state.borrow_mut_scoped::<LState, _>(|s| s.0.fill(state));
        let _ = o.time.borrow_mut_scoped::<LTime, _>(|t| t.0.fill(time));
        if call.sensors {
            let _ = o
                .sensors
                .borrow_mut_scoped::<LSensors, _>(|s| s.0.fill(sensors));
        }
        match call.function.call::<()>(()) {
            Ok(()) => {
                let _ = o.state.borrow_scoped::<LState, _>(|s| s.0.apply(state));
            }
            Err(e) => {
                eprintln!("{e}");
                eprintln!(
                    "kmr: {}: `{}` raised; nothing it wrote is applied, the robot shuts down",
                    self.vm.label(),
                    self.name
                );
                // DECISION: an error is a graceful shutdown, not an emergency stop, as in
                // `python!`: the tick completes with the other controllers' writes.
                kmr_core::shutdown!("lua: the script raised");
            }
        }
    }
}

/// The node a script is stored as: wraps the core handles and runs the [`Runtime`].
#[doc(hidden)]
pub struct LuaTick;

impl<const N: usize> ControlFnWith<Runtime, N> for LuaTick {
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
        let call = match self.vm.resolve(&self.env, &self.source, &name) {
            Ok(call) => Some(call),
            Err(why) => {
                eprintln!(
                    "kmr: {}: {why}: the robot will shut down on its first tick",
                    self.vm.label()
                );
                None
            }
        };
        Runtime {
            vm: self.vm,
            name,
            call,
            told: false,
        }
    }
}

impl<const N: usize> Sealed<N> for Script {}

impl<const N: usize, S: Schedule, C> IntoController<N, S, C> for Script
where
    C: Insert<S, InlineWith<LuaTick, Runtime>>,
{
    type Output = C::Output;

    fn add_to<E>(
        self,
        schedule: S,
        robot: kmr_core::Robot<N, C, E>,
    ) -> kmr_core::Robot<N, C::Output, Filled> {
        robot.add_control_fn_with(schedule, self.runtime::<S>(), LuaTick)
    }
}

#[cfg(test)]
mod tests;
