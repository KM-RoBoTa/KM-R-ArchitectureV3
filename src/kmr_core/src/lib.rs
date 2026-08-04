#![forbid(clippy::disallowed_types)]
#![forbid(clippy::unwrap_used)]
#![allow(dead_code, unused_imports)] // note: only for dev
pub(crate) mod clock;
pub(crate) mod config;
pub(crate) mod contracts;
pub(crate) mod controllers;
pub mod error;
pub(crate) mod runtime;
pub(crate) mod schedule;
pub(crate) mod sealed;
pub(crate) mod sensors;
pub(crate) mod signal;
pub mod state;

mod robot;

// API re-exposure. `kmr_api` is a separate crate, so every type it names must be
// reachable here: the HList shape, the controller node types (fields sealed, so
// naming one is harmless), the signal surface, plus `Sensors`/`RobotState` and
// the `state`/`error` module paths.
pub use robot::{HCons, HList, HNil, Robot};
pub use signal::{Signal, SignalKind, SignalMeta};
pub use controllers::{Inline, InlineWith, Threaded};
pub use schedule::{First, Init, Last, PostInit, PreInit, Schedule, StateTransition, EachTick};
pub use sensors::Sensors;
pub use state::RobotState;
