//! The core of the Architecture.
//!
//! This crate is the engine, meant to be used along side `kmr_api`.
//! The core is the main hardware abstraction machinery.
//! Its primary goal is to abstract away the control loop to allow the users
//! to focus on what matters: the math control.
//!
//! # What are its responsabilities ?
//!
//! 1) Abstraction
//!    It abstracts most of the require elements and concepts in robotics.
//!    Mainly the time management, the control loop, the controllers, the hardware,
//!    the state machinery and more !
//! 2) Hardware genericity
//!    Via the kmr_adapter layer, the core manipulates over hardware
//!    generically to allow the user the freedom to not care about hardware,
//!    the main source of friction in robotics.
//! 3) Control Scheduling and threads
//!    Provide internal machinery to handle the sequences of the user defined
//!    controllers and the spawned threads in a safe way.
//! 4) Guarantee begginer friendly development
//!    Allowing the user to write the most basic rust possible is a priority.
//!    The user shouldn't be required to master the language.
//! 5) Static dispatch guarantee and performance
//!    The architecture is designed to avoid heap allocation at all cost during
//!    runtime.
//!
//! ## What it is not
//!
//! It is not a general purpose OS. The usage of this library on robots other
//! than KM-RoBoTa's designs is not supported by default.
//! Using a different robotics model other than the furnished
//! ones is possible, but untested and not supported.
//!
//! The core itself is NOT the API but the internal machinery alone. Its
//! surface may change without notice: the stable, user-facing surface is
//! `kmr_api`.
//!
//! It is not a microcontroller software architecture. While limited no_std can
//! be used in the future, the overall architectural design is not meant to be
//! no_std.

#![forbid(clippy::disallowed_types)]
#![forbid(clippy::unwrap_used)]
#![allow(dead_code, unused_imports)] // note: only for dev
#![warn(missing_docs)]
// #![warn(clippy::missing_docs_in_private_items)]
#![warn(clippy::missing_safety_doc)]

pub(crate) mod clock;
pub(crate) mod config;
pub(crate) mod contracts;
pub(crate) mod controllers;
#[doc(hidden)]
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
pub use clock::Time;
pub use controllers::{ControlFn, ControlFnWith, Inline, InlineWith, ThreadFn, Threaded};
pub use robot::{HCons, HList, HNil, Robot};
pub use schedule::{
    Drive, EachTick, First, Init, Insert, Last, PostInit, PreInit, Schedule, Scheduled,
};
pub use sensors::Sensors;
pub use signal::{Signal, SignalKind, SignalMeta};
pub use state::{HISTORY_DEPTH, JOINTS, RobotState, State};
