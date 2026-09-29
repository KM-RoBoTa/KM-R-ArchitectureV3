//! Module that holds the Robot data structure and its implementation.
//!
//! [`Robot`] is meant to be a builder following the Builder Pattern.
//! It is the primary user interface.
//!
//! # Examples
//!
//! ```no_run
//! // `no_run`: `run()` enters the control loop and only returns on a signal.
//! use std::time::Duration;
//!
//! use kmr_core::{EachTick, JOINTS, Robot, RobotState, Sensors, Time};
//!
//! fn foo(_time: &Time, _state: &mut RobotState<JOINTS>, _sensors: &Sensors) {
//!     println!("hello world");
//! }
//!
//! Robot::new()
//!     // Settings
//!     .set_dt(Duration::from_millis(1))
//!     // Controllers
//!     .add_controller(EachTick, foo)
//!     .run()
//!     .expect("the robot has a controller");
//! ```

use std::time::Duration;

pub use frunk::hlist::HList;
pub use frunk::{HCons, HNil};

use crate::clock::Time;
use crate::config::user_config::RobotConfig;
use crate::controllers::{
    ControlFn, ControlFnWith, Controller, Inline, InlineWith, ThreadFn, Threaded,
};
use crate::error::ApiError;
use crate::schedule::{Drive, Insert, Schedule};
use crate::state::RobotState;
use crate::{Scheduled, Sensors, runtime};

// todo: build time variable from model, not implemented yet
pub const JOINTS: usize = 4;

/// The primary librairy API.
///
/// CS means Core Scheduled. This is useful to differenciate between
/// the user specified schedule label (named `S` in the impl blocks) and the
/// generic representation within the core.
pub struct Robot<CS = Scheduled> {
    pub(crate) config: RobotConfig,
    pub(crate) controllers: CS,
}

impl Robot<Scheduled> {
    /// Returns a default Robot.
    ///
    /// This is the main entry point for the API User.
    ///
    /// # Usage
    ///
    /// Assuming the most minimal control possible,
    ///
    /// ```no_run
    /// // `no_run`: `run()` enters the control loop and only returns on a signal.
    /// use kmr_core::{EachTick, JOINTS, Robot, RobotState, Sensors, Time};
    ///
    /// fn hello_world(_time: &Time, _robot: &mut RobotState<JOINTS>, _sensors: &Sensors) {
    ///     println!("Hello World !");
    /// }
    ///
    /// Robot::new()
    ///     .add_controller(EachTick, hello_world)
    ///     .run()
    ///     .expect("the robot has a controller");
    /// ```
    pub fn new() -> Self {
        Robot {
            config: RobotConfig::default(),
            controllers: Scheduled::new(),
        }
    }
}

// The impl is meant to be the API wrapper.
//
// If you want to access the config to perform internal actions, use the
// fields directly as namespaces.
//
// E.g: To set the elapsed time, an action disallowed to the user, call
// directly `robot.config.time.set_elapsed(t)`.
impl<CS> Robot<CS> {
    /// Allows the user to set the value of [`Time::delta`].
    pub fn set_dt(mut self, dt: Duration) -> Self {
        self.config.time.set_delta(dt);
        self
    }

    /// Private generic method to insert any state.
    fn insert_controller<C, S>(self, schedule: S, controller: C) -> Robot<CS::Output>
    where
        C: Controller,
        S: Schedule,
        CS: Insert<S, C>,
    {
        let _ = schedule;
        Robot {
            config: self.config,
            controllers: self.controllers.insert(controller),
        }
    }

    /// Adds a new plain user defined controller.
    ///
    /// Direct-core callers pass a bare closure (inference fills the argument
    /// types from the `Fn` bound). `kmr_api` does NOT use this entry — it goes
    /// through [`Robot::add_control_fn`] with its own [`ControlFn`] wrapper so
    /// it can hide the core argument types behind api newtypes.
    pub fn add_controller<S, F>(self, schedule: S, f: F) -> Robot<CS::Output>
    where
        CS: Insert<S, Inline<F>>,
        F: Fn(&Time, &mut RobotState<JOINTS>, &Sensors),
        S: Schedule,
    {
        self.insert_controller(schedule, Inline(f))
    }

    /// Inserts a pre-wrapped inline controller — the entry `kmr_api` uses.
    ///
    /// It hands us a value that already implements [`ControlFn`] (its api→core
    /// translation wrapper). Because the node type is a plain named type (not
    /// an unnameable closure), the builder's type-state still threads through
    /// the return type, yet the public signature never names the core argument
    /// types the api is hiding.
    pub fn add_control_fn<S, C>(self, schedule: S, controller: C) -> Robot<CS::Output>
    where
        CS: Insert<S, Inline<C>>,
        C: ControlFn,
        S: Schedule,
    {
        self.insert_controller(schedule, Inline(controller))
    }

    /// Adds a new plain user defined controller with a persistant context
    /// managed by the user.
    pub fn add_controller_with<S, T, F>(self, schedule: S, ctx: T, f: F) -> Robot<CS::Output>
    where
        CS: Insert<S, InlineWith<F, T>>,
        F: Fn(&Time, &mut RobotState<JOINTS>, &Sensors, &mut T),
        S: Schedule,
    {
        self.insert_controller(schedule, InlineWith(f, ctx))
    }

    /// Pre-wrapped `add_controller_with` — the entry `kmr_api` uses. See
    /// [`Robot::add_control_fn`]; the `T` context is threaded through unchanged.
    pub fn add_control_fn_with<S, T, C>(
        self,
        schedule: S,
        ctx: T,
        controller: C,
    ) -> Robot<CS::Output>
    where
        CS: Insert<S, InlineWith<C, T>>,
        C: ControlFnWith<T>,
        S: Schedule,
    {
        self.insert_controller(schedule, InlineWith(controller, ctx))
    }

    /// Adds a new plain user defined threaded controller with a persistant
    /// context managed by the user.
    ///
    /// A threaded controller is not allowed to write the
    /// [`crate::state::RobotState`] — it gets only its context `T` and the clock.
    pub fn add_controller_as_thread<S, T, F>(self, schedule: S, ctx: T, f: F) -> Robot<CS::Output>
    where
        CS: Insert<S, Threaded<F, T>>,
        T: Sync + Send,
        F: Fn(&Time, &mut T),
        S: Schedule,
    {
        self.insert_controller(schedule, Threaded(f, ctx))
    }

    /// Pre-wrapped `add_controller_as_thread` — the entry `kmr_api` uses.
    pub fn add_control_fn_as_thread<S, T, C>(
        self,
        schedule: S,
        ctx: T,
        controller: C,
    ) -> Robot<CS::Output>
    where
        CS: Insert<S, Threaded<C, T>>,
        T: Sync + Send,
        C: ThreadFn<T>,
        S: Schedule,
    {
        self.insert_controller(schedule, Threaded(controller, ctx))
    }

    /// Runs the robot.
    ///
    /// The `Drive` bound is the schedule-walk trait: it's `pub` but
    /// `#[doc(hidden)]`, and every schedule the builder produces satisfies it,
    /// so users never trip it. `kmr_api` forwards this same one bound from its
    /// own `run`.
    pub fn run(self) -> Result<(), ApiError>
    where
        CS: Drive,
    {
        runtime::runtime(self);
        Ok(())
    }
}

impl Default for Robot<Scheduled> {
    fn default() -> Self {
        Self::new()
    }
}
