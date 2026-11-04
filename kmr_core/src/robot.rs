//! Module that holds the Robot data structure and its implementation.
//!
//! [`Robot`] is meant to be a builder following the Builder Pattern.
//! It is the primary user interface.
//!
//! # Examples
//!
//! ```no_run
//! // `no_run`: `run()` enters the control loop and only returns on a signal.
//! use core::time::Duration;
//!
//! use kmr_core::{EachTick, Robot, RobotState, Sensors, Time};
//!
//! fn foo(_time: &Time, _state: &mut RobotState<4>, _sensors: &Sensors) {
//!     println!("hello world");
//! }
//!
//! Robot::new()
//!     // Settings
//!     .set_dt(Duration::from_millis(1))
//!     // Controllers
//!     .add_control_fn(EachTick, foo)
//!     .run()
//!     .expect("the robot has a controller");
//! ```

use core::convert::Infallible;
use core::marker::PhantomData;
use core::time::Duration;

pub use frunk_core::hlist::{HCons, HNil};

use crate::clock::Time;
use crate::config::user_config::RobotConfig;
use crate::controllers::{
    ControlFn, ControlFnWith, Controller, Inline, InlineWith, ThreadFn, Threaded,
};
use crate::error::RunError;
use crate::hal::{Drivers, HardwareError, Mask, MemberError, NoHardware};
use crate::limits::{Envelope, Limit, Policy};
use crate::schedule::{Drive, Insert, Schedule};
use crate::state::RobotState;
use crate::{Scheduled, Sensors, runtime};

/// The primary librairy API.
///
/// `N` is the actuator count, from the robot's descriptor (SRS FR-6.1). CS
/// means Core Scheduled. This is useful to differenciate between
/// the user specified schedule label (named `S` in the impl blocks) and the
/// generic representation within the core.
///
/// `E` is [`Empty`] until the first controller is added, [`Filled`] after:
/// [`Robot::run`] and [`Robot::run_on`] need [`HasController`], so a robot
/// without a controller does not build (SRS NFR-4).
///
/// ```compile_fail,E0277
/// // kmr: this robot has no controller
/// kmr_core::Robot::<4>::new().run().ok();
/// ```
pub struct Robot<const N: usize, CS = Scheduled, E = Empty> {
    pub(crate) config: RobotConfig<N>,
    pub(crate) controllers: CS,
    pub(crate) filled: PhantomData<E>,
}

/// Type-state of a [`Robot`] that has no controller yet.
pub struct Empty;

/// Type-state of a [`Robot`] that has at least one controller.
pub struct Filled;

/// The seal of [`HasController`].
mod seal {
    /// Implemented by [`super::Filled`] only.
    pub trait Sealed {}
}

impl seal::Sealed for Filled {}

/// The type-state of a [`Robot`] that may run: [`Filled`] only, sealed.
#[diagnostic::on_unimplemented(
    message = "kmr: this robot has no controller",
    label = "`run()` needs at least one controller",
    note = "add one with `.add_controller(EachTick, f)` before `.run()` (SRS NFR-4)"
)]
pub trait HasController: seal::Sealed {}

impl HasController for Filled {}

/// Hooks around the controllers of each tick, for kmr_api's gateway (SRS
/// FR-7.1): `inbound` after `read` and the external flags, before `First`;
/// `outbound` after the write and the clock bookkeeping, so this tick's
/// sample, its faults, flags and age are reachable. Static dispatch, called
/// on the tick: no heap, lock, wait or I/O.
#[doc(hidden)]
pub trait Stages<const N: usize> {
    /// Before `First`: may write Desired.
    fn inbound(&mut self, state: &mut RobotState<N>, time: &Time) {
        let _ = (state, time);
    }

    /// After the write: reads the tick.
    fn outbound(&mut self, state: &RobotState<N>, time: &Time) {
        let _ = (state, time);
    }
}

/// No stage: what [`Robot::run_on`] runs with.
impl<const N: usize> Stages<N> for () {}

impl<const N: usize> Robot<N, Scheduled, Empty> {
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
    /// use kmr_core::{EachTick, Robot, RobotState, Sensors, Time};
    ///
    /// fn hello_world(_time: &Time, _robot: &mut RobotState<4>, _sensors: &Sensors) {
    ///     println!("Hello World !");
    /// }
    ///
    /// Robot::new()
    ///     .add_control_fn(EachTick, hello_world)
    ///     .run()
    ///     .expect("the robot has a controller");
    /// ```
    ///
    /// More actuators than the write mask holds fail the build:
    ///
    /// ```compile_fail
    /// let _ = kmr_core::Robot::<65>::new();
    /// ```
    pub fn new() -> Self {
        let () = Mask::<N>::CHECK;
        Robot {
            config: RobotConfig::default(),
            controllers: Scheduled::new(),
            filled: PhantomData,
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
impl<const N: usize, CS, E> Robot<N, CS, E> {
    /// Sets the tick period. Outside [10 µs, 1 s], zero included, `run` and
    /// `run_on` refuse it before any I/O; never set, the run uses 1 ms with
    /// one warning (SRS FR-3.1).
    pub fn set_dt(mut self, dt: Duration) -> Self {
        self.config.dt = Some(dt);
        self
    }

    /// After a shutdown, holds the actuators for `duration` before torque off
    /// (SRS FR-4.1). Without it, a shutdown cuts torque at once; with
    /// `Duration::ZERO`, the hold command is sent, then torque is cut.
    pub fn set_hold(mut self, duration: Duration) -> Self {
        self.config.hold = Some(duration);
        self
    }

    /// Sets each actuator's limit and the policy of its breach (SRS FR-4.5).
    /// An actuator whose limit bounds anything needs a policy, or `run` and
    /// `run_on` refuse it before any I/O ([`ConfigError::NoPolicy`]).
    ///
    /// [`ConfigError::NoPolicy`]: crate::error::ConfigError::NoPolicy
    pub fn set_limits(mut self, limits: [Limit; N], policies: [Option<Policy>; N]) -> Self {
        self.config.limits.set_limits(limits, policies);
        self
    }

    /// Sets each actuator's envelope: a written value outside it is clamped
    /// and flagged (SRS FR-13.5).
    pub fn set_envelopes(mut self, envelopes: [Envelope; N]) -> Self {
        self.config.limits.set_envelopes(envelopes);
        self
    }

    /// Private generic method to insert any state.
    fn insert_controller<C, S>(self, schedule: S, controller: C) -> Robot<N, CS::Output, Filled>
    where
        C: Controller,
        S: Schedule,
        CS: Insert<S, C>,
    {
        let _ = schedule;
        Robot {
            config: self.config,
            controllers: self.controllers.insert(controller),
            filled: PhantomData,
        }
    }

    /// Adds a new plain user defined controller.
    ///
    /// In-crate callers pass a bare closure (inference fills the argument
    /// types from the `Fn` bound). `kmr_api` does NOT use this entry — it goes
    /// through [`Robot::add_control_fn`] with its own [`ControlFn`] wrapper so
    /// it can hide the core argument types behind api newtypes. Hence
    /// `pub(crate)`: outside the crate, `add_control_fn` is the single entry
    /// (it also takes a plain `fn`, through the blanket [`ControlFn`] impl).
    pub(crate) fn add_controller<S, F>(self, schedule: S, f: F) -> Robot<N, CS::Output, Filled>
    where
        CS: Insert<S, Inline<F>>,
        F: Fn(&Time, &mut RobotState<N>, &Sensors),
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
    pub fn add_control_fn<S, C>(self, schedule: S, controller: C) -> Robot<N, CS::Output, Filled>
    where
        CS: Insert<S, Inline<C>>,
        C: ControlFn<N>,
        S: Schedule,
    {
        self.insert_controller(schedule, Inline(controller))
    }

    /// Adds a new plain user defined controller with a persistant context
    /// managed by the user. `pub(crate)`, see [`Robot::add_controller`].
    pub(crate) fn add_controller_with<S, T, F>(
        self,
        schedule: S,
        ctx: T,
        f: F,
    ) -> Robot<N, CS::Output, Filled>
    where
        CS: Insert<S, InlineWith<F, T>>,
        F: Fn(&Time, &mut RobotState<N>, &Sensors, &mut T),
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
    ) -> Robot<N, CS::Output, Filled>
    where
        CS: Insert<S, InlineWith<C, T>>,
        C: ControlFnWith<T, N>,
        S: Schedule,
    {
        self.insert_controller(schedule, InlineWith(controller, ctx))
    }

    #[cfg(feature = "std")]
    /// Adds a new plain user defined threaded controller with a persistant
    /// context managed by the user.
    ///
    /// A threaded controller is not allowed to write the
    /// [`crate::state::RobotState`] — it gets only its context `T` and the clock.
    /// `pub(crate)`, see [`Robot::add_controller`].
    pub(crate) fn add_controller_as_thread<S, T, F>(
        self,
        schedule: S,
        ctx: T,
        f: F,
    ) -> Robot<N, CS::Output, Filled>
    where
        CS: Insert<S, Threaded<F, T>>,
        T: Sync + Send,
        F: Fn(&Time, &mut T),
        S: Schedule,
    {
        self.insert_controller(schedule, Threaded(f, ctx))
    }

    #[cfg(feature = "std")]
    /// Pre-wrapped `add_controller_as_thread` — the entry `kmr_api` uses.
    pub fn add_control_fn_as_thread<S, T, C>(
        self,
        schedule: S,
        ctx: T,
        controller: C,
    ) -> Robot<N, CS::Output, Filled>
    where
        CS: Insert<S, Threaded<C, T>>,
        T: Sync + Send,
        C: ThreadFn<T>,
        S: Schedule,
    {
        self.insert_controller(schedule, Threaded(controller, ctx))
    }

    /// The same robot, typed [`Filled`]; only called where `E` is.
    fn filled(self) -> Robot<N, CS, Filled> {
        Robot {
            config: self.config,
            controllers: self.controllers,
            filled: PhantomData,
        }
    }

    #[cfg(feature = "std")]
    /// Runs the robot on `hal`, its hardware: a list of members,
    /// `HCons { head: dxl, tail: HCons { head: cubemars, tail: HNil } }`.
    /// Pass `&mut adapter` as a member to keep it after the run.
    ///
    /// The `Drive` bound is the schedule-walk trait (its methods hidden):
    /// every schedule the builder produces satisfies it, so users never trip
    /// it. `kmr_api` forwards this same one bound from its
    /// own `run`.
    ///
    /// Returns once a signal ends the loop: `Ok` for a shutdown and for an
    /// emergency stop alike. Before any I/O: [`RunError::Config`] for a
    /// refused configuration, then [`RunError::Hal`] for a claim mismatch.
    /// After: [`RunError::Hal`] for a member's error from `check` or a
    /// bring-up call.
    pub fn run_on<D: Drivers<N>>(self, hal: D) -> Result<(), RunError<HardwareError<D::Error>>>
    where
        CS: Drive<N>,
        E: HasController,
    {
        self.run_on_staged(hal, ())
    }

    #[cfg(feature = "std")]
    /// [`Robot::run_on`], with `stages` around the controllers of each tick.
    #[doc(hidden)]
    pub fn run_on_staged<D: Drivers<N>, S: Stages<N>>(
        self,
        hal: D,
        stages: S,
    ) -> Result<(), RunError<HardwareError<D::Error>>>
    where
        CS: Drive<N>,
        E: HasController,
    {
        runtime::runtime(self.filled(), hal, stages).map(|_signal| ())
    }

    #[cfg(feature = "std")]
    /// Runs the robot on [`NoHardware`]: nothing is read or written. Only a
    /// [`RunError::Config`] can fail it. See [`Robot::run_on`].
    pub fn run(self) -> Result<(), RunError<Infallible>>
    where
        CS: Drive<N>,
        E: HasController,
    {
        let hal = HCons {
            head: NoHardware,
            tail: HNil,
        };
        match self.run_on(hal) {
            Ok(()) => Ok(()),
            Err(RunError::Config(config)) => Err(RunError::Config(config)),
            // `NoHardware` claims every actuator: no coverage gap.
            Err(RunError::Hal(HardwareError::Coverage(_))) => Ok(()),
            Err(RunError::Hal(HardwareError::Member(
                MemberError::Head { error: never, .. } | MemberError::Tail(never),
            ))) => match never {},
        }
    }
}

impl<const N: usize> Default for Robot<N, Scheduled, Empty> {
    fn default() -> Self {
        Self::new()
    }
}
