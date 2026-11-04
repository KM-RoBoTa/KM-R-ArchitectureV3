//! The module that specified the behaviors of the user defined controllers.
//!
//! There's 3 types of controllers supported at the moment in this crate.
//! - [`Inline`]: Non-threaded controllers.
//! - [`InlineWith`]: Non-threaded controller with a persistent context `T`.
//! - [`Threaded`]: Threaded controllers with a persistent thread-safe context
//!   `T`.
//!
//! Each have their own signature of only 2 families
//! - The plain function family
//! - The "With" family, which supports an user defined `T`. This generic
//!   isn't used within the architecture, but is a convinience to allow the
//!   passthrough of custom data, tracked by the user.
//!
//! - [ ] `T` implements an API trait that standardize the usage, allowing
//!       for some automated updates. E.g: T::incr(mut &selfby: u32);
use crate::schedule::RunPhase;
use crate::{Sensors, clock::Time, state::RobotState};

/// The per-tick environment handed to every controller: read-only [`Time`] and
/// [`Sensors`], plus exclusive (`&mut`) access to the robot [`RobotState`].
/// Bundled into one type so a new field never ripples through every controller
/// signature or [`RunPhase`] impl.
// `pub` only so the `pub Drive` trait can name it in its method signatures; the
// fields and the constructor stay `pub(crate)`, and the type is not re-exported
// at the crate root, so downstream code can neither name, read nor build one.
// `#[doc(hidden)]` keeps it out of the docs.
#[doc(hidden)]
pub struct Env<'a, const N: usize> {
    pub(crate) time: &'a Time,
    pub(crate) state: &'a mut RobotState<N>,
    pub(crate) sensors: &'a Sensors,
}

impl<'a, const N: usize> Env<'a, N> {
    /// Bundles one phase's borrows.
    pub(crate) fn new(time: &'a Time, state: &'a mut RobotState<N>, sensors: &'a Sensors) -> Self {
        Self {
            time,
            state,
            sensors,
        }
    }
}

/// The core's call convention for an inline controller — the ONE signature the
/// runtime invokes. Blanket-implemented for every closure of the plain
/// `Fn(&Time, &mut RobotState, &Sensors)` shape, so direct-core callers keep
/// passing bare closures. `kmr_api` implements it on its OWN named wrapper
/// instead: that lets it translate api types → core types at the boundary
/// WITHOUT the core ever naming the api's closure signature, and keeps the
/// stored node type nameable (a closure type is not) so the builder's
/// type-state still threads through.
pub trait ControlFn<const N: usize> {
    /// Runs the controller once.
    fn call(&self, time: &Time, state: &mut RobotState<N>, sensors: &Sensors);
}

impl<const N: usize, F: Fn(&Time, &mut RobotState<N>, &Sensors)> ControlFn<N> for F {
    fn call(&self, time: &Time, state: &mut RobotState<N>, sensors: &Sensors) {
        self(time, state, sensors)
    }
}

/// Same as [`ControlFn`], for a controller carrying persistent context `T`
/// (added via `add_controller_with`). Blanket-implemented for the core-native
/// `Fn(&Time, &mut RobotState, &Sensors, &mut T)`; `kmr_api` implements it on
/// its own wrapper so it can present the api argument order/types instead.
pub trait ControlFnWith<T, const N: usize> {
    /// Runs the controller once with its context.
    fn call(&self, time: &Time, state: &mut RobotState<N>, sensors: &Sensors, ctx: &mut T);
}

impl<T, const N: usize, F: Fn(&Time, &mut RobotState<N>, &Sensors, &mut T)> ControlFnWith<T, N>
    for F
{
    fn call(&self, time: &Time, state: &mut RobotState<N>, sensors: &Sensors, ctx: &mut T) {
        self(time, state, sensors, ctx)
    }
}

/// Call convention for a THREADED controller: it gets its own context `T` and
/// the clock, but no robot state (state stays single-writer on the main loop).
/// Blanket-implemented for `Fn(&Time, &mut T)`; `kmr_api` wraps it to present
/// `Fn(&mut T, &Time)`.
pub trait ThreadFn<T> {
    /// Runs the threaded controller once with its context.
    fn call(&self, time: &Time, ctx: &mut T);
}

impl<T, F: Fn(&Time, &mut T)> ThreadFn<T> for F {
    fn call(&self, time: &Time, ctx: &mut T) {
        self(time, ctx)
    }
}

/// Marker trait for Controller wrappers.
///
/// It's only purpose is to allow the generilization of the controller
/// insertion logic. See [`crate::robot::Robot::insert_controller`]
pub(crate) trait Controller {}
/// A controller that runs inline on the main tick loop.
pub struct Inline<F>(pub(crate) F);
/// A main-loop controller that also owns persistent context `T` across ticks.
pub struct InlineWith<F, T>(pub(crate) F, pub(crate) T);
/// A controller that runs on its OWN dedicated thread. No robot `State`.
pub struct Threaded<F, T: Sync + Send>(pub(crate) F, pub(crate) T);

impl<F> Controller for Inline<F> {}
impl<F, T> Controller for InlineWith<F, T> {}
impl<F, T: Sync + Send> Controller for Threaded<F, T> {}

// Schedule implementation — the leaf of the `RunPhase` walk. Each kind knows how
// to call its own `F`; the context `T` (when present) comes out of the node.

impl<const N: usize, F: ControlFn<N>> RunPhase<N> for Inline<F> {
    fn run_phase(&mut self, env: &mut Env<'_, N>) {
        self.0.call(env.time, env.state, env.sensors);
    }
}
impl<const N: usize, F: ControlFnWith<T, N>, T> RunPhase<N> for InlineWith<F, T> {
    fn run_phase(&mut self, env: &mut Env<'_, N>) {
        self.0.call(env.time, env.state, env.sensors, &mut self.1);
    }
}
// A threaded controller runs on its OWN thread (spawned before the loop), so it
// does nothing on the inline tick walk.
impl<const N: usize, F, T: Sync + Send> RunPhase<N> for Threaded<F, T> {
    fn run_phase(&mut self, _env: &mut Env<'_, N>) {}
}
