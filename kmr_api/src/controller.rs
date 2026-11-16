//! What [`Robot::add_controller`](crate::Robot::add_controller) takes: a plain controller,
//! [`with`], [`threaded`], [`or_when`](OrWhen::or_when), or a tuple of them (SRS FR-1.1).

use kmr_core::{ControlFn, ControlFnWith, Filled, Inline, InlineWith, Insert, ThreadFn, Threaded};

use crate::fallback::Guarded;
use crate::{Faults, Schedule, Sensors, State, Time};

// The core calls the stored node with borrowed core handles; each wrapper borrows them into
// the api newtypes for the call. A named type, so the builder's type-state stays nameable.
/// Api→core wrapper of an inline controller. Opaque: users can neither forge nor read one.
#[doc(hidden)]
pub struct ApiInline<F>(pub(crate) F);

impl<const N: usize, F> ControlFn<N> for ApiInline<F>
where
    F: Fn(&Time, &mut State<'_, N>, &Sensors),
{
    fn call(
        &self,
        time: &kmr_core::Time,
        state: &mut kmr_core::RobotState<N>,
        sensors: &kmr_core::Sensors,
    ) {
        (self.0)(
            &Time::new(time),
            &mut State::new(state),
            &Sensors::new(sensors),
        );
    }
}

/// [`ApiInline`] for a controller with a context; the api passes `ctx` first.
#[doc(hidden)]
pub struct ApiInlineWith<F>(F);

impl<const N: usize, F, T> ControlFnWith<T, N> for ApiInlineWith<F>
where
    F: Fn(&mut T, &Time, &mut State<'_, N>, &Sensors),
{
    fn call(
        &self,
        time: &kmr_core::Time,
        state: &mut kmr_core::RobotState<N>,
        sensors: &kmr_core::Sensors,
        ctx: &mut T,
    ) {
        (self.0)(
            ctx,
            &Time::new(time),
            &mut State::new(state),
            &Sensors::new(sensors),
        );
    }
}

/// [`ApiInline`] for a threaded controller: its context and the clock, no state.
#[doc(hidden)]
pub struct ApiThreaded<F>(F);

impl<F, T> ThreadFn<T> for ApiThreaded<F>
where
    F: Fn(&mut T, &Time),
{
    fn call(&self, time: &kmr_core::Time, ctx: &mut T) {
        (self.0)(ctx, &Time::new(time));
    }
}

/// A controller and the context it is lent each tick. Built by [`with`].
pub struct With<T, F> {
    ctx: T,
    f: F,
}

/// A controller with its own thread. Built by [`threaded`].
pub struct InThread<T, F> {
    ctx: T,
    f: F,
}

/// `f` with its own data: the robot owns `ctx` for the whole run and lends it to `f` as
/// `&mut T` each call, so `f` is `Fn(&mut T, &Time, &mut State, &Sensors)`, no lock needed.
///
/// ```ignore
/// fn count(n: &mut u32, _t: &Time, _s: &mut State, _se: &Sensors) { *n += 1; }
/// robot.add_controller(EachTick, with(0_u32, count))
/// ```
pub fn with<T, F>(ctx: T, f: F) -> With<T, F> {
    With { ctx, f }
}

/// `f` on a thread of its own, for work with its own rhythm or that blocks (a gamepad, a
/// link, a planner). `f` is `Fn(&mut T, &Time)`: no robot state, so Desired keeps one writer;
/// a main-loop controller reads what the thread left in `ctx`. Stored, not spawned yet.
pub fn threaded<T, F>(ctx: T, f: F) -> InThread<T, F> {
    InThread { ctx, f }
}

/// A plain controller with a fallback.
pub trait OrWhen<const N: usize>: Fn(&Time, &mut State<'_, N>, &Sensors) + Sized {
    /// Each tick, `when` reads the tick's [`Faults`] and exactly one of the two runs,
    /// `fallback` when `when` is true (SRS FR-13.5). Build `when` from
    /// [`stale_in`](crate::stale_in), [`remote_timeout`](crate::remote_timeout) and
    /// [`any_of`](crate::any_of), or any `Fn(&Faults<N>) -> bool`. No hold policy: a fallback
    /// that writes nothing sends nothing (FR-050).
    fn or_when<P, G>(self, when: P, fallback: G) -> Guarded<Self, G, P>
    where
        P: Fn(&Faults<N>) -> bool,
        G: Fn(&Time, &mut State<'_, N>, &Sensors);
}

impl<const N: usize, F> OrWhen<N> for F
where
    F: Fn(&Time, &mut State<'_, N>, &Sensors),
{
    fn or_when<P, G>(self, when: P, fallback: G) -> Guarded<Self, G, P>
    where
        P: Fn(&Faults<N>) -> bool,
        G: Fn(&Time, &mut State<'_, N>, &Sensors),
    {
        Guarded {
            controller: self,
            fallback,
            when,
        }
    }
}

mod seal {
    /// Implemented for exactly the types that are controllers.
    pub trait Sealed<const N: usize> {}
}

/// Something [`Robot::add_controller`](crate::Robot::add_controller) adds to schedule `S` of
/// the controller set `C`, growing it to `Output`.
#[doc(hidden)]
#[diagnostic::on_unimplemented(
    message = "kmr: this is not a controller",
    label = "not a controller for this robot",
    note = "a controller is `Fn(&Time, &mut State, &Sensors)`, `with(ctx, f)`, \
            `f.or_when(when, fallback)`, `threaded(ctx, f)`, or a tuple of them",
    note = "a closure needs its argument types written: `|t: &Time, s: &mut State, se: &Sensors|`"
)]
pub trait IntoController<const N: usize, S, C>: seal::Sealed<N> {
    /// The controller set once this is added.
    type Output;

    /// Inserts this into `robot`'s schedule `schedule`.
    fn add_to<E>(
        self,
        schedule: S,
        robot: kmr_core::Robot<N, C, E>,
    ) -> kmr_core::Robot<N, Self::Output, Filled>;
}

impl<const N: usize, F> seal::Sealed<N> for F where F: Fn(&Time, &mut State<'_, N>, &Sensors) {}

// DECISION: `do_not_recommend`, so a non-controller reports "kmr: this is not a controller",
// not the blanket impl's `Fn` bound.
#[diagnostic::do_not_recommend]
impl<const N: usize, S: Schedule, C, F> IntoController<N, S, C> for F
where
    F: Fn(&Time, &mut State<'_, N>, &Sensors),
    C: Insert<S, Inline<ApiInline<F>>>,
{
    type Output = C::Output;

    fn add_to<E>(
        self,
        schedule: S,
        robot: kmr_core::Robot<N, C, E>,
    ) -> kmr_core::Robot<N, C::Output, Filled> {
        robot.add_control_fn(schedule, ApiInline(self))
    }
}

impl<const N: usize, F, G, P> seal::Sealed<N> for Guarded<F, G, P> where
    ApiInline<Guarded<F, G, P>>: ControlFn<N>
{
}

impl<const N: usize, S: Schedule, C, F, G, P> IntoController<N, S, C> for Guarded<F, G, P>
where
    ApiInline<Guarded<F, G, P>>: ControlFn<N>,
    C: Insert<S, Inline<ApiInline<Guarded<F, G, P>>>>,
{
    type Output = C::Output;

    fn add_to<E>(
        self,
        schedule: S,
        robot: kmr_core::Robot<N, C, E>,
    ) -> kmr_core::Robot<N, C::Output, Filled> {
        robot.add_control_fn(schedule, ApiInline(self))
    }
}

impl<const N: usize, T, F> seal::Sealed<N> for With<T, F> where
    F: Fn(&mut T, &Time, &mut State<'_, N>, &Sensors)
{
}

impl<const N: usize, S: Schedule, C, T, F> IntoController<N, S, C> for With<T, F>
where
    F: Fn(&mut T, &Time, &mut State<'_, N>, &Sensors),
    C: Insert<S, InlineWith<ApiInlineWith<F>, T>>,
{
    type Output = C::Output;

    fn add_to<E>(
        self,
        schedule: S,
        robot: kmr_core::Robot<N, C, E>,
    ) -> kmr_core::Robot<N, C::Output, Filled> {
        robot.add_control_fn_with(schedule, self.ctx, ApiInlineWith(self.f))
    }
}

impl<const N: usize, T, F> seal::Sealed<N> for InThread<T, F>
where
    T: Sync + Send,
    F: Fn(&mut T, &Time),
{
}

impl<const N: usize, S: Schedule, C, T, F> IntoController<N, S, C> for InThread<T, F>
where
    T: Sync + Send,
    F: Fn(&mut T, &Time),
    C: Insert<S, Threaded<ApiThreaded<F>, T>>,
{
    type Output = C::Output;

    fn add_to<E>(
        self,
        schedule: S,
        robot: kmr_core::Robot<N, C, E>,
    ) -> kmr_core::Robot<N, C::Output, Filled> {
        robot.add_control_fn_as_thread(schedule, self.ctx, ApiThreaded(self.f))
    }
}

// DECISION: a tuple adds its head, then the tuple of the rest, on the grown set: left to right.
macro_rules! tuples {
    ($head:ident) => {
        impl<const N: usize, $head> seal::Sealed<N> for ($head,) {}

        impl<const N: usize, S: Schedule, C, $head> IntoController<N, S, C> for ($head,)
        where
            $head: IntoController<N, S, C>,
        {
            type Output = $head::Output;

            fn add_to<E>(
                self,
                schedule: S,
                robot: kmr_core::Robot<N, C, E>,
            ) -> kmr_core::Robot<N, Self::Output, Filled> {
                self.0.add_to(schedule, robot)
            }
        }
    };
    ($head:ident, $($tail:ident),+) => {
        impl<const N: usize, $head, $($tail),+> seal::Sealed<N> for ($head, $($tail),+) {}

        impl<const N: usize, S: Schedule, C, $head, $($tail),+> IntoController<N, S, C>
            for ($head, $($tail),+)
        where
            $head: IntoController<N, S, C>,
            ($($tail,)+): IntoController<N, S, $head::Output>,
        {
            type Output = <($($tail,)+) as IntoController<N, S, $head::Output>>::Output;

            #[allow(non_snake_case)]
            fn add_to<E>(
                self,
                schedule: S,
                robot: kmr_core::Robot<N, C, E>,
            ) -> kmr_core::Robot<N, Self::Output, Filled> {
                let ($head, $($tail),+) = self;
                let robot = $head.add_to(schedule, robot);
                ($($tail,)+).add_to(schedule, robot)
            }
        }

        tuples!($($tail),+);
    };
}

tuples!(T1, T2, T3, T4, T5, T6, T7, T8);
