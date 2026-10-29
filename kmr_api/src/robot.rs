use std::marker::PhantomData;
use std::time::Duration;

use crate::{Binding, Bus, Schedule, Sensors, Spec, State, Time, UserError, payload::Payload};
use kmr_core::hal::NoHardware;
use kmr_core::{
    ControlFn, ControlFnWith, Inline, InlineWith, Insert, Scheduled, ThreadFn, Threaded,
};

// The named api→core translation wrapper stored in place of the user's closure.
//
// Why a named struct and not just the closure: `add_controller` returns
// `Robot<C::Output>`, and `C::Output` is computed from the node type inserted
// (`Inline<..>`). A closure type is unnameable, so if we handed the core a
// wrapping closure the return type would be inexpressible. `ApiInline<F>` is a
// plain named type over the user's `F`, so the builder's type-state still
// threads through.
//
// Why it exists at all: the core calls it with BORROWED core handles; here we
// wrap each in the api newtype for the duration of the call and hand them to
// the user's api-typed closure. That single borrow-wrap is the whole boundary —
// no copy, and `kmr_core::{Time, RobotState, Sensors}` never appears in the
// public `add_controller` signature.
//
// `pub` + `#[doc(hidden)]`: it appears in the builder's public type-state
// (`Inline<ApiInline<F>>`), so it must be nameable at that visibility, but its
// field is private — users can neither forge one nor read it. Same opaque
// posture as `kmr_core::Inline`.
#[doc(hidden)]
pub struct ApiInline<F>(F);

impl<F> ControlFn for ApiInline<F>
where
    F: Fn(&Time, &mut State, &Sensors),
{
    fn call(
        &self,
        time: &kmr_core::Time,
        state: &mut kmr_core::RobotState,
        sensors: &kmr_core::Sensors,
    ) {
        (self.0)(
            &Time::new(time),
            &mut State::new(state),
            &Sensors::new(sensors),
        );
    }
}

// Same wrapper as `ApiInline`, for `add_controller_with`: it also lends the
// user their own `ctx: &mut T`. Note the api argument ORDER — `ctx` first — so
// controllers read `fn(&mut T, &Time, &mut State, &Sensors)`. The core calls
// with `ctx` last (its native order); we re-order here.
#[doc(hidden)]
pub struct ApiInlineWith<F>(F);

impl<F, T> ControlFnWith<T> for ApiInlineWith<F>
where
    F: Fn(&mut T, &Time, &mut State, &Sensors),
{
    fn call(
        &self,
        time: &kmr_core::Time,
        state: &mut kmr_core::RobotState,
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

// Wrapper for `add_controller_as_thread`: a threaded controller gets ONLY its
// own `ctx: &mut T` and the clock — no robot state. Api order is `ctx` first:
// `fn(&mut T, &Time)`.
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

// The "App" struct. The main user interface, and the whole public re-exposure
// of the `kmr_core` engine.
//
// `kmr_api::Robot<H, C>` is a thin wrapper around `kmr_core::Robot<C>`: the core
// owns the real HLists (`HCons`/`HNil`, one per schedule) and grows them
// itself. This layer adds ONLY the api-facing bounds the core deliberately
// stays ignorant of — the concrete `Time`/`State`/`Sensors` controller
// signature, and `Send` for threaded controllers.
//
// `H` is the robot `robot!` names (its hidden `__Spec`): users write the
// `Robot` alias and never see it.
//
// `C` is the *type-state* of the controller set: it starts as `Scheduled`
// (every schedule empty) and one schedule grows by one `kmr_core` node per
// `add_controller*` call. Users cannot construct those nodes (their fields are
// `pub(crate)` to the core) nor name the `Env` a schedule walk needs, so they
// can neither forge a node nor drive a schedule themselves: the engine's
// invariants cannot be bypassed and its internals can change without breaking
// this crate's API.
pub struct Robot<H, C = Scheduled> {
    inner: kmr_core::Robot<C>,
    plugins: Vec<Bus>,
    dt: Option<Duration>,
    spec: PhantomData<fn() -> H>,
}

impl<H: Spec> Robot<H> {
    /// A robot with no controller, no plugin and no `dt` yet.
    pub fn new() -> Self {
        Self {
            inner: kmr_core::Robot::new(),
            plugins: Vec::new(),
            dt: None,
            spec: PhantomData,
        }
    }
}

impl<H: Spec> Default for Robot<H> {
    fn default() -> Self {
        Self::new()
    }
}

// Settings. These keep the SAME type-state `C`: tuning the config never adds or
// removes a controller, so `Self` is returned unchanged.
impl<H, C> Robot<H, C> {
    /// Sets the tick period, handed to the engine by [`run`](Self::run).
    pub fn set_dt(self, dt: Duration) -> Self {
        Self {
            dt: Some(dt),
            ..self
        }
    }

    /// Carries the builder data over to a robot whose controller set grew.
    fn grown<D>(self, grow: impl FnOnce(kmr_core::Robot<C>) -> kmr_core::Robot<D>) -> Robot<H, D> {
        Robot {
            inner: grow(self.inner),
            plugins: self.plugins,
            dt: self.dt,
            spec: PhantomData,
        }
    }

    /// Plugs in hardware, Bevy-style: one call per bus (SRS FR-6.3).
    ///
    /// ```ignore
    /// Robot::new().add_plugins(Bus::by_id("FT8ISZ6H"))
    /// ```
    ///
    /// Without any bus, `run()` simulates. Binding one port twice is a
    /// `run()` error.
    pub fn add_plugins(mut self, bus: Bus) -> Self {
        self.plugins.push(bus);
        self
    }

    // todo: consider this... but forces us to add a modifier to a build time
    // const...
    // pub fn history_depth(self, depth: usize) {}

    // FakeModel. Api-only: registering a payload does not add a controller, so
    // the type-state `C` is unchanged.
    pub fn register_payload(self, model: Payload) -> Self {
        let _ = model;
        todo!()
    }

    // Controllers. Each of these GROWS the type-state: the returned `Robot`
    // carries `C::Output`, where the chosen schedule holds one more node, so
    // the builder must stay one chained expression. That is the price of the zero-cost, cap-free design.
    pub fn add_controller<S: Schedule, F>(self, schedule: S, f: F) -> Robot<H, C::Output>
    where
        C: Insert<S, Inline<ApiInline<F>>>,
        F: Fn(&Time, &mut State, &Sensors),
    {
        // Wrap the user's api-typed closure in `ApiInline`, then hand it to
        // the core's `ControlFn` entry. The core stores `Inline<ApiInline<F>>`
        // and calls back through `ControlFn::call` (above) each tick.
        self.grown(|inner| inner.add_control_fn(schedule, ApiInline(f)))
    }

    // Same as [`add_controller`], but for controllers that need their OWN
    // persistent data across ticks.
    //
    // You hand us the data `ctx` once. We own it, keep it alive for the whole
    // run, and lend it back to you (`&mut T`) on every tick. Your controller
    // becomes a plain `fn` — no `move`, no closure, no `Arc<Mutex<..>>`.
    //
    // ```
    // struct Counter { ticks: u32 }
    //
    // fn count(c: &mut Counter, _t: &Time, _r: &mut State, _s: &Sensors) {
    //     c.ticks += 1;
    // }
    //
    // robot.add_controller_with(PerTick, Counter { ticks: 0 }, count);
    // ```
    //
    // `T` is whatever you want: a struct, a number, a `Vec`... your data.
    // Because we lend it as `&mut T`, access is exclusive and single-threaded,
    // so you never need locks here.
    //
    pub fn add_controller_with<S: Schedule, T, F>(
        self,
        schedule: S,
        ctx: T,
        f: F,
    ) -> Robot<H, C::Output>
    where
        C: Insert<S, InlineWith<ApiInlineWith<F>, T>>,
        F: Fn(&mut T, &Time, &mut State, &Sensors),
    {
        self.grown(|inner| inner.add_control_fn_with(schedule, ctx, ApiInlineWith(f)))
    }

    // Runs a controller on its OWN dedicated thread instead of inline in the
    // main tick loop. For work that has its own rhythm or blocks — a gamepad
    // reader, a network link, a slow planner. Most controllers do NOT need this.
    //
    // Deliberately, a threaded controller gets ONLY `&mut T` — your own data,
    // whatever you need to track. It has NO access to robot `State`:
    //
    //   - Robot commands (`desired`) stay single-writer on the main loop, so
    //     two controllers can never race to write the actuators.
    //   - No shared `State` means no cross-thread borrow of the tick loop's data.
    //
    // To act on the robot, the thread writes what it learned into `T`; a
    // main-loop controller reads `T` and turns it into commands. `ctx` and the
    // function must be `Send + 'static` so they can move to the new thread.
    pub fn add_controller_as_thread<S: Schedule, T, F>(
        self,
        schedule: S,
        ctx: T,
        f: F,
    ) -> Robot<H, C::Output>
    where
        C: Insert<S, Threaded<ApiThreaded<F>, T>>,
        T: Sync + Send,
        F: Fn(&mut T, &Time),
    {
        self.grown(|inner| inner.add_control_fn_as_thread(schedule, ctx, ApiThreaded(f)))
    }

    /// Finalizes the builder, binds the bus plugins to the robot's driver, then runs on it;
    /// without a plugin, simulates on no hardware.
    ///
    /// The `Drive` bound is `kmr_core`'s schedule-walk trait — `pub` but `#[doc(hidden)]`,
    /// satisfied by every schedule the builder can produce, so users never see or trip it.
    ///
    /// # Errors
    ///
    /// Before any I/O: [`UserError::DtOutOfRange`], [`UserError::DuplicateBus`], then the
    /// binding errors of [`Spec::bind`]. After: the driver's bring-up error as
    /// [`UserError::BringUp`]. A shutdown and an emergency stop both return `Ok(())`.
    pub fn run(self) -> Result<(), UserError>
    where
        H: Spec,
        C: kmr_core::Drive,
    {
        let (inner, plugins) = self.finalize()?;
        if plugins.is_empty() {
            return inner.run_on(NoHardware).map_err(|never| match never {});
        }
        let adapter = H::bind(&plugins)?;
        inner.run_on(adapter).map_err(|e| UserError::BringUp {
            cause: e.to_string(),
        })
    }

    /// Checks the builder data, no I/O, and hands `dt` to the core.
    ///
    /// # Errors
    ///
    /// [`UserError::DtOutOfRange`], then [`UserError::DuplicateBus`].
    fn finalize(self) -> Result<(kmr_core::Robot<C>, Vec<Binding>), UserError> {
        if let Some(dt) = self.dt
            && !(DT_MIN..=DT_MAX).contains(&dt)
        {
            return Err(UserError::DtOutOfRange { dt });
        }
        for (i, bus) in self.plugins.iter().enumerate() {
            if self.plugins[..i].iter().any(|b| b.port() == bus.port()) {
                return Err(UserError::DuplicateBus { port: bus.port() });
            }
        }
        let inner = match self.dt {
            Some(dt) => self.inner.set_dt(dt),
            None => self.inner,
        };
        Ok((inner, self.plugins.iter().map(|bus| bus.binding).collect()))
    }
}

/// Shortest tick period `run()` accepts (SRS FR-3.1); interim until the core checks it (T040).
const DT_MIN: Duration = Duration::from_micros(10);

/// Longest tick period `run()` accepts (SRS FR-3.1).
const DT_MAX: Duration = Duration::from_secs(1);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EachTick, Q};
    use std::sync::{Arc, Mutex, PoisonError};
    use std::time::Duration;

    // The signal bus is process-global: tests that run or raise take this first.
    static BUS: Mutex<()> = Mutex::new(());

    /// Stands for the `__Spec` that `robot!` emits.
    struct TestSpec;

    impl Spec for TestSpec {
        type Adapter = crate::__NoHardware;

        fn bind(plugins: &[crate::Binding]) -> Result<Self::Adapter, UserError> {
            crate::__bind_none(plugins)
        }
    }

    type TestRobot = Robot<TestSpec>;

    // A controller written ENTIRELY in api types — no `kmr_core` anywhere in
    // sight. Reads current `Q`, adds one, writes it back. If the boundary is
    // wired, this compiles and every method resolves against the api `State`.
    fn bump_q(_t: &Time, robot: &mut State, _s: &Sensors) {
        if let Some(q) = robot.q() {
            let desired = q.map(|x| x + Q(1.0));
            robot.set_all_q(desired);
        }
    }

    // Proves the plain `add_controller` boundary: it accepts an all-api-typed
    // `fn` item and a closure, and the type-state threads through repeated adds.
    #[test]
    fn add_controller_accepts_api_typed_controllers() {
        let _robot = TestRobot::new()
            .set_dt(Duration::from_millis(1))
            .add_controller(EachTick, bump_q)
            .add_controller(EachTick, |_t: &Time, robot: &mut State, _s: &Sensors| {
                let _ = robot.q();
            });
    }

    #[test]
    fn a_port_bound_twice_is_a_run_error() {
        let res = TestRobot::new()
            .add_plugins(Bus::by_id("FT8ISZ6H"))
            .add_plugins(Bus::by_id("FT8ISZ6H").baud(3_000_000))
            .add_controller(EachTick, bump_q)
            .run();
        assert_eq!(res, Err(UserError::DuplicateBus { port: "FT8ISZ6H" }));
    }

    #[test]
    fn a_dt_outside_10_us_to_1_s_is_refused_before_the_plugins() {
        for dt in [
            Duration::ZERO,
            Duration::from_micros(5),
            Duration::from_millis(1001),
        ] {
            let res = TestRobot::new()
                .set_dt(dt)
                .add_plugins(Bus::by_id("FT8ISZ6H"))
                .add_plugins(Bus::by_id("FT8ISZ6H"))
                .add_controller(EachTick, bump_q)
                .run();
            assert_eq!(res, Err(UserError::DtOutOfRange { dt }));
        }
    }

    #[test]
    fn a_dt_of_10_us_or_1_s_or_unset_passes_the_check() {
        for dt in [
            Some(Duration::from_micros(10)),
            Some(Duration::from_secs(1)),
            None,
        ] {
            let robot = TestRobot::new().add_controller(EachTick, bump_q);
            let robot = match dt {
                Some(dt) => robot.set_dt(dt),
                None => robot,
            };
            assert!(robot.finalize().is_ok());
        }
    }

    #[test]
    fn finalize_hands_every_plugin_on_in_order() {
        let robot = TestRobot::new()
            .add_plugins(Bus::by_id("FT8ISZ6H"))
            .add_plugins(Bus::by_dev("/dev/ttyACM0").bus("arm"));
        let (_, plugins) = robot.finalize().expect("valid builder");
        assert_eq!(
            plugins,
            [
                Bus::by_id("FT8ISZ6H").binding,
                Bus::by_dev("/dev/ttyACM0").bus("arm").binding
            ]
        );
    }

    #[test]
    fn plugins_survive_every_builder_call() {
        let robot = TestRobot::new()
            .add_plugins(Bus::by_id("FT8ISZ6H"))
            .set_dt(Duration::from_millis(1))
            .add_controller(EachTick, bump_q)
            .add_plugins(Bus::by_dev("/dev/ttyACM0"));
        assert_eq!(robot.plugins.len(), 2);
    }

    // Proves all THREE builders compile with api types AND that `run()` drives
    // the schedule end to end. The `_with` controller increments a shared
    // counter through its `ctx: &mut T`; after one tick we observe it fired —
    // so the ctx passthrough and the state-wrapping boundary both work at
    // runtime, not just at compile time. (Threaded controllers are stored but
    // their thread machinery is still WIP, so the thread closure just has to
    // type-check.)
    #[test]
    fn run_drives_every_builder_kind() {
        let _bus = BUS.lock().unwrap_or_else(PoisonError::into_inner);

        let ticks = Arc::new(Mutex::new(0u32));
        let ticks_ctx = Arc::clone(&ticks);
        let pad = Arc::new(Mutex::new(0u32));

        TestRobot::new()
            .set_dt(Duration::from_millis(1))
            .add_controller(EachTick, bump_q)
            // ctx FIRST — the api argument order.
            .add_controller_with(
                EachTick,
                ticks_ctx,
                |c: &mut Arc<Mutex<u32>>, _t: &Time, _r: &mut State, _s: &Sensors| {
                    if let Ok(mut n) = c.lock() {
                        *n += 1;
                        assert!(*n < 500, "the loop ignored the shutdown");
                    }
                    crate::shutdown!("test: one tick is enough");
                },
            )
            // threaded: only `&mut T` and the clock, no state.
            .add_controller_as_thread(EachTick, pad, |_p: &mut Arc<Mutex<u32>>, _t: &Time| {})
            .run()
            .expect("run should drive the schedule and return Ok");

        // Graceful shutdown: the raising tick completes, no other starts.
        assert_eq!(*ticks.lock().expect("counter lock"), 1);
    }
}
