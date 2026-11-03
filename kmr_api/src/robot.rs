use std::marker::PhantomData;
use std::time::Duration;

use crate::spec::Lend;
use crate::{Binding, Bus, Payload, Schedule, Sensors, Spec, State, Time, UserError};
use kmr_core::hal::{Drivers, NoHardware};
use kmr_core::{
    ControlFn, ControlFnWith, Empty, Filled, HCons, HNil, HasController, Inline, InlineWith,
    Insert, Scheduled, ThreadFn, Threaded,
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

// Same wrapper as `ApiInline`, for `add_controller_with`: it also lends the
// user their own `ctx: &mut T`. Note the api argument ORDER — `ctx` first — so
// controllers read `fn(&mut T, &Time, &mut State, &Sensors)`. The core calls
// with `ctx` last (its native order); we re-order here.
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
// `kmr_api::Robot<N, H, C>` is a thin wrapper around `kmr_core::Robot<N, C>`: the core
// owns the real HLists (`HCons`/`HNil`, one per schedule) and grows them
// itself. This layer adds ONLY the api-facing bounds the core deliberately
// stays ignorant of — the concrete `Time`/`State`/`Sensors` controller
// signature, and `Send` for threaded controllers.
//
// `N` is the robot's actuator count and `H` the robot `robot!` names (its hidden
// `__Spec`): users write the `Robot` alias and never see either.
//
// `C` is the *type-state* of the controller set: it starts as `Scheduled`
// (every schedule empty) and one schedule grows by one `kmr_core` node per
// `add_controller*` call. Users cannot construct those nodes (their fields are
// `pub(crate)` to the core) nor name the `Env` a schedule walk needs, so they
// can neither forge a node nor drive a schedule themselves: the engine's
// invariants cannot be bypassed and its internals can change without breaking
// this crate's API.
//
// `E` is the core's controller type-state, `Empty` until the first `add_controller*`, then
// `Filled`: `run()` needs `Filled`, so a robot without a controller does not build, with the
// core's "kmr: this robot has no controller" (SRS NFR-4).
pub struct Robot<const N: usize, H, C = Scheduled, E = Empty> {
    inner: kmr_core::Robot<N, C, E>,
    plugins: Vec<Bus>,
    payloads: Vec<Payload>,
    spec: PhantomData<fn() -> H>,
}

impl<const N: usize, H: Spec<N>> Robot<N, H> {
    /// A robot with no controller, no plugin, no payload and no `dt` yet.
    pub fn new() -> Self {
        Self {
            inner: kmr_core::Robot::new(),
            plugins: Vec::new(),
            payloads: Vec::new(),
            spec: PhantomData,
        }
    }
}

impl<const N: usize, H: Spec<N>> Default for Robot<N, H> {
    fn default() -> Self {
        Self::new()
    }
}

// Settings. These keep the SAME type-state `C`: tuning the config never adds or
// removes a controller, so `Self` is returned unchanged.
impl<const N: usize, H, C, E> Robot<N, H, C, E> {
    /// Sets the tick period. Zero or outside [10 µs, 1 s], [`run`](Self::run) refuses it
    /// with [`UserError::DtOutOfRange`] before any I/O; never set, the run uses 1 ms with one
    /// warning (SRS FR-3.1).
    pub fn set_dt(self, dt: Duration) -> Self {
        Self {
            inner: self.inner.set_dt(dt),
            ..self
        }
    }

    /// After a shutdown, holds the actuators for `duration` before torque off. Without
    /// it, a shutdown cuts torque at once; with `Duration::ZERO`, the hold command is
    /// sent, then torque is cut.
    pub fn set_hold(self, duration: Duration) -> Self {
        Self {
            inner: self.inner.set_hold(duration),
            ..self
        }
    }

    /// Carries the builder data over to a robot whose controller set grew.
    fn grown<D>(
        self,
        grow: impl FnOnce(kmr_core::Robot<N, C, E>) -> kmr_core::Robot<N, D, Filled>,
    ) -> Robot<N, H, D, Filled> {
        Robot {
            inner: grow(self.inner),
            plugins: self.plugins,
            payloads: self.payloads,
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

    /// Registers a medium or heavy payload, here and never from a controller (SRS FR-8.4).
    /// Its mount link is checked by [`run`](Self::run), before any I/O.
    pub fn register_payload(mut self, payload: Payload) -> Self {
        self.payloads.push(payload);
        self
    }

    // Controllers. Each of these GROWS the type-state: the returned `Robot`
    // carries `C::Output`, where the chosen schedule holds one more node, so
    // the builder must stay one chained expression. That is the price of the zero-cost, cap-free design.
    pub fn add_controller<S: Schedule, F>(self, schedule: S, f: F) -> Robot<N, H, C::Output, Filled>
    where
        C: Insert<S, Inline<ApiInline<F>>>,
        F: Fn(&Time, &mut State<'_, N>, &Sensors),
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
    ) -> Robot<N, H, C::Output, Filled>
    where
        C: Insert<S, InlineWith<ApiInlineWith<F>, T>>,
        F: Fn(&mut T, &Time, &mut State<'_, N>, &Sensors),
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
    ) -> Robot<N, H, C::Output, Filled>
    where
        C: Insert<S, Threaded<ApiThreaded<F>, T>>,
        T: Sync + Send,
        F: Fn(&mut T, &Time),
    {
        self.grown(|inner| inner.add_control_fn_as_thread(schedule, ctx, ApiThreaded(f)))
    }

    /// Finalizes the builder, binds the bus plugins to the robot's drivers, then runs on them;
    /// without a plugin, simulates on no hardware.
    ///
    /// The [`Drive`](crate::Drive) bound is satisfied by every schedule the builder can
    /// produce, so users never trip it.
    ///
    /// # Errors
    ///
    /// Those of [`bind`](Self::bind), then those of [`Bound::run`]. A shutdown and an emergency
    /// stop both return `Ok(())`.
    pub fn run(self) -> Result<(), UserError>
    where
        H: Spec<N>,
        C: kmr_core::Drive<N>,
        E: HasController,
    {
        let Bound { inner, drivers } = self.bind()?;
        match drivers {
            Some(drivers) => drive(inner, drivers),
            None => drive(inner, simulated()),
        }
    }

    /// Finalizes the builder and binds the bus plugins to the robot's drivers, no I/O: the
    /// drivers are reachable before the run (SRS FR-5.9).
    ///
    /// # Errors
    ///
    /// [`UserError::DuplicateBus`], [`UserError::UnknownLink`], then the binding errors of
    /// [`Spec::bind`]. The tick period is checked by the run.
    pub fn bind(self) -> Result<Bound<N, H, C, E>, UserError>
    where
        H: Spec<N>,
    {
        let (inner, plugins) = self.finalize()?;
        let drivers = match plugins.as_slice() {
            [] => None,
            plugins => Some(H::bind(plugins)?),
        };
        Ok(Bound { inner, drivers })
    }

    /// Checks the builder data, no I/O.
    ///
    /// # Errors
    ///
    /// [`UserError::DuplicateBus`], then [`UserError::UnknownLink`].
    fn finalize(self) -> Result<(kmr_core::Robot<N, C, E>, Vec<Binding>), UserError>
    where
        H: Spec<N>,
    {
        for (i, bus) in self.plugins.iter().enumerate() {
            if self.plugins[..i].iter().any(|b| b.port() == bus.port()) {
                return Err(UserError::DuplicateBus { port: bus.port() });
            }
        }
        if let Some(payload) = self
            .payloads
            .iter()
            .find(|payload| !H::LINK_NAMES.contains(&payload.mount()))
        {
            return Err(UserError::UnknownLink {
                link: payload.mount(),
            });
        }
        Ok((
            self.inner,
            self.plugins.iter().map(|bus| bus.binding).collect(),
        ))
    }
}

/// A finalized robot, its bus plugins bound to its drivers: the drivers are reachable here,
/// before the run and after it, never inside a tick (SRS FR-5.9).
pub struct Bound<const N: usize, H: Spec<N>, C, E = Filled> {
    inner: kmr_core::Robot<N, C, E>,
    drivers: Option<H::Drivers>,
}

impl<const N: usize, H: Spec<N>, C, E> Bound<N, H, C, E> {
    /// The robot's drivers, `HCons<A, HCons<B, HNil>>`; `None` without a bus plugin, where the
    /// run simulates.
    pub fn drivers(&mut self) -> Option<&mut H::Drivers> {
        self.drivers.as_mut()
    }

    /// Runs the robot on its drivers, lent to the run, then hands them back, closed; `None`
    /// when it simulated. A member's error names it by its type, short in the message; its
    /// source names the full `&mut <type>`, the member the run saw.
    ///
    /// # Errors
    ///
    /// Before any I/O: [`UserError::DtOutOfRange`], then the claims that do not cover the
    /// actuators as [`UserError::BringUp`].
    /// After: a member's bring-up error as [`UserError::BringUp`]. A shutdown and an emergency
    /// stop both return `Ok`.
    pub fn run(self) -> Result<Option<H::Drivers>, UserError>
    where
        C: kmr_core::Drive<N>,
        E: HasController,
    {
        let Self { inner, mut drivers } = self;
        match &mut drivers {
            Some(drivers) => drive(inner, drivers.lend())?,
            None => drive(inner, simulated())?,
        }
        Ok(drivers)
    }
}

/// The drivers of a run without a bus plugin.
fn simulated() -> HCons<NoHardware, HNil> {
    HCons {
        head: NoHardware,
        tail: HNil,
    }
}

/// Runs `inner` on `drivers`: a refused configuration as its [`UserError`], a hardware error
/// kept typed inside [`UserError::BringUp`].
fn drive<const N: usize, C, E, D>(
    inner: kmr_core::Robot<N, C, E>,
    drivers: D,
) -> Result<(), UserError>
where
    C: kmr_core::Drive<N>,
    E: HasController,
    D: Drivers<N, Error: Send + Sync>,
{
    inner.run_on(drivers).map_err(UserError::from)
}

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

    impl Spec<4> for TestSpec {
        type Drivers = HCons<crate::__NoHardware, HNil>;

        const LINK_NAMES: &'static [&'static str] = &["base", "tool"];
        const PRISMATIC: [u64; crate::MASK_WORDS] = [0; crate::MASK_WORDS];
        const CONTINUOUS: [u64; crate::MASK_WORDS] = [0; crate::MASK_WORDS];
        const LOWER: [crate::Q; 4] = [crate::Q(-1.0); 4];
        const UPPER: [crate::Q; 4] = [crate::Q(1.0); 4];
        const SPEED: [crate::Qd; 4] = [crate::Qd(1.0); 4];
        const EFFORT: [crate::Tau; 4] = [crate::Tau(1.0); 4];

        fn bind(plugins: &[crate::Binding]) -> Result<Self::Drivers, UserError> {
            crate::__bind_none(plugins)
        }
    }

    type TestRobot = Robot<4, TestSpec>;

    // A controller written ENTIRELY in api types — no `kmr_core` anywhere in
    // sight. Reads current `Q`, adds one, writes it back. If the boundary is
    // wired, this compiles and every method resolves against the api `State`.
    fn bump_q(_t: &Time, robot: &mut State<'_, 4>, _s: &Sensors) {
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
            .add_controller(
                EachTick,
                |_t: &Time, robot: &mut State<'_, 4>, _s: &Sensors| {
                    let _ = robot.q();
                },
            );
    }

    #[test]
    fn a_port_bound_twice_is_a_run_error() {
        let res = TestRobot::new()
            .add_plugins(Bus::by_id("FT8ISZ6H"))
            .add_plugins(Bus::by_id("FT8ISZ6H").baud(3_000_000))
            .add_controller(EachTick, bump_q)
            .run();
        assert!(
            matches!(res, Err(UserError::DuplicateBus { port: "FT8ISZ6H" })),
            "{res:?}"
        );
    }

    #[test]
    fn a_dt_outside_10_us_to_1_s_is_refused_by_the_core() {
        // DECISION: the builder's own errors (a bus bound twice, an unknown link, the binding)
        // come first; the core refuses the dt next, still before any I/O.
        for dt in [
            Duration::ZERO,
            Duration::from_micros(5),
            Duration::from_millis(1001),
        ] {
            let res = TestRobot::new()
                .set_dt(dt)
                .add_controller(EachTick, bump_q)
                .run();
            assert!(
                matches!(res, Err(UserError::DtOutOfRange { dt: refused }) if refused == dt),
                "{res:?}"
            );
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
                |c: &mut Arc<Mutex<u32>>, _t: &Time, _r: &mut State<'_, 4>, _s: &Sensors| {
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
