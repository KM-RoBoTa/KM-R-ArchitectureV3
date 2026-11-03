use std::marker::PhantomData;
use std::time::Duration;

use crate::fallback::Guarded;
use crate::plugin::{Plugin, Plugins};
use crate::spec::Lend;
use crate::{
    Binding, Envelope, Limit, Payload, Policy, Schedule, Sensors, Spec, State, Time, UserError,
};
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
    plugins: Plugins,
    payloads: Vec<Payload>,
    guards: Guards<N>,
    spec: PhantomData<fn() -> H>,
}

/// The limit policies and envelopes the builder collected, handed to the core at
/// finalization.
struct Guards<const N: usize> {
    policy: [Option<Policy>; N],
    envelope: [Envelope; N],
    /// The first actuator index past N a setting named.
    stray: Option<usize>,
}

impl<const N: usize> Guards<N> {
    /// No policy, no envelope.
    fn new() -> Self {
        Self {
            policy: [None; N],
            envelope: [Envelope::NONE; N],
            stray: None,
        }
    }

    /// Applies `set` to actuator `i`, or notes `i` past N.
    fn at(&mut self, i: usize, set: impl FnOnce(&mut Option<Policy>, &mut Envelope)) {
        match (self.policy.get_mut(i), self.envelope.get_mut(i)) {
            (Some(policy), Some(envelope)) => set(policy, envelope),
            _ => {
                self.stray.get_or_insert(i);
            }
        }
    }
}

impl<const N: usize, H: Spec<N>> Robot<N, H> {
    /// A robot with no controller, no plugin, no payload and no `dt` yet.
    pub fn new() -> Self {
        Self {
            inner: kmr_core::Robot::new(),
            plugins: Plugins::default(),
            payloads: Vec::new(),
            guards: Guards::new(),
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

    /// What a breach of actuator `i`'s limit does (SRS FR-4.5). Every actuator whose model
    /// limit bounds anything needs one, or [`run`](Self::run) refuses with
    /// [`UserError::NoPolicy`] before any I/O. An `i` past N is [`UserError::NoActuator`].
    pub fn set_limit_policy(mut self, i: usize, policy: Policy) -> Self {
        self.guards.at(i, |slot, _| *slot = Some(policy));
        self
    }

    /// [`set_limit_policy`](Self::set_limit_policy) for every actuator of `group`.
    pub fn set_limit_policy_group<const K: usize>(
        mut self,
        group: &[usize; K],
        policy: Policy,
    ) -> Self {
        for &i in group {
            self.guards.at(i, |slot, _| *slot = Some(policy));
        }
        self
    }

    /// [`set_limit_policy`](Self::set_limit_policy) for every actuator.
    pub fn set_limit_policy_all(mut self, policy: Policy) -> Self {
        self.guards.policy = [Some(policy); N];
        self
    }

    /// Bounds actuator `i`'s written values: a value outside is clamped and its `envelope`
    /// fault bit set (SRS FR-13.5). Each bound must be finite with `lower <= upper`, or
    /// [`run`](Self::run) refuses with [`UserError::InvalidEnvelope`]. An `i` past N is
    /// [`UserError::NoActuator`].
    pub fn set_envelope(mut self, i: usize, envelope: Envelope) -> Self {
        self.guards.at(i, |_, slot| *slot = envelope);
        self
    }

    /// [`set_envelope`](Self::set_envelope) for every actuator of `group`.
    pub fn set_envelope_group<const K: usize>(
        mut self,
        group: &[usize; K],
        envelope: Envelope,
    ) -> Self {
        for &i in group {
            self.guards.at(i, |_, slot| *slot = envelope);
        }
        self
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
            guards: self.guards,
            spec: PhantomData,
        }
    }

    /// Plugs in hardware, Bevy-style: one call per bus (SRS FR-6.3); with the `zenoh` feature,
    /// also the Zenoh gateway (SRS FR-7.1).
    ///
    /// ```ignore
    /// Robot::new().add_plugins(Bus::by_id("FT8ISZ6H"))
    /// ```
    ///
    /// Without any bus, `run()` simulates. Binding one port twice is a
    /// `run()` error.
    pub fn add_plugins(mut self, plugin: impl Plugin) -> Self {
        plugin.plug(&mut self.plugins);
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

    /// Adds `f` with its `fallback`: each tick, `when` reads the tick's [`Faults`](crate::Faults)
    /// and exactly one of the two runs, `fallback` when `when` is true (SRS FR-13.5). Build
    /// `when` from [`stale_in`](crate::stale_in), [`remote_timeout`](crate::remote_timeout) and
    /// [`any_of`](crate::any_of), or any `Fn(&Faults<N>) -> bool`. No hold policy: a fallback
    /// that writes nothing sends nothing (FR-050).
    pub fn add_controller_or<S: Schedule, F, P, G>(
        self,
        schedule: S,
        f: F,
        when: P,
        fallback: G,
    ) -> Robot<N, H, C::Output, Filled>
    where
        C: Insert<S, Inline<ApiInline<Guarded<F, G, P>>>>,
        F: Fn(&Time, &mut State<'_, N>, &Sensors),
        G: Fn(&Time, &mut State<'_, N>, &Sensors),
        P: Fn(&crate::Faults<N>) -> bool,
    {
        let guarded = Guarded {
            controller: f,
            fallback,
            when,
        };
        self.grown(|inner| inner.add_control_fn(schedule, ApiInline(guarded)))
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
        let Bound {
            inner,
            drivers,
            #[cfg(feature = "zenoh")]
            gateway,
        } = self.bind()?;
        let net = Net {
            #[cfg(feature = "zenoh")]
            gateway: gateway.as_ref(),
            _robot: PhantomData,
        };
        match drivers {
            Some(drivers) => drive(inner, drivers, net),
            None => drive(inner, simulated(), net),
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
        let Finalized {
            inner,
            bindings,
            #[cfg(feature = "zenoh")]
            gateway,
        } = self.finalize()?;
        let drivers = match bindings.as_slice() {
            [] => None,
            bindings => Some(H::bind(bindings)?),
        };
        Ok(Bound {
            inner,
            drivers,
            #[cfg(feature = "zenoh")]
            gateway,
        })
    }

    /// Checks the builder data, no I/O, and hands the limits, their policies and the envelopes
    /// to the core. A placeholder model's limits are not handed over.
    ///
    /// # Errors
    ///
    /// [`UserError::DuplicateBus`], [`UserError::UnknownLink`], [`UserError::NoActuator`],
    /// then [`UserError::InvalidEnvelope`].
    fn finalize(self) -> Result<Finalized<N, C, E>, UserError>
    where
        H: Spec<N>,
    {
        let buses = &self.plugins.buses;
        for (i, bus) in buses.iter().enumerate() {
            if buses[..i].iter().any(|b| b.port() == bus.port()) {
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
        if let Some(index) = self.guards.stray {
            return Err(UserError::NoActuator { index, n: N });
        }
        if let Some(actuator) = self.guards.envelope.iter().position(|e| !valid(e)) {
            return Err(UserError::InvalidEnvelope { actuator });
        }
        // DECISION: a placeholder model (kmr_model T023) hands no limit, so no policy is
        // needed until its real descriptor lands (kmr_model T029).
        let limits = if H::PLACEHOLDER {
            [Limit::NONE; N]
        } else {
            H::LIMITS
        };
        let inner = self
            .inner
            .set_limits(limits, self.guards.policy)
            .set_envelopes(self.guards.envelope);
        Ok(Finalized {
            inner,
            bindings: buses.iter().map(|bus| bus.binding).collect(),
            #[cfg(feature = "zenoh")]
            gateway: crate::zenoh::one(self.plugins.gateways)?,
        })
    }
}

#[cfg(any(test, feature = "test-util"))]
impl<const N: usize, H: Spec<N>, C: kmr_core::Drive<N>> Robot<N, H, C, Filled> {
    /// Finalizes the builder like [`run`](Self::run), then runs it on `drivers` with `host`
    /// as its clock, sleep and bus: no real sleep, no global signal bus.
    pub(crate) fn run_scripted<P, D>(
        self,
        host: &mut P,
        drivers: &mut D,
    ) -> Result<kmr_core::Signal, UserError>
    where
        P: kmr_core::test_util::Host,
        D: Drivers<N, Error: Send + Sync>,
    {
        let finalized = self.finalize()?;
        let mut inner = finalized.inner;
        #[cfg(feature = "zenoh")]
        if let Some(gateway) = &finalized.gateway {
            let (running, mut stages) = gateway.start::<N>()?;
            let signal =
                kmr_core::test_util::run_scripted_staged(&mut inner, host, drivers, &mut stages);
            drop(running);
            return signal.map_err(UserError::from);
        }
        kmr_core::test_util::run_scripted(&mut inner, host, drivers).map_err(UserError::from)
    }
}

#[cfg(test)]
impl<const N: usize, H: Spec<N>, C, E> Robot<N, H, C, E> {
    /// The core robot `run()` would hand to the loop.
    pub(crate) fn finalized(self) -> kmr_core::Robot<N, C, E> {
        self.finalize().expect("a valid builder").inner
    }
}

/// A finalized robot, its bus plugins bound to its drivers: the drivers are reachable here,
/// before the run and after it, never inside a tick (SRS FR-5.9).
pub struct Bound<const N: usize, H: Spec<N>, C, E = Filled> {
    inner: kmr_core::Robot<N, C, E>,
    drivers: Option<H::Drivers>,
    /// The gateway plugged in, started by the run.
    #[cfg(feature = "zenoh")]
    gateway: Option<crate::zenoh::Gateway>,
}

/// The builder's data checked and handed over: the core robot and the bus bindings.
struct Finalized<const N: usize, C, E> {
    inner: kmr_core::Robot<N, C, E>,
    bindings: Vec<Binding>,
    /// The one gateway plugged in, if any.
    #[cfg(feature = "zenoh")]
    gateway: Option<crate::zenoh::Gateway>,
}

/// What a run starts next to the tick loop: the gateway, with the `zenoh` feature.
struct Net<'a, const N: usize> {
    #[cfg(feature = "zenoh")]
    gateway: Option<&'a crate::zenoh::Gateway>,
    _robot: PhantomData<&'a [(); N]>,
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
        let Self {
            inner,
            mut drivers,
            #[cfg(feature = "zenoh")]
            gateway,
        } = self;
        let net = Net {
            #[cfg(feature = "zenoh")]
            gateway: gateway.as_ref(),
            _robot: PhantomData,
        };
        match &mut drivers {
            Some(drivers) => drive(inner, drivers.lend(), net)?,
            None => drive(inner, simulated(), net)?,
        }
        Ok(drivers)
    }
}

/// Whether every bound of `envelope` is finite with `lower <= upper`.
fn valid(envelope: &Envelope) -> bool {
    let ok = |(lo, hi): (f32, f32)| lo.is_finite() && hi.is_finite() && lo <= hi;
    envelope.q.is_none_or(|(lo, hi)| ok((lo.0, hi.0)))
        && envelope.qd.is_none_or(|(lo, hi)| ok((lo.0, hi.0)))
        && envelope.tau.is_none_or(|(lo, hi)| ok((lo.0, hi.0)))
}

/// The drivers of a run without a bus plugin.
fn simulated() -> HCons<NoHardware, HNil> {
    HCons {
        head: NoHardware,
        tail: HNil,
    }
}

/// Runs `inner` on `drivers`, the gateway of `net` around each tick when there is one: a
/// refused configuration as its [`UserError`], a hardware error kept typed inside
/// [`UserError::BringUp`]. The gateway stops when the run ends.
fn drive<const N: usize, C, E, D>(
    inner: kmr_core::Robot<N, C, E>,
    drivers: D,
    net: Net<'_, N>,
) -> Result<(), UserError>
where
    C: kmr_core::Drive<N>,
    E: HasController,
    D: Drivers<N, Error: Send + Sync>,
{
    #[cfg(feature = "zenoh")]
    if let Some(gateway) = net.gateway {
        let (running, stages) = gateway.start::<N>()?;
        let result = inner
            .run_on_staged(drivers, stages)
            .map_err(UserError::from);
        drop(running);
        return result;
    }
    let _ = net;
    inner.run_on(drivers).map_err(UserError::from)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::{Bus, EachTick, Q};
    use std::sync::{Arc, Mutex, PoisonError};
    use std::time::Duration;

    // The signal bus is process-global: tests that run or raise take this first.
    static BUS: Mutex<()> = Mutex::new(());

    /// Stands for the `__Spec` that `robot!` emits.
    pub(crate) struct TestSpec;

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
        let plugins = robot.finalize().expect("valid builder").bindings;
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
        assert_eq!(robot.plugins.buses.len(), 2);
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
            .set_limit_policy_all(Policy::Clamp)
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

/// The state and clock getters on the core's scripted host and recording adapter: no real
/// sleep, no global bus (core `test-util`).
#[cfg(test)]
mod scripted {
    use core::cell::{Cell, RefCell};
    use core::num::NonZeroU32;
    use std::time::Instant;

    use kmr_core::test_util::{DT, Log, PER_TICK, Recorder, Scripted, TICK_END, run_scripted};

    use super::tests::TestSpec;
    use super::*;
    use crate::{First, StateError};

    /// What a `First` controller saw on each of the first ticks.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    struct Seen {
        stale: [bool; 4],
        overrun_before: bool,
        age: Option<u32>,
        sample_age: Option<u32>,
        overruns: u64,
        last_overrun: Option<Duration>,
        elapsed: Duration,
        dt: Duration,
    }

    /// Runs a robot whose one `First` controller records [`Seen`] per tick, on `recorder`, for
    /// `ticks` ticks of the scripted `host`.
    fn record<const T: usize>(
        host: Scripted,
        recorder: Option<Recorder<'_, 4>>,
    ) -> ([Seen; T], Result<NonZeroU32, StateError>) {
        let seen = RefCell::new([Seen::default(); T]);
        let tick = Cell::new(0_usize);
        let every = Cell::new(Err(StateError::CriticalValueMissing));
        let mut robot = Robot::<4, TestSpec>::new().set_dt(DT).add_controller(
            First,
            |time: &Time, state: &mut State<'_, 4>, _: &Sensors| {
                if let Some(slot) = seen.borrow_mut().get_mut(tick.get()) {
                    *slot = Seen {
                        stale: core::array::from_fn(|i| state.stale_at(i)),
                        overrun_before: state.prev_faults(1).is_ok_and(|f| f.overrun()),
                        age: state.age_at(0).ok().flatten(),
                        sample_age: state.sample_age(),
                        overruns: time.overrun_count(),
                        last_overrun: time.last_overrun(),
                        elapsed: time.elapsed(),
                        dt: time.dt(),
                    };
                }
                every.set(state.read_every(0));
                tick.set(tick.get() + 1);
            },
        );
        let mut host = host.shutdown_from(TICK_END + (T - 1) * PER_TICK);
        let signal = match recorder {
            Some(recorder) => run_scripted(
                &mut robot.inner,
                &mut host,
                &mut HCons {
                    head: recorder,
                    tail: HNil,
                },
            )
            .map_err(|_| ()),
            None => run_scripted(&mut robot.inner, &mut host, &mut simulated()).map_err(|_| ()),
        };
        assert!(signal.is_ok());
        (seen.into_inner(), every.into_inner())
    }

    #[test]
    fn a_tick_without_a_read_ages_the_sample_by_one() {
        let log = Log::new();
        let recorder = Recorder {
            every: NonZeroU32::new(3).expect("3 is not zero"),
            ..Recorder::new(&log)
        };

        let (seen, every) = record::<5>(Scripted::new(Instant::now()), Some(recorder));

        assert_eq!(seen.map(|s| s.sample_age), [0, 1, 2, 0, 1].map(Some));
        assert_eq!(seen.map(|s| s.age), [0, 1, 2, 0, 1].map(Some));
        assert_eq!(every, Ok(NonZeroU32::new(3).expect("3 is not zero")));
    }

    #[test]
    fn a_failed_read_of_actuator_2_sets_stale_at_2_on_that_tick_only() {
        let log = Log::new();
        let mut faults = kmr_core::hal::Faults::default();
        faults.stale.set(2);
        let recorder = Recorder {
            faults,
            faults_on: Some(1),
            ..Recorder::new(&log)
        };

        let (seen, _) = record::<3>(Scripted::new(Instant::now()), Some(recorder));

        let none = [false; 4];
        assert_eq!(
            seen.map(|s| s.stale),
            [none, [false, false, true, false], none]
        );
    }

    #[test]
    fn a_run_without_hardware_has_no_age_and_no_sample() {
        let (seen, every) = record::<2>(Scripted::new(Instant::now()), None);

        assert_eq!(seen.map(|s| (s.age, s.sample_age)), [(None, None); 2]);
        assert_eq!(every, Ok(NonZeroU32::MIN));
    }

    #[test]
    fn an_overrun_tick_counts_once_and_reports_its_lateness() {
        // `now` at arm, then at the start and the end of each tick: tick 0 ends at 1.5 dt.
        let host = Scripted::new(Instant::now()).script(&[0, 0, 15, 20, 25, 30, 35]);

        let (seen, _) = record::<3>(host, Some(Recorder::new(&Log::new())));

        assert_eq!(seen.map(|s| s.overruns), [0, 1, 1]);
        assert_eq!(seen.map(|s| s.overrun_before), [false, true, false]);
        assert_eq!(seen[1].last_overrun, Some(DT / 2));
        assert_eq!(seen.map(|s| s.dt), [DT; 3]);
        assert_eq!(seen[0].elapsed, Duration::ZERO);
        assert_eq!(seen[1].elapsed, DT * 2);
    }
}

/// Fallbacks and envelopes on the core's scripted host and recording adapter (quickstart
/// § Phase E). In the crate: the run needs the finalized core robot.
// DECISION: here rather than kmr_api/tests/run.rs: the scripted host drives the core robot,
// which only the crate can reach after `finalize`.
#[cfg(test)]
mod guards {
    use core::cell::RefCell;
    use std::time::Instant;

    use kmr_core::test_util::{DT, Log, PER_TICK, Recorder, Scripted, TICK_END, run_scripted};

    use super::tests::TestSpec;
    use super::*;
    use crate::{EachTick, Mask, Q, any_of, remote_timeout, stale_in};

    /// Runs the finalized `robot` on `recorder` for `ticks` ticks.
    fn run<C: kmr_core::Drive<4>>(
        robot: Robot<4, TestSpec, C, Filled>,
        recorder: Recorder<'_, 4>,
        ticks: usize,
    ) {
        let mut inner = robot.finalize().expect("a valid builder").inner;
        let mut host =
            Scripted::new(Instant::now()).shutdown_from(TICK_END + (ticks - 1) * PER_TICK);
        let mut drivers = HCons {
            head: recorder,
            tail: HNil,
        };
        let signal = run_scripted(&mut inner, &mut host, &mut drivers);
        assert!(signal.is_ok());
    }

    #[test]
    fn us5_a_fallback_on_stale_in_a_runs_in_place_of_the_controller_for_that_tick() {
        const A: [usize; 2] = [2, 3];
        let log = Log::new();
        let mut faults = kmr_core::hal::Faults::default();
        faults.stale.set(3);
        let recorder = Recorder {
            faults,
            faults_on: Some(1),
            ..Recorder::new(&log)
        };
        let ran = RefCell::new(Vec::new());
        let mark = |name: &'static str| {
            let ran = &ran;
            move |_t: &Time, _s: &mut State<'_, 4>, _se: &Sensors| ran.borrow_mut().push(name)
        };
        let robot = Robot::<4, TestSpec>::new()
            .set_dt(DT)
            .set_limit_policy_all(Policy::Clamp)
            .add_controller_or(
                EachTick,
                mark("controller"),
                any_of(stale_in(&A), remote_timeout()),
                mark("fallback"),
            );

        run(robot, recorder, 3);

        assert_eq!(*ran.borrow(), ["controller", "fallback", "controller"]);
    }

    /// The position the adapter was sent for `value` written to `targets`, and the
    /// `envelope` bits of that tick, under `envelope` on `slice`.
    fn sweep<const K: usize>(
        slice: &[usize; K],
        targets: &[usize],
        value: f32,
    ) -> ([Q; 4], Mask<4>) {
        let log = Log::new();
        let bits = RefCell::new(Mask::NONE);
        let envelope = Envelope {
            q: Some((Q(-0.5), Q(0.5))),
            ..Envelope::NONE
        };
        let robot = Robot::<4, TestSpec>::new()
            .set_dt(DT)
            .set_limit_policy_all(Policy::Clamp)
            .set_envelope_group(slice, envelope)
            .add_controller(
                EachTick,
                |time: &Time, state: &mut State<'_, 4>, _: &Sensors| {
                    if time.tick() == 0 {
                        for &i in targets {
                            state.set_q_at(Q(value), i).expect("in range");
                        }
                    } else if let Ok(faults) = state.prev_faults(1) {
                        *bits.borrow_mut() = faults.envelope();
                    }
                },
            );

        run(robot, Recorder::new(&log), 2);

        let sent = log.write.get().expect("tick 0 wrote");
        (sent.q, bits.into_inner())
    }

    #[test]
    fn sc_013_out_of_envelope_targets_reach_the_adapter_clamped_and_flagged() {
        for (value, sent) in [(-2.0, -0.5), (0.2, 0.2), (3.0, 0.5)] {
            let (q, bits) = sweep(&[0], &[0], value);
            assert_eq!(q[0], Q(sent));
            assert_eq!(bits.get(0), value != sent, "{value}");
            assert!(!bits.get(1));
        }
    }

    #[test]
    fn us5_an_envelope_on_a_group_slice_reaches_every_actuator_of_the_slice() {
        let (q, bits) = sweep(&[1, 3], &[0, 1, 2, 3], 0.9);
        assert_eq!(q, [Q(0.9), Q(0.5), Q(0.9), Q(0.5)]);
        assert_eq!(bits.iter().collect::<Vec<_>>(), [1, 3]);
    }
}
