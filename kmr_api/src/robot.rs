use crate::{Schedule, Sensors, State, Time, UserError, payload::Payload};
use kmr_core::{HCons, HList, HNil, Inline, InlineWith, Threaded};

// The "App" struct. The main user interface, and the whole public re-exposure
// of the closed-source `kmr_core`.
//
// `kmr_api::Robot<L>` is a thin newtype around `kmr_core::Robot<L>`: the core
// owns the real HList (`HCons`/`HNil`) and grows it itself. This layer adds
// ONLY the api-facing bounds the core deliberately stays ignorant of — the
// `FakeSchedule` gate, the concrete `Time`/`State`/`Sensors` controller
// signature, and `Send` for threaded controllers.
//
// `L` is the *type-state* of the controller set: it starts as `HNil` (empty)
// and grows one `kmr_core` node per `add_controller*` call. Users cannot
// construct those nodes (their fields are `pub(crate)` to the core) nor
// implement `Runnable` (sealed in the core), so the IP stays behind the
// compiled `kmr_core` rlib while this crate stays public.
pub struct Robot<L: HList = HNil> {
    inner: kmr_core::Robot<L>,
}

impl Robot<HNil> {
    // Instanciate a new default robot with no controllers yet.
    pub fn new() -> Self {
        Self {
            inner: kmr_core::Robot::new(),
        }
    }
}

impl Default for Robot<HNil> {
    fn default() -> Self {
        Self::new()
    }
}

// Settings. These keep the SAME type-state `L`: tuning the config never adds or
// removes a controller, so `Self` is returned unchanged.
impl<L: HList> Robot<L> {
    pub fn dt_ms(self, ms: u32) -> Self {
        Self {
            inner: self.inner.set_dt_ms(ms),
        }
    }

    pub fn dt_us(self, us: u32) -> Self {
        Self {
            inner: self.inner.set_dt_us(us),
        }
    }

    // todo: consider this... but forces us to add a modifier to a build time
    // const...
    // pub fn history_depth(self, depth: usize) {}

    // FakeModel. Api-only: registering a payload does not add a controller, so
    // the type-state `L` is unchanged.
    pub fn register_payload(self, model: Payload) -> Self {
        let _ = model;
        todo!()
    }

    // Controllers. Each of these GROWS the type-state: the returned `Robot`
    // carries a new `HCons<Node, L>`, so the builder must stay one chained
    // expression. That is the price of the zero-cost, cap-free design.

    pub fn add_controller<S, F>(self, schedule: S, f: F) -> Robot<HCons<Inline<S, F>, L>>
    where
        S: Schedule,
        F: Fn(&Time, &mut State, &Sensors),
    {
        Robot {
            inner: self.inner.add_controller(schedule, f),
        }
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
    pub fn add_controller_with<S, T, F>(
        self,
        schedule: S,
        ctx: T,
        f: F,
    ) -> Robot<HCons<InlineWith<S, T, F>, L>>
    where
        S: Schedule,
        F: Fn(&mut T, &Time, &mut State, &Sensors),
    {
        Robot {
            inner: self.inner.add_controller_with(schedule, ctx, f),
        }
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
    //
    pub fn add_controller_as_thread<S, T, F>(
        self,
        schedule: S,
        ctx: T,
        f: F,
    ) -> Robot<HCons<Threaded<S, T, F>, L>>
    where
        S: Schedule,
        T: Send + 'static,
        F: Fn(&mut T, &Time) + Send + 'static,
    {
        Robot {
            inner: self.inner.add_controller_as_thread(schedule, ctx, f),
        }
    }

    // Runtime check, not compile-time: `kmr_core::Robot::run` panics on an
    // empty controller set. Forwarding plainly here rather than gating this
    // method behind a `Runnable`-style bound, which would spill kmr_core's
    // internal paths/types into the caller's compile errors.
    pub fn run(self) -> Result<(), UserError> {
        self.inner.run().map_err(|e| e.into())
    }
}
