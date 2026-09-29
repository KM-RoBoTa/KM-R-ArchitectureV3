//! Module that holds the Time data structure.
//!
//! Allows the tracking of a fixed time duration as well as the real one.
//! Meant to be read-only for the user, and entirely managed within the core.
//!
//! Configurable at init time (see [`RobotConfig`]).
//!
//! ```ignore
//! // Runtime-side usage only — all methods are `pub(crate)`, unreachable from
//! // user code. Each tick, the runtime stamps the new delta and elapsed time:
//! let mut time = Time::default();
//! time.set_delta_ms(1); // ~1kHz
//! time.set_last_update(Instant::now());
//! time.set_elapsed(*time.startup() - Instant::now());
//!
//! // Downstream (e.g. `last` phase) reads it back to compute overrun:
//! let target = *time.delta();
//! let actual = time.last_update().unwrap().elapsed();
//! if actual > target {
//!     time.set_overrun(actual - target);
//! }
//! ```

use std::{
    marker::PhantomData,
    time::{Duration, Instant},
};

/// Unit struct used as a flag for [`Time`].
///
/// It's goal is to specify whether the time is simulated for debug or
/// experimentation purposes.
/// If not, the time is assumed to be real time.
///
/// Simulated time allows for pause, steps, runtime manipulation and so on.
/// It is not meant to be used in real robotics condition but only, as the name
/// suggests, during simulation.
///
/// ```ignore
/// // Real time (default): ticks follow the system clock.
/// let real = Time::default();
///
/// // Simulated time: ticks are driven by the runtime, not the clock.
/// let sim = Time::<Simulation>::default();
/// ```
///
/// WIP:
/// - [ ] `step()` — advance simulated time by one tick without waiting on the
///   system clock
/// - [ ] `pause()` / `resume()` — freeze simulated time in place
/// - [ ] `Simulation` must derive `Debug`
#[derive(Debug)]
struct Simulation;

/// Represents either real or simulated time.
///
/// [`Time<Simulation>`] is the variant in which time is simulated. It allows
/// for more control over time at the cost of not being one to one with the
/// system clock anymore.
///
/// # Example
///
/// ```ignore
/// // Runtime-side usage only — see the module-level example above.
/// let time = Time::default();
/// ```
pub struct Time<T = ()> {
    // todo: initialization time for debug ?
    // todo: couple a Peeker::initialization_time(); be useful
    // for the user to debug without calling Robot::new() ?
    /// Instant the robot started, set once at construction.
    startup: Instant,
    /// Instant the previous tick completed, `None` before the first tick.
    last_update: Option<Instant>,
    /// Fixed tick duration, configured via `Robot::dt`.
    delta: Duration,
    /// Wall-clock duration since `startup`.
    elapsed: Duration,
    /// How far the last tick ran over `delta`, if it did.
    overrun: Option<Duration>,
    /// Running total of overrun across ticks, if any tick has overrun.
    accumulated_overrun: Option<Duration>,
    /// Marks whether this `Time` is real or [`Simulation`]-driven; occupies no space.
    _sim: PhantomData<T>,
}

impl Default for Time {
    fn default() -> Self {
        Self {
            startup: Instant::now(),
            last_update: None,
            delta: Duration::ZERO,
            elapsed: Duration::ZERO,
            overrun: None,
            accumulated_overrun: None,
            _sim: Default::default(),
        }
    }
}
// All methods are `pub(crate)`: `Time` is internal state. The only knob the
// user gets is `Robot::dt_ms`/`dt_us` (robot.rs), which forwards into
// `set_delta_ms`/`set_delta_us` below. Everything else here is written and
// read by the runtime, never by user code.
impl Time {
    // ── Setters ─────────────────────────────────────────────────────────

    /// Sets the last known update time, also known as the last tick time.
    pub(crate) fn set_last_update(&mut self, update_time: Instant) {
        self.last_update = Some(update_time);
    }
    /// Sets the delta.
    pub(crate) fn set_delta(&mut self, dt: Duration) {
        self.delta = dt;
    }
    /// Sets the elapsed time relative to the `startup` time.
    ///
    /// <div class="warning">This method is not meant to give you the elapsed
    /// relative to some other time. To do so, use `Instant::elapsed()` instead.
    /// </div>
    pub(crate) fn set_elapsed(&mut self) {
        self.elapsed = self.startup().elapsed();
    }
    /// Sets the overrun.
    ///
    /// Overrun is "how late" a tick is.
    ///
    /// # Example
    ///
    /// If `self.dt` equals 30ms, but the tick ran for 35ms,
    /// then the overrun equals 35-30, or 5ms.
    ///
    /// <div class="warning">Overrun is not cumulative. The overrun is only for
    /// the current tick we're reading from.</div>
    ///
    /// To set the total overrun (sum of all ticks' overruns), use
    /// `self.increment_accumulated_overrun` instead.
    pub(crate) fn set_overrun(&mut self, overrun: Option<Duration>) {
        self.overrun = overrun;
    }
    /// Sets the accumulated overrung.
    ///
    /// Overrun is "how late" a tick is.
    /// It's the sum of all overruns.
    ///
    /// To set an overrun for the current tick, use `self.set_overrun` instead.
    pub(crate) fn increment_accumulated_overrun(&mut self) {
        if let Some(acc) = self.overrun {
            self.accumulated_overrun = Some(acc);
        }
    }

    // ── Getters ─────────────────────────────────────────────────────────
    /// Returns the instant the robot started, set once at construction.
    pub(crate) fn startup(&self) -> &Instant {
        &self.startup
    }

    /// Returns the instant the previous tick completed, or `None` before the
    /// first tick.
    pub(crate) fn last_update(&self) -> Option<&Instant> {
        self.last_update.as_ref()
    }

    /// Returns the fixed tick duration, configured via `Robot::dt`.
    pub(crate) fn delta(&self) -> &Duration {
        &self.delta
    }

    /// Returns the wall-clock duration since `startup`.
    pub(crate) fn elapsed(&self) -> &Duration {
        &self.elapsed
    }

    /// Returns how far the last tick ran over `delta`, if it did.
    ///
    /// Overrun is not cumulative; it only reflects the current tick. For the
    /// running total, use `accumulated_overrun` instead.
    pub fn overrun(&self) -> Option<&Duration> {
        self.overrun.as_ref()
    }

    /// Returns the running total of overrun across ticks, if any tick has
    /// overrun.
    pub fn accumulated_overrun(&self) -> Option<&Duration> {
        self.accumulated_overrun.as_ref()
    }
}

impl Time<Simulation> {
    // See the WIP list on `Simulation`'s doc comment.
}
