//! Module that holds the Time data structure.
//!
//! Allows the tracking of a fixed time duration as well as the real one.
//! Meant to be read-only for the user, and entirely managed within the core.
//!
//! Configurable at init time (see [`RobotConfig`]).
//!
//! ```ignore
//! // Runtime-side usage only — all methods are `pub(crate)`, unreachable from
//! // user code. The clock never reads the system time itself: the runtime
//! // injects every instant, which is what keeps the arithmetic testable.
//! let mut time = Time::default();
//! time.set_delta(Duration::from_millis(1)); // ~1kHz
//! time.arm(Instant::now()); // once, after the startup phases
//!
//! loop {
//!     time.begin_tick(Instant::now());
//!     // ... controllers read `time`, never write it ...
//!     if let Some(deadline) = time.end_tick(Instant::now()) {
//!         // on time: sleep until `deadline`
//!     } // else late: `time.overrun()` is set, start the next tick at once
//! }
//! ```
//!
//! # Tick grid
//!
//! Deadlines are ABSOLUTE: every one of them is `origin + k * delta`, where
//! `origin` is the instant given to `arm`. Sleeping for `delta` after each
//! tick would instead make the real period `work + delta + wake latency`, an
//! error that adds up tick after tick. With absolute deadlines a late wake-up
//! is jitter on one tick and is absorbed by the next sleep.
//!
//! After an overrun the grid is kept: the slots that were missed are skipped
//! and the next deadline is the first grid point still ahead. Neither of the
//! two alternatives is acceptable for a controller that integrates with `dt`:
//! a catch-up burst runs ticks with a real period far below `delta`, and
//! re-anchoring the grid at "now" shifts the schedule for good on each overrun.

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
    /// Absolute instant the tick in progress must be finished by, `None` until
    /// the runtime arms the clock. Always a point of the tick grid.
    deadline: Option<Instant>,
    /// Fixed tick duration, configured via `Robot::dt`.
    delta: Duration,
    /// Wall-clock duration since `startup`.
    elapsed: Duration,
    /// How late the last tick finished against its deadline, if it was late.
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
            deadline: None,
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
    /// Sets the overrun.
    ///
    /// Overrun is "how late" a tick is, measured against its absolute
    /// deadline and not against the time the tick spent working: a tick that
    /// started late because the previous sleep woke up late is late too.
    ///
    /// # Example
    ///
    /// If the deadline of the tick was 30ms after the grid origin and the tick
    /// finished at 35ms, then the overrun equals 35-30, or 5ms.
    ///
    /// <div class="warning">Overrun is not cumulative. The overrun is only for
    /// the current tick we're reading from.</div>
    ///
    /// To set the total overrun (sum of all ticks' overruns), use
    /// `self.increment_accumulated_overrun` instead.
    pub(crate) fn set_overrun(&mut self, overrun: Option<Duration>) {
        self.overrun = overrun;
    }
    /// Adds the overrun of the current tick to the accumulated overrun.
    ///
    /// Overrun is "how late" a tick is.
    /// It's the sum of all overruns.
    ///
    /// To set an overrun for the current tick, use `self.set_overrun` instead.
    pub(crate) fn increment_accumulated_overrun(&mut self) {
        if let Some(overrun) = self.overrun {
            // Saturating: a total that no longer fits must not panic the loop.
            let total = self.accumulated_overrun.unwrap_or(Duration::ZERO);
            self.accumulated_overrun = Some(total.saturating_add(overrun));
        }
    }

    // ── Tick grid ───────────────────────────────────────────────────────
    //
    // `now` is always injected. Reading the system clock in here would make
    // the grid arithmetic untestable without sleeping, and would tie `Time` to
    // the real clock, which `Time<Simulation>` must not be.

    /// Anchors the tick grid: the first tick is due one `delta` after `now`.
    ///
    /// Called once, after the startup phases, so that the time they took is
    /// not reported as an overrun of the first tick.
    pub(crate) fn arm(&mut self, now: Instant) {
        self.deadline = Some(now.checked_add(self.delta).unwrap_or(now));
    }

    /// Stamps the elapsed time, relative to `startup`, at the start of a tick.
    ///
    /// Stamped before the controllers run so that all of them read the same
    /// value during one tick.
    pub(crate) fn begin_tick(&mut self, now: Instant) {
        self.elapsed = now.saturating_duration_since(self.startup);
    }

    /// Closes the tick that finished at `now` and moves to the next deadline.
    ///
    /// Returns the instant to sleep until, or `None` if the tick was late and
    /// the next one must start at once.
    pub(crate) fn end_tick(&mut self, now: Instant) -> Option<Instant> {
        let Some(due) = self.deadline else {
            // Never armed: there is no grid to be late against yet.
            self.arm(now);
            return None;
        };
        self.last_update = Some(now);

        let late = now.saturating_duration_since(due);
        if late.is_zero() {
            self.overrun = None;
            self.deadline = Some(due.checked_add(self.delta).unwrap_or(due));
            return Some(due);
        }

        self.overrun = Some(late);
        self.increment_accumulated_overrun();

        // The runtime never lets `delta` be zero. Guarded anyway: the division
        // below would panic, and a panic here takes the control loop down.
        if self.delta.is_zero() {
            self.deadline = Some(now);
            return None;
        }
        // Whole slots already behind us. The next deadline is computed from
        // `due`, not from `now`, so it lands exactly on the grid.
        let missed = late.as_nanos() / self.delta.as_nanos();
        let slots = u32::try_from(missed).unwrap_or(u32::MAX).saturating_add(1);
        let step = self.delta.saturating_mul(slots);
        self.deadline = Some(due.checked_add(step).unwrap_or(now));
        None
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

    /// Returns the instant the tick in progress must be finished by, or
    /// `None` while the clock is not armed.
    pub(crate) fn deadline(&self) -> Option<&Instant> {
        self.deadline.as_ref()
    }

    /// Returns how late the last tick finished against its deadline, if it
    /// was late.
    ///
    /// Overrun is not cumulative; it only reflects the current tick. For the
    /// running total, use `accumulated_overrun` instead.
    pub(crate) fn overrun(&self) -> Option<&Duration> {
        self.overrun.as_ref()
    }

    /// Returns the running total of overrun across ticks, if any tick has
    /// overrun.
    pub(crate) fn accumulated_overrun(&self) -> Option<&Duration> {
        self.accumulated_overrun.as_ref()
    }
}

impl Time<Simulation> {
    // See the WIP list on `Simulation`'s doc comment.
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: Duration = Duration::from_millis(1);

    // Every instant is synthetic (`origin + offset`): nothing here sleeps or
    // compares against the system clock, so the results cannot depend on the
    // load of the machine running the tests.
    fn armed() -> (Time, Instant) {
        let origin = Instant::now();
        let mut time = Time::default();
        time.set_delta(DT);
        time.arm(origin);
        (time, origin)
    }

    #[test]
    fn on_time_tick_sleeps_until_its_deadline_and_advances_by_delta() {
        let (mut time, origin) = armed();
        let now = origin + DT / 4;

        assert_eq!(time.end_tick(now), Some(origin + DT));
        assert_eq!(time.deadline(), Some(&(origin + DT * 2)));
        assert_eq!(time.last_update(), Some(&now));
        assert_eq!(time.overrun(), None);
        assert_eq!(time.accumulated_overrun(), None);
    }

    #[test]
    fn finishing_exactly_on_the_deadline_is_on_time() {
        let (mut time, origin) = armed();

        assert_eq!(time.end_tick(origin + DT), Some(origin + DT));
        assert_eq!(time.overrun(), None);
    }

    #[test]
    fn late_by_less_than_delta_keeps_the_next_slot() {
        let (mut time, origin) = armed();
        let late = DT * 3 / 10;

        assert_eq!(time.end_tick(origin + DT + late), None);
        assert_eq!(time.overrun(), Some(&late));
        assert_eq!(time.deadline(), Some(&(origin + DT * 2)));
    }

    #[test]
    fn late_by_several_deltas_skips_the_missed_slots() {
        let (mut time, origin) = armed();
        let late = DT * 23 / 10;

        assert_eq!(time.end_tick(origin + DT + late), None);
        assert_eq!(time.overrun(), Some(&late));
        // due = 1, now = 3.3: slots 2 and 3 are gone, 4 is the next one.
        assert_eq!(time.deadline(), Some(&(origin + DT * 4)));
    }

    #[test]
    fn late_by_an_exact_multiple_gives_a_full_budget() {
        let (mut time, origin) = armed();

        // now = 3 is itself a grid point: the next tick gets a whole delta.
        assert_eq!(time.end_tick(origin + DT * 3), None);
        assert_eq!(time.deadline(), Some(&(origin + DT * 4)));
    }

    #[test]
    fn accumulated_overrun_sums_and_overrun_resets() {
        let (mut time, origin) = armed();

        assert_eq!(time.end_tick(origin + DT + DT / 2), None);
        assert_eq!(time.end_tick(origin + DT * 2 + DT / 4), None);
        assert_eq!(time.overrun(), Some(&(DT / 4)));
        assert_eq!(time.accumulated_overrun(), Some(&(DT / 2 + DT / 4)));

        // An on-time tick clears the per-tick value and keeps the total.
        assert_eq!(time.end_tick(origin + DT * 3), Some(origin + DT * 3));
        assert_eq!(time.overrun(), None);
        assert_eq!(time.accumulated_overrun(), Some(&(DT / 2 + DT / 4)));
    }

    #[test]
    fn late_wake_ups_never_move_the_grid() {
        let (mut time, origin) = armed();

        // Each tick starts a bit late, as after a sleep that overslept, and
        // one tick in ten overruns. Every deadline must stay on the grid.
        for k in 1..=1000u32 {
            let due = *time.deadline().expect("armed");
            let offset = due.duration_since(origin);
            assert_eq!(offset.as_nanos() % DT.as_nanos(), 0, "tick {k}");

            let finished = if k % 10 == 0 {
                due + DT * 17 / 10
            } else {
                due - DT / 3
            };
            let wake = time.end_tick(finished);
            assert_eq!(wake.is_none(), k % 10 == 0, "tick {k}");
        }
        // 100 overruns skipped one slot each: 1000 ticks used 1100 slots.
        assert_eq!(time.deadline(), Some(&(origin + DT * 1101)));
        assert_eq!(time.accumulated_overrun(), Some(&(DT * 17 / 10 * 100)));
    }

    #[test]
    fn unarmed_clock_arms_itself_and_reports_nothing() {
        let origin = Instant::now();
        let mut time = Time::default();
        time.set_delta(DT);

        assert_eq!(time.end_tick(origin), None);
        assert_eq!(time.deadline(), Some(&(origin + DT)));
        assert_eq!(time.last_update(), None);
        assert_eq!(time.overrun(), None);
    }

    #[test]
    fn zero_delta_does_not_panic() {
        let origin = Instant::now();
        let mut time = Time::default();
        time.arm(origin);

        assert_eq!(time.end_tick(origin + DT), None);
        assert_eq!(time.deadline(), Some(&(origin + DT)));
    }

    #[test]
    fn elapsed_is_measured_from_startup() {
        let mut time = Time::default();
        let startup = *time.startup();

        time.begin_tick(startup + DT * 5);
        assert_eq!(time.elapsed(), &(DT * 5));
    }
}
