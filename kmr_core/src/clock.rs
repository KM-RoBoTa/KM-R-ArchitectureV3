//! Module that holds the Time data structure.
//!
//! Allows the tracking of a fixed time duration as well as the real one.
//! Meant to be read-only for the user, and entirely managed within the core.
//!
//! Configurable at init time (see [`RobotConfig`]).
//!
//! ```ignore
//! // Runtime side only: every method is `pub(crate)`.
//! let mut time = Time::default();
//! time.set_delta(Duration::from_millis(1)); // ~1kHz
//! time.arm(Instant::now()); // once, after the startup phases
//!
//! loop {
//!     time.begin_tick(Instant::now());
//!     // ... controllers read `time` ...
//!     if let Some(start) = time.end_tick(Instant::now()) {
//!         // sleep until `start`
//!     }
//! }
//! ```
//!
//! # Tick grid
//!
//! Deadlines are absolute, `origin + k * delta`, so the schedule does not
//! drift. The runtime injects every instant; the clock never reads the system
//! time. After an overrun the missed slots are skipped and the next tick
//! starts on the grid with a whole `delta`: no catch-up burst, no re-anchoring.

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
    /// Instant the tick in progress must finish by, on the grid. `None` until
    /// the clock is armed.
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
    /// Overrun is "how late" a tick is, measured against its deadline.
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
            let total = self.accumulated_overrun.unwrap_or(Duration::ZERO);
            self.accumulated_overrun = Some(total.saturating_add(overrun));
        }
    }

    // ── Tick grid ───────────────────────────────────────────────────────

    /// Anchors the tick grid: the first tick is due one `delta` after `now`.
    ///
    /// Called once, after the startup phases, so they never count as an overrun.
    pub(crate) fn arm(&mut self, now: Instant) {
        self.deadline = Some(now.checked_add(self.delta).unwrap_or(now));
    }

    /// Stamps the elapsed time since `startup`, before the controllers run, so
    /// that they all read the same value during a tick.
    pub(crate) fn begin_tick(&mut self, now: Instant) {
        self.elapsed = now.saturating_duration_since(self.startup);
    }

    /// Closes the tick that finished at `now`.
    ///
    /// Returns the grid point the next tick starts on, never before `now`.
    /// `None` when there is no grid: clock not armed, or `delta` of zero.
    pub(crate) fn end_tick(&mut self, now: Instant) -> Option<Instant> {
        let Some(due) = self.deadline else {
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

        if self.delta.is_zero() {
            self.deadline = Some(now);
            return None;
        }
        // Rounded up from `due`: the next tick starts on the grid, not mid-slot.
        let slots = late.as_nanos().div_ceil(self.delta.as_nanos());
        let slots = u32::try_from(slots).unwrap_or(u32::MAX);
        let start = due
            .checked_add(self.delta.saturating_mul(slots))
            .unwrap_or(now);
        self.deadline = Some(start.checked_add(self.delta).unwrap_or(start));
        Some(start)
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
    fn late_by_less_than_delta_starts_the_next_tick_on_the_next_grid_point() {
        let (mut time, origin) = armed();
        let late = DT * 3 / 10;

        assert_eq!(time.end_tick(origin + DT + late), Some(origin + DT * 2));
        assert_eq!(time.overrun(), Some(&late));
        assert_eq!(time.deadline(), Some(&(origin + DT * 3)));
    }

    #[test]
    fn late_by_several_deltas_skips_the_missed_slots() {
        let (mut time, origin) = armed();
        let late = DT * 23 / 10;

        assert_eq!(time.end_tick(origin + DT + late), Some(origin + DT * 4));
        assert_eq!(time.overrun(), Some(&late));
        assert_eq!(time.deadline(), Some(&(origin + DT * 5)));
    }

    #[test]
    fn late_by_an_exact_multiple_gives_a_full_budget() {
        let (mut time, origin) = armed();

        assert_eq!(time.end_tick(origin + DT * 3), Some(origin + DT * 3));
        assert_eq!(time.overrun(), Some(&(DT * 2)));
        assert_eq!(time.deadline(), Some(&(origin + DT * 4)));
    }

    #[test]
    fn accumulated_overrun_sums_and_overrun_resets() {
        let (mut time, origin) = armed();

        assert_eq!(time.end_tick(origin + DT + DT / 2), Some(origin + DT * 2));
        assert_eq!(
            time.end_tick(origin + DT * 3 + DT / 4),
            Some(origin + DT * 4)
        );
        assert_eq!(time.overrun(), Some(&(DT / 4)));
        assert_eq!(time.accumulated_overrun(), Some(&(DT / 2 + DT / 4)));

        assert_eq!(time.end_tick(origin + DT * 5), Some(origin + DT * 5));
        assert_eq!(time.overrun(), None);
        assert_eq!(time.accumulated_overrun(), Some(&(DT / 2 + DT / 4)));
    }

    #[test]
    fn late_wake_ups_never_move_the_grid() {
        let (mut time, origin) = armed();

        for k in 1..=1000u32 {
            let due = *time.deadline().expect("armed");
            let offset = due.duration_since(origin);
            assert_eq!(offset.as_nanos() % DT.as_nanos(), 0, "tick {k}");

            let finished = if k % 10 == 0 {
                due + DT * 17 / 10
            } else {
                due - DT / 3
            };
            let start = time.end_tick(finished).expect("armed, delta not zero");
            let next_due = *time.deadline().expect("armed");

            assert!(start >= finished, "tick {k}");
            assert_eq!(next_due, start + DT, "tick {k}");
            assert_eq!(time.overrun().is_some(), k % 10 == 0, "tick {k}");
        }
        // 100 overruns skip 2 slots each: 1000 ticks use 1200 slots.
        assert_eq!(time.deadline(), Some(&(origin + DT * 1201)));
        assert_eq!(time.accumulated_overrun(), Some(&(DT * 17 / 10 * 100)));
    }

    #[test]
    fn one_stall_is_one_overrun_for_a_workload_that_fits_in_delta() {
        let (mut time, origin) = armed();
        let work = DT * 8 / 10;
        let stall = DT * 3 / 2;

        let mut start = origin + DT;
        assert_eq!(time.end_tick(origin + work), Some(start));

        let mut late_ticks = 0;
        for k in 0..10u32 {
            let spent = if k == 0 { stall } else { work };
            let next = time.end_tick(start + spent).expect("armed, delta not zero");
            late_ticks += u32::from(time.overrun().is_some());

            assert!(next.duration_since(start) >= DT, "tick {k}");
            start = next;
        }
        assert_eq!(late_ticks, 1);
        assert_eq!(time.accumulated_overrun(), Some(&(DT / 2)));
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
