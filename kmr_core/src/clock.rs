//! Module that holds the Time data structure.
//!
//! Allows the tracking of a fixed time duration as well as the real one.
//! Read-only for the user through the getters of SRS FR-3.3, and entirely
//! managed within the core.
//!
//! Configurable at init time (see [`RobotConfig`]).
//!
//! ```ignore
//! // Runtime side only: every method but the read-only getters is `pub(crate)`.
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

use core::marker::PhantomData;
use core::ops::{Add, AddAssign, Sub};
use core::time::Duration;

/// A point on the monotonic clock of the host, as its distance from the
/// host's epoch. The engine's own, so the core needs no `std` (SRS NFR-7):
/// a host without `std` builds it from its timer with [`Instant::from_epoch`].
#[derive(Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct Instant(Duration);

impl Instant {
    /// The host's epoch.
    pub const EPOCH: Self = Self(Duration::ZERO);

    /// The instant `since` after the host's epoch.
    #[must_use]
    pub const fn from_epoch(since: Duration) -> Self {
        Self(since)
    }

    /// The distance from the host's epoch.
    #[must_use]
    pub const fn since_epoch(self) -> Duration {
        self.0
    }

    /// The current instant of the system's monotonic clock, from an epoch
    /// set on the first call.
    #[cfg(feature = "std")]
    #[must_use]
    pub fn now() -> Self {
        Self::from(std::time::Instant::now())
    }

    /// `self + duration`, `None` on overflow.
    #[must_use]
    pub const fn checked_add(self, duration: Duration) -> Option<Self> {
        match self.0.checked_add(duration) {
            Some(since) => Some(Self(since)),
            None => None,
        }
    }

    /// `self - duration`, `None` before the epoch.
    #[must_use]
    pub const fn checked_sub(self, duration: Duration) -> Option<Self> {
        match self.0.checked_sub(duration) {
            Some(since) => Some(Self(since)),
            None => None,
        }
    }

    /// The time from `earlier` to `self`, zero if `earlier` is later.
    #[must_use]
    pub const fn saturating_duration_since(self, earlier: Self) -> Duration {
        self.0.saturating_sub(earlier.0)
    }

    /// The time from `earlier` to `self`, zero if `earlier` is later, as
    /// `std::time::Instant::duration_since`.
    #[must_use]
    pub const fn duration_since(self, earlier: Self) -> Duration {
        self.saturating_duration_since(earlier)
    }
}

/// The system clock's instants on one epoch: the first one converted. An
/// instant before it maps to the epoch.
#[cfg(feature = "std")]
impl From<std::time::Instant> for Instant {
    fn from(instant: std::time::Instant) -> Self {
        static EPOCH: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
        let epoch = *EPOCH.get_or_init(|| instant);
        Self(instant.saturating_duration_since(epoch))
    }
}

impl Add<Duration> for Instant {
    type Output = Self;

    // DECISION: saturates where `std::time::Instant` panics: no panic path
    // in the core (SRS NFR-2), and no instant is that far from the epoch.
    /// Saturates at the last representable instant.
    fn add(self, duration: Duration) -> Self {
        Self(self.0.saturating_add(duration))
    }
}

impl AddAssign<Duration> for Instant {
    fn add_assign(&mut self, duration: Duration) {
        *self = *self + duration;
    }
}

impl Sub<Duration> for Instant {
    type Output = Self;

    /// Saturates at the epoch.
    fn sub(self, duration: Duration) -> Self {
        Self(self.0.saturating_sub(duration))
    }
}

impl Sub for Instant {
    type Output = Duration;

    /// Zero if `earlier` is later, as `std::time::Instant`.
    fn sub(self, earlier: Self) -> Duration {
        self.saturating_duration_since(earlier)
    }
}

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
    /// Origin of the tick grid, set by `arm`. `None` until the clock is armed.
    origin: Option<Instant>,
    /// Instant the previous tick completed, `None` before the first tick.
    last_update: Option<Instant>,
    /// Instant the tick in progress must finish by, on the grid. `None` until
    /// the clock is armed.
    deadline: Option<Instant>,
    /// Fixed tick duration, configured via `Robot::dt`.
    delta: Duration,
    /// Wall-clock duration since `origin`, stamped at the start of each tick.
    elapsed: Duration,
    /// Ticks closed so far: the index of the tick in progress.
    tick: u64,
    /// Ticks that finished after their deadline.
    overrun_count: u64,
    /// How late the last tick finished against its deadline, if it was late.
    overrun: Option<Duration>,
    /// Running total of overrun across ticks, zero while no tick has overrun.
    accumulated_overrun: Duration,
    /// Marks whether this `Time` is real or [`Simulation`]-driven; occupies no space.
    _sim: PhantomData<T>,
}

impl Default for Time {
    fn default() -> Self {
        Self {
            origin: None,
            last_update: None,
            deadline: None,
            delta: Duration::ZERO,
            elapsed: Duration::ZERO,
            tick: 0,
            overrun_count: 0,
            overrun: None,
            accumulated_overrun: Duration::ZERO,
            _sim: Default::default(),
        }
    }
}
// The methods of this block are `pub(crate)`: `Time` is internal state. The
// only knob the user gets is `Robot::set_dt` (robot.rs), which forwards into
// `set_delta` below. Users read the clock through the `pub` getters further
// down, by copy.
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
            self.accumulated_overrun = self.accumulated_overrun.saturating_add(overrun);
        }
    }

    // ── Tick grid ───────────────────────────────────────────────────────

    /// Anchors the tick grid: the first tick is due one `delta` after `now`.
    ///
    /// Called once, after the startup phases, so they never count as an overrun.
    pub(crate) fn arm(&mut self, now: Instant) {
        self.origin = Some(now);
        self.deadline = Some(now.checked_add(self.delta).unwrap_or(now));
    }

    /// Stamps the elapsed time since `arm`, before the controllers run, so
    /// that they all read the same value during a tick. Zero while unarmed.
    pub(crate) fn begin_tick(&mut self, now: Instant) {
        self.elapsed = self.origin.map_or(Duration::ZERO, |origin| {
            now.saturating_duration_since(origin)
        });
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
        self.tick = self.tick.saturating_add(1);

        let late = now.saturating_duration_since(due);
        if late.is_zero() {
            self.overrun = None;
            self.deadline = Some(due.checked_add(self.delta).unwrap_or(due));
            return Some(due);
        }

        self.overrun = Some(late);
        self.overrun_count = self.overrun_count.saturating_add(1);
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
    /// Returns the instant the previous tick completed, or `None` before the
    /// first tick.
    pub(crate) fn last_update(&self) -> Option<&Instant> {
        self.last_update.as_ref()
    }

    /// Returns the fixed tick duration, configured via `Robot::dt`.
    pub(crate) fn delta(&self) -> &Duration {
        &self.delta
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
}

/// What a controller reads, by copy (SRS FR-3.3).
impl Time {
    /// Time since the tick grid was armed, stamped at the start of the tick.
    pub fn elapsed(&self) -> Duration {
        self.elapsed
    }

    /// The fixed tick duration.
    pub fn dt(&self) -> Duration {
        self.delta
    }

    /// Index of the tick in progress, 0 for the first one.
    pub fn tick(&self) -> u64 {
        self.tick
    }

    /// How many ticks finished after their deadline.
    pub fn overrun_count(&self) -> u64 {
        self.overrun_count
    }

    /// How late the previous tick finished, `None` if it was on time.
    pub fn last_overrun(&self) -> Option<Duration> {
        self.overrun
    }

    /// Sum of every overrun so far, zero when there was none.
    pub fn accumulated_overrun(&self) -> Duration {
        self.accumulated_overrun
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
        assert_eq!(time.accumulated_overrun(), Duration::ZERO);
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
        assert_eq!(time.accumulated_overrun(), DT / 2 + DT / 4);

        assert_eq!(time.end_tick(origin + DT * 5), Some(origin + DT * 5));
        assert_eq!(time.overrun(), None);
        assert_eq!(time.accumulated_overrun(), DT / 2 + DT / 4);
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
        assert_eq!(time.accumulated_overrun(), DT * 17 / 10 * 100);
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
        assert_eq!(time.accumulated_overrun(), DT / 2);
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
    fn elapsed_is_measured_from_arm() {
        let mut time = Time::default();
        let origin = Instant::now();

        time.begin_tick(origin + DT * 5);
        assert_eq!(time.elapsed(), Duration::ZERO);

        time.arm(origin);
        time.begin_tick(origin + DT * 5);
        assert_eq!(time.elapsed(), DT * 5);
    }

    #[test]
    fn tick_and_overrun_count_follow_end_tick() {
        let (mut time, origin) = armed();
        assert_eq!((time.tick(), time.overrun_count()), (0, 0));

        time.end_tick(origin + DT / 2);
        assert_eq!((time.tick(), time.overrun_count()), (1, 0));
        assert_eq!(time.last_overrun(), None);

        time.end_tick(origin + DT * 2 + DT / 4);
        assert_eq!((time.tick(), time.overrun_count()), (2, 1));
        assert_eq!(time.last_overrun(), Some(DT / 4));

        time.end_tick(origin + DT * 3);
        assert_eq!((time.tick(), time.overrun_count()), (3, 1));
        assert_eq!(time.last_overrun(), None);
        assert_eq!(time.accumulated_overrun(), DT / 4);
    }

    #[test]
    fn instant_arithmetic_saturates_instead_of_panicking() {
        let late = Instant::from_epoch(Duration::MAX);
        assert_eq!(late + DT, late);
        assert_eq!(Instant::EPOCH - DT, Instant::EPOCH);
        assert_eq!(Instant::EPOCH - late, Duration::ZERO);
        assert_eq!(late.checked_add(DT), None);
        assert_eq!(Instant::EPOCH.checked_sub(DT), None);
    }

    #[test]
    fn system_instants_keep_their_order_and_distance() {
        // Ahead of now: another test may set the shared epoch meanwhile.
        let first = std::time::Instant::now() + Duration::from_secs(60);
        let second = first + DT;
        let (a, b) = (Instant::from(first), Instant::from(second));
        assert!(a < b);
        assert_eq!(b - a, DT);
    }
}
