//! Hot-plug of declared actuators: missing at start, lost or joining while the loop runs.
//!
//! Niche, and not part of the maintained surface: it may change or go without notice. Built
//! only with the `experimental` feature; bring it in with `use kmr_api::experimental::*`.

use std::time::Duration;

use crate::{HISTORY_DEPTH, Mask, Q, Robot, State, StateError, Time};

/// What a declared actuator that does not answer means, whether an absent one may join the
/// run, and what a failed bring-up health check means: set them with [`RobotExt`].
pub use kmr_core::hal::{HealthPolicy, JoinPolicy, MissingPolicy};

mod seal {
    /// Only kmr_api's types take the extension traits of [`super`].
    pub trait Sealed {}

    impl<const N: usize, H, C, E, SN> Sealed for crate::Robot<N, H, C, E, SN> {}

    impl<const N: usize> Sealed for crate::State<'_, N> {}
}

/// The presence and health policies on the robot builder.
// DECISION: `RobotExt`, on `Robot` only: `Bound` takes no setting.
pub trait RobotExt: seal::Sealed + Sized {
    /// What a declared actuator that does not answer, at open or at run time, means. Without
    /// it, [`MissingPolicy::Fail`]: bring-up fails. With [`MissingPolicy::Tolerate`] it is
    /// absent, logged once, and [`Presence::present`] leaves it out.
    fn set_missing_policy(self, policy: MissingPolicy) -> Self;

    /// Whether an absent declared actuator may join the run. Without it, [`JoinPolicy::Never`].
    /// It needs [`MissingPolicy::Tolerate`] to have any effect.
    fn set_join_policy(self, policy: JoinPolicy) -> Self;

    /// What an actuator whose bring-up health check fails (a hardware fault that does not
    /// clear, its voltage outside its limits, its temperature above its limit, a hardware error
    /// status) means. Without it, [`HealthPolicy::Fail`]: bring-up fails. With
    /// [`HealthPolicy::Warn`] it is logged once and set up and driven like the others; one
    /// whose own alarm cut its torque does not move. A bus error, a wrong model or limits that
    /// do not read back as written still fail. A joiner gets the same policy.
    fn set_health_policy(self, policy: HealthPolicy) -> Self;
}

impl<const N: usize, H, C, E, SN> RobotExt for Robot<N, H, C, E, SN> {
    fn set_missing_policy(self, policy: MissingPolicy) -> Self {
        self.map_core(|inner| inner.set_missing_policy(policy))
    }

    fn set_join_policy(self, policy: JoinPolicy) -> Self {
        self.map_core(|inner| inner.set_join_policy(policy))
    }

    fn set_health_policy(self, policy: HealthPolicy) -> Self {
        self.map_core(|inner| inner.set_health_policy(policy))
    }
}

/// Which actuators are present, as a controller reads them from its [`State`].
///
/// Every actuator is present unless [`MissingPolicy::Tolerate`] is set and one stopped
/// answering or never answered. A write to an absent actuator is skipped, not an error.
///
/// Reads of an absent actuator: one-actuator reads (`q_at`, `prev_q_at`, and the same for
/// `qd` and `tau`) give [`StateError::NoSample`](crate::StateError::NoSample), and `age_at`
/// gives `Ok(None)`, from the tick it left until its first read after it rejoins. The
/// whole-array reads (`q()`, `prev_q(..)`) leave it out of their check: its entry holds its
/// last sample, or zero if it was never read. With no actuator present at all, they give
/// `None` or `NoSample`, and `sample_age()` gives `None`: nothing is fresh. A group read that
/// includes it (`group(..).q()`) gives `None`.
pub trait Presence<const N: usize>: seal::Sealed {
    /// The actuators present this tick: read and written.
    fn present(&self) -> Mask<N>;

    /// Whether actuator `i` is present this tick. `false` for an index past N.
    fn present_at(&self, i: usize) -> bool;

    /// The actuators that became present at this tick, their first sample read. Empty at the
    /// start of a run: one present from bring-up on never joined.
    fn joined(&self) -> Mask<N>;

    /// The actuators that became absent at this tick.
    fn left(&self) -> Mask<N>;
}

impl<const N: usize> Presence<N> for State<'_, N> {
    #[inline]
    fn present(&self) -> Mask<N> {
        self.core().present()
    }

    #[inline]
    fn present_at(&self, i: usize) -> bool {
        self.core().present().get(i)
    }

    #[inline]
    fn joined(&self) -> Mask<N> {
        self.core().joined()
    }

    #[inline]
    fn left(&self) -> Mask<N> {
        self.core().left()
    }
}

/// Eases each joining actuator into the controller's motion: from where it was when it
/// joined to the controller's target, linearly, over a fixed duration.
///
/// ```no_run
/// # use std::time::Duration;
/// # use kmr_api::{Q, Sensors, Time};
/// # use kmr_api::experimental::JoinRamp;
/// # kmr_api::robot!(XmBench);
/// fn sway(ramp: &mut JoinRamp<N>, time: &Time, robot: &mut State, _: &Sensors) {
///     let mut target = [Q(0.0); N];
///     ramp.apply(time, robot, &mut target);
///     robot.set_all_q(target);
/// }
/// # fn main() {
/// #     let _ = kmr_api::with(JoinRamp::<N>::new(Duration::from_secs(1)), sway);
/// # }
/// ```
#[derive(Clone, Copy, Debug)]
pub struct JoinRamp<const N: usize> {
    /// How long a joiner takes to reach the controller's target.
    duration: Duration,
    /// Where each easing actuator was when it joined.
    from: [Q; N],
    /// When each easing actuator joined, on the controller's clock.
    since: [Duration; N],
    /// The actuators still easing.
    easing: Mask<N>,
    /// Every actuator taken as a joiner since the run's first call.
    joiners: Mask<N>,
    /// The actuators present at the last call.
    seen: Mask<N>,
    /// The tick of the last call this run; `None` before the first.
    last: Option<u64>,
}

impl<const N: usize> JoinRamp<N> {
    /// A ramp of `duration` per joiner, none easing yet.
    pub const fn new(duration: Duration) -> Self {
        Self {
            duration,
            from: [Q(0.0); N],
            since: [Duration::ZERO; N],
            easing: Mask::NONE,
            joiners: Mask::NONE,
            seen: Mask::NONE,
            last: None,
        }
    }

    /// Once per tick, before the writes: takes the actuators that joined since the last call
    /// and where they are now, then blends the target of each easing one from there towards
    /// `target`, in place. An actuator that leaves stops easing; one that joins again starts
    /// over from where it is then. The others' targets are untouched.
    ///
    /// A call may be skipped: a joiner is still taken at the next call, from where it is then.
    /// The first call of a run is the baseline, as for [`Presence::joined`]: the actuators
    /// present then never joined. A clock that goes back, a new run or a `Home::then` hand-over,
    /// starts a new baseline.
    // DECISION: a joiner without a sample at the call that takes it is not eased: there is no
    // start to blend from, so it takes the controller's target as it would without the ramp.
    pub fn apply(&mut self, time: &Time, robot: &State<'_, N>, target: &mut [Q; N]) {
        let now = time.elapsed();
        let joining = self.take_joiners(time.tick(), robot);
        self.start(joining, now, robot);
        self.blend(now, target);
    }

    /// The actuators that joined since the last call and are still present, noted as
    /// joiners. The first call of a run, or a clock that went back, is a new baseline:
    /// nothing eases and nobody joined yet.
    fn take_joiners(&mut self, tick: u64, robot: &State<'_, N>) -> Mask<N> {
        let present = robot.present();
        let mut joining = robot.joined();
        match self.last {
            Some(last) if tick >= last => {
                joining = joining
                    .union(present.difference(self.seen))
                    .union(self.rejoined(robot, tick - last));
            }
            _ => (self.easing, self.joiners) = (Mask::NONE, Mask::NONE),
        }
        (self.seen, self.last) = (present, Some(tick));
        let joining = joining.intersection(present);
        self.joiners = self.joiners.union(joining);
        joining
    }

    /// Starts the ease of each actuator of `joining` from where it is at `now`; one
    /// without a sample is not eased.
    fn start(&mut self, joining: Mask<N>, now: Duration, robot: &State<'_, N>) {
        for i in joining.iter() {
            let (Ok(at), Some(from), Some(since)) =
                (robot.q_at(i), self.from.get_mut(i), self.since.get_mut(i))
            else {
                self.easing.clear(i);
                continue;
            };
            (*from, *since) = (at, now);
            self.easing.set(i);
        }
    }

    /// Moves the target of each easing actuator to its point at `now` on the line from
    /// where it joined; one that left, or whose duration is over, stops easing.
    fn blend(&mut self, now: Duration, target: &mut [Q; N]) {
        let easing = self.easing;
        for i in easing.iter() {
            let (Some(&from), Some(&since), Some(goal)) =
                (self.from.get(i), self.since.get(i), target.get_mut(i))
            else {
                continue;
            };
            let age = now.saturating_sub(since);
            if !self.seen.get(i) || age >= self.duration {
                self.easing.clear(i);
                continue;
            }
            let s = age.as_secs_f32() / self.duration.as_secs_f32();
            *goal = Q(from.0 + (goal.0 - from.0) * s);
        }
    }

    /// The actuators present at the last call and now that left and came back in the `gap`
    /// ticks between: their samples start after the last call.
    // DECISION: told by the history, not by the presence masks, which show the same slot
    // present at both calls: a rejoin clears a slot's samples, so one that stayed holds one
    // `gap` ticks back and a rejoined one does not. A rejoin older than the history is missed;
    // it was driven at the controller's target since, as without the ramp.
    fn rejoined(&self, robot: &State<'_, N>, gap: u64) -> Mask<N> {
        let mut rejoined = Mask::NONE;
        if gap == 0 {
            return rejoined;
        }
        let depth =
            usize::try_from(gap).map_or(HISTORY_DEPTH - 1, |gap| gap.min(HISTORY_DEPTH - 1));
        for i in robot.present().intersection(self.seen).iter() {
            if matches!(robot.prev_q_at(i, depth), Err(StateError::NoSample { .. })) {
                rejoined.set(i);
            }
        }
        rejoined
    }

    /// The actuators still easing: their target is not the controller's yet.
    pub fn easing(&self) -> Mask<N> {
        self.easing
    }

    /// Every actuator that joined since the run's first call, easing or done. A controller
    /// that leaves the others to a preset, such as `Home` during its ramp, writes these
    /// itself: once eased in, a joiner stays on the controller's target, not back on the
    /// preset's line, which started from where it was before it left.
    // DECISION: `joiners`, kept for the whole run (a new baseline clears it), one taken
    // without a sample included: it is off the preset's line either way.
    pub fn joiners(&self) -> Mask<N> {
        self.joiners
    }
}
