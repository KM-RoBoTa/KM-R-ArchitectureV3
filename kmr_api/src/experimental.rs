//! Hot-plug of declared actuators: missing at start, lost or joining while the loop runs.
//!
//! Niche, and not part of the maintained surface: it may change or go without notice. Built
//! only with the `experimental` feature; bring it in with `use kmr_api::experimental::*`.

use std::time::Duration;

use crate::{Mask, Q, Robot, State, Time};

/// What a declared actuator that does not answer means, and whether an absent one may join
/// the run: set them with [`RobotExt`].
pub use kmr_core::hal::{JoinPolicy, MissingPolicy};

mod seal {
    /// Only kmr_api's types take the extension traits of [`super`].
    pub trait Sealed {}

    impl<const N: usize, H, C, E, SN> Sealed for crate::Robot<N, H, C, E, SN> {}

    impl<const N: usize> Sealed for crate::State<'_, N> {}
}

/// The presence policies on the robot builder.
// DECISION: `RobotExt`, on `Robot` only: `Bound` takes no setting.
pub trait RobotExt: seal::Sealed + Sized {
    /// What a declared actuator that does not answer, at open or at run time, means. Without
    /// it, [`MissingPolicy::Fail`]: bring-up fails. With [`MissingPolicy::Tolerate`] it is
    /// absent, logged once, and [`Presence::present`] leaves it out.
    fn set_missing_policy(self, policy: MissingPolicy) -> Self;

    /// Whether an absent declared actuator may join the run. Without it, [`JoinPolicy::Never`].
    /// It needs [`MissingPolicy::Tolerate`] to have any effect.
    fn set_join_policy(self, policy: JoinPolicy) -> Self;
}

impl<const N: usize, H, C, E, SN> RobotExt for Robot<N, H, C, E, SN> {
    fn set_missing_policy(self, policy: MissingPolicy) -> Self {
        self.map_core(|inner| inner.set_missing_policy(policy))
    }

    fn set_join_policy(self, policy: JoinPolicy) -> Self {
        self.map_core(|inner| inner.set_join_policy(policy))
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
}

impl<const N: usize> JoinRamp<N> {
    /// A ramp of `duration` per joiner, none easing yet.
    pub const fn new(duration: Duration) -> Self {
        Self {
            duration,
            from: [Q(0.0); N],
            since: [Duration::ZERO; N],
            easing: Mask::NONE,
        }
    }

    /// Once per tick, before the writes: notes the actuators that joined at this tick and
    /// where they are, then blends the target of each easing one from there towards
    /// `target`, in place. An actuator that leaves stops easing; one that joins again starts
    /// over from where it is then. The others' targets are untouched.
    // DECISION: a joiner without a sample at its join tick is not eased: there is no start to
    // blend from, so it takes the controller's target as it would without the ramp.
    pub fn apply(&mut self, time: &Time, robot: &State<'_, N>, target: &mut [Q; N]) {
        let now = time.elapsed();
        for i in robot.joined().iter() {
            let (Ok(at), Some(from), Some(since)) =
                (robot.q_at(i), self.from.get_mut(i), self.since.get_mut(i))
            else {
                self.easing.clear(i);
                continue;
            };
            (*from, *since) = (at, now);
            self.easing.set(i);
        }
        let easing = self.easing;
        for i in easing.iter() {
            let (Some(&from), Some(&since), Some(goal)) =
                (self.from.get(i), self.since.get(i), target.get_mut(i))
            else {
                continue;
            };
            let age = now.saturating_sub(since);
            if !robot.present_at(i) || age >= self.duration {
                self.easing.clear(i);
                continue;
            }
            let s = age.as_secs_f32() / self.duration.as_secs_f32();
            *goal = Q(from.0 + (goal.0 - from.0) * s);
        }
    }

    /// The actuators still easing: their target is not the controller's yet.
    pub fn easing(&self) -> Mask<N> {
        self.easing
    }
}
