//! Home: ramps every actuator from where it is to a home pose, on a tick phase. Optional: a
//! robot needs no home (SRS FR-2.7).
//!
//! ```ignore
//! const HOME: Home<N> = Home::new([Q(0.0); N], Duration::from_secs(3));
//!
//! Robot::new()
//!     .add_controller(EachTick, HOME.then(sine_all))
//!     .run()
//! ```

use std::time::Duration;

use crate::{Q, Sensors, State, Time};

/// A home pose for every actuator and the ramp that reaches it, built in a `const` (SRS
/// FR-2.7).
#[derive(Clone, Copy, Debug)]
pub struct Home<const N: usize> {
    pose: [Q; N],
    ramp: Duration,
}

impl<const N: usize> Home<N> {
    /// Reaches `pose` after `ramp`; a zero `ramp` writes `pose` on the first tick (SRS
    /// FR-2.7).
    pub const fn new(pose: [Q; N], ramp: Duration) -> Self {
        let _ = (pose, ramp);
        todo!()
    }

    /// The preset as a controller for `add_controller`: from the first tick with a sample,
    /// writes the straight line from that sample to the pose, reached at the ramp, then the
    /// pose every tick. Never overshoots; the limit policy applies to its writes as to any
    /// controller's (SRS FR-2.7, FR-4.5).
    pub fn controller(self) -> impl Fn(&Time, &mut State<'_, N>, &Sensors) {
        let _ = self;
        |_time, _state, _sensors| todo!()
    }

    /// Homes as [`controller`](Self::controller) does, then hands every tick to `f`: from the
    /// first tick at or past the ramp, `f` runs and the preset writes nothing. `f`'s
    /// [`Time::elapsed`] and [`Time::tick`] start at zero on that tick; `dt` and the overrun
    /// counters are the robot's (SRS FR-2.7).
    pub fn then<F>(self, f: F) -> impl Fn(&Time, &mut State<'_, N>, &Sensors)
    where
        F: Fn(&Time, &mut State<'_, N>, &Sensors),
    {
        let _ = (self, f);
        |_time, _state, _sensors| todo!()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn fr_2_7_home_is_const() {
        todo!()
    }

    #[test]
    fn fr_2_7_starts_from_the_first_sample() {
        todo!()
    }

    #[test]
    fn fr_2_7_reaches_home_at_ramp() {
        todo!()
    }

    #[test]
    fn fr_2_7_never_overshoots() {
        todo!()
    }

    #[test]
    fn fr_2_7_zero_ramp_writes_home_on_the_first_tick() {
        todo!()
    }

    #[test]
    fn fr_2_7_holds_home_after_the_ramp() {
        todo!()
    }

    #[test]
    fn fr_2_7_then_waits_for_home() {
        todo!()
    }

    #[test]
    fn fr_2_7_then_writes_nothing_itself_once_home() {
        todo!()
    }

    #[test]
    fn fr_2_7_then_time_starts_at_zero() {
        todo!()
    }

    #[test]
    fn fr_4_5_limit_policy_applies_to_home() {
        todo!()
    }
}
