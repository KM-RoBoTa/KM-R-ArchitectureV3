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

use core::cell::Cell;
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
        Self { pose, ramp }
    }

    /// The preset as a controller for `add_controller`: from the first tick with a sample,
    /// writes the straight line from that sample to the pose, reached at the ramp, then the
    /// pose every tick. Never overshoots; the limit policy applies to its writes as to any
    /// controller's (SRS FR-2.7, FR-4.5).
    pub fn controller(self) -> impl Fn(&Time, &mut State<'_, N>, &Sensors) {
        let phase = Cell::new(Phase::Waiting);
        move |time, state, _sensors| {
            if self.reached(&phase, time, state).is_some() {
                state.set_all_q(self.pose);
            }
        }
    }

    /// Homes as [`controller`](Self::controller) does, then hands every tick to `f`: from the
    /// first tick at or past the ramp, `f` runs; the preset writes the pose once on that tick,
    /// before `f`, and nothing after. `f`'s [`Time::elapsed`] and [`Time::tick`] start at zero
    /// on that tick; `dt` and the overrun counters are the robot's (SRS FR-2.7).
    pub fn then<F>(self, f: F) -> impl Fn(&Time, &mut State<'_, N>, &Sensors)
    where
        F: Fn(&Time, &mut State<'_, N>, &Sensors),
    {
        let phase = Cell::new(Phase::Waiting);
        move |time, state, sensors| {
            if let Some((elapsed, tick)) = self.reached(&phase, time, state) {
                let since = time.since(elapsed, tick);
                // DECISION: the hand-over tick writes the pose before `f`, so an actuator `f`
                // leaves alone ends on home, not one ramp step short of it.
                if since.tick() == 0 {
                    state.set_all_q(self.pose);
                }
                f(&since, state, sensors);
            }
        }
    }

    /// Writes this tick's point of the line while ramping; once the ramp is over, writes
    /// nothing and returns the `elapsed` and `tick` home was reached at.
    fn reached(
        &self,
        phase: &Cell<Phase<N>>,
        time: &Time,
        state: &mut State<'_, N>,
    ) -> Option<(Duration, u64)> {
        let (start, begun) = match phase.get() {
            Phase::Home { elapsed, tick } => return Some((elapsed, tick)),
            Phase::Ramping { start, begun } => (start, begun),
            Phase::Waiting => (state.q()?, time.elapsed()),
        };
        let t = time.elapsed().saturating_sub(begun);
        if t >= self.ramp {
            phase.set(Phase::Home {
                elapsed: time.elapsed(),
                tick: time.tick(),
            });
            return Some((time.elapsed(), time.tick()));
        }
        phase.set(Phase::Ramping { start, begun });
        let a = t.as_secs_f32() / self.ramp.as_secs_f32();
        state.set_all_q(core::array::from_fn(|i| lerp(start[i], self.pose[i], a)));
        None
    }
}

/// Where a preset's run is, in a `Cell` its closure captures (SRS FR-1.1).
#[derive(Clone, Copy)]
enum Phase<const N: usize> {
    /// No sample yet.
    Waiting,
    /// On the line from `start`, begun at `begun`.
    Ramping { start: [Q; N], begun: Duration },
    /// Home reached at `elapsed`, on tick `tick`.
    Home { elapsed: Duration, tick: u64 },
}

/// The point at `a`, in `[0, 1)`, of the line from `from` to `to`.
fn lerp(from: Q, to: Q, a: f32) -> Q {
    let (low, high) = (from.0.min(to.0), from.0.max(to.0));
    // DECISION: min/max, not `clamp`, which panics on a NaN read; they keep rounding from
    // taking the point past `to`.
    Q((from.0 + (to.0 - from.0) * a).max(low).min(high))
}

#[cfg(test)]
mod tests {
    use core::cell::RefCell;
    use std::time::{Duration, Instant};

    use kmr_core::test_util::{DT, Log, Recorder, Sent, Ticking, run_scripted};

    use super::Home;
    use crate::robot::tests::TestSpec;
    use crate::{EachTick, Filled, HCons, HNil, Policy, Q, Robot, Sensors, State, Time};

    /// The first sample: the recorder seeds 10, its read of tick 0 moves actuator 0 to 11.
    const START: [Q; 4] = [Q(11.0), Q(10.0), Q(10.0), Q(10.0)];
    const POSE: [Q; 4] = [Q(3.0), Q(6.0), Q(14.0), Q(10.0)];
    /// Four ticks of `DT`.
    const RAMP: Duration = Duration::from_millis(4);
    const HOME: Home<4> = Home::new(POSE, RAMP);

    /// A robot that passes every write through unchanged, with a warning.
    fn robot() -> Robot<4, TestSpec> {
        Robot::new().set_dt(DT).set_limit_policy_all(Policy::Warn)
    }

    /// What the adapter was sent on the last of `ticks` ticks.
    fn last_sent<C: kmr_core::Drive<4>>(
        robot: Robot<4, TestSpec, C, Filled>,
        ticks: u64,
    ) -> Sent<4> {
        let log = Log::new();
        let mut inner = robot.finalized();
        let mut host = Ticking::new(Instant::now(), ticks);
        let mut drivers = HCons {
            head: Recorder::new(&log),
            tail: HNil,
        };
        assert!(run_scripted(&mut inner, &mut host, &mut drivers).is_ok());
        log.write.get().expect("the last tick wrote")
    }

    fn q(values: [f32; 4]) -> [Q; 4] {
        values.map(Q)
    }

    #[test]
    fn fr_2_7_home_is_const() {
        const STILL: Home<4> = Home::new(POSE, Duration::ZERO);
        let copy = STILL;
        assert!(format!("{copy:?}").contains("ramp: 0ns"), "{copy:?}");
    }

    #[test]
    fn fr_2_7_starts_from_the_first_sample() {
        assert_eq!(
            last_sent(robot().add_controller(EachTick, HOME.controller()), 1).q,
            START
        );
        // Actuator 0 reads 12 on tick 1; the line still runs from 11.
        let sent = last_sent(robot().add_controller(EachTick, HOME.controller()), 2);
        assert_eq!(sent.q, q([9.0, 9.0, 11.0, 10.0]));
    }

    #[test]
    fn fr_2_7_reaches_home_at_ramp() {
        let before = last_sent(robot().add_controller(EachTick, HOME.controller()), 4);
        assert_eq!(before.q, q([5.0, 7.0, 13.0, 10.0]));
        let at = last_sent(robot().add_controller(EachTick, HOME.controller()), 5);
        assert_eq!(at.q, POSE);
    }

    #[test]
    fn fr_2_7_never_overshoots() {
        let mut previous = START;
        for ticks in 1..=8 {
            let sent = last_sent(robot().add_controller(EachTick, HOME.controller()), ticks).q;
            for i in 0..4 {
                let (low, high) = (START[i].0.min(POSE[i].0), START[i].0.max(POSE[i].0));
                assert!((low..=high).contains(&sent[i].0), "tick {ticks}: {sent:?}");
                assert!((sent[i].0 - POSE[i].0).abs() <= (previous[i].0 - POSE[i].0).abs());
            }
            previous = sent;
        }
    }

    #[test]
    fn fr_2_7_zero_ramp_writes_home_on_the_first_tick() {
        let now = Home::new(POSE, Duration::ZERO);
        assert_eq!(
            last_sent(robot().add_controller(EachTick, now.controller()), 1).q,
            POSE
        );
    }

    #[test]
    fn fr_2_7_holds_home_after_the_ramp() {
        for ticks in [6, 7] {
            let sent = last_sent(robot().add_controller(EachTick, HOME.controller()), ticks);
            assert_eq!(sent.q, POSE);
            assert_eq!(sent.written.q.iter().collect::<Vec<_>>(), [0, 1, 2, 3]);
        }
    }

    #[test]
    fn fr_2_7_then_waits_for_home() {
        for (ticks, calls) in [(4, 0), (6, 2)] {
            let ran = RefCell::new(0);
            let count = |_: &Time, _: &mut State<'_, 4>, _: &Sensors| *ran.borrow_mut() += 1;
            last_sent(robot().add_controller(EachTick, HOME.then(count)), ticks);
            assert_eq!(ran.into_inner(), calls, "{ticks} ticks");
        }
    }

    #[test]
    fn fr_2_7_then_ends_on_home_for_actuators_f_leaves_alone() {
        let first = |_: &Time, state: &mut State<'_, 4>, _: &Sensors| {
            state.set_q_at(Q(5.0), 0).expect("in range");
        };
        let sent = last_sent(robot().add_controller(EachTick, HOME.then(first)), 5);
        assert_eq!(sent.q, q([5.0, 6.0, 14.0, 10.0]));
    }

    #[test]
    fn fr_2_7_then_writes_nothing_itself_after_the_hand_over() {
        let first = |_: &Time, state: &mut State<'_, 4>, _: &Sensors| {
            state.set_q_at(Q(5.0), 0).expect("in range");
        };
        let sent = last_sent(robot().add_controller(EachTick, HOME.then(first)), 6);
        assert_eq!(sent.written.q.iter().collect::<Vec<_>>(), [0]);
    }

    #[test]
    fn fr_2_7_then_time_starts_at_zero() {
        let seen = RefCell::new(Vec::new());
        let clock = |time: &Time, _: &mut State<'_, 4>, _: &Sensors| {
            seen.borrow_mut()
                .push((time.tick(), time.elapsed(), time.dt()));
        };
        last_sent(robot().add_controller(EachTick, HOME.then(clock)), 6);
        assert_eq!(seen.into_inner(), [(0, Duration::ZERO, DT), (1, DT, DT)]);
    }

    #[test]
    fn fr_4_5_limit_policy_applies_to_home() {
        // TestSpec's limits are [-1, 1]: the start pose reaches the adapter clamped.
        let clamped = Robot::<4, TestSpec>::new()
            .set_dt(DT)
            .set_limit_policy_all(Policy::Clamp)
            .add_controller(EachTick, HOME.controller());
        assert_eq!(last_sent(clamped, 1).q, [Q(1.0); 4]);
    }
}
