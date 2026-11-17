//! Which actuators are present, as their adapters report them after each
//! read, and which of them joined or left at this tick.

use crate::hal::Mask;

use super::RobotState;

/// The presence of the actuators over one run.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Presence<const N: usize> {
    /// Present now.
    now: Mask<N>,
    /// Present now and absent at the previous report.
    joined: Mask<N>,
    /// Absent now and present at the previous report.
    left: Mask<N>,
    /// Whether this run's first report came in.
    started: bool,
}

impl<const N: usize> Presence<N> {
    /// Every actuator present, nothing joined or left, no report yet.
    pub(crate) const fn new() -> Self {
        Self {
            now: Mask::ALL,
            joined: Mask::NONE,
            left: Mask::NONE,
            started: false,
        }
    }

    /// The actuators present now.
    pub(crate) const fn now(&self) -> Mask<N> {
        self.now
    }

    /// Takes one report: what joined and left since the previous one.
    // DECISION: the first report of a run, PostInit's full read, is the
    // baseline: an actuator present from the start never joined.
    fn report(&mut self, present: Mask<N>) {
        if self.started {
            self.joined = present.difference(self.now);
            self.left = self.now.difference(present);
        }
        self.started = true;
        self.now = present;
    }
}

impl<const N: usize> RobotState<N> {
    /// The actuators present now: read and driven. An absent one keeps its
    /// writes out of the command and has no sample.
    pub fn present(&self) -> Mask<N> {
        self.presence.now
    }

    /// The actuators that became present at this tick. None at the first
    /// presence of a run, PostInit's: one present from the start never
    /// joined.
    pub fn joined(&self) -> Mask<N> {
        self.presence.joined
    }

    /// The actuators that became absent at this tick.
    pub fn left(&self) -> Mask<N> {
        self.presence.left
    }

    /// Takes the adapters' presence after a read. An actuator that left
    /// forgets its samples, so a rejoin is seeded by its first read.
    pub(crate) fn set_present(&mut self, present: Mask<N>) {
        for index in self.presence.now.difference(present).iter() {
            if let Some(held) = self.sampled.get_mut(index) {
                *held = 0;
            }
            if let Some(age) = self.age.get_mut(index) {
                *age = 0;
            }
        }
        self.presence.report(present);
    }

    /// Clears the written bits of absent actuators, before the limit stage
    /// and the batch see them.
    pub(crate) fn drop_absent(&mut self) {
        // DECISION: nothing to cut while every actuator is present, so the
        // path from `Last` to the end stamp stays as short as without presence.
        let present = self.presence.now;
        if present != Mask::ALL {
            self.desired.written = self.desired.written.within(present);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::RobotState;
    use crate::error::StateError;
    use crate::hal::Mask;
    use crate::state::{N, Q, Qd, Tau};

    /// Every actuator but `index`.
    fn all_but(index: usize) -> Mask<N> {
        let mut mask = Mask::ALL;
        mask.clear(index);
        mask
    }

    /// Actuator 0 alone.
    fn first_only() -> Mask<N> {
        let mut mask = Mask::NONE;
        mask.set(0);
        mask
    }

    #[test]
    fn all_present_by_default() {
        assert_eq!(RobotState::<N>::default().present(), Mask::ALL);
    }

    #[test]
    fn fr_2_3_prev_ignores_absent_slots_never_sampled() {
        let mut robot = RobotState::<N>::default();
        let _ = robot.push_slot();
        robot.refreshed(all_but(2));
        robot.set_present(all_but(2));

        assert!(robot.prev::<Q>(0).is_ok());
        assert!(robot.current::<Q>().is_some());
        assert_eq!(
            robot.prev_at::<Q>(0, 2),
            Err(StateError::NoSample { depth: 0 })
        );
    }

    #[test]
    fn sample_age_ignores_absent_slots() {
        let mut robot = RobotState::<N>::default();
        let _ = robot.push_slot();
        robot.refreshed(all_but(2));
        robot.set_present(all_but(2));
        let _ = robot.push_slot();
        robot.refreshed(first_only());
        robot.set_present(all_but(2));

        assert_eq!(robot.sample_age(), Some(1));
    }

    #[test]
    fn a_slot_going_absent_forgets_its_samples() {
        let mut robot = RobotState::<N>::default();
        for _ in 0..2 {
            let _ = robot.push_slot();
            robot.refreshed(Mask::ALL);
        }
        let _ = robot.push_slot();
        robot.refreshed(all_but(1));
        robot.set_present(all_but(1));

        assert!(!robot.present().get(1));
        assert_eq!(robot.age(1), Ok(None));
        assert_eq!(
            robot.prev_at::<Q>(0, 1),
            Err(StateError::NoSample { depth: 0 })
        );
        assert_eq!(
            robot.prev_at::<Q>(1, 1),
            Err(StateError::NoSample { depth: 1 })
        );
        assert_eq!(robot.age(0), Ok(Some(0)));
        assert!(robot.prev::<Q>(2).is_ok());
    }

    #[test]
    fn an_absent_slot_stays_unsampled_while_the_others_are_read() {
        let mut robot = RobotState::<N>::default();
        let _ = robot.push_slot();
        robot.refreshed(Mask::ALL);
        robot.set_present(all_but(1));
        for _ in 0..3 {
            let _ = robot.push_slot();
            robot.refreshed(all_but(1));
            robot.set_present(all_but(1));
        }

        assert_eq!(
            robot.current_at::<Q>(1),
            Err(StateError::NoSample { depth: 0 })
        );
        assert_eq!(robot.age(1), Ok(None));
        assert!(robot.current::<Q>().is_some());
    }

    #[test]
    fn fr_2_6_a_rejoined_slot_is_seeded_by_its_first_read() {
        let mut robot = RobotState::<N>::default();
        let _ = robot.push_slot();
        robot.refreshed(Mask::ALL);
        let _ = robot.push_slot();
        robot.refreshed(all_but(1));
        robot.set_present(all_but(1));
        let _ = robot.push_slot();
        robot.refreshed(all_but(1));
        robot.set_present(all_but(1));
        let _ = robot.push_slot();
        robot.refreshed(Mask::ALL);
        robot.set_present(Mask::ALL);

        assert_eq!(robot.age(1), Ok(Some(0)));
        assert!(robot.prev_at::<Q>(0, 1).is_ok());
        assert_eq!(
            robot.prev_at::<Q>(1, 1),
            Err(StateError::NoSample { depth: 1 })
        );
        assert!(robot.prev_at::<Q>(1, 0).is_ok());
    }

    #[test]
    fn fr_2_5_drop_absent_clears_written_bits_of_absent_slots() {
        let mut robot = RobotState::<N>::default();
        robot.set_present(all_but(2));
        robot.set_all([Q(1.0); N]);
        assert_eq!(robot.set_at(Qd(1.0), 2), Ok(()));
        assert_eq!(robot.set_at(Tau(1.0), 0), Ok(()));
        robot.drop_absent();

        assert_eq!(robot.written().q, all_but(2));
        assert!(robot.written().qd.is_empty());
        assert_eq!(robot.written().tau, first_only());
    }

    #[test]
    fn set_at_on_an_absent_slot_is_ok() {
        let mut robot = RobotState::<N>::default();
        robot.set_present(all_but(2));
        assert_eq!(robot.set_at(Q(1.0), 2), Ok(()));
        assert!(robot.written().q.get(2));
    }

    #[test]
    fn nothing_joined_or_left_before_the_first_report() {
        let robot = RobotState::<N>::default();
        assert!(robot.joined().is_empty());
        assert!(robot.left().is_empty());
    }

    #[test]
    fn the_first_report_is_the_baseline() {
        let mut robot = RobotState::<N>::default();
        robot.set_present(all_but(2));

        assert!(robot.joined().is_empty());
        assert!(robot.left().is_empty());
        assert_eq!(robot.present(), all_but(2));
    }

    #[test]
    fn joined_and_left_last_one_report() {
        let mut robot = RobotState::<N>::default();
        robot.set_present(all_but(2));
        robot.set_present(all_but(1));

        let mut two = Mask::NONE;
        two.set(2);
        let mut one = Mask::NONE;
        one.set(1);
        assert_eq!(robot.joined(), two);
        assert_eq!(robot.left(), one);

        robot.set_present(all_but(1));
        assert!(robot.joined().is_empty());
        assert!(robot.left().is_empty());

        robot.set_present(Mask::ALL);
        assert_eq!(robot.joined(), one);
        assert!(robot.left().is_empty());
    }
}
