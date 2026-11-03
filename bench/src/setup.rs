//! The robot every bench binary starts from (FR-003, FR-027 (SRS NFR-8, NFR-1)).
//!
//! The one place for D2: the joint-limit policy (decided 2026-10-28, for now: D2 B).

use std::time::Duration;

use kmr_api::{Policy, Robot, Spec};

/// A robot of `N` actuators and spec `H` with tick period `dt`, `Warn` on every limit, and no
/// controller yet. The probe never moves a motor, so no limit path runs.
pub fn base<const N: usize, H: Spec<N>>(dt: Duration) -> Robot<N, H> {
    Robot::new().set_dt(dt).set_limit_policy_all(Policy::Warn)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kmr_api::EachTick;

    kmr_api::robot!(XmBench);

    /// A model that hands its limits to the core, so a run without a policy is refused.
    mod limited {
        kmr_api::robot!(model = "../kmr_api/tests/fixtures/four.urdf");

        /// Shuts down at tick 2.
        pub fn stop(time: &kmr_api::Time, _robot: &mut State, _s: &kmr_api::Sensors) {
            if time.tick() == 2 {
                kmr_api::shutdown!("bench test: tick 2");
            }
        }
    }

    #[test]
    fn the_bench_robot_builds_from_base() {
        let _robot: Robot = base::<N, __Spec>(Duration::from_millis(1));
        assert_eq!(N, 4);
    }

    #[test]
    fn base_sets_a_policy_on_every_limited_actuator() {
        let res = base::<{ limited::N }, limited::__Spec>(Duration::from_millis(1))
            .add_controller(EachTick, limited::stop)
            .run();
        assert!(res.is_ok(), "{res:?}");
    }
}
