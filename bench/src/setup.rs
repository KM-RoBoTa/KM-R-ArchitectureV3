//! The robot every bench binary starts from (FR-003, FR-027 (SRS NFR-8, NFR-1)).
//!
//! The one place for D2: the joint-limit policy line joins `base()` with
//! kmr_api C4 (T051).

use std::time::Duration;

use kmr_api::{Robot, Spec};

/// A robot of spec `H` with tick period `dt` and no controller yet.
pub fn base<H: Spec>(dt: Duration) -> Robot<H> {
    Robot::new().set_dt(dt)
}

#[cfg(test)]
mod tests {
    use super::*;

    kmr_api::robot!(XmBench);

    #[test]
    fn the_bench_robot_builds_from_base() {
        let _robot: Robot = base::<__Spec>(Duration::from_millis(1));
        assert_eq!(N, 4);
    }
}
