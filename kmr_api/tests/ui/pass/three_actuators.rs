//! A robot of three actuators builds: N comes from the descriptor, not from kmr_core.
//!
//! trybuild builds each case under `target/tests/trybuild/kmr_api/`, so the path climbs to
//! the workspace root.

kmr_api::robot!(model = "../../../../kmr_api/tests/fixtures/three.urdf");

fn main() {
    let _robot = Robot::new();
    assert_eq!(N, 3);
}
