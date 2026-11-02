//! A robot of three actuators against kmr_core's four: refused until N is a const generic.
//!
//! trybuild builds each case under `target/tests/trybuild/kmr_api/`, so the path climbs to
//! the workspace root.

kmr_api::robot!(model = "../../../../kmr_api/tests/fixtures/three.urdf");

fn main() {}
