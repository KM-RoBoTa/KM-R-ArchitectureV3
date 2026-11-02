//! A descriptor that does not exist does not build: the error names the path.

kmr_api::robot!(model = "../../../../kmr_api/tests/fixtures/no_such_robot.urdf");

fn main() {}
