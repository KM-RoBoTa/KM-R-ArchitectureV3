//! A name the descriptor does not give is no actuator.

kmr_api::robot!(model = "../../../../kmr_api/tests/fixtures/four.urdf");

const WRIST: usize = actuator!("wrist");

fn main() {}
