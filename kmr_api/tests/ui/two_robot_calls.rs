//! Two robots in one module do not build: one `robot!` per module.

kmr_api::robot!(model = "../../../../kmr_api/tests/fixtures/four.urdf");
kmr_api::robot!(XmBench);

fn main() {}
