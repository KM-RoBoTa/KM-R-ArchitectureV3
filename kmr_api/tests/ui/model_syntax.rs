//! `bad_syntax.urdf` does not build: the error names the file, the element and the reason.

kmr_api::robot!(model = "../../../../kmr_api/tests/fixtures/bad_syntax.urdf");

fn main() {}
