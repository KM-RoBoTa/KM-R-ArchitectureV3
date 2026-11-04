//! A robot of 65 actuators does not build: the write mask holds 64 (SRS Definitions
//! "Write mask"). The error fires under `cargo check`, at the `robot!` line.

kmr_api::robot!(model = "../../../../kmr_api/tests/fixtures/sixty_five.urdf");

fn main() {}
