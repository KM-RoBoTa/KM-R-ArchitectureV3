//! A robot without a controller does not build (SRS NFR-4).

kmr_api::robot!(XmBench);

fn main() {
    Robot::new().run().ok();
}
