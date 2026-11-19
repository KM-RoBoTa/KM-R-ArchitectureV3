// With the `experimental` feature, the hot-plug API is still out of the crate root and off the
// inherent methods: without `use kmr_api::experimental::...`, none of it is reachable.

use kmr_api::MissingPolicy;

kmr_api::robot!(XmBench);

fn watch(_time: &kmr_api::Time, robot: &mut State, _sensors: &kmr_api::Sensors) {
    let _ = robot.present();
}

fn main() {
    let _ = Robot::new().set_missing_policy(MissingPolicy::Tolerate);
    let _ = watch;
}
