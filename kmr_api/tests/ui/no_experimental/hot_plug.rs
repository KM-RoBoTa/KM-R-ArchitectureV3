// Without the `experimental` feature, nothing of the hot-plug API is reachable: not its
// module, not the policies at the crate root, not a presence query on the state.

use kmr_api::MissingPolicy;
use kmr_api::experimental::JoinPolicy;

kmr_api::robot!(XmBench);

fn watch(_time: &kmr_api::Time, robot: &mut State, _sensors: &kmr_api::Sensors) {
    let _ = robot.present();
}

fn main() {
    let _ = (MissingPolicy::Tolerate, JoinPolicy::Declared, watch);
}
