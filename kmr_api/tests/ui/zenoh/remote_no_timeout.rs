// A remote source without its timeout is not a `Remote` (FR-058).
use kmr_api::{EachTick, Gateway, GatewayConfig, Identity, Remote, Sensors, Time};

kmr_api::robot!(XmBench);

fn hold(_time: &Time, _state: &mut State, _sensors: &Sensors) {}

fn main() {
    let gateway = Gateway::new(GatewayConfig {
        instance: "arm-1",
        identity: Identity::loopback(),
        read_history: &[],
    });
    let _robot = Robot::new()
        .add_plugins(gateway)
        .add_remote_source(EachTick, Remote::allow("ctl-2"), hold);
}
