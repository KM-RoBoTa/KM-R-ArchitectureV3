// A remote source without its fallback does not compile (US6-AS3, FR-059).
use std::time::Duration;

use kmr_api::{EachTick, Gateway, GatewayConfig, Identity, Remote};

kmr_api::robot!(XmBench);

fn main() {
    let gateway = Gateway::new(GatewayConfig {
        instance: "arm-1",
        identity: Identity::loopback(),
        read_history: &[],
    });
    let _robot = Robot::new()
        .add_plugins(gateway)
        .add_remote_source(EachTick, Remote::allow("ctl-2").timeout(Duration::from_millis(20)));
}
