use crate::clock::Time;

/// Plain settings bag. Non-generic: holds configuration only, never the
/// controllers (those live in the separate `L` list on `Robot`).
#[derive(Default)]
pub(crate) struct RobotConfig {
    pub time: Time,
}
