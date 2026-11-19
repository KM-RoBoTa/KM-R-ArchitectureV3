//! The adapters' presence through the loop: copied into the state after each
//! read, the writes of absent actuators cut before the limit stage, and the
//! join policy's upkeep at the end of the tick.

use crate::Signal;
use crate::hal::{Drivers, JoinPolicy, Overrides, Sensing};
use crate::runtime::{Host, Rig, estop};
use crate::state::RobotState;

impl<const N: usize, D: Drivers<N>, SN: Sensing> Rig<'_, N, D, SN> {
    /// Copies the drivers' presence into `state`, after a read.
    pub(super) fn sync_presence(&self, state: &mut RobotState<N>) {
        state.set_present(self.drivers.present(self.layout.slots()));
    }

    /// Clears the writes of absent actuators, before the limit stage, the
    /// batch and the hold target.
    pub(super) fn drop_absent(&self, state: &mut RobotState<N>) {
        state.drop_absent();
    }

    /// Only under [`JoinPolicy::Declared`]: one bus check, then the drivers'
    /// upkeep when nothing is raised. An emergency stop raised before it is
    /// returned, never kept waiting behind a join stage; a shutdown skips it
    /// and lets the tick complete.
    pub(super) fn upkeep<H: Host>(&mut self, overrides: Overrides, host: &mut H) -> Option<Signal> {
        if overrides.join != JoinPolicy::Declared {
            return None;
        }
        let pending = host.poll();
        if pending.is_none() {
            self.drivers.upkeep();
        }
        estop(pending)
    }
}
