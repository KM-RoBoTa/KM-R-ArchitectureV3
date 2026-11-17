//! Hot-plug of declared actuators: missing at start, lost or joining while the loop runs.
//!
//! Niche, and not part of the maintained surface: it may change or go without notice. Built
//! only with the `experimental` feature; bring it in with `use kmr_api::experimental::*`.

use crate::{Mask, Robot, State};

/// What a declared actuator that does not answer means, and whether an absent one may join
/// the run: set them with [`RobotExt`].
pub use kmr_core::hal::{JoinPolicy, MissingPolicy};

mod seal {
    /// Only kmr_api's types take the extension traits of [`super`].
    pub trait Sealed {}

    impl<const N: usize, H, C, E, SN> Sealed for crate::Robot<N, H, C, E, SN> {}

    impl<const N: usize> Sealed for crate::State<'_, N> {}
}

/// The presence policies on the robot builder.
// DECISION: `RobotExt`, on `Robot` only: `Bound` takes no setting.
pub trait RobotExt: seal::Sealed + Sized {
    /// What a declared actuator that does not answer, at open or at run time, means. Without
    /// it, [`MissingPolicy::Fail`]: bring-up fails. With [`MissingPolicy::Tolerate`] it is
    /// absent, logged once, and [`Presence::present`] leaves it out.
    fn set_missing_policy(self, policy: MissingPolicy) -> Self;

    /// Whether an absent declared actuator may join the run. Without it, [`JoinPolicy::Never`].
    /// It needs [`MissingPolicy::Tolerate`] to have any effect.
    fn set_join_policy(self, policy: JoinPolicy) -> Self;
}

impl<const N: usize, H, C, E, SN> RobotExt for Robot<N, H, C, E, SN> {
    fn set_missing_policy(self, policy: MissingPolicy) -> Self {
        self.map_core(|inner| inner.set_missing_policy(policy))
    }

    fn set_join_policy(self, policy: JoinPolicy) -> Self {
        self.map_core(|inner| inner.set_join_policy(policy))
    }
}

/// Which actuators are present, as a controller reads them from its [`State`].
///
/// Every actuator is present unless [`MissingPolicy::Tolerate`] is set and one stopped
/// answering or never answered. A write to an absent actuator is skipped, not an error.
pub trait Presence<const N: usize>: seal::Sealed {
    /// The actuators present this tick: read and written.
    fn present(&self) -> Mask<N>;

    /// Whether actuator `i` is present this tick. `false` for an index past N.
    fn present_at(&self, i: usize) -> bool;
}

impl<const N: usize> Presence<N> for State<'_, N> {
    #[inline]
    fn present(&self) -> Mask<N> {
        self.core().present()
    }

    #[inline]
    fn present_at(&self, i: usize) -> bool {
        self.core().present().get(i)
    }
}
