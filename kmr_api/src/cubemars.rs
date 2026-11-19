//! The CubeMars hardware: the models, for [`Robot::add_actuator`](crate::Robot::add_actuator),
//! and the table, for `robot!(model = .., hardware = { cubemars: .. })`.
//!
//! ```ignore
//! let leg = kmr_api::Bus::by_dev("can0").build();
//! Robot::new().add_actuator(kmr_api::cubemars::Ak60::ids(1..=3).gains(150.0, 1.0).on(leg))
//! ```
//!
//! Each declared actuator runs in MIT mode (`MitAk2`) on cubemars_rs. The pose it stands in at
//! bring-up is its joint 0, unless [`start`](Ids::start) or [`starts`](Ids::starts) says otherwise; the motor's stored zero
//! is never used nor written. Its limits are checked by cubemars_rs on every command and every
//! reply. Declaring needs no feature; driving them needs `kmr_api/cubemars`.

use core::ops::RangeInclusive;

/// The CubeMars driver [`Bound::drivers`](crate::Bound::drivers) hands out.
#[cfg(feature = "cubemars")]
pub use kmr_adapters::cubemars::CubeMars;
/// Why a CubeMars bind or bring-up failed; reach it with `downcast_ref`.
#[cfg(feature = "cubemars")]
pub use kmr_adapters::cubemars::CubeMarsError;
/// The SocketCAN opener of the CubeMars driver.
#[cfg(feature = "cubemars")]
pub use kmr_adapters::cubemars::SocketCan;
pub use kmr_adapters::cubemars::{
    Actuator, Bus, DEFAULT_KD, DEFAULT_KP, Limits, Model, Placement, Table,
};

use crate::actuator::{Actuators, Driver, IdSet};
use crate::{BusHandle, Q, Qd, Tau};

/// Actuators of one model at a range of CAN ids, not yet on a bus.
#[derive(Clone, Debug, PartialEq)]
pub struct Ids {
    ids: IdSet,
    row: Actuator,
    each: PerJoint,
}

/// The settings of a declaration laid over its row at lowering: the per-actuator lists, entry k
/// overriding the shared setting on the k-th actuator, then the position margin.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct PerJoint {
    /// From [`Ids::position_limits`]: the lows, then the highs.
    position: Option<(Vec<f64>, Vec<f64>)>,
    /// From [`Ids::starts`].
    start: Option<Vec<f64>>,
    /// From [`Ids::position_margin`].
    margin: f32,
}

impl PerJoint {
    /// Each list given, by setter label, with its length.
    pub(crate) fn lengths(&self) -> impl Iterator<Item = (&'static str, usize)> {
        let (lows, highs) = self
            .position
            .as_ref()
            .map(|(l, h)| (l.len(), h.len()))
            .unzip();
        [
            ("`position_limits` lows", lows),
            ("`position_limits` highs", highs),
            ("`starts`", self.start.as_ref().map(Vec::len)),
        ]
        .into_iter()
        .filter_map(|(setter, len)| Some((setter, len?)))
    }

    /// `row`, the k-th actuator of the declaration, with entry `k` of each list, then its range
    /// widened by the margin.
    // DECISION: the f64 entries narrow to the f32 that Q and cubemars_rs store here, at
    // lowering, so the setters keep whatever precision the user handed in until then.
    pub(crate) fn apply(&self, k: usize, row: &mut Actuator) {
        if let Some((lows, highs)) = &self.position
            && let (Some(&lo), Some(&hi)) = (lows.get(k), highs.get(k))
        {
            row.limits.position = Some((lo as f32, hi as f32));
        }
        if let Some(&q) = self.start.as_ref().and_then(|s| s.get(k)) {
            row.start = q as f32;
        }
        if let Some((lo, hi)) = &mut row.limits.position {
            (*lo, *hi) = (*lo - self.margin, *hi + self.margin);
        }
    }
}

impl Ids {
    /// The actuators of `model` at `ids`, with the defaults.
    fn new(model: Model, ids: IdSet) -> Self {
        Self {
            ids,
            row: Actuator::new(model, 0, 0),
            each: PerJoint::default(),
        }
    }

    /// The row, `change`d.
    fn with(mut self, change: impl FnOnce(&mut Actuator)) -> Self {
        change(&mut self.row);
        self
    }

    /// Sends these gains on a tick that writes none: `kp` in `0..=500`, `kd` in `0..=5`.
    /// Default [`DEFAULT_KP`] and [`DEFAULT_KD`]; `run()` refuses a gain outside the range.
    pub fn gains(self, kp: f32, kd: f32) -> Self {
        self.with(|row| (row.kp, row.kd) = (kp, kd))
    }

    /// The joint position of the pose these actuators stand in at bring-up. Default 0.
    pub fn start(self, q: Q) -> Self {
        self.with(|row| row.start = q.0)
    }

    /// The k-th pose goes to the k-th actuator.
    ///
    /// Overrides [`start`](Self::start) there; `run()` refuses a list whose length is not the
    /// declaration's actuator count.
    pub fn starts(mut self, qs: impl IntoIterator<Item = impl Into<f64>>) -> Self {
        self.each.start = Some(qs.into_iter().map(Into::into).collect());
        self
    }

    /// Mounted the other way round: positions, velocities and torques are negated.
    pub fn reversed(self) -> Self {
        self.with(|row| row.reversed = true)
    }

    /// Allowed joint positions; a goal outside is refused, a reply outside cuts the torque.
    pub fn position_limit(self, range: RangeInclusive<Q>) -> Self {
        self.with(|row| row.limits.position = Some((range.start().0, range.end().0)))
    }

    /// The k-th actuator stays within `lows[k]..=highs[k]`, rad.
    ///
    /// Overrides [`position_limit`](Self::position_limit) there; `run()` refuses a list whose
    /// length is not the declaration's actuator count.
    pub fn position_limits(
        mut self,
        lows: impl IntoIterator<Item = impl Into<f64>>,
        highs: impl IntoIterator<Item = impl Into<f64>>,
    ) -> Self {
        self.each.position = Some((
            lows.into_iter().map(Into::into).collect(),
            highs.into_iter().map(Into::into).collect(),
        ));
        self
    }

    /// Widens every position range of this declaration by `margin` at both ends.
    ///
    /// Applies to [`position_limit`](Self::position_limit) and
    /// [`position_limits`](Self::position_limits) whatever the order; no range, no effect.
    pub fn position_margin(mut self, margin: Q) -> Self {
        self.each.margin = margin.0;
        self
    }

    /// Largest joint speed either way, on goals and replies.
    pub fn velocity_limit(self, max: Qd) -> Self {
        self.with(|row| row.limits.velocity = Some(max.0))
    }

    /// Largest torque either way, on goals and replies.
    pub fn torque_limit(self, max: Tau) -> Self {
        self.with(|row| row.limits.torque = Some(max.0))
    }

    /// Highest temperature, °C; a reply above it cuts the torque, and bring-up refuses it.
    pub fn temperature_limit(self, celsius: f32) -> Self {
        self.with(|row| row.limits.temperature = Some(celsius))
    }

    /// Largest distance between a position goal and the last measured position.
    pub fn max_goal_step(self, step: Q) -> Self {
        self.with(|row| row.limits.max_goal_step = Some(step.0))
    }

    /// These actuators on `bus`, a CAN interface: `Bus::by_dev("can0").build()`.
    pub fn on(self, bus: BusHandle) -> Actuators {
        Actuators {
            bus,
            ids: self.ids,
            driver: Driver::CubeMars {
                row: self.row,
                each: self.each,
            },
        }
    }
}

/// Declares one model type per CubeMars [`Model`].
macro_rules! models {
    ($($(#[$doc:meta])* $name:ident => $model:ident,)*) => {$(
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub struct $name;

        impl $name {
            /// The actuator at CAN id `id`.
            pub fn id(id: u8) -> Ids {
                Ids::new(Model::$model, IdSet::Range(id..=id))
            }

            /// The actuators at CAN ids `ids`, in this order.
            pub fn ids(ids: RangeInclusive<u8>) -> Ids {
                Ids::new(Model::$model, IdSet::Range(ids))
            }

            /// One actuator per actuator of the robot model, at CAN ids `1..=N`.
            pub fn all() -> Ids {
                Ids::new(Model::$model, IdSet::All)
            }
        }
    )*};
}

models! {
    /// AK60-6, MIT mode on an AK 2.0 board.
    Ak60 => Ak60_6,
    /// AK80-8, MIT mode on an AK 2.0 board.
    Ak80 => Ak80_8,
}
