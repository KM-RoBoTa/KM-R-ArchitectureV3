//! The robot's sensors as a list of members, next to the drivers (SRS
//! FR-5.7). A frunk `HCons`, walked head first: static dispatch, no heap.

use core::convert::Infallible;

use frunk_core::hlist::{HCons, HNil};

use super::set::seal::Nested;
use super::{MemberError, SensorHal};
use crate::error::ConfigError;
use crate::sensors::Sensors;

/// What a sensor lost at run time does, beyond raising `sensor missing`,
/// which it always does (SRS FR-5.7, FR-13.4).
#[derive(Default)]
pub enum SensorPolicy {
    /// Nothing more: the flag is the signal, the controllers decide.
    #[default]
    Flag,
    /// The loss is logged, once per member.
    Warn,
    /// An emergency stop, in the tick of the loss.
    Abort,
}

/// The members' slots and silences, planned at run() finalization.
#[doc(hidden)]
pub struct SensorPlan {}

mod seal {
    /// Implemented by the engine for `HNil` and `HCons` only.
    pub trait Sensing {}
}

/// The robot's sensors: a list of [`SensorHal`] members, `HCons<A, HCons<B,
/// HNil>>`. Implemented by the engine only; the walk calls every member,
/// head first, and closes them in reverse.
pub trait Sensing: seal::Sensing {
    /// One variant per member, nested.
    type Error: Nested;

    /// The needs of every member, slots handed out in declaration order.
    /// Past the slots of a kind: [`ConfigError::TooManySensors`].
    #[doc(hidden)]
    fn claim(&self, plan: &mut SensorPlan) -> Result<(), ConfigError>;

    /// [`SensorHal::open`] on every member; the first error stops the walk.
    #[doc(hidden)]
    fn open(&mut self) -> Result<(), Self::Error>;

    /// [`SensorHal::read`] on every member, each into its own slots; this
    /// node is member `member`.
    #[doc(hidden)]
    fn read(&mut self, sensors: &mut Sensors, plan: &mut SensorPlan, member: usize);

    /// [`SensorHal::close`] on every member, last first.
    #[doc(hidden)]
    fn close(&mut self);
}

impl seal::Sensing for HNil {}

impl Sensing for HNil {
    type Error = Infallible;

    fn claim(&self, _plan: &mut SensorPlan) -> Result<(), ConfigError> {
        todo!()
    }

    fn open(&mut self) -> Result<(), Infallible> {
        todo!()
    }

    fn read(&mut self, _sensors: &mut Sensors, _plan: &mut SensorPlan, _member: usize) {
        todo!()
    }

    fn close(&mut self) {
        todo!()
    }
}

impl<H: SensorHal, T: seal::Sensing> seal::Sensing for HCons<H, T> {}

impl<H: SensorHal, T: Sensing> Sensing for HCons<H, T> {
    type Error = MemberError<H::Error, T::Error>;

    fn claim(&self, _plan: &mut SensorPlan) -> Result<(), ConfigError> {
        todo!()
    }

    fn open(&mut self) -> Result<(), Self::Error> {
        todo!()
    }

    fn read(&mut self, _sensors: &mut Sensors, _plan: &mut SensorPlan, _member: usize) {
        todo!()
    }

    fn close(&mut self) {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn slots_follow_declaration_order() {
        todo!()
    }

    #[test]
    fn too_many_imus_is_refused_before_any_open() {
        todo!()
    }

    #[test]
    fn two_mics_are_refused() {
        todo!()
    }

    #[test]
    fn open_stops_at_the_first_error_and_names_the_member() {
        todo!()
    }

    #[test]
    fn close_runs_on_every_member_tail_first() {
        todo!()
    }

    #[test]
    fn an_empty_list_claims_nothing() {
        todo!()
    }
}
