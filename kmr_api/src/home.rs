//! The home state, open in SRS FR-2.7: every call is a compile error naming it.
//!
//! Assumed, pending human: D5, option A. No marker runs after `open`, with the torque on
//! (SRS NFR-2): the methods keep their names and argument shapes, and a bound no type meets
//! stops each call site at compile time.

use core::convert::Infallible;

use crate::{Field, N, State};

/// The seal of [`HomeDecided`].
mod sealed {
    /// Implemented by no type.
    pub trait Sealed {}
}

/// Met by no type until SRS FR-2.7 decides the home state.
#[diagnostic::on_unimplemented(
    message = "kmr: SRS FR-2.7: home state is open; `initial_state` and `go_home` do not build until it is decided",
    label = "home state is open (SRS FR-2.7)"
)]
pub trait HomeDecided: sealed::Sealed {
    /// No value exists: a call that reaches a body cannot be built.
    const OPEN: Infallible;
}

impl State<'_> {
    /// The home state of field `T`, per actuator (SRS FR-2.7, open).
    pub fn initial_state<T: Field + HomeDecided>(&self) -> [T; N] {
        match T::OPEN {}
    }

    /// Ramps every actuator to its home position, during `Init` only (SRS FR-2.7, open).
    pub fn go_home<R: HomeDecided>(&mut self, _ramp: R) {
        match R::OPEN {}
    }
}
