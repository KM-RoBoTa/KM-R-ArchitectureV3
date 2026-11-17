//! Without the `experimental` feature: every owned actuator is live for the
//! whole run, a silent one fails bring-up and nothing is ever probed. The
//! same calls as the feature's presence, each a constant or a no-op.

use kmr_core::hal::{Mask, Overrides};

use super::Dynamixel;
use super::backend::Undeclared;

/// Where the actuators of one adapter stand: all live, always.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Presence<const N: usize>;

impl<const N: usize> Presence<N> {
    /// Every slot live.
    pub(crate) const fn new() -> Self {
        Self
    }

    /// A run: as [`Presence::new`].
    pub(crate) const fn run(_overrides: Overrides) -> Self {
        Self
    }

    /// Nothing to note.
    pub(crate) fn claim(&mut self, _owned: Mask<N>) {}

    /// Every slot.
    pub(crate) const fn live(&self) -> Mask<N> {
        Mask::ALL
    }

    /// A silent actuator is an error.
    pub(crate) const fn tolerates(&self) -> bool {
        false
    }

    /// Every slot due, none seeded.
    pub(crate) fn begin_read(&mut self) -> (Mask<N>, Mask<N>) {
        (Mask::ALL, Mask::NONE)
    }

    /// Nothing seeded to promote.
    pub(crate) fn promote(&mut self, _index: usize) {}

    /// None seeded.
    pub(crate) const fn seed_read(&self) -> Mask<N> {
        Mask::NONE
    }

    /// A timed-out read never loses a slot.
    pub(crate) fn miss(&mut self, _index: usize) -> bool {
        false
    }

    /// Nothing counted.
    pub(crate) fn answered(&mut self, _index: usize) {}

    /// Nothing to note.
    pub(crate) fn powered(&mut self, _powered: Mask<N>) {}
}

impl<B, const NB: usize, const N: usize> Dynamixel<B, NB, N> {
    /// Nothing is ever lost.
    pub(super) fn lose_slots(&mut self, _lost: Mask<N>) {}

    /// The first setup's answer, as it is.
    pub(super) fn declare_present(
        &mut self,
        _bus: usize,
        _passes: usize,
        answer: Result<(), Undeclared>,
    ) -> Result<(), Undeclared> {
        answer
    }

    /// Every bus holds every owned actuator.
    pub(super) fn unheld(&self) -> Mask<N> {
        Mask::NONE
    }

    /// Nothing to cut.
    pub(super) fn cut_unheld(&mut self, _unheld: Mask<N>) {}

    /// The exit path cuts every actuator.
    pub(super) fn left_to_close(&self, _index: usize) -> bool {
        false
    }
}
