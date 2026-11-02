//! Test-only brands of fake members, so kmr_api's tests expand `robot!` over two brands.
//!
//! The types are `pub`: `robot!`'s `__Spec` names them in its public `Drivers`.

use std::num::NonZeroU32;
use std::time::Duration;

use kmr_adapters::binding::{Binding, Port};
use kmr_core::hal::{Batch, Claim, Command, Faults, Group, Hal, Mask, Overrides, Sample, seal};
use kmr_core::state::JOINTS;

use crate::brand::{Brand, Refused};

/// A fake part: its bus labels and the engine indices it drives.
#[derive(Clone, Copy, Debug)]
pub struct Hardware<const NB: usize, const M: usize> {
    /// Label of each bus.
    pub labels: [&'static str; NB],
    /// Engine index of each actuator.
    pub indices: [usize; M],
}

/// Bus and actuator counts of `hardware`.
pub(crate) const fn shape<const NB: usize, const M: usize>(
    _hardware: &Hardware<NB, M>,
) -> (usize, usize) {
    (NB, M)
}

/// Engine indices `hardware` drives.
pub(crate) const fn indices<const NB: usize, const M: usize>(
    hardware: &Hardware<NB, M>,
) -> &[usize] {
    &hardware.indices
}

/// What a fake part binds to: it owns its indices and keeps the ports it was handed.
#[derive(Debug, PartialEq)]
pub struct Fake {
    /// Actuators it drives.
    pub owned: Mask,
    /// Ports of the plugins routed to it, in order.
    pub ports: Vec<&'static str>,
}

impl<const NB: usize, const M: usize> Brand for Hardware<NB, M> {
    type Member = Fake;

    fn labels(&self) -> Vec<&'static str> {
        self.labels.to_vec()
    }

    fn bind(self, plugins: &[Binding]) -> Result<Fake, Refused<core::convert::Infallible>> {
        let mut owned = Mask::NONE;
        self.indices.iter().for_each(|&index| owned.set(index));
        let ports = plugins
            .iter()
            .map(|plugin| match plugin.port {
                Port::ById(port) | Port::ByDev(port) => port,
            })
            .collect();
        Ok(Fake { owned, ports })
    }
}

impl seal::Adapter for Fake {}

/// Claims its indices in one group; every other call does nothing.
impl Hal for Fake {
    type Error = core::convert::Infallible;

    fn check(&mut self, _overrides: Overrides) -> Result<Claim, Self::Error> {
        Ok(Claim {
            owned: self.owned,
            group: [Group(0); JOINTS],
            read_every: [NonZeroU32::MIN; JOINTS],
            read_cost: Duration::ZERO,
            write_cost: Duration::ZERO,
        })
    }

    fn open(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn zero(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn ping(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn health(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn configure(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read_all(&mut self, _sample: Sample<'_>) -> Result<(), Self::Error> {
        Ok(())
    }

    fn torque_on(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, _sample: Sample<'_>, _faults: &mut Faults) -> Mask {
        self.owned
    }

    fn write(&mut self, _command: Command<'_>, _batch: &Batch<'_>, _faults: &mut Faults) {}

    fn hold(&mut self, _target: Command<'_>) {}

    fn torque_off(&mut self) {}

    fn close(&mut self) {}
}

/// The arm brand of the tests.
pub(crate) mod arm {
    pub(crate) use super::{Hardware, indices, shape};
}

/// The leg brand of the tests.
pub(crate) mod leg {
    pub(crate) use super::{Hardware, indices, shape};
}
