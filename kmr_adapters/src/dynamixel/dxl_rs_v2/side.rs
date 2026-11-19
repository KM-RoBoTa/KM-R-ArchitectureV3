//! The side buses of a port and the hot-plug calls of the backend: a probe
//! pings on a side bus, so it never touches what the main buses hold.

use ::dxl_rs_v2::{Bus, Clock, Id, ModelNumber, P1, P2, Transport};

use super::{DxlRsV2, Open, Ports, backend_error, config, held, id_of};
use crate::dynamixel::backend::{BackendError, HotPlug};
use crate::dynamixel::config::{Model, Protocol};

/// The side buses of a port, one per protocol, each holding at most the one
/// actuator a probe named.
// DECISION: two more handles of the device, not a ping on the main buses:
// dxl_rs_v2 has no public ping, and a Setup on a main bus replaces its list.
pub(super) struct Side<T: Transport, C> {
    /// Protocol 1 probes.
    pub(super) one: Bus<P1, T, C, 1>,
    /// Protocol 2 probes.
    pub(super) two: Bus<P2, T, C, 1>,
}

/// A handle of a device with its clock, and two more of the same device.
type Lines<T, C> = ((T, C), [(T, C); 2]);

/// `line` back, with two more handles of its device for the side buses.
pub(super) fn open<T: Open<C>, C>(mut line: (T, C)) -> Result<Lines<T, C>, BackendError> {
    let side = T::open_side(&mut line.0)?;
    Ok((line, side))
}

impl<const N: usize, T: Transport, C: Clock> Ports<N, T, C> {
    /// Gives open port `port` its side buses on lines `side`, at `baud`
    /// bit/s.
    pub(super) fn attach_side(&mut self, port: usize, side: [(T, C); 2], baud: u32) {
        let [three, four] = side;
        let config = config(baud);
        if let Some(Some(open)) = self.ports.get_mut(port) {
            open.side = Some(Side {
                one: Bus::new(three.0, three.1, config),
                two: Bus::new(four.0, four.1, config),
            });
        }
    }
}

impl<const N: usize, T: Transport, C: Clock> HotPlug for Ports<N, T, C> {
    /// A Setup of `id` alone on the side bus of `model`'s protocol: a ping
    /// and the reads `dxl_rs_v2` checks the model with.
    fn probe(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        let (open, target) = (self.port(port)?, id_of(id)?);
        let side = open
            .side
            .as_mut()
            .ok_or(BackendError::PortNotOpen { port })?;
        let entry = [(target, ModelNumber(model.number()))];
        match model.protocol() {
            Protocol::One => side.one.declare(entry).map(|_| ()),
            Protocol::Two => side.two.declare(entry).map(|_| ()),
        }
        .map_err(backend_error)
    }

    /// `enable_torque` on the main bus.
    // DECISION: no start goal is asked, unlike `torque`: a rearmed actuator
    // kept its torque on and its goal through the re-declare that held it
    // Off, so it does not move.
    fn rearm(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        let (open, target) = (self.port(port)?, id_of(id)?);
        on_bus!(open, model, |bus| held(bus, target)?
            .enable_torque()
            .map_err(backend_error))?;
        open.started.remove(target);
        Ok(())
    }

    /// One `dxl_rs_v2` `clear_fault`, whatever the protocol.
    fn clear_fault_once(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        let (open, target) = (self.port(port)?, id_of(id)?);
        on_bus!(open, model, |bus| held(bus, target)?
            .clear_fault()
            .map_err(backend_error))
    }

    /// Whether the bus of `model`'s protocol keeps a torque state for `id`.
    fn holds(&self, port: usize, model: Model, id: u8) -> bool {
        let (Some(Some(open)), Ok(target)) = (self.ports.get(port), Id::new(id)) else {
            return false;
        };
        match model.protocol() {
            Protocol::One => open.one.torque_state(target).is_some(),
            Protocol::Two => open.two.torque_state(target).is_some(),
        }
    }
}

impl<const N: usize> HotPlug for DxlRsV2<N> {
    fn probe(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        self.0.probe(port, model, id)
    }

    fn rearm(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        self.0.rearm(port, model, id)
    }

    fn clear_fault_once(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        self.0.clear_fault_once(port, model, id)
    }

    fn holds(&self, port: usize, model: Model, id: u8) -> bool {
        self.0.holds(port, model, id)
    }
}
