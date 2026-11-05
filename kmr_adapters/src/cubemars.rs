//! The CubeMars adapter: AK60-6 actuators in MIT mode on raw SocketCAN, one
//! transaction group per CAN interface (SRS FR-5.1, FR-5.3).
//!
//! The CAN bus sits behind a private transport, so the adapter is tested on a
//! scripted one without hardware. Positions are offsets from where each
//! actuator stood at Init: the zero frame `FF×7 FE`, which rewrites the
//! motor's stored zero, is never sent.

mod config;
mod frame;
#[cfg(feature = "cubemars")]
mod socket;
mod transport;

#[cfg(test)]
mod scripted;
#[cfg(test)]
mod tests;

use core::time::Duration;

use kmr_core::hal::{Batch, Claim, Command, Faults, Group, Hal, Mask, Overrides, Sample, seal};
use kmr_core::state::{Kd, Kp, Q, Qd, Tau};
use kmr_core::telemetry::Telemetry;
use tracing::warn;

use crate::binding::{self, Binding, Port, Unmatched};
use config::Placed;
pub use config::{Actuator, Bus, DEFAULT_KD, DEFAULT_KP, Placement, Table};
use frame::{ENTER, EXIT, Feedback, Mit};
#[cfg(feature = "cubemars")]
pub use socket::SocketCan;
use transport::{Transport, TransportError};

/// Attempts per actuator of a bring-up question before it counts as missing.
const ATTEMPTS: usize = 3;
/// Frames of other actuators a bring-up question skips per attempt.
const FOREIGN_FRAMES: usize = 16;
/// Wait for the reply to a bring-up question, cubemars_rs's.
const REPLY_TIMEOUT: Duration = Duration::from_millis(30);
/// Wait for the reply to the enter frame, cubemars_rs's.
const ENTER_TIMEOUT: Duration = Duration::from_millis(500);
/// Frames `read` drains per bus at most, so a flooded bus cannot hold the
/// tick: two per actuator the robot can have.
const DRAIN_LIMIT: usize = 128;
/// Exit frame attempts per actuator at `torque_off`, back to back.
const TORQUE_OFF_ATTEMPTS: usize = 3;
// DECISION: 80 °C, under the AK60-6 winding's rating, until a bench gives
// the motor's own limit; `data[6]` read as °C as cubemars_rs does.
/// Highest temperature `health` accepts, °C.
const MAX_TEMPERATURE_C: u8 = 80;
// DECISION: an 8-byte standard frame at 1 Mbit/s with worst-case stuffing,
// about 130 bits; the interface's bitrate is set by `ip link`, not here.
/// Bus time of one frame.
const FRAME_TIME: Duration = Duration::from_micros(130);
/// Local transaction groups one member may claim: `Group` is a `u8`.
const MAX_GROUPS: usize = 256;

/// Why the CubeMars bind or bring-up failed. Names the bus, and the
/// interface or the actuator when there is one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum CubeMarsError {
    /// A table row placed at an engine index the robot does not have.
    #[error(
        "actuator {actuator} of the table is placed at engine index {index}, past the robot's {actuators} actuators"
    )]
    PastTheRobot {
        /// Row of the table.
        actuator: usize,
        /// Engine index it was placed at.
        index: usize,
        /// The robot's actuators.
        actuators: usize,
    },
    /// More buses than transaction groups.
    #[error("{buses} CAN interfaces declared, one member takes at most {max}")]
    TooManyBuses {
        /// Buses declared.
        buses: usize,
        /// Groups one member may claim.
        max: usize,
    },
    /// A bus plugin names a label no table bus has.
    #[error("the bus plugin of {port} names bus {label}, which the hardware table does not have")]
    UnknownBus {
        /// Label the plugin names.
        label: &'static str,
        /// Port of the plugin.
        port: Port,
    },
    /// A table bus no bus plugin binds.
    #[error("bus {bus}: no bus plugin binds it; add one with `.bus(\"{bus}\")`")]
    UnboundBus {
        /// Bus label.
        bus: &'static str,
    },
    /// A bus plugin left over once every bus is bound.
    #[error("the bus plugin of {port} has no bus left to bind")]
    ExtraPlugin {
        /// Port of the plugin.
        port: Port,
    },
    /// A bus plugin without a label, on a table of several buses.
    #[error(
        "the bus plugin of {port} names no bus; the hardware table has several, name one with `.bus(\"label\")`"
    )]
    LabelRequired {
        /// Port of the plugin.
        port: Port,
    },
    /// Two bus plugins bind one bus.
    #[error("bus {bus}: bound twice, again by the bus plugin of {port}")]
    BusBoundTwice {
        /// Bus label.
        bus: &'static str,
        /// Port of the second plugin.
        port: Port,
    },
    /// A CAN bus bound by serial id: an interface has a name, no serial.
    #[error("bus {bus}: {port} names no CAN interface; bind it with `Bus::by_dev(\"can0\")`")]
    NotAnInterface {
        /// Bus label.
        bus: &'static str,
        /// Port of the plugin.
        port: Port,
    },
    /// A bus plugin baud on a CAN bus: the bitrate is the interface's.
    #[error(
        "bus {bus}: a CAN interface's bitrate is set by `ip link set {interface} type can bitrate ..`, not by its bus plugin ({baud} given)"
    )]
    BaudOnCan {
        /// Bus label.
        bus: &'static str,
        /// Interface name.
        interface: &'static str,
        /// Baud of the plugin.
        baud: u32,
    },
    /// Two buses name one interface.
    #[error("buses {first} and {bus} name the same CAN interface {interface}")]
    SharedInterface {
        /// The earlier bus.
        first: &'static str,
        /// The later bus.
        bus: &'static str,
        /// Interface name.
        interface: &'static str,
    },
    /// The interface could not be opened.
    #[error("bus {bus}: CAN interface {interface} could not be opened: {source}")]
    InterfaceOpen {
        /// Bus label.
        bus: &'static str,
        /// Interface name.
        interface: &'static str,
        /// What the transport reported.
        #[expect(
            private_interfaces,
            reason = "the CAN seam stays private: callers get its message, not its type"
        )]
        source: TransportError,
    },
    /// The actuator does not answer.
    #[error("actuator {actuator}: CAN id {id} on bus {bus} does not answer")]
    Missing {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// CAN id.
        id: u8,
    },
    /// The actuator reports an error code.
    #[error("actuator {actuator}: CAN id {id} on bus {bus}: error code {code}")]
    Fault {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// CAN id.
        id: u8,
        /// `data[7]` of its reply.
        code: u8,
    },
    /// The temperature is above [`MAX_TEMPERATURE_C`].
    #[error(
        "actuator {actuator}: CAN id {id} on bus {bus}: temperature {measured_c} °C above {limit_c} °C"
    )]
    Temperature {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// CAN id.
        id: u8,
        /// `data[6]` of its reply.
        measured_c: u8,
        /// The limit.
        limit_c: u8,
    },
    /// A frame to the actuator could not be sent or its reply received.
    #[error("actuator {actuator}: CAN id {id} on bus {bus}: {source}")]
    Transport {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// CAN id.
        id: u8,
        /// What the transport reported.
        #[expect(
            private_interfaces,
            reason = "the CAN seam stays private: callers get its message, not its type"
        )]
        source: TransportError,
    },
}

/// The plugins' mismatch, as the CubeMars error naming it.
impl From<Unmatched> for CubeMarsError {
    fn from(unmatched: Unmatched) -> Self {
        match unmatched {
            Unmatched::UnknownBus { label, port } => Self::UnknownBus { label, port },
            Unmatched::UnboundBus { bus } => Self::UnboundBus { bus },
            Unmatched::ExtraPlugin { port } => Self::ExtraPlugin { port },
            Unmatched::LabelRequired { port } => Self::LabelRequired { port },
            Unmatched::BusBoundTwice { bus, port } => Self::BusBoundTwice { bus, port },
        }
    }
}

/// The last MIT command sent to one actuator.
#[derive(Clone, Copy, Debug)]
struct Sent {
    /// Its bytes, sent again on a tick that does not write the actuator.
    data: [u8; 8],
    /// Its position, engine offset.
    q: f32,
    /// Its position gain.
    kp: f32,
    /// Its velocity gain.
    kd: f32,
}

/// The CubeMars adapter over `NB` CAN interfaces, talking through transport
/// `T`: one member of the robot's hardware, owning the engine indices of its
/// [`Placement`] (SRS FR-5.1) out of the robot's `N` actuators.
pub struct CubeMars<T, const NB: usize, const N: usize> {
    /// The CAN seam.
    transport: T,
    /// Interfaces of the table.
    buses: [Bus; NB],
    /// The actuators the member owns, by engine index.
    actuators: Placed<N>,
    /// Interface name of each bus, from its bus plugin; `None` until bound.
    interfaces: [Option<&'static str>; NB],
    /// Motor position of each actuator at Init, the engine's 0.
    origin: [f32; N],
    /// Last MIT command of each actuator, once powered.
    sent: [Option<Sent>; N],
    /// Actuators sent a frame whose reply the next `read` drains.
    pending: Mask<N>,
    /// `data[6]` and `data[7]` of the newest reply of each actuator.
    vitals: [Option<(u8, u8)>; N],
    /// Actuators with a reply since the last `telemetry`.
    fresh: Mask<N>,
    /// Whether the transport opened the interfaces.
    open: bool,
}

impl<T, const NB: usize, const N: usize> CubeMars<T, NB, N> {
    /// An adapter owning the actuators of `placement` and no other, with no
    /// interface bound yet: `open` fails until [`CubeMars::bind`] binds them.
    pub fn new<const M: usize>(transport: T, placement: Placement<NB, M>) -> Self {
        Self {
            transport,
            buses: placement.table.buses,
            actuators: Placed::new(&placement),
            interfaces: [None; NB],
            origin: [0.0; N],
            sent: [None; N],
            pending: Mask::NONE,
            vitals: [None; N],
            fresh: Mask::NONE,
            open: false,
        }
    }

    /// An adapter for `placement` with `plugins` bound to its buses by label,
    /// each a `Bus::by_dev("can0")` naming a CAN interface. No I/O.
    ///
    /// # Errors
    ///
    /// A plugin that binds no bus or one already bound, a bus left unbound, a
    /// plugin by serial id, and a plugin baud.
    pub fn bind<const M: usize>(
        transport: T,
        placement: Placement<NB, M>,
        plugins: &[Binding],
    ) -> Result<Self, CubeMarsError> {
        let mut adapter = Self::new(transport, placement);
        let bindings = binding::match_plugins(adapter.buses.map(|bus| bus.label), plugins)?;
        for (bus, binding) in adapter.buses.iter().zip(&bindings) {
            let interface = match binding.port {
                Port::ByDev(interface) => interface,
                Port::ById(_) => {
                    return Err(CubeMarsError::NotAnInterface {
                        bus: bus.label,
                        port: binding.port,
                    });
                }
            };
            // DECISION: a baud on a CAN plugin is refused, not ignored: the
            // adapter cannot set the bitrate, `ip link` does.
            if let Some(baud) = binding.baud {
                return Err(CubeMarsError::BaudOnCan {
                    bus: bus.label,
                    interface,
                    baud,
                });
            }
        }
        adapter.interfaces = bindings.map(|binding| match binding.port {
            Port::ByDev(interface) | Port::ById(interface) => Some(interface),
        });
        Ok(adapter)
    }

    /// The scripted transport, for the tests to inspect.
    #[cfg(test)]
    pub(crate) fn transport(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Label of bus `bus`.
    fn label(&self, bus: usize) -> &'static str {
        self.buses.get(bus).map_or("", |bus| bus.label)
    }

    /// Bus label and CAN id of actuator `index`.
    fn named(&self, index: usize) -> (&'static str, u8) {
        self.actuators
            .get(index)
            .map_or(("", 0), |actuator| (self.label(actuator.bus), actuator.id))
    }

    /// The bring-up error of actuator `index` failing with `source`.
    fn transport_error(&self, index: usize, source: TransportError) -> CubeMarsError {
        let (bus, id) = self.named(index);
        CubeMarsError::Transport {
            actuator: index,
            bus,
            id,
            source,
        }
    }

    /// Position of actuator `index` as the engine sees it: its offset from
    /// the Init position.
    fn engine_q(&self, index: usize, motor: f32) -> Q {
        Q(motor - self.origin.get(index).copied().unwrap_or(0.0))
    }

    /// Writes `feedback` into actuator `index`'s slots of `sample`.
    fn fill(&self, index: usize, feedback: &Feedback, sample: &mut Sample<'_, N>) {
        let gains = self
            .sent
            .get(index)
            .copied()
            .flatten()
            .map(|sent| (sent.kp, sent.kd))
            .or_else(|| {
                self.actuators
                    .get(index)
                    .map(|actuator| (actuator.kp, actuator.kd))
            });
        let slots = (
            sample.q.get_mut(index),
            sample.qd.get_mut(index),
            sample.tau.get_mut(index),
        );
        if let (Some(q), Some(qd), Some(tau)) = slots {
            *q = self.engine_q(index, feedback.p);
            *qd = Qd(feedback.v);
            *tau = Tau(feedback.t);
        }
        if let (Some((kp, kd)), Some(kp_slot), Some(kd_slot)) =
            (gains, sample.kp.get_mut(index), sample.kd.get_mut(index))
        {
            *kp_slot = Kp(kp);
            *kd_slot = Kd(kd);
        }
    }

    /// Keeps the temperature and the error code of `feedback`, actuator
    /// `index`'s newest reply, for the next `telemetry`.
    fn note(&mut self, index: usize, feedback: &Feedback) {
        if let Some(slot) = self.vitals.get_mut(index) {
            *slot = Some((feedback.temperature, feedback.fault));
            self.fresh.set(index);
        }
    }
}

#[expect(
    private_bounds,
    reason = "the CAN seam stays private: these helpers serve the Hal impl only"
)]
impl<T: Transport, const NB: usize, const N: usize> CubeMars<T, NB, N> {
    /// Sends `data` to actuator `index` and waits for its reply, matched on
    /// `data[0]`: [`ATTEMPTS`] tries, each skipping up to [`FOREIGN_FRAMES`]
    /// frames of other actuators. Bring-up only.
    fn ask(
        &mut self,
        index: usize,
        data: [u8; 8],
        timeout: Duration,
    ) -> Result<Feedback, CubeMarsError> {
        let Some(&Actuator { bus, id, .. }) = self.actuators.get(index) else {
            let (bus, id) = self.named(index);
            return Err(CubeMarsError::Missing {
                actuator: index,
                bus,
                id,
            });
        };
        for _ in 0..ATTEMPTS {
            self.transport
                .send(bus, id, data)
                .map_err(|source| self.transport_error(index, source))?;
            for _ in 0..FOREIGN_FRAMES {
                let reply = self
                    .transport
                    .wait(bus, timeout)
                    .map_err(|source| self.transport_error(index, source))?;
                match reply.map(frame::unpack) {
                    Some(feedback) if feedback.id == id => return Ok(feedback),
                    Some(_) => continue,
                    None => break,
                }
            }
        }
        Err(CubeMarsError::Missing {
            actuator: index,
            bus: self.label(bus),
            id,
        })
    }

    /// The owned engine indices, ascending, collected so `self` stays free.
    fn indices(&self) -> ([usize; N], usize) {
        let mut indices = [0; N];
        let mut len = 0;
        for (index, _) in self.actuators.iter() {
            if let Some(slot) = indices.get_mut(len) {
                *slot = index;
                len += 1;
            }
        }
        (indices, len)
    }

    /// Asks every actuator for its state with the exit frame, limp.
    fn ask_all(
        &mut self,
        mut each: impl FnMut(&mut Self, usize, Feedback) -> Result<(), CubeMarsError>,
    ) -> Result<(), CubeMarsError> {
        let (indices, len) = self.indices();
        for &index in indices.get(..len).unwrap_or_default() {
            let feedback = self.ask(index, EXIT, REPLY_TIMEOUT)?;
            each(self, index, feedback)?;
        }
        Ok(())
    }

    /// The MIT frame of actuator `index` for engine position `q`; `None`
    /// when a value is outside the frame's range, never clipped.
    fn mit(&self, index: usize, q: f32, v: f32, gains: (f32, f32), t: f32) -> Option<Sent> {
        let origin = self.origin.get(index).copied()?;
        let (kp, kd) = gains;
        let data = frame::pack(Mit {
            p: q + origin,
            v,
            kp,
            kd,
            t,
        })
        .ok()?;
        Some(Sent { data, q, kp, kd })
    }

    /// Sends the last command of every owned actuator once, marking each
    /// reply pending; a failed send is a bus error.
    fn send_all(&mut self, faults: &mut Faults<N>) {
        for (index, actuator) in self.actuators.iter() {
            let Some(sent) = self.sent.get(index).copied().flatten() else {
                continue;
            };
            match self.transport.send(actuator.bus, actuator.id, sent.data) {
                Ok(()) => self.pending.set(index),
                Err(_) => faults.bus_error.set(index),
            }
        }
    }
}

impl<T, const NB: usize, const N: usize> seal::Adapter for CubeMars<T, NB, N> {}

impl<T: Transport, const NB: usize, const N: usize> Hal<N> for CubeMars<T, NB, N> {
    type Error = CubeMarsError;

    /// Claims the placement's indices, one local group per CAN interface,
    /// each read every tick: `read` drains the replies of the frames `write`
    /// sent the tick before.
    fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, CubeMarsError> {
        if let Some((actuator, index)) = self.actuators.past() {
            return Err(CubeMarsError::PastTheRobot {
                actuator,
                index,
                actuators: N,
            });
        }
        if NB > MAX_GROUPS {
            return Err(CubeMarsError::TooManyBuses {
                buses: NB,
                max: MAX_GROUPS,
            });
        }
        let mut group = [Group(0); N];
        let mut frames = [0u32; NB];
        for (index, actuator) in self.actuators.iter() {
            if let Some(slot) = group.get_mut(index) {
                *slot = Group(u8::try_from(actuator.bus).unwrap_or(u8::MAX));
            }
            if let Some(count) = frames.get_mut(actuator.bus) {
                *count += 1;
            }
        }
        let busiest = frames.iter().copied().max().unwrap_or(0);
        Ok(Claim {
            owned: self.actuators.owned(),
            group,
            read_every: [core::num::NonZeroU32::MIN; N],
            read_cost: FRAME_TIME * busiest,
            write_cost: FRAME_TIME * busiest,
        })
    }

    /// Refuses two buses on one interface, then opens one CAN socket per
    /// interface (SRS FR-5.10).
    fn open(&mut self) -> Result<(), CubeMarsError> {
        let mut names = [""; NB];
        for (bus, name) in names.iter_mut().enumerate() {
            let label = self.label(bus);
            let Some(interface) = self.interfaces.get(bus).copied().flatten() else {
                return Err(CubeMarsError::UnboundBus { bus: label });
            };
            *name = interface;
        }
        for (bus, &interface) in names.iter().enumerate() {
            let earlier = names.get(..bus).unwrap_or_default();
            if let Some(first) = earlier.iter().position(|&seen| seen == interface) {
                return Err(CubeMarsError::SharedInterface {
                    first: self.label(first),
                    bus: self.label(bus),
                    interface,
                });
            }
        }
        self.transport
            .open(&names)
            .map_err(|(bus, source)| CubeMarsError::InterfaceOpen {
                bus: self.label(bus),
                interface: names.get(bus).copied().unwrap_or(""),
                source,
            })?;
        self.open = true;
        Ok(())
    }

    /// Takes where each actuator stands, asked limp with the exit frame, as
    /// its engine 0. The motor's stored zero is left alone: `FF×7 FE` is
    /// never sent.
    fn zero(&mut self) -> Result<(), CubeMarsError> {
        self.ask_all(|adapter, index, feedback| {
            if let Some(origin) = adapter.origin.get_mut(index) {
                *origin = feedback.p;
            }
            Ok(())
        })
    }

    /// Every actuator answers the exit frame (verify on hardware).
    fn ping(&mut self) -> Result<(), CubeMarsError> {
        self.ask_all(|_, _, _| Ok(()))
    }

    /// Every actuator's reply carries no error code and a temperature at
    /// most [`MAX_TEMPERATURE_C`]. The MIT reply carries no voltage.
    // DECISION: `data[7]` other than 0 is refused as an error code, per the
    // AK manual's MIT reply; cubemars_rs never read it (verify on hardware).
    fn health(&mut self) -> Result<(), CubeMarsError> {
        self.ask_all(|adapter, index, feedback| {
            let (bus, id) = adapter.named(index);
            if feedback.fault != 0 {
                return Err(CubeMarsError::Fault {
                    actuator: index,
                    bus,
                    id,
                    code: feedback.fault,
                });
            }
            if feedback.temperature > MAX_TEMPERATURE_C {
                return Err(CubeMarsError::Temperature {
                    actuator: index,
                    bus,
                    id,
                    measured_c: feedback.temperature,
                    limit_c: MAX_TEMPERATURE_C,
                });
            }
            Ok(())
        })
    }

    /// Nothing to set: MIT mode has no mode register, `torque_on` enters it.
    fn configure(&mut self) -> Result<(), CubeMarsError> {
        Ok(())
    }

    /// Asks every actuator for its state with the exit frame; the gains
    /// read back are the table's.
    fn read_all(&mut self, mut sample: Sample<'_, N>) -> Result<(), CubeMarsError> {
        self.ask_all(|adapter, index, feedback| {
            adapter.fill(index, &feedback, &mut sample);
            adapter.note(index, &feedback);
            Ok(())
        })
    }

    /// Enters MIT mode on each actuator, then sends each a command at the
    /// position its enter reply measured, zero velocity and torque, the
    /// table's gains: none jumps (SRS FR-5.2). Every enter comes first, so no
    /// question swallows the reply of a command, which is the first `read`'s.
    fn torque_on(&mut self) -> Result<(), CubeMarsError> {
        let (indices, len) = self.indices();
        let indices = indices.get(..len).unwrap_or_default();
        for &index in indices {
            let feedback = self.ask(index, ENTER, ENTER_TIMEOUT)?;
            let Some(actuator) = self.actuators.get(index).copied() else {
                continue;
            };
            let start = Mit {
                p: feedback.p,
                v: 0.0,
                kp: actuator.kp,
                kd: actuator.kd,
                t: 0.0,
            };
            // Never taken: a reply's position is inside the frame's range.
            let Ok(data) = frame::pack(start) else {
                continue;
            };
            let sent = Sent {
                data,
                q: self.engine_q(index, feedback.p).0,
                kp: actuator.kp,
                kd: actuator.kd,
            };
            if let Some(slot) = self.sent.get_mut(index) {
                *slot = Some(sent);
            }
        }
        for &index in indices {
            let (Some(actuator), Some(sent)) = (
                self.actuators.get(index).copied(),
                self.sent.get(index).copied().flatten(),
            ) else {
                continue;
            };
            self.transport
                .send(actuator.bus, actuator.id, sent.data)
                .map_err(|source| self.transport_error(index, source))?;
            self.pending.set(index);
        }
        Ok(())
    }

    /// Drains every interface without blocking, matching each reply on
    /// `data[0]`: the state is one tick old. The newest reply of an actuator
    /// wins; an actuator sent a frame without a reply is offline and stale.
    fn read(&mut self, mut sample: Sample<'_, N>, faults: &mut Faults<N>) -> Mask<N> {
        let mut refreshed = Mask::NONE;
        for bus in 0..NB {
            for _ in 0..DRAIN_LIMIT {
                match self.transport.recv(bus) {
                    Ok(Some(data)) => {
                        let feedback = frame::unpack(data);
                        if let Some(index) = self.actuators.find(bus, feedback.id) {
                            self.fill(index, &feedback, &mut sample);
                            self.note(index, &feedback);
                            refreshed.set(index);
                        }
                    }
                    Ok(None) => break,
                    Err(_) => {
                        for (index, actuator) in self.actuators.iter() {
                            if actuator.bus == bus {
                                faults.bus_error.set(index);
                            }
                        }
                        break;
                    }
                }
            }
        }
        let missing = self.actuators.owned().difference(refreshed);
        for index in missing.iter() {
            faults.stale.set(index);
            if self.pending.get(index) && !faults.bus_error.get(index) {
                faults.offline.set(index);
            }
        }
        self.pending = Mask::NONE;
        refreshed
    }

    /// One frame per owned actuator, never blocking. A written actuator gets
    /// a new command: `Q` written, else its last position; `Qd` and `Tau`
    /// written, else 0; `Kp` and `Kd` written, else the table's. A value
    /// outside the frame's range is rejected, never clipped, and the last
    /// command goes again.
    // DECISION (SRS FR-2.5): an actuator not written this tick, or rejected,
    // gets its last command again, so it keeps answering and `read` stays
    // fresh; the frame changes nothing on the motor.
    // DECISION: `Qd` and `Tau` unwritten this tick are 0, not latched: a
    // tick's command is what the tick wrote, as for the gains.
    fn write(&mut self, command: Command<'_, N>, batch: &Batch<'_, N>, faults: &mut Faults<N>) {
        let written = command.written;
        for (_, run) in batch.runs() {
            for &index in run {
                let (Some(actuator), Some(last)) = (
                    self.actuators.get(index).copied(),
                    self.sent.get(index).copied().flatten(),
                ) else {
                    continue;
                };
                let pick = |mask: Mask<N>, value: Option<f32>, otherwise: f32| {
                    value.filter(|_| mask.get(index)).unwrap_or(otherwise)
                };
                let q = pick(written.q, command.q.get(index).map(|q| q.0), last.q);
                let v = pick(written.qd, command.qd.get(index).map(|v| v.0), 0.0);
                let t = pick(written.tau, command.tau.get(index).map(|t| t.0), 0.0);
                let kp = pick(written.kp, command.kp.get(index).map(|k| k.0), actuator.kp);
                let kd = pick(written.kd, command.kd.get(index).map(|k| k.0), actuator.kd);
                match self.mit(index, q, v, (kp, kd), t) {
                    Some(sent) => {
                        if let Some(slot) = self.sent.get_mut(index) {
                            *slot = Some(sent);
                        }
                    }
                    None => faults.rejected.set(index),
                }
            }
        }
        self.send_all(faults);
    }

    /// Copies the temperature and the error code of each actuator's newest
    /// reply, kept by `read` and `read_all`, into `telemetry`: no frame
    /// sent, none received. Returns the actuators with a reply since the
    /// last call. The MIT reply carries no voltage.
    // DECISION: `data[6]` is °C with no offset, as `health` and cubemars_rs
    // read it, and `data[7]` the raw hardware error; both unverified on the
    // AK60-6 (docs/deferred.md).
    fn telemetry(&mut self, telemetry: &mut Telemetry<N>, _faults: &mut Faults<N>) -> Mask<N> {
        let fresh = self.fresh;
        self.fresh = Mask::NONE;
        for index in fresh.iter() {
            if let Some((celsius, code)) = self.vitals.get(index).copied().flatten() {
                telemetry.set_temperature_c(index, f32::from(celsius));
                telemetry.set_hardware_error(index, code);
            }
        }
        fresh
    }

    /// Sends each powered actuator a pure position command at `target.q`,
    /// zero velocity and torque, `Kp` and `Kd` as written this tick, else
    /// the table's (SRS FR-4.1). A target outside the frame's range keeps
    /// the last command.
    fn hold(&mut self, target: Command<'_, N>) {
        if !self.open {
            return;
        }
        let written = target.written;
        let (indices, len) = self.indices();
        for &index in indices.get(..len).unwrap_or_default() {
            let (Some(actuator), Some(&q)) =
                (self.actuators.get(index).copied(), target.q.get(index))
            else {
                continue;
            };
            if self.sent.get(index).copied().flatten().is_none() {
                continue;
            }
            let kp = match target.kp.get(index) {
                Some(kp) if written.kp.get(index) => kp.0,
                _ => actuator.kp,
            };
            let kd = match target.kd.get(index) {
                Some(kd) if written.kd.get(index) => kd.0,
                _ => actuator.kd,
            };
            match self.mit(index, q.0, 0.0, (kp, kd), 0.0) {
                Some(sent) => {
                    if let Some(slot) = self.sent.get_mut(index) {
                        *slot = Some(sent);
                    }
                }
                None => warn!(
                    actuator = index,
                    "hold target outside the AK60-6 frame's range, last command kept"
                ),
            }
        }
        let mut faults = Faults::default();
        self.send_all(&mut faults);
        for index in faults.bus_error.iter() {
            warn!(actuator = index, "hold frame not sent");
        }
    }

    /// Sends the exit frame to every actuator of every opened interface,
    /// powered or not, after the hold the engine sequenced (SRS FR-4.1,
    /// FR-4.2, FR-5.6). Up to [`TORQUE_OFF_ATTEMPTS`] sends each; one that
    /// fails them all is logged and the next actuator follows.
    // DECISION (SRS FR-4.1): torque off after the hold is the exit frame, as
    // the core sequences it; what the actuator then carries (the GeckoLeg
    // drops) is the hold duration's to cover.
    fn torque_off(&mut self) {
        if !self.open {
            return;
        }
        for (index, actuator) in self.actuators.iter() {
            let mut answer = Err(TransportError::NotOpen { bus: actuator.bus });
            for _ in 0..TORQUE_OFF_ATTEMPTS {
                answer = self.transport.send(actuator.bus, actuator.id, EXIT);
                if answer.is_ok() {
                    break;
                }
            }
            if let Err(error) = answer {
                warn!(
                    actuator = index,
                    bus = self.label(actuator.bus),
                    id = actuator.id,
                    attempts = TORQUE_OFF_ATTEMPTS,
                    %error,
                    "exit frame not sent"
                );
            }
        }
        self.sent = [None; N];
        self.pending = Mask::NONE;
    }

    /// Closes every interface.
    fn close(&mut self) {
        self.transport.close();
        self.open = false;
    }
}
