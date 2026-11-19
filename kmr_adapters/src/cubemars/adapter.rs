//! The CubeMars member on cubemars_rs: one SDK bus per CAN interface,
//! `MitAk2`, one command and its reply per actuator and tick.

use core::fmt;
use core::time::Duration;

use cubemars_rs::{
    ActuatorKind, Ak60_6, Ak80_8, Declaration, Direction, Effort, Error as SdkError, HasStart, Id,
    LimitError, MitAk2, NoStart, Position, StartPosition, State as SdkState, Temperature, Torque,
    Transport, TransportError as SdkTransportError, Velocity,
};
use kmr_core::hal::{Batch, Claim, Command, Faults, Group, Hal, Mask, Overrides, Sample, seal};
use kmr_core::state::{Kd, Kp, Q, Qd, Tau};
use kmr_core::telemetry::Telemetry;
use tracing::warn;

use super::config::{Actuator, Bus, Model, Placed, Placement, Table};
use crate::binding::{self, Binding, Port, Unmatched};

/// The SDK bus of one interface.
type Link<T> = cubemars_rs::Bus<MitAk2, T>;

// DECISION: 80 °C, under the AK windings' rating, for an actuator whose table
// gives no temperature limit; one that does is checked against its own.
/// Highest temperature `health` accepts when the table gives none, °C.
const MAX_TEMPERATURE_C: f32 = 80.0;
// DECISION: an 8-byte standard frame at 1 Mbit/s with worst-case stuffing,
// about 130 bits; the interface's bitrate is set by `ip link`, not here.
/// Bus time of one frame.
const FRAME_TIME: Duration = Duration::from_micros(130);
/// Local transaction groups one member may claim: `Group` is a `u8`.
const MAX_GROUPS: usize = 256;

/// Opens the CAN interface a bus is bound to. The seam the tests replace.
pub(crate) trait Open {
    /// What the SDK bus sends and receives through.
    type Transport: Transport;

    /// The transport of interface `interface`.
    fn open(&mut self, interface: &'static str) -> Result<Self::Transport, SdkError>;
}

/// Raw SocketCAN, cubemars_rs's transport: one `CAN_RAW` socket per
/// interface, set up beforehand with `ip link set can0 up type can bitrate
/// 1000000`.
#[derive(Clone, Copy, Debug, Default)]
pub struct SocketCan;

impl SocketCan {
    /// The SocketCAN opener.
    pub const fn new() -> Self {
        Self
    }
}

impl Open for SocketCan {
    type Transport = cubemars_rs::SocketCan;

    fn open(&mut self, interface: &'static str) -> Result<Self::Transport, SdkError> {
        Ok(cubemars_rs::SocketCan::open(interface)?)
    }
}

/// The SDK's hints after its error, when it has some.
struct Hints<'a>(&'a SdkError);

impl fmt::Display for Hints<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for hint in self.0.hint() {
            write!(f, "; maybe {hint}")?;
        }
        Ok(())
    }
}

/// Why the CubeMars bind or bring-up failed. Names the bus, and the
/// interface or the actuator when there is one.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
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
    /// Two actuators of one bus share a CAN id.
    #[error("actuator {actuator}: CAN id {id} on bus {bus} already taken by actuator {first}")]
    DuplicateId {
        /// Engine index of the later one.
        actuator: usize,
        /// Engine index of the earlier one.
        first: usize,
        /// Bus label.
        bus: &'static str,
        /// CAN id.
        id: u8,
    },
    /// A row cubemars_rs refuses before any I/O: its id or its gains.
    #[error("actuator {actuator}: CAN id {id} on bus {bus}: {source}")]
    Declaration {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// CAN id.
        id: u8,
        /// What cubemars_rs refused.
        source: SdkError,
    },
    /// The interface could not be opened, or its actuators not declared.
    #[error("bus {bus}: CAN interface {interface}: {source}{}", Hints(source))]
    Open {
        /// Bus label.
        bus: &'static str,
        /// Interface name.
        interface: &'static str,
        /// What cubemars_rs reported.
        source: SdkError,
    },
    /// The actuator failed a bring-up step: no reply, a fault code, a limit.
    #[error(
        "actuator {actuator}: CAN id {id} on bus {bus}: {source}{}",
        Hints(source)
    )]
    Actuator {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// CAN id.
        id: u8,
        /// What cubemars_rs reported.
        source: SdkError,
    },
    /// The temperature is above the table's limit, else [`MAX_TEMPERATURE_C`].
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
        /// Measured.
        measured_c: f32,
        /// The limit.
        limit_c: f32,
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

/// One MIT command, in joint terms.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Goal {
    /// Position, rad.
    q: f32,
    /// Velocity, rad/s.
    v: f32,
    /// Position gain.
    kp: f32,
    /// Velocity gain.
    kd: f32,
    /// Feed-forward torque, N·m.
    t: f32,
}

/// What one tick's command did to one actuator.
enum Outcome {
    /// Its reply came back.
    Replied,
    /// Refused before sending; the last command went instead.
    Refused,
    /// cubemars_rs cut its torque: a fault code or a reply past a limit.
    Cut,
    /// No reply.
    Silent,
    /// The interface failed.
    Wire,
}

/// The CubeMars adapter over `NB` CAN interfaces opened by `O`: one member
/// of the robot's hardware, owning the engine indices of its [`Placement`]
/// (SRS FR-5.1) out of the robot's `N` actuators.
///
/// Every actuator is an AK60-6 or AK80-8 in MIT mode (`MitAk2`). Positions
/// are joint positions as cubemars_rs tracks them: the pose an actuator
/// stands in at bring-up is its declared start, 0 unless set; the motor's
/// stored zero is never used nor written.
#[expect(
    private_bounds,
    reason = "the CAN seam stays private: users name `CubeMars<SocketCan, ..>` only"
)]
pub struct CubeMars<O: Open, const NB: usize, const N: usize> {
    /// Opens the interfaces.
    opener: O,
    /// Interfaces of the table.
    buses: [Bus; NB],
    /// The actuators the member owns, by engine index.
    actuators: Placed<N>,
    /// Interface name of each bus; `None` until bound.
    interfaces: [Option<&'static str>; NB],
    /// The SDK bus of each interface, once opened.
    links: [Option<Link<O::Transport>>; NB],
    /// Last command sent to each actuator, once powered.
    sent: [Option<Goal>; N],
    /// Newest reply of each actuator.
    states: [Option<SdkState>; N],
    /// Actuators with a reply the next `read` hands over.
    replied: Mask<N>,
    /// Actuators whose last command got no reply.
    silent: Mask<N>,
    /// Actuators whose last command failed on the interface.
    wire: Mask<N>,
    /// Actuators with a reply since the last `telemetry`.
    fresh: Mask<N>,
    /// Whether `open` ran, so `torque_off` and `close` have work.
    open: bool,
}

#[expect(private_bounds, reason = "the CAN seam stays private")]
impl<O: Open, const NB: usize, const N: usize> CubeMars<O, NB, N> {
    /// An adapter owning the actuators of `placement` and no other, with no
    /// interface bound yet: `open` fails until [`CubeMars::bind`] binds them.
    pub fn new<const M: usize>(opener: O, placement: Placement<NB, M>) -> Self {
        Self {
            opener,
            buses: placement.table.buses,
            actuators: Placed::new(&placement),
            interfaces: [None; NB],
            links: core::array::from_fn(|_| None),
            sent: [None; N],
            states: [None; N],
            replied: Mask::NONE,
            silent: Mask::NONE,
            wire: Mask::NONE,
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
        opener: O,
        placement: Placement<NB, M>,
        plugins: &[Binding],
    ) -> Result<Self, CubeMarsError> {
        let mut adapter = Self::new(opener, placement);
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

    /// An adapter for the actuators `add_actuator` declared, `table` in
    /// engine order, bus `b` on interface `interfaces[b]`; a bus without an
    /// interface carries no actuator. No I/O: the rows are checked by
    /// [`Hal::check`].
    pub fn declared(
        opener: O,
        table: Table<NB, N>,
        interfaces: [Option<&'static str>; NB],
    ) -> Self {
        let mut adapter = Self::new(opener, Placement::unchecked(table));
        adapter.interfaces = interfaces;
        adapter
    }

    /// Label of bus `bus`.
    fn label(&self, bus: usize) -> &'static str {
        self.buses.get(bus).map_or("", |bus| bus.label)
    }

    /// Whether an owned actuator sits on bus `bus`.
    fn carries(&self, bus: usize) -> bool {
        self.actuators
            .iter()
            .any(|(_, actuator)| actuator.bus == bus)
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

    /// The bring-up error of actuator `index` failing with `source`.
    fn failed(&self, index: usize, source: SdkError) -> CubeMarsError {
        let (bus, id) = self
            .actuators
            .get(index)
            .map_or(("", 0), |actuator| (self.label(actuator.bus), actuator.id));
        CubeMarsError::Actuator {
            actuator: index,
            bus,
            id,
            source,
        }
    }

    /// Runs `call` on the SDK handle of actuator `index`; `None` when the
    /// member does not own it or its interface is not open.
    fn with<R>(
        &mut self,
        index: usize,
        call: impl FnOnce(
            &mut cubemars_rs::ActuatorRef<'_, MitAk2, O::Transport, cubemars_rs::StdClock>,
        ) -> Result<R, SdkError>,
    ) -> Option<Result<R, SdkError>> {
        let actuator = *self.actuators.get(index)?;
        let link = self.links.get_mut(actuator.bus)?.as_mut()?;
        Some(Id::new(actuator.id).map_err(SdkError::from).and_then(|id| {
            let mut handle = link.actuator(id)?;
            call(&mut handle)
        }))
    }

    /// Sends `goal` to actuator `index` and keeps its reply.
    fn send(&mut self, index: usize, goal: Goal) -> Option<Result<SdkState, SdkError>> {
        let result = self.with(index, |handle| {
            handle.command(
                Position::rad(goal.q),
                Velocity::rad_per_s(goal.v),
                cubemars_rs::Kp::new(goal.kp)?,
                cubemars_rs::Kd::new(goal.kd)?,
                Torque::nm(goal.t),
            )
        })?;
        if let Ok(state) = result {
            if let Some(slot) = self.states.get_mut(index) {
                *slot = Some(state);
            }
            self.fresh.set(index);
        }
        Some(result)
    }

    /// Bring-up: runs `step` on every owned actuator in engine order, the
    /// first failure naming its actuator.
    fn each(
        &mut self,
        mut step: impl FnMut(&mut Self, usize) -> Option<Result<(), SdkError>>,
    ) -> Result<(), CubeMarsError> {
        let (indices, len) = self.indices();
        for &index in indices.get(..len).unwrap_or_default() {
            if let Some(Err(source)) = step(self, index) {
                return Err(self.failed(index, source));
            }
        }
        Ok(())
    }

    /// One tick's command to actuator `index`: `goal`, or the last command
    /// again when cubemars_rs refuses it.
    fn exchange(&mut self, index: usize, goal: Goal) -> Outcome {
        let last = self.sent.get(index).copied().flatten();
        let outcome = match self.send(index, goal) {
            None => return Outcome::Wire,
            Some(Ok(_)) => {
                if let Some(slot) = self.sent.get_mut(index) {
                    *slot = Some(goal);
                }
                return Outcome::Replied;
            }
            Some(Err(error)) => error,
        };
        let kind = classify(&outcome);
        if let Outcome::Cut = kind {
            warn!(actuator = index, %outcome, "cubemars_rs cut this actuator's torque");
        }
        // DECISION (SRS FR-2.5): a refused goal sends the last command again,
        // so the actuator keeps answering and `read` stays fresh.
        if let (Outcome::Refused, Some(last)) = (&kind, last)
            && last != goal
            && let Some(Ok(_)) = self.send(index, last)
        {
            self.replied.set(index);
        }
        kind
    }

    /// Writes actuator `index`'s newest reply into `sample`.
    fn fill(&self, index: usize, sample: &mut Sample<'_, N>) {
        let Some(state) = self.states.get(index).copied().flatten() else {
            return;
        };
        let tau = match state.effort {
            Effort::Torque(torque) => torque.as_nm(),
            Effort::Current(_) => 0.0,
        };
        if let (Some(q), Some(qd), Some(t)) = (
            sample.q.get_mut(index),
            sample.qd.get_mut(index),
            sample.tau.get_mut(index),
        ) {
            *q = Q(state.position.as_rad());
            *qd = Qd(state.velocity.as_rad_per_s());
            *t = Tau(tau);
        }
        let gains = self
            .sent
            .get(index)
            .copied()
            .flatten()
            .map(|goal| (goal.kp, goal.kd))
            .or_else(|| self.actuators.get(index).map(|row| (row.kp, row.kd)));
        if let (Some((kp, kd)), Some(kp_slot), Some(kd_slot)) =
            (gains, sample.kp.get_mut(index), sample.kd.get_mut(index))
        {
            *kp_slot = Kp(kp);
            *kd_slot = Kd(kd);
        }
    }
}

/// What a refused or failed command means for the tick.
fn classify(error: &SdkError) -> Outcome {
    match error {
        SdkError::Fault(_) | SdkError::Limit(LimitError::MeasuredOutsideRange { .. }) => {
            Outcome::Cut
        }
        SdkError::Validation(_) | SdkError::Limit(_) | SdkError::Zero(_) => Outcome::Refused,
        SdkError::Transport(
            SdkTransportError::NoReply { .. }
            | SdkTransportError::Timeout { .. }
            | SdkTransportError::NoFeedback { .. }
            | SdkTransportError::StaleFeedback { .. },
        ) => Outcome::Silent,
        _ => Outcome::Wire,
    }
}

/// The SDK declaration of `row`.
fn declaration(row: &Actuator) -> Result<ActuatorKind, SdkError> {
    let id = Id::new(row.id)?;
    Ok(match row.model {
        Model::Ak60_6 => settle(Ak60_6::declare(id), row)?.into(),
        Model::Ak80_8 => settle(Ak80_8::declare(id), row)?.into(),
    })
}

/// `declaration` with the start, gains, direction and limits of `row`.
fn settle<M>(
    declaration: Declaration<M, NoStart>,
    row: &Actuator,
) -> Result<Declaration<M, HasStart>, SdkError> {
    // DECISION: a start of exactly 0 is `ZeroHere`, any other `Declared`:
    // both resolve the offset from the first reply, the same way.
    let start = if row.start == 0.0 {
        StartPosition::ZeroHere
    } else {
        StartPosition::Declared(Position::rad(row.start))
    };
    let direction = if row.reversed {
        Direction::Reversed
    } else {
        Direction::Normal
    };
    let mut declaration = declaration
        .start(start)
        .gains(cubemars_rs::Kp::new(row.kp)?, cubemars_rs::Kd::new(row.kd)?)
        .direction(direction);
    let limits = row.limits;
    if let Some((min, max)) = limits.position {
        declaration = declaration.position_limit(Position::rad(min)..=Position::rad(max));
    }
    if let Some(max) = limits.velocity {
        declaration = declaration.velocity_limit(Velocity::rad_per_s(max));
    }
    if let Some(max) = limits.torque {
        declaration = declaration.torque_limit(Torque::nm(max));
    }
    if let Some(max) = limits.temperature {
        declaration = declaration.temperature_limit(Temperature::celsius(max));
    }
    if let Some(step) = limits.max_goal_step {
        declaration = declaration.max_goal_step(Position::rad(step));
    }
    Ok(declaration)
}

impl<O: Open, const NB: usize, const N: usize> seal::Adapter for CubeMars<O, NB, N> {}

impl<O: Open, const NB: usize, const N: usize> Hal<N> for CubeMars<O, NB, N> {
    type Error = CubeMarsError;

    /// Checks every row as cubemars_rs will (id, gains, an id twice on one
    /// bus) and claims the placement's indices, one local group per CAN
    /// interface, each read every tick: `read` hands over the replies of the
    /// commands `write` sent the tick before.
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
        for (index, row) in self.actuators.iter() {
            let bus = self.label(row.bus);
            if let Some((first, _)) = self
                .actuators
                .iter()
                .take_while(|(earlier, _)| *earlier < index)
                .find(|(_, other)| other.bus == row.bus && other.id == row.id)
            {
                return Err(CubeMarsError::DuplicateId {
                    actuator: index,
                    first,
                    bus,
                    id: row.id,
                });
            }
            declaration(row).map_err(|source| CubeMarsError::Declaration {
                actuator: index,
                bus,
                id: row.id,
                source,
            })?;
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
        // DECISION: `write` carries the whole exchange, a command and its
        // reply per actuator; `read` hands over what it kept, no frame.
        Ok(Claim {
            owned: self.actuators.owned(),
            group,
            read_every: [core::num::NonZeroU32::MIN; N],
            read_cost: Duration::ZERO,
            write_cost: FRAME_TIME * 2 * busiest,
        })
    }

    /// Refuses two buses on one interface, then opens each interface that
    /// carries an actuator and declares its actuators to a cubemars_rs bus
    /// (SRS FR-5.10). No frame is sent.
    fn open(&mut self) -> Result<(), CubeMarsError> {
        let mut names: [Option<&'static str>; NB] = [None; NB];
        for (bus, name) in names.iter_mut().enumerate() {
            if !self.carries(bus) {
                continue;
            }
            let Some(interface) = self.interfaces.get(bus).copied().flatten() else {
                return Err(CubeMarsError::UnboundBus {
                    bus: self.label(bus),
                });
            };
            *name = Some(interface);
        }
        for (bus, interface) in names.iter().enumerate() {
            let Some(interface) = *interface else {
                continue;
            };
            let earlier = names.get(..bus).unwrap_or_default();
            if let Some(first) = earlier.iter().position(|&seen| seen == Some(interface)) {
                return Err(CubeMarsError::SharedInterface {
                    first: self.label(first),
                    bus: self.label(bus),
                    interface,
                });
            }
        }
        self.open = true;
        for (bus, interface) in names.iter().enumerate() {
            let Some(interface) = *interface else {
                continue;
            };
            let label = self.label(bus);
            let opened = |source| CubeMarsError::Open {
                bus: label,
                interface,
                source,
            };
            let transport = self.opener.open(interface).map_err(opened)?;
            let mut builder = Link::builder(MitAk2, transport);
            for (_, row) in self.actuators.iter().filter(|(_, row)| row.bus == bus) {
                builder = builder.actuator(declaration(row).map_err(opened)?);
            }
            let link = builder.build().map_err(opened)?;
            if let Some(slot) = self.links.get_mut(bus) {
                *slot = Some(link);
            }
        }
        Ok(())
    }

    /// Nothing to do: cubemars_rs takes each actuator's zero from its first
    /// reply, which `ping` asks for.
    fn zero(&mut self) -> Result<(), CubeMarsError> {
        Ok(())
    }

    /// Every actuator enters MIT mode on a command of zero gains and zero
    /// torque, limp, and answers it; its reply resolves its zero.
    // DECISION: the MIT protocol has no question frame and cubemars_rs reads
    // an actuator only through a command, so the enter sequence, limp, is
    // the presence check; `torque_off` sends the exit frame on every exit.
    fn ping(&mut self) -> Result<(), CubeMarsError> {
        self.each(|adapter, index| {
            let state = adapter.with(index, |handle| {
                handle.enable_torque()?;
                Ok(handle.state())
            })?;
            let state = match state {
                Ok(state) => state,
                Err(error) => return Some(Err(error)),
            };
            if let (Some(slot), Some(state)) = (adapter.states.get_mut(index), state) {
                *slot = Some(state);
                adapter.fresh.set(index);
            }
            Some(Ok(()))
        })
    }

    /// Every actuator's newest reply is at most its temperature limit, else
    /// [`MAX_TEMPERATURE_C`]. A fault code already failed `ping`; the MIT
    /// reply carries no voltage.
    fn health(&mut self) -> Result<(), CubeMarsError> {
        for (index, row) in self.actuators.iter() {
            let Some(state) = self.states.get(index).copied().flatten() else {
                continue;
            };
            let measured_c = state.temperature.as_celsius();
            let limit_c = row.limits.temperature.unwrap_or(MAX_TEMPERATURE_C);
            if measured_c > limit_c {
                return Err(CubeMarsError::Temperature {
                    actuator: index,
                    bus: self.label(row.bus),
                    id: row.id,
                    measured_c,
                    limit_c,
                });
            }
        }
        Ok(())
    }

    /// Nothing to set: MIT mode has no mode register.
    fn configure(&mut self) -> Result<(), CubeMarsError> {
        Ok(())
    }

    /// Sends each actuator a limp command, zero gains and torque, and reads
    /// its reply; the gains read back are the table's.
    fn read_all(&mut self, mut sample: Sample<'_, N>) -> Result<(), CubeMarsError> {
        self.each(|adapter, index| {
            let q = adapter
                .states
                .get(index)
                .copied()
                .flatten()
                .map_or(0.0, |state| state.position.as_rad());
            let limp = Goal {
                q,
                v: 0.0,
                kp: 0.0,
                kd: 0.0,
                t: 0.0,
            };
            Some(adapter.send(index, limp)?.map(|_| ()))
        })?;
        for (index, _) in self.actuators.iter() {
            self.fill(index, &mut sample);
        }
        Ok(())
    }

    /// Sends each actuator a command at the position its last reply
    /// measured, zero velocity and torque, the table's gains: none jumps
    /// (SRS FR-5.2). Its reply is the first `read`'s.
    fn torque_on(&mut self) -> Result<(), CubeMarsError> {
        self.each(|adapter, index| {
            let row = *adapter.actuators.get(index)?;
            let state = adapter.states.get(index).copied().flatten()?;
            let goal = Goal {
                q: state.position.as_rad(),
                v: 0.0,
                kp: row.kp,
                kd: row.kd,
                t: 0.0,
            };
            let sent = adapter.send(index, goal)?;
            if sent.is_ok() {
                if let Some(slot) = adapter.sent.get_mut(index) {
                    *slot = Some(goal);
                }
                adapter.replied.set(index);
            }
            Some(sent.map(|_| ()))
        })
    }

    /// Hands over the replies `write` received: the state is one tick old.
    /// An actuator without one is stale, offline when it did not answer.
    fn read(&mut self, mut sample: Sample<'_, N>, faults: &mut Faults<N>) -> Mask<N> {
        let replied = self.replied;
        for index in replied.iter() {
            self.fill(index, &mut sample);
        }
        for index in self.actuators.owned().difference(replied).iter() {
            faults.stale.set(index);
            if self.silent.get(index) {
                faults.offline.set(index);
            }
            if self.wire.get(index) {
                faults.bus_error.set(index);
            }
        }
        self.replied = Mask::NONE;
        self.silent = Mask::NONE;
        self.wire = Mask::NONE;
        replied
    }

    /// One command per powered actuator, its reply awaited. A written
    /// actuator gets a new command: `Q` written, else its last position;
    /// `Qd` and `Tau` written, else 0; `Kp` and `Kd` written, else the
    /// table's. cubemars_rs refuses a value outside a declared limit or the
    /// frame's range, never clipped, and the last command goes again.
    // DECISION (SRS FR-2.5): an actuator not written this tick gets its last
    // command again, so it keeps answering and `read` stays fresh.
    // DECISION: `Qd` and `Tau` unwritten this tick are 0, not latched: a
    // tick's command is what the tick wrote, as for the gains.
    // DECISION (SDK boundary): cubemars_rs waits for each reply, at most its
    // 3 ms reply timeout; a silent actuator costs that wait once a tick.
    fn write(&mut self, command: Command<'_, N>, batch: &Batch<'_, N>, faults: &mut Faults<N>) {
        let written = command.written;
        let mut goals = self.sent;
        for (_, run) in batch.runs() {
            for &index in run {
                let (Some(row), Some(Some(last))) = (
                    self.actuators.get(index).copied(),
                    goals.get(index).copied(),
                ) else {
                    continue;
                };
                let pick = |mask: Mask<N>, value: Option<f32>, otherwise: f32| {
                    value.filter(|_| mask.get(index)).unwrap_or(otherwise)
                };
                let goal = Goal {
                    q: pick(written.q, command.q.get(index).map(|q| q.0), last.q),
                    v: pick(written.qd, command.qd.get(index).map(|v| v.0), 0.0),
                    t: pick(written.tau, command.tau.get(index).map(|t| t.0), 0.0),
                    kp: pick(written.kp, command.kp.get(index).map(|k| k.0), row.kp),
                    kd: pick(written.kd, command.kd.get(index).map(|k| k.0), row.kd),
                };
                if let Some(slot) = goals.get_mut(index) {
                    *slot = Some(goal);
                }
            }
        }
        let (indices, len) = self.indices();
        for &index in indices.get(..len).unwrap_or_default() {
            let Some(goal) = goals.get(index).copied().flatten() else {
                continue;
            };
            match self.exchange(index, goal) {
                Outcome::Replied => self.replied.set(index),
                Outcome::Refused => {
                    faults.rejected.set(index);
                }
                Outcome::Cut => {
                    faults.rejected.set(index);
                    if let Some(slot) = self.sent.get_mut(index) {
                        *slot = None;
                    }
                }
                Outcome::Silent => {
                    faults.offline.set(index);
                    self.silent.set(index);
                }
                Outcome::Wire => {
                    faults.bus_error.set(index);
                    self.wire.set(index);
                }
            }
        }
    }

    /// Copies the temperature of each actuator's newest reply into
    /// `telemetry`: no frame sent, none received. Returns the actuators with
    /// a reply since the last call. The MIT reply carries no voltage, and a
    /// fault code arrives as cubemars_rs's error, not as a raw byte.
    fn telemetry(&mut self, telemetry: &mut Telemetry<N>, _faults: &mut Faults<N>) -> Mask<N> {
        let fresh = self.fresh;
        self.fresh = Mask::NONE;
        for index in fresh.iter() {
            if let Some(state) = self.states.get(index).copied().flatten() {
                telemetry.set_temperature_c(index, state.temperature.as_celsius());
            }
        }
        fresh
    }

    /// Sends each powered actuator a pure position command at `target.q`,
    /// zero velocity and torque, `Kp` and `Kd` as written this tick, else
    /// the table's (SRS FR-4.1). A target cubemars_rs refuses keeps the last
    /// command.
    fn hold(&mut self, target: Command<'_, N>) {
        if !self.open {
            return;
        }
        let written = target.written;
        let (indices, len) = self.indices();
        for &index in indices.get(..len).unwrap_or_default() {
            let (Some(row), Some(&q), Some(Some(_))) = (
                self.actuators.get(index).copied(),
                target.q.get(index),
                self.sent.get(index).copied(),
            ) else {
                continue;
            };
            let kp = match target.kp.get(index) {
                Some(kp) if written.kp.get(index) => kp.0,
                _ => row.kp,
            };
            let kd = match target.kd.get(index) {
                Some(kd) if written.kd.get(index) => kd.0,
                _ => row.kd,
            };
            let goal = Goal {
                q: q.0,
                v: 0.0,
                kp,
                kd,
                t: 0.0,
            };
            if !matches!(self.exchange(index, goal), Outcome::Replied) {
                warn!(actuator = index, "hold command not answered");
            }
        }
    }

    /// Sends the exit frame to every actuator of every opened interface,
    /// powered or not, after the hold the engine sequenced (SRS FR-4.1,
    /// FR-4.2, FR-5.6). A failure is logged and the next actuator follows.
    // DECISION (SRS FR-4.1): torque off after the hold is the exit frame, as
    // the core sequences it; what the actuator then carries (the GeckoLeg
    // drops) is the hold duration's to cover.
    fn torque_off(&mut self) {
        if !self.open {
            return;
        }
        for bus in 0..NB {
            let label = self.label(bus);
            if let Some(Some(link)) = self.links.get_mut(bus)
                && let Err(error) = link.disable_torque_all()
            {
                warn!(bus = label, %error, "exit frame not sent to every actuator");
            }
        }
        self.sent = [None; N];
        self.replied = Mask::NONE;
    }

    /// Closes every interface.
    fn close(&mut self) {
        self.links = core::array::from_fn(|_| None);
        self.open = false;
    }
}
