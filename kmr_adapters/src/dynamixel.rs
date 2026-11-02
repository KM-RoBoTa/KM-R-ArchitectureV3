//! The ROBOTIS Dynamixel adapter: Protocol 1 and 2 actuators in position
//! mode, one transaction per (bus, series) group where the protocol allows it.
//!
//! The SDK sits behind a private backend, so the adapter is tested on a
//! scripted one without hardware.

mod backend;
mod config;
#[cfg(feature = "dynamixel")]
mod dxl_rs;
mod io;
mod port;
mod units;

#[cfg(test)]
mod scripted;
#[cfg(test)]
mod tests;

use core::fmt;
use core::time::Duration;
use std::path::Path;

use kmr_core::hal::{Batch, Claim, Command, Faults, Hal, Mask, Overrides, Sample, seal};
use kmr_core::state::{JOINTS, Q};
use tracing::{debug, warn};

use crate::binding::{self, Binding, Port};
use backend::{Backend, BackendError, POSITION_MODE, PortSpec, Setting};
use io::{Goals, Groups, Outcome};
use port::DevicePath;

#[cfg(feature = "dynamixel")]
pub use self::dxl_rs::DxlRs;
use config::Placed;
pub use config::{
    Actuator, Bus, DEFAULT_READ_EVERY, Mode, Model, Placement, Protocol, Series, Table, Torque,
    Zero,
};

/// The read and write cost `check` declares for `table` with every actuator
/// owned, in const evaluation: the presets' `READ_COST` and `WRITE_COST`
/// (FR-035 (SRS NFR-11, FR-5.3)).
pub(crate) const fn declared_costs<const NB: usize, const N: usize>(
    table: &Table<NB, N>,
) -> (Duration, Duration) {
    io::declared_costs(table, backend::STATE_READS)
}

/// Ping attempts per actuator before it counts as missing (FR-007).
const PING_ATTEMPTS: usize = 3;
/// Local transaction groups one member may claim: `Group` is a `u8`.
const MAX_GROUPS: usize = 256;
/// Series a model can be of; the match stops compiling when one is added.
const SERIES: usize = match Series::X {
    Series::Ax | Series::X => 2,
};
/// Force torque off attempts per actuator, back to back, before a warning
/// names it.
const TORQUE_OFF_ATTEMPTS: usize = 3;

/// Why the Dynamixel bind or bring-up failed. Names the bus, and the port
/// or the actuator when there is one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum DynamixelError {
    /// More buses than the backend can open.
    #[error("{buses} buses declared, the Dynamixel backend opens at most {max}")]
    TooManyBuses {
        /// Buses declared.
        buses: usize,
        /// Ports the backend opens.
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
    /// A bus plugin baud a model on its bus does not accept.
    #[error("actuator {actuator}: {} on bus {bus} does not accept {baud} baud, set by its bus plugin", .model.name())]
    BaudRejected {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Model of the actuator.
        model: Model,
        /// Baud of the plugin.
        baud: u32,
    },
    /// The port names no single device.
    #[error("bus {bus}: {port} names no single device; check the bus plugin and the cable")]
    PortNotFound {
        /// Bus label.
        bus: &'static str,
        /// Port of the bus plugin.
        port: Port,
    },
    /// The device is not a character device this process can read and write
    /// (FR-008).
    #[error(
        "bus {bus}: the device of {port} is not a character device this user can read and write; check the dialout group"
    )]
    PortAccess {
        /// Bus label.
        bus: &'static str,
        /// Port of the bus plugin.
        port: Port,
    },
    /// Two buses resolve to one device: the SDK would merge them.
    #[error("buses {first} and {bus} resolve to the same device; check the bus plugins")]
    SharedPort {
        /// The earlier bus.
        first: &'static str,
        /// The later bus.
        bus: &'static str,
    },
    /// The port could not be opened to flush its queues.
    #[error("bus {bus}: the device of {port} could not be flushed")]
    PortFlush {
        /// Bus label.
        bus: &'static str,
        /// Port of the bus plugin.
        port: Port,
    },
    /// The USB latency timer does not read back as 1 ms (research R8).
    #[error(
        "bus {bus}: the latency timer of {port} is not 1 ms; install the udev rule ACTION==\"add\", SUBSYSTEM==\"usb-serial\", ATTR{{latency_timer}}=\"1\""
    )]
    LatencyTimer {
        /// Bus label.
        bus: &'static str,
        /// Port of the bus plugin.
        port: Port,
    },
    /// The backend could not open the port.
    #[error("bus {bus}: the device of {port} could not be opened: {source}")]
    PortOpen {
        /// Bus label.
        bus: &'static str,
        /// Port of the bus plugin.
        port: Port,
        /// What the backend reported.
        #[expect(
            private_interfaces,
            reason = "the SDK seam stays private: callers get its message, not its type"
        )]
        source: BackendError,
    },
    /// The SDK could not be brought up, on no port in particular.
    #[error("the Dynamixel SDK could not be brought up: {source}")]
    Sdk {
        /// What the backend reported.
        #[expect(
            private_interfaces,
            reason = "the SDK seam stays private: callers get its message, not its type"
        )]
        source: BackendError,
    },
    /// The actuator does not answer.
    #[error("actuator {actuator}: id {id} on bus {bus} does not answer")]
    Missing {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
    },
    /// The input voltage is outside the limits the actuator holds.
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: input voltage {} V outside the actuator's limits {}–{} V",
        Volts(*.measured_dv),
        Volts(*.min_dv),
        Volts(*.max_dv)
    )]
    Voltage {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// Present input voltage, in 0.1 V.
        measured_dv: u16,
        /// The actuator's Min Voltage Limit, in 0.1 V.
        min_dv: u16,
        /// The actuator's Max Voltage Limit, in 0.1 V.
        max_dv: u16,
    },
    /// The temperature is above the limit the actuator holds.
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: temperature {measured_c} °C above the actuator's limit {limit_c} °C"
    )]
    Temperature {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// Present temperature, in °C.
        measured_c: u8,
        /// The actuator's Temperature Limit, in °C.
        limit_c: u8,
    },
    /// The actuator reports a hardware error (Protocol 2).
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: Hardware Error Status {}",
        Alarms(*.status)
    )]
    HardwareError {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// Hardware Error Status bits.
        status: u8,
    },
    /// The present position is outside the register range: a homing offset
    /// or another operating mode moved it.
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: present position {raw} outside the register range 0–{max}"
    )]
    PositionRange {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// Present position, raw.
        raw: i64,
        /// Highest position of the model, raw.
        max: u32,
    },
    /// The present position is outside the CW/CCW Angle Limit: powered, the
    /// actuator would not hold there (Protocol 1).
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: present position {raw} outside the CW/CCW Angle Limit {cw}–{ccw}"
    )]
    OutsideAngleLimits {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// Present position, raw.
        raw: u32,
        /// CW Angle Limit, raw.
        cw: u32,
        /// CCW Angle Limit, raw.
        ccw: u32,
    },
    /// CW and CCW Angle Limit both 0: wheel mode, not joint mode
    /// (Protocol 1).
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: CW and CCW Angle Limit both 0: wheel mode, not joint mode"
    )]
    WheelMode {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
    },
    /// CW and CCW Angle Limit both at the model's multi-turn value: multi-turn
    /// mode, not joint mode (Protocol 1 MX-64).
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: CW and CCW Angle Limit both {limit}: multi-turn mode, not joint mode"
    )]
    MultiTurnMode {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// CW and CCW Angle Limit, raw.
        limit: u32,
    },
    /// CW Angle Limit above CCW Angle Limit (Protocol 1).
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: CW Angle Limit {cw} above CCW Angle Limit {ccw}"
    )]
    AngleLimitsReversed {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// CW Angle Limit, raw.
        cw: u32,
        /// CCW Angle Limit, raw.
        ccw: u32,
    },
    /// The Operating Mode is not the one the table asks for (Protocol 2).
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: Operating Mode {}, expected {}",
        ModeName(*.found),
        ModeName(*.expected)
    )]
    OperatingMode {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// Operating Mode read.
        found: u8,
        /// Operating Mode the table asks for.
        expected: u8,
    },
    /// Torque Ctrl Mode Enable set: the actuator drives Goal Torque and
    /// ignores Goal Position (Protocol 1 MX-64).
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: Torque Ctrl Mode Enable {found}, expected 0: it ignores Goal Position"
    )]
    TorqueCtrlMode {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// Torque Ctrl Mode Enable read.
        found: u8,
    },
    /// The Model Number read is not the declared model's: another model
    /// answers at that id, its registers at another scale.
    #[error(
        "actuator {actuator}: id {id} on bus {bus}: Model Number {found}, expected {expected} for the declared {}",
        .declared.name()
    )]
    ModelMismatch {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// Model the table declares.
        declared: Model,
        /// Model Number read.
        found: u16,
        /// Model Number the declared model answers with.
        expected: u16,
    },
    /// A transaction with one actuator failed.
    #[error("actuator {actuator}: id {id} on bus {bus}: {source}")]
    Actuator {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// What the backend reported.
        #[expect(
            private_interfaces,
            reason = "the SDK seam stays private: callers get its message, not its type"
        )]
        source: BackendError,
    },
    /// A transaction on a whole bus failed.
    #[error("bus {bus}: {source}")]
    Backend {
        /// Bus label.
        bus: &'static str,
        /// What the backend reported.
        #[expect(
            private_interfaces,
            reason = "the SDK seam stays private: callers get its message, not its type"
        )]
        source: BackendError,
    },
}

/// Stands in for the port of a bus no plugin bound, in an error that cannot
/// name one.
const UNBOUND: Port = Port::ByDev("");

/// A voltage in 0.1 V, printed in volts: 111 is `11.1`.
struct Volts(u16);

impl fmt::Display for Volts {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.0 / 10, self.0 % 10)
    }
}

/// An X series Operating Mode value, printed with its name: `3 (position)`.
struct ModeName(u8);

impl fmt::Display for ModeName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self.0 {
            0 => "current",
            1 => "velocity",
            3 => "position",
            4 => "extended position",
            5 => "current-based position",
            16 => "PWM",
            _ => "unknown",
        };
        write!(f, "{} ({name})", self.0)
    }
}

/// What each bit of the X series Hardware Error Status reports, bit 0 first.
const ALARMS: [&str; 8] = [
    "input voltage",
    "bit 1",
    "overheating",
    "motor encoder",
    "electrical shock",
    "overload",
    "bit 6",
    "bit 7",
];

/// Hardware Error Status bits, printed with what each set bit reports:
/// `0b00100100 (overheating, overload)`.
struct Alarms(u8);

impl fmt::Display for Alarms {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:#010b}", self.0)?;
        let mut separator = " (";
        for (bit, alarm) in ALARMS.iter().enumerate() {
            if self.0 & (1 << bit) != 0 {
                write!(f, "{separator}{alarm}")?;
                separator = ", ";
            }
        }
        if self.0 == 0 {
            Ok(())
        } else {
            f.write_str(")")
        }
    }
}

/// The Dynamixel adapter over `NB` buses, talking through backend `B`: one
/// member of the robot's hardware, owning the engine indices of its
/// [`Placement`] (SRS FR-5.1).
///
/// One per process: the C SDK keeps global port state (research R7).
pub struct Dynamixel<B, const NB: usize> {
    /// The SDK seam.
    backend: B,
    /// Buses, with the bus plugins' bauds.
    buses: [Bus; NB],
    /// The actuators the member owns, by engine index.
    actuators: Placed,
    /// Port of each bus, from its bus plugin; `None` until bound.
    ports: [Option<Port>; NB],
    /// Where serial ids are resolved; `None` is [`port::BY_ID`].
    by_id: Option<&'static Path>,
    /// Where latency timers live; `None` is [`port::USB_SERIAL_SYSFS`].
    sysfs: Option<&'static Path>,
    /// Transaction groups, built by `check`.
    groups: Groups,
    /// Device of each bus, once resolved.
    devices: [Option<DevicePath>; NB],
    /// Raw goal range of each actuator: CW to CCW Angle Limit once
    /// `configure` read them on Protocol 1, else any.
    limits: [(u32, u32); JOINTS],
    /// Whether the backend opened the ports.
    open: bool,
    /// `read` calls so far, for the cadence.
    reads: u64,
}

impl<B, const NB: usize> Dynamixel<B, NB> {
    /// An adapter owning the actuators of `placement` and no other, with no
    /// bus bound yet: `open` fails until [`Dynamixel::bind`] binds them.
    pub const fn new<const M: usize>(backend: B, placement: Placement<NB, M>) -> Self {
        Self {
            backend,
            buses: placement.table.buses,
            actuators: Placed::new(&placement),
            ports: [None; NB],
            by_id: None,
            sysfs: None,
            groups: Groups::EMPTY,
            devices: [None; NB],
            limits: [(0, u32::MAX); JOINTS],
            open: false,
            reads: 0,
        }
    }

    /// An adapter for `placement` with `plugins` bound to its buses by label,
    /// each plugin baud replacing its bus's (FR-030 (SRS FR-6.3, FR-6.4)).
    /// No I/O: the backend is not called.
    ///
    /// # Errors
    ///
    /// A plugin that binds no bus or one already bound, a bus left unbound,
    /// and a plugin baud a model on its bus rejects (FR-027 (SRS FR-6.4)).
    pub fn bind<const M: usize>(
        backend: B,
        placement: Placement<NB, M>,
        plugins: &[Binding],
    ) -> Result<Self, DynamixelError> {
        let mut adapter = Self::new(backend, placement);
        adapter.attach(plugins)?;
        Ok(adapter)
    }

    /// Binds `plugins` to the buses and takes their bauds; changes nothing
    /// on an error.
    fn attach(&mut self, plugins: &[Binding]) -> Result<(), DynamixelError> {
        let bindings = binding::match_plugins(self.buses.map(|bus| bus.label), plugins)?;
        for (index, actuator) in self.actuators.iter() {
            let baud = bindings.get(actuator.bus).and_then(|binding| binding.baud);
            if let Some(baud) = baud
                && !actuator.model.accepts_baud(baud)
            {
                return Err(DynamixelError::BaudRejected {
                    actuator: index,
                    bus: self.label(actuator.bus),
                    model: actuator.model,
                    baud,
                });
            }
        }
        for (bus, binding) in self.buses.iter_mut().zip(&bindings) {
            if let Some(baud) = binding.baud {
                bus.baud = baud;
            }
        }
        self.ports = bindings.map(|binding| Some(binding.port));
        Ok(())
    }

    /// Resolves serial ids under `by_id` and latency timers under `sysfs`.
    #[cfg(test)]
    pub(crate) fn with_roots(mut self, by_id: &'static Path, sysfs: &'static Path) -> Self {
        self.by_id = Some(by_id);
        self.sysfs = Some(sysfs);
        self
    }

    /// Label of bus `bus`.
    fn label(&self, bus: usize) -> &'static str {
        self.buses.get(bus).map_or("", |bus| bus.label)
    }

    /// Port of bus `bus`, from its bus plugin.
    fn port(&self, bus: usize) -> Port {
        self.ports.get(bus).copied().flatten().unwrap_or(UNBOUND)
    }

    /// Bus label and firmware id of actuator `index`.
    fn named(&self, index: usize) -> (&'static str, u8) {
        self.actuators
            .get(index)
            .map_or(("", 0), |actuator| (self.label(actuator.bus), actuator.id))
    }

    /// The bring-up error of actuator `index` failing with `error`: missing
    /// on a timeout, a failed transaction with it otherwise.
    fn actuator_error(&self, index: usize, error: BackendError) -> DynamixelError {
        let (bus, id) = self.named(index);
        match error {
            BackendError::Timeout => DynamixelError::Missing {
                actuator: index,
                bus,
                id,
            },
            source => DynamixelError::Actuator {
                actuator: index,
                bus,
                id,
                source,
            },
        }
    }

    /// The bring-up error of actuator `index`, whose present position `raw`
    /// is outside the register range.
    fn position_range(&self, index: usize, raw: i64) -> DynamixelError {
        let (bus, id) = self.named(index);
        let max = self
            .actuators
            .get(index)
            .map_or(0, |actuator| units::max_position(actuator.model));
        DynamixelError::PositionRange {
            actuator: index,
            bus,
            id,
            raw,
            max,
        }
    }

    /// The bring-up error of actuator `index`, whose present position `raw`
    /// is outside its angle limits: powered, it would not hold there.
    fn outside_limits(&self, index: usize, raw: u32) -> DynamixelError {
        let (bus, id) = self.named(index);
        let (cw, ccw) = self.limits.get(index).copied().unwrap_or_default();
        DynamixelError::OutsideAngleLimits {
            actuator: index,
            bus,
            id,
            raw,
            cw,
            ccw,
        }
    }

    /// Whether raw goal `raw` is inside the angle limits of actuator `index`.
    fn within_limits(&self, index: usize, raw: u32) -> bool {
        self.limits
            .get(index)
            .is_some_and(|&(low, high)| (low..=high).contains(&raw))
    }

    /// Raw goal position of actuator `index` for `q`: [`io::position_goal`],
    /// and `None` too outside its angle limits, never clamped.
    fn goal(&self, index: usize, q: Q) -> Option<u32> {
        let actuator = self.actuators.get(index)?;
        io::position_goal(actuator, q).filter(|&raw| self.within_limits(index, raw))
    }

    /// The bring-up error of the actuator a read `failed` on, if any. A
    /// failed group transaction names the bus, not one of its actuators.
    fn read_failure(&self, failed: Option<(usize, Outcome)>) -> Result<(), DynamixelError> {
        match failed {
            None | Some((_, Outcome::Read)) => Ok(()),
            Some((index, Outcome::Lost(error))) => Err(self.actuator_error(index, error)),
            Some((index, Outcome::Bus(source))) => Err(DynamixelError::Backend {
                bus: self.named(index).0,
                source,
            }),
            Some((index, Outcome::Invalid(raw))) => Err(self.position_range(index, raw)),
        }
    }
}

impl<B, const NB: usize> seal::Adapter for Dynamixel<B, NB> {}

impl<B: Backend, const NB: usize> Hal for Dynamixel<B, NB> {
    type Error = DynamixelError;

    /// Claims the placement's indices, grouped per (bus, series) with local
    /// ids from 0; each bus is read every `read_every` ticks, else every
    /// [`DEFAULT_READ_EVERY`] (FR-009). `Overrides` carries no cadence: it is
    /// a table field (core K3.8).
    fn check(&mut self, _overrides: Overrides) -> Result<Claim, DynamixelError> {
        if NB > B::MAX_PORTS {
            return Err(DynamixelError::TooManyBuses {
                buses: NB,
                max: B::MAX_PORTS,
            });
        }
        // One group per (bus, series) and at most `MAX_PORTS` buses: the local
        // numbers fit a `Group` for every backend that builds.
        const {
            assert!(
                SERIES * B::MAX_PORTS <= MAX_GROUPS,
                "too many ports for u8 groups"
            )
        };
        let every = self
            .buses
            .map(|bus| bus.read_every.unwrap_or(DEFAULT_READ_EVERY));
        self.groups = Groups::build(&self.actuators, &every);
        let unread = io::unread_rates(&self.actuators);
        for index in (0..JOINTS).filter(|&index| unread.get(index)) {
            warn!(
                actuator = index,
                "Protocol 1: Qd and Tau are never read and keep their last value (research R12)"
            );
        }
        let (read_cost, write_cost) = self.groups.costs(&self.buses, B::STATE_READS);
        Ok(Claim {
            owned: self.actuators.owned(),
            group: self.groups.numbers(),
            read_every: self.groups.cadence(),
            read_cost,
            write_cost,
        })
    }

    /// Resolves and pre-flights the port of every bus, refuses two buses on
    /// one device, then flushes each and sets its latency timer, then opens
    /// them all at once (FR-004, FR-008 (SRS FR-5.10)).
    fn open(&mut self) -> Result<(), DynamixelError> {
        let by_id = self.by_id.unwrap_or(Path::new(port::BY_ID));
        let sysfs = self.sysfs.unwrap_or(Path::new(port::USB_SERIAL_SYSFS));
        for bus in 0..NB {
            let label = self.label(bus);
            let Some(plugin) = self.ports.get(bus).copied().flatten() else {
                return Err(DynamixelError::UnboundBus { bus: label });
            };
            let device = port::locate(by_id, plugin).map_err(|_| DynamixelError::PortNotFound {
                bus: label,
                port: plugin,
            })?;
            let earlier = self.devices.get(..bus).unwrap_or_default();
            if let Some(first) = earlier.iter().position(|&seen| seen == Some(device)) {
                return Err(DynamixelError::SharedPort {
                    first: self.label(first),
                    bus: label,
                });
            }
            port::preflight(device.as_path()).map_err(|_| DynamixelError::PortAccess {
                bus: label,
                port: plugin,
            })?;
            if let Some(slot) = self.devices.get_mut(bus) {
                *slot = Some(device);
            }
        }
        for (bus, device) in self.devices.iter().enumerate() {
            let Some(device) = device else { continue };
            let (label, plugin) = (self.label(bus), self.port(bus));
            port::flush(device.as_path()).map_err(|_| DynamixelError::PortFlush {
                bus: label,
                port: plugin,
            })?;
            port::enforce_latency_timer(sysfs, device.as_path()).map_err(|_| {
                DynamixelError::LatencyTimer {
                    bus: label,
                    port: plugin,
                }
            })?;
        }
        let mut specs = [PortSpec {
            path: Path::new(""),
            baud: 0,
        }; NB];
        for ((spec, device), bus) in specs.iter_mut().zip(&self.devices).zip(&self.buses) {
            if let Some(device) = device {
                *spec = PortSpec {
                    path: device.as_path(),
                    baud: bus.baud,
                };
            }
        }
        self.backend
            .open_ports(&specs)
            .map_err(|failure| match failure.port {
                Some(port) => DynamixelError::PortOpen {
                    bus: self.label(port),
                    port: self.port(port),
                    source: failure.source,
                },
                None => DynamixelError::Sdk {
                    source: failure.source,
                },
            })?;
        self.open = true;
        Ok(())
    }

    /// Nothing to write: Dynamixel encoders are absolute and the zero is a
    /// software offset, each actuator's `Zero` read by `units::origin`
    /// (FR-005; D8, assumed, pending human).
    fn zero(&mut self) -> Result<(), DynamixelError> {
        Ok(())
    }

    /// Pings each actuator up to [`PING_ATTEMPTS`] times, without sleep,
    /// then reads its Model Number once and refuses another model's: no
    /// register is read through the declared table before that.
    fn ping(&mut self) -> Result<(), DynamixelError> {
        for (index, actuator) in self.actuators.iter() {
            let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
            let mut answer = Err(BackendError::Timeout);
            for _ in 0..PING_ATTEMPTS {
                answer = self.backend.ping(bus, model, id);
                if answer.is_ok() {
                    break;
                }
            }
            answer.map_err(|error| self.actuator_error(index, error))?;
            let found = self
                .backend
                .read_model_number(bus, model, id)
                .map_err(|error| self.actuator_error(index, error))?;
            let expected = model.number();
            if found != expected {
                return Err(DynamixelError::ModelMismatch {
                    actuator: index,
                    bus: self.label(bus),
                    id,
                    declared: model,
                    found,
                    expected,
                });
            }
        }
        Ok(())
    }

    /// Reads every actuator's voltage and temperature and checks them
    /// against the limits it holds in its own control table, and on
    /// Protocol 2 its Hardware Error Status. Inside those limits but outside
    /// the model's datasheet range is one warning per value, no error.
    fn health(&mut self) -> Result<(), DynamixelError> {
        for (index, actuator) in self.actuators.iter() {
            let (model, id) = (actuator.model, actuator.id);
            let health = self
                .backend
                .read_health(actuator.bus, model, id)
                .map_err(|error| self.actuator_error(index, error))?;
            let bus = self.label(actuator.bus);
            if model.protocol() == Protocol::Two && health.error_status != 0 {
                return Err(DynamixelError::HardwareError {
                    actuator: index,
                    bus,
                    id,
                    status: health.error_status,
                });
            }
            let (min_dv, max_dv) = (health.min_voltage_dv, health.max_voltage_dv);
            if !(min_dv..=max_dv).contains(&health.voltage_dv) {
                return Err(DynamixelError::Voltage {
                    actuator: index,
                    bus,
                    id,
                    measured_dv: health.voltage_dv,
                    min_dv,
                    max_dv,
                });
            }
            if health.temperature_c > health.temperature_limit_c {
                return Err(DynamixelError::Temperature {
                    actuator: index,
                    bus,
                    id,
                    measured_c: health.temperature_c,
                    limit_c: health.temperature_limit_c,
                });
            }
            let (low_dv, high_dv) = model.datasheet_voltage_dv();
            if !(low_dv..=high_dv).contains(&health.voltage_dv) {
                warn!(
                    actuator = index,
                    bus,
                    id,
                    "input voltage {} V outside the {} datasheet range {}–{} V, inside the actuator's limits {}–{} V",
                    Volts(health.voltage_dv),
                    model.name(),
                    Volts(low_dv),
                    Volts(high_dv),
                    Volts(min_dv),
                    Volts(max_dv),
                );
            }
            let hottest = model.datasheet_max_temperature_c();
            if health.temperature_c > hottest {
                warn!(
                    actuator = index,
                    bus,
                    id,
                    "temperature {} °C above the {} datasheet maximum {hottest} °C, within the actuator's limit {} °C",
                    health.temperature_c,
                    model.name(),
                    health.temperature_limit_c,
                );
            }
        }
        Ok(())
    }

    /// Checks that every Protocol 2 actuator is in position mode, and warns
    /// about one found powered: `torque_on` switches a [`Torque::ReadOnly`]
    /// one off (research R1). Protocol 1 has no Operating Mode: its joint
    /// mode is CW and CCW Angle Limit not both 0, nor both the model's
    /// multi-turn value, read here and kept as the range its goals must stay
    /// in; on the MX-64, Torque Ctrl Mode Enable 0 too, its counterpart of
    /// position mode. Its torque state is not read.
    fn configure(&mut self) -> Result<(), DynamixelError> {
        for (index, actuator) in self.actuators.iter() {
            let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
            if model.protocol() != Protocol::Two {
                if model.has_torque_ctrl_mode() {
                    let found = self
                        .backend
                        .read_setting(bus, model, id, Setting::TorqueCtrlModeEnable)
                        .map_err(|error| self.actuator_error(index, error))?;
                    if found != 0 {
                        return Err(DynamixelError::TorqueCtrlMode {
                            actuator: index,
                            bus: self.label(bus),
                            id,
                            found,
                        });
                    }
                }
                let (cw, ccw) = self
                    .backend
                    .read_angle_limits(bus, model, id)
                    .map_err(|error| self.actuator_error(index, error))?;
                let bus = self.label(bus);
                if cw == 0 && ccw == 0 {
                    return Err(DynamixelError::WheelMode {
                        actuator: index,
                        bus,
                        id,
                    });
                }
                if model
                    .multi_turn_limit()
                    .is_some_and(|limit| cw == limit && ccw == limit)
                {
                    return Err(DynamixelError::MultiTurnMode {
                        actuator: index,
                        bus,
                        id,
                        limit: cw,
                    });
                }
                if cw > ccw {
                    return Err(DynamixelError::AngleLimitsReversed {
                        actuator: index,
                        bus,
                        id,
                        cw,
                        ccw,
                    });
                }
                if let Some(limits) = self.limits.get_mut(index) {
                    *limits = (cw, ccw);
                }
                debug!(actuator = index, "Protocol 1 torque state not read");
                continue;
            }
            let mode = self
                .backend
                .read_setting(bus, model, id, Setting::OperatingMode)
                .map_err(|error| self.actuator_error(index, error))?;
            if mode != POSITION_MODE {
                return Err(DynamixelError::OperatingMode {
                    actuator: index,
                    bus: self.label(bus),
                    id,
                    found: mode,
                    expected: POSITION_MODE,
                });
            }
            let powered = self
                .backend
                .read_setting(bus, model, id, Setting::TorqueEnable)
                .map_err(|error| self.actuator_error(index, error))?
                != 0;
            match (actuator.torque, powered) {
                (Torque::ReadOnly, true) => warn!(
                    actuator = index,
                    id,
                    "read-only but torque already on before bring-up; switched off at torque_on"
                ),
                (Torque::On, true) => {
                    warn!(actuator = index, id, "torque already on before bring-up")
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Reads every group whatever the cadence; stops at the first failure.
    /// A failed group transaction names the bus, not one of its actuators.
    /// A seed position outside the register range is `PositionRange`: the
    /// check of the zeros SRS FR-5.2 asks for, since with a software zero
    /// every position inside it converts (research R7).
    fn read_all(&mut self, mut sample: Sample<'_>) -> Result<(), DynamixelError> {
        let mut failed = None;
        for table in self.groups.tables() {
            io::read_table(
                &mut self.backend,
                table,
                &self.actuators,
                &mut sample,
                |index, outcome| {
                    if !matches!(outcome, Outcome::Read) {
                        failed.get_or_insert((index, outcome));
                    }
                },
            );
            if failed.is_some() {
                break;
            }
        }
        self.read_failure(failed)
    }

    /// Torque on for [`Torque::On`] actuators, each where it is, so none
    /// jumps: every group first gets, torque still off, Goal Position = its
    /// present position read now, or Goal Velocity or Goal Current = 0. A
    /// failed read or goal write powers nothing. Torque written off for
    /// [`Torque::ReadOnly`] ones, whatever an earlier run left (SRS FR-5.2,
    /// FR-5.8).
    fn torque_on(&mut self) -> Result<(), DynamixelError> {
        for number in 0..self.groups.tables().len() {
            io::write_start_goals(self, number)?;
        }
        for (index, actuator) in self.actuators.iter() {
            let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
            match actuator.torque {
                Torque::On => self.backend.torque(bus, model, id, true),
                Torque::ReadOnly => self.backend.force_torque_off(bus, model, id),
            }
            .map_err(|error| self.actuator_error(index, error))?;
        }
        Ok(())
    }

    fn read(&mut self, sample: Sample<'_>, faults: &mut Faults) -> Mask {
        io::read_due(self, sample, faults)
    }

    fn write(&mut self, command: Command<'_>, batch: &Batch<'_>, faults: &mut Faults) {
        io::write_batch(self, &command, batch, faults);
    }

    /// Sends `last.q` to every powered position actuator, one transaction per
    /// group (FR-016).
    fn hold(&mut self, last: Command<'_>) {
        if !self.open {
            return;
        }
        for (number, table) in self.groups.tables().iter().enumerate() {
            let mut goals = Goals::new(number);
            for &index in table.members() {
                let Some(actuator) = self.actuators.get(index) else {
                    continue;
                };
                let raw = last.q.get(index).and_then(|&q| self.goal(index, q));
                match raw {
                    Some(raw) => goals.push(index, actuator.id, raw),
                    None if actuator.torque == Torque::On => {
                        warn!(
                            actuator = index,
                            "hold target outside the register range or the angle limits, not sent"
                        );
                    }
                    None => {}
                }
            }
            if let Err(error) = goals.send(&mut self.backend, &self.groups) {
                warn!(bus = self.label(table.bus), %error, "hold write failed");
            }
        }
    }

    /// Torque off on every actuator of every opened bus, read-only ones and
    /// ones an earlier run left on included (SRS FR-4.2, FR-5.6, FR-5.8).
    /// Nothing before the ports are open. Each actuator gets up to
    /// [`TORQUE_OFF_ATTEMPTS`], without sleep; one that fails them all is
    /// logged and the next actuator follows.
    fn torque_off(&mut self) {
        if !self.open {
            return;
        }
        for (index, actuator) in self.actuators.iter() {
            let mut answer = Err(BackendError::Timeout);
            for _ in 0..TORQUE_OFF_ATTEMPTS {
                answer = self
                    .backend
                    .force_torque_off(actuator.bus, actuator.model, actuator.id);
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
                    "torque off failed"
                );
            }
        }
    }

    /// Flushes every resolved port, then closes them all (SRS FR-5.6).
    fn close(&mut self) {
        for (bus, device) in self.devices.iter().enumerate() {
            if let Some(device) = device
                && port::flush(device.as_path()).is_err()
            {
                warn!(bus = self.label(bus), "flush before close failed");
            }
        }
        self.backend.close_ports();
        self.open = false;
    }
}
