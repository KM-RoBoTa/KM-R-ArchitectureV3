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

use std::path::Path;

use kmr_core::hal::{Batch, Command, Faults, Hal, Layout, Mask, Overrides, Sample, seal};
use kmr_core::state::{JOINTS, Q};
use tracing::{debug, warn};

use crate::binding::{self, Binding, Port};
use backend::{Backend, BackendError, POSITION_MODE, PortSpec, Setting};
use io::{Goals, Groups, Outcome};
use port::DevicePath;

#[cfg(feature = "dynamixel")]
pub use self::dxl_rs::DxlRs;
pub use config::{
    Actuator, Bus, DEFAULT_READ_EVERY, Mode, Model, Protocol, Series, Table, Torque, Zero,
};

/// Ping attempts per actuator before it counts as missing (FR-007).
const PING_ATTEMPTS: usize = 3;
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
    /// The actuator answered outside its limits.
    #[error("actuator {actuator}: id {id} on bus {bus}: {reason}")]
    Health {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// What is out of limits.
        reason: &'static str,
    },
    /// The actuator is not set up as the table says.
    #[error("actuator {actuator}: id {id} on bus {bus}: {reason}")]
    Setup {
        /// Engine index.
        actuator: usize,
        /// Bus label.
        bus: &'static str,
        /// Firmware id.
        id: u8,
        /// What differs.
        reason: &'static str,
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

/// The Dynamixel adapter over `NB` buses, talking through backend `B`.
///
/// One per process: the C SDK keeps global port state (research R7).
pub struct Dynamixel<B, const NB: usize> {
    /// The SDK seam.
    backend: B,
    /// Buses and actuators, with the bus plugins' bauds.
    table: Table<NB, JOINTS>,
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
    /// An adapter for `table`, one that passed [`Table::check`], with no bus
    /// bound yet: `open` fails until [`Dynamixel::bind`] binds them.
    pub const fn new(backend: B, table: Table<NB, JOINTS>) -> Self {
        Self {
            backend,
            table,
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

    /// An adapter for `table` with `plugins` bound to its buses by label,
    /// each plugin baud replacing its bus's (FR-030 (SRS FR-6.3, FR-6.4)).
    /// No I/O: the backend is not called.
    ///
    /// # Errors
    ///
    /// A plugin that binds no bus or one already bound, a bus left unbound,
    /// and a plugin baud a model on its bus rejects (FR-027 (SRS FR-6.4)).
    pub fn bind(
        backend: B,
        table: Table<NB, JOINTS>,
        plugins: &[Binding],
    ) -> Result<Self, DynamixelError> {
        let mut adapter = Self::new(backend, table);
        adapter.attach(plugins)?;
        Ok(adapter)
    }

    /// Binds `plugins` to the buses and takes their bauds; changes nothing
    /// on an error.
    fn attach(&mut self, plugins: &[Binding]) -> Result<(), DynamixelError> {
        let bindings = binding::match_plugins(self.table.buses.map(|bus| bus.label), plugins)?;
        for (index, actuator) in self.table.actuators.iter().enumerate() {
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
        for (bus, binding) in self.table.buses.iter_mut().zip(&bindings) {
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
        self.table.buses.get(bus).map_or("", |bus| bus.label)
    }

    /// Port of bus `bus`, from its bus plugin.
    fn port(&self, bus: usize) -> Port {
        self.ports.get(bus).copied().flatten().unwrap_or(UNBOUND)
    }

    /// Bus label and firmware id of actuator `index`.
    fn named(&self, index: usize) -> (&'static str, u8) {
        self.table
            .actuators
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

    /// The bring-up error of actuator `index` set up otherwise than the
    /// table says.
    fn setup_error(&self, index: usize, reason: &'static str) -> DynamixelError {
        let (bus, id) = self.named(index);
        DynamixelError::Setup {
            actuator: index,
            bus,
            id,
            reason,
        }
    }

    /// The bring-up error of actuator `index`, whose present position is
    /// outside the register range.
    fn invalid_position(&self, index: usize) -> DynamixelError {
        let (bus, id) = self.named(index);
        DynamixelError::Health {
            actuator: index,
            bus,
            id,
            reason: "present position outside the register range",
        }
    }

    /// The bring-up error of actuator `index`, whose present position is
    /// outside its angle limits: powered, it would not hold there.
    fn outside_limits(&self, index: usize) -> DynamixelError {
        let (bus, id) = self.named(index);
        DynamixelError::Health {
            actuator: index,
            bus,
            id,
            reason: "present position outside the CW/CCW Angle Limit",
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
        let actuator = self.table.actuators.get(index)?;
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
            Some((index, Outcome::Invalid)) => Err(self.invalid_position(index)),
        }
    }
}

impl<B, const NB: usize> seal::Adapter for Dynamixel<B, NB> {}

impl<B: Backend, const NB: usize> Hal for Dynamixel<B, NB> {
    type Error = DynamixelError;

    /// Groups per (bus, series); each bus is read every `read_every` ticks,
    /// else every [`DEFAULT_READ_EVERY`] (FR-009). `Overrides` carries no
    /// cadence: it is a table field (core K3.8).
    fn check(&mut self, _overrides: Overrides) -> Result<Layout, DynamixelError> {
        if NB > B::MAX_PORTS {
            return Err(DynamixelError::TooManyBuses {
                buses: NB,
                max: B::MAX_PORTS,
            });
        }
        let every = self
            .table
            .buses
            .map(|bus| bus.read_every.unwrap_or(DEFAULT_READ_EVERY));
        self.groups = Groups::build(&self.table.actuators, &every);
        let unread = io::unread_rates(&self.table.actuators);
        for index in (0..JOINTS).filter(|&index| unread.get(index)) {
            warn!(
                actuator = index,
                "Protocol 1: Qd and Tau are never read and keep their last value (research R12)"
            );
        }
        let (read_cost, write_cost) = self.groups.costs(&self.table.buses, B::STATE_READS);
        Ok(Layout::new(
            self.groups.numbers(),
            self.groups.cadence(),
            read_cost,
            write_cost,
        ))
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
        for ((spec, device), bus) in specs.iter_mut().zip(&self.devices).zip(&self.table.buses) {
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

    /// Nothing to do: Dynamixel encoders are absolute (FR-005).
    fn zero(&mut self) -> Result<(), DynamixelError> {
        Ok(())
    }

    /// Pings each actuator up to [`PING_ATTEMPTS`] times, without sleep.
    fn ping(&mut self) -> Result<(), DynamixelError> {
        for (index, actuator) in self.table.actuators.iter().enumerate() {
            let mut answer = Err(BackendError::Timeout);
            for _ in 0..PING_ATTEMPTS {
                answer = self.backend.ping(actuator.bus, actuator.model, actuator.id);
                if answer.is_ok() {
                    break;
                }
            }
            answer.map_err(|error| self.actuator_error(index, error))?;
        }
        Ok(())
    }

    /// Reads every actuator's voltage and temperature against its model's
    /// range, and on Protocol 2 its Hardware Error Status.
    fn health(&mut self) -> Result<(), DynamixelError> {
        for (index, actuator) in self.table.actuators.iter().enumerate() {
            let health = self
                .backend
                .read_health(actuator.bus, actuator.model, actuator.id)
                .map_err(|error| self.actuator_error(index, error))?;
            let (min_dv, max_dv) = actuator.model.voltage_limits_dv();
            let reason = if actuator.model.protocol() == Protocol::Two && health.error_status != 0 {
                "hardware error status"
            } else if !(min_dv..=max_dv).contains(&health.voltage_dv) {
                "input voltage out of range"
            } else if health.temperature_c > actuator.model.max_temperature_c() {
                "temperature too high"
            } else {
                continue;
            };
            return Err(DynamixelError::Health {
                actuator: index,
                bus: self.label(actuator.bus),
                id: actuator.id,
                reason,
            });
        }
        Ok(())
    }

    /// Checks that every Protocol 2 actuator is in position mode, and warns
    /// about one found powered: `torque_on` switches a [`Torque::ReadOnly`]
    /// one off (research R1). Protocol 1 has no Operating Mode: its joint
    /// mode is CW and CCW Angle Limit not both 0, read here and kept as the
    /// range its goals must stay in. Its torque state is not read.
    fn configure(&mut self) -> Result<(), DynamixelError> {
        for (index, actuator) in self.table.actuators.iter().enumerate() {
            let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
            if model.protocol() != Protocol::Two {
                let (cw, ccw) = self
                    .backend
                    .read_angle_limits(bus, model, id)
                    .map_err(|error| self.actuator_error(index, error))?;
                if cw == 0 && ccw == 0 {
                    return Err(self.setup_error(
                        index,
                        "CW and CCW Angle Limit both 0: wheel mode, not joint mode",
                    ));
                }
                if cw > ccw {
                    return Err(self.setup_error(index, "CW Angle Limit above CCW Angle Limit"));
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
                return Err(self.setup_error(index, "operating mode is not position (3)"));
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
    fn read_all(&mut self, mut sample: Sample<'_>) -> Result<(), DynamixelError> {
        let mut failed = None;
        for table in self.groups.tables() {
            io::read_table(
                &mut self.backend,
                table,
                &self.table.actuators,
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
        for (index, actuator) in self.table.actuators.iter().enumerate() {
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
                let Some(actuator) = self.table.actuators.get(index) else {
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
        for (index, actuator) in self.table.actuators.iter().enumerate() {
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
