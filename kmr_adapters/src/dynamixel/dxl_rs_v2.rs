//! The [`Backend`] on `dxl_rs_v2`, the pure-Rust Dynamixel SDK: every call
//! kmr makes into it is in this file.
//!
//! Each port holds one `dxl_rs_v2` bus per protocol, each on its own handle
//! of the one serial device, so a bus may carry Protocol 1 and Protocol 2
//! actuators. `dxl_rs_v2` keeps the torque state of each actuator it holds
//! and refuses a goal to one whose torque is off; the backend keeps, per
//! port, which actuators got their start goal.

use core::time::Duration;
use std::io::ErrorKind;

use ::dxl_rs_v2::field::{
    HardwareErrorStatus, PresentCurrent, PresentPosition, PresentTemperature, PresentVelocity,
    PresentVoltage, Raw,
};
use ::dxl_rs_v2::{
    ActuatorRef, Bus, Clock, Config, Error, FaultError, Id, IdSet, ModelNumber, P1, P2,
    ProtocolError, SerialTransport, SetupError, StatusReport, StdClock, TemperatureError,
    TorqueState, Transport, TransportError, ValidationError, VoltageError,
};

use super::backend::{
    Backend, BackendError, Fault, GoalRegister, Health, LimitRegisters, OpenFailure, PortSpec,
    RawState, Reading, Register, Reply, Setting, Undeclared, Vital, Wire,
};
use super::config::{HealthLimits, Model, Protocol};

#[cfg(test)]
mod contract;

/// Ports one backend opens.
const MAX_PORTS: usize = 4;

/// Reply timeout before the line time of the reply.
// DECISION: 1.5 ms: the 1 ms USB latency timer `open` enforces (FR-5.10) and
// the 0.5 ms factory Return Delay Time. v2's 10 ms default loses ten ticks per
// silent actuator at 1 kHz; the bench (T063) may shorten it.
const REPLY_WAIT: Duration = Duration::from_micros(1_500);
/// Bytes of the longest reply the backend waits for: a Protocol 2 status
/// packet with 4 data bytes is 15.
const REPLY_BYTES: u64 = 20;

/// Operating Mode, Protocol 2: one byte at 11.
const OPERATING_MODE: u16 = 11;
/// Torque Enable, Protocol 2: one byte at 64.
const TORQUE_ENABLE: u16 = 64;
/// Torque Ctrl Mode Enable, Protocol 1 MX-64: one byte at 70.
const TORQUE_CTRL_MODE_ENABLE: u16 = 70;
/// Present Voltage then Present Temperature, Protocol 1: one byte each from
/// 42 (AX, RX and MX tables).
// DECISION: one raw read of both, as the backend author advises: dxl_rs_v2
// has no read of two fields on Protocol 1, and two reads double the cost.
const P1_VITALS: u16 = 42;

/// Goal Position of `model`: address and width.
const fn goal_position(model: Model) -> Register {
    match model.protocol() {
        Protocol::One => Register {
            address: 30,
            size: 2,
        },
        Protocol::Two => Register {
            address: 116,
            size: 4,
        },
    }
}

/// The `dxl_rs_v2` timing of a bus at `baud` bit/s, with no retry.
// DECISION: no v2 retry: the adapter owns the attempts (PING_ATTEMPTS,
// TORQUE_OFF_ATTEMPTS), which v2's per-exchange retries would multiply.
fn config(baud: u32) -> Config {
    let line = REPLY_BYTES * 10 * 1_000_000_000 / u64::from(baud.max(1));
    Config::new()
        .timeout(REPLY_WAIT + Duration::from_nanos(line))
        .retries(0)
}

/// One open port: a bus per protocol, on two handles of one device.
struct Port<T: Transport, C, const N: usize> {
    /// Its Protocol 1 actuators.
    one: Bus<P1, T, C, N>,
    /// Its Protocol 2 actuators.
    two: Bus<P2, T, C, N>,
    /// Actuators given a start goal since their torque was last switched.
    started: IdSet,
}

/// Binds `$bus` to the bus of `$port` that speaks `$model`'s protocol, then
/// runs `$body`.
macro_rules! on_bus {
    ($port:expr, $model:expr, |$bus:ident| $body:expr) => {
        match $model.protocol() {
            Protocol::One => {
                let $bus = &mut $port.one;
                $body
            }
            Protocol::Two => {
                let $bus = &mut $port.two;
                $body
            }
        }
    };
}

/// The `dxl_rs_v2` backend for a robot of `N` actuators, on serial ports.
pub struct DxlRsV2<const N: usize>(Ports<N, SerialTransport, StdClock>);

impl<const N: usize> DxlRsV2<N> {
    /// A backend with no port open.
    pub fn new() -> Self {
        Self(Ports {
            ports: [const { None }; MAX_PORTS],
        })
    }
}

impl<const N: usize> Default for DxlRsV2<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// A line the backend opens from a device path: two handles of one device,
/// each with its clock.
pub(crate) trait Open<C>: Sized {
    /// Opens `path` at `baud` bit/s.
    fn open_pair(path: &str, baud: u32) -> Result<[(Self, C); 2], BackendError>;
}

impl Open<StdClock> for SerialTransport {
    /// 8N1, no flow control; the second handle is a duplicate of the first.
    fn open_pair(path: &str, baud: u32) -> Result<[(Self, StdClock); 2], BackendError> {
        let mut first = Self::open(path, baud).map_err(refused)?;
        let second = first.port_mut().try_clone_native().map_err(refused)?;
        Ok([
            (first, StdClock::default()),
            (Self::from(second), StdClock::default()),
        ])
    }
}

/// Why the serial port did not open.
fn refused(error: ::dxl_rs_v2::serialport::Error) -> BackendError {
    use ::dxl_rs_v2::serialport::ErrorKind as Kind;
    BackendError::PortOpen(match error.kind() {
        Kind::NoDevice => ErrorKind::ResourceBusy,
        Kind::InvalidInput => ErrorKind::InvalidInput,
        Kind::Io(kind) => kind,
        Kind::Unknown => ErrorKind::Other,
    })
}

/// The ports of the backend, on lines `T` timed by `C`: generic, so the
/// contract tests run it on a scripted line.
pub(crate) struct Ports<const N: usize, T: Transport, C> {
    /// Each opened port, by index.
    ports: [Option<Port<T, C, N>>; MAX_PORTS],
}

impl<const N: usize, T: Transport, C: Clock> Ports<N, T, C> {
    /// Port `port`, open.
    fn port(&mut self, port: usize) -> Result<&mut Port<T, C, N>, BackendError> {
        self.ports
            .get_mut(port)
            .and_then(Option::as_mut)
            .ok_or(BackendError::PortNotOpen { port })
    }

    /// Opens port `port` on lines `one` and `two`, at `baud` bit/s.
    fn attach(&mut self, port: usize, one: (T, C), two: (T, C), baud: u32) {
        let config = config(baud);
        if let Some(slot) = self.ports.get_mut(port) {
            *slot = Some(Port {
                one: Bus::new(one.0, one.1, config),
                two: Bus::new(two.0, two.1, config),
                started: IdSet::new(),
            });
        }
    }
}

/// `id` as a `dxl_rs_v2` id.
fn id_of(id: u8) -> Result<Id, BackendError> {
    Id::new(id).map_err(|_| BackendError::Refused {
        reason: "an id above 252",
    })
}

/// The held actuator `id` of `bus`.
fn held<P: ::dxl_rs_v2::Protocol, T: Transport, C: Clock, const N: usize>(
    bus: &mut Bus<P, T, C, N>,
    id: Id,
) -> Result<ActuatorRef<'_, P, T, C>, BackendError> {
    bus.actuator(id)
        .ok_or(BackendError::NotDeclared { id: id.get() })
}

/// Sets up `list` on `bus`, replacing what it held.
fn declare_on<P: ::dxl_rs_v2::Protocol, T: Transport, C: Clock, const N: usize>(
    bus: &mut Bus<P, T, C, N>,
    list: &[(u8, Model)],
    protocol: Protocol,
) -> Result<(), Undeclared> {
    if let Some(&(id, _)) = list.iter().find(|&&(id, _)| Id::new(id).is_err()) {
        return Err(Undeclared {
            id: Some(id),
            source: BackendError::Refused {
                reason: "an id above 252",
            },
        });
    }
    let entries = list
        .iter()
        .filter(|(_, model)| model.protocol() == protocol)
        .filter_map(|&(id, model)| Some((Id::new(id).ok()?, ModelNumber(model.number()))));
    bus.declare(entries).map(drop).map_err(|error| Undeclared {
        id: failing_id(&error),
        source: backend_error(error),
    })
}

/// Reads `register`, at most two bytes, from `actuator`.
fn read_register<P: ::dxl_rs_v2::Protocol, T: Transport, C: Clock>(
    actuator: &mut ActuatorRef<'_, P, T, C>,
    register: Register,
) -> Result<u16, BackendError> {
    let mut bytes = [0; 2];
    let out = bytes
        .get_mut(..usize::from(register.size))
        .ok_or(BackendError::Refused {
            reason: "a register wider than two bytes",
        })?;
    actuator
        .read_raw(register.address, out)
        .map_err(backend_error)?;
    Ok(u16::from_le_bytes(bytes))
}

/// Writes `value` to `register`, at most two bytes, of `actuator`.
fn write_register<P: ::dxl_rs_v2::Protocol, T: Transport, C: Clock>(
    actuator: &mut ActuatorRef<'_, P, T, C>,
    register: Register,
    value: u16,
) -> Result<(), BackendError> {
    let bytes = value.to_le_bytes();
    let data = bytes
        .get(..usize::from(register.size))
        .ok_or(BackendError::Refused {
            reason: "a register wider than two bytes",
        })?;
    actuator
        .write_raw(register.address, data)
        .map_err(backend_error)
}

/// `value`, read from `register`, as a `U`.
fn narrow<U: TryFrom<i32>>(register: &'static str, value: i32) -> Result<U, BackendError> {
    U::try_from(value).map_err(|_| BackendError::Overflow { register, value })
}

/// What `read_health` reads from one actuator.
fn health_of<P: ::dxl_rs_v2::Protocol, T: Transport, C: Clock>(
    actuator: &mut ActuatorRef<'_, P, T, C>,
    model: Model,
) -> Result<Health, BackendError> {
    let error_status = match model.protocol() {
        Protocol::One => 0,
        Protocol::Two => actuator.hardware_errors().map_err(backend_error)?.raw(),
    };
    let voltage = actuator.read(Raw(PresentVoltage)).map_err(backend_error)?;
    let temperature = actuator
        .read(Raw(PresentTemperature))
        .map_err(backend_error)?;
    let limits = LimitRegisters::of(model);
    let temperature_limit = read_register(actuator, limits.temperature)?;
    Ok(Health {
        error_status,
        voltage_dv: narrow("Present Input Voltage", voltage)?,
        temperature_c: narrow("Present Temperature", temperature)?,
        min_voltage_dv: read_register(actuator, limits.min_voltage)?,
        max_voltage_dv: read_register(actuator, limits.max_voltage)?,
        temperature_limit_c: narrow("Temperature Limit", i32::from(temperature_limit))?,
    })
}

/// Folds one field of a partial read into `out`: `apply` on a value, or on
/// the fault an actuator reported when `measured` takes it; the first error
/// of each actuator kept. Returns a line failure, if any.
fn merge<S>(
    out: &mut [Result<S, BackendError>],
    read: &[Result<i32, Error>],
    apply: impl Fn(&mut S, i32),
    measured: impl Fn(&mut S, &FaultError) -> bool,
) -> Option<BackendError> {
    let mut line = None;
    for (slot, value) in out.iter_mut().zip(read) {
        match (slot.as_mut(), value) {
            (Ok(state), Ok(raw)) => apply(state, *raw),
            (Ok(state), Err(error)) => {
                if let Error::Fault(fault) = error
                    && measured(state, fault)
                {
                    continue;
                }
                let error = backend_error(*error);
                if error.is_bus() {
                    line.get_or_insert(error);
                }
                *slot = Err(error);
            }
            (Err(_), _) => {}
        }
    }
    line
}

/// What `reading` gains from a fault `dxl_rs_v2` measured: its Hardware
/// Error Status bits, and the voltage or the temperature it measured.
fn note_fault(reading: &mut Reading, fault: &FaultError) -> bool {
    reading.hardware_error = Some(fault.faults().raw());
    match fault {
        FaultError::Voltage(
            VoltageError::BelowMin { measured, .. }
            | VoltageError::AboveMax { measured, .. }
            | VoltageError::InsideLimits { measured, .. },
        ) => reading.voltage_dv = Some(measured.0),
        FaultError::Temperature(
            TemperatureError::AboveLimit { measured, .. }
            | TemperatureError::WithinLimit { measured, .. },
        ) => reading.temperature_c = u8::try_from(measured.0).ok(),
        _ => {}
    }
    true
}

/// The id a `dxl_rs_v2` error names, if any.
fn failing_id(error: &Error) -> Option<u8> {
    let id = match error {
        Error::Setup(
            SetupError::Missing { id, .. }
            | SetupError::WrongModel { id, .. }
            | SetupError::UnsupportedModel { id, .. }
            | SetupError::UnknownOperatingMode { id, .. }
            | SetupError::NotOnBus { id },
        ) => Some(*id),
        Error::Fault(fault) => Some(fault.id()),
        Error::Status(status) => Some(status.id),
        other => other.request().and_then(|request| request.id),
    };
    id.map(Id::get)
}

/// The backend error of a `dxl_rs_v2` error. Its types stay in this file.
fn backend_error(error: Error) -> BackendError {
    match error {
        Error::Transport(failure) => match failure {
            TransportError::NoReply { .. } | TransportError::Skipped { .. } => {
                BackendError::Timeout
            }
            TransportError::LateReply { .. } => BackendError::Reply(Reply::Late),
            TransportError::PartialReply { .. } => BackendError::Reply(Reply::Partial),
            TransportError::PortTimeout { .. } => BackendError::Io(Wire::PortTimeout),
            TransportError::EndOfStream { .. } => BackendError::Io(Wire::Closed),
            TransportError::Io { kind, .. } => BackendError::Io(Wire::Os(kind.into())),
            _ => BackendError::Io(Wire::Os(ErrorKind::Other)),
        },
        Error::Protocol(ProtocolError::Checksum { .. } | ProtocolError::Crc { .. }) => {
            BackendError::Reply(Reply::Corrupt)
        }
        Error::Protocol(_) => BackendError::Reply(Reply::Malformed),
        Error::Status(status) => BackendError::Status(status_byte(status.report)),
        Error::Fault(fault) => BackendError::Fault(fault_of(fault)),
        Error::Setup(setup) => match setup {
            SetupError::Missing { .. } => BackendError::Timeout,
            SetupError::WrongModel { found, .. } => BackendError::WrongModel { found },
            SetupError::UnsupportedModel { model_number, .. } => BackendError::UnsupportedModel {
                number: model_number,
            },
            SetupError::UnknownOperatingMode { value, .. } => BackendError::UnknownMode { value },
            SetupError::NotOnBus { id } => BackendError::NotDeclared { id: id.get() },
            _ => BackendError::Refused {
                reason: "more actuators than the bus holds",
            },
        },
        Error::Validation(refusal) => match refusal {
            ValidationError::ActuatorIsOff { id } => BackendError::NotPowered { id: id.get() },
            ValidationError::ActuatorFaulted { id } => BackendError::Faulted { id: id.get() },
            ValidationError::OutOfLimits { raw, min, max, .. } => BackendError::GoalOutsideLimits {
                goal: raw,
                min,
                max,
            },
            other => BackendError::Refused {
                reason: refusal_reason(&other),
            },
        },
        _ => BackendError::Refused {
            reason: "an error this build of kmr does not name",
        },
    }
}

/// What a `dxl_rs_v2` refusal refused, in a few words.
fn refusal_reason(refusal: &ValidationError) -> &'static str {
    match refusal {
        ValidationError::InvalidId(_) => "an id above 252",
        ValidationError::DuplicateId { .. } => "an id named twice",
        ValidationError::BufferLength { .. } => "a buffer of another length than the targets",
        ValidationError::DoesNotFit { .. } => "a value wider than its register",
        ValidationError::NotPositionMode { .. } => "a goal position outside position mode",
        ValidationError::Unsupported { .. } => "a register the model does not have",
        ValidationError::NoUnit { .. } => "a register with no unit in this mode",
        ValidationError::InvertedLimits { .. } => "position limits min above max",
        _ => "a request the SDK does not send",
    }
}

/// The error byte of a status packet, as the actuator sent it.
fn status_byte(report: StatusReport) -> u8 {
    use ::dxl_rs_v2::P2Code;
    match report {
        StatusReport::P1(flags) => flags.0,
        StatusReport::P2 { alert, code } => {
            let code = match code {
                None => 0,
                Some(P2Code::ResultFail) => 1,
                Some(P2Code::Instruction) => 2,
                Some(P2Code::Crc) => 3,
                Some(P2Code::DataRange) => 4,
                Some(P2Code::DataLength) => 5,
                Some(P2Code::DataLimit) => 6,
                Some(P2Code::Access) => 7,
                Some(P2Code::Unknown(bits)) => bits,
                Some(_) => 0x7F,
            };
            u8::from(alert) << 7 | code
        }
        _ => 0,
    }
}

/// The fault `dxl_rs_v2` measured: voltage or temperature while still
/// outside the limits, its Hardware Error Status otherwise.
fn fault_of(fault: FaultError) -> Fault {
    let celsius = |value: ::dxl_rs_v2::Celsius| u8::try_from(value.0.max(0)).unwrap_or(u8::MAX);
    match fault {
        FaultError::Voltage(
            VoltageError::BelowMin {
                measured, min, max, ..
            }
            | VoltageError::AboveMax {
                measured, min, max, ..
            },
        ) => Fault::Voltage {
            measured_dv: measured.0,
            min_dv: min.0,
            max_dv: max.0,
        },
        FaultError::Temperature(TemperatureError::AboveLimit {
            measured, limit, ..
        }) => Fault::Temperature {
            measured_c: celsius(measured),
            limit_c: celsius(limit),
        },
        other => Fault::Hardware {
            status: other.faults().raw(),
        },
    }
}

/// The open failure of port `port` failing with `source`.
fn on_port(port: usize, source: BackendError) -> OpenFailure {
    OpenFailure {
        port: Some(port),
        source,
    }
}

impl<const N: usize, T: Transport + Open<C>, C: Clock> Backend for Ports<N, T, C> {
    const MAX_PORTS: usize = MAX_PORTS;
    const STATE_READS: &'static [u16] = super::backend::STATE_READS;

    /// Every model but the XW430-T200.
    // DECISION (B4): dxl_rs_v2 at 66035e1 has no table under its Model Number
    // 1280 (it files the T200 under the T333's 1270), so declaring one would
    // fail on the bus; `check` refuses it before any I/O instead.
    fn supports(model: Model) -> bool {
        !matches!(model, Model::Xw430T200)
    }

    /// Opens each port twice, one handle per protocol; ports already open
    /// are closed first.
    fn open_ports(&mut self, ports: &[PortSpec<'_>]) -> Result<(), OpenFailure> {
        if ports.len() > MAX_PORTS {
            return Err(OpenFailure {
                port: None,
                source: BackendError::TooManyPorts {
                    ports: ports.len(),
                    max: MAX_PORTS,
                },
            });
        }
        self.close_ports();
        for (port, spec) in ports.iter().enumerate() {
            let path = spec
                .path
                .to_str()
                .ok_or(on_port(port, BackendError::PathNotUtf8))?;
            let [one, two] =
                T::open_pair(path, spec.baud).map_err(|source| on_port(port, source))?;
            self.attach(port, one, two, spec.baud);
        }
        Ok(())
    }

    /// One `dxl_rs_v2` Setup per protocol: Protocol 1 actuators first.
    /// Start goals given before are forgotten.
    fn declare(&mut self, port: usize, actuators: &[(u8, Model)]) -> Result<(), Undeclared> {
        let open = self
            .port(port)
            .map_err(|source| Undeclared { id: None, source })?;
        open.started = IdSet::new();
        declare_on(&mut open.one, actuators, Protocol::One)?;
        declare_on(&mut open.two, actuators, Protocol::Two)
    }

    fn read_position(&mut self, port: usize, model: Model, id: u8) -> Result<u32, BackendError> {
        let (open, id) = (self.port(port)?, id_of(id)?);
        let raw = on_bus!(open, model, |bus| held(bus, id)?
            .present_position_raw()
            .map_err(backend_error))?;
        Ok(raw.cast_unsigned())
    }

    /// Three partial sync reads, current, velocity then position; the
    /// first line failure ends the call.
    fn sync_read_state(
        &mut self,
        port: usize,
        model: Model,
        ids: &[u8],
        out: &mut [Result<RawState, BackendError>],
    ) -> Result<(), BackendError> {
        if model.protocol() != Protocol::Two {
            return Err(BackendError::NoSyncRead);
        }
        if out.len() != ids.len() {
            return Err(BackendError::SlotMismatch {
                ids: ids.len(),
                slots: out.len(),
            });
        }
        let bus = &mut self.port(port)?.two;
        for &id in ids {
            id_of(id)?;
        }
        let targets = || ids.iter().filter_map(|&id| Id::new(id).ok());
        out.fill(Ok(RawState::default()));
        let mut read = [Ok(0); N];
        let read = read
            .get_mut(..ids.len())
            .ok_or(BackendError::SlotMismatch {
                ids: ids.len(),
                slots: N,
            })?;
        bus.read_field_partial(Raw(PresentCurrent), targets(), read)
            .map_err(backend_error)?;
        let kept = |_: &mut RawState, _: &FaultError| false;
        if let Some(line) = merge(
            out,
            read,
            |state, raw| {
                let [low, high, ..] = raw.to_le_bytes();
                state.current = i16::from_le_bytes([low, high]);
            },
            kept,
        ) {
            return Err(line);
        }
        bus.read_field_partial(Raw(PresentVelocity), targets(), read)
            .map_err(backend_error)?;
        if let Some(line) = merge(out, read, |state, raw| state.velocity = raw, kept) {
            return Err(line);
        }
        bus.read_field_partial(Raw(PresentPosition), targets(), read)
            .map_err(backend_error)?;
        merge(out, read, |state, raw| state.position = raw, kept).map_or(Ok(()), Err)
    }

    /// One partial sync read of `vital` on Protocol 2; on Protocol 1 one
    /// raw read of Present Voltage and Present Temperature per id, the first
    /// line failure ending the call. A clean Protocol 1 reply carries no
    /// fault bit, so its Hardware Error Status reads 0.
    // DECISION: Protocol 1 reports the fault bits of its status packet as its
    // hardware error, in the X series layout `dxl_rs_v2` decodes them to, and
    // 0 on a clean reply: they come with the read, at no cost.
    fn read_vitals(
        &mut self,
        port: usize,
        model: Model,
        vital: Vital,
        ids: &[u8],
        out: &mut [Result<Reading, BackendError>],
    ) -> Result<(), BackendError> {
        if out.len() != ids.len() {
            return Err(BackendError::SlotMismatch {
                ids: ids.len(),
                slots: out.len(),
            });
        }
        let open = self.port(port)?;
        for &id in ids {
            id_of(id)?;
        }
        out.fill(Ok(Reading::default()));
        if model.protocol() == Protocol::One {
            for (slot, &id) in out.iter_mut().zip(ids) {
                let mut bytes = [0; 2];
                *slot = match held(&mut open.one, id_of(id)?) {
                    Ok(mut actuator) => match actuator.read_raw(P1_VITALS, &mut bytes) {
                        Ok(()) => {
                            let [voltage, temperature] = bytes;
                            Ok(Reading {
                                voltage_dv: Some(u16::from(voltage)),
                                temperature_c: Some(temperature),
                                hardware_error: Some(0),
                            })
                        }
                        Err(Error::Fault(fault)) => {
                            let mut reading = Reading::default();
                            note_fault(&mut reading, &fault);
                            Ok(reading)
                        }
                        Err(error) => Err(backend_error(error)),
                    },
                    Err(error) => Err(error),
                };
                if let Err(error) = slot
                    && error.is_bus()
                {
                    return Err(*error);
                }
            }
            return Ok(());
        }
        let bus = &mut open.two;
        let targets = ids.iter().filter_map(|&id| Id::new(id).ok());
        let mut read = [Ok(0); N];
        let read = read
            .get_mut(..ids.len())
            .ok_or(BackendError::SlotMismatch {
                ids: ids.len(),
                slots: N,
            })?;
        match vital {
            Vital::Voltage => bus.read_field_partial(Raw(PresentVoltage), targets, read),
            Vital::Temperature => bus.read_field_partial(Raw(PresentTemperature), targets, read),
            Vital::HardwareError => bus.read_field_partial(Raw(HardwareErrorStatus), targets, read),
        }
        .map_err(backend_error)?;
        let apply = |reading: &mut Reading, raw| *reading = Reading::of(vital, raw);
        merge(out, read, apply, note_fault).map_or(Ok(()), Err)
    }

    /// One sync write. Refused whole when an actuator of `goals` was not
    /// switched on here.
    // DECISION: one that reported a hardware fault since is left out and the
    // rest is sent: dxl_rs_v2 refuses the whole write otherwise, so one hot
    // servo would freeze its group. Its torque is off already; its reads
    // report the fault.
    fn sync_write_goal(
        &mut self,
        port: usize,
        model: Model,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError> {
        let open = self.port(port)?;
        let (mut ids, mut raws) = ([0; N], [0; N]);
        let mut len = 0;
        for &(id, raw) in goals {
            let target = id_of(id)?;
            match on_bus!(open, model, |bus| bus.torque_state(target)) {
                Some(TorqueState::On) => {}
                Some(TorqueState::Faulted) => continue,
                _ => return Err(BackendError::NotPowered { id }),
            }
            let goal = i32::try_from(raw).map_err(|_| BackendError::Refused {
                reason: "a goal wider than its register",
            })?;
            let (Some(slot), Some(value)) = (ids.get_mut(len), raws.get_mut(len)) else {
                return Err(BackendError::SlotMismatch {
                    ids: goals.len(),
                    slots: N,
                });
            };
            (*slot, *value) = (id, goal);
            len += 1;
        }
        let ids = ids.get(..len).unwrap_or_default();
        let ids = ids.iter().filter_map(|&id| Id::new(id).ok());
        let raws = raws.get(..len).unwrap_or_default();
        on_bus!(open, model, |bus| bus.write_goal_positions_raw(ids, raws)).map_err(backend_error)
    }

    /// One acknowledged raw write of Goal Position per actuator, torque
    /// off. Refused whole, before any write, when one of `goals` is switched
    /// on here; every register but Goal Position is refused.
    // DECISION (B1): dxl_rs_v2 refuses goals while the torque is off and has
    // no "torque on at the present position". The start goal goes through
    // `write_raw` (`raw-registers`), which bypasses that check and touches no
    // cache v2 keeps: Goal Position is no limit register. Torque on stays a
    // separate, later call, so no servo jumps to a stale goal.
    fn sync_write_start_goal(
        &mut self,
        port: usize,
        model: Model,
        register: GoalRegister,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError> {
        let open = self.port(port)?;
        if register != GoalRegister::Position {
            return Err(BackendError::Unwritable { register });
        }
        for &(id, _) in goals {
            let target = id_of(id)?;
            if on_bus!(open, model, |bus| bus.torque_state(target)) == Some(TorqueState::On) {
                return Err(BackendError::AlreadyPowered { id });
            }
        }
        let goal = goal_position(model);
        for &(id, raw) in goals {
            let target = id_of(id)?;
            let bytes = raw.to_le_bytes();
            let (data, wider) = bytes.split_at(usize::from(goal.size));
            if wider.iter().any(|&byte| byte != 0) {
                return Err(BackendError::Refused {
                    reason: "a goal wider than its register",
                });
            }
            on_bus!(open, model, |bus| held(bus, target)?
                .write_raw(goal.address, data)
                .map_err(backend_error))?;
            open.started.insert(target);
        }
        Ok(())
    }

    /// On only once a start goal was written since the actuator was last
    /// switched; that goal is then used up.
    fn torque(&mut self, port: usize, model: Model, id: u8, on: bool) -> Result<(), BackendError> {
        let (open, target) = (self.port(port)?, id_of(id)?);
        if on {
            if !open.started.contains(target) {
                return Err(BackendError::NoStartGoal { id });
            }
            on_bus!(open, model, |bus| held(bus, target)?
                .enable_torque()
                .map_err(backend_error))?;
        } else {
            let state = on_bus!(open, model, |bus| bus.torque_state(target));
            if state != Some(TorqueState::On) {
                return Err(BackendError::NotPowered { id });
            }
            on_bus!(open, model, |bus| held(bus, target)?
                .disable_torque()
                .map_err(backend_error))?;
        }
        open.started.remove(target);
        Ok(())
    }

    /// `disable_torque`, sent in every state.
    // DECISION: an actuator its port does not hold (its declare failed or
    // never ran) is declared alone first, replacing what the port held:
    // dxl_rs_v2 reaches no other, and the exit path must reach every one.
    fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        let (open, target) = (self.port(port)?, id_of(id)?);
        open.started.remove(target);
        on_bus!(open, model, |bus| {
            if bus.actuator(target).is_none() {
                bus.declare([(target, ModelNumber(model.number()))])
                    .map_err(backend_error)?;
            }
            held(bus, target)?.disable_torque().map_err(backend_error)
        })
    }

    /// One raw read of the register: `dxl_rs_v2` has no field for these.
    fn read_setting(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        setting: Setting,
    ) -> Result<u8, BackendError> {
        let (open, target) = (self.port(port)?, id_of(id)?);
        let address = match setting {
            Setting::TorqueCtrlModeEnable if !model.has_torque_ctrl_mode() => {
                return Err(BackendError::NoRegister {
                    register: setting.name(),
                });
            }
            Setting::OperatingMode | Setting::TorqueEnable if model.protocol() != Protocol::Two => {
                return Err(BackendError::NoSetting { setting });
            }
            Setting::OperatingMode => OPERATING_MODE,
            Setting::TorqueEnable => TORQUE_ENABLE,
            Setting::TorqueCtrlModeEnable => TORQUE_CTRL_MODE_ENABLE,
        };
        let mut value = [0];
        on_bus!(open, model, |bus| held(bus, target)?
            .read_raw(address, &mut value)
            .map_err(backend_error))?;
        Ok(value[0])
    }

    /// Hardware Error Status (Protocol 2), Present Input Voltage and Present
    /// Temperature through their fields, the three limits raw: six reads on
    /// Protocol 2, five on Protocol 1.
    fn read_health(&mut self, port: usize, model: Model, id: u8) -> Result<Health, BackendError> {
        let (open, target) = (self.port(port)?, id_of(id)?);
        on_bus!(open, model, |bus| health_of(&mut held(bus, target)?, model))
    }

    /// Three raw writes, Temperature Limit first. A voltage wider than its
    /// one-byte Protocol 1 register is refused before any write.
    fn write_health_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        limits: HealthLimits,
    ) -> Result<(), BackendError> {
        let (open, target) = (self.port(port)?, id_of(id)?);
        let registers = LimitRegisters::of(model);
        if !registers.fit(limits) {
            return Err(BackendError::Refused {
                reason: "a voltage limit wider than its one-byte register",
            });
        }
        on_bus!(open, model, |bus| {
            let mut actuator = held(bus, target)?;
            write_register(
                &mut actuator,
                registers.temperature,
                u16::from(limits.temperature_c),
            )?;
            write_register(&mut actuator, registers.min_voltage, limits.min_voltage_dv)?;
            write_register(&mut actuator, registers.max_voltage, limits.max_voltage_dv)
        })
    }

    /// The limits `dxl_rs_v2` read at Setup: no I/O.
    fn read_angle_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<(u32, u32), BackendError> {
        use ::dxl_rs_v2::Actuator as _;
        let (open, target) = (self.port(port)?, id_of(id)?);
        let limits = on_bus!(open, model, |bus| held(bus, target)?.position_limits_raw())
            .ok_or(BackendError::NoAngleLimits)?;
        let bound = |value: i32| narrow::<u32>("position limit", value);
        Ok((bound(*limits.start())?, bound(*limits.end())?))
    }

    /// Drops every bus, which closes both handles of its device.
    fn close_ports(&mut self) {
        self.ports = [const { None }; MAX_PORTS];
    }
}

impl<const N: usize> Backend for DxlRsV2<N> {
    const MAX_PORTS: usize = MAX_PORTS;
    const STATE_READS: &'static [u16] = super::backend::STATE_READS;

    fn supports(model: Model) -> bool {
        Ports::<N, SerialTransport, StdClock>::supports(model)
    }

    fn open_ports(&mut self, ports: &[PortSpec<'_>]) -> Result<(), OpenFailure> {
        self.0.open_ports(ports)
    }

    fn declare(&mut self, port: usize, actuators: &[(u8, Model)]) -> Result<(), Undeclared> {
        self.0.declare(port, actuators)
    }

    fn read_position(&mut self, port: usize, model: Model, id: u8) -> Result<u32, BackendError> {
        self.0.read_position(port, model, id)
    }

    fn sync_read_state(
        &mut self,
        port: usize,
        model: Model,
        ids: &[u8],
        out: &mut [Result<RawState, BackendError>],
    ) -> Result<(), BackendError> {
        self.0.sync_read_state(port, model, ids, out)
    }

    fn read_vitals(
        &mut self,
        port: usize,
        model: Model,
        vital: Vital,
        ids: &[u8],
        out: &mut [Result<Reading, BackendError>],
    ) -> Result<(), BackendError> {
        self.0.read_vitals(port, model, vital, ids, out)
    }

    fn sync_write_goal(
        &mut self,
        port: usize,
        model: Model,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError> {
        self.0.sync_write_goal(port, model, goals)
    }

    fn sync_write_start_goal(
        &mut self,
        port: usize,
        model: Model,
        register: GoalRegister,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError> {
        self.0.sync_write_start_goal(port, model, register, goals)
    }

    fn torque(&mut self, port: usize, model: Model, id: u8, on: bool) -> Result<(), BackendError> {
        self.0.torque(port, model, id, on)
    }

    fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        self.0.force_torque_off(port, model, id)
    }

    fn read_setting(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        setting: Setting,
    ) -> Result<u8, BackendError> {
        self.0.read_setting(port, model, id, setting)
    }

    fn read_health(&mut self, port: usize, model: Model, id: u8) -> Result<Health, BackendError> {
        self.0.read_health(port, model, id)
    }

    fn write_health_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        limits: HealthLimits,
    ) -> Result<(), BackendError> {
        self.0.write_health_limits(port, model, id, limits)
    }

    fn read_angle_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<(u32, u32), BackendError> {
        self.0.read_angle_limits(port, model, id)
    }

    fn close_ports(&mut self) {
        self.0.close_ports();
    }
}
