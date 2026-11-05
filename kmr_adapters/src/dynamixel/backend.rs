//! The private seam between the adapter and a Dynamixel SDK.
//!
//! `DxlRsV2` implements it on `dxl_rs_v2`, the scripted backend for the
//! tests (research R7). Values stay raw here: units are converted by the
//! adapter.

use core::fmt;
use std::io::ErrorKind;
use std::path::Path;

use super::config::{HealthLimits, Model, Protocol};

/// Why a backend call failed: one variant per cause, with its data. `Io` is
/// the serial line failing, nothing else.
#[cfg_attr(
    not(feature = "dynamixel"),
    expect(
        dead_code,
        reason = "the SDK backend fails with every variant, the scripted one with some"
    )
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(crate) enum BackendError {
    /// No status packet before the timeout.
    #[error("no answer before the timeout")]
    Timeout,
    /// A status packet came, but not one the SDK could use.
    #[error("{0}")]
    Reply(Reply),
    /// The serial line failed.
    #[error("serial line failed: {0}")]
    Io(Wire),
    /// The actuator answered with these error bits set.
    #[error("status packet error {0:#04x}")]
    Status(u8),
    /// The actuator reports a hardware fault.
    #[error("{0}")]
    Fault(Fault),
    /// Torque on refused: the actuator reported a hardware fault this
    /// process did not clear.
    #[error("id {id} reported a hardware fault; its torque stays off")]
    Faulted {
        /// Firmware id.
        id: u8,
    },
    /// Another model answers at the id.
    #[error("Model Number {found} answers")]
    WrongModel {
        /// Model Number read.
        found: u16,
    },
    /// The SDK has no control table for the model.
    #[error("model number {number}: the SDK has no control table for it")]
    UnsupportedModel {
        /// Model Number of the model.
        number: u16,
    },
    /// The mode register holds a value the model's table does not list.
    #[error("unknown operating mode {value}")]
    UnknownMode {
        /// Value read.
        value: u8,
    },
    /// The call names a port that is not open.
    #[error("port {port} is not open")]
    PortNotOpen {
        /// Index of the port.
        port: usize,
    },
    /// More ports than the backend opens.
    #[error("{ports} ports given, the backend opens at most {max}")]
    TooManyPorts {
        /// Ports given.
        ports: usize,
        /// Ports the backend opens.
        max: usize,
    },
    /// The device path is not UTF-8, which the serial port needs.
    #[error("the device path is not UTF-8")]
    PathNotUtf8,
    /// The serial port could not be opened.
    #[error("the serial port could not be opened: {0}")]
    PortOpen(ErrorKind),
    /// An actuator its port does not hold: no `declare` named it, or the
    /// last one failed.
    #[error("id {id} is not set up on its port")]
    NotDeclared {
        /// Firmware id.
        id: u8,
    },
    /// An actuator this process did not switch on.
    #[error("id {id} was not switched on by this process")]
    NotPowered {
        /// Firmware id.
        id: u8,
    },
    /// A start goal names an actuator already switched on.
    #[error("id {id} is already switched on; a start goal is written with the torque off")]
    AlreadyPowered {
        /// Firmware id.
        id: u8,
    },
    /// Torque on, with no start goal written since the torque was last
    /// switched.
    #[error(
        "torque on refused for id {id}: no start goal written since its torque was last switched"
    )]
    NoStartGoal {
        /// Firmware id.
        id: u8,
    },
    /// The SDK refused a goal outside the position limits it holds.
    #[error("goal position {goal} outside the position limits {min}–{max}")]
    GoalOutsideLimits {
        /// Raw goal.
        goal: i32,
        /// Lowest raw goal.
        min: i32,
        /// Highest raw goal.
        max: i32,
    },
    /// A goal register written before the torque that the backend does not
    /// write.
    #[error("cannot write {register} before the torque: only Goal Position is")]
    Unwritable {
        /// The register.
        register: GoalRegister,
    },
    /// A sync read asked of Protocol 1, which has none.
    #[error("Protocol 1 has no sync read")]
    NoSyncRead,
    /// A Protocol 2 setting asked of Protocol 1.
    #[error("{setting} is not read on Protocol 1")]
    NoSetting {
        /// The setting.
        setting: Setting,
    },
    /// Position limits asked of an actuator the SDK read none of.
    #[error("no position limits read for the actuator")]
    NoAngleLimits,
    /// Result slots and ids differ in number.
    #[error("{slots} result slots for {ids} ids")]
    SlotMismatch {
        /// Ids named.
        ids: usize,
        /// Result slots given.
        slots: usize,
    },
    /// The model has no such register.
    #[error("the model has no {register} register")]
    NoRegister {
        /// ROBOTIS name of the register.
        register: &'static str,
    },
    /// A register value wider than the type it is read into.
    #[error("{register} read {value}, wider than the register")]
    Overflow {
        /// ROBOTIS name of the register.
        register: &'static str,
        /// Value read.
        value: i32,
    },
    /// The SDK refused the call before sending anything.
    #[error("refused before sending: {reason}")]
    Refused {
        /// What the SDK refused.
        reason: &'static str,
    },
}

/// A status packet that came but could not be used.
#[cfg_attr(
    not(feature = "dynamixel"),
    expect(dead_code, reason = "only the SDK backend reports them")
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(crate) enum Reply {
    /// It came after the timeout: a USB latency timer above it.
    #[error("status packet after the timeout")]
    Late,
    /// Part of it came, then nothing.
    #[error("status packet cut short")]
    Partial,
    /// It failed its checksum or CRC.
    #[error("status packet corrupted")]
    Corrupt,
    /// Its id or length was not the one asked for.
    #[error("status packet malformed")]
    Malformed,
}

/// How the serial line failed.
#[cfg_attr(
    not(feature = "dynamixel"),
    expect(dead_code, reason = "only the SDK backend reports every one")
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(crate) enum Wire {
    /// The port's own read or write timeout expired.
    #[error("port timeout")]
    PortTimeout,
    /// The port is gone: a USB adapter unplugged.
    #[error("port closed")]
    Closed,
    /// The operating system refused the call.
    #[error("{0}")]
    Os(ErrorKind),
}

/// A hardware fault an actuator reports, measured against its own limits.
#[cfg_attr(
    not(feature = "dynamixel"),
    expect(dead_code, reason = "only the SDK backend measures faults")
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(crate) enum Fault {
    /// The input voltage is outside its limits.
    #[error("input voltage {measured_dv} dV outside {min_dv}–{max_dv} dV")]
    Voltage {
        /// Present input voltage, in 0.1 V.
        measured_dv: u16,
        /// Min Voltage Limit, in 0.1 V.
        min_dv: u16,
        /// Max Voltage Limit, in 0.1 V.
        max_dv: u16,
    },
    /// The temperature is above its limit.
    #[error("temperature {measured_c} °C above {limit_c} °C")]
    Temperature {
        /// Present temperature, in °C.
        measured_c: u8,
        /// Temperature Limit, in °C.
        limit_c: u8,
    },
    /// Any other fault, or one the measure no longer shows.
    #[error("Hardware Error Status {status:#010b}")]
    Hardware {
        /// Hardware Error Status bits, X series layout.
        status: u8,
    },
}

/// Why `open_ports` failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct OpenFailure {
    /// Index of the port that failed; `None` when no single port did.
    pub(crate) port: Option<usize>,
    /// What the backend reported.
    pub(crate) source: BackendError,
}

/// Present state of one actuator, in register units.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct RawState {
    /// Present position, signed on Protocol 2.
    pub(crate) position: i32,
    /// Present velocity, signed.
    pub(crate) velocity: i32,
    /// Present current, signed.
    pub(crate) current: i16,
}

/// What the PostInit health check reads from one actuator: present values,
/// and the limits it holds in its own control table.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Health {
    /// Hardware Error Status bits; 0 where the protocol has no such register.
    pub(crate) error_status: u8,
    /// Present input voltage, in 0.1 V.
    pub(crate) voltage_dv: u16,
    /// Present temperature, in °C.
    pub(crate) temperature_c: u8,
    /// Min Voltage Limit, in 0.1 V.
    pub(crate) min_voltage_dv: u16,
    /// Max Voltage Limit, in 0.1 V.
    pub(crate) max_voltage_dv: u16,
    /// Temperature Limit, in °C.
    pub(crate) temperature_limit_c: u8,
}

/// One telemetry register, read for a whole Protocol 2 group in one sync
/// read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Vital {
    /// Present Input Voltage.
    Voltage,
    /// Present Temperature.
    Temperature,
    /// Hardware Error Status.
    HardwareError,
}

impl Vital {
    /// Every register, in the order the telemetry rounds read them.
    pub(crate) const ALL: [Self; 3] = [Self::Voltage, Self::Temperature, Self::HardwareError];
}

/// What one telemetry read got from one actuator, in register units: the
/// values it carried, `None` for the rest. A faulted actuator answers with
/// what its fault measured.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Reading {
    /// Present input voltage, in 0.1 V.
    pub(crate) voltage_dv: Option<u16>,
    /// Present temperature, in °C.
    pub(crate) temperature_c: Option<u8>,
    /// Hardware Error Status bits, X series layout.
    pub(crate) hardware_error: Option<u8>,
}

#[cfg_attr(
    not(any(test, feature = "dynamixel")),
    expect(
        dead_code,
        reason = "only the backends use it, and both are compiled out"
    )
)]
impl Reading {
    /// The reading of `raw`, read from `vital`'s register.
    pub(crate) fn of(vital: Vital, raw: i32) -> Self {
        let byte = |raw: i32| u8::try_from(raw).ok();
        match vital {
            Vital::Voltage => Self {
                voltage_dv: u16::try_from(raw).ok(),
                ..Self::default()
            },
            Vital::Temperature => Self {
                temperature_c: byte(raw),
                ..Self::default()
            },
            Vital::HardwareError => Self {
                hardware_error: byte(raw),
                ..Self::default()
            },
        }
    }
}

/// Address and width of one register.
#[cfg_attr(
    not(any(test, feature = "dynamixel")),
    expect(
        dead_code,
        reason = "only the backends use them, and both are compiled out"
    )
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Register {
    /// Address in the control table.
    pub(crate) address: u16,
    /// Width, in bytes.
    pub(crate) size: u16,
}

/// The three health limit registers of a model, all EEPROM: written with
/// the torque off only on Protocol 2.
#[cfg_attr(
    not(any(test, feature = "dynamixel")),
    expect(
        dead_code,
        reason = "only the backends use them, and both are compiled out"
    )
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LimitRegisters {
    /// Temperature Limit; Highest Limit Temperature on Protocol 1.
    pub(crate) temperature: Register,
    /// Min Voltage Limit; Lowest Limit Voltage on Protocol 1.
    pub(crate) min_voltage: Register,
    /// Max Voltage Limit; Highest Limit Voltage on Protocol 1.
    pub(crate) max_voltage: Register,
}

#[cfg_attr(
    not(any(test, feature = "dynamixel")),
    expect(
        dead_code,
        reason = "only the backends use them, and both are compiled out"
    )
)]
impl LimitRegisters {
    /// The registers of `model`: AX, RX and Protocol 1 MX at 11, 12 and 13,
    /// one byte each; X series and MX(2.0) at 31, 34 and 32, the voltages
    /// two bytes (ROBOTIS e-Manual).
    pub(crate) const fn of(model: Model) -> Self {
        const fn at(address: u16, size: u16) -> Register {
            Register { address, size }
        }
        match model.protocol() {
            Protocol::One => Self {
                temperature: at(11, 1),
                min_voltage: at(12, 1),
                max_voltage: at(13, 1),
            },
            Protocol::Two => Self {
                temperature: at(31, 1),
                min_voltage: at(34, 2),
                max_voltage: at(32, 2),
            },
        }
    }

    /// Whether every value of `limits` fits its register.
    pub(crate) const fn fit(self, limits: HealthLimits) -> bool {
        const fn fits(register: Register, value: u16) -> bool {
            register.size >= 2 || value <= u8::MAX as u16
        }
        fits(self.min_voltage, limits.min_voltage_dv)
            && fits(self.max_voltage, limits.max_voltage_dv)
    }
}

/// A one-byte register checked at bring-up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Setting {
    /// Operating Mode, Protocol 2; 3 is position mode.
    OperatingMode,
    /// Torque Enable, Protocol 2; 1 is powered.
    TorqueEnable,
    /// Torque Ctrl Mode Enable (70), Protocol 1 MX-64 only; 1 drives Goal
    /// Torque and ignores Goal Position.
    TorqueCtrlModeEnable,
}

impl Setting {
    /// The register's ROBOTIS name.
    pub(crate) const fn name(self) -> &'static str {
        match self {
            Self::OperatingMode => "Operating Mode",
            Self::TorqueEnable => "Torque Enable",
            Self::TorqueCtrlModeEnable => "Torque Ctrl Mode Enable",
        }
    }
}

impl fmt::Display for Setting {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Operating Mode value of position mode, `0..=4095` (X series).
pub(crate) const POSITION_MODE: u8 = 3;

/// The goal register an operating mode consumes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum GoalRegister {
    /// Goal Position.
    Position,
    /// Goal Velocity; Moving Speed on Protocol 1.
    Velocity,
    /// Goal Current.
    Current,
}

impl GoalRegister {
    /// Every register, in declaration order.
    pub(crate) const ALL: [Self; 3] = [Self::Position, Self::Velocity, Self::Current];
}

impl fmt::Display for GoalRegister {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Position => "Goal Position",
            Self::Velocity => "Goal Velocity",
            Self::Current => "Goal Current",
        })
    }
}

impl BackendError {
    /// Whether the failure is the bus's, not one actuator's: the port
    /// failed or is not open.
    pub(crate) const fn is_bus(self) -> bool {
        matches!(self, Self::Io(_) | Self::PortNotOpen { .. })
    }
}

/// One port to open.
#[cfg_attr(
    not(any(test, feature = "dynamixel")),
    expect(dead_code, reason = "only the SDK backend and the tests open ports")
)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct PortSpec<'a> {
    /// Resolved device, never a `/dev/serial/by-id` link.
    pub(crate) path: &'a Path,
    /// Baud rate of the bus.
    pub(crate) baud: u32,
}

/// Data bytes of each Protocol 2 sync read one state read sends: present
/// current, velocity and position, one sync read each. `dxl_rs_v2` keeps a
/// result per actuator (SRS FR-2.6) for one field per sync read only; its
/// one-transaction bulk read fails whole (research R12). Both backends and
/// the presets' declared costs read it, so one change moves all three.
pub(crate) const STATE_READS: &[u16] = &[2, 4, 4];

/// Why `declare` failed: the actuator it stopped at, and why.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Undeclared {
    /// Firmware id; `None` when no single actuator failed.
    pub(crate) id: Option<u8>,
    /// What the backend reported.
    pub(crate) source: BackendError,
}

/// What the adapter needs from an SDK.
///
/// `port` indexes the slice given to `open_ports`; `model` is the model of
/// the actuators named, one series per call.
pub(crate) trait Backend {
    /// Ports one backend opens.
    const MAX_PORTS: usize;

    /// Data bytes of each Protocol 2 sync read one `sync_read_state` sends.
    const STATE_READS: &'static [u16];

    /// Whether the backend has a control table for `model`. No I/O: `check`
    /// asks it.
    fn supports(model: Model) -> bool {
        let _ = model;
        true
    }

    /// Opens every port. Nothing is sent.
    fn open_ports(&mut self, ports: &[PortSpec<'_>]) -> Result<(), OpenFailure>;

    /// Sets up the actuators of `port`, in id order: each answers its ping
    /// and reports the Model Number of its declared model, then is held,
    /// its torque state off. Stops at the first that fails. Replaces what
    /// the port held; every later call names an actuator held here.
    fn declare(&mut self, port: usize, actuators: &[(u8, Model)]) -> Result<(), Undeclared>;

    /// Present position of one actuator.
    fn read_position(&mut self, port: usize, model: Model, id: u8) -> Result<u32, BackendError>;

    /// Present state of one Protocol 2 group: `out[i]` answers `ids[i]`.
    /// `Err` when the line failed under the whole group.
    fn sync_read_state(
        &mut self,
        port: usize,
        model: Model,
        ids: &[u8],
        out: &mut [Result<RawState, BackendError>],
    ) -> Result<(), BackendError>;

    /// Telemetry of one group: `out[i]` answers `ids[i]`. Protocol 2 reads
    /// `vital` of every id in one sync read; Protocol 1 reads the voltage
    /// and the temperature of each id in one read, whatever `vital`, and
    /// the fault bits its status packet carries. `Err` when the line failed
    /// under the whole group.
    fn read_vitals(
        &mut self,
        port: usize,
        model: Model,
        vital: Vital,
        ids: &[u8],
        out: &mut [Result<Reading, BackendError>],
    ) -> Result<(), BackendError>;

    /// Goal positions of one group, `(id, raw)`, in one transaction.
    fn sync_write_goal(
        &mut self,
        port: usize,
        model: Model,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError>;

    /// Goals of one group, `(id, raw)`, in `register`, none of them switched
    /// on here: what each holds once `torque` switches it on. One
    /// transaction where the SDK allows it; `DxlRsV2` writes and confirms
    /// one goal per actuator. Refused whole, before any write, when one of
    /// them is switched on here.
    fn sync_write_start_goal(
        &mut self,
        port: usize,
        model: Model,
        register: GoalRegister,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError>;

    /// Switches the torque of one actuator: on only once
    /// `sync_write_start_goal` wrote its goal since it was last switched on
    /// or off; off only for one this backend switched on.
    fn torque(&mut self, port: usize, model: Model, id: u8, on: bool) -> Result<(), BackendError>;

    /// Writes Torque Enable = 0 to actuator `id`, whether or not this
    /// backend switched it on, and whether or not its port holds it.
    fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError>;

    /// One setting of one actuator: Operating Mode and Torque Enable on
    /// Protocol 2, Torque Ctrl Mode Enable on the Protocol 1 MX-64.
    fn read_setting(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        setting: Setting,
    ) -> Result<u8, BackendError>;

    /// Error status, voltage and temperature of one actuator, and its
    /// voltage and temperature limits.
    fn read_health(&mut self, port: usize, model: Model, id: u8) -> Result<Health, BackendError>;

    /// Writes the Min and Max Voltage Limit and the Temperature Limit of one
    /// actuator ([`LimitRegisters`]). On Protocol 2 the actuator refuses it
    /// while its torque is on; reading them back is `read_health`.
    fn write_health_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        limits: HealthLimits,
    ) -> Result<(), BackendError>;

    /// Position limits of one actuator, raw, `(low, high)`: CW and CCW Angle
    /// Limit on Protocol 1, both 0 being wheel mode; Min and Max Position
    /// Limit on Protocol 2. Its goals stay in `low..=high`.
    fn read_angle_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<(u32, u32), BackendError>;

    /// Closes every port. Best effort.
    fn close_ports(&mut self);
}
