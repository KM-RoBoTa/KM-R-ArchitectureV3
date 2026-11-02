//! The private seam between the adapter and a Dynamixel SDK.
//!
//! One implementation per SDK, plus the scripted one the tests run on
//! (research R7). Values stay raw here: units are converted by the adapter.

use core::fmt;
use std::io::ErrorKind;
use std::path::Path;

use super::config::Model;

/// Why a backend call failed: one variant per cause, with its data. `Io`
/// is a failed serial transaction, nothing else.
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
    /// A serial transaction failed, as the SDK reports it.
    #[error("serial transaction failed: {0}")]
    Io(Wire),
    /// The actuator answered with these error bits set.
    #[error("status packet error {0:#04x}")]
    Status(u8),
    /// The actuator answered with an error the SDK gave no code for.
    #[error("status packet error, its code not given by the SDK")]
    StatusUncoded,
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
    /// The C SDK was already brought up in this process.
    #[error("the C SDK was already brought up in this process, and takes one bring-up per process")]
    SdkClaimed,
    /// The device path is not UTF-8, which the SDK needs.
    #[error("the device path is not UTF-8")]
    PathNotUtf8,
    /// The device path holds a NUL byte.
    #[error("the device path holds a NUL byte")]
    PathNul,
    /// The baud does not fit the SDK's `i32`.
    #[error("baud {baud} does not fit the SDK's i32")]
    BaudTooHigh {
        /// Baud of the bus.
        baud: u32,
    },
    /// The SDK's port builder refused the settings.
    #[error("the SDK refused the port settings")]
    PortSettings,
    /// The SDK could not open the port.
    #[error("the SDK could not open the port")]
    PortOpen,
    /// The SDK could not set the baud rate.
    #[error("the SDK could not set the baud rate")]
    BaudRate,
    /// The SDK has no control table to load.
    #[error("the SDK found no control table")]
    NoControlTable,
    /// The SDK refuses both protocols on one bus.
    #[error("the SDK refuses both protocols on one bus")]
    MixedProtocols,
    /// The SDK could not build a sync read or sync write group.
    #[error("the SDK could not build the sync group")]
    SyncGroup,
    /// A broadcast ping failed or got no answer.
    #[error("the broadcast ping failed or got no answer")]
    BroadcastPing,
    /// An actuator this process did not switch on: no handle is held for it.
    #[error("id {id} was not switched on by this process, which holds no handle for it")]
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
    /// Every handle slot is taken.
    #[error("no handle slot left: at most {max} actuators switched on")]
    HandlesFull {
        /// Handle slots.
        max: usize,
    },
    /// The SDK's torque-on list does not hold the actuator.
    #[error("id {id} is not in the SDK's torque-on list")]
    NotListed {
        /// Firmware id.
        id: u8,
    },
    /// More goals than one write carries.
    #[error("{goals} goals in one write, at most {max}")]
    TooManyGoals {
        /// Goals given.
        goals: usize,
        /// Goals one write carries.
        max: usize,
    },
    /// A goal position the SDK's `u16` sync write cannot carry.
    #[error("goal position {raw} for id {id} above u16::MAX; the dxl_rs sync write takes u16")]
    GoalTooWide {
        /// Firmware id.
        id: u8,
        /// Raw goal.
        raw: u32,
    },
    /// The SDK refused a goal outside the angle limits it holds.
    #[error("goal position {goal} outside the angle limits {min}–{max}")]
    GoalOutsideLimits {
        /// Raw goal.
        goal: u32,
        /// Lowest raw goal.
        min: u32,
        /// Highest raw goal.
        max: u32,
    },
    /// A goal register the SDK does not write.
    #[error("cannot write {register}: dxl_rs writes Goal Position only")]
    Unwritable {
        /// The register.
        register: GoalRegister,
    },
    /// A sync read asked of Protocol 1, which has none.
    #[error("Protocol 1 has no sync read")]
    NoSyncRead,
    /// A setting asked of Protocol 1.
    #[error("{setting} is not read on Protocol 1")]
    NoSetting {
        /// The setting.
        setting: Setting,
    },
    /// CW and CCW Angle Limit asked of a Protocol 2 actuator.
    #[error("CW and CCW Angle Limit are Protocol 1 registers")]
    NoAngleLimits,
    /// Result slots and ids differ in number.
    #[error("{slots} result slots for {ids} ids")]
    SlotMismatch {
        /// Ids named.
        ids: usize,
        /// Result slots given.
        slots: usize,
    },
    /// A sync read answered another number of rows than of ids.
    #[error("the sync read answered {rows} rows for {ids} ids")]
    RowMismatch {
        /// Ids named.
        ids: usize,
        /// Rows answered.
        rows: usize,
    },
    /// A sync read gave no value for one actuator.
    #[error("the sync read gave no value for id {id}")]
    NoValue {
        /// Firmware id.
        id: u8,
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
        value: u32,
    },
}

/// How a serial transaction failed, as the SDK reports it.
#[cfg_attr(
    not(feature = "dynamixel"),
    expect(dead_code, reason = "only the SDK backend reports every one")
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(crate) enum Wire {
    /// The port is in use.
    #[error("port busy")]
    PortBusy,
    /// The instruction packet was not sent.
    #[error("instruction packet not sent")]
    TxFail,
    /// No status packet was received.
    #[error("status packet not received")]
    RxFail,
    /// The instruction packet was malformed.
    #[error("instruction packet malformed")]
    TxError,
    /// The status packet was incomplete.
    #[error("status packet incomplete")]
    RxWaiting,
    /// The status packet was corrupted.
    #[error("status packet corrupted")]
    RxCorrupt,
    /// The instruction is not available.
    #[error("instruction not available")]
    NotAvailable,
    /// A result code the SDK does not name.
    #[error("SDK result code {0}")]
    Unknown(i32),
    /// The operating system refused the call.
    #[error("{0}")]
    Os(ErrorKind),
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

/// A one-byte Protocol 2 register checked at bring-up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Setting {
    /// Operating Mode; 3 is position mode.
    OperatingMode,
    /// Torque Enable; 1 is powered.
    TorqueEnable,
}

impl Setting {
    /// The register's ROBOTIS name.
    pub(crate) const fn name(self) -> &'static str {
        match self {
            Self::OperatingMode => "Operating Mode",
            Self::TorqueEnable => "Torque Enable",
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
/// current, velocity and position, one sync read each, as `dxl_rs` reads
/// them until its 10-byte block read (research R12). Both backends and the
/// presets' declared costs read it, so one change moves all three.
pub(crate) const STATE_READS: &[u16] = &[2, 4, 4];

/// What the adapter needs from an SDK.
///
/// `port` indexes the slice given to `open_ports`; `model` is the model of
/// the actuators named, one series per call.
pub(crate) trait Backend {
    /// Ports one process can open.
    const MAX_PORTS: usize;

    /// Data bytes of each Protocol 2 sync read one `sync_read_state` sends.
    const STATE_READS: &'static [u16];

    /// Registers every port, opens every one, then initializes once: the C
    /// SDK keeps global port state (ROBOTIS issue #124).
    fn open_ports(&mut self, ports: &[PortSpec<'_>]) -> Result<(), OpenFailure>;

    /// Whether actuator `id` answers.
    fn ping(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError>;

    /// Present position of one actuator.
    fn read_position(&mut self, port: usize, model: Model, id: u8) -> Result<u32, BackendError>;

    /// Present state of one Protocol 2 group in one transaction: `out[i]`
    /// answers `ids[i]`. `Err` when the whole transaction failed.
    fn sync_read_state(
        &mut self,
        port: usize,
        model: Model,
        ids: &[u8],
        out: &mut [Result<RawState, BackendError>],
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
    /// transaction where the SDK allows it; `DxlRs` writes and confirms one
    /// goal per actuator. Refused whole, before any write, when one of them
    /// is switched on here.
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
    /// backend switched it on; clears any handle kept for it.
    fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError>;

    /// One setting of one Protocol 2 actuator.
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

    /// CW and CCW Angle Limit of one Protocol 1 actuator, raw: both 0 is
    /// wheel mode, anything else joint mode, its goals in `cw..=ccw`.
    fn read_angle_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<(u32, u32), BackendError>;

    /// Closes every port. Best effort.
    fn close_ports(&mut self);
}
