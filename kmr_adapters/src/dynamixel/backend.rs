//! The private seam between the adapter and a Dynamixel SDK.
//!
//! One implementation per SDK, plus the scripted one the tests run on
//! (research R7). Values stay raw here: units are converted by the adapter.

use std::path::Path;

use super::config::Model;

/// Why a backend call failed.
#[cfg_attr(
    not(any(test, feature = "dynamixel")),
    expect(
        dead_code,
        reason = "only the SDK backend and the tests fail with these"
    )
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(crate) enum BackendError {
    /// No status packet before the timeout.
    #[error("no answer before the timeout")]
    Timeout,
    /// The port could not be read or written.
    #[error("serial port I/O failed")]
    Io,
    /// The actuator answered with these error bits set.
    #[error("status packet error {0:#04x}")]
    Status(u8),
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

/// What the PostInit health check reads from one actuator.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Health {
    /// Hardware Error Status bits; 0 where the protocol has no such register.
    pub(crate) error_status: u8,
    /// Present input voltage, in 0.1 V.
    pub(crate) voltage_dv: u16,
    /// Present temperature, in °C.
    pub(crate) temperature_c: u8,
}

/// A one-byte Protocol 2 register checked at bring-up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Setting {
    /// Operating Mode; 3 is position mode.
    OperatingMode,
    /// Torque Enable; 1 is powered.
    TorqueEnable,
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

    /// Error status, voltage and temperature of one actuator.
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
