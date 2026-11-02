//! The [`Backend`] on the `dxl_rs` SDK, a wrapper over the ROBOTIS C SDK
//! (research R7).
//!
//! `dxl_rs` hands out a torque-on actuator only from the call that switches
//! its torque on, and only that value takes a goal in the loop. The backend
//! keeps those values, so it commands only the actuators it powered itself.
//! A torque-off actuator, built for any id without I/O, writes the start
//! goal before the torque and forces the torque off.

use core::sync::atomic::{AtomicBool, Ordering};

use ::dxl_rs::actuator::ax_series::Ax12A;
use ::dxl_rs::actuator::mx_series::Mx64Ar;
use ::dxl_rs::actuator::{
    Actuator, ActuatorOff, ActuatorOn, Mx64P1, Readable, TorqueOn, Xm430W210, Xw430T200, Xw540T140,
    Xw540T260,
};
use ::dxl_rs::actuators_list::{ActuatorVariant, ActuatorsList, P1, P2};
use ::dxl_rs::error::{
    ActuatorError, CommunicationError, Error, PortError, StatusPacketError, TransportError,
};
use ::dxl_rs::port::{PortBuilder, PortHandler, PortNumber};
use ::dxl_rs::transport::group_sync_read;
use kmr_core::state::JOINTS;
use tracing::warn;

use super::backend::{
    Backend, BackendError, GoalRegister, Health, OpenFailure, PortSpec, RawState, Setting, Wire,
};
use super::config::{Model, Protocol};

/// Ports one process can drive.
const MAX_PORTS: usize = 4;
/// Firmware ids, `0..=252`.
const IDS: usize = 253;

/// Present Current, Protocol 2: `i16` at 126.
const PRESENT_CURRENT: (u16, u16) = (126, 2);
/// Present Velocity, Protocol 2: `i32` at 128.
const PRESENT_VELOCITY: (u16, u16) = (128, 4);
/// Present Position, Protocol 2: `i32` at 132.
const PRESENT_POSITION: (u16, u16) = (132, 4);

// `STATE_READS` is what `sync_read_state` sends: one sync read per register.
const _: () = assert!(
    matches!(
        super::backend::STATE_READS,
        [current, velocity, position]
            if *current == PRESENT_CURRENT.1
                && *velocity == PRESENT_VELOCITY.1
                && *position == PRESENT_POSITION.1
    ),
    "STATE_READS differs from the registers sync_read_state reads"
);
/// Operating Mode, X series: `u8` at 11.
const OPERATING_MODE: (u16, u16) = (11, 1);
/// Torque Enable, X series: `u8` at 64.
const TORQUE_ENABLE: (u16, u16) = (64, 1);

/// Whether this process claimed the C SDK. It keeps global port state and
/// takes one bring-up per process (ROBOTIS issue #124).
static SDK_CLAIMED: AtomicBool = AtomicBool::new(false);

/// Whether the three-transaction state read was reported.
static SPLIT_READ_WARNED: AtomicBool = AtomicBool::new(false);
/// Whether `torque(.., false)` on an actuator not switched on here was
/// reported.
static UNPOWERED_WARNED: AtomicBool = AtomicBool::new(false);
/// Whether the 16-bit goal limit was reported.
static GOAL_RANGE_WARNED: AtomicBool = AtomicBool::new(false);
/// Whether the missing velocity and current goals were reported.
static GOAL_REGISTER_WARNED: AtomicBool = AtomicBool::new(false);

/// Logs `message` the first time `flag` is raised.
fn warn_once(flag: &AtomicBool, message: &str) {
    if !flag.swap(true, Ordering::Relaxed) {
        warn!("dxl_rs: {message}");
    }
}

/// Runs `$body` with `$actuator` bound to the torque-off `dxl_rs` actuator of
/// `$model`. The RX-28 goes through the AX-12A table (research R7), and so
/// does the Protocol 1 MX-64, so it joins the AX sync write: every register
/// used here has the AX address and size on both. Only its Torque Ctrl Mode
/// Enable, past the AX table, is read through the `dxl_rs` `Mx64P1`.
macro_rules! with_actuator {
    ($model:expr, $id:expr, $number:expr, $actuator:ident => $body:expr) => {
        match $model {
            Model::Ax12a | Model::Rx28 | Model::Mx64 => {
                let $actuator = Ax12A::new($id, $number);
                $body
            }
            Model::Mx64V2 => {
                let $actuator = Mx64Ar::new($id, $number);
                $body
            }
            Model::Xm430W210 => {
                let $actuator = Xm430W210::new($id, $number);
                $body
            }
            Model::Xw430T200 => {
                let $actuator = Xw430T200::new($id, $number);
                $body
            }
            Model::Xw540T140 => {
                let $actuator = Xw540T140::new($id, $number);
                $body
            }
            Model::Xw540T260 => {
                let $actuator = Xw540T260::new($id, $number);
                $body
            }
        }
    };
}

/// The torque-on actuators of one port, for its sync writes.
#[derive(Debug, Default)]
struct Lists {
    /// Protocol 1 actuators.
    one: ActuatorsList<P1, TorqueOn>,
    /// Protocol 2 actuators.
    two: ActuatorsList<P2, TorqueOn>,
}

/// One actuator this backend switched on.
#[derive(Debug)]
struct Powered {
    /// Index into the opened ports.
    port: usize,
    /// The value that can switch it off.
    actuator: ActuatorVariant<TorqueOn>,
}

/// The `dxl_rs` backend. One per process: the C SDK keeps global port state.
#[derive(Debug)]
pub struct DxlRs {
    /// SDK number of each opened port.
    numbers: [PortNumber; MAX_PORTS],
    /// Ports opened.
    opened: usize,
    /// Sync write lists of each port.
    lists: [Lists; MAX_PORTS],
    /// Actuators switched on, at most one entry each.
    powered: [Option<Powered>; JOINTS],
    /// Whether `(port, id)` is switched on.
    on: [[bool; IDS]; MAX_PORTS],
    /// Whether `(port, id)` got its start goal since it was last switched.
    started: [[bool; IDS]; MAX_PORTS],
}

impl DxlRs {
    /// A backend with no port open.
    pub fn new() -> Self {
        Self {
            numbers: [0; MAX_PORTS],
            opened: 0,
            lists: Default::default(),
            powered: [const { None }; JOINTS],
            on: [[false; IDS]; MAX_PORTS],
            started: [[false; IDS]; MAX_PORTS],
        }
    }

    /// Whether actuator `id` on `port` got its start goal.
    fn start_slot(&mut self, port: usize, id: u8) -> Option<&mut bool> {
        self.started
            .get_mut(port)
            .and_then(|ids| ids.get_mut(usize::from(id)))
    }

    /// SDK number of port `port`.
    fn number(&self, port: usize) -> Result<PortNumber, BackendError> {
        self.numbers
            .get(..self.opened)
            .and_then(|numbers| numbers.get(port))
            .copied()
            .ok_or(BackendError::PortNotOpen { port })
    }

    /// Whether actuator `id` on `port` was switched on here.
    fn is_on(&self, port: usize, id: u8) -> bool {
        self.on
            .get(port)
            .and_then(|ids| ids.get(usize::from(id)))
            .copied()
            .unwrap_or(false)
    }

    /// Records whether actuator `id` on `port` is switched on.
    fn set_on(&mut self, port: usize, id: u8, on: bool) {
        if let Some(slot) = self
            .on
            .get_mut(port)
            .and_then(|ids| ids.get_mut(usize::from(id)))
        {
            *slot = on;
        }
    }

    /// Switches actuator `id` on, and keeps the value that switches it off.
    fn power(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        number: PortNumber,
    ) -> Result<(), BackendError> {
        let known = self.is_on(port, id);
        let free = self.powered.iter().position(Option::is_none);
        let full = BackendError::HandlesFull { max: JOINTS };
        if !known && free.is_none() {
            return Err(full);
        }
        let lists = self
            .lists
            .get_mut(port)
            .ok_or(BackendError::PortNotOpen { port })?;
        let actuator: ActuatorVariant<TorqueOn> = match model {
            Model::Ax12a | Model::Rx28 | Model::Mx64 => {
                let on = enable(Ax12A::new(id, number))?;
                if !known {
                    lists.one.push(on);
                }
                on.into()
            }
            Model::Mx64V2 => {
                let on = enable(Mx64Ar::new(id, number))?;
                if !known {
                    lists.two.push(on);
                }
                on.into()
            }
            Model::Xm430W210 => {
                let on = enable(Xm430W210::new(id, number))?;
                if !known {
                    lists.two.push(on);
                }
                on.into()
            }
            Model::Xw430T200 => {
                let on = enable(Xw430T200::new(id, number))?;
                if !known {
                    lists.two.push(on);
                }
                on.into()
            }
            Model::Xw540T140 => {
                let on = enable(Xw540T140::new(id, number))?;
                if !known {
                    lists.two.push(on);
                }
                on.into()
            }
            Model::Xw540T260 => {
                let on = enable(Xw540T260::new(id, number))?;
                if !known {
                    lists.two.push(on);
                }
                on.into()
            }
        };
        if known {
            return Ok(());
        }
        let slot = free
            .and_then(|free| self.powered.get_mut(free))
            .ok_or(full)?;
        *slot = Some(Powered { port, actuator });
        self.set_on(port, id, true);
        Ok(())
    }

    /// Switches actuator `id` off with the value kept when it was switched
    /// on.
    fn unpower(&mut self, port: usize, id: u8) -> Result<(), BackendError> {
        let Some(slot) = self.powered_slot(port, id) else {
            warn_once(
                &UNPOWERED_WARNED,
                "cannot switch off the torque of an actuator this process did not switch on",
            );
            return Err(BackendError::NotPowered { id });
        };
        if let Some(powered) = slot.as_ref() {
            disable(&powered.actuator)?;
        }
        self.forget(port, id);
        Ok(())
    }

    /// Slot of the value kept for actuator `id` on `port`, if any.
    fn powered_slot(&mut self, port: usize, id: u8) -> Option<&mut Option<Powered>> {
        self.powered.iter_mut().find(|slot| {
            slot.as_ref()
                .is_some_and(|powered| powered.port == port && powered.actuator.id() == id)
        })
    }

    /// Records actuator `id` on `port` as switched off: drops the value
    /// kept for it and its start goal.
    fn forget(&mut self, port: usize, id: u8) {
        if let Some(slot) = self.powered_slot(port, id) {
            *slot = None;
        }
        self.set_on(port, id, false);
        if let Some(started) = self.start_slot(port, id) {
            *started = false;
        }
    }
}

impl Default for DxlRs {
    fn default() -> Self {
        Self::new()
    }
}

/// The backend error of an SDK error, one for each: `Io` for a failed
/// transaction only.
fn sdk_error(error: &Error) -> BackendError {
    match error {
        Error::Communication(failure) => match *failure {
            CommunicationError::RxTimeout => BackendError::Timeout,
            CommunicationError::PortBusy => BackendError::Io(Wire::PortBusy),
            CommunicationError::TxFail => BackendError::Io(Wire::TxFail),
            CommunicationError::RxFail => BackendError::Io(Wire::RxFail),
            CommunicationError::TxError => BackendError::Io(Wire::TxError),
            CommunicationError::RxWaiting => BackendError::Io(Wire::RxWaiting),
            CommunicationError::RxCorrupt => BackendError::Io(Wire::RxCorrupt),
            CommunicationError::NotAvailable => BackendError::Io(Wire::NotAvailable),
            CommunicationError::Unknown(code) => BackendError::Io(Wire::Unknown(code)),
        },
        Error::Io(error) => BackendError::Io(Wire::Os(error.kind())),
        Error::StatusPacket(StatusPacketError::NullMessage { code }) => BackendError::Status(*code),
        Error::StatusPacket(StatusPacketError::Message(_)) => BackendError::StatusUncoded,
        Error::Port(PortError::FailedToOpenPort) => BackendError::PortOpen,
        Error::Port(PortError::FailedToSetBaudrate) => BackendError::BaudRate,
        Error::Transport(TransportError::DisallowedAction) => BackendError::MixedProtocols,
        Error::Transport(TransportError::BroadcastPingFailed) => BackendError::BroadcastPing,
        Error::Transport(
            TransportError::GroupSyncWriteInitFailed
            | TransportError::GroupSyncWriteAddParamFailed
            | TransportError::GroupSyncReadAddParamFailed,
        ) => BackendError::SyncGroup,
        Error::Actuator(ActuatorError::ExeededAngleLimit(goal, min, max)) => {
            BackendError::GoalOutsideLimits {
                goal: *goal,
                min: *min,
                max: *max,
            }
        }
        Error::Actuator(ActuatorError::ActuatorNotFound(id)) => BackendError::NotListed { id: *id },
        Error::Actuator(ActuatorError::NoRegister(readable)) => BackendError::NoRegister {
            register: register_name(*readable),
        },
        Error::Cstring(_) => BackendError::PathNul,
        Error::FolderEmpty => BackendError::NoControlTable,
    }
}

/// Name of the register `readable` reads.
const fn register_name(readable: Readable) -> &'static str {
    match readable {
        Readable::PresentPosition => "Present Position",
        Readable::PresentVelocity => "Present Velocity",
        Readable::PresentPwm => "Present PWM",
        Readable::PresentCurrent => "Present Current",
        Readable::PresentLoad => "Present Load",
        Readable::PresentInputVoltage => "Present Input Voltage",
        Readable::PresentTemperature => "Present Temperature",
        Readable::Moving => "Moving",
        Readable::MovingStatus => "Moving Status",
        Readable::HardwareErrorStatus => "Hardware Error Status",
        Readable::RealtimeTick => "Realtime Tick",
        Readable::TemperatureLimit => "Temperature Limit",
        Readable::MinVoltageLimit => "Min Voltage Limit",
        Readable::MaxVoltageLimit => "Max Voltage Limit",
        Readable::ModelNumber => "Model Number",
        Readable::TorqueCtrlModeEnable => "Torque Ctrl Mode Enable",
    }
}

/// `value`, read from `register`, as a `T`.
fn narrow<T: TryFrom<u32>>(register: &'static str, value: u32) -> Result<T, BackendError> {
    T::try_from(value).map_err(|_| BackendError::Overflow { register, value })
}

/// `readable`, read with `read`, as a `T`.
fn read_as<T: TryFrom<u32>>(
    read: &impl Fn(Readable) -> Result<u32, BackendError>,
    readable: Readable,
) -> Result<T, BackendError> {
    narrow(register_name(readable), read(readable)?)
}

/// Switches the torque of `actuator` on.
fn enable<A: ActuatorOff>(actuator: A) -> Result<A::On, BackendError> {
    actuator.enable_torque().map_err(|error| sdk_error(&error))
}

/// Switches the torque of `actuator` off.
fn disable(actuator: &ActuatorVariant<TorqueOn>) -> Result<(), BackendError> {
    let off = match actuator {
        ActuatorVariant::Ax12A(inner) => inner.disable_torque().map(drop),
        ActuatorVariant::Mx64Ar(inner) => inner.disable_torque().map(drop),
        ActuatorVariant::Xm430W210(inner) => inner.disable_torque().map(drop),
        ActuatorVariant::Xw430T200(inner) => inner.disable_torque().map(drop),
        ActuatorVariant::Xw540T140(inner) => inner.disable_torque().map(drop),
        ActuatorVariant::Xw540T260(inner) => inner.disable_torque().map(drop),
    };
    off.map_err(|error| sdk_error(&error))
}

/// One Protocol 2 register of one actuator, in one sync read.
#[allow(clippy::disallowed_types)] // SDK boundary (research R7)
fn read_register(
    number: PortNumber,
    (address, size): (u16, u16),
    id: u8,
) -> Result<u32, BackendError> {
    let rows = group_sync_read(number, address, size, &[id]).map_err(|error| sdk_error(&error))?;
    rows.first()
        .and_then(|&(_, value)| value)
        .ok_or(BackendError::NoValue { id })
}

/// One Protocol 2 register of every actuator of `ids`, in one sync read:
/// `each(i, raw)` answers `ids[i]`.
#[allow(clippy::disallowed_types)] // SDK boundary (research R7)
fn sync_read(
    number: PortNumber,
    (address, size): (u16, u16),
    ids: &[u8],
    mut each: impl FnMut(usize, Result<u32, BackendError>),
) -> Result<(), BackendError> {
    let rows = group_sync_read(number, address, size, ids).map_err(|error| sdk_error(&error))?;
    if rows.len() != ids.len() {
        return Err(BackendError::RowMismatch {
            ids: ids.len(),
            rows: rows.len(),
        });
    }
    for (index, &(id, value)) in rows.iter().enumerate() {
        each(index, value.ok_or(BackendError::NoValue { id }));
    }
    Ok(())
}

/// Folds `raw` into `out[index]` with `apply`; an error replaces the state.
fn merge(
    out: &mut [Result<RawState, BackendError>],
    index: usize,
    raw: Result<u32, BackendError>,
    apply: impl FnOnce(&mut RawState, u32),
) {
    if let Some(slot) = out.get_mut(index) {
        *slot = match (*slot, raw) {
            (Ok(mut state), Ok(raw)) => {
                apply(&mut state, raw);
                Ok(state)
            }
            (Err(error), _) | (_, Err(error)) => Err(error),
        };
    }
}

/// The open failure of port `port` failing with `source`.
fn on_port(port: usize, source: BackendError) -> OpenFailure {
    OpenFailure {
        port: Some(port),
        source,
    }
}

impl Backend for DxlRs {
    const MAX_PORTS: usize = MAX_PORTS;
    const STATE_READS: &'static [u16] = super::backend::STATE_READS;

    /// Claims the SDK before its first call: a second bring-up in one
    /// process is an error, never a second init.
    fn open_ports(&mut self, ports: &[PortSpec<'_>]) -> Result<(), OpenFailure> {
        let whole = |source| OpenFailure { port: None, source };
        if ports.len() > MAX_PORTS {
            return Err(whole(BackendError::TooManyPorts {
                ports: ports.len(),
                max: MAX_PORTS,
            }));
        }
        if SDK_CLAIMED
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return Err(whole(BackendError::SdkClaimed));
        }
        let mut handlers: [Option<PortHandler<'_>>; MAX_PORTS] = [const { None }; MAX_PORTS];
        for (port, (handler, spec)) in handlers.iter_mut().zip(ports).enumerate() {
            let dev = spec
                .path
                .to_str()
                .ok_or(on_port(port, BackendError::PathNotUtf8))?;
            let baud = i32::try_from(spec.baud)
                .map_err(|_| on_port(port, BackendError::BaudTooHigh { baud: spec.baud }))?;
            let built = PortBuilder::default()
                .dev(dev)
                .baud(baud)
                .build()
                .map_err(|_| on_port(port, BackendError::PortSettings))?;
            *handler = Some(built);
        }
        for (port, (number, handler)) in self
            .numbers
            .iter_mut()
            .zip(handlers.iter().flatten())
            .enumerate()
        {
            *number = handler
                .get_port_number()
                .map_err(|error| on_port(port, sdk_error(&error)))?;
        }
        for (port, (&number, handler)) in self
            .numbers
            .iter()
            .zip(handlers.iter().flatten())
            .enumerate()
        {
            // Counted before the attempt, so `close_ports` also closes a port
            // whose baud rate failed.
            self.opened += 1;
            handler
                .open_port(number)
                .map_err(|error| on_port(port, sdk_error(&error)))?;
        }
        ::dxl_rs::init();
        warn_once(
            &SPLIT_READ_WARNED,
            "sync read decodes at most 4 bytes per register: present state takes three transactions per group",
        );
        Ok(())
    }

    fn ping(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        let number = self.number(port)?;
        with_actuator!(model, id, number, actuator => actuator.ping())
            .map_err(|error| sdk_error(&error))
    }

    /// One read instruction, on either protocol.
    fn read_model_number(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<u16, BackendError> {
        let number = self.number(port)?;
        let raw = with_actuator!(model, id, number, actuator => actuator.read_register(Readable::ModelNumber))
            .map_err(|error| sdk_error(&error))?;
        narrow(register_name(Readable::ModelNumber), raw)
    }

    fn read_position(&mut self, port: usize, model: Model, id: u8) -> Result<u32, BackendError> {
        let number = self.number(port)?;
        with_actuator!(model, id, number, actuator => actuator.read_present_position())
            .map_err(|error| sdk_error(&error))
    }

    /// Three sync reads, current, velocity then position: `dxl_rs` decodes
    /// at most 4 bytes of a sync read.
    fn sync_read_state(
        &mut self,
        port: usize,
        model: Model,
        ids: &[u8],
        out: &mut [Result<RawState, BackendError>],
    ) -> Result<(), BackendError> {
        let number = self.number(port)?;
        if model.protocol() != Protocol::Two {
            return Err(BackendError::NoSyncRead);
        }
        if out.len() != ids.len() {
            return Err(BackendError::SlotMismatch {
                ids: ids.len(),
                slots: out.len(),
            });
        }
        out.fill(Ok(RawState::default()));
        sync_read(number, PRESENT_CURRENT, ids, |index, raw| {
            merge(out, index, raw, |state, raw| {
                let [low, high, ..] = raw.to_le_bytes();
                state.current = i16::from_le_bytes([low, high]);
            });
        })?;
        sync_read(number, PRESENT_VELOCITY, ids, |index, raw| {
            merge(out, index, raw, |state, raw| {
                state.velocity = raw.cast_signed()
            });
        })?;
        sync_read(number, PRESENT_POSITION, ids, |index, raw| {
            merge(out, index, raw, |state, raw| {
                state.position = raw.cast_signed()
            });
        })
    }

    /// One `dxl_rs` sync write, refused whole when an actuator of `goals` was
    /// not switched on here or its goal does not fit the SDK's `u16`.
    fn sync_write_goal(
        &mut self,
        port: usize,
        model: Model,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError> {
        self.number(port)?;
        let mut narrow = [(0, 0); JOINTS];
        let narrow = narrow
            .get_mut(..goals.len())
            .ok_or(BackendError::TooManyGoals {
                goals: goals.len(),
                max: JOINTS,
            })?;
        for (slot, &(id, raw)) in narrow.iter_mut().zip(goals) {
            if !self.is_on(port, id) {
                return Err(BackendError::NotPowered { id });
            }
            let Ok(narrowed) = u16::try_from(raw) else {
                warn_once(
                    &GOAL_RANGE_WARNED,
                    "goal position above u16::MAX, the sync write of dxl_rs takes u16",
                );
                return Err(BackendError::GoalTooWide { id, raw });
            };
            *slot = (id, narrowed);
        }
        let lists = self
            .lists
            .get(port)
            .ok_or(BackendError::PortNotOpen { port })?;
        match model.protocol() {
            Protocol::One => lists.one.sync_write_goal_positions(narrow),
            Protocol::Two => lists.two.sync_write_goal_positions(narrow),
        }
        .map_err(|error| sdk_error(&error))
    }

    /// One confirmed write per actuator through a torque-off `dxl_rs`
    /// actuator, not one transaction: `dxl_rs` sync-writes only from
    /// torque-on lists. Refused whole, before any write, when one of `goals`
    /// is switched on here. Refuses every other register: `dxl_rs` writes
    /// Goal Position only.
    fn sync_write_start_goal(
        &mut self,
        port: usize,
        model: Model,
        register: GoalRegister,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError> {
        let number = self.number(port)?;
        if register != GoalRegister::Position {
            warn_once(
                &GOAL_REGISTER_WARNED,
                "cannot write Goal Velocity or Goal Current, only Goal Position",
            );
            return Err(BackendError::Unwritable { register });
        }
        if let Some(&(id, _)) = goals.iter().find(|&&(id, _)| self.is_on(port, id)) {
            return Err(BackendError::AlreadyPowered { id });
        }
        for &(id, raw) in goals {
            with_actuator!(model, id, number, actuator => actuator.set_goal_position_raw(raw))
                .map_err(|error| sdk_error(&error))?;
        }
        for &(id, _) in goals {
            if let Some(started) = self.start_slot(port, id) {
                *started = true;
            }
        }
        Ok(())
    }

    /// On only once a start goal was written since the actuator was last
    /// switched; that goal is then used up.
    fn torque(&mut self, port: usize, model: Model, id: u8, on: bool) -> Result<(), BackendError> {
        let number = self.number(port)?;
        if !on {
            return self.unpower(port, id);
        }
        if !self.start_slot(port, id).is_some_and(|started| *started) {
            return Err(BackendError::NoStartGoal { id });
        }
        self.power(port, model, id, number)?;
        if let Some(started) = self.start_slot(port, id) {
            *started = false;
        }
        Ok(())
    }

    /// Through a torque-off `dxl_rs` actuator, so it reaches an actuator
    /// this backend did not switch on; drops the value kept for it after
    /// the write.
    fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        let number = self.number(port)?;
        with_actuator!(model, id, number, actuator => actuator.force_torque_off())
            .map_err(|error| sdk_error(&error))?;
        self.forget(port, id);
        Ok(())
    }

    /// Protocol 2 settings fail on Protocol 1: it has no Operating Mode,
    /// and `dxl_rs` has no [`Readable`] for its Torque Enable. Torque Ctrl
    /// Mode Enable is one read through `Mx64P1`; any other model fails
    /// before any I/O.
    fn read_setting(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        setting: Setting,
    ) -> Result<u8, BackendError> {
        let number = self.number(port)?;
        let register = match setting {
            Setting::OperatingMode => OPERATING_MODE,
            Setting::TorqueEnable => TORQUE_ENABLE,
            Setting::TorqueCtrlModeEnable => {
                if model != Model::Mx64 {
                    return Err(BackendError::NoRegister {
                        register: setting.name(),
                    });
                }
                let raw = Mx64P1::new(id, number)
                    .read_register(Readable::TorqueCtrlModeEnable)
                    .map_err(|error| sdk_error(&error))?;
                return narrow(setting.name(), raw);
            }
        };
        if model.protocol() != Protocol::Two {
            return Err(BackendError::NoSetting { setting });
        }
        narrow(setting.name(), read_register(number, register, id)?)
    }

    /// Present voltage and temperature, and the actuator's Min and Max
    /// Voltage Limit and Temperature Limit. Protocol 2: one sync read per
    /// register, Hardware Error Status too. Protocol 1 has no sync read and
    /// no Hardware Error Status: one read per register, error status 0.
    fn read_health(&mut self, port: usize, model: Model, id: u8) -> Result<Health, BackendError> {
        let number = self.number(port)?;
        let read = |readable| match model.protocol() {
            Protocol::One => {
                with_actuator!(model, id, number, actuator => actuator.read_register(readable))
                    .map_err(|error| sdk_error(&error))
            }
            Protocol::Two => {
                let register =
                    with_actuator!(model, id, number, actuator => actuator.register_of(readable))
                        .ok_or(BackendError::NoRegister {
                        register: register_name(readable),
                    })?;
                read_register(number, (register.address, register.size), id)
            }
        };
        let error_status = match model.protocol() {
            Protocol::One => 0,
            Protocol::Two => read_as(&read, Readable::HardwareErrorStatus)?,
        };
        Ok(Health {
            error_status,
            voltage_dv: read_as(&read, Readable::PresentInputVoltage)?,
            temperature_c: read_as(&read, Readable::PresentTemperature)?,
            min_voltage_dv: read_as(&read, Readable::MinVoltageLimit)?,
            max_voltage_dv: read_as(&read, Readable::MaxVoltageLimit)?,
            temperature_limit_c: read_as(&read, Readable::TemperatureLimit)?,
        })
    }

    /// Through the goal limits `dxl_rs` reads on a torque-off actuator.
    /// Protocol 2 fails: its position limits are not read here.
    fn read_angle_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<(u32, u32), BackendError> {
        let number = self.number(port)?;
        match model {
            Model::Ax12a | Model::Rx28 | Model::Mx64 => {
                let mut actuator = Ax12A::new(id, number);
                actuator
                    .read_goal_limits()
                    .map_err(|error| sdk_error(&error))?;
                let (cw, ccw) = actuator.goal_limits();
                Ok((u32::from(cw), u32::from(ccw)))
            }
            Model::Mx64V2
            | Model::Xm430W210
            | Model::Xw430T200
            | Model::Xw540T140
            | Model::Xw540T260 => Err(BackendError::NoAngleLimits),
        }
    }

    fn close_ports(&mut self) {
        for &number in self.numbers.get(..self.opened).unwrap_or_default() {
            PortHandler::close(number);
        }
        self.opened = 0;
    }
}
