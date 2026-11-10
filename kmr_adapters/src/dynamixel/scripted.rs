//! A backend for tests: records every call and answers from a script.
//!
//! Every actuator starts present and healthy, at `RawState::default()`, its
//! goal registers at 0: what an earlier session left. Faults are scripted
//! per (port, id) and per call kind, and last until [`Scripted::heal`] or
//! for a counted number of calls. Failed calls are recorded too.
//!
//! It refuses what `DxlRsV2` refuses, with the same error, and never panics
//! (contracts/backend.md § Strictness). One exception: it answers an
//! actuator no `declare` named. As `dxl_rs_v2`, `declare` holds faulted an
//! actuator whose health script reports a hardware fault (see
//! [`Scripted::measured`]); a scripted error of a call kind never does. As
//! `dxl_rs_v2`, it takes a limit write only once a torque off answered since
//! the actuator's setup, its last torque on or a raw write.

use core::iter;
use std::ffi::OsStr;
use std::os::unix::ffi::OsStrExt;
use std::path::Path;

use super::backend::{
    Backend, BackendError, Fault, GoalRegister, Health, OpenFailure, POSITION_MODE, PortSpec,
    RamSetting, RawState, Reading, Setting, Undeclared, Vital, Wire, voltage_limits_fit,
};
use super::config::{HealthLimits, Model, Protocol};
use super::units;

/// Calls recorded; later ones are only counted.
pub(crate) const CAPACITY: usize = 256;
/// Ids recorded over all calls; later ones are dropped.
const ENTRIES: usize = 4 * CAPACITY;
/// Ports the script covers.
pub(crate) const PORTS: usize = 4;
/// Ids the script covers on each port, `0..=252`.
pub(crate) const IDS: usize = 253;
/// Variants of [`Call`].
const KINDS: usize = 21;
/// Distinct errors scripted at once; past it, the last one is reused.
const ERRORS: usize = 16;
/// Failures left of a fault that lasts until [`Scripted::heal`].
// DECISION: a sentinel, not an `Option`: the countdown table is one byte per
// entry, so the fake, moved by value through every test, stays small.
const UNTIL_HEALED: u8 = u8::MAX;
/// Variants of [`GoalRegister`].
const REGISTERS: usize = GoalRegister::ALL.len();
/// Variants of [`RamSetting`].
const RAM: usize = 4;
/// Device path bytes kept per port; longer paths are cut.
const PATH: usize = 256;

/// Answer of a present actuator until scripted otherwise: 12.0 V and 30 °C,
/// inside every model's datasheet range, and the RX-28 factory limits,
/// 6.0–19.0 V and 80 °C.
pub(crate) const HEALTHY: Health = Health {
    error_status: 0,
    voltage_dv: 120,
    temperature_c: 30,
    min_voltage_dv: 60,
    max_voltage_dv: 190,
    temperature_limit_c: 80,
};

/// Protocol 2 status error of a write the actuator refuses: Access Error,
/// an EEPROM register written with the torque on.
pub(crate) const ACCESS_ERROR: u8 = 0x07;
/// Hardware Error Status bit of an input voltage outside the limits.
const INPUT_VOLTAGE: u8 = 0x01;
/// Hardware Error Status bit of a temperature above the limit.
const OVERHEATING: u8 = 0x04;

/// CW and CCW Angle Limit of a Protocol 1 actuator until scripted
/// otherwise: the AX and RX factory values, joint mode over the whole range.
const JOINT_MODE: (u32, u32) = (0, 1023);

/// Position limits of `model` until scripted otherwise: [`JOINT_MODE`] on
/// Protocol 1, the whole register range on Protocol 2.
const fn factory_limits(model: Model) -> (u32, u32) {
    match model.protocol() {
        Protocol::One => JOINT_MODE,
        Protocol::Two => (0, units::max_position(model)),
    }
}

/// A [`Backend`] method.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Call {
    /// [`Backend::open_ports`].
    OpenPorts,
    /// [`Backend::declare`].
    Declare,
    /// [`Backend::clear_fault`].
    ClearFault,
    /// [`Backend::read_position`].
    ReadPosition,
    /// [`Backend::sync_read_state`].
    SyncReadState,
    /// [`Backend::read_vitals`]: one record per transaction, a sync read
    /// on Protocol 2, a read per actuator on Protocol 1.
    ReadVitals,
    /// [`Backend::sync_write_goal`].
    SyncWriteGoal,
    /// [`Backend::sync_write_start_goal`].
    SyncWriteStartGoal,
    /// [`Backend::torque`].
    Torque,
    /// [`Backend::force_torque_off`].
    ForceTorqueOff,
    /// [`Backend::confirm_torque_off`].
    ConfirmTorqueOff,
    /// [`Backend::read_health`].
    ReadHealth,
    /// [`Backend::read_setting`].
    ReadSetting,
    /// [`Backend::read_angle_limits`].
    ReadAngleLimits,
    /// [`Backend::read_voltage_limits`].
    ReadVoltageLimits,
    /// [`Backend::read_temperature_limit`].
    ReadTemperatureLimit,
    /// [`Backend::write_voltage_limits`]; its entry is `(id, min << 16 |
    /// max)`, in 0.1 V.
    WriteVoltageLimits,
    /// [`Backend::write_temperature_limit`]; its entry is `(id, °C)`.
    WriteTemperatureLimit,
    /// [`Backend::write_ram`]; its entry is `(id, value)`.
    WriteRam,
    /// [`Backend::read_ram`].
    ReadRam,
    /// [`Backend::close_ports`].
    ClosePorts,
}

impl Call {
    /// Slot of the kind in the per-kind tables.
    const fn index(self) -> usize {
        self as usize
    }
}

/// One recorded call.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Record {
    /// The method called.
    pub(crate) call: Call,
    /// Bus index; 0 for `OpenPorts` and `ClosePorts`.
    pub(crate) port: usize,
    /// Model named; `None` for `OpenPorts` and `ClosePorts`.
    pub(crate) model: Option<Model>,
    /// First id named; 0 when the call names none.
    pub(crate) id: u8,
    /// Ids named; ports for `OpenPorts`.
    pub(crate) count: usize,
    /// Torque requested; `false` outside `Torque`.
    pub(crate) on: bool,
    /// First of the call's `(id, raw)` entries.
    start: usize,
    /// Entries kept, below `count` once the entry log is full.
    len: usize,
}

/// Placeholder of the unused record slots.
const BLANK: Record = Record {
    call: Call::OpenPorts,
    port: 0,
    model: None,
    id: 0,
    count: 0,
    on: false,
    start: 0,
    len: 0,
};

/// What a port was last opened with.
#[derive(Clone, Copy, Debug)]
struct Opened {
    /// Baud rate of the bus.
    baud: u32,
    /// Device path bytes, `len` of them used.
    path: [u8; PATH],
    /// Bytes used in `path`.
    len: usize,
}

/// The scripted backend. Fixed capacity: nothing allocates.
#[derive(Debug)]
pub(crate) struct Scripted {
    /// Recorded calls, `logged` of them used.
    log: [Record; CAPACITY],
    /// Records used in `log`.
    logged: usize,
    /// `(id, raw)` entries of the recorded calls, `entered` of them used.
    entries: [(u8, u32); ENTRIES],
    /// Entries used in `entries`.
    entered: usize,
    /// Calls per kind, recorded or not.
    counts: [usize; KINDS],
    /// Between a successful `open_ports` and `close_ports`.
    open: bool,
    /// Per port, what it was last opened with.
    opened: [Option<Opened>; PORTS],
    /// Per actuator, switched on through this backend: what `DxlRsV2` holds
    /// as torque on.
    powered: [[bool; IDS]; PORTS],
    /// Per actuator, held faulted: what `DxlRsV2` holds as torque faulted.
    faulted: [[bool; IDS]; PORTS],
    /// Per actuator, the Torque Enable register.
    torque: [[bool; IDS]; PORTS],
    /// Per actuator, a torque off answered since its setup, its last torque
    /// on or a raw write: what `dxl_rs_v2` needs before a limit write.
    confirmed: [[bool; IDS]; PORTS],
    /// Per actuator, the goal registers, in [`GoalRegister::ALL`] order.
    goals: [[[u32; REGISTERS]; IDS]; PORTS],
    /// Per actuator, a start goal written since its torque was last
    /// switched on or off.
    started: [[bool; IDS]; PORTS],
    /// Per actuator, what reads return.
    states: [[RawState; IDS]; PORTS],
    /// Per actuator, what health reads return.
    vitals: [[Health; IDS]; PORTS],
    /// Per actuator, what Operating Mode reads return.
    modes: [[u8; IDS]; PORTS],
    /// Per actuator, what Model Number reads return; `None` answers the
    /// model named.
    numbers: [[Option<u16>; IDS]; PORTS],
    /// Per actuator, what Torque Ctrl Mode Enable reads return.
    torque_modes: [[u8; IDS]; PORTS],
    /// Per actuator, what position limit reads return; `None` answers
    /// [`factory_limits`] of the model named.
    limits: [[Option<(u32, u32)>; IDS]; PORTS],
    /// Per actuator, whether it never answers.
    missing: [[bool; IDS]; PORTS],
    /// Per actuator, whether a limit write answers but leaves the
    /// registers as they were.
    stuck: [[bool; IDS]; PORTS],
    /// Per actuator, its RAM settings, in [`RamSetting`] order.
    ram: [[[u16; RAM]; IDS]; PORTS],
    /// Per actuator, whether a RAM write answers but leaves the register as
    /// it was.
    stuck_ram: [[bool; IDS]; PORTS],
    /// Errors the faults name, `errored` of them used: a fault holds a slot,
    /// so the per-actuator table stays small.
    errors: [BackendError; ERRORS],
    /// Slots used in `errors`.
    errored: usize,
    /// Per actuator and kind, 1 + the slot of the error its calls fail
    /// with; 0 while they answer.
    faults: [[[u8; KINDS]; IDS]; PORTS],
    /// Per actuator and kind, failures left before the fault clears;
    /// [`UNTIL_HEALED`] while it lasts until [`Scripted::heal`].
    left: [[[u8; KINDS]; IDS]; PORTS],
    /// Per port and kind, the error whole transactions fail with.
    port_faults: [[Option<BackendError>; KINDS]; PORTS],
    /// The failure `open_ports` returns.
    open_fault: Option<OpenFailure>,
}

impl Default for Scripted {
    fn default() -> Self {
        Self::new()
    }
}

impl Scripted {
    /// Every actuator present and healthy; nothing recorded.
    pub(crate) fn new() -> Self {
        Self {
            log: [BLANK; CAPACITY],
            logged: 0,
            entries: [(0, 0); ENTRIES],
            entered: 0,
            counts: [0; KINDS],
            open: false,
            opened: [None; PORTS],
            powered: [[false; IDS]; PORTS],
            faulted: [[false; IDS]; PORTS],
            torque: [[false; IDS]; PORTS],
            confirmed: [[false; IDS]; PORTS],
            goals: [[[0; REGISTERS]; IDS]; PORTS],
            started: [[false; IDS]; PORTS],
            states: [[RawState::default(); IDS]; PORTS],
            vitals: [[HEALTHY; IDS]; PORTS],
            modes: [[POSITION_MODE; IDS]; PORTS],
            numbers: [[None; IDS]; PORTS],
            torque_modes: [[0; IDS]; PORTS],
            limits: [[None; IDS]; PORTS],
            missing: [[false; IDS]; PORTS],
            stuck: [[false; IDS]; PORTS],
            ram: [[[0; RAM]; IDS]; PORTS],
            stuck_ram: [[false; IDS]; PORTS],
            errors: [BackendError::Timeout; ERRORS],
            errored: 0,
            faults: [[[0; KINDS]; IDS]; PORTS],
            left: [[[UNTIL_HEALED; KINDS]; IDS]; PORTS],
            port_faults: [[None; KINDS]; PORTS],
            open_fault: None,
        }
    }

    /// Limit writes to actuator `id` answer but change nothing, as an
    /// actuator clamping or ignoring them would.
    pub(crate) fn stuck_limits(&mut self, port: usize, id: u8) -> &mut Self {
        if let Some(stuck) = at(&mut self.stuck, port, id) {
            *stuck = true;
        }
        self
    }

    /// RAM writes to actuator `id` answer but change nothing, as an
    /// actuator ignoring them would.
    pub(crate) fn stuck_ram(&mut self, port: usize, id: u8) -> &mut Self {
        if let Some(stuck) = at(&mut self.stuck_ram, port, id) {
            *stuck = true;
        }
        self
    }

    /// RAM register `setting` of actuator `id`; 0 outside the script.
    pub(crate) fn ram(&self, port: usize, id: u8, setting: RamSetting) -> u16 {
        self.ram
            .get(port)
            .and_then(|ids| ids.get(usize::from(id)))
            .and_then(|registers| registers.get(setting as usize))
            .copied()
            .unwrap_or(0)
    }

    /// Health limits actuator `id` holds now.
    pub(crate) fn limits(&self, port: usize, id: u8) -> Option<HealthLimits> {
        let health = self.vitals.get(port)?.get(usize::from(id))?;
        Some(HealthLimits {
            min_voltage_dv: health.min_voltage_dv,
            max_voltage_dv: health.max_voltage_dv,
            temperature_c: health.temperature_limit_c,
        })
    }

    /// Actuator `id` never answers: `Timeout` from every call naming it.
    pub(crate) fn missing(&mut self, port: usize, id: u8) -> &mut Self {
        if let Some(missing) = at(&mut self.missing, port, id) {
            *missing = true;
        }
        self
    }

    /// Calls of kind `call` naming actuator `id` fail with `error`; for
    /// `SyncReadState`, its entry of `out` does, and for `Declare`, the
    /// setup of its port stops at it. No effect on `SyncWriteGoal`, which no
    /// status packet answers.
    pub(crate) fn fail(
        &mut self,
        call: Call,
        port: usize,
        id: u8,
        error: BackendError,
    ) -> &mut Self {
        let slot = self.slot(error);
        if let Some(faults) = at(&mut self.faults, port, id) {
            faults[call.index()] = slot;
        }
        if let Some(left) = at(&mut self.left, port, id) {
            left[call.index()] = UNTIL_HEALED;
        }
        self
    }

    /// The next `times` calls of kind `call` naming actuator `id`, at most
    /// 254, fail with `error`, as [`Scripted::fail`]; later ones answer.
    pub(crate) fn fail_times(
        &mut self,
        call: Call,
        port: usize,
        id: u8,
        error: BackendError,
        times: u8,
    ) -> &mut Self {
        let slot = if times > 0 { self.slot(error) } else { 0 };
        if let Some(faults) = at(&mut self.faults, port, id) {
            faults[call.index()] = slot;
        }
        if let Some(left) = at(&mut self.left, port, id) {
            left[call.index()] = times.min(UNTIL_HEALED - 1);
        }
        self
    }

    /// The ping of actuator `id` at `declare` times out; its other calls
    /// still answer.
    pub(crate) fn fail_ping(&mut self, port: usize, id: u8) -> &mut Self {
        self.fail(Call::Declare, port, id, BackendError::Timeout)
    }

    /// Every call of kind `call` on `port` fails as a whole with `error`.
    pub(crate) fn fail_port(&mut self, call: Call, port: usize, error: BackendError) -> &mut Self {
        if let Some(faults) = self.port_faults.get_mut(port) {
            faults[call.index()] = Some(error);
        }
        self
    }

    /// Group reads on `port` fail: the port is gone.
    pub(crate) fn fail_sync_read(&mut self, port: usize) -> &mut Self {
        self.fail_port(Call::SyncReadState, port, BackendError::Io(Wire::Closed))
    }

    /// Group writes on `port` fail: the port timed out.
    pub(crate) fn fail_sync_write(&mut self, port: usize) -> &mut Self {
        self.fail_port(
            Call::SyncWriteGoal,
            port,
            BackendError::Io(Wire::PortTimeout),
        )
    }

    /// `open_ports` fails with `error`, on no port in particular.
    pub(crate) fn fail_open(&mut self, error: BackendError) -> &mut Self {
        self.open_fault = Some(OpenFailure {
            port: None,
            source: error,
        });
        self
    }

    /// `open_ports` fails with `error` on port `port`.
    pub(crate) fn fail_open_port(&mut self, port: usize, error: BackendError) -> &mut Self {
        self.open_fault = Some(OpenFailure {
            port: Some(port),
            source: error,
        });
        self
    }

    /// Clears every scripted fault and missing actuator; values and the log
    /// are kept.
    pub(crate) fn heal(&mut self) -> &mut Self {
        self.missing = [[false; IDS]; PORTS];
        self.errored = 0;
        self.faults = [[[0; KINDS]; IDS]; PORTS];
        self.left = [[[UNTIL_HEALED; KINDS]; IDS]; PORTS];
        self.port_faults = [[None; KINDS]; PORTS];
        self.open_fault = None;
        self
    }

    /// Reads of actuator `id` return `state`.
    pub(crate) fn state(&mut self, port: usize, id: u8, state: RawState) -> &mut Self {
        if let Some(slot) = at(&mut self.states, port, id) {
            *slot = state;
        }
        self
    }

    /// Reads of actuator `id` return position `raw`; velocity and current
    /// are kept.
    pub(crate) fn position(&mut self, port: usize, id: u8, raw: i32) -> &mut Self {
        if let Some(slot) = at(&mut self.states, port, id) {
            slot.position = raw;
        }
        self
    }

    /// Health reads of actuator `id` return `health`.
    pub(crate) fn health(&mut self, port: usize, id: u8, health: Health) -> &mut Self {
        if let Some(slot) = at(&mut self.vitals, port, id) {
            *slot = health;
        }
        self
    }

    /// Operating Mode reads of actuator `id` return `mode`.
    pub(crate) fn mode(&mut self, port: usize, id: u8, mode: u8) -> &mut Self {
        if let Some(slot) = at(&mut self.modes, port, id) {
            *slot = mode;
        }
        self
    }

    /// Actuator `id` reports Model Number `number` at `declare`, whatever
    /// model it names: another actuator than the table declares.
    pub(crate) fn model_number(&mut self, port: usize, id: u8, number: u16) -> &mut Self {
        if let Some(slot) = at(&mut self.numbers, port, id) {
            *slot = Some(number);
        }
        self
    }

    /// Torque Ctrl Mode Enable reads of actuator `id` return `value`.
    pub(crate) fn torque_mode(&mut self, port: usize, id: u8, value: u8) -> &mut Self {
        if let Some(slot) = at(&mut self.torque_modes, port, id) {
            *slot = value;
        }
        self
    }

    /// Position limit reads of actuator `id` return `(cw, ccw)`.
    pub(crate) fn angle_limits(&mut self, port: usize, id: u8, cw: u32, ccw: u32) -> &mut Self {
        if let Some(slot) = at(&mut self.limits, port, id) {
            *slot = Some((cw, ccw));
        }
        self
    }

    /// Actuator `id` is torque-on before any `torque` call, as an earlier
    /// run can leave it: the register is set, this backend holds no handle.
    pub(crate) fn already_on(&mut self, port: usize, id: u8) -> &mut Self {
        if let Some(torque) = at(&mut self.torque, port, id) {
            *torque = true;
        }
        self
    }

    /// An earlier session left `raw` in goal register `register` of
    /// actuator `id`.
    pub(crate) fn left_goal(
        &mut self,
        port: usize,
        id: u8,
        register: GoalRegister,
        raw: u32,
    ) -> &mut Self {
        self.set_goal(port, id, register, raw);
        self
    }

    /// Goal register `register` of actuator `id`; 0 outside the script.
    pub(crate) fn goal(&self, port: usize, id: u8, register: GoalRegister) -> u32 {
        self.goals
            .get(port)
            .and_then(|ids| ids.get(usize::from(id)))
            .and_then(|registers| registers.get(register as usize))
            .copied()
            .unwrap_or(0)
    }

    /// Recorded calls, oldest first, at most [`CAPACITY`].
    pub(crate) fn calls(&self) -> &[Record] {
        &self.log[..self.logged]
    }

    /// Calls of kind `call` since the last [`Scripted::clear_log`], past
    /// [`CAPACITY`] included.
    pub(crate) fn count(&self, call: Call) -> usize {
        self.counts[call.index()]
    }

    /// The newest recorded call of kind `call`.
    pub(crate) fn last(&self, call: Call) -> Option<&Record> {
        self.calls().iter().rev().find(|record| record.call == call)
    }

    /// Ids named by `record`, in call order.
    pub(crate) fn ids(&self, record: &Record) -> impl Iterator<Item = u8> + '_ {
        self.entries(record).iter().map(|&(id, _)| id)
    }

    /// `(id, raw)` goals sent by a `SyncWriteGoal` or `SyncWriteStartGoal`
    /// record; empty for any other kind.
    pub(crate) fn goals(&self, record: &Record) -> &[(u8, u32)] {
        match record.call {
            Call::SyncWriteGoal | Call::SyncWriteStartGoal => self.entries(record),
            _ => &[],
        }
    }

    /// `(id, Model Number)` entries of a `Declare` record; empty for any
    /// other kind.
    pub(crate) fn models(&self, record: &Record) -> &[(u8, u32)] {
        match record.call {
            Call::Declare => self.entries(record),
            _ => &[],
        }
    }

    /// The register a `ReadVitals` record read on Protocol 2; `None` for any
    /// other kind.
    pub(crate) fn vital(&self, record: &Record) -> Option<Vital> {
        let raw = match record.call {
            Call::ReadVitals => self.entries(record).first()?.1,
            _ => return None,
        };
        Vital::ALL.get(usize::try_from(raw).ok()?).copied()
    }

    /// Goals of the newest recorded `SyncWriteGoal`; empty when there is none.
    pub(crate) fn last_goals(&self) -> &[(u8, u32)] {
        self.last(Call::SyncWriteGoal)
            .map_or(&[], |record| self.goals(record))
    }

    /// Whether the ports are open: `open_ports` succeeded and `close_ports`
    /// was not called since.
    pub(crate) fn is_open(&self) -> bool {
        self.open
    }

    /// Baud rate `port` was last opened at.
    pub(crate) fn baud(&self, port: usize) -> Option<u32> {
        self.opened.get(port)?.as_ref().map(|opened| opened.baud)
    }

    /// Device `port` was last opened on.
    pub(crate) fn path(&self, port: usize) -> Option<&Path> {
        let opened = self.opened.get(port)?.as_ref()?;
        let bytes = opened.path.get(..opened.len)?;
        Some(Path::new(OsStr::from_bytes(bytes)))
    }

    /// The value a `WriteVoltageLimits` or `WriteTemperatureLimit` record
    /// wrote, as its entry holds it; `None` for any other kind.
    pub(crate) fn written(&self, record: &Record) -> Option<u32> {
        match record.call {
            Call::WriteVoltageLimits | Call::WriteTemperatureLimit => {
                Some(self.entries(record).first()?.1)
            }
            _ => None,
        }
    }

    /// Whether the Torque Enable register of actuator `id` is set.
    pub(crate) fn torqued(&self, port: usize, id: u8) -> bool {
        get(&self.torque, port, id)
    }

    /// Forgets the recorded calls and the counts; the script is kept.
    pub(crate) fn clear_log(&mut self) {
        self.logged = 0;
        self.entered = 0;
        self.counts = [0; KINDS];
    }

    /// The `(id, raw)` entries kept for `record`.
    fn entries(&self, record: &Record) -> &[(u8, u32)] {
        self.entries
            .get(record.start..record.start + record.len)
            .unwrap_or(&[])
    }

    /// Counts the call, then records it while the log has room.
    fn record(
        &mut self,
        call: Call,
        port: usize,
        model: Option<Model>,
        count: usize,
        on: bool,
        entries: impl Iterator<Item = (u8, u32)>,
    ) {
        self.counts[call.index()] += 1;
        let Some(slot) = self.log.get_mut(self.logged) else {
            return;
        };
        let start = self.entered;
        let mut id = None;
        for entry in entries {
            id.get_or_insert(entry.0);
            if let Some(kept) = self.entries.get_mut(self.entered) {
                *kept = entry;
                self.entered += 1;
            }
        }
        *slot = Record {
            call,
            port,
            model,
            id: id.unwrap_or(0),
            count,
            on,
            start,
            len: self.entered - start,
        };
        self.logged += 1;
    }

    /// Whether a transaction of kind `call` on `port` goes through:
    /// `PortNotOpen` while the ports are closed.
    fn transaction(&self, call: Call, port: usize) -> Result<(), BackendError> {
        match self.port_faults.get(port) {
            Some(faults) if self.open => faults[call.index()].map_or(Ok(()), Err),
            _ => Err(BackendError::PortNotOpen { port }),
        }
    }

    /// Whether actuator `id` answers a call of kind `call`; a failure counts
    /// against a counted fault.
    fn answer(&mut self, call: Call, port: usize, id: u8) -> Result<(), BackendError> {
        self.transaction(call, port)?;
        let (id, kind) = (usize::from(id), call.index());
        if id >= IDS || self.missing[port][id] {
            return Err(BackendError::Timeout);
        }
        let Some(&error) = usize::from(self.faults[port][id][kind])
            .checked_sub(1)
            .and_then(|slot| self.errors.get(slot))
        else {
            return Ok(());
        };
        let left = &mut self.left[port][id][kind];
        if *left != UNTIL_HEALED {
            *left = left.saturating_sub(1);
            if *left == 0 {
                self.faults[port][id][kind] = 0;
            }
        }
        Err(error)
    }

    /// 1 + the slot of `error` in `errors`, taken when it is new; the last
    /// slot once they are all taken.
    fn slot(&mut self, error: BackendError) -> u8 {
        let used = self.errors.get(..self.errored).unwrap_or_default();
        let index = match used.iter().position(|&known| known == error) {
            Some(index) => index,
            None if self.errored < ERRORS => {
                self.errored += 1;
                self.errored - 1
            }
            None => ERRORS - 1,
        };
        if let Some(slot) = self.errors.get_mut(index) {
            *slot = error;
        }
        u8::try_from(index + 1).unwrap_or(u8::MAX)
    }

    /// Records a call naming one actuator.
    fn note(&mut self, call: Call, port: usize, model: Model, id: u8, on: bool) {
        let entry = (id, u32::from(on));
        self.record(call, port, Some(model), 1, on, iter::once(entry));
    }

    /// Records a call naming one actuator, then answers it.
    fn one(
        &mut self,
        call: Call,
        port: usize,
        model: Model,
        id: u8,
        on: bool,
    ) -> Result<(), BackendError> {
        self.note(call, port, model, id, on);
        self.answer(call, port, id)
    }

    /// `NotPowered` unless actuator `id` was switched on through this
    /// backend, as `DxlRsV2` holds no other as torque on.
    fn held(&self, port: usize, id: u8) -> Result<(), BackendError> {
        if get(&self.powered, port, id) {
            Ok(())
        } else {
            Err(BackendError::NotPowered { id })
        }
    }

    /// The fault actuator `id` reports, measured from its health script as
    /// `dxl_rs_v2` measures it: on Protocol 1 a voltage outside its limits,
    /// then a temperature above its limit, then any error status scripted;
    /// on Protocol 2 only a non-zero Hardware Error Status, which it keeps
    /// until a reboot, voltage first then temperature where its bit is set.
    fn measured(&self, port: usize, model: Model, id: u8) -> Option<Fault> {
        let health = *self.vitals.get(port)?.get(usize::from(id))?;
        let low = !(health.min_voltage_dv..=health.max_voltage_dv).contains(&health.voltage_dv);
        let hot = health.temperature_c > health.temperature_limit_c;
        let status = health.error_status;
        let voltage = Fault::Voltage {
            measured_dv: health.voltage_dv,
            min_dv: health.min_voltage_dv,
            max_dv: health.max_voltage_dv,
        };
        let temperature = Fault::Temperature {
            measured_c: health.temperature_c,
            limit_c: health.temperature_limit_c,
        };
        let hardware = Fault::Hardware { status };
        match model.protocol() {
            Protocol::One if low => Some(voltage),
            Protocol::One if hot => Some(temperature),
            _ if status == 0 => None,
            Protocol::Two if status & INPUT_VOLTAGE != 0 => {
                Some(if low { voltage } else { hardware })
            }
            Protocol::Two if status & OVERHEATING != 0 => {
                Some(if hot { temperature } else { hardware })
            }
            _ => Some(hardware),
        }
    }

    /// The fault measured when actuator `id` is held faulted and its script
    /// still reports one: `dxl_rs_v2` returns no value beside it.
    fn unflagged(&self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        match self.measured(port, model, id) {
            Some(fault) if get(&self.faulted, port, id) => Err(BackendError::Fault(fault)),
            _ => Ok(()),
        }
    }

    /// Records a limit write of `value` to actuator `id`, then refuses it
    /// as `DxlRsV2` does before sending: powered here, its torque off not
    /// confirmed, a model whose limits [`limits_have_units`] says no to, or
    /// `refused` (a value its register does not take).
    fn limit_write(
        &mut self,
        call: Call,
        (port, model, id): (usize, Model, u8),
        value: u32,
        refused: Option<&'static str>,
    ) -> Result<(), BackendError> {
        self.record(call, port, Some(model), 1, false, iter::once((id, value)));
        self.transaction(call, port)?;
        if get(&self.powered, port, id) {
            return Err(BackendError::TorqueOn { id });
        }
        if !get(&self.confirmed, port, id) {
            return Err(BackendError::TorqueUnconfirmed { id });
        }
        if !limits_have_units(model) {
            return Err(BackendError::Refused {
                reason: "a limit dxl_rs_v2 has no unit for on this model",
            });
        }
        if let Some(reason) = refused {
            return Err(BackendError::Refused { reason });
        }
        self.answer(call, port, id)
    }

    /// The status a limit write to actuator `id` is refused with by the
    /// actuator: Access Error with its torque on on Protocol 2.
    fn limit_refusal(&self, port: usize, model: Model, id: u8) -> Option<u8> {
        (model.protocol() == Protocol::Two && get(&self.torque, port, id)).then_some(ACCESS_ERROR)
    }

    /// Marks the torque of actuator `id` confirmed off, or not.
    fn confirm(&mut self, port: usize, id: u8, off: bool) {
        if let Some(confirmed) = at(&mut self.confirmed, port, id) {
            *confirmed = off;
        }
    }

    /// Answers a limit write to actuator `id`: `apply`d to its script unless
    /// refused or its limits are stuck, then held faulted while its script
    /// reports a fault, as `dxl_rs_v2` holds one whose reply reports it.
    /// `refusal` fails it; a reply reporting a fault fails with that fault
    /// instead, applied or not.
    // DECISION: a refusal whose reply carries a fault is not applied and
    // fails with the fault: `DxlRsV2` measures that reply into a fault
    // whatever error code is beside it; the caller's read back decides.
    fn answer_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        refusal: Option<u8>,
        apply: impl FnOnce(&mut Health),
    ) -> Result<(), BackendError> {
        if refusal.is_none()
            && !get(&self.stuck, port, id)
            && let Some(health) = at(&mut self.vitals, port, id)
        {
            apply(health);
        }
        let fault = self.measured(port, model, id);
        if let (Some(_), Some(faulted)) = (fault, at(&mut self.faulted, port, id)) {
            *faulted = true;
        }
        match (fault, refusal) {
            (Some(fault), _) => Err(BackendError::Fault(fault)),
            (None, Some(status)) => Err(BackendError::Status(status)),
            (None, None) => Ok(()),
        }
    }

    /// `NoStartGoal` unless a start goal was written to actuator `id` since
    /// its torque was last switched: torque on would drive it to whatever
    /// goal an earlier session left.
    fn start_written(&self, port: usize, id: u8) -> Result<(), BackendError> {
        if get(&self.started, port, id) {
            Ok(())
        } else {
            Err(BackendError::NoStartGoal { id })
        }
    }

    /// Sets both the handle and the register of actuator `id`; a start goal
    /// written before is used up.
    fn set_torque(&mut self, port: usize, id: u8, on: bool) {
        if let Some(powered) = at(&mut self.powered, port, id) {
            *powered = on;
        }
        if let Some(torque) = at(&mut self.torque, port, id) {
            *torque = on;
        }
        if let Some(started) = at(&mut self.started, port, id) {
            *started = false;
        }
    }

    /// Writes `raw` to goal register `register` of actuator `id`.
    fn set_goal(&mut self, port: usize, id: u8, register: GoalRegister, raw: u32) {
        if let Some(slot) =
            at(&mut self.goals, port, id).and_then(|registers| registers.get_mut(register as usize))
        {
            *slot = raw;
        }
    }
}

/// Whether `dxl_rs_v2`'s table for `model` gives its limit registers a
/// unit, so its typed setters take them: at e53b249 not the XM430-W210 and
/// XW540s (`DxlRsV2` does not declare the XW430-T200 at all).
const fn limits_have_units(model: Model) -> bool {
    !matches!(
        model,
        Model::Xm430W210 | Model::Xw540T140 | Model::Xw540T260
    )
}

/// Entry of `(port, id)` in a per-actuator table; `None` outside the script.
fn at<T>(table: &mut [[T; IDS]; PORTS], port: usize, id: u8) -> Option<&mut T> {
    table.get_mut(port)?.get_mut(usize::from(id))
}

/// Flag of `(port, id)` in a per-actuator table; `false` outside the script.
fn get(table: &[[bool; IDS]; PORTS], port: usize, id: u8) -> bool {
    table
        .get(port)
        .and_then(|ids| ids.get(usize::from(id)))
        .copied()
        .unwrap_or(false)
}

/// What `DxlRsV2` reads of `vital` from an actuator in state `health`: the
/// one register on Protocol 2, voltage, temperature and error status on
/// Protocol 1.
fn vitals_of(model: Model, vital: Vital, health: Health) -> Reading {
    match (model.protocol(), vital) {
        (Protocol::One, _) => Reading {
            voltage_dv: Some(health.voltage_dv),
            temperature_c: Some(health.temperature_c),
            hardware_error: Some(health.error_status),
        },
        (Protocol::Two, Vital::Voltage) => Reading::of(vital, i32::from(health.voltage_dv)),
        (Protocol::Two, Vital::Temperature) => Reading::of(vital, i32::from(health.temperature_c)),
        (Protocol::Two, Vital::HardwareError) => Reading::of(vital, i32::from(health.error_status)),
    }
}

impl Backend for Scripted {
    const MAX_PORTS: usize = PORTS;
    const STATE_READS: &'static [u16] = super::backend::STATE_READS;

    fn open_ports(&mut self, ports: &[PortSpec<'_>]) -> Result<(), OpenFailure> {
        self.record(Call::OpenPorts, 0, None, ports.len(), false, iter::empty());
        if let Some(failure) = self.open_fault {
            return Err(failure);
        }
        if ports.len() > PORTS {
            return Err(OpenFailure {
                port: None,
                source: BackendError::TooManyPorts {
                    ports: ports.len(),
                    max: PORTS,
                },
            });
        }
        self.opened = [None; PORTS];
        for (opened, spec) in self.opened.iter_mut().zip(ports) {
            let bytes = spec.path.as_os_str().as_bytes();
            let len = bytes.len().min(PATH);
            let mut path = [0; PATH];
            path[..len].copy_from_slice(&bytes[..len]);
            *opened = Some(Opened {
                baud: spec.baud,
                path,
                len,
            });
        }
        self.open = true;
        Ok(())
    }

    /// One record naming every actuator, `(id, Model Number)`, in the order
    /// given; answered in id order, as `dxl_rs_v2` sets them up. Each held
    /// is faulted when its script reports a fault ([`Scripted::measured`]);
    /// a failed setup holds none.
    fn declare(&mut self, port: usize, actuators: &[(u8, Model)]) -> Result<(), Undeclared> {
        let model = actuators.first().map(|&(_, model)| model);
        let entries = actuators
            .iter()
            .map(|&(id, model)| (id, u32::from(model.number())));
        self.record(Call::Declare, port, model, actuators.len(), false, entries);
        self.transaction(Call::Declare, port)
            .map_err(|source| Undeclared { id: None, source })?;
        if let Some(faulted) = self.faulted.get_mut(port) {
            *faulted = [false; IDS];
        }
        if let Some(confirmed) = self.confirmed.get_mut(port) {
            *confirmed = [false; IDS];
        }
        let ids = (0..=u8::MAX).filter_map(|id| actuators.iter().find(|entry| entry.0 == id));
        for &(id, model) in ids {
            let undeclared = |source| Undeclared {
                id: Some(id),
                source,
            };
            self.answer(Call::Declare, port, id).map_err(undeclared)?;
            match self.numbers[port].get(usize::from(id)).copied().flatten() {
                Some(found) if found != model.number() => {
                    return Err(undeclared(BackendError::WrongModel { found }));
                }
                _ => {}
            }
        }
        for &(id, model) in actuators {
            let held = self.measured(port, model, id).is_some();
            if let Some(faulted) = at(&mut self.faulted, port, id) {
                *faulted = held;
            }
        }
        Ok(())
    }

    fn faulted(&self, port: usize, _model: Model, id: u8) -> bool {
        get(&self.faulted, port, id)
    }

    /// Clears unless the script still reports a fault, which keeps the
    /// actuator faulted, held so before or not, as `dxl_rs_v2`'s answer does.
    fn clear_fault(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        self.one(Call::ClearFault, port, model, id, false)?;
        let still = self.measured(port, model, id);
        if let Some(faulted) = at(&mut self.faulted, port, id) {
            *faulted = still.is_some();
        }
        still.map_or(Ok(()), |fault| Err(BackendError::Fault(fault)))
    }

    fn read_position(&mut self, port: usize, model: Model, id: u8) -> Result<u32, BackendError> {
        self.one(Call::ReadPosition, port, model, id, false)?;
        Ok(self.states[port][usize::from(id)].position.cast_unsigned())
    }

    /// Logs one record per sync read `DxlRsV2` sends: one per entry of
    /// `STATE_READS`.
    fn sync_read_state(
        &mut self,
        port: usize,
        model: Model,
        ids: &[u8],
        out: &mut [Result<RawState, BackendError>],
    ) -> Result<(), BackendError> {
        for _ in Self::STATE_READS {
            self.record(
                Call::SyncReadState,
                port,
                Some(model),
                ids.len(),
                false,
                ids.iter().map(|&id| (id, 0)),
            );
        }
        if model.protocol() != Protocol::Two {
            return Err(BackendError::NoSyncRead);
        }
        if out.len() != ids.len() {
            return Err(BackendError::SlotMismatch {
                ids: ids.len(),
                slots: out.len(),
            });
        }
        self.transaction(Call::SyncReadState, port)?;
        for (entry, &id) in out.iter_mut().zip(ids) {
            *entry = self
                .answer(Call::SyncReadState, port, id)
                .map(|()| self.states[port][usize::from(id)]);
        }
        Ok(())
    }

    /// Answers from the health script: its voltage, temperature and error
    /// status. Logs one record per transaction `DxlRsV2` sends, each naming
    /// its ids with `vital` as their entry.
    fn read_vitals(
        &mut self,
        port: usize,
        model: Model,
        vital: Vital,
        ids: &[u8],
        out: &mut [Result<Reading, BackendError>],
    ) -> Result<(), BackendError> {
        let entry = |&id: &u8| (id, vital as u32);
        if model.protocol() == Protocol::Two {
            self.record(
                Call::ReadVitals,
                port,
                Some(model),
                ids.len(),
                false,
                ids.iter().map(entry),
            );
        }
        if out.len() != ids.len() {
            return Err(BackendError::SlotMismatch {
                ids: ids.len(),
                slots: out.len(),
            });
        }
        for (slot, id) in out.iter_mut().zip(ids) {
            if model.protocol() == Protocol::One {
                self.record(
                    Call::ReadVitals,
                    port,
                    Some(model),
                    1,
                    false,
                    iter::once(entry(id)),
                );
            }
            self.transaction(Call::ReadVitals, port)?;
            *slot = self.answer(Call::ReadVitals, port, *id).map(|()| {
                let health = self.vitals[port][usize::from(*id)];
                vitals_of(model, vital, health)
            });
        }
        Ok(())
    }

    fn sync_write_goal(
        &mut self,
        port: usize,
        model: Model,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError> {
        self.record(
            Call::SyncWriteGoal,
            port,
            Some(model),
            goals.len(),
            false,
            goals.iter().copied(),
        );
        for &(id, _) in goals {
            self.held(port, id)?;
        }
        // No status packet answers a sync write: only the port can fail it.
        self.transaction(Call::SyncWriteGoal, port)?;
        for &(id, raw) in goals {
            self.set_goal(port, id, GoalRegister::Position, raw);
        }
        Ok(())
    }

    /// Refused whole when it names an actuator switched on here: written
    /// then, it is no start goal. Every register is taken, as `DxlRsV2` will
    /// once velocity and current modes land (B3): the tests pin that.
    fn sync_write_start_goal(
        &mut self,
        port: usize,
        model: Model,
        register: GoalRegister,
        goals: &[(u8, u32)],
    ) -> Result<(), BackendError> {
        self.record(
            Call::SyncWriteStartGoal,
            port,
            Some(model),
            goals.len(),
            false,
            goals.iter().copied(),
        );
        if let Some(&(id, _)) = goals.iter().find(|&&(id, _)| get(&self.powered, port, id)) {
            return Err(BackendError::AlreadyPowered { id });
        }
        self.transaction(Call::SyncWriteStartGoal, port)?;
        for &(id, raw) in goals {
            self.set_goal(port, id, register, raw);
            self.confirm(port, id, false);
            if let Some(started) = at(&mut self.started, port, id) {
                *started = true;
            }
        }
        Ok(())
    }

    fn torque(&mut self, port: usize, model: Model, id: u8, on: bool) -> Result<(), BackendError> {
        self.note(Call::Torque, port, model, id, on);
        if on {
            self.start_written(port, id)?;
            if get(&self.faulted, port, id) {
                return Err(BackendError::Faulted { id });
            }
        } else {
            self.held(port, id)?;
        }
        self.answer(Call::Torque, port, id)?;
        self.set_torque(port, id, on);
        self.confirm(port, id, !on);
        Ok(())
    }

    /// Written in every state; the reply of a faulted actuator carries its
    /// fault, as `dxl_rs_v2` returns it.
    // DECISION: a write that fails past the port leaves the handle off and
    // the torque unconfirmed, the register as it was, as `dxl_rs_v2` does.
    fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        if let Err(error) = self.one(Call::ForceTorqueOff, port, model, id, false) {
            if !error.is_bus()
                && let Some(powered) = at(&mut self.powered, port, id)
            {
                *powered = false;
                self.confirm(port, id, false);
            }
            return Err(error);
        }
        self.set_torque(port, id, false);
        self.confirm(port, id, true);
        self.unflagged(port, model, id)
    }

    /// Written in every state; the fault a faulted actuator answers with
    /// confirms it too, as `DxlRsV2` takes it.
    fn confirm_torque_off(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<(), BackendError> {
        self.one(Call::ConfirmTorqueOff, port, model, id, false)?;
        self.set_torque(port, id, false);
        self.confirm(port, id, true);
        Ok(())
    }

    /// Protocol 1 answers too, error status included: scripted, not read.
    fn read_health(&mut self, port: usize, model: Model, id: u8) -> Result<Health, BackendError> {
        self.one(Call::ReadHealth, port, model, id, false)?;
        self.unflagged(port, model, id)?;
        Ok(self.vitals[port][usize::from(id)])
    }

    fn read_setting(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        setting: Setting,
    ) -> Result<u8, BackendError> {
        self.note(Call::ReadSetting, port, model, id, false);
        match setting {
            Setting::TorqueCtrlModeEnable if !model.has_torque_ctrl_mode() => {
                return Err(BackendError::NoRegister {
                    register: setting.name(),
                });
            }
            Setting::OperatingMode if model.protocol() != Protocol::Two => {
                return Err(BackendError::NoSetting { setting });
            }
            _ => {}
        }
        self.answer(Call::ReadSetting, port, id)?;
        self.unflagged(port, model, id)?;
        let id = usize::from(id);
        Ok(match setting {
            Setting::OperatingMode => self.modes[port][id],
            Setting::TorqueEnable => u8::from(self.torque[port][id]),
            Setting::TorqueCtrlModeEnable => self.torque_modes[port][id],
        })
    }

    fn read_angle_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<(u32, u32), BackendError> {
        self.note(Call::ReadAngleLimits, port, model, id, false);
        self.answer(Call::ReadAngleLimits, port, id)?;
        Ok(self.limits[port][usize::from(id)].unwrap_or(factory_limits(model)))
    }

    /// Answered from the health script; a faulted actuator fails with its
    /// fault, as `dxl_rs_v2` returns no value beside it.
    fn read_voltage_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<(u16, u16), BackendError> {
        self.one(Call::ReadVoltageLimits, port, model, id, false)?;
        self.unflagged(port, model, id)?;
        let health = self.vitals[port][usize::from(id)];
        Ok((health.min_voltage_dv, health.max_voltage_dv))
    }

    /// Answered as `read_voltage_limits` is.
    fn read_temperature_limit(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
    ) -> Result<u8, BackendError> {
        self.one(Call::ReadTemperatureLimit, port, model, id, false)?;
        self.unflagged(port, model, id)?;
        Ok(self.vitals[port][usize::from(id)].temperature_limit_c)
    }

    /// Refused as [`Scripted::limit_write`] says, a voltage wider than its
    /// one-byte Protocol 1 register or a min above the max included; then as
    /// the actuator refuses it, and answered as [`Scripted::answer_limits`]
    /// says.
    fn write_voltage_limits(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        min_dv: u16,
        max_dv: u16,
    ) -> Result<(), BackendError> {
        let refused = if !voltage_limits_fit(model, min_dv, max_dv) {
            Some("a voltage limit wider than its one-byte register")
        } else if min_dv > max_dv {
            Some("limits with the min above the max")
        } else {
            None
        };
        let value = u32::from(min_dv) << 16 | u32::from(max_dv);
        let call = Call::WriteVoltageLimits;
        self.limit_write(call, (port, model, id), value, refused)?;
        let refusal = self.limit_refusal(port, model, id);
        self.answer_limits(port, model, id, refusal, |health| {
            health.min_voltage_dv = min_dv;
            health.max_voltage_dv = max_dv;
        })
    }

    /// Refused and answered as `write_voltage_limits` is.
    fn write_temperature_limit(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        celsius: u8,
    ) -> Result<(), BackendError> {
        let call = Call::WriteTemperatureLimit;
        self.limit_write(call, (port, model, id), u32::from(celsius), None)?;
        let refusal = self.limit_refusal(port, model, id);
        self.answer_limits(port, model, id, refusal, |health| {
            health.temperature_limit_c = celsius;
        })
    }

    /// Refused as `DxlRsV2` refuses it: a register the model lacks, before
    /// sending.
    fn write_ram(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        setting: RamSetting,
        value: u16,
    ) -> Result<(), BackendError> {
        let entry = (id, u32::from(value));
        self.record(
            Call::WriteRam,
            port,
            Some(model),
            1,
            false,
            iter::once(entry),
        );
        if !setting.exists_on(model) {
            return Err(BackendError::NoRegister {
                register: setting.name(),
            });
        }
        self.answer(Call::WriteRam, port, id)?;
        if setting == RamSetting::IGain {
            self.confirm(port, id, false);
        }
        if get(&self.stuck_ram, port, id) {
            return Ok(());
        }
        if let Some(slot) =
            at(&mut self.ram, port, id).and_then(|registers| registers.get_mut(setting as usize))
        {
            *slot = value;
        }
        Ok(())
    }

    fn read_ram(
        &mut self,
        port: usize,
        model: Model,
        id: u8,
        setting: RamSetting,
    ) -> Result<u16, BackendError> {
        self.note(Call::ReadRam, port, model, id, false);
        if !setting.exists_on(model) {
            return Err(BackendError::NoRegister {
                register: setting.name(),
            });
        }
        self.answer(Call::ReadRam, port, id)?;
        Ok(self.ram(port, id, setting))
    }

    fn close_ports(&mut self) {
        self.record(Call::ClosePorts, 0, None, 0, false, iter::empty());
        self.open = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODEL: Model = Model::Xm430W210;
    /// A failed transaction, as scripted faults inject it.
    const IO: BackendError = BackendError::Io(Wire::Closed);

    /// Declares actuator `id` of `model` alone on `port`: its ping, as the
    /// fake answers it.
    fn ping(backend: &mut Scripted, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        backend
            .declare(port, &[(id, model)])
            .map_err(|failure| failure.source)
    }

    /// Start goal 2048 for actuator `id` on port 0.
    fn start(backend: &mut Scripted, id: u8) -> Result<(), BackendError> {
        backend.sync_write_start_goal(0, MODEL, GoalRegister::Position, &[(id, 2048)])
    }

    #[test]
    fn records_calls_in_order_with_their_ids_and_goals() {
        let mut backend = Scripted::new();
        let ports = [PortSpec {
            path: Path::new("/dev/ttyUSB0"),
            baud: 1_000_000,
        }];

        backend.open_ports(&ports).unwrap();
        ping(&mut backend, 0, MODEL, 1).unwrap();
        backend
            .sync_write_start_goal(0, MODEL, GoalRegister::Position, &[(1, 100), (3, 300)])
            .unwrap();
        backend.torque(0, MODEL, 1, true).unwrap();
        backend.torque(0, MODEL, 3, true).unwrap();
        backend
            .sync_write_goal(0, MODEL, &[(1, 2048), (3, 1024)])
            .unwrap();
        backend.torque(0, MODEL, 1, false).unwrap();
        backend.close_ports();

        let kinds = backend.calls().iter().map(|record| record.call);
        assert!(kinds.eq([
            Call::OpenPorts,
            Call::Declare,
            Call::SyncWriteStartGoal,
            Call::Torque,
            Call::Torque,
            Call::SyncWriteGoal,
            Call::Torque,
            Call::ClosePorts,
        ]));
        let write = backend.last(Call::SyncWriteGoal).unwrap();
        assert_eq!(
            (write.port, write.model, write.id, write.count),
            (0, Some(MODEL), 1, 2)
        );
        assert!(backend.ids(write).eq([1, 3]));
        assert_eq!(backend.last_goals(), &[(1, 2048), (3, 1024)]);
        let start = backend.last(Call::SyncWriteStartGoal).unwrap();
        assert_eq!(backend.goals(start), &[(1, 100), (3, 300)]);
        assert_eq!(backend.goal(0, 3, GoalRegister::Position), 1024);
        assert!(
            backend
                .goals(backend.last(Call::Declare).unwrap())
                .is_empty()
        );
        assert!(!backend.last(Call::Torque).unwrap().on);
        assert!(backend.torqued(0, 3) && !backend.torqued(0, 1));
        assert_eq!(backend.baud(0), Some(1_000_000));
        assert_eq!(backend.path(0), Some(Path::new("/dev/ttyUSB0")));
        assert_eq!(backend.path(1), None);
        assert!(!backend.is_open());
    }

    #[test]
    fn missing_actuator_times_out_everywhere() {
        let state = RawState {
            position: 2048,
            velocity: -3,
            current: 7,
        };
        let mut backend = Scripted::default();
        backend
            .missing(0, 2)
            .state(0, 1, state)
            .position(0, 1, 2100);
        backend.open_ports(&[]).unwrap();

        assert_eq!(ping(&mut backend, 0, MODEL, 2), Err(BackendError::Timeout));
        assert_eq!(
            backend.read_position(0, MODEL, 2),
            Err(BackendError::Timeout)
        );
        assert_eq!(start(&mut backend, 2), Ok(()));
        assert_eq!(
            backend.torque(0, MODEL, 2, true),
            Err(BackendError::Timeout)
        );
        assert_eq!(
            backend.force_torque_off(0, MODEL, 2),
            Err(BackendError::Timeout)
        );
        assert_eq!(backend.read_health(0, MODEL, 2), Err(BackendError::Timeout));
        let mut out = [Ok(RawState::default()); 2];
        backend
            .sync_read_state(0, MODEL, &[1, 2], &mut out)
            .unwrap();
        assert_eq!(
            out,
            [
                Ok(RawState {
                    position: 2100,
                    ..state
                }),
                Err(BackendError::Timeout)
            ]
        );
        assert_eq!(backend.read_health(0, MODEL, 1), Ok(HEALTHY));
    }

    #[test]
    fn scripted_faults_fail_their_call_until_healed() {
        let hot = Health {
            temperature_c: 80,
            ..HEALTHY
        };
        let mut backend = Scripted::new();
        backend.fail_open(IO);
        assert_eq!(
            backend.open_ports(&[]),
            Err(OpenFailure {
                port: None,
                source: IO
            })
        );
        assert!(!backend.is_open());

        backend
            .heal()
            .fail_sync_write(0)
            .fail_sync_read(1)
            .fail_ping(0, 4)
            .fail(Call::ReadHealth, 0, 4, BackendError::Status(0x20))
            .health(0, 5, hot);
        assert_eq!(backend.open_ports(&[]), Ok(()));
        assert_eq!(start(&mut backend, 4), Ok(()));
        assert_eq!(backend.torque(0, MODEL, 4, true), Ok(()));
        assert_eq!(
            backend.sync_write_goal(0, MODEL, &[(4, 0)]),
            Err(BackendError::Io(Wire::PortTimeout))
        );
        let mut out = [Ok(RawState::default())];
        assert_eq!(
            backend.sync_read_state(1, MODEL, &[4], &mut out),
            Err(BackendError::Io(Wire::Closed))
        );
        assert_eq!(ping(&mut backend, 0, MODEL, 4), Err(BackendError::Timeout));
        assert_eq!(
            backend.read_health(0, MODEL, 4),
            Err(BackendError::Status(0x20))
        );
        assert_eq!(backend.read_position(0, MODEL, 4), Ok(0));
        assert_eq!(backend.read_health(0, MODEL, 5), Ok(hot));
        assert_eq!(
            ping(&mut backend, PORTS, MODEL, 4),
            Err(BackendError::PortNotOpen { port: PORTS })
        );

        backend.heal();
        assert!(backend.is_open());
        assert_eq!(backend.sync_write_goal(0, MODEL, &[(4, 0)]), Ok(()));
        assert_eq!(ping(&mut backend, 0, MODEL, 4), Ok(()));
        assert_eq!(backend.count(Call::SyncWriteGoal), 2);
    }

    #[test]
    fn a_counted_fault_fails_its_calls_then_answers() {
        let mut backend = Scripted::new();
        backend
            .already_on(0, 1)
            .fail_times(Call::ForceTorqueOff, 0, 1, BackendError::Timeout, 2);
        backend.open_ports(&[]).unwrap();

        assert_eq!(
            backend.force_torque_off(0, MODEL, 1),
            Err(BackendError::Timeout)
        );
        assert_eq!(ping(&mut backend, 0, MODEL, 1), Ok(()));
        assert_eq!(
            backend.force_torque_off(0, MODEL, 1),
            Err(BackendError::Timeout)
        );
        assert!(backend.torqued(0, 1));
        assert_eq!(backend.force_torque_off(0, MODEL, 1), Ok(()));
        assert_eq!(backend.force_torque_off(0, MODEL, 1), Ok(()));
        assert!(!backend.torqued(0, 1));
        assert_eq!(backend.count(Call::ForceTorqueOff), 4);

        backend
            .fail_times(Call::Declare, 0, 1, IO, 1)
            .fail(Call::Declare, 0, 1, IO);
        for _ in 0..3 {
            assert_eq!(ping(&mut backend, 0, MODEL, 1), Err(IO));
        }
        backend.fail_times(Call::Declare, 0, 1, IO, 0);
        assert_eq!(ping(&mut backend, 0, MODEL, 1), Ok(()));
    }

    #[test]
    fn log_is_bounded_but_counts_are_exact() {
        let mut backend = Scripted::new();
        backend.open_ports(&[]).unwrap();
        backend.clear_log();
        for _ in 0..CAPACITY + 10 {
            ping(&mut backend, 0, MODEL, 1).unwrap();
        }
        assert_eq!(backend.calls().len(), CAPACITY);
        assert_eq!(backend.count(Call::Declare), CAPACITY + 10);

        backend.clear_log();
        assert!(backend.calls().is_empty());
        assert_eq!(backend.count(Call::Declare), 0);
        assert_eq!(backend.last(Call::Declare), None);
    }

    #[test]
    fn the_fake_refuses_a_plain_torque_off_it_never_powered() {
        let mut backend = Scripted::new();
        backend.already_on(0, 1);
        backend.open_ports(&[]).unwrap();

        assert_eq!(
            backend.torque(0, MODEL, 1, false),
            Err(BackendError::NotPowered { id: 1 })
        );
        assert_eq!(
            backend.torque(0, MODEL, 2, false),
            Err(BackendError::NotPowered { id: 2 })
        );
        assert!(backend.torqued(0, 1));

        assert_eq!(backend.force_torque_off(0, MODEL, 1), Ok(()));
        assert_eq!(backend.force_torque_off(0, MODEL, 2), Ok(()));
        assert!(!backend.torqued(0, 1) && !backend.torqued(0, 2));
        assert_eq!(backend.count(Call::ForceTorqueOff), 2);

        assert_eq!(start(&mut backend, 1), Ok(()));
        assert_eq!(backend.torque(0, MODEL, 1, true), Ok(()));
        assert_eq!(backend.force_torque_off(0, MODEL, 1), Ok(()));
        assert_eq!(
            backend.torque(0, MODEL, 1, false),
            Err(BackendError::NotPowered { id: 1 })
        );
    }

    #[test]
    fn the_fake_refuses_a_sync_write_naming_one_it_never_powered() {
        let mut backend = Scripted::new();
        backend.already_on(0, 1);
        backend.open_ports(&[]).unwrap();
        start(&mut backend, 2).unwrap();
        backend.torque(0, MODEL, 2, true).unwrap();

        assert_eq!(
            backend.sync_write_goal(0, MODEL, &[(2, 2048), (1, 2048)]),
            Err(BackendError::NotPowered { id: 1 })
        );
        assert_eq!(backend.sync_write_goal(0, MODEL, &[(2, 2048)]), Ok(()));
        assert_eq!(backend.count(Call::SyncWriteGoal), 2);
    }

    #[test]
    fn the_fake_refuses_torque_on_without_a_start_goal_since_the_last_switch() {
        let mut backend = Scripted::new();
        backend.left_goal(0, 1, GoalRegister::Position, 4000);
        backend.open_ports(&[]).unwrap();

        let refused = Err(BackendError::NoStartGoal { id: 1 });
        assert_eq!(backend.torque(0, MODEL, 1, true), refused);
        assert!(!backend.torqued(0, 1));

        assert_eq!(start(&mut backend, 1), Ok(()));
        assert_eq!(backend.goal(0, 1, GoalRegister::Position), 2048);
        assert_eq!(backend.torque(0, MODEL, 1, true), Ok(()));
        assert!(backend.torqued(0, 1));

        assert_eq!(backend.force_torque_off(0, MODEL, 1), Ok(()));
        assert_eq!(backend.torque(0, MODEL, 1, true), refused);
        assert!(!backend.torqued(0, 1));
    }

    #[test]
    fn the_fake_refuses_a_start_goal_naming_one_powered_here() {
        let mut backend = Scripted::new();
        backend
            .left_goal(0, 1, GoalRegister::Position, 100)
            .already_on(0, 1);
        backend.open_ports(&[]).unwrap();
        start(&mut backend, 2).unwrap();
        backend.torque(0, MODEL, 2, true).unwrap();

        assert_eq!(
            backend.sync_write_start_goal(0, MODEL, GoalRegister::Position, &[(1, 7), (2, 7)]),
            Err(BackendError::AlreadyPowered { id: 2 })
        );
        assert_eq!(backend.goal(0, 1, GoalRegister::Position), 100);
        assert_eq!(start(&mut backend, 1), Ok(()));
        assert_eq!(backend.count(Call::SyncWriteStartGoal), 3);
    }

    #[test]
    fn the_fake_answers_health_and_the_position_limits_of_either_protocol() {
        let mut backend = Scripted::new();
        backend
            .angle_limits(0, 2, 0, 0)
            .angle_limits(0, 3, 1000, 3000);
        backend.open_ports(&[]).unwrap();

        assert_eq!(backend.read_health(0, Model::Rx28, 1), Ok(HEALTHY));
        assert_eq!(backend.read_health(0, MODEL, 1), Ok(HEALTHY));
        assert_eq!(backend.count(Call::ReadHealth), 2);
        assert_eq!(backend.read_angle_limits(0, Model::Rx28, 1), Ok(JOINT_MODE));
        assert_eq!(backend.read_angle_limits(0, Model::Ax12a, 2), Ok((0, 0)));
        assert_eq!(backend.read_angle_limits(0, MODEL, 1), Ok((0, 4095)));
        assert_eq!(backend.read_angle_limits(0, MODEL, 3), Ok((1000, 3000)));
        assert_eq!(backend.count(Call::ReadAngleLimits), 4);
    }

    #[test]
    fn the_fake_refuses_protocol_one_settings_and_group_reads() {
        let mut backend = Scripted::new();
        backend.open_ports(&[]).unwrap();
        let mut out = [Ok(RawState::default()); 2];

        assert_eq!(
            backend.read_setting(0, Model::Rx28, 1, Setting::OperatingMode),
            Err(BackendError::NoSetting {
                setting: Setting::OperatingMode
            })
        );
        assert_eq!(
            backend.sync_read_state(0, Model::Ax12a, &[1, 2], &mut out),
            Err(BackendError::NoSyncRead)
        );
        assert_eq!(
            backend.sync_read_state(0, MODEL, &[1], &mut out),
            Err(BackendError::SlotMismatch { ids: 1, slots: 2 })
        );
        assert_eq!(backend.sync_read_state(0, MODEL, &[1, 2], &mut out), Ok(()));
    }

    #[test]
    fn the_fake_answers_the_named_models_number_until_scripted() {
        let mut backend = Scripted::new();
        backend.model_number(0, 2, Model::Ax12a.number());
        backend.open_ports(&[]).unwrap();

        assert_eq!(ping(&mut backend, 0, Model::Mx64, 1), Ok(()));
        assert_eq!(ping(&mut backend, 0, MODEL, 1), Ok(()));
        assert_eq!(
            ping(&mut backend, 0, Model::Mx64, 2),
            Err(BackendError::WrongModel { found: 12 })
        );
        assert_eq!(backend.count(Call::Declare), 3);
    }

    #[test]
    fn the_fake_declares_in_id_order_and_stops_at_the_first_failure() {
        let mut backend = Scripted::new();
        backend.missing(0, 3).missing(0, 5);
        backend.open_ports(&[]).unwrap();

        assert_eq!(
            backend.declare(0, &[(5, MODEL), (1, MODEL), (3, MODEL)]),
            Err(Undeclared {
                id: Some(3),
                source: BackendError::Timeout
            })
        );
        let record = backend.last(Call::Declare).unwrap();
        assert!(backend.ids(record).eq([5, 1, 3]));
        assert_eq!(record.count, 3);
    }

    #[test]
    fn the_fake_reads_torque_ctrl_mode_enable_only_on_the_protocol_one_mx64() {
        let mut backend = Scripted::new();
        backend.torque_mode(0, 2, 1);
        backend.open_ports(&[]).unwrap();
        let setting = Setting::TorqueCtrlModeEnable;
        let refused = Err(BackendError::NoRegister {
            register: "Torque Ctrl Mode Enable",
        });

        assert_eq!(backend.read_setting(0, Model::Mx64, 1, setting), Ok(0));
        assert_eq!(backend.read_setting(0, Model::Mx64, 2, setting), Ok(1));
        assert_eq!(backend.read_setting(0, Model::Ax12a, 2, setting), refused);
        assert_eq!(backend.read_setting(0, Model::Mx64V2, 2, setting), refused);
    }

    #[test]
    fn the_fake_answers_nothing_while_its_ports_are_closed() {
        let mut backend = Scripted::new();
        let mut out = [Ok(RawState::default())];

        let closed = Err(BackendError::PortNotOpen { port: 0 });
        assert_eq!(ping(&mut backend, 0, MODEL, 1), closed);
        assert_eq!(backend.force_torque_off(0, MODEL, 1), closed);
        assert_eq!(backend.sync_read_state(0, MODEL, &[1], &mut out), closed);
        assert_eq!(start(&mut backend, 1), closed);

        backend.open_ports(&[]).unwrap();
        start(&mut backend, 1).unwrap();
        backend.torque(0, MODEL, 1, true).unwrap();
        backend.close_ports();

        assert_eq!(ping(&mut backend, 0, MODEL, 1), closed);
        assert_eq!(backend.sync_write_goal(0, MODEL, &[(1, 2048)]), closed);
        assert_eq!(backend.force_torque_off(0, MODEL, 1), closed);
        assert!(backend.torqued(0, 1));
    }

    #[test]
    fn a_state_read_logs_one_record_per_sync_read_of_dxl_rs_v2() {
        let mut backend = Scripted::new();
        backend.open_ports(&[]).unwrap();
        backend.clear_log();
        let mut out = [Ok(RawState::default()); 2];

        backend
            .sync_read_state(0, MODEL, &[1, 2], &mut out)
            .unwrap();

        assert_eq!(
            backend.count(Call::SyncReadState),
            Scripted::STATE_READS.len()
        );
        assert!(
            backend
                .calls()
                .iter()
                .all(|record| record.call == Call::SyncReadState && record.count == 2)
        );
    }

    #[test]
    fn the_fake_reads_one_register_per_sync_read_and_both_on_protocol_1() {
        let warm = Health {
            voltage_dv: 118,
            temperature_c: 41,
            error_status: 0x04,
            ..HEALTHY
        };
        let mut backend = Scripted::new();
        backend.health(0, 2, warm).missing(0, 3);
        backend.open_ports(&[]).unwrap();
        backend.clear_log();
        let mut out = [Ok(Reading::default()); 2];

        backend
            .read_vitals(0, MODEL, Vital::Temperature, &[2, 3], &mut out)
            .unwrap();
        assert_eq!(
            out,
            [
                Ok(Reading::of(Vital::Temperature, 41)),
                Err(BackendError::Timeout)
            ]
        );
        backend
            .read_vitals(0, Model::Ax12a, Vital::Voltage, &[2, 1], &mut out)
            .unwrap();
        let both = Reading {
            voltage_dv: Some(118),
            temperature_c: Some(41),
            hardware_error: Some(0x04),
        };
        assert_eq!(out[0], Ok(both));
        assert_eq!(backend.count(Call::ReadVitals), 3);
        assert_eq!(
            backend.read_vitals(0, MODEL, Vital::Voltage, &[2], &mut out),
            Err(BackendError::SlotMismatch { ids: 1, slots: 2 })
        );
    }

    #[test]
    fn setup_outside_the_script_is_ignored() {
        let mut backend = Scripted::new();
        backend
            .missing(PORTS, 1)
            .already_on(0, 253)
            .fail_port(Call::Declare, PORTS, IO);

        assert!(!backend.torqued(0, 253));
        assert!(!backend.torqued(PORTS, 1));
    }
}
