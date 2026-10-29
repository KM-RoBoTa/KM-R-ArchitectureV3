//! A backend for tests: records every call and answers from a script.
//!
//! Every actuator starts present and healthy, at `RawState::default()`, its
//! goal registers at 0: what an earlier session left. Faults are scripted
//! per (port, id) and per call kind, and last until [`Scripted::heal`] or
//! for a counted number of calls. Failed calls are recorded too.
//!
//! It refuses what `DxlRs` refuses, with the same error, and never panics
//! (contracts/backend.md § Strictness).

use core::iter;
use std::ffi::OsStr;
use std::os::unix::ffi::OsStrExt;
use std::path::Path;

use super::backend::{
    Backend, BackendError, GoalRegister, Health, OpenFailure, POSITION_MODE, PortSpec, RawState,
    Setting,
};
use super::config::{Model, Protocol};

/// Calls recorded; later ones are only counted.
pub(crate) const CAPACITY: usize = 256;
/// Ids recorded over all calls; later ones are dropped.
const ENTRIES: usize = 4 * CAPACITY;
/// Ports the script covers.
pub(crate) const PORTS: usize = 4;
/// Ids the script covers on each port, `0..=252`.
pub(crate) const IDS: usize = 253;
/// Variants of [`Call`].
const KINDS: usize = 11;
/// Variants of [`GoalRegister`].
const REGISTERS: usize = GoalRegister::ALL.len();
/// Device path bytes kept per port; longer paths are cut.
const PATH: usize = 256;

/// Answer of a present actuator until scripted otherwise.
const HEALTHY: Health = Health {
    error_status: 0,
    voltage_dv: 120,
    temperature_c: 30,
};

/// A [`Backend`] method.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Call {
    /// [`Backend::open_ports`].
    OpenPorts,
    /// [`Backend::ping`].
    Ping,
    /// [`Backend::read_position`].
    ReadPosition,
    /// [`Backend::sync_read_state`].
    SyncReadState,
    /// [`Backend::sync_write_goal`].
    SyncWriteGoal,
    /// [`Backend::sync_write_start_goal`].
    SyncWriteStartGoal,
    /// [`Backend::torque`].
    Torque,
    /// [`Backend::force_torque_off`].
    ForceTorqueOff,
    /// [`Backend::read_health`].
    ReadHealth,
    /// [`Backend::read_setting`].
    ReadSetting,
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
    /// Per actuator, switched on through this backend: what `DxlRs` keeps a
    /// handle for.
    powered: [[bool; IDS]; PORTS],
    /// Per actuator, the Torque Enable register.
    torque: [[bool; IDS]; PORTS],
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
    /// Per actuator, whether it never answers.
    missing: [[bool; IDS]; PORTS],
    /// Per actuator and kind, the error its calls fail with.
    faults: [[[Option<BackendError>; KINDS]; IDS]; PORTS],
    /// Per actuator and kind, failures left before the fault clears; `None`
    /// while it lasts until [`Scripted::heal`].
    left: [[[Option<u8>; KINDS]; IDS]; PORTS],
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
            torque: [[false; IDS]; PORTS],
            goals: [[[0; REGISTERS]; IDS]; PORTS],
            started: [[false; IDS]; PORTS],
            states: [[RawState::default(); IDS]; PORTS],
            vitals: [[HEALTHY; IDS]; PORTS],
            modes: [[POSITION_MODE; IDS]; PORTS],
            missing: [[false; IDS]; PORTS],
            faults: [[[None; KINDS]; IDS]; PORTS],
            left: [[[None; KINDS]; IDS]; PORTS],
            port_faults: [[None; KINDS]; PORTS],
            open_fault: None,
        }
    }

    /// Actuator `id` never answers: `Timeout` from every call naming it.
    pub(crate) fn missing(&mut self, port: usize, id: u8) -> &mut Self {
        if let Some(missing) = at(&mut self.missing, port, id) {
            *missing = true;
        }
        self
    }

    /// Calls of kind `call` naming actuator `id` fail with `error`; for
    /// `SyncReadState`, its entry of `out` does. No effect on
    /// `SyncWriteGoal`, which no status packet answers.
    pub(crate) fn fail(
        &mut self,
        call: Call,
        port: usize,
        id: u8,
        error: BackendError,
    ) -> &mut Self {
        if let Some(faults) = at(&mut self.faults, port, id) {
            faults[call.index()] = Some(error);
        }
        if let Some(left) = at(&mut self.left, port, id) {
            left[call.index()] = None;
        }
        self
    }

    /// The next `times` calls of kind `call` naming actuator `id` fail with
    /// `error`, as [`Scripted::fail`]; later ones answer.
    pub(crate) fn fail_times(
        &mut self,
        call: Call,
        port: usize,
        id: u8,
        error: BackendError,
        times: u8,
    ) -> &mut Self {
        if let Some(faults) = at(&mut self.faults, port, id) {
            faults[call.index()] = (times > 0).then_some(error);
        }
        if let Some(left) = at(&mut self.left, port, id) {
            left[call.index()] = Some(times);
        }
        self
    }

    /// Pings of actuator `id` time out; its other calls still answer.
    pub(crate) fn fail_ping(&mut self, port: usize, id: u8) -> &mut Self {
        self.fail(Call::Ping, port, id, BackendError::Timeout)
    }

    /// Every call of kind `call` on `port` fails as a whole with `error`.
    pub(crate) fn fail_port(&mut self, call: Call, port: usize, error: BackendError) -> &mut Self {
        if let Some(faults) = self.port_faults.get_mut(port) {
            faults[call.index()] = Some(error);
        }
        self
    }

    /// Group reads on `port` fail with `Io`.
    pub(crate) fn fail_sync_read(&mut self, port: usize) -> &mut Self {
        self.fail_port(Call::SyncReadState, port, BackendError::Io)
    }

    /// Group writes on `port` fail with `Io`.
    pub(crate) fn fail_sync_write(&mut self, port: usize) -> &mut Self {
        self.fail_port(Call::SyncWriteGoal, port, BackendError::Io)
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
        self.faults = [[[None; KINDS]; IDS]; PORTS];
        self.left = [[[None; KINDS]; IDS]; PORTS];
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

    /// Whether a transaction of kind `call` on `port` goes through: `Io`
    /// while the ports are closed.
    fn transaction(&self, call: Call, port: usize) -> Result<(), BackendError> {
        match self.port_faults.get(port) {
            Some(faults) if self.open => faults[call.index()].map_or(Ok(()), Err),
            _ => Err(BackendError::Io),
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
        let Some(error) = self.faults[port][id][kind] else {
            return Ok(());
        };
        if let Some(left) = &mut self.left[port][id][kind] {
            *left = left.saturating_sub(1);
            if *left == 0 {
                self.faults[port][id][kind] = None;
            }
        }
        Err(error)
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

    /// `Io` unless actuator `id` was switched on through this backend, as
    /// `DxlRs` holds no handle for any other.
    fn held(&self, port: usize, id: u8) -> Result<(), BackendError> {
        if get(&self.powered, port, id) {
            Ok(())
        } else {
            Err(BackendError::Io)
        }
    }

    /// `Io` unless a start goal was written to actuator `id` since its
    /// torque was last switched: torque on would drive it to whatever goal
    /// an earlier session left.
    fn start_written(&self, port: usize, id: u8) -> Result<(), BackendError> {
        if get(&self.started, port, id) {
            Ok(())
        } else {
            Err(BackendError::Io)
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

impl Backend for Scripted {
    const MAX_PORTS: usize = PORTS;
    /// As `DxlRs`: present current, velocity and position, one sync read each.
    const STATE_READS: &'static [u16] = &[2, 4, 4];

    fn open_ports(&mut self, ports: &[PortSpec<'_>]) -> Result<(), OpenFailure> {
        self.record(Call::OpenPorts, 0, None, ports.len(), false, iter::empty());
        if let Some(failure) = self.open_fault {
            return Err(failure);
        }
        if ports.len() > PORTS {
            return Err(OpenFailure {
                port: None,
                source: BackendError::Io,
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

    fn ping(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        self.one(Call::Ping, port, model, id, false)
    }

    fn read_position(&mut self, port: usize, model: Model, id: u8) -> Result<u32, BackendError> {
        self.one(Call::ReadPosition, port, model, id, false)?;
        Ok(self.states[port][usize::from(id)].position.cast_unsigned())
    }

    /// Logs one record per sync read `DxlRs` sends: one per entry of
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
        if model.protocol() != Protocol::Two || out.len() != ids.len() {
            return Err(BackendError::Io);
        }
        self.transaction(Call::SyncReadState, port)?;
        for (entry, &id) in out.iter_mut().zip(ids) {
            *entry = self
                .answer(Call::SyncReadState, port, id)
                .map(|()| self.states[port][usize::from(id)]);
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
    /// then, it is no start goal. Every register is taken, as `DxlRs` will
    /// once `dxl_rs` writes them before the torque: the tests pin that.
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
        if goals.iter().any(|&(id, _)| get(&self.powered, port, id)) {
            return Err(BackendError::Io);
        }
        self.transaction(Call::SyncWriteStartGoal, port)?;
        for &(id, raw) in goals {
            self.set_goal(port, id, register, raw);
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
        } else {
            self.held(port, id)?;
        }
        self.answer(Call::Torque, port, id)?;
        self.set_torque(port, id, on);
        Ok(())
    }

    fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError> {
        self.one(Call::ForceTorqueOff, port, model, id, false)?;
        self.set_torque(port, id, false);
        Ok(())
    }

    fn read_health(&mut self, port: usize, model: Model, id: u8) -> Result<Health, BackendError> {
        self.note(Call::ReadHealth, port, model, id, false);
        if model.protocol() != Protocol::Two {
            return Err(BackendError::Io);
        }
        self.answer(Call::ReadHealth, port, id)?;
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
        if model.protocol() != Protocol::Two {
            return Err(BackendError::Io);
        }
        self.answer(Call::ReadSetting, port, id)?;
        let id = usize::from(id);
        Ok(match setting {
            Setting::OperatingMode => self.modes[port][id],
            Setting::TorqueEnable => u8::from(self.torque[port][id]),
        })
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
        backend.ping(0, MODEL, 1).unwrap();
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
            Call::Ping,
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
        assert!(backend.goals(backend.last(Call::Ping).unwrap()).is_empty());
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

        assert_eq!(backend.ping(0, MODEL, 2), Err(BackendError::Timeout));
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
        backend.fail_open(BackendError::Io);
        assert_eq!(
            backend.open_ports(&[]),
            Err(OpenFailure {
                port: None,
                source: BackendError::Io
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
            Err(BackendError::Io)
        );
        let mut out = [Ok(RawState::default())];
        assert_eq!(
            backend.sync_read_state(1, MODEL, &[4], &mut out),
            Err(BackendError::Io)
        );
        assert_eq!(backend.ping(0, MODEL, 4), Err(BackendError::Timeout));
        assert_eq!(
            backend.read_health(0, MODEL, 4),
            Err(BackendError::Status(0x20))
        );
        assert_eq!(backend.read_position(0, MODEL, 4), Ok(0));
        assert_eq!(backend.read_health(0, MODEL, 5), Ok(hot));
        assert_eq!(backend.ping(PORTS, MODEL, 4), Err(BackendError::Io));

        backend.heal();
        assert!(backend.is_open());
        assert_eq!(backend.sync_write_goal(0, MODEL, &[(4, 0)]), Ok(()));
        assert_eq!(backend.ping(0, MODEL, 4), Ok(()));
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
        assert_eq!(backend.ping(0, MODEL, 1), Ok(()));
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
            .fail_times(Call::Ping, 0, 1, BackendError::Io, 1)
            .fail(Call::Ping, 0, 1, BackendError::Io);
        for _ in 0..3 {
            assert_eq!(backend.ping(0, MODEL, 1), Err(BackendError::Io));
        }
        backend.fail_times(Call::Ping, 0, 1, BackendError::Io, 0);
        assert_eq!(backend.ping(0, MODEL, 1), Ok(()));
    }

    #[test]
    fn log_is_bounded_but_counts_are_exact() {
        let mut backend = Scripted::new();
        backend.open_ports(&[]).unwrap();
        backend.clear_log();
        for _ in 0..CAPACITY + 10 {
            backend.ping(0, MODEL, 1).unwrap();
        }
        assert_eq!(backend.calls().len(), CAPACITY);
        assert_eq!(backend.count(Call::Ping), CAPACITY + 10);

        backend.clear_log();
        assert!(backend.calls().is_empty());
        assert_eq!(backend.count(Call::Ping), 0);
        assert_eq!(backend.last(Call::Ping), None);
    }

    #[test]
    fn the_fake_refuses_a_plain_torque_off_it_never_powered() {
        let mut backend = Scripted::new();
        backend.already_on(0, 1);
        backend.open_ports(&[]).unwrap();

        assert_eq!(backend.torque(0, MODEL, 1, false), Err(BackendError::Io));
        assert_eq!(backend.torque(0, MODEL, 2, false), Err(BackendError::Io));
        assert!(backend.torqued(0, 1));

        assert_eq!(backend.force_torque_off(0, MODEL, 1), Ok(()));
        assert_eq!(backend.force_torque_off(0, MODEL, 2), Ok(()));
        assert!(!backend.torqued(0, 1) && !backend.torqued(0, 2));
        assert_eq!(backend.count(Call::ForceTorqueOff), 2);

        assert_eq!(start(&mut backend, 1), Ok(()));
        assert_eq!(backend.torque(0, MODEL, 1, true), Ok(()));
        assert_eq!(backend.force_torque_off(0, MODEL, 1), Ok(()));
        assert_eq!(backend.torque(0, MODEL, 1, false), Err(BackendError::Io));
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
            Err(BackendError::Io)
        );
        assert_eq!(backend.sync_write_goal(0, MODEL, &[(2, 2048)]), Ok(()));
        assert_eq!(backend.count(Call::SyncWriteGoal), 2);
    }

    #[test]
    fn the_fake_refuses_torque_on_without_a_start_goal_since_the_last_switch() {
        let mut backend = Scripted::new();
        backend.left_goal(0, 1, GoalRegister::Position, 4000);
        backend.open_ports(&[]).unwrap();

        assert_eq!(backend.torque(0, MODEL, 1, true), Err(BackendError::Io));
        assert!(!backend.torqued(0, 1));

        assert_eq!(start(&mut backend, 1), Ok(()));
        assert_eq!(backend.goal(0, 1, GoalRegister::Position), 2048);
        assert_eq!(backend.torque(0, MODEL, 1, true), Ok(()));
        assert!(backend.torqued(0, 1));

        assert_eq!(backend.force_torque_off(0, MODEL, 1), Ok(()));
        assert_eq!(backend.torque(0, MODEL, 1, true), Err(BackendError::Io));
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
            Err(BackendError::Io)
        );
        assert_eq!(backend.goal(0, 1, GoalRegister::Position), 100);
        assert_eq!(start(&mut backend, 1), Ok(()));
        assert_eq!(backend.count(Call::SyncWriteStartGoal), 3);
    }

    #[test]
    fn the_fake_refuses_protocol_one_health() {
        let mut backend = Scripted::new();
        backend.open_ports(&[]).unwrap();

        assert_eq!(
            backend.read_health(0, Model::Ax12a, 1),
            Err(BackendError::Io)
        );
        assert_eq!(backend.read_position(0, Model::Ax12a, 1), Ok(0));
        assert_eq!(backend.read_health(0, MODEL, 1), Ok(HEALTHY));
        assert_eq!(backend.count(Call::ReadHealth), 2);
    }

    #[test]
    fn the_fake_refuses_protocol_one_settings_and_group_reads() {
        let mut backend = Scripted::new();
        backend.open_ports(&[]).unwrap();
        let mut out = [Ok(RawState::default()); 2];

        assert_eq!(
            backend.read_setting(0, Model::Rx28, 1, Setting::OperatingMode),
            Err(BackendError::Io)
        );
        assert_eq!(
            backend.sync_read_state(0, Model::Ax12a, &[1, 2], &mut out),
            Err(BackendError::Io)
        );
        assert_eq!(
            backend.sync_read_state(0, MODEL, &[1], &mut out),
            Err(BackendError::Io)
        );
        assert_eq!(backend.sync_read_state(0, MODEL, &[1, 2], &mut out), Ok(()));
    }

    #[test]
    fn the_fake_answers_nothing_while_its_ports_are_closed() {
        let mut backend = Scripted::new();
        let mut out = [Ok(RawState::default())];

        assert_eq!(backend.ping(0, MODEL, 1), Err(BackendError::Io));
        assert_eq!(backend.force_torque_off(0, MODEL, 1), Err(BackendError::Io));
        assert_eq!(
            backend.sync_read_state(0, MODEL, &[1], &mut out),
            Err(BackendError::Io)
        );
        assert_eq!(start(&mut backend, 1), Err(BackendError::Io));

        backend.open_ports(&[]).unwrap();
        start(&mut backend, 1).unwrap();
        backend.torque(0, MODEL, 1, true).unwrap();
        backend.close_ports();

        assert_eq!(backend.ping(0, MODEL, 1), Err(BackendError::Io));
        assert_eq!(
            backend.sync_write_goal(0, MODEL, &[(1, 2048)]),
            Err(BackendError::Io)
        );
        assert_eq!(backend.force_torque_off(0, MODEL, 1), Err(BackendError::Io));
        assert!(backend.torqued(0, 1));
    }

    #[test]
    fn a_state_read_logs_one_record_per_sync_read_of_dxl_rs() {
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
    fn setup_outside_the_script_is_ignored() {
        let mut backend = Scripted::new();
        backend
            .missing(PORTS, 1)
            .already_on(0, 253)
            .fail_port(Call::Ping, PORTS, BackendError::Io);

        assert!(!backend.torqued(0, 253));
        assert!(!backend.torqued(PORTS, 1));
    }
}
