//! The EEPROM limits `health` writes to an actuator, through `dxl_rs_v2`'s
//! typed setters, and the values they replaced, put back at `close` for an
//! actuator whose settings ask for it (T083, T084).

use tracing::{debug, warn};

use super::backend::{Backend, BackendError, Fault, Setting};
use super::config::{Actuator, HealthLimits, Model, Protocol};
use super::{Dynamixel, DynamixelError};

/// A typed write of one limit register to the actuator at `(bus, model,
/// id)`.
type LimitWrite<B, T> = fn(&mut B, (usize, Model, u8), T) -> Result<(), BackendError>;

/// Limit registers of one actuator, each `None` where it is left alone:
/// what `health` writes, or what it found there before writing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Limits {
    /// Min and Max Voltage Limit, in 0.1 V.
    pub(crate) voltage_dv: Option<(u16, u16)>,
    /// Temperature Limit, in °C.
    pub(crate) temperature_c: Option<u8>,
}

impl Limits {
    /// Every register left alone.
    pub(crate) const NONE: Self = Self {
        voltage_dv: None,
        temperature_c: None,
    };

    /// Whether every register is left alone.
    pub(crate) const fn is_empty(self) -> bool {
        self.voltage_dv.is_none() && self.temperature_c.is_none()
    }

    /// `read`, each register `self` sets replaced by its value: what a read
    /// back after writing `self` must find.
    fn over(self, read: HealthLimits) -> HealthLimits {
        let (min_voltage_dv, max_voltage_dv) = self
            .voltage_dv
            .unwrap_or((read.min_voltage_dv, read.max_voltage_dv));
        HealthLimits {
            min_voltage_dv,
            max_voltage_dv,
            temperature_c: self.temperature_c.unwrap_or(read.temperature_c),
        }
    }
}

#[expect(
    private_bounds,
    reason = "the SDK seam stays private: these helpers serve the Hal impl only"
)]
impl<B: Backend, const NB: usize, const N: usize> Dynamixel<B, NB, N> {
    /// The limits `health` writes to actuator `index`: its placement's
    /// health limits, each register its settings set replaced.
    // DECISION: the settings of the declared ids override the placement's
    // limits register by register: they are the narrower knob.
    pub(super) fn limit_writes(&self, index: usize) -> Limits {
        let settings = self.actuators.settings(index);
        let placed = self.actuators.limits(index);
        Limits {
            voltage_dv: settings
                .voltage_limits_dv
                .or(placed.map(|limits| (limits.min_voltage_dv, limits.max_voltage_dv))),
            temperature_c: settings
                .temperature_limit_c
                .or(placed.map(|limits| limits.temperature_c)),
        }
    }

    /// The `LimitsMismatch` of actuator `index`, if `read` is not what
    /// writing `written` leaves.
    pub(super) fn mismatch(
        &self,
        index: usize,
        written: Limits,
        read: HealthLimits,
    ) -> Option<DynamixelError> {
        let expected = written.over(read);
        let (bus, id) = self.named(index);
        (!written.is_empty() && expected != read).then_some(DynamixelError::LimitsMismatch {
            actuator: index,
            bus,
            id,
            written: expected,
            read,
        })
    }

    /// Writes `writes` to `actuator` at engine index `index`, its torque off:
    /// refused when it is found powered, never switched off. With
    /// `restore_limits` set, each register is saved before its write: from
    /// the fault the actuator answers with when that carries it, written
    /// first, then once `clear_fault` cleared it, read clean.
    pub(super) fn write_limits(
        &mut self,
        index: usize,
        actuator: Actuator,
        writes: Limits,
    ) -> Result<(), DynamixelError> {
        let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
        // DECISION: an actuator found powered is refused, never switched off
        // here: another process left it on, it may be holding a load, and
        // cutting torque is the loop's safety call, not the bring-up's.
        // DECISION: a faulted one is not asked: dxl_rs_v2 returns no register
        // value beside a fault. So a held faulted one is switched off below
        // even if powered: on Protocol 1 only the faults set in Alarm
        // Shutdown cut torque (the AX-12A's default leaves Input Voltage
        // out), and the bench AX-12A at 10.1 V is held faulted.
        if !self.backend.faulted(bus, model, id) {
            let on = self
                .backend
                .read_setting(bus, model, id, Setting::TorqueEnable)
                .map_err(|error| self.actuator_error(index, error))?;
            if on != 0 {
                return Err(DynamixelError::LimitsTorqueOn {
                    actuator: index,
                    bus: self.label(bus),
                    id,
                });
            }
        }
        // DECISION (dxl_rs_v2 D-34): Torque Enable = 0 is sent to an actuator
        // known off or held faulted, as dxl_rs_v2 takes no limit write before
        // it saw one answered since Setup; drop it once D-34 is fixed.
        self.backend
            .confirm_torque_off(bus, model, id)
            .map_err(|error| self.actuator_error(index, error))?;
        if !self.actuators.settings(index).restore_limits {
            return self.send_limits(index, actuator, writes);
        }
        let (mut now, mut later, mut saved) = (Limits::NONE, Limits::NONE, Limits::NONE);
        if writes.temperature_c.is_some() {
            match self.old_temperature(actuator) {
                Ok(old) => {
                    (saved.temperature_c, now.temperature_c) = (Some(old), writes.temperature_c)
                }
                Err(BackendError::Fault(_)) => later.temperature_c = writes.temperature_c,
                Err(error) => return Err(self.actuator_error(index, error)),
            }
        }
        if writes.voltage_dv.is_some() {
            match self.old_voltages(actuator) {
                Ok(old) => (saved.voltage_dv, now.voltage_dv) = (Some(old), writes.voltage_dv),
                Err(BackendError::Fault(_)) => later.voltage_dv = writes.voltage_dv,
                Err(error) => return Err(self.actuator_error(index, error)),
            }
        }
        self.save(index, saved);
        self.send_limits(index, actuator, now)?;
        if later.is_empty() {
            return Ok(());
        }
        self.clear_fault(index, actuator)?;
        let mut saved = Limits::NONE;
        if later.temperature_c.is_some() {
            let old = self
                .old_temperature(actuator)
                .map_err(|error| self.actuator_error(index, error))?;
            saved.temperature_c = Some(old);
        }
        if later.voltage_dv.is_some() {
            let old = self
                .old_voltages(actuator)
                .map_err(|error| self.actuator_error(index, error))?;
            saved.voltage_dv = Some(old);
        }
        self.save(index, saved);
        self.send_limits(index, actuator, later)
    }

    /// Asks `actuator` at engine index `index` whether its fault is gone;
    /// one warning to reboot it when a Protocol 2 one keeps it.
    pub(super) fn clear_fault(
        &mut self,
        index: usize,
        actuator: Actuator,
    ) -> Result<(), DynamixelError> {
        let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
        let label = self.label(bus);
        if let Err(error) = self.backend.clear_fault(bus, model, id) {
            if model.protocol() == Protocol::Two && matches!(error, BackendError::Fault(_)) {
                warn!(
                    actuator = index,
                    bus = label,
                    id,
                    "Protocol 2 keeps a hardware error until the actuator reboots: power-cycle it once its cause is gone"
                );
            }
            return Err(self.actuator_error(index, error));
        }
        debug!(actuator = index, bus = label, id, "hardware fault cleared");
        Ok(())
    }

    /// Puts back every limit register `health` saved, Temperature Limit
    /// first: one attempt each, a warning for each that fails or that a
    /// read back does not confirm, and the saved values forgotten.
    // DECISION: Temperature Limit first: putting back the voltages is what
    // brings back the bench AX-12A's voltage fault, and that fault carries
    // the voltages their read back needs; a temperature read back beside it
    // would fail unconfirmed.
    pub(super) fn restore_limits(&mut self) {
        for index in self.actuators.owned().iter() {
            let saved = self.saved.get_mut(index).map(core::mem::take);
            let (Some(saved), Some(&actuator)) = (saved, self.actuators.get(index)) else {
                continue;
            };
            if let Some(celsius) = saved.temperature_c {
                self.restore(
                    index,
                    actuator,
                    "Temperature Limit",
                    celsius,
                    |backend, (bus, model, id), celsius| {
                        backend.write_temperature_limit(bus, model, id, celsius)
                    },
                    Self::old_temperature,
                );
            }
            if let Some(voltages) = saved.voltage_dv {
                self.restore(
                    index,
                    actuator,
                    "Min and Max Voltage Limit",
                    voltages,
                    |backend, (bus, model, id), (min_dv, max_dv)| {
                        backend.write_voltage_limits(bus, model, id, min_dv, max_dv)
                    },
                    Self::old_voltages,
                );
            }
        }
    }

    /// Writes `saved` back to `register` of `actuator` at engine index
    /// `index` once; a reply carrying a fault is checked by one `read`, which
    /// a fault carrying the register answers too. Warns unless put back.
    // DECISION: a fault reply is doubted: dxl_rs_v2 measures a reply that
    // carries a hardware fault beside an instruction error into the same
    // fault as one whose write ran.
    fn restore<T: PartialEq + Copy>(
        &mut self,
        index: usize,
        actuator: Actuator,
        register: &'static str,
        saved: T,
        write: LimitWrite<B, T>,
        read: fn(&mut Self, Actuator) -> Result<T, BackendError>,
    ) {
        let (bus, id) = (actuator.bus, actuator.id);
        let label = self.label(bus);
        let fault = match write(&mut self.backend, (bus, actuator.model, id), saved) {
            Ok(()) => {
                debug!(
                    actuator = index,
                    bus = label,
                    id,
                    register,
                    "limit restored"
                );
                return;
            }
            Err(error @ BackendError::Fault(_)) => error,
            Err(error) => {
                warn!(actuator = index, bus = label, id, register, %error, "limit not restored");
                return;
            }
        };
        match read(self, actuator) {
            Ok(held) if held == saved => {
                debug!(actuator = index, bus = label, id, register, %fault, "limit restored");
            }
            Ok(_) => warn!(
                actuator = index,
                bus = label,
                id,
                register,
                %fault,
                "limit not restored: its reply carried a fault and its read back finds another value"
            ),
            Err(error) => warn!(
                actuator = index,
                bus = label,
                id,
                register,
                %fault,
                %error,
                "limit maybe not restored: its reply carried a fault and its read back failed"
            ),
        }
    }

    /// Writes `limits` to `actuator` at engine index `index`, Temperature
    /// Limit first; a reply carrying a fault counts as written, `clear_fault`
    /// and the read back decide.
    // DECISION: a limit is written to fix the fault the actuator reports, so
    // its reply still carries it.
    fn send_limits(
        &mut self,
        index: usize,
        actuator: Actuator,
        limits: Limits,
    ) -> Result<(), DynamixelError> {
        let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
        if let Some(celsius) = limits.temperature_c {
            let answer = self
                .backend
                .write_temperature_limit(bus, model, id, celsius);
            written(answer).map_err(|error| self.actuator_error(index, error))?;
        }
        if let Some((min_dv, max_dv)) = limits.voltage_dv {
            let answer = self
                .backend
                .write_voltage_limits(bus, model, id, min_dv, max_dv);
            written(answer).map_err(|error| self.actuator_error(index, error))?;
        }
        Ok(())
    }

    /// Keeps `saved` for actuator `index`, a register saved before kept.
    // DECISION: saved before its write is sent, so a write that fails after
    // it may have run is put back too; writing back the value read is
    // harmless when it did not.
    fn save(&mut self, index: usize, saved: Limits) {
        if let Some(slot) = self.saved.get_mut(index) {
            slot.voltage_dv = slot.voltage_dv.or(saved.voltage_dv);
            slot.temperature_c = slot.temperature_c.or(saved.temperature_c);
        }
    }

    /// The Temperature Limit `actuator` holds: read, or carried by the
    /// temperature fault it answers with.
    fn old_temperature(&mut self, actuator: Actuator) -> Result<u8, BackendError> {
        let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
        match self.backend.read_temperature_limit(bus, model, id) {
            Err(BackendError::Fault(Fault::Temperature { limit_c, .. })) => Ok(limit_c),
            read => read,
        }
    }

    /// The Min and Max Voltage Limit `actuator` holds: read, or carried by
    /// the voltage fault it answers with.
    fn old_voltages(&mut self, actuator: Actuator) -> Result<(u16, u16), BackendError> {
        let (bus, model, id) = (actuator.bus, actuator.model, actuator.id);
        match self.backend.read_voltage_limits(bus, model, id) {
            Err(BackendError::Fault(
                Fault::Voltage { min_dv, max_dv, .. } | Fault::VoltageInside { min_dv, max_dv, .. },
            )) => Ok((min_dv, max_dv)),
            read => read,
        }
    }
}

/// `answer` with a reply reporting only a hardware fault taken as written:
/// the actuator ran it, and the caller's read back confirms the value.
// DECISION: a status reply counts too. The actuator builds it before the new
// limit applies, so the write that lowers the Min Voltage Limit below a low
// supply answers with the Input Voltage bit it is about to clear.
fn written(answer: Result<(), BackendError>) -> Result<(), BackendError> {
    match answer {
        Err(BackendError::Fault(_)) => Ok(()),
        Err(BackendError::Status(status)) if status.report.hardware_only() => Ok(()),
        answer => answer,
    }
}
