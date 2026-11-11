use core::num::NonZeroU32;

use super::HISTORY_DEPTH;
use super::desired::Desired;
use super::field::{StateField, sealed};
#[cfg(feature = "test-util")]
use super::history::State;
use super::history::{Flags, History};
#[cfg(feature = "test-util")]
use super::joint_state::{Q, Qd, Tau};
use crate::error::StateError;
use crate::hal::{Command, Faults, HoldTarget, Mask, Sample, TickFlags, Written};
use crate::limits::Staged;
use crate::telemetry::Telemetry;

/// The robot state abstraction.
///
/// It holds both the desired values, and the history of states. The history
/// keeps [`HISTORY_DEPTH`] ticks, a fixed capacity (SRS section 8, decision
/// 0001). Next to it, one block of actuator telemetry, read-only (SRS
/// FR-5.11). `N` is the actuator count.
pub struct RobotState<const N: usize> {
    pub(in crate::state) desired: Desired<N>,
    pub(in crate::state) history: History<N, HISTORY_DEPTH>,
    /// Read cadence of each actuator in ticks, from the layout; 1 until set.
    pub(in crate::state) cadence: [NonZeroU32; N],
    /// Ticks since each actuator was last read; meaningless while unsampled.
    pub(in crate::state) age: [u32; N],
    /// History slots, newest first, that hold a sample of each actuator:
    /// 0 until its first read, then up to [`HISTORY_DEPTH`].
    // DECISION: a count per actuator rather than a mask per slot, so the
    // slot layout of T038 stays as it is.
    pub(in crate::state) sampled: [u32; N],
    /// The newest telemetry of each actuator: one block, not one per slot.
    pub(in crate::state) telemetry: Telemetry<N>,
}

impl<const N: usize> Default for RobotState<N> {
    fn default() -> Self {
        Self {
            desired: Desired::default(),
            history: History::default(),
            cadence: [NonZeroU32::MIN; N],
            age: [0; N],
            sampled: [0; N],
            telemetry: Telemetry::default(),
        }
    }
}

/// Direct writes to desired state are not allowed:
/// ```compile_fail
/// # use kmr_core::RobotState;
/// let mut robot = RobotState::<4>::default();
/// robot.desired.q[0] = 1.0;
/// ```
impl<const N: usize> RobotState<N> {
    // ── Desired setters ─────────────────────────────────────────────────

    /// Write one desired value to `index` in the slot selected by `T`, and
    /// mark it written. `T` is inferred from `value` — no turbofish at the
    /// call site.
    pub fn set_at<T: StateField>(&mut self, desired: T, index: usize) -> Result<(), StateError> {
        let slot = <T as sealed::Slot>::slot(&mut self.desired);
        match slot.get_mut(index) {
            Some(desired_slot) => {
                *desired_slot = desired;
                <T as sealed::Slot>::mask(&mut self.desired.written).set(index);
                Ok(())
            }
            None => Err(StateError::OutOfRange { index, len: N }),
        }
    }

    /// Overwrite the entire slot selected by `T` (inferred from `values`),
    /// and mark every actuator of it written.
    pub fn set_all<T: StateField>(&mut self, values: [T; N]) {
        *<T as sealed::Slot>::slot(&mut self.desired) = values;
        *<T as sealed::Slot>::mask(&mut self.desired.written) = Mask::ALL;
    }

    // ── State getters ───────────────────────────────────────────────────
    /// Newest entry's slot for field type `T`. None if history is empty or
    /// an actuator has no sample yet.
    pub fn current<T: StateField>(&self) -> Option<&[T; N]> {
        self.prev(0).ok()
    }

    /// Newest value at `index` for field type `&T`.
    pub fn current_at<T: StateField>(&self, index: usize) -> Result<&T, StateError> {
        self.prev_at(0, index)
    }

    /// Field slot for type `T` at `history_depth` steps back (0 = current).
    /// Err if the requested depth isn't present in the buffer, or if an
    /// actuator had no sample yet at that depth ([`StateError::NoSample`]).
    pub fn prev<T>(&self, history_depth: usize) -> Result<&[T; N], StateError>
    where
        T: StateField,
    {
        let slot = self.slot::<T>(history_depth)?;
        if self.sampled.iter().any(|&held| !holds(held, history_depth)) {
            return Err(StateError::NoSample {
                depth: history_depth,
            });
        }
        Ok(slot)
    }

    /// The slot `history_depth` back, sampled or not.
    fn slot<T: StateField>(&self, history_depth: usize) -> Result<&[T; N], StateError> {
        self.history
            .buf
            .nth_back(history_depth)
            .map(T::field)
            .ok_or(StateError::HistoryValueOutOfRange {
                provided_depth: history_depth,
                // Deepest depth currently available (`nth_back` accepts 0..len).
                // NOT `N`: that only matched while N and the depth were both 4.
                max_depth: self.history.buf.len().saturating_sub(1),
            })
    }

    /// Returns the previous state at the specified depth.
    ///
    /// A "previous state" means "the state values at the previous tick number N"
    ///
    /// If Q[0] increases by +5 each tick, starting from 0:
    /// At tick 10, Q[0] equals 50.
    /// At tick 10, the previous Q[0] of depth 2 refers to the value of Q[0]
    /// at tick 8. So prev_at() in this example would return Q[0] == 40.
    ///
    /// For memory reasons, the history is not infinite.
    /// If you need more depth, you can modify it's maximum value with ...
    /// TODO: add the method for setting the max depth
    pub fn prev_at<T>(&self, history_depth: usize, index: usize) -> Result<&T, StateError>
    where
        T: StateField,
    {
        let slice = self.slot::<T>(history_depth)?;
        let value = slice
            .get(index)
            .ok_or(StateError::OutOfRange { index, len: N })?;
        match self.sampled.get(index) {
            Some(&held) if holds(held, history_depth) => Ok(value),
            _ => Err(StateError::NoSample {
                depth: history_depth,
            }),
        }
    }

    /// Ticks since actuator `index` was last read: 0 on the tick it was read.
    /// `None` until its first read, and always on a run without hardware.
    pub fn age(&self, index: usize) -> Result<Option<u32>, StateError> {
        match (self.age.get(index), self.sampled.get(index)) {
            (Some(&age), Some(&held)) => Ok((held > 0).then_some(age)),
            _ => Err(StateError::OutOfRange { index, len: N }),
        }
    }

    /// Age of the oldest sample over all actuators. `None` while one of them
    /// was never read.
    // DECISION: an actuator never read makes the whole sample age unknown,
    // not the maximum of the others.
    pub fn sample_age(&self) -> Option<u32> {
        self.age
            .iter()
            .zip(&self.sampled)
            .try_fold(0, |oldest, (&age, &held)| {
                (held > 0).then(|| oldest.max(age))
            })
    }

    /// The fault bits of the tick `history_depth` back (0 = this tick). A
    /// tick's `read` faults show at depth 0 from `First` on; its `write` and
    /// limit faults from the next tick on, at depth 1 (SRS FR-13.4).
    pub fn faults(&self, history_depth: usize) -> Result<&Faults<N>, StateError> {
        self.flags(history_depth).map(|flags| &flags.faults)
    }

    /// The whole-tick flags of the tick `history_depth` back (0 = this
    /// tick). `OVERRUN` is set once the tick ended, so it reads at depth 1.
    pub fn tick_flags(&self, history_depth: usize) -> Result<TickFlags, StateError> {
        self.flags(history_depth).map(|flags| flags.tick)
    }

    /// The flags of the slot `history_depth` back.
    fn flags(&self, history_depth: usize) -> Result<&Flags<N>, StateError> {
        self.history
            .flags
            .nth_back(history_depth)
            .ok_or(StateError::HistoryValueOutOfRange {
                provided_depth: history_depth,
                max_depth: self.history.flags.len().saturating_sub(1),
            })
    }

    /// Read cadence of actuator `index`: it is read every `cadence` ticks.
    /// Set by the adapter at bring-up, read-only here.
    pub fn cadence(&self, index: usize) -> Result<NonZeroU32, StateError> {
        self.cadence
            .get(index)
            .copied()
            .ok_or(StateError::OutOfRange { index, len: N })
    }

    /// The actuators' telemetry, as their adapters last reported it (SRS
    /// FR-5.11). Read-only: no controller writes it.
    pub fn telemetry(&self) -> &Telemetry<N> {
        &self.telemetry
    }
}

/// The engine's side: what it hands the adapter and resets once applied.
impl<const N: usize> RobotState<N> {
    /// Takes the read cadence of the finished layout.
    pub(crate) fn set_cadence(&mut self, cadence: [NonZeroU32; N]) {
        self.cadence = cadence;
    }

    /// Fields written since the last apply.
    pub(crate) fn written(&self) -> &Written<N> {
        &self.desired.written
    }

    /// Borrows Desired as the command `write` sends.
    pub(crate) fn command(&self) -> Command<'_, N> {
        Command::new(
            &self.desired.q,
            &self.desired.qd,
            &self.desired.tau,
            &self.desired.kp,
            &self.desired.kd,
            &self.desired.written,
        )
    }

    /// Resets the values and the masks once applied (SRS FR-2.5).
    pub(crate) fn clear_desired(&mut self) {
        self.desired = Desired::default();
    }

    /// Pushes a copy of `prev(0)` (zeros when empty) and lends it to the
    /// adapter: what it does not read keeps its last value (SRS FR-2.6). Every
    /// sampled actuator ages one tick.
    pub(crate) fn push_slot(&mut self) -> Option<Sample<'_, N>> {
        self.push_tick().map(|(sample, _faults)| sample)
    }

    /// [`Self::push_slot`], plus the new slot's faults, empty, for `read` to
    /// fill.
    pub(crate) fn push_tick(&mut self) -> Option<(Sample<'_, N>, &mut Faults<N>)> {
        for (age, held) in self.age.iter_mut().zip(&mut self.sampled) {
            if *held > 0 {
                *age = age.saturating_add(1);
                *held = held.saturating_add(1).min(DEPTH);
            }
        }
        let next = self.history.buf.back().copied().unwrap_or_default();
        self.history.buf.push_back(next);
        self.history.flags.push_back(Flags::default());
        let slot = self.history.buf.back_mut()?; // None only with a depth of 0
        let flags = self.history.flags.back_mut()?;
        Some((
            Sample::new(
                &mut slot.q,
                &mut slot.qd,
                &mut slot.tau,
                &mut slot.kp,
                &mut slot.kd,
            ),
            &mut flags.faults,
        ))
    }

    /// The written `Q`, `Qd` and `Tau` with their masks, and the newest
    /// slot's faults: what the limit stage checks and marks.
    pub(crate) fn guarded(&mut self) -> (Staged<'_, N>, Option<&mut Faults<N>>) {
        let desired = &mut self.desired;
        (
            Staged {
                q: &mut desired.q,
                qd: &mut desired.qd,
                tau: &mut desired.tau,
                written: &mut desired.written,
            },
            self.history.flags.back_mut().map(|flags| &mut flags.faults),
        )
    }

    /// The telemetry block for `telemetry` to fill, and the newest slot's
    /// faults beside it.
    pub(crate) fn telemetry_mut(&mut self) -> (&mut Telemetry<N>, Option<&mut Faults<N>>) {
        (
            &mut self.telemetry,
            self.history.flags.back_mut().map(|flags| &mut flags.faults),
        )
    }

    /// Notes the tick in progress, which telemetry ages are counted from.
    pub(crate) fn set_tick(&mut self, tick: u64) {
        self.telemetry.set_tick(tick);
    }

    /// The newest slot's faults, for the limit stage and `write`.
    pub(crate) fn faults_mut(&mut self) -> Option<&mut Faults<N>> {
        self.history.flags.back_mut().map(|flags| &mut flags.faults)
    }

    /// Sets `flags` in the newest slot.
    pub(crate) fn raise(&mut self, flags: TickFlags) {
        if let Some(slot) = self.history.flags.back_mut() {
            slot.tick = slot.tick.union(flags);
        }
    }

    /// The actuators of `read` were refreshed in the newest slot: age 0.
    pub(crate) fn refreshed(&mut self, read: Mask<N>) {
        for (index, (age, held)) in self.age.iter_mut().zip(&mut self.sampled).enumerate() {
            if read.get(index) {
                *age = 0;
                *held = (*held).max(1);
            }
        }
    }

    /// The latest sample overlaid with this tick's writes, never the cleared
    /// Desired. `None` before the first sample.
    pub(crate) fn hold_target(&self) -> Option<HoldTarget<N>> {
        let sample = self.history.buf.back()?;
        let written = self.desired.written;
        Some(HoldTarget {
            q: overlay(sample.q, &self.desired.q, written.q),
            qd: overlay(sample.qd, &self.desired.qd, written.qd),
            tau: overlay(sample.tau, &self.desired.tau, written.tau),
            kp: overlay(sample.kp, &self.desired.kp, written.kp),
            kd: overlay(sample.kd, &self.desired.kd, written.kd),
            written,
        })
    }
}

/// [`HISTORY_DEPTH`] as a sample count.
const DEPTH: u32 = if HISTORY_DEPTH < u32::MAX as usize {
    HISTORY_DEPTH as u32
} else {
    u32::MAX
};

/// Whether an actuator holding `held` sampled slots has one `depth` back.
fn holds(held: u32, depth: usize) -> bool {
    usize::try_from(held).is_ok_and(|held| depth < held)
}

/// `sample` with the values of `desired` whose bit is set in `mask`.
fn overlay<T: Copy, const N: usize>(mut sample: [T; N], desired: &[T; N], mask: Mask<N>) -> [T; N] {
    for (index, (value, wanted)) in sample.iter_mut().zip(desired).enumerate() {
        if mask.get(index) {
            *value = *wanted;
        }
    }
    sample
}

// Test hooks, behind the `test-util` feature. `kmr_api` builds its group logic
// on top of this type but lives in another crate, so `#[cfg(test)]` here would
// not reach its tests and the private fields are out of its sight. A feature
// keeps both hooks out of every normal build: recording a sample is the
// runtime's job, and the desired buffer stays write-only for users.
#[cfg(feature = "test-util")]
#[doc(hidden)]
impl<const N: usize> RobotState<N> {
    /// Push one sensed sample as the newest history entry.
    pub fn record_sample(&mut self, q: [Q; N], qd: [Qd; N], tau: [Tau; N]) {
        if let Some(slot) = self.push_slot() {
            *slot.q = q;
            *slot.qd = qd;
            *slot.tau = tau;
        }
        self.refreshed(Mask::ALL);
    }

    /// Copy of the desired buffer selected by `T`.
    // `&mut self` because the sealed `Slot::slot` only hands out `&mut`.
    pub fn staged<T: StateField>(&mut self) -> [T; N] {
        *<T as sealed::Slot>::slot(&mut self.desired)
    }
}
