use core::num::NonZeroU32;

use super::desired::Desired;
use super::field::{StateField, sealed};
use super::history::History;
#[cfg(feature = "test-util")]
use super::history::State;
#[cfg(feature = "test-util")]
use super::joint_state::{Q, Qd, Tau};
use super::{HISTORY_DEPTH, JOINTS};
use crate::error::StateError;
use crate::hal::{Command, HoldTarget, Mask, Sample, Written};

/// The robot state abstraction.
///
/// It holds both the desired values, and the history of states. The history
/// keeps [`HISTORY_DEPTH`] ticks, a fixed capacity (SRS section 8, decision
/// 0001). `N` is the actuator count.
pub struct RobotState<const N: usize = JOINTS> {
    pub(in crate::state) desired: Desired<N>,
    pub(in crate::state) history: History<N, HISTORY_DEPTH>,
    /// Read cadence of each actuator in ticks, from the layout; 1 until set.
    pub(in crate::state) cadence: [NonZeroU32; N],
}

impl<const N: usize> Default for RobotState<N> {
    fn default() -> Self {
        Self {
            desired: Desired::default(),
            history: History::default(),
            cadence: [NonZeroU32::MIN; N],
        }
    }
}

/// Direct writes to desired state are not allowed:
/// ```compile_fail
/// # use kmr_core::RobotState;
/// let mut robot = <RobotState>::default();
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
    /// Returns the array [`StateField`] defined as the home position for the
    /// current robot model.
    pub(crate) fn home_state<T: StateField>(&self) -> Result<[T; N], StateError> {
        Err(StateError::Undecided("SRS FR-2.7: home state source"))
    }

    /// Newest entry's slot for field type `T`. None if history is empty.
    pub fn current<T: StateField>(&self) -> Option<&[T; N]> {
        self.history.buf.back().map(T::field)
    }

    /// Newest value at `index` for field type `&T`.
    pub fn current_at<T: StateField>(&self, index: usize) -> Result<&T, StateError> {
        let slice = self
            .current::<T>()
            .ok_or(StateError::OutOfRange { index, len: 0 })?;

        slice.get(index).ok_or(StateError::OutOfRange {
            index,
            len: slice.len(),
        })
    }

    /// Field slot for type `T` at `history_depth` steps back (0 = current).
    /// Err if the requested depth isn't present in the buffer.
    pub fn prev<T>(&self, history_depth: usize) -> Result<&[T; N], StateError>
    where
        T: StateField,
    {
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
        let slice = self.prev::<T>(history_depth)?;

        slice.get(index).ok_or(StateError::OutOfRange {
            index,
            len: slice.len(),
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
}

/// The engine's side: what it hands the adapter and resets once applied.
impl RobotState<JOINTS> {
    /// Takes the read cadence of the finished layout.
    pub(crate) fn set_cadence(&mut self, cadence: [NonZeroU32; JOINTS]) {
        self.cadence = cadence;
    }

    /// Fields written since the last apply.
    pub(crate) fn written(&self) -> &Written {
        &self.desired.written
    }

    /// Borrows Desired as the command `write` sends.
    pub(crate) fn command(&self) -> Command<'_> {
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
    /// adapter: what it does not read keeps its last value (SRS FR-2.6).
    pub(crate) fn push_slot(&mut self) -> Option<Sample<'_>> {
        let next = self.history.buf.back().copied().unwrap_or_default();
        self.history.buf.push_back(next);
        let slot = self.history.buf.back_mut()?; // None only with a depth of 0
        Some(Sample::new(
            &mut slot.q,
            &mut slot.qd,
            &mut slot.tau,
            &mut slot.kp,
            &mut slot.kd,
        ))
    }

    /// The latest sample overlaid with this tick's writes, never the cleared
    /// Desired. `None` before the first sample.
    pub(crate) fn hold_target(&self) -> Option<HoldTarget> {
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

/// `sample` with the values of `desired` whose bit is set in `mask`.
fn overlay<T: Copy, const N: usize>(mut sample: [T; N], desired: &[T; N], mask: Mask) -> [T; N] {
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
        let gains = State::default();
        self.history.buf.push_back(State {
            q,
            qd,
            tau,
            ..gains
        });
    }

    /// Copy of the desired buffer selected by `T`.
    // `&mut self` because the sealed `Slot::slot` only hands out `&mut`.
    pub fn staged<T: StateField>(&mut self) -> [T; N] {
        *<T as sealed::Slot>::slot(&mut self.desired)
    }
}
