use super::desired::Desired;
use super::field::{StateField, sealed};
use super::history::History;
use super::{HISTORY_DEPTH, JOINTS};
use crate::error::StateError;

#[derive(Default)]
/// The robot state abstraction.
///
/// It holds both the desired values, and the history of states.
///
/// `DEPTH` is the number of ticks kept in the history, NOT the joint count
/// (that one is fixed by [`JOINTS`]). It defaults to [`HISTORY_DEPTH`] so that
/// signatures write a bare `RobotState`: with nothing to pass, the joint count
/// cannot be passed as the depth by mistake.
pub struct RobotState<const DEPTH: usize = HISTORY_DEPTH> {
    pub(in crate::state) desired: Desired,
    pub(in crate::state) history: History<JOINTS, DEPTH>,
}

/// Direct writes to desired state are not allowed:
/// ```compile_fail
/// # use kmr_core::RobotState;
/// let mut robot = <RobotState>::default();
/// robot.desired.q[0] = 1.0;
/// ```
impl<const DEPTH: usize> RobotState<DEPTH> {
    // ── Desired setters ─────────────────────────────────────────────────

    /// Write one desired value to `index` in the slot selected by `T`.
    /// `T` is inferred from `value` — no turbofish at the call site.
    pub fn set_at<T: StateField>(&mut self, desired: T, index: usize) -> Result<(), StateError> {
        let slot = <T as sealed::Slot>::slot(&mut self.desired);
        match slot.get_mut(index) {
            Some(desired_slot) => {
                *desired_slot = desired;
                Ok(())
            }
            None => Err(StateError::OutOfRange { index, len: JOINTS }),
        }
    }

    /// Overwrite the entire slot selected by `T` (inferred from `values`).
    pub fn set_all<T: StateField>(&mut self, values: [T; JOINTS]) {
        *<T as sealed::Slot>::slot(&mut self.desired) = values;
    }

    // ── State getters ───────────────────────────────────────────────────
    /// Returns the array [`StateField`] defined as the home position for the
    /// current robot model.
    pub fn home_state<T: StateField>(&self) -> [T; JOINTS] {
        todo!("return home state for specified state field")
    }

    /// Newest entry's slot for field type `T`. None if history is empty.
    pub fn current<T: StateField>(&self) -> Option<&[T; JOINTS]> {
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
    pub fn prev<T>(&self, history_depth: usize) -> Result<&[T; JOINTS], StateError>
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
                // NOT `JOINTS` — that only matched by coincidence (JOINTS == DEPTH).
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

    // ── Public state setters ────────────────────────────────────────────
}
