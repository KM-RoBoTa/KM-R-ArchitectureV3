use circular_buffer::CircularBuffer;

use crate::error::StateError;

pub const JOINTS: usize = 4; // todo: build time variable from model
pub const HISTORY_DEPTH: usize = 4; // todo: build time variable from model

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Q(pub f32);

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Qd(pub f32);

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Tau(pub f32);

/// WRITE ONLY
#[derive(Debug, PartialEq)]
struct Desired {
    q: [Q; JOINTS],
    qd: [Qd; JOINTS],
    tau: [Tau; JOINTS],
}

impl Default for Desired {
    fn default() -> Self {
        Self {
            q: State::default().q,
            qd: State::default().qd,
            tau: State::default().tau,
        }
    }
}

#[derive(Debug, PartialEq)]
pub struct State<const N: usize> {
    pub q: [Q; N],
    pub qd: [Qd; N],
    pub tau: [Tau; N],
}

impl State<JOINTS> {
    fn home_state() -> Self {
        todo!(
            "I must save somewhere permanant the home state. This must be the default because zeroing all actuators as default is dangerous."
        )
    }
}

impl Default for State<JOINTS> {
    fn default() -> Self {
        // WARN: default must depend on the robot model
        Self::home_state()
    }
}

#[derive(Default, Debug, PartialEq)]
/// READ ONY
pub struct History<const N: usize, const DEPTH: usize> {
    buf: CircularBuffer<DEPTH, State<N>>,
}

/// Routes a field element type to its storage slot in [`DesiredState`].
///
/// Lives in a private module so external crates can never name or implement it
/// — that *seals* [`StateField`] (you cannot add new field types from outside)
/// and keeps the private `DesiredState` out of the public API, even though
/// `StateField` itself bounds public methods.
mod sealed;

/// Marks a type that can be stored in [`RobotState`]'s desired state.
/// Sealed: implemented only for the field types defined in this module.
pub trait StateField: sealed::Slot {}

impl sealed::Slot for Q {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.q
    }
    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.q
    }
}
impl StateField for Q {}
impl From<f32> for Q {
    fn from(value: f32) -> Self {
        Q(value)
    }
}

impl sealed::Slot for Qd {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.qd
    }
    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.qd
    }
}
impl StateField for Qd {}
impl From<f32> for Qd {
    fn from(value: f32) -> Self {
        Qd(value)
    }
}

impl sealed::Slot for Tau {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS] {
        &mut desired.tau
    }

    fn field(state: &State<JOINTS>) -> &[Self; JOINTS] {
        &state.tau
    }
}
impl StateField for Tau {}
impl From<f32> for Tau {
    fn from(value: f32) -> Self {
        Tau(value)
    }
}

#[derive(Default)]
pub struct RobotState<const DEPTH: usize> {
    desired: Desired,
    history: History<JOINTS, DEPTH>,
}

/// Direct writes to desired state are not allowed:
/// ```compile_fail
/// # use kmr_core::RobotState;
/// let mut robot = RobotState::default();
/// robot.desired.q[0] = 1.0;
/// ```
impl RobotState<JOINTS> {
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
    pub fn home_state<T: StateField>(&self) -> [T; JOINTS] {
        todo!("return home state for specigied state field")
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

#[cfg(test)]
mod test;
