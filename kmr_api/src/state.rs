use crate::StateError;

/// Number of actuators.
pub const N: usize = kmr_core::state::JOINTS;
/// Ticks of history kept: every `prev_*` depth is below it.
pub use kmr_core::state::HISTORY_DEPTH;

// ---------------------------------------------------------------------------
// Field vocabulary (principle #2 + #3)
//
// `Q` / `Qd` / `Tau` / `Kp` / `Kd` are defined ONCE, in `kmr_core`, and re-exported here.
// An api-side copy would need a conversion at every accessor and a second
// place to keep in sync; since the core is open source there is nothing left
// to hide that would pay for that. Their arithmetic is derived in the core
// too — the orphan rule does not let this crate add it.
// ---------------------------------------------------------------------------
pub use kmr_core::state::{Kd, Kp, Q, Qd, Tau};

/// A writable/readable state field: `Q`, `Qd`, `Tau`, `Kp` or `Kd`.
///
/// This is the core's `StateField` under the name users already know. It is
/// sealed in the core (its supertrait lives in a crate-private module), so
/// downstream crates can name it as a bound but cannot add a field type.
pub use kmr_core::state::StateField as Field;

/// API boundary over the core robot state.
///
/// The inner core handle is private. User code lives in a separate crate, so
/// the `inner` field is unreachable — state is only touched through the getters
/// and setters defined below.
///
/// This guarantee is pinned: the following must NOT compile (`E0616` = private
/// field). If a refactor ever exposes the inner type, this doc-test breaks.
///
/// ```compile_fail,E0616
/// fn leak(robot: kmr_api::State) {
///     // error[E0616]: field `inner` of struct `State` is private
///     let _core = robot.inner;
/// }
/// ```
pub struct State<'a> {
    /// The core state the runtime lends for one tick.
    inner: &'a mut kmr_core::RobotState,
}

impl<'a> State<'a> {
    /// Wrap the borrowed core state the runtime lends for one tick. Called only
    /// at the api→core boundary (`ApiInline::call`); never by users.
    pub(crate) fn new(inner: &'a mut kmr_core::RobotState) -> Self {
        State { inner }
    }

    /// A shorter-lived `State` over the same core state.
    pub(crate) fn reborrow(&mut self) -> State<'_> {
        State { inner: self.inner }
    }

    // ---------------------------------------------------------------------------
    // State accessors
    //
    // Every method returns **owned** values, copied out of the references the
    // core hands back: the fields are `Copy`, and an owned `Q` spares beginner
    // users a borrow of `State` that would block the next setter call.
    //
    // Each field exposes the same
    // shape — `<field>()` / `<field>_at(i)` for the newest sample, the `prev_`
    // variants for `depth` steps back (`0` = newest), and `set_all_<field>` /
    // `set_<field>_at` to stage desired values — plus human-friendly aliases
    // (`position`, `velocity`, `torque`, `effort`).
    // ---------------------------------------------------------------------------

    // ---- Q (position) ----

    /// Newest sensed `Q` for every actuator. `None` until the first sample is recorded.
    #[inline]
    pub fn q(&self) -> Option<[Q; N]> {
        self.inner.current::<Q>().copied()
    }

    /// Alias of [`RobotState::q`].
    #[inline]
    pub fn position(&self) -> Option<[Q; N]> {
        self.q()
    }

    /// Newest `Q` at actuator `i`.
    #[inline]
    pub fn q_at(&self, i: usize) -> Result<Q, StateError> {
        self.inner
            .current_at::<Q>(i)
            .copied()
            .map_err(StateError::from)
    }

    /// `Q` for every actuator, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_q(&self, depth: usize) -> Result<[Q; N], StateError> {
        self.inner
            .prev::<Q>(depth)
            .copied()
            .map_err(StateError::from)
    }

    /// `Q` at actuator `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_q_at(&self, i: usize, depth: usize) -> Result<Q, StateError> {
        self.inner
            .prev_at::<Q>(depth, i)
            .copied()
            .map_err(StateError::from)
    }

    /// Stage `Q` for **every** actuator, overwriting the whole desired `Q` buffer.
    #[inline]
    pub fn set_all_q(&mut self, values: [Q; N]) {
        self.inner.set_all(values);
    }

    /// Stage `Q` at actuator `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_q_at(&mut self, value: Q, i: usize) -> Result<(), StateError> {
        self.inner.set_at(value, i).map_err(StateError::from)
    }

    // ---- Qd (velocity) ----

    /// Newest sensed `Qd` for every actuator. `None` until the first sample is recorded.
    #[inline]
    pub fn qd(&self) -> Option<[Qd; N]> {
        self.inner.current::<Qd>().copied()
    }

    /// Alias of [`RobotState::qd`].
    #[inline]
    pub fn velocity(&self) -> Option<[Qd; N]> {
        self.qd()
    }

    /// Newest `Qd` at actuator `i`.
    #[inline]
    pub fn qd_at(&self, i: usize) -> Result<Qd, StateError> {
        self.inner
            .current_at::<Qd>(i)
            .copied()
            .map_err(StateError::from)
    }

    /// `Qd` for every actuator, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_qd(&self, depth: usize) -> Result<[Qd; N], StateError> {
        self.inner
            .prev::<Qd>(depth)
            .copied()
            .map_err(StateError::from)
    }

    /// `Qd` at actuator `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_qd_at(&self, i: usize, depth: usize) -> Result<Qd, StateError> {
        self.inner
            .prev_at::<Qd>(depth, i)
            .copied()
            .map_err(StateError::from)
    }

    /// Stage `Qd` for **every** actuator, overwriting the whole desired `Qd` buffer.
    #[inline]
    pub fn set_all_qd(&mut self, values: [Qd; N]) {
        self.inner.set_all(values);
    }

    /// Stage `Qd` at actuator `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_qd_at(&mut self, value: Qd, i: usize) -> Result<(), StateError> {
        self.inner.set_at(value, i).map_err(StateError::from)
    }

    // ---- Tau (torque / effort) ----

    /// Newest sensed `Tau` for every actuator. `None` until the first sample is recorded.
    #[inline]
    pub fn tau(&self) -> Option<[Tau; N]> {
        self.inner.current::<Tau>().copied()
    }

    /// Alias of [`RobotState::tau`].
    #[inline]
    pub fn torque(&self) -> Option<[Tau; N]> {
        self.tau()
    }

    /// Alias of [`RobotState::tau`].
    #[inline]
    pub fn effort(&self) -> Option<[Tau; N]> {
        self.tau()
    }

    /// Newest `Tau` at actuator `i`.
    #[inline]
    pub fn tau_at(&self, i: usize) -> Result<Tau, StateError> {
        self.inner
            .current_at::<Tau>(i)
            .copied()
            .map_err(StateError::from)
    }

    /// `Tau` for every actuator, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_tau(&self, depth: usize) -> Result<[Tau; N], StateError> {
        self.inner
            .prev::<Tau>(depth)
            .copied()
            .map_err(StateError::from)
    }

    /// `Tau` at actuator `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_tau_at(&self, i: usize, depth: usize) -> Result<Tau, StateError> {
        self.inner
            .prev_at::<Tau>(depth, i)
            .copied()
            .map_err(StateError::from)
    }

    /// Stage `Tau` for **every** actuator, overwriting the whole desired `Tau` buffer.
    #[inline]
    pub fn set_all_tau(&mut self, values: [Tau; N]) {
        self.inner.set_all(values);
    }

    /// Stage `Tau` at actuator `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_tau_at(&mut self, value: Tau, i: usize) -> Result<(), StateError> {
        self.inner.set_at(value, i).map_err(StateError::from)
    }

    // ---- Kp (a position gain) ----

    /// Newest `Kp` for every actuator: the gains the adapter last applied, zero
    /// where an actuator has none. `None` until the first sample is recorded.
    #[inline]
    pub fn kp(&self) -> Option<[Kp; N]> {
        self.inner.current::<Kp>().copied()
    }

    /// Newest `Kp` at actuator `i`.
    #[inline]
    pub fn kp_at(&self, i: usize) -> Result<Kp, StateError> {
        self.inner
            .current_at::<Kp>(i)
            .copied()
            .map_err(StateError::from)
    }

    /// `Kp` for every actuator, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_kp(&self, depth: usize) -> Result<[Kp; N], StateError> {
        self.inner
            .prev::<Kp>(depth)
            .copied()
            .map_err(StateError::from)
    }

    /// `Kp` at actuator `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_kp_at(&self, i: usize, depth: usize) -> Result<Kp, StateError> {
        self.inner
            .prev_at::<Kp>(depth, i)
            .copied()
            .map_err(StateError::from)
    }

    /// Stage `Kp` for **every** actuator, overwriting the whole desired `Kp` buffer.
    /// Actuators without a position gain ignore it.
    #[inline]
    pub fn set_all_kp(&mut self, values: [Kp; N]) {
        self.inner.set_all(values);
    }

    /// Stage `Kp` at actuator `i`. `Err` if `i` is out of range. Actuators without
    /// a position gain ignore it.
    #[inline]
    pub fn set_kp_at(&mut self, value: Kp, i: usize) -> Result<(), StateError> {
        self.inner.set_at(value, i).map_err(StateError::from)
    }

    // ---- Kd (a velocity gain) ----

    /// Newest `Kd` for every actuator: the gains the adapter last applied, zero
    /// where an actuator has none. `None` until the first sample is recorded.
    #[inline]
    pub fn kd(&self) -> Option<[Kd; N]> {
        self.inner.current::<Kd>().copied()
    }

    /// Newest `Kd` at actuator `i`.
    #[inline]
    pub fn kd_at(&self, i: usize) -> Result<Kd, StateError> {
        self.inner
            .current_at::<Kd>(i)
            .copied()
            .map_err(StateError::from)
    }

    /// `Kd` for every actuator, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_kd(&self, depth: usize) -> Result<[Kd; N], StateError> {
        self.inner
            .prev::<Kd>(depth)
            .copied()
            .map_err(StateError::from)
    }

    /// `Kd` at actuator `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_kd_at(&self, i: usize, depth: usize) -> Result<Kd, StateError> {
        self.inner
            .prev_at::<Kd>(depth, i)
            .copied()
            .map_err(StateError::from)
    }

    /// Stage `Kd` for **every** actuator, overwriting the whole desired `Kd` buffer.
    /// Actuators without a velocity gain ignore it.
    #[inline]
    pub fn set_all_kd(&mut self, values: [Kd; N]) {
        self.inner.set_all(values);
    }

    /// Stage `Kd` at actuator `i`. `Err` if `i` is out of range. Actuators without
    /// a velocity gain ignore it.
    #[inline]
    pub fn set_kd_at(&mut self, value: Kd, i: usize) -> Result<(), StateError> {
        self.inner.set_at(value, i).map_err(StateError::from)
    }
}
