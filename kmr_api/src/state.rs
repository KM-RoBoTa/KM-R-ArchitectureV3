use crate::StateError;

/// Number of actuators.
pub const N: usize = kmr_core::state::JOINTS;

// ---------------------------------------------------------------------------
// Field vocabulary (principle #2 + #3)
//
// `Q` / `Qd` / `Tau` are defined ONCE, in `kmr_core`, and re-exported here.
// An api-side copy would need a conversion at every accessor and a second
// place to keep in sync; since the core is open source there is nothing left
// to hide that would pay for that. Their arithmetic is derived in the core
// too — the orphan rule does not let this crate add it.
// ---------------------------------------------------------------------------
pub use kmr_core::state::{Q, Qd, Tau};

/// A writable/readable state field: `Q`, `Qd` or `Tau`.
///
/// This is the core's `StateField` under the name users already know. It is
/// sealed in the core (its supertrait lives in a crate-private module), so
/// downstream crates can name it as a bound but cannot add a field type.
pub use kmr_core::state::StateField as Field;

/// API boundary over the core robot state.
///
/// The inner core handle is private. User code lives in a separate crate, so
/// the `.0` field is unreachable — state is only touched through the getters
/// and setters defined below.
///
/// This guarantee is pinned: the following must NOT compile (`E0616` = private
/// field). If a refactor ever exposes the inner type, this doc-test breaks.
///
/// ```compile_fail,E0616
/// fn leak(robot: kmr_api::State) {
///     // error[E0616]: field `0` of struct `State` is private
///     let _core = robot.0;
/// }
/// ```
pub struct State<'a>(&'a mut kmr_core::RobotState<N>);

impl<'a> State<'a> {
    /// Wrap the borrowed core state the runtime lends for one tick. Called only
    /// at the api→core boundary (`ApiInline::call`); never by users.
    pub(crate) fn new(inner: &'a mut kmr_core::RobotState<N>) -> Self {
        State(inner)
    }

    pub fn initial_state<T: Field>(&self) -> [T; N] {
        todo!("return home state for specified state field");
        // self.0.home_state()
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

    /// Newest sensed `Q` for every joint. `None` until the first sample is recorded.
    #[inline]
    pub fn q(&self) -> Option<[Q; N]> {
        self.0.current::<Q>().copied()
    }

    /// Alias of [`RobotState::q`].
    #[inline]
    pub fn position(&self) -> Option<[Q; N]> {
        self.q()
    }

    /// Newest `Q` at joint `i`.
    #[inline]
    pub fn q_at(&self, i: usize) -> Result<Q, StateError> {
        self.0.current_at::<Q>(i).copied().map_err(StateError::from)
    }

    /// `Q` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_q(&self, depth: usize) -> Result<[Q; N], StateError> {
        self.0.prev::<Q>(depth).copied().map_err(StateError::from)
    }

    /// `Q` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_q_at(&self, i: usize, depth: usize) -> Result<Q, StateError> {
        self.0
            .prev_at::<Q>(depth, i)
            .copied()
            .map_err(StateError::from)
    }

    /// Stage `Q` for **every** joint, overwriting the whole desired `Q` buffer.
    #[inline]
    pub fn set_all_q(&mut self, values: [Q; N]) {
        self.0.set_all(values);
    }

    /// Special case of `set_all_q` where instead of staging, the robot slowly
    /// goes up with a specified ramping to the home position.
    ///
    /// This method is only meant to be used during the initialization phase.
    #[inline]
    pub fn go_home(&mut self, _ramp: u32) {
        let values: [Q; N] = self.initial_state();
        self.0.set_all(values);

        // todo: must check if initialization phase and must take a ramp value
        // of a better though out type (consider u32 as a placeholder)

        todo!()
    }

    /// Stage `Q` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_q_at(&mut self, value: Q, i: usize) -> Result<(), StateError> {
        self.0.set_at(value, i).map_err(StateError::from)
    }

    // ---- Qd (velocity) ----

    /// Newest sensed `Qd` for every joint. `None` until the first sample is recorded.
    #[inline]
    pub fn qd(&self) -> Option<[Qd; N]> {
        self.0.current::<Qd>().copied()
    }

    /// Alias of [`RobotState::qd`].
    #[inline]
    pub fn velocity(&self) -> Option<[Qd; N]> {
        self.qd()
    }

    /// Newest `Qd` at joint `i`.
    #[inline]
    pub fn qd_at(&self, i: usize) -> Result<Qd, StateError> {
        self.0
            .current_at::<Qd>(i)
            .copied()
            .map_err(StateError::from)
    }

    /// `Qd` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_qd(&self, depth: usize) -> Result<[Qd; N], StateError> {
        self.0.prev::<Qd>(depth).copied().map_err(StateError::from)
    }

    /// `Qd` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_qd_at(&self, i: usize, depth: usize) -> Result<Qd, StateError> {
        self.0
            .prev_at::<Qd>(depth, i)
            .copied()
            .map_err(StateError::from)
    }

    /// Stage `Qd` for **every** joint, overwriting the whole desired `Qd` buffer.
    #[inline]
    pub fn set_all_qd(&mut self, values: [Qd; N]) {
        self.0.set_all(values);
    }

    /// Stage `Qd` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_qd_at(&mut self, value: Qd, i: usize) -> Result<(), StateError> {
        self.0.set_at(value, i).map_err(StateError::from)
    }

    // ---- Tau (torque / effort) ----

    /// Newest sensed `Tau` for every joint. `None` until the first sample is recorded.
    #[inline]
    pub fn tau(&self) -> Option<[Tau; N]> {
        self.0.current::<Tau>().copied()
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

    /// Newest `Tau` at joint `i`.
    #[inline]
    pub fn tau_at(&self, i: usize) -> Result<Tau, StateError> {
        self.0
            .current_at::<Tau>(i)
            .copied()
            .map_err(StateError::from)
    }

    /// `Tau` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_tau(&self, depth: usize) -> Result<[Tau; N], StateError> {
        self.0.prev::<Tau>(depth).copied().map_err(StateError::from)
    }

    /// `Tau` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_tau_at(&self, i: usize, depth: usize) -> Result<Tau, StateError> {
        self.0
            .prev_at::<Tau>(depth, i)
            .copied()
            .map_err(StateError::from)
    }

    /// Stage `Tau` for **every** joint, overwriting the whole desired `Tau` buffer.
    #[inline]
    pub fn set_all_tau(&mut self, values: [Tau; N]) {
        self.0.set_all(values);
    }

    /// Stage `Tau` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_tau_at(&mut self, value: Tau, i: usize) -> Result<(), StateError> {
        self.0.set_at(value, i).map_err(StateError::from)
    }
}
