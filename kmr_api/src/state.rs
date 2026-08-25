use crate::StateError;
use derive_more::{Add, Div, Mul, Sub};
/// Number of actuators. Re-exposed as a value (not a path into `kmr_core`) so
/// the core stays a private dependency.
pub const N: usize = kmr_core::state::JOINTS;

// ---------------------------------------------------------------------------
// Field vocabulary (principle #2 + #3)
//
// `Q` / `Qd` / `Tau` are kmr_api's OWN types — they are the precise, intent-
// carrying vocabulary the user speaks in. They are NOT kmr_core's types: the
// core keeps its own internal copies, and we convert at this boundary. That
// way `kmr_core` never appears in a public signature and stays fully sealed,
// while the user still gets type-checked position/velocity/torque.
// ---------------------------------------------------------------------------

/// Desired / sensed joint position.
#[derive(Clone, Copy, Debug, Default, PartialEq, Add, Sub, Mul, Div)]
pub struct Q(pub f32);
impl From<f32> for Q {
    fn from(x: f32) -> Q {
        Q(x)
    }
}
/// Desired / sensed joint velocity.
#[derive(Clone, Copy, Debug, Default, PartialEq, Add, Sub, Mul, Div)]
pub struct Qd(pub f32);
impl From<f32> for Qd {
    fn from(x: f32) -> Qd {
        Qd(x)
    }
}
/// Desired / sensed joint torque.
#[derive(Clone, Copy, Debug, Default, PartialEq, Add, Sub, Mul, Div)]
pub struct Tau(pub f32);
impl From<f32> for Tau {
    fn from(x: f32) -> Tau {
        Tau(x)
    }
}

/// A writable/readable state field. Sealed: only `Q`, `Qd`, `Tau` implement it,
/// and downstream crates cannot add their own (the supertrait lives in a
/// private module). This is what lets the generic setters/getters stay generic
/// without opening the field set.
pub trait Field: sealed::FieldConv {}

pub mod sealed {
    /// Bridges a public api field type to its private `kmr_core` counterpart.
    /// Private module ⇒ external crates can neither name nor implement it.
    pub trait FieldConv: Copy {
        type Core: kmr_core::state::StateField + Copy;
        fn to_core(self) -> Self::Core;
        fn from_core(core: Self::Core) -> Self;
    }
}
use sealed::FieldConv;

impl FieldConv for Q {
    type Core = kmr_core::state::Q;
    fn to_core(self) -> Self::Core {
        kmr_core::state::Q(self.0)
    }
    fn from_core(core: Self::Core) -> Self {
        Q(core.0)
    }
}
impl Field for Q {}

impl FieldConv for Qd {
    type Core = kmr_core::state::Qd;
    fn to_core(self) -> Self::Core {
        kmr_core::state::Qd(self.0)
    }
    fn from_core(core: Self::Core) -> Self {
        Qd(core.0)
    }
}
impl Field for Qd {}

impl FieldConv for Tau {
    type Core = kmr_core::state::Tau;
    fn to_core(self) -> Self::Core {
        kmr_core::state::Tau(self.0)
    }
    fn from_core(core: Self::Core) -> Self {
        Tau(core.0)
    }
}
impl Field for Tau {}

/// Vendor boundary over the core robot state.
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
    // Every method returns **owned** api values: this boundary converts
    // `kmr_core` → api types via [`FieldConv`] (`to_core` / `from_core`), so
    // borrowing the converted value is impossible. Each field exposes the same
    // shape — `<field>()` / `<field>_at(i)` for the newest sample, the `prev_`
    // variants for `depth` steps back (`0` = newest), and `set_all_<field>` /
    // `set_<field>_at` to stage desired values — plus human-friendly aliases
    // (`position`, `velocity`, `torque`, `effort`).
    // ---------------------------------------------------------------------------

    // ---- Q (position) ----

    /// Newest sensed `Q` for every joint. `None` until the first sample is recorded.
    #[inline]
    pub fn q(&self) -> Option<[Q; N]> {
        Some(self.0.current::<kmr_core::state::Q>()?.map(Q::from_core))
    }

    /// Alias of [`RobotState::q`].
    #[inline]
    pub fn position(&self) -> Option<[Q; N]> {
        self.q()
    }

    /// Newest `Q` at joint `i`.
    #[inline]
    pub fn q_at(&self, i: usize) -> Result<Q, StateError> {
        self.0
            .current_at::<kmr_core::state::Q>(i)
            .map(|v| Q::from_core(*v))
            .map_err(StateError::from)
    }

    /// `Q` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_q(&self, depth: usize) -> Result<[Q; N], StateError> {
        self.0
            .prev::<kmr_core::state::Q>(depth)
            .map(|s| s.map(Q::from_core))
            .map_err(StateError::from)
    }

    /// `Q` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_q_at(&self, i: usize, depth: usize) -> Result<Q, StateError> {
        self.0
            .prev_at::<kmr_core::state::Q>(depth, i)
            .map(|v| Q::from_core(*v))
            .map_err(StateError::from)
    }

    /// Stage `Q` for **every** joint, overwriting the whole desired `Q` buffer.
    #[inline]
    pub fn set_all_q(&mut self, values: [Q; N]) {
        self.0.set_all(values.map(Q::to_core));
    }

    /// Special case of `set_all_q` where instead of staging, the robot slowly
    /// goes up with a specified ramping to the home position.
    ///
    /// This method is only meant to be used during the initialization phase.
    #[inline]
    pub fn go_home(&mut self, _ramp: u32) {
        let values = self.initial_state();
        self.0.set_all(values.map(Q::to_core));

        // todo: must check if initialization phase and must take a ramp value
        // of a better though out type (consider u32 as a placeholder)

        todo!()
    }

    /// Stage `Q` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_q_at(&mut self, value: Q, i: usize) -> Result<(), StateError> {
        self.0.set_at(value.to_core(), i).map_err(StateError::from)
    }

    // ---- Qd (velocity) ----

    /// Newest sensed `Qd` for every joint. `None` until the first sample is recorded.
    #[inline]
    pub fn qd(&self) -> Option<[Qd; N]> {
        Some(self.0.current::<kmr_core::state::Qd>()?.map(Qd::from_core))
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
            .current_at::<kmr_core::state::Qd>(i)
            .map(|v| Qd::from_core(*v))
            .map_err(StateError::from)
    }

    /// `Qd` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_qd(&self, depth: usize) -> Result<[Qd; N], StateError> {
        self.0
            .prev::<kmr_core::state::Qd>(depth)
            .map(|s| s.map(Qd::from_core))
            .map_err(StateError::from)
    }

    /// `Qd` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_qd_at(&self, i: usize, depth: usize) -> Result<Qd, StateError> {
        self.0
            .prev_at::<kmr_core::state::Qd>(depth, i)
            .map(|v| Qd::from_core(*v))
            .map_err(StateError::from)
    }

    /// Stage `Qd` for **every** joint, overwriting the whole desired `Qd` buffer.
    #[inline]
    pub fn set_all_qd(&mut self, values: [Qd; N]) {
        self.0.set_all(values.map(Qd::to_core));
    }

    /// Stage `Qd` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_qd_at(&mut self, value: Qd, i: usize) -> Result<(), StateError> {
        self.0.set_at(value.to_core(), i).map_err(StateError::from)
    }

    // ---- Tau (torque / effort) ----

    /// Newest sensed `Tau` for every joint. `None` until the first sample is recorded.
    #[inline]
    pub fn tau(&self) -> Option<[Tau; N]> {
        Some(
            self.0
                .current::<kmr_core::state::Tau>()?
                .map(Tau::from_core),
        )
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
            .current_at::<kmr_core::state::Tau>(i)
            .map(|v| Tau::from_core(*v))
            .map_err(StateError::from)
    }

    /// `Tau` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_tau(&self, depth: usize) -> Result<[Tau; N], StateError> {
        self.0
            .prev::<kmr_core::state::Tau>(depth)
            .map(|s| s.map(Tau::from_core))
            .map_err(StateError::from)
    }

    /// `Tau` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_tau_at(&self, i: usize, depth: usize) -> Result<Tau, StateError> {
        self.0
            .prev_at::<kmr_core::state::Tau>(depth, i)
            .map(|v| Tau::from_core(*v))
            .map_err(StateError::from)
    }

    /// Stage `Tau` for **every** joint, overwriting the whole desired `Tau` buffer.
    #[inline]
    pub fn set_all_tau(&mut self, values: [Tau; N]) {
        self.0.set_all(values.map(Tau::to_core));
    }

    /// Stage `Tau` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_tau_at(&mut self, value: Tau, i: usize) -> Result<(), StateError> {
        self.0.set_at(value.to_core(), i).map_err(StateError::from)
    }
}
