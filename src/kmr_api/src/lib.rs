use std::time::Duration;

use derive_more::{Add, Div, Mul, Sub};

mod group;
pub use group::GroupView;

mod sensors;

// ---------------------------------------------------------------------------
// Error vocabulary
//
// `StateError` is kmr_api's OWN error — re-exposed, not the core's. `kmr_core`
// stays a sealed private dependency: its error type never appears in a public
// signature. We mirror the variants the user can meaningfully act on and
// convert at the boundary via `From<kmr_core::error::StateError>`, so the user
// can `match` on the cause without ever naming `kmr_core`.
// ---------------------------------------------------------------------------

/// Bare, copyable discriminant of a [`StateError`] — carries no data, so it
/// matches without `{ .. }`. Use `error.kind()` to branch on the cause; read
/// the error's `Display` (or destructure [`StateError`]) for the actual numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateError {
    OutOfRange,
    HistoryValueOutOfRange,
    CriticalValueMissing,
}

/// Error returned by fallible state accessors.
///
/// The data (offending index, depth, bounds) lives here so `Display` can report
/// it — e.g. `"history depth 999 is out of range: maximum depth is 2"`. The
/// library fills these fields when it builds the error; the user never inputs
/// them. To match the cause without touching the numbers, use [`StateError::kind`].
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum PrivateStateError {
    /// A joint/sample index exceeded the number of recorded entries.
    #[error("index {index} is out of range: only {len} entries are available")]
    OutOfRange { index: usize, len: usize },

    /// The requested history depth is deeper than the ring records.
    #[error("history depth {provided_depth} is out of range: maximum depth is {max_depth}")]
    HistoryValueOutOfRange {
        provided_depth: usize,
        max_depth: usize,
    },

    /// A required value was not available.
    #[error("value requested is unavailable")]
    CriticalValueMissing,
}

impl PrivateStateError {
    /// The bare cause, free of payload — match this when you only need to know
    /// *what* went wrong, not the specific numbers.
    pub fn kind(&self) -> StateError {
        match self {
            PrivateStateError::OutOfRange { .. } => StateError::OutOfRange,
            PrivateStateError::HistoryValueOutOfRange { .. } => StateError::HistoryValueOutOfRange,
            PrivateStateError::CriticalValueMissing => StateError::CriticalValueMissing,
        }
    }
}

impl From<kmr_core::error::StateError> for PrivateStateError {
    fn from(e: kmr_core::error::StateError) -> Self {
        use kmr_core::error::StateError as Core;
        match e {
            Core::OutOfRange { index, len } => PrivateStateError::OutOfRange { index, len },
            Core::HistoryValueOutOfRange {
                provided_depth,
                max_depth,
            } => PrivateStateError::HistoryValueOutOfRange {
                provided_depth,
                max_depth,
            },
            Core::CriticalValueMissing => PrivateStateError::CriticalValueMissing,
        }
    }
}

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

mod sealed {
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

pub trait FakeSchedule {}
trait FakeSensor {}
trait FakeModel {}

pub struct Initialization;
impl FakeSchedule for Initialization {}
pub struct PerTick;
impl FakeSchedule for PerTick {}

#[derive(Default)]
enum PayloadMode {
    #[default]
    Light,
    Medium,
    Heavy,
}

#[derive(Default)]
struct TreePayload {
    flag: bool,
    from_joint: u16, // canonical joint id from which the tree payload is extended
}

impl TreePayload {
    // Assums that if new() is called.. then flag is true by default.
    fn new(flag: bool, from_joint: u16) -> Self {
        Self { flag, from_joint }
    }
}

#[derive(Default)]
pub struct Payload {
    // TODO: force the user to specify all the architecture needs
    tree_payload: TreePayload,
    mode: PayloadMode,
}

impl Payload {
    fn is_tree(&self) -> bool {
        self.tree_payload.flag
    }

    pub fn as_tree(self) -> Self {
        // TODO: The user specifies from where this new payload is registered
        // from.
        // e.g: the user adds a new arm that extends from the left knee of the
        // precompiled model.

        todo!("");
    }

    // payload reference: https://dev.bostondynamics.com/docs/payload/readme
    //
    // These must define which trait will the payload impl
    pub fn as_light_payload(self) -> Self {
        // TODO: The user specified a new mode:
        // Light — Attaching an inert payload to the robot without connecting to the robot’s [hardware interface] or network.
        todo!("");
    }
    pub fn as_medium_payload(self) -> Self {
        // TODO: The user specified a new mode:
        // Medium — The payload connects to a [hardware interface] and uses
        // [the state SoA] provided by the robot.
        todo!("");
    }
    pub fn as_heavy_payload(self) -> Self {
        // TODO: The user specified a new mode:
        // Heavy — A payload that registers and provides standard services that
        // integrate with other components of the robot system, such as the tablet
        // driving interface, or other payloads.
        todo!("");
    }
}

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
/// use kmr_api::RobotState;
///
/// fn leak(robot: RobotState) {
///     // error[E0616]: field `0` of struct `RobotState` is private
///     let _core = robot.0;
/// }
/// ```
pub struct State(kmr_core::RobotState<N>);

impl State {
    pub fn initial_state<T: Field>(&self) -> [T; N] {
        // self.0.home_state();
        todo!("return home state for specigied state field")
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
    pub fn q_at(&self, i: usize) -> Result<Q, PrivateStateError> {
        self.0
            .current_at::<kmr_core::state::Q>(i)
            .map(|v| Q::from_core(*v))
            .map_err(PrivateStateError::from)
    }

    /// `Q` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_q(&self, depth: usize) -> Result<[Q; N], PrivateStateError> {
        self.0
            .prev::<kmr_core::state::Q>(depth)
            .map(|s| s.map(Q::from_core))
            .map_err(PrivateStateError::from)
    }

    /// `Q` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_q_at(&self, i: usize, depth: usize) -> Result<Q, PrivateStateError> {
        self.0
            .prev_at::<kmr_core::state::Q>(depth, i)
            .map(|v| Q::from_core(*v))
            .map_err(PrivateStateError::from)
    }

    /// Stage `Q` for **every** joint, overwriting the whole desired `Q` buffer.
    #[inline]
    pub fn set_all_q(&mut self, values: [Q; N]) {
        self.0.set_all(values.map(Q::to_core));
    }

    /// Stage `Q` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_q_at(&mut self, value: Q, i: usize) -> Result<(), PrivateStateError> {
        self.0
            .set_at(value.to_core(), i)
            .map_err(PrivateStateError::from)
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
    pub fn qd_at(&self, i: usize) -> Result<Qd, PrivateStateError> {
        self.0
            .current_at::<kmr_core::state::Qd>(i)
            .map(|v| Qd::from_core(*v))
            .map_err(PrivateStateError::from)
    }

    /// `Qd` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_qd(&self, depth: usize) -> Result<[Qd; N], PrivateStateError> {
        self.0
            .prev::<kmr_core::state::Qd>(depth)
            .map(|s| s.map(Qd::from_core))
            .map_err(PrivateStateError::from)
    }

    /// `Qd` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_qd_at(&self, i: usize, depth: usize) -> Result<Qd, PrivateStateError> {
        self.0
            .prev_at::<kmr_core::state::Qd>(depth, i)
            .map(|v| Qd::from_core(*v))
            .map_err(PrivateStateError::from)
    }

    /// Stage `Qd` for **every** joint, overwriting the whole desired `Qd` buffer.
    #[inline]
    pub fn set_all_qd(&mut self, values: [Qd; N]) {
        self.0.set_all(values.map(Qd::to_core));
    }

    /// Stage `Qd` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_qd_at(&mut self, value: Qd, i: usize) -> Result<(), PrivateStateError> {
        self.0
            .set_at(value.to_core(), i)
            .map_err(PrivateStateError::from)
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
    pub fn tau_at(&self, i: usize) -> Result<Tau, PrivateStateError> {
        self.0
            .current_at::<kmr_core::state::Tau>(i)
            .map(|v| Tau::from_core(*v))
            .map_err(PrivateStateError::from)
    }

    /// `Tau` for every joint, `depth` steps back (`0` = newest). `Err` if fewer
    /// than `depth + 1` samples recorded.
    #[inline]
    pub fn prev_tau(&self, depth: usize) -> Result<[Tau; N], PrivateStateError> {
        self.0
            .prev::<kmr_core::state::Tau>(depth)
            .map(|s| s.map(Tau::from_core))
            .map_err(PrivateStateError::from)
    }

    /// `Tau` at joint `i`, `depth` steps back (`0` = newest).
    #[inline]
    pub fn prev_tau_at(&self, i: usize, depth: usize) -> Result<Tau, PrivateStateError> {
        self.0
            .prev_at::<kmr_core::state::Tau>(depth, i)
            .map(|v| Tau::from_core(*v))
            .map_err(PrivateStateError::from)
    }

    /// Stage `Tau` for **every** joint, overwriting the whole desired `Tau` buffer.
    #[inline]
    pub fn set_all_tau(&mut self, values: [Tau; N]) {
        self.0.set_all(values.map(Tau::to_core));
    }

    /// Stage `Tau` at joint `i`. `Err` if `i` is out of range.
    #[inline]
    pub fn set_tau_at(&mut self, value: Tau, i: usize) -> Result<(), PrivateStateError> {
        self.0
            .set_at(value.to_core(), i)
            .map_err(PrivateStateError::from)
    }
}

// todo: kmr_core
pub struct Time;
pub struct Sensors(kmr_core::Sensors);

// #[derive(Debug, thiserror::Error)]
// pub enum Error {
//     #[error("state error: {0}")]
//     State(#[from] kmr_core::error::StateError),
// }

#[derive(Default)]
pub struct Robot {}

// The "App" struct. The main user interface.
//
// [`Robot`] is the exposure of all the action we allow the user to execute.
// This goes from configuration to declaring controller modules.
impl Robot {
    // Instanciate a new default robot
    pub fn new() -> Self {
        Self::default()
    }

    pub fn start_time(self, time: u32) -> Self {
        todo!()
    }

    pub fn dt_ms(self, ms: u32) -> Self {
        todo!()
    }

    pub fn dt_us(self, ms: u32) -> Self {
        todo!()
    }

    // todo: consider this... but forces us to add a modifier to a build time
    // const...
    // pub fn history_depth(self, depth: usize) {}

    // pub fn add_sensor<S: FakeSensor>(self, sensor: S) -> Self {
    //     todo!()
    // }
    // pub fn add_joint<S: FakeModel>(self, sensor: S) -> Self {
    //     todo!()
    // }

    // FakeModel
    pub fn register_payload(self, model: Payload) -> Self {
        todo!()
    }

    pub fn add_controller<S, F>(self, schedule: S, f: F) -> Self
    where
        S: FakeSchedule,
        F: FnMut(&Time, &mut State, &Sensors),
    {
        todo!()
    }

    pub fn run(self) {
        // if self.controllers.is_empty() { return Err("No controller defined") }
        todo!()
    }
}
