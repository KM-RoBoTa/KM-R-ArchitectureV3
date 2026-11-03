//! The limit and envelope stage: run each tick after `Last`, before the batch,
//! over the written `Q`, `Qd` and `Tau` values (SRS FR-4.5, FR-13.5).
//!
//! An envelope breach clamps the value and sets the actuator's `envelope`
//! bit (D3 A). A limit breach sets its `joint_limit` bit, then the actuator's
//! [`Policy`] decides. A non-finite value is a breach of both (research R16)
//! and is never sent: its written bit is cleared.

use tracing::{error, warn};

use crate::error::ConfigError;
use crate::hal::{Faults, Mask, Written};
use crate::signal::SignalMeta;
use crate::state::{Q, Qd, Tau};
use crate::{Signal, SignalKind};

/// The limit of one actuator, from the robot's model: position bounds, speed
/// and effort (SRS FR-4.5). Typed, never a bare `f32`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limit {
    /// Lowest position; `None`: no bound (a continuous joint), never 0.
    pub lower: Option<Q>,
    /// Highest position; `None`: no bound (a continuous joint), never 0.
    pub upper: Option<Q>,
    /// Highest speed, in both directions.
    pub speed: Qd,
    /// Highest effort, in both directions.
    pub effort: Tau,
}

impl Limit {
    /// Bounds nothing: an actuator with it needs no [`Policy`].
    pub const NONE: Self = Self {
        lower: None,
        upper: None,
        speed: Qd(f32::INFINITY),
        effort: Tau(f32::INFINITY),
    };

    /// Whether the limit bounds anything.
    pub fn bounds_anything(&self) -> bool {
        *self != Self::NONE
    }
}

impl Default for Limit {
    fn default() -> Self {
        Self::NONE
    }
}

/// What a limit breach does, per actuator (SRS FR-4.5). No default: an
/// actuator with a limit and no policy does not run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Policy {
    /// The value is clamped to the bound and sent.
    Clamp,
    /// The value is sent as written; the first breach of each actuator is
    /// logged.
    Warn,
    /// An emergency stop: nothing is written that tick, then torque off.
    Abort,
}

/// Bounds the control engineer sets per actuator, inside its limit; a breach
/// clamps (SRS FR-13.5). `None`: that field is not bounded.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Envelope {
    /// Position bounds, `(lower, upper)`.
    pub q: Option<(Q, Q)>,
    /// Velocity bounds, `(lower, upper)`.
    pub qd: Option<(Qd, Qd)>,
    /// Effort bounds, `(lower, upper)`.
    pub tau: Option<(Tau, Tau)>,
}

impl Envelope {
    /// Bounds nothing.
    pub const NONE: Self = Self {
        q: None,
        qd: None,
        tau: None,
    };
}

/// The limits, policies and envelopes of a robot, set on the builder.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Limits<const N: usize> {
    pub(crate) limit: [Limit; N],
    pub(crate) policy: [Option<Policy>; N],
    pub(crate) envelope: [Envelope; N],
    /// Actuators whose `Warn` breach was logged this run.
    warned: Mask<N>,
}

impl<const N: usize> Default for Limits<N> {
    fn default() -> Self {
        Self {
            limit: [Limit::NONE; N],
            policy: [None; N],
            envelope: [Envelope::NONE; N],
            warned: Mask::NONE,
        }
    }
}

/// The written values the stage checks, lent by the state.
pub(crate) struct Staged<'a, const N: usize> {
    pub(crate) q: &'a mut [Q; N],
    pub(crate) qd: &'a mut [Qd; N],
    pub(crate) tau: &'a mut [Tau; N],
    pub(crate) written: &'a mut Written<N>,
}

/// Refuses an actuator whose limit bounds something but has no policy, at
/// finalization, before any I/O (SRS FR-4.5, FR-6.4).
// DECISION: D2 B (assumed, pending human): this is the one place a missing
// policy is refused.
pub(crate) fn require_policies<const N: usize>(limits: &Limits<N>) -> Result<(), ConfigError> {
    let missing = limits
        .limit
        .iter()
        .zip(&limits.policy)
        .position(|(limit, policy)| limit.bounds_anything() && policy.is_none());
    match missing {
        Some(actuator) => Err(ConfigError::NoPolicy { actuator }),
        None => Ok(()),
    }
}

/// The breach of one value.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Breach {
    None,
    /// Outside its bounds: clamped or kept.
    Outside,
    /// Not finite: never sent.
    NotFinite,
}

/// Checks `value` against `[lo, hi]`; a NaN bound bounds nothing.
fn check(value: f32, lo: f32, hi: f32) -> (Breach, f32) {
    let lo = if lo.is_nan() { f32::NEG_INFINITY } else { lo };
    let hi = if hi.is_nan() { f32::INFINITY } else { hi };
    if !value.is_finite() {
        (Breach::NotFinite, value)
    } else if value < lo || value > hi {
        (Breach::Outside, value.max(lo).min(hi))
    } else {
        (Breach::None, value)
    }
}

/// A value type over one `f32`, with its bounds in a [`Limit`] and an
/// [`Envelope`].
trait Value: Copy {
    /// The `f32`.
    fn get(self) -> f32;
    /// The value of `raw`.
    fn of(raw: f32) -> Self;
    /// Its bounds in `limit`.
    fn bounds(limit: &Limit) -> (f32, f32);
    /// Its bounds in `envelope`, if any.
    fn envelope(envelope: &Envelope) -> Option<(f32, f32)>;
}

impl Value for Q {
    fn get(self) -> f32 {
        self.0
    }
    fn of(raw: f32) -> Self {
        Self(raw)
    }
    fn bounds(limit: &Limit) -> (f32, f32) {
        (
            limit.lower.map_or(f32::NEG_INFINITY, |q| q.0),
            limit.upper.map_or(f32::INFINITY, |q| q.0),
        )
    }
    fn envelope(envelope: &Envelope) -> Option<(f32, f32)> {
        envelope.q.map(|(lo, hi)| (lo.0, hi.0))
    }
}

impl Value for Qd {
    fn get(self) -> f32 {
        self.0
    }
    fn of(raw: f32) -> Self {
        Self(raw)
    }
    fn bounds(limit: &Limit) -> (f32, f32) {
        (-limit.speed.0.abs(), limit.speed.0.abs())
    }
    fn envelope(envelope: &Envelope) -> Option<(f32, f32)> {
        envelope.qd.map(|(lo, hi)| (lo.0, hi.0))
    }
}

impl Value for Tau {
    fn get(self) -> f32 {
        self.0
    }
    fn of(raw: f32) -> Self {
        Self(raw)
    }
    fn bounds(limit: &Limit) -> (f32, f32) {
        (-limit.effort.0.abs(), limit.effort.0.abs())
    }
    fn envelope(envelope: &Envelope) -> Option<(f32, f32)> {
        envelope.tau.map(|(lo, hi)| (lo.0, hi.0))
    }
}

impl<const N: usize> Limits<N> {
    /// Sets the limits and their policies.
    pub(crate) fn set_limits(&mut self, limit: [Limit; N], policy: [Option<Policy>; N]) {
        self.limit = limit;
        self.policy = policy;
    }

    /// Sets the envelopes.
    pub(crate) fn set_envelopes(&mut self, envelope: [Envelope; N]) {
        self.envelope = envelope;
    }

    /// The stage: clamps to the envelopes, checks the limits, sets the fault
    /// bits. Returns the emergency stop of an `Abort` breach; the tick then
    /// writes nothing.
    pub(crate) fn enforce(
        &mut self,
        staged: Staged<'_, N>,
        faults: &mut Faults<N>,
    ) -> Option<Signal> {
        let Staged {
            q,
            qd,
            tau,
            written,
        } = staged;
        let abort = [
            self.enforce_field(q, &mut written.q, faults),
            self.enforce_field(qd, &mut written.qd, faults),
            self.enforce_field(tau, &mut written.tau, faults),
        ];
        let actuator = abort.into_iter().flatten().min()?;
        error!(
            actuator,
            "joint limit breached under Policy::Abort: emergency stop"
        );
        Some(Signal {
            kind: SignalKind::EmergencyStop,
            meta: SignalMeta::new(
                "JointLimit: a written value breached its actuator's limit (SRS FR-4.5)",
                file!(),
                line!(),
            ),
        })
    }

    /// The stage over one field; the first actuator whose breach aborts.
    fn enforce_field<T: Value>(
        &mut self,
        values: &mut [T; N],
        written: &mut Mask<N>,
        faults: &mut Faults<N>,
    ) -> Option<usize> {
        let mut abort = None;
        let candidates = *written;
        for actuator in candidates.iter() {
            let Some(value) = values.get_mut(actuator) else {
                continue;
            };
            if let Some((lo, hi)) = self.envelope.get(actuator).and_then(T::envelope) {
                let (breach, clamped) = check(value.get(), lo, hi);
                if breach != Breach::None {
                    faults.envelope.set(actuator);
                }
                match breach {
                    Breach::None => {}
                    Breach::Outside => *value = T::of(clamped),
                    Breach::NotFinite => {
                        written.clear(actuator);
                        continue;
                    }
                }
            }
            let (Some(limit), Some(Some(policy))) =
                (self.limit.get(actuator), self.policy.get(actuator))
            else {
                continue;
            };
            let (lo, hi) = T::bounds(limit);
            let (breach, clamped) = check(value.get(), lo, hi);
            if breach == Breach::None {
                continue;
            }
            faults.joint_limit.set(actuator);
            if breach == Breach::NotFinite {
                written.clear(actuator);
            }
            match policy {
                Policy::Clamp => *value = T::of(clamped),
                Policy::Warn => {
                    if !self.warned.get(actuator) {
                        self.warned.set(actuator);
                        warn!(
                            actuator,
                            "a written value breached its actuator's limit; Policy::Warn sends it"
                        );
                    }
                }
                Policy::Abort => {
                    abort.get_or_insert(actuator);
                }
            }
        }
        abort
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::N;

    /// Desired values and masks a test writes.
    struct Desired {
        q: [Q; N],
        qd: [Qd; N],
        tau: [Tau; N],
        written: Written<N>,
    }

    impl Desired {
        fn q(index: usize, value: f32) -> Self {
            let mut desired = Self {
                q: [Q(0.0); N],
                qd: [Qd(0.0); N],
                tau: [Tau(0.0); N],
                written: Written::default(),
            };
            desired.q[index] = Q(value);
            desired.written.q.set(index);
            desired
        }

        fn enforce(&mut self, limits: &mut Limits<N>) -> (Option<Signal>, Faults<N>) {
            let mut faults = Faults::default();
            let staged = Staged {
                q: &mut self.q,
                qd: &mut self.qd,
                tau: &mut self.tau,
                written: &mut self.written,
            };
            (limits.enforce(staged, &mut faults), faults)
        }
    }

    const ARM: Limit = Limit {
        lower: Some(Q(-1.0)),
        upper: Some(Q(1.0)),
        speed: Qd(2.0),
        effort: Tau(3.0),
    };

    fn limits(policy: Policy) -> Limits<N> {
        let mut limits = Limits::default();
        limits.set_limits([ARM; N], [Some(policy); N]);
        limits
    }

    #[test]
    fn clamp_sends_the_bound_and_sets_joint_limit() {
        let mut desired = Desired::q(1, 5.0);
        let (signal, faults) = desired.enforce(&mut limits(Policy::Clamp));
        assert!(signal.is_none());
        assert_eq!(desired.q[1], Q(1.0));
        assert!(desired.written.q.get(1));
        assert!(faults.joint_limit.get(1) && !faults.joint_limit.get(0));
    }

    #[test]
    fn warn_sends_the_value_as_written() {
        let mut desired = Desired::q(0, -5.0);
        let (signal, faults) = desired.enforce(&mut limits(Policy::Warn));
        assert!(signal.is_none());
        assert_eq!(desired.q[0], Q(-5.0));
        assert!(faults.joint_limit.get(0));
    }

    #[test]
    fn abort_is_an_emergency_stop() {
        let mut desired = Desired::q(2, 1.5);
        let (signal, faults) = desired.enforce(&mut limits(Policy::Abort));
        assert_eq!(signal.map(|s| s.kind()), Some(SignalKind::EmergencyStop));
        assert!(faults.joint_limit.get(2));
    }

    #[test]
    fn a_value_inside_the_limit_is_untouched() {
        let mut desired = Desired::q(1, 0.5);
        let (signal, faults) = desired.enforce(&mut limits(Policy::Abort));
        assert!(signal.is_none() && faults.is_empty());
        assert_eq!(desired.q[1], Q(0.5));
    }

    #[test]
    fn a_continuous_actuator_has_no_position_bound_but_keeps_speed_and_effort() {
        let mut limits = Limits::default();
        let continuous = Limit {
            lower: None,
            upper: None,
            ..ARM
        };
        limits.set_limits([continuous; N], [Some(Policy::Clamp); N]);
        let mut desired = Desired::q(0, 10.0);
        desired.qd[0] = Qd(5.0);
        desired.written.qd.set(0);

        let (_, faults) = desired.enforce(&mut limits);

        assert_eq!(desired.q[0], Q(10.0));
        assert_eq!(desired.qd[0], Qd(2.0));
        assert!(faults.joint_limit.get(0));
    }

    #[test]
    fn a_non_finite_value_is_a_breach_and_never_sent() {
        for value in [f32::NAN, f32::INFINITY] {
            let mut desired = Desired::q(3, value);
            let (signal, faults) = desired.enforce(&mut limits(Policy::Clamp));
            assert!(signal.is_none());
            assert!(!desired.written.q.get(3));
            assert!(faults.joint_limit.get(3));
        }
    }

    #[test]
    fn an_envelope_breach_clamps_and_sets_envelope_only() {
        let mut limits = Limits::default();
        let mut envelopes = [Envelope::NONE; N];
        envelopes[1].tau = Some((Tau(-0.5), Tau(0.5)));
        limits.set_envelopes(envelopes);
        let mut desired = Desired::q(1, 0.0);
        desired.tau[1] = Tau(2.0);
        desired.written.tau.set(1);

        let (signal, faults) = desired.enforce(&mut limits);

        assert!(signal.is_none());
        assert_eq!(desired.tau[1], Tau(0.5));
        assert!(faults.envelope.get(1) && !faults.envelope.get(0));
        assert!(faults.joint_limit.is_empty());
    }

    #[test]
    fn unwritten_values_are_not_checked() {
        let mut desired = Desired::q(0, 0.0);
        desired.q[1] = Q(f32::NAN);
        let (signal, faults) = desired.enforce(&mut limits(Policy::Abort));
        assert!(signal.is_none() && faults.is_empty());
    }

    #[test]
    fn a_limited_actuator_without_a_policy_is_refused() {
        let mut limits = Limits::<N>::default();
        let mut policies = [Some(Policy::Clamp); N];
        policies[2] = None;
        limits.set_limits([ARM; N], policies);
        assert_eq!(
            require_policies(&limits),
            Err(ConfigError::NoPolicy { actuator: 2 })
        );

        let mut free = [ARM; N];
        free[2] = Limit::NONE;
        limits.set_limits(free, policies);
        assert_eq!(require_policies(&limits), Ok(()));
        assert_eq!(require_policies(&Limits::<N>::default()), Ok(()));
    }
}
