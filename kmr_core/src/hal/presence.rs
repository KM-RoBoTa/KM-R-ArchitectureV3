//! The two opt-in policies on declared actuators that do not answer: missing
//! at open or lost at run time, and joining while the loop runs. Both are off
//! by default; the adapter applies them, the engine only forwards them.

/// What a declared actuator that does not answer means (policy (a)).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MissingPolicy {
    /// A silent actuator fails bring-up, as without the policy.
    #[default]
    Fail,
    /// A silent actuator, at open or at run time, is absent: neither an
    /// error nor an emergency stop. The adapter logs it once.
    Tolerate,
}

/// Whether a declared actuator absent from the run may join it (policy (b)).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum JoinPolicy {
    /// An absent actuator stays absent for the run.
    #[default]
    Never,
    /// An absent declared actuator is probed quietly; once it answers it is
    /// set up like the others and then driven.
    Declared,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::Overrides;

    #[test]
    fn policies_default_to_fail_and_never() {
        assert_eq!(MissingPolicy::default(), MissingPolicy::Fail);
        assert_eq!(JoinPolicy::default(), JoinPolicy::Never);
        assert_eq!(Overrides::default().missing, MissingPolicy::Fail);
        assert_eq!(Overrides::default().join, JoinPolicy::Never);
    }
}
