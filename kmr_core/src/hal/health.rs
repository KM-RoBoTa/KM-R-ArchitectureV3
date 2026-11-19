//! The opt-in policy on a declared actuator whose bring-up health check
//! fails. Off by default; the adapter applies it, the engine only forwards it.

/// What an actuator whose bring-up health check fails means.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HealthPolicy {
    /// A failed health check fails bring-up, as without the policy.
    #[default]
    Fail,
    /// A failed health check is logged once and the actuator is set up and
    /// driven like the others.
    Warn,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::Overrides;

    #[test]
    fn health_policy_defaults_to_fail() {
        assert_eq!(HealthPolicy::default(), HealthPolicy::Fail);
        assert_eq!(Overrides::default().health, HealthPolicy::Fail);
    }
}
