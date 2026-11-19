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
    use core::convert::Infallible;

    use super::*;
    use crate::hal::{
        Batch, Claim, Command, Faults, Hal, Mask, NoHardware, Overrides, Sample, seal,
    };

    const N: usize = 4;

    #[test]
    fn policies_default_to_fail_and_never() {
        assert_eq!(MissingPolicy::default(), MissingPolicy::Fail);
        assert_eq!(JoinPolicy::default(), JoinPolicy::Never);
        assert_eq!(Overrides::default().missing, MissingPolicy::Fail);
        assert_eq!(Overrides::default().join, JoinPolicy::Never);
    }

    /// A member that reports actuator 0 present and counts its upkeeps.
    #[derive(Default)]
    struct Present {
        upkeeps: u32,
    }

    impl seal::Adapter for Present {}

    impl Hal<N> for Present {
        type Error = Infallible;

        fn check(&mut self, overrides: Overrides) -> Result<Claim<N>, Infallible> {
            NoHardware.check(overrides)
        }

        fn open(&mut self) -> Result<(), Infallible> {
            Ok(())
        }

        fn zero(&mut self) -> Result<(), Infallible> {
            Ok(())
        }

        fn ping(&mut self) -> Result<(), Infallible> {
            Ok(())
        }

        fn health(&mut self) -> Result<(), Infallible> {
            Ok(())
        }

        fn configure(&mut self) -> Result<(), Infallible> {
            Ok(())
        }

        fn read_all(&mut self, _sample: Sample<'_, N>) -> Result<(), Infallible> {
            Ok(())
        }

        fn torque_on(&mut self) -> Result<(), Infallible> {
            Ok(())
        }

        fn read(&mut self, _sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
            Mask::NONE
        }

        fn write(&mut self, _: Command<'_, N>, _: &Batch<'_, N>, _: &mut Faults<N>) {}

        fn present(&self) -> Mask<N> {
            let mut mask = Mask::NONE;
            mask.set(0);
            mask
        }

        fn upkeep(&mut self) {
            self.upkeeps += 1;
        }

        fn hold(&mut self, _target: Command<'_, N>) {}

        fn torque_off(&mut self) {}

        fn close(&mut self) {}
    }

    #[test]
    fn forwarded_hal_reports_present_and_upkeep() {
        let mut member = Present::default();
        let mut lent = &mut member;
        assert!(Hal::<N>::present(&lent).iter().eq([0]));
        Hal::<N>::upkeep(&mut lent);
        assert_eq!(member.upkeeps, 1);
        assert_eq!(Hal::<N>::present(&NoHardware), Mask::ALL);
    }
}
