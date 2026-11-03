//! The read-only fault view of one tick (SRS FR-13.4).

use kmr_core::hal::{Faults as CoreFaults, TickFlags};

/// One bit per actuator: bit `i` is actuator `i`.
pub use kmr_core::hal::Mask;

/// The fault bits and flags of one tick, read from its History slot: per actuator `stale`,
/// `bus_error`, `offline`, `rejected`, `joint_limit`, `envelope`; per tick `overrun`,
/// `sensor_missing`, `remote_timeout`, `unauthorized`. A copy: controllers cannot set or
/// clear a flag.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Faults<const N: usize> {
    actuators: CoreFaults<N>,
    tick: TickFlags,
}

impl<const N: usize> Faults<N> {
    /// The view of a slot's faults and flags.
    pub(crate) fn new(actuators: CoreFaults<N>, tick: TickFlags) -> Self {
        Self { actuators, tick }
    }

    /// The whole-tick flags as bits, the core's `TickFlags` layout (the FFI view's
    /// `tick_flags`).
    #[cfg_attr(not(feature = "ffi"), allow(dead_code))]
    pub(crate) fn tick_bits(&self) -> u32 {
        self.tick.bits()
    }

    /// Actuators whose read failed; their previous value was kept (SRS FR-2.6).
    pub fn stale(&self) -> Mask<N> {
        self.actuators.stale
    }

    /// Actuators whose bus transaction failed.
    pub fn bus_error(&self) -> Mask<N> {
        self.actuators.bus_error
    }

    /// Actuators that did not answer.
    pub fn offline(&self) -> Mask<N> {
        self.actuators.offline
    }

    /// Actuators written but not sent: torque off, field not taken by the mode, or a value
    /// outside the actuator's range.
    pub fn rejected(&self) -> Mask<N> {
        self.actuators.rejected
    }

    /// Actuators whose written value breached their limit (SRS FR-4.5).
    pub fn joint_limit(&self) -> Mask<N> {
        self.actuators.joint_limit
    }

    /// Actuators whose written value left their envelope and was clamped (SRS FR-13.5).
    pub fn envelope(&self) -> Mask<N> {
        self.actuators.envelope
    }

    /// The tick finished after its deadline.
    pub fn overrun(&self) -> bool {
        self.tick.contains(TickFlags::OVERRUN)
    }

    /// A sensor gave no reading this tick.
    pub fn sensor_missing(&self) -> bool {
        self.tick.contains(TickFlags::SENSOR_MISSING)
    }

    /// A remote source fell silent past its timeout (SRS FR-13.3).
    pub fn remote_timeout(&self) -> bool {
        self.tick.contains(TickFlags::REMOTE_TIMEOUT)
    }

    /// A message from an identity that is not a remote source was dropped (SRS FR-13.2).
    pub fn unauthorized(&self) -> bool {
        self.tick.contains(TickFlags::UNAUTHORIZED)
    }

    /// Actuators with at least one fault bit set.
    pub fn any_actuator(&self) -> Mask<N> {
        self.actuators.any()
    }

    /// Whether no bit and no flag is set.
    pub fn is_clear(&self) -> bool {
        self.actuators.is_empty() && self.tick.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_accessor_reads_its_own_bit() {
        let mut actuators = CoreFaults::<4>::default();
        actuators.stale.set(0);
        actuators.envelope.set(3);
        let faults = Faults::new(actuators, TickFlags::OVERRUN.union(TickFlags::UNAUTHORIZED));

        assert!(faults.stale().get(0) && !faults.stale().get(3));
        assert!(faults.envelope().get(3));
        assert!(faults.bus_error().is_empty() && faults.joint_limit().is_empty());
        assert!(faults.overrun() && faults.unauthorized());
        assert!(!faults.sensor_missing() && !faults.remote_timeout());
        assert_eq!(faults.any_actuator().iter().collect::<Vec<_>>(), [0, 3]);
        assert!(!faults.is_clear());
        assert!(Faults::<4>::default().is_clear());
        assert_eq!(faults.tick_bits(), 0b1001);
    }
}
