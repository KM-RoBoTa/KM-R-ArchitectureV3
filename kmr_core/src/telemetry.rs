//! Actuator telemetry: temperature, input voltage and hardware error status
//! (SRS FR-5.11).
//!
//! Out of band, like the fault flags: not a [`StateField`](crate::state::StateField),
//! never in Desired or History, never written by a controller. One block per
//! robot, next to History. The adapters fill it through
//! [`Hal::telemetry`](crate::hal::Hal::telemetry), at their own cadence; the
//! engine stamps each refreshed actuator with the tick that refreshed it.

use crate::error::StateError;
use crate::hal::Mask;

/// The newest telemetry reading of each of the `N` actuators.
///
/// Read-only for controllers, through [`crate::RobotState::telemetry`]. An
/// adapter writes it with the `set_*` methods in its `telemetry` call, at the
/// engine indices it already uses for `Sample`; a value it never reports
/// stays `None`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Telemetry<const N: usize> {
    /// Temperature, °C.
    temperature_c: [Option<f32>; N],
    /// Input voltage, V.
    voltage_v: [Option<f32>; N],
    /// Raw hardware error status bits, brand-defined.
    hardware_error: [Option<u8>; N],
    /// `Time::tick` of the call that last refreshed each actuator.
    refreshed: [Option<u64>; N],
    /// The tick in progress.
    now: u64,
}

impl<const N: usize> Default for Telemetry<N> {
    fn default() -> Self {
        Self {
            temperature_c: [None; N],
            voltage_v: [None; N],
            hardware_error: [None; N],
            refreshed: [None; N],
            now: 0,
        }
    }
}

impl<const N: usize> Telemetry<N> {
    /// Temperature of actuator `index`, °C. `Ok(None)` until its adapter
    /// reports one.
    pub fn temperature_c(&self, index: usize) -> Result<Option<f32>, StateError> {
        at(&self.temperature_c, index)
    }

    /// Input voltage of actuator `index`, V. `Ok(None)` until its adapter
    /// reports one.
    pub fn voltage_v(&self, index: usize) -> Result<Option<f32>, StateError> {
        at(&self.voltage_v, index)
    }

    /// Raw hardware error status of actuator `index`, bits defined by its
    /// brand. `Ok(None)` until its adapter reports one.
    pub fn hardware_error(&self, index: usize) -> Result<Option<u8>, StateError> {
        at(&self.hardware_error, index)
    }

    /// Ticks since the telemetry of actuator `index` was last refreshed.
    /// `Ok(None)` until its first refresh, and always on a run without
    /// hardware.
    ///
    /// The engine calls `telemetry` after the tick's write, so a reading
    /// taken in tick `k` is seen from tick `k + 1` on, aged 1. A reading
    /// taken in `PostInit` is stamped tick 0.
    // DECISION: stamped with `Time::tick` as it stands at the call, as the
    // plan asks; the bring-up call therefore reads age 0 in tick 0.
    pub fn age(&self, index: usize) -> Result<Option<u64>, StateError> {
        at(&self.refreshed, index).map(|tick| tick.map(|tick| self.now.saturating_sub(tick)))
    }

    /// Sets the temperature of actuator `index`, °C. An index past `N` is
    /// ignored.
    pub fn set_temperature_c(&mut self, index: usize, celsius: f32) {
        set(&mut self.temperature_c, index, celsius);
    }

    /// Sets the input voltage of actuator `index`, V. An index past `N` is
    /// ignored.
    pub fn set_voltage_v(&mut self, index: usize, volts: f32) {
        set(&mut self.voltage_v, index, volts);
    }

    /// Sets the raw hardware error status of actuator `index`. An index past
    /// `N` is ignored.
    pub fn set_hardware_error(&mut self, index: usize, bits: u8) {
        set(&mut self.hardware_error, index, bits);
    }

    /// Notes the tick in progress, which every age is counted from.
    pub(crate) fn set_tick(&mut self, tick: u64) {
        self.now = tick;
    }

    /// Stamps the actuators of `refreshed` with `tick`.
    pub(crate) fn stamp(&mut self, refreshed: Mask<N>, tick: u64) {
        for (index, stamp) in self.refreshed.iter_mut().enumerate() {
            if refreshed.get(index) {
                *stamp = Some(tick);
            }
        }
    }

    /// The bits of the readings of `index`, NaN included: what the debug
    /// audit compares.
    #[cfg(debug_assertions)]
    pub(crate) fn bits(&self, index: usize) -> [Option<u32>; 3] {
        let bits = |value: Option<&Option<f32>>| value.copied().flatten().map(f32::to_bits);
        [
            bits(self.temperature_c.get(index)),
            bits(self.voltage_v.get(index)),
            self.hardware_error
                .get(index)
                .copied()
                .flatten()
                .map(u32::from),
        ]
    }
}

/// The reading of `index`, or why there is none.
fn at<T: Copy, const N: usize>(
    values: &[Option<T>; N],
    index: usize,
) -> Result<Option<T>, StateError> {
    values
        .get(index)
        .copied()
        .ok_or(StateError::OutOfRange { index, len: N })
}

/// Sets the reading of `index`; past `N`, nothing.
fn set<T, const N: usize>(values: &mut [Option<T>; N], index: usize, value: T) {
    if let Some(slot) = values.get_mut(index) {
        *slot = Some(value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::N;

    #[test]
    fn a_new_block_has_no_reading_and_no_age() {
        let telemetry = Telemetry::<N>::default();
        for index in 0..N {
            assert_eq!(telemetry.temperature_c(index), Ok(None));
            assert_eq!(telemetry.voltage_v(index), Ok(None));
            assert_eq!(telemetry.hardware_error(index), Ok(None));
            assert_eq!(telemetry.age(index), Ok(None));
        }
    }

    #[test]
    fn a_reading_lands_on_its_actuator_only() {
        let mut telemetry = Telemetry::<N>::default();
        telemetry.set_temperature_c(1, 41.5);
        telemetry.set_voltage_v(1, 12.1);
        telemetry.set_hardware_error(2, 0b100);

        assert_eq!(telemetry.temperature_c(1), Ok(Some(41.5)));
        assert_eq!(telemetry.voltage_v(1), Ok(Some(12.1)));
        assert_eq!(telemetry.hardware_error(2), Ok(Some(0b100)));
        assert_eq!(telemetry.temperature_c(0), Ok(None));
        assert_eq!(telemetry.hardware_error(1), Ok(None));
    }

    #[test]
    fn an_index_past_n_is_ignored_on_write_and_refused_on_read() {
        let mut telemetry = Telemetry::<N>::default();
        telemetry.set_temperature_c(N, 99.0);
        assert_eq!(telemetry, Telemetry::default());
        assert_eq!(
            telemetry.temperature_c(N),
            Err(StateError::OutOfRange { index: N, len: N })
        );
        assert_eq!(
            telemetry.age(N),
            Err(StateError::OutOfRange { index: N, len: N })
        );
    }

    #[test]
    fn the_age_counts_ticks_from_the_stamp_of_each_actuator() {
        let mut telemetry = Telemetry::<N>::default();
        let mut first = Mask::NONE;
        first.set(0);
        telemetry.stamp(first, 3);
        let mut second = Mask::NONE;
        second.set(1);
        telemetry.stamp(second, 5);
        telemetry.set_tick(7);

        assert_eq!(telemetry.age(0), Ok(Some(4)));
        assert_eq!(telemetry.age(1), Ok(Some(2)));
        assert_eq!(telemetry.age(2), Ok(None));
    }
}
