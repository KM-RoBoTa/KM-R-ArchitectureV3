//! The sensor contract: what the engine calls on a sensor adapter (SRS
//! FR-5.7). Separate from [`Hal`](super::Hal): a sensor owns no actuator and
//! has no torque to cut.

use core::time::Duration;

use super::seal;
use crate::sensors::Feed;

/// What one sensor adapter asks for at run() finalization, no I/O.
pub struct Needs {
    /// IMU slots it fills.
    pub imus: u8,
    /// Microphone slots it fills.
    pub mics: u8,
    /// Sample rate of its microphones, Hz.
    pub mic_rate_hz: u32,
    /// Longest silence of any of its slots before the tick raises `sensor
    /// missing` (SRS FR-13.4). A sensor slower than the tick is not missing.
    pub max_silence: Duration,
}

/// The contract of one sensor adapter (SRS FR-5.7). Static dispatch only.
///
/// The engine calls `needs` at run() finalization, `open` in Init after the
/// actuators' `open` and `zero`, `read` once in PostInit and once per tick
/// before `First`. Once `open` was called, every exit ends with `close`,
/// after the actuators' `torque_off` and `close`.
pub trait SensorHal: seal::Adapter {
    /// Bring-up failures. Never ambiguous: says why and what causes it,
    /// naming the sensor and its link.
    type Error: core::error::Error + 'static;

    /// run() finalization, no I/O: the slots and the silence it needs.
    fn needs(&self) -> Needs;

    /// Init: opens the link and turns the sensors on.
    fn open(&mut self) -> Result<(), Self::Error>;

    /// PostInit, then each tick before `First`: what arrived since the last
    /// call, into `feed`. Never waits: no allocation, lock, sleep or retry.
    fn read(&mut self, feed: Feed<'_>);

    /// Last call: turns the sensors off and closes the link. Best effort,
    /// never fails; called after a failed `open` too.
    fn close(&mut self);
}

/// Lends the adapter to a run, so the caller keeps it before and after
/// (SRS FR-5.9).
impl<A: SensorHal> SensorHal for &mut A {
    type Error = A::Error;

    fn needs(&self) -> Needs {
        (**self).needs()
    }

    fn open(&mut self) -> Result<(), Self::Error> {
        (**self).open()
    }

    fn read(&mut self, feed: Feed<'_>) {
        (**self).read(feed);
    }

    fn close(&mut self) {
        (**self).close();
    }
}
