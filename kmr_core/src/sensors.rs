//! Sensors: a closed, typed set of slots the sensor adapters fill and the
//! controllers read, each reading with its age in ticks (SRS FR-5.7).
//!
//! Like State, the set is sealed: two kinds (IMU, microphone), a fixed number
//! of slots per kind, no heap. Slots are handed out at run() finalization, in
//! declaration order per kind; an adapter writes its own slots only, through
//! a [`Feed`].
//!
//! WIP: step 4 contract, bodies to come (docs/plans/sensors.md).

mod feed;

use core::marker::PhantomData;

pub use feed::Feed;

/// IMU slots of a robot (SRS FR-5.7).
pub const MAX_IMUS: usize = 4;

/// Microphone slots of a robot (SRS FR-5.7).
pub const MAX_MICS: usize = 1;

/// Samples one microphone slot holds per tick: 64 ms at 16 kHz (SRS FR-5.7).
pub const MIC_WINDOW: usize = 1024;

/// An acceleration, m/s², on the sensor's x, y and z axes (SRS FR-5.7).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Accel(pub [f32; 3]);

impl Accel {
    /// Along x, m/s².
    pub fn x(self) -> f32 {
        todo!()
    }

    /// Along y, m/s².
    pub fn y(self) -> f32 {
        todo!()
    }

    /// Along z, m/s².
    pub fn z(self) -> f32 {
        todo!()
    }
}

/// An angular rate, rad/s, around the sensor's x, y and z axes (SRS FR-5.7).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Gyro(pub [f32; 3]);

impl Gyro {
    /// Around x, rad/s.
    pub fn x(self) -> f32 {
        todo!()
    }

    /// Around y, rad/s.
    pub fn y(self) -> f32 {
        todo!()
    }

    /// Around z, rad/s.
    pub fn z(self) -> f32 {
        todo!()
    }
}

/// What the sensors read, handed to every controller as `&Sensors` (SRS
/// FR-5.7). Read-only for controllers.
pub struct Sensors {}

impl Sensors {
    /// IMU `index`, in declaration order. `None` when no IMU was declared at
    /// that index or none has reported yet (SRS FR-5.7).
    pub fn imu(&self, index: usize) -> Option<Imu<'_>> {
        let _ = index;
        todo!()
    }

    /// Microphone `index`, in declaration order. `None` when no microphone
    /// was declared at that index or none has reported yet (SRS FR-5.7).
    pub fn mic(&self, index: usize) -> Option<Mic<'_>> {
        let _ = index;
        todo!()
    }
}

/// The newest reading of one IMU (SRS FR-5.7).
pub struct Imu<'a> {
    of: PhantomData<&'a Sensors>,
}

impl Imu<'_> {
    /// Acceleration, m/s². `None` when the reading carries none: that half
    /// of the sensor is off.
    pub fn accel(&self) -> Option<Accel> {
        todo!()
    }

    /// Angular rate, rad/s. `None` when the reading carries none: that half
    /// of the sensor is off.
    pub fn gyro(&self) -> Option<Gyro> {
        todo!()
    }

    /// Ticks since the reading arrived, 0 in the tick it arrived.
    pub fn age(&self) -> u64 {
        todo!()
    }

    /// Readings lost so far, as the adapter counts them.
    pub fn dropped(&self) -> u64 {
        todo!()
    }
}

/// The samples one microphone delivered this tick (SRS FR-5.7).
pub struct Mic<'a> {
    of: PhantomData<&'a Sensors>,
}

impl<'a> Mic<'a> {
    /// PCM samples that arrived since the last tick, oldest first, at most
    /// [`MIC_WINDOW`].
    pub fn samples(&self) -> &'a [i16] {
        todo!()
    }

    /// Sample rate, Hz.
    pub fn rate_hz(&self) -> u32 {
        todo!()
    }

    /// Samples lost so far: those past the window, and those the adapter
    /// counts lost.
    pub fn dropped(&self) -> u64 {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn an_undeclared_slot_reads_none() {
        todo!()
    }

    #[test]
    fn an_index_past_the_slots_reads_none() {
        todo!()
    }

    #[test]
    fn a_declared_slot_without_a_reading_reads_none() {
        todo!()
    }

    #[test]
    fn sensors_fit_in_three_kilobytes() {
        todo!()
    }
}
