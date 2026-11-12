//! The writer view of [`Sensors`]: what one sensor adapter may write, its own
//! slots only, each write stamped with the tick (SRS FR-5.7).

use core::marker::PhantomData;

use super::{Accel, Gyro, Sensors};

/// One adapter's slots, lent for one [`SensorHal::read`]. `local` counts
/// within the adapter's own slots of each kind, from 0; past them a call is
/// ignored.
///
/// [`SensorHal::read`]: crate::hal::SensorHal::read
pub struct Feed<'a> {
    of: PhantomData<&'a mut Sensors>,
}

impl Feed<'_> {
    /// One IMU reading: both halves, as they came. `None`: that half is off.
    pub fn imu(&mut self, local: usize, accel: Option<Accel>, gyro: Option<Gyro>) {
        let _ = (local, accel, gyro);
        todo!()
    }

    /// PCM samples, appended to this tick's window; those past
    /// [`MIC_WINDOW`](super::MIC_WINDOW) are counted dropped.
    pub fn audio(&mut self, local: usize, samples: &[i16]) {
        let _ = (local, samples);
        todo!()
    }

    /// The adapter's running total of IMU readings lost.
    pub fn imu_dropped(&mut self, local: usize, total: u64) {
        let _ = (local, total);
        todo!()
    }

    /// The adapter's running total of audio samples lost.
    pub fn audio_dropped(&mut self, local: usize, total: u64) {
        let _ = (local, total);
        todo!()
    }

    /// The adapter lost its link this read: the tick raises `sensor missing`
    /// (SRS FR-13.4).
    pub fn link_down(&mut self) {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_fed_imu_reads_back_with_age_zero() {
        todo!()
    }

    #[test]
    fn age_counts_ticks_since_the_reading() {
        todo!()
    }

    #[test]
    fn a_half_fed_imu_keeps_the_other_half_none() {
        todo!()
    }

    #[test]
    fn audio_appends_within_a_tick() {
        todo!()
    }

    #[test]
    fn audio_is_emptied_at_the_next_tick() {
        todo!()
    }

    #[test]
    fn audio_past_the_window_is_counted_dropped() {
        todo!()
    }

    #[test]
    fn a_local_index_past_the_member_is_ignored() {
        todo!()
    }

    #[test]
    fn a_feed_cannot_reach_another_members_slot() {
        todo!()
    }
}
