//! The writer view of [`Sensors`]: what one sensor adapter may write, its own
//! slots only, each write stamped with the tick (SRS FR-5.7).

use super::{Accel, Gyro, ImuSlot, MIC_WINDOW, MicSlot};

/// One adapter's slots, lent for one [`SensorHal::read`]. `local` counts
/// within the adapter's own slots of each kind, from 0; past them a call is
/// ignored.
///
/// [`SensorHal::read`]: crate::hal::SensorHal::read
pub struct Feed<'a> {
    imus: &'a mut [ImuSlot],
    mics: &'a mut [MicSlot],
    /// The tick every write is stamped with.
    now: u64,
    link_down: &'a mut bool,
}

impl<'a> Feed<'a> {
    /// The view of `imus` and `mics`, one member's, at tick `now`.
    pub(super) fn new(
        imus: &'a mut [ImuSlot],
        mics: &'a mut [MicSlot],
        now: u64,
        link_down: &'a mut bool,
    ) -> Self {
        Self {
            imus,
            mics,
            now,
            link_down,
        }
    }

    /// One IMU reading: both halves, as they came. `None`: that half is off.
    // DECISION: a call replaces both halves, so `age()` is true of each; a
    // call with neither half is no reading and is ignored.
    pub fn imu(&mut self, local: usize, accel: Option<Accel>, gyro: Option<Gyro>) {
        if accel.is_none() && gyro.is_none() {
            return;
        }
        if let Some(slot) = self.imus.get_mut(local) {
            slot.accel = accel;
            slot.gyro = gyro;
            slot.refreshed = Some(self.now);
        }
    }

    /// PCM samples, appended to this tick's window; those past
    /// [`MIC_WINDOW`] are counted dropped.
    // DECISION: an empty call is no reading: it does not reset the silence.
    pub fn audio(&mut self, local: usize, samples: &[i16]) {
        let Some(slot) = self.mics.get_mut(local) else {
            return;
        };
        if samples.is_empty() {
            return;
        }
        let room = MIC_WINDOW.saturating_sub(slot.len);
        let (fits, past) = samples.split_at_checked(room).unwrap_or((samples, &[]));
        if let Some(window) = slot.window.get_mut(slot.len..slot.len + fits.len()) {
            window.copy_from_slice(fits);
            slot.len += fits.len();
        }
        let past = u64::try_from(past.len()).unwrap_or(u64::MAX);
        slot.overflow = slot.overflow.saturating_add(past);
        slot.refreshed = Some(self.now);
    }

    /// The adapter's running total of IMU readings lost.
    pub fn imu_dropped(&mut self, local: usize, total: u64) {
        if let Some(slot) = self.imus.get_mut(local) {
            slot.dropped = total;
        }
    }

    /// The adapter's running total of audio samples lost.
    pub fn audio_dropped(&mut self, local: usize, total: u64) {
        if let Some(slot) = self.mics.get_mut(local) {
            slot.reported = total;
        }
    }

    /// The adapter lost its link this read: the tick raises `sensor missing`
    /// (SRS FR-13.4).
    pub fn link_down(&mut self) {
        *self.link_down = true;
    }
}

#[cfg(test)]
mod tests {
    use super::super::{Sensors, Slots};
    use super::*;

    const ONE_G: Accel = Accel([0.0, 0.0, 9.81]);
    const SPIN: Gyro = Gyro([0.5, 0.0, 0.0]);

    /// Member 0 has IMU 0 and the microphone; member 1 has IMUs 1 and 2.
    fn two_members() -> (Sensors, Slots, Slots) {
        let first = Slots::new(0..1, 0..1);
        let second = Slots::new(1..3, 1..1);
        let mut sensors = Sensors::new();
        sensors.declare(first, 16_000);
        sensors.declare(second, 0);
        (sensors, first, second)
    }

    /// `N` samples counting up from `from`.
    fn ramp<const N: usize>(from: i16) -> [i16; N] {
        core::array::from_fn(|i| from + i16::try_from(i).expect("short ramp"))
    }

    #[test]
    fn a_fed_imu_reads_back_with_age_zero() {
        let (mut sensors, first, _) = two_members();
        sensors.begin_tick(4);
        sensors.feed(first).imu(0, Some(ONE_G), Some(SPIN));

        let imu = sensors.imu(0).expect("a reading");
        assert_eq!((imu.accel(), imu.gyro()), (Some(ONE_G), Some(SPIN)));
        assert_eq!(imu.age(), 0);
    }

    #[test]
    fn age_counts_ticks_since_the_reading() {
        let (mut sensors, first, _) = two_members();
        sensors.begin_tick(5);
        sensors.feed(first).imu(0, Some(ONE_G), None);
        sensors.begin_tick(8);

        assert_eq!(sensors.imu(0).map(|imu| imu.age()), Some(3));
    }

    #[test]
    fn a_half_fed_imu_keeps_the_other_half_none() {
        let (mut sensors, first, _) = two_members();
        sensors.feed(first).imu(0, Some(ONE_G), None);

        let imu = sensors.imu(0).expect("a reading");
        assert_eq!((imu.accel(), imu.gyro()), (Some(ONE_G), None));
    }

    #[test]
    fn audio_appends_within_a_tick() {
        let (mut sensors, first, _) = two_members();
        let mut feed = sensors.feed(first);
        feed.audio(0, &ramp::<96>(0));
        feed.audio(0, &ramp::<96>(96));

        let samples = sensors.mic(0).expect("samples").samples();
        assert_eq!(samples, ramp::<192>(0));
    }

    #[test]
    fn audio_is_emptied_at_the_next_tick() {
        let (mut sensors, first, _) = two_members();
        sensors.begin_tick(1);
        sensors.feed(first).audio(0, &ramp::<96>(0));
        sensors.begin_tick(2);

        let mic = sensors.mic(0).expect("the slot reported once");
        assert!(mic.samples().is_empty());
        assert_eq!(mic.rate_hz(), 16_000);
    }

    #[test]
    fn audio_past_the_window_is_counted_dropped() {
        let (mut sensors, first, _) = two_members();
        let burst = ramp::<1100>(0);
        sensors.feed(first).audio(0, &burst);

        let mic = sensors.mic(0).expect("samples");
        assert_eq!(mic.samples().len(), MIC_WINDOW);
        assert_eq!(mic.samples(), &burst[..MIC_WINDOW]);
        assert_eq!(mic.dropped(), 76);
    }

    #[test]
    fn a_local_index_past_the_member_is_ignored() {
        let (mut sensors, first, _) = two_members();
        let mut feed = sensors.feed(first);
        feed.imu(1, Some(ONE_G), Some(SPIN));
        feed.audio(1, &ramp::<8>(0));
        feed.imu_dropped(1, 7);

        assert!(sensors.imu(1).is_none());
        assert!(sensors.mic(0).is_none());
    }

    #[test]
    fn a_feed_cannot_reach_another_members_slot() {
        let (mut sensors, _, second) = two_members();
        let mut feed = sensors.feed(second);
        feed.imu(0, Some(ONE_G), None);
        feed.audio(0, &ramp::<8>(0));

        assert!(sensors.imu(0).is_none());
        assert_eq!(sensors.imu(1).and_then(|imu| imu.accel()), Some(ONE_G));
        assert!(sensors.imu(2).is_none());
        assert!(sensors.mic(0).is_none());
    }

    #[test]
    fn the_adapter_totals_are_set_and_sum_with_the_overflow() {
        let (mut sensors, first, _) = two_members();
        let mut feed = sensors.feed(first);
        feed.imu(0, Some(ONE_G), None);
        feed.imu_dropped(0, 3);
        feed.imu_dropped(0, 5);
        feed.audio(0, &ramp::<1030>(0));
        feed.audio_dropped(0, 10);

        assert_eq!(sensors.imu(0).map(|imu| imu.dropped()), Some(5));
        assert_eq!(sensors.mic(0).map(|mic| mic.dropped()), Some(16));
    }

    #[test]
    fn a_link_down_is_taken_once() {
        let (mut sensors, first, _) = two_members();
        sensors.feed(first).link_down();

        assert!(sensors.took_link_down());
        assert!(!sensors.took_link_down());
    }
}
