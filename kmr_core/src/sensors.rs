//! Sensors: a closed, typed set of slots the sensor adapters fill and the
//! controllers read, each reading with its age in ticks (SRS FR-5.7).
//!
//! Like State, the set is sealed: two kinds (IMU, microphone), a fixed number
//! of slots per kind, no heap. Slots are handed out at run() finalization, in
//! declaration order per kind; an adapter writes its own slots only, through
//! a [`Feed`].
//!
//! WIP: the feed, the list and the loop come with S03 to S05
//! (docs/plans/sensors.md).

mod feed;

use core::ops::Range;

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
        let [x, _, _] = self.0;
        x
    }

    /// Along y, m/s².
    pub fn y(self) -> f32 {
        let [_, y, _] = self.0;
        y
    }

    /// Along z, m/s².
    pub fn z(self) -> f32 {
        let [_, _, z] = self.0;
        z
    }
}

/// An angular rate, rad/s, around the sensor's x, y and z axes (SRS FR-5.7).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Gyro(pub [f32; 3]);

impl Gyro {
    /// Around x, rad/s.
    pub fn x(self) -> f32 {
        let [x, _, _] = self.0;
        x
    }

    /// Around y, rad/s.
    pub fn y(self) -> f32 {
        let [_, y, _] = self.0;
        y
    }

    /// Around z, rad/s.
    pub fn z(self) -> f32 {
        let [_, _, z] = self.0;
        z
    }
}

/// What the sensors read, handed to every controller as `&Sensors` (SRS
/// FR-5.7). Read-only for controllers.
pub struct Sensors {
    imus: [ImuSlot; MAX_IMUS],
    mics: [MicSlot; MAX_MICS],
    /// IMU slots handed out, from 0.
    imus_declared: usize,
    /// Microphone slots handed out, from 0.
    mics_declared: usize,
    /// The tick in progress, which every age is counted from.
    now: u64,
    /// A member reported its link down in the read in progress.
    link_down: bool,
}

/// The newest reading of one IMU slot.
#[derive(Clone, Copy, Debug)]
struct ImuSlot {
    accel: Option<Accel>,
    gyro: Option<Gyro>,
    /// The tick the reading arrived in; `None` until the first.
    refreshed: Option<u64>,
    /// The adapter's running total of readings lost.
    dropped: u64,
}

impl ImuSlot {
    /// No reading yet.
    const EMPTY: Self = Self {
        accel: None,
        gyro: None,
        refreshed: None,
        dropped: 0,
    };
}

/// One microphone slot: this tick's window of samples.
#[derive(Clone, Copy, Debug)]
struct MicSlot {
    window: [i16; MIC_WINDOW],
    /// Samples of `window` that arrived this tick.
    len: usize,
    rate_hz: u32,
    /// The last tick samples arrived in; `None` until the first.
    refreshed: Option<u64>,
    /// Samples past the window, counted by the engine.
    overflow: u64,
    /// The adapter's running total of samples lost.
    reported: u64,
}

impl MicSlot {
    /// No sample yet.
    const EMPTY: Self = Self {
        window: [0; MIC_WINDOW],
        len: 0,
        rate_hz: 0,
        refreshed: None,
        overflow: 0,
        reported: 0,
    };
}

/// One member's slots of each kind, engine indices.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Slots {
    imu_start: usize,
    imu_end: usize,
    mic_start: usize,
    mic_end: usize,
}

impl Slots {
    /// The IMU slots `imus` and the microphone slots `mics`.
    pub(crate) const fn new(imus: Range<usize>, mics: Range<usize>) -> Self {
        Self {
            imu_start: imus.start,
            imu_end: imus.end,
            mic_start: mics.start,
            mic_end: mics.end,
        }
    }

    /// The IMU slots.
    pub(crate) const fn imus(&self) -> Range<usize> {
        self.imu_start..self.imu_end
    }

    /// The microphone slots.
    pub(crate) const fn mics(&self) -> Range<usize> {
        self.mic_start..self.mic_end
    }
}

impl Sensors {
    /// No slot declared, no reading.
    pub(crate) const fn new() -> Self {
        Self {
            imus: [ImuSlot::EMPTY; MAX_IMUS],
            mics: [MicSlot::EMPTY; MAX_MICS],
            imus_declared: 0,
            mics_declared: 0,
            now: 0,
            link_down: false,
        }
    }

    /// Declares one member's `slots`, its microphones at `mic_rate_hz`.
    /// Slots past the kind's count are ignored.
    pub(crate) fn declare(&mut self, slots: Slots, mic_rate_hz: u32) {
        let imus = slots.imus().end.min(MAX_IMUS);
        let mics = slots.mics().end.min(MAX_MICS);
        self.imus_declared = self.imus_declared.max(imus);
        self.mics_declared = self.mics_declared.max(mics);
        for slot in self.mics.get_mut(slots.mics()).into_iter().flatten() {
            slot.rate_hz = mic_rate_hz;
        }
    }

    /// IMU `index`, in declaration order. `None` when no IMU was declared at
    /// that index or none has reported yet (SRS FR-5.7).
    pub fn imu(&self, index: usize) -> Option<Imu<'_>> {
        let slot = self.imus.get(..self.imus_declared)?.get(index)?;
        let refreshed = slot.refreshed?;
        Some(Imu {
            slot,
            age: self.now.saturating_sub(refreshed),
        })
    }

    /// Microphone `index`, in declaration order. `None` when no microphone
    /// was declared at that index or none has reported yet (SRS FR-5.7).
    pub fn mic(&self, index: usize) -> Option<Mic<'_>> {
        let slot = self.mics.get(..self.mics_declared)?.get(index)?;
        slot.refreshed.map(|_| Mic { slot })
    }
}

/// The newest reading of one IMU (SRS FR-5.7).
pub struct Imu<'a> {
    slot: &'a ImuSlot,
    age: u64,
}

impl Imu<'_> {
    /// Acceleration, m/s². `None` when the reading carries none: that half
    /// of the sensor is off.
    pub fn accel(&self) -> Option<Accel> {
        self.slot.accel
    }

    /// Angular rate, rad/s. `None` when the reading carries none: that half
    /// of the sensor is off.
    pub fn gyro(&self) -> Option<Gyro> {
        self.slot.gyro
    }

    /// Ticks since the reading arrived, 0 in the tick it arrived.
    pub fn age(&self) -> u64 {
        self.age
    }

    /// Readings lost so far, as the adapter counts them.
    pub fn dropped(&self) -> u64 {
        self.slot.dropped
    }
}

/// The samples one microphone delivered this tick (SRS FR-5.7).
pub struct Mic<'a> {
    slot: &'a MicSlot,
}

impl<'a> Mic<'a> {
    /// PCM samples that arrived since the last tick, oldest first, at most
    /// [`MIC_WINDOW`].
    // DECISION: the samples borrow the `Sensors`, not the view, so
    // `sensors.mic(0)?.samples()` outlives its temporary view.
    pub fn samples(&self) -> &'a [i16] {
        self.slot.window.get(..self.slot.len).unwrap_or(&[])
    }

    /// Sample rate, Hz.
    pub fn rate_hz(&self) -> u32 {
        self.slot.rate_hz
    }

    /// Samples lost so far: those past the window, and those the adapter
    /// counts lost.
    pub fn dropped(&self) -> u64 {
        self.slot.overflow.saturating_add(self.slot.reported)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every slot of each kind declared, by one member.
    fn all_declared() -> Sensors {
        let mut sensors = Sensors::new();
        sensors.declare(Slots::new(0..MAX_IMUS, 0..MAX_MICS), 16_000);
        sensors
    }

    #[test]
    fn an_undeclared_slot_reads_none() {
        let sensors = Sensors::new();
        assert!(sensors.imu(0).is_none());
        assert!(sensors.mic(0).is_none());
    }

    #[test]
    fn an_index_past_the_slots_reads_none() {
        let mut sensors = all_declared();
        for slot in &mut sensors.imus {
            slot.refreshed = Some(0);
        }
        for slot in &mut sensors.mics {
            slot.refreshed = Some(0);
        }
        assert!(sensors.imu(MAX_IMUS - 1).is_some());
        assert!(sensors.imu(MAX_IMUS).is_none());
        assert!(sensors.mic(MAX_MICS - 1).is_some());
        assert!(sensors.mic(MAX_MICS).is_none());
    }

    #[test]
    fn a_declared_slot_without_a_reading_reads_none() {
        let mut sensors = all_declared();
        assert!(sensors.imu(0).is_none());
        assert!(sensors.mic(0).is_none());

        sensors.imus[0].refreshed = Some(0);
        sensors.mics[0].refreshed = Some(0);
        assert!(sensors.imu(0).is_some());
        assert_eq!(sensors.mic(0).map(|mic| mic.rate_hz()), Some(16_000));
    }

    #[test]
    fn sensors_fit_in_three_kilobytes() {
        assert!(size_of::<Sensors>() <= 3072, "{}", size_of::<Sensors>());
    }

    #[test]
    fn accel_and_gyro_name_their_axes_in_order() {
        let accel = Accel([1.0, 2.0, 3.0]);
        let gyro = Gyro([4.0, 5.0, 6.0]);
        assert_eq!([accel.x(), accel.y(), accel.z()], [1.0, 2.0, 3.0]);
        assert_eq!([gyro.x(), gyro.y(), gyro.z()], [4.0, 5.0, 6.0]);
    }
}
