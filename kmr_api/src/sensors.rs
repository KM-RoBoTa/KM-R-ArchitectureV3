//! The sensors a controller reads each tick: IMUs and microphones, each reading with its age
//! (SRS FR-5.7).

/// An acceleration, m/s², and an angular rate, rad/s, on the sensor's x, y and z axes.
pub use kmr_core::sensors::{Accel, Gyro};

/// What a sensor lost at run time does beyond raising `sensor_missing`: set it with
/// [`Robot::set_sensor_policy`](crate::Robot::set_sensor_policy).
pub use kmr_core::hal::SensorPolicy;

/// Read-only view of the robot sensors handed to a controller each tick.
///
/// Borrowing newtype over `&kmr_core::Sensors`, wrapped at the api→core
/// boundary so the core type never appears in a public signature.
pub struct Sensors<'a> {
    /// The core sensor handle the runtime lends for one tick.
    inner: &'a kmr_core::Sensors,
}

impl<'a> Sensors<'a> {
    /// Wrap a borrowed core sensor handle. Called only at the boundary
    /// (`ApiInline::call`); never by users.
    pub(crate) fn new(inner: &'a kmr_core::Sensors) -> Self {
        Sensors { inner }
    }

    /// IMU `index`, counted over the IMUs of the sensors in the order they were added. `None`
    /// when no IMU was added at that index or none has reported yet.
    pub fn imu(&self, index: usize) -> Option<Imu<'_>> {
        self.inner.imu(index).map(|inner| Imu { inner })
    }

    /// Microphone `index`, counted over the microphones of the sensors in the order they were
    /// added. `None` when no microphone was added at that index or none has reported yet.
    pub fn mic(&self, index: usize) -> Option<Mic<'_>> {
        self.inner.mic(index).map(|inner| Mic { inner })
    }
}

/// The newest reading of one IMU.
pub struct Imu<'a> {
    inner: kmr_core::sensors::Imu<'a>,
}

impl Imu<'_> {
    /// Acceleration, m/s². `None` when the reading carries none: that half of the sensor is
    /// off.
    pub fn accel(&self) -> Option<Accel> {
        self.inner.accel()
    }

    /// Angular rate, rad/s. `None` when the reading carries none: that half of the sensor is
    /// off.
    pub fn gyro(&self) -> Option<Gyro> {
        self.inner.gyro()
    }

    /// Ticks since the reading arrived, 0 in the tick it arrived. A sensor slower than the
    /// tick is not missing: its age grows, then returns to 0.
    pub fn age(&self) -> u64 {
        self.inner.age()
    }

    /// Readings lost so far, as the sensor's adapter counts them.
    pub fn dropped(&self) -> u64 {
        self.inner.dropped()
    }
}

/// The samples one microphone delivered this tick.
pub struct Mic<'a> {
    inner: kmr_core::sensors::Mic<'a>,
}

impl<'a> Mic<'a> {
    /// PCM samples that arrived since the last tick, oldest first; empty when none did.
    pub fn samples(&self) -> &'a [i16] {
        self.inner.samples()
    }

    /// Sample rate, Hz.
    pub fn rate_hz(&self) -> u32 {
        self.inner.rate_hz()
    }

    /// Samples lost so far: those past the tick's window, and those the adapter counts lost.
    pub fn dropped(&self) -> u64 {
        self.inner.dropped()
    }
}
