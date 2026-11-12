//! The Seeed XIAO nRF52840 Sense as a sensor (feature `xiao`): its IMU and microphone over USB,
//! UART or BLE, for [`Robot::add_sensor`](crate::Robot::add_sensor) (SRS FR-5.7). Nothing opens
//! before Init; controllers read it through [`Sensors`](crate::Sensors), in m/s², rad/s and
//! 16 kHz PCM.
//!
//! ```no_run
//! use kmr_api::xiao::Xiao;
//! use kmr_api::{EachTick, Sensors, Time};
//!
//! kmr_api::robot!(XmBench);
//!
//! fn balance(_time: &Time, _robot: &mut State, sensors: &Sensors) {
//!     let _rate = sensors.imu(0).and_then(|imu| imu.gyro());
//!     let _pcm = sensors.mic(0).map(|mic| mic.samples());
//! }
//!
//! fn main() {
//!     let run = Robot::new()
//!         .add_sensor(Xiao::usb().imu().mic())
//!         .add_controller(EachTick, balance)
//!         .run();
//!     if let Err(e) = run {
//!         eprintln!("{e}");
//!     }
//! }
//! ```

/// The IMU's rates and ranges, for [`ImuSpec`].
pub use kmr_adapters::xiao::{AccelRange, GyroRange, Rate};
/// The board, the link it opens and the sensors asked of it: `Xiao::usb().imu().mic()`.
pub use kmr_adapters::xiao::{ImuSpec, LinkSpec, Xiao};
/// Why the board did not come up, the sensor it names and the link that holds it.
pub use kmr_adapters::xiao::{Link, Sensor, XiaoError};
