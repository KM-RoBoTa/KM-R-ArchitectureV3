//! The XIAO adapter: the Seeed XIAO nRF52840 Sense's IMU and microphone as
//! one sensor member, over USB, UART or BLE (SRS FR-5.7).
//!
//! It wraps `xiao-api`, the board's SDK, whose link threads and heap stay
//! inside it. Building a [`Xiao`] opens nothing (SRS FR-5.5): `open` opens
//! the link and turns the asked sensors on, `read` takes what arrived without
//! waiting, in m/s² and rad/s, and `close` turns them off.

#[cfg(test)]
mod tests;

use kmr_core::hal::{Needs, SensorHal, seal};
use kmr_core::sensors::Feed;

pub use xiao_api::{AccelRange, GyroRange, Link, Rate};

/// UART speed when none is given, the firmware's (FW-3).
const DEFAULT_BAUD: u32 = 921_600;
/// BLE connection attempts when none are given, `xiao-api`'s (API-2).
const DEFAULT_ATTEMPTS: u8 = 3;
/// The name the firmware advertises over BLE.
const BLE_NAME: &str = "XIAO-Sense";
/// IMU rate of [`Xiao::imu`], [`Xiao::gyro`] and [`Xiao::accel`].
const DEFAULT_RATE: Rate = Rate::Hz104;
/// Accelerometer range of [`Xiao::imu`] and [`Xiao::accel`].
const DEFAULT_ACCEL: AccelRange = AccelRange::G4;
/// Gyroscope range of [`Xiao::imu`] and [`Xiao::gyro`].
const DEFAULT_GYRO: GyroRange = GyroRange::Dps500;

/// The link a [`Xiao`] opens at Init.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LinkSpec {
    /// The board's USB plug: the first XIAO found.
    Usb,
    /// The board's TX/RX pins, on a serial port.
    Uart {
        /// The serial port, such as `/dev/ttyUSB0`.
        port: &'static str,
        /// Bits per second; must match the firmware's.
        baud: u32,
    },
    /// Bluetooth Low Energy.
    Ble {
        /// The name the board advertises; `XIAO-Sense` when `None`.
        name: Option<&'static str>,
        /// Tries to connect, and to reconnect after a drop.
        attempts: u8,
    },
}

/// Which IMU halves a [`Xiao`] turns on, each at its rate and range. A
/// half left `None` stays off on the board (FW-6) and reads `None`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSpec {
    /// The accelerometer.
    pub accel: Option<(Rate, AccelRange)>,
    /// The gyroscope.
    pub gyro: Option<(Rate, GyroRange)>,
}

/// A sensor of the board. The IMU is one sensor, both halves (FW-2).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sensor {
    /// The accelerometer and gyroscope chip.
    Imu,
    /// The PDM microphone.
    Mic,
}

/// Why a XIAO did not come up. Each names the sensor or the link, says why
/// and what to check.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum XiaoError {
    /// Another link holds the sensor (FW-1).
    #[error(
        "the XIAO's {} is busy: the board already streams it over its {} link, to another program or another sensor member; stop that one first",
        sensor_name(.sensor),
        link_name(.held_by)
    )]
    Busy {
        /// The sensor asked for.
        sensor: Sensor,
        /// The link that holds it.
        held_by: Link,
    },
    /// The sensor's stream does not fit the link (FW-4).
    #[error(
        "the XIAO's {} needs {needs_bytes_per_s} B/s and this link carries {link_has_bytes_per_s} B/s: use USB, or UART at a higher baud",
        sensor_name(.sensor)
    )]
    TooSlow {
        /// The sensor asked for.
        sensor: Sensor,
        /// What its stream needs.
        needs_bytes_per_s: u32,
        /// What the link carries.
        link_has_bytes_per_s: u32,
    },
    /// The board did not answer. On BLE this also covers a board never found
    /// in the scan and a link lost whose reconnects all failed: `xiao-api`
    /// reports the three alike.
    #[error(
        "no answer from the XIAO on {} after {attempts} attempt(s): the board is off, out of range or not running the sensor firmware{}",
        Named(.link),
        ble_note(.link)
    )]
    Timeout {
        /// The link tried.
        link: LinkSpec,
        /// How many times.
        attempts: u8,
    },
    /// No XIAO on the link.
    #[error(
        "no XIAO found on {}: is it plugged in, powered and flashed with the sensor firmware?",
        Named(.link)
    )]
    NotFound {
        /// The link searched.
        link: LinkSpec,
    },
    /// The link went away while in use.
    #[error(
        "the XIAO on {} is gone: unplugged, powered off or its port closed",
        Named(.link)
    )]
    Disconnected {
        /// The link lost.
        link: LinkSpec,
    },
    /// The operating system refused the link.
    #[error(
        "the operating system refused the XIAO's {} link ({kind}): check that the port exists, that you may open it and that no other program holds it",
        Named(.link)
    )]
    Io {
        /// The link refused.
        link: LinkSpec,
        /// What the operating system said.
        kind: std::io::ErrorKind,
    },
}

/// How a sensor reads in a message.
fn sensor_name(sensor: &Sensor) -> &'static str {
    match sensor {
        Sensor::Imu => "IMU",
        Sensor::Mic => "microphone",
    }
}

/// How a link kind reads in a message.
fn link_name(link: &Link) -> &'static str {
    match link {
        Link::Usb => "USB",
        Link::Ble => "BLE",
        Link::Uart => "UART",
    }
}

/// What a BLE timeout may also mean.
fn ble_note(link: &LinkSpec) -> &'static str {
    match link {
        LinkSpec::Ble { .. } => {
            "; on BLE it may also never have been found in the scan, or the link was lost and every reconnect failed"
        }
        LinkSpec::Usb | LinkSpec::Uart { .. } => "",
    }
}

/// How a link reads in a message: its kind, port or name.
struct Named<'a>(&'a LinkSpec);

impl core::fmt::Display for Named<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self.0 {
            LinkSpec::Usb => f.write_str("USB"),
            LinkSpec::Uart { port, baud } => write!(f, "UART {port} at {baud} baud"),
            LinkSpec::Ble { name, .. } => write!(f, "BLE \"{}\"", name.unwrap_or(BLE_NAME)),
        }
    }
}

/// One XIAO nRF52840 Sense as a sensor member: its link and the sensors
/// asked of it, `Xiao::usb().imu().mic()`. One IMU slot and one microphone
/// slot at most.
#[derive(Debug)]
// Read from X05 on.
#[allow(dead_code)]
pub struct Xiao {
    link: LinkSpec,
    imu: ImuSpec,
    mic: bool,
}

impl Xiao {
    /// A board on USB, the first XIAO found. Nothing asked yet.
    #[must_use]
    pub fn usb() -> Self {
        Self::on(LinkSpec::Usb)
    }

    /// A board on the serial `port` of its TX/RX pins, at 921600 baud.
    #[must_use]
    pub fn uart(port: &'static str) -> Self {
        Self::on(LinkSpec::Uart {
            port,
            baud: DEFAULT_BAUD,
        })
    }

    /// A board on BLE, advertised as `XIAO-Sense`, 3 attempts.
    #[must_use]
    pub fn ble() -> Self {
        Self::on(LinkSpec::Ble {
            name: None,
            attempts: DEFAULT_ATTEMPTS,
        })
    }

    /// The UART speed. Only for a UART link: it does nothing on another.
    #[must_use]
    pub fn baud(mut self, baud: u32) -> Self {
        if let LinkSpec::Uart { baud: speed, .. } = &mut self.link {
            *speed = baud;
        }
        self
    }

    /// The name the board advertises. Only for a BLE link: it does nothing
    /// on another.
    #[must_use]
    pub fn named(mut self, name: &'static str) -> Self {
        if let LinkSpec::Ble { name: named, .. } = &mut self.link {
            *named = Some(name);
        }
        self
    }

    /// Tries to connect, and to reconnect after a drop. Only for a BLE link:
    /// it does nothing on another.
    #[must_use]
    pub fn attempts(mut self, attempts: u8) -> Self {
        if let LinkSpec::Ble {
            attempts: tries, ..
        } = &mut self.link
        {
            *tries = attempts;
        }
        self
    }

    /// Both IMU halves: 104 Hz, ±4 g, ±500 °/s.
    #[must_use]
    pub fn imu(self) -> Self {
        self.accel().gyro()
    }

    /// The IMU halves `spec` names, at its rates and ranges.
    #[must_use]
    pub fn imu_with(mut self, spec: ImuSpec) -> Self {
        self.imu = spec;
        self
    }

    /// The gyroscope: 104 Hz, ±500 °/s.
    #[must_use]
    pub fn gyro(mut self) -> Self {
        self.imu.gyro = Some((DEFAULT_RATE, DEFAULT_GYRO));
        self
    }

    /// The accelerometer: 104 Hz, ±4 g.
    #[must_use]
    pub fn accel(mut self) -> Self {
        self.imu.accel = Some((DEFAULT_RATE, DEFAULT_ACCEL));
        self
    }

    /// The microphone: 16 kHz PCM.
    #[must_use]
    pub fn mic(mut self) -> Self {
        self.mic = true;
        self
    }

    /// A board on `link`, nothing asked.
    fn on(link: LinkSpec) -> Self {
        Self {
            link,
            imu: ImuSpec {
                accel: None,
                gyro: None,
            },
            mic: false,
        }
    }
}

impl seal::Adapter for Xiao {}

impl SensorHal for Xiao {
    type Error = XiaoError;

    fn needs(&self) -> Needs {
        unimplemented!("X05: needs")
    }

    fn open(&mut self) -> Result<(), XiaoError> {
        unimplemented!("X05: bring-up")
    }

    fn read(&mut self, _feed: Feed<'_>) {
        unimplemented!("X06: per-tick read")
    }

    fn close(&mut self) {
        unimplemented!("X06: exit")
    }
}
