//! The XIAO adapter: the Seeed XIAO nRF52840 Sense's IMU and microphone as
//! one sensor member, over USB, UART or BLE (SRS FR-5.7).
//!
//! It wraps `xiao-api`, the board's SDK, whose link threads and heap stay
//! inside it. Building a [`Xiao`] opens nothing (SRS FR-5.5): `open` opens
//! the link and turns the asked sensors on, `read` takes what arrived without
//! waiting, in m/s² and rad/s, and `close` turns them off.

#[cfg(test)]
mod tests;

use core::time::Duration;

use kmr_core::hal::{Needs, SensorHal, seal};
use kmr_core::sensors::{Accel, Feed, Gyro};
use tracing::warn;
use xiao_api::link::AsLink;
use xiao_api::{Acceleration, Baud, Ble, Rotation, Uart, Usb};

pub use xiao_api::{AccelRange, GyroRange, Link, Rate};

/// Standard gravity, m/s² per g.
const STANDARD_GRAVITY: f32 = 9.806_65;
/// Microphone sample rate, Hz (FW-5).
const MIC_RATE_HZ: u32 = 16_000;
/// Samples in one audio block (FW-5).
const BLOCK_SAMPLES: u64 = 96;
// DECISION: every queued block is fed each read, in order; the bound only ends the loop, at the
// 4096 readings `xiao-api` queues per sensor (24 s of audio).
/// Audio blocks one read takes at most.
const MAX_BLOCKS: usize = 4096;
/// Periods of its slowest stream a board may stay silent before the tick
/// raises `sensor missing`.
const SILENT_PERIODS: u64 = 10;
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
///
/// At two different rates, each reading of the faster half comes with the
/// slower half's last value fed again, so neither half reads `None` between
/// its own readings. The reading's `age()` is then the age of the newest
/// half: the slower half's value can be older than it says.
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
#[derive(Debug, thiserror::Error)]
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
    /// The operating system or the Bluetooth stack refused the link.
    #[error("the link to the XIAO on {} failed: {source}; {}", Named(.link), io_hint(.link))]
    Io {
        /// The link refused.
        link: LinkSpec,
        /// `xiao-api`'s own error, in the operating system's or the
        /// Bluetooth stack's words, the port path included.
        #[source]
        source: std::io::Error,
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

/// What to check when a link of this kind is refused.
fn io_hint(link: &LinkSpec) -> &'static str {
    match link {
        LinkSpec::Usb => {
            "check that the board's port exists, that you may open it (on Linux, the `dialout` or `uucp` group) and that no other program holds it"
        }
        LinkSpec::Uart { .. } => {
            "check the port path, that you may open it and that no other program holds it, and that the baud matches the firmware's"
        }
        LinkSpec::Ble { .. } => {
            "check that the Bluetooth adapter is present and powered on, and that the board is powered, in range and advertising that name"
        }
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
pub struct Xiao {
    link: LinkSpec,
    imu: ImuSpec,
    mic: bool,
    /// The link, from `open` to `close`.
    opened: Option<Opened>,
    /// The IMU, on from `open` to `close`.
    halves: Option<Halves>,
    /// The microphone, on from `open` to `close`.
    microphone: Option<xiao_api::Mic>,
    /// A stream ended: the board is lost until the next `open`.
    lost: bool,
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

    /// The board on `link`, already open: `open` turns the sensors on over
    /// it instead of opening one.
    #[cfg(test)]
    pub(crate) fn on_link(mut self, link: Opened) -> Self {
        self.opened = Some(link);
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
            opened: None,
            halves: None,
            microphone: None,
            lost: false,
        }
    }

    /// What arrived since the last call, into `sink`, never waiting. Once a
    /// stream has ended, only `link_lost`, every call.
    fn drain(&mut self, sink: &mut impl Sink) {
        if self.lost {
            sink.link_lost();
            return;
        }
        let mut ended = None;
        if let Some(halves) = &mut self.halves {
            match halves.latest() {
                Ok(Some((accel, gyro))) => sink.motion(accel, gyro),
                Ok(None) => {}
                Err(error) => ended = Some((Sensor::Imu, error)),
            }
            sink.motion_lost(halves.dropped());
        }
        if let Some(microphone) = &self.microphone {
            for _ in 0..MAX_BLOCKS {
                match microphone.try_next() {
                    Ok(Some(block)) => sink.samples(block.samples()),
                    Ok(None) => break,
                    Err(error) => {
                        ended.get_or_insert((Sensor::Mic, error));
                        break;
                    }
                }
            }
            // DECISION: `xiao-api` counts lost blocks and the feed takes samples, so a block
            // counts 96.
            sink.samples_lost(microphone.dropped().0.saturating_mul(BLOCK_SAMPLES));
        }
        if let Some((sensor, error)) = ended {
            self.lose(sensor, error);
            sink.link_lost();
        }
    }

    // DECISION: the end of any one stream loses the whole board, the IMU and the microphone:
    // one link, one member; nothing reconnects it, as the link died or BLE ran out of attempts.
    /// The board is lost: logged once, here; the sensors stay held until `close`.
    fn lose(&mut self, sensor: Sensor, error: xiao_api::Error) {
        self.lost = true;
        let cause = fault(self.link, sensor, error);
        warn!(
            %cause,
            "XIAO lost: its sensors stop updating and `sensor missing` is raised every tick; kmr does not reconnect it"
        );
    }

    /// The first sensor asked, IMU before microphone; `None` when nothing is.
    fn first_sensor(&self) -> Option<Sensor> {
        if self.imu.accel.is_some() || self.imu.gyro.is_some() {
            Some(Sensor::Imu)
        } else if self.mic {
            Some(Sensor::Mic)
        } else {
            None
        }
    }

    /// Ten periods of the slowest stream asked, rounded up to the ms; zero
    /// when nothing is asked.
    fn max_silence(&self) -> Duration {
        let rates = [
            self.imu.accel.map(|(rate, _)| rate),
            self.imu.gyro.map(|(rate, _)| rate),
        ];
        let imu = rates.into_iter().flatten().map(|rate| {
            let millihertz = (rate.hertz() * 1000.0).round() as u64;
            (SILENT_PERIODS * 1_000_000).div_ceil(millihertz)
        });
        let mic = self
            .mic
            .then(|| (SILENT_PERIODS * BLOCK_SAMPLES * 1000).div_ceil(u64::from(MIC_RATE_HZ)));
        Duration::from_millis(imu.chain(mic).max().unwrap_or(0))
    }
}

/// An open link, the one the sensors are turned on over.
#[derive(Debug)]
pub(crate) enum Opened {
    /// The USB plug.
    Usb(Usb),
    /// The TX/RX pins.
    Uart(Uart),
    /// Bluetooth Low Energy.
    Ble(Ble),
}

impl Opened {
    /// Opens the link `spec` names.
    fn open(spec: LinkSpec) -> Result<Self, xiao_api::Error> {
        Ok(match spec {
            LinkSpec::Usb => Self::Usb(Usb::open()?),
            LinkSpec::Uart { port, baud } => Self::Uart(Uart::port(port).baud(Baud(baud)).open()?),
            LinkSpec::Ble { name, attempts } => {
                let ble = Ble::attempts(attempts);
                let ble = match name {
                    Some(name) => ble.named(name),
                    None => ble,
                };
                Self::Ble(ble.open()?)
            }
        })
    }

    /// Turns on over this link the IMU halves `imu` names, then the
    /// microphone when `mic`.
    fn turn_on(&self, imu: ImuSpec, mic: bool) -> Result<Turned, (Sensor, xiao_api::Error)> {
        match self {
            Self::Usb(link) => turn_on(link, imu, mic),
            Self::Uart(link) => turn_on(link, imu, mic),
            Self::Ble(link) => turn_on(link, imu, mic),
        }
    }
}

/// The IMU and the microphone `open` turned on.
type Turned = (Option<Halves>, Option<xiao_api::Mic>);

/// Turns on over `link` the IMU halves `imu` names, then the microphone when
/// `mic`. On a refusal, what it turned on is dropped, so off on the board
/// again, and the refused sensor is named.
fn turn_on(
    link: &impl AsLink,
    imu: ImuSpec,
    mic: bool,
) -> Result<Turned, (Sensor, xiao_api::Error)> {
    let halves = Halves::on(link, imu).map_err(|error| (Sensor::Imu, error))?;
    let microphone = if mic {
        Some(xiao_api::Mic::on(link).map_err(|error| (Sensor::Mic, error))?)
    } else {
        None
    };
    Ok((halves, microphone))
}

/// The IMU, on.
#[derive(Debug)]
enum Halves {
    /// Both halves at one rate: one stream, each reading carries both.
    Both(xiao_api::Imu),
    /// One half, or both at two rates: one stream per half.
    Apart {
        accel: Option<xiao_api::Accel>,
        gyro: Option<xiao_api::Gyro>,
        /// The newest reading of each half so far.
        last: Reading,
    },
}

/// One IMU reading in SI units: each half, `None` when it is off.
type Reading = (Option<Accel>, Option<Gyro>);

impl Halves {
    /// Turns on over `link` the halves `spec` names; `None` when it names
    /// none.
    fn on(link: &impl AsLink, spec: ImuSpec) -> Result<Option<Self>, xiao_api::Error> {
        let halves = match (spec.accel, spec.gyro) {
            (None, None) => return Ok(None),
            // DECISION: both halves at one rate go on as one `Imu`: one `ImuOn`, and each
            // reading carries both halves of one instant. At two rates they go on apart, since
            // `xiao-api`'s `Imu` sets one rate for both.
            (Some((rate, accel)), Some((same, gyro))) if rate == same => Self::Both(
                xiao_api::Imu::rate(rate)
                    .accel_range(accel)
                    .gyro_range(gyro)
                    .on(link)?,
            ),
            (accel, gyro) => {
                let accel = accel
                    .map(|(rate, range)| xiao_api::Accel::rate(rate).range(range).on(link))
                    .transpose()?;
                let gyro = gyro
                    .map(|(rate, range)| xiao_api::Gyro::rate(rate).range(range).on(link))
                    .transpose()?;
                Self::Apart {
                    accel,
                    gyro,
                    last: (None, None),
                }
            }
        };
        Ok(Some(halves))
    }

    /// The newest reading since the last call, in SI units; `None` when none
    /// came. Never waits.
    // DECISION: at two rates a reading carries the half that came and the other half's last
    // value, so neither reads `None` between its own readings; `age` is then the newer half's.
    fn latest(&mut self) -> Result<Option<Reading>, xiao_api::Error> {
        match self {
            Self::Both(imu) => Ok(imu
                .try_latest()?
                .map(|motion| (Some(si_accel(motion.accel)), Some(si_gyro(motion.gyro))))),
            Self::Apart { accel, gyro, last } => {
                let accel = accel
                    .as_ref()
                    .map(xiao_api::Accel::try_latest)
                    .transpose()?
                    .flatten();
                let gyro = gyro
                    .as_ref()
                    .map(xiao_api::Gyro::try_latest)
                    .transpose()?
                    .flatten();
                if accel.is_none() && gyro.is_none() {
                    return Ok(None);
                }
                if let Some(accel) = accel {
                    last.0 = Some(si_accel(accel));
                }
                if let Some(gyro) = gyro {
                    last.1 = Some(si_gyro(gyro));
                }
                Ok(Some(*last))
            }
        }
    }

    /// Readings lost since the IMU went on. Apart, both streams see every IMU
    /// message, so their counts agree; the larger is taken.
    fn dropped(&self) -> u64 {
        match self {
            Self::Both(imu) => imu.dropped().0,
            Self::Apart { accel, gyro, .. } => {
                let accel = accel.as_ref().map_or(0, |accel| accel.dropped().0);
                let gyro = gyro.as_ref().map_or(0, |gyro| gyro.dropped().0);
                accel.max(gyro)
            }
        }
    }
}

/// An acceleration in g, in m/s².
fn si_accel(reading: Acceleration) -> Accel {
    Accel([reading.x.0, reading.y.0, reading.z.0].map(|g| g * STANDARD_GRAVITY))
}

/// An angular rate in °/s, in rad/s.
fn si_gyro(reading: Rotation) -> Gyro {
    Gyro([reading.x.0, reading.y.0, reading.z.0].map(f32::to_radians))
}

/// Where one read goes: the engine's [`Feed`], or a test's recorder.
// DECISION: a private seam over `Feed`, which only the engine can build, so the per-tick read
// is tested on the fake board without running the engine and its process-wide signal bus.
trait Sink {
    /// One IMU reading, both halves as they are.
    fn motion(&mut self, accel: Option<Accel>, gyro: Option<Gyro>);
    /// PCM samples, oldest first.
    fn samples(&mut self, samples: &[i16]);
    /// The running total of IMU readings lost.
    fn motion_lost(&mut self, total: u64);
    /// The running total of samples lost.
    fn samples_lost(&mut self, total: u64);
    /// The board is lost.
    fn link_lost(&mut self);
}

/// The board's IMU and microphone are its member's first slot of each kind.
impl Sink for Feed<'_> {
    fn motion(&mut self, accel: Option<Accel>, gyro: Option<Gyro>) {
        self.imu(0, accel, gyro);
    }

    fn samples(&mut self, samples: &[i16]) {
        self.audio(0, samples);
    }

    fn motion_lost(&mut self, total: u64) {
        self.imu_dropped(0, total);
    }

    fn samples_lost(&mut self, total: u64) {
        self.audio_dropped(0, total);
    }

    fn link_lost(&mut self) {
        self.link_down();
    }
}

/// `error`, met turning `sensor` on over `link`, in the adapter's words: the
/// sensor and the link it knows added, nothing of `error` dropped.
fn fault(link: LinkSpec, sensor: Sensor, error: xiao_api::Error) -> XiaoError {
    match error {
        xiao_api::Error::Busy { held_by } => XiaoError::Busy { sensor, held_by },
        xiao_api::Error::TooSlow { needs, link_has } => XiaoError::TooSlow {
            sensor,
            needs_bytes_per_s: needs.0,
            link_has_bytes_per_s: link_has.0,
        },
        xiao_api::Error::Timeout { attempts } => XiaoError::Timeout { link, attempts },
        xiao_api::Error::NotFound => XiaoError::NotFound { link },
        xiao_api::Error::Disconnected => XiaoError::Disconnected { link },
        xiao_api::Error::Io(source) => XiaoError::Io { link, source },
    }
}

impl seal::Adapter for Xiao {}

impl SensorHal for Xiao {
    type Error = XiaoError;

    fn needs(&self) -> Needs {
        Needs {
            imus: u8::from(self.imu.accel.is_some() || self.imu.gyro.is_some()),
            mics: u8::from(self.mic),
            mic_rate_hz: if self.mic { MIC_RATE_HZ } else { 0 },
            max_silence: self.max_silence(),
        }
    }

    fn open(&mut self) -> Result<(), XiaoError> {
        // DECISION: a board asked for nothing opens no link: it would have nothing to read.
        let Some(first) = self.first_sensor() else {
            return Ok(());
        };
        let link = match self.opened.take() {
            Some(link) => link,
            // DECISION: a link that does not open names the first sensor asked; `xiao-api`
            // refuses a link with none of the sensor variants (busy, too slow) anyway.
            None => Opened::open(self.link).map_err(|error| fault(self.link, first, error))?,
        };
        // On a refusal `link` drops here, closing it, with whatever was turned on.
        let (halves, microphone) = link
            .turn_on(self.imu, self.mic)
            .map_err(|(sensor, error)| fault(self.link, sensor, error))?;
        self.halves = halves;
        self.microphone = microphone;
        self.opened = Some(link);
        self.lost = false;
        Ok(())
    }

    fn read(&mut self, mut feed: Feed<'_>) {
        self.drain(&mut feed);
    }

    fn close(&mut self) {
        // Dropping a sensor turns it off on the board; the link closes last.
        self.microphone = None;
        self.halves = None;
        self.opened = None;
    }
}
