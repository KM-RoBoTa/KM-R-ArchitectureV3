//! Serial port bring-up: resolution of a bus plugin's port, pre-flight,
//! flush and latency timer (FR-004, FR-008, FR-029, research R8). Never
//! called per tick.

use std::ffi::OsStr;
use std::fs::{self, File, OpenOptions};
use std::io::{ErrorKind, Read, Write};
use std::os::unix::ffi::OsStrExt;
use std::os::unix::fs::FileTypeExt;
use std::path::Path;

use rustix::fs::{Access, Mode, OFlags};
use rustix::termios::{self, QueueSelector};

use crate::binding::Port;

/// Where udev links each USB serial adapter under a name holding its serial
/// id. `ttyUSB` numbers change across reboots, serial ids do not.
pub(crate) const BY_ID: &str = "/dev/serial/by-id";

/// The sysfs directory of USB serial devices, one entry per `ttyUSBn`.
pub(crate) const USB_SERIAL_SYSFS: &str = "/sys/bus/usb-serial/devices";

/// Room for a resolved device path such as `/dev/ttyUSB0`.
const CAPACITY: usize = 256;

/// Why a port failed its bring-up. The caller names the bus and the port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PortError {
    /// No single entry holds the serial id, its link is dangling, or the
    /// path does not exist.
    NotFound,
    /// Not a character device, or not readable and writable by this process.
    Access,
    /// The port could not be opened or its queues flushed.
    Flush,
    /// The latency timer does not read back as 1 ms.
    LatencyTimer,
}

/// A resolved device path, held without allocation.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct DevicePath {
    /// Path bytes, meaningful up to `len`.
    bytes: [u8; CAPACITY],
    /// Number of bytes in use.
    len: usize,
}

impl DevicePath {
    /// Copies `path`, `None` when it does not fit.
    fn new(path: &Path) -> Option<Self> {
        let src = path.as_os_str().as_bytes();
        let mut bytes = [0; CAPACITY];
        bytes.get_mut(..src.len())?.copy_from_slice(src);
        Some(Self {
            bytes,
            len: src.len(),
        })
    }

    /// The device path.
    pub(crate) fn as_path(&self) -> &Path {
        Path::new(OsStr::from_bytes(
            self.bytes.get(..self.len).unwrap_or_default(),
        ))
    }
}

/// Shows the path, not the buffer.
impl core::fmt::Debug for DevicePath {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.as_path().fmt(f)
    }
}

/// The device behind `port`: the one entry of `by_id` holding its serial, or
/// its path as given, links followed (FR-029 (SRS FR-6.3)).
pub(crate) fn locate(by_id: &Path, port: Port) -> Result<DevicePath, PortError> {
    match port {
        Port::ById(serial) => resolve(by_id, serial),
        Port::ByDev(path) => {
            let device = fs::canonicalize(path).map_err(|_| PortError::NotFound)?;
            DevicePath::new(&device).ok_or(PortError::NotFound)
        }
    }
}

/// Resolves `serial` to the device behind the one entry of `by_id` whose name
/// contains it.
///
/// Two matching entries are refused, not guessed: a multi-port adapter lists
/// one entry per interface, and the serial its bus plugin gives must then name
/// more of the entry, e.g. `FT4TFSVK-if01`.
pub(crate) fn resolve(by_id: &Path, serial: &str) -> Result<DevicePath, PortError> {
    if serial.is_empty() {
        return Err(PortError::NotFound);
    }
    let mut found = None;
    for entry in fs::read_dir(by_id)
        .map_err(|_| PortError::NotFound)?
        .flatten()
    {
        let name = entry.file_name();
        if !name.to_str().is_some_and(|n| n.contains(serial)) {
            continue;
        }
        if found.is_some() {
            tracing::warn!(
                serial,
                by_id = %by_id.display(),
                "serial id matches more than one entry, name more of it in the bus plugin"
            );
            return Err(PortError::NotFound);
        }
        found = Some(entry.path());
    }
    let link = found.ok_or(PortError::NotFound)?;
    let device = fs::canonicalize(link).map_err(|_| PortError::NotFound)?;
    DevicePath::new(&device).ok_or(PortError::NotFound)
}

/// Checks that `path` is a character device this process can read and write,
/// before any vendor code sees it (FR-008).
pub(crate) fn preflight(path: &Path) -> Result<(), PortError> {
    let char_device = fs::metadata(path).is_ok_and(|meta| meta.file_type().is_char_device());
    if !char_device {
        return Err(PortError::Access);
    }
    rustix::fs::access(path, Access::READ_OK | Access::WRITE_OK).map_err(|_| PortError::Access)
}

/// Discards both queues of the port, so no byte left by a previous run
/// corrupts the first status packet.
///
/// No settle delay: v2 slept 100 ms after changing termios, and this leaves
/// termios alone.
pub(crate) fn flush(path: &Path) -> Result<(), PortError> {
    let fd = rustix::fs::open(
        path,
        OFlags::RDWR | OFlags::NONBLOCK | OFlags::NOCTTY | OFlags::CLOEXEC,
        Mode::empty(),
    )
    .map_err(|_| PortError::Flush)?;
    termios::tcflush(&fd, QueueSelector::IOFlush).map_err(|_| PortError::Flush)
}

/// Sets the latency timer of the USB serial `device` to 1 ms and reads it
/// back.
///
/// At the 16 ms default every status packet waits the timer out, 128 ms per
/// tick for 8 actuators in v2 (research R8). A native UART has no such file
/// and passes.
pub(crate) fn enforce_latency_timer(sysfs: &Path, device: &Path) -> Result<(), PortError> {
    let name = device.file_name().ok_or(PortError::LatencyTimer)?;
    let file = sysfs.join(name).join("latency_timer");
    let written = OpenOptions::new()
        .write(true)
        .truncate(true)
        .open(&file)
        .and_then(|mut timer| timer.write_all(b"1"));
    if let Err(error) = written
        && error.kind() == ErrorKind::NotFound
    {
        return Ok(());
    }
    // A refused write still passes when a udev rule has already set 1.
    let mut buf = [0; 8];
    let len = File::open(&file)
        .and_then(|mut timer| timer.read(&mut buf))
        .map_err(|_| PortError::LatencyTimer)?;
    let value = buf
        .get(..len)
        .and_then(|text| core::str::from_utf8(text).ok())
        .and_then(|text| text.trim().parse::<u8>().ok());
    if value == Some(1) {
        Ok(())
    } else {
        Err(PortError::LatencyTimer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::Permissions;
    use std::os::unix::fs::{PermissionsExt, symlink};
    use std::path::PathBuf;

    /// A directory of one test under the system temp dir, removed on drop.
    struct Scratch(PathBuf);

    impl Scratch {
        /// Creates the directory of `test`, unique to this process.
        fn new(test: &str) -> Self {
            let dir = std::env::temp_dir().join(format!("kmr_port_{}_{test}", std::process::id()));
            fs::remove_dir_all(&dir).ok();
            fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }

        /// A fake device file, and a `by-id` directory linking `entries` to it.
        fn by_id(&self, entries: &[&str]) -> (PathBuf, PathBuf) {
            let device = self.0.join("ttyUSB7");
            fs::write(&device, b"").unwrap();
            let by_id = self.0.join("by-id");
            fs::create_dir(&by_id).unwrap();
            for entry in entries {
                symlink(&device, by_id.join(entry)).unwrap();
            }
            (by_id, device)
        }

        /// A fake sysfs tree whose `ttyUSB0/latency_timer` holds `value`.
        fn sysfs(&self, value: &[u8]) -> PathBuf {
            let dir = self.0.join("ttyUSB0");
            fs::create_dir(&dir).unwrap();
            let timer = dir.join("latency_timer");
            fs::write(&timer, value).unwrap();
            timer
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            fs::remove_dir_all(&self.0).ok();
        }
    }

    /// A `by-id` entry as udev names it for an FTDI adapter.
    const FTDI: &str = "usb-FTDI_USB__-__Serial_Converter_FT4TFSVK-if00-port0";

    #[test]
    fn resolve_finds_the_entry_holding_the_serial() {
        let scratch = Scratch::new("resolve_finds");
        let (by_id, device) = scratch.by_id(&[FTDI, "usb-Other_AL02L1EU-if00-port0"]);

        let resolved = resolve(&by_id, "FT4TFSVK").unwrap();
        assert_eq!(resolved.as_path(), device.canonicalize().unwrap());
    }

    #[test]
    fn resolve_without_a_match_is_not_found() {
        let scratch = Scratch::new("resolve_none");
        let (by_id, _) = scratch.by_id(&[FTDI]);

        assert_eq!(resolve(&by_id, "AL02L1EU"), Err(PortError::NotFound));
        assert_eq!(resolve(&by_id, ""), Err(PortError::NotFound));
        assert_eq!(
            resolve(&scratch.0.join("absent"), "FT4TFSVK"),
            Err(PortError::NotFound)
        );
    }

    #[test]
    fn resolve_refuses_two_matching_entries() {
        let scratch = Scratch::new("resolve_two");
        let (by_id, _) = scratch.by_id(&[FTDI, "usb-FTDI_Quad_FT4TFSVK-if01-port0"]);

        assert_eq!(resolve(&by_id, "FT4TFSVK"), Err(PortError::NotFound));
        assert!(resolve(&by_id, "FT4TFSVK-if01").is_ok());
    }

    #[test]
    fn locate_takes_a_device_path_as_given_and_follows_links() {
        let scratch = Scratch::new("locate_by_dev");
        let (by_id, device) = scratch.by_id(&[FTDI]);
        let link = PathBuf::leak(by_id.join(FTDI));
        let link: &'static str = link.to_str().unwrap();

        let found = locate(&by_id, Port::ByDev(link)).unwrap();
        assert_eq!(found.as_path(), device.canonicalize().unwrap());
        assert_eq!(
            locate(&by_id, Port::ByDev("/dev/null"))
                .map(|found| found.as_path() == Path::new("/dev/null")),
            Ok(true)
        );
        assert_eq!(
            locate(&by_id, Port::ByDev("/dev/kmr-absent")),
            Err(PortError::NotFound)
        );
        assert_eq!(locate(&by_id, Port::ByDev("")), Err(PortError::NotFound));
    }

    #[test]
    fn latency_timer_is_written_and_read_back() {
        let scratch = Scratch::new("latency_written");
        let timer = scratch.sysfs(b"16\n");

        assert_eq!(
            enforce_latency_timer(&scratch.0, Path::new("/dev/ttyUSB0")),
            Ok(())
        );
        assert_eq!(fs::read(&timer).unwrap(), b"1");
    }

    #[test]
    fn latency_timer_absent_passes_as_a_native_uart() {
        let scratch = Scratch::new("latency_absent");

        assert_eq!(
            enforce_latency_timer(&scratch.0, Path::new("/dev/ttyS0")),
            Ok(())
        );
    }

    #[test]
    fn latency_timer_stuck_at_16_fails() {
        let scratch = Scratch::new("latency_stuck");
        let timer = scratch.sysfs(b"16\n");
        fs::set_permissions(&timer, Permissions::from_mode(0o444)).unwrap();
        // Root writes through the mode bits: nothing to observe.
        if OpenOptions::new().write(true).open(&timer).is_ok() {
            return;
        }

        assert_eq!(
            enforce_latency_timer(&scratch.0, Path::new("/dev/ttyUSB0")),
            Err(PortError::LatencyTimer)
        );
    }

    #[test]
    fn preflight_refuses_a_regular_or_missing_file() {
        let scratch = Scratch::new("preflight_regular");
        let file = scratch.0.join("ttyUSB0");
        fs::write(&file, b"").unwrap();

        assert_eq!(preflight(&file), Err(PortError::Access));
        assert_eq!(preflight(&scratch.0.join("absent")), Err(PortError::Access));
    }

    #[test]
    fn preflight_accepts_a_readable_writable_character_device() {
        assert_eq!(preflight(Path::new("/dev/null")), Ok(()));
    }

    #[test]
    fn flush_refuses_a_non_terminal() {
        let scratch = Scratch::new("flush_regular");
        let file = scratch.0.join("ttyUSB0");
        fs::write(&file, b"").unwrap();

        assert_eq!(flush(&file), Err(PortError::Flush));
    }
}
