//! The SocketCAN transport: one non-blocking `CAN_RAW` socket per interface,
//! through rustix's safe API only.
//!
//! Binding a socket to its interface takes a `sockaddr_can`, which safe
//! rustix cannot build: `open` resolves the interface, then fails with
//! [`TransportError::BindUnsupported`] (docs/deferred.md). Sending,
//! receiving and waiting are written and ready for it.

use core::num::NonZeroU32;
use core::time::Duration;
use std::os::fd::OwnedFd;

use rustix::event::{PollFd, PollFlags, Timespec, poll};
use rustix::io::Errno;
use rustix::net::netdevice::name_to_index;
use rustix::net::{AddressFamily, Protocol, SocketFlags, SocketType, socket_with};

use super::transport::{Transport, TransportError};

/// `CAN_RAW`, linux/can.h.
const CAN_RAW: Protocol = Protocol::from_raw(NonZeroU32::MIN);
/// Bytes of a classic `struct can_frame`.
const CAN_MTU: usize = 16;
/// Data bytes of a frame.
const DATA_LEN: u8 = 8;

/// The SocketCAN transport over `NB` interfaces.
#[derive(Debug)]
pub struct SocketCan<const NB: usize> {
    /// Socket of each bus, once opened.
    sockets: [Option<OwnedFd>; NB],
}

impl<const NB: usize> SocketCan<NB> {
    /// A transport with no socket open.
    pub const fn new() -> Self {
        Self {
            sockets: [const { None }; NB],
        }
    }

    /// The socket of bus `bus`.
    fn socket(&self, bus: usize) -> Result<&OwnedFd, TransportError> {
        self.sockets
            .get(bus)
            .and_then(Option::as_ref)
            .ok_or(TransportError::NotOpen { bus })
    }
}

impl<const NB: usize> Default for SocketCan<NB> {
    fn default() -> Self {
        Self::new()
    }
}

/// The `struct can_frame` of `data` to standard id `id`.
fn encode(id: u8, data: [u8; 8]) -> [u8; CAN_MTU] {
    let mut frame = [0; CAN_MTU];
    let (head, payload) = frame.split_at_mut(8);
    let (can_id, rest) = head.split_at_mut(4);
    can_id.copy_from_slice(&u32::from(id).to_ne_bytes());
    if let Some(len) = rest.first_mut() {
        *len = DATA_LEN;
    }
    payload.copy_from_slice(&data);
    frame
}

/// The data of a received `struct can_frame`; `None` for a remote or error
/// frame, or one of fewer than 8 data bytes.
fn decode(frame: &[u8; CAN_MTU]) -> Option<[u8; 8]> {
    /// `CAN_RTR_FLAG | CAN_ERR_FLAG`.
    const NOT_DATA: u32 = 0x4000_0000 | 0x2000_0000;
    let (head, payload) = frame.split_at(8);
    let can_id = u32::from_ne_bytes(head.get(..4)?.try_into().ok()?);
    if can_id & NOT_DATA != 0 || head.get(4).copied() != Some(DATA_LEN) {
        return None;
    }
    payload.try_into().ok()
}

/// Binds `socket` to interface `index`: needs a `sockaddr_can`, which only
/// an `unsafe impl SocketAddrArg` can give rustix (docs/deferred.md).
fn bind(_socket: &OwnedFd, _index: u32) -> Result<(), TransportError> {
    Err(TransportError::BindUnsupported)
}

impl<const NB: usize> Transport for SocketCan<NB> {
    fn open(&mut self, interfaces: &[&'static str]) -> Result<(), (usize, TransportError)> {
        for (bus, &interface) in interfaces.iter().enumerate() {
            let opened = socket_with(
                AddressFamily::CAN,
                SocketType::RAW,
                SocketFlags::NONBLOCK | SocketFlags::CLOEXEC,
                Some(CAN_RAW),
            )
            .map_err(|errno| TransportError::Socket(errno.raw_os_error()))
            .and_then(|socket| {
                let index =
                    name_to_index(&socket, interface).map_err(|_| TransportError::NoInterface)?;
                bind(&socket, index)?;
                Ok(socket)
            });
            match (opened, self.sockets.get_mut(bus)) {
                (Ok(socket), Some(slot)) => *slot = Some(socket),
                (Ok(_), None) => {
                    self.close();
                    return Err((bus, TransportError::NotOpen { bus }));
                }
                (Err(error), _) => {
                    self.close();
                    return Err((bus, error));
                }
            }
        }
        Ok(())
    }

    fn send(&mut self, bus: usize, id: u8, data: [u8; 8]) -> Result<(), TransportError> {
        let socket = self.socket(bus)?;
        match rustix::io::write(socket, &encode(id, data)) {
            Ok(CAN_MTU) => Ok(()),
            Ok(_) => Err(TransportError::Io(Errno::IO.raw_os_error())),
            Err(Errno::AGAIN | Errno::NOBUFS) => Err(TransportError::Full),
            Err(errno) => Err(TransportError::Io(errno.raw_os_error())),
        }
    }

    fn recv(&mut self, bus: usize) -> Result<Option<[u8; 8]>, TransportError> {
        let socket = self.socket(bus)?;
        let mut frame = [0; CAN_MTU];
        loop {
            match rustix::io::read(socket, &mut frame) {
                Ok(CAN_MTU) => match decode(&frame) {
                    Some(data) => return Ok(Some(data)),
                    None => continue,
                },
                Ok(_) => continue,
                Err(Errno::AGAIN) => return Ok(None),
                Err(Errno::INTR) => continue,
                Err(errno) => return Err(TransportError::Io(errno.raw_os_error())),
            }
        }
    }

    fn wait(&mut self, bus: usize, timeout: Duration) -> Result<Option<[u8; 8]>, TransportError> {
        if let Some(data) = self.recv(bus)? {
            return Ok(Some(data));
        }
        let socket = self.socket(bus)?;
        let timeout = Timespec {
            tv_sec: timeout.as_secs().try_into().unwrap_or(i64::MAX),
            tv_nsec: timeout.subsec_nanos().into(),
        };
        let mut fds = [PollFd::new(socket, PollFlags::IN)];
        match poll(&mut fds, Some(&timeout)) {
            Ok(0) => Ok(None),
            Ok(_) | Err(Errno::INTR) => self.recv(bus),
            Err(errno) => Err(TransportError::Io(errno.raw_os_error())),
        }
    }

    fn close(&mut self) {
        self.sockets = [const { None }; NB];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_is_the_kernels_can_frame() {
        let frame = encode(3, [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(&frame[..4], &3u32.to_ne_bytes());
        assert_eq!(frame[4], 8);
        assert_eq!(&frame[5..8], &[0, 0, 0]);
        assert_eq!(&frame[8..], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(decode(&frame), Some([1, 2, 3, 4, 5, 6, 7, 8]));
    }

    #[test]
    fn remote_error_and_short_frames_carry_no_reply() {
        let mut remote = encode(3, [0; 8]);
        remote[3] |= if cfg!(target_endian = "little") {
            0x40
        } else {
            0
        };
        remote[0] |= if cfg!(target_endian = "big") { 0x40 } else { 0 };
        assert_eq!(decode(&remote), None);
        let mut short = encode(3, [0; 8]);
        short[4] = 6;
        assert_eq!(decode(&short), None);
    }

    #[test]
    fn nothing_is_sent_or_read_before_open() {
        let mut can = SocketCan::<1>::new();
        assert_eq!(
            can.send(0, 1, [0; 8]),
            Err(TransportError::NotOpen { bus: 0 })
        );
        assert_eq!(can.recv(0), Err(TransportError::NotOpen { bus: 0 }));
    }
}
