//! The SocketCAN transport: one non-blocking `CAN_RAW` socket per interface,
//! bound by interface name through the `socketcan` crate's safe API.

// DECISION: the `socketcan` crate (maintainer-approved 2026-11-03) binds
// the socket: safe rustix builds no `sockaddr_can`, and agents write no
// `unsafe`.

use core::time::Duration;
use std::io::{self, ErrorKind};

use socketcan::{CanDataFrame, CanFrame, CanSocket, EmbeddedFrame, Socket, StandardId};

use super::transport::{Transport, TransportError};

/// `ENODEV`: no interface has the name.
const ENODEV: i32 = 19;
/// `ENOBUFS`: the transmit queue is full.
const ENOBUFS: i32 = 105;

/// The SocketCAN transport over `NB` interfaces.
#[derive(Debug)]
pub struct SocketCan<const NB: usize> {
    /// Socket of each bus, once opened.
    sockets: [Option<CanSocket>; NB],
}

impl<const NB: usize> SocketCan<NB> {
    /// A transport with no socket open.
    pub const fn new() -> Self {
        Self {
            sockets: [const { None }; NB],
        }
    }

    /// The socket of bus `bus`.
    fn socket(&self, bus: usize) -> Result<&CanSocket, TransportError> {
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

/// The OS error number of `error`, 0 when it carries none.
fn errno(error: &io::Error) -> i32 {
    error.raw_os_error().unwrap_or(0)
}

/// The socket bound to `interface`, non-blocking.
fn open_one(interface: &str) -> Result<CanSocket, TransportError> {
    let socket = CanSocket::open(interface).map_err(|error| match error.raw_os_error() {
        Some(ENODEV) => TransportError::NoInterface,
        _ => TransportError::Socket(errno(&error)),
    })?;
    socket
        .set_nonblocking(true)
        .map_err(|error| TransportError::Socket(errno(&error)))?;
    Ok(socket)
}

/// The 8 data bytes of a received frame; `None` for a remote or error
/// frame, or a data frame of another length.
fn data(frame: &CanFrame) -> Option<[u8; 8]> {
    match frame {
        CanFrame::Data(data) => data.data().try_into().ok(),
        CanFrame::Remote(_) | CanFrame::Error(_) => None,
    }
}

impl<const NB: usize> Transport for SocketCan<NB> {
    fn open(&mut self, interfaces: &[&'static str]) -> Result<(), (usize, TransportError)> {
        for (bus, &interface) in interfaces.iter().enumerate() {
            match (open_one(interface), self.sockets.get_mut(bus)) {
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
        let frame = StandardId::new(u16::from(id))
            .and_then(|id| CanDataFrame::new(id, &data))
            .ok_or(TransportError::Io(0))?;
        socket
            .write_frame(&frame)
            .map_err(|error| match (error.kind(), error.raw_os_error()) {
                (ErrorKind::WouldBlock, _) | (_, Some(ENOBUFS)) => TransportError::Full,
                _ => TransportError::Io(errno(&error)),
            })
    }

    fn recv(&mut self, bus: usize) -> Result<Option<[u8; 8]>, TransportError> {
        let socket = self.socket(bus)?;
        loop {
            match socket.read_frame() {
                Ok(frame) => {
                    if let Some(data) = data(&frame) {
                        return Ok(Some(data));
                    }
                }
                Err(error) if error.kind() == ErrorKind::WouldBlock => return Ok(None),
                Err(error) if error.kind() == ErrorKind::Interrupted => {}
                Err(error) => return Err(TransportError::Io(errno(&error))),
            }
        }
    }

    fn wait(&mut self, bus: usize, timeout: Duration) -> Result<Option<[u8; 8]>, TransportError> {
        if let Some(data) = self.recv(bus)? {
            return Ok(Some(data));
        }
        match self.socket(bus)?.read_frame_timeout(timeout) {
            Ok(frame) => match data(&frame) {
                Some(data) => Ok(Some(data)),
                None => self.recv(bus),
            },
            Err(error)
                if matches!(
                    error.kind(),
                    ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::Interrupted
                ) =>
            {
                Ok(None)
            }
            Err(error) => Err(TransportError::Io(errno(&error))),
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
    fn nothing_is_sent_or_read_before_open() {
        let mut can = SocketCan::<1>::new();
        assert_eq!(
            can.send(0, 1, [0; 8]),
            Err(TransportError::NotOpen { bus: 0 })
        );
        assert_eq!(can.recv(0), Err(TransportError::NotOpen { bus: 0 }));
    }

    #[test]
    fn an_interface_no_machine_has_is_an_error_naming_its_bus() {
        let mut can = SocketCan::<2>::new();
        let error = can.open(&["nonexistent0", "nonexistent1"]);
        assert!(matches!(error, Err((0, _))), "{error:?}");
        assert_eq!(can.recv(0), Err(TransportError::NotOpen { bus: 0 }));
    }

    #[test]
    fn only_eight_byte_data_frames_carry_a_reply() {
        let id = StandardId::new(1).unwrap();
        let full = CanFrame::Data(CanDataFrame::new(id, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap());
        assert_eq!(data(&full), Some([1, 2, 3, 4, 5, 6, 7, 8]));
        let short = CanFrame::Data(CanDataFrame::new(id, &[1, 2]).unwrap());
        assert_eq!(data(&short), None);
    }
}
