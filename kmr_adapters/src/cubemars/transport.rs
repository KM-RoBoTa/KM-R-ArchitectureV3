//! The private seam between the adapter and the CAN bus: one implementation
//! on SocketCAN, plus the scripted one the tests run on.

use core::time::Duration;

/// Why a transport call failed.
#[cfg_attr(
    not(feature = "cubemars"),
    expect(
        dead_code,
        reason = "the SocketCAN transport fails with every variant, the scripted one with some"
    )
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(crate) enum TransportError {
    /// No network interface has this name.
    #[error("no network interface has this name")]
    NoInterface,
    /// The CAN socket could not be opened.
    #[error("the CAN socket could not be opened (errno {0})")]
    Socket(i32),
    /// Binding the socket needs a `sockaddr_can`, which safe rustix cannot
    /// build (docs/deferred.md).
    #[error("binding a CAN socket to its interface is not implemented yet (docs/deferred.md)")]
    BindUnsupported,
    /// The kernel's transmit queue is full: the frame was not queued.
    #[error("the transmit queue is full")]
    Full,
    /// A send or a receive failed.
    #[error("CAN I/O failed (errno {0})")]
    Io(i32),
    /// The call names a bus that is not open.
    #[error("bus {bus} is not open")]
    NotOpen {
        /// Index of the bus.
        bus: usize,
    },
}

/// The CAN bus as the adapter drives it: 8-byte data frames, standard ids.
/// Nothing allocates, `send` and `recv` never block.
pub(crate) trait Transport {
    /// Opens one socket per interface name, bus `k` on `interfaces[k]`; on
    /// a failure, the bus and why, nothing left open.
    fn open(&mut self, interfaces: &[&'static str]) -> Result<(), (usize, TransportError)>;

    /// Queues `data` to CAN id `id` on bus `bus`, without blocking.
    fn send(&mut self, bus: usize, id: u8, data: [u8; 8]) -> Result<(), TransportError>;

    /// The next frame received on bus `bus`, `None` when none is waiting.
    fn recv(&mut self, bus: usize) -> Result<Option<[u8; 8]>, TransportError>;

    /// The next frame on bus `bus`, waiting up to `timeout` for one.
    /// Bring-up only.
    fn wait(&mut self, bus: usize, timeout: Duration) -> Result<Option<[u8; 8]>, TransportError>;

    /// Closes every socket. Best effort.
    fn close(&mut self);
}
