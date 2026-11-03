//! A scripted CAN bus for the tests: simulated AK60-6 motors that answer
//! every frame sent to them with their state, and a log of every call. Fixed
//! capacities, no heap: past one, the test panics.

use std::time::Duration;

use super::frame::{self, Feedback};
use super::transport::{Transport, TransportError};

/// Motors the bus may hold.
const MOTORS: usize = 8;
/// Buses the bus may hold.
pub(crate) const BUSES: usize = 2;
/// Frames waiting on one bus.
const QUEUE: usize = 64;
/// Calls the log keeps.
const CALLS: usize = 256;

/// One simulated motor.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Motor {
    /// Bus it is on.
    pub(crate) bus: usize,
    /// The state it answers with; `state.id` is its CAN id.
    pub(crate) state: Feedback,
    /// Whether it answers at all.
    pub(crate) answers: bool,
}

/// One transport call, as the adapter made it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Call {
    /// `open` with these interface names, the first `count`.
    Open {
        /// Interface names.
        names: [&'static str; BUSES],
        /// Names given.
        count: usize,
    },
    /// `send` of `data` to `id` on `bus`, failed or not.
    Send {
        /// Bus index.
        bus: usize,
        /// CAN id.
        id: u8,
        /// Data bytes.
        data: [u8; 8],
    },
    /// `close`.
    Close,
}

/// The frames waiting on one bus, oldest first.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Queue {
    frames: [[u8; 8]; QUEUE],
    head: usize,
    len: usize,
}

impl Queue {
    /// No frame.
    const EMPTY: Self = Self {
        frames: [[0; 8]; QUEUE],
        head: 0,
        len: 0,
    };

    /// Appends `data`.
    fn push(&mut self, data: [u8; 8]) {
        assert!(self.len < QUEUE, "scripted queue full");
        self.frames[(self.head + self.len) % QUEUE] = data;
        self.len += 1;
    }

    /// The oldest frame, removed.
    fn pop(&mut self) -> Option<[u8; 8]> {
        if self.len == 0 {
            return None;
        }
        let data = self.frames[self.head];
        self.head = (self.head + 1) % QUEUE;
        self.len -= 1;
        Some(data)
    }

    /// Drops every frame.
    pub(crate) fn clear(&mut self) {
        self.len = 0;
    }
}

/// The scripted transport.
#[derive(Debug)]
pub(crate) struct Scripted {
    motors: [Option<Motor>; MOTORS],
    /// Frames waiting on each bus.
    pub(crate) waiting: [Queue; BUSES],
    calls: [Option<Call>; CALLS],
    /// `open` fails on this bus with this error.
    pub(crate) open_error: Option<(usize, TransportError)>,
    /// Every `send` fails with this error.
    pub(crate) send_error: Option<TransportError>,
    /// The next `recv` on this bus fails with this error.
    pub(crate) recv_error: Option<(usize, TransportError)>,
}

impl Default for Scripted {
    fn default() -> Self {
        Self {
            motors: [None; MOTORS],
            waiting: [Queue::EMPTY; BUSES],
            calls: [None; CALLS],
            open_error: None,
            send_error: None,
            recv_error: None,
        }
    }
}

impl Scripted {
    /// Motors on bus 0 with ids `ids`, at rest at position 0, 30 °C, no
    /// fault, all answering.
    pub(crate) fn with_ids(ids: &[u8]) -> Self {
        let mut scripted = Self::default();
        for (slot, &id) in scripted.motors.iter_mut().zip(ids) {
            *slot = Some(Motor {
                bus: 0,
                state: Feedback {
                    id,
                    p: 0.0,
                    v: 0.0,
                    t: 0.0,
                    temperature: 30,
                    fault: 0,
                },
                answers: true,
            });
        }
        scripted
    }

    /// The motor with CAN id `id` on bus `bus`.
    pub(crate) fn motor(&mut self, bus: usize, id: u8) -> &mut Motor {
        self.motors
            .iter_mut()
            .flatten()
            .find(|motor| motor.bus == bus && motor.state.id == id)
            .expect("a scripted motor")
    }

    /// Puts `data` on bus `bus`, as another node would.
    pub(crate) fn inject(&mut self, bus: usize, data: [u8; 8]) {
        self.waiting[bus].push(data);
    }

    /// Every call, in order.
    pub(crate) fn calls(&self) -> impl Iterator<Item = &Call> {
        self.calls.iter().map_while(Option::as_ref)
    }

    /// The frames sent, in order, failed or not.
    pub(crate) fn sent(&self) -> impl Iterator<Item = (usize, u8, [u8; 8])> + '_ {
        self.calls().filter_map(|call| match *call {
            Call::Send { bus, id, data } => Some((bus, id, data)),
            _ => None,
        })
    }

    /// Forgets the calls made so far and the frames waiting.
    pub(crate) fn clear(&mut self) {
        self.calls = [None; CALLS];
        self.waiting.iter_mut().for_each(Queue::clear);
    }

    /// Logs `call`.
    fn log(&mut self, call: Call) {
        let slot = self.calls.iter_mut().find(|slot| slot.is_none());
        *slot.expect("scripted call log full") = Some(call);
    }
}

impl Transport for Scripted {
    fn open(&mut self, interfaces: &[&'static str]) -> Result<(), (usize, TransportError)> {
        let mut names = [""; BUSES];
        for (slot, &name) in names.iter_mut().zip(interfaces) {
            *slot = name;
        }
        self.log(Call::Open {
            names,
            count: interfaces.len(),
        });
        match self.open_error {
            Some(failure) => Err(failure),
            None => Ok(()),
        }
    }

    fn send(&mut self, bus: usize, id: u8, data: [u8; 8]) -> Result<(), TransportError> {
        self.log(Call::Send { bus, id, data });
        if let Some(error) = self.send_error {
            return Err(error);
        }
        let reply = self
            .motors
            .iter()
            .flatten()
            .find(|motor| motor.bus == bus && motor.state.id == id && motor.answers)
            .map(|motor| frame::reply(motor.state));
        if let Some(reply) = reply {
            self.waiting[bus].push(reply);
        }
        Ok(())
    }

    fn recv(&mut self, bus: usize) -> Result<Option<[u8; 8]>, TransportError> {
        if let Some((failing, error)) = self.recv_error
            && failing == bus
        {
            self.recv_error = None;
            return Err(error);
        }
        Ok(self.waiting[bus].pop())
    }

    fn wait(&mut self, bus: usize, _timeout: Duration) -> Result<Option<[u8; 8]>, TransportError> {
        self.recv(bus)
    }

    fn close(&mut self) {
        self.log(Call::Close);
    }
}
