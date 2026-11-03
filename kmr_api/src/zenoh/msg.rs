//! The two Zenoh messages and their wire form: fixed-size little-endian fields, no schema
//! crate (contracts/zenoh.md, data-model.md § Zenoh data).

use kmr_core::hal::{Faults as CoreFaults, Mask, TickFlags};

use crate::Faults;

/// A Desired from a remote source: values per field and one written mask per field; only the
/// fields whose bit is set are applied.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DesiredMsg<const N: usize> {
    /// The source's sequence number: at or below the last accepted, the message is dropped.
    pub seq: u64,
    /// The sender's clock, nanoseconds; carried and logged, never checked.
    pub sent_ns: u64,
    /// Positions, rad or mm.
    pub q: [f32; N],
    /// Velocities.
    pub qd: [f32; N],
    /// Torques or forces.
    pub tau: [f32; N],
    /// The written masks of `q`, `qd`, `tau`: bit `i` is actuator `i`.
    pub written: [u64; 3],
}

/// One tick's History slot as the gateway sends it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SampleMsg<const N: usize> {
    /// The tick index: gaps between two messages are ticks the gateway did not take.
    pub tick: u64,
    /// The oldest actuator's sample age in ticks; `None` when one was never read.
    pub age: Option<u32>,
    // DECISION: a slot without a sample (a run without hardware) sends NaN values.
    /// Positions read this tick, NaN without a sample.
    pub q: [f32; N],
    /// Velocities read this tick, NaN without a sample.
    pub qd: [f32; N],
    /// Torques read this tick, NaN without a sample.
    pub tau: [f32; N],
    /// The tick's fault bits and flags.
    pub faults: Faults<N>,
}

/// The bytes of a [`DesiredMsg`] for `n` actuators.
pub const fn desired_len(n: usize) -> usize {
    8 + 8 + 3 * 4 * n + 3 * 8
}

/// The bytes of a [`SampleMsg`] for `n` actuators.
pub const fn sample_len(n: usize) -> usize {
    8 + 1 + 4 + 3 * 4 * n + 6 * 8 + 4
}

impl<const N: usize> DesiredMsg<N> {
    /// No value, no bit.
    pub const EMPTY: Self = Self {
        seq: 0,
        sent_ns: 0,
        q: [0.0; N],
        qd: [0.0; N],
        tau: [0.0; N],
        written: [0; 3],
    };

    /// The wire form.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(desired_len(N));
        out.extend_from_slice(&self.seq.to_le_bytes());
        out.extend_from_slice(&self.sent_ns.to_le_bytes());
        for values in [&self.q, &self.qd, &self.tau] {
            put_f32s(&mut out, values);
        }
        for mask in self.written {
            out.extend_from_slice(&mask.to_le_bytes());
        }
        out
    }

    /// The message of `bytes`; `None` unless they are exactly [`desired_len`]`(N)` long.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != desired_len(N) {
            return None;
        }
        let mut r = Reader(bytes);
        let seq = r.u64()?;
        let sent_ns = r.u64()?;
        let q = r.f32s()?;
        let qd = r.f32s()?;
        let tau = r.f32s()?;
        let written = [r.u64()?, r.u64()?, r.u64()?];
        Some(Self {
            seq,
            sent_ns,
            q,
            qd,
            tau,
            written,
        })
    }
}

impl<const N: usize> SampleMsg<N> {
    /// The wire form.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(sample_len(N));
        out.extend_from_slice(&self.tick.to_le_bytes());
        out.push(u8::from(self.age.is_some()));
        out.extend_from_slice(&self.age.unwrap_or(0).to_le_bytes());
        for values in [&self.q, &self.qd, &self.tau] {
            put_f32s(&mut out, values);
        }
        let f = &self.faults;
        for mask in [
            f.stale(),
            f.bus_error(),
            f.offline(),
            f.rejected(),
            f.joint_limit(),
            f.envelope(),
        ] {
            out.extend_from_slice(&word(mask).to_le_bytes());
        }
        out.extend_from_slice(&f.tick_bits().to_le_bytes());
        out
    }

    /// The message of `bytes`; `None` unless they are exactly [`sample_len`]`(N)` long.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != sample_len(N) {
            return None;
        }
        let mut r = Reader(bytes);
        let tick = r.u64()?;
        let has_age = r.take::<1>()?[0] != 0;
        let age = r.u32()?;
        let q = r.f32s()?;
        let qd = r.f32s()?;
        let tau = r.f32s()?;
        let mut masks = [Mask::NONE; 6];
        for mask in &mut masks {
            *mask = from_word(r.u64()?);
        }
        let [stale, bus_error, offline, rejected, joint_limit, envelope] = masks;
        let mut actuators = CoreFaults::default();
        actuators.stale = stale;
        actuators.bus_error = bus_error;
        actuators.offline = offline;
        actuators.rejected = rejected;
        actuators.joint_limit = joint_limit;
        actuators.envelope = envelope;
        let tick_flags = TickFlags::from_bits(r.u32()?);
        Some(Self {
            tick,
            age: has_age.then_some(age),
            q,
            qd,
            tau,
            faults: Faults::new(actuators, tick_flags),
        })
    }
}

/// Appends `values`, little-endian.
fn put_f32s(out: &mut Vec<u8>, values: &[f32]) {
    for value in values {
        out.extend_from_slice(&value.to_le_bytes());
    }
}

/// The mask as one word: N is at most 64.
fn word<const N: usize>(mask: Mask<N>) -> u64 {
    mask.words().first().copied().unwrap_or(0)
}

/// The mask of `word`'s bits below N.
pub(crate) fn from_word<const N: usize>(word: u64) -> Mask<N> {
    let mut mask = Mask::NONE;
    for i in (0..N.min(64)).filter(|i| word & (1 << i) != 0) {
        mask.set(i);
    }
    mask
}

/// A cursor over a message's bytes.
struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    /// The next `K` bytes.
    fn take<const K: usize>(&mut self) -> Option<[u8; K]> {
        let (head, rest) = self.0.split_first_chunk::<K>()?;
        self.0 = rest;
        Some(*head)
    }

    fn u64(&mut self) -> Option<u64> {
        self.take().map(u64::from_le_bytes)
    }

    fn u32(&mut self) -> Option<u32> {
        self.take().map(u32::from_le_bytes)
    }

    /// `N` values.
    fn f32s<const N: usize>(&mut self) -> Option<[f32; N]> {
        let mut values = [0.0; N];
        for value in &mut values {
            *value = self.take().map(f32::from_le_bytes)?;
        }
        Some(values)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_desired_round_trips() {
        let msg = DesiredMsg::<3> {
            seq: 9,
            sent_ns: 12,
            q: [1.0, 2.0, 3.0],
            qd: [0.5; 3],
            tau: [-1.0; 3],
            written: [0b101, 0, 0b010],
        };
        let bytes = msg.encode();
        assert_eq!(bytes.len(), desired_len(3));
        assert_eq!(DesiredMsg::decode(&bytes), Some(msg));
        assert_eq!(DesiredMsg::<4>::decode(&bytes), None, "another N");
    }

    #[test]
    fn a_sample_round_trips() {
        let mut actuators = CoreFaults::<2>::default();
        actuators.stale.set(1);
        let msg = SampleMsg::<2> {
            tick: 77,
            age: Some(3),
            q: [1.0, f32::NAN],
            qd: [0.0; 2],
            tau: [0.0; 2],
            faults: Faults::new(actuators, TickFlags::UNAUTHORIZED),
        };
        let back = SampleMsg::<2>::decode(&msg.encode()).expect("decodes");
        assert_eq!((back.tick, back.age), (77, Some(3)));
        assert!(back.q[1].is_nan() && back.q[0] == 1.0);
        assert_eq!(back.faults, msg.faults);
        let none = SampleMsg { age: None, ..msg };
        assert_eq!(
            SampleMsg::<2>::decode(&none.encode()).map(|m| m.age),
            Some(None)
        );
    }
}
