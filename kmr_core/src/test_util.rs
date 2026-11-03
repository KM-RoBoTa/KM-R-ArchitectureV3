//! Test plumbing for the crates built on the core, behind the `test-util`
//! feature: a scripted [`Host`] and a recording adapter, so a test drives
//! ticks, reads, faults and overruns without a real clock, a real sleep or
//! the global signal bus. Nothing here is public without the feature.

use core::cell::{Cell, Ref, RefCell};
use core::num::NonZeroU32;
use std::time::{Duration, Instant};

use crate::hal::{
    Batch, Claim, Command, Drivers, Faults, Group, Hal, HardwareError, Mask, Overrides, Sample,
    Written, seal,
};
use crate::schedule::Drive;
use crate::signal::SignalMeta;
use crate::state::Q;
use crate::{Robot, Signal, SignalKind};

pub use crate::runtime::Host;

/// The script's time unit: [`Scripted`] instants are tenths of it.
pub const DT: Duration = Duration::from_millis(1);

// Index of each bus check: three at startup, then five per tick.
/// Bus check after the `PreInit` walk.
pub const AFTER_PRE_INIT: usize = 0;
/// Bus check after the `Init` walk.
pub const AFTER_INIT: usize = 1;
/// Bus check before `torque_on`.
pub const BEFORE_TORQUE_ON: usize = 2;
/// Bus check at the start of tick 0.
pub const TICK_START: usize = 3;
/// Bus check after the `First` walk of tick 0.
pub const AFTER_FIRST: usize = 4;
/// Bus check after the `EachTick` walk of tick 0.
pub const AFTER_EACH_TICK: usize = 5;
/// Bus check after the `Last` walk of tick 0.
pub const AFTER_LAST: usize = 6;
/// Bus check at the end of tick 0, before its sleep.
pub const TICK_END: usize = 7;
/// Bus checks per tick: add `k * PER_TICK` for tick `k`.
pub const PER_TICK: usize = 5;

/// Runs `robot` on `drivers` with `host` as its clock, sleep and bus.
/// Returns the signal that ended the run, like the real runtime.
pub fn run_scripted<const N: usize, CS: Drive<N>, H: Host, D: Drivers<N>>(
    robot: &mut Robot<N, CS>,
    host: &mut H,
    drivers: &mut D,
) -> Result<Signal, HardwareError<D::Error>> {
    crate::runtime::run_with(robot, host, drivers)
}

/// Whether `signal` is an emergency stop rather than a shutdown.
pub fn is_emergency_stop(signal: &Signal) -> bool {
    signal.kind() == SignalKind::EmergencyStop
}

/// Scripted host: never sleeps, never touches the global bus. A raised
/// signal stays raised, like on the real bus.
#[derive(Debug)]
pub struct Scripted {
    /// Instant 0 of the script.
    pub origin: Instant,
    /// Offset from `origin` returned by each call to `now`, in tenths of
    /// `DT`. Past the end of the script the last value is repeated.
    pub now_tenths: [u32; 16],
    /// Calls to `now` so far.
    pub now_calls: usize,
    /// The deadline of each of the first 16 sleeps.
    pub sleeps: [Option<Instant>; 16],
    /// Calls to `sleep_until` so far.
    pub sleep_calls: usize,
    /// Bus checks so far.
    pub poll_calls: usize,
    /// Check from which the shutdown is on the bus.
    pub shutdown_from: Option<usize>,
    /// Check from which the emergency stop is on the bus.
    pub estop_from: Option<usize>,
}

impl Scripted {
    /// A host whose clock stays at `origin` and whose bus stays quiet.
    pub fn new(origin: Instant) -> Self {
        Self {
            origin,
            now_tenths: [0; 16],
            now_calls: 0,
            sleeps: [None; 16],
            sleep_calls: 0,
            poll_calls: 0,
            shutdown_from: None,
            estop_from: None,
        }
    }

    /// The instants `now` returns, in tenths of `DT` after `origin`.
    pub fn script(mut self, tenths: &[u32]) -> Self {
        let last = tenths.last().copied().unwrap_or(0);
        for (i, slot) in self.now_tenths.iter_mut().enumerate() {
            *slot = tenths.get(i).copied().unwrap_or(last);
        }
        self
    }

    /// Raises a shutdown from bus check `check` on.
    pub fn shutdown_from(mut self, check: usize) -> Self {
        self.shutdown_from = Some(check);
        self
    }

    /// Raises an emergency stop from bus check `check` on.
    pub fn estop_from(mut self, check: usize) -> Self {
        self.estop_from = Some(check);
        self
    }

    /// The instant `tenths` tenths of `DT` after `origin`.
    pub fn at(&self, tenths: u32) -> Instant {
        self.origin + DT * tenths / 10
    }
}

impl Host for Scripted {
    fn now(&mut self) -> Instant {
        let i = self.now_calls.min(self.now_tenths.len() - 1);
        self.now_calls += 1;
        self.at(self.now_tenths.get(i).copied().unwrap_or(0))
    }

    fn sleep_until(&mut self, deadline: Instant) {
        if let Some(slot) = self.sleeps.get_mut(self.sleep_calls) {
            *slot = Some(deadline);
        }
        self.sleep_calls += 1;
    }

    fn poll(&mut self) -> Option<Signal> {
        let check = self.poll_calls;
        self.poll_calls += 1;
        assert!(check < 256, "the loop did not stop on the signal");

        let raised = |from: Option<usize>| from.is_some_and(|from| check >= from);
        let kind = if raised(self.estop_from) {
            SignalKind::EmergencyStop
        } else if raised(self.shutdown_from) {
            SignalKind::Shutdown
        } else {
            return None;
        };
        Some(Signal {
            kind,
            meta: SignalMeta::new("scripted", file!(), line!()),
        })
    }
}

/// An adapter call or a bucket walk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(missing_docs, reason = "one variant per call or walk of the same name")]
pub enum Call {
    Check,
    Open,
    Zero,
    Ping,
    Health,
    Configure,
    ReadAll,
    TorqueOn,
    Read,
    Write,
    Hold,
    TorqueOff,
    Close,
    PreInit,
    Init,
    PostInit,
    First,
    EachTick,
    Last,
}

/// What `write` or `hold` was handed: the positions, the masks and the
/// actuators of the batch (every one for `hold`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sent<const N: usize> {
    /// The positions of the command.
    pub q: [Q; N],
    /// The written masks of the command.
    pub written: Written<N>,
    /// The actuators of the batch.
    pub batch: Mask<N>,
}

/// Calls a [`Log`] holds.
pub const CAPACITY: usize = 96;

/// The log the recorder and the controllers share. Fixed capacity: no heap
/// in the core, tests included.
#[derive(Debug)]
pub struct Log<const N: usize> {
    calls: RefCell<([Call; CAPACITY], usize)>,
    /// The last command `write` was handed.
    pub write: Cell<Option<Sent<N>>>,
    /// The command `hold` was handed.
    pub hold: Cell<Option<Sent<N>>>,
}

impl<const N: usize> Default for Log<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Log<N> {
    /// An empty log.
    pub fn new() -> Self {
        Self {
            calls: RefCell::new(([Call::Check; CAPACITY], 0)),
            write: Cell::new(None),
            hold: Cell::new(None),
        }
    }

    /// Appends `call`. Panics past [`CAPACITY`].
    pub fn push(&self, call: Call) {
        let (calls, len) = &mut *self.calls.borrow_mut();
        *calls.get_mut(*len).expect("log capacity") = call;
        *len += 1;
    }

    /// The calls so far, in order.
    pub fn calls(&self) -> Ref<'_, [Call]> {
        Ref::map(self.calls.borrow(), |(calls, len)| {
            calls.get(..*len).unwrap_or(&[])
        })
    }
}

/// Position the recorder's `read_all` reads on every actuator.
pub const SEED: Q = Q(10.0);

/// A fake adapter: logs every call, can fail one bring-up step. Each `read`
/// moves actuator 0 by one and reports `faults`.
#[derive(Debug)]
pub struct Recorder<'a, const N: usize> {
    /// Where the calls go.
    pub log: &'a Log<N>,
    /// The bring-up step that fails, if any.
    pub fail: Option<Call>,
    /// The actuators it claims.
    pub owned: Mask<N>,
    /// The read cadence it claims for each of them.
    pub every: NonZeroU32,
    /// The fault bits each `read` reports.
    pub faults: Faults<N>,
}

impl<'a, const N: usize> Recorder<'a, N> {
    /// A recorder owning every actuator, reading every tick, never failing.
    pub fn new(log: &'a Log<N>) -> Self {
        Self {
            log,
            fail: None,
            owned: Mask::ALL,
            every: NonZeroU32::MIN,
            faults: Faults::default(),
        }
    }

    /// A recorder failing at `call`.
    pub fn failing(log: &'a Log<N>, call: Call) -> Self {
        Self {
            fail: Some(call),
            ..Self::new(log)
        }
    }

    fn step(&mut self, call: Call) -> Result<(), Failed> {
        self.log.push(call);
        match self.fail {
            Some(fail) if fail == call => Err(Failed(call)),
            _ => Ok(()),
        }
    }
}

/// The error of the step a [`Recorder`] was told to fail.
#[derive(Debug, PartialEq)]
pub struct Failed(pub Call);

impl core::fmt::Display for Failed {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?} failed", self.0)
    }
}

impl core::error::Error for Failed {}

impl<const N: usize> seal::Adapter for Recorder<'_, N> {}

impl<const N: usize> Hal<N> for Recorder<'_, N> {
    type Error = Failed;

    fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, Failed> {
        self.step(Call::Check)?;
        Ok(Claim {
            owned: self.owned,
            group: [Group(0); N],
            read_every: [self.every; N],
            read_cost: Duration::ZERO,
            write_cost: Duration::ZERO,
        })
    }

    fn open(&mut self) -> Result<(), Failed> {
        self.step(Call::Open)
    }

    fn zero(&mut self) -> Result<(), Failed> {
        self.step(Call::Zero)
    }

    fn ping(&mut self) -> Result<(), Failed> {
        self.step(Call::Ping)
    }

    fn health(&mut self) -> Result<(), Failed> {
        self.step(Call::Health)
    }

    fn configure(&mut self) -> Result<(), Failed> {
        self.step(Call::Configure)
    }

    fn read_all(&mut self, sample: Sample<'_, N>) -> Result<(), Failed> {
        *sample.q = [SEED; N];
        self.step(Call::ReadAll)
    }

    fn torque_on(&mut self) -> Result<(), Failed> {
        self.step(Call::TorqueOn)
    }

    fn read(&mut self, sample: Sample<'_, N>, faults: &mut Faults<N>) -> Mask<N> {
        self.log.push(Call::Read);
        if let Some(first) = sample.q.first_mut() {
            *first = *first + Q(1.0);
        }
        *faults = self.faults;
        Mask::ALL
    }

    fn write(&mut self, command: Command<'_, N>, batch: &Batch<'_, N>, _faults: &mut Faults<N>) {
        self.log.push(Call::Write);
        let mut actuators = Mask::NONE;
        for &actuator in batch.runs().flat_map(|(_group, run)| run) {
            actuators.set(actuator);
        }
        self.log.write.set(Some(Sent {
            q: *command.q,
            written: *command.written,
            batch: actuators,
        }));
    }

    fn hold(&mut self, target: Command<'_, N>) {
        self.log.push(Call::Hold);
        self.log.hold.set(Some(Sent {
            q: *target.q,
            written: *target.written,
            batch: Mask::ALL,
        }));
    }

    fn torque_off(&mut self) {
        self.log.push(Call::TorqueOff);
    }

    fn close(&mut self) {
        self.log.push(Call::Close);
    }
}
