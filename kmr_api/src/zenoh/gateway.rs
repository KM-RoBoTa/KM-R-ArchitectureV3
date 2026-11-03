//! The gateway thread: owns the Zenoh session, publishes each sample it takes, keeps the ring
//! of the last `HISTORY_DEPTH` and serves it, hands Desired to the tick (SRS FR-7.1, FR-13.4).

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use ::zenoh::Wait;
use ::zenoh::handlers::FifoChannelHandler;
use ::zenoh::pubsub::{Publisher, Subscriber};
use ::zenoh::query::{Query, Queryable};
use ::zenoh::sample::Sample;

use crate::slot::{LatestReader, LatestWriter};
use crate::state::HISTORY_DEPTH;

use super::msg::{DesiredMsg, SampleMsg};
use super::{GatewayError, keys};

/// How long the gateway thread sleeps when nothing came in.
// DECISION: a poll with a short sleep, not a wake-up from the tick: the tick side stays one
// atomic swap, no futex call.
const POLL: Duration = Duration::from_micros(200);

/// The gateway's counters, atomics read outside the tick: clone it from
/// [`Gateway::counters`](super::Gateway::counters) before the robot runs.
#[derive(Clone, Debug, Default)]
pub struct Counters(pub(crate) Arc<Shared>);

/// The atomics behind [`Counters`].
#[derive(Debug, Default)]
pub(crate) struct Shared {
    /// Samples the tick put that the gateway thread never took.
    pub(crate) missed_samples: Arc<AtomicU64>,
    /// Desired the gateway put that the tick never took.
    pub(crate) superseded_desired: Arc<AtomicU64>,
    /// Samples the gateway took and published.
    pub(crate) published: AtomicU64,
    /// Desired put for the tick.
    pub(crate) accepted: AtomicU64,
    /// Desired from an identity that is not a remote source.
    pub(crate) unauthorized: AtomicU64,
    /// Desired whose `seq` was at or below the source's last accepted one.
    pub(crate) out_of_order: AtomicU64,
    /// Desired that did not decode for this robot's N.
    pub(crate) malformed: AtomicU64,
}

impl Counters {
    /// Samples the tick put that the gateway thread never took (SRS section 7 Zenoh).
    pub fn missed_samples(&self) -> u64 {
        self.0.missed_samples.load(Ordering::Relaxed)
    }

    /// Desired overwritten before a tick took them.
    pub fn superseded_desired(&self) -> u64 {
        self.0.superseded_desired.load(Ordering::Relaxed)
    }

    /// Samples taken and published on `kmr/<instance>/history`.
    pub fn published(&self) -> u64 {
        self.0.published.load(Ordering::Relaxed)
    }

    /// Desired that passed every check and were handed to the tick.
    pub fn accepted(&self) -> u64 {
        self.0.accepted.load(Ordering::Relaxed)
    }

    /// Desired dropped: their identity is not a remote source (SRS FR-13.2).
    pub fn unauthorized(&self) -> u64 {
        self.0.unauthorized.load(Ordering::Relaxed)
    }

    /// Desired dropped: their `seq` was at or below the last accepted (SRS FR-13.3).
    pub fn out_of_order(&self) -> u64 {
        self.0.out_of_order.load(Ordering::Relaxed)
    }

    /// Desired dropped: their length does not match this robot's N.
    pub fn malformed(&self) -> u64 {
        self.0.malformed.load(Ordering::Relaxed)
    }
}

/// Bumps `counter` by one.
pub(crate) fn bump(counter: &AtomicU64) {
    counter.fetch_add(1, Ordering::Relaxed);
}

/// The last samples the gateway took, oldest first, each with its tick index.
pub(crate) struct Ring<const N: usize>(VecDeque<SampleMsg<N>>);

impl<const N: usize> Ring<N> {
    /// An empty ring.
    pub(crate) fn new() -> Self {
        Self(VecDeque::with_capacity(HISTORY_DEPTH))
    }

    /// Keeps `sample`, dropping the oldest past `HISTORY_DEPTH`.
    pub(crate) fn push(&mut self, sample: SampleMsg<N>) {
        if self.0.len() == HISTORY_DEPTH {
            self.0.pop_front();
        }
        self.0.push_back(sample);
    }

    /// The samples, oldest first.
    pub(crate) fn iter(&self) -> impl Iterator<Item = &SampleMsg<N>> {
        self.0.iter()
    }
}

/// What the gateway thread does with one Desired: the source rule, then the slot.
pub(crate) trait Inbox<const N: usize>: Send + 'static {
    /// Checks the Desired published on `key` and hands it to the tick, or drops and counts it.
    fn receive(&mut self, key: &str, payload: &[u8], desired: &mut LatestWriter<DesiredMsg<N>>);
}

/// The Zenoh entities the thread drives, declared before the run so their errors are
/// `run()` errors.
pub(crate) struct Entities {
    pub(crate) session: ::zenoh::Session,
    pub(crate) history: Publisher<'static>,
    pub(crate) desired: Subscriber<FifoChannelHandler<Sample>>,
    pub(crate) ring: Queryable<FifoChannelHandler<Query>>,
}

impl Entities {
    /// Declares the publisher, the Desired subscriber and the ring queryable of `instance`.
    pub(crate) fn declare(session: ::zenoh::Session, instance: &str) -> Result<Self, GatewayError> {
        let history = session
            .declare_publisher(keys::history(instance))
            .wait()
            .map_err(GatewayError::session)?;
        let desired = session
            .declare_subscriber(keys::desired_any(instance))
            .wait()
            .map_err(GatewayError::session)?;
        let ring = session
            .declare_queryable(keys::ring(instance))
            .wait()
            .map_err(GatewayError::session)?;
        Ok(Self {
            session,
            history,
            desired,
            ring,
        })
    }
}

/// The running gateway thread; dropping it stops the thread and closes the session.
pub(crate) struct Running {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Drop for Running {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            // A panic of the gateway thread was already printed; the run is over either way.
            let _ = thread.join();
        }
    }
}

/// Spawns the gateway thread over `entities`.
pub(crate) fn spawn<const N: usize, I: Inbox<N>>(
    entities: Entities,
    mut inbox: I,
    mut desired: LatestWriter<DesiredMsg<N>>,
    mut samples: LatestReader<SampleMsg<N>>,
    counters: Counters,
) -> Result<Running, GatewayError> {
    let stop = Arc::new(AtomicBool::new(false));
    let stopped = Arc::clone(&stop);
    let thread = thread::Builder::new()
        .name("kmr-gateway".into())
        .spawn(move || {
            let mut ring = Ring::<N>::new();
            while !stopped.load(Ordering::Relaxed) {
                let mut idle = true;
                while let Ok(Some(sample)) = entities.desired.try_recv() {
                    idle = false;
                    let payload = sample.payload().to_bytes();
                    inbox.receive(sample.key_expr().as_str(), &payload, &mut desired);
                }
                if let Some(sample) = samples.take() {
                    idle = false;
                    ring.push(sample);
                    if entities.history.put(sample.encode()).wait().is_ok() {
                        bump(&counters.0.published);
                    }
                }
                while let Ok(Some(query)) = entities.ring.try_recv() {
                    idle = false;
                    for sample in ring.iter() {
                        // A query whose asker left fails its reply: nothing to do.
                        let _ = query.reply(query.key_expr(), sample.encode()).wait();
                    }
                }
                if idle {
                    thread::sleep(POLL);
                }
            }
            // Undeclares everything; the run is over, a close error has no one to go to.
            let _ = entities.session.close().wait();
        })
        .map_err(|e| GatewayError::Session(e.to_string()))?;
    Ok(Running {
        stop,
        thread: Some(thread),
    })
}
