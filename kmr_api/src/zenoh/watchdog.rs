//! Remote sources: who may publish Desired, and the watchdog that raises the remote-timeout
//! flag on silence (SRS FR-13.3, FR-13.5).

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use kmr_core::hal::{ExternalFlags, TickFlags};

use super::{GatewayError, keys};

/// A remote source being declared: [`Remote::allow`] names it, [`Allowed::timeout`] finishes
/// it. Only a finished [`Remote`] reaches
/// [`Robot::add_remote_source`](crate::Robot::add_remote_source), which also requires its
/// fallback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Allowed {
    identity: &'static str,
}

/// A remote source: the identity allowed to publish Desired, and how long it may stay silent
/// before its fallback runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Remote {
    pub(crate) identity: &'static str,
    pub(crate) timeout: Duration,
}

impl Remote {
    /// Allows `identity` (its certificate's common name, or its name on loopback) to publish
    /// Desired on `kmr/<instance>/desired/<identity>`.
    pub const fn allow(identity: &'static str) -> Allowed {
        Allowed { identity }
    }

    /// The identity.
    pub const fn identity(&self) -> &'static str {
        self.identity
    }

    /// How long it may stay silent.
    pub const fn timeout(&self) -> Duration {
        self.timeout
    }
}

impl Allowed {
    /// The source falls silent once no Desired of it was accepted for `timeout`.
    pub const fn timeout(self, timeout: Duration) -> Remote {
        Remote {
            identity: self.identity,
            timeout,
        }
    }
}

/// The checked sources: each identity one key chunk, once.
pub(crate) fn checked(sources: &[Remote]) -> Result<(), GatewayError> {
    for (i, source) in sources.iter().enumerate() {
        if !keys::is_chunk(source.identity) {
            return Err(GatewayError::Name {
                name: source.identity,
            });
        }
        if sources[..i].iter().any(|s| s.identity == source.identity) {
            return Err(GatewayError::TwoSources {
                identity: source.identity,
            });
        }
    }
    Ok(())
}

/// The tick's watchdog: counts each source's silence in ticks and raises the remote-timeout
/// flag while one is silent past its timeout. Atomic loads only.
// DECISION: on the tick, not the gateway thread: the loop takes and clears the external flags
// each tick, so a timer on another thread would race that take and flicker the fallback; the
// silence counted in ticks is exact on a scripted clock too. The flag goes through the core's
// external flags, so it shows on the next tick: within timeout + 1 tick.
pub(crate) struct Watchdog {
    watch: Vec<Watch>,
    flags: &'static ExternalFlags,
}

/// One source's silence, as the tick sees it.
struct Watch {
    heard: Arc<AtomicU64>,
    seen: u64,
    silent: Duration,
    timeout: Duration,
}

impl Watchdog {
    /// The watchdog of `sources`, and for each the count of its accepted Desired, which the
    /// gateway's rule bumps.
    pub(crate) fn new(
        sources: &[Remote],
        flags: &'static ExternalFlags,
    ) -> (Self, Vec<Arc<AtomicU64>>) {
        let heard: Vec<Arc<AtomicU64>> = sources.iter().map(|_| Arc::default()).collect();
        let watch = sources
            .iter()
            .zip(&heard)
            .map(|(source, heard)| Watch {
                heard: Arc::clone(heard),
                seen: 0,
                silent: Duration::ZERO,
                timeout: source.timeout,
            })
            .collect();
        (Self { watch, flags }, heard)
    }

    /// No source.
    #[cfg(test)]
    pub(crate) fn none(flags: &'static ExternalFlags) -> Self {
        Self {
            watch: Vec::new(),
            flags,
        }
    }

    /// One tick of `dt`: raises the remote-timeout flag when a source has been silent for at
    /// least its timeout.
    pub(crate) fn tick(&mut self, dt: Duration) {
        let mut silent = false;
        for watch in &mut self.watch {
            let heard = watch.heard.load(Ordering::Acquire);
            if heard == watch.seen {
                watch.silent = watch.silent.saturating_add(dt);
            } else {
                watch.seen = heard;
                watch.silent = Duration::ZERO;
            }
            silent |= watch.silent >= watch.timeout;
        }
        if silent {
            self.flags.raise(TickFlags::REMOTE_TIMEOUT);
        }
    }
}
