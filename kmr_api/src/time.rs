use std::time::Duration;

/// Read-only view of the runtime clock handed to a controller each tick.
///
/// Borrowing newtype: it holds a `&kmr_core::Time` the runtime lends for the
/// duration of the call and a zero point, never an owned copy of the clock, so
/// `kmr_core::Time` never appears in a public signature.
pub struct Time<'a> {
    inner: &'a kmr_core::Time,
    /// Where `elapsed` counts from: zero, or a preset's hand-over to its `then`.
    origin: Duration,
    /// Where `tick` counts from, likewise.
    first_tick: u64,
}

impl<'a> Time<'a> {
    /// Wrap a borrowed core clock. Called only at the api→core boundary
    /// (`ApiInline::call`); never by users.
    pub(crate) fn new(inner: &'a kmr_core::Time) -> Self {
        Time {
            inner,
            origin: Duration::ZERO,
            first_tick: 0,
        }
    }

    /// This clock with `elapsed` and `tick` counted from the `elapsed` and `tick` it read
    /// then.
    pub(crate) fn since(&self, elapsed: Duration, tick: u64) -> Self {
        Time {
            inner: self.inner,
            origin: self.origin + elapsed,
            first_tick: self.first_tick + tick,
        }
    }

    /// Time since the tick grid was armed, stamped at the start of the tick; under
    /// [`Home::then`](crate::presets::home::Home::then), since the hand-over.
    pub fn elapsed(&self) -> Duration {
        self.inner.elapsed().saturating_sub(self.origin)
    }

    /// The fixed tick duration.
    pub fn dt(&self) -> Duration {
        self.inner.dt()
    }

    /// Index of the tick in progress, 0 for the first one; under
    /// [`Home::then`](crate::presets::home::Home::then), 0 on the hand-over tick.
    pub fn tick(&self) -> u64 {
        self.inner.tick().saturating_sub(self.first_tick)
    }

    /// How many ticks finished after their deadline.
    pub fn overrun_count(&self) -> u64 {
        self.inner.overrun_count()
    }

    /// How late the previous tick finished, `None` if it was on time.
    pub fn last_overrun(&self) -> Option<Duration> {
        self.inner.last_overrun()
    }

    /// Sum of every overrun so far, zero when there was none.
    pub fn accumulated_overrun(&self) -> Duration {
        self.inner.accumulated_overrun()
    }
}
