use std::time::Duration;

/// Read-only view of the runtime clock handed to a controller each tick.
///
/// Borrowing newtype: it holds a `&kmr_core::Time` the runtime lends for the
/// duration of the call, never an owned copy — so wrapping it at the boundary
/// is free and `kmr_core::Time` never appears in a public signature.
pub struct Time<'a> {
    inner: &'a kmr_core::Time,
}

impl<'a> Time<'a> {
    /// Wrap a borrowed core clock. Called only at the api→core boundary
    /// (`ApiInline::call`); never by users.
    pub(crate) fn new(inner: &'a kmr_core::Time) -> Self {
        Time { inner }
    }

    /// Time since the tick grid was armed, stamped at the start of the tick.
    pub fn elapsed(&self) -> Duration {
        self.inner.elapsed()
    }

    /// The fixed tick duration.
    pub fn dt(&self) -> Duration {
        self.inner.dt()
    }

    /// Index of the tick in progress, 0 for the first one.
    pub fn tick(&self) -> u64 {
        self.inner.tick()
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
