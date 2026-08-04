use std::{
    marker::PhantomData,
    time::{Duration, Instant},
};

struct Simulation;

pub struct Time<T = ()> {
    startup: Instant,
    last_update: Option<Instant>,
    delta: Duration,
    elapsed: Duration,
    overrun: Option<Duration>,
    accumulated_overrun: Option<Duration>,
    _sim: PhantomData<T>,
}

impl Default for Time {
    fn default() -> Self {
        Self {
            startup: Instant::now(),
            last_update: None,
            delta: Duration::ZERO,
            elapsed: Duration::ZERO,
            overrun: None,
            accumulated_overrun: None,
            _sim: Default::default(),
        }
    }
}
// All methods are `pub(crate)`: `Time` is internal state. The only knob the
// user gets is `Robot::dt_ms`/`dt_us` (robot.rs), which forwards into
// `set_delta_ms`/`set_delta_us` below. Everything else here is written and
// read by the runtime, never by user code.
impl Time {
    // Setters
    pub(crate) fn set_last_update(&mut self, update_time: Instant) {
        self.last_update = Some(update_time);
    }
    pub(crate) fn set_delta_ms(&mut self, ms: u32) {
        self.delta = Duration::from_millis(ms as u64);
    }
    pub(crate) fn set_delta_us(&mut self, us: u32) {
        self.delta = Duration::from_micros(us as u64);
    }
    pub(crate) fn set_elapsed(&mut self, elapsed: Duration) {
        self.elapsed = elapsed;
    }
    pub(crate) fn set_overrun(&mut self, overrun: Duration) {
        self.overrun = Some(overrun);
    }
    pub(crate) fn set_accumulated_overrun(&mut self, accumulated_overrun: Duration) {
        self.accumulated_overrun = Some(accumulated_overrun);
    }

    // Getters
    pub(crate) fn startup(&self) -> &Instant {
        &self.startup
    }

    pub(crate) fn last_update(&self) -> Option<&Instant> {
        self.last_update.as_ref()
    }

    pub(crate) fn delta(&self) -> &Duration {
        &self.delta
    }

    pub(crate) fn elapsed(&self) -> &Duration {
        &self.elapsed
    }

    pub(crate) fn overrun(&self) -> Option<&Duration> {
        self.overrun.as_ref()
    }

    pub(crate) fn accumulated_overrun(&self) -> Option<&Duration> {
        self.accumulated_overrun.as_ref()
    }
}

impl Time<Simulation> {
    // TODO: pause time ? YAGNI ? Debugger-like time-step ?
}
