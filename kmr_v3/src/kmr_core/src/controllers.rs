use crate::schedule::Schedule;

/// A controller that runs inline on the main tick loop.
pub struct Inline<S: Schedule, F>(pub(crate) S, pub(crate) F);
/// A main-loop controller that also owns persistent context `T` across ticks.
pub struct InlineWith<S: Schedule, T, F>(pub(crate) S, pub(crate) T, pub(crate) F);
/// A controller that runs on its OWN dedicated thread. No robot `State`.
pub struct Threaded<S: Schedule, T, F>(pub(crate) S, pub(crate) T, pub(crate) F);

impl<S: Schedule, F> Inline<S, F> {
    #[doc(hidden)]
    pub fn new(schedule: S, f: F) -> Self {
        Self(schedule, f)
    }
}
impl<S: Schedule, T, F> InlineWith<S, T, F> {
    #[doc(hidden)]
    pub fn new(schedule: S, ctx: T, f: F) -> Self {
        Self(schedule, ctx, f)
    }
}
impl<S: Schedule, T, F> Threaded<S, T, F> {
    #[doc(hidden)]
    pub fn new(schedule: S, ctx: T, f: F) -> Self {
        Self(schedule, ctx, f)
    }
}

pub trait Callable {
    fn call() {}
    fn call_all() {}
}
pub trait Concurent: Callable {}
pub trait Parallel: Callable {
    fn spawn_thread() {}
}
