//! Fallback controllers: a controller and its stand-in, picked each tick by the tick's faults
//! (SRS FR-13.5, FR-13.3).

use kmr_core::ControlFn;

use crate::robot::ApiInline;
use crate::{Faults, Sensors, State, Time};

/// A controller `F`, its fallback `G`, and the predicate `P` over this tick's [`Faults`] that
/// picks the fallback. Each tick exactly one of the two runs. Built by
/// [`Robot::add_controller_or`](crate::Robot::add_controller_or); a named type, so the builder
/// stores it as any controller: no `dyn`, no heap.
#[doc(hidden)]
pub struct Guarded<F, G, P> {
    pub(crate) controller: F,
    pub(crate) fallback: G,
    pub(crate) when: P,
}

impl<const N: usize, F, G, P> ControlFn<N> for ApiInline<Guarded<F, G, P>>
where
    F: Fn(&Time, &mut State<'_, N>, &Sensors),
    G: Fn(&Time, &mut State<'_, N>, &Sensors),
    P: Fn(&Faults<N>) -> bool,
{
    fn call(
        &self,
        time: &kmr_core::Time,
        state: &mut kmr_core::RobotState<N>,
        sensors: &kmr_core::Sensors,
    ) {
        let guarded = &self.0;
        let time = Time::new(time);
        let mut state = State::new(state);
        let sensors = Sensors::new(sensors);
        // DECISION: before the first tick (no slot yet) nothing is faulted: the controller runs.
        let faulted = state.faults().is_some_and(|faults| (guarded.when)(&faults));
        if faulted {
            (guarded.fallback)(&time, &mut state, &sensors);
        } else {
            (guarded.controller)(&time, &mut state, &sensors);
        }
    }
}

/// True when this tick's read failed on any actuator of `group`.
pub fn stale_in<const N: usize, const K: usize>(
    group: &[usize; K],
) -> impl Fn(&Faults<N>) -> bool + use<N, K> {
    let group = *group;
    move |faults| group.iter().any(|&i| faults.stale().get(i))
}

/// True when a remote source fell silent past its timeout this tick.
// DECISION: one flag for every remote source: what it means per source is open (U3), and
// the core keeps one tick flag, so the predicate takes no source yet.
pub fn remote_timeout<const N: usize>() -> impl Fn(&Faults<N>) -> bool {
    |faults| faults.remote_timeout()
}

/// True when `a` or `b` is.
pub fn any_of<const N: usize>(
    a: impl Fn(&Faults<N>) -> bool,
    b: impl Fn(&Faults<N>) -> bool,
) -> impl Fn(&Faults<N>) -> bool {
    move |faults| a(faults) || b(faults)
}
