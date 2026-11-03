//! The `test-util` feature drives the runtime from outside the core,
//! through its public items only.
#![cfg(feature = "test-util")]

use std::cell::Cell;
use std::time::Instant;

use kmr_core::test_util::{
    Call, Log, PER_TICK, Recorder, Scripted, TICK_END, is_emergency_stop, run_scripted,
};
use kmr_core::{EachTick, HCons, HNil, Robot, RobotState, Sensors, Time};

/// Actuators of the test robot.
const N: usize = 4;

#[test]
fn three_scripted_ticks_with_one_overrun_keep_the_grid() {
    let ticks = Cell::new(0);
    let log = Log::new();
    let mut robot = Robot::<N>::new()
        .set_dt(kmr_core::test_util::DT)
        .add_control_fn(EachTick, |_: &Time, _: &mut RobotState<N>, _: &Sensors| {
            ticks.set(ticks.get() + 1);
        });
    // `now` at arm, then at the start and end of each tick; tick 0 ends at
    // 1.5 dt, half a tick late.
    let mut host = Scripted::new(Instant::now())
        .script(&[0, 0, 15, 20, 25, 30, 35])
        .shutdown_from(TICK_END + 2 * PER_TICK);
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };

    let signal =
        run_scripted(&mut robot, &mut host, &mut drivers).expect("the recorder never fails");

    assert!(!is_emergency_stop(&signal));
    assert_eq!(ticks.get(), 3);
    assert_eq!(log.calls().iter().filter(|&&c| c == Call::Read).count(), 3);
    // The late tick skips the slot it overran: the next one starts at 2 dt.
    assert_eq!(host.sleep_calls, 2);
    assert_eq!(host.sleeps[..2], [Some(host.at(20)), Some(host.at(30))]);
}
