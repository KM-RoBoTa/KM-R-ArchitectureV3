//! `swing` of src/swing.c over scripted ticks: actuators 0 and 2 move 0.1 rad from the newest
//! sample, nothing else is written.

use std::time::Instant;

use kmr_api::test_util::{DT, Log, PER_TICK, Recorder, Scripted, TICK_END, run_scripted};
use kmr_api::{EachTick, HCons, HNil, Q, Sensors, Time};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn swing; }

#[test]
fn swing_moves_actuators_0_and_2_from_the_newest_sample() {
    let log = Log::new();
    let robot = Robot::new().set_dt(DT).add_controller(
        EachTick,
        |time: &Time, state: &mut State, _: &Sensors| {
            assert!(kmr_api::ffi::call(swing, time, state, None::<&mut ()>).is_ok());
        },
    );
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 2 * PER_TICK);
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");

    let sent = log.write.get().expect("a write");
    // The recorder's reads move actuator 0 by one a tick from the seed: 3 reads by tick 2.
    let seed = kmr_api::test_util::SEED.0;
    assert_eq!(sent.q[0], Q(seed + 3.0 + 0.1));
    assert_eq!(sent.q[2], Q(seed + 0.1));
    assert_eq!(sent.written.q.iter().collect::<Vec<_>>(), [0, 2]);
    assert!(sent.written.qd.is_empty() && sent.written.tau.is_empty());
}
