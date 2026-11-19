//! `walk_cpp` over scripted ticks: the first goals are the all-zero posture, and two gait cycles
//! stay inside the joint range the actuators get.

// The Init check's half of it goes unused here.
#[allow(dead_code)]
#[path = "../src/leg.rs"]
mod leg;

use core::cell::Cell;
use std::time::Instant;

use kmr_api::test_util::{DT, Log, Recorder, Ticking, run_scripted};
use kmr_api::{EachTick, First, HCons, HNil, Policy, Sensors, Time};
use leg::{Gait, RANGE_HI, RANGE_LO};

kmr_api::robot!(model = "../../../controllers/models/geckoleg.urdf");

/// Runs `walk_cpp` at model speed 1 for `ticks` ticks of [`DT`]; calls `each` with the goals of
/// every tick, hip, knee, ankle, rad.
fn walk(ticks: u64, each: impl Fn([f32; 3])) {
    let log = Log::new();
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_controller(First, |time: &Time, _: &mut State, _: &Sensors| {
            if time.tick() > 0 {
                let sent = log.write.take().expect("the previous tick wrote");
                each([sent.q[0].0, sent.q[1].0, sent.q[2].0]);
            }
        })
        .add_controller(EachTick, kmr_api::c!(walk_cpp, with = Gait::new(1.0)));
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(
        robot,
        &mut Ticking::new(Instant::now(), ticks),
        &mut drivers,
    )
    .expect("runs");
}

#[test]
fn the_first_goals_are_the_all_zero_posture() {
    let first = Cell::new(None);
    walk(2, |q| {
        if first.get().is_none() {
            first.set(Some(q));
        }
    });
    let q = first.get().expect("tick 0 wrote");
    // FOOT0 is the zero posture's foot rounded to the millimetre: about 1 degree.
    assert!(q.iter().all(|q| q.abs() < 0.02), "{q:?}");
}

#[test]
fn two_cycles_stay_inside_the_declared_range() {
    let ticks = Cell::new(0);
    // Two cycles of 2 model seconds, at one goal a millisecond.
    walk(4001, |q| {
        ticks.set(ticks.get() + 1);
        for j in 0..3 {
            let q = f64::from(q[j]);
            assert!(
                (RANGE_LO[j]..=RANGE_HI[j]).contains(&q),
                "joint {j} at {q} rad, tick {}",
                ticks.get()
            );
        }
    });
    assert_eq!(ticks.get(), 4000);
}
