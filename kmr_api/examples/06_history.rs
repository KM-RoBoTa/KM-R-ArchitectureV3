// Tutorial 06: looking back in time with History.
//
// `robot.q()` gives you the newest reading. But a controller often wants to
// know where the robot WAS: to see if a joint is moving, to compute a speed
// by hand, or to go back to the last value it trusted (08 does exactly that).
//
// So each tick, kmr keeps a copy of the state (positions, velocities,
// torques, faults) in a small History. Here we see how to read it back.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, EachTick, Sensors, Time};
use std::time::Duration;

kmr_api::robot!(
    XmBench,
    hardware = |hw| hw.baud("bus0", 1_000_000).read_every("bus0", 4)
);

const DEMO_LENGTH: Duration = Duration::from_millis(50);

// The same helper as in the earlier examples: it ends the demo after 50 ms
// with a graceful shutdown.
fn use_time(time: &Time, _robot: &mut State, _sensors: &Sensors) {
    let _period: Duration = time.dt();
    let _tick: u64 = time.tick();
    if let Some(late) = time.last_overrun() {
        println!("tick {} finished {late:?} late", time.tick());
    }

    if time.elapsed() >= DEMO_LENGTH {
        kmr_api::shutdown!("demo over");
    }
}

// The next two examples talk about previous states.
// `HISTORY_DEPTH` (4 ticks today) tells you how far you can go.
//
// This feature allows you to read the state from past "ticks".
//
// The depth counts back from the newest tick: `0` is the newest value (what
// `robot.q()` gives you), `1` the tick before, and so on up to
// `HISTORY_DEPTH - 1`.
//
// One trap: the `hardware` line above reads the bus only every 4th tick. On
// the ticks in between, History repeats the last reading, so neighbouring
// depths can hold the same value. `robot.age_at(i)` (see 08) tells you how
// old a reading really is.
fn previous_state_at(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    let index: usize = 1;
    let history_depth = 2;
    // Read the next line as "I want to read the state from 2 ticks ago".
    // The first two ticks have no such state yet: skip them.
    let Ok(prev) = robot.prev_q_at(index, history_depth) else {
        return;
    };
    // then simply multiply the value by 3.
    let desired = prev * 3_f32;

    robot.set_q_at(desired, index).unwrap();
}

// Same example but not constrained to a specific joint index.
fn previous_state(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    let history_depth = 2;
    // A `Result`: `Err` while History is not that deep yet, or for a depth at
    // or past `HISTORY_DEPTH`. 07 shows how to handle each case.
    let _previous = robot.prev_q(history_depth);
    // and so on...
}

fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;

    let robot = Robot::new();
    let robot = if cfg!(feature = "dynamixel") {
        robot.add_plugins(Bus::by_id("FT8ISZ6H").baud(1_000_000).bus("bus0"))
    } else {
        robot
    };
    robot
        .set_dt(Duration::from_millis(1u64))
        .add_controller(EachTick, previous_state)
        .add_controller(EachTick, previous_state_at)
        // .add_controller(OnCollision, (previous_state_at))
        // .add_controller(OnCollision, (pause, previous_state, stop))
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
