// Reading and writing positions, and going home.
//
// Two sides to the robot's state, and they never mix:
//
// - READING (`robot.q()`, `robot.q_at(i)`) gives the latest values the
//   actuators reported.
// - WRITING (`robot.set_all_q(..)`, `robot.set_q_at(..)`) only stages a
//   command. kmr sends it once, at the end of the tick, then forgets it.
//   Write nothing, and no new command is sent.
//
// So reading `q` right after writing it still gives the reported value, not
// your command. And `Q` is a position: rad on a rotating joint, mm on a
// sliding one.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, EachTick, Q, Sensors, Time, presets::home::Home};
use std::time::Duration;

kmr_api::robot!(
    XmBench,
    hardware = |hw| hw.baud("bus0", 1_000_000).read_every("bus0", 4)
);

const DEMO_LENGTH: Duration = Duration::from_millis(50);

// The clock helper from 01: it stops the demo after `DEMO_LENGTH`.
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

// A home pose: every actuator at 0, reached in 3 s.
//
// Why a ramp? At power on, the robot is wherever you left it. Jumping straight
// to a target would snap every joint there at full speed. `Home` starts from
// the first sample it gets and moves in a straight line, gently.
const HOME: Home<N> = Home::new([Q(0.0); N], Duration::from_secs(3));

fn replace_all_q(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    // Let's start this example with a simple example.
    // Here, we want to replace all Q (positions) to an arbitrary
    // value.
    //
    // To do so, we will:
    //
    // 1) Read the current position with `robot.q()`.
    // 2) Replace for all q we receive its value by a new one (0, or q + 300).
    // 3) Commit the change with `robot.set_all_q()`.

    // Here, we read the current value of q. By current, we understand
    // "the latest read value from the actuators themselves".
    // A run without hardware reads nothing: no sample to work from, skip the tick.
    if robot.q().is_none() {
        return;
    }
    let current_q = robot.q();

    // NOTE: the into() needs context to work.
    let _desired_q: [Q; N] = current_q.unwrap().map(|_| 0_f32.into());

    // The code below won't work because `into()` doesn't know what the f32 must be converted to.
    // let desired = current_q.unwrap().map(|q| 0_f32.into());

    // Using Q(x) directly gives the type: no `into()` needed. And once a `Q`
    // is in the expression, it gives context to `into()`, as below.
    let _desired = current_q.unwrap().map(|_| Q(0.0));

    let desired = current_q
        .unwrap()
        .map(|q| q + Q(100_f32) + 100_f32.into() + 100.0.into());

    // shows that all of these syntaxes evaluate to the same result.
    assert_eq!(Q(100_f32), 100_f32.into());
    assert_eq!(Q(100_f32), 100.0.into());

    // `set_all_q` takes exactly one value per actuator, `[Q; N]`, so it can't
    // fail: no `Result` here.
    robot.set_all_q(desired);
}

fn replace_one_q(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    // We don't always want to set all positions at once. For example,
    // controlling the stem angle of a mounted camera separately would
    // be completely separate.

    // A run without hardware reads nothing: no sample to work from, skip the tick.
    if robot.q().is_none() {
        return;
    }
    let index: usize = 0;
    let current_q = robot.q_at(index);

    // To do so, we call the desired q by its index.
    // Here, we simply add 1.0 to the current position.
    // Keep in mind that you will mostly never see a real useful math
    // control in these examples unless specified so.
    let desired_q = current_q.unwrap() + 1.0.into();

    // Then simply pass your desired value and the index.
    // Because the index might be out of range, you must handle the error.
    // We will show later how to handle error.
    // Right now, keep in mind that we crash the app with `unwrap()`,
    // which is obviously a bad practice we DO NOT recommend.
    //
    // The rust standard library itself states in regards to `unwrap()`:
    // > Because this function may panic, its use is generally discouraged.
    // > Panics are meant for unrecoverable errors, and
    // > [may abort the entire program](https://doc.rust-lang.org/book/ch09-01-unrecoverable-errors-with-panic.html).
    robot.set_q_at(desired_q, index).unwrap();
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
        // A real robot would pick ONE way of homing. Here, we show both:
        //
        // - `HOME.controller()` ramps to the pose, then holds it forever.
        // - `HOME.then(replace_all_q)` ramps, then hands every tick to
        //   `replace_all_q`, whose clock starts at zero on the hand-over.
        //
        // Careful: every controller below writes `Q` on the same tick. They
        // run in the order added and the LAST write wins. So here,
        // `replace_one_q` has the last word on actuator 0, and `replace_all_q`
        // on all the others.
        .add_controller(EachTick, HOME.controller())
        .add_controller(EachTick, HOME.then(replace_all_q))
        .add_controller(EachTick, replace_all_q)
        .add_controller(EachTick, replace_all_q) // just to show you can call the same controller
        // multiple times if you wish so.
        .add_controller(EachTick, replace_one_q)
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
