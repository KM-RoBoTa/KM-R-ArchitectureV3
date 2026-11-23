// Groups: work on a few joints at once.
//
// A real robot has parts: an arm, a leg, a camera head. You want to command
// the right arm without spelling out its indices every time, and without
// touching the left one by accident.
//
// A group is just that: a list of indices you name once, then use like a
// small robot of its own.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, Celsius, EachTick, Q, Sensors, Time};
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

// A group is a fixed set of joint indices — non-contiguous allowed, e.g.
// [1, 3, 4]. Here, the right arm: joints 1, 2, 3 of the 4-actuator bench.
// DECISION: was [3, 4, 5], past the XmBench's 4 actuators, so it never read.
const R_ARM: [usize; 3] = [1, 2, 3];
fn use_groups(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    // One other nice feature are groups. You can specify a dedicated group
    // in the form of an array of indexes, and work with them using almost
    // the exact same set of methods (`q()`, `prev_q()`, `set_all_q()`, ...).
    // A run without hardware reads nothing: no sample to work from, skip the tick.
    if robot.q().is_none() {
        return;
    }
    let mut r_arm = robot.group(R_ARM);

    // Accessing `q()` on a group returns only the current values for the chosen
    // indexes in that group. E.g: R_ARM only has 3 joint indexes, so you
    // only read the current values of those 3.
    //
    // Note: a group's `q()` is also `None` if one of its indices is out of
    // range. A typo in `R_ARM` reads as "never any sample".
    let current_q = r_arm.q();

    let desired = current_q.unwrap().map(|q| q + Q(100_f32));
    // group index can be out of range, so this returns a `Result` to propagate
    // that cause — it is not swallowed. Joints outside R_ARM are untouched.
    // And it is all or nothing: one bad index, and nothing is written.
    r_arm.set_all_q(desired).unwrap();
}

// The rest of what a group can do, at a glance. Same names as on the whole
// robot; every array is sized to the group (3 here), not to the robot.
fn use_groups_fields(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    let mut r_arm = robot.group(R_ARM);
    // Did this tick's read fail on any joint of the arm? Then its values are
    // last tick's: skip rather than act on old data (more on that in 08).
    if r_arm.stale() {
        return;
    }
    // How many ticks old the arm's oldest reading is.
    let _age: Option<u32> = r_arm.sample_age();
    // Telemetry, read-only: `None` until the actuator reports one.
    let _temperatures: [Option<Celsius>; 3] = r_arm.temperatures();

    let Some(position) = r_arm.position() else {
        return;
    };
    let _previous_q = r_arm.prev_q(2);

    let _qd = r_arm.qd();
    let _previous_qd = r_arm.prev_qd(2);
    if let Some(qd) = r_arm.velocity() {
        r_arm.set_all_qd(qd).unwrap();
    }

    let _tau = r_arm.tau();
    let _torque = r_arm.torque();
    let _previous_tau = r_arm.prev_tau(2);
    if let Some(tau) = r_arm.effort() {
        r_arm.set_all_tau(tau).unwrap();
    }

    let _previous_kp = r_arm.prev_kp(1);
    if let Some(kp) = r_arm.kp() {
        r_arm.set_all_kp(kp).unwrap();
    }
    let _previous_kd = r_arm.prev_kd(1);
    if let Some(kd) = r_arm.kd() {
        r_arm.set_all_kd(kd).unwrap();
    }

    // Hold the arm where it is.
    r_arm.set_all_q(position).unwrap();
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
        .add_controller(EachTick, use_groups)
        .add_controller(EachTick, use_groups_fields)
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
