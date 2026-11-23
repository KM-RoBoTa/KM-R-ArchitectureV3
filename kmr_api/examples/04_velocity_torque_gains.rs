// Beyond positions: velocity, torque and gains.
//
// Good news: once you know `q`, you know them all. Every field has the same
// set of methods:
//
// - `qd()` / `qd_at(i)`: the newest reading, for all actuators or just one.
// - `prev_qd(depth)` / `prev_qd_at(i, depth)`: the reading `depth` ticks ago.
// - `set_all_qd(..)` / `set_qd_at(.., i)`: stage a command.
//
// Swap `qd` for `tau` or `kp` and it works the same. Units: `Qd` is rad/s
// (mm/s on a sliding joint), `Tau` is N·m (N on a sliding joint).
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, EachTick, Kd, Kp, Qd, Sensors, Tau, Time};
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

// Which field the hardware actually follows depends on the actuator and its
// mode, set in the hardware table. The XmBench follows `q`: a velocity or
// torque command alone is refused (you'd see it in the `rejected` faults, see
// 08). The controllers below only tour the methods.
fn use_velocity(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    // `let .. else` reads the value and skips the tick in one go when there
    // is no sample yet. Same idea as the `is_none()` check of 02, shorter.
    let Some(current_qd) = robot.velocity() else {
        return;
    };
    let _same: Option<[Qd; N]> = robot.qd();
    let index: usize = 0;
    // The `_at` readers return a `Result`: the index may be out of range.
    let Ok(qd) = robot.qd_at(index) else {
        return;
    };
    // History: 2 ticks ago. An `Err` until enough ticks have run.
    let _previous = robot.prev_qd(2);
    let _previous_at = robot.prev_qd_at(index, 2);

    // We just send back what we read, to show the calls.
    robot.set_all_qd(current_qd);
    robot.set_qd_at(qd, index).unwrap();
}

// Torque has three names, pick the one that reads best to you:
// `tau()`, `torque()` and `effort()` are the same thing.
fn use_torque(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    let Some(current_tau) = robot.tau() else {
        return;
    };
    let _torque: Option<[Tau; N]> = robot.torque();
    let _effort: Option<[Tau; N]> = robot.effort();
    let index: usize = 0;
    let Ok(tau) = robot.tau_at(index) else {
        return;
    };
    let _previous = robot.prev_tau(2);
    let _previous_at = robot.prev_tau_at(index, 2);

    robot.set_all_tau(current_tau);
    robot.set_tau_at(tau, index).unwrap();
}

// Gains tune how stiff (`Kp`) and how damped (`Kd`) an actuator holds its
// target. They matter on actuators that take gains, like a CubeMars in MIT
// mode; the others (the XmBench's Dynamixels included) ignore them. Reading
// them gives the gains last applied, 0 where an actuator has none.
fn use_gains(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    let index: usize = 0;

    if let Some(kp) = robot.kp() {
        robot.set_all_kp(kp.map(|_| Kp(10.0)));
    }
    if let Ok(kp) = robot.kp_at(index) {
        robot.set_kp_at(kp, index).unwrap();
    }
    let _previous_kp = robot.prev_kp(1);
    let _previous_kp_at = robot.prev_kp_at(index, 1);

    if let Some(kd) = robot.kd() {
        robot.set_all_kd(kd.map(|_| Kd(0.5)));
    }
    if let Ok(kd) = robot.kd_at(index) {
        robot.set_kd_at(kd, index).unwrap();
    }
    let _previous_kd = robot.prev_kd(1);
    let _previous_kd_at = robot.prev_kd_at(index, 1);
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
        // Several controllers on one schedule, in one go: a tuple. They run
        // left to right, exactly as if you had added them one by one.
        .add_controller(EachTick, (use_velocity, use_torque, use_gains))
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
