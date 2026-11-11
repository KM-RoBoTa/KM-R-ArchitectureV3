//! Your controllers live here. This crate depends on `kmr_api` only.

use kmr_api::{EachTick, Q, Sensors, Time, dynamixel, shutdown};
use std::{f32::consts::TAU, time::Duration};
// RX-28 bench: ten RX-28, ids 1 to 10, on the FTDI adapter AL02L1KL at 1 Mbit/s, declared in
// main() below; `cargo run -p controllers --features hardware,rx28`. Without `hardware`, run()
// refuses the declared actuators: remove them to simulate. Protocol 1 reads one servo per round
// trip, ~12 ms for ten: set_dt(20 ms) or more.
// kmr_api::robot!(XmBench);
// kmr_api::robot!(Rx28Chain);
kmr_api::robot!(model = "models/ax12_snake_12actuators.urdf");

// fn nudge_first_joint(_time: &Time, robot: &mut State, _sensors: &Sensors) {
//     let actuator_id = 0;
//
//     match robot.q_at(actuator_id) {
//         Ok(q) => {
//             // let _ = robot.set_q_at(q + Q(0.05), 0);
//             robot.set_q_at(q.0.sin().into(), actuator_id);
//         }
//         Err(e) => {
//             eprintln!("{}", e);
//             shutdown!("q_at failed");
//         }
//     }
// }

/// UNUSED: A small sine around where the actuators were at the first tick.
fn _sine_all_with_start(
    start: &mut Option<[Q; N]>,
    time: &Time,
    robot: &mut State,
    _sensors: &Sensors,
) {
    if start.is_none() {
        *start = robot.q();
    }
    if let Some(start) = start {
        let offset = Q(0.15 * (TAU * 0.5 * time.elapsed().as_secs_f32()).sin());
        robot.set_all_q(start.map(|q| q + offset));
    }
}

/// AX-12 middle: raw 512, which dxl_rs_v2 and the adapter both call 0 rad (v1 called it 2.618).
const CENTER: Q = Q(0.0);
const AMPLITUDE_RAD: f32 = 0.2;
const FREQ_HZ: f32 = 0.3;

/// Time `home` takes to ramp to the centre; the sine starts after it.
const HOME_TIME: Duration = Duration::from_secs(3);

/// Sine left and right around the middle of the actuator, once `home` is done.
fn sine_all(time: &Time, robot: &mut State, _sensors: &Sensors) {
    let Some(t) = time.elapsed().checked_sub(HOME_TIME) else {
        return;
    };
    let offset = Q(AMPLITUDE_RAD * (TAU * FREQ_HZ * t.as_secs_f32()).sin());
    robot.set_all_q([CENTER + offset; N]);
}

/// Ramps every actuator from where it was at the first tick to the centre, over `HOME_TIME`.
// Init writes are discarded (SRS FR-5.2) and `go_home` is open (FR-2.7): homing runs on ticks.
fn home(start: &mut Option<[Q; N]>, time: &Time, robot: &mut State, _sensors: &Sensors) {
    if start.is_none() {
        *start = robot.q();
    }
    let Some(start) = start else { return };
    let a = time.elapsed().as_secs_f32() / HOME_TIME.as_secs_f32();
    if a < 1.0 {
        robot.set_all_q(start.map(|q| Q(q.0 + (CENTER.0 - q.0) * a)));
    }
}

// fn _home_fixed(time: &Time, robot: &mut State, _sensors: &Sensors) {
//     let ramp = 0;
//     robot.go_home(ramp);
// }

fn shutdown_after_some_time(time: &Time, _robot: &mut State, _sensors: &Sensors) {
    if time.elapsed() > TIME_LIMIT {
        shutdown!("");
    } else {
        // println!("Time elapsed: {:?}", time.elapsed())
    }
}
const TIME_LIMIT: Duration = Duration::from_secs(20);

fn main() {
    let spine = kmr_api::Bus::by_id("AL02L1KL").baud(1_000_000).build();
    // let spine = kmr_api::Bus::by_id("AL02L1EU").baud(1_000_000).build();

    if let Err(e) = Robot::new()
        //
        // settings
        //
        .set_dt(Duration::from_millis(20))
        .set_hold(Duration::from_millis(500)) // On normal shutdown, hold position for 500ms
        .set_limit_policy_all(kmr_api::Policy::Clamp)
        //
        // actuators
        //
        // BUG:
        // .add_actuator(kmr_api::dynamixel::Ax12a::ids(6..=10).read_only().on(spine))
        // .add_actuator(kmr_api::dynamixel::Ax12a::ids(1..=10).read_only().on(spine))
        //
        // NOT_BUG:
        //
        // AX12a
        .add_actuator(
            dynamixel::Ax12a::ids(1..=12)
                .voltage_limits(9.0, 13.0) // ax12 (undervoltage)
                .restore_limits() // bug noted in the method
                .on(spine),
        )
        // RX-28
        // .add_actuator(
        //     dynamixel::Rx28::ids(1..=10)
        //         .voltage_limits(8.0, 18.5) // rx28 (undervoltage)
        //         .restore_limits()
        //         .on(spine),
        // )
        //
        // controllers
        //
        // .add_controller(EachTick, nudge_first_joint)
        // .add_controller_with(EachTick, None, sine_all)
        .add_controller_with(EachTick, None, home)
        .add_controller(EachTick, sine_all)
        .add_controller(EachTick, shutdown_after_some_time)
        .run()
    {
        println!("{}", e);
    }
}
