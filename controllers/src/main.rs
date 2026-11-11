//! Your controllers live here. This crate depends on `kmr_api` only.

use kmr_api::{EachTick, Q, Sensors, Time, presets::home::Home, shutdown};
use std::{f32::consts::TAU, time::Duration};
// Two snakes: twelve AX-12A, ids 1 to 12, on AL02L1KL, and ten RX-28, ids 1 to 10, on AL02L1EU,
// both at 1 Mbit/s; `cargo run -p controllers --features hardware`. Protocol 1 reads one servo
// per round trip, ~27 ms for the 22: set_dt(40 ms).
// kmr_api::robot!(XmBench);
// kmr_api::robot!(Rx28Chain);
kmr_api::robot!(model = "models/ax_12_12act_an_rx28_10act.urdf");
// kmr_api::robot!(actuators = 12);

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

/// AX-12A and RX-28 middle: raw 512, which the adapter calls 0 rad.
const CENTER: Q = Q(0.0);

/// Time `HOME` takes to ramp to the centre; the sine starts after it.
const HOME_TIME: Duration = Duration::from_secs(3);

/// A small sine around the centre. Behind `HOME.then`, its time starts at zero once home.
fn sine_all(time: &Time, robot: &mut State, _sensors: &Sensors) {
    let offset = Q(0.15 * (TAU * 0.5 * time.elapsed().as_secs_f32()).sin());
    robot.set_all_q([CENTER + offset; N]);
}

// DECISION: 20 s, not 2: the run has to outlast HOME_TIME for the sine to start.
const TIME_LIMIT: Duration = Duration::from_secs(20);

fn shutdown_after_some_time(time: &Time, _robot: &mut State, _sensors: &Sensors) {
    if time.elapsed() > TIME_LIMIT {
        shutdown!("");
    } else {
        // println!("Time elapsed: {:?}", time.elapsed())
    }
}

fn main() {
    let ax_snake = kmr_api::Bus::by_id("AL02L1KL").baud(1_000_000).build();
    let rx_snake = kmr_api::Bus::by_id("AL02L1EU").baud(1_000_000).build();

    if let Err(e) = Robot::new()
        .set_dt(Duration::from_millis(40))
        .set_hold(Duration::from_millis(500))
        .set_limit_policy_all(kmr_api::Policy::Clamp)
        // Declaration order is the URDF's actuated-joint order: AX joints first, then RX.
        .add_actuator(kmr_api::dynamixel::Ax12a::ids(1..=12).on(ax_snake))
        .add_actuator(kmr_api::dynamixel::Rx28::ids(1..=10).on(rx_snake))
        // .add_controller(EachTick, nudge_first_joint)
        // .add_controller(EachTick, HOME.then(sine_all))
        .add_controller(EachTick, Home::new([CENTER; N], HOME_TIME).then(sine_all))
        .add_controller(EachTick, shutdown_after_some_time)
        .run()
    {
        println!("{}", e);
    }
}
