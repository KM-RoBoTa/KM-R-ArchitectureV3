//! Your controllers live here. This crate depends on `kmr_api` only.

use kmr_api::{EachTick, Q, Sensors, Time, UserError, shutdown};
use std::{f32::consts::TAU, time::Duration};
// RX-28 bench: ten RX-28, ids 1 to 10, on the FTDI adapter AL02L1KL at 1 Mbit/s, declared in
// main() below; `cargo run -p controllers --features hardware,rx28`. Without `hardware`, run()
// refuses the declared actuators: remove them to simulate. Protocol 1 reads one servo per round
// trip, ~12 ms for ten: set_dt(20 ms) or more.
// kmr_api::robot!(XmBench);
// kmr_api::robot!(Rx28Chain);
kmr_api::robot!(model = "models/rx28_snake.urdf");

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

/// A small sine around where the actuators were at the first tick.
fn sine_all(start: &mut Option<[Q; N]>, time: &Time, robot: &mut State, _sensors: &Sensors) {
    if start.is_none() {
        *start = robot.q();
    }
    if let Some(start) = start {
        let offset = Q(0.15 * (TAU * 0.5 * time.elapsed().as_secs_f32()).sin());
        robot.set_all_q(start.map(|q| q + offset));
    }
}

fn shutdown_after_some_time(time: &Time, _robot: &mut State, _sensors: &Sensors) {
    if time.elapsed() > Duration::from_secs(2) {
        shutdown!("");
    } else {
        println!("Time elapsed: {:?}", time.elapsed())
    }
}

fn main() {
    let spine = kmr_api::Bus::by_id("AL02L1KL").baud(1_000_000).build();

    if let Err(e) = Robot::new()
        .set_dt(Duration::from_millis(20))
        .set_hold(Duration::from_millis(500))
        .add_actuator(kmr_api::dynamixel::Rx28::ids(1..=10).on(spine))
        .set_limit_policy_all(kmr_api::Policy::Clamp)
        // .add_controller(EachTick, nudge_first_joint)
        .add_controller_with(EachTick, None, sine_all)
        .add_controller(EachTick, shutdown_after_some_time)
        .run()
    {
        println!("{}", e);
    }
}
