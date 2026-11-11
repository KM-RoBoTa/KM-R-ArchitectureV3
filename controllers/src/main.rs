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

/// Time `HOME` takes to ramp to the centre; the sines start after it.
const HOME_TIME: Duration = Duration::from_secs(3);

/// Ramps every actuator from where it was at the first tick to the centre.
const HOME: Home<N> = Home::new([CENTER; N], HOME_TIME);

/// AX-12A count: the URDF's first actuators.
const AX_COUNT: usize = 12;

/// The AX-12A snake.
const AX: [usize; AX_COUNT] = from(0);

/// The RX-28 snake: every actuator after the AX-12A.
const RX: [usize; N - AX_COUNT] = from(AX_COUNT);

/// Amplitude of the AX-12A sine; the RX-28 sine has twice it.
const AMPLITUDE: f32 = 0.15;

/// `K` consecutive actuator indices from `first`.
const fn from<const K: usize>(first: usize) -> [usize; K] {
    let mut indices = [0; K];
    let mut i = 0;
    while i < K {
        indices[i] = first + i;
        i += 1;
    }
    indices
}

/// A 0.5 Hz sine of `amplitude` around the centre. Behind `HOME.then`, its time starts at zero
/// once home.
fn sine(amplitude: f32, time: &Time) -> Q {
    CENTER + Q(amplitude * (TAU * 0.5 * time.elapsed().as_secs_f32()).sin())
}

fn sine_ax(time: &Time, robot: &mut State, _sensors: &Sensors) {
    if let Err(e) = robot
        .group(AX)
        .set_all_q([sine(2.0 * AMPLITUDE, time); AX.len()])
    {
        eprintln!("{e}");
        shutdown!("AX-12A group write failed");
    }
}

fn sine_rx(time: &Time, robot: &mut State, _sensors: &Sensors) {
    if let Err(e) = robot
        .group(RX)
        .set_all_q([sine(1.0 * AMPLITUDE, time); RX.len()])
    {
        eprintln!("{e}");
        shutdown!("RX-28 group write failed");
    }
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
        .set_dt(Duration::from_millis(5))
        .set_hold(Duration::from_millis(50))
        .set_limit_policy_all(kmr_api::Policy::Clamp)
        // Declaration order is the URDF's actuated-joint order: AX joints first, then RX.
        .add_actuator(
            kmr_api::dynamixel::Ax12a::ids(1..=12)
                .voltage_limits(9.0, 13.0)
                .restore_limits()
                .on(ax_snake),
        )
        .add_actuator(
            kmr_api::dynamixel::Rx28::ids(1..=10)
                .voltage_limits(8.0, 18.5)
                .restore_limits()
                .on(rx_snake),
        )
        // .add_controller(EachTick, nudge_first_joint)
        // DECISION: one HOME per snake controller; both ramps write the same line, so the
        // duplicate write during the ramp changes nothing.
        .add_controller(EachTick, HOME.then(sine_ax))
        .add_controller(EachTick, HOME.then(sine_rx))
        .add_controller(EachTick, shutdown_after_some_time)
        .run()
    {
        println!("{}", e);
    }
}
