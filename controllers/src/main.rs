//! Your controllers live here. This crate depends on `kmr_api` only.

use kmr_api::{EachTick, Q, Sensors, Time, presets::home::Home, shutdown, with};
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

/// The run shuts down after this, whatever happened.
const TIME_LIMIT: Duration = Duration::from_secs(60);

/// AX-12A and RX-28 middle: raw 512, which the adapter calls 0 rad.
const CENTER: Q = Q(0.0);

/// Time `HOME` takes to ramp to the centre; claps are heard after it.
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

/// A microphone sample louder than this is a clap.
const PEAK: u16 = 1000;

/// How long both snakes move after a clap: one period of the 0.5 Hz sine.
const MOVE_FOR: Duration = Duration::from_secs(2);

/// A 0.5 Hz sine of `amplitude` around the centre, `t` after it started.
#[inline]
fn sine(amplitude: f32, t: Duration) -> Q {
    CENTER + Q(amplitude * (TAU * 0.5 * t.as_secs_f32()).sin())
}

/// How long the last clap's motion has run; `None` before the first clap and once it is over.
#[inline]
fn moving_for(clap: &Clap, now: Duration) -> Option<Duration> {
    let t = now - clap.since?;
    (t < MOVE_FOR).then_some(t)
}

/// Where the AX-12A snake goes: its sine while moving, the centre otherwise.
#[inline]
fn ax_target(moving_for: Option<Duration>) -> Q {
    moving_for.map_or(CENTER, |t| sine(2.0 * AMPLITUDE, t))
}

/// Where the RX-28 snake goes: its sine while moving, the centre otherwise.
#[inline]
fn rx_target(moving_for: Option<Duration>) -> Q {
    moving_for.map_or(CENTER, |t| sine(AMPLITUDE, t))
}

/// The microphone's loudest sample this tick; 0 without a microphone or a sample.
#[inline]
fn peak(sensors: &Sensors) -> u16 {
    let Some(mic) = sensors.mic(0) else {
        return 0;
    };
    mic.samples()
        .iter()
        .map(|s| s.unsigned_abs())
        .max()
        .unwrap_or(0)
}

/// When the last clap's motion started; `None` before the first clap.
#[derive(Default)]
struct Clap {
    since: Option<Duration>,
}

/// Once home, holds both snakes at the centre; a clap runs both sines for `MOVE_FOR`, their
/// time starting at the clap.
// DECISION: gated on `HOME_TIME`, not behind `HOME.then`, which takes no context controller.
fn move_on_clap(clap: &mut Clap, time: &Time, robot: &mut State, sensors: &Sensors) {
    let now = time.elapsed();
    if now < HOME_TIME {
        return;
    }
    // DECISION: a peak heard while moving is ignored (the servos are loud), so one clap is one
    // 2 s motion.
    let loudest = peak(sensors);
    if moving_for(clap, now).is_none() && loudest > PEAK {
        println!("clap at {now:?}: peak {loudest} > {PEAK}, both snakes move for {MOVE_FOR:?}");
        clap.since = Some(now);
    }

    let t = moving_for(clap, now);
    let ax = ax_target(t);
    let rx = rx_target(t);
    if let Err(e) = robot.group(AX).set_all_q([ax; AX.len()]) {
        eprintln!("{e}");
        shutdown!("AX-12A group write failed");
    }
    if let Err(e) = robot.group(RX).set_all_q([rx; RX.len()]) {
        eprintln!("{e}");
        shutdown!("RX-28 group write failed");
    }
}

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

    let robot = Robot::new()
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
        // DECISION: HOME before `move_on_clap`: controllers run in the order added, so once home
        // the actuators get what `move_on_clap` writes, not HOME's pose.
        .add_controller(EachTick, HOME.controller())
        .add_controller(EachTick, with(Clap::default(), move_on_clap))
        .add_controller(EachTick, shutdown_after_some_time);
    // The microphone, on the XIAO's USB plug; without `hardware` no clap is heard.
    #[cfg(feature = "hardware")]
    let robot = robot.add_sensor(kmr_api::xiao::Xiao::usb().mic());

    if let Err(e) = robot.run() {
        println!("{}", e);
    }
}
