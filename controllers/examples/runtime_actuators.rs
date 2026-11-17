//! The clap controller of `src/main.rs` without the microphone: both sines run continuously
//! from the end of the home ramp, AX-12A 0.30 rad and RX-28 0.15 rad, at 0.5 Hz.
//!
//! Actuators may be unplugged and plugged while it runs (`MissingPolicy::Tolerate`,
//! `JoinPolicy::Declared`): an absent one is skipped, a joiner eases into the motion over 1 s.
//! Stop with Ctrl-C.
//!
//! `cargo run -p controllers --example runtime_actuators --features hardware --release`

use kmr_api::{
    Bus, EachTick, JoinPolicy, Mask, MissingPolicy, Policy, Q, Sensors, Time, presets::home::Home,
    shutdown, with,
};
use std::{f32::consts::TAU, time::Duration};

kmr_api::robot!(model = "models/ax_12_12act_an_rx28_10act.urdf");

/// AX-12A and RX-28 middle: raw 512, which the adapter calls 0 rad.
const CENTER: Q = Q(0.0);

/// Time `HOME` takes to ramp to the centre; both sines start after it.
const HOME_TIME: Duration = Duration::from_secs(3);

/// Ramps every actuator from where it was at the first tick to the centre.
const HOME: Home<N> = Home::new([CENTER; N], HOME_TIME);

/// AX-12A count: the URDF's first actuators.
const AX_COUNT: usize = 12;

/// The AX-12A snake.
const AX: [usize; AX_COUNT] = from(0);

/// The RX-28 snake: every actuator after the AX-12A.
const RX: [usize; N - AX_COUNT] = from(AX_COUNT);

/// Amplitude of the RX-28 sine; the AX-12A sine has twice it.
const AMPLITUDE: f32 = 0.15;

/// How long a joiner takes to ease from where it is into the motion.
const JOIN_RAMP: Duration = Duration::from_secs(1);

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

/// A 0.5 Hz sine of `amplitude` around the centre, `t` after it started.
#[inline]
fn sine(amplitude: f32, t: Duration) -> Q {
    CENTER + Q(amplitude * (TAU * 0.5 * t.as_secs_f32()).sin())
}

/// Where actuator `i` goes at `now`: the centre during the home ramp, its snake's sine after.
#[inline]
fn base(i: usize, now: Duration) -> Q {
    let Some(t) = now.checked_sub(HOME_TIME) else {
        return CENTER;
    };
    if i < AX_COUNT {
        sine(2.0 * AMPLITUDE, t)
    } else {
        sine(AMPLITUDE, t)
    }
}

/// The point a fraction `s` of the way from `from` to `to`.
#[inline]
fn lerp(from: Q, to: Q, s: f32) -> Q {
    Q(from.0 + (to.0 - from.0) * s)
}

/// The actuators seen present last tick, and when and where each joiner started its ease-in.
struct Joins {
    seen: Mask<N>,
    since: [Option<(Duration, Q)>; N],
    started: bool,
}

impl Joins {
    /// Nothing seen yet, no joiner.
    const fn new() -> Self {
        Self {
            seen: Mask::NONE,
            since: [None; N],
            started: false,
        }
    }
}

/// Both sines after the home ramp; an actuator that becomes present after the first tick eases
/// in from its first sample over `JOIN_RAMP`, during the ramp as after it. Added after `HOME`,
/// so a joiner's ease-in during the ramp wins over Home's write.
fn sway(joins: &mut Joins, time: &Time, robot: &mut State, _: &Sensors) {
    let now = time.elapsed();
    let present = robot.present();
    // Actuators present at the first tick never ease in: Home covers them.
    if joins.started {
        for i in present.difference(joins.seen).iter() {
            joins.since[i] = robot.q_at(i).ok().map(|q| (now, q));
        }
    }
    joins.seen = present;
    joins.started = true;

    let mut target = [CENTER; N];
    let mut easing = Mask::<N>::NONE;
    for (i, q) in target.iter_mut().enumerate() {
        *q = base(i, now);
        if let Some((start, from)) = joins.since[i] {
            let age = now - start;
            if age < JOIN_RAMP {
                *q = lerp(from, *q, age.as_secs_f32() / JOIN_RAMP.as_secs_f32());
                easing.set(i);
            } else {
                joins.since[i] = None;
            }
        }
    }

    if now < HOME_TIME {
        for i in easing.iter() {
            if let Err(e) = robot.set_q_at(target[i], i) {
                eprintln!("{e}");
                shutdown!("joiner write failed");
            }
        }
        return;
    }
    if let Err(e) = robot.group(AX).set_all_q(AX.map(|i| target[i])) {
        eprintln!("{e}");
        shutdown!("AX-12A group write failed");
    }
    if let Err(e) = robot.group(RX).set_all_q(RX.map(|i| target[i])) {
        eprintln!("{e}");
        shutdown!("RX-28 group write failed");
    }
}

/// Both snakes with both policies on, until Ctrl-C.
fn main() {
    let ax_snake = Bus::by_id("AL02L1KL").baud(1_000_000).build();
    let rx_snake = Bus::by_id("AL02L1EU").baud(1_000_000).build();

    let robot = Robot::new()
        .set_dt(Duration::from_millis(5))
        .set_hold(Duration::from_millis(50))
        .set_limit_policy_all(Policy::Clamp)
        .set_missing_policy(MissingPolicy::Tolerate)
        .set_join_policy(JoinPolicy::Declared)
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
        .add_controller(EachTick, HOME.controller())
        .add_controller(EachTick, with(Joins::new(), sway));

    if let Err(e) = robot.run() {
        eprintln!("{e}");
    }
}
