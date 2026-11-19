//! The clap controller of `src/main.rs` without the microphone: both sines run continuously
//! from the end of the home ramp, AX-12A 0.30 rad and RX-28 0.15 rad, at 0.5 Hz.
//!
//! Actuators may be unplugged and plugged while it runs (`MissingPolicy::Tolerate`,
//! `JoinPolicy::Declared`): an absent one is skipped, a joiner eases into the motion over 1 s.
//! Stop with Ctrl-C.
//!
//! `cargo run -p controllers --example runtime_actuators --features hardware,experimental --release`

use kmr_api::experimental::{HealthPolicy, JoinPolicy, JoinRamp, MissingPolicy, RobotExt};
use kmr_api::{
    Bus, EachTick, Policy, Q, Sensors, Time, Volts, presets::home::Home, shutdown, with,
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

/// TEMPORARY diagnostic, not to commit: per sine period, what the AX-12A were asked and read.
struct Trace {
    ramp: JoinRamp<N>,
    window: u64,
    ticks: u32,
    lo: f32,
    hi: f32,
    sum: [f32; AX_COUNT],
    reads: [u32; AX_COUNT],
    /// Latest and lowest input voltage seen per AX-12A.
    volts: [Option<(f32, f32)>; AX_COUNT],
}

impl Trace {
    fn note(&mut self, time: &Time, robot: &State, target: &[Q; N]) {
        let window = time.elapsed().as_secs() / 2;
        if window != self.window {
            // eprint!(
            //     "[trace] t={}s ticks={} overruns={} ax_target=[{:+.3},{:+.3}] absent=",
            //     2 * window,
            //     self.ticks,
            //     time.overrun_count(),
            //     self.lo,
            //     self.hi
            // );
            // for i in (0..N).filter(|&i| !robot.present_at(i)) {
            //     eprint!("{i},");
            // }
            // eprint!(" joiners=");
            // for i in self.ramp.joiners().iter() {
            //     eprint!("{i},");
            // }
            // eprint!(" easing=");
            // for i in self.ramp.easing().iter() {
            //     eprint!("{i},");
            // }
            // eprintln!();
            // let faults = robot.faults().unwrap_or_default();
            // for i in AX {
            //     eprintln!(
            //         "[trace]   ax[{i}] mean_q={:+.3} reads={} age={:?} hw_error={:?} volts={:?} stale={} offline={} rejected={} bus_error={}",
            //         self.sum[i] / self.reads[i].max(1) as f32,
            //         self.reads[i],
            //         robot.age_at(i).ok().flatten(),
            //         robot.hardware_error_at(i).ok().flatten(),
            //         robot.voltage_at(i).ok().flatten(),
            //         faults.stale().get(i),
            //         faults.offline().get(i),
            //         faults.rejected().get(i),
            //         faults.bus_error().get(i),
            //     );
            // }
            eprint!("[volts] t={:>3}s  ", 2 * window);
            for i in AX {
                let age = robot.telemetry_age_at(i).ok().flatten();
                let error = robot.hardware_error_at(i).ok().flatten().unwrap_or(0);
                match (self.volts[i], age) {
                    (Some((now, min)), Some(age)) => {
                        eprint!("id{}: {now:.1}V min {min:.1} age {age}", i + 1);
                    }
                    _ => eprint!("id{}: no reading", i + 1),
                }
                let error = kmr_api::dynamixel::Ax12a::hardware_error(error);
                if error.any() {
                    eprint!(" HW_ERROR {error}");
                }
                eprint!(" | ");
            }
            eprintln!();
            (self.window, self.ticks) = (window, 0);
            (self.lo, self.hi) = (f32::MAX, f32::MIN);
            (self.sum, self.reads) = ([0.0; AX_COUNT], [0; AX_COUNT]);
        }
        self.ticks += 1;
        for i in AX {
            self.lo = self.lo.min(target[i].0);
            self.hi = self.hi.max(target[i].0);
            if let Ok(Some(Volts(now))) = robot.voltage_at(i) {
                let min = self.volts[i].map_or(now, |(_, min)| min.min(now));
                self.volts[i] = Some((now, min));
            }
            if let (Ok(q), Ok(Some(0))) = (robot.q_at(i), robot.age_at(i)) {
                self.sum[i] += q.0;
                self.reads[i] += 1;
            }
        }
    }
}

/// Both sines after the home ramp; a joiner eases in from where it joined over `JOIN_RAMP`,
/// during the ramp as after it. Added after `HOME`, so during the ramp a joiner's target wins
/// over Home's write, whose line started from where it was before it left.
fn sway(trace: &mut Trace, time: &Time, robot: &mut State, _: &Sensors) {
    let now = time.elapsed();
    let mut target: [Q; N] = core::array::from_fn(|i| base(i, now));
    trace.ramp.apply(time, robot, &mut target);
    trace.note(time, robot, &target);
    let ramp = &trace.ramp;

    if now < HOME_TIME {
        for i in ramp.joiners().iter() {
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
        .set_health_policy(HealthPolicy::Warn)
        // Declaration order is the URDF's actuated-joint order: AX joints first, then RX.
        .add_actuator(
            kmr_api::dynamixel::Ax12a::ids(1..=12)
                .voltage_limits(8.0, 13.0)
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
        .add_controller(
            EachTick,
            with(
                Trace {
                    ramp: JoinRamp::new(JOIN_RAMP),
                    window: 0,
                    ticks: 0,
                    lo: f32::MAX,
                    hi: f32::MIN,
                    sum: [0.0; AX_COUNT],
                    reads: [0; AX_COUNT],
                    volts: [None; AX_COUNT],
                },
                sway,
            ),
        );

    if let Err(e) = robot.run() {
        eprintln!("{e}");
    }
}
