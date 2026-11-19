//! The GeckoLeg walking gait on three AK60-6: hip id 1, knee id 2, ankle id 3, on `can0`.
//!
//! The maths of cubemars_rs's `examples/geckoleg.rs`, itself ported from
//! `cubemars/GeckoLeg-main` (C++):
//! - `foot`, `control_points`, `bezier` <- `WalkingTrajectory`: stance line P0->P3, swing Bezier
//!   P3->P0, starting at t = 0.5 of the stance as the C++ does.
//! - `forward_kinematics`, `jacobian`, `inverse_kinematics` <- `Controller` (DH table,
//!   hip-to-global matrix, `motor = theta - offset`), the Jacobian keeping the C++ quirk
//!   (`z_prev = H * (0,0,1,1)` also adds the origin).
//! - `IK_ITERATIONS` Newton steps per tick from the previous goal, where the C++ does one from the
//!   measured angles.
//!
//! In kmr: one `EachTick` controller writes the three goals every 5 ms (C++ `Tc`); the gains
//! (Kp 150, Kd 1), the joint ranges of one cycle and the speed cap go to cubemars_rs as each
//! actuator's limits. The model's t = 0 posture is all joints 0, which cubemars_rs takes where
//! each actuator stands at bring-up: place the leg by hand in the C++ zero posture BEFORE starting.
//! The run stops after `PROGRAM_S` model seconds, or on Ctrl-C; then the leg holds briefly and
//! the torque goes off.
//!
//! `cargo run -p controllers --example geckoleg --features cubemars --release`
//! `-- --dry-run` simulates (no CAN); `-- --speed X` overrides `SPEED`.
//! `scripts/build-pi.sh` cross-compiles it for the Pi and copies it there.

use kmr_api::cubemars::Ak60;
use kmr_api::{Bus, EachTick, Policy, Q, Qd, Sensors, Time, shutdown, with};
use std::f64::consts::FRAC_PI_2;
use std::time::Duration;

kmr_api::robot!(model = "models/geckoleg.urdf");

/// Model time = wall time x SPEED (1.0 = the C++ project's real speed). Start very low.
const SPEED: f64 = 1.0;
/// Control period, s (C++ `Tc`). Unchanged by `SPEED`.
const TC: f64 = 0.005;
const KP: f32 = 150.0;
const KD: f32 = 1.0;
/// Run length in model seconds (C++ `programDuration`).
const PROGRAM_S: f64 = 15.0;
/// Margin added to the joint range of one cycle, rad.
const LIMIT_MARGIN: f32 = 0.15;
/// Largest distance between a goal and the measured position, deg.
const MAX_GOAL_STEP_DEG: f32 = 20.0;
/// Newton steps per tick (the C++ does one per tick, from feedback).
const IK_ITERATIONS: usize = 5;
/// How long the leg holds its last goal before the torque goes off.
const HOLD: Duration = Duration::from_millis(200);

// Gait (C++ `global.config` + `BaseTrajectory`).
const TG: f64 = 2.0;
const DUTY: f64 = 0.7;
const LX: f64 = 0.20;
const R1: f64 = 0.08;
const R2: f64 = 0.08;
const PHI1: f64 = 0.0;
const PHI2: f64 = 0.0;
/// x0, y0, z0: the foot at all-zero motors.
const FOOT0: [f64; 3] = [-0.107, -0.089, -0.217];
// DH table (C++ `Controller` constructor), per joint hip, knee, ankle.
const OFFSET: [f64; 3] = [-FRAC_PI_2, 0.52, -1.447];
const ALPHA: [f64; 3] = [-FRAC_PI_2, 0.0, 0.0];
const A: [f64; 3] = [0.0, 0.147, 0.150];
const D: [f64; 3] = [0.060, 0.090, 0.0];

/// The leg's joints, in the model's actuator order.
const JOINTS: [usize; N] = [actuator!("hip"), actuator!("knee"), actuator!("ankle")];

type V3 = [f64; 3];
type M4 = [f64; 16];

/// Bezier control points P0..P3 (`WalkingTrajectory` constructor), global frame.
fn control_points() -> [V3; 4] {
    let [x0, y0, z0] = FOOT0;
    let p0 = [x0 + LX / 2.0, y0, z0];
    let p3 = [x0 - LX / 2.0, y0, z0];
    let p1 = [
        p0[0] + (p3[0] - p0[0]) / 3.0,
        y0 + R1 * PHI1.sin(),
        z0 + R1 * PHI1.cos(),
    ];
    let p2 = [
        p0[0] + (p3[0] - p0[0]) * 2.0 / 3.0,
        y0 + R2 * PHI2.sin(),
        z0 + R2 * PHI2.cos(),
    ];
    [p0, p1, p2, p3]
}

fn bezier(p: &[V3; 4], t: f64) -> V3 {
    let [p0, p1, p2, p3] = p;
    let u = 1.0 - t;
    std::array::from_fn(|k| {
        u.powi(3) * p3[k] + 3.0 * u * u * t * p2[k] + 3.0 * u * t * t * p1[k] + t.powi(3) * p0[k]
    })
}

/// Foot position in the global frame at model time `tau` (forward gait).
fn foot(tau: f64) -> V3 {
    let p = control_points();
    let [p0, _, _, p3] = p;
    let (stance, swing) = (DUTY * TG, TG - DUTY * TG);
    // The C++ starts in stance at t = 0.5.
    let m = (tau + 0.5 * stance).rem_euclid(TG);
    if m < stance {
        let t = m / stance;
        std::array::from_fn(|k| p0[k] + (p3[k] - p0[k]) * t)
    } else {
        bezier(&p, (m - stance) / swing)
    }
}

fn mul4(a: &M4, b: &M4) -> M4 {
    std::array::from_fn(|n| (0..4).map(|k| a[n / 4 * 4 + k] * b[k * 4 + n % 4]).sum())
}

/// DH transforms of the hip, knee and ankle frames in the hip reference, from DH angles `theta`.
fn forward_kinematics(theta: &V3) -> [M4; 3] {
    let local: [M4; 3] = std::array::from_fn(|i| {
        let (ct, st) = (theta[i].cos(), theta[i].sin());
        let (ca, sa) = (ALPHA[i].cos(), ALPHA[i].sin());
        #[rustfmt::skip]
        let m = [
            ct, -st * ca, st * sa, A[i] * ct,
            st, ct * ca, -ct * sa, A[i] * st,
            0.0, sa, ca, D[i],
            0.0, 0.0, 0.0, 1.0,
        ];
        m
    });
    let h0 = local[0];
    let h1 = mul4(&h0, &local[1]);
    let h2 = mul4(&h1, &local[2]);
    [h0, h1, h2]
}

fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Origin of a homogeneous transform.
fn origin(m: &M4) -> V3 {
    [m[3], m[7], m[11]]
}

/// Jacobian columns of `getJacobian`, row-major 3x3.
fn jacobian(h: &[M4; 3]) -> [f64; 9] {
    let eff = origin(&h[2]);
    let cols: [V3; 3] = std::array::from_fn(|i| {
        let (z, o) = match i.checked_sub(1).map(|p| &h[p]) {
            None => ([0.0, 0.0, 1.0], [0.0; 3]),
            // C++ quirk kept: H * (0,0,1,1) = z axis + origin.
            Some(m) => ([m[2] + m[3], m[6] + m[7], m[10] + m[11]], origin(m)),
        };
        cross(z, [eff[0] - o[0], eff[1] - o[1], eff[2] - o[2]])
    });
    std::array::from_fn(|n| cols[n % 3][n / 3])
}

/// `J^-1 * v` (adjugate form), `None` when singular.
fn solve(j: &[f64; 9], v: V3) -> Option<V3> {
    let [a, b, c, d, e, f, g, h, i] = *j;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if det.abs() < 1e-12 {
        return None;
    }
    #[rustfmt::skip]
    let inv = [
        e * i - f * h, c * h - b * i, b * f - c * e,
        f * g - d * i, a * i - c * g, c * d - a * f,
        d * h - e * g, b * g - a * h, a * e - b * d,
    ];
    Some(std::array::from_fn(|r| {
        (0..3).map(|k| inv[r * 3 + k] * v[k]).sum::<f64>() / det
    }))
}

/// `getInverseKinematics`: Newton steps from the motor angles `from` to the global foot `goal`.
/// Returns motor angles (`theta - offset`). Hip-to-global matrix is (0,0,-1; -1,0,0; 0,1,0).
fn inverse_kinematics(from: &V3, goal: V3) -> V3 {
    // Its transpose is its inverse.
    let goal_hip = [-goal[1], goal[2], -goal[0]];
    let mut theta: V3 = std::array::from_fn(|i| from[i] + OFFSET[i]);
    for _ in 0..IK_ITERATIONS {
        let h = forward_kinematics(&theta);
        let now = origin(&h[2]);
        let delta = std::array::from_fn(|k| goal_hip[k] - now[k]);
        let Some(dtheta) = solve(&jacobian(&h), delta) else {
            break;
        };
        theta = std::array::from_fn(|i| theta[i] + dtheta[i]);
    }
    std::array::from_fn(|i| theta[i] - OFFSET[i])
}

/// Motor angles (rad) at model time `tau`, given the previous goals.
fn goals(prev: &V3, tau: f64) -> V3 {
    inverse_kinematics(prev, foot(tau))
}

/// Joint range and peak joint speed (rad/s, SPEED 1) over two cycles, simulated from the zero
/// posture.
fn cycle_stats() -> (V3, V3, f64) {
    let (mut lo, mut hi, mut peak) = ([f64::MAX; 3], [f64::MIN; 3], 0.0f64);
    let mut prev = [0.0; 3];
    let steps = (2.0 * TG / TC) as usize;
    for k in 0..=steps {
        let q = goals(&prev, k as f64 * TC);
        lo = std::array::from_fn(|i| lo[i].min(q[i]));
        hi = std::array::from_fn(|i| hi[i].max(q[i]));
        peak = (0..3).fold(peak, |p, i| p.max((q[i] - prev[i]).abs() / TC));
        prev = q;
    }
    (lo, hi, peak)
}

/// The gait's state between ticks.
struct Gait {
    /// Model time per wall second.
    speed: f64,
    /// Last goals, rad: where the next Newton steps start.
    prev: V3,
    /// Wall time of the next printout, s.
    next_print: f64,
}

/// Writes the three goals of model time `elapsed x speed`; prints goals and measures once a
/// second; stops after `PROGRAM_S`.
fn walk(gait: &mut Gait, time: &Time, robot: &mut State, _: &Sensors) {
    let wall = time.elapsed().as_secs_f64();
    let tau = wall * gait.speed;
    if tau > PROGRAM_S {
        shutdown!("geckoleg: program over");
        return;
    }
    let q = goals(&gait.prev, tau);
    gait.prev = q;
    if let Err(e) = robot.group(JOINTS).set_all_q(q.map(|q| Q(q as f32))) {
        eprintln!("{e}");
        shutdown!("geckoleg: goal write failed");
        return;
    }
    if wall >= gait.next_print {
        gait.next_print += 1.0;
        let measured = JOINTS.map(|i| robot.q_at(i).map_or(f32::NAN, |q| q.0.to_degrees()));
        println!(
            "t={tau:7.3}s goal[deg]={:7.2?} meas[deg]={measured:7.2?}",
            q.map(f64::to_degrees)
        );
    }
}

fn main() {
    let dry = std::env::args().any(|a| a == "--dry-run");
    let speed = std::env::args()
        .skip_while(|a| a != "--speed")
        .nth(1)
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(SPEED);

    let (lo, hi, peak) = cycle_stats();
    println!(
        "range[deg] lo={:.1?} hi={:.1?} peak={peak:.2} rad/s at SPEED 1, SPEED {speed}",
        lo.map(f64::to_degrees),
        hi.map(f64::to_degrees),
    );
    println!("Ctrl-C stops.");

    let robot = Robot::new()
        .set_dt(Duration::from_secs_f64(TC))
        .set_hold(HOLD)
        .set_limit_policy_all(Policy::Clamp)
        .add_controller(
            EachTick,
            with(
                Gait {
                    speed,
                    prev: [0.0; 3],
                    next_print: 0.0,
                },
                walk,
            ),
        );
    // Without actuators the run simulates.
    let robot = if dry {
        robot
    } else {
        let leg = Bus::by_dev("can0").build();
        let vmax = (peak * speed * 1.5 + 2.0) as f32;
        robot.add_actuator(
            Ak60::ids(1..=3)
                .gains(KP, KD)
                .position_limits(lo, hi)
                .position_margin(Q(LIMIT_MARGIN))
                .velocity_limit(Qd(vmax))
                .max_goal_step(Q(MAX_GOAL_STEP_DEG.to_radians()))
                .on(leg),
        )
    };
    if let Err(e) = robot.run() {
        eprintln!("{e}");
        let mut source = core::error::Error::source(&e);
        while let Some(cause) = source {
            eprintln!("  caused by: {cause}");
            source = cause.source();
        }
    }
}
