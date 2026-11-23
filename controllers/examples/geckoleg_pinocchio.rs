//! The GeckoLeg walking gait of `examples/geckoleg.rs`, its controller written in C++
//! with Pinocchio doing the kinematics: three AK60-6, hip id 1, knee id 2, ankle id 3, on `can0`.
//!
//! Rust keeps what kmr declares: the command line, the builder (5 ms tick, hold, clamp), the bus
//! and the `Ak60` gains and limits. C++ (`cpp/geckoleg.cpp`) owns the gait, the inverse kinematics
//! and the printout: `check_gait_cpp` on `Init` builds the Pinocchio model and simulates two
//! cycles against [`leg::LIMITS`], the limits the actuators get, with an emergency stop before
//! any motion when the gait leaves them; `walk_cpp` on `EachTick` writes the three goals.
//!
//! One URDF, `models/geckoleg.urdf`, describes the leg: `robot!` reads it for kmr's
//! actuators, and build.rs embeds it in the binary for Pinocchio's `buildModelFromXML`. The
//! binary reads no model file at run time; it loads Pinocchio's shared libraries through its
//! rpath: Pinocchio's lib dir, or with `PINOCCHIO_RPATH` set at build time, that path instead.
//!
//! The model's t = 0 posture is all joints 0, which cubemars_rs takes where each actuator stands
//! at bring-up: place the leg by hand in the C++ zero posture BEFORE starting. The run stops
//! after `PROGRAM_S` model seconds, or on Ctrl-C; then the leg holds briefly and the torque goes
//! off.
//!
//! Needs Pinocchio through pkg-config, its include dirs in `CXXFLAGS` (kmr's build helper takes
//! none):
//! `CXXFLAGS="$(pkg-config --cflags pinocchio)" cargo run -p controllers --example
//! geckoleg_pinocchio --features pinocchio --release -- --dry-run`
//! `--dry-run` simulates (no CAN); `--speed X` overrides `SPEED`; add the feature `cubemars` for
//! the leg.
//!
//! For the robot's Pi 4: `PI_HOST=<addr> scripts/build-pi.sh geckoleg_pinocchio --deploy` builds
//! for aarch64 against Pinocchio's pip wheels and copies the binary with, next to it,
//! `geckoleg_pinocchio-libs/lib/`, the libraries it loads.

// DECISION: the shared module beside the example, in its own folder so Cargo does not take it
// for an example; a crate root looks up `mod` in its own folder, hence `#[path]`.
#[path = "geckoleg_pinocchio/leg.rs"]
mod leg;

use kmr_api::cubemars::Ak60;
use kmr_api::{Bus, EachTick, Init, Policy, Q, Qd};
use leg::{Gait, LIMITS, PEAK, RANGE_HI, RANGE_LO};
use std::time::Duration;

kmr_api::robot!(model = "models/geckoleg.urdf");

/// Model time = wall time x SPEED (1.0 = the C++ project's real speed). Start very low.
const SPEED: f64 = 1.0;
/// Control period, s (C++ `Tc`). Unchanged by `SPEED`.
const TC: f64 = 0.005;
const KP: f32 = 150.0;
const KD: f32 = 1.0;
/// Margin added to the joint range of one cycle, rad.
const LIMIT_MARGIN: f32 = 0.15;
/// Largest distance between a goal and the measured position, deg.
const MAX_GOAL_STEP_DEG: f32 = 20.0;
/// How long the leg holds its last goal before the torque goes off.
const HOLD: Duration = Duration::from_millis(200);

fn main() {
    let dry = std::env::args().any(|a| a == "--dry-run");
    let speed = std::env::args()
        .skip_while(|a| a != "--speed")
        .nth(1)
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(SPEED);
    // DECISION: the speed is printed here, not after the range line: C++ prints that one and
    // its context carries no speed.
    println!("SPEED {speed}. Ctrl-C stops.");

    let robot = Robot::new()
        .set_dt(Duration::from_secs_f64(TC))
        .set_hold(HOLD)
        .set_limit_policy_all(Policy::Clamp)
        // If you get the `OUT_DIR` error: most likely build.rs failed because pkg-config can't
        // find Pinocchio's C++ library. Install it either in
        // - /usr: found automatically
        // - /usr/local or /opt/<dir>: set PKG_CONFIG_PATH to its lib/pkgconfig (also in your
        //   editor's rust-analyzer env)
        // and set CXXFLAGS="$(pkg-config --cflags pinocchio)".
        .add_controller(Init, kmr_api::c!(check_gait_cpp, with = LIMITS))
        .add_controller(EachTick, kmr_api::c!(walk_cpp, with = Gait::new(speed)));
    // Without actuators the run simulates.
    let robot = if dry {
        robot
    } else {
        let leg = Bus::by_dev("can0").build();
        let vmax = (PEAK * speed * 1.5 + 2.0) as f32;
        robot.add_actuator(
            Ak60::ids(1..=3)
                .gains(KP, KD)
                .position_limits(RANGE_LO, RANGE_HI)
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
