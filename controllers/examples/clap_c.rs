//! The clap controller of `src/main.rs`, its logic in `c/clap.c`: Rust keeps the builder, the
//! buses, the actuators and the HOME ramp; C moves both snakes on a clap.
//!
//! `cargo run -p controllers --example clap_c --features c,hardware`; the Dynamixel
//! declarations need `hardware`, as in `src/main.rs`.

use std::time::Duration;

use kmr_api::{EachTick, Q, presets::home::Home};

kmr_api::robot!(model = "models/ax_12_12act_an_rx28_10act.urdf");

/// Ramps every actuator to the centre, raw 512, in 3 s; the C controller waits for it.
const HOME: Home<N> = Home::new([Q(0.0); N], Duration::from_secs(3));

/// `clap` of `c/clap.c`: when the last clap's motion started. Only C reads it.
#[repr(C)]
#[derive(Default)]
#[allow(dead_code)]
struct Clap {
    since: f64,
    clapped: bool,
}

fn main() {
    let ax_snake = kmr_api::Bus::by_id("AL02L1KL").baud(1_000_000).build();
    let rx_snake = kmr_api::Bus::by_id("AL02L1EU").baud(1_000_000).build();

    let robot = Robot::new()
        .set_dt(Duration::from_millis(5))
        .set_hold(Duration::from_millis(50))
        .set_limit_policy_all(kmr_api::Policy::Clamp)
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
        .add_controller(
            EachTick,
            kmr_api::c!(move_on_clap_c, with = Clap::default()),
        );
    #[cfg(feature = "hardware")]
    let robot = robot.add_sensor(kmr_api::xiao::Xiao::usb().mic());

    if let Err(e) = robot.run() {
        eprintln!("{e}");
    }
}
