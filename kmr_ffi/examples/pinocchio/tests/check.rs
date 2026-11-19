//! `check_gait_cpp` on the real loop: limits the gait leaves stop the run in `Init`, before
//! `PostInit` and any tick; the declared limits let it start.
//!
//! The scripted hosts of kmr_api's test hooks never read the global signal bus, where a C++
//! `emergency_stop` lands, so these runs use `run()`, in a test binary of their own.

// The walk's half of it goes unused here.
#[allow(dead_code)]
#[path = "../src/leg.rs"]
mod leg;

use core::cell::Cell;
use std::sync::Mutex;
use std::time::Duration;

use kmr_api::{First, Init, Policy, PostInit, Sensors, Time};
use leg::{LIMITS, Limits, PEAK, RANGE_HI, RANGE_LO};

kmr_api::robot!(model = "../../../controllers/models/geckoleg.urdf");

/// One run at a time: the signal bus and the Ctrl-C handler are the process's.
static RUN: Mutex<()> = Mutex::new(());

/// Runs the check with `limits`, no actuators, the 5 ms tick of src/main.rs, shutting down on
/// tick 0; whether `PostInit` ran.
fn starts_with(limits: Limits) -> bool {
    let _one = RUN.lock().unwrap_or_else(|e| e.into_inner());
    let started = Cell::new(false);
    Robot::new()
        .set_dt(Duration::from_millis(5))
        .set_limit_policy_all(Policy::Clamp)
        .add_controller(Init, kmr_api::c!(check_gait_cpp, with = limits))
        .add_controller(PostInit, |_: &Time, _: &mut State, _: &Sensors| {
            started.set(true);
        })
        .add_controller(First, |_: &Time, _: &mut State, _: &Sensors| {
            kmr_api::shutdown!("check: tick 0");
        })
        .run()
        .expect("runs");
    started.get()
}

#[test]
fn the_declared_limits_let_the_run_start() {
    assert!(starts_with(LIMITS));
}

#[test]
fn limits_the_gait_leaves_stop_the_run_in_init() {
    let narrow_knee = Limits {
        lo: RANGE_LO,
        hi: [RANGE_HI[0], 0.0, RANGE_HI[2]],
        peak: PEAK,
    };
    let slow = Limits {
        lo: RANGE_LO,
        hi: RANGE_HI,
        peak: 1.0,
    };
    assert!(!starts_with(narrow_knee));
    assert!(!starts_with(slow));
}
