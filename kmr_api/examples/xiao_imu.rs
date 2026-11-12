//! The XIAO nRF52840 Sense's gyroscope over USB, logged every 100 ticks; the actuators simulate.
//!
//! `cargo run -p kmr_api --example xiao_imu --features xiao`, the board flashed with the sensor
//! firmware and plugged in. Ctrl-C stops it.

use std::time::Duration;

use kmr_api::xiao::Xiao;
use kmr_api::{EachTick, Sensors, Time};

kmr_api::robot!(XmBench);

/// Logs the gyroscope's newest reading and its age every 100 ticks.
fn log_gyro(time: &Time, _robot: &mut State, sensors: &Sensors) {
    if !time.tick().is_multiple_of(100) {
        return;
    }
    match sensors.imu(0) {
        Some(imu) => match imu.gyro() {
            Some(w) => println!(
                "tick {}: gyro x {:+.3} y {:+.3} z {:+.3} rad/s, {} tick(s) old",
                time.tick(),
                w.x(),
                w.y(),
                w.z(),
                imu.age()
            ),
            None => println!("tick {}: the gyroscope is off", time.tick()),
        },
        None => println!("tick {}: no reading yet", time.tick()),
    }
}

fn main() {
    let run = Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_sensor(Xiao::usb().imu())
        .add_controller(EachTick, log_gyro)
        .run();
    if let Err(e) = run {
        eprintln!("{e}");
    }
}
