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
    // At 1 kHz, printing every tick would flood the terminal and slow the
    // loop down: once every 100 ticks is plenty to watch it live.
    if !time.tick().is_multiple_of(100) {
        return;
    }
    // `imu(0)` is the first IMU added to the robot: the board's. It stays
    // `None` until the board has sent a first reading, so the first lines
    // may say so while the USB link comes up.
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
            // `gyro()` is `None` when that half of the IMU is switched off.
            None => println!("tick {}: the gyroscope is off", time.tick()),
        },
        None => println!("tick {}: no reading yet", time.tick()),
    }
}

fn main() {
    // No bus is plugged in, so the actuators simulate: only the sensor is
    // real. `Xiao::usb().imu()` asks the board for its IMU over USB; kmr
    // opens it at startup, after the actuators, and reads it each tick
    // before your controllers run.
    let run = Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_sensor(Xiao::usb().imu())
        .add_controller(EachTick, log_gyro)
        .run();
    // A board that does not come up (unplugged, say) ends the run here, with
    // an error that names the sensor and the link.
    if let Err(e) = run {
        eprintln!("{e}");
    }
}
