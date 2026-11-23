// Tutorial 12: reading sensors, and stopping the robot NOW.
//
// Actuators are not the only thing a robot feels the world with. Here we see
// the third argument every controller gets, `&Sensors`, and the big red
// button: `emergency_stop!`, for when carrying on would be dangerous.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Accel, Bus, EachTick, Gyro, Sensors, Time};
use std::time::Duration;

kmr_api::robot!(
    XmBench,
    hardware = |hw| hw.baud("bus0", 1_000_000).read_every("bus0", 4)
);

const DEMO_LENGTH: Duration = Duration::from_millis(50);

// The same helper as in the earlier examples: it ends the demo after 50 ms
// with a graceful shutdown.
fn use_time(time: &Time, _robot: &mut State, _sensors: &Sensors) {
    let _period: Duration = time.dt();
    let _tick: u64 = time.tick();
    if let Some(late) = time.last_overrun() {
        println!("tick {} finished {late:?} late", time.tick());
    }

    if time.elapsed() >= DEMO_LENGTH {
        kmr_api::shutdown!("demo over");
    }
}

// `shutdown!` (see `use_time`) is the polite way out: the tick that asked
// for it is finished, then the loop stops. `emergency_stop!` does not wait:
// the loop leaves at its next check, between two phases, and every actuator
// gets its torque cut on the way out.
//
// Use it when one more tick could hurt the robot or someone near it. Like
// `shutdown!`, you can call it from anywhere: a controller, a helper deep
// inside it, another thread.
fn estop(_time: &Time, _robot: &mut State, _sensors: &Sensors) {
    // Your battery reading here; an emergency stop cuts torque at once.
    let battery_percent = 80;
    if battery_percent < 10 {
        kmr_api::emergency_stop!("battery too low");
    }
}

// This function showcases how you would read the sensors added to the
// robot with `Robot::add_sensor`. kmr knows 2 kinds of sensors:
//
// 1) An IMU: acceleration (m/s²) and angular rate (rad/s), on x, y and z.
// 2) A microphone: raw 16-bit sound samples.
//
// Sensors are counted per kind, in the order you added them: `imu(0)` is
// the first IMU, `mic(0)` the first microphone. Each one is `None` until a
// sensor at that index has sent a first reading. This robot adds no sensor,
// so here both stay `None`; `xiao_imu.rs` shows a real board.
//
// And if a sensor goes quiet mid-run? kmr raises `sensor_missing` in the
// faults (tutorial 08), and `set_sensor_policy` (tutorial 14) can make it
// log a warning or stop the robot for you.
fn use_sensors(_time: &Time, _robot: &mut State, sensors: &Sensors) {
    if let Some(imu) = sensors.imu(0) {
        let _accel: Option<Accel> = imu.accel();
        let _gyro: Option<Gyro> = imu.gyro();
        let _x: Option<f32> = imu.accel().map(|a| a.x());
        // An IMU is often slower than the tick: the same reading can show
        // for several ticks. `age` says how many ticks old it is, 0 when it
        // just arrived, so you know when you are looking at old news.
        let _age: u64 = imu.age();
        let _dropped: u64 = imu.dropped();
    }
    if let Some(mic) = sensors.mic(0) {
        // Only the samples that arrived since the last tick, oldest first.
        let _samples: &[i16] = mic.samples();
        let _rate: u32 = mic.rate_hz();
        let _dropped: u64 = mic.dropped();
    }
}

fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;

    let robot = Robot::new();
    let robot = if cfg!(feature = "dynamixel") {
        robot.add_plugins(Bus::by_id("FT8ISZ6H").baud(1_000_000).bus("bus0"))
    } else {
        robot
    };
    robot
        .set_dt(Duration::from_millis(1u64))
        .add_controller(EachTick, use_sensors)
        .add_controller(EachTick, estop)
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
