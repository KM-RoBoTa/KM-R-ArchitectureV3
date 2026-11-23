// Tutorial 13: telemetry, the health of your actuators.
//
// Positions tell you where a joint is. Telemetry tells you how it FEELS:
// how hot it runs, what voltage it gets, whether its own firmware flagged a
// problem. A motor rarely dies without warning; it heats up first, or its
// supply sags. Watching that lets you stop the robot cleanly before a joint
// gives up on its own in the middle of a move.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, Celsius, EachTick, Sensors, Time, Volts, dynamixel};
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

fn use_telemetry(time: &Time, robot: &mut State, _sensors: &Sensors) {
    // Temperature, input voltage and the hardware error status are telemetry: the
    // adapter reads them at its own, slower cadence, and you can only read them.
    // `telemetry_age_at(i)` says how many ticks old a reading is.
    // A run without hardware has none: `Ok(None)`.
    if time.tick() == 0 {
        match robot.temperature_at(0) {
            Ok(Some(Celsius(t))) => println!("actuator 0 is at {t} °C"),
            Ok(None) => println!("actuator 0: no temperature reading"),
            Err(e) => println!("{e}"),
        }
    }
    // Every actuator at once, `None` where there is no reading yet.
    let _all: [Option<Celsius>; N] = robot.temperatures();
    let _voltage: Option<Volts> = robot.voltage_at(0).unwrap_or(None);
    let _age: Option<u64> = robot.telemetry_age_at(0).unwrap_or(None);
    // The hardware error status comes as a raw byte whose bits mean
    // different things on each brand. Decode it with the model your actuator
    // is, and you get named flags instead of bit fiddling.
    if let Ok(Some(raw)) = robot.hardware_error_at(0) {
        let error = dynamixel::Xm430W210::hardware_error(raw);
        let _raw: u8 = error.raw();
        let _clear: bool = error.is_empty();
        let _any: bool = error.any();
        let _voltage: bool = error.input_voltage();
        let _encoder: bool = error.motor_encoder();
        let _shock: bool = error.electrical_shock();
        // A hot or overloaded motor is a good reason to stop: stopping here,
        // on purpose, beats finding out a joint gave up halfway through a move.
        if error.overheating() || error.overload() {
            kmr_api::emergency_stop!("actuator 0 hardware error");
        }
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
        .add_controller(EachTick, use_telemetry)
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
