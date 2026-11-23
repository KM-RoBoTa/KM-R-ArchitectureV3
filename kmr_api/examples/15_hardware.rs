// Tutorial 15: plugging in real hardware.
//
// Until now, every example could run on a laptop with nothing plugged in.
// Here we see the ways to tell kmr which motors sit on which cable, and how
// to get your hands on the drivers before and after a run.
//
// 1) A robot whose `robot!` comes with a hardware table (like `XmBench`):
//    plug in one bus with `add_plugins`, see `main`.
// 2) A robot without one: declare the actuators yourself, see
//    `declared_bench`.
// 3) Bind first, run later, see `bind_then_run`.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{
    Actuators, Bus, EachTick, Sensors, Time,
    dynamixel::{self, Mode, Zero},
};
use std::time::Duration;

// `XmBench` is a preset: it already knows its motors and the bus they sit
// on. The `|hw|` closure only tweaks that table: run bus "bus0" at
// 1 000 000 baud, and read it every 4 ticks. A typo in a bus label here is
// a compile error, not a surprise at run time.
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

// WARN: Think of what happens if an actuator needs to shutdown mid-run for
// an effect. Also think of read-only actuators (off but we just read its
// state like in driver-driven displays)

fn declared_bench() -> Actuators {
    // When your `robot!` has no hardware table (say,
    // `robot!(model = "arm.urdf")`), you declare the actuators in code, then
    // hand them to `Robot::add_actuator`. `XmBench` already has a table, so
    // this function is only here to show the shape: `main` never calls it,
    // and a robot with a table refuses `add_actuator` at `run()`.
    //
    // First the cable. `build()` gives a handle you pass to every actuator on
    // that bus.
    let bus = Bus::by_dev("/dev/ttyUSB0").baud(1_000_000).build();
    // One actuator, firmware id 5, that kmr reads but never powers.
    let _one = dynamixel::Xm430W210::id(5).read_only().on(bus);
    // One actuator per joint of the robot model, at ids 1 to N.
    let _all = dynamixel::Xm430W210::all().on(bus);
    // Ids 1 to 4, with their setup written at bring-up, before the run.
    dynamixel::Xm430W210::ids(1..=4)
        .mode(Mode::Position) // only `Position` runs today
        .zero(Zero::DEFAULT) // where 0 rad sits
        .gains(800, 0, 0) // P, I, D, as register values
        .led(true)
        // Written to the motor's own memory, so its firmware watches them
        // too, not only your code.
        .voltage_limits(10.0, 14.0)
        .temperature_limit(70)
        // ...and put back as they were when the run ends, so a test run
        // doesn't leave the motor changed for good.
        .restore_limits()
        .on(bus)
}

// `run()` does two things: bind the hardware to its drivers, then run. You
// can split them. `bind()` checks the setup and builds the drivers without
// talking to the hardware yet, so a wrong setup fails here, early.
fn bind_then_run() -> color_eyre::Result<()> {
    let mut bound = Robot::new().add_controller(EachTick, use_time).bind()?;
    // The drivers, before the run, for anything brand-specific you need to
    // do first. `None` when there is no hardware and the run simulates.
    if let Some(_drivers) = bound.drivers() {}
    // The run lends the drivers and hands them back, closed, when it ends.
    let _drivers_back = bound.run()?;
    Ok(())
}

fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;

    let robot = Robot::new();
    // With the `dynamixel` feature, we plug in the real bus: the USB adapter
    // with this serial id. `by_id` survives reboots and replugging, unlike
    // `/dev/ttyUSB0`. Without the feature, no bus is added and kmr simulates
    // the actuators, so the same example runs on any laptop.
    let robot = if cfg!(feature = "dynamixel") {
        robot.add_plugins(Bus::by_id("FT8ISZ6H").baud(1_000_000).bus("bus0"))
    } else {
        robot
    };
    robot
        .set_dt(Duration::from_millis(1u64))
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
