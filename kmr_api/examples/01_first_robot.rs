// Your first kmr robot.
//
// A kmr program has three parts, and this file shows each of them:
//
// 1) Name the robot once, with `robot!`.
// 2) Write controllers: plain functions that kmr calls for you.
// 3) In `main`, build the robot, say WHEN each controller runs, and `run()`.
//
// kmr owns the control loop, the clock and the hardware. You only write what
// happens inside a tick. Run it with:
// `cargo run -p kmr_api --example 01_first_robot`
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, EachTick, Init, Last, PostInit, PreInit, Sensors, Time};
use std::time::Duration;

// The robot: model, N, brand, baud, ids. Gives this crate `Robot`, `State` and `N`.
//
// `XmBench` is a preset: four Dynamixel XM430 on one bus. The `|hw| ..` part
// tweaks its hardware table: `bus0` runs at 1 Mbit/s and is read every 4
// ticks. Between two reads, you keep seeing the last sample.
kmr_api::robot!(
    XmBench,
    hardware = |hw| hw.baud("bus0", 1_000_000).read_every("bus0", 4)
);

// Every controller has the same shape: `fn(&Time, &mut State, &Sensors)`.
// The clock, the robot's state, and its sensors. That's all you get, and all
// you need.
//
// This one is registered below on the three startup phases. Each of them runs
// once, before the loop starts:
//
// - `PreInit`: software only, no hardware touched yet.
// - `Init`: the hardware is opened, not powered yet.
// - `PostInit`: the hardware is checked and read once, so `robot.q()` has a
//   first sample here (on real hardware).
fn initialization(_time: &Time, _robot: &mut State, _sensors: &Sensors) {
    // Init runs once, before the first tick, and its writes are discarded (SRS FR-5.2):
    // no homing here. A robot needs no home (SRS FR-2.7); when you want one, register
    // the preset `kmr_api::presets::home::Home` on a tick phase:
    //
    // const HOME: Home<N> = Home::new([Q(0.0); N], Duration::from_secs(3));
    //
    // `.add_controller(EachTick, HOME.controller())` ramps every actuator in a straight
    // line from its first sample to the pose, reached after 3 s, then holds the pose.
    // `.add_controller(EachTick, HOME.then(replace_all_q))` homes the same way, then
    // hands every tick to `replace_all_q`, whose `time.elapsed()` starts at zero.
}

// How long this example runs before it stops itself.
const DEMO_LENGTH: Duration = Duration::from_millis(50);

fn use_time(time: &Time, _robot: &mut State, _sensors: &Sensors) {
    // You may have noticed the `&Time` struct in the signature. It is the
    // loop's clock, read-only: the time since the start, the tick period,
    // the tick number and how late the loop ran.
    let _period: Duration = time.dt();
    let _tick: u64 = time.tick();
    // `last_overrun()` is about the tick BEFORE this one: `None` if it was on time.
    if let Some(late) = time.last_overrun() {
        println!("tick {} finished {late:?} late", time.tick());
    }

    // Here, the example ends itself once it has run long enough. A shutdown
    // is graceful: this tick finishes, then `run()` returns `Ok(())`.
    if time.elapsed() >= DEMO_LENGTH {
        kmr_api::shutdown!("demo over");
    }
}

// Registered on `Last`, so it runs at the end of every tick. A loop that is
// often late means a controller does too much work for your `dt`; these two
// numbers tell you how often, and by how much in total.
fn use_overruns(time: &Time, _robot: &mut State, _sensors: &Sensors) {
    let _count: u64 = time.overrun_count();
    let _total: Duration = time.accumulated_overrun();
}

fn main() -> color_eyre::Result<()> {
    // color_eyre is optional but it's nicer to work with.
    // We already provide plenty of human readable error messages.
    // color_eyre just makes them more pleasing and easy to read.
    color_eyre::install()?;

    let robot = Robot::new();

    // If no hardware is plugged in, the architecture defaults to simulation
    // because inferring where your hardware is plugged in is impossible
    // unless you tell it with a plugin. Here, the `dynamixel` feature picks.
    // A simulation reads nothing: `robot.q()` stays `None` the whole run.
    //
    // You can think of plugins as "settings" for the architecture and the
    // hardware itself. Not all plugins are mandatory for the architecture to
    // work with your hardware.
    //
    // `FT8ISZ6H` is the serial id of OUR USB adapter. Find yours under
    // `/dev/serial/by-id`: unlike `/dev/ttyUSB0`, it survives a reboot.
    let robot = if cfg!(feature = "dynamixel") {
        robot.add_plugins(Bus::by_id("FT8ISZ6H").baud(1_000_000).bus("bus0"))
    } else {
        robot
    };

    robot
        // Settings
        //
        // These settings are not plugins. They are the setters for the
        // internal control loop. Here, we simply specify the length of each
        // tick, `dt` (1 ms = a 1 kHz loop). Outside [10 µs, 1 s], `run()`
        // refuses it before touching any hardware; never set, it is 1 ms.
        // If you don't know about `Duration` yet: `Duration` comes from the
        // rust standard lib. It allows you to specify any valid time scale.
        .set_dt(Duration::from_millis(1u64))
        // Controllers
        //
        // Here is where you specify your controllers.
        // You can use ONE, or more.
        // They are pretty simple in concept.
        // 2 parameters : a "WHEN" and "WHAT".
        // The WHEN is a `Schedule` which specifies when (before or during)
        // the control loop your controller will execute. In most cases,
        // the `EachTick` does most of what you need.
        // The second parameter is your controller, as a function that takes
        // `&Time`, `&mut State` and `&Sensors`.
        //
        // Each tick runs `First`, then `EachTick`, then `Last`. Controllers on
        // the same schedule run in the order you add them. And a robot with no
        // controller at all does not even compile.
        .add_controller(PreInit, initialization)
        .add_controller(Init, initialization)
        .add_controller(PostInit, initialization)
        .add_controller(EachTick, use_time)
        .add_controller(Last, use_overruns)
        // `run()` blocks until a `shutdown!`, an `emergency_stop!` or Ctrl-C.
        // However it ends, kmr cuts the torque and closes the hardware for
        // you, even on a panic.
        .run()?;
    Ok(())
}
