// Tutorial 11: a gamepad, or anything that lives on its own thread.
//
// A gamepad, a network link, a slow planner: some inputs have their own
// rhythm and may block while they wait. Put that inside a normal controller
// and every tick waits with it. Here we see how to keep such work away from
// the tick loop, and how to hand its result back to the robot safely:
//
// 1) A threaded controller reads the pad and never touches the robot.
// 2) A `Latest` slot carries the newest reading across threads.
// 3) A normal controller on the main loop takes it and commands the robot.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{
    Bus, EachTick, Latest, LatestReader, LatestWriter, Q, Sensors, Time, threaded, with,
};
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

// Whatever a threaded controller wants to track. Here: latest gamepad input.
// `Copy`: the pad crosses threads by value, through a `Latest` slot.
#[derive(Clone, Copy, Default)]
struct GamepadInput {
    x: f32,
    y: f32,
    buttons: u8,
}

// The pad is written on the gamepad THREAD and read on the main loop by
// `use_gamepad`: two threads, one value. A `Latest` slot carries it: the
// writer puts the newest reading, the reader takes it, and neither ever
// waits, locks or allocates. A reading the loop had no time to take is
// simply replaced by the next one (and counted in `superseded()`).
//
// A controller that belongs on its OWN thread: a gamepad reader. It runs its
// own blocking poll loop at its own rate — you don't want that stalling the
// tick loop, and it must NOT touch robot state (that stays single-writer on
// the main loop, so no actuator-write races are possible).
//
// A threaded controller gets ONLY `&mut T` and the clock. Here `T` is the
// writing half.
fn gamepad_controller(pad: &mut LatestWriter<GamepadInput>, _time: &Time) {
    // let ev = read_gamepad();  // your blocking poll
    pad.put(GamepadInput {
        x: 0.0,
        y: 0.0,
        buttons: 0,
    });
    let _replaced: u64 = pad.superseded();
}

// The main loop's side: the reading half, and the last reading taken, kept
// for the ticks where nothing new arrived.
struct Pad {
    latest: LatestReader<GamepadInput>,
    input: GamepadInput,
}

// The other half: a MAIN-LOOP controller that uses the gamepad to command
// the robot. It needs `&mut State`, so it can't be threaded.
fn use_gamepad(pad: &mut Pad, _time: &Time, robot: &mut State, _sensors: &Sensors) {
    // One atomic load when nothing is new: the tick never waits on the pad.
    if let Some(input) = pad.latest.take() {
        pad.input = input;
    }
    let _missed: u64 = pad.latest.superseded();
    let (x, _y) = (pad.input.x, pad.input.y);

    // Turn the stick into a robot command.
    // A run without hardware reads nothing: no sample to work from, skip the tick.
    if robot.q().is_none() {
        return;
    }
    let current = robot.q();
    let desired = current.unwrap().map(|q| q + Q(x));
    robot.set_all_q(desired);
}

fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;

    // The gamepad slot, built ONCE here and split: the writer goes to the
    // gamepad thread, the reader to `use_gamepad` on the main loop.
    let (pad_writer, pad_reader) = Latest::new().split();
    let pad = Pad {
        latest: pad_reader,
        input: GamepadInput::default(),
    };

    let robot = Robot::new();
    let robot = if cfg!(feature = "dynamixel") {
        robot.add_plugins(Bus::by_id("FT8ISZ6H").baud(1_000_000).bus("bus0"))
    } else {
        robot
    };
    robot
        .set_dt(Duration::from_millis(1u64))
        // Gamepad WRITER on its own thread. No robot state → can't race commands.
        //
        // Heads-up: in this version of kmr a threaded controller is stored but
        // its thread is not started yet, so `gamepad_controller` never runs
        // and `use_gamepad` keeps its default input. The wiring stays the same
        // once it does.
        .add_controller(EachTick, threaded(pad_writer, gamepad_controller))
        // Gamepad READER on the main loop: needs State, so not threaded. The
        // other half of the same slot: the bridge from thread back to the robot.
        .add_controller(EachTick, with(pad, use_gamepad))
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
