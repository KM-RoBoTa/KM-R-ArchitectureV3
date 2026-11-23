// Tutorial 09: giving a controller its own memory.
//
// A controller is a plain `fn`: when it returns, its local variables are
// gone. But real controllers need to remember things from one tick to the
// next: a flag, a counter, a filter's last value, a target.
//
// `with(ctx, f)` is how. You build your data once in `main`, hand it to the
// robot, and the robot lends it back to your controller every tick. No
// globals, no locks.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, EachTick, Q, Sensors, Time, with};
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

// If you wish to track your own data, flags or else, you would use
// `with(ctx, f)`: `add_controller(EachTick, with(ctx, f))` hands `f` the
// `ctx: &mut T` (which is whatever you want) each tick.
//
// Note the signature: your data comes first, then the usual three.
fn context_controller(
    tracker: &mut UserDefinedDataTracker,
    _time: &Time,
    robot: &mut State,
    _sensors: &Sensors,
) {
    // here, we simply read a flag the user defined and set all positions
    // if true.
    // A run without hardware reads nothing: no sample to work from, skip the tick.
    if robot.q().is_none() {
        return;
    }
    if *tracker.read_flag() {
        let current = robot.q().unwrap();
        let desired = current.map(|q| q + 100.0.into());
        robot.set_all_q(desired);
    }
    // This sticks: next tick, `tracker` is the same value, the flag is still
    // `false`, and we don't move again.
    tracker.write_flag(false);
}

// // Simple case of a controller that tracks the array of Q itself.
// // This is a simple showcase of how you would track data or use your own
// // structures.
// fn tracking_controller(
//     tracker: Arc<Mutex<UserDefinedDataTracker>>,
// ) -> impl FnMut(&Time, &mut State, &Sensors) {
//     move |_time: &Time, robot: &mut State, _sensors: &Sensors| {
//         tracker.lock().unwrap().save_q(&robot.q().unwrap());
//
//         let desired = robot.q().unwrap().map(|q| q + 100.0.into());
//         robot.set_all_q(desired);
//     }
// }

// Your own data: any plain struct works, no trait to implement. `with` moves
// it into the robot once, so each controller you give a tracker owns its
// own; it lives as long as the run.
#[derive(Default)]
struct UserDefinedDataTracker {
    tracked_q: [Q; N],
    camera_data: [bool; 2],
    shared_flag: bool,
}
impl UserDefinedDataTracker {
    fn save_q(&mut self, q: &[Q; N]) {
        self.tracked_q = *q;
    }
    fn save_camera_feed(&mut self, data: &[bool; 2]) {
        self.camera_data = *data
    }

    fn read_flag(&self) -> &bool {
        &self.shared_flag
    }
    fn write_flag(&mut self, value: bool) {
        self.shared_flag = value;
    }
}

fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;

    // The own-data controller's tracker. Built and initialized ONCE, here —
    // not on every tick. We hand ownership to the API below; from then on it
    // lends this exact value back to `context_controller` as `&mut`.
    let mut own_tracker = UserDefinedDataTracker::default();
    own_tracker.write_flag(true); // your one-time init

    let robot = Robot::new();
    let robot = if cfg!(feature = "dynamixel") {
        robot.add_plugins(Bus::by_id("FT8ISZ6H").baud(1_000_000).bus("bus0"))
    } else {
        robot
    };
    robot
        .set_dt(Duration::from_millis(1u64))
        // Own-data controller: no closure, no Arc/Mutex. We move the already
        // initialized `own_tracker` in — the API keeps it alive across ticks.
        .add_controller(EachTick, with(own_tracker, context_controller))
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
