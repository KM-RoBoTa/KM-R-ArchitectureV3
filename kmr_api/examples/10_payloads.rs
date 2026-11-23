// Tutorial 10: payloads, the things you bolt onto the robot.
//
// An arm, a camera, a tablet: robots rarely stay as they left the factory.
// Here we see how to tell kmr about them, once, in the builder. In return,
// `run()` checks that each one is mounted on a link your model really has,
// before the robot moves, instead of you finding the typo on the bench.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, EachTick, Init, Payload, Q, Sensors, Tier, Time, with};
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

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

// Fundamentally, a payload is just a thing the user added.
// These come in 3 types:
// Light  — inert attachment, no hardware interface or network connection
// Medium — connects to a hardware interface and reads from the robot's state SoA
// Heavy  — registers services that integrate with the broader robot system (tablet UI, other payloads, etc.)
//
// (payload reference: https://dev.bostondynamics.com/docs/payload/readme)
//
// A medium or heavy payload is registered once, through the builder:
// `.register_payload(Payload::medium(kg, "link"))` below, never from a
// controller. `run()` refuses a link the model does not have. A light
// payload is not registered at all.
//
// Its new sensors or actuators are not moved by the architecture: the
// integration layer is yours (unless you want us to do it for you). Below,
// a pseudo-implementation of a 3 DOF arm with a camera, brought up by an
// `Init` controller. Controllers run inline in the loop; only
// `threaded(ctx, f)` gives one its own thread (stored, not spawned yet: see 11).
//
// A plain `fn` — no closure, no returned `impl FnMut`, no `move`. The
// shared tracker arrives as `&mut T` because we register it through
// `with(ctx, f)`: the robot OWNS the `Arc<Mutex<..>>` for us and lends it
// back each time it runs. Here that is once: `Init` runs a single time, at
// startup.
fn use_payload(tracker: &mut ArcMutTracker, _time: &Time, _robot: &mut State, _sensors: &Sensors) {
    // Let's imagine a 3 DOF arm with a camera at the EE.

    type Degrees = u32;

    // Structs are declared within `use_payload` body for clarity purposes.
    // This is not a good practice because you redefine the structure at each
    // declaration and make it private to that function scope.

    #[derive(Default)]
    struct Joints;
    impl Joints {
        // Your integration of 3 joints
        pub fn move_joint(&self, goal: Degrees) {
            println!("Moves joint X at {}", goal);
        }
    }

    struct Camera {
        output: [bool; 2],
    }
    impl Default for Camera {
        fn default() -> Self {
            let on = true;
            let off = false;
            let pixel1 = on;
            let pixel2 = off;
            let output = [pixel1, pixel2];

            Self { output }
        }
    }
    impl Camera {
        // returns output of the camera as a set of 2 pixels that are either
        // on or off.
        // Crazy resolution, right ?
        pub fn read_feed(&self) -> &[bool; 2] {
            &self.output
        }
    }

    #[derive(Default)]
    struct CameraArm {
        joints: [Joints; 3], // three joints
        camera: Camera,
    }

    // Bring it up: move one joint, read a frame, and keep the frame in the
    // shared tracker so the rest of your code can use it.
    let my_arm = CameraArm::default();
    my_arm.joints.get(1).unwrap().move_joint(90); // 90 Degrees

    let feed = my_arm.camera.read_feed();

    // `lock()` gives exclusive access until the guard is dropped (end of
    // this line). `unwrap()` here panics only if another thread panicked
    // while holding the lock — fine for an example.
    tracker.lock().unwrap().save_camera_feed(feed);
}

type ArcMutTracker = Arc<Mutex<UserDefinedDataTracker>>;

fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;

    // Shared tracker. `Arc` lets several threads OWN the same value (cheap
    // reference-counted handle), and `Mutex` lets them mutate it safely one at
    // a time. Each controller will get its own `Arc::clone(&tracker)` — same
    // data underneath, just another handle to it.
    let tracker = Arc::new(Mutex::new(UserDefinedDataTracker::default()));

    // A payload is a small value: its tier, its mass in kg, and the model
    // link it sits on. Today kmr checks the link at `run()`; the mass is
    // kept for later (the center of mass is on the roadmap).
    let tablet = Payload::heavy(2.5, "link4");
    let _tier: Tier = tablet.tier();
    let _mass_kg: f32 = tablet.mass_kg();
    let _mount: &str = tablet.mount();

    let robot = Robot::new();
    let robot = if cfg!(feature = "dynamixel") {
        robot.add_plugins(Bus::by_id("FT8ISZ6H").baud(1_000_000).bus("bus0"))
    } else {
        robot
    };
    robot
        .set_dt(Duration::from_millis(1u64))
        // The camera arm, mounted on the model's last link.
        .register_payload(Payload::medium(1.2, "link4"))
        // The tablet from above, on the same link.
        .register_payload(tablet)
        // `Arc::clone` makes another handle to the SAME tracker (cheap — just
        // bumps a counter). `with` hands this handle to the robot, which lends it
        // to `use_payload` as `&mut` — no closure needed anymore.
        .add_controller(Init, with(Arc::clone(&tracker), use_payload))
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
