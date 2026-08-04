#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{
    Init, N, EachTick, Q, Robot, Sensors, State, StateErrorKind as StateError, Time,
};
use std::sync::{Arc, Mutex};

// fn stop(robot: RobotState) {
//     a.map(|x| Q(0_f32))
//     robot.command
// }

fn main() -> color_eyre::Result<()> {
    // color_eyre is optional but it's nicer to work with.
    // We already provide plenty human readable error messages.
    // color_eyre just makes them more pleasing and easy to read.
    color_eyre::install()?;

    fn initialization(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        // `home_state` returns ANY state. To ensure home_state() knows what
        // to return you, you must specify the type you wish. In that case,
        // this is done by the `let variable: Type` notation.
        let _desired: [Q; N] = robot.initial_state();

        // You can also write it with the "turbofish" notation if you desire.
        let _desired = robot.initial_state::<Q>();

        // But, since we call `set_all_q` at the end, the compiler is
        // smart enough to know you want Q.
        let desired = robot.initial_state();

        // You can convince yourself of that fact by commenting out the next
        // line. This is precisely why we learned about this now... Because
        // you might get surprised by this error, which is in reality just
        // the compiler being confused until you tell it "i want to set Q".
        robot.set_all_q(desired);
        // Note that set_all_q here is a bad practice. All joint will go
        // instantinously to the desired position.

        // Prefer this special case method for that specific scenario.
        robot.go_home(90);
    }

    fn use_time(time: &Time, _robot: &mut State, _sensors: &Sensors) {
        // You may have notices the `&Time` struct in the signature.
        todo!()
    }

    fn replace_all_q(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        // Let's start this example with a simple example.
        // Here, we want to repalace all Q (positions) to an arbitrary
        // value.
        //
        // To do so, we will:
        //
        // 1) Read the current position with `robot.q()`.
        // 2) Replace for all q we receive its value by 0.
        // 3) Commit the change with `robot.set_all_q()`.

        // Here, we read the current value of q. By current, we understand
        // "the latest read value from the actuators themselves".
        let current_q = robot.q();

        // NOTE: the into() needs context to work.
        let _desired_q: [Q; N] = current_q.unwrap().map(|_| 0_f32.into());

        // The code below won't work because `into()` doesn"t know what the f32 must be converted to"
        // let desired = current.unwrap().map(|q| 0_f32.into());

        // Using Q(x) directly gives context to `into()`.
        let _desired = current_q.unwrap().map(|_| Q(0.0));

        let desired = current_q
            .unwrap()
            .map(|q| q + Q(100_f32) + 100_f32.into() + 100.0.into());

        // shows that all of these syntaxes evaluate to the same result.
        assert_eq!(Q(100_f32), 100_f32.into());
        assert_eq!(Q(100_f32), 100.0.into());

        robot.set_all_q(desired);
    }

    fn replace_one_q(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        // We don't always want to set all positions at once. For example,
        // controlling the stem angle of a mounted camera separatly would
        // be complitely separate.

        let index: usize = 0;
        let current_q = robot.q_at(index);

        // To do so, we call the desired q by it's index.
        // Here, we simply add 1.0 to the current position.
        // Keep in mind that you will mosly never see a real useful math
        // control in these examples unless specified so.
        let desired_q = current_q.unwrap() + 1.0.into();

        // Then simply pass you desired value and the index.
        // Because the index might be out of range, you must handle the error.
        // We will show later how to handle error.
        // Right now, keep in mind that we crash the app with `unwrap()`,
        // which is obviously a bad practice we DO NOT recommend.
        //
        // The rust standard library itself states in regards to `unwarp()`:
        // > Because this function may panic, its use is generally discouraged.
        // > Panics are meant for unrecoverable errors, and
        // > [may abort the entire program](https://doc.rust-lang.org/book/ch09-01-unrecoverable-errors-with-panic.html).
        robot.set_q_at(desired_q, index).unwrap();
    }

    // A group is a fixed set of joint indices — non-contiguous allowed, e.g.
    // [1, 3, 4]. Here, the right arm: joints 3, 4, 5.
    const R_ARM: [usize; 3] = [3, 4, 5];
    fn use_groups(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        // One other nice feature are groups. You can specify a dedicated group
        // in the form of an array of indexes, and work with them using the
        // exact same set of methods.
        let mut r_arm = robot.group(R_ARM);

        // Accessing `q()` on a group returns only the current for the chosen
        // indexes in that group. E.g: R_ARM only has 3 joint indexes, so you
        // only read the current values of those 3.
        let current_q = r_arm.q();

        let desired = current_q.unwrap().map(|q| q + Q(100_f32));
        // group index can be out of range, so this returns a `Result` to propagate
        // that cause — it is not swallowed. Joints outside R_ARM are untouched.
        r_arm.set_all_q(desired).unwrap();
    }

    // The next two examples talk about previous states.
    // Using `.history_depth()` you can specify how far you can go.
    //
    // This feature allows you to read the state from past "ticks".
    fn previous_state_at(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        let index: usize = 1;
        let history_depth = 2;
        // Read the next line as "I want to read the state from 2 ticks ago".
        let prev = robot.prev_q_at(index, history_depth).unwrap();
        // then simply multiply the value by 3.
        //
        // WARN: wait, why can i mult without wrapping nor converting the f32 ?
        // this doesnt work in closures, and here, doesnt work with additions.
        let desired = prev * 3_f32;

        robot.set_q_at(desired, index).unwrap();
    }

    // Same example but not constrained to a specific joint index.
    fn previous_state(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        let history_depth = 2;
        let _previous = robot.prev_q(history_depth);
        // and so on...
    }

    fn error_handling(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        let current: Option<[Q; N]> = robot.q();
        // `robot.q()` returns an Option<[Q;N]>.
        // This is because an internal bug can occure, let's say because of an I/O
        // lost paquet. Instead of returning silently a defaulted array, we chose
        // to explicitly let you decide what you would like to do.

        // In rust, you can handle this situation with a bunch of options.
        // Here, we will list some, with short explainations. Note that this is
        // not meant as a rust guide, but rather a guide for good practices.
        //
        // Also note that here, we dont return an error, but an option.
        // So by error, we mean "nothing got returned, but this is recoverable !"
        //
        // Some methods returns `Result<T>`, we will see these cases later.

        // Unwrap().
        //
        // The rust std library is really clear about this:
        //
        // Because this function may panic, its use is generally discouraged.
        // Panics are meant for unrecoverable errors, and
        // [may abort the entire program](https://doc.rust-lang.org/book/ch09-01-unrecoverable-errors-with-panic.html).
        //
        // We are not reinventing the wheel and also discourage its usage because
        // panicking here means: not handling the I/O in case of danger, not
        // shutting down the power on hardware proprely.
        let _error_handled_current = current.unwrap();

        // Pattern matching
        //
        // This one is explicit and doesnt require too much imperative
        // (see: functional programming) rust knowledge.
        //
        // This consist of declaring explicitly what we want to do in each Option<>
        // cases. Option holds either `Some(x)` or `None`.

        let _error_handled_current = match current {
            Some(q_array) => q_array, // on success, we just pass retrive the array

            // We could retry on error.. but the state didn't change since then
            // because the I/O is called only when all controllers finished
            // their tasks.
            //None => robot.q().unwrap(), // And we need to error handle again ...

            // An other option would be to call `home_state()` to then command the
            // robot to got to these. Then call an other controller that handles
            // the motion gracefuly and stop the program entirely with a graceful
            // shutdown call.
            None => {
                // your home controller logic
                initialization(_time, robot, _sensors);

                todo!("call graceful shutdown");
            }
        };

        // Result
        let index: usize = 1; // ok
        let history_depth: usize = 999; // absurd
        // Some errors might come from logic errors. Because its less evident to
        // know what happened, an `Option<>` wouldn't be useful here. `None` doesn't
        // tell you if the error was because of the `index` or the `history_depth`.
        let _current = match robot.prev_q_at(index, history_depth) {
            Ok(q) => q,

            // This is much more useful isn't it ?
            // It's a lot of work, but you handle everything gracefuly.
            //
            // We recommend learning about Options and Results more in depth as
            // this will eliviate the pain of error handling by learning core
            // principles such as ignoring errors, mapping errors, returning it with
            // the `?` operator to handle it elsewhere and much more.
            // `e.kind()` is the bare, payload-free cause — match it without
            // `{ .. }`. The numbers are still in `e` (its `Display`) if you want them.
            Err(e) => match e.kind() {
                StateError::OutOfRange => todo!(),
                StateError::HistoryValueOutOfRange => todo!(),
                StateError::CriticalValueMissing => todo!(),
            },
        };

        // Let's say that you want to only handle `HistoryValueOutOfRange`.
        //
        // On Result<T,E>, you can simply write only the error you want to handle
        //
        use StateError as SE; // Just to use a shorter namespace
        let _current = match robot.prev_q_at(index, history_depth) {
            Ok(q) => q,
            Err(e) if e.kind() == SE::HistoryValueOutOfRange => {
                robot.q_at(index).expect("Error: q_at also failed !")
            } // Try to get the current instead of the previous Q or panic with a custom message.
            Err(_) => return, // early return nothing. The rest of you controller doesn't get
                              // executed
        };

        // robot.set_all_q(desired);
    }

    // If your wish to track your own data, flags or else, you would use
    // `add_controller_with`. The difference with `add_controller` is the
    // ability to receive T (which is whatever you want).
    //
    // This is more explicit with the signature.
    // `pub fn add_controller_with<S, T, F>(self, schedule: S, ctx: T, f: F) -> Self`
    fn context_controller(
        tracker: &mut UserDefinedDataTracker,
        _time: &Time,
        robot: &mut State,
        _sensors: &Sensors,
    ) {
        // here, we simply read a flag the user defined and set all positions
        // if true.
        if *tracker.read_flag() {
            let current = robot.q().unwrap();
            let desired = current.map(|q| q + 100.0.into());
            robot.set_all_q(desired);
        }
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
    //
    // fn pause(robot: RobotState) {
    //     a.map(|x| Q(0_f32))
    // }

    // NOTE: no `Copy` here. `Copy` would hand each controller its OWN duplicate
    // of the tracker, so a write in one thread would never be seen by another.
    // We want ONE shared tracker — see how it's wrapped in `Arc<Mutex<..>>` below.
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

    // Shared tracker. `Arc` lets several threads OWN the same value (cheap
    // reference-counted handle), and `Mutex` lets them mutate it safely one at
    // a time. Each controller will get its own `Arc::clone(&tracker)` — same
    // data underneath, just another handle to it.
    let tracker = Arc::new(Mutex::new(UserDefinedDataTracker::default()));

    // Fundamentally, a payload is just a thing the user added.
    // These comes in 3 types:
    // Light  — inert attachment, no hardware interface or network connection
    // Medium — connects to a hardware interface and reads from the robot's state SoA
    // Heavy  — registers services that integrate with the broader robot system (tablet UI, other payloads, etc.)
    //
    // (payload reference: https://dev.bostondynamics.com/docs/payload/readme)
    //
    // So, this means the architecture doesn't really need to know about them.
    // You can freely create them, but keep in mind that new sensors or new actuators
    // can't be moved by the architecture. This is your responsability to create
    // the integration layer (unless you want us to do it for you).
    //
    // Because it's new controller added via [`add_controller()`] creates
    // automatically a new thread, you are free to use this method however you like.
    // In that case, we will show a pseudo-implementation of a 3 DOF arm as a
    // payload.
    //
    // WARN: this must be only used as initialization
    //
    // A plain `fn` now — no closure, no returned `impl FnMut`, no `move`. The
    // shared tracker arrives as `&mut T` because we register it through
    // `add_controller_with`, which OWNS the `Arc<Mutex<..>>` for us and lends it
    // back each tick. (This kills the old `use_payload(Arc::clone(..))` pattern
    // the todo asked to remove.)
    fn use_payload(
        tracker: &mut ArcMutTracker,
        _time: &Time,
        _robot: &mut State,
        _sensors: &Sensors,
    ) {
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

        let my_arm = CameraArm::default();
        my_arm.joints.get(1).unwrap().move_joint(90); // 90 Degrees

        let feed = my_arm.camera.read_feed();

        // `lock()` gives exclusive access until the guard is dropped (end of
        // this line). `unwrap()` here panics only if another thread panicked
        // while holding the lock — fine for an example.
        tracker.lock().unwrap().save_camera_feed(feed);
    }

    type ArcMutTracker = Arc<Mutex<UserDefinedDataTracker>>;

    // Whatever a threaded controller wants to track. Here: latest gamepad input.
    #[derive(Default)]
    struct GamepadInput {
        x: f32,
        y: f32,
        buttons: u8,
    }

    // The pad is written on the gamepad THREAD and read on the main loop by
    // `use_gamepad`. That's two threads touching one value — the ONE situation
    // where a lock is genuinely required. So the shared pad is an
    // `Arc<Mutex<GamepadInput>>`. (Unlike the fake "share a flag between two
    // main-loop controllers" case, this sharing is real, so the lock is earned.)
    type SharedPad = Arc<Mutex<GamepadInput>>;

    // A controller that belongs on its OWN thread: a gamepad reader. It runs its
    // own blocking poll loop at its own rate — you don't want that stalling the
    // tick loop, and it must NOT touch robot state (that stays single-writer on
    // the main loop, so no actuator-write races are possible).
    //
    // A threaded controller gets ONLY `&mut T`. Here `T = SharedPad`: it locks,
    // writes the latest reading, unlocks. The main loop reads it via `use_gamepad`.
    fn gamepad_controller(pad: &mut SharedPad, _time: &Time) {
        // let ev = read_gamepad();  // your blocking poll
        let mut input = pad.lock().unwrap();
        input.x = 0.0;
        input.y = 0.0;
        input.buttons = 0;
    }

    // The other half: a MAIN-LOOP controller that uses the gamepad to command
    // the robot. It needs `&mut State`, so it can't be threaded — and that's
    // fine: it reaches the pad through the same `SharedPad`. Lock, copy out what
    // you need, unlock FAST (don't hold the lock while doing heavy work — that
    // would stall the gamepad thread).
    fn use_gamepad(pad: &mut SharedPad, _time: &Time, robot: &mut State, _sensors: &Sensors) {
        let (x, _y) = {
            let input = pad.lock().unwrap();
            (input.x, input.y)
        }; // lock released here

        // Turn the stick into a robot command.
        let current = robot.q();
        let desired = current.unwrap().map(|q| q + Q(x));
        robot.set_all_q(desired);
    }

    // This function showcases how you would read the sensors pre-installed
    // on the robot. In this example, we assume 2 sensors:
    //
    // 1) A humidity sensor
    // 2) A camera sensor
    fn estop(_time: &Time, _robot: &mut State, _sensors: &Sensors) {
        kmr_api::emergency_stop!("battery too low");
    }

    fn use_sensors(_time: &Time, _robot: &mut State, _sensors: &Sensors) {
        // let humidity: f32 = sensors.humidity();
        // let imu = sensors.imu();
    }

    // WARN: Think of what happens if an actuator needs to shutdown mid-run for
    // an effect. Also think of read-only actuators (off but we just read its
    // state like in driver-driven displays)

    // WARN: Ensure that the controller can be compiled for another device
    // without having to manually check if the hardware components are
    // present at build time. In the v2, this prevented multiple time
    // to compile a given robot on aarch64 without having to plug the robot
    // on the x86 architecture before.
    // This prevents remote flashing and just convinent compilation workflow.
    // It'll better to check presence at runtime instead (at init phase).

    // The own-data controller's tracker. Built and initialized ONCE, here —
    // not on every tick. We hand ownership to the API below; from then on it
    // lends this exact value back to `context_controller` as `&mut`.
    let mut own_tracker = UserDefinedDataTracker::default();
    own_tracker.write_flag(true); // your one-time init

    // The shared gamepad. Built ONCE here. `Arc::clone` gives a cheap second
    // handle to the SAME pad — one goes to the gamepad thread (writer), the
    // other to `use_gamepad` on the main loop (reader).
    let pad: SharedPad = Arc::new(Mutex::new(GamepadInput::default()));

    Robot::new()
        // Settings
        .dt_ms(1) // NOTE: or .dt_us(10000)
        // .history_depth(1_usize) // with a maximum recommended of 5.
        // Controllers
        .add_controller(Init, initialization)
        .add_controller(EachTick, replace_all_q)
        .add_controller(EachTick, replace_all_q) // just to show you can call the same controller
        // multiple times if you wish so.
        .add_controller(EachTick, replace_one_q)
        .add_controller(EachTick, use_time)
        .add_controller(EachTick, use_groups)
        .add_controller(EachTick, previous_state)
        .add_controller(EachTick, previous_state_at)
        .add_controller(EachTick, error_handling)
        .add_controller_with(EachTick, own_tracker, context_controller)
        // `Arc::clone` makes another handle to the SAME tracker (cheap — just
        // bumps a counter). `add_controller_with` owns this handle and lends it
        // to `use_payload` as `&mut` — no closure needed anymore.
        .add_controller_with(Init, Arc::clone(&tracker), use_payload)
        // .add_controller(OnCollision, (previous_state_at))
        // .add_controller(OnCollision, (pause, previous_state, stop))
        // Own-data controller: no closure, no Arc/Mutex. We move the already
        // initialized `own_tracker` in — the API keeps it alive across ticks.
        // Gamepad WRITER on its own thread. No robot state → can't race commands.
        .add_controller_as_thread(EachTick, Arc::clone(&pad), gamepad_controller)
        // Gamepad READER on the main loop: needs State, so not threaded. Same
        // pad, other handle. This is the bridge from thread back to the robot.
        .add_controller_with(EachTick, Arc::clone(&pad), use_gamepad)
        .add_controller(EachTick, use_sensors)
        .add_controller(EachTick, estop)
        .run()?;
    Ok(())
}
