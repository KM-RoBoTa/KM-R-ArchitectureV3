use kmr_api::{Initialization, N, Payload, PerTick, Q, Robot, Sensors, State, StateError, Time};
use std::sync::{Arc, Mutex};

// fn stop(robot: RobotState) {
//     a.map(|x| Q(0_f32))
//     robot.command
// }

fn main() {
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
        let index: usize = 0;
        let current_q = robot.q_at(index);

        let desired_q = current_q.unwrap() + 1.0.into();

        robot.set_q_at(desired_q, index);
    }

    // A group is a fixed set of joint indices — non-contiguous allowed, e.g.
    // [1, 3, 4]. Here, the right arm: joints 3, 4, 5.
    const R_ARM: [usize; 3] = [3, 4, 5];
    fn use_groups(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        let mut r_arm = robot.group(R_ARM);

        // Group reads gather into a `[Q; K]` (here K = 3), not the whole-robot
        // `[Q; N]`. `q()` mirrors the whole-robot read: `Option`, where `None`
        // means "no sample yet" (recoverable).
        let current_q = r_arm.q();

        let desired = current_q.unwrap().map(|q| q + Q(100_f32));
        // group index can be out of range, so this returns a `Result` to propagate
        // that cause — it is not swallowed. Joints outside R_ARM are untouched.
        r_arm.set_all_q(desired).unwrap();
    }

    fn initialization(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        // NOTE: home_state returns ANY state. To ensure home_state() knows what
        // to return you, you must specify the type you wish. In that case,
        // this is done by the `let variable: Type` notation.
        let _desired: [Q; N] = robot.initial_state();

        // NOTE: You can also write it with the "turbofish" notation if you desire.
        let desired = robot.initial_state::<Q>();

        &robot.set_all_q(desired);
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
        let error_handled_current = current.unwrap();

        // Pattern matching
        //
        // This one is explicit and doesnt require too much imperative
        // (see: functional programming) rust knowledge.
        //
        // This consist of declaring explicitly what we want to do in each Option<>
        // cases. Option holds either `Some(x)` or `None`.

        let error_handled_current = match current {
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
        let current = match robot.prev_q_at(index, history_depth) {
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
        let current = match robot.prev_q_at(index, history_depth) {
            Ok(q) => q,
            Err(e) if e.kind() == SE::HistoryValueOutOfRange => {
                robot.q_at(index).ok().expect("Error: q_at also failed !")
            } // Try to get the current instead of the previous Q or panic with a custom message.
            Err(e) => return (), // early return nothing. The rest of you controller doesn't get
                                 // executed
        };

        // robot.set_all_q(desired);
    }

    // fn pause(robot: RobotState) {
    //     a.map(|x| Q(0_f32))
    // }

    fn previous_state_at(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        let index: usize = 1;
        let history_depth = 2;
        let current = robot.prev_q_at(index, history_depth).unwrap();
        let desired = current * 3_f32;

        robot.set_q_at(desired, index).unwrap();
    }

    fn previous_state(_time: &Time, robot: &mut State, _sensors: &Sensors) {
        let history_depth = 2;
        let _previous = robot.prev_q(history_depth);
    }

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
    fn use_payload(
        tracker: Arc<Mutex<UserDefinedDataTracker>>,
    ) -> impl FnMut(&Time, &mut State, &Sensors) {
        move |_time: &Time, _robot: &mut State, _sensors: &Sensors| {
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
    }

    // Simple case of a controller that tracks the array of Q itself.
    // This is a simple showcase of how you would track data or use your own
    // structures.
    fn tracking_controller(
        tracker: Arc<Mutex<UserDefinedDataTracker>>,
    ) -> impl FnMut(&Time, &mut State, &Sensors) {
        move |_time: &Time, robot: &mut State, _sensors: &Sensors| {
            tracker.lock().unwrap().save_q(&robot.q().unwrap());

            let desired = robot.q().unwrap().map(|q| q + 100.0.into());
            robot.set_all_q(desired);
        }
    }

    // The next two functions showcase how you can safely share data between
    // threads.
    //
    // This basic example still uses [`UserDefinedDataTracker`].
    // We just want to read a boolean flag and set it to true, then false.
    // Note that threads are not guaranteed to run in this defined order.
    //
    // We encourage reasing chapter 16 of the rust book to better understand
    // rust concurrency.
    //
    // https://doc.rust-lang.org/book/ch16-00-concurrency.html
    fn read_write_shared_flag_1(
        tracker: Arc<Mutex<UserDefinedDataTracker>>,
    ) -> impl FnMut(&Time, &mut State, &Sensors) {
        move |_time: &Time, _robot: &mut State, _sensors: &Sensors| {
            let mut mutex = tracker.lock().unwrap();

            mutex.read_flag();

            mutex.write_flag(true);
        }
    }

    fn read_write_shared_flag_2(
        tracker: Arc<Mutex<UserDefinedDataTracker>>,
    ) -> impl FnMut(&Time, &mut State, &Sensors) {
        move |_time: &Time, _robot: &mut State, _sensors: &Sensors| {
            let mut mutex = tracker.lock().unwrap();

            mutex.read_flag();

            mutex.write_flag(false);
        }
    }

    // Quick note on closures.
    //
    // We realize that
    // fn name(/*args*/) -> impl FnMut(RobotState) {}
    // as a function is hard to read.
    //
    // Here's an begginer friendly alternative if you'd prefer.
    // Both are completely valid and do exactly the same.
    type ArcMutTracker = Arc<Mutex<UserDefinedDataTracker>>;

    fn named_closure(tracker: ArcMutTracker) -> impl FnMut(&Time, &mut State, &Sensors) {
        // Clean signature, no `move |..|` noise.
        fn logic(tracker: &ArcMutTracker) {
            let mut guard = tracker.lock().unwrap();
            guard.read_flag();
            guard.write_flag(false);
        }

        // But its still fundamentally unavoidable because you need to
        // somehow use more parameters than we actually give you permission.
        //
        // Looking at the required return type `impl FnMut(RobotState)`,
        // you are required to return a function that takes one parameter.
        //
        // This notation, called closure, actually allows you to respect that
        // by creating a function which takes ONE parameter `|robot: RobotState|`
        // but has a nammed function that can CAPTURE "tracker" from the
        // current scope.
        //
        // Why is it mandatory ?
        //
        // Two reasons.
        //
        // 1) Because we can't plan for "tracker". This is your code !
        // We just cannot know in advance what you will write.
        //
        // We made the choice to let you do whatever you want, but you must
        // respect rust's rules in return.
        //
        // 2) We want to be sure you proprely take ownership of [`RobotState`]
        // so that you actually use it ! It's the do-all structure, your main
        // tool. Making it optional would require even more difficult code
        // just to use this fundamental data structure.
        move |_time: &Time, _robot: &mut State, _sensors: &Sensors| logic(&tracker)
    }

    // This function showcases how you would read the sensors pre-installed
    // on the robot. In this example, we assume 2 sensors:
    //
    // 1) A humidity sensor
    // 2) A camera sensor
    fn use_sensors(_time: &Time, _robot: &mut State, sensors: &Sensors) {
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

    Robot::new()
        // Settings
        .dt_ms(1) // NOTE: or .dt_us(10000)
        // Controllers
        .add_controller(Initialization, initialization)
        .add_controller(PerTick, replace_all_q)
        .add_controller(PerTick, replace_all_q)
        // `Arc::clone` makes another handle to the SAME tracker (cheap — just
        // bumps a counter). Both controllers below now read/write one tracker.
        .add_controller(PerTick, tracking_controller(Arc::clone(&tracker)))
        .add_controller(PerTick, use_groups)
        .add_controller(Initialization, use_payload(Arc::clone(&tracker)))
        // .add_controller(OnCollision, (previous_state_at))
        // .add_controller(OnCollision, (pause, previous_state, stop))
        .add_controller(PerTick, error_handling)
        .add_controller(PerTick, read_write_shared_flag_1(Arc::clone(&tracker)))
        .add_controller(PerTick, read_write_shared_flag_2(Arc::clone(&tracker)))
        .add_controller(PerTick, named_closure(Arc::clone(&tracker)))
        .add_controller(PerTick, use_sensors)
        .run();
}
