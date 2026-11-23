// Tutorial 07: when kmr can't give you what you asked for.
//
// Up to now we used `unwrap()` and moved on. On a real robot that is a bad
// habit: one surprise and the whole program stops. kmr answers in two ways
// when it can't give you a value:
//
// 1) An `Option`: there is simply nothing there yet.
// 2) A `Result`: something went wrong, and the error tells you what.
//
// Here we see how to read both, and how to react calmly: skip a tick, use
// another value, or stop the robot gracefully.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, EachTick, Q, Sensors, StateErrorKind as StateError, Time};
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

fn error_handling(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    // A run without hardware reads nothing: no sample to work from, skip the tick.
    if robot.q().is_none() {
        return;
    }
    let current: Option<[Q; N]> = robot.q();
    // `robot.q()` returns an Option<[Q;N]>.
    // This is because there may be no sample yet: before the first read, or
    // in a run without hardware. (A lost packet does not give `None`: you get
    // the last good value, see 08.) Instead of returning silently a defaulted
    // array, we chose to explicitly let you decide what you would like to do.

    // In rust, you can handle this situation with a bunch of options.
    // Here, we will list some, with short explanations. Note that this is
    // not meant as a rust guide, but rather a guide for good practices.
    //
    // Also note that here, we don't return an error, but an option.
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
    // panicking here means: not handling the danger yourself. The loop stops
    // on the spot; kmr still turns the torque off on the way out, but a robot
    // that goes limp in the middle of a move can fall.
    //
    // (It can't panic here only because we returned early on `None` above.)
    let _error_handled_current = current.unwrap();

    // Pattern matching
    //
    // This one is explicit and doesn't require too much imperative
    // (see: functional programming) rust knowledge.
    //
    // This consists of declaring explicitly what we want to do in each Option<>
    // cases. Option holds either `Some(x)` or `None`.

    let _error_handled_current = match current {
        Some(q_array) => q_array, // on success, we just retrieve the array

        // We could retry on error.. but the state didn't change since then
        // because the I/O is called only when all controllers finished
        // their tasks.
        //None => robot.q().unwrap(), // And we need to error handle again ...

        // Another option would be to stop the program entirely with a graceful
        // shutdown call. Homing would not help here: the home preset ramps from
        // the first sample, so with no sample it waits too.
        None => {
            // Stop gracefully, and leave this controller for this tick.
            // `shutdown!` only raises the flag, it doesn't leave your
            // function: the loop finishes this tick, then stops. Hence the
            // `return`.
            kmr_api::shutdown!("no sample to control from");
            return;
        }
    };

    // Result
    let index: usize = 1; // ok
    let history_depth: usize = 999; // absurd
    // Some errors might come from logic errors. Because it's less evident to
    // know what happened, an `Option<>` wouldn't be useful here. `None` doesn't
    // tell you if the error was because of the `index` or the `history_depth`.
    let _current = match robot.prev_q_at(index, history_depth) {
        Ok(q) => q,

        // This is much more useful isn't it ?
        // It's a lot of work, but you handle everything gracefully.
        //
        // We recommend learning about Options and Results more in depth as
        // this will alleviate the pain of error handling by learning core
        // principles such as ignoring errors, mapping errors, returning it with
        // the `?` operator to handle it elsewhere and much more.
        // `e.kind()` is the bare, payload-free cause — match it without
        // `{ .. }`. The numbers are still in `e` (its `Display`) if you want them.
        Err(e) => match e.kind() {
            // A wrong index is a bug in this controller: stop the robot.
            StateError::OutOfRange => {
                kmr_api::shutdown!("index out of range");
                return;
            }
            // Not that far back yet: use the newest value instead.
            StateError::HistoryValueOutOfRange => match robot.q_at(index) {
                Ok(q) => q,
                Err(_) => return,
            },
            // Nothing to control from this tick: skip it.
            StateError::CriticalValueMissing | StateError::NoSample => return,
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
        Err(_) => return, // early return nothing. The rest of your controller doesn't get
                          // executed
    };

    // robot.set_all_q(desired);
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
        .add_controller(EachTick, error_handling)
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
