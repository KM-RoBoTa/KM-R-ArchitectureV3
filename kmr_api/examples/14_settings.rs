// Tutorial 14: the robot's settings, your safety net.
//
// Everything here is set once, in `main`, before the robot runs. None of it
// makes the robot do anything; it decides what kmr does for you when things
// go wrong: a sensor goes quiet, a command goes too far, the run ends. Set
// it up front, and your controllers can stay simple.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{Bus, EachTick, Envelope, Policy, Q, Qd, SensorPolicy, Sensors, Tau, Time};
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

const R_ARM: [usize; 3] = [1, 2, 3];

fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;

    let robot = Robot::new();
    let robot = if cfg!(feature = "dynamixel") {
        robot.add_plugins(Bus::by_id("FT8ISZ6H").baud(1_000_000).bus("bus0"))
    } else {
        robot
    };
    robot
        // One tick every 1 ms: the loop runs at 1 kHz.
        .set_dt(Duration::from_millis(1u64))
        // When a shutdown comes, keep the actuators holding for 500 ms before
        // cutting their torque. Without it, torque goes off at once, and an
        // arm or a leg drops the moment you ask the robot to stop. An
        // emergency stop does not wait for this.
        .set_hold(Duration::from_millis(500))
        // A sensor that goes quiet always raises `sensor_missing` in the
        // faults. The policy says what happens on top: `Flag` (the default,
        // nothing more, your controllers decide), `Warn` (log it once), or
        // `Abort` (emergency stop in that very tick).
        .set_sensor_policy(SensorPolicy::Warn)
        // Each actuator model has limits it must not go past. The limit
        // policy says what happens when a command does: `Clamp` cuts it back
        // to the limit and sends it, `Warn` sends it as written and logs the
        // first breach, `Abort` sends nothing that tick and stops the robot.
        //
        // kmr won't guess for you: an actuator with a limit but no policy
        // makes `run()` refuse to start. When several calls name the same
        // actuator, the last one wins, so here actuator 0 aborts, the right
        // arm warns, and every other actuator clamps.
        .set_limit_policy_all(Policy::Clamp)
        .set_limit_policy_group(&R_ARM, Policy::Warn)
        .set_limit_policy(0, Policy::Abort)
        // An envelope is your OWN, tighter bounds, inside the model's limits:
        // say, the range where this joint can't hit the table. A command
        // outside is clamped to it and the `envelope` fault is set, so you
        // can see it happened. A field you leave `None` is not bounded.
        .set_envelope(
            0,
            Envelope {
                q: Some((Q(-3.0), Q(3.0))),
                ..Envelope::NONE
            },
        )
        // The same, for every actuator of the right arm at once.
        .set_envelope_group(
            &R_ARM,
            Envelope {
                qd: Some((Qd(-2.0), Qd(2.0))),
                tau: Some((Tau(-1.0), Tau(1.0))),
                ..Envelope::NONE
            },
        )
        // .history_depth(1_usize) // with a maximum recommended of 5.
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
