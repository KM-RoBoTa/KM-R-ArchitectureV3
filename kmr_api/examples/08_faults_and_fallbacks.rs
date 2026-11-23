#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{
    Bus, EachTick, First, Mask, OrWhen, Q, Sensors, Time, any_of, remote_timeout, stale_in,
};
use std::{num::NonZeroU32, time::Duration};

kmr_api::robot!(
    XmBench,
    hardware = |hw| hw.baud("bus0", 1_000_000).read_every("bus0", 4)
);

const DEMO_LENGTH: Duration = Duration::from_millis(50);

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
fn use_groups(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    if robot.q().is_none() {
        return;
    }
    let mut r_arm = robot.group(R_ARM);

    let current_q = r_arm.q();

    let desired = current_q.unwrap().map(|q| q + Q(100_f32));
    r_arm.set_all_q(desired).unwrap();
}

fn replace_one_q(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    if robot.q().is_none() {
        return;
    }
    let index: usize = 0;
    let current_q = robot.q_at(index);

    let desired_q = current_q.unwrap() + 1.0.into();

    robot.set_q_at(desired_q, index).unwrap();
}

// Real hardware is messy. A cable wiggles, a motor browns out, a packet gets
// lost on the bus. When that happens, kmr does NOT stop your controller: it
// keeps ticking and hands you the best data it has, which may be the OLD
// value from the last tick that worked.
//
// That is a quiet trap. Your controller reads `q`, sees a number, and happily
// computes a command from it, never knowing the number is 50 ms old. On a
// walking robot, that is how you end up on the floor.
//
// Faults are how kmr tells you "this tick, something went wrong, and here is
// exactly what". They are read-only: you can look at them, you can't hide them.
fn use_faults(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    // No faults before the first tick ran: nothing to report yet.
    let Some(faults) = robot.faults() else {
        return;
    };
    // The happy path. Most ticks, nothing is wrong and you stop here.
    if faults.is_clear() {
        return;
    }

    // Something went wrong. Each of these is a `Mask`: one bit per actuator,
    // bit `i` set means "actuator `i` had this problem this tick".
    //
    // Problems on the way IN (reading the actuators):
    let stale: Mask<N> = faults.stale(); // the read failed, you got last tick's value
    let bus_error: Mask<N> = faults.bus_error(); // the bus itself had an error
    let offline: Mask<N> = faults.offline(); // the actuator didn't answer at all
    //
    // Problems on the way OUT (your commands):
    let _rejected: Mask<N> = faults.rejected(); // your command was never sent (torque off, wrong mode, out of range)
    let _joint_limit: Mask<N> = faults.joint_limit(); // your command went past the joint's limit
    let _envelope: Mask<N> = faults.envelope(); // your command left its safe envelope and was clamped
    //
    // And the lazy one: "which actuators had ANY of the above?"
    let any: Mask<N> = faults.any_actuator();

    // Some problems are not about one actuator but about the whole tick,
    // so they are a plain yes/no.
    let _overrun: bool = faults.overrun(); // the tick finished too late
    let _sensor_missing: bool = faults.sensor_missing(); // a sensor went quiet
    let _remote_timeout: bool = faults.remote_timeout(); // a remote command source went quiet
    let _unauthorized: bool = faults.unauthorized(); // someone you don't trust tried to send commands

    // A `Mask` behaves like a small set of actuator indices.
    let _first_is_stale: bool = stale.get(0);
    let _none_offline: bool = offline.is_empty();
    let _faulted: usize = any.iter().count();
    let _lost: Mask<N> = stale.union(offline).intersection(Mask::ALL);
    let _only_bus: Mask<N> = bus_error.difference(stale);

    // Faults are kept in History, like positions. Comparing with the tick
    // before tells you when a problem STARTED, handy to log it once instead
    // of a thousand times per second.
    if let Ok(previous) = robot.prev_faults(1) {
        let _was_clear: bool = previous.is_clear();
    }

    // Shortcuts for the most common question: "how fresh is this reading?"
    let _stale_at: bool = robot.stale_at(0);
    let _age_at: Option<u32> = robot.age_at(0).unwrap_or(None);
    let _newest_age: Option<u32> = robot.sample_age();
    let _read_every: Option<NonZeroU32> = robot.read_every(0).ok();
}

// A fallback: the boring, safe thing to do when you can't trust your data.
// Here we just ask every actuator to stay where it was last tick. Boring is
// good: boring doesn't fall over.
fn hold_still(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    if let Ok(previous) = robot.prev_q(1) {
        robot.set_all_q(previous);
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
        .add_controller(First, use_faults)
        // Checking faults by hand in every controller gets old fast. `or_when`
        // does it for you: each tick, run `use_groups` normally, but if the
        // right arm's reading is stale, run `hold_still` instead. Exactly one
        // of the two runs, never both.
        //
        // Your controller stays simple and only deals with good data; the
        // "what if the cable falls off" plan lives right next to it.
        .add_controller(EachTick, use_groups.or_when(stale_in(&R_ARM), hold_still))
        // Conditions combine: hold still if the arm is stale OR the remote
        // operator stopped talking to us.
        .add_controller(
            EachTick,
            replace_one_q.or_when(any_of(stale_in(&R_ARM), remote_timeout()), hold_still),
        )
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
