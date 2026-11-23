// Units, and naming your joints.
//
// Inside kmr, every value has ONE unit: a `Q` is rad on a rotating joint and
// mm on a sliding one, a `Qd` is rad/s or mm/s. No guessing, no "was that
// degrees?" bug at 2 am.
//
// But you don't have to think in rad. Write the unit you like (`Deg`, `Mm`,
// `Cm`, `M`, ...) and `.into()` converts it for you.
//
// And instead of remembering that "joint2" is index 1, ask the model.
#![allow(dead_code)]
#![allow(unused_variables)]
use kmr_api::{
    Bus, Cm, CmPerS, Deg, DegPerS, EachTick, M, MPerS, Mm, MmPerS, Q, Qd, Sensors, Time,
};
use std::time::Duration;

kmr_api::robot!(
    XmBench,
    hardware = |hw| hw.baud("bus0", 1_000_000).read_every("bus0", 4)
);

const DEMO_LENGTH: Duration = Duration::from_millis(50);

// The clock helper from 01: it stops the demo after `DEMO_LENGTH`.
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

fn use_units(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    // `position()` is just another name for `q()`.
    if robot.position().is_none() {
        return;
    }
    // Angles become rad, lengths become mm. One catch: a unit does not know
    // what kind of joint it goes to. `Deg` on a sliding joint compiles, and
    // is wrong. Match the unit to the joint.
    let from_degrees: Q = Deg(90.0).into();
    let _from_mm: Q = Mm(15.0).into();
    let _from_cm: Q = Cm(1.5).into();
    let _from_m: Q = M(0.015).into();
    let _from_deg_per_s: Qd = DegPerS(30.0).into();
    let _from_mm_per_s: Qd = MmPerS(10.0).into();
    let _from_cm_per_s: Qd = CmPerS(1.0).into();
    let _from_m_per_s: Qd = MPerS(0.01).into();

    // `robot!` read the model at compile time, so it knows every joint by
    // name. `actuator!` turns a name into its index, and a typo in the name
    // is a compile error, not the wrong joint moving.
    //
    // The `model` module holds the rest: names, and the joint limits, handy
    // to check a target before you write it.
    let joint2: usize = actuator!("joint2");
    let _name: &str = model::NAMES[joint2];
    let _lower: Q = model::LOWER[joint2];
    let _upper: Q = model::UPPER[joint2];

    robot.set_q_at(from_degrees, joint2).unwrap();

    // if you know the index of the joint, you can also use it directly, but
    // depending on your model, we wouldn't always recommend it.
    //
    // See why: on the XmBench, "joint2" is index 1 (indices count from 0, in
    // model order), while its Dynamixel id is 2. Easy to mix up.

    // robot.set_q_at(from_degrees, 1).unwrap();
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
        .add_controller(EachTick, use_units)
        .add_controller(EachTick, use_time)
        .run()?;
    Ok(())
}
