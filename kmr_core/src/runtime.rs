//! Module that holds the main runtime logic, also called "control loop".
//!
//! This module is responsable of properly initializing the program, then
//! calling on the user's behalf his controllers while manipulating
//! [`crate::clock::Time`].
//!
//! The runtime is called by [`crate::robot::Robot::run`].
//! It is organized by phases, specified by the schedule labels in
//! [`crate::schedule`].

use tracing::trace;

use crate::controllers::Env;
use crate::schedule::Drive;
use crate::state::RobotState;
use crate::{JOINTS, Robot, Sensors, shutdown};

/// The initialization phase, organized in 3 "sub-phases".
///
/// It's main goals are to create and handoff the the main runtime most of the
/// runtime initialized structs such as [`crate::clock::Time`], initialize
/// the hardware and run some healthchecks.
mod init_phase {
    use std::env;

    use tracing::trace;

    use crate::clock::Time;
    use crate::controllers::Env;
    use crate::schedule::Drive;
    use crate::{JOINTS, Robot, State};
    use crate::{RobotState, Sensors};

    /// Installs the tracing subscriber from the [`tracing`] crate.
    ///
    /// It installs the tracing subscriber with the INFO level in release mode,.
    /// DEBUG level in debug mode and TRACE level only and only if the RUST_LOG
    /// environment variable is set.
    ///
    /// # WIP
    /// - [ ] Disable it completely in no_std.
    fn install_tracing() {
        use tracing::Level;
        use tracing_subscriber::FmtSubscriber;

        let level = if env::var("RUST_LOG").is_ok() {
            Level::TRACE
        } else if cfg!(debug_assertions) {
            Level::DEBUG
        } else {
            Level::INFO
        };

        // try_init succeeds only if no subscriber is set yet.
        // If the user installed their own before calling Robot::start(), this is a no-op.
        let _ = FmtSubscriber::builder().with_max_level(level).try_init();
        trace!("Tracing level set");
    }

    /// Pre-initialization phase.
    ///
    /// Used for the initialization of processes like subscribers, and other
    /// optional/mandatory systems such as [`Time`].
    ///
    /// Returns owned values for [`Time`], [`RobotState`] and [`Sensors`].
    pub(super) fn pre_init() -> (Time, RobotState<JOINTS>, Sensors) {
        install_tracing();

        trace!("Executing pre-init phase ...");
        // TODO: feature gated:
        // Color_eyre

        // BUG: time's already constructed when Robot::new() is called.
        // State is decoupled for the robot for an unknown reason and
        // sensor being part of Env is weird. Something went wrong in the
        // design
        let time = Time::default();
        let state = RobotState::<JOINTS>::default();
        let sensors = Sensors {};

        (time, state, sensors)
    }

    /// Normal Initialization phase.
    ///
    /// Phase in which the hardware components are initialized, registered,
    /// preprocessed, etc.
    pub(super) fn init<CS: Drive>(robot: &mut Robot<CS>, env: &mut Env<'_>) {
        trace!("Executing init phase ...");
        // TODO:
        // - Init actuators (zero position if needed for some brands of actuators)
        // - Init sensors
        // - Init payloads
        // - Init battery (if available)

        trace!("Executing user init controllers ...");
        robot.controllers.init(env);
    }

    /// Post processing of initialized components.
    ///
    /// Phase in which the architecture further prepares initialized components.
    /// It's most critical tasks are to
    /// - health check
    /// - ensure proper defaults
    /// - read the current position of the actuators and signal the user if
    ///   one or more are misplaced
    /// - Double check the zero position of some brands of actuators
    /// - Enable/Disable the power state of each hardware components in regards
    ///   to default values or values specified by the user.
    pub(super) fn post_init<CS: Drive>(robot: &mut Robot<CS>, env: &mut Env<'_>) {
        trace!("Executing post-init phase ...");
        // TODO:
        // - Ping + health check for
        //      1) actuators
        //      2) sensors
        //      3) payloads
        //      4) battery
        // - Register the actuators modes
        // - Register sensors modes
        // - On/Off power states of hardware components.
        // - etc

        trace!("Executing post-init controllers ...");
        robot.controllers.post_init(env);
    }
}

/// The control phase, organized in 3 "sub-phases".
///
/// The control phases are called each tick. Its main goal is to call the user
/// defined controllers by their respective phases (as defined in
/// [`crate::schedule`]).
mod ctrl_phase {
    use std::time::{Duration, Instant};

    use tracing::trace;

    use crate::Robot;
    use crate::controllers::Env;
    use crate::schedule::Drive;

    /// User specified actions to undertake before calling the user defined
    /// controllers.
    pub(super) fn first<CS: Drive>(robot: &mut Robot<CS>, env: &mut Env<'_>) {
        trace!("Executing first tick controllers ...");
        // TODO: compute next tick with dt
        // engine per-tick pre-work (WIP)

        // User-registered `First` controllers.
        robot.controllers.first(env);
    }

    /// The actual run of all the user defined controllers.
    /// todo: what about the threaded ones ?
    /// todo: what about different tick rates for some ?
    /// this is a naive implementation.
    pub(super) fn each_tick<CS: Drive>(robot: &mut Robot<CS>, env: &mut Env<'_>) {
        trace!("Executing main tick controllers ...");
        robot.controllers.each_tick(env);
    }

    /// Actions to undertake after calling the user defined
    /// controllers.
    ///
    /// The main goal is to handles the [`Time`] state.
    /// It'll
    /// - Set the elapsed time
    /// - compute the overrun and accumulated overrun
    /// - set the last update
    /// - Increment the time
    /// - sleep if no overrun
    pub(super) fn last<CS: Drive>(robot: &mut Robot<CS>, env: &mut Env<'_>) {
        trace!("Executing last tick controllers ...");
        // User-registered `Last` controllers.
        robot.controllers.last(env);

        robot.config.time.set_elapsed();

        // TODO: clock work — elapsed/overrun/accumulated-overrun, then sleep.
        // temporary sleep so clippy shut-up
        std::thread::sleep(Duration::from_millis(30));
    }
}

/// The entry point of the runtime of the core.
///
/// The runtime is separated in multiple phases, each owning its engine work and
/// then walking its own controller bucket via `Drive`.
pub(crate) fn runtime<CS: Drive>(mut robot: Robot<CS>) {
    // TODO: Actuators impl
    let _ = ctrlc::set_handler(|| {
        // trace!("Spawn Ctrl-C handler");
        shutdown!("Ctrl-C received");
    });

    let (time, mut state, sensors) = init_phase::pre_init();
    let mut env = Env::new(&time, &mut state, &sensors);
    trace!("Executing user pre-init controllers ...");
    robot.controllers.pre_init(&mut env);

    init_phase::init(&mut robot, &mut env);
    init_phase::post_init(&mut robot, &mut env);

    // todo: machinery to spin up the dedicated threads for the threaded
    // controllers

    loop {
        ctrl_phase::first(&mut robot, &mut env);
        ctrl_phase::each_tick(&mut robot, &mut env);
        ctrl_phase::last(&mut robot, &mut env);
        // note: temporary forced exit for dev
        if true {
            break;
        }
    }
}
