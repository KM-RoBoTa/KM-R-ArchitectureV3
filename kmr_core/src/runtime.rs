//! Module that holds the main runtime logic, also called "control loop".
//!
//! This module is responsable of properly initializing the program, then
//! calling on the user's behalf his controllers while manipulating
//! [`crate::clock::Time`].
//!
//! The runtime is called by [`crate::robot::Robot::run`].
//! It is organized by phases, specified by the schedule labels in
//! [`crate::schedule`].
//!
//! One clock, `robot.config.time`: the engine mutates it between the phases,
//! the controllers only get `&Time` through each phase's short-lived [`Env`].
//!
//! The loop ends only on a signal (see [`crate::signal`]). A shutdown
//! completes its tick. An emergency stop exits at the next bus check, between
//! two phase buckets.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use tracing::{debug, error, info, trace, warn};

use crate::controllers::Env;
use crate::schedule::Drive;
use crate::state::RobotState;
use crate::{Robot, Sensors, Signal, SignalKind, shutdown};

/// Tick duration used when `Robot::set_dt` was never called, or with zero.
const FALLBACK_DT: Duration = Duration::from_millis(1);

/// The instant, the sleep and the signal bus: a seam so the loop tests run on
/// scripted values instead of the wall clock and the global bus.
trait Host {
    fn now(&mut self) -> Instant;
    fn sleep_until(&mut self, deadline: Instant);
    fn poll(&mut self) -> Option<Signal>;
}

/// The real world: system clock, thread sleep, global signal bus.
struct RealHost;

impl Host for RealHost {
    fn now(&mut self) -> Instant {
        Instant::now()
    }

    fn sleep_until(&mut self, deadline: Instant) {
        // TODO: jitter under a non real-time scheduler. Spin tail or
        // `clock_nanosleep(TIMER_ABSTIME)`, with the hardware layer.
        std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
    }

    fn poll(&mut self) -> Option<Signal> {
        Signal::drain()
    }
}

/// Set once the Ctrl-C handler of the process is the one of the engine.
static CTRL_C_WIRED: AtomicBool = AtomicBool::new(false);

/// Treats a failed Ctrl-C install as harmless when the handler in place is
/// ours, from an earlier run of the process. Otherwise hands the error back.
///
/// `wired` is a parameter so the tests do not share the process flag.
fn ctrl_c_wired(
    installed: Result<(), ctrlc::Error>,
    wired: &AtomicBool,
) -> Result<(), ctrlc::Error> {
    match installed {
        Ok(()) => {
            // Relaxed: the flag guards no other data.
            wired.store(true, Ordering::Relaxed);
            Ok(())
        }
        Err(_) if wired.load(Ordering::Relaxed) => Ok(()),
        Err(error) => Err(error),
    }
}

/// Keeps the signal only if it is an emergency stop: mid-tick checks let a
/// shutdown finish its tick.
fn estop(signal: Option<Signal>) -> Option<Signal> {
    signal.filter(|s| s.kind() == SignalKind::EmergencyStop)
}

/// The initialization phase, organized in 3 "sub-phases".
///
/// It's main goals are to create and handoff the the main runtime most of the
/// runtime initialized structs, initialize the hardware and run some
/// healthchecks.
mod init_phase {
    use std::env;

    use tracing::trace;

    use crate::controllers::Env;
    use crate::schedule::Drive;
    use crate::{Robot, RobotState, Sensors};

    /// Installs the tracing subscriber from the [`tracing`] crate.
    ///
    /// It installs the tracing subscriber with the INFO level in release mode,.
    /// DEBUG level in debug mode and TRACE level only and only if the RUST_LOG
    /// environment variable is set.
    ///
    /// # WIP
    /// - [ ] Disable it completely in no_std.
    pub(super) fn install_tracing() {
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
    /// Used for the initialization of optional/mandatory systems.
    ///
    /// Returns owned values for [`RobotState`] and [`Sensors`].
    pub(super) fn pre_init() -> (RobotState, Sensors) {
        trace!("Executing pre-init phase ...");
        // TODO: feature gated:
        // Color_eyre

        (<RobotState>::default(), Sensors {})
    }

    /// Walks the user's `PreInit` controllers.
    pub(super) fn pre_init_controllers<CS: Drive>(
        robot: &mut Robot<CS>,
        state: &mut RobotState,
        sensors: &Sensors,
    ) {
        trace!("Executing user pre-init controllers ...");
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.pre_init(&mut env);
    }

    /// Normal Initialization phase.
    ///
    /// Phase in which the hardware components are initialized, registered,
    /// preprocessed, etc.
    pub(super) fn init<CS: Drive>(
        robot: &mut Robot<CS>,
        state: &mut RobotState,
        sensors: &Sensors,
    ) {
        trace!("Executing init phase ...");
        // TODO: no hardware layer yet.
        // - Init actuators (zero position if needed for some brands of actuators)
        // - Init sensors
        // - Init payloads
        // - Init battery (if available)

        trace!("Executing user init controllers ...");
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.init(&mut env);
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
    pub(super) fn post_init<CS: Drive>(
        robot: &mut Robot<CS>,
        state: &mut RobotState,
        sensors: &Sensors,
    ) {
        trace!("Executing post-init phase ...");
        // TODO: no hardware layer yet.
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
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.post_init(&mut env);
    }
}

/// The control phase, organized in 3 "sub-phases".
///
/// The control phases are called each tick. Its main goal is to call the user
/// defined controllers by their respective phases (as defined in
/// [`crate::schedule`]).
mod ctrl_phase {
    use tracing::trace;

    use crate::controllers::Env;
    use crate::schedule::Drive;
    use crate::{Robot, RobotState, Sensors};

    /// User specified actions to undertake before calling the user defined
    /// controllers.
    pub(super) fn first<CS: Drive>(
        robot: &mut Robot<CS>,
        state: &mut RobotState,
        sensors: &Sensors,
    ) {
        trace!("Executing first tick controllers ...");
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.first(&mut env);
    }

    /// The actual run of all the user defined controllers.
    pub(super) fn each_tick<CS: Drive>(
        robot: &mut Robot<CS>,
        state: &mut RobotState,
        sensors: &Sensors,
    ) {
        trace!("Executing main tick controllers ...");
        // TODO: a tick rate per controller (a divider in the node).
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.each_tick(&mut env);
    }

    /// Actions to undertake after calling the user defined controllers.
    pub(super) fn last<CS: Drive>(
        robot: &mut Robot<CS>,
        state: &mut RobotState,
        sensors: &Sensors,
    ) {
        trace!("Executing last tick controllers ...");
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.last(&mut env);
    }
}

/// The tick loop. Returns the signal that ended it.
fn control_loop<CS: Drive, H: Host>(
    robot: &mut Robot<CS>,
    state: &mut RobotState,
    sensors: &Sensors,
    host: &mut H,
) -> Signal {
    loop {
        if let Some(signal) = host.poll() {
            break signal;
        }
        robot.config.time.begin_tick(host.now());

        ctrl_phase::first(robot, state, sensors);
        if let Some(signal) = estop(host.poll()) {
            break signal;
        }

        ctrl_phase::each_tick(robot, state, sensors);
        if let Some(signal) = estop(host.poll()) {
            break signal;
        }

        // TODO: state_transition (write desired, record history), needs the
        // hardware layer.

        ctrl_phase::last(robot, state, sensors);

        let wake = robot.config.time.end_tick(host.now());
        if let Some(overrun) = robot.config.time.overrun() {
            // `debug!`, not `warn!`: a log line per late tick makes the next one late.
            debug!(?overrun, "tick finished after its deadline");
        }

        // Before the sleep: a stop must not wait for it.
        if let Some(signal) = host.poll() {
            break signal;
        }
        if let Some(deadline) = wake {
            host.sleep_until(deadline);
        }
    }
}

/// Startup phases, then the tick loop. Returns the signal that ended the run.
fn run_with<CS: Drive, H: Host>(robot: &mut Robot<CS>, host: &mut H) -> Signal {
    let (mut state, sensors) = init_phase::pre_init();

    if robot.config.time.delta().is_zero() {
        warn!(
            fallback = ?FALLBACK_DT,
            "no tick duration configured (`Robot::set_dt`), using the fallback"
        );
        robot.config.time.set_delta(FALLBACK_DT);
    }

    let signal = 'run: {
        init_phase::pre_init_controllers(robot, &mut state, &sensors);
        if let Some(signal) = estop(host.poll()) {
            break 'run signal;
        }
        init_phase::init(robot, &mut state, &sensors);
        if let Some(signal) = estop(host.poll()) {
            break 'run signal;
        }
        init_phase::post_init(robot, &mut state, &sensors);

        // TODO: spawn the `Threaded` controllers (hand-off open in docs/ROADMAP.md).

        robot.config.time.arm(host.now());

        control_loop(robot, &mut state, &sensors, host)
    };

    // TODO: last write on shutdown, torque cut on emergency stop (hardware layer).
    match signal.kind() {
        SignalKind::Shutdown => info!("control loop stopped, {signal}"),
        SignalKind::EmergencyStop => error!("control loop aborted, {signal}"),
    }
    if let Some(total) = robot.config.time.accumulated_overrun() {
        info!(?total, "ticks finished after their deadline during the run");
    }

    signal
}

/// The entry point of the runtime of the core.
///
/// Returns the signal that ended the run.
pub(crate) fn runtime<CS: Drive>(mut robot: Robot<CS>) -> Signal {
    Signal::reset();

    // Before the Ctrl-C install, which may warn.
    init_phase::install_tracing();

    // After the reset, or a Ctrl-C landing in between would be erased.
    let installed = ctrlc::set_handler(|| {
        shutdown!("Ctrl-C received");
    });
    // A warning, not an error: a user handler may raise `shutdown!` itself.
    if let Err(error) = ctrl_c_wired(installed, &CTRL_C_WIRED) {
        warn!(
            %error,
            "Ctrl-C will not stop the control loop: its handler could not be \
             installed. If the process has its own, it must raise `shutdown!`"
        );
    }

    run_with(&mut robot, &mut RealHost)
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;

    use super::*;
    use crate::clock::Time;
    use crate::signal::SignalMeta;
    use crate::{EachTick, First, Init, Last, PostInit, PreInit};

    const DT: Duration = Duration::from_millis(1);

    // Index of each bus check: two at startup, then four per tick.
    const AFTER_PRE_INIT: usize = 0;
    const TICK_START: usize = 2;
    const AFTER_FIRST: usize = 3;
    const AFTER_EACH_TICK: usize = 4;
    const TICK_END: usize = 5;
    const PER_TICK: usize = 4;

    /// Scripted host: never sleeps, never touches the global bus. A raised
    /// signal stays raised, like on the real bus.
    struct Scripted {
        origin: Instant,
        /// Offset from `origin` returned by each call to `now`, in tenths of
        /// `DT`. Past the end of the script the last value is repeated.
        now_tenths: [u32; 16],
        now_calls: usize,
        sleeps: [Option<Instant>; 16],
        sleep_calls: usize,
        poll_calls: usize,
        /// Check from which the shutdown / the emergency stop is on the bus.
        shutdown_from: Option<usize>,
        estop_from: Option<usize>,
    }

    impl Scripted {
        fn new(origin: Instant) -> Self {
            Self {
                origin,
                now_tenths: [0; 16],
                now_calls: 0,
                sleeps: [None; 16],
                sleep_calls: 0,
                poll_calls: 0,
                shutdown_from: None,
                estop_from: None,
            }
        }

        fn script(mut self, tenths: &[u32]) -> Self {
            let last = tenths.last().copied().unwrap_or(0);
            for (i, slot) in self.now_tenths.iter_mut().enumerate() {
                *slot = tenths.get(i).copied().unwrap_or(last);
            }
            self
        }

        fn shutdown_from(mut self, check: usize) -> Self {
            self.shutdown_from = Some(check);
            self
        }

        fn estop_from(mut self, check: usize) -> Self {
            self.estop_from = Some(check);
            self
        }

        fn at(&self, tenths: u32) -> Instant {
            self.origin + DT * tenths / 10
        }
    }

    impl Host for Scripted {
        fn now(&mut self) -> Instant {
            let i = self.now_calls.min(self.now_tenths.len() - 1);
            self.now_calls += 1;
            self.at(self.now_tenths[i])
        }

        fn sleep_until(&mut self, deadline: Instant) {
            if let Some(slot) = self.sleeps.get_mut(self.sleep_calls) {
                *slot = Some(deadline);
            }
            self.sleep_calls += 1;
        }

        fn poll(&mut self) -> Option<Signal> {
            let check = self.poll_calls;
            self.poll_calls += 1;
            assert!(check < 256, "the loop did not stop on the signal");

            let raised = |from: Option<usize>| from.is_some_and(|from| check >= from);
            let kind = if raised(self.estop_from) {
                SignalKind::EmergencyStop
            } else if raised(self.shutdown_from) {
                SignalKind::Shutdown
            } else {
                return None;
            };
            Some(Signal {
                kind,
                meta: SignalMeta::new("scripted", file!(), line!()),
            })
        }
    }

    /// How many times each bucket was walked.
    #[derive(Default)]
    struct Walks {
        pre_init: Cell<u32>,
        init: Cell<u32>,
        post_init: Cell<u32>,
        first: Cell<u32>,
        each_tick: Cell<u32>,
        last: Cell<u32>,
    }

    impl Walks {
        fn counts(&self) -> [u32; 6] {
            [
                self.pre_init.get(),
                self.init.get(),
                self.post_init.get(),
                self.first.get(),
                self.each_tick.get(),
                self.last.get(),
            ]
        }
    }

    fn bump(cell: &Cell<u32>) -> impl Fn(&Time, &mut RobotState, &Sensors) + '_ {
        move |_t, _s, _se| cell.set(cell.get() + 1)
    }

    // A macro, not a fn: the robot's `HCons` type is not worth spelling.
    macro_rules! counting_robot {
        ($walks:expr) => {
            Robot::new()
                .set_dt(DT)
                .add_controller(PreInit, bump(&$walks.pre_init))
                .add_controller(Init, bump(&$walks.init))
                .add_controller(PostInit, bump(&$walks.post_init))
                .add_controller(First, bump(&$walks.first))
                .add_controller(EachTick, bump(&$walks.each_tick))
                .add_controller(Last, bump(&$walks.last))
        };
    }

    #[test]
    fn shutdown_seen_mid_tick_completes_the_tick() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now())
            .script(&[0, 0, 4])
            .shutdown_from(AFTER_FIRST);

        let signal = run_with(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::Shutdown);
        assert_eq!(walks.counts(), [1, 1, 1, 1, 1, 1]);
        assert_eq!(robot.config.time.last_update(), Some(&host.at(4)));
        assert_eq!(host.sleep_calls, 0);
        assert_eq!(host.poll_calls, TICK_END + 1);
    }

    #[test]
    fn estop_after_first_skips_the_rest_of_the_tick() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now()).estop_from(AFTER_FIRST);

        let signal = run_with(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::EmergencyStop);
        assert_eq!(walks.counts(), [1, 1, 1, 1, 0, 0]);
        assert_eq!(robot.config.time.last_update(), None);
        assert_eq!(host.sleep_calls, 0);
    }

    #[test]
    fn estop_after_each_tick_skips_last() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now()).estop_from(AFTER_EACH_TICK);

        let signal = run_with(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::EmergencyStop);
        assert_eq!(walks.counts(), [1, 1, 1, 1, 1, 0]);
        assert_eq!(robot.config.time.last_update(), None);
    }

    #[test]
    fn estop_wins_over_an_earlier_shutdown_in_the_same_tick() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now())
            .shutdown_from(AFTER_FIRST)
            .estop_from(AFTER_EACH_TICK);

        let signal = run_with(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::EmergencyStop);
        assert_eq!(walks.counts(), [1, 1, 1, 1, 1, 0]);
    }

    #[test]
    fn shutdown_during_startup_finishes_startup_and_runs_no_tick() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);

        let signal = run_with(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::Shutdown);
        assert_eq!(walks.counts(), [1, 1, 1, 0, 0, 0]);
        assert_eq!(host.poll_calls, TICK_START + 1);
    }

    #[test]
    fn estop_during_startup_skips_the_remaining_startup_phases() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now()).estop_from(AFTER_PRE_INIT);

        let signal = run_with(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::EmergencyStop);
        assert_eq!(walks.counts(), [1, 0, 0, 0, 0, 0]);
        assert_eq!(robot.config.time.deadline(), None);
    }

    #[test]
    fn sleeps_until_the_grid_after_an_overrun_too_and_not_on_exit() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now())
            // arm, then (start, end) of each tick, in tenths of DT.
            .script(&[0, 0, 3, 10, 25, 30, 38, 40, 42])
            .shutdown_from(TICK_END + 3 * PER_TICK);

        let signal = run_with(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::Shutdown);
        assert_eq!(walks.counts(), [1, 1, 1, 4, 4, 4]);
        // Tick 1 is late by 0.5: it sleeps to 3.0, so tick 2 starts on the
        // grid. The exit tick does not sleep.
        assert_eq!(host.sleep_calls, 3);
        assert_eq!(host.sleeps[0], Some(host.at(10)));
        assert_eq!(host.sleeps[1], Some(host.at(30)));
        assert_eq!(host.sleeps[2], Some(host.at(40)));
        assert_eq!(robot.config.time.accumulated_overrun(), Some(&(DT / 2)));
        assert_eq!(robot.config.time.overrun(), None);
    }

    #[test]
    fn real_host_sleeps_until_the_deadline_and_its_clock_moves() {
        // Lower bound only: a sleep never returns early, whatever the load.
        let mut host = RealHost;
        let deadline = host.now() + Duration::from_millis(2);

        host.sleep_until(deadline);
        assert!(host.now() >= deadline);

        // A deadline already behind must not block, nor panic.
        host.sleep_until(deadline);
    }

    #[test]
    fn failing_to_install_ctrl_c_is_harmless_only_after_our_own_install() {
        let failed = || Err(ctrlc::Error::MultipleHandlers);

        // The user's own handler, installed before the first run.
        let wired = AtomicBool::new(false);
        assert!(ctrl_c_wired(failed(), &wired).is_err());
        assert!(ctrl_c_wired(failed(), &wired).is_err());

        // Second run of a process whose first run installed ours.
        let wired = AtomicBool::new(false);
        assert!(ctrl_c_wired(Ok(()), &wired).is_ok());
        assert!(ctrl_c_wired(failed(), &wired).is_ok());
    }

    #[test]
    fn controllers_read_the_clock_the_engine_writes() {
        let dt = Duration::from_millis(7);
        let seen_dt = Cell::new(Duration::ZERO);
        let seen_elapsed = Cell::new(Duration::ZERO);

        let mut robot = Robot::new()
            .set_dt(dt)
            .add_controller(EachTick, |time: &Time, _s, _se| {
                seen_dt.set(*time.delta());
                seen_elapsed.set(*time.elapsed());
            });
        // Same origin as the clock, so the elapsed time is the scripted one.
        let mut host = Scripted::new(*robot.config.time.startup())
            .script(&[0, 0, 3, 10, 12])
            .shutdown_from(TICK_END + PER_TICK);

        run_with(&mut robot, &mut host);

        assert_eq!(seen_dt.get(), dt);
        // Stamped at the start of the second tick, not at its end.
        assert_eq!(seen_elapsed.get(), DT);
    }

    #[test]
    fn zero_dt_falls_back_before_any_controller_runs() {
        let seen_dt = Cell::new(Duration::ZERO);

        let mut robot = Robot::new().add_controller(PreInit, |time: &Time, _s, _se| {
            seen_dt.set(*time.delta());
        });
        let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);

        run_with(&mut robot, &mut host);

        assert_eq!(seen_dt.get(), FALLBACK_DT);
    }
}
