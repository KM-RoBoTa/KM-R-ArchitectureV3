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
//! # One clock
//!
//! There is a single [`crate::clock::Time`]: the one in `robot.config`, which
//! is also the one `Robot::set_dt` writes. The engine mutates it BETWEEN the
//! controller walks; the controllers only ever get `&Time`. That is enforced
//! by the borrow checker and not by convention: every phase builds its own
//! short-lived [`Env`], whose shared borrow of the clock ends with the walk.
//! Mutating the clock while a controller runs does not compile.
//!
//! # Stopping
//!
//! The loop only ends on a signal (see [`crate::signal`]):
//!
//! - shutdown is graceful: the tick that saw it is completed, `Last` bucket
//!   and clock bookkeeping included, then the loop exits without sleeping;
//! - emergency stop is immediate: the loop exits at the first check after it
//!   was raised. The remaining phases of the tick, the bookkeeping and the
//!   sleep are skipped.
//!
//! The bus is checked between the phases, so the granularity of an emergency
//! stop is one phase bucket: the controllers registered after the raising one
//! in the SAME bucket still run. A finer grain needs a check inside the
//! `RunPhase` walk.
//!
//! `Signal::drain` loads the bus and never clears it. Checking several times
//! per tick is therefore safe, and a shutdown seen in the middle of a tick is
//! still there at the end of it.

use std::time::{Duration, Instant};

use tracing::{debug, error, info, trace, warn};

use crate::controllers::Env;
use crate::schedule::Drive;
use crate::state::RobotState;
use crate::{Robot, Sensors, Signal, SignalKind, shutdown};

/// Tick duration used when the user never called `Robot::set_dt`, or called
/// it with zero.
///
/// A zero `dt` would turn the loop into a busy spin that reports every tick
/// as late. 1 ms is the rate the framework targets.
const FALLBACK_DT: Duration = Duration::from_millis(1);

/// Everything the loop takes from the outside world: the instant, the sleep
/// and the signal bus.
///
/// The bus is process-global and the clock is the wall clock, so a loop wired
/// straight to them can only be tested by sleeping and by serializing every
/// test of the process. Behind this seam the loop tests run on scripted
/// instants and fabricated signals. Generic parameter, monomorphized: no
/// `dyn`, no cost on the real path.
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
        // The remaining time is computed here, at the last moment, so that
        // whatever ran since the deadline was computed is not slept on top.
        //
        // TODO: `thread::sleep` wakes up tens of microseconds late under a
        // non real-time scheduler. It does not add up (absolute deadlines),
        // but it is jitter. A spin tail or `clock_nanosleep(TIMER_ABSTIME)`
        // under SCHED_FIFO belongs with the hardware layer: the first costs a
        // core, the second a libc dependency.
        std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
    }

    fn poll(&mut self) -> Option<Signal> {
        Signal::drain()
    }
}

/// Keeps the signal only if it is an emergency stop.
///
/// Used by the checks placed in the middle of a tick: a shutdown must let the
/// tick finish, so it is ignored there and picked up at the end of the tick.
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
    /// optional/mandatory systems.
    ///
    /// Returns owned values for [`RobotState`] and [`Sensors`]. The clock is
    /// not built here: it already lives in `robot.config`, where
    /// `Robot::set_dt` configured it.
    pub(super) fn pre_init() -> (RobotState, Sensors) {
        install_tracing();

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
///
/// The clock is not handled here but in [`control_loop`], between the phases.
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
        // TODO: every controller runs at the rate of the loop. A tick rate per
        // controller needs a divider stored in the node, that is a change to
        // the controller nodes and to their `RunPhase` walk.
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
        // Both kinds. Catches what was raised during the startup phases, and
        // what another thread (Ctrl-C) raised while the loop was asleep.
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

        // TODO: state_transition. Writes the desired state to the actuators
        // and records the sensed sample in the history. Both need the hardware
        // layer, which does not exist yet.

        ctrl_phase::last(robot, state, sensors);

        let wake = robot.config.time.end_tick(host.now());
        if let Some(overrun) = robot.config.time.overrun() {
            // `debug!` and not `warn!`: formatting a line per late tick makes
            // the next tick late too once the loop is already struggling. The
            // total is reported once, when the loop exits.
            debug!(?overrun, "tick finished after its deadline");
        }

        // Both kinds again, BEFORE the sleep: a stop request must not wait
        // for a sleep that serves a tick which will never run.
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

    // Before any controller runs, so that all of them read the `dt` the loop
    // really uses.
    if robot.config.time.delta().is_zero() {
        warn!(
            fallback = ?FALLBACK_DT,
            "no tick duration configured (`Robot::set_dt`), using the fallback"
        );
        robot.config.time.set_delta(FALLBACK_DT);
    }

    let signal = 'run: {
        // An emergency stop raised during startup skips the startup phases
        // that are left. A shutdown does not: it lets the startup finish and
        // is picked up by the first check of the loop, before any tick.
        init_phase::pre_init_controllers(robot, &mut state, &sensors);
        if let Some(signal) = estop(host.poll()) {
            break 'run signal;
        }
        init_phase::init(robot, &mut state, &sensors);
        if let Some(signal) = estop(host.poll()) {
            break 'run signal;
        }
        init_phase::post_init(robot, &mut state, &sensors);

        // TODO: spawn the `Threaded` controllers. Left out because the way
        // their result is handed to the main loop is an open design question
        // (docs/ROADMAP.md). Until then they are stored and never run.

        // Anchored here and not at `Robot::new()`: the time spent in the
        // startup phases must not count as an overrun of the first tick.
        robot.config.time.arm(host.now());

        control_loop(robot, &mut state, &sensors, host)
    };

    // TODO: no hardware layer yet. A shutdown must send one last desired
    // state before it exits, an emergency stop must cut the torque.
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
/// Returns the signal that ended the run, so that the caller decides what a
/// shutdown and an emergency stop mean for the user.
pub(crate) fn runtime<CS: Drive>(mut robot: Robot<CS>) -> Signal {
    // The bus is process-global: without this, a signal left by a previous
    // run would end this one before its first tick.
    Signal::reset();

    // Installed AFTER the reset, or a Ctrl-C landing between the two would be
    // erased. The error is ignored: the handler can only be installed once
    // per process, so a second run in the same process always fails here and
    // keeps the handler of the first one, which is the same closure.
    let _ = ctrlc::set_handler(|| {
        shutdown!("Ctrl-C received");
    });

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

    // Index of each bus check of `run_with`, counted from its first one.
    // Startup makes two of them, then every tick makes four.
    const AFTER_PRE_INIT: usize = 0;
    const TICK_START: usize = 2;
    const AFTER_FIRST: usize = 3;
    const AFTER_EACH_TICK: usize = 4;
    const TICK_END: usize = 5;
    const PER_TICK: usize = 4;

    /// A scripted world. It never sleeps and never touches the global bus, so
    /// these tests can run in parallel with each other and with
    /// `signal::test::bus_semantics`.
    ///
    /// They cannot hang either: once raised, the signal is returned by every
    /// later check, like the real bus which is never cleared by a drain.
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
            // Guard against a loop that would ignore the signals: the test
            // fails instead of spinning forever.
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

    // One counting controller per bucket. A macro and not a fn: the type of
    // the built robot is one `HCons` layer per controller, not worth spelling.
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
        // The bookkeeping of the last tick was done, the sleep was not.
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
        // The grid was never anchored: the loop was not entered.
        assert_eq!(robot.config.time.deadline(), None);
    }

    #[test]
    fn sleeps_until_the_grid_and_not_after_an_overrun_or_on_exit() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now())
            // arm, then (start, end) of each tick, in tenths of DT.
            .script(&[0, 0, 3, 10, 25, 25, 28, 30, 32])
            .shutdown_from(TICK_END + 3 * PER_TICK);

        let signal = run_with(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::Shutdown);
        assert_eq!(walks.counts(), [1, 1, 1, 4, 4, 4]);
        // Tick 0 on time, tick 1 late by half a DT (no sleep), tick 2 on time
        // for the slot the grid kept, tick 3 is the exit tick (no sleep).
        assert_eq!(host.sleep_calls, 2);
        assert_eq!(host.sleeps[0], Some(host.at(10)));
        assert_eq!(host.sleeps[1], Some(host.at(30)));
        assert_eq!(robot.config.time.accumulated_overrun(), Some(&(DT / 2)));
        assert_eq!(robot.config.time.overrun(), None);
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
