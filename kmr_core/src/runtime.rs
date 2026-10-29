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
//!
//! From `open` on, the [`Powered`] guard ends every exit, unwind included,
//! with `torque_off` then `close`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use tracing::{debug, error, info, trace, warn};

use crate::controllers::Env;
use crate::hal::{Batch, Faults, Hal, Layout, Overrides};
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

/// Owns the adapter from `open` on: every exit, unwind included, ends with
/// `torque_off` then `close` (SRS FR-5.6, NFR-2).
struct Powered<'a, A: Hal>(&'a mut A);

impl<A: Hal> Drop for Powered<'_, A> {
    fn drop(&mut self) {
        self.0.torque_off();
        self.0.close();
    }
}

/// The startup phases whose controllers wrote Desired. Those writes are
/// dropped, never sent: only ticks command the hardware (SRS FR-5.2).
#[derive(Default)]
struct StartupWrites {
    phases: [&'static str; 3],
    len: usize,
}

impl StartupWrites {
    /// Drops what the walk of `phase` wrote, and notes the phase.
    fn discard(&mut self, state: &mut RobotState, phase: &'static str) {
        if state.written().any().is_empty() {
            return;
        }
        state.clear_desired();
        if let Some(slot) = self.phases.get_mut(self.len) {
            *slot = phase;
            self.len += 1;
        }
    }

    /// One warning for the run, naming every phase that wrote (SC-011).
    fn report(&self) {
        if let Some(phases) = self.phases.get(..self.len)
            && !phases.is_empty()
        {
            warn!(
                ?phases,
                "desired values written during startup were discarded: only ticks command the hardware"
            );
        }
    }
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

    /// Walks the user's `Init` controllers, once the engine has opened and
    /// zeroed the hardware.
    pub(super) fn init<CS: Drive>(
        robot: &mut Robot<CS>,
        state: &mut RobotState,
        sensors: &Sensors,
    ) {
        trace!("Executing user init controllers ...");
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.init(&mut env);
    }

    /// Walks the user's `PostInit` controllers, once the engine has checked
    /// and configured the hardware, read it in full and turned torque on.
    pub(super) fn post_init<CS: Drive>(
        robot: &mut Robot<CS>,
        state: &mut RobotState,
        sensors: &Sensors,
    ) {
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

/// The tick loop: read, the three buckets, write. Returns the signal that
/// ended it.
fn control_loop<CS: Drive, H: Host, A: Hal>(
    robot: &mut Robot<CS>,
    state: &mut RobotState,
    sensors: &Sensors,
    host: &mut H,
    hal: &mut A,
    layout: &Layout,
) -> Signal {
    loop {
        if let Some(signal) = host.poll() {
            break signal;
        }
        robot.config.time.begin_tick(host.now());

        // Dropped until the per-slot fault flags land (SRS FR-13.4).
        let mut faults = Faults::default();
        if let Some(sample) = state.push_slot() {
            hal.read(sample, &mut faults);
        }

        ctrl_phase::first(robot, state, sensors);
        if let Some(signal) = estop(host.poll()) {
            break signal;
        }

        ctrl_phase::each_tick(robot, state, sensors);
        if let Some(signal) = estop(host.poll()) {
            break signal;
        }

        ctrl_phase::last(robot, state, sensors);
        // An emergency stop raised in `Last` skips the write.
        if let Some(signal) = estop(host.poll()) {
            break signal;
        }

        let batch = Batch::new(layout, state.written());
        if !batch.is_empty() {
            hal.write(state.command(), &batch, &mut faults);
        }

        let wake = robot.config.time.end_tick(host.now());
        if let Some(overrun) = robot.config.time.overrun() {
            // `debug!`, not `warn!`: a log line per late tick makes the next one late.
            debug!(?overrun, "tick finished after its deadline");
        }

        // Before the sleep: a stop must not wait for it. Before the clear: the
        // hold of a shutdown keeps this tick's writes.
        if let Some(signal) = host.poll() {
            break signal;
        }
        state.clear_desired();
        if let Some(deadline) = wake {
            host.sleep_until(deadline);
        }
    }
}

/// Startup phases with the bring-up calls, the tick loop, then the hold of a
/// shutdown. Every exit after `open` leaves through [`Powered`].
fn run_phases<CS: Drive, H: Host, A: Hal>(
    robot: &mut Robot<CS>,
    state: &mut RobotState,
    sensors: &Sensors,
    host: &mut H,
    hal: &mut A,
    layout: &Layout,
) -> Result<Signal, A::Error> {
    let mut writes = StartupWrites::default();

    init_phase::pre_init_controllers(robot, state, sensors);
    writes.discard(state, "PreInit");
    if let Some(signal) = estop(host.poll()) {
        return Ok(signal);
    }

    // Built before `open`, so a failed `open` is closed too.
    let powered = Powered(hal);
    powered.0.open()?;
    powered.0.zero()?;
    init_phase::init(robot, state, sensors);
    writes.discard(state, "Init");
    if let Some(signal) = estop(host.poll()) {
        return Ok(signal);
    }

    powered.0.ping()?;
    powered.0.health()?;
    powered.0.configure()?;
    // The seed: `prev(1)` of tick 0.
    if let Some(sample) = state.push_slot() {
        powered.0.read_all(sample)?;
    }
    // A pending emergency stop never powers the actuators (SRS FR-4.2).
    if let Some(signal) = estop(host.poll()) {
        return Ok(signal);
    }
    powered.0.torque_on()?;
    init_phase::post_init(robot, state, sensors);
    writes.discard(state, "PostInit");
    writes.report();

    // TODO: spawn the `Threaded` controllers (hand-off open in docs/ROADMAP.md).

    // After the bring-up, so tick 0 is not charged with it (SRS FR-5.10).
    robot.config.time.arm(host.now());

    let signal = control_loop(robot, state, sensors, host, powered.0, layout);
    // An emergency stop raised since the loop ended skips the hold.
    let signal = estop(host.poll()).unwrap_or(signal);
    if signal.kind() == SignalKind::Shutdown
        && let Some(target) = state.hold_target()
    {
        powered.0.hold(target.command());
    }
    state.clear_desired();
    Ok(signal)
}

/// Finalization, startup phases, then the tick loop. Returns the signal that
/// ended the run, or the adapter's error.
fn run_with<CS: Drive, H: Host, A: Hal>(
    robot: &mut Robot<CS>,
    host: &mut H,
    hal: &mut A,
) -> Result<Signal, A::Error> {
    let layout = hal.check(Overrides::default())?;
    let (mut state, sensors) = init_phase::pre_init();

    if robot.config.time.delta().is_zero() {
        warn!(
            fallback = ?FALLBACK_DT,
            "no tick duration configured (`Robot::set_dt`), using the fallback"
        );
        robot.config.time.set_delta(FALLBACK_DT);
    }

    // The guard has dropped by now: torque is off before the log lines.
    let signal = run_phases(robot, &mut state, &sensors, host, hal, &layout)?;

    match signal.kind() {
        SignalKind::Shutdown => info!("control loop stopped, {signal}"),
        SignalKind::EmergencyStop => error!("control loop aborted, {signal}"),
    }
    let total = robot.config.time.accumulated_overrun();
    if !total.is_zero() {
        info!(?total, "ticks finished after their deadline during the run");
    }

    Ok(signal)
}

/// The entry point of the runtime of the core.
///
/// Returns the signal that ended the run, or the adapter's error.
pub(crate) fn runtime<CS: Drive, A: Hal>(
    mut robot: Robot<CS>,
    mut hal: A,
) -> Result<Signal, A::Error> {
    Signal::reset();

    // Before the Ctrl-C install, which may warn.
    init_phase::install_tracing();

    // After the reset, or a Ctrl-C landing in between would be erased.
    let installed = ctrlc::set_handler(|| {
        shutdown!("Ctrl-C, SIGTERM or SIGHUP received");
    });
    // A warning, not an error: a user handler may raise `shutdown!` itself.
    if let Err(error) = ctrl_c_wired(installed, &CTRL_C_WIRED) {
        warn!(
            %error,
            "Ctrl-C, SIGTERM and SIGHUP will not stop the control loop: their \
             handler could not be installed. If the process has its own, it \
             must raise `shutdown!`"
        );
    }

    run_with(&mut robot, &mut RealHost, &mut hal)
}

#[cfg(test)]
mod tests {
    use std::cell::{Cell, Ref, RefCell};
    use std::num::NonZeroU32;
    use std::sync::atomic::AtomicU32;

    use super::*;
    use crate::clock::Time;
    use crate::hal::{
        Batch, Command, Faults, Group, Layout, Mask, NoHardware, Sample, Written, seal,
    };
    use crate::signal::SignalMeta;
    use crate::state::{JOINTS, Q, Qd};
    use crate::{EachTick, First, Init, Last, PostInit, PreInit};

    const DT: Duration = Duration::from_millis(1);

    // Index of each bus check: three at startup, then five per tick.
    const AFTER_PRE_INIT: usize = 0;
    const AFTER_INIT: usize = 1;
    const BEFORE_TORQUE_ON: usize = 2;
    const TICK_START: usize = 3;
    const AFTER_FIRST: usize = 4;
    const AFTER_EACH_TICK: usize = 5;
    const AFTER_LAST: usize = 6;
    const TICK_END: usize = 7;
    const PER_TICK: usize = 5;

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

    /// A run on [`NoHardware`], which cannot fail.
    fn simulated<CS: Drive, H: Host>(robot: &mut Robot<CS>, host: &mut H) -> Signal {
        match run_with(robot, host, &mut NoHardware) {
            Ok(signal) => signal,
            Err(never) => match never {},
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

        let signal = simulated(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::Shutdown);
        assert_eq!(walks.counts(), [1, 1, 1, 1, 1, 1]);
        assert_eq!(robot.config.time.last_update(), Some(&host.at(4)));
        assert_eq!(host.sleep_calls, 0);
        // + 1: the estop check after the loop, before the hold.
        assert_eq!(host.poll_calls, TICK_END + 2);
    }

    #[test]
    fn estop_after_first_skips_the_rest_of_the_tick() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now()).estop_from(AFTER_FIRST);

        let signal = simulated(&mut robot, &mut host);

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

        let signal = simulated(&mut robot, &mut host);

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

        let signal = simulated(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::EmergencyStop);
        assert_eq!(walks.counts(), [1, 1, 1, 1, 1, 0]);
    }

    #[test]
    fn shutdown_during_startup_finishes_startup_and_runs_no_tick() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);

        let signal = simulated(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::Shutdown);
        assert_eq!(walks.counts(), [1, 1, 1, 0, 0, 0]);
        assert_eq!(host.poll_calls, TICK_START + 2);
    }

    #[test]
    fn estop_during_startup_skips_the_remaining_startup_phases() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now()).estop_from(AFTER_PRE_INIT);

        let signal = simulated(&mut robot, &mut host);

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

        let signal = simulated(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::Shutdown);
        assert_eq!(walks.counts(), [1, 1, 1, 4, 4, 4]);
        // Tick 1 is late by 0.5: it sleeps to 3.0, so tick 2 starts on the
        // grid. The exit tick does not sleep.
        assert_eq!(host.sleep_calls, 3);
        assert_eq!(host.sleeps[0], Some(host.at(10)));
        assert_eq!(host.sleeps[1], Some(host.at(30)));
        assert_eq!(host.sleeps[2], Some(host.at(40)));
        assert_eq!(robot.config.time.accumulated_overrun(), DT / 2);
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
                seen_dt.set(time.dt());
                seen_elapsed.set(time.elapsed());
            });
        // The clock is armed at the first scripted instant, 0.
        let mut host = Scripted::new(Instant::now())
            .script(&[0, 0, 3, 10, 12])
            .shutdown_from(TICK_END + PER_TICK);

        simulated(&mut robot, &mut host);

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

        simulated(&mut robot, &mut host);

        assert_eq!(seen_dt.get(), FALLBACK_DT);
    }

    mod exit_paths {
        //! The adapter calls of a run, interleaved with the bucket walks, on
        //! every way out.

        use super::*;

        /// An adapter call or a bucket walk.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        enum Call {
            Check,
            Open,
            Zero,
            Ping,
            Health,
            Configure,
            ReadAll,
            TorqueOn,
            Read,
            Write,
            Hold,
            TorqueOff,
            Close,
            PreInit,
            Init,
            PostInit,
            First,
            EachTick,
            Last,
        }

        // Named, not a glob: they shadow the schedule labels of `super::*`.
        use Call::{
            Check, Close, Configure, EachTick, First, Health, Hold, Init, Last, Open, Ping,
            PostInit, PreInit, Read, ReadAll, TorqueOff, TorqueOn, Write, Zero,
        };

        /// What `write` or `hold` was handed: the positions, the masks and the
        /// actuators of the batch (every one for `hold`).
        #[derive(Clone, Copy, Debug, PartialEq)]
        struct Sent {
            q: [Q; JOINTS],
            written: Written,
            batch: Mask,
        }

        const CAPACITY: usize = 96;

        /// The log the recorder and the controllers share. Fixed capacity: no
        /// heap in the core, tests included.
        struct Log {
            calls: RefCell<([Call; CAPACITY], usize)>,
            write: Cell<Option<Sent>>,
            hold: Cell<Option<Sent>>,
        }

        impl Log {
            fn new() -> Self {
                Self {
                    calls: RefCell::new(([Check; CAPACITY], 0)),
                    write: Cell::new(None),
                    hold: Cell::new(None),
                }
            }

            fn push(&self, call: Call) {
                let (calls, len) = &mut *self.calls.borrow_mut();
                *calls.get_mut(*len).expect("log capacity") = call;
                *len += 1;
            }

            fn calls(&self) -> Ref<'_, [Call]> {
                Ref::map(self.calls.borrow(), |(calls, len)| &calls[..*len])
            }
        }

        /// Position the recorder's `read_all` reads on every actuator.
        const SEED: Q = Q(10.0);

        /// A fake adapter: logs every call, can fail one bring-up step. Each
        /// `read` moves actuator 0 by one.
        struct Recorder<'a> {
            log: &'a Log,
            fail: Option<Call>,
        }

        impl<'a> Recorder<'a> {
            fn new(log: &'a Log) -> Self {
                Self { log, fail: None }
            }

            fn failing(log: &'a Log, call: Call) -> Self {
                Self {
                    log,
                    fail: Some(call),
                }
            }

            fn step(&mut self, call: Call) -> Result<(), Failed> {
                self.log.push(call);
                match self.fail {
                    Some(fail) if fail == call => Err(Failed(call)),
                    _ => Ok(()),
                }
            }
        }

        #[derive(Debug, PartialEq)]
        struct Failed(Call);

        impl core::fmt::Display for Failed {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(f, "{:?} failed", self.0)
            }
        }

        impl core::error::Error for Failed {}

        impl seal::Adapter for Recorder<'_> {}

        impl Hal for Recorder<'_> {
            type Error = Failed;

            fn check(&mut self, _overrides: Overrides) -> Result<Layout, Failed> {
                self.step(Check)?;
                Ok(Layout::new(
                    [Group(0); JOINTS],
                    [NonZeroU32::MIN; JOINTS],
                    Duration::ZERO,
                    Duration::ZERO,
                ))
            }

            fn open(&mut self) -> Result<(), Failed> {
                self.step(Open)
            }

            fn zero(&mut self) -> Result<(), Failed> {
                self.step(Zero)
            }

            fn ping(&mut self) -> Result<(), Failed> {
                self.step(Ping)
            }

            fn health(&mut self) -> Result<(), Failed> {
                self.step(Health)
            }

            fn configure(&mut self) -> Result<(), Failed> {
                self.step(Configure)
            }

            fn read_all(&mut self, sample: Sample<'_>) -> Result<(), Failed> {
                *sample.q = [SEED; JOINTS];
                self.step(ReadAll)
            }

            fn torque_on(&mut self) -> Result<(), Failed> {
                self.step(TorqueOn)
            }

            fn read(&mut self, sample: Sample<'_>, _faults: &mut Faults) -> Mask {
                self.log.push(Read);
                sample.q[0] = sample.q[0] + Q(1.0);
                Mask::ALL
            }

            fn write(&mut self, command: Command<'_>, batch: &Batch<'_>, _faults: &mut Faults) {
                self.log.push(Write);
                let mut actuators = Mask::NONE;
                for &actuator in batch.runs().flat_map(|(_group, run)| run) {
                    actuators.set(actuator);
                }
                self.log.write.set(Some(Sent {
                    q: *command.q,
                    written: *command.written,
                    batch: actuators,
                }));
            }

            fn hold(&mut self, target: Command<'_>) {
                self.log.push(Hold);
                self.log.hold.set(Some(Sent {
                    q: *target.q,
                    written: *target.written,
                    batch: Mask::ALL,
                }));
            }

            fn torque_off(&mut self) {
                self.log.push(TorqueOff);
            }

            fn close(&mut self) {
                self.log.push(Close);
            }
        }

        fn walk(log: &Log, call: Call) -> impl Fn(&Time, &mut RobotState, &Sensors) + '_ {
            move |_t, _s, _se| log.push(call)
        }

        // A macro, not a fn: the robot's `HCons` type is not worth spelling.
        macro_rules! recording_robot {
            ($log:expr) => {
                Robot::new()
                    .set_dt(DT)
                    .add_controller(crate::PreInit, walk(&$log, PreInit))
                    .add_controller(crate::Init, walk(&$log, Init))
                    .add_controller(crate::PostInit, walk(&$log, PostInit))
                    .add_controller(crate::First, walk(&$log, First))
                    .add_controller(crate::EachTick, walk(&$log, EachTick))
                    .add_controller(crate::Last, walk(&$log, Last))
            };
        }

        #[test]
        fn a_check_error_is_returned_before_any_io() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now());

            let result = run_with(&mut robot, &mut host, &mut Recorder::failing(&log, Check));

            assert_eq!(result.err(), Some(Failed(Check)));
            assert_eq!(*log.calls(), [Check]);
            assert_eq!(host.poll_calls, 0);
        }

        /// Every bring-up call, PostInit's walk included.
        const STARTUP: [Call; 11] = [
            Check, PreInit, Open, Zero, Init, Ping, Health, Configure, ReadAll, TorqueOn, PostInit,
        ];

        #[test]
        fn a_failed_open_is_still_closed() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now());

            let result = run_with(&mut robot, &mut host, &mut Recorder::failing(&log, Open));

            assert_eq!(result.err(), Some(Failed(Open)));
            assert_eq!(*log.calls(), [Check, PreInit, Open, TorqueOff, Close]);
        }

        #[test]
        fn a_ping_error_skips_post_init_cuts_torque_and_is_returned() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now());

            let result = run_with(&mut robot, &mut host, &mut Recorder::failing(&log, Ping));

            assert_eq!(result.err(), Some(Failed(Ping)));
            assert_eq!(
                *log.calls(),
                [Check, PreInit, Open, Zero, Init, Ping, TorqueOff, Close]
            );
        }

        #[test]
        fn a_failed_torque_on_still_cuts_torque() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now());

            let result = run_with(
                &mut robot,
                &mut host,
                &mut Recorder::failing(&log, TorqueOn),
            );

            assert_eq!(result.err(), Some(Failed(TorqueOn)));
            assert_eq!(log.calls()[..10], STARTUP[..10]);
            assert_eq!(log.calls()[10..], [TorqueOff, Close]);
        }

        #[test]
        fn an_estop_in_pre_init_touches_no_hardware_after_check() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).estop_from(AFTER_PRE_INIT);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(*log.calls(), [Check, PreInit]);
        }

        #[test]
        fn an_estop_in_init_after_open_cuts_torque_and_closes() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).estop_from(AFTER_INIT);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(
                *log.calls(),
                [Check, PreInit, Open, Zero, Init, TorqueOff, Close]
            );
        }

        #[test]
        fn an_estop_in_post_init_never_powers() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).estop_from(BEFORE_TORQUE_ON);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(log.calls()[..9], STARTUP[..9]);
            assert_eq!(log.calls()[9..], [TorqueOff, Close]);
        }

        #[test]
        fn a_shutdown_before_the_first_tick_holds_the_seed() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(log.calls()[..11], STARTUP);
            assert_eq!(log.calls()[11..], [Hold, TorqueOff, Close]);
            let hold = log.hold.get().expect("a hold");
            assert_eq!(hold.q, [SEED; JOINTS]);
            assert_eq!(hold.written, Written::default());
        }

        #[test]
        fn startup_writes_never_reach_the_adapter() {
            static WARNINGS: AtomicU32 = AtomicU32::new(0);
            let log = Log::new();
            let mut robot = recording_robot!(log)
                .add_controller(crate::PreInit, |_t, state, _se| {
                    state.set_all([Q(99.0); JOINTS]);
                })
                .add_controller(crate::PostInit, |_t, state, _se| {
                    state.set_at(Q(98.0), 1).expect("in range");
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);
            let counter = Counter::new("discarded", &WARNINGS);

            let result = tracing::subscriber::with_default(counter, || {
                run_with(&mut robot, &mut host, &mut Recorder::new(&log))
            });

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert!(!log.calls().contains(&Write));
            let hold = log.hold.get().expect("a hold");
            assert_eq!(hold.q, [SEED; JOINTS]);
            assert_eq!(hold.written, Written::default());
            // One warning for the two phases that wrote (SC-011).
            assert_eq!(WARNINGS.load(Ordering::Relaxed), 1);
        }

        #[test]
        fn a_panicking_controller_still_cuts_torque_and_closes() {
            let log = Log::new();
            let mut robot = recording_robot!(log).add_controller(crate::EachTick, |_t, _s, _se| {
                panic!("a controller bug");
            });
            let mut host = Scripted::new(Instant::now());

            let unwound = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                run_with(&mut robot, &mut host, &mut Recorder::new(&log))
            }));

            assert!(unwound.is_err());
            assert_eq!(log.calls()[..11], STARTUP);
            assert_eq!(log.calls()[11..], [Read, First, EachTick, TorqueOff, Close]);
        }

        /// The mask of `indices`.
        fn mask(indices: &[usize]) -> Mask {
            let mut mask = Mask::NONE;
            for &index in indices {
                mask.set(index);
            }
            mask
        }

        #[test]
        fn shutdown_reads_before_first_writes_after_last_then_holds_and_cuts_torque() {
            let log = Log::new();
            let mut robot =
                recording_robot!(log).add_controller(crate::EachTick, |_t, state, _se| {
                    state.set_at(Q(5.0), 1).expect("in range");
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(log.calls()[..11], STARTUP);
            assert_eq!(
                log.calls()[11..],
                [Read, First, EachTick, Last, Write, Hold, TorqueOff, Close]
            );
            let write = log.write.get().expect("a write");
            assert_eq!(write.q[1], Q(5.0));
            assert_eq!(write.batch, mask(&[1]));
            // The latest sample, overlaid with the tick's write.
            let hold = log.hold.get().expect("a hold");
            assert_eq!(hold.q, [Q(11.0), Q(5.0), SEED, SEED]);
            assert_eq!(hold.written.q, mask(&[1]));
        }

        #[test]
        fn every_tick_runs_read_first_each_last_write_and_desired_clears() {
            let log = Log::new();
            let mut robot =
                recording_robot!(log).add_controller(crate::EachTick, |time, state, _se| {
                    if time.tick() == 0 {
                        state.set_at(Q(5.0), 1).expect("in range");
                    }
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST + 2 * PER_TICK);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(log.calls()[..11], STARTUP);
            assert_eq!(
                log.calls()[11..],
                [
                    Read, First, EachTick, Last, Write, // tick 0 wrote
                    Read, First, EachTick, Last, // nothing written: no write
                    Read, First, EachTick, Last, //
                    Hold, TorqueOff, Close,
                ]
            );
            let hold = log.hold.get().expect("a hold");
            assert_eq!(hold.q, [Q(13.0), SEED, SEED, SEED]);
            assert_eq!(hold.written, Written::default());
        }

        #[test]
        fn an_estop_after_each_tick_skips_last_the_write_and_the_hold() {
            let log = Log::new();
            let mut robot =
                recording_robot!(log).add_controller(crate::EachTick, |_t, state, _se| {
                    state.set_at(Q(5.0), 1).expect("in range");
                });
            let mut host = Scripted::new(Instant::now()).estop_from(AFTER_EACH_TICK);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(log.calls()[11..], [Read, First, EachTick, TorqueOff, Close]);
        }

        #[test]
        fn an_estop_raised_in_last_skips_the_write() {
            let log = Log::new();
            let mut robot = recording_robot!(log).add_controller(crate::Last, |_t, state, _se| {
                state.set_at(Q(5.0), 1).expect("in range");
            });
            let mut host = Scripted::new(Instant::now()).estop_from(AFTER_LAST);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(
                log.calls()[11..],
                [Read, First, EachTick, Last, TorqueOff, Close]
            );
        }

        #[test]
        fn an_estop_raised_after_the_shutdown_tick_skips_the_hold() {
            let log = Log::new();
            let mut robot =
                recording_robot!(log).add_controller(crate::EachTick, |_t, state, _se| {
                    state.set_at(Q(5.0), 1).expect("in range");
                });
            let mut host = Scripted::new(Instant::now())
                .shutdown_from(TICK_END)
                .estop_from(TICK_END + 1);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(
                log.calls()[11..],
                [Read, First, EachTick, Last, Write, TorqueOff, Close]
            );
            assert_eq!(log.hold.get(), None);
        }

        #[test]
        fn an_unwritten_actuator_gets_no_command_and_only_its_field_is_sent() {
            let log = Log::new();
            let mut robot =
                recording_robot!(log).add_controller(crate::EachTick, |_t, state, _se| {
                    state.set_at(Q(1.0), 0).expect("in range");
                    state.set_at(Qd(0.5), 2).expect("in range");
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            let write = log.write.get().expect("a write");
            assert_eq!(write.batch, mask(&[0, 2]));
            assert_eq!(write.written.q, mask(&[0]));
            assert_eq!(write.written.qd, mask(&[2]));
            assert_eq!(write.written.tau, Mask::NONE);
        }

        #[test]
        fn history_gets_one_slot_per_tick_seeded_by_read_all() {
            let log = Log::new();
            // (prev(0), prev(1)) of actuator 0, seen in `First` of each tick.
            let seen = Cell::new([(Q(0.0), Q(0.0)); 2]);
            let mut robot =
                recording_robot!(log).add_controller(crate::First, |time, state, _se| {
                    let mut all = seen.get();
                    if let Some(slot) = all.get_mut(time.tick() as usize) {
                        *slot = (
                            *state.prev_at::<Q>(0, 0).expect("a sample"),
                            *state.prev_at::<Q>(1, 0).expect("the slot before"),
                        );
                    }
                    seen.set(all);
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST + PER_TICK);

            let result = run_with(&mut robot, &mut host, &mut Recorder::new(&log));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            // Tick 0 reads 11 over the seed 10; tick 1 reads 12.
            assert_eq!(seen.get(), [(Q(11.0), SEED), (Q(12.0), Q(11.0))]);
        }

        #[test]
        fn a_no_hardware_run_logs_simulating_exactly_once() {
            static WARNINGS: AtomicU32 = AtomicU32::new(0);
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);
            let counter = Counter::new("no bus plugin: simulating", &WARNINGS);

            tracing::subscriber::with_default(counter, || simulated(&mut robot, &mut host));

            assert_eq!(WARNINGS.load(Ordering::Relaxed), 1);
        }

        /// Counts the warnings whose message holds `needle`, drops every
        /// other event. No text is stored.
        struct Counter {
            needle: &'static str,
            count: &'static AtomicU32,
        }

        impl Counter {
            fn new(needle: &'static str, count: &'static AtomicU32) -> Self {
                Self { needle, count }
            }
        }

        /// Looks for the needle in what is written to it.
        struct Finder {
            needle: &'static str,
            found: bool,
        }

        impl core::fmt::Write for Finder {
            fn write_str(&mut self, chunk: &str) -> core::fmt::Result {
                self.found |= chunk.contains(self.needle);
                Ok(())
            }
        }

        impl tracing::field::Visit for Finder {
            fn record_debug(
                &mut self,
                field: &tracing::field::Field,
                value: &dyn core::fmt::Debug,
            ) {
                if field.name() == "message" {
                    let _ = core::fmt::Write::write_fmt(self, format_args!("{value:?}"));
                }
            }
        }

        impl tracing::Subscriber for Counter {
            fn enabled(&self, metadata: &tracing::Metadata<'_>) -> bool {
                *metadata.level() == tracing::Level::WARN
            }

            fn event(&self, event: &tracing::Event<'_>) {
                let mut finder = Finder {
                    needle: self.needle,
                    found: false,
                };
                event.record(&mut finder);
                if finder.found {
                    self.count.fetch_add(1, Ordering::Relaxed);
                }
            }

            fn new_span(&self, _span: &tracing::span::Attributes<'_>) -> tracing::span::Id {
                tracing::span::Id::from_u64(1)
            }
            fn record(&self, _span: &tracing::span::Id, _values: &tracing::span::Record<'_>) {}
            fn record_follows_from(&self, _span: &tracing::span::Id, _follows: &tracing::span::Id) {
            }
            fn enter(&self, _span: &tracing::span::Id) {}
            fn exit(&self, _span: &tracing::span::Id) {}
        }
    }
}
