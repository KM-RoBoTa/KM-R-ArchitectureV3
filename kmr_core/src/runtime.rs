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
//! two phase buckets. Any signal raised during the bring-up, a shutdown
//! included, stops it before `torque_on`.
//!
//! From `open` on, the [`Powered`] guard ends every exit, unwind included,
//! with `torque_off` then `close`, then the sensors' `close` once they were
//! opened.

use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;

use crate::logging::{error, info, trace, warn};

use crate::clock::Instant;
use crate::controllers::Env;
use crate::error::{ConfigError, RunError};
use crate::hal::{
    Audit, Batch, Drivers, EXTERNAL_FLAGS, Faults, HardwareError, Layout, Plan, Sensing,
    SensorPlan, SensorPolicy, Step, TickFlags,
};
use crate::schedule::Drive;
use crate::signal::SignalMeta;
use crate::state::RobotState;
use crate::{HNil, Robot, Sensors, Signal, SignalKind, Stages, shutdown};

/// Tick duration used when `Robot::set_dt` was never called.
const FALLBACK_DT: Duration = Duration::from_millis(1);

/// Shortest tick period a run accepts (SRS FR-3.1).
pub(crate) const DT_MIN: Duration = Duration::from_micros(10);

/// Longest tick period a run accepts (SRS FR-3.1).
pub(crate) const DT_MAX: Duration = Duration::from_secs(1);

/// The tick period of the run: the configured one if in range, else the
/// fallback when none was configured.
fn tick_period(configured: Option<Duration>) -> Result<Duration, ConfigError> {
    match configured {
        None => {
            warn!(
                fallback = ?FALLBACK_DT,
                "no tick duration configured (`Robot::set_dt`), using the fallback"
            );
            Ok(FALLBACK_DT)
        }
        Some(dt) if (DT_MIN..=DT_MAX).contains(&dt) => Ok(dt),
        Some(dt) => Err(ConfigError::DtOutOfRange { dt }),
    }
}

/// Seal of [`Host`]: open to platform crates, closed to controllers, which
/// cannot depend on `kmr_core`.
#[doc(hidden)]
pub mod seal {
    /// Implemented next to [`super::Host`] by every host.
    pub trait Host {}
}

/// The instant, the sleep and the signal bus: a seam so the loop tests run on
/// scripted values instead of the wall clock and the global bus, and so a
/// target without `std` brings its own timer (SRS NFR-7).
pub trait Host: seal::Host {
    /// The current instant.
    fn now(&mut self) -> Instant;
    /// Blocks until `deadline`.
    fn sleep_until(&mut self, deadline: Instant);
    /// The signal on the bus, if any.
    fn poll(&mut self) -> Option<Signal>;
    /// The tick flags raised from outside the loop since the last call.
    fn take_flags(&mut self) -> TickFlags {
        TickFlags::NONE
    }
}

/// The real world: system clock, absolute sleep, global signal bus.
#[cfg(feature = "std")]
struct RealHost {
    /// Wakes this long before each deadline and spins the rest; zero: no spin.
    spin_tail: Duration,
}

#[cfg(feature = "std")]
impl RealHost {
    /// The host of a run: no spin tail.
    // DECISION: no spin by default and no user knob yet; the NFR-1 bench
    // (bench A-10) measures whether a tail is worth its CPU first.
    const fn new() -> Self {
        Self {
            spin_tail: Duration::ZERO,
        }
    }
}

#[cfg(feature = "std")]
impl seal::Host for RealHost {}

#[cfg(feature = "std")]
impl Host for RealHost {
    fn now(&mut self) -> Instant {
        Instant::now()
    }

    fn sleep_until(&mut self, deadline: Instant) {
        let wake = deadline.checked_sub(self.spin_tail).unwrap_or(deadline);
        sleep_until(wake);
        while Instant::now() < deadline {
            core::hint::spin_loop();
        }
    }

    fn poll(&mut self) -> Option<Signal> {
        Signal::drain()
    }

    fn take_flags(&mut self) -> TickFlags {
        EXTERNAL_FLAGS.take()
    }
}

/// Blocks until `deadline` with an absolute `clock_nanosleep` on the
/// monotonic clock, so a late wake never shifts the next one (SRS FR-3.2).
///
/// [`Instant`]'s epoch is not the kernel's: the deadline is carried over as
/// its distance from now, added to the monotonic clock read right after.
#[cfg(all(feature = "std", any(target_os = "linux", target_os = "android")))]
fn sleep_until(deadline: Instant) {
    use rustix::thread::{ClockId, clock_nanosleep_absolute};
    use rustix::time::{Timespec, clock_gettime};

    let ahead = deadline.saturating_duration_since(Instant::now());
    if ahead.is_zero() {
        return;
    }
    let target = Timespec::try_from(ahead)
        .ok()
        .and_then(|ahead| clock_gettime(ClockId::Monotonic).checked_add(ahead));
    let Some(target) = target else {
        std::thread::sleep(ahead);
        return;
    };
    // A signal handler woke it early: sleep again to the same deadline.
    while clock_nanosleep_absolute(ClockId::Monotonic, &target) == Err(rustix::io::Errno::INTR) {}
}

/// Blocks until `deadline` (no absolute monotonic sleep on this target).
#[cfg(all(feature = "std", not(any(target_os = "linux", target_os = "android"))))]
fn sleep_until(deadline: Instant) {
    std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
}

/// Set once the Ctrl-C handler of the process is the one of the engine.
#[cfg(feature = "std")]
static CTRL_C_WIRED: AtomicBool = AtomicBool::new(false);

/// Treats a failed Ctrl-C install as harmless when the handler in place is
/// ours, from an earlier run of the process. Otherwise hands the error back.
///
/// `wired` is a parameter so the tests do not share the process flag.
#[cfg(feature = "std")]
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

/// Waits `duration` after the hold command, in steps of `dt` on absolute
/// deadlines, so an emergency stop still cuts torque within one step (SRS
/// FR-4.1, FR-4.2). Returns that emergency stop, if any.
fn hold_for<H: Host>(host: &mut H, duration: Duration, dt: Duration) -> Option<Signal> {
    let start = host.now();
    let end = start.checked_add(duration)?;
    let mut deadline = start;
    while deadline < end {
        if let Some(signal) = estop(host.poll()) {
            return Some(signal);
        }
        deadline = deadline.checked_add(dt).map_or(end, |next| next.min(end));
        host.sleep_until(deadline);
    }
    None
}

/// Debug builds: an emergency stop when a member changed an actuator it does
/// not own, logged with both (SRS NFR-2: no panic, torque off on exit).
fn stray_stop(audit: &mut Audit) -> Option<Signal> {
    let stray = audit.take()?;
    error!(
        member = stray.member,
        actuator = stray.actuator,
        "a member changed an actuator it does not own"
    );
    Some(Signal {
        kind: SignalKind::EmergencyStop,
        meta: SignalMeta::new(
            "a member changed an actuator it does not own",
            file!(),
            line!(),
        ),
    })
}

/// [`Drivers::telemetry`] into the state's block, beside the newest slot's
/// faults; the refreshed actuators are stamped with `tick` (SRS FR-5.11).
fn refresh_telemetry<const N: usize, D: Drivers<N>>(
    hal: &mut D,
    state: &mut RobotState<N>,
    layout: &Layout<N>,
    audit: &mut Audit,
    tick: u64,
) {
    let (telemetry, faults) = state.telemetry_mut();
    let mut spare = Faults::default();
    let faults = faults.unwrap_or(&mut spare);
    let refreshed = hal.telemetry(telemetry, faults, layout.slots(), audit);
    telemetry.stamp(refreshed, tick);
}

/// Keeps the signal only if it is an emergency stop: mid-tick checks let a
/// shutdown finish its tick.
fn estop(signal: Option<Signal>) -> Option<Signal> {
    signal.filter(|s| s.kind() == SignalKind::EmergencyStop)
}

/// A driver's error, as the run's.
fn member<E, S>(error: E) -> RunError<HardwareError<E>, S> {
    RunError::Hal(HardwareError::Member(error))
}

#[cfg(feature = "experimental")]
mod presence;

/// Without the `experimental` feature, every actuator is present for the
/// whole run: nothing to copy, cut or keep up.
#[cfg(not(feature = "experimental"))]
mod presence {
    use crate::Signal;
    use crate::hal::Overrides;
    use crate::runtime::{Host, Rig};
    use crate::state::RobotState;

    impl<const N: usize, D, SN> Rig<'_, N, D, SN> {
        /// Nothing to copy.
        #[inline(always)]
        pub(super) fn sync_presence(&self, _state: &mut RobotState<N>) {}

        /// Nothing to cut.
        #[inline(always)]
        pub(super) fn drop_absent(&self, _state: &mut RobotState<N>) {}

        /// No upkeep, no bus check.
        #[inline(always)]
        pub(super) fn upkeep<H: Host>(&mut self, _: Overrides, _host: &mut H) -> Option<Signal> {
            None
        }
    }
}

/// The robot's hardware for one run: the drivers and their layout, the
/// sensor members and their plan.
struct Rig<'a, const N: usize, D, SN> {
    drivers: &'a mut D,
    layout: &'a Layout<N>,
    sensing: &'a mut SN,
    plan: SensorPlan,
}

impl<const N: usize, D: Drivers<N>, SN: Sensing> Rig<'_, N, D, SN> {
    /// One read of every sensor member at `tick`; `sensor missing` raised in
    /// the newest slot when one is missing (SRS FR-5.7, FR-13.4). Returns the
    /// emergency stop `policy` asks for then.
    fn read_sensors(
        &mut self,
        sensors: &mut Sensors,
        state: &mut RobotState<N>,
        tick: u64,
        policy: &SensorPolicy,
    ) -> Option<Signal> {
        sensors.begin_tick(tick);
        self.sensing.read(sensors, &mut self.plan, 0);
        let (missing, stop) = self.plan.judge(sensors, policy);
        if missing {
            state.raise(TickFlags::SENSOR_MISSING);
        }
        stop
    }
}

/// Owns the rig from `open` on: every exit, unwind included, ends with
/// `torque_off` on every driver, then `close` on every driver, then `close`
/// on every sensor member once their `open` was called (SRS FR-5.6, FR-5.7,
/// NFR-2).
struct Powered<'a, const N: usize, D: Drivers<N>, SN: Sensing> {
    rig: Rig<'a, N, D, SN>,
    /// The sensors' `open` was called: they are closed too.
    sensing_open: bool,
}

impl<const N: usize, D: Drivers<N>, SN: Sensing> Drop for Powered<'_, N, D, SN> {
    fn drop(&mut self) {
        self.rig.drivers.torque_off();
        self.rig.drivers.close();
        // DECISION: sensors are closed once their `open` was called, never on
        // an exit before it: nothing of theirs is open then (SRS FR-5.5).
        if self.sensing_open {
            self.rig.sensing.close();
        }
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
    fn discard<const N: usize>(&mut self, state: &mut RobotState<N>, phase: &'static str) {
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
    use crate::logging::trace;

    use crate::controllers::Env;
    use crate::hal::SensorPlan;
    use crate::schedule::Drive;
    use crate::{Robot, RobotState, Sensors};

    /// Installs the tracing subscriber from the [`tracing`] crate.
    ///
    /// It installs the tracing subscriber with the INFO level in release mode,.
    /// DEBUG level in debug mode and TRACE level only and only if the RUST_LOG
    /// environment variable is set.
    ///
    #[cfg(feature = "std")]
    pub(super) fn install_tracing() {
        use tracing::Level;
        use tracing_subscriber::FmtSubscriber;

        let level = if std::env::var("RUST_LOG").is_ok() {
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
    /// Returns owned values for [`RobotState`] and [`Sensors`], the slots of
    /// `plan` declared.
    pub(super) fn pre_init<const N: usize>(plan: &SensorPlan) -> (RobotState<N>, Sensors) {
        trace!("Executing pre-init phase ...");
        // TODO: feature gated:
        // Color_eyre

        let mut sensors = Sensors::new();
        plan.declare(&mut sensors);
        (RobotState::default(), sensors)
    }

    /// Walks the user's `PreInit` controllers.
    pub(super) fn pre_init_controllers<const N: usize, CS: Drive<N>>(
        robot: &mut Robot<N, CS, crate::Filled>,
        state: &mut RobotState<N>,
        sensors: &Sensors,
    ) {
        trace!("Executing user pre-init controllers ...");
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.pre_init(&mut env);
    }

    /// Walks the user's `Init` controllers, once the engine has opened and
    /// zeroed the hardware.
    pub(super) fn init<const N: usize, CS: Drive<N>>(
        robot: &mut Robot<N, CS, crate::Filled>,
        state: &mut RobotState<N>,
        sensors: &Sensors,
    ) {
        trace!("Executing user init controllers ...");
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.init(&mut env);
    }

    /// Walks the user's `PostInit` controllers, once the engine has checked
    /// and configured the hardware, read it in full and turned torque on.
    pub(super) fn post_init<const N: usize, CS: Drive<N>>(
        robot: &mut Robot<N, CS, crate::Filled>,
        state: &mut RobotState<N>,
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
    use crate::controllers::Env;
    use crate::schedule::Drive;
    use crate::{Robot, RobotState, Sensors};

    /// User specified actions to undertake before calling the user defined
    /// controllers.
    pub(super) fn first<const N: usize, CS: Drive<N>>(
        robot: &mut Robot<N, CS, crate::Filled>,
        state: &mut RobotState<N>,
        sensors: &Sensors,
    ) {
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.first(&mut env);
    }

    /// The actual run of all the user defined controllers.
    pub(super) fn each_tick<const N: usize, CS: Drive<N>>(
        robot: &mut Robot<N, CS, crate::Filled>,
        state: &mut RobotState<N>,
        sensors: &Sensors,
    ) {
        // TODO: a tick rate per controller (a divider in the node).
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.each_tick(&mut env);
    }

    /// Actions to undertake after calling the user defined controllers.
    pub(super) fn last<const N: usize, CS: Drive<N>>(
        robot: &mut Robot<N, CS, crate::Filled>,
        state: &mut RobotState<N>,
        sensors: &Sensors,
    ) {
        let mut env = Env::new(&robot.config.time, state, sensors);
        robot.controllers.last(&mut env);
    }
}

/// The tick loop: read, the sensors, the three buckets, write, telemetry.
/// Returns the signal that ended it.
fn control_loop<const N: usize, CS: Drive<N>, H: Host, D: Drivers<N>, SN: Sensing, S: Stages<N>>(
    robot: &mut Robot<N, CS, crate::Filled>,
    state: &mut RobotState<N>,
    sensors: &mut Sensors,
    host: &mut H,
    rig: &mut Rig<'_, N, D, SN>,
    stages: &mut S,
) -> Signal {
    loop {
        if let Some(signal) = host.poll() {
            break signal;
        }
        robot.config.time.begin_tick(host.now());
        let tick = robot.config.time.tick();
        state.set_tick(tick);

        // The faults go to the new slot, where controllers read them (SRS FR-13.4).
        let mut audit = Audit::default();
        if let Some((sample, faults)) = state.push_tick() {
            let read = rig
                .drivers
                .read(sample, faults, rig.layout.slots(), &mut audit);
            state.refreshed(read);
        }
        rig.sync_presence(state);
        let sensor_stop = rig.read_sensors(sensors, state, tick, &robot.config.sensor_policy);
        state.raise(host.take_flags());
        if let Some(signal) = stray_stop(&mut audit) {
            break signal;
        }
        // DECISION: after the stray check: both stop the run, a stray names a
        // bug of the engine's own.
        if let Some(signal) = sensor_stop {
            break signal;
        }
        stages.inbound(state, &robot.config.time);

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
        rig.drop_absent(state);
        // The limit stage, before the batch: an `Abort` breach skips the write.
        let (staged, faults) = state.guarded();
        let mut spare = Faults::default();
        let faults = faults.unwrap_or(&mut spare);
        if let Some(signal) = robot.config.limits.enforce(staged, faults) {
            break signal;
        }

        let mut batch = Batch::new(rig.layout, state.written());
        if !batch.is_empty() {
            let mut faults = state.faults_mut().map(|f| *f).unwrap_or_default();
            rig.drivers.write(
                state.command(),
                &mut batch,
                &mut faults,
                rig.layout.slots(),
                &mut audit,
            );
            if let Some(slot) = state.faults_mut() {
                *slot = faults;
            }
        }
        // Every tick, written or not: the adapter keeps its own cadence.
        refresh_telemetry(rig.drivers, state, rig.layout, &mut audit, tick);
        if let Some(signal) = stray_stop(&mut audit) {
            break signal;
        }
        if let Some(signal) = rig.upkeep(robot.config.overrides, host) {
            break signal;
        }

        // No log per tick (SRS NFR-1): the overrun total is logged at exit.
        let wake = robot.config.time.end_tick(host.now());
        if robot.config.time.overrun().is_some() {
            state.raise(TickFlags::OVERRUN);
        }
        stages.outbound(state, &robot.config.time);

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
fn run_phases<const N: usize, CS: Drive<N>, H: Host, D: Drivers<N>, SN: Sensing, S: Stages<N>>(
    robot: &mut Robot<N, CS, crate::Filled>,
    state: &mut RobotState<N>,
    sensors: &mut Sensors,
    host: &mut H,
    rig: Rig<'_, N, D, SN>,
    stages: &mut S,
) -> Result<Signal, RunError<HardwareError<D::Error>, SN::Error>> {
    let mut writes = StartupWrites::default();

    init_phase::pre_init_controllers(robot, state, sensors);
    writes.discard(state, "PreInit");
    if let Some(signal) = estop(host.poll()) {
        return Ok(signal);
    }

    // Built before `open`, so a failed `open` is closed too.
    let mut powered = Powered {
        rig,
        sensing_open: false,
    };
    powered.rig.drivers.step(Step::Open).map_err(member)?;
    powered.rig.drivers.step(Step::Zero).map_err(member)?;
    // After the actuators' bring-up of Init, before its controllers (SRS FR-5.2).
    powered.sensing_open = true;
    powered.rig.sensing.open().map_err(RunError::Sensor)?;
    init_phase::init(robot, state, sensors);
    writes.discard(state, "Init");
    if let Some(signal) = estop(host.poll()) {
        return Ok(signal);
    }

    let rig = &mut powered.rig;
    rig.drivers.step(Step::Ping).map_err(member)?;
    rig.drivers.step(Step::Health).map_err(member)?;
    rig.drivers.step(Step::Configure).map_err(member)?;
    // The seed: `prev(1)` of tick 0.
    if let Some(sample) = state.push_slot() {
        let mut audit = Audit::default();
        let read = rig
            .drivers
            .read_all(sample, rig.layout.slots(), &mut audit)
            .map_err(member)?;
        state.refreshed(read);
        rig.sync_presence(state);
        let tick = robot.config.time.tick();
        refresh_telemetry(rig.drivers, state, rig.layout, &mut audit, tick);
        // The first reading, for the PostInit controllers (SRS FR-5.7).
        // DECISION: judged like a tick's read: a loss flags the seed slot and
        // its policy applies, before torque on.
        let sensor_stop = rig.read_sensors(sensors, state, tick, &robot.config.sensor_policy);
        if let Some(signal) = stray_stop(&mut audit) {
            return Ok(signal);
        }
        if let Some(signal) = sensor_stop {
            return Ok(signal);
        }
    }
    // A pending signal, a shutdown included, never powers the actuators
    // (SRS FR-4.2).
    if let Some(signal) = host.poll() {
        return Ok(signal);
    }
    rig.drivers.step(Step::TorqueOn).map_err(member)?;
    // An actuator lost during torque on is absent for the PostInit
    // controllers.
    rig.sync_presence(state);
    init_phase::post_init(robot, state, sensors);
    writes.discard(state, "PostInit");
    writes.report();

    // TODO: spawn the `Threaded` controllers (hand-off open in docs/ROADMAP.md).

    // After the bring-up, so tick 0 is not charged with it (SRS FR-5.10).
    robot.config.time.arm(host.now());

    let signal = control_loop(robot, state, sensors, host, rig, stages);
    // An emergency stop raised since the loop ended skips the hold.
    let mut signal = estop(host.poll()).unwrap_or(signal);
    if signal.kind() == SignalKind::Shutdown
        && let Some(duration) = robot.config.hold
        && let Some(target) = state.hold_target()
    {
        rig.drivers.hold(target.command(), rig.layout.slots());
        let dt = *robot.config.time.delta();
        signal = hold_for(host, duration, dt).unwrap_or(signal);
    }
    state.clear_desired();
    Ok(signal)
}

/// Finalization, startup phases, then the tick loop. Returns the signal that
/// ended the run, a configuration fault, a claim mismatch, or a member's
/// error.
pub(crate) fn run_with<const N: usize, CS: Drive<N>, H: Host, D: Drivers<N>>(
    robot: &mut Robot<N, CS, crate::Filled>,
    host: &mut H,
    hal: &mut D,
) -> Result<Signal, RunError<HardwareError<D::Error>>> {
    run_staged(robot, host, hal, &mut ())
}

/// [`run_with`], with `stages` around the controllers of each tick.
pub(crate) fn run_staged<const N: usize, CS: Drive<N>, H: Host, D: Drivers<N>, S: Stages<N>>(
    robot: &mut Robot<N, CS, crate::Filled>,
    host: &mut H,
    hal: &mut D,
    stages: &mut S,
) -> Result<Signal, RunError<HardwareError<D::Error>>> {
    run_all(robot, host, hal, &mut HNil, stages)
}

/// Finalization, startup phases, then the tick loop, on the drivers `hal`
/// and the sensor members `sensing`, with `stages` around the controllers of
/// each tick. Returns the signal that ended the run, a configuration fault, a
/// claim mismatch, or a member's error, a driver's or a sensor's.
pub(crate) fn run_all<
    const N: usize,
    CS: Drive<N>,
    H: Host,
    D: Drivers<N>,
    SN: Sensing,
    S: Stages<N>,
>(
    robot: &mut Robot<N, CS, crate::Filled>,
    host: &mut H,
    hal: &mut D,
    sensing: &mut SN,
    stages: &mut S,
) -> Result<Signal, RunError<HardwareError<D::Error>, SN::Error>> {
    // Flags raised before the run belong to no tick of it.
    let _ = host.take_flags();
    let dt = tick_period(robot.config.dt)?;
    crate::limits::require_policies(&robot.config.limits)?;
    robot.config.time.set_delta(dt);

    let mut plan = Plan::new();
    hal.claim(robot.config.overrides, &mut plan)
        .map_err(RunError::Hal)?;
    let layout = plan.finish().map_err(|c| RunError::Hal(c.into()))?;
    // DECISION: after the drivers' claim, so a run with both faults names the
    // actuators' first, as it did before sensors existed.
    let mut sensor_plan = SensorPlan::new(dt);
    sensing.claim(&mut sensor_plan)?;
    let (mut state, mut sensors) = init_phase::pre_init(&sensor_plan);
    state.set_cadence(layout.read_every);

    let rig = Rig {
        drivers: hal,
        layout: &layout,
        sensing,
        plan: sensor_plan,
    };
    // The guard has dropped by now: torque is off before the log lines.
    let signal = run_phases(robot, &mut state, &mut sensors, host, rig, stages)?;

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
/// Returns the signal that ended the run, a claim mismatch, or a member's
/// error, a driver's or a sensor's.
#[cfg(feature = "std")]
pub(crate) fn runtime<const N: usize, CS: Drive<N>, D: Drivers<N>, SN: Sensing, S: Stages<N>>(
    mut robot: Robot<N, CS, crate::Filled>,
    mut hal: D,
    mut sensing: SN,
    mut stages: S,
) -> Result<Signal, RunError<HardwareError<D::Error>, SN::Error>> {
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

    run_all(
        &mut robot,
        &mut RealHost::new(),
        &mut hal,
        &mut sensing,
        &mut stages,
    )
}

#[cfg(test)]
mod tests {
    use std::cell::{Cell, Ref, RefCell};
    use std::num::NonZeroU32;
    use std::sync::atomic::AtomicU32;

    use super::*;
    use crate::clock::Time;
    use crate::hal::{
        Batch, Claim, Command, Coverage, Faults, Group, Hal, Layout, Mask, MemberError, NoHardware,
        Overrides, Sample, Written, seal,
    };
    use crate::signal::SignalMeta;
    use crate::state::{N, Q, Qd};
    use crate::test_util::*;
    use crate::{EachTick, First, Init, Last, PostInit, PreInit};
    use crate::{HCons, HNil};

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

    /// The list of the one member `head`.
    fn one<H>(head: H) -> HCons<H, HNil> {
        HCons { head, tail: HNil }
    }

    /// A run on [`NoHardware`], which cannot fail.
    fn simulated<CS: Drive<N>, H: Host>(
        robot: &mut Robot<N, CS, crate::Filled>,
        host: &mut H,
    ) -> Signal {
        run_with(robot, host, &mut one(NoHardware)).expect("NoHardware owns every actuator")
    }

    fn bump(cell: &Cell<u32>) -> impl Fn(&Time, &mut RobotState<N>, &Sensors) + '_ {
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
    fn shutdown_during_startup_stops_before_post_init_and_runs_no_tick() {
        let walks = Walks::default();
        let mut robot = counting_robot!(walks);
        let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);

        let signal = simulated(&mut robot, &mut host);

        assert_eq!(signal.kind(), SignalKind::Shutdown);
        assert_eq!(walks.counts(), [1, 1, 0, 0, 0, 0]);
        assert_eq!(host.poll_calls, BEFORE_TORQUE_ON + 1);
        assert_eq!(robot.config.time.deadline(), None);
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
        let mut host = RealHost::new();
        let deadline = host.now() + Duration::from_millis(2);

        host.sleep_until(deadline);
        assert!(host.now() >= deadline);

        // A deadline already behind must not block, nor panic.
        host.sleep_until(deadline);
    }

    #[test]
    fn the_spin_tail_still_wakes_at_the_deadline_never_before() {
        let mut host = RealHost {
            spin_tail: Duration::from_micros(300),
        };
        for _ in 0..5 {
            let deadline = host.now() + Duration::from_millis(1);
            host.sleep_until(deadline);
            assert!(host.now() >= deadline);
        }
    }

    #[test]
    fn a_long_sleep_ends_near_its_deadline() {
        // Upper bound loose enough for a loaded CI host: the absolute sleep
        // must not drift by the length of the wait.
        let mut host = RealHost::new();
        let deadline = host.now() + Duration::from_millis(20);
        host.sleep_until(deadline);
        let late = host.now().saturating_duration_since(deadline);
        assert!(late < Duration::from_millis(15), "{late:?}");
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
    fn an_unset_dt_falls_back_before_any_controller_runs() {
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

        // Named, not a glob: they shadow the schedule labels of `super::*`.
        use Call::{
            Check, Close, Configure, EachTick, First, Health, Hold, Inbound, Init, Last, Open,
            Outbound, Ping, PostInit, PreInit, Read, ReadAll, TorqueOff, TorqueOn, Write, Zero,
        };

        /// The run error of a one-member list whose recorder failed `call`.
        fn failed(
            call: Call,
        ) -> RunError<HardwareError<MemberError<Failed, core::convert::Infallible>>> {
            RunError::Hal(HardwareError::Member(MemberError::Head {
                name: core::any::type_name::<Recorder<'static, N>>(),
                error: Failed(call),
            }))
        }

        fn walk(log: &Log<N>, call: Call) -> impl Fn(&Time, &mut RobotState<N>, &Sensors) + '_ {
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
        fn a_dt_out_of_range_is_refused_before_any_io() {
            for dt in [
                Duration::ZERO,
                Duration::from_micros(5),
                Duration::from_secs(2),
            ] {
                let log = Log::new();
                let mut robot = recording_robot!(log).set_dt(dt);
                let mut host = Scripted::new(Instant::now());

                let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

                assert_eq!(
                    result.err(),
                    Some(RunError::Config(ConfigError::DtOutOfRange { dt }))
                );
                assert!(log.calls().is_empty(), "{dt:?}");
                assert_eq!(host.poll_calls, 0);
            }
        }

        #[test]
        fn the_bounds_of_the_dt_range_are_accepted() {
            for dt in [DT_MIN, DT_MAX] {
                let log = Log::new();
                let mut robot = recording_robot!(log).set_dt(dt);
                let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);

                let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

                assert!(result.is_ok(), "{dt:?}");
            }
        }

        #[test]
        fn a_run_without_hardware_has_no_sample_and_no_age() {
            let seen = Cell::new(None);
            let mut robot = Robot::new().set_dt(DT).add_controller(
                crate::EachTick,
                |_t, s: &mut RobotState<N>, _se| {
                    seen.set(Some((s.prev::<Q>(0).err(), s.age(0), s.sample_age())));
                },
            );
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + PER_TICK);

            simulated(&mut robot, &mut host);

            assert_eq!(
                seen.take(),
                Some((
                    Some(crate::error::StateError::NoSample { depth: 0 }),
                    Ok(None),
                    None
                ))
            );
        }

        #[test]
        fn an_actuator_read_every_third_tick_ages_between_reads() {
            let log = Log::new();
            let ages = RefCell::new([None; 6]);
            let tick = Cell::new(0_usize);
            let mut robot = Robot::new().set_dt(DT).add_controller(
                crate::First,
                |_t, s: &mut RobotState<N>, _se| {
                    if let Some(age) = ages.borrow_mut().get_mut(tick.get()) {
                        *age = s.age(0).ok().flatten();
                    }
                    tick.set(tick.get() + 1);
                },
            );
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 5 * PER_TICK);
            let mut recorder = Recorder {
                every: NonZeroU32::new(3).expect("3 is not zero"),
                ..Recorder::new(&log)
            };

            run_with(&mut robot, &mut host, &mut one(&mut recorder)).expect("recorder");

            assert_eq!(
                *ages.borrow(),
                [Some(0), Some(1), Some(2), Some(0), Some(1), Some(2)]
            );
        }

        #[test]
        fn a_check_error_is_returned_before_any_io() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now());

            let result = run_with(
                &mut robot,
                &mut host,
                &mut one(Recorder::failing(&log, Check)),
            );

            assert_eq!(result.err(), Some(failed(Check)));
            assert_eq!(*log.calls(), [Check]);
            assert_eq!(host.poll_calls, 0);
        }

        #[test]
        fn a_coverage_gap_is_returned_before_any_io() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now());
            let mut owned = Mask::ALL;
            owned.clear(1);
            let mut recorder = Recorder {
                owned,
                ..Recorder::new(&log)
            };

            let result = run_with(&mut robot, &mut host, &mut one(&mut recorder));

            assert_eq!(
                result.err(),
                Some(RunError::Hal(HardwareError::Coverage(Coverage::Unowned {
                    actuator: 1
                })))
            );
            assert_eq!(*log.calls(), [Check]);
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

            let result = run_with(
                &mut robot,
                &mut host,
                &mut one(Recorder::failing(&log, Open)),
            );

            assert_eq!(result.err(), Some(failed(Open)));
            assert_eq!(*log.calls(), [Check, PreInit, Open, TorqueOff, Close]);
        }

        #[test]
        fn a_ping_error_skips_post_init_cuts_torque_and_is_returned() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now());

            let result = run_with(
                &mut robot,
                &mut host,
                &mut one(Recorder::failing(&log, Ping)),
            );

            assert_eq!(result.err(), Some(failed(Ping)));
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
                &mut one(Recorder::failing(&log, TorqueOn)),
            );

            assert_eq!(result.err(), Some(failed(TorqueOn)));
            assert_eq!(log.calls()[..10], STARTUP[..10]);
            assert_eq!(log.calls()[10..], [TorqueOff, Close]);
        }

        #[test]
        fn an_estop_in_pre_init_touches_no_hardware_after_check() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).estop_from(AFTER_PRE_INIT);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(*log.calls(), [Check, PreInit]);
        }

        #[test]
        fn an_estop_in_init_after_open_cuts_torque_and_closes() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).estop_from(AFTER_INIT);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

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

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(log.calls()[..9], STARTUP[..9]);
            assert_eq!(log.calls()[9..], [TorqueOff, Close]);
        }

        #[test]
        fn a_shutdown_in_post_init_never_powers() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).shutdown_from(BEFORE_TORQUE_ON);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(log.calls()[..9], STARTUP[..9]);
            assert_eq!(log.calls()[9..], [TorqueOff, Close]);
            assert_eq!(log.hold.get(), None);
        }

        #[test]
        fn a_shutdown_before_the_first_tick_holds_the_seed() {
            let log = Log::new();
            let mut robot = recording_robot!(log).set_hold(Duration::ZERO);
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_START);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(log.calls()[..11], STARTUP);
            assert_eq!(log.calls()[11..], [Hold, TorqueOff, Close]);
            let hold = log.hold.get().expect("a hold");
            assert_eq!(hold.q, [SEED; N]);
            assert_eq!(hold.written, Written::default());
        }

        #[test]
        fn a_shutdown_without_a_hold_cuts_torque_at_once() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_START);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(log.calls()[11..], [TorqueOff, Close]);
            assert_eq!(log.hold.get(), None);
        }

        #[test]
        fn a_hold_lasts_its_duration_in_tick_steps() {
            let log = Log::new();
            let mut robot = recording_robot!(log).set_hold(DT * 5 / 2);
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_START);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(log.calls()[11..], [Hold, TorqueOff, Close]);
            assert_eq!(host.sleep_calls, 3);
            assert_eq!(
                host.sleeps[..3],
                [Some(host.at(10)), Some(host.at(20)), Some(host.at(25))]
            );
        }

        #[test]
        fn an_emergency_stop_during_the_hold_cuts_torque_at_once() {
            let log = Log::new();
            let mut robot = recording_robot!(log).set_hold(DT * 10);
            // Check 4 follows the loop; the hold polls from check 5.
            let mut host = Scripted::new(Instant::now())
                .shutdown_from(TICK_START)
                .estop_from(TICK_START + 3);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(log.calls()[11..], [Hold, TorqueOff, Close]);
            assert_eq!(host.sleep_calls, 1);
        }

        #[test]
        fn startup_writes_never_reach_the_adapter() {
            static WARNINGS: AtomicU32 = AtomicU32::new(0);
            let log = Log::new();
            let mut robot = recording_robot!(log)
                .set_hold(Duration::ZERO)
                .add_controller(crate::PreInit, |_t, state, _se| {
                    state.set_all([Q(99.0); N]);
                })
                .add_controller(crate::PostInit, |_t, state, _se| {
                    state.set_at(Q(98.0), 1).expect("in range");
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_START);
            let counter = Counter::new("discarded", &WARNINGS);

            let result = tracing::subscriber::with_default(counter, || {
                run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)))
            });

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert!(!log.calls().contains(&Write));
            let hold = log.hold.get().expect("a hold");
            assert_eq!(hold.q, [SEED; N]);
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
                run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)))
            }));

            assert!(unwound.is_err());
            assert_eq!(log.calls()[..11], STARTUP);
            assert_eq!(log.calls()[11..], [Read, First, EachTick, TorqueOff, Close]);
        }

        /// The mask of `indices`.
        fn mask(indices: &[usize]) -> Mask<N> {
            let mut mask = Mask::NONE;
            for &index in indices {
                mask.set(index);
            }
            mask
        }

        #[test]
        fn shutdown_reads_before_first_writes_after_last_then_holds_and_cuts_torque() {
            let log = Log::new();
            let mut robot = recording_robot!(log)
                .set_hold(Duration::ZERO)
                .add_controller(crate::EachTick, |_t, state, _se| {
                    state.set_at(Q(5.0), 1).expect("in range");
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

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
            let mut robot = recording_robot!(log)
                .set_hold(Duration::ZERO)
                .add_controller(crate::EachTick, |time, state, _se| {
                    if time.tick() == 0 {
                        state.set_at(Q(5.0), 1).expect("in range");
                    }
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST + 2 * PER_TICK);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

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

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

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

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

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

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

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

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

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

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

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

        #[test]
        fn controllers_read_the_cadence_the_adapter_claimed() {
            let log = Log::new();
            let seen = Cell::new(None);
            let mut robot = Robot::new().set_dt(DT).add_controller(
                crate::PreInit,
                |_: &Time, state: &mut RobotState<N>, _: &Sensors| {
                    seen.set(state.cadence(0).ok());
                },
            );
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);
            let mut recorder = Recorder::new(&log);
            recorder.every = NonZeroU32::new(3).expect("3 is not zero");

            let result = run_with(&mut robot, &mut host, &mut one(recorder));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(seen.get().map(NonZeroU32::get), Some(3));
        }

        /// Logs each stage into the shared log.
        struct Staged<'a>(&'a Log<N>);

        impl Stages<N> for Staged<'_> {
            fn inbound(&mut self, state: &mut RobotState<N>, _time: &Time) {
                self.0.push(Inbound);
                state.set_at(Q(7.0), 2).expect("in range");
            }

            fn outbound(&mut self, state: &RobotState<N>, _time: &Time) {
                self.0.push(Outbound);
                assert!(state.faults(0).is_ok());
            }
        }

        #[test]
        fn stages_run_after_read_before_first_and_after_write() {
            let log = Log::new();
            let mut robot = recording_robot!(log);
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);

            let result = run_staged(
                &mut robot,
                &mut host,
                &mut one(Recorder::new(&log)),
                &mut Staged(&log),
            );

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(log.calls()[..11], STARTUP);
            assert_eq!(
                log.calls()[11..],
                [
                    Read, Inbound, First, EachTick, Last, Write, Outbound, TorqueOff, Close
                ]
            );
            // The inbound write reaches the tick's command.
            assert_eq!(log.write.get().map(|w| w.q[2]), Some(Q(7.0)));
        }

        #[test]
        fn telemetry_runs_after_read_all_then_once_per_tick_after_the_write() {
            let log = Log::new();
            let mut robot =
                recording_robot!(log).add_controller(crate::EachTick, |time, state, _se| {
                    if time.tick() == 0 {
                        state.set_at(Q(5.0), 1).expect("in range");
                    }
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST + PER_TICK);
            let recorder = Recorder {
                temperature: Some(40.0),
                ..Recorder::new(&log)
            };

            let result = run_with(&mut robot, &mut host, &mut one(recorder));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(
                *log.calls(),
                [
                    Check,
                    PreInit,
                    Open,
                    Zero,
                    Init,
                    Ping,
                    Health,
                    Configure,
                    ReadAll,
                    Call::Telemetry,
                    TorqueOn,
                    PostInit,
                    Read,
                    First,
                    EachTick,
                    Last,
                    Write,
                    Call::Telemetry, // tick 0 wrote
                    Read,
                    First,
                    EachTick,
                    Last,
                    Call::Telemetry, // nothing written
                    TorqueOff,
                    Close,
                ]
            );
        }

        #[test]
        fn a_reading_shows_from_the_next_tick_on_with_its_age() {
            let log = Log::new();
            // (temperature, age) of actuator 0, seen in `First` of each tick.
            let seen = Cell::new([(None, None); 3]);
            let mut robot = Robot::new().set_dt(DT).add_controller(
                crate::First,
                |time: &Time, state: &mut RobotState<N>, _: &Sensors| {
                    let mut all = seen.get();
                    let telemetry = state.telemetry();
                    if let Some(slot) = all.get_mut(time.tick() as usize) {
                        *slot = (
                            telemetry.temperature_c(0).ok().flatten(),
                            telemetry.age(0).ok().flatten(),
                        );
                    }
                    seen.set(all);
                },
            );
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST + 2 * PER_TICK);
            let recorder = Recorder {
                temperature: Some(40.0),
                ..Recorder::new(&log)
            };

            run_with(&mut robot, &mut host, &mut one(recorder)).expect("recorder");

            // PostInit reads 40, stamped tick 0; tick k reads 41 + k after its write.
            assert_eq!(
                seen.get(),
                [
                    (Some(40.0), Some(0)),
                    (Some(41.0), Some(1)),
                    (Some(42.0), Some(1))
                ]
            );
        }

        #[test]
        fn the_default_telemetry_leaves_every_reading_none() {
            let seen = Cell::new(None);
            let mut robot = Robot::new().set_dt(DT).add_controller(
                crate::EachTick,
                |_t, state: &mut RobotState<N>, _se| {
                    seen.set(Some(*state.telemetry()));
                },
            );
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + PER_TICK);

            simulated(&mut robot, &mut host);

            let telemetry = seen.get().expect("a tick ran");
            for actuator in 0..N {
                assert_eq!(telemetry.temperature_c(actuator), Ok(None));
                assert_eq!(telemetry.voltage_v(actuator), Ok(None));
                assert_eq!(telemetry.hardware_error(actuator), Ok(None));
                assert_eq!(telemetry.age(actuator), Ok(None));
            }
        }

        const LIMIT: crate::limits::Limit = crate::limits::Limit {
            lower: Some(Q(-1.0)),
            upper: Some(Q(1.0)),
            speed: Qd(1.0),
            effort: crate::state::Tau(1.0),
        };

        #[test]
        fn a_limit_without_a_policy_is_refused_before_any_io() {
            let log = Log::new();
            let mut policies = [Some(crate::limits::Policy::Clamp); N];
            policies[1] = None;
            let mut robot = recording_robot!(log).set_limits([LIMIT; N], policies);
            let mut host = Scripted::new(Instant::now());

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert!(matches!(
                result,
                Err(RunError::Config(ConfigError::NoPolicy { actuator: 1 }))
            ));
            assert!(log.calls().is_empty());
        }

        #[test]
        fn clamp_sends_the_bound_and_flags_the_tick() {
            let log = Log::new();
            let seen = Cell::new(Mask::NONE);
            let mut robot = recording_robot!(log)
                .set_limits([LIMIT; N], [Some(crate::limits::Policy::Clamp); N])
                .add_controller(crate::EachTick, |time, state, _se| {
                    if time.tick() == 0 {
                        state.set_at(Q(5.0), 1).expect("in range");
                    } else if let Ok(faults) = state.faults(1) {
                        seen.set(faults.joint_limit);
                    }
                });
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + PER_TICK);

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            let write = log.write.get().expect("a write");
            assert_eq!(write.q[1], Q(1.0));
            assert_eq!(seen.get(), mask(&[1]));
        }

        #[test]
        fn abort_writes_nothing_then_cuts_torque() {
            let log = Log::new();
            let mut robot = recording_robot!(log)
                .set_hold(Duration::ZERO)
                .set_limits([LIMIT; N], [Some(crate::limits::Policy::Abort); N])
                .add_controller(crate::EachTick, |_t, state, _se| {
                    state.set_at(Q(5.0), 0).expect("in range");
                });
            let mut host = Scripted::new(Instant::now());

            let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(
                log.calls()[11..],
                [Read, First, EachTick, Last, TorqueOff, Close]
            );
        }

        #[test]
        fn the_signal_that_ended_the_run_is_logged_once_at_exit() {
            static STOPPED: AtomicU32 = AtomicU32::new(0);
            static ABORTED: AtomicU32 = AtomicU32::new(0);
            let runs = [
                (
                    Scripted::new(Instant::now()).shutdown_from(TICK_END + 2 * PER_TICK),
                    tracing::Level::INFO,
                    "control loop stopped",
                    &STOPPED,
                ),
                (
                    Scripted::new(Instant::now()).estop_from(AFTER_FIRST + PER_TICK),
                    tracing::Level::ERROR,
                    "control loop aborted",
                    &ABORTED,
                ),
            ];
            for (mut host, level, needle, count) in runs {
                let log = Log::new();
                let mut robot = recording_robot!(log);
                let counter = Counter::new(needle, count).at(level);

                let result = tracing::subscriber::with_default(counter, || {
                    run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)))
                });

                assert!(result.is_ok());
            }

            assert_eq!(STOPPED.load(Ordering::Relaxed), 1);
            assert_eq!(ABORTED.load(Ordering::Relaxed), 1);
        }

        #[test]
        fn the_tick_loop_logs_nothing_per_tick() {
            static ONE_TICK: AtomicU32 = AtomicU32::new(0);
            static FIVE_TICKS: AtomicU32 = AtomicU32::new(0);
            for (ticks, events) in [(1, &ONE_TICK), (5, &FIVE_TICKS)] {
                let log = Log::new();
                let mut robot = recording_robot!(log);
                let stop = TICK_END + (ticks - 1) * PER_TICK;
                let mut host = Scripted::new(Instant::now()).shutdown_from(stop);
                let every = Events::new(events);

                tracing::subscriber::with_default(every, || simulated(&mut robot, &mut host));
            }

            assert_eq!(
                ONE_TICK.load(Ordering::Relaxed),
                FIVE_TICKS.load(Ordering::Relaxed)
            );
        }

        /// Counts every event, at every level.
        struct Events(&'static AtomicU32);

        impl Events {
            fn new(count: &'static AtomicU32) -> Self {
                // Installs the silent global subscriber.
                let _ = Counter::new("", count);
                Self(count)
            }
        }

        impl tracing::Subscriber for Events {
            fn enabled(&self, _metadata: &tracing::Metadata<'_>) -> bool {
                true
            }

            fn event(&self, _event: &tracing::Event<'_>) {
                self.0.fetch_add(1, Ordering::Relaxed);
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

        /// Counts the warnings whose message holds `needle`, drops every
        /// other event. No text is stored.
        struct Counter {
            needle: &'static str,
            count: &'static AtomicU32,
            level: tracing::Level,
        }

        impl Counter {
            fn new(needle: &'static str, count: &'static AtomicU32) -> Self {
                static SILENT: std::sync::Once = std::sync::Once::new();
                SILENT.call_once(|| {
                    let _ = tracing::subscriber::set_global_default(Silent);
                });
                Self {
                    needle,
                    count,
                    level: tracing::Level::WARN,
                }
            }

            /// Counts the events of `level` instead of the warnings.
            fn at(self, level: tracing::Level) -> Self {
                Self { level, ..self }
            }
        }

        /// The process-wide subscriber of the tests: prints nothing, but asks
        /// each time. Tracing caches per log line whether anyone listens; a
        /// line first hit on a thread without a `Counter` while another
        /// thread installs one could cache "never" and hide the counted
        /// warning.
        struct Silent;

        impl tracing::Subscriber for Silent {
            fn register_callsite(
                &self,
                _metadata: &'static tracing::Metadata<'static>,
            ) -> tracing::subscriber::Interest {
                tracing::subscriber::Interest::sometimes()
            }

            fn enabled(&self, _metadata: &tracing::Metadata<'_>) -> bool {
                false
            }

            fn event(&self, _event: &tracing::Event<'_>) {}

            fn new_span(&self, _span: &tracing::span::Attributes<'_>) -> tracing::span::Id {
                tracing::span::Id::from_u64(1)
            }
            fn record(&self, _span: &tracing::span::Id, _values: &tracing::span::Record<'_>) {}
            fn record_follows_from(&self, _span: &tracing::span::Id, _follows: &tracing::span::Id) {
            }
            fn enter(&self, _span: &tracing::span::Id) {}
            fn exit(&self, _span: &tracing::span::Id) {}
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
                *metadata.level() == self.level
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

        mod sensing {
            //! The sensor members of a run: open, read, missing, close (SRS
            //! FR-5.7, FR-13.4).

            use core::any::type_name;
            use core::convert::Infallible;

            use super::*;
            use crate::hal::{Sensing, SensorPolicy};
            use crate::sensors::{Accel, Gyro};
            use crate::test_util::{FakeSensor, FakeSensorError, Reading};

            // Named, not a glob: they shadow the schedule labels of `super::*`.
            use Call::{
                Check, Close, Configure, EachTick, First, Health, Hold, Init, Last, Open, Ping,
                PostInit, PreInit, Read, ReadAll, SensorClose, SensorOpen, SensorRead, TorqueOff,
                TorqueOn, Zero,
            };

            /// Every bring-up call with one sensor of id 0, PostInit's walk
            /// included.
            const STARTUP: [Call; 13] = [
                Check,
                PreInit,
                Open,
                Zero,
                SensorOpen(0),
                Init,
                Ping,
                Health,
                Configure,
                ReadAll,
                SensorRead(0),
                TorqueOn,
                PostInit,
            ];

            /// The run error of a recorder and the sensors `SN`.
            type Failure<SN> =
                RunError<HardwareError<MemberError<Failed, Infallible>>, <SN as Sensing>::Error>;

            /// Runs `robot` on one recorder logging into `log`, and on
            /// `sensing`.
            fn run_sensed<CS: Drive<N>, SN: Sensing>(
                robot: &mut Robot<N, CS, crate::Filled>,
                host: &mut Scripted,
                log: &Log<N>,
                sensing: &mut SN,
            ) -> Result<Signal, Failure<SN>> {
                run_all(robot, host, &mut one(Recorder::new(log)), sensing, &mut ())
            }

            /// A reading of `x` along x, and nothing else.
            fn along_x(x: f32) -> Reading<'static> {
                Reading {
                    accel: Some(Accel([x, 0.0, 0.0])),
                    ..Reading::QUIET
                }
            }

            /// Whether the tick's own slot raises `sensor missing`.
            fn missing(state: &RobotState<N>) -> bool {
                state
                    .tick_flags(0)
                    .is_ok_and(|flags| flags.contains(TickFlags::SENSOR_MISSING))
            }

            /// What `First` saw on each of `T` ticks: the age of IMU 0 and
            /// the missing flag.
            type Watched<const T: usize> = [(Option<u64>, bool); T];

            /// Runs `T` ticks of a robot on `sensing` and `policy`.
            fn watch<const T: usize, SN: Sensing>(
                log: &Log<N>,
                sensing: &mut SN,
                policy: SensorPolicy,
            ) -> (Result<SignalKind, Failure<SN>>, Watched<T>) {
                let seen = RefCell::new([(None, false); T]);
                let mut robot = Robot::new()
                    .set_dt(DT)
                    .set_sensor_policy(policy)
                    .add_controller(
                        crate::First,
                        |time: &Time, state: &mut RobotState<N>, sensors: &Sensors| {
                            let tick = usize::try_from(time.tick()).expect("small");
                            if let Some(slot) = seen.borrow_mut().get_mut(tick) {
                                *slot = (sensors.imu(0).map(|imu| imu.age()), missing(state));
                            }
                        },
                    );
                let mut host =
                    Scripted::new(Instant::now()).shutdown_from(TICK_END + (T - 1) * PER_TICK);
                let result = run_sensed(&mut robot, &mut host, log, sensing);
                (result.map(|s| s.kind()), seen.into_inner())
            }

            #[test]
            fn sensors_open_in_init_after_the_actuators() {
                let log = Log::new();
                let mut robot = recording_robot!(log);
                let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_START);
                let mut sensing = one(FakeSensor::new(&log, 0));

                let result = run_sensed(&mut robot, &mut host, &log, &mut sensing);

                assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
                assert_eq!(log.calls()[..13], STARTUP);
                assert_eq!(log.calls()[2..6], [Open, Zero, SensorOpen(0), Init]);
            }

            #[test]
            fn a_sensor_error_at_open_ends_the_run_before_torque_on() {
                let log = Log::new();
                let mut robot = recording_robot!(log);
                let mut host = Scripted::new(Instant::now());
                let mut sensing = one(FakeSensor {
                    fail_open: true,
                    ..FakeSensor::new(&log, 0)
                });

                let result = run_sensed(&mut robot, &mut host, &log, &mut sensing);

                assert_eq!(
                    result.err(),
                    Some(RunError::Sensor(MemberError::Head {
                        name: type_name::<FakeSensor<'static, N>>(),
                        error: FakeSensorError::Open { id: 0 },
                    }))
                );
                assert_eq!(
                    *log.calls(),
                    [
                        Check,
                        PreInit,
                        Open,
                        Zero,
                        SensorOpen(0),
                        TorqueOff,
                        Close,
                        SensorClose(0)
                    ]
                );
            }

            #[test]
            fn a_reading_is_visible_in_first_of_the_same_tick() {
                let log = Log::new();
                // Read call k reads k along x: call 0 is PostInit's, call t + 1 tick t's.
                let script: [Reading<'static>; 4] = core::array::from_fn(|k| along_x(k as f32));
                let mut sensing = one(FakeSensor {
                    script: &script,
                    ..FakeSensor::new(&log, 0)
                });
                let seen = Cell::new([None; 3]);
                let mut robot = Robot::new().set_dt(DT).add_controller(
                    crate::First,
                    |time: &Time, _: &mut RobotState<N>, sensors: &Sensors| {
                        let mut all = seen.get();
                        if let Some(slot) = all.get_mut(time.tick() as usize) {
                            *slot = sensors
                                .imu(0)
                                .map(|imu| (imu.accel().map(Accel::x), imu.age()));
                        }
                        seen.set(all);
                    },
                );
                let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 2 * PER_TICK);

                let result = run_sensed(&mut robot, &mut host, &log, &mut sensing);

                assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
                assert_eq!(
                    seen.get(),
                    [
                        Some((Some(1.0), 0)),
                        Some((Some(2.0), 0)),
                        Some((Some(3.0), 0))
                    ]
                );
            }

            #[test]
            fn post_init_controllers_see_the_first_reading() {
                let log = Log::new();
                let script = [Reading {
                    accel: Some(Accel([1.0, 2.0, 3.0])),
                    gyro: Some(Gyro([0.1, 0.0, 0.0])),
                    ..Reading::QUIET
                }];
                let mut sensing = one(FakeSensor {
                    script: &script,
                    ..FakeSensor::new(&log, 0)
                });
                let seen = Cell::new(None);
                let mut robot = Robot::new().set_dt(DT).add_controller(
                    crate::PostInit,
                    |_: &Time, _: &mut RobotState<N>, sensors: &Sensors| {
                        seen.set(
                            sensors
                                .imu(0)
                                .map(|imu| (imu.accel(), imu.gyro(), imu.age())),
                        );
                    },
                );
                let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_START);

                let result = run_sensed(&mut robot, &mut host, &log, &mut sensing);

                assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
                assert_eq!(
                    seen.get(),
                    Some((Some(Accel([1.0, 2.0, 3.0])), Some(Gyro([0.1, 0.0, 0.0])), 0))
                );
            }

            #[test]
            fn a_slow_sensor_is_not_missing() {
                let log = Log::new();
                // A reading in ticks 0, 10, 20; PostInit's read gets none.
                let script: [Reading<'static>; 31] = core::array::from_fn(|k| {
                    if k % 10 == 1 {
                        along_x(1.0)
                    } else {
                        Reading::QUIET
                    }
                });
                let mut sensing = one(FakeSensor {
                    max_silence: DT * 20,
                    script: &script,
                    ..FakeSensor::new(&log, 0)
                });

                let (result, seen) = watch::<30, _>(&log, &mut sensing, SensorPolicy::Flag);

                assert_eq!(result, Ok(SignalKind::Shutdown));
                for (tick, (age, missing)) in seen.into_iter().enumerate() {
                    assert_eq!(age, Some(tick as u64 % 10), "tick {tick}");
                    assert!(!missing, "tick {tick}");
                }
            }

            #[test]
            fn silence_past_max_silence_raises_sensor_missing() {
                let log = Log::new();
                // Readings up to tick 2, then silence; the limit is 3 ticks.
                let script = [along_x(1.0); 4];
                let mut sensing = one(FakeSensor {
                    max_silence: DT * 3,
                    script: &script,
                    ..FakeSensor::new(&log, 0)
                });

                let (result, seen) = watch::<9, _>(&log, &mut sensing, SensorPolicy::Flag);

                assert_eq!(result, Ok(SignalKind::Shutdown));
                assert_eq!(
                    seen.map(|(_, missing)| missing),
                    [false, false, false, false, false, false, true, true, true]
                );
                assert_eq!(seen[8].0, Some(6));
            }

            #[test]
            fn link_down_raises_sensor_missing_that_tick() {
                let log = Log::new();
                let mut script = [along_x(1.0); 6];
                // Read call 4: tick 3.
                script[4].link_down = true;
                let mut sensing = one(FakeSensor {
                    script: &script,
                    ..FakeSensor::new(&log, 0)
                });

                let (result, seen) = watch::<5, _>(&log, &mut sensing, SensorPolicy::Flag);

                assert_eq!(result, Ok(SignalKind::Shutdown));
                assert_eq!(
                    seen.map(|(_, missing)| missing),
                    [false, false, false, true, false]
                );
            }

            #[test]
            fn sensors_close_after_the_drivers_on_shutdown() {
                let log = Log::new();
                let mut robot = recording_robot!(log).set_hold(Duration::ZERO);
                let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST);
                let mut sensing = one(FakeSensor::new(&log, 0));

                let result = run_sensed(&mut robot, &mut host, &log, &mut sensing);

                assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
                assert_eq!(log.calls()[..13], STARTUP);
                assert_eq!(
                    log.calls()[13..],
                    [
                        Read,
                        SensorRead(0),
                        First,
                        EachTick,
                        Last,
                        Hold,
                        TorqueOff,
                        Close,
                        SensorClose(0)
                    ]
                );
            }

            #[test]
            fn sensors_close_when_a_controller_panics() {
                let log = Log::new();
                let mut robot =
                    recording_robot!(log).add_controller(crate::EachTick, |_t, _s, _se| {
                        panic!("a controller bug");
                    });
                let mut host = Scripted::new(Instant::now());
                let mut sensing = one(FakeSensor::new(&log, 0));

                let unwound = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    run_sensed(&mut robot, &mut host, &log, &mut sensing)
                }));

                assert!(unwound.is_err());
                assert_eq!(log.calls()[..13], STARTUP);
                assert_eq!(
                    log.calls()[13..],
                    [
                        Read,
                        SensorRead(0),
                        First,
                        EachTick,
                        TorqueOff,
                        Close,
                        SensorClose(0)
                    ]
                );
            }

            #[test]
            fn a_run_without_sensors_never_raises_sensor_missing() {
                let log = Log::new();
                let flagged = Cell::new(false);
                let seen_a_sensor = Cell::new(false);
                let mut robot = recording_robot!(log).add_controller(
                    crate::First,
                    |_: &Time, state: &mut RobotState<N>, sensors: &Sensors| {
                        flagged.set(flagged.get() || missing(state));
                        let any = sensors.imu(0).is_some() || sensors.mic(0).is_some();
                        seen_a_sensor.set(seen_a_sensor.get() || any);
                    },
                );
                let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 4 * PER_TICK);

                let result = run_with(&mut robot, &mut host, &mut one(Recorder::new(&log)));

                assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
                assert!(!flagged.get());
                assert!(!seen_a_sensor.get());
                let sensor_call =
                    |call: &Call| matches!(call, SensorOpen(_) | SensorRead(_) | SensorClose(_));
                assert!(!log.calls().iter().any(sensor_call));
            }

            #[test]
            fn the_default_policy_only_flags() {
                static WARNINGS: AtomicU32 = AtomicU32::new(0);
                let log = Log::new();
                let mut sensing = one(FakeSensor {
                    max_silence: DT,
                    ..FakeSensor::new(&log, 0)
                });
                let counter = Counter::new("a sensor is missing", &WARNINGS);

                let (result, seen) = tracing::subscriber::with_default(counter, || {
                    watch::<5, _>(&log, &mut sensing, SensorPolicy::default())
                });

                assert_eq!(result, Ok(SignalKind::Shutdown));
                assert_eq!(
                    seen.map(|(_, missing)| missing),
                    [false, false, true, true, true]
                );
                assert_eq!(WARNINGS.load(Ordering::Relaxed), 0);
            }

            #[test]
            fn policy_warn_logs_once_per_member() {
                static WARNINGS: AtomicU32 = AtomicU32::new(0);
                let log = Log::new();
                let mut sensing = HCons {
                    head: FakeSensor {
                        max_silence: DT,
                        ..FakeSensor::new(&log, 0)
                    },
                    tail: one(FakeSensor {
                        max_silence: DT,
                        ..FakeSensor::new(&log, 1)
                    }),
                };
                let counter = Counter::new("a sensor is missing", &WARNINGS);

                let (result, seen) = tracing::subscriber::with_default(counter, || {
                    watch::<6, _>(&log, &mut sensing, SensorPolicy::Warn)
                });

                assert_eq!(result, Ok(SignalKind::Shutdown));
                assert!(seen[5].1);
                assert_eq!(WARNINGS.load(Ordering::Relaxed), 2);
            }

            #[test]
            fn policy_abort_is_an_emergency_stop_that_tick() {
                let log = Log::new();
                let mut script = [along_x(1.0); 3];
                // Read call 2: tick 1.
                script[2].link_down = true;
                let mut sensing = one(FakeSensor {
                    script: &script,
                    ..FakeSensor::new(&log, 0)
                });
                let mut robot = recording_robot!(log).set_sensor_policy(SensorPolicy::Abort);
                let mut host = Scripted::new(Instant::now());

                let result = run_sensed(&mut robot, &mut host, &log, &mut sensing);

                assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
                assert_eq!(log.calls()[..13], STARTUP);
                assert_eq!(
                    log.calls()[13..],
                    [
                        Read,
                        SensorRead(0),
                        First,
                        EachTick,
                        Last,
                        Read,
                        SensorRead(0),
                        TorqueOff,
                        Close,
                        SensorClose(0)
                    ]
                );
            }
        }
    }

    mod long_runs {
        //! SC-001 and SC-002 over 10 000 scripted ticks: the order of every
        //! tick, and the grid under injected overruns.

        use core::convert::Infallible;

        use super::*;

        const TICKS: u32 = 10_000;

        /// One step of a tick, in the order SC-001 expects.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        enum Event {
            Read,
            FirstA,
            FirstB,
            EachA,
            EachB,
            LastA,
            LastB,
            Write,
            Sleep,
        }

        const TICK: [Event; 9] = [
            Event::Read,
            Event::FirstA,
            Event::FirstB,
            Event::EachA,
            Event::EachB,
            Event::LastA,
            Event::LastB,
            Event::Write,
            Event::Sleep,
        ];

        /// Shared by the host, the adapter and the controllers: the clock,
        /// the step expected next, the finished ticks and the tick starts.
        struct World {
            clock: Cell<Instant>,
            step: Cell<usize>,
            ticks: Cell<u32>,
            origin: Cell<Option<Instant>>,
            last_slot: Cell<Option<u128>>,
        }

        impl World {
            fn new() -> Self {
                Self {
                    clock: Cell::new(Instant::now()),
                    step: Cell::new(0),
                    ticks: Cell::new(0),
                    origin: Cell::new(None),
                    last_slot: Cell::new(None),
                }
            }

            /// Checks that `event` is the step expected next.
            fn saw(&self, event: Event) {
                let step = self.step.get();
                assert_eq!(event, TICK[step % TICK.len()], "step {step}");
                self.step.set(step + 1);
            }

            /// Checks that a tick starting now is on the grid, in a new slot.
            fn tick_starts(&self) {
                let origin = self.origin.get().expect("armed before tick 0");
                let offset = (self.clock.get() - origin).as_nanos();
                assert_eq!(offset % DT.as_nanos(), 0, "off the grid");
                let slot = offset / DT.as_nanos();
                if let Some(last) = self.last_slot.get() {
                    assert!(slot > last, "two ticks in slot {slot}");
                }
                self.last_slot.set(Some(slot));
            }

            fn work(&self, duration: Duration) {
                self.clock.set(self.clock.get() + duration);
            }
        }

        /// The host: a clock moved by the controllers' work and by the sleeps.
        struct Clockwork<'a>(&'a World);

        impl crate::runtime::seal::Host for Clockwork<'_> {}

        impl Host for Clockwork<'_> {
            fn now(&mut self) -> Instant {
                let now = self.0.clock.get();
                if self.0.origin.get().is_none() {
                    self.0.origin.set(Some(now));
                }
                now
            }

            fn sleep_until(&mut self, deadline: Instant) {
                self.0.saw(Event::Sleep);
                assert!(deadline >= self.0.clock.get(), "sleeps into the past");
                self.0.clock.set(deadline);
            }

            fn poll(&mut self) -> Option<Signal> {
                (self.0.ticks.get() >= TICKS).then(|| Signal {
                    kind: SignalKind::Shutdown,
                    meta: SignalMeta::new("scripted", file!(), line!()),
                })
            }
        }

        /// The adapter: reads and writes are steps of the tick.
        struct Steps<'a>(&'a World);

        impl seal::Adapter for Steps<'_> {}

        impl Hal<N> for Steps<'_> {
            type Error = Infallible;

            fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, Infallible> {
                Ok(Claim {
                    owned: Mask::ALL,
                    group: [Group(0); N],
                    read_every: [NonZeroU32::MIN; N],
                    read_cost: Duration::ZERO,
                    write_cost: Duration::ZERO,
                })
            }
            fn open(&mut self) -> Result<(), Infallible> {
                Ok(())
            }
            fn zero(&mut self) -> Result<(), Infallible> {
                Ok(())
            }
            fn ping(&mut self) -> Result<(), Infallible> {
                Ok(())
            }
            fn health(&mut self) -> Result<(), Infallible> {
                Ok(())
            }
            fn configure(&mut self) -> Result<(), Infallible> {
                Ok(())
            }
            fn read_all(&mut self, _sample: Sample<'_, N>) -> Result<(), Infallible> {
                Ok(())
            }
            fn torque_on(&mut self) -> Result<(), Infallible> {
                Ok(())
            }
            fn read(&mut self, _sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
                self.0.saw(Event::Read);
                Mask::ALL
            }
            fn write(
                &mut self,
                _command: Command<'_, N>,
                _batch: &Batch<'_, N>,
                _faults: &mut Faults<N>,
            ) {
                self.0.saw(Event::Write);
            }
            fn hold(&mut self, _target: Command<'_, N>) {}
            fn torque_off(&mut self) {}
            fn close(&mut self) {}
        }

        /// Runs `TICKS` ticks; `work(k)` is how long the controllers of tick
        /// `k` take.
        fn run(world: &World, work: impl Fn(u32) -> Duration) {
            let step = |event| move |_: &Time, _: &mut RobotState<N>, _: &Sensors| world.saw(event);
            let mut robot = Robot::new()
                .set_dt(DT)
                .add_controller(
                    First,
                    move |_: &Time, _: &mut RobotState<N>, _: &Sensors| {
                        world.tick_starts();
                        world.saw(Event::FirstA);
                    },
                )
                .add_controller(First, step(Event::FirstB))
                .add_controller(
                    EachTick,
                    move |_: &Time, state: &mut RobotState<N>, _: &Sensors| {
                        world.saw(Event::EachA);
                        world.work(work(world.ticks.get()));
                        state.set_at(Q(1.0), 0).expect("actuator 0 exists");
                    },
                )
                .add_controller(EachTick, step(Event::EachB))
                .add_controller(Last, step(Event::LastA))
                .add_controller(Last, move |_: &Time, _: &mut RobotState<N>, _: &Sensors| {
                    world.saw(Event::LastB);
                    world.ticks.set(world.ticks.get() + 1);
                });

            let result = run_with(&mut robot, &mut Clockwork(world), &mut one(Steps(world)));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(world.ticks.get(), TICKS);
            // The last tick stops before its sleep.
            assert_eq!(world.step.get(), TICK.len() * TICKS as usize - 1);
        }

        #[test]
        fn sc_001_every_tick_reads_walks_in_order_writes_and_sleeps() {
            let world = World::new();
            run(&world, |_| DT * 3 / 10);
            assert_eq!(world.last_slot.get(), Some(u128::from(TICKS - 1)));
        }

        #[test]
        fn sc_002_overrun_ticks_still_start_on_the_grid_one_per_slot() {
            let world = World::new();
            run(&world, |k| match k {
                k if k % 13 == 0 => DT * 5 / 2,
                k if k % 7 == 0 => DT * 17 / 10,
                k if k % 5 == 0 => DT,
                _ => DT * 3 / 10,
            });
            let slots = world.last_slot.get().expect("ticks ran");
            assert!(slots > u128::from(TICKS - 1), "overruns skip slots");
        }
    }

    mod three_members {
        //! An AX-12 on one Dynamixel bus, an MX-64 on another and an AK60 on
        //! CAN, as one robot of three members: each sees only its own
        //! actuators, from `check` to `close`.

        use core::convert::Infallible;
        use core::ops::Range;

        use super::*;

        /// What one member was handed during the run.
        #[derive(Default)]
        struct Seen {
            steps: Cell<u32>,
            reads: Cell<u32>,
            writes: Cell<u32>,
            /// Actuators of its runs and of its write masks, every tick.
            written: Cell<Mask<N>>,
            holds: Cell<u32>,
            held: Cell<Mask<N>>,
            torque_off: Cell<bool>,
            closed: Cell<bool>,
        }

        impl Seen {
            fn step(&self) -> Result<(), Infallible> {
                self.steps.set(self.steps.get() + 1);
                Ok(())
            }
        }

        /// A member on `owned` that reads `value` on each of them.
        struct Fake<'a> {
            owned: Mask<N>,
            value: Q,
            /// An actuator it reads into without owning it.
            stray: Option<usize>,
            seen: &'a Seen,
        }

        impl<'a> Fake<'a> {
            fn new(owned: Mask<N>, value: Q, seen: &'a Seen) -> Self {
                Self {
                    owned,
                    value,
                    stray: None,
                    seen,
                }
            }

            fn fill(&self, sample: Sample<'_, N>) {
                for actuator in self.owned.iter().chain(self.stray) {
                    sample.q[actuator] = self.value;
                }
            }
        }

        impl seal::Adapter for Fake<'_> {}

        impl Hal<N> for Fake<'_> {
            type Error = Infallible;

            fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, Infallible> {
                Ok(Claim {
                    owned: self.owned,
                    group: [Group(0); N],
                    read_every: [NonZeroU32::MIN; N],
                    read_cost: Duration::ZERO,
                    write_cost: Duration::ZERO,
                })
            }

            fn open(&mut self) -> Result<(), Infallible> {
                self.seen.step()
            }

            fn zero(&mut self) -> Result<(), Infallible> {
                self.seen.step()
            }

            fn ping(&mut self) -> Result<(), Infallible> {
                self.seen.step()
            }

            fn health(&mut self) -> Result<(), Infallible> {
                self.seen.step()
            }

            fn configure(&mut self) -> Result<(), Infallible> {
                self.seen.step()
            }

            fn read_all(&mut self, sample: Sample<'_, N>) -> Result<(), Infallible> {
                self.fill(sample);
                self.seen.step()
            }

            fn torque_on(&mut self) -> Result<(), Infallible> {
                self.seen.step()
            }

            fn read(&mut self, sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
                self.fill(sample);
                self.seen.reads.set(self.seen.reads.get() + 1);
                self.owned
            }

            fn write(
                &mut self,
                command: Command<'_, N>,
                batch: &Batch<'_, N>,
                _faults: &mut Faults<N>,
            ) {
                self.seen.writes.set(self.seen.writes.get() + 1);
                let mut written = self.seen.written.get().union(command.written.any());
                for &actuator in batch.runs().flat_map(|(_group, run)| run) {
                    written.set(actuator);
                }
                self.seen.written.set(written);
            }

            fn hold(&mut self, target: Command<'_, N>) {
                self.seen.holds.set(self.seen.holds.get() + 1);
                let held = self.seen.held.get().union(target.written.any());
                self.seen.held.set(held);
            }

            fn torque_off(&mut self) {
                self.seen.torque_off.set(true);
            }

            fn close(&mut self) {
                self.seen.closed.set(true);
            }
        }

        fn owned(actuators: Range<usize>) -> Mask<N> {
            let mut mask = Mask::NONE;
            actuators.for_each(|actuator| mask.set(actuator));
            mask
        }

        #[test]
        fn ax12_mx64_and_ak60_each_see_only_their_actuators() {
            let (ax12, mx64, ak60) = (Seen::default(), Seen::default(), Seen::default());
            let members = [
                (&ax12, owned(0..1)),
                (&mx64, owned(1..2)),
                (&ak60, owned(2..N)),
            ];
            let mut hal = HCons {
                head: Fake::new(members[0].1, Q(1.0), &ax12),
                tail: HCons {
                    head: Fake::new(members[1].1, Q(2.0), &mx64),
                    tail: HCons {
                        head: Fake::new(members[2].1, Q(3.0), &ak60),
                        tail: HNil,
                    },
                },
            };
            let sampled = Cell::new([Q(0.0); N]);
            let mut robot = Robot::new()
                .set_dt(DT)
                .set_hold(Duration::ZERO)
                .add_controller(crate::First, |_t, state: &mut RobotState<N>, _se| {
                    if let Some(q) = state.current::<Q>() {
                        sampled.set(*q);
                    }
                })
                .add_controller(crate::EachTick, |_t, state: &mut RobotState<N>, _se| {
                    for actuator in 0..N {
                        state.set_at(Q(0.5), actuator).expect("in range");
                    }
                });
            // Raised at the last check of tick 1: two ticks, then the hold.
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_START + 9);

            let signal =
                run_with(&mut robot, &mut host, &mut hal).expect("the claims cover the robot");

            assert_eq!(signal.kind(), SignalKind::Shutdown);
            let expected: [Q; N] = core::array::from_fn(|actuator| match actuator {
                0 => Q(1.0),
                1 => Q(2.0),
                _ => Q(3.0),
            });
            assert_eq!(sampled.get(), expected);
            for (seen, mine) in members {
                assert_eq!(seen.steps.get(), 7, "every bring-up step");
                assert_eq!((seen.reads.get(), seen.writes.get()), (2, 2));
                assert_eq!(seen.written.get(), mine);
                assert_eq!((seen.holds.get(), seen.held.get()), (1, mine));
                assert!(seen.torque_off.get() && seen.closed.get());
            }
        }

        #[cfg(debug_assertions)]
        #[test]
        fn a_member_reading_into_another_s_actuator_stops_the_run() {
            let (ax12, ak60) = (Seen::default(), Seen::default());
            let mut rogue = Fake::new(owned(1..N), Q(3.0), &ak60);
            rogue.stray = Some(0);
            let mut hal = HCons {
                head: Fake::new(owned(0..1), Q(1.0), &ax12),
                tail: HCons {
                    head: rogue,
                    tail: HNil,
                },
            };
            let mut robot = Robot::new()
                .set_dt(DT)
                .add_controller(crate::Last, |_t, _s, _se| {});
            let mut host = Scripted::new(Instant::now());

            let signal =
                run_with(&mut robot, &mut host, &mut hal).expect("the claims cover the robot");

            assert_eq!(signal.kind(), SignalKind::EmergencyStop);
            for seen in [&ax12, &ak60] {
                assert_eq!(seen.steps.get(), 6, "up to read_all, no torque_on");
                assert!(seen.torque_off.get() && seen.closed.get());
            }
        }
    }

    /// The per-slot faults and tick flags (SRS FR-13.4).
    mod per_slot_flags {
        use super::*;
        use crate::hal::TickFlags;

        /// What a `First` controller saw on one tick.
        #[derive(Clone, Copy, Debug, Default, PartialEq)]
        struct Seen {
            now: Faults<N>,
            before: Option<Faults<N>>,
            flags_before: Option<TickFlags>,
        }

        /// The mask of `indices`.
        fn mask(indices: &[usize]) -> Mask<N> {
            let mut mask = Mask::NONE;
            for &index in indices {
                mask.set(index);
            }
            mask
        }

        /// Runs `ticks` ticks of a robot writing actuator 1 on tick 0, on
        /// `recorder` and `host`; what `First` saw on each tick.
        fn run<const T: usize>(recorder: Recorder<'_, N>, host: Scripted) -> [Seen; T] {
            let seen = RefCell::new([Seen::default(); T]);
            let mut robot = Robot::new().set_dt(DT).add_controller(
                crate::First,
                |time: &Time, state: &mut RobotState<N>, _: &Sensors| {
                    let tick = usize::try_from(time.tick()).expect("small");
                    if let Some(slot) = seen.borrow_mut().get_mut(tick) {
                        *slot = Seen {
                            now: *state.faults(0).expect("this tick's slot"),
                            before: state.faults(1).ok().copied(),
                            flags_before: state.tick_flags(1).ok(),
                        };
                    }
                    if tick == 0 {
                        state.set_at(Q(1.0), 1).expect("in range");
                    }
                },
            );
            let mut host = host.shutdown_from(TICK_END + (T - 1) * PER_TICK);
            let result = run_with(&mut robot, &mut host, &mut one(recorder));
            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            seen.into_inner()
        }

        #[test]
        fn sc_006_a_fault_shows_in_its_ticks_slot_at_depth_0_then_1() {
            let log = Log::new();
            let mut faults = Faults::default();
            faults.stale.set(2);
            let mut write_faults = Faults::default();
            write_faults.rejected.set(1);
            let recorder = Recorder {
                faults,
                faults_on: Some(1),
                write_faults,
                ..Recorder::new(&log)
            };

            let seen = run::<3>(recorder, Scripted::new(Instant::now()));

            assert_eq!(
                seen.map(|s| s.now.stale),
                [mask(&[]), mask(&[2]), mask(&[])]
            );
            assert_eq!(seen[2].before.map(|f| f.stale), Some(mask(&[2])));
            // The write of tick 0 reads at depth 1 on tick 1, never at depth 0.
            assert_eq!(seen[0].now.rejected, Mask::NONE);
            assert_eq!(seen[1].before.map(|f| f.rejected), Some(mask(&[1])));
            assert_eq!(seen[1].now.rejected, Mask::NONE);
        }

        #[test]
        fn a_new_slot_starts_without_a_fault() {
            let log = Log::new();
            let mut faults = Faults::default();
            faults.offline.set(0);
            let recorder = Recorder {
                faults,
                faults_on: Some(0),
                ..Recorder::new(&log)
            };

            let seen = run::<3>(recorder, Scripted::new(Instant::now()));

            assert_eq!(seen.map(|s| s.now.is_empty()), [false, true, true]);
            assert_eq!(seen[0].flags_before, Some(TickFlags::NONE));
        }

        #[test]
        fn us6_an_overrun_on_tick_7_reads_set_at_depth_1_on_tick_8() {
            // `now` at arm, then at the start and the end of each tick, in
            // tenths of DT: tick 7 starts at 75 and ends at 85, past its 80.
            let mut script = [0_u32; 19];
            script[0] = 0;
            for (tick, pair) in script[1..].chunks_mut(2).enumerate() {
                let start = u32::try_from(tick).expect("small") * 10 + 5;
                pair[0] = start;
                if let Some(end) = pair.get_mut(1) {
                    *end = if tick == 7 { start + 10 } else { start };
                }
            }
            let host = Scripted::new(Instant::now()).script(&script);

            let seen = run::<9>(Recorder::new(&Log::new()), host);

            let overrun = seen.map(|s| {
                s.flags_before
                    .is_some_and(|f| f.contains(TickFlags::OVERRUN))
            });
            assert_eq!(
                overrun,
                [false, false, false, false, false, false, false, false, true]
            );
        }

        #[test]
        fn a_flag_raised_from_another_thread_during_tick_0_reads_in_tick_1() {
            use std::sync::atomic::AtomicBool;
            static FLAGS: crate::hal::ExternalFlags = crate::hal::ExternalFlags::new();
            static GO: AtomicBool = AtomicBool::new(false);
            static RAISED: AtomicBool = AtomicBool::new(false);
            FLAGS.raise(TickFlags::SENSOR_MISSING);

            let seen = RefCell::new([TickFlags::NONE; 3]);
            let mut robot = Robot::new().set_dt(DT).add_controller(
                crate::First,
                |time: &Time, state: &mut RobotState<N>, _: &Sensors| {
                    let tick = usize::try_from(time.tick()).expect("small");
                    if let Some(slot) = seen.borrow_mut().get_mut(tick) {
                        *slot = state.tick_flags(0).expect("this tick's slot");
                    }
                    if tick == 0 {
                        GO.store(true, Ordering::Release);
                        while !RAISED.load(Ordering::Acquire) {
                            std::hint::spin_loop();
                        }
                    }
                },
            );
            let mut host = Scripted {
                external: Some(&FLAGS),
                ..Scripted::new(Instant::now())
            }
            .shutdown_from(TICK_END + 2 * PER_TICK);

            std::thread::scope(|scope| {
                scope.spawn(|| {
                    while !GO.load(Ordering::Acquire) {
                        std::hint::spin_loop();
                    }
                    FLAGS.raise(TickFlags::UNAUTHORIZED);
                    RAISED.store(true, Ordering::Release);
                });
                let result = run_with(&mut robot, &mut host, &mut one(NoHardware));
                assert!(result.is_ok());
            });

            // The flag raised before the run is dropped, the one of tick 0 lands once.
            assert_eq!(
                seen.into_inner(),
                [TickFlags::NONE, TickFlags::UNAUTHORIZED, TickFlags::NONE]
            );
        }

        #[test]
        fn a_depth_past_the_history_is_an_error() {
            let state = RobotState::<N>::default();
            assert!(state.faults(0).is_err());
            assert!(state.tick_flags(crate::HISTORY_DEPTH).is_err());
        }
    }

    #[cfg(feature = "experimental")]
    mod presence {
        //! The adapters' presence through the loop, and the upkeep call of
        //! the join policy.

        use super::*;
        use crate::hal::{JoinPolicy, MissingPolicy, Overrides};
        use crate::telemetry::Telemetry;

        /// An adapter call, a host call or a walk, in the order they came.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        enum Ev {
            Read,
            Write,
            Telemetry,
            Upkeep,
            Now,
            Poll,
            Last,
        }

        /// Events a run holds; past them, dropped.
        const EVENTS: usize = 128;

        /// The events of a run, shared by the fake, the host and the walks.
        struct Events(RefCell<([Ev; EVENTS], usize)>);

        impl Events {
            fn new() -> Self {
                Self(RefCell::new(([Ev::Poll; EVENTS], 0)))
            }

            fn push(&self, event: Ev) {
                let (seen, len) = &mut *self.0.borrow_mut();
                if let Some(slot) = seen.get_mut(*len) {
                    *slot = event;
                    *len += 1;
                }
            }

            fn seen(&self) -> Ref<'_, [Ev]> {
                Ref::map(self.0.borrow(), |(seen, len)| {
                    seen.get(..*len).unwrap_or(&[])
                })
            }

            fn count(&self, event: Ev) -> usize {
                self.seen().iter().filter(|&&e| e == event).count()
            }
        }

        /// Every actuator but `index`.
        fn all_but(index: usize) -> Mask<N> {
            let mut mask = Mask::ALL;
            mask.clear(index);
            mask
        }

        /// A member owning every actuator whose presence after its read
        /// call `k` (read_all included) is `script[k]`, the last one
        /// past the script. Each read refreshes the present actuators.
        struct Flaky<'a> {
            events: &'a Events,
            log: &'a Log<N>,
            script: [Mask<N>; 4],
            reads: usize,
            overrides: Option<Overrides>,
        }

        impl<'a> Flaky<'a> {
            fn new(events: &'a Events, log: &'a Log<N>, script: [Mask<N>; 4]) -> Self {
                Self {
                    events,
                    log,
                    script,
                    reads: 0,
                    overrides: None,
                }
            }
        }

        impl seal::Adapter for Flaky<'_> {}

        impl Hal<N> for Flaky<'_> {
            type Error = core::convert::Infallible;

            fn check(&mut self, overrides: Overrides) -> Result<Claim<N>, Self::Error> {
                self.overrides = Some(overrides);
                Hal::<N>::check(&mut NoHardware, overrides)
            }

            fn open(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }

            fn zero(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }

            fn ping(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }

            fn health(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }

            fn configure(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }

            fn read_all(&mut self, sample: Sample<'_, N>) -> Result<(), Self::Error> {
                *sample.q = [SEED; N];
                self.reads += 1;
                Ok(())
            }

            fn torque_on(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }

            fn read(&mut self, _sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
                self.events.push(Ev::Read);
                self.reads += 1;
                self.present()
            }

            fn write(&mut self, command: Command<'_, N>, batch: &Batch<'_, N>, _: &mut Faults<N>) {
                self.events.push(Ev::Write);
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

            fn telemetry(&mut self, _: &mut Telemetry<N>, _: &mut Faults<N>) -> Mask<N> {
                self.events.push(Ev::Telemetry);
                Mask::NONE
            }

            fn present(&self) -> Mask<N> {
                let k = self.reads.saturating_sub(1).min(self.script.len() - 1);
                self.script.get(k).copied().unwrap_or(Mask::ALL)
            }

            fn upkeep(&mut self) {
                self.events.push(Ev::Upkeep);
            }

            fn hold(&mut self, target: Command<'_, N>) {
                self.log.hold.set(Some(Sent {
                    q: *target.q,
                    written: *target.written,
                    batch: Mask::ALL,
                }));
            }

            fn torque_off(&mut self) {}

            fn close(&mut self) {}
        }

        /// A scripted host whose clock reads and bus checks are events.
        struct Watched<'a> {
            inner: Scripted,
            events: &'a Events,
        }

        impl crate::runtime::seal::Host for Watched<'_> {}

        impl Host for Watched<'_> {
            fn now(&mut self) -> Instant {
                self.events.push(Ev::Now);
                self.inner.now()
            }

            fn sleep_until(&mut self, deadline: Instant) {
                self.inner.sleep_until(deadline);
            }

            fn poll(&mut self) -> Option<Signal> {
                self.events.push(Ev::Poll);
                self.inner.poll()
            }
        }

        /// Bus checks per tick under [`JoinPolicy::Declared`]: one more, before
        /// the upkeep.
        const PER_JOIN_TICK: usize = PER_TICK + 1;

        /// The bus check before the upkeep of tick `k`, under
        /// [`JoinPolicy::Declared`].
        const fn before_upkeep(k: usize) -> usize {
            TICK_START + k * PER_JOIN_TICK + 4
        }

        /// The bus check at the end of tick `k`, under [`JoinPolicy::Declared`].
        const fn join_tick_end(k: usize) -> usize {
            before_upkeep(k) + 1
        }

        #[test]
        fn claim_gets_the_policies_in_overrides() {
            let (events, log) = (Events::new(), Log::new());
            let mut flaky = Flaky::new(&events, &log, [Mask::ALL; 4]);
            let mut robot = Robot::new()
                .set_dt(DT)
                .set_missing_policy(MissingPolicy::Tolerate)
                .set_join_policy(JoinPolicy::Declared)
                .add_controller(
                    crate::EachTick,
                    |_t: &Time, _s: &mut RobotState<N>, _: &Sensors| {},
                );
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_PRE_INIT);

            let result = run_with(&mut robot, &mut host, &mut one(&mut flaky));

            assert!(result.is_ok());
            let overrides = flaky.overrides.expect("checked");
            assert_eq!(overrides.missing, MissingPolicy::Tolerate);
            assert_eq!(overrides.join, JoinPolicy::Declared);
        }

        #[test]
        fn present_reaches_state_after_read_before_first() {
            let (events, log) = (Events::new(), Log::new());
            let flaky = Flaky::new(
                &events,
                &log,
                [all_but(3), Mask::ALL, all_but(1), all_but(1)],
            );
            let post_init = Cell::new(Mask::NONE);
            let first = RefCell::new([Mask::NONE; 2]);
            let mut robot = Robot::new()
                .set_dt(DT)
                .add_controller(
                    crate::PostInit,
                    |_t: &Time, state: &mut RobotState<N>, _: &Sensors| {
                        post_init.set(state.present());
                    },
                )
                .add_controller(
                    crate::First,
                    |time: &Time, state: &mut RobotState<N>, _: &Sensors| {
                        let tick = usize::try_from(time.tick()).expect("small");
                        if let Some(slot) = first.borrow_mut().get_mut(tick) {
                            *slot = state.present();
                        }
                    },
                );
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + PER_TICK);

            let result = run_with(&mut robot, &mut host, &mut one(flaky));

            assert!(result.is_ok());
            assert_eq!(post_init.get(), all_but(3));
            assert_eq!(first.into_inner(), [Mask::ALL, all_but(1)]);
        }

        #[test]
        fn read_all_samples_only_present_slots() {
            let (events, log) = (Events::new(), Log::new());
            let mut drivers = one(Flaky::new(&events, &log, [all_but(2); 4]));
            let layout = Layout::new(
                [Group(0); N],
                [NonZeroU32::MIN; N],
                Duration::ZERO,
                Duration::ZERO,
            );
            let mut state = RobotState::<N>::default();
            let sample = state.push_slot().expect("a history slot");

            let read = drivers.read_all(sample, layout.slots(), &mut Audit::default());

            assert_eq!(read.ok(), Some(all_but(2)));
            assert_eq!(drivers.present(layout.slots()), all_but(2));
        }

        #[test]
        fn set_all_writes_only_present_slots() {
            let (events, log) = (Events::new(), Log::new());
            let flaky = Flaky::new(
                &events,
                &log,
                [Mask::ALL, all_but(2), all_but(2), all_but(2)],
            );
            let mut robot = Robot::new().set_dt(DT).add_controller(
                crate::EachTick,
                |_t: &Time, state: &mut RobotState<N>, _: &Sensors| {
                    state.set_all([Q(1.0); N]);
                    state.set_at(Qd(1.0), 2).expect("in range");
                },
            );
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);

            let result = run_with(&mut robot, &mut host, &mut one(flaky));

            assert!(result.is_ok());
            let sent = log.write.get().expect("tick 0 wrote");
            assert_eq!(sent.written.q, all_but(2));
            assert!(sent.written.qd.is_empty());
            assert_eq!(sent.batch, all_but(2));
        }

        #[test]
        fn hold_skips_absent_slots() {
            let (events, log) = (Events::new(), Log::new());
            let flaky = Flaky::new(
                &events,
                &log,
                [Mask::ALL, all_but(2), all_but(2), all_but(2)],
            );
            let mut robot = Robot::new()
                .set_dt(DT)
                .set_hold(Duration::ZERO)
                .add_controller(
                    crate::EachTick,
                    |_t: &Time, state: &mut RobotState<N>, _: &Sensors| {
                        state.set_all([Q(1.0); N]);
                    },
                );
            let mut host = Scripted::new(Instant::now()).shutdown_from(AFTER_FIRST);

            let result = run_with(&mut robot, &mut host, &mut one(flaky));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            let held = log.hold.get().expect("the shutdown held");
            assert_eq!(held.written.q, all_but(2));
        }

        // A macro, not a fn: the robot's `HCons` type is not worth spelling.
        macro_rules! logging_robot {
            ($events:expr) => {
                Robot::new().set_dt(DT).add_controller(
                    crate::Last,
                    |_t: &Time, _s: &mut RobotState<N>, _: &Sensors| $events.push(Ev::Last),
                )
            };
        }

        #[test]
        fn upkeep_runs_once_per_tick_after_telemetry_before_end_tick() {
            let (events, log) = (Events::new(), Log::new());
            let flaky = Flaky::new(&events, &log, [Mask::ALL; 4]);
            let mut robot = logging_robot!(events).set_join_policy(JoinPolicy::Declared);
            let mut host = Watched {
                inner: Scripted::new(Instant::now()).shutdown_from(join_tick_end(2)),
                events: &events,
            };

            let result = run_with(&mut robot, &mut host, &mut one(flaky));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(events.count(Ev::Last), 3);
            assert_eq!(events.count(Ev::Upkeep), 3);
            let seen = events.seen();
            for (at, _) in seen.iter().enumerate().filter(|(_, e)| **e == Ev::Upkeep) {
                let around = seen.get(at.saturating_sub(2)..at + 2);
                assert_eq!(
                    around,
                    Some(&[Ev::Telemetry, Ev::Poll, Ev::Upkeep, Ev::Now][..])
                );
            }
        }

        #[test]
        fn upkeep_never_runs_with_join_never() {
            let (events, log) = (Events::new(), Log::new());
            let flaky = Flaky::new(&events, &log, [Mask::ALL; 4]);
            let mut robot = logging_robot!(events).set_missing_policy(MissingPolicy::Tolerate);
            let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 2 * PER_TICK);

            let result = run_with(&mut robot, &mut host, &mut one(flaky));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(events.count(Ev::Last), 3);
            assert_eq!(events.count(Ev::Upkeep), 0);
            assert_eq!(host.poll_calls, TICK_END + 2 * PER_TICK + 2);
        }

        #[test]
        fn estop_raised_before_upkeep_skips_it() {
            let (events, log) = (Events::new(), Log::new());
            let flaky = Flaky::new(&events, &log, [Mask::ALL; 4]);
            let mut robot = logging_robot!(events).set_join_policy(JoinPolicy::Declared);
            let mut host = Scripted::new(Instant::now()).estop_from(before_upkeep(0));

            let result = run_with(&mut robot, &mut host, &mut one(flaky));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::EmergencyStop));
            assert_eq!(events.count(Ev::Telemetry), 2);
            assert_eq!(events.count(Ev::Upkeep), 0);
            assert_eq!(robot.config.time.last_update(), None);
        }

        #[test]
        fn shutdown_raised_before_upkeep_skips_it_and_completes_the_tick() {
            let (events, log) = (Events::new(), Log::new());
            let flaky = Flaky::new(&events, &log, [Mask::ALL; 4]);
            let mut robot = logging_robot!(events).set_join_policy(JoinPolicy::Declared);
            let mut host = Scripted::new(Instant::now())
                .script(&[0, 0, 4])
                .shutdown_from(before_upkeep(0));

            let result = run_with(&mut robot, &mut host, &mut one(flaky));

            assert_eq!(result.map(|s| s.kind()), Ok(SignalKind::Shutdown));
            assert_eq!(events.count(Ev::Upkeep), 0);
            assert_eq!(robot.config.time.last_update(), Some(&host.at(4)));
        }
    }
}
