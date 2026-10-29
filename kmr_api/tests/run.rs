//! `run()` end to end: finalization, binding, then the run. Never opens a serial device.

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use kmr_api::{Bus, EachTick, Sensors, Time, UserError};

kmr_api::robot!(XmBench);

/// The signal bus is process-global: every test that runs takes this first.
static BUS: Mutex<()> = Mutex::new(());

fn bus() -> MutexGuard<'static, ()> {
    BUS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// `(tick, dt)` of every call, shared with the test.
type Seen = Arc<Mutex<Vec<(u64, Duration)>>>;

/// Records the clock, and shuts down at tick `STOP`.
fn stop_at<const STOP: u64>(seen: &mut Seen, time: &Time, _r: &mut State, _s: &Sensors) {
    if let Ok(mut seen) = seen.lock() {
        seen.push((time.tick(), time.dt()));
    }
    assert!(time.tick() <= STOP, "the loop ignored the shutdown");
    if time.tick() == STOP {
        kmr_api::shutdown!("test: stop tick reached");
    }
}

fn seen() -> Seen {
    Arc::new(Mutex::new(Vec::new()))
}

fn ticks(seen: &Seen) -> Vec<(u64, Duration)> {
    seen.lock().unwrap_or_else(PoisonError::into_inner).clone()
}

#[test]
fn without_a_plugin_it_simulates_until_the_shutdown() {
    let _bus = bus();
    let record = seen();
    let res = Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_controller_with(EachTick, Arc::clone(&record), stop_at::<3>)
        .run();
    assert!(res.is_ok(), "{res:?}");
    let ms = Duration::from_millis(1);
    assert_eq!(ticks(&record), [(0, ms), (1, ms), (2, ms), (3, ms)]);
}

#[cfg(not(feature = "dynamixel"))]
#[test]
fn a_plugin_without_the_feature_is_refused_before_the_run() {
    let record = seen();
    let res = Robot::new()
        .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
        .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
        .run();
    assert!(
        matches!(
            res,
            Err(UserError::FeatureOff {
                feature: "dynamixel"
            })
        ),
        "{res:?}"
    );
    assert!(ticks(&record).is_empty());
}

#[test]
fn two_plugins_on_one_port_are_refused_before_the_run() {
    let record = seen();
    let res = Robot::new()
        .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
        .add_plugins(Bus::by_id("kmr-test-no-such-serial").bus("arm"))
        .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
        .run();
    assert!(
        matches!(
            res,
            Err(UserError::DuplicateBus {
                port: "kmr-test-no-such-serial"
            })
        ),
        "{res:?}"
    );
    assert!(ticks(&record).is_empty());
}

#[test]
fn a_dt_out_of_range_is_refused_before_the_run() {
    for dt in [Duration::ZERO, Duration::from_micros(5)] {
        let record = seen();
        let res = Robot::new()
            .set_dt(dt)
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
            .run();
        assert!(
            matches!(res, Err(UserError::DtOutOfRange { dt: refused }) if refused == dt),
            "{res:?}"
        );
        assert!(ticks(&record).is_empty());
    }
}

#[test]
fn a_dt_of_10_us_or_1_s_runs() {
    let _bus = bus();
    for dt in [Duration::from_micros(10), Duration::from_secs(1)] {
        let record = seen();
        let res = Robot::new()
            .set_dt(dt)
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
            .run();
        assert!(res.is_ok(), "{res:?}");
        assert_eq!(ticks(&record), [(0, dt)]);
    }
}

/// The real Dynamixel adapter: the serial resolves to no device, so no port is ever opened.
#[cfg(feature = "dynamixel")]
#[test]
fn a_missing_serial_is_a_bring_up_error_naming_it() {
    let _bus = bus();
    let record = seen();
    let res = Robot::new()
        .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
        .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
        .run();
    assert!(
        matches!(&res, Err(UserError::BringUp { source }) if source.to_string().contains("kmr-test-no-such-serial")),
        "{res:?}"
    );
    assert!(ticks(&record).is_empty());
}

/// A two-bus Dynamixel robot: the adapter's binding errors reach `run()` unchanged.
#[cfg(feature = "dynamixel")]
mod two_buses {
    use super::*;
    use kmr_adapters::dynamixel::{
        Actuator, Bus as TableBus, DynamixelError, Mode, Model, Table, Torque, Zero,
    };

    struct TwoBuses;

    const fn xm430(bus: usize, id: u8) -> Actuator {
        Actuator {
            bus,
            id,
            model: Model::Xm430W210,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
        }
    }

    const fn bus(label: &'static str) -> TableBus {
        TableBus {
            label,
            baud: 1_000_000,
            read_every: None,
        }
    }

    const TABLE: Table<2, 4> = Table {
        buses: [bus("left"), bus("right")],
        actuators: [xm430(0, 1), xm430(0, 2), xm430(1, 1), xm430(1, 2)],
    }
    .check();

    impl kmr_api::Spec for TwoBuses {
        type Adapter = kmr_api::__DxlAdapter<2>;

        fn bind(plugins: &[kmr_api::Binding]) -> Result<Self::Adapter, UserError> {
            kmr_api::__bind_dynamixel(TABLE, plugins)
        }
    }

    #[test]
    fn one_plugin_on_two_buses_is_the_adapters_binding_error() {
        let record = seen();
        let res = kmr_api::Robot::<TwoBuses>::new()
            .add_plugins(Bus::by_id("kmr-test-no-such-serial").bus("left"))
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
            .run();
        let Err(e) = res else {
            panic!("bound one bus of two: {res:?}");
        };
        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: bus right: no bus plugin binds it; add one with `.bus(\"right\")`"
        );
        let adapter =
            std::error::Error::source(&e).and_then(|s| s.downcast_ref::<DynamixelError>());
        assert_eq!(
            adapter,
            Some(&DynamixelError::UnboundBus { bus: "right" }),
            "{e:?}"
        );
        assert!(ticks(&record).is_empty());
    }
}

/// A recording driver: every HAL call lands in one log, in order.
mod recording {
    use super::*;
    use core::fmt;
    use kmr_api::{Q, Spec};
    use kmr_core::hal::{
        Batch, Command, Faults, Group, Hal, Layout, Mask, Overrides, Sample, seal,
    };
    use kmr_core::state::JOINTS;
    use std::num::NonZeroU32;

    static LOG: Mutex<Vec<&'static str>> = Mutex::new(Vec::new());

    fn log(call: &'static str) {
        LOG.lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(call);
    }

    /// The log so far, emptied.
    fn take() -> Vec<&'static str> {
        std::mem::take(&mut *LOG.lock().unwrap_or_else(PoisonError::into_inner))
    }

    /// The bring-up error of a recorder told to fail `open`.
    #[derive(Debug)]
    struct NoAnswer;

    impl fmt::Display for NoAnswer {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.write_str("actuator 2 on serial kmr-test-no-such-serial: no answer")
        }
    }

    impl core::error::Error for NoAnswer {}

    struct Recorder {
        fail_open: bool,
    }

    impl seal::Adapter for Recorder {}

    impl Hal for Recorder {
        type Error = NoAnswer;

        fn check(&mut self, _overrides: Overrides) -> Result<Layout, NoAnswer> {
            log("check");
            Ok(Layout::new(
                [Group(0); JOINTS],
                [NonZeroU32::MIN; JOINTS],
                Duration::ZERO,
                Duration::ZERO,
            ))
        }

        fn open(&mut self) -> Result<(), NoAnswer> {
            log("open");
            if self.fail_open {
                Err(NoAnswer)
            } else {
                Ok(())
            }
        }

        fn zero(&mut self) -> Result<(), NoAnswer> {
            log("zero");
            Ok(())
        }

        fn ping(&mut self) -> Result<(), NoAnswer> {
            log("ping");
            Ok(())
        }

        fn health(&mut self) -> Result<(), NoAnswer> {
            log("health");
            Ok(())
        }

        fn configure(&mut self) -> Result<(), NoAnswer> {
            log("configure");
            Ok(())
        }

        fn read_all(&mut self, _sample: Sample<'_>) -> Result<(), NoAnswer> {
            log("read_all");
            Ok(())
        }

        fn torque_on(&mut self) -> Result<(), NoAnswer> {
            log("torque_on");
            Ok(())
        }

        fn read(&mut self, _sample: Sample<'_>, _faults: &mut Faults) -> Mask {
            log("read");
            Mask::NONE
        }

        fn write(&mut self, _command: Command<'_>, _batch: &Batch<'_>, _faults: &mut Faults) {
            log("write");
        }

        fn hold(&mut self, _last: Command<'_>) {
            log("hold");
        }

        fn torque_off(&mut self) {
            log("torque_off");
        }

        fn close(&mut self) {
            log("close");
        }
    }

    /// Binds every plugin to a recorder.
    struct Records;

    impl Spec for Records {
        type Adapter = Recorder;

        fn bind(_plugins: &[kmr_api::Binding]) -> Result<Recorder, UserError> {
            Ok(Recorder { fail_open: false })
        }
    }

    /// Binds every plugin to a recorder whose `open` fails.
    struct FailsOpen;

    impl Spec for FailsOpen {
        type Adapter = Recorder;

        fn bind(_plugins: &[kmr_api::Binding]) -> Result<Recorder, UserError> {
            Ok(Recorder { fail_open: true })
        }
    }

    /// Writes every position, then shuts down at tick 2.
    fn write_then_stop(time: &Time, robot: &mut State, _s: &Sensors) {
        robot.set_all_q([Q(0.0); N]);
        if time.tick() == 2 {
            kmr_api::shutdown!("test: tick 2");
        }
    }

    #[test]
    fn a_plugin_runs_on_the_bound_driver() {
        let _bus = bus();
        take();
        let res = kmr_api::Robot::<Records>::new()
            .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(res.is_ok(), "{res:?}");
        assert_eq!(
            take(),
            [
                "check",
                "open",
                "zero",
                "ping",
                "health",
                "configure",
                "read_all",
                "torque_on",
                "read",
                "write",
                "read",
                "write",
                "read",
                "write",
                "hold",
                "torque_off",
                "close",
            ]
        );
    }

    #[test]
    fn a_bring_up_error_is_returned_after_torque_off_and_close() {
        let _bus = bus();
        take();
        let res = kmr_api::Robot::<FailsOpen>::new()
            .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
            .add_controller(EachTick, write_then_stop)
            .run();
        let Err(e) = res else {
            panic!("open failed, yet the run went on: {res:?}");
        };
        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: actuator 2 on serial kmr-test-no-such-serial: no answer"
        );
        let source = std::error::Error::source(&e).and_then(|s| s.downcast_ref::<NoAnswer>());
        assert!(source.is_some(), "{e:?}");
        assert_eq!(take(), ["check", "open", "torque_off", "close"]);
    }

    #[test]
    fn without_a_plugin_the_bound_driver_is_never_called() {
        let _bus = bus();
        take();
        let res = kmr_api::Robot::<Records>::new()
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(res.is_ok(), "{res:?}");
        assert!(take().is_empty());
    }
}
