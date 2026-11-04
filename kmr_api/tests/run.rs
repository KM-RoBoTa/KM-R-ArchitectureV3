//! `run()` end to end: finalization, binding, then the run. Never opens a serial device.

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use kmr_api::{Bus, EachTick, Payload, Sensors, Time, UserError};

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

#[test]
fn a_payload_on_an_unknown_link_is_refused_before_the_plugins() {
    let record = seen();
    let res = Robot::new()
        .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
        .register_payload(Payload::medium(1.2, "link4"))
        .register_payload(Payload::medium(1.2, "nope"))
        .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
        .run();
    assert!(
        matches!(res, Err(UserError::UnknownLink { link: "nope" })),
        "{res:?}"
    );
    assert!(ticks(&record).is_empty());
}

#[test]
fn a_payload_on_a_model_link_runs() {
    let _bus = bus();
    let record = seen();
    let res = Robot::new()
        .set_dt(Duration::from_millis(1))
        .register_payload(Payload::medium(1.2, "link4"))
        .register_payload(Payload::heavy(3.0, "base"))
        .add_controller_with(EachTick, Arc::clone(&record), stop_at::<1>)
        .run();
    assert!(res.is_ok(), "{res:?}");
    assert_eq!(ticks(&record).len(), 2);
}

#[test]
fn actuators_on_a_preset_robot_are_refused_before_the_run() {
    let spine = Bus::by_id("kmr-test-no-such-serial")
        .baud(1_000_000)
        .build();
    for plugged in [false, true] {
        let record = seen();
        let robot = Robot::new().add_actuator(kmr_api::dynamixel::Xm430W210::ids(1..=4).on(spine));
        let robot = if plugged {
            robot.add_plugins(Bus::by_id("kmr-test-no-such-serial"))
        } else {
            robot
        };
        let res = robot
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
            .run();
        assert!(
            matches!(res, Err(UserError::PresetHardware { preset: "XmBench" })),
            "{res:?}"
        );
        assert!(ticks(&record).is_empty());
    }
}

/// A `robot!(model = ..)` robot without a table: its actuators are declared in main.rs.
mod urdf {
    use super::*;
    use kmr_api::Policy;
    use kmr_api::dynamixel::Xm430W210;

    kmr_api::robot!(model = "tests/fixtures/four_dxl.urdf");

    fn spine() -> kmr_api::BusHandle {
        Bus::by_id("kmr-test-no-such-serial")
            .baud(1_000_000)
            .build()
    }

    #[test]
    fn without_an_actuator_it_simulates() {
        let _bus = bus();
        let record = seen();
        let res = Robot::new()
            .set_dt(Duration::from_millis(1))
            .set_limit_policy_all(Policy::Clamp)
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<1>)
            .run();
        assert!(res.is_ok(), "{res:?}");
        assert_eq!(ticks(&record).len(), 2);
    }

    #[test]
    fn a_plugin_is_refused_actuators_carry_their_bus() {
        let record = seen();
        let res = Robot::new()
            .set_limit_policy_all(Policy::Clamp)
            .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
            .add_actuator(Xm430W210::ids(1..=4).on(spine()))
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
            .run();
        assert!(matches!(res, Err(UserError::NoHardwareDeclared)), "{res:?}");
        assert!(ticks(&record).is_empty());
    }

    #[test]
    fn a_count_other_than_n_is_refused_naming_both() {
        let record = seen();
        let res = Robot::new()
            .set_limit_policy_all(Policy::Clamp)
            .add_actuator(Xm430W210::ids(1..=2).on(spine()))
            .add_actuator(Xm430W210::id(3).on(spine()))
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
            .run();
        let Err(e) = res else {
            panic!("three actuators of four ran: {res:?}");
        };
        assert!(
            matches!(
                e,
                UserError::ActuatorCount {
                    declared: 3,
                    expected: 4
                }
            ),
            "{e:?}"
        );
        assert!(e.to_string().starts_with("3 actuators"), "{e}");
        assert!(e.to_string().contains("has 4"), "{e}");
        assert!(ticks(&record).is_empty());
    }

    #[test]
    fn a_baud_the_model_lacks_is_refused_as_check_words_it() {
        let slow = Bus::by_id("kmr-test-no-such-serial").baud(500_000).build();
        let res = Robot::new()
            .set_limit_policy_all(Policy::Clamp)
            .add_actuator(Xm430W210::ids(1..=4).on(slow))
            .add_controller(EachTick, |_t: &Time, _r: &mut State, _s: &Sensors| {})
            .run();
        let Err(e) = res else {
            panic!("a 500 kbit/s XM430 bus ran: {res:?}");
        };
        assert!(matches!(e, UserError::HardwareRejected { .. }), "{e:?}");
        assert_eq!(
            e.to_string(),
            "the declared actuators are refused: actuator 0: XM430-W210 does not accept 500000 \
             baud on bus kmr-test-no-such-serial"
        );
    }

    #[test]
    fn a_bus_without_a_baud_is_refused() {
        let bare = Bus::by_id("kmr-test-no-such-serial").build();
        let res = Robot::new()
            .set_limit_policy_all(Policy::Clamp)
            .add_actuator(Xm430W210::ids(1..=4).on(bare))
            .add_controller(EachTick, |_t: &Time, _r: &mut State, _s: &Sensors| {})
            .run();
        assert!(
            matches!(
                res,
                Err(UserError::BusWithoutBaud {
                    port: "kmr-test-no-such-serial"
                })
            ),
            "{res:?}"
        );
    }

    #[test]
    fn two_handles_on_one_port_are_refused() {
        let fast = Bus::by_id("kmr-test-no-such-serial")
            .baud(3_000_000)
            .build();
        let res = Robot::new()
            .set_limit_policy_all(Policy::Clamp)
            .add_actuator(Xm430W210::ids(1..=2).on(spine()))
            .add_actuator(Xm430W210::ids(3..=4).on(fast))
            .add_controller(EachTick, |_t: &Time, _r: &mut State, _s: &Sensors| {})
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
    }

    #[cfg(not(feature = "dynamixel"))]
    #[test]
    fn actuators_without_the_feature_are_refused() {
        let res = Robot::new()
            .set_limit_policy_all(Policy::Clamp)
            .add_actuator(Xm430W210::ids(1..=4).on(spine()))
            .add_controller(EachTick, |_t: &Time, _r: &mut State, _s: &Sensors| {})
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
    }

    /// The real Dynamixel adapter: the serial resolves to no device, so no port is opened.
    #[cfg(feature = "dynamixel")]
    #[test]
    fn actuators_bind_the_dynamixel_adapter_and_bring_up_names_the_serial() {
        let _bus = bus();
        let res = Robot::new()
            .set_limit_policy_all(Policy::Clamp)
            .add_actuator(Xm430W210::ids(1..=4).on(spine()))
            .add_controller(EachTick, |_t: &Time, _r: &mut State, _s: &Sensors| {})
            .run();
        let Err(e @ UserError::BringUp { .. }) = &res else {
            panic!("no device, yet: {res:?}");
        };
        assert!(
            e.to_string()
                .starts_with("hardware bring-up failed: member 0 (Dynamixel): ")
                && e.to_string().contains("kmr-test-no-such-serial"),
            "{e}"
        );
    }

    /// A prismatic joint: a Dynamixel actuator has no mm conversion (SRS FR-2.2).
    mod prismatic {
        use super::super::*;
        use kmr_api::dynamixel::Xm430W210;

        kmr_api::robot!(model = "tests/fixtures/four.urdf");

        #[test]
        fn a_dynamixel_actuator_on_it_is_refused() {
            let spine = Bus::by_id("kmr-test-no-such-serial")
                .baud(1_000_000)
                .build();
            let res = Robot::new()
                .set_limit_policy_all(kmr_api::Policy::Clamp)
                .add_actuator(Xm430W210::ids(1..=4).on(spine))
                .add_controller(EachTick, |_t: &Time, _r: &mut State, _s: &Sensors| {})
                .run();
            let Err(e) = res else {
                panic!("a prismatic XM430 ran: {res:?}");
            };
            assert!(matches!(e, UserError::HardwareRejected { .. }), "{e:?}");
            assert!(e.to_string().contains("actuator 2: SRS FR-2.2"), "{e}");
        }
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
    use kmr_api::dynamixel::{
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

    impl kmr_api::Spec<N> for TwoBuses {
        type Drivers = kmr_api::HCons<kmr_api::__DxlAdapter<2, N>, kmr_api::HNil>;

        const LINK_NAMES: &'static [&'static str] = &["base"];
        const PRISMATIC: [u64; kmr_api::MASK_WORDS] = [0; kmr_api::MASK_WORDS];
        const CONTINUOUS: [u64; kmr_api::MASK_WORDS] = [0; kmr_api::MASK_WORDS];
        const LOWER: [kmr_api::Q; N] = [kmr_api::Q(-1.0); N];
        const UPPER: [kmr_api::Q; N] = [kmr_api::Q(1.0); N];
        const SPEED: [kmr_api::Qd; N] = [kmr_api::Qd(1.0); N];
        const EFFORT: [kmr_api::Tau; N] = [kmr_api::Tau(1.0); N];
        const PRESET: Option<&'static str> = Some("TwoBuses");

        fn bind(plugins: &[kmr_api::Binding]) -> Result<Self::Drivers, UserError> {
            let hardware = kmr_api::HCons {
                head: TABLE.in_order(),
                tail: kmr_api::HNil,
            };
            kmr_api::__bind::<N, _>(hardware, plugins)
        }

        fn declare(_declared: kmr_api::__Declared<N>) -> Result<Self::Drivers, UserError> {
            kmr_api::__declare_none("TwoBuses")
        }
    }

    #[test]
    fn one_plugin_on_two_buses_is_the_adapters_binding_error() {
        let record = seen();
        let res = kmr_api::Robot::<N, TwoBuses>::new()
            .set_limit_policy_all(kmr_api::Policy::Clamp)
            .add_plugins(Bus::by_id("kmr-test-no-such-serial").bus("left"))
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
            .run();
        let Err(e) = res else {
            panic!("bound one bus of two: {res:?}");
        };
        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: member 0 (Dynamixel): bus right: no bus plugin binds it; \
             add one with `.bus(\"right\")`"
        );
        // UserError -> the hardware error -> the member's own.
        let adapter = std::error::Error::source(&e)
            .and_then(std::error::Error::source)
            .and_then(|s| s.downcast_ref::<DynamixelError>());
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
    use kmr_core::hal::{Batch, Claim, Command, Faults, Group, Hal, Mask, Overrides, Sample, seal};
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

    impl Hal<N> for Recorder {
        type Error = NoAnswer;

        fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, NoAnswer> {
            log("check");
            Ok(Claim {
                owned: Mask::ALL,
                group: [Group(0); N],
                read_every: [NonZeroU32::MIN; N],
                read_cost: Duration::ZERO,
                write_cost: Duration::ZERO,
            })
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

        fn read_all(&mut self, _sample: Sample<'_, N>) -> Result<(), NoAnswer> {
            log("read_all");
            Ok(())
        }

        fn torque_on(&mut self) -> Result<(), NoAnswer> {
            log("torque_on");
            Ok(())
        }

        fn read(&mut self, _sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
            log("read");
            Mask::NONE
        }

        fn write(
            &mut self,
            _command: Command<'_, N>,
            _batch: &Batch<'_, N>,
            _faults: &mut Faults<N>,
        ) {
            log("write");
        }

        fn hold(&mut self, _last: Command<'_, N>) {
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

    impl Spec<N> for Records {
        type Drivers = kmr_api::HCons<Recorder, kmr_api::HNil>;

        const LINK_NAMES: &'static [&'static str] = &["base"];
        const PRISMATIC: [u64; kmr_api::MASK_WORDS] = [0; kmr_api::MASK_WORDS];
        const CONTINUOUS: [u64; kmr_api::MASK_WORDS] = [0; kmr_api::MASK_WORDS];
        const LOWER: [kmr_api::Q; N] = [kmr_api::Q(-1.0); N];
        const UPPER: [kmr_api::Q; N] = [kmr_api::Q(1.0); N];
        const SPEED: [kmr_api::Qd; N] = [kmr_api::Qd(1.0); N];
        const EFFORT: [kmr_api::Tau; N] = [kmr_api::Tau(1.0); N];
        const PRESET: Option<&'static str> = None;

        fn bind(_plugins: &[kmr_api::Binding]) -> Result<Self::Drivers, UserError> {
            Ok(kmr_api::HCons {
                head: Recorder { fail_open: false },
                tail: kmr_api::HNil,
            })
        }

        fn declare(_declared: kmr_api::__Declared<N>) -> Result<Self::Drivers, UserError> {
            log("declare");
            Ok(kmr_api::HCons {
                head: Recorder { fail_open: false },
                tail: kmr_api::HNil,
            })
        }
    }

    /// Binds every plugin to a recorder whose `open` fails.
    struct FailsOpen;

    impl Spec<N> for FailsOpen {
        type Drivers = kmr_api::HCons<Recorder, kmr_api::HNil>;

        const LINK_NAMES: &'static [&'static str] = &["base"];
        const PRISMATIC: [u64; kmr_api::MASK_WORDS] = [0; kmr_api::MASK_WORDS];
        const CONTINUOUS: [u64; kmr_api::MASK_WORDS] = [0; kmr_api::MASK_WORDS];
        const LOWER: [kmr_api::Q; N] = [kmr_api::Q(-1.0); N];
        const UPPER: [kmr_api::Q; N] = [kmr_api::Q(1.0); N];
        const SPEED: [kmr_api::Qd; N] = [kmr_api::Qd(1.0); N];
        const EFFORT: [kmr_api::Tau; N] = [kmr_api::Tau(1.0); N];
        const PRESET: Option<&'static str> = None;

        fn bind(_plugins: &[kmr_api::Binding]) -> Result<Self::Drivers, UserError> {
            Ok(kmr_api::HCons {
                head: Recorder { fail_open: true },
                tail: kmr_api::HNil,
            })
        }

        fn declare(_declared: kmr_api::__Declared<N>) -> Result<Self::Drivers, UserError> {
            log("declare");
            Ok(kmr_api::HCons {
                head: Recorder { fail_open: true },
                tail: kmr_api::HNil,
            })
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
        let res = kmr_api::Robot::<N, Records>::new()
            .set_limit_policy_all(kmr_api::Policy::Clamp)
            .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
            .set_hold(Duration::ZERO)
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
        let res = kmr_api::Robot::<N, FailsOpen>::new()
            .set_limit_policy_all(kmr_api::Policy::Clamp)
            .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
            .add_controller(EachTick, write_then_stop)
            .run();
        let Err(e) = res else {
            panic!("open failed, yet the run went on: {res:?}");
        };
        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: member 0 (Recorder): actuator 2 on serial \
             kmr-test-no-such-serial: no answer"
        );
        // UserError -> the hardware error -> the member's own.
        let source = std::error::Error::source(&e)
            .and_then(std::error::Error::source)
            .and_then(|s| s.downcast_ref::<NoAnswer>());
        assert!(source.is_some(), "{e:?}");
        assert_eq!(take(), ["check", "open", "torque_off", "close"]);
    }

    #[test]
    fn a_limited_actuator_without_a_policy_is_refused_before_any_io() {
        let _bus = bus();
        take();
        let res = kmr_api::Robot::<N, Records>::new()
            .set_limit_policy(0, kmr_api::Policy::Abort)
            .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(
            matches!(res, Err(UserError::NoPolicy { actuator: 1 })),
            "{res:?}"
        );
        assert!(take().is_empty());
    }

    #[test]
    fn a_policy_for_every_actuator_lets_it_run_and_dt_is_still_checked() {
        let _bus = bus();
        take();
        let res = kmr_api::Robot::<N, Records>::new()
            .set_limit_policy_group(&[0, 1], kmr_api::Policy::Warn)
            .set_limit_policy_group(&[2, 3], kmr_api::Policy::Clamp)
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(res.is_ok(), "{res:?}");

        let res = kmr_api::Robot::<N, Records>::new()
            .set_dt(Duration::from_micros(5))
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(
            matches!(res, Err(UserError::DtOutOfRange { .. })),
            "{res:?}"
        );
    }

    #[test]
    fn an_index_past_n_and_a_bad_envelope_are_refused() {
        let res = kmr_api::Robot::<N, Records>::new()
            .set_limit_policy_all(kmr_api::Policy::Clamp)
            .set_limit_policy(N, kmr_api::Policy::Clamp)
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(
            matches!(res, Err(UserError::NoActuator { index: N, n: N })),
            "{res:?}"
        );

        let backwards = kmr_api::Envelope {
            q: Some((Q(1.0), Q(-1.0))),
            ..kmr_api::Envelope::NONE
        };
        let res = kmr_api::Robot::<N, Records>::new()
            .set_limit_policy_all(kmr_api::Policy::Clamp)
            .set_envelope(3, backwards)
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(
            matches!(res, Err(UserError::InvalidEnvelope { actuator: 3 })),
            "{res:?}"
        );
    }

    #[test]
    fn declared_actuators_run_on_the_declared_driver() {
        let _bus = bus();
        take();
        let spine = Bus::by_dev("/dev/kmr-test-none").baud(1_000_000).build();
        let res = kmr_api::Robot::<N, Records>::new()
            .set_limit_policy_all(kmr_api::Policy::Clamp)
            .add_actuator(kmr_api::dynamixel::Rx28::id(7).on(spine))
            .add_actuator(kmr_api::dynamixel::Rx28::ids(1..=3).on(spine))
            .set_hold(Duration::ZERO)
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(res.is_ok(), "{res:?}");
        assert_eq!(take().get(..3), Some(&["declare", "check", "open"][..]));
    }

    #[test]
    fn without_a_plugin_the_bound_driver_is_never_called() {
        let _bus = bus();
        take();
        let res = kmr_api::Robot::<N, Records>::new()
            .set_limit_policy_all(kmr_api::Policy::Clamp)
            .add_controller(EachTick, write_then_stop)
            .run();
        assert!(res.is_ok(), "{res:?}");
        assert!(take().is_empty());
    }
}

/// `robot!(Preset, hardware = |hw| ..)`: the preset's table through const overrides.
mod overrides {
    use kmr_api::dynamixel::Torque;

    kmr_api::robot!(
        XmBench,
        hardware = |hw| hw.read_only(3).read_only_group(&[0, 1])
    );

    #[test]
    fn overrides_mark_actuators_read_only_and_keep_the_shape() {
        let read_only = __TABLE
            .actuators
            .map(|a| matches!(a.torque, Torque::ReadOnly));
        assert_eq!(read_only, [true, true, false, true]);
        assert_eq!((__NB, __M, N), (1, 4, 4));
    }
}

/// `robot!(model = ..)` names no hardware: a plugin is refused before any I/O.
mod no_table {
    use super::*;

    kmr_api::robot!(model = "tests/fixtures/four.urdf");

    #[test]
    fn a_plugin_is_refused_before_the_run() {
        let record = seen();
        let res = Robot::new()
            .set_limit_policy_all(kmr_api::Policy::Clamp)
            .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
            .add_controller_with(EachTick, Arc::clone(&record), stop_at::<0>)
            .run();
        assert!(matches!(res, Err(UserError::NoHardwareDeclared)), "{res:?}");
        assert!(ticks(&record).is_empty());
    }
}

/// `robot!(model = .., hardware = Table { .. })`: one table, one row per actuator.
mod table {
    use kmr_api::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};

    const fn xm430(id: u8) -> Actuator {
        Actuator {
            bus: 0,
            id,
            model: Model::Xm430W210,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
        }
    }

    kmr_api::robot!(
        model = "tests/fixtures/four_dxl.urdf",
        hardware = Table {
            buses: [Bus {
                label: "arm",
                baud: 1_000_000,
                read_every: None
            }],
            actuators: [xm430(1), xm430(2), xm430(3), xm430(4)],
        }
    );

    #[test]
    fn the_table_is_the_robots_hardware_in_model_order() {
        assert_eq!((__NB, __M, N), (1, 4, 4));
        assert_eq!(__TABLE.actuators.map(|a| a.id), [1, 2, 3, 4]);
        assert_eq!(__TABLE.buses[0].label, "arm");
    }
}
