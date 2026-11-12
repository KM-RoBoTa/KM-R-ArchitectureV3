//! Sensors through kmr_api: `add_sensor`, the controller's `&Sensors`, the tick's faults and
//! the typed error of a run, on the core's scripted host and fake sensor (SRS FR-5.7).

use std::cell::RefCell;
use std::error::Error;
use std::time::{Duration, Instant};

use kmr_api::test_util::Call::{Check, SensorClose, SensorOpen};
use kmr_api::test_util::{
    FakeSensor, FakeSensorError, Log, PER_TICK, Reading, Recorder, Scripted, Stop, TICK_END,
    run_scripted_all,
};
use kmr_api::{
    Accel, Drive, EachTick, Empty, Filled, First, Gyro, HCons, HNil, MemberError, Scheduled,
    SensorPolicy, Sensors, Time, UserError,
};

kmr_api::robot!(XmBench);

/// The tick period of every run here.
const DT: Duration = Duration::from_millis(1);

/// A host that shuts the run down after `ticks` ticks.
fn host(ticks: usize) -> Scripted {
    Scripted::new(Instant::now()).shutdown_from(TICK_END + (ticks - 1) * PER_TICK)
}

/// The drivers of a scripted run: one recorder logging to `log`.
fn drivers(log: &Log<N>) -> HCons<Recorder<'_, N>, HNil> {
    HCons {
        head: Recorder::new(log),
        tail: HNil,
    }
}

/// A controller that does nothing.
fn idle(_time: &Time, _state: &mut State, _sensors: &Sensors) {}

/// A reading of `x` m/s² along x, nothing else.
fn along_x(x: f32) -> Reading<'static> {
    Reading {
        accel: Some(Accel([x, 0.0, 0.0])),
        ..Reading::QUIET
    }
}

#[test]
fn a_controller_reads_the_imu_through_the_api() {
    let log = Log::new();
    // Read call 0 is PostInit's, call t + 1 tick t's: one reading, in tick 0.
    let script = [
        Reading::QUIET,
        Reading {
            accel: Some(Accel([0.0, 0.0, 9.806_65])),
            gyro: Some(Gyro([0.1, 0.2, 0.3])),
            ..Reading::QUIET
        },
    ];
    let seen = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_sensor(FakeSensor {
            script: &script,
            ..FakeSensor::new(&log, 0)
        })
        .add_controller(First, |_: &Time, _: &mut State, sensors: &Sensors| {
            let imu = sensors
                .imu(0)
                .map(|imu| (imu.accel(), imu.gyro(), imu.age(), imu.dropped()));
            seen.borrow_mut().push((imu, sensors.imu(1).is_none()));
        });

    let stop = run_scripted_all(robot, &mut host(3), &mut drivers(&log)).expect("the run");

    assert_eq!(stop, Stop::Shutdown);
    let reading = (
        Some(Accel([0.0, 0.0, 9.806_65])),
        Some(Gyro([0.1, 0.2, 0.3])),
    );
    assert_eq!(
        *seen.borrow(),
        [0, 1, 2].map(|age| (Some((reading.0, reading.1, age, 0)), true))
    );
    let z = seen.borrow()[0]
        .0
        .and_then(|(accel, ..)| accel)
        .map(Accel::z);
    assert_eq!(z, Some(9.806_65));
}

#[test]
fn a_controller_reads_audio_through_the_api() {
    let log = Log::new();
    let audio: [i16; 96] = core::array::from_fn(|k| i16::try_from(k).expect("small"));
    let script = [
        Reading::QUIET,
        Reading {
            audio: &audio,
            ..Reading::QUIET
        },
    ];
    let seen = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_sensor(FakeSensor {
            imus: 0,
            mics: 1,
            script: &script,
            ..FakeSensor::new(&log, 0)
        })
        .add_controller(First, |_: &Time, _: &mut State, sensors: &Sensors| {
            let mic = sensors
                .mic(0)
                .map(|mic| (mic.samples().to_vec(), mic.rate_hz(), mic.dropped()));
            seen.borrow_mut().push(mic);
        });

    run_scripted_all(robot, &mut host(2), &mut drivers(&log)).expect("the run");

    assert_eq!(
        *seen.borrow(),
        [
            Some((audio.to_vec(), 16_000, 0)),
            Some((Vec::new(), 16_000, 0))
        ]
    );
}

#[test]
fn faults_report_sensor_missing_when_the_sensor_goes_silent() {
    let log = Log::new();
    // Readings up to tick 2, then silence; the limit is 3 ticks.
    let script = [along_x(1.0); 4];
    let seen = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .set_sensor_policy(SensorPolicy::Flag)
        .add_controller(First, |_: &Time, state: &mut State, _: &Sensors| {
            let missing = state.faults().map(|faults| faults.sensor_missing());
            seen.borrow_mut().push(missing);
        })
        .add_sensor(FakeSensor {
            max_silence: DT * 3,
            script: &script,
            ..FakeSensor::new(&log, 0)
        });

    let stop = run_scripted_all(robot, &mut host(9), &mut drivers(&log)).expect("the run");

    assert_eq!(stop, Stop::Shutdown);
    assert_eq!(
        *seen.borrow(),
        [false, false, false, false, false, false, true, true, true].map(Some)
    );
}

#[test]
fn two_sensors_are_indexed_in_add_order() {
    let log = Log::new();
    let first = [along_x(1.0)];
    let second = [along_x(2.0)];
    let seen = RefCell::new(None);
    let robot = Robot::new()
        .set_dt(DT)
        .add_sensor(FakeSensor {
            script: &first,
            ..FakeSensor::new(&log, 0)
        })
        .add_sensor(FakeSensor {
            script: &second,
            ..FakeSensor::new(&log, 1)
        })
        .add_controller(
            kmr_api::PostInit,
            |_: &Time, _: &mut State, sensors: &Sensors| {
                let x = |i| sensors.imu(i).and_then(|imu| imu.accel()).map(Accel::x);
                *seen.borrow_mut() = Some((x(0), x(1), x(2)));
            },
        );

    run_scripted_all(robot, &mut host(1), &mut drivers(&log)).expect("the run");

    assert_eq!(*seen.borrow(), Some((Some(1.0), Some(2.0), None)));
    let opened: Vec<_> = log
        .calls()
        .iter()
        .copied()
        .filter(|call| matches!(call, SensorOpen(_)))
        .collect();
    assert_eq!(opened, [SensorOpen(0), SensorOpen(1)]);
}

#[test]
fn a_sensor_error_reaches_the_user_typed() {
    let log = Log::<N>::new();
    let failing = || FakeSensor {
        fail_open: true,
        ..FakeSensor::new(&log, 3)
    };

    let result = Robot::new()
        .set_dt(DT)
        .add_controller(EachTick, idle)
        .add_sensor(failing())
        .run();

    let Err(UserError::Sensor {
        source:
            MemberError::Head {
                error: FakeSensorError::Open { id: 3 },
                ..
            },
    }) = &result
    else {
        panic!("not the fake's own error: {result:?}");
    };
    let error = result.expect_err("matched above");
    assert_eq!(
        error.to_string(),
        "sensor bring-up failed: member 0 (FakeSensor): fake sensor 3: open refused by its script"
    );
    let own = error.source().and_then(Error::source);
    assert_eq!(
        own.and_then(|own| own.downcast_ref::<FakeSensorError>()),
        Some(&FakeSensorError::Open { id: 3 })
    );
    assert_eq!(*log.calls(), [SensorOpen(3), SensorClose(3)]);

    let bound = Robot::new()
        .add_sensor(failing())
        .add_controller(EachTick, idle)
        .bind()
        .expect("nothing to bind");
    let error = bound.run().err();
    assert!(
        matches!(
            error,
            Some(UserError::Sensor {
                source: MemberError::Head {
                    error: FakeSensorError::Open { id: 3 },
                    ..
                }
            })
        ),
        "{error:?}"
    );
}

#[test]
fn more_microphones_than_slots_are_refused_before_any_open() {
    let log = Log::new();
    let mic = |id| FakeSensor {
        imus: 0,
        mics: 1,
        ..FakeSensor::new(&log, id)
    };
    let robot = Robot::new()
        .set_dt(DT)
        .add_sensor(mic(0))
        .add_sensor(mic(1))
        .add_controller(EachTick, idle);

    let result = run_scripted_all(robot, &mut host(1), &mut drivers(&log));

    let Err(error) = result else {
        panic!("two microphones ran: {result:?}");
    };
    assert!(
        matches!(
            error,
            UserError::TooManySensors {
                kind: "microphone",
                asked: 2,
                slots: 1
            }
        ),
        "{error:?}"
    );
    assert!(error.to_string().contains("`.add_sensor(..)`"), "{error}");
    // The drivers' claim only: no driver and no sensor opened.
    assert_eq!(*log.calls(), [Check]);
}

/// The run of a robot without a sensor, by the signature it has always had.
fn run_as_before<C: Drive<N>>(
    _robot: &kmr_api::Robot<N, __Spec, C, Filled>,
) -> fn(kmr_api::Robot<N, __Spec, C, Filled>) -> Result<(), UserError> {
    <kmr_api::Robot<N, __Spec, C, Filled>>::run
}

#[test]
fn a_robot_without_sensors_still_builds_with_its_old_type() {
    let robot: kmr_api::Robot<N, __Spec> = Robot::new();
    let robot: kmr_api::Robot<N, __Spec, Scheduled, Empty> = robot.set_dt(DT);
    let built = robot.add_controller(EachTick, idle);

    let _run = run_as_before(&built);
    let _bound: Result<kmr_api::Bound<N, __Spec, _>, UserError> = built.bind();
}
