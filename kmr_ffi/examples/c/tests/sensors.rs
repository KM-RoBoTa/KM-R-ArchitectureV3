//! IMU and microphone as C reads them equal the `Sensors` readers, on the core's scripted host
//! and fake sensor (SRS FR-5.7).

use core::cell::{Cell, RefCell};
use std::time::Instant;

use kmr_api::test_util::{
    DT, FakeSensor, Log, PER_TICK, Reading, Recorder, Scripted, TICK_END, run_scripted_all,
};
use kmr_api::{Accel, EachTick, Gyro, HCons, HNil, Sensors, Time};

kmr_api::robot!(XmBench);

/// `kmr_axes`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Axes([f32; 3]);

/// `sensors_out` of tests/sensors.c.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SensorsOut {
    imu_age: u64,
    imu_dropped: u64,
    mic_dropped: u64,
    mic_len: u64,
    accel: Axes,
    gyro: Axes,
    has_imu: u32,
    has_imu_past_slots: u32,
    has_accel: u32,
    has_gyro: u32,
    has_mic: u32,
    mic_rate_hz: u32,
    samples: [i16; 4],
}

/// What the `Sensors` readers give, in the same shape.
fn readers(sensors: &Sensors) -> SensorsOut {
    let mut out = SensorsOut {
        has_imu_past_slots: u32::from(sensors.imu(1).is_some()),
        ..SensorsOut::default()
    };
    if let Some(imu) = sensors.imu(0) {
        out.has_imu = 1;
        out.has_accel = u32::from(imu.accel().is_some());
        out.has_gyro = u32::from(imu.gyro().is_some());
        out.accel = Axes(imu.accel().map_or([0.0; 3], |a| a.0));
        out.gyro = Axes(imu.gyro().map_or([0.0; 3], |g| g.0));
        out.imu_age = imu.age();
        out.imu_dropped = imu.dropped();
    }
    if let Some(mic) = sensors.mic(0) {
        out.has_mic = 1;
        out.mic_len = mic.samples().len() as u64;
        for (to, from) in out.samples.iter_mut().zip(mic.samples()) {
            *to = *from;
        }
        out.mic_rate_hz = mic.rate_hz();
        out.mic_dropped = mic.dropped();
    }
    out
}

#[test]
fn c_reads_the_imu_and_the_microphone_the_readers_give() {
    let log = Log::new();
    let clap = [5_i16, -2000, 40];
    // Read call 0 is PostInit's, call t + 1 tick t's.
    let readings = [
        Reading::QUIET,
        Reading {
            accel: Some(Accel([0.0, 0.0, 9.5])),
            gyro: Some(Gyro([0.1, 0.2, 0.3])),
            audio: &clap,
            ..Reading::QUIET
        },
        Reading::QUIET,
        Reading {
            gyro: Some(Gyro([0.0, 0.0, 1.0])),
            ..Reading::QUIET
        },
    ];
    // C writes through the reference it is handed a pointer to: interior mutability.
    let copy = Cell::new(SensorsOut::default());
    let seen = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_sensor(FakeSensor {
            script: &readings,
            mics: 1,
            ..FakeSensor::new(&log, 0)
        })
        .add_controller(EachTick, kmr_api::c!(copy_sensors, with = &copy))
        .add_controller(EachTick, |_: &Time, _: &mut State, sensors: &Sensors| {
            seen.borrow_mut().push((copy.get(), readers(sensors)));
        });
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 3 * PER_TICK);
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted_all(robot, &mut host, &mut drivers).expect("runs");

    let seen = seen.borrow();
    assert_eq!(seen.len(), 4);
    for (c, rust) in seen.iter() {
        assert_eq!(c, rust);
    }
    let (tick0, _) = seen[0];
    assert_eq!((tick0.has_imu, tick0.has_mic, tick0.mic_len), (1, 1, 3));
    assert_eq!(tick0.samples[..3], clap);
    assert_eq!(tick0.accel, Axes([0.0, 0.0, 9.5]));
    let (tick2, _) = seen[2];
    assert_eq!(
        (tick2.has_accel, tick2.has_gyro, tick2.gyro),
        (0, 1, Axes([0.0, 0.0, 1.0]))
    );
}
