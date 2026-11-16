//! The C path over scripted runs: the snapshot against the `State`, `Time` and `Sensors`
//! readers, the apply step, and the context. Rust `extern "C"` functions stand in for C; the
//! headers' functions over the snapshot are tested in kmr_ffi and kmr_ffi/examples/c. A request
//! raises the global bus, so it is tested in tests/ffi.rs, in its own process.

use core::cell::RefCell;
use core::ffi::c_void;
use core::num::NonZeroU32;
use std::time::Instant;

use super::*;
use crate::test_util::{
    DT, FakeSensor, Faults, Log, PER_TICK, Reading, Recorder, Scripted, TICK_END, run_scripted,
    run_scripted_all,
};
use crate::{Accel, EachTick, Envelope, First, Gyro, HCons, HNil, Qd, Robot, Spec, UserError};

/// Stands for the `__Spec` that `robot!` emits.
struct TestSpec;

impl Spec<4> for TestSpec {
    type Drivers = HCons<crate::__NoHardware, HNil>;

    const LINK_NAMES: &'static [&'static str] = &["base", "tool"];
    const PRISMATIC: [u64; crate::MASK_WORDS] = [0; crate::MASK_WORDS];
    const CONTINUOUS: [u64; crate::MASK_WORDS] = [0; crate::MASK_WORDS];
    const LOWER: [Q; 4] = [Q(-1.0); 4];
    const UPPER: [Q; 4] = [Q(1.0); 4];
    const SPEED: [Qd; 4] = [Qd(1.0); 4];
    const EFFORT: [crate::Tau; 4] = [crate::Tau(1.0); 4];
    const PRESET: Option<&'static str> = None;

    fn bind(plugins: &[crate::Binding]) -> Result<Self::Drivers, UserError> {
        crate::__bind_none(plugins)
    }

    fn declare(_declared: crate::__Declared<4>) -> Result<Self::Drivers, UserError> {
        crate::__bind_none(&[])
    }
}

/// A robot of [`TestSpec`] at [`DT`], its limits clamped.
fn robot() -> Robot<4, TestSpec> {
    Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(crate::Policy::Clamp)
}

/// What the stand-ins copy out of a call: the three handles.
type Seen = RefCell<Vec<(__Time, __State<4>, __Sensors)>>;

/// Copies the snapshot into `ctx`, a `&Seen`.
unsafe extern "C" fn copy(
    time: *const __Time,
    state: *mut __State<4>,
    sensors: *const __Sensors,
    ctx: *mut c_void,
) {
    // SAFETY: `invoke` passes live handles; these tests give a `&Seen` context.
    let (time, state, sensors, seen) =
        unsafe { (&*time, &*state, &*sensors, &*ctx.cast::<&Seen>()) };
    seen.borrow_mut().push((*time, *state, *sensors));
}

/// Stages `q[1]` = 0.5 and `kd[3]` = 2; leaves `q[2]` = 9 unflagged; flags bit 10, past N.
extern "C" fn some(_: *const __Time, state: *mut __State<4>, _: *const __Sensors, _: *mut c_void) {
    // SAFETY: `invoke` passes a live exclusive state.
    let w = unsafe { &mut (*state).write };
    w.q[1] = 0.5;
    w.written.q |= 1 << 1;
    w.q[2] = 9.0;
    w.kd[3] = 2.0;
    w.written.kd |= 1 << 3;
    w.written.tau |= 1 << 10;
}

/// A C controller over `f` with the context `seen`.
fn c_with(f: __CFn<4>, seen: &Seen) -> CController<4, &Seen> {
    CController::__new("stand_in", f, Some(seen))
}

/// The mask as one word.
fn bits(mask: crate::Mask<4>) -> u64 {
    mask.iter().fold(0, |w, i| w | 1 << i)
}

/// Whether `snap` answers what the `State` readers return, at every depth.
fn agrees(snap: &__State<4>, state: &State<'_, 4>) -> bool {
    let filled = (0..HISTORY_DEPTH)
        .take_while(|&k| {
            !matches!(
                state.prev_q_at(0, k),
                Err(StateError::HistoryValueOutOfRange { .. })
            )
        })
        .count();
    let values = (0..filled).all(|k| {
        let slot = &snap.slot[k];
        (0..4).all(|i| {
            let sampled = slot.sampled & (1 << i) != 0;
            let same = |got: Result<f32, StateError>, mine: f32| match got {
                Ok(v) => sampled && v == mine,
                Err(StateError::NoSample { .. }) => !sampled,
                Err(_) => false,
            };
            same(state.prev_q_at(i, k).map(|v| v.0), slot.q[i])
                && same(state.prev_qd_at(i, k).map(|v| v.0), slot.qd[i])
                && same(state.prev_tau_at(i, k).map(|v| v.0), slot.tau[i])
                && same(state.prev_kp_at(i, k).map(|v| v.0), slot.kp[i])
                && same(state.prev_kd_at(i, k).map(|v| v.0), slot.kd[i])
        })
    });
    let faults = (0..HISTORY_DEPTH).all(|k| match state.prev_faults(k) {
        Ok(f) => {
            let c = &snap.slot[k].faults;
            k < snap.faults_filled as usize
                && c.stale == bits(f.stale())
                && c.bus_error == bits(f.bus_error())
                && c.offline == bits(f.offline())
                && c.rejected == bits(f.rejected())
                && c.joint_limit == bits(f.joint_limit())
                && c.envelope == bits(f.envelope())
                && (c.tick & kmr_ffi::TICK_OVERRUN != 0) == f.overrun()
        }
        Err(_) => k >= snap.faults_filled as usize,
    });
    let per_actuator = (0..4).all(|i| {
        let bit = 1u64 << i;
        let age = match state.age_at(i) {
            Ok(Some(a)) => snap.has_age & bit != 0 && snap.age[i] == a,
            Ok(None) => snap.has_age & bit == 0,
            Err(_) => false,
        };
        let temperature = match state.temperature_at(i) {
            Ok(Some(c)) => snap.has_temperature & bit != 0 && snap.temperature[i] == c.0,
            Ok(None) => snap.has_temperature & bit == 0,
            Err(_) => false,
        };
        let every = state
            .read_every(i)
            .is_ok_and(|e| e.get() == snap.read_every[i]);
        age && temperature && every
    });
    let sample_age = match state.sample_age() {
        Some(a) => snap.has_sample_age == 1 && snap.sample_age == a,
        None => snap.has_sample_age == 0,
    };
    snap.filled as usize == filled && values && faults && per_actuator && sample_age
}

#[test]
fn the_snapshot_answers_what_the_state_readers_return() {
    let log = Log::new();
    let mut faults = Faults::default();
    faults.stale.set(1);
    faults.offline.set(3);
    let mut drivers = HCons {
        head: Recorder {
            faults,
            faults_on: Some(2),
            every: NonZeroU32::new(2).expect("2 is not zero"),
            temperature: Some(30.0),
            ..Recorder::new(&log)
        },
        tail: HNil,
    };
    let seen = Seen::default();
    let ok = RefCell::new(Vec::new());
    let robot = robot()
        .add_controller(First, c_with(copy, &seen))
        .add_controller(First, |_: &Time, state: &mut State<'_, 4>, _: &Sensors| {
            let seen = seen.borrow();
            let (_, snap, _) = seen.last().expect("one call");
            ok.borrow_mut().push(agrees(snap, state));
        });
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 6 * PER_TICK);
    run_scripted(robot, &mut host, &mut drivers).expect("runs");

    assert_eq!(*ok.borrow(), [true; 7]);
    let seen = seen.borrow();
    let (_, tick2, _) = &seen[2];
    assert_eq!(tick2.slot[0].faults.stale, 1 << 1);
    assert_eq!(tick2.slot[0].faults.offline, 1 << 3);
    let ages: Vec<u32> = seen.iter().map(|(_, s, _)| s.age[0]).collect();
    assert_eq!(ages, [0, 1, 0, 1, 0, 1, 0]);
    assert!(seen.iter().all(|(_, s, _)| s.read_every[0] == 2));
    assert!(
        seen.iter()
            .skip(1)
            .all(|(_, s, _)| s.has_temperature == 0b1111)
    );
}

#[test]
fn a_run_without_hardware_has_no_sample_and_no_age() {
    let seen = Seen::default();
    let robot = robot().add_controller(EachTick, c_with(copy, &seen));
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 2 * PER_TICK);
    let mut drivers = HCons {
        head: crate::NoHardware,
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");

    let seen = seen.borrow();
    assert_eq!(seen.len(), 3);
    for (_, s, _) in seen.iter() {
        assert!(
            s.slot[..s.filled as usize]
                .iter()
                .all(|slot| slot.sampled == 0)
        );
        assert_eq!((s.has_age, s.has_sample_age), (0, 0));
    }
}

#[test]
fn the_clock_equals_the_time_getters() {
    let seen = Seen::default();
    let getters = RefCell::new(Vec::new());
    let robot = robot()
        .add_controller(EachTick, c_with(copy, &seen))
        .add_controller(
            EachTick,
            |time: &Time, _: &mut State<'_, 4>, _: &Sensors| {
                getters.borrow_mut().push(CTime {
                    tick: time.tick(),
                    overrun_count: time.overrun_count(),
                    elapsed: time.elapsed().into(),
                    dt: time.dt().into(),
                    last_overrun: time.last_overrun().map_or(CDuration::default(), Into::into),
                    accumulated_overrun: time.accumulated_overrun().into(),
                    late: u32::from(time.last_overrun().is_some()),
                    reserved: 0,
                });
            },
        );
    // `now` at arm, then at the start and the end of each tick: tick 0 ends at 1.5 dt.
    let mut host = Scripted::new(Instant::now())
        .script(&[0, 0, 15, 20, 25, 30, 35])
        .shutdown_from(TICK_END + 2 * PER_TICK);
    let log = Log::new();
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");

    let times: Vec<CTime> = seen.borrow().iter().map(|(t, _, _)| *t).collect();
    assert_eq!(times, *getters.borrow());
    assert_eq!(times[1].overrun_count, 1);
    assert_eq!((times[1].late, times[1].last_overrun.nanos), (1, 500_000));
    assert_eq!(times[2].tick, 2);
}

#[test]
fn the_sensors_equal_the_sensor_readers() {
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
    ];
    let seen = Seen::default();
    let robot = robot()
        .add_sensor(FakeSensor {
            script: &readings,
            mics: 1,
            ..FakeSensor::new(&log, 0)
        })
        .add_controller(EachTick, c_with(copy, &seen));
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 2 * PER_TICK);
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted_all(robot, &mut host, &mut drivers).expect("runs");

    let seen = seen.borrow();
    let (_, _, tick0) = &seen[0];
    let imu = tick0.imu[0];
    assert_eq!(
        (imu.present, imu.has_accel, imu.has_gyro, imu.age),
        (1, 1, 1, 0)
    );
    assert_eq!((imu.accel.z, imu.gyro.y), (9.5, 0.2));
    assert_eq!(tick0.imu[1].present, 0);
    let mic = &tick0.mic[0];
    assert_eq!((mic.present, mic.len, mic.rate_hz), (1, 3, 16_000));
    assert_eq!(&mic.samples[..3], &clap);
    let (_, _, tick1) = &seen[1];
    assert_eq!((tick1.imu[0].age, tick1.mic[0].len), (1, 0));
}

/// Runs one tick whose `EachTick` is the stand-in `f` on a robot with `envelope` on actuator
/// 1; returns what the adapter was sent.
fn one_tick(f: __CFn<4>, envelope: Envelope) -> Option<kmr_core::test_util::Sent<4>> {
    let robot = robot()
        .set_envelope(1, envelope)
        .add_controller(EachTick, CController::<4, ()>::__new("stand_in", f, None));
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);
    let log = Log::new();
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    log.write.get()
}

#[test]
fn only_the_staged_entries_are_applied() {
    let sent = one_tick(some, Envelope::NONE).expect("one write");
    assert_eq!(sent.q[1], Q(0.5));
    assert_eq!(bits(sent.written.q), 1 << 1);
    assert_eq!(bits(sent.written.kd), 1 << 3);
    assert!(sent.written.qd.is_empty() && sent.written.tau.is_empty());
    assert!(sent.written.kp.is_empty());
}

#[test]
fn a_staged_value_outside_the_envelope_is_clamped_as_a_rust_write() {
    let envelope = Envelope {
        q: Some((Q(-0.25), Q(0.25))),
        ..Envelope::NONE
    };
    let sent = one_tick(some, envelope).expect("one write");
    assert_eq!(sent.q[1], Q(0.25));
}

/// Counts its calls in `ctx`, a `u32`, and stages the count as `q[0]`.
unsafe extern "C" fn count(
    _: *const __Time,
    state: *mut __State<4>,
    _: *const __Sensors,
    ctx: *mut c_void,
) {
    // SAFETY: `invoke` passes a live exclusive state; this test gives a `u32` context.
    let (state, n) = unsafe { (&mut *state, &mut *ctx.cast::<u32>()) };
    *n += 1;
    state.write.q[0] = *n as f32 / 10.0;
    state.write.written.q = 1;
}

#[test]
fn the_context_is_kept_across_ticks() {
    let robot = robot().add_controller(EachTick, CController::__new("count", count, Some(0u32)));
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 2 * PER_TICK);
    let log = Log::new();
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    let sent = log.write.get().expect("a write");
    assert_eq!(sent.q[0], Q(0.3));
}
