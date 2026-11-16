//! The C++ layer of `kmr.hpp` (kmr_ffi T019, SC-007 C++ part): an exception leaving the body
//! applies nothing, a typed context is the robot's for the whole run, the methods answer what
//! the Rust readers return, and `swing` moves actuators 0 and 2.

use core::cell::{Cell, RefCell};
use core::num::NonZeroU32;
use std::time::Instant;

use kmr_api::ffi::CController;
use kmr_api::test_util::{
    DT, Faults, Log, PER_TICK, Recorder, Scripted, Sent, TICK_END, run_scripted,
};
use kmr_api::{EachTick, HCons, HISTORY_DEPTH, HNil, Q, Sensors, StateError, Time};

kmr_api::robot!(XmBench);

/// What `controller` sent over `ticks` scripted ticks on a recorder.
fn sent_by<T>(controller: CController<N, T>, ticks: usize) -> Option<Sent<N>> {
    let log = Log::new();
    let robot = Robot::new().set_dt(DT).add_controller(EachTick, controller);
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + (ticks - 1) * PER_TICK);
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    log.write.get()
}

#[test]
fn an_exception_applies_nothing() {
    assert_eq!(sent_by(kmr_api::c!(throwing_cpp), 1), None);
}

#[test]
fn a_typed_context_is_the_robots_for_the_whole_run() {
    let sent = sent_by(kmr_api::c!(counting_cpp, with = 0_u32), 3).expect("a write");
    assert_eq!(sent.q[0], Q(3.0));
}

#[test]
fn swing_moves_actuators_0_and_2_from_the_newest_sample() {
    let sent = sent_by(kmr_api::c!(swing_cpp), 3).expect("a write");
    // The recorder's reads move actuator 0 by one a tick from the seed: 3 reads by tick 2.
    let seed = kmr_api::test_util::SEED.0;
    assert_eq!(sent.q[0], Q(seed + 3.0 + 0.1));
    assert_eq!(sent.q[2], Q(seed + 0.1));
    assert_eq!(sent.written.q.iter().collect::<Vec<_>>(), [0, 2]);
}

/// `CppOut` of tests/cpp.cpp.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct CppOut {
    q1: f32,
    group_tau: [f32; 2],
    age3: u32,
    q1_status: i32,
    past_n_status: i32,
    deep_status: i32,
    prev_qd_status: i32,
    has_q: i32,
    has_age3: i32,
    age_past_n_status: i32,
    has_faults: i32,
    offline3: i32,
    has_group_tau: i32,
    late: i32,
    has_mic: i32,
}

/// `kmr_status` of a Rust result.
fn code<T>(r: &Result<T, StateError>) -> i32 {
    match r {
        Ok(_) => 0,
        Err(StateError::OutOfRange { .. }) => 1,
        Err(StateError::HistoryValueOutOfRange { .. }) => 2,
        Err(StateError::CriticalValueMissing) => 3,
        Err(StateError::NoSample { .. }) => 4,
    }
}

/// What the Rust readers answer, in the shape of `CppOut`.
fn readers(time: &Time, state: &mut State, sensors: &Sensors) -> CppOut {
    let q1 = state.q_at(1);
    let age = state.age_at(3);
    let faults = state.faults();
    let tau = state.group([2, 0]).tau();
    CppOut {
        q1: q1.as_ref().map_or(-1.0, |q| q.0),
        group_tau: tau.map_or([0.0; 2], |t| t.map(|x| x.0)),
        age3: age.as_ref().ok().copied().flatten().unwrap_or(0),
        q1_status: code(&q1),
        past_n_status: code(&state.q_at(N)),
        deep_status: code(&state.prev_kp_at(0, HISTORY_DEPTH)),
        prev_qd_status: code(&state.prev_qd(1)),
        has_q: i32::from(state.q().is_some()),
        has_age3: i32::from(matches!(age, Ok(Some(_)))),
        age_past_n_status: code(&state.age_at(N)),
        has_faults: i32::from(faults.is_some()),
        offline3: i32::from(faults.is_some_and(|f| f.offline().get(3))),
        has_group_tau: i32::from(tau.is_some()),
        late: i32::from(time.last_overrun().is_some()),
        has_mic: i32::from(sensors.mic(0).is_some()),
    }
}

#[test]
fn the_methods_answer_what_the_rust_readers_return() {
    let log = Log::new();
    let mut faults = Faults::default();
    faults.offline.set(3);
    let mut drivers = HCons {
        head: Recorder {
            faults,
            faults_on: Some(2),
            every: NonZeroU32::new(2).expect("2 is not zero"),
            ..Recorder::new(&log)
        },
        tail: HNil,
    };
    // C++ writes through the reference it is handed a pointer to: interior mutability.
    let copy = Cell::new(CppOut::default());
    let seen = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(EachTick, kmr_api::c!(read_methods_cpp, with = &copy))
        .add_controller(
            EachTick,
            |time: &Time, state: &mut State, sensors: &Sensors| {
                seen.borrow_mut()
                    .push((copy.get(), readers(time, state, sensors)));
            },
        );
    let mut host = Scripted::new(Instant::now())
        .script(&[0, 0, 15, 20, 25, 30, 35, 40, 45])
        .shutdown_from(TICK_END + 4 * PER_TICK);
    run_scripted(robot, &mut host, &mut drivers).expect("runs");

    let seen = seen.borrow();
    assert_eq!(seen.len(), 5);
    for (cpp, rust) in seen.iter() {
        assert_eq!(cpp, rust);
    }
    assert!(seen.iter().any(|(cpp, _)| cpp.offline3 == 1));
    assert!(seen.iter().any(|(cpp, _)| cpp.late == 1));
    assert!(
        seen.iter()
            .all(|(cpp, _)| cpp.past_n_status == 1 && cpp.deep_status == 2)
    );
}
