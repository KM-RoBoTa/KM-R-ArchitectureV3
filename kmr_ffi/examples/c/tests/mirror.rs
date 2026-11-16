//! Every state read as C makes it through `kmr.h` answers what the `State` readers return, each
//! tick, errors included (kmr_ffi T025, SRS FR-2.1, FR-2.3, FR-2.6, FR-5.11, FR-13.4): History
//! at every depth and one past it, an index past N, staleness, faults, ages, cadence,
//! telemetry and a group.

use core::cell::{Cell, RefCell};
use core::num::NonZeroU32;
use std::time::Instant;

use kmr_api::test_util::{DT, Faults, Log, PER_TICK, Recorder, Scripted, TICK_END, run_scripted};
use kmr_api::{EachTick, HCons, HISTORY_DEPTH, HNil, Mask, Sensors, StateError, Time};

kmr_api::robot!(XmBench);

/// Every actuator and one past N.
const M: usize = N + 1;
/// Every depth and one past the History.
const D: usize = HISTORY_DEPTH + 1;

/// `mirror` of tests/mirror.c.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct Mirror {
    q_at: [f32; M],
    prev_q_at: [[f32; M]; D],
    prev_kd_at: [[f32; M]; D],
    prev_tau: [[f32; N]; HISTORY_DEPTH],
    temperature: [f32; M],
    group_q: [f32; 2],
    group_prev_qd: [f32; 2],
    stale: [u64; D],
    offline: [u64; D],
    telemetry_age: [u64; M],
    age: [u32; M],
    read_every: [u32; M],
    sample_age: u32,
    group_sample_age: u32,
    q_at_status: [i32; M],
    prev_q_at_status: [[i32; M]; D],
    prev_kd_at_status: [[i32; M]; D],
    prev_tau_status: [i32; D],
    faults_status: [i32; D],
    overrun: [i32; D],
    age_status: [i32; M],
    read_every_status: [i32; M],
    temperature_status: [i32; M],
    telemetry_age_status: [i32; M],
    stale_at: [i32; M],
    has_q: i32,
    has_sample_age: i32,
    group_q_ok: i32,
    group_prev_qd_status: i32,
    group_stale: i32,
    group_sample_age_ok: i32,
}

/// `kmr_status` of a Rust result, `KMR_NONE` (5) for `Ok(None)`.
fn code<T>(r: &Result<T, StateError>) -> i32 {
    match r {
        Ok(_) => 0,
        Err(StateError::OutOfRange { .. }) => 1,
        Err(StateError::HistoryValueOutOfRange { .. }) => 2,
        Err(StateError::CriticalValueMissing) => 3,
        Err(StateError::NoSample { .. }) => 4,
    }
}

/// `kmr_status` of a Rust `Result<Option<_>>`.
fn code_opt<T>(r: &Result<Option<T>, StateError>) -> i32 {
    match r {
        Ok(None) => 5,
        other => code(other),
    }
}

/// The mask as one word.
fn word(mask: Mask<N>) -> u64 {
    mask.iter().fold(0, |w, i| w | 1 << i)
}

/// The differences between what C read and what the readers give, by name.
fn differences(c: &Mirror, state: &mut State) -> Vec<String> {
    let mut out = Vec::new();
    let mut check = |what: String, same: bool| {
        if !same {
            out.push(what);
        }
    };
    for i in 0..M {
        let q = state.q_at(i);
        check(
            format!("q_at({i})"),
            c.q_at_status[i] == code(&q) && q.is_err_or(|v| v.0 == c.q_at[i]),
        );
        let age = state.age_at(i);
        check(
            format!("age_at({i})"),
            c.age_status[i] == code_opt(&age) && !matches!(age, Ok(Some(a)) if a != c.age[i]),
        );
        let every = state.read_every(i);
        check(
            format!("read_every({i})"),
            c.read_every_status[i] == code(&every)
                && every.is_err_or(|e| e.get() == c.read_every[i]),
        );
        let t = state.temperature_at(i);
        check(
            format!("temperature_at({i})"),
            c.temperature_status[i] == code_opt(&t)
                && !matches!(t, Ok(Some(v)) if v.0 != c.temperature[i]),
        );
        let ta = state.telemetry_age_at(i);
        check(
            format!("telemetry_age_at({i})"),
            c.telemetry_age_status[i] == code_opt(&ta)
                && !matches!(ta, Ok(Some(a)) if a != c.telemetry_age[i]),
        );
        check(
            format!("stale_at({i})"),
            (c.stale_at[i] != 0) == state.stale_at(i),
        );
        for d in 0..D {
            let q = state.prev_q_at(i, d);
            check(
                format!("prev_q_at({i}, {d})"),
                c.prev_q_at_status[d][i] == code(&q) && q.is_err_or(|v| v.0 == c.prev_q_at[d][i]),
            );
            let kd = state.prev_kd_at(i, d);
            check(
                format!("prev_kd_at({i}, {d})"),
                c.prev_kd_at_status[d][i] == code(&kd)
                    && kd.is_err_or(|v| v.0 == c.prev_kd_at[d][i]),
            );
        }
    }
    for d in 0..D {
        let tau = state.prev_tau(d);
        check(
            format!("prev_tau({d})"),
            c.prev_tau_status[d] == code(&tau)
                && tau.is_err_or(|v| c.prev_tau.get(d).is_some_and(|row| v.map(|x| x.0) == *row)),
        );
        let f = state.prev_faults(d);
        check(
            format!("prev_faults({d})"),
            c.faults_status[d] == code(&f)
                && f.is_err_or(|f| {
                    c.stale[d] == word(f.stale())
                        && c.offline[d] == word(f.offline())
                        && (c.overrun[d] != 0) == f.overrun()
                }),
        );
    }
    check("q()".into(), (c.has_q != 0) == state.q().is_some());
    let sample_age = state.sample_age();
    check(
        "sample_age()".into(),
        (c.has_sample_age != 0) == sample_age.is_some()
            && sample_age.is_none_or(|a| a == c.sample_age),
    );
    let group = state.group([3, 1]);
    let q = group.q();
    check(
        "group.q()".into(),
        (c.group_q_ok != 0) == q.is_some() && q.is_none_or(|v| v.map(|x| x.0) == c.group_q),
    );
    let qd = group.prev_qd(1);
    check(
        "group.prev_qd(1)".into(),
        c.group_prev_qd_status == code(&qd) && qd.is_err_or(|v| v.map(|x| x.0) == c.group_prev_qd),
    );
    check(
        "group.stale()".into(),
        (c.group_stale != 0) == group.stale(),
    );
    let age = group.sample_age();
    check(
        "group.sample_age()".into(),
        (c.group_sample_age_ok != 0) == age.is_some()
            && age.is_none_or(|a| a == c.group_sample_age),
    );
    out
}

/// `Ok(v)` satisfies `f`, or it is an error.
trait ErrOr<T> {
    /// Whether this is an error or its value satisfies `f`.
    fn is_err_or(&self, f: impl FnOnce(&T) -> bool) -> bool;
}

impl<T, E> ErrOr<T> for Result<T, E> {
    fn is_err_or(&self, f: impl FnOnce(&T) -> bool) -> bool {
        match self {
            Ok(v) => f(v),
            Err(_) => true,
        }
    }
}

#[test]
fn c_reads_what_the_state_readers_return() {
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
    // C writes through the reference it is handed a pointer to: interior mutability.
    let copy = Cell::new(Mirror {
        q_at: [0.0; M],
        prev_q_at: [[0.0; M]; D],
        prev_kd_at: [[0.0; M]; D],
        prev_tau: [[0.0; N]; HISTORY_DEPTH],
        temperature: [0.0; M],
        group_q: [0.0; 2],
        group_prev_qd: [0.0; 2],
        stale: [0; D],
        offline: [0; D],
        telemetry_age: [0; M],
        age: [0; M],
        read_every: [0; M],
        sample_age: 0,
        group_sample_age: 0,
        q_at_status: [0; M],
        prev_q_at_status: [[0; M]; D],
        prev_kd_at_status: [[0; M]; D],
        prev_tau_status: [0; D],
        faults_status: [0; D],
        overrun: [0; D],
        age_status: [0; M],
        read_every_status: [0; M],
        temperature_status: [0; M],
        telemetry_age_status: [0; M],
        stale_at: [0; M],
        has_q: 0,
        has_sample_age: 0,
        group_q_ok: 0,
        group_prev_qd_status: 0,
        group_stale: 0,
        group_sample_age_ok: 0,
    });
    let diffs = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(EachTick, kmr_api::c!(read_all, with = &copy))
        .add_controller(EachTick, |time: &Time, state: &mut State, _: &Sensors| {
            for d in differences(&copy.get(), state) {
                diffs
                    .borrow_mut()
                    .push(format!("tick {}: {d}", time.tick()));
            }
        });
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 7 * PER_TICK);
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    assert_eq!(*diffs.borrow(), Vec::<String>::new());
}
