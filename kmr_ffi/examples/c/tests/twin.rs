//! A C forwarder against its Rust twin over scripted ticks (kmr_ffi T016, SC-001): the same
//! commands bit for bit, from an inline controller and from one with a context; nothing
//! written sends nothing; a Rust write C leaves alone is kept; two registrations of one C
//! function run in declaration order; a NaN from C ends as a NaN from Rust.

use core::cell::{Cell, RefCell};
use std::time::Instant;

use kmr_api::ffi::{Status, call};
use kmr_api::test_util::{DT, Log, Recorder, Sent, Ticking, run_scripted};
use kmr_api::{
    Drive, EachTick, Envelope, Faults, Filled, First, HCons, HNil, Q, Qd, Sensors, Tau, Time,
};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn twin; fn silent; fn skip_one; fn ctx_value; fn nan_q0; }

/// Ticks of the long runs.
const TICKS: u64 = 1000;

/// What one tick sent and the faults the next tick read for it.
type Tick = (Option<Sent<N>>, Option<Faults<N>>);

/// The step `twin` adds to q, by tick % 7, as in tests/twin.c.
const STEP: [f32; 7] = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6];

/// The Rust twin of `twin` in tests/twin.c.
fn rust_twin(time: &Time, state: &mut State) {
    let Ok(q) = state.prev_q(0) else {
        return;
    };
    let step = STEP[(time.tick() % 7) as usize];
    for (i, qi) in q.iter().enumerate() {
        if !(time.tick() + i as u64).is_multiple_of(3) {
            let _ = state.set_q_at(Q(qi.0 + step), i);
        }
    }
    if time.tick().is_multiple_of(5) {
        let _ = state.set_tau_at(Tau(0.25), 1);
    }
    if let Ok(q1) = state.prev_q(1) {
        let _ = state.set_qd_at(Qd(q[3].0 - q1[3].0), 3);
    }
}

/// A `First` controller that records what the previous tick sent and its faults.
fn recording<'a>(
    log: &'a Log<N>,
    ticks: &'a RefCell<Vec<Tick>>,
) -> impl Fn(&Time, &mut State, &Sensors) + 'a {
    move |time, state, _| {
        if time.tick() > 0 {
            ticks
                .borrow_mut()
                .push((log.write.take(), state.prev_faults(1).ok()));
        }
    }
}

/// Runs `robot` for `n` ticks on a recorder whose log is `log`; returns what each tick sent.
fn run<C: Drive<N>>(
    robot: kmr_api::Robot<N, __Spec, C, Filled>,
    log: &Log<N>,
    ticks: &RefCell<Vec<Tick>>,
    n: u64,
) -> Vec<Tick> {
    let mut host = Ticking::new(Instant::now(), n);
    let mut drivers = HCons {
        head: Recorder::new(log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    let mut ticks = ticks.take();
    ticks.push((log.write.take(), None));
    ticks
}

/// What the Rust twin sends over `TICKS` ticks.
fn rust_sends() -> Vec<Tick> {
    let log = Log::new();
    let ticks = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(First, recording(&log, &ticks))
        .add_controller(EachTick, |time: &Time, state: &mut State, _: &Sensors| {
            rust_twin(time, state);
        });
    run(robot, &log, &ticks, TICKS)
}

/// A tick with each sent position as its bits.
type Bits = (Option<([u32; N], Sent<N>)>, Option<Faults<N>>);

/// The ticks with each sent position as its bits, so NaNs compare too.
fn bits(ticks: &[Tick]) -> Vec<Bits> {
    ticks
        .iter()
        .map(|&(sent, faults)| {
            let sent = sent.map(|s| {
                let q = s.q.map(|q| q.0.to_bits());
                (
                    q,
                    Sent {
                        q: [Q(0.0); N],
                        ..s
                    },
                )
            });
            (sent, faults)
        })
        .collect()
}

#[test]
fn an_inline_c_forwarder_sends_what_its_rust_twin_sends() {
    let log = Log::new();
    let ticks = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(First, recording(&log, &ticks))
        .add_controller(EachTick, |time: &Time, state: &mut State, _: &Sensors| {
            assert_eq!(
                call(twin, time, state, None::<&mut ()>),
                Ok(Status::Continue)
            );
        });
    let c = run(robot, &log, &ticks, TICKS);
    let rust = rust_sends();
    assert!(c.len() as u64 >= TICKS, "{} ticks", c.len());
    assert!(c.iter().filter(|(sent, _)| sent.is_some()).count() as u64 >= TICKS);
    assert_eq!(bits(&c), bits(&rust));
}

#[test]
fn a_c_forwarder_with_a_context_counts_its_calls_and_sends_the_same() {
    let log = Log::new();
    let ticks = RefCell::new(Vec::new());
    let calls = Cell::new(0);
    let walks = Cell::new(0);
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(First, recording(&log, &ticks))
        .add_controller_with(
            EachTick,
            0_u32,
            |n: &mut u32, time: &Time, state: &mut State, _: &Sensors| {
                assert_eq!(call(twin, time, state, Some(n)), Ok(Status::Continue));
                calls.set(*n);
                walks.set(walks.get() + 1);
            },
        );
    let c = run(robot, &log, &ticks, TICKS);
    assert!(walks.get() as u64 >= TICKS);
    assert_eq!(calls.get(), walks.get());
    assert_eq!(bits(&c), bits(&rust_sends()));
}

#[test]
fn c_that_writes_nothing_sends_no_command() {
    let log = Log::new();
    let ticks = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(First, recording(&log, &ticks))
        .add_controller(EachTick, |time: &Time, state: &mut State, _: &Sensors| {
            let _ = call(silent, time, state, None::<&mut ()>);
        });
    let c = run(robot, &log, &ticks, 50);
    assert!(c.len() >= 50);
    assert!(c.iter().all(|(sent, _)| sent.is_none()));
}

#[test]
fn a_rust_write_c_leaves_unwritten_is_kept() {
    let log = Log::new();
    let ticks = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(First, recording(&log, &ticks))
        .add_controller(EachTick, |_: &Time, state: &mut State, _: &Sensors| {
            let _ = state.set_q_at(Q(0.25), 1);
        })
        .add_controller(EachTick, |time: &Time, state: &mut State, _: &Sensors| {
            let _ = call(skip_one, time, state, None::<&mut ()>);
        });
    let c = run(robot, &log, &ticks, 3);
    let sent = c.last().and_then(|(sent, _)| *sent).expect("a write");
    assert_eq!(sent.q[..3], [Q(0.5), Q(0.25), Q(0.5)]);
    assert_eq!(sent.written.q.iter().collect::<Vec<_>>(), [0, 1, 2]);
}

/// Forwards to `ctx_value` with this registration's number.
fn numbered(n: &mut u32, time: &Time, state: &mut State, _: &Sensors) {
    let _ = call(ctx_value, time, state, Some(n));
}

#[test]
fn one_c_function_on_two_registrations_runs_in_declaration_order() {
    let log = Log::new();
    let ticks = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(First, recording(&log, &ticks))
        .add_controller_with(EachTick, 1_u32, numbered)
        .add_controller_with(EachTick, 2_u32, numbered);
    let c = run(robot, &log, &ticks, 3);
    let sent = c.last().and_then(|(sent, _)| *sent).expect("a write");
    assert_eq!(sent.q[0], Q(2.0), "the second registration wrote last");
}

/// What a NaN on q[0] from C (`from_c`) or from Rust sends and faults, with or without an
/// envelope on actuator 0.
fn nan_run(from_c: bool, envelope: bool) -> Vec<Tick> {
    let log = Log::new();
    let ticks = RefCell::new(Vec::new());
    let mut robot = Robot::new().set_dt(DT);
    if envelope {
        robot = robot.set_envelope(
            0,
            Envelope {
                q: Some((Q(-100.0), Q(100.0))),
                ..Envelope::NONE
            },
        );
    }
    let robot = robot
        .add_controller(First, recording(&log, &ticks))
        .add_controller(
            EachTick,
            move |time: &Time, state: &mut State, _: &Sensors| {
                if from_c {
                    let _ = call(nan_q0, time, state, None::<&mut ()>);
                } else {
                    let _ = state.set_q_at(Q(f32::NAN), 0);
                }
            },
        );
    run(robot, &log, &ticks, 3)
}

#[test]
fn a_nan_from_c_ends_as_a_nan_from_rust() {
    for envelope in [false, true] {
        let c = nan_run(true, envelope);
        let rust = nan_run(false, envelope);
        assert_eq!(bits(&c), bits(&rust), "envelope: {envelope}");
        let (sent, faults) = c.first().copied().expect("one tick");
        if envelope {
            assert!(
                sent.is_none_or(|s| !s.written.q.get(0)),
                "a breach is never sent"
            );
            assert!(faults.is_some_and(|f| f.envelope().get(0) || f.joint_limit().get(0)));
        } else {
            let sent = sent.expect("sent as written");
            assert!(sent.q[0].0.is_nan() && sent.written.q.get(0));
        }
    }
}
