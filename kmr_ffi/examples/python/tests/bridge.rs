//! The Python bridge over scripted ticks (kmr_ffi T024, SC-002, SC-007 bridge part): swing.py
//! sends what its Rust twin sends over 1000 ticks; a missing file or function is a load error
//! before `run()`; an exception, a wrong return value and an index out of range apply nothing.

use core::cell::RefCell;
use std::time::Instant;

use kmr_api::ffi::python::Bridge;
use kmr_api::ffi::{ForeignError, call};
use kmr_api::test_util::{DT, Log, Recorder, Sent, Ticking, run_scripted};
use kmr_api::{Drive, EachTick, Filled, First, HCons, HNil, Q, Sensors, Time};

kmr_api::robot!(XmBench);

/// The example's script.
const SWING: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/swing.py");

/// What each tick sent, as the next tick's `First` saw it.
type Sends = RefCell<Vec<Option<Sent<N>>>>;

/// A `First` controller that records what the previous tick sent.
fn recording<'a>(log: &'a Log<N>, sends: &'a Sends) -> impl Fn(&Time, &mut State, &Sensors) + 'a {
    move |time, _, _| {
        if time.tick() > 0 {
            sends.borrow_mut().push(log.write.take());
        }
    }
}

/// Runs `robot` for `ticks` ticks on a recorder; returns what each tick sent.
fn run<C: Drive<N>>(
    robot: kmr_api::Robot<N, __Spec, C, Filled>,
    log: &Log<N>,
    sends: &Sends,
    ticks: u64,
) -> Vec<Option<Sent<N>>> {
    let mut drivers = HCons {
        head: Recorder::new(log),
        tail: HNil,
    };
    run_scripted(
        robot,
        &mut Ticking::new(Instant::now(), ticks),
        &mut drivers,
    )
    .expect("runs");
    let mut sends = sends.take();
    sends.push(log.write.take());
    sends
}

/// The sends with each position as its bits.
fn bits(sends: &[Option<Sent<N>>]) -> Vec<Option<([u32; N], Sent<N>)>> {
    sends
        .iter()
        .map(|sent| {
            sent.map(|s| {
                (
                    s.q.map(|q| q.0.to_bits()),
                    Sent {
                        q: [Q(0.0); N],
                        ..s
                    },
                )
            })
        })
        .collect()
}

/// The Rust twin of swing.py: the script adds 0.1 in double precision, then rounds to `f32`.
fn rust_twin(_: &Time, state: &mut State) {
    if let Ok(q) = state.prev_q(0) {
        for (i, qi) in q.iter().enumerate() {
            let _ = state.set_q_at(Q((f64::from(qi.0) + 0.1) as f32), i);
        }
    }
}

#[test]
fn swing_sends_what_its_rust_twin_sends_over_1000_ticks() {
    let log = Log::new();
    let sends = Sends::default();
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(First, recording(&log, &sends))
        .add_controller_with(
            EachTick,
            Bridge::load(SWING, "step").expect("loads"),
            |bridge: &mut Bridge, time: &Time, state: &mut State, _: &Sensors| {
                assert!(call(bridge, time, state, None::<&mut ()>).is_ok());
            },
        );
    let script = run(robot, &log, &sends, 1000);

    let log = Log::new();
    let sends = Sends::default();
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller(First, recording(&log, &sends))
        .add_controller(EachTick, |time: &Time, state: &mut State, _: &Sensors| {
            rust_twin(time, state);
        });
    let twin = run(robot, &log, &sends, 1000);

    assert!(script.len() >= 1000);
    assert!(script.iter().all(Option::is_some));
    assert_eq!(bits(&script), bits(&twin));
}

#[test]
fn a_missing_file_or_function_is_a_load_error() {
    let missing = Bridge::load("no/such/swing.py", "step");
    assert!(matches!(missing, Err(ForeignError::Load(_))));
    let no_function = Bridge::load(SWING, "no_such_step");
    assert!(matches!(no_function, Err(ForeignError::Load(_))));
}

/// Writes `body` to a fresh script file and returns its path.
fn script(name: &str, body: &str) -> String {
    let path = std::env::temp_dir().join(format!("kmr_ffi_python_{}_{name}", std::process::id()));
    std::fs::write(&path, body).expect("temp dir is writable");
    path.to_string_lossy().into_owned()
}

/// One scripted tick of `step` in `body`; what `call` returned and what was sent.
fn one_tick(
    name: &str,
    body: &str,
) -> (
    Option<Result<kmr_api::ffi::Status, ForeignError>>,
    Option<Sent<N>>,
) {
    let bridge = Bridge::load(&script(name, body), "step").expect("loads");
    let log = Log::new();
    let got = core::cell::Cell::new(None);
    let robot = Robot::new().set_dt(DT).add_controller_with(
        EachTick,
        bridge,
        |bridge: &mut Bridge, time: &Time, state: &mut State, _: &Sensors| {
            if time.tick() == 0 {
                got.set(Some(call(bridge, time, state, None::<&mut ()>)));
            }
        },
    );
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut Ticking::new(Instant::now(), 1), &mut drivers).expect("runs");
    (got.take(), log.write.get())
}

#[test]
fn an_exception_applies_nothing() {
    let (got, sent) = one_tick(
        "raise.py",
        "def step(read, write, time):\n    write.set_q(0, 0.5)\n    raise RuntimeError('boom')\n",
    );
    assert!(matches!(got, Some(Err(ForeignError::Python(text))) if text.contains("boom")));
    assert_eq!(sent, None);
}

#[test]
fn a_wrong_return_value_applies_nothing() {
    let (got, sent) = one_tick(
        "seven.py",
        "def step(read, write, time):\n    write.set_q(0, 0.5)\n    return 7\n",
    );
    assert_eq!(got, Some(Err(ForeignError::Code(-1))));
    assert_eq!(sent, None);
}

#[test]
fn an_index_out_of_range_applies_nothing() {
    let (got, sent) = one_tick(
        "index.py",
        "def step(read, write, time):\n    write.set_q(0, 0.5)\n    write.set_q(99, 0.5)\n",
    );
    assert!(matches!(got, Some(Err(ForeignError::Python(_)))), "{got:?}");
    assert_eq!(sent, None);
}
