//! `c/clap.c` and `cpp/clap.cpp` over scripted ticks with a fake microphone: nothing before
//! HOME is done, the centre after it, both sines for 2 s from a clap, a clap heard while moving
//! ignored, the next one heard again; without its context the controller writes nothing.

use core::cell::{Cell, RefCell};
use std::f64::consts::TAU;
use std::time::Instant;

use kmr_api::ffi::CController;
use kmr_api::test_util::{DT, FakeSensor, Log, Reading, Recorder, Sent, Ticking, run_scripted_all};
use kmr_api::{EachTick, First, HCons, HNil, Sensors, Time};

kmr_api::robot!(model = "models/ax_12_12act_an_rx28_10act.urdf");

/// The context of both controllers, as they declare it.
#[repr(C)]
#[derive(Default)]
#[allow(dead_code)]
struct Clap {
    since: f64,
    clapped: bool,
}

/// The ticks a microphone sample louder than the peak arrives in.
const CLAPS: [usize; 3] = [3005, 3505, 5100];

/// Ticks of a run: past the third clap's quarter period.
const TICKS: u64 = 5400;

/// The ticks whose sends are checked.
const CHECKED: [usize; 12] = [
    3000, 3004, 3005, 3255, 3505, 4005, 4505, 5004, 5006, 5099, 5100, 5350,
];

/// A quiet tick's samples.
const QUIET: [i16; 3] = [3, -20, 7];
/// A clap's samples.
const LOUD: [i16; 3] = [5, -2000, 40];

/// The microphone's readings: read call 0 is PostInit's, call t + 1 tick t's.
static SCRIPT: [Reading<'static>; CLAPS[2] + 2] = script();

/// [`SCRIPT`]: quiet, loud on the ticks of [`CLAPS`].
const fn script() -> [Reading<'static>; CLAPS[2] + 2] {
    let mut readings = [Reading {
        audio: &QUIET,
        ..Reading::QUIET
    }; CLAPS[2] + 2];
    let mut k = 0;
    while k < CLAPS.len() {
        readings[CLAPS[k] + 1] = Reading {
            audio: &LOUD,
            ..Reading::QUIET
        };
        k += 1;
    }
    readings
}

/// What a run sent: whether anything was written to `Q` before tick 3000, and the sends of
/// the [`CHECKED`] ticks.
struct Seen {
    homing_wrote: bool,
    checked: [Option<Sent<N>>; CHECKED.len()],
}

/// Runs `clap` for [`TICKS`] ticks with a microphone hearing [`CLAPS`].
fn run<T>(clap: CController<N, T>) -> Seen {
    let log = Log::new();
    let homing_wrote = Cell::new(false);
    let checked = RefCell::new([None; CHECKED.len()]);
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(kmr_api::Policy::Clamp)
        .add_sensor(FakeSensor {
            script: &SCRIPT,
            mics: 1,
            ..FakeSensor::new(&log, 0)
        })
        .add_controller(First, |time: &Time, _: &mut State, _: &Sensors| {
            let Some(previous) = (time.tick() as usize).checked_sub(1) else {
                return;
            };
            let sent = log.write.take();
            if previous < 3000 && sent.is_some_and(|s| !s.written.q.is_empty()) {
                homing_wrote.set(true);
            }
            if let Some(k) = CHECKED.iter().position(|&t| t == previous) {
                checked.borrow_mut()[k] = sent;
            }
        })
        .add_controller(EachTick, clap);
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted_all(
        robot,
        &mut Ticking::new(Instant::now(), TICKS),
        &mut drivers,
    )
    .expect("runs");
    Seen {
        homing_wrote: homing_wrote.get(),
        checked: checked.into_inner(),
    }
}

/// The AX-12A and RX-28 targets of the tick `k` ticks after a clap.
fn targets(k: usize) -> (f32, f32) {
    let t = k as f64 * DT.as_secs_f64();
    let s = (TAU * 0.5 * t).sin() as f32;
    (0.3 * s, 0.15 * s)
}

/// Checks a run's sends against clap.py.
fn moves_on_claps(seen: &Seen) {
    let q = |tick: usize| {
        let k = CHECKED
            .iter()
            .position(|&t| t == tick)
            .expect("a checked tick");
        let sent = seen.checked[k].expect("a write");
        assert_eq!(
            sent.written.q.iter().count(),
            N,
            "tick {tick} writes every actuator"
        );
        (sent.q[0].0, sent.q[N - 1].0)
    };
    let close = |(ax, rx): (f32, f32), (want_ax, want_rx): (f32, f32)| {
        (ax - want_ax).abs() < 1e-4 && (rx - want_rx).abs() < 1e-4
    };
    // Elapsed is stamped half a dt into the tick: tick 2999 reads 2.9995 s, still homing.
    assert!(!seen.homing_wrote);
    assert_eq!(q(3000), (0.0, 0.0));
    assert_eq!(q(3004), (0.0, 0.0));
    for k in [0, 250, 1000, 1500] {
        assert!(close(q(3005 + k), targets(k)), "{k} ticks after the clap");
    }
    assert!(
        close(q(3505), targets(500)),
        "the clap heard while moving is ignored"
    );
    assert!(close(q(5004), targets(1999)));
    assert_eq!(q(5006), (0.0, 0.0));
    assert_eq!(q(5099), (0.0, 0.0));
    assert!(close(q(5100), targets(0)));
    assert!(close(q(5350), targets(250)), "the next clap is heard");
}

#[test]
fn the_c_clap_moves_both_snakes_as_clap_py() {
    moves_on_claps(&run(kmr_api::c!(move_on_clap_c, with = Clap::default())));
}

#[test]
fn the_cpp_clap_moves_both_snakes_as_clap_py() {
    moves_on_claps(&run(kmr_api::c!(move_on_clap_cpp, with = Clap::default())));
}

#[test]
fn without_its_context_a_clap_writes_nothing() {
    for seen in [
        run(kmr_api::c!(move_on_clap_c)),
        run(kmr_api::c!(move_on_clap_cpp)),
    ] {
        assert!(!seen.homing_wrote);
        assert!(
            seen.checked
                .iter()
                .all(|s| s.is_none_or(|s| s.written.q.is_empty()))
        );
    }
}
