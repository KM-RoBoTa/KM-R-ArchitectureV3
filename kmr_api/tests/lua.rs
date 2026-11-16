//! Lua controllers through `lua!` on the core's scripted host: reads, writes, ctx, an error,
//! the microphone, load errors.

use std::time::{Duration, Instant};

use kmr_api::ffi::lua::Error;
use kmr_api::test_util::{
    FakeSensor, Log, PER_TICK, Reading, Recorder, Scripted, TICK_END, run_scripted,
    run_scripted_all,
};
use kmr_api::{EachTick, HCons, HNil, Q};

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

#[test]
fn a_script_reads_writes_and_keeps_its_ctx() {
    let log = Log::new();
    let script = kmr_api::lua!("tests/lua/rw.lua").expect("loads");
    let robot = Robot::new().set_dt(DT).add_controller(EachTick, script);
    run_scripted(robot, &mut host(4), &mut drivers(&log)).expect("runs");

    let sent = log.write.get().expect("a write");
    assert_eq!(sent.written.q.iter().collect::<Vec<_>>(), [0, 1, 2]);
    assert_eq!(&sent.q[..3], &[Q(-0.5), Q(0.25), Q(0.5)]);
}

#[test]
fn an_error_applies_nothing() {
    let log = Log::new();
    let script = kmr_api::lua!("tests/lua/raise.lua").expect("loads");
    let robot = Robot::new().set_dt(DT).add_controller(EachTick, script);
    run_scripted(robot, &mut host(2), &mut drivers(&log)).expect("runs");

    assert!(log.write.get().is_none_or(|s| s.written.q.is_empty()));
}

#[test]
fn the_script_hears_the_microphone() {
    let log = Log::new();
    let quiet = [3_i16, -20, 7];
    let clap = [5_i16, -2000, 40];
    // Read call 0 is PostInit's, call t + 1 tick t's: the clap lands in tick 1.
    let readings = [
        Reading::QUIET,
        Reading {
            audio: &quiet,
            ..Reading::QUIET
        },
        Reading {
            audio: &clap,
            ..Reading::QUIET
        },
    ];
    let script = kmr_api::lua!("tests/lua/mic.lua").expect("loads");
    let robot = Robot::new()
        .set_dt(DT)
        .add_sensor(FakeSensor {
            script: &readings,
            mics: 1,
            ..FakeSensor::new(&log, 0)
        })
        .add_controller(EachTick, script);
    run_scripted_all(robot, &mut host(2), &mut drivers(&log)).expect("runs");

    let sent = log.write.get().expect("a write");
    assert_eq!(sent.written.q.iter().collect::<Vec<_>>(), [0]);
    assert_eq!(sent.q[0], Q(0.1));
}

#[test]
fn load_errors_are_returned() {
    assert!(matches!(
        kmr_api::lua!("tests/lua/rw.lua", nope),
        Err(Error::NoFunction { function, .. }) if function == "nope"
    ));
    assert!(matches!(
        kmr_api::lua!("tests/lua/missing.lua"),
        Err(Error::Read { .. })
    ));
    assert!(matches!(
        kmr_api::lua!("tests/lua/broken.lua"),
        Err(Error::Lua { .. })
    ));
}
