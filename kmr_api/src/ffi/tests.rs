//! The glue over scripted runs: the view against the `State` and `Time` readers, the apply
//! step, and an error that applies nothing. Rust doubles stand in for the foreign body.

use core::cell::RefCell;
use core::num::NonZeroU32;
use std::time::Instant;

use super::*;
use crate::test_util::{DT, Faults, Log, PER_TICK, Recorder, Scripted, TICK_END, run_scripted};
use crate::{EachTick, Envelope, First, HCons, HNil, Q, Qd, Robot, Sensors, Spec, UserError};

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

    fn bind(plugins: &[crate::Binding]) -> Result<Self::Drivers, UserError> {
        crate::__bind_none(plugins)
    }
}

type View = ReadView<4, HISTORY_DEPTH>;

/// A robot of [`TestSpec`] at [`DT`], its limits clamped.
fn robot() -> Robot<4, TestSpec> {
    Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(crate::Policy::Clamp)
}

/// A Rust stand-in for a foreign body: records what it read, writes with `write`, returns `out`.
struct Twin<'a> {
    seen: &'a RefCell<Vec<(View, TimeView)>>,
    write: fn(&mut WriteBuf<4>),
    out: Result<Status, ForeignError>,
}

impl Foreign<4, HISTORY_DEPTH> for Twin<'_> {
    fn invoke<C>(
        &mut self,
        read: &View,
        buf: &mut WriteBuf<4>,
        time: &TimeView,
        _ctx: Option<&mut C>,
    ) -> Result<Status, ForeignError> {
        self.seen.borrow_mut().push((*read, *time));
        (self.write)(buf);
        self.out.clone()
    }
}

/// Writes nothing.
fn none(_: &mut WriteBuf<4>) {}

/// Flags `q[1]` = 0.5 and `kd[3]` = 2; leaves `q[2]` = 9 unflagged; flags bit 10, past N.
fn some(buf: &mut WriteBuf<4>) {
    buf.q[1] = 0.5;
    buf.written.q |= 1 << 1;
    buf.q[2] = 9.0;
    buf.kd[3] = 2.0;
    buf.written.kd |= 1 << 3;
    buf.written.tau |= 1 << 10;
}

/// The mask as the view's word.
fn bits(mask: crate::Mask<4>) -> u64 {
    mask.iter().fold(0, |w, i| w | 1 << i)
}

/// Whether `view` holds what the `State` readers return, at every depth.
fn agrees(view: &View, state: &State<'_, 4>) -> bool {
    let samples = (0..HISTORY_DEPTH)
        .take_while(|&k| state.prev_q(k).is_ok())
        .count();
    let ages = (0..4).all(|i| view.age[i] == state.age_at(i).ok().flatten().unwrap_or(AGE_NONE));
    let slots = view.slot.iter().enumerate().all(|(k, slot)| {
        let values = k >= samples
            || (state.prev_q(k).is_ok_and(|v| v.map(|x| x.0) == slot.q)
                && state.prev_qd(k).is_ok_and(|v| v.map(|x| x.0) == slot.qd)
                && state.prev_tau(k).is_ok_and(|v| v.map(|x| x.0) == slot.tau)
                && state.prev_kp(k).is_ok_and(|v| v.map(|x| x.0) == slot.kp)
                && state.prev_kd(k).is_ok_and(|v| v.map(|x| x.0) == slot.kd));
        let faults = state.prev_faults(k).map_or(true, |f| {
            slot.flags.stale == bits(f.stale())
                && slot.flags.bus_error == bits(f.bus_error())
                && slot.flags.offline == bits(f.offline())
                && slot.flags.rejected == bits(f.rejected())
                && slot.flags.joint_limit == bits(f.joint_limit())
                && slot.flags.envelope == bits(f.envelope())
                && slot.tick_flags == f.tick_bits()
        });
        values && faults
    });
    view.samples as usize == samples && ages && slots
}

#[test]
fn the_view_holds_every_depth_its_faults_and_the_ages() {
    let log = Log::new();
    let mut faults = Faults::default();
    faults.stale.set(1);
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
    let seen = RefCell::new(Vec::new());
    let ok = RefCell::new(Vec::new());
    let robot = robot().add_controller(
        First,
        |time: &Time, state: &mut State<'_, 4>, _: &Sensors| {
            let twin = Twin {
                seen: &seen,
                write: none,
                out: Ok(Status::Continue),
            };
            assert_eq!(
                call(twin, time, state, None::<&mut ()>),
                Ok(Status::Continue)
            );
            let (view, _) = *seen.borrow().last().expect("one call");
            ok.borrow_mut().push(agrees(&view, state));
        },
    );
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 6 * PER_TICK);
    run_scripted(robot, &mut host, &mut drivers).expect("runs");

    assert_eq!(*ok.borrow(), [true; 7]);
    let seen = seen.borrow();
    let samples: Vec<u32> = seen.iter().map(|(v, _)| v.samples).collect();
    let full = HISTORY_DEPTH as u32;
    // Bring-up's read fills one slot before tick 0 reads the next.
    let expect: Vec<u32> = (2..9).map(|s: u32| s.min(full)).collect();
    assert_eq!(samples, expect);
    let (tick2, _) = seen[2];
    assert_eq!(tick2.slot[0].flags.stale, 1 << 1);
    assert_eq!(tick2.slot[0].flags.offline, 1 << 3);
    let (tick3, _) = seen[3];
    assert_eq!(
        tick3.slot[1].flags.stale,
        1 << 1,
        "a tick back, one slot deeper"
    );
    assert_eq!(tick3.slot[0].flags.stale, 0);
    assert_eq!(
        seen.iter().map(|(v, _)| v.age[0]).collect::<Vec<_>>(),
        [0, 1, 0, 1, 0, 1, 0]
    );
}

#[test]
fn a_run_without_hardware_has_no_sample_and_no_age() {
    let seen = RefCell::new(Vec::new());
    let robot = robot().add_controller(
        EachTick,
        |time: &Time, state: &mut State<'_, 4>, _: &Sensors| {
            let twin = Twin {
                seen: &seen,
                write: none,
                out: Ok(Status::Continue),
            };
            let _ = call(twin, time, state, None::<&mut ()>);
        },
    );
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 2 * PER_TICK);
    let mut drivers = HCons {
        head: crate::NoHardware,
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");

    let seen = seen.borrow();
    assert_eq!(seen.len(), 3);
    for (view, _) in seen.iter() {
        assert_eq!(view.samples, 0);
        assert_eq!(view.age, [AGE_NONE; 4]);
    }
}

#[test]
fn the_time_view_equals_the_time_getters() {
    let seen = RefCell::new(Vec::new());
    let getters = RefCell::new(Vec::new());
    let robot = robot().add_controller(
        EachTick,
        |time: &Time, state: &mut State<'_, 4>, _: &Sensors| {
            getters.borrow_mut().push(TimeView {
                tick: time.tick(),
                dt_ns: time.dt().as_nanos() as u64,
                elapsed_ns: time.elapsed().as_nanos() as u64,
                overrun_count: time.overrun_count(),
                last_overrun_ns: time.last_overrun().map_or(0, |d| d.as_nanos() as u64),
                accumulated_overrun_ns: time.accumulated_overrun().as_nanos() as u64,
            });
            let twin = Twin {
                seen: &seen,
                write: none,
                out: Ok(Status::Continue),
            };
            let _ = call(twin, time, state, None::<&mut ()>);
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

    let views: Vec<TimeView> = seen.borrow().iter().map(|(_, t)| *t).collect();
    assert_eq!(views, *getters.borrow());
    assert_eq!(views[1].overrun_count, 1);
    assert_eq!(views[1].last_overrun_ns, 500_000);
    assert_eq!(views[2].tick, 2);
}

/// Runs one tick whose `EachTick` calls a twin writing with `write` and returning `out`;
/// returns what `call` gave and what the adapter was sent.
fn one_tick(
    write: fn(&mut WriteBuf<4>),
    out: Result<Status, ForeignError>,
) -> (
    Result<Status, ForeignError>,
    Option<kmr_core::test_util::Sent<4>>,
) {
    let seen = RefCell::new(Vec::new());
    let got = RefCell::new(None);
    let robot = robot().add_controller(
        EachTick,
        |time: &Time, state: &mut State<'_, 4>, _: &Sensors| {
            let twin = Twin {
                seen: &seen,
                write,
                out: out.clone(),
            };
            *got.borrow_mut() = Some(call(twin, time, state, None::<&mut ()>));
        },
    );
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);
    let log = Log::new();
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    let got = got.into_inner().expect("called once");
    (got, log.write.get())
}

#[test]
fn only_the_flagged_entries_are_applied() {
    let (got, sent) = one_tick(some, Ok(Status::Continue));
    assert_eq!(got, Ok(Status::Continue));
    let sent = sent.expect("one write");
    assert_eq!(sent.q[1], Q(0.5));
    assert_eq!(bits(sent.written.q), 1 << 1);
    assert_eq!(bits(sent.written.kd), 1 << 3);
    assert!(sent.written.qd.is_empty() && sent.written.tau.is_empty());
    assert!(sent.written.kp.is_empty());
}

#[test]
fn an_error_applies_nothing() {
    let (got, sent) = one_tick(some, Err(ForeignError::Code(-7)));
    assert_eq!(got, Err(ForeignError::Code(-7)));
    assert_eq!(sent, None, "nothing written, nothing sent");
}

#[test]
fn a_write_outside_the_envelope_is_clamped_as_a_rust_write() {
    let seen = RefCell::new(Vec::new());
    let robot = robot()
        .set_envelope(
            1,
            Envelope {
                q: Some((Q(-0.25), Q(0.25))),
                ..Envelope::NONE
            },
        )
        .add_controller(
            EachTick,
            |time: &Time, state: &mut State<'_, 4>, _: &Sensors| {
                let twin = Twin {
                    seen: &seen,
                    write: some,
                    out: Ok(Status::Continue),
                };
                let _ = call(twin, time, state, None::<&mut ()>);
            },
        );
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);
    let log = Log::new();
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    let sent = log.write.get().expect("one write");
    assert_eq!(sent.q[1], Q(0.25));
}

/// Writes `body` to a fresh script file and returns its path.
#[cfg(any(feature = "ffi-python", feature = "ffi-lua"))]
fn script(name: &str, body: &str) -> String {
    let path = std::env::temp_dir().join(format!("kmr_api_ffi_{}_{name}", std::process::id()));
    std::fs::write(&path, body).expect("temp dir is writable");
    path.to_string_lossy().into_owned()
}

/// Runs one tick whose `EachTick` calls `bridge`; returns what the adapter was sent.
#[cfg(any(feature = "ffi-python", feature = "ffi-lua"))]
fn bridge_tick<B: Foreign<4, HISTORY_DEPTH>>(bridge: B) -> Option<kmr_core::test_util::Sent<4>> {
    let bridge = RefCell::new(bridge);
    let robot = robot().add_controller(
        EachTick,
        |time: &Time, state: &mut State<'_, 4>, _: &Sensors| {
            let got = call(&mut *bridge.borrow_mut(), time, state, None::<&mut ()>);
            assert_eq!(got, Ok(Status::Continue));
        },
    );
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);
    let log = Log::new();
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    log.write.get()
}

#[cfg(feature = "ffi-python")]
#[test]
fn a_python_bridge_writes_through_the_glue() {
    let path = script(
        "step.py",
        "def step(read, write, time):\n    if read.samples > 0:\n        write.set_q(1, 0.5)\n",
    );
    let bridge = python::Bridge::load(&path, "step").expect("loads");
    let sent = bridge_tick(bridge).expect("one write");
    assert_eq!(sent.q[1], Q(0.5));
    assert_eq!(bits(sent.written.q), 1 << 1);
}

#[cfg(feature = "ffi-lua")]
#[test]
fn a_lua_bridge_writes_through_the_glue() {
    let path = script(
        "step.lua",
        "function step(read, write, time)\n  if read.samples > 0 then write.set_q(2, 0.5) end\nend\n",
    );
    let bridge = lua::Bridge::load(&path, "step").expect("loads");
    let sent = bridge_tick(bridge).expect("one write");
    assert_eq!(sent.q[1], Q(0.5));
    assert_eq!(bits(sent.written.q), 1 << 1);
}
