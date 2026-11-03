//! The C++ layer (kmr_ffi T019, SC-007 C++ part): an exception leaving the body returns -100
//! and applies nothing, its non-throwing twin applies `q[0]`, and the typed accessors read the
//! same values as the C view.

use std::time::Instant;

use kmr_api::ffi::{ForeignError, ReadView, Status, TimeView, WriteBuf, call};
use kmr_api::test_util::{DT, Log, Recorder, Scripted, Sent, TICK_END, run_scripted};
use kmr_api::{EachTick, HCons, HISTORY_DEPTH, HNil, Q, Sensors, Time};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn throwing; fn calm; fn typed_copy; }

type View = ReadView<N, HISTORY_DEPTH>;

/// One scripted tick forwarding to `f`; what `call` returned and what was sent.
fn one_tick(
    f: kmr_api::ffi::CFn<N, HISTORY_DEPTH>,
) -> (Option<Result<Status, ForeignError>>, Option<Sent<N>>) {
    let log = Log::new();
    let got = core::cell::Cell::new(None);
    let robot = Robot::new().set_dt(DT).add_controller(
        EachTick,
        |time: &Time, state: &mut State, _: &Sensors| {
            got.set(Some(call(f, time, state, None::<&mut ()>)));
        },
    );
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    (got.take(), log.write.get())
}

#[test]
fn an_exception_returns_minus_100_and_applies_nothing() {
    let (got, sent) = one_tick(throwing);
    assert_eq!(got, Some(Err(ForeignError::Code(-100))));
    assert_eq!(sent, None);
}

#[test]
fn the_non_throwing_twin_applies_q0() {
    let (got, sent) = one_tick(calm);
    assert_eq!(got, Some(Ok(Status::Continue)));
    let sent = sent.expect("a write");
    assert_eq!(sent.q[0], Q(0.5));
    assert_eq!(sent.written.q.iter().collect::<Vec<_>>(), [0]);
}

#[test]
fn the_typed_accessors_read_the_c_view() {
    let mut view = View {
        samples: 3,
        ..View::default()
    };
    for (i, age) in view.age.iter_mut().enumerate() {
        *age = if i == 1 { u32::MAX } else { i as u32 * 7 };
    }
    for (k, slot) in view.slot.iter_mut().enumerate() {
        for i in 0..N {
            let x = (k * N + i) as f32;
            slot.q[i] = x + 0.25;
            slot.qd[i] = -x;
            slot.tau[i] = x * 0.5;
            slot.kp[i] = x + 100.0;
            slot.kd[i] = x - 100.0;
        }
        slot.flags.stale = 1 << k;
        slot.flags.bus_error = 0b11 << k >> 1;
        slot.flags.offline = 1;
        slot.flags.rejected = (1 << N) - 1;
        slot.flags.joint_limit = 1 << (N - 1);
        slot.flags.envelope = (k as u64) & 1;
        slot.tick_flags = k as u32 + 1;
    }
    let mut copy = View::default();
    let mut buf = WriteBuf::default();
    // SAFETY: every pointer comes from a live reference of the type the prototype names; the
    // body writes one `kmr_read_view` through `ctx` and keeps no pointer.
    let rc = unsafe {
        typed_copy(
            &view,
            &mut buf,
            &TimeView::default(),
            core::ptr::from_mut(&mut copy).cast(),
        )
    };
    assert_eq!(rc, 0);
    assert_eq!(copy, view);
}
