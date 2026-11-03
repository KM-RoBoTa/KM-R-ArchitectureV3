//! The ABI as C sees it, through the probes of tests/abi.c built by the example's build.rs
//! (kmr_ffi T015, SC-007 C part): the layout of every crossing type, named reads of a view
//! Rust filled, setters past N, and a failing body that applies nothing.
//!
//! Layout at other N (1, 3, 4, 10, 64) is checked by kmr_ffi's own header tests, which compile
//! the generated header for each N.

use core::mem::{align_of, offset_of, size_of};
use std::time::Instant;

use kmr_api::ffi::{
    AGE_NONE, ActuatorFlags, ForeignError, ReadView, Slot, TimeView, WriteBuf, Written, call,
};
use kmr_api::test_util::{DT, Log, Recorder, Scripted, TICK_END, run_scripted};
use kmr_api::{EachTick, HCons, HISTORY_DEPTH, HNil, Sensors, Time};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn probe_read; fn probe_past_n; fn probe_fail; }

// SAFETY: tests/abi.c defines `probe_layout` with this C signature (`size_t *`, `size_t`,
// returning `size_t`; `size_t` is `usize` on every target kmr builds for).
unsafe extern "C" {
    fn probe_layout(out: *mut usize, cap: usize) -> usize;
}

type View = ReadView<N, HISTORY_DEPTH>;

/// Rust's layout, in the order of `probe_layout`.
fn rust_layout() -> [usize; 49] {
    [
        N,
        HISTORY_DEPTH,
        size_of::<ActuatorFlags>(),
        align_of::<ActuatorFlags>(),
        offset_of!(ActuatorFlags, stale),
        offset_of!(ActuatorFlags, bus_error),
        offset_of!(ActuatorFlags, offline),
        offset_of!(ActuatorFlags, rejected),
        offset_of!(ActuatorFlags, joint_limit),
        offset_of!(ActuatorFlags, envelope),
        size_of::<Slot<N>>(),
        align_of::<Slot<N>>(),
        offset_of!(Slot<N>, q),
        offset_of!(Slot<N>, qd),
        offset_of!(Slot<N>, tau),
        offset_of!(Slot<N>, kp),
        offset_of!(Slot<N>, kd),
        offset_of!(Slot<N>, flags),
        offset_of!(Slot<N>, tick_flags),
        offset_of!(Slot<N>, reserved),
        size_of::<View>(),
        align_of::<View>(),
        offset_of!(View, samples),
        offset_of!(View, reserved),
        offset_of!(View, age),
        offset_of!(View, slot),
        size_of::<Written>(),
        align_of::<Written>(),
        offset_of!(Written, q),
        offset_of!(Written, qd),
        offset_of!(Written, tau),
        offset_of!(Written, kp),
        offset_of!(Written, kd),
        size_of::<WriteBuf<N>>(),
        align_of::<WriteBuf<N>>(),
        offset_of!(WriteBuf<N>, q),
        offset_of!(WriteBuf<N>, qd),
        offset_of!(WriteBuf<N>, tau),
        offset_of!(WriteBuf<N>, kp),
        offset_of!(WriteBuf<N>, kd),
        offset_of!(WriteBuf<N>, written),
        size_of::<TimeView>(),
        align_of::<TimeView>(),
        offset_of!(TimeView, tick),
        offset_of!(TimeView, dt_ns),
        offset_of!(TimeView, elapsed_ns),
        offset_of!(TimeView, overrun_count),
        offset_of!(TimeView, last_overrun_ns),
        offset_of!(TimeView, accumulated_overrun_ns),
    ]
}

#[test]
fn c_sees_the_layout_rust_has() {
    let rust = rust_layout();
    let mut c = [0usize; 49];
    // SAFETY: `c` is a live, exclusive array of `c.len()` `size_t`s; the probe writes at most
    // `cap` of them and keeps no pointer.
    let n = unsafe { probe_layout(c.as_mut_ptr(), c.len()) };
    assert_eq!(
        n,
        rust.len(),
        "probe_layout lists another number of entries"
    );
    assert_eq!(c, rust);
}

#[test]
fn c_reads_a_rust_filled_view_by_name_bit_for_bit() {
    let mut view = View {
        samples: 3,
        ..View::default()
    };
    view.age[N - 1] = AGE_NONE;
    view.slot[3].q[N - 1] = -1.234_567_9;
    view.slot[1].kd[0] = f32::from_bits(0x7fc0_0001);
    view.slot[2].flags.envelope = 1 << (N - 1) | 1;
    view.slot[0].tick_flags = 0b1001;
    let time = TimeView {
        elapsed_ns: (1 << 40) + 7,
        accumulated_overrun_ns: u64::MAX - 1,
        ..TimeView::default()
    };
    let mut buf = WriteBuf::default();
    let mut out = [0u64; 8];
    // SAFETY: every pointer comes from a live reference of the type the C prototype names;
    // `probe_read` writes the eight `uint64_t`s of `out` through `ctx` and keeps no pointer.
    let rc = unsafe { probe_read(&view, &mut buf, &time, out.as_mut_ptr().cast()) };
    assert_eq!(rc, 0);
    assert_eq!(
        out,
        [
            3,
            u64::from(AGE_NONE),
            u64::from((-1.234_567_9_f32).to_bits()),
            0x7fc0_0001,
            1 << (N - 1) | 1,
            0b1001,
            (1 << 40) + 7,
            u64::MAX - 1,
        ]
    );
}

#[test]
fn the_setters_ignore_an_index_past_n() {
    let mut buf = WriteBuf::default();
    // SAFETY: live references of the prototype's types, a null context the body ignores.
    let rc = unsafe {
        probe_past_n(
            &View::default(),
            &mut buf,
            &TimeView::default(),
            core::ptr::null_mut(),
        )
    };
    assert_eq!(rc, 0);
    assert_eq!(buf, WriteBuf::default());
}

#[test]
fn a_c_body_that_writes_then_fails_applies_nothing() {
    let log = Log::new();
    let got = core::cell::Cell::new(None);
    let robot = Robot::new().set_dt(DT).add_controller(
        EachTick,
        |time: &Time, state: &mut State, _: &Sensors| {
            got.set(Some(call(probe_fail, time, state, None::<&mut ()>)));
        },
    );
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END);
    let mut drivers = HCons {
        head: Recorder::new(&log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("runs");
    assert_eq!(got.take(), Some(Err(ForeignError::Code(-7))));
    assert_eq!(log.write.get(), None, "nothing written, nothing sent");
}
