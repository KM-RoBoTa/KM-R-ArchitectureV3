//! History, staleness, faults and ages as C reads them (kmr_ffi T025, SRS FR-2.1, FR-2.3,
//! FR-2.6, FR-13.4): every depth equals the `prev_*` readers, a failed read shows as stale with
//! its last good value and an age, the oldest age equals `sample_age()`, and every flag of the
//! flag list equals kmr_api's `Faults` view.

use core::cell::RefCell;
use core::num::NonZeroU32;
use std::time::Instant;

use kmr_api::ffi::{AGE_NONE, ReadView, Slot, call};
use kmr_api::test_util::{
    DT, Drivers, Faults, Log, PER_TICK, Recorder, Scripted, TICK_END, run_scripted,
};
use kmr_api::{EachTick, HCons, HISTORY_DEPTH, HNil, Mask, Sensors, Tau, Time};

kmr_api::robot!(XmBench);
kmr_api::ffi::foreign! { fn copy_history; }

/// `history_out` of tests/history.c.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct Out {
    view: ReadView<N, HISTORY_DEPTH>,
    sample_age: u32,
}

/// What one tick's checks found.
#[derive(Clone, Copy, Debug, Default)]
struct Seen {
    out: Out,
    /// Every slot C read equals the readers, values and flags.
    agrees: bool,
    /// `sample_age()`, `AGE_NONE` for `None`.
    sample_age: u32,
}

/// A mask as the view's word.
fn word(mask: Mask<N>) -> u64 {
    mask.iter().fold(0, |w, i| w | 1 << i)
}

/// Whether `slot`, depth `k` of the view C read, equals what the readers return.
fn slot_agrees(state: &State, k: usize, samples: usize, slot: &Slot<N>) -> bool {
    let values = k >= samples
        || (state.prev_q(k).is_ok_and(|v| v.map(|x| x.0) == slot.q)
            && state.prev_qd(k).is_ok_and(|v| v.map(|x| x.0) == slot.qd)
            && state.prev_tau(k).is_ok_and(|v| v.map(|x| x.0) == slot.tau));
    let Ok(faults) = state.prev_faults(k) else {
        return values;
    };
    let mut flags = true;
    macro_rules! each_flag {
        ($($flag:ident),*) => {
            $(flags &= slot.flags.$flag == word(faults.$flag());)*
        };
    }
    kmr_api::ffi::__for_each_flag!(each_flag);
    values && flags
}

/// The controller: C copies the view, Rust checks it against the readers.
fn check(seen: &RefCell<Vec<Seen>>) -> impl Fn(&mut Out, &Time, &mut State, &Sensors) + '_ {
    move |out, time, state, _| {
        assert!(call(copy_history, time, state, Some(out)).is_ok());
        let samples = (0..HISTORY_DEPTH)
            .take_while(|&k| state.prev_q(k).is_ok())
            .count();
        let agrees = out.view.samples as usize == samples
            && out
                .view
                .slot
                .iter()
                .enumerate()
                .all(|(k, slot)| slot_agrees(state, k, samples, slot));
        // A write, so the recorder's write faults reach the next slot.
        let _ = state.set_tau_at(Tau(0.0), 3);
        seen.borrow_mut().push(Seen {
            out: *out,
            agrees,
            sample_age: state.sample_age().unwrap_or(AGE_NONE),
        });
    }
}

/// Runs `ticks` ticks with `drivers`; returns what each tick saw.
fn run<D>(drivers: &mut D, ticks: usize) -> Vec<Seen>
where
    D: Drivers<N, Error: Send + Sync>,
{
    let seen = RefCell::new(Vec::new());
    let robot = Robot::new()
        .set_dt(DT)
        .add_controller_with(EachTick, Out::default(), check(&seen));
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + (ticks - 1) * PER_TICK);
    run_scripted(robot, &mut host, drivers).expect("runs");
    seen.into_inner()
}

#[test]
fn c_reads_every_depth_the_faults_and_a_stale_actuator_as_the_readers_do() {
    let log = Log::new();
    let mut stale = Faults::default();
    stale.stale.set(1);
    // Every actuator is read every third tick; the read of tick 4, not due, fails on actuator 1.
    let mut drivers = HCons {
        head: Recorder {
            every: NonZeroU32::new(3).expect("3 is not zero"),
            faults: stale,
            faults_on: Some(4),
            write_faults: {
                let mut faults = Faults::default();
                faults.rejected.set(3);
                faults
            },
            ..Recorder::new(&log)
        },
        tail: HNil,
    };
    let seen = run(&mut drivers, 8);

    assert_eq!(seen.len(), 8);
    assert!(seen.iter().all(|s| s.agrees), "{seen:#?}");
    assert!(seen.iter().all(|s| s.out.sample_age == s.sample_age));
    let samples: Vec<u32> = seen.iter().map(|s| s.out.view.samples).collect();
    assert_eq!(samples.last(), Some(&(HISTORY_DEPTH as u32)));

    let tick4 = seen[4].out.view;
    assert_eq!(tick4.slot[0].flags.stale, 1 << 1);
    assert!(tick4.age[1] > 0 && tick4.age[1] != AGE_NONE);
    assert_ne!(
        tick4.slot[0].q[1], 0.0,
        "a stale actuator keeps its last good value"
    );
    assert_eq!(tick4.slot[0].q[1], tick4.slot[1].q[1]);
    assert!(seen[2].out.view.slot[1].flags.rejected & 1 << 3 != 0);
    assert!(seen.iter().any(|s| s.out.sample_age > 0));
}

#[test]
fn a_run_without_hardware_has_no_sample_and_no_age() {
    let mut drivers = HCons {
        head: kmr_api::NoHardware,
        tail: HNil,
    };
    let seen = run(&mut drivers, 3);
    assert_eq!(seen.len(), 3);
    for s in &seen {
        assert_eq!(s.out.view.samples, 0);
        assert_eq!(s.out.view.age, [AGE_NONE; N]);
        assert_eq!(s.out.sample_age, AGE_NONE);
        assert!(s.agrees);
    }
}
