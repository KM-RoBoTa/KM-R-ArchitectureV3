//! The tick side of the gateway on the core's scripted host and recording adapter: no
//! session, no network (SC-010 (SRS FR-7.1), US4-AS2, US4-AS3).

use std::time::Instant;

use kmr_core::test_util::{
    Call, DT, Log, PER_TICK, Recorder, SEED, Scripted, TICK_END, run_scripted_staged,
};

use super::*;
use crate::robot::tests::TestSpec;
use crate::{EachTick, Filled, Last, Policy, Robot, Sensors, State};

/// What a test controller holds: the Desired writer and what it puts on which tick.
struct Remote {
    writer: LatestWriter<DesiredMsg<4>>,
    /// `(tick, position of actuator 1)`, put in that tick's `Last`.
    puts: &'static [(u64, f32)],
}

/// A Desired that writes `q` to actuator 1 only.
fn desired(seq: u64, q: f32) -> DesiredMsg<4> {
    let mut msg = DesiredMsg::EMPTY;
    msg.seq = seq;
    msg.q[1] = q;
    msg.written[0] = 1 << 1;
    msg
}

/// Puts the remote's Desired of this tick, as the gateway thread would between two ticks.
fn put(remote: &mut Remote, time: &Time, _state: &mut State<'_, 4>, _sensors: &Sensors) {
    for (seq, &(tick, q)) in (1..).zip(remote.puts) {
        if tick == time.tick() {
            remote.writer.put(desired(seq, q));
        }
    }
}

/// Runs `robot` for `ticks` ticks on a recorder logging to `log`, with `stages`.
fn run<C: kmr_core::Drive<4>>(
    robot: Robot<4, TestSpec, C, Filled>,
    stages: &mut TickSide<4>,
    ticks: usize,
    log: &Log<4>,
) {
    let mut inner = robot.finalized();
    let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + (ticks - 1) * PER_TICK);
    let mut drivers = kmr_core::HCons {
        head: Recorder::new(log),
        tail: kmr_core::HNil,
    };
    assert!(run_scripted_staged(&mut inner, &mut host, &mut drivers, stages).is_ok());
}

/// The tick of each `Write` call: the reads count the ticks.
fn write_ticks(log: &Log<4>) -> Vec<usize> {
    let mut tick = None::<usize>;
    let mut writes = Vec::new();
    for call in log.calls().iter() {
        match call {
            Call::Read => tick = Some(tick.map_or(0, |t| t + 1)),
            Call::Write => writes.extend(tick),
            _ => {}
        }
    }
    writes
}

/// The two slots and the tick side between them.
fn slots() -> (
    LatestWriter<DesiredMsg<4>>,
    TickSide<4>,
    LatestReader<SampleMsg<4>>,
) {
    let (desired_in, desired_out) = Latest::new().split();
    let (history_in, history_out) = Latest::new().split();
    (
        desired_in,
        TickSide::new(desired_out, history_in, Watchdog::none(&EXTERNAL_FLAGS)),
        history_out,
    )
}

/// A robot that may run: a dt and a policy for the model's limits.
fn robot() -> Robot<4, TestSpec> {
    Robot::new().set_dt(DT).set_limit_policy_all(Policy::Clamp)
}

/// A controller that does nothing.
fn idle(_time: &Time, _state: &mut State<'_, 4>, _sensors: &Sensors) {}

#[test]
fn a_desired_taken_in_tick_k_is_applied_in_k_and_never_in_k_plus_1() {
    let (writer, mut stages, _history) = slots();
    let log = Log::new();
    let robot = robot().add_controller_with(
        Last,
        Remote {
            writer,
            puts: &[(1, 0.5)],
        },
        put,
    );

    run(robot, &mut stages, 4, &log);

    assert_eq!(write_ticks(&log), [2]);
    let sent = log.write.get().expect("one write");
    assert_eq!(sent.q[1], Q(0.5));
    assert!(sent.written.q.get(1) && !sent.written.q.get(0));
}

#[test]
fn no_desired_no_command() {
    let (_writer, mut stages, _history) = slots();
    let log = Log::new();

    run(robot().add_controller(EachTick, idle), &mut stages, 4, &log);

    assert!(write_ticks(&log).is_empty());
}

#[test]
fn two_desired_within_one_tick_the_latest_is_applied_and_one_is_superseded() {
    let (writer, mut stages, _history) = slots();
    let log = Log::new();
    let robot = robot().add_controller_with(
        Last,
        Remote {
            writer,
            puts: &[(1, 0.25), (1, 0.75)],
        },
        put,
    );

    run(robot, &mut stages, 3, &log);

    assert_eq!(write_ticks(&log), [2]);
    assert_eq!(log.write.get().map(|sent| sent.q[1]), Some(Q(0.75)));
    assert_eq!(stages.desired.superseded(), 1);
}

/// Writes actuator 1 once there is a sample.
fn local(_time: &Time, state: &mut State<'_, 4>, _sensors: &Sensors) {
    if state.q().is_some() {
        let _ = state.set_q_at(Q(-0.5), 1);
    }
}

#[test]
fn a_local_controller_writing_the_same_actuator_later_in_the_tick_wins() {
    let (writer, mut stages, _history) = slots();
    let log = Log::new();
    let robot = robot().add_controller(EachTick, local).add_controller_with(
        Last,
        Remote {
            writer,
            puts: &[(1, 0.5)],
        },
        put,
    );

    run(robot, &mut stages, 3, &log);

    assert_eq!(log.write.get().map(|sent| sent.q[1]), Some(Q(-0.5)));
}

#[test]
fn a_non_finite_remote_value_is_not_written() {
    let (writer, mut stages, _history) = slots();
    let log = Log::new();
    let robot = robot().add_controller_with(
        Last,
        Remote {
            writer,
            puts: &[(1, f32::NAN)],
        },
        put,
    );

    run(robot, &mut stages, 3, &log);

    assert!(write_ticks(&log).is_empty());
}

#[test]
fn each_tick_puts_its_sample_with_its_index_age_and_faults() {
    let (_writer, mut stages, mut history) = slots();
    let log = Log::new();

    run(robot().add_controller(EachTick, idle), &mut stages, 5, &log);

    let last = history.take().expect("the last tick's sample");
    assert_eq!(last.tick, 4);
    assert_eq!(last.age, Some(0));
    // The recorder moves actuator 0 by one per read, from the seed.
    assert_eq!(last.q[0], SEED.0 + 5.0);
    assert!(last.faults.is_clear());
    assert_eq!(history.take(), None);
    assert_eq!(history.superseded(), 4, "nobody took ticks 0 to 3");
}

mod sources {
    use core::cell::RefCell;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::Duration;

    use kmr_core::hal::{ExternalFlags, TickFlags};

    use super::*;
    use crate::zenoh::gateway::{Counters, Inbox};
    use crate::zenoh::security::Rule;
    use crate::zenoh::watchdog::Watchdog;
    use crate::zenoh::{Gateway, GatewayConfig, Identity, Remote};

    /// The source of these tests.
    const CTL: Remote = Remote::allow("ctl-2").timeout(Duration::from_millis(3));

    /// The rule of instance `arm-1` with [`CTL`] as its one source, raising into `flags`.
    fn rule(flags: &'static ExternalFlags) -> (Rule, Counters, Arc<AtomicU64>) {
        let counters = Counters::default();
        let heard = Arc::new(AtomicU64::new(0));
        let rule = Rule::new(
            "arm-1",
            &[CTL],
            vec![Arc::clone(&heard)],
            counters.clone(),
            flags,
        );
        (rule, counters, heard)
    }

    /// `msg` received on `key` by `rule`.
    fn receive(rule: &mut Rule, key: &str, msg: &[u8], slot: &mut LatestWriter<DesiredMsg<4>>) {
        Inbox::<4>::receive(rule, key, msg, slot);
    }

    #[test]
    fn sc_011_a_thousand_desired_from_an_unknown_identity_none_applied_all_counted() {
        static FLAGS: ExternalFlags = ExternalFlags::new();
        let (mut rule, counters, heard) = rule(&FLAGS);
        let (mut writer, mut reader) = Latest::new().split();
        let key = keys::desired("arm-1", "intruder");
        for seq in 1..=1000 {
            receive(&mut rule, &key, &desired(seq, 0.5).encode(), &mut writer);
        }
        assert_eq!(reader.take(), None, "nothing reached the tick");
        assert_eq!((counters.unauthorized(), counters.accepted()), (1000, 0));
        assert_eq!(heard.load(Ordering::Relaxed), 0);
        assert!(FLAGS.take().contains(TickFlags::UNAUTHORIZED));
    }

    #[test]
    fn us6_as1_a_desired_at_or_below_the_last_seq_is_dropped_and_counted() {
        static FLAGS: ExternalFlags = ExternalFlags::new();
        let (mut rule, counters, heard) = rule(&FLAGS);
        let (mut writer, mut reader) = Latest::new().split();
        let key = keys::desired("arm-1", "ctl-2");
        for seq in [5, 5, 4, 6] {
            receive(&mut rule, &key, &desired(seq, 0.5).encode(), &mut writer);
        }
        assert_eq!(reader.take().map(|m| m.seq), Some(6));
        assert_eq!((counters.accepted(), counters.out_of_order()), (2, 2));
        assert_eq!(heard.load(Ordering::Relaxed), 2);
        assert!(FLAGS.take().is_empty());
    }

    #[test]
    fn a_desired_of_another_length_or_instance_never_reaches_the_tick() {
        static FLAGS: ExternalFlags = ExternalFlags::new();
        let (mut rule, counters, _) = rule(&FLAGS);
        let (mut writer, mut reader) = Latest::new().split();
        receive(
            &mut rule,
            &keys::desired("arm-1", "ctl-2"),
            &[0; 7],
            &mut writer,
        );
        let other = keys::desired("arm-9", "ctl-2");
        receive(&mut rule, &other, &desired(1, 0.5).encode(), &mut writer);
        assert_eq!(reader.take(), None);
        assert_eq!((counters.malformed(), counters.accepted()), (1, 0));
    }

    /// What the test controllers hold: the source's accepted count, and when it speaks.
    struct Speaker {
        heard: Arc<AtomicU64>,
        on: &'static [u64],
    }

    /// Counts one accepted Desired in this tick's `Last` when the source speaks then.
    fn speak(speaker: &mut Speaker, time: &Time, _state: &mut State<'_, 4>, _sensors: &Sensors) {
        if speaker.on.contains(&time.tick()) {
            speaker.heard.fetch_add(1, Ordering::Release);
        }
    }

    #[test]
    fn us6_as2_a_silent_source_raises_the_flag_within_timeout_plus_one_tick_and_its_fallback_runs()
    {
        static FLAGS: ExternalFlags = ExternalFlags::new();
        let (watchdog, heard) = Watchdog::new(&[CTL], &FLAGS);
        let (_writer, desired_out) = Latest::new().split();
        let (history_in, _history_out) = Latest::new().split();
        let mut stages = TickSide::new(desired_out, history_in, watchdog);
        let ran = RefCell::new(Vec::new());
        let fallback = |time: &Time, _: &mut State<'_, 4>, _: &Sensors| {
            ran.borrow_mut().push(time.tick());
        };
        let gateway = Gateway::new(GatewayConfig {
            instance: "arm-1",
            identity: Identity::loopback(),
            read_history: &[],
        })
        .flags(&FLAGS);
        let speaker = Speaker {
            heard: Arc::clone(&heard[0]),
            on: &[4],
        };
        let robot = robot()
            .add_plugins(gateway)
            .add_remote_source(EachTick, CTL, fallback)
            .add_controller_with(Last, speaker, speak);
        let mut inner = robot.finalized();
        let log = Log::new();
        let mut host = Scripted::new(Instant::now()).shutdown_from(TICK_END + 7 * PER_TICK);
        host.external = Some(&FLAGS);
        let mut drivers = kmr_core::HCons {
            head: Recorder::new(&log),
            tail: kmr_core::HNil,
        };
        assert!(run_scripted_staged(&mut inner, &mut host, &mut drivers, &mut stages).is_ok());

        // 3 ms at 1 ms a tick: silent through tick 2, flagged from tick 3; the Desired
        // accepted in tick 4 clears it from tick 6 on.
        assert_eq!(*ran.borrow(), [3, 4, 5]);
    }
}
