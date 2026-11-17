//! Presence under the missing policy: bring-up with silent actuators, the
//! skip of absent slots, loss detection and its log lines, and the exit path
//! (runtime-actuators plan, T5). Every scenario runs on a strict fake, which
//! keeps what `dxl_rs_v2` holds.

use core::fmt::Write as _;

use kmr_core::hal::MissingPolicy;
use tracing::Level;

use super::*;
use crate::dynamixel::backend::{Fault, Reply};
use crate::dynamixel::presence::LOST_AFTER;

/// `Overrides` with the missing policy `Tolerate`, the join policy off.
fn tolerate() -> Overrides {
    let mut overrides = Overrides::default();
    overrides.missing = MissingPolicy::Tolerate;
    overrides
}

/// An adapter on a fresh strict fake owning `0..M` in table order.
fn strict<const NB: usize, const M: usize>(table: Table<NB, M>) -> Dynamixel<Scripted, NB, N> {
    strict_placed(table.in_order())
}

/// An adapter on a fresh strict fake owning `placement`.
fn strict_placed<const NB: usize, const M: usize>(
    placement: Placement<NB, M>,
) -> Dynamixel<Scripted, NB, N> {
    let mut backend = Scripted::new();
    backend.strict();
    Dynamixel::<_, _, N>::new(backend, placement)
}

/// `check` with `overrides`, then Init and PostInit in the engine's order on
/// an adapter marked open, as [`powered`] does; the log is cleared after.
fn powered_with<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    overrides: Overrides,
) -> Result<Claim<N>, DynamixelError> {
    let claim = adapter.check(overrides)?;
    mark_open(adapter);
    adapter.zero()?;
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;
    adapter.torque_on()?;
    adapter.backend.clear_log();
    Ok(claim)
}

/// Whether a recorded call names firmware id `id`.
fn names(backend: &Scripted, id: u8) -> bool {
    backend
        .calls()
        .iter()
        .any(|record| backend.ids(record).any(|named| named == id))
}

/// `times` reads of every due group.
fn reads<const NB: usize>(adapter: &mut Dynamixel<Scripted, NB, N>, times: u8) -> Faults<N> {
    let mut seen = Faults::default();
    for _ in 0..times {
        let (_, faults) = read(adapter, &mut Slot::filled(0.0));
        seen.offline = seen.offline.union(faults.offline);
        seen.bus_error = seen.bus_error.union(faults.bus_error);
        seen.stale = seen.stale.union(faults.stale);
    }
    seen
}

/// Events at `level` logged on this thread by `run`: how many hold
/// `family` in their message, and how many say exactly `message` with the
/// field `bus` at `bus`.
fn logged(
    level: Level,
    (family, message, bus): (&'static str, &'static str, &'static str),
    run: impl FnOnce(),
) -> (usize, usize) {
    let lines = Lines {
        level,
        family,
        message,
        bus,
        all: AtomicUsize::new(0),
        matched: AtomicUsize::new(0),
    };
    let dispatch = tracing::Dispatch::new(tracing_subscriber::registry().with(lines));
    tracing::dispatcher::with_default(&dispatch, run);
    let lines = dispatch.downcast_ref::<Lines>().expect("the line layer");
    (
        lines.all.load(Ordering::Relaxed),
        lines.matched.load(Ordering::Relaxed),
    )
}

/// Counts the events [`logged`] asks for.
struct Lines {
    level: Level,
    family: &'static str,
    message: &'static str,
    bus: &'static str,
    all: AtomicUsize,
    matched: AtomicUsize,
}

impl<S: tracing::Subscriber> Layer<S> for Lines {
    fn on_event(&self, event: &tracing::Event<'_>, _context: Context<'_, S>) {
        if *event.metadata().level() != self.level {
            return;
        }
        let blank = || Message {
            bytes: [0; MESSAGE],
            len: 0,
        };
        let mut seen = Seen {
            message: blank(),
            bus: blank(),
        };
        event.record(&mut seen);
        let family = self.family.as_bytes();
        if seen
            .message
            .text()
            .windows(family.len())
            .any(|window| window == family)
        {
            self.all.fetch_add(1, Ordering::Relaxed);
        }
        if seen.message.text() == self.message.as_bytes() && seen.bus.text() == self.bus.as_bytes()
        {
            self.matched.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// The message and the `bus` field of one event.
struct Seen {
    message: Message,
    bus: Message,
}

impl tracing::field::Visit for Seen {
    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        let text = match field.name() {
            "bus" => &mut self.bus,
            "message" => &mut self.message,
            _ => return,
        };
        let _ = text.write_str(value);
    }

    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn fmt::Debug) {
        let text = match field.name() {
            "bus" => &mut self.bus,
            "message" => &mut self.message,
            _ => return,
        };
        let _ = write!(text, "{value:?}");
    }
}

// ── Bring-up ────────────────────────────────────────────────────────────────

#[test]
fn fr_5_5_strict_missing_at_open_still_fails_after_bounded_retries() {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));

    assert_eq!(
        powered_with(&mut adapter, Overrides::default()).err(),
        Some(DynamixelError::Missing {
            actuator: 2,
            bus: "ax",
            id: id(2)
        })
    );
    assert_eq!(adapter.backend.count(Call::Declare), PING_ATTEMPTS);
    assert_eq!(adapter.present(), mask(|_| true));
}

#[test]
fn tolerate_missing_at_open_brings_up_the_rest() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));

    let claim = powered_with(&mut adapter, tolerate())?;

    assert_eq!(claim.owned, mask(|_| true), "the claim keeps every slot");
    assert_eq!(adapter.present(), mask(|index| index != 2));
    for index in [0, 1, 3] {
        assert!(adapter.backend.torqued(0, id(index)), "actuator {index}");
        assert!(adapter.backend.holds(0, Model::Ax12a, id(index)));
    }
    assert!(!adapter.backend.holds(0, Model::Ax12a, id(2)));
    assert!(!adapter.backend.torqued(0, id(2)));
    Ok(())
}

#[test]
fn slots_stay_sparse_index_3_is_id_4() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2)).position(0, id(3), 700);
    let claim = powered_with(&mut adapter, tolerate())?;

    let mut slot = Slot::filled(9.0);
    let (refreshed, faults) = read(&mut adapter, &mut slot);

    assert_eq!(refreshed, mask(|index| index != 2));
    assert_eq!(faults, Faults::default());
    assert_eq!(slot.q[2], Q(9.0), "the absent slot is left as copied");
    let fourth = adapter.actuators.get(3).copied().unwrap();
    assert_eq!(Some(slot.q[3]), units::position_to_si(&fourth, 700));
    let faults = write(&mut adapter, &claim, &Desired::new().q(3, slot.q[3]));
    assert_eq!(faults, Faults::default());
    assert_eq!(adapter.backend.last_goals(), &[(id(3), 700)]);
    Ok(())
}

#[test]
fn absent_slot_is_never_read_written_or_telemetered() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = adapter.check(tolerate())?;
    mark_open(&mut adapter);
    adapter.ping()?;
    adapter.backend.clear_log();
    adapter.health()?;
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;
    adapter.torque_on()?;
    let mut telemetry = Telemetry::default();
    refresh(&mut adapter, &mut telemetry);
    assert!(!names(&adapter.backend, id(2)), "bring-up");

    for tick in 0..5 * EVERY {
        adapter.backend.clear_log();
        read(&mut adapter, &mut Slot::filled(0.0));
        write(&mut adapter, &claim, &Desired::all_q());
        refresh(&mut adapter, &mut telemetry);
        assert!(!names(&adapter.backend, id(2)), "tick {tick}");
    }
    adapter.backend.clear_log();
    adapter.hold(Desired::all_q().command());
    assert!(!names(&adapter.backend, id(2)), "hold");
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
    Ok(())
}

#[test]
fn fr_2_4_set_all_on_an_absent_slot_is_skipped_without_fault() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    let claim = powered_with(&mut adapter, tolerate())?;

    let faults = write(&mut adapter, &claim, &Desired::all_q());

    assert_eq!(faults, Faults::default());
    let sent = adapter.backend.last(Call::SyncWriteGoal).unwrap();
    assert!(adapter.backend.ids(sent).eq([id(0), id(1), id(3)]));
    Ok(())
}

// ── Loss at run time ────────────────────────────────────────────────────────

#[test]
fn three_timed_out_reads_mark_a_slot_lost() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    powered_with(&mut adapter, tolerate())?;
    adapter.backend.missing(0, id(2));

    for read in 1..LOST_AFTER {
        let faults = reads(&mut adapter, 1);
        assert!(faults.offline.get(2) && faults.stale.get(2), "read {read}");
        assert_eq!(adapter.present(), mask(|_| true), "read {read}");
    }
    let faults = reads(&mut adapter, 1);

    assert!(faults.offline.get(2));
    assert_eq!(adapter.present(), mask(|index| index != 2));
    adapter.backend.clear_log();
    assert_eq!(reads(&mut adapter, 1), Faults::default());
    assert!(!names(&adapter.backend, id(2)));
    Ok(())
}

#[test]
fn one_answered_read_resets_the_miss_count() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    powered_with(&mut adapter, tolerate())?;

    adapter.backend.missing(0, id(2));
    reads(&mut adapter, LOST_AFTER - 1);
    adapter.backend.plug(0, id(2), false);
    reads(&mut adapter, 1);
    adapter.backend.missing(0, id(2));
    reads(&mut adapter, LOST_AFTER - 1);

    assert_eq!(adapter.present(), mask(|_| true));
    reads(&mut adapter, 1);
    assert_eq!(adapter.present(), mask(|index| index != 2));
    Ok(())
}

#[test]
fn answers_with_error_bits_are_not_misses() -> Result<(), DynamixelError> {
    let overload = BackendError::Status(StatusError {
        report: StatusReport::P1(P1Errors::from_bits(1 << P1Error::Overload as u8)),
        request: Request {
            instruction: Some(Instruction::Read),
            register: Some("Present Position"),
            addr: None,
        },
    });
    let hot = BackendError::Fault(Fault::Hardware { status: 0x04 });
    for error in [overload, hot, BackendError::Reply(Reply::Corrupt)] {
        let mut adapter = strict(AX_TABLE);
        powered_with(&mut adapter, tolerate())?;
        adapter.backend.fail(Call::ReadPosition, 0, id(2), error);

        let faults = reads(&mut adapter, 2 * LOST_AFTER);

        assert!(faults.offline.get(2), "{error}: flagged as today");
        assert_eq!(adapter.present(), mask(|_| true), "{error}");
    }
    Ok(())
}

#[test]
fn bus_errors_are_not_misses() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    powered_with(&mut adapter, tolerate())?;
    let closed = BackendError::Io(Wire::Closed);
    adapter.backend.fail_port(Call::ReadPosition, 0, closed);

    let faults = reads(&mut adapter, 2 * LOST_AFTER);

    assert_eq!(faults.bus_error, mask(|_| true));
    assert_eq!(adapter.present(), mask(|_| true));

    let mut adapter = strict(X_TABLE);
    powered_with(&mut adapter, tolerate())?;
    adapter.backend.fail_sync_read(0);
    let faults = reads(&mut adapter, 2 * LOST_AFTER);
    assert_eq!(faults.bus_error, mask(|_| true));
    assert_eq!(adapter.present(), mask(|_| true));
    Ok(())
}

#[test]
fn strict_policy_never_drops_a_silent_slot() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    powered_with(&mut adapter, Overrides::default())?;
    adapter.backend.missing(0, id(2));

    let faults = reads(&mut adapter, 2 * LOST_AFTER);

    assert!(faults.offline.get(2));
    assert_eq!(adapter.present(), mask(|_| true));
    adapter.backend.clear_log();
    reads(&mut adapter, 1);
    assert!(names(&adapter.backend, id(2)), "still read");
    Ok(())
}

// ── Log lines ───────────────────────────────────────────────────────────────

#[test]
fn lost_ids_in_a_row_log_one_line() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    powered_with(&mut adapter, tolerate())?;
    for index in 1..N {
        adapter.backend.missing(0, id(index));
    }

    let line = ("not available", "ids 2 to 4 not available anymore", "ax");
    let seen = logged(Level::WARN, line, || {
        reads(&mut adapter, 2 * LOST_AFTER);
    });

    assert_eq!(seen, (1, 1));
    assert_eq!(adapter.present(), mask(|index| index == 0));
    Ok(())
}

#[test]
fn a_lone_lost_id_logs_actuator_id() -> Result<(), DynamixelError> {
    let mut adapter = strict(SNAKES_TABLE);
    powered_with(&mut adapter, tolerate())?;
    adapter.backend.missing(0, 1).missing(1, 2);

    let ax = ("not available", "actuator id 1 not available anymore", "ax");
    let rx = ("not available", "actuator id 2 not available anymore", "rx");
    let mut snakes = adapter;
    let seen = logged(Level::WARN, ax, || {
        reads(&mut snakes, LOST_AFTER);
    });
    assert_eq!(seen, (2, 1), "one line per bus");

    let mut adapter = strict(SNAKES_TABLE);
    powered_with(&mut adapter, tolerate())?;
    adapter.backend.missing(1, 2);
    let seen = logged(Level::WARN, rx, || {
        reads(&mut adapter, LOST_AFTER);
    });
    assert_eq!(seen, (1, 1));
    Ok(())
}

#[test]
fn missing_ids_at_start_log_one_line() {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(1)).missing(0, id(2));

    let line = ("not available", "ids 2 to 3 not available at start", "ax");
    let seen = logged(Level::WARN, line, || {
        powered_with(&mut adapter, tolerate()).unwrap();
    });

    assert_eq!(seen, (1, 1));
    assert_eq!(adapter.present(), mask(|index| index == 0 || index == 3));
}

// ── Exit ────────────────────────────────────────────────────────────────────

/// [`AX_TABLE`] in order, every actuator writing [`VOLTAGES`] and putting
/// them back at `close`.
fn restoring() -> Placement<1, N> {
    let mut placement = AX_TABLE.in_order();
    placement.settings = [restored(VOLTAGES); N];
    placement
}

/// Position of the first record of `call` naming `id`, from the end when
/// `last`.
fn at(backend: &Scripted, call: Call, id: Option<u8>, last: bool) -> Option<usize> {
    let hit = |record: &Record| record.call == call && id.is_none_or(|id| record.id == id);
    if last {
        backend.calls().iter().rposition(hit)
    } else {
        backend.calls().iter().position(hit)
    }
}

#[test]
fn fr_5_6_torque_off_cuts_every_held_slot_then_close_cuts_the_rest() -> Result<(), DynamixelError> {
    let mut adapter = strict_placed(restoring());
    adapter.backend.missing(0, id(2));
    powered_with(&mut adapter, tolerate())?;
    adapter.backend.missing(0, id(1));
    reads(&mut adapter, LOST_AFTER);
    assert_eq!(adapter.present(), mask(|index| index == 0 || index == 3));
    adapter.backend.clear_log();

    let line = (
        "not reached",
        "actuator id 3 not reached for torque off",
        "ax",
    );
    let seen = logged(Level::WARN, line, || {
        adapter.torque_off();
        adapter.close();
    });

    let backend = &adapter.backend;
    let first_restore = at(backend, Call::WriteVoltageLimits, None, false).unwrap();
    let last_restore = at(backend, Call::WriteVoltageLimits, None, true).unwrap();
    for held in [0, 1, 3] {
        let cut = at(backend, Call::ForceTorqueOff, Some(id(held)), true).unwrap();
        assert!(
            cut < first_restore,
            "actuator {held}: cut before any restore"
        );
    }
    let cut = at(backend, Call::ForceTorqueOff, Some(id(2)), false).unwrap();
    assert!(cut > last_restore, "the unheld one after the restores");
    assert_eq!(forced(backend).filter(|&cut| cut == id(2)).count(), 1);
    let lost = forced(backend).filter(|&cut| cut == id(1)).count();
    assert_eq!(
        lost, TORQUE_OFF_ATTEMPTS,
        "held, though lost: every attempt"
    );
    assert_eq!(seen, (1, 1));
    assert_eq!(last_call(backend), Some(Call::ClosePorts));
    Ok(())
}

#[test]
fn close_restores_limits_of_held_slots_only() -> Result<(), DynamixelError> {
    let mut adapter = strict_placed(restoring());
    powered_with(&mut adapter, tolerate())?;
    let rest = [
        (id(0), Model::Ax12a),
        (id(1), Model::Ax12a),
        (id(3), Model::Ax12a),
    ];
    adapter.backend.declare(0, &rest).unwrap();
    adapter.backend.clear_log();

    let line = (
        "not restored",
        "limits of actuator id 3 not restored: not reachable",
        "ax",
    );
    let seen = logged(Level::WARN, line, || {
        adapter.torque_off();
        adapter.close();
    });

    assert_eq!(seen, (1, 1));
    let restored = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::WriteVoltageLimits)
        .map(|record| record.id);
    assert!(restored.eq([id(0), id(1), id(3)]));
    assert_eq!(
        adapter.saved[2],
        super::super::limits::Limits::NONE,
        "forgotten"
    );
    assert_eq!(
        forced(&adapter.backend).filter(|&cut| cut == id(2)).count(),
        1
    );
    Ok(())
}

#[test]
fn telemetry_skips_absent_members_without_sending() -> Result<(), DynamixelError> {
    let mut adapter = strict(AX_TABLE);
    adapter.backend.missing(0, id(2));
    powered_with(&mut adapter, tolerate())?;
    let mut telemetry = Telemetry::default();
    let (refreshed, _) = refresh(&mut adapter, &mut telemetry);
    assert_eq!(refreshed, mask(|index| index != 2));
    assert_eq!(adapter.backend.count(Call::ReadVitals), N - 1);
    adapter.backend.clear_log();

    let mut sent = [(0, 0); 4];
    let mut taken = 0;
    for tick in 0..4 * EVERY + 1 {
        adapter.backend.clear_log();
        tick_with_telemetry(&mut adapter, &mut telemetry);
        let record = vitals_read(&adapter.backend).last();
        assert_eq!(super::super::telemetry::sent(&adapter), record.is_some());
        if let (Some(slot), Some((_, _, id, _))) = (sent.get_mut(taken), record) {
            *slot = (tick, id);
            taken += 1;
        }
    }
    // The step of id 3 is due at tick 149: nothing is sent, and id 4 goes
    // the tick after.
    assert_eq!(
        sent,
        [
            (EVERY - 1, id(0)),
            (2 * EVERY - 1, id(1)),
            (3 * EVERY, id(3)),
            (4 * EVERY, id(0))
        ]
    );

    let mut adapter = strict(X_TABLE);
    adapter.backend.missing(0, id(2));
    powered_with(&mut adapter, tolerate())?;
    refresh(&mut adapter, &mut telemetry);
    let fields = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::ReadVitals);
    for record in fields {
        assert!(adapter.backend.ids(record).eq([id(0), id(1), id(3)]));
    }
    assert_eq!(adapter.backend.count(Call::ReadVitals), 3);
    Ok(())
}

#[test]
fn policies_off_exit_is_unchanged() -> Result<(), DynamixelError> {
    let mut adapter = rig(AX_TABLE);
    powered_with(&mut adapter, Overrides::default())?;
    adapter.backend.missing(0, id(2));
    reads(&mut adapter, 2 * LOST_AFTER);
    adapter.backend.clear_log();

    adapter.torque_off();
    adapter.close();

    let tries = |index| if index == 2 { TORQUE_OFF_ATTEMPTS } else { 1 };
    assert!(forced(&adapter.backend).eq(attempted(tries)));
    let kinds = adapter.backend.calls().iter().map(|record| record.call);
    let off = iter::repeat_n(Call::ForceTorqueOff, N - 1 + TORQUE_OFF_ATTEMPTS);
    assert!(kinds.eq(off.chain([Call::ClosePorts])));
    Ok(())
}
