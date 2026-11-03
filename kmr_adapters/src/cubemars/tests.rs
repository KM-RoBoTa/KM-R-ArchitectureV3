//! Adapter tests on the scripted CAN bus, without hardware. The adapter is
//! driven through [`Hal`] in the engine's order.

use kmr_core::hal::{Batch, Command, Faults, Group, Hal, Layout, Mask, Overrides, Sample, Written};
use kmr_core::state::{Kd, Kp, Q, Qd, Tau};

use super::frame::{self, ENTER, EXIT, Feedback, Mit};
use super::scripted::{Call, Scripted};
use super::transport::TransportError;
use super::{
    ATTEMPTS, Actuator, Bus, CubeMars, CubeMarsError, DEFAULT_KD, DEFAULT_KP, MAX_TEMPERATURE_C,
    TORQUE_OFF_ATTEMPTS, Table,
};
use crate::binding::{Binding, Port};

/// Actuators of the robot: the GeckoLeg's hip, knee and ankle.
const N: usize = 3;

/// One interface, ids 1 to 3, the ankle with its own gains.
const LEG: Table<1, N> = Table {
    buses: [Bus { label: "leg" }],
    actuators: [
        Actuator::ak60_6(0, 1),
        Actuator::ak60_6(0, 2),
        Actuator::ak60_6(0, 3),
    ],
}
.gains(2, 40.0, 0.5)
.check();

/// The bus plugin of interface `interface`, naming bus `bus`.
const fn plugin(interface: &'static str, bus: Option<&'static str>) -> Binding {
    Binding {
        port: Port::ByDev(interface),
        baud: None,
        bus,
    }
}

/// The leg on a scripted bus whose motors stand at `positions`.
fn leg(positions: [f32; N]) -> CubeMars<Scripted, 1, N> {
    let mut scripted = Scripted::with_ids(&[1, 2, 3]);
    for (id, p) in (1..).zip(positions) {
        scripted.motor(0, id).state.p = p;
    }
    CubeMars::bind(scripted, LEG.in_order(), &[plugin("vcan-test", None)]).unwrap()
}

/// The history slot `read` fills.
#[derive(Default)]
struct Slot {
    q: [Q; N],
    qd: [Qd; N],
    tau: [Tau; N],
    kp: [Kp; N],
    kd: [Kd; N],
}

impl Slot {
    /// Lends it as a sample.
    fn sample(&mut self) -> Sample<'_, N> {
        Sample::new(
            &mut self.q,
            &mut self.qd,
            &mut self.tau,
            &mut self.kp,
            &mut self.kd,
        )
    }
}

/// `leg` through every bring-up call, torque on, the calls forgotten.
fn powered(positions: [f32; N]) -> (CubeMars<Scripted, 1, N>, Layout<N>) {
    let mut adapter = leg(positions);
    let claim = adapter.check(Overrides::default()).unwrap();
    let layout = Layout::new(
        claim.group,
        claim.read_every,
        claim.read_cost,
        claim.write_cost,
    );
    adapter.open().unwrap();
    adapter.zero().unwrap();
    adapter.ping().unwrap();
    adapter.health().unwrap();
    adapter.configure().unwrap();
    adapter.read_all(Slot::default().sample()).unwrap();
    adapter.torque_on().unwrap();
    (adapter, layout)
}

/// Desired values of one tick, with what was written.
#[derive(Default)]
struct Desired {
    q: [Q; N],
    qd: [Qd; N],
    tau: [Tau; N],
    kp: [Kp; N],
    kd: [Kd; N],
    written: Written<N>,
}

impl Desired {
    /// Writes `q` of actuator `index`.
    fn q(mut self, index: usize, q: f32) -> Self {
        self.q[index] = Q(q);
        self.written.q.set(index);
        self
    }

    /// Writes the gains of actuator `index`.
    fn gains(mut self, index: usize, kp: f32, kd: f32) -> Self {
        self.kp[index] = Kp(kp);
        self.kd[index] = Kd(kd);
        self.written.kp.set(index);
        self.written.kd.set(index);
        self
    }

    /// Writes `qd` and `tau` of actuator `index`.
    fn feed_forward(mut self, index: usize, qd: f32, tau: f32) -> Self {
        self.qd[index] = Qd(qd);
        self.tau[index] = Tau(tau);
        self.written.qd.set(index);
        self.written.tau.set(index);
        self
    }

    /// Borrowed as the engine hands it over.
    fn command(&self) -> Command<'_, N> {
        Command::new(
            &self.q,
            &self.qd,
            &self.tau,
            &self.kp,
            &self.kd,
            &self.written,
        )
    }
}

/// One `write` of `desired`, the calls made before forgotten.
fn write(
    adapter: &mut CubeMars<Scripted, 1, N>,
    layout: &Layout<N>,
    desired: &Desired,
) -> Faults<N> {
    let mut faults = Faults::default();
    let batch = Batch::new(layout, &desired.written);
    adapter.write(desired.command(), &batch, &mut faults);
    faults
}

/// The MIT commands sent, decoded, by CAN id: exactly `K` of them.
fn commands<const K: usize>(scripted: &Scripted) -> [(u8, Mit); K] {
    let mut sent = scripted
        .sent()
        .filter(|&(_, _, data)| data != ENTER && data != EXIT)
        .map(|(_, id, data)| (id, frame::command(data)));
    let commands = core::array::from_fn(|_| sent.next().expect("fewer commands sent"));
    assert!(sent.next().is_none(), "more commands sent");
    commands
}

/// Whether `a` and `b` agree within one step of the coarsest field.
fn near(a: f32, b: f32) -> bool {
    (a - b).abs() < 0.03
}

#[test]
fn check_claims_the_placement_one_group_per_interface_read_every_tick() {
    const TWO: Table<2, N> = Table {
        buses: [Bus { label: "left" }, Bus { label: "right" }],
        actuators: [
            Actuator::ak60_6(0, 1),
            Actuator::ak60_6(1, 1),
            Actuator::ak60_6(1, 2),
        ],
    }
    .check();
    let plugins = [
        plugin("vcan0", Some("left")),
        plugin("vcan1", Some("right")),
    ];
    let mut adapter =
        CubeMars::<_, 2, 4>::bind(Scripted::default(), TWO.at([3, 0, 1]), &plugins).unwrap();
    let claim = adapter.check(Overrides::default()).unwrap();
    assert!(claim.owned.iter().eq([0, 1, 3]));
    assert_eq!(
        (claim.group[0], claim.group[1], claim.group[3]),
        (Group(1), Group(1), Group(0))
    );
    assert!(claim.read_every.iter().all(|every| every.get() == 1));
    assert_eq!(claim.write_cost, super::FRAME_TIME * 2);
}

#[test]
fn a_row_past_the_robot_is_refused_at_check() {
    let mut adapter = CubeMars::<_, 1, 2>::bind(
        Scripted::default(),
        LEG.in_order(),
        &[plugin("vcan0", None)],
    )
    .unwrap();
    assert_eq!(
        adapter.check(Overrides::default()).unwrap_err(),
        CubeMarsError::PastTheRobot {
            actuator: 2,
            index: 2,
            actuators: 2
        }
    );
}

#[test]
fn a_can_bus_binds_an_interface_by_name_only() {
    let by_id = Binding {
        port: Port::ById("FT8ISZ6H"),
        baud: None,
        bus: None,
    };
    let error = CubeMars::<_, 1, N>::bind(Scripted::default(), LEG.in_order(), &[by_id]);
    assert!(matches!(
        error,
        Err(CubeMarsError::NotAnInterface { bus: "leg", .. })
    ));

    let baud = Binding {
        baud: Some(500_000),
        ..plugin("can0", None)
    };
    let error = CubeMars::<_, 1, N>::bind(Scripted::default(), LEG.in_order(), &[baud]);
    assert!(matches!(
        error,
        Err(CubeMarsError::BaudOnCan {
            interface: "can0",
            baud: 500_000,
            ..
        })
    ));

    let unknown = plugin("can0", Some("arm"));
    let error = CubeMars::<_, 1, N>::bind(Scripted::default(), LEG.in_order(), &[unknown]);
    assert!(matches!(
        error,
        Err(CubeMarsError::UnknownBus { label: "arm", .. })
    ));

    let error = CubeMars::<_, 1, N>::bind(Scripted::default(), LEG.in_order(), &[]);
    assert!(matches!(
        error,
        Err(CubeMarsError::UnboundBus { bus: "leg" })
    ));
}

#[test]
fn open_without_a_plugin_and_on_a_shared_interface_is_refused() {
    let mut unbound = CubeMars::<_, 1, N>::new(Scripted::default(), LEG.in_order());
    assert_eq!(
        unbound.open(),
        Err(CubeMarsError::UnboundBus { bus: "leg" })
    );

    const TWO: Table<2, 2> = Table {
        buses: [Bus { label: "left" }, Bus { label: "right" }],
        actuators: [Actuator::ak60_6(0, 1), Actuator::ak60_6(1, 1)],
    }
    .check();
    let plugins = [
        plugin("vcan0", Some("left")),
        plugin("vcan0", Some("right")),
    ];
    let mut shared =
        CubeMars::<_, 2, 2>::bind(Scripted::default(), TWO.in_order(), &plugins).unwrap();
    assert_eq!(
        shared.open(),
        Err(CubeMarsError::SharedInterface {
            first: "left",
            bus: "right",
            interface: "vcan0"
        })
    );
    assert_eq!(shared.transport().calls().count(), 0);
}

#[test]
fn open_opens_every_interface_and_names_the_one_that_fails() {
    let mut adapter = leg([0.0; N]);
    adapter.open().unwrap();
    let open = Call::Open {
        names: ["vcan-test", ""],
        count: 1,
    };
    assert!(adapter.transport().calls().eq([&open]));

    let mut failing = leg([0.0; N]);
    failing.transport().open_error = Some((0, TransportError::NoInterface));
    assert_eq!(
        failing.open(),
        Err(CubeMarsError::InterfaceOpen {
            bus: "leg",
            interface: "vcan-test",
            source: TransportError::NoInterface
        })
    );
}

#[test]
fn positions_are_offsets_from_where_each_actuator_stood_at_zero() {
    let mut adapter = leg([1.0, -2.0, 0.5]);
    adapter.open().unwrap();
    adapter.zero().unwrap();
    adapter.transport().motor(0, 1).state.p = 1.25;
    let mut slot = Slot::default();
    adapter.read_all(slot.sample()).unwrap();
    assert!(near(slot.q[0].0, 0.25), "{:?}", slot.q);
    assert!(near(slot.q[1].0, 0.0) && near(slot.q[2].0, 0.0));
    assert_eq!(slot.kp.map(|k| k.0), [DEFAULT_KP, DEFAULT_KP, 40.0]);
    assert_eq!(slot.kd.map(|k| k.0), [DEFAULT_KD, DEFAULT_KD, 0.5]);
}

#[test]
fn bring_up_asks_limp_with_the_exit_frame_and_never_sends_the_zero_frame() {
    let (mut adapter, _) = powered([0.0; N]);
    let scripted = adapter.transport();
    let before_enter = || scripted.sent().take_while(|(_, _, data)| *data != ENTER);
    assert!(before_enter().all(|(_, _, data)| data == EXIT));
    // zero, ping, health and read_all: four questions per actuator.
    assert_eq!(before_enter().count(), 4 * N);
    let zero_frame = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE];
    assert!(scripted.sent().all(|(_, _, data)| data != zero_frame));
}

#[test]
fn a_silent_actuator_is_missing_after_its_attempts() {
    let mut adapter = leg([0.0; N]);
    adapter.transport().motor(0, 2).answers = false;
    adapter.open().unwrap();
    assert_eq!(
        adapter.ping(),
        Err(CubeMarsError::Missing {
            actuator: 1,
            bus: "leg",
            id: 2
        })
    );
    let to_two = adapter.transport().sent().filter(|s| s.1 == 2).count();
    assert_eq!(to_two, ATTEMPTS);
}

#[test]
fn replies_of_other_actuators_are_skipped_at_bring_up() {
    let mut adapter = leg([0.0, 3.0, 0.0]);
    adapter.open().unwrap();
    let stranger = frame::reply(Feedback {
        id: 9,
        p: -4.0,
        v: 0.0,
        t: 0.0,
        temperature: 30,
        fault: 0,
    });
    adapter.transport().inject(0, stranger);
    let mut slot = Slot::default();
    adapter.zero().unwrap();
    adapter.read_all(slot.sample()).unwrap();
    assert!(slot.q.iter().all(|q| near(q.0, 0.0)), "{:?}", slot.q);
}

#[test]
fn health_refuses_an_error_code_and_a_hot_actuator() {
    let mut faulty = leg([0.0; N]);
    faulty.transport().motor(0, 3).state.fault = 2;
    faulty.open().unwrap();
    assert_eq!(
        faulty.health(),
        Err(CubeMarsError::Fault {
            actuator: 2,
            bus: "leg",
            id: 3,
            code: 2
        })
    );

    let mut hot = leg([0.0; N]);
    hot.transport().motor(0, 1).state.temperature = MAX_TEMPERATURE_C + 1;
    hot.open().unwrap();
    assert_eq!(
        hot.health(),
        Err(CubeMarsError::Temperature {
            actuator: 0,
            bus: "leg",
            id: 1,
            measured_c: MAX_TEMPERATURE_C + 1,
            limit_c: MAX_TEMPERATURE_C
        })
    );
}

#[test]
fn torque_on_enters_mit_mode_then_commands_the_measured_position() {
    let mut adapter = leg([1.0, -2.0, 0.5]);
    adapter.check(Overrides::default()).unwrap();
    adapter.open().unwrap();
    adapter.zero().unwrap();
    adapter.transport().motor(0, 2).state.p = -1.5;
    adapter.transport().clear();
    adapter.torque_on().unwrap();
    let order = adapter
        .transport()
        .sent()
        .map(|(_, id, data)| (id, data == ENTER));
    assert!(order.eq([
        (1, true),
        (2, true),
        (3, true),
        (1, false),
        (2, false),
        (3, false)
    ]));
    let sent: [_; N] = commands(adapter.transport());
    let expected = [(1, 1.0, DEFAULT_KP), (2, -1.5, DEFAULT_KP), (3, 0.5, 40.0)];
    for ((id, mit), (want_id, p, kp)) in sent.into_iter().zip(expected) {
        assert_eq!(id, want_id);
        assert!(
            near(mit.p, p),
            "id {id}: {} on the wire, {p} measured",
            mit.p
        );
        assert!(near(mit.v, 0.0) && near(mit.t, 0.0));
        assert!((mit.kp - kp).abs() < 0.2);
    }
}

#[test]
fn the_first_read_gets_the_replies_of_the_start_commands() {
    let (mut adapter, _) = powered([0.5, 0.0, 0.0]);
    let mut slot = Slot::default();
    let mut faults = Faults::default();
    assert_eq!(adapter.read(slot.sample(), &mut faults), Mask::ALL);
    assert_eq!(faults, Faults::default());
    assert!(slot.q.iter().all(|q| near(q.0, 0.0)), "{:?}", slot.q);
}

#[test]
fn read_drains_the_replies_of_the_previous_tick() {
    let (mut adapter, layout) = powered([1.0, 0.0, 0.0]);
    adapter.transport().waiting[0].clear();
    adapter.transport().motor(0, 1).state.p = 1.5;
    adapter.transport().motor(0, 1).state.v = 2.0;
    adapter.transport().motor(0, 1).state.t = -1.0;
    write(&mut adapter, &layout, &Desired::default().q(0, 0.5));

    let mut slot = Slot::default();
    let mut faults = Faults::default();
    let refreshed = adapter.read(slot.sample(), &mut faults);
    assert_eq!(refreshed, Mask::ALL);
    assert_eq!(faults, Faults::default());
    assert!(near(slot.q[0].0, 0.5), "{:?}", slot.q);
    assert!(near(slot.qd[0].0, 2.0) && near(slot.tau[0].0, -1.0));

    let mut faults = Faults::default();
    assert_eq!(adapter.read(slot.sample(), &mut faults), Mask::NONE);
    assert_eq!(faults.stale, Mask::ALL);
    assert_eq!(faults.offline, Mask::NONE);
}

#[test]
fn a_reply_that_does_not_come_is_stale_and_offline() {
    let (mut adapter, layout) = powered([0.0; N]);
    let mut slot = Slot::default();
    adapter.read(slot.sample(), &mut Faults::default());
    adapter.transport().motor(0, 2).answers = false;
    write(&mut adapter, &layout, &Desired::default());
    let unknown = frame::reply(Feedback {
        id: 42,
        p: 0.0,
        v: 0.0,
        t: 0.0,
        temperature: 30,
        fault: 0,
    });
    adapter.transport().inject(0, unknown);
    let mut faults = Faults::default();
    let refreshed = adapter.read(slot.sample(), &mut faults);
    assert!(refreshed.iter().eq([0, 2]));
    assert!(faults.stale.iter().eq([1]));
    assert!(faults.offline.iter().eq([1]));
}

#[test]
fn a_failed_receive_is_a_bus_error_not_an_offline_actuator() {
    let (mut adapter, _) = powered([0.0; N]);
    adapter.transport().recv_error = Some((0, TransportError::Io(5)));
    let mut faults = Faults::default();
    let refreshed = adapter.read(Slot::default().sample(), &mut faults);
    assert_eq!(refreshed, Mask::NONE);
    assert_eq!(faults.bus_error, Mask::ALL);
    assert_eq!(faults.stale, Mask::ALL);
    assert_eq!(faults.offline, Mask::NONE);
}

#[test]
fn every_actuator_gets_one_frame_per_tick_written_or_not() {
    let (mut adapter, layout) = powered([0.0; N]);
    adapter.transport().clear();
    write(&mut adapter, &layout, &Desired::default().q(1, 0.25));
    let sent: [_; N] = commands(adapter.transport());
    assert_eq!(sent.map(|(id, _)| id), [1, 2, 3]);
    assert!(near(sent[0].1.p, 0.0));
    assert!(near(sent[1].1.p, 0.25));

    adapter.transport().clear();
    write(&mut adapter, &layout, &Desired::default());
    let again: [_; N] = commands(adapter.transport());
    assert!(
        near(again[1].1.p, 0.25),
        "an unwritten actuator gets its last command"
    );
}

#[test]
fn written_gains_are_sent_else_the_tables() {
    let (mut adapter, layout) = powered([0.0; N]);
    adapter.transport().clear();
    write(
        &mut adapter,
        &layout,
        &Desired::default().q(0, 0.1).gains(0, 300.0, 2.0).q(2, 0.1),
    );
    let sent: [_; N] = commands(adapter.transport());
    assert!((sent[0].1.kp - 300.0).abs() < 0.2 && (sent[0].1.kd - 2.0).abs() < 0.01);
    assert!((sent[2].1.kp - 40.0).abs() < 0.2 && (sent[2].1.kd - 0.5).abs() < 0.01);

    adapter.transport().clear();
    write(&mut adapter, &layout, &Desired::default().q(0, 0.2));
    let sent: [_; N] = commands(adapter.transport());
    assert!(
        (sent[0].1.kp - DEFAULT_KP).abs() < 0.2,
        "gains fall back each tick"
    );

    let mut slot = Slot::default();
    adapter.read(slot.sample(), &mut Faults::default());
    assert_eq!((slot.kp[0], slot.kd[0]), (Kp(DEFAULT_KP), Kd(DEFAULT_KD)));
}

#[test]
fn velocity_and_torque_are_sent_when_written_else_zero() {
    let (mut adapter, layout) = powered([0.0; N]);
    adapter.transport().clear();
    write(
        &mut adapter,
        &layout,
        &Desired::default().feed_forward(0, 3.0, -2.0),
    );
    let sent: [_; N] = commands(adapter.transport());
    assert!(near(sent[0].1.v, 3.0) && near(sent[0].1.t, -2.0));
    assert!(
        near(sent[0].1.p, 0.0),
        "Q unwritten keeps the last position"
    );

    adapter.transport().clear();
    write(&mut adapter, &layout, &Desired::default().q(0, 0.1));
    let sent: [_; N] = commands(adapter.transport());
    assert!(near(sent[0].1.v, 0.0) && near(sent[0].1.t, 0.0));
}

#[test]
fn an_out_of_range_value_is_rejected_never_clipped() {
    let (mut adapter, layout) = powered([12.0, 0.0, 0.0]);
    adapter.transport().clear();
    // 12.0 + 0.6 is past the 12.5 rad the frame carries.
    let faults = write(
        &mut adapter,
        &layout,
        &Desired::default().q(0, 0.6).feed_forward(1, 0.0, 16.0),
    );
    assert!(faults.rejected.iter().eq([0, 1]));
    let sent: [_; N] = commands(adapter.transport());
    assert!(near(sent[0].1.p, 12.0), "the last command goes again");
    assert!(near(sent[1].1.t, 0.0));

    adapter.transport().clear();
    let faults = write(
        &mut adapter,
        &layout,
        &Desired::default().gains(2, 501.0, 1.0),
    );
    assert!(faults.rejected.iter().eq([2]));
    let nan = write(&mut adapter, &layout, &Desired::default().q(2, f32::NAN));
    assert!(nan.rejected.iter().eq([2]));
}

#[test]
fn a_full_transmit_queue_is_a_bus_error() {
    let (mut adapter, layout) = powered([0.0; N]);
    adapter.read(Slot::default().sample(), &mut Faults::default());
    adapter.transport().send_error = Some(TransportError::Full);
    let faults = write(&mut adapter, &layout, &Desired::default().q(0, 0.1));
    assert_eq!(faults.bus_error, Mask::ALL);
    let mut faults = Faults::default();
    adapter.read(Slot::default().sample(), &mut faults);
    assert_eq!(faults.stale, Mask::ALL);
    assert_eq!(faults.offline, Mask::NONE);
}

#[test]
fn nothing_is_written_before_torque_on() {
    let mut adapter = leg([0.0; N]);
    let claim = adapter.check(Overrides::default()).unwrap();
    let layout = Layout::new(
        claim.group,
        claim.read_every,
        claim.read_cost,
        claim.write_cost,
    );
    adapter.open().unwrap();
    adapter.zero().unwrap();
    adapter.transport().clear();
    write(&mut adapter, &layout, &Desired::default().q(0, 0.1));
    assert_eq!(adapter.transport().sent().count(), 0);
}

#[test]
fn hold_sends_a_pure_position_command_at_the_target() {
    let (mut adapter, _) = powered([1.0, 0.0, 0.0]);
    adapter.transport().clear();
    let target = Desired {
        q: [Q(0.3), Q(-0.2), Q(0.0)],
        qd: [Qd(4.0); N],
        tau: [Tau(1.0); N],
        ..Desired::default()
    }
    .gains(1, 200.0, 2.0);
    adapter.hold(target.command());
    let sent: [_; N] = commands(adapter.transport());
    assert!(near(sent[0].1.p, 1.3) && near(sent[1].1.p, -0.2));
    assert!(
        sent.iter()
            .all(|(_, mit)| near(mit.v, 0.0) && near(mit.t, 0.0))
    );
    assert!((sent[1].1.kp - 200.0).abs() < 0.2);
    assert!((sent[0].1.kp - DEFAULT_KP).abs() < 0.2);
}

#[test]
fn torque_off_sends_the_exit_frame_to_every_actuator() {
    let (mut adapter, _) = powered([0.0; N]);
    adapter.transport().clear();
    adapter.torque_off();
    let sent = adapter.transport().sent();
    assert!(sent.eq([(0, 1, EXIT), (0, 2, EXIT), (0, 3, EXIT)]));
    adapter.close();
    assert_eq!(adapter.transport().calls().last(), Some(&Call::Close));
}

#[test]
fn torque_off_tries_each_actuator_then_moves_on() {
    let (mut adapter, _) = powered([0.0; N]);
    adapter.transport().clear();
    adapter.transport().send_error = Some(TransportError::Full);
    adapter.torque_off();
    assert_eq!(adapter.transport().sent().count(), N * TORQUE_OFF_ATTEMPTS);
}

#[test]
fn nothing_is_sent_before_open() {
    let mut adapter = leg([0.0; N]);
    adapter.torque_off();
    adapter.hold(Desired::default().command());
    assert_eq!(adapter.transport().calls().count(), 0);
}
