//! Adapter tests on a fake CAN bus of AK actuators, without hardware. The
//! adapter is driven through [`Hal`] in the engine's order.

use core::cell::{Cell, RefCell};
use core::time::Duration;

use cubemars_rs::{Error as SdkError, Frame, FrameId, Transport, TransportError};
use kmr_core::hal::{Batch, Command, Faults, Group, Hal, Layout, Overrides, Sample, Written};
use kmr_core::state::{Kd, Kp, Q, Qd, Tau};
use kmr_core::telemetry::Telemetry;

use super::adapter::Open;
use super::{Actuator, Bus, CubeMars, CubeMarsError, DEFAULT_KD, DEFAULT_KP, Limits, Table};
use crate::binding::{Binding, Port};

/// Actuators of the robot: the GeckoLeg's hip, knee and ankle.
const N: usize = 3;
/// AK60-6 position range on an AK 2.0 board, ±rad.
const P_MAX: f32 = 12.5;
const ENTER: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
const EXIT: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
/// Replies the fake bus holds before the adapter reads them.
const INBOX: usize = 16;

/// One interface, ids 1 to 3, the ankle within ±0.5 rad.
const LEG: Table<1, N> = {
    let mut table = Table {
        buses: [Bus { label: "leg" }],
        actuators: [
            Actuator::ak60_6(0, 1),
            Actuator::ak60_6(0, 2),
            Actuator::ak80_8(0, 3),
        ],
    };
    table.actuators[2].limits = Limits {
        position: Some((-0.5, 0.5)),
        ..Limits::NONE
    };
    table.check()
};

/// The MIT position code of `rad`.
fn code(rad: f32) -> [u8; 2] {
    (((rad + P_MAX) * 65535.0 / (2.0 * P_MAX)).round() as u16).to_be_bytes()
}

/// The position of MIT code `code`.
fn rad(code: [u8; 2]) -> f32 {
    f32::from(u16::from_be_bytes(code)) * 2.0 * P_MAX / 65535.0 - P_MAX
}

/// One fake AK.
#[derive(Clone, Copy)]
struct Motor {
    /// CAN id.
    id: u8,
    /// Answers frames.
    present: bool,
    /// Where it stands, as a MIT position code; a command with a position
    /// gain moves it there at once.
    position: [u8; 2],
    /// Reply temperature byte, °C + 40.
    temperature: u8,
    /// Reply error byte.
    fault: u8,
}

/// The fake CAN interface the transports share.
struct World {
    motors: RefCell<[Motor; N]>,
    inbox: RefCell<[Option<Frame>; INBOX]>,
    /// Command frames received, enter and exit included.
    frames: Cell<usize>,
    enters: Cell<usize>,
    exits: Cell<usize>,
    /// Interfaces opened, by count.
    opened: Cell<usize>,
}

impl World {
    /// Ids 1 to 3, standing at `positions` rad, 30 °C, no fault.
    fn new(positions: [f32; N]) -> Self {
        let motors = core::array::from_fn(|i| Motor {
            id: u8::try_from(i + 1).unwrap(),
            present: true,
            position: code(positions[i]),
            temperature: 70,
            fault: 0,
        });
        Self {
            motors: RefCell::new(motors),
            inbox: RefCell::new([None; INBOX]),
            frames: Cell::new(0),
            enters: Cell::new(0),
            exits: Cell::new(0),
            opened: Cell::new(0),
        }
    }

    /// Changes motor `index`.
    fn motor(&self, index: usize, change: impl FnOnce(&mut Motor)) {
        change(&mut self.motors.borrow_mut()[index]);
    }

    /// Where motor `index` stands, rad.
    fn position(&self, index: usize) -> f32 {
        rad(self.motors.borrow()[index].position)
    }

    /// Queues `frame` for the adapter.
    fn post(&self, frame: Frame) {
        let mut inbox = self.inbox.borrow_mut();
        let slot = inbox.iter_mut().find(|slot| slot.is_none()).unwrap();
        *slot = Some(frame);
    }

    /// The oldest queued frame.
    fn take(&self) -> Option<Frame> {
        let mut inbox = self.inbox.borrow_mut();
        let first = inbox[0].take();
        inbox.rotate_left(1);
        first
    }
}

/// A transport on the fake interface.
struct Fake<'w>(&'w World);

impl Transport for Fake<'_> {
    fn send(&mut self, frame: &Frame) -> Result<(), TransportError> {
        let world = self.0;
        world.frames.set(world.frames.get() + 1);
        let FrameId::Standard(id) = frame.id() else {
            return Ok(());
        };
        let data: [u8; 8] = frame.data().try_into().unwrap();
        match data {
            ENTER => world.enters.set(world.enters.get() + 1),
            EXIT => world.exits.set(world.exits.get() + 1),
            _ => {}
        }
        let mut motors = world.motors.borrow_mut();
        let Some(motor) = motors.iter_mut().find(|m| u16::from(m.id) == id) else {
            return Ok(());
        };
        if !motor.present {
            return Ok(());
        }
        let kp = (u16::from(data[3] & 0x0F) << 8) | u16::from(data[4]);
        if data != ENTER && data != EXIT && kp != 0 {
            motor.position = [data[0], data[1]];
        }
        let [p0, p1] = motor.position;
        let reply = [
            motor.id,
            p0,
            p1,
            0x80,
            0x08,
            0x00,
            motor.temperature,
            motor.fault,
        ];
        let reply = Frame::new(FrameId::Standard(u16::from(motor.id)), &reply).unwrap();
        drop(motors);
        world.post(reply);
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<Frame, TransportError> {
        self.0
            .take()
            .ok_or(TransportError::Timeout { waited: timeout })
    }

    fn try_recv(&mut self) -> Result<Option<Frame>, TransportError> {
        Ok(self.0.take())
    }
}

/// Opens every interface on the fake one.
struct Opener<'w>(&'w World);

impl<'w> Open for Opener<'w> {
    type Transport = Fake<'w>;

    fn open(&mut self, _interface: &'static str) -> Result<Fake<'w>, SdkError> {
        self.0.opened.set(self.0.opened.get() + 1);
        Ok(Fake(self.0))
    }
}

/// The bus plugin of interface `interface`.
const fn plugin(interface: &'static str) -> Binding {
    Binding {
        port: Port::ByDev(interface),
        baud: None,
        bus: None,
    }
}

/// The leg on `world`.
fn leg(world: &World) -> CubeMars<Opener<'_>, 1, N> {
    CubeMars::bind(Opener(world), LEG.in_order(), &[plugin("vcan-test")]).unwrap()
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

/// `adapter` through every bring-up call, torque on.
fn bring_up<O: Open>(adapter: &mut CubeMars<O, 1, N>) -> Layout<N> {
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
    let mut faults = Faults::default();
    adapter.read(Slot::default().sample(), &mut faults);
    assert!(faults.is_empty(), "{faults:?}");
    layout
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

/// One `write` of `desired`.
fn write<O: Open>(
    adapter: &mut CubeMars<O, 1, N>,
    layout: &Layout<N>,
    desired: &Desired,
) -> Faults<N> {
    let mut faults = Faults::default();
    let batch = Batch::new(layout, &desired.written);
    adapter.write(desired.command(), &batch, &mut faults);
    faults
}

/// One `read` into `slot`.
fn read<O: Open>(
    adapter: &mut CubeMars<O, 1, N>,
    slot: &mut Slot,
) -> (kmr_core::hal::Mask<N>, Faults<N>) {
    let mut faults = Faults::default();
    let refreshed = adapter.read(slot.sample(), &mut faults);
    (refreshed, faults)
}

fn near(a: f32, b: f32) -> bool {
    (a - b).abs() < 1e-3
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
    let world = World::new([0.0; N]);
    let plugins = [
        Binding {
            bus: Some("left"),
            ..plugin("vcan0")
        },
        Binding {
            bus: Some("right"),
            ..plugin("vcan1")
        },
    ];
    let mut adapter =
        CubeMars::<_, 2, 4>::bind(Opener(&world), TWO.at([3, 0, 1]), &plugins).unwrap();
    let claim = adapter.check(Overrides::default()).unwrap();
    assert!(claim.owned.iter().eq([0, 1, 3]));
    assert_eq!(
        (claim.group[0], claim.group[1], claim.group[3]),
        (Group(1), Group(1), Group(0))
    );
    assert!(claim.read_every.iter().all(|every| every.get() == 1));
    adapter.open().unwrap();
    assert_eq!(world.opened.get(), 2, "one transport per interface");
}

#[test]
fn bring_up_enters_mit_limp_then_holds_where_each_stands_as_joint_zero() {
    let world = World::new([1.0, -2.0, 0.25]);
    let mut adapter = leg(&world);
    let claim = adapter.check(Overrides::default()).unwrap();
    let _ = claim;
    adapter.open().unwrap();
    assert_eq!(world.frames.get(), 0, "open sends no frame");
    adapter.zero().unwrap();
    adapter.ping().unwrap();
    assert_eq!(world.enters.get(), N);
    adapter.health().unwrap();
    let mut slot = Slot::default();
    adapter.read_all(slot.sample()).unwrap();
    assert!(slot.q.iter().all(|q| near(q.0, 0.0)), "{:?}", slot.q);
    assert_eq!((slot.kp[0], slot.kd[0]), (Kp(DEFAULT_KP), Kd(DEFAULT_KD)));
    adapter.torque_on().unwrap();
    for (i, p) in [1.0, -2.0, 0.25].into_iter().enumerate() {
        assert!(near(world.position(i), p), "actuator {i} moved");
    }
}

#[test]
fn a_written_q_moves_its_actuator_from_its_bring_up_pose() {
    let world = World::new([1.0, 0.0, 0.0]);
    let mut adapter = leg(&world);
    let layout = bring_up(&mut adapter);
    let faults = write(&mut adapter, &layout, &Desired::default().q(0, 0.1));
    assert!(faults.is_empty(), "{faults:?}");
    assert!((world.position(0) - 1.1).abs() < 1e-3);
    let mut slot = Slot::default();
    let (refreshed, faults) = read(&mut adapter, &mut slot);
    assert!(refreshed.iter().eq(0..N), "every actuator answered");
    assert!(faults.is_empty());
    assert!((slot.q[0].0 - 0.1).abs() < 1e-3);
}

#[test]
fn an_unwritten_actuator_gets_its_last_command_again() {
    let world = World::new([0.0; N]);
    let mut adapter = leg(&world);
    let layout = bring_up(&mut adapter);
    write(&mut adapter, &layout, &Desired::default().q(1, 0.2));
    let before = world.frames.get();
    write(&mut adapter, &layout, &Desired::default());
    assert_eq!(world.frames.get() - before, N, "one command per actuator");
    assert!((world.position(1) - 0.2).abs() < 1e-3, "the last goal kept");
}

#[test]
fn a_silent_actuator_is_offline_and_stale_the_others_fresh() {
    let world = World::new([0.0; N]);
    let mut adapter = leg(&world);
    let layout = bring_up(&mut adapter);
    world.motor(1, |m| m.present = false);
    let faults = write(&mut adapter, &layout, &Desired::default());
    assert!(faults.offline.iter().eq([1]));
    let (refreshed, faults) = read(&mut adapter, &mut Slot::default());
    assert!(refreshed.iter().eq([0, 2]));
    assert!(faults.stale.iter().eq([1]));
    assert!(faults.offline.iter().eq([1]));
}

#[test]
fn a_goal_past_the_declared_limit_is_rejected_and_the_last_goes_again() {
    let world = World::new([0.0; N]);
    let mut adapter = leg(&world);
    let layout = bring_up(&mut adapter);
    let faults = write(&mut adapter, &layout, &Desired::default().q(2, 0.9));
    assert!(faults.rejected.iter().eq([2]));
    assert!(near(world.position(2), 0.0), "never sent");
    let (refreshed, _) = read(&mut adapter, &mut Slot::default());
    assert!(refreshed.get(2), "the last command answered");
}

#[test]
fn a_fault_code_cuts_that_actuator_only() {
    let world = World::new([0.0; N]);
    let mut adapter = leg(&world);
    let layout = bring_up(&mut adapter);
    world.motor(0, |m| m.fault = 1);
    let exits = world.exits.get();
    let faults = write(&mut adapter, &layout, &Desired::default());
    assert!(faults.rejected.iter().eq([0]));
    assert_eq!(
        world.exits.get() - exits,
        1,
        "cubemars_rs sent its exit frame"
    );
    world.motor(0, |m| m.fault = 0);
    let (refreshed, _) = read(&mut adapter, &mut Slot::default());
    assert!(refreshed.iter().eq([1, 2]));
    let faults = write(&mut adapter, &layout, &Desired::default().q(0, 0.1));
    assert!(faults.is_empty(), "a cut actuator is no longer commanded");
}

#[test]
fn torque_off_sends_the_exit_frame_to_every_actuator() {
    let world = World::new([0.0; N]);
    let mut adapter = leg(&world);
    bring_up(&mut adapter);
    let exits = world.exits.get();
    adapter.torque_off();
    assert_eq!(world.exits.get() - exits, N);
    adapter.close();
}

#[test]
fn a_missing_actuator_fails_ping_naming_it() {
    let world = World::new([0.0; N]);
    world.motor(2, |m| m.present = false);
    let mut adapter = leg(&world);
    adapter.check(Overrides::default()).unwrap();
    adapter.open().unwrap();
    let error = adapter.ping().unwrap_err();
    assert!(
        matches!(
            error,
            CubeMarsError::Actuator {
                actuator: 2,
                id: 3,
                ..
            }
        ),
        "{error}"
    );
}

#[test]
fn health_refuses_a_temperature_above_the_limit() {
    let world = World::new([0.0; N]);
    world.motor(1, |m| m.temperature = 40 + 95);
    let mut adapter = leg(&world);
    adapter.check(Overrides::default()).unwrap();
    adapter.open().unwrap();
    adapter.ping().unwrap();
    let error = adapter.health().unwrap_err();
    assert!(
        matches!(error, CubeMarsError::Temperature { actuator: 1, .. }),
        "{error}"
    );
}

#[test]
fn telemetry_reports_the_temperature_of_fresh_replies() {
    let world = World::new([0.0; N]);
    let mut adapter = leg(&world);
    bring_up(&mut adapter);
    let mut telemetry = Telemetry::default();
    let fresh = adapter.telemetry(&mut telemetry, &mut Faults::default());
    assert!(fresh.iter().eq(0..N));
    assert_eq!(telemetry.temperature_c(0), Ok(Some(30.0)));
}

#[test]
fn declared_rows_are_checked_at_check() {
    let world = World::new([0.0; N]);
    let mut table = LEG;
    table.actuators[2].id = 1;
    let mut adapter = CubeMars::declared(Opener(&world), table, [Some("vcan-test")]);
    let error = adapter.check(Overrides::default()).unwrap_err();
    assert!(
        matches!(
            error,
            CubeMarsError::DuplicateId {
                actuator: 2,
                first: 0,
                id: 1,
                ..
            }
        ),
        "{error}"
    );
    let mut table = LEG;
    table.actuators[1].kp = 900.0;
    let mut adapter = CubeMars::declared(Opener(&world), table, [Some("vcan-test")]);
    let error = adapter.check(Overrides::default()).unwrap_err();
    assert!(
        matches!(error, CubeMarsError::Declaration { actuator: 1, .. }),
        "{error}"
    );
}

#[test]
fn a_can_bus_refuses_a_serial_id_and_a_baud() {
    let world = World::new([0.0; N]);
    let by_id = Binding {
        port: Port::ById("FT8ISZ6H"),
        ..plugin("")
    };
    let error = CubeMars::<_, 1, N>::bind(Opener(&world), LEG.in_order(), &[by_id]).err();
    assert!(matches!(error, Some(CubeMarsError::NotAnInterface { .. })));
    let baud = Binding {
        baud: Some(1_000_000),
        ..plugin("can0")
    };
    let error = CubeMars::<_, 1, N>::bind(Opener(&world), LEG.in_order(), &[baud]).err();
    assert!(matches!(error, Some(CubeMarsError::BaudOnCan { .. })));
}
