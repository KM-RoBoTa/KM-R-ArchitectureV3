//! Contract tests of [`DxlRsV2`] on `dxl_rs_v2`'s scripted serial line: the
//! bytes it sends and how it reads the replies, without hardware. A
//! `dxl_rs_v2` bump that changes either fails here.

use core::cell::Cell;
use std::io::ErrorKind;

use ::dxl_rs_v2::{MockTime, MockTransport};

use super::*;
use crate::dynamixel::tests::allocations;

/// The scripted line, with room for every reply a test scripts.
type Line<'t> = MockTransport<'t, 4096, 4096, 64>;

const XM: Model = Model::Xm430W210;
const AX: Model = Model::Ax12a;

impl<C, const RX: usize, const TX: usize, const CHUNKS: usize> Open<C>
    for MockTransport<'_, RX, TX, CHUNKS>
{
    /// Never: a scripted line is attached, not opened.
    fn open_pair(_path: &str, _baud: u32) -> Result<[(Self, C); 2], BackendError> {
        Err(BackendError::PortOpen(ErrorKind::Unsupported))
    }
}

thread_local! {
    /// Microseconds of this test's clock.
    static NOW: Cell<u64> = const { Cell::new(0) };
}

/// The test clock: 10 µs further on every call.
fn tick() -> u64 {
    NOW.with(|now| {
        now.set(now.get() + 10);
        now.get()
    })
}

/// A backend whose port 0 is `one` (Protocol 1) and `two` (Protocol 2), at
/// 1 Mbit/s.
fn backend<T: Transport>(one: T, two: T) -> Ports<4, T, fn() -> u64> {
    let mut backend = Ports {
        ports: [const { None }; MAX_PORTS],
    };
    let clock: fn() -> u64 = tick;
    backend.attach(0, (one, clock), (two, clock), 1_000_000);
    backend
}

/// The two lines of port 0, taken back from `backend`.
fn lines<T: Transport>(backend: &mut Ports<4, T, fn() -> u64>) -> (T, T) {
    let Some(Some(port)) = backend.ports.get_mut(0).map(Option::take) else {
        panic!("port 0 is attached");
    };
    (port.one.into_parts().0, port.two.into_parts().0)
}

/// One packet, built on the stack.
struct Packet {
    /// Bytes, `len` of them used.
    bytes: [u8; 32],
    /// Bytes used.
    len: usize,
}

impl Packet {
    /// `parts` one after the other.
    fn of(parts: &[&[u8]]) -> Self {
        let mut packet = Self {
            bytes: [0; 32],
            len: 0,
        };
        for byte in parts.iter().flat_map(|part| part.iter()) {
            packet.bytes[packet.len] = *byte;
            packet.len += 1;
        }
        packet
    }

    /// The packet's bytes.
    fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// The Protocol 2 CRC of `bytes` (ROBOTIS: polynomial 0x8005).
fn crc(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x8005
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// A Protocol 2 packet to `id`: `body` after the length, CRC appended.
fn p2(id: u8, body: &[u8]) -> Packet {
    let len = u16::try_from(body.len() + 2).unwrap().to_le_bytes();
    let head = Packet::of(&[&[0xFF, 0xFF, 0xFD, 0x00, id], &len, body]);
    Packet::of(&[head.bytes(), &crc(head.bytes()).to_le_bytes()])
}

/// A Protocol 2 status packet of `id`: error byte `error`, then `data`.
fn status2(id: u8, error: u8, data: &[u8]) -> Packet {
    p2(id, Packet::of(&[&[0x55, error], data]).bytes())
}

/// A Protocol 2 Write instruction of `data` at `address` of `id`.
fn write2(id: u8, address: u16, data: &[u8]) -> Packet {
    p2(
        id,
        Packet::of(&[&[0x03], &address.to_le_bytes(), data]).bytes(),
    )
}

/// A Protocol 1 packet of `id`: `body` after the length, checksum appended.
fn p1(id: u8, body: &[u8]) -> Packet {
    let len = u8::try_from(body.len() + 1).unwrap();
    let sum = body
        .iter()
        .fold(id.wrapping_add(len), |sum, &byte| sum.wrapping_add(byte));
    Packet::of(&[&[0xFF, 0xFF, id, len], body, &[!sum]])
}

/// A Protocol 1 status packet of `id`: error byte 0, then `data`.
fn status1(id: u8, data: &[u8]) -> Packet {
    fault1(id, 0x00, data)
}

/// A Protocol 1 status packet of `id`: error byte `error`, then `data`.
fn fault1(id: u8, error: u8, data: &[u8]) -> Packet {
    p1(id, Packet::of(&[&[error], data]).bytes())
}

/// Where `needle` first starts in `haystack`.
fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack
        .windows(needle.len())
        .position(|window| window == needle)
}

/// Scripts the four replies `dxl_rs_v2`'s Setup of an XM430-W210 at `id`
/// reads, from the `first`-th packet sent: ping, Min and Max Position
/// Limit 0 and 4095, Operating Mode 3.
fn xm_setup<const RX: usize, const TX: usize, const CHUNKS: usize>(
    line: &mut MockTransport<'_, RX, TX, CHUNKS>,
    first: usize,
    id: u8,
) {
    line.reply_to(first, status2(id, 0, &[0x06, 0x04, 45]).bytes())
        .reply_to(first + 1, status2(id, 0, &[0, 0, 0, 0]).bytes())
        .reply_to(first + 2, status2(id, 0, &[0xFF, 0x0F, 0, 0]).bytes())
        .reply_to(first + 3, status2(id, 0, &[3]).bytes());
}

#[test]
fn bring_up_writes_the_start_goal_with_the_torque_off_then_the_torque() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    xm_setup(&mut two, 5, 2);
    // Start goals, torque on, then torque off after the sync write (13),
    // which no status packet answers.
    for (packet, id) in [(9, 1), (10, 2), (11, 1), (12, 2), (14, 1), (15, 2)] {
        two.reply_to(packet, status2(id, 0, &[]).bytes());
    }
    let mut backend = backend(Line::with_capacity(&time), two);

    assert_eq!(backend.declare(0, &[(2, XM), (1, XM)]), Ok(()));
    assert_eq!(
        backend.torque(0, XM, 1, true),
        Err(BackendError::NoStartGoal { id: 1 })
    );
    let start = [(1, 2048), (2, 1000)];
    assert_eq!(
        backend.sync_write_start_goal(0, XM, GoalRegister::Position, &start),
        Ok(())
    );
    assert_eq!(backend.torque(0, XM, 1, true), Ok(()));
    assert_eq!(backend.torque(0, XM, 2, true), Ok(()));
    assert_eq!(
        backend.sync_write_start_goal(0, XM, GoalRegister::Position, &start),
        Err(BackendError::AlreadyPowered { id: 1 })
    );
    assert_eq!(
        backend.sync_write_goal(0, XM, &[(1, 2100), (2, 1100)]),
        Ok(())
    );
    assert_eq!(backend.force_torque_off(0, XM, 1), Ok(()));
    assert_eq!(backend.force_torque_off(0, XM, 2), Ok(()));
    assert_eq!(
        backend.sync_write_goal(0, XM, &[(1, 2100)]),
        Err(BackendError::NotPowered { id: 1 })
    );

    let (one, two) = lines(&mut backend);
    assert!(one.sent().is_empty());
    let sent = two.sent();
    let at = |packet: Packet| find(sent, packet.bytes());
    let goal = |raw: u32| raw.to_le_bytes();
    for (id, raw) in start {
        let goal_at = at(write2(id, 116, &goal(raw)));
        let torque_at = at(write2(id, 64, &[1]));
        assert!(goal_at.is_some(), "id {id}: start goal sent");
        assert!(goal_at < torque_at, "id {id}: start goal before the torque");
        assert!(
            torque_at < at(write2(id, 64, &[0])),
            "id {id}: torque off last"
        );
    }
    let goals = Packet::of(&[&[0x83, 116, 0, 4, 0, 1], &goal(2100), &[2], &goal(1100)]);
    assert!(at(p2(0xFE, goals.bytes())).is_some(), "one sync write");
    assert_eq!(two.unread(), 0);
}

#[test]
fn a_state_read_is_three_partial_sync_reads_folded_per_actuator() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    xm_setup(&mut two, 5, 2);
    let current: i16 = -10;
    two.reply_to(9, status2(1, 0, &current.to_le_bytes()).bytes())
        .reply_to(9, status2(2, 0, &[7, 0]).bytes())
        .reply_to(10, status2(1, 0, &(-3_i32).to_le_bytes()).bytes())
        .reply_to(10, status2(2, 0, &5_i32.to_le_bytes()).bytes())
        .reply_to(11, status2(1, 0, &2048_i32.to_le_bytes()).bytes())
        .reply_to(11, status2(2, 0, &1000_i32.to_le_bytes()).bytes());
    let mut backend = backend(Line::with_capacity(&time), two);
    backend.declare(0, &[(1, XM), (2, XM)]).unwrap();

    let mut out = [Ok(RawState::default()); 2];
    assert_eq!(backend.sync_read_state(0, XM, &[1, 2], &mut out), Ok(()));

    assert_eq!(
        out,
        [
            Ok(RawState {
                position: 2048,
                velocity: -3,
                current: -10,
            }),
            Ok(RawState {
                position: 1000,
                velocity: 5,
                current: 7,
            }),
        ]
    );
    let (_, two) = lines(&mut backend);
    let sync_read = |address: u8, size: u8| p2(0xFE, &[0x82, address, 0, size, 0, 1, 2]);
    for (address, size) in [(126, 2), (128, 4), (132, 4)] {
        let read = sync_read(address, size);
        assert!(find(two.sent(), read.bytes()).is_some(), "{address}");
    }
}

#[test]
fn a_silent_actuator_is_stale_alone_and_the_read_goes_on() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    xm_setup(&mut two, 5, 2);
    two.reply_to(9, status2(1, 0, &[1, 0]).bytes())
        .reply_to(10, status2(1, 0, &[2, 0, 0, 0]).bytes())
        .reply_to(11, status2(1, 0, &[3, 0, 0, 0]).bytes());
    let mut backend = backend(Line::with_capacity(&time), two);
    backend.declare(0, &[(1, XM), (2, XM)]).unwrap();

    let mut out = [Ok(RawState::default()); 2];
    assert_eq!(backend.sync_read_state(0, XM, &[1, 2], &mut out), Ok(()));

    let first = RawState {
        position: 3,
        velocity: 2,
        current: 1,
    };
    assert_eq!(out, [Ok(first), Err(BackendError::Timeout)]);
}

/// Bytes `dxl_rs_v2` sends to set up one XM430-W210: a ping, then three
/// reads.
const XM_SETUP_BYTES: usize = 10 + 3 * 14;

#[test]
fn a_failed_line_fails_the_whole_group_read() {
    let time = MockTime::new();
    let mut two = MockTransport::<'_, 4096, XM_SETUP_BYTES, 64>::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    let mut backend = backend(MockTransport::with_capacity(&time), two);
    backend.declare(0, &[(1, XM)]).unwrap();

    let mut out = [Ok(RawState::default())];
    assert_eq!(
        backend.sync_read_state(0, XM, &[1], &mut out),
        Err(BackendError::Io(Wire::Os(ErrorKind::WriteZero)))
    );
    let (_, two) = lines(&mut backend);
    assert_eq!(two.flushes(), 4, "no read sent after the line failed");
}

#[test]
fn torque_off_reaches_an_actuator_whose_setup_failed() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    xm_setup(&mut two, 6, 1);
    two.reply_to(10, status2(1, 0, &[]).bytes());
    let mut backend = backend(Line::with_capacity(&time), two);

    assert_eq!(
        backend.declare(0, &[(1, XM), (2, XM)]),
        Err(Undeclared {
            id: Some(2),
            source: BackendError::Timeout
        })
    );
    assert_eq!(
        backend.read_position(0, XM, 1),
        Err(BackendError::NotDeclared { id: 1 })
    );
    assert_eq!(backend.force_torque_off(0, XM, 1), Ok(()));

    let (_, two) = lines(&mut backend);
    assert!(find(two.sent(), write2(1, 64, &[0]).bytes()).is_some());
    assert_eq!(two.unread(), 0);
}

/// FR-2.7 of dxl_rs_v2 at f391263: replies flagged with the alert alone
/// hold the actuator faulted instead of failing Setup.
#[test]
fn a_hardware_faulted_actuator_is_held_at_setup() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_alert_setup(&mut two);
    let mut backend = backend(Line::with_capacity(&time), two);

    assert_eq!(backend.declare(0, &[(1, XM)]), Ok(()));
    assert!(backend.faulted(0, XM, 1));
    assert!(!backend.faulted(0, XM, 2) && !backend.faulted(0, AX, 1));
    assert_eq!(backend.read_angle_limits(0, XM, 1), Ok((0, 4095)));
    let (_, two) = lines(&mut backend);
    assert_eq!(two.unread(), 0);
}

#[test]
fn a_setup_reply_with_an_error_beside_the_fault_fails_with_the_fault_measured() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    // Alert with Data Range Error: the reply's data is not trusted.
    two.reply_to(1, status2(1, 0x84, &[0x06, 0x04, 45]).bytes())
        .reply_to(2, status2(1, 0x80, &[0x04]).bytes())
        .reply_to(3, status2(1, 0x80, &[85]).bytes())
        .reply_to(4, status2(1, 0x80, &[80]).bytes());
    let mut backend = backend(Line::with_capacity(&time), two);

    assert_eq!(
        backend.declare(0, &[(1, XM)]),
        Err(Undeclared {
            id: Some(1),
            source: BackendError::Fault(Fault::Temperature {
                measured_c: 85,
                limit_c: 80
            })
        })
    );
    assert!(!backend.faulted(0, XM, 1));
}

#[test]
fn health_reads_its_registers_and_the_limits_through_their_getters() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    // dxl_rs_v2 gives the XM430-W210's limits no unit: each read raw.
    two.reply_to(5, status2(1, 0, &[0]).bytes())
        .reply_to(6, status2(1, 0, &[121, 0]).bytes())
        .reply_to(7, status2(1, 0, &[31]).bytes())
        .reply_to(8, status2(1, 0, &[80]).bytes())
        .reply_to(9, status2(1, 0, &[95, 0]).bytes())
        .reply_to(10, status2(1, 0, &[160, 0]).bytes());
    let mut backend = backend(Line::with_capacity(&time), two);
    backend.declare(0, &[(1, XM)]).unwrap();

    assert_eq!(
        backend.read_health(0, XM, 1),
        Ok(Health {
            error_status: 0,
            voltage_dv: 121,
            temperature_c: 31,
            min_voltage_dv: 95,
            max_voltage_dv: 160,
            temperature_limit_c: 80,
        })
    );
    assert_eq!(backend.read_angle_limits(0, XM, 1), Ok((0, 4095)));
    let (_, two) = lines(&mut backend);
    for (address, size) in [(31, 1), (34, 2), (32, 2)] {
        let read = p2(1, &[0x02, address, 0, size, 0]);
        assert!(find(two.sent(), read.bytes()).is_some(), "{address}");
    }
    assert_eq!(two.unread(), 0);
}

#[test]
fn a_protocol_1_actuator_comes_up_on_its_own_line() {
    let time = MockTime::new();
    let mut one = Line::with_capacity(&time);
    one.reply_to(1, status1(1, &[]).bytes())
        .reply_to(2, status1(1, &[12, 0]).bytes())
        .reply_to(3, status1(1, &[0, 0]).bytes())
        .reply_to(4, status1(1, &[0xFF, 0x03]).bytes())
        .reply_to(5, status1(1, &[]).bytes())
        .reply_to(6, status1(1, &[]).bytes())
        .reply_to(7, status1(1, &[0x00, 0x02]).bytes());
    let mut backend = backend(one, Line::with_capacity(&time));

    assert_eq!(backend.declare(0, &[(1, AX)]), Ok(()));
    assert_eq!(backend.read_angle_limits(0, AX, 1), Ok((0, 1023)));
    let start = [(1, 512)];
    assert_eq!(
        backend.sync_write_start_goal(0, AX, GoalRegister::Position, &start),
        Ok(())
    );
    assert_eq!(backend.torque(0, AX, 1, true), Ok(()));
    assert_eq!(backend.read_position(0, AX, 1), Ok(512));
    let mut out = [Ok(RawState::default())];
    assert_eq!(
        backend.sync_read_state(0, AX, &[1], &mut out),
        Err(BackendError::NoSyncRead)
    );

    let (one, two) = lines(&mut backend);
    assert!(two.sent().is_empty());
    let goal_at = find(one.sent(), p1(1, &[0x03, 30, 0x00, 0x02]).bytes());
    let torque_at = find(one.sent(), p1(1, &[0x03, 24, 1]).bytes());
    assert!(goal_at.is_some() && goal_at < torque_at);
}

#[test]
fn only_goal_position_is_written_before_the_torque() {
    let time = MockTime::new();
    let mut backend = backend(Line::with_capacity(&time), Line::with_capacity(&time));

    assert_eq!(
        backend.sync_write_start_goal(0, XM, GoalRegister::Velocity, &[(1, 0)]),
        Err(BackendError::Unwritable {
            register: GoalRegister::Velocity
        })
    );
    let (one, two) = lines(&mut backend);
    assert!(one.sent().is_empty() && two.sent().is_empty());
}

#[test]
fn every_model_but_the_xw430_t200_is_supported() {
    for model in [
        Model::Ax12a,
        Model::Rx28,
        Model::Mx64,
        Model::Mx64V2,
        Model::Xm430W210,
        Model::Xw540T140,
        Model::Xw540T260,
    ] {
        assert!(<DxlRsV2<4> as Backend>::supports(model), "{model:?}");
    }
    assert!(!<DxlRsV2<4> as Backend>::supports(Model::Xw430T200));
}

#[test]
fn the_reply_timeout_covers_the_latency_timer_and_one_reply_at_the_baud() {
    assert_eq!(config(1_000_000).timeout, Duration::from_micros(1_700));
    assert_eq!(config(57_600).timeout.as_micros(), 1_500 + 3_472);
    assert_eq!(config(1_000_000).retries, 0);
}

#[test]
fn a_state_read_and_a_goal_write_allocate_nothing() {
    const TICKS: usize = 100;
    let time = MockTime::new();
    let mut two = MockTransport::<'_, 4096, 16_384, 64>::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    xm_setup(&mut two, 5, 2);
    for (packet, id) in [(9, 1), (10, 2), (11, 1), (12, 2)] {
        two.reply_to(packet, status2(id, 0, &[]).bytes());
    }
    let mut backend = backend(MockTransport::with_capacity(&time), two);
    backend.declare(0, &[(1, XM), (2, XM)]).unwrap();
    let start = [(1, 2048), (2, 2048)];
    backend
        .sync_write_start_goal(0, XM, GoalRegister::Position, &start)
        .unwrap();
    backend.torque(0, XM, 1, true).unwrap();
    backend.torque(0, XM, 2, true).unwrap();
    let mut out = [Ok(RawState::default()); 2];
    let tick = |backend: &mut Ports<4, _, _>, out: &mut [_]| {
        assert_eq!(backend.sync_read_state(0, XM, &[1, 2], out), Ok(()));
        assert_eq!(backend.sync_write_goal(0, XM, &start), Ok(()));
    };
    tick(&mut backend, &mut out);

    let counted = allocations(|| {
        for _ in 0..TICKS {
            tick(&mut backend, &mut out);
        }
    });

    assert_eq!(counted, 0, "{counted} allocations in {TICKS} ticks");
    assert_eq!(out, [Err(BackendError::Timeout); 2], "nothing answers");
}

#[test]
fn check_refuses_the_xw430_t200_before_any_io() {
    use kmr_core::hal::{Hal, Overrides};

    use crate::dynamixel::{Actuator, Bus as TableBus, Dynamixel, DynamixelError, Mode, Table};
    use crate::dynamixel::{Torque, Zero};

    let actuator = |id, model| Actuator {
        bus: 0,
        id,
        model,
        mode: Mode::Position,
        torque: Torque::On,
        zero: Zero::DEFAULT,
    };
    let table = Table {
        buses: [TableBus {
            label: "arm",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: [actuator(1, XM), actuator(2, Model::Xw430T200)],
    }
    .check();
    let mut adapter = Dynamixel::<_, 1, 2>::new(DxlRsV2::<2>::new(), table.in_order());

    let error = adapter.check(Overrides::default()).err();

    assert_eq!(
        error,
        Some(DynamixelError::Actuator {
            actuator: 1,
            bus: "arm",
            id: 2,
            source: BackendError::UnsupportedModel { number: 1280 },
        })
    );
    assert_eq!(
        error.map(|error| error.to_string()).as_deref(),
        Some("actuator 1: id 2 on bus arm: model number 1280: the SDK has no control table for it")
    );
}

/// One XM430-W210 at id 1 on bus `arm`, owned at engine index 0.
fn one_xm() -> crate::dynamixel::Placement<1, 1> {
    use crate::dynamixel::{Actuator, Bus as TableBus, Mode, Table, Torque, Zero};

    Table {
        buses: [TableBus {
            label: "arm",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: [Actuator {
            bus: 0,
            id: 1,
            model: XM,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
        }],
    }
    .in_order()
}

#[test]
fn the_bring_up_telemetry_is_one_sync_read_per_register_and_lands_in_the_block() {
    use kmr_core::hal::{Faults, Hal, Mask, Overrides};
    use kmr_core::telemetry::Telemetry;

    use crate::dynamixel::Dynamixel;

    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    two.reply_to(5, status2(1, 0, &[121, 0]).bytes())
        .reply_to(6, status2(1, 0, &[31]).bytes())
        .reply_to(7, status2(1, 0, &[0]).bytes());
    let mut adapter = Dynamixel::<_, 1, 1>::new(backend(Line::with_capacity(&time), two), one_xm());
    adapter.check(Overrides::default()).unwrap();
    adapter.open = true;
    adapter.ping().unwrap();
    let (mut telemetry, mut faults) = (Telemetry::default(), Faults::default());

    let refreshed = adapter.telemetry(&mut telemetry, &mut faults);

    assert_eq!((refreshed, faults), (Mask::ALL, Faults::default()));
    assert_eq!(telemetry.voltage_v(0), Ok(Some(12.1)));
    assert_eq!(telemetry.temperature_c(0), Ok(Some(31.0)));
    assert_eq!(telemetry.hardware_error(0), Ok(Some(0)));
    let (_, two) = lines(&mut adapter.backend);
    for (address, size) in [(144, 2), (146, 1), (70, 1)] {
        let read = p2(0xFE, &[0x82, address, 0, size, 0, 1]);
        assert!(find(two.sent(), read.bytes()).is_some(), "{address}");
    }
    assert_eq!(two.unread(), 0);
}

#[test]
fn a_protocol_1_telemetry_read_is_one_read_of_voltage_and_temperature() {
    let time = MockTime::new();
    let mut one = Line::with_capacity(&time);
    one.reply_to(1, status1(1, &[]).bytes())
        .reply_to(2, status1(1, &[12, 0]).bytes())
        .reply_to(3, status1(1, &[0, 0]).bytes())
        .reply_to(4, status1(1, &[0xFF, 0x03]).bytes())
        .reply_to(5, status1(1, &[118, 36]).bytes());
    let mut backend = backend(one, Line::with_capacity(&time));
    backend.declare(0, &[(1, AX)]).unwrap();

    let mut out = [Ok(Reading::default()), Ok(Reading::default())];
    assert_eq!(
        backend.read_vitals(0, AX, Vital::Voltage, &[1, 2], &mut out),
        Ok(())
    );

    assert_eq!(
        out,
        [
            Ok(Reading {
                voltage_dv: Some(118),
                temperature_c: Some(36),
                hardware_error: Some(0),
            }),
            Err(BackendError::NotDeclared { id: 2 }),
        ]
    );
    let (one, two) = lines(&mut backend);
    assert!(two.sent().is_empty());
    assert!(find(one.sent(), p1(1, &[0x02, 42, 2]).bytes()).is_some());
    assert_eq!(one.unread(), 0);
}

#[test]
fn a_faulted_actuator_s_telemetry_is_what_its_fault_measured() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    xm_setup(&mut two, 5, 2);
    // The sync read of Present Temperature: id 1 raises the alert, which
    // dxl_rs_v2 then measures with three reads.
    two.reply_to(9, status2(1, 0x80, &[85]).bytes())
        .reply_to(9, status2(2, 0, &[33]).bytes())
        .reply_to(10, status2(1, 0x80, &[0x04]).bytes())
        .reply_to(11, status2(1, 0x80, &[85]).bytes())
        .reply_to(12, status2(1, 0x80, &[80]).bytes());
    let mut backend = backend(Line::with_capacity(&time), two);
    backend.declare(0, &[(1, XM), (2, XM)]).unwrap();

    let mut out = [Ok(Reading::default()); 2];
    assert_eq!(
        backend.read_vitals(0, XM, Vital::Temperature, &[1, 2], &mut out),
        Ok(())
    );

    assert_eq!(
        out,
        [
            Ok(Reading {
                voltage_dv: None,
                temperature_c: Some(85),
                hardware_error: Some(0x04),
            }),
            Ok(Reading {
                temperature_c: Some(33),
                ..Reading::default()
            }),
        ]
    );
}

#[test]
fn telemetry_reads_allocate_nothing() {
    const ROUNDS: usize = 100;
    let time = MockTime::new();
    let mut two = MockTransport::<'_, 4096, 16_384, 64>::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    xm_setup(&mut two, 5, 2);
    let mut backend = backend(MockTransport::with_capacity(&time), two);
    backend.declare(0, &[(1, XM), (2, XM)]).unwrap();
    let mut out = [Ok(Reading::default()); 2];
    let round = |backend: &mut Ports<4, _, _>, out: &mut [_]| {
        for vital in Vital::ALL {
            assert_eq!(backend.read_vitals(0, XM, vital, &[1, 2], out), Ok(()));
        }
    };
    round(&mut backend, &mut out);

    let counted = allocations(|| {
        for _ in 0..ROUNDS {
            round(&mut backend, &mut out);
        }
    });

    assert_eq!(counted, 0, "{counted} allocations in {ROUNDS} rounds");
    assert_eq!(out, [Err(BackendError::Timeout); 2], "nothing answers");
}

#[test]
fn ram_settings_are_written_then_read_back_at_their_addresses() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    for packet in 5..=8 {
        two.reply_to(packet, status2(1, 0, &[]).bytes());
    }
    two.reply_to(9, status2(1, 0, &800_u16.to_le_bytes()).bytes())
        .reply_to(10, status2(1, 0, &20_u16.to_le_bytes()).bytes())
        .reply_to(11, status2(1, 0, &4000_u16.to_le_bytes()).bytes())
        .reply_to(12, status2(1, 0, &[1]).bytes());
    let mut backend = backend(Line::with_capacity(&time), two);
    backend.declare(0, &[(1, XM)]).unwrap();
    let settings = [
        (RamSetting::PGain, 800, 84),
        (RamSetting::IGain, 20, 82),
        (RamSetting::DGain, 4000, 80),
        (RamSetting::Led, 1, 65),
    ];

    for (setting, value, _) in settings {
        assert_eq!(backend.write_ram(0, XM, 1, setting, value), Ok(()));
    }
    for (setting, value, _) in settings {
        assert_eq!(backend.read_ram(0, XM, 1, setting), Ok(value));
    }
    assert_eq!(
        backend.write_ram(0, Model::Xw540T140, 1, RamSetting::Led, 1),
        Err(BackendError::NoRegister { register: "LED" })
    );

    let (_, two) = lines(&mut backend);
    for (setting, value, address) in settings {
        let data = value.to_le_bytes();
        let data = if setting == RamSetting::Led {
            &data[..1]
        } else {
            &data[..]
        };
        let write = write2(1, address, data);
        assert!(find(two.sent(), write.bytes()).is_some(), "{setting:?}");
    }
    assert_eq!(two.unread(), 0);
}

/// A Protocol 1 line whose AX-12A at id 1 comes up, answers the Torque
/// Enable = 0 write, then answers the voltage limit write with error byte
/// `error`.
fn ax_limit_line<'t>(time: &'t MockTime, error: u8) -> Line<'t> {
    let mut one = Line::with_capacity(time);
    one.reply_to(1, status1(1, &[]).bytes())
        .reply_to(2, status1(1, &[12, 0]).bytes())
        .reply_to(3, status1(1, &[0, 0]).bytes())
        .reply_to(4, status1(1, &[0xFF, 0x03]).bytes())
        .reply_to(5, status1(1, &[]).bytes())
        .reply_to(6, fault1(1, error, &[]).bytes());
    one
}

/// Bytes `dxl_rs_v2` sends to set up one AX-12A: a ping, then three reads.
const AX_SETUP_BYTES: usize = 6 + 3 * 8;

#[test]
fn a_limit_write_whose_reply_reports_the_voltage_fault_it_fixes_counts_as_written() {
    let time = MockTime::new();
    let mut one = ax_limit_line(&time, 0x01);
    // The fault measured: Present Voltage 10.1 V, inside the limits written.
    one.reply_to(7, fault1(1, 0x01, &[101]).bytes())
        .reply_to(8, fault1(1, 0x01, &[95]).bytes())
        .reply_to(9, fault1(1, 0x01, &[130]).bytes());
    let mut backend = backend(one, Line::with_capacity(&time));
    backend.declare(0, &[(1, AX)]).unwrap();

    assert_eq!(backend.confirm_torque_off(0, AX, 1), Ok(()));
    assert_eq!(backend.write_voltage_limits(0, AX, 1, 95, 130), Ok(()));

    let (one, _) = lines(&mut backend);
    let off_at = find(one.sent(), p1(1, &[0x03, 24, 0]).bytes());
    let both_at = find(one.sent(), p1(1, &[0x03, 12, 95, 130]).bytes());
    assert!(
        off_at.is_some() && off_at < both_at,
        "set_voltage_limits: one write of both"
    );
    assert!(find(one.sent(), p1(1, &[0x03, 13, 130]).bytes()).is_none());
    assert_eq!(one.unread(), 0);
}

#[test]
fn a_limit_write_before_the_torque_is_confirmed_off_sends_nothing() {
    let time = MockTime::new();
    let one = ax_limit_line(&time, 0x00);
    let mut backend = backend(one, Line::with_capacity(&time));
    backend.declare(0, &[(1, AX)]).unwrap();

    assert_eq!(
        backend.write_voltage_limits(0, AX, 1, 95, 130),
        Err(BackendError::TorqueUnconfirmed { id: 1 })
    );
    assert_eq!(
        backend.write_temperature_limit(0, AX, 1, 70),
        Err(BackendError::TorqueUnconfirmed { id: 1 })
    );
    assert_eq!(backend.confirm_torque_off(0, AX, 1), Ok(()));
    assert_eq!(
        backend.write_voltage_limits(0, AX, 1, 95, 170),
        Err(BackendError::LimitOutsideRange {
            value: 170,
            min: 50,
            max: 160
        })
    );

    let (one, _) = lines(&mut backend);
    assert_eq!(one.sent().len(), AX_SETUP_BYTES + 8, "the torque off alone");
}

#[test]
fn torque_enable_is_read_raw_at_24_on_protocol_1() {
    let time = MockTime::new();
    let mut one = Line::with_capacity(&time);
    one.reply_to(1, status1(1, &[]).bytes())
        .reply_to(2, status1(1, &[12, 0]).bytes())
        .reply_to(3, status1(1, &[0, 0]).bytes())
        .reply_to(4, status1(1, &[0xFF, 0x03]).bytes())
        .reply_to(5, status1(1, &[1]).bytes());
    let mut backend = backend(one, Line::with_capacity(&time));
    backend.declare(0, &[(1, AX)]).unwrap();

    assert_eq!(backend.read_setting(0, AX, 1, Setting::TorqueEnable), Ok(1));

    let (one, _) = lines(&mut backend);
    assert!(find(one.sent(), p1(1, &[0x02, 24, 1]).bytes()).is_some());
}

#[test]
fn a_temperature_limit_written_to_an_overheating_actuator_counts_as_written() {
    let time = MockTime::new();
    let mut one = Line::with_capacity(&time);
    one.reply_to(1, status1(1, &[]).bytes())
        .reply_to(2, status1(1, &[12, 0]).bytes())
        .reply_to(3, status1(1, &[0, 0]).bytes())
        .reply_to(4, status1(1, &[0xFF, 0x03]).bytes())
        // Torque Enable 0, then Temperature Limit 80 °C, answered
        // Overheating; measured 75 °C against the 80 °C now held. Then both
        // voltage limits.
        .reply_to(5, status1(1, &[]).bytes())
        .reply_to(6, fault1(1, 0x04, &[]).bytes())
        .reply_to(7, fault1(1, 0x04, &[75]).bytes())
        .reply_to(8, fault1(1, 0x04, &[80]).bytes())
        .reply_to(9, status1(1, &[]).bytes());
    let mut backend = backend(one, Line::with_capacity(&time));
    backend.declare(0, &[(1, AX)]).unwrap();

    assert_eq!(backend.confirm_torque_off(0, AX, 1), Ok(()));
    assert_eq!(backend.write_temperature_limit(0, AX, 1, 80), Ok(()));
    assert_eq!(backend.write_voltage_limits(0, AX, 1, 95, 130), Ok(()));

    let (one, _) = lines(&mut backend);
    assert!(find(one.sent(), p1(1, &[0x03, 11, 80]).bytes()).is_some());
    assert!(find(one.sent(), p1(1, &[0x03, 12, 95, 130]).bytes()).is_some());
    assert_eq!(one.unread(), 0);
}

#[test]
fn a_limit_write_refused_by_the_actuator_fails() {
    let time = MockTime::new();
    let one = ax_limit_line(&time, 0x08);
    let mut backend = backend(one, Line::with_capacity(&time));
    backend.declare(0, &[(1, AX)]).unwrap();
    backend.confirm_torque_off(0, AX, 1).unwrap();

    assert_eq!(
        backend.write_voltage_limits(0, AX, 1, 95, 130),
        Err(BackendError::Status(0x08))
    );
}

// ── An actuator held faulted at Setup (docs/deferred.md, the B2 row) ──────

/// One AX-12A at id 1 on bus `arm`, owned at engine index 0, given
/// `settings`.
fn one_ax(settings: crate::dynamixel::Settings) -> crate::dynamixel::Placement<1, 1> {
    use crate::dynamixel::{Actuator, Bus as TableBus, Mode, Table, Torque, Zero};

    let mut placement = Table {
        buses: [TableBus {
            label: "arm",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: [Actuator {
            bus: 0,
            id: 1,
            model: AX,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
        }],
    }
    .in_order();
    placement.settings[0] = settings;
    placement
}

/// Settings writing Min and Max Voltage Limit `min_dv` and `max_dv`.
fn voltage_limits(min_dv: u16, max_dv: u16) -> crate::dynamixel::Settings {
    crate::dynamixel::Settings {
        voltage_limits_dv: Some((min_dv, max_dv)),
        ..crate::dynamixel::Settings::NONE
    }
}

/// A Protocol 1 line whose AX-12A at id 1 measures 10.1 V against its
/// EEPROM limits 10.5–13.0 V: its four Setup replies flag Input Voltage.
fn ax_at_10_1_v(time: &MockTime) -> Line<'_> {
    let mut one = Line::with_capacity(time);
    one.reply_to(1, fault1(1, 0x01, &[]).bytes())
        .reply_to(2, fault1(1, 0x01, &[12, 0]).bytes())
        .reply_to(3, fault1(1, 0x01, &[0, 0]).bytes())
        .reply_to(4, fault1(1, 0x01, &[0xFF, 0x03]).bytes());
    one
}

/// Scripts, from packet `first`, a reply flagged Input Voltage then the
/// three reads `dxl_rs_v2` measures it with: Present Voltage 10.1 V, Min
/// Voltage Limit `min_dv`, Max Voltage Limit 13.0 V.
fn ax_low_voltage(line: &mut Line<'_>, first: usize, min_dv: u8) {
    line.reply_to(first, fault1(1, 0x01, &[]).bytes())
        .reply_to(first + 1, fault1(1, 0x01, &[101]).bytes())
        .reply_to(first + 2, fault1(1, 0x01, &[min_dv]).bytes())
        .reply_to(first + 3, fault1(1, 0x01, &[130]).bytes());
}

/// The adapter for `placement` on lines `one` and `two`, checked, its port
/// open and set up.
fn brought_up<'t>(
    placement: crate::dynamixel::Placement<1, 1>,
    one: Line<'t>,
    two: Line<'t>,
) -> crate::dynamixel::Dynamixel<Ports<4, Line<'t>, fn() -> u64>, 1, 1> {
    use kmr_core::hal::{Hal, Overrides};

    let mut adapter = crate::dynamixel::Dynamixel::<_, 1, 1>::new(backend(one, two), placement);
    adapter.check(Overrides::default()).unwrap();
    adapter.open = true;
    adapter.ping().unwrap();
    adapter
}

/// Whether `sent` holds a Protocol 1 Torque Enable = 1 write to id 1.
fn torque_on_sent(sent: &[u8]) -> bool {
    find(sent, p1(1, &[0x03, 24, 1]).bytes()).is_some()
}

/// The error `torque_on` fails with on the actuator at id 1 of bus `arm`,
/// held faulted: `dxl_rs_v2` refuses its torque, nothing sent.
fn refused() -> crate::dynamixel::DynamixelError {
    crate::dynamixel::DynamixelError::Actuator {
        actuator: 0,
        bus: "arm",
        id: 1,
        source: BackendError::Faulted { id: 1 },
    }
}

/// Scripts, from packet `first`, the replies `torque_on` reads from the
/// AX-12A at id 1 still held faulted, clean again: present position 512,
/// then its start goal written. Torque Enable must never follow.
// DECISION: clean replies, so only the fault the bus holds stands before
// its torque: a flagged present position read would fail `torque_on`
// first and hide a torque let through.
fn ax_clean_start(line: &mut Line<'_>, first: usize) {
    line.reply_to(first, status1(1, &[0x00, 0x02]).bytes())
        .reply_to(first + 1, status1(1, &[]).bytes());
}

/// SRS FR-5.2, FR-6.1: `voltage_limits(9.0, 13.0)` brings up the AX-12A that
/// Setup holds faulted at 10.1 V below its 10.5 V Min Voltage Limit.
#[test]
fn an_ax12a_below_its_voltage_limits_comes_up_once_voltage_limits_are_written() {
    use kmr_core::hal::Hal;

    let time = MockTime::new();
    let mut one = ax_at_10_1_v(&time);
    // Torque Enable 0, answered flagged: measured 10.1 V below 10.5 V.
    ax_low_voltage(&mut one, 5, 105);
    // Both voltage limits, 9.0–13.0 V, answered still flagged: measured
    // 10.1 V inside them. Then the clean ping of `clear_fault`.
    ax_low_voltage(&mut one, 9, 90);
    one.reply_to(13, status1(1, &[]).bytes())
        // Health: voltage, temperature, its limit, both voltage limits.
        .reply_to(14, status1(1, &[101]).bytes())
        .reply_to(15, status1(1, &[30]).bytes())
        .reply_to(16, status1(1, &[70]).bytes())
        .reply_to(17, status1(1, &[90, 130]).bytes())
        // Torque on: present position 512, the start goal, Torque Enable.
        .reply_to(18, status1(1, &[0x00, 0x02]).bytes())
        .reply_to(19, status1(1, &[]).bytes())
        .reply_to(20, status1(1, &[]).bytes());
    let placement = one_ax(voltage_limits(90, 130));
    let mut adapter = brought_up(placement, one, Line::with_capacity(&time));
    assert!(adapter.backend.faulted(0, AX, 1));

    assert_eq!(adapter.health(), Ok(()));
    assert!(!adapter.backend.faulted(0, AX, 1));
    assert_eq!(adapter.configure(), Ok(()));
    assert_eq!(adapter.torque_on(), Ok(()));

    let (one, two) = lines(&mut adapter.backend);
    let sent = one.sent();
    let off_at = find(sent, p1(1, &[0x03, 24, 0]).bytes());
    let limits_at = find(sent, p1(1, &[0x03, 12, 90, 130]).bytes());
    let goal_at = find(sent, p1(1, &[0x03, 30, 0x00, 0x02]).bytes());
    let torque_at = find(sent, p1(1, &[0x03, 24, 1]).bytes());
    assert!(off_at.is_some() && off_at < limits_at && limits_at < goal_at && goal_at < torque_at);
    assert_eq!((one.unread(), two.sent().len()), (0, 0));
}

/// SRS FR-5.2: the same AX-12A with no voltage limits set stays faulted and
/// fails health with its voltage measured; its torque is refused, never
/// written, even once its replies are clean.
#[test]
fn an_ax12a_below_its_voltage_limits_with_none_set_fails_health_torque_off() {
    use kmr_core::hal::Hal;

    use crate::dynamixel::{DynamixelError, Settings};

    let time = MockTime::new();
    let mut one = ax_at_10_1_v(&time);
    // `clear_fault`'s ping, still flagged, then the fault measured.
    one.reply_to(5, fault1(1, 0x01, &[]).bytes())
        .reply_to(6, fault1(1, 0x01, &[101]).bytes())
        .reply_to(7, fault1(1, 0x01, &[105]).bytes())
        .reply_to(8, fault1(1, 0x01, &[130]).bytes());
    ax_clean_start(&mut one, 9);
    let mut adapter = brought_up(one_ax(Settings::NONE), one, Line::with_capacity(&time));

    let error = adapter.health();

    assert_eq!(
        error,
        Err(DynamixelError::Voltage {
            actuator: 0,
            bus: "arm",
            id: 1,
            measured_dv: 101,
            min_dv: 105,
            max_dv: 130,
        })
    );
    assert_eq!(
        error.err().map(|error| error.to_string()).as_deref(),
        Some(
            "actuator 0: id 1 on bus arm: input voltage 10.1 V outside the actuator's limits 10.5–13.0 V"
        )
    );
    assert!(adapter.backend.faulted(0, AX, 1));
    assert_eq!(adapter.configure(), Ok(()));
    assert_eq!(adapter.torque_on(), Err(refused()));
    assert!(adapter.backend.faulted(0, AX, 1));
    let (one, _) = lines(&mut adapter.backend);
    assert!(!torque_on_sent(one.sent()));
    assert_eq!(one.unread(), 0);
}

/// SRS FR-5.2: voltage limits written that still exclude 10.1 V leave the
/// actuator faulted; health fails naming the limits it now holds, and its
/// torque is refused, never written, even once its replies are clean.
#[test]
fn voltage_limits_that_still_exclude_its_voltage_fail_health_torque_off() {
    use kmr_core::hal::Hal;

    use crate::dynamixel::DynamixelError;

    let time = MockTime::new();
    let mut one = ax_at_10_1_v(&time);
    // Torque Enable 0, then both voltage limits 11.0–13.0 V: each answered
    // flagged, measured below the Min Voltage Limit held. Then
    // `clear_fault`'s ping, still flagged.
    ax_low_voltage(&mut one, 5, 105);
    ax_low_voltage(&mut one, 9, 110);
    ax_low_voltage(&mut one, 13, 110);
    ax_clean_start(&mut one, 17);
    let mut adapter = brought_up(
        one_ax(voltage_limits(110, 130)),
        one,
        Line::with_capacity(&time),
    );

    assert_eq!(
        adapter.health(),
        Err(DynamixelError::Voltage {
            actuator: 0,
            bus: "arm",
            id: 1,
            measured_dv: 101,
            min_dv: 110,
            max_dv: 130,
        })
    );
    assert!(adapter.backend.faulted(0, AX, 1));
    assert_eq!(adapter.configure(), Ok(()));
    assert_eq!(adapter.torque_on(), Err(refused()));
    assert!(adapter.backend.faulted(0, AX, 1));
    let (one, _) = lines(&mut adapter.backend);
    assert!(find(one.sent(), p1(1, &[0x03, 12, 110, 130]).bytes()).is_some());
    assert!(!torque_on_sent(one.sent()));
    assert_eq!(one.unread(), 0);
}

/// Scripts the five replies `dxl_rs_v2`'s Setup of an XM430-W210 at id 1
/// whose alert is raised reads: ping, Model Number 1030, Min and Max
/// Position Limit 0 and 4095, Operating Mode 3, every one flagged.
fn xm_alert_setup(line: &mut Line<'_>) {
    alert_setup(line, Model::Xm430W210);
}

/// [`xm_alert_setup`] for `model`, its Model Number in the first two replies.
fn alert_setup(line: &mut Line<'_>, model: Model) {
    let [low, high] = model.number().to_le_bytes();
    line.reply_to(1, status2(1, 0x80, &[low, high, 45]).bytes())
        .reply_to(2, status2(1, 0x80, &[low, high]).bytes())
        .reply_to(3, status2(1, 0x80, &[0, 0, 0, 0]).bytes())
        .reply_to(4, status2(1, 0x80, &[0xFF, 0x0F, 0, 0]).bytes())
        .reply_to(5, status2(1, 0x80, &[3]).bytes());
}

/// Scripts, from packet `first`, a flagged reply carrying `data` then the
/// four reads `dxl_rs_v2` measures a Protocol 2 Input Voltage fault with:
/// Hardware Error Status 0x01, Present Input Voltage 12.0 V, Min and Max
/// Voltage Limit 9.5 and 16.0 V.
fn xm_voltage_alert(line: &mut Line<'_>, first: usize, data: &[u8]) {
    line.reply_to(first, status2(1, 0x80, data).bytes())
        .reply_to(first + 1, status2(1, 0x80, &[0x01]).bytes())
        .reply_to(first + 2, status2(1, 0x80, &[120, 0]).bytes())
        .reply_to(first + 3, status2(1, 0x80, &[95, 0]).bytes())
        .reply_to(first + 4, status2(1, 0x80, &[160, 0]).bytes());
}

/// SRS FR-5.2: a Protocol 2 actuator keeps its alert after its limits are
/// written, until it reboots: they are written, then health fails with its
/// Hardware Error Status, its torque not read. Its torque is refused, never
/// written, even once its replies are clean.
// DECISION: an MX-64(2.0): dxl_rs_v2 gives the XM430-W210's limits no unit,
// so its typed setter refuses them (see the test below).
#[test]
fn a_protocol_2_actuator_whose_alert_persists_after_its_limits_are_written_fails_torque_off() {
    use kmr_core::hal::Hal;

    use crate::dynamixel::{DynamixelError, Placement};

    const MX: Model = Model::Mx64V2;
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    alert_setup(&mut two, MX);
    // Torque Enable 0, then both voltage limits, each answered flagged.
    xm_voltage_alert(&mut two, 6, &[]);
    xm_voltage_alert(&mut two, 11, &[]);
    // `clear_fault`: Hardware Error Status, its reply flagged, then measured.
    xm_voltage_alert(&mut two, 16, &[0x01]);
    // Clean again: `configure` reads Operating Mode 3 and Torque Enable 0;
    // `torque_on` current, velocity and position 2048, then the start goal.
    two.reply_to(21, status2(1, 0, &[3]).bytes())
        .reply_to(22, status2(1, 0, &[0]).bytes())
        .reply_to(23, status2(1, 0, &[0, 0]).bytes())
        .reply_to(24, status2(1, 0, &[0, 0, 0, 0]).bytes())
        .reply_to(25, status2(1, 0, &2048_i32.to_le_bytes()).bytes())
        .reply_to(26, status2(1, 0, &[]).bytes());
    let mut placement: Placement<1, 1> = one_xm();
    placement.table.actuators[0].model = MX;
    placement.settings[0] = voltage_limits(95, 160);
    let mut adapter = brought_up(placement, Line::with_capacity(&time), two);
    assert!(adapter.backend.faulted(0, MX, 1));

    assert_eq!(
        adapter.health(),
        Err(DynamixelError::HardwareError {
            actuator: 0,
            bus: "arm",
            id: 1,
            status: 0x01,
        })
    );
    assert_eq!(adapter.configure(), Ok(()));
    assert_eq!(adapter.torque_on(), Err(refused()));
    assert!(adapter.backend.faulted(0, MX, 1));
    let (_, two) = lines(&mut adapter.backend);
    let sent = two.sent();
    let off_at = find(sent, write2(1, 64, &[0]).bytes());
    // Max then Min Voltage Limit, adjacent at 32: one write of both.
    let limits_at = find(sent, write2(1, 32, &[160, 0, 95, 0]).bytes());
    // The first Torque Enable read is `configure`'s, after the limits.
    let torque_read_at = find(sent, p2(1, &[0x02, 64, 0, 1, 0]).bytes());
    assert!(off_at.is_some() && off_at < limits_at && limits_at < torque_read_at);
    assert!(find(sent, write2(1, 64, &[1]).bytes()).is_none());
    assert_eq!(two.unread(), 0);
}

/// SRS FR-5.2, FR-6.1 (T084): `voltage_limits(9.0, 13.0).restore_limits()`
/// on the AX-12A at 10.1 V. The limits it held, 10.5–13.0 V, come from the
/// fault its read answers with; the run's are written, its fault cleared,
/// and after torque off the old ones go back, the fault in that reply only
/// counted as written.
#[test]
fn the_bench_ax12a_gets_its_voltage_limits_back_after_torque_off() {
    use kmr_core::hal::Hal;

    let time = MockTime::new();
    let mut one = ax_at_10_1_v(&time);
    // Torque Enable 0; the voltage limits read, answered with the fault
    // carrying them; then both written, 9.0–13.0 V.
    ax_low_voltage(&mut one, 5, 105);
    ax_low_voltage(&mut one, 9, 105);
    ax_low_voltage(&mut one, 13, 90);
    // `clear_fault`'s clean ping, then health's four reads.
    one.reply_to(17, status1(1, &[]).bytes())
        .reply_to(18, status1(1, &[101]).bytes())
        .reply_to(19, status1(1, &[30]).bytes())
        .reply_to(20, status1(1, &[70]).bytes())
        .reply_to(21, status1(1, &[90, 130]).bytes())
        // Torque off, then the old limits written back, answered flagged.
        .reply_to(22, status1(1, &[]).bytes());
    ax_low_voltage(&mut one, 23, 105);
    let mut settings = voltage_limits(90, 130);
    settings.restore_limits = true;
    let mut adapter = brought_up(one_ax(settings), one, Line::with_capacity(&time));

    assert_eq!(adapter.health(), Ok(()));
    adapter.torque_off();
    adapter.restore_limits();

    let (one, _) = lines(&mut adapter.backend);
    let sent = one.sent();
    let read_at = find(sent, p1(1, &[0x02, 12, 2]).bytes());
    let run_at = find(sent, p1(1, &[0x03, 12, 90, 130]).bytes());
    let off_at = sent
        .windows(p1(1, &[0x03, 24, 0]).bytes().len())
        .rposition(|window| window == p1(1, &[0x03, 24, 0]).bytes());
    let back_at = find(sent, p1(1, &[0x03, 12, 105, 130]).bytes());
    assert!(read_at.is_some() && read_at < run_at && run_at < off_at && off_at < back_at);
    assert_eq!(one.unread(), 0);
}

/// dxl_rs_v2 at e53b249 gives the XM430-W210's limits no unit: its typed
/// setters refuse them with nothing sent, and kmr says why.
#[test]
fn an_xm430_limit_write_is_refused_without_a_unit() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
    two.reply_to(5, status2(1, 0, &[]).bytes());
    let mut backend = backend(Line::with_capacity(&time), two);
    backend.declare(0, &[(1, XM)]).unwrap();
    backend.confirm_torque_off(0, XM, 1).unwrap();

    let refused = Err(BackendError::Refused {
        reason: "a limit dxl_rs_v2 has no unit for on this model",
    });
    assert_eq!(backend.write_voltage_limits(0, XM, 1, 95, 160), refused);
    assert_eq!(backend.write_temperature_limit(0, XM, 1, 70), refused);
    let (_, two) = lines(&mut backend);
    assert_eq!(two.unread(), 0);
    assert_eq!(two.flushes(), 5, "Setup and the torque off alone");
}
