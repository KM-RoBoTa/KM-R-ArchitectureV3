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
    p1(id, Packet::of(&[&[0x00], data]).bytes())
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

#[test]
fn a_faulted_actuator_fails_its_setup_with_the_fault_measured() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    two.reply_to(1, status2(1, 0x80, &[0x06, 0x04, 45]).bytes())
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
}

#[test]
fn health_reads_its_registers_and_the_limits_raw() {
    let time = MockTime::new();
    let mut two = Line::with_capacity(&time);
    xm_setup(&mut two, 1, 1);
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
