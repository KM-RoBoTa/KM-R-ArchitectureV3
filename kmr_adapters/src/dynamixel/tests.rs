//! Adapter tests on the scripted backend, without hardware.
//!
//! The adapter is driven through [`Hal`] in the engine's order. `open` needs
//! a terminal to flush, so the later phases run on an adapter whose scripted
//! ports [`mark_open`] opened; `open` itself is tested on its error paths,
//! and on pseudo-terminals where the system lends them.

use core::num::NonZeroU32;
use core::time::Duration;
use core::{fmt, iter};
use std::ffi::OsStr;
use std::fs;
use std::io;
use std::os::fd::OwnedFd;
use std::os::unix::ffi::OsStrExt;
use std::os::unix::fs::symlink;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};

use kmr_core::hal::{
    Batch, Claim, Command, Faults, Group, Hal, Layout, Mask, Overrides, Sample, Written,
};
use kmr_core::state::{JOINTS, Q, Qd, Tau};
use tracing_subscriber::layer::{Context, Layer, SubscriberExt};

use super::backend::{Backend, BackendError, GoalRegister, Health, PortSpec, RawState, Wire};
use super::scripted::{Call, HEALTHY, Scripted};
use super::{
    Actuator, Bus, DEFAULT_READ_EVERY, Dynamixel, DynamixelError, Mode, Model, PING_ATTEMPTS,
    Placed, Placement, Protocol, TORQUE_OFF_ATTEMPTS, Table, Torque, Zero, io as tick, port, units,
};
use crate::binding::{Binding, Port};
use crate::presets::{Preset, Rx28Chain, XmBench};

// The scenarios name actuators 0 to 3.
const _: () = assert!(JOINTS >= 4, "the adapter tests need four actuators");

/// Firmware id of actuator `index`: ids `1..=JOINTS`.
const fn id(index: usize) -> u8 {
    index as u8 + 1
}

/// `JOINTS` actuators of `model` on bus 0, powered, in position mode.
const fn chain(model: Model) -> [Actuator; JOINTS] {
    let mut actuators = [Actuator {
        bus: 0,
        id: 0,
        model,
        mode: Mode::Position,
        torque: Torque::On,
        zero: Zero::DEFAULT,
    }; JOINTS];
    let mut index = 0;
    while index < JOINTS {
        actuators[index].id = id(index);
        index += 1;
    }
    actuators
}

/// One bus at 1 Mbit/s, read every `every` ticks.
const fn one_bus(label: &'static str, every: Option<NonZeroU32>) -> [Bus; 1] {
    [Bus {
        label,
        baud: 1_000_000,
        read_every: every,
    }]
}

/// The bus plugin of the USB adapter with serial `serial`, naming bus `bus`.
const fn plugin(serial: &'static str, bus: Option<&'static str>) -> Binding {
    Binding {
        port: Port::ById(serial),
        baud: None,
        bus,
    }
}

/// Protocol 2 chain, read every tick.
const X_TABLE: Table<1, JOINTS> = Table {
    buses: one_bus("x", NonZeroU32::new(1)),
    actuators: chain(Model::Xm430W210),
}
.check();
const X_PLUGINS: [Binding; 1] = [plugin("XCHAIN01", None)];

/// Protocol 1 chain of AX-12A, read every tick.
const AX_TABLE: Table<1, JOINTS> = Table {
    buses: one_bus("ax", NonZeroU32::new(1)),
    actuators: chain(Model::Ax12a),
}
.check();

/// Protocol 1 chain of RX-28, read every tick.
const RX_TABLE: Table<1, JOINTS> = Table {
    buses: one_bus("rx", NonZeroU32::new(1)),
    actuators: chain(Model::Rx28),
}
.check();

/// Serials of the `arm` and `wrist` USB adapters.
const ARM: &str = "ARM00001";
const WRIST: &str = "WRIST001";

/// Even actuators: X on bus `arm`, read every tick. Odd ones: AX-12A on bus
/// `wrist`, read every other tick.
const MIX_TABLE: Table<2, JOINTS> = {
    let mut actuators = chain(Model::Xm430W210);
    let mut index = 1;
    while index < JOINTS {
        actuators[index].bus = 1;
        actuators[index].model = Model::Ax12a;
        index += 2;
    }
    Table {
        buses: [
            Bus {
                label: "arm",
                baud: 1_000_000,
                read_every: NonZeroU32::new(1),
            },
            Bus {
                label: "wrist",
                baud: 57_600,
                read_every: NonZeroU32::new(2),
            },
        ],
        actuators,
    }
    .check()
};
const MIX_PLUGINS: [Binding; 2] = [plugin(ARM, Some("arm")), plugin(WRIST, Some("wrist"))];

/// The read-only actuator of `RO_TABLE`.
const LAST: usize = JOINTS - 1;
/// The Protocol 2 chain with its last actuator read-only.
const RO_TABLE: Table<1, JOINTS> = {
    let mut table = X_TABLE;
    table.actuators[LAST].torque = Torque::ReadOnly;
    table.check()
};

/// The Protocol 2 chain with no cadence on its bus.
const SILENT_TABLE: Table<1, JOINTS> = Table {
    buses: one_bus("silent", None),
    ..X_TABLE
}
.check();

/// Five buses, one more than the backends open.
const FIVE_TABLE: Table<5, JOINTS> = {
    let bus = X_TABLE.buses[0];
    Table {
        buses: [
            Bus { label: "p0", ..bus },
            Bus { label: "p1", ..bus },
            Bus { label: "p2", ..bus },
            Bus { label: "p3", ..bus },
            Bus { label: "p4", ..bus },
        ],
        actuators: X_TABLE.actuators,
    }
    .check()
};

/// An adapter on a fresh scripted backend owning `0..M` in table order, no
/// bus bound.
fn rig<const NB: usize, const M: usize>(table: Table<NB, M>) -> Dynamixel<Scripted, NB> {
    Dynamixel::new(Scripted::new(), table.in_order())
}

/// An adapter on a fresh scripted backend owning `0..M` in table order,
/// `plugins` bound to its buses.
fn bound<const NB: usize, const M: usize>(
    table: Table<NB, M>,
    plugins: &[Binding],
) -> Dynamixel<Scripted, NB> {
    Dynamixel::bind(Scripted::new(), table.in_order(), plugins).unwrap()
}

/// `table` with every bus read every `ticks` ticks.
fn every<const NB: usize>(mut table: Table<NB, JOINTS>, ticks: u32) -> Table<NB, JOINTS> {
    for bus in &mut table.buses {
        bus.read_every = NonZeroU32::new(ticks);
    }
    table
}

/// The newest history slot, as the engine hands it to `read`.
struct Slot {
    q: [Q; JOINTS],
    qd: [Qd; JOINTS],
    tau: [Tau; JOINTS],
}

impl Slot {
    /// Every field at `value`, so an untouched entry is recognizable.
    fn filled(value: f32) -> Self {
        Self {
            q: [Q(value); JOINTS],
            qd: [Qd(value); JOINTS],
            tau: [Tau(value); JOINTS],
        }
    }

    /// The slot as a `Sample`.
    fn sample(&mut self) -> Sample<'_> {
        Sample::new(&mut self.q, &mut self.qd, &mut self.tau)
    }
}

/// This tick's desired state and its write masks.
struct Desired {
    q: [Q; JOINTS],
    qd: [Qd; JOINTS],
    tau: [Tau; JOINTS],
    written: Written,
}

impl Desired {
    /// Nothing written yet, every target at zero.
    fn new() -> Self {
        Self {
            q: [Q(0.0); JOINTS],
            qd: [Qd(0.0); JOINTS],
            tau: [Tau(0.0); JOINTS],
            written: Written::default(),
        }
    }

    /// Writes `q` to actuator `index`.
    fn q(mut self, index: usize, q: Q) -> Self {
        if let Some(slot) = self.q.get_mut(index) {
            *slot = q;
        }
        self.written.q.set(index);
        self
    }

    /// Writes `Q(0.0)` to every actuator.
    fn all_q() -> Self {
        (0..JOINTS).fold(Self::new(), |desired, index| desired.q(index, Q(0.0)))
    }

    /// The desired state as a `Command`.
    fn command(&self) -> Command<'_> {
        Command::new(&self.q, &self.qd, &self.tau, &self.written)
    }
}

/// The layout the engine builds for a robot whose only member made `claim`.
pub(super) fn layout_of(claim: &Claim) -> Layout {
    Layout::new(
        claim.group,
        claim.read_every,
        claim.read_cost,
        claim.write_cost,
    )
}

/// One `write` of `desired` as the engine sends it to the member that made
/// `claim`: its runs, local groups, its written masks only.
fn write<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB>,
    claim: &Claim,
    desired: &Desired,
) -> Faults {
    let mut faults = Faults::default();
    let mut written = desired.written;
    written.q = written.q.intersection(claim.owned);
    written.qd = written.qd.intersection(claim.owned);
    written.tau = written.tau.intersection(claim.owned);
    let layout = layout_of(claim);
    let batch = Batch::new(&layout, &written);
    let command = Command::new(&desired.q, &desired.qd, &desired.tau, &written);
    adapter.write(command, &batch, &mut faults);
    faults
}

/// One `read` into `slot`, with its faults.
fn read<const NB: usize>(adapter: &mut Dynamixel<Scripted, NB>, slot: &mut Slot) -> (Mask, Faults) {
    let mut faults = Faults::default();
    let refreshed = adapter.read(slot.sample(), &mut faults);
    (refreshed, faults)
}

/// Opens the scripted ports and marks the adapter open, as a successful
/// `open` would; the backend log is cleared afterwards.
fn mark_open<const NB: usize>(adapter: &mut Dynamixel<Scripted, NB>) {
    let specs = adapter.buses.map(|bus| PortSpec {
        path: Path::new(NULL),
        baud: bus.baud,
    });
    assert_eq!(adapter.backend.open_ports(&specs), Ok(()));
    adapter.backend.clear_log();
    adapter.open = true;
}

/// `check`, then Init and PostInit in the engine's order on an adapter
/// marked open; the backend log is cleared afterwards.
fn powered<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB>,
) -> Result<Claim, DynamixelError> {
    let layout = adapter.check(Overrides::default())?;
    mark_open(adapter);
    adapter.zero()?;
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;
    adapter.torque_on()?;
    adapter.backend.clear_log();
    Ok(layout)
}

/// Mask of the actuators `index` selects.
fn mask(select: impl Fn(usize) -> bool) -> Mask {
    let mut mask = Mask::NONE;
    (0..JOINTS)
        .filter(|&index| select(index))
        .for_each(|index| mask.set(index));
    mask
}

/// `units::position_to_si` of an in-range `raw`.
fn si(model: Model, raw: i32) -> Q {
    units::position_to_si(model, raw).unwrap()
}

/// Every actuator of `adapter` torque-on before bring-up, as an earlier run
/// can leave it.
fn left_on<const NB: usize>(adapter: &mut Dynamixel<Scripted, NB>) {
    for (_, actuator) in adapter.actuators.iter() {
        adapter.backend.already_on(actuator.bus, actuator.id);
    }
}

/// Ids sent `ForceTorqueOff`, in call order.
fn forced(backend: &Scripted) -> impl Iterator<Item = u8> + '_ {
    backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::ForceTorqueOff)
        .map(|record| record.id)
}

/// Ids `torque_off` sends `ForceTorqueOff` when actuator `index` takes
/// `tries(index)` attempts, in call order.
fn attempted(tries: impl Fn(usize) -> usize) -> impl Iterator<Item = u8> {
    (0..JOINTS).flat_map(move |index| iter::repeat_n(id(index), tries(index)))
}

/// Message of the warning `torque_off` logs per actuator it could not cut.
const OFF_FAILED: &str = "torque off failed";

/// Warnings whose message holds `needle`, logged on this thread by `run`.
fn warnings(needle: &'static str, run: impl FnOnce()) -> usize {
    let counter = Warnings {
        needle,
        count: AtomicUsize::new(0),
    };
    let dispatch = tracing::Dispatch::new(tracing_subscriber::registry().with(counter));
    tracing::dispatcher::with_default(&dispatch, run);
    dispatch
        .downcast_ref::<Warnings>()
        .expect("the counting layer")
        .count
        .load(Ordering::Relaxed)
}

/// Counts the warnings whose message holds `needle`.
struct Warnings {
    needle: &'static str,
    count: AtomicUsize,
}

impl<S: tracing::Subscriber> Layer<S> for Warnings {
    fn on_event(&self, event: &tracing::Event<'_>, _context: Context<'_, S>) {
        if *event.metadata().level() != tracing::Level::WARN {
            return;
        }
        let mut message = Message {
            bytes: [0; MESSAGE],
            len: 0,
        };
        event.record(&mut message);
        let needle = self.needle.as_bytes();
        if message
            .text()
            .windows(needle.len())
            .any(|window| window == needle)
        {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Bytes of an event message kept; the rest is cut.
const MESSAGE: usize = 512;

/// The message of an event, whole: it is written in several chunks when it
/// has arguments.
struct Message {
    bytes: [u8; MESSAGE],
    len: usize,
}

impl Message {
    /// The bytes written so far.
    fn text(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl fmt::Write for Message {
    fn write_str(&mut self, chunk: &str) -> fmt::Result {
        for &byte in chunk.as_bytes() {
            if let Some(slot) = self.bytes.get_mut(self.len) {
                *slot = byte;
                self.len += 1;
            }
        }
        Ok(())
    }
}

impl tracing::field::Visit for Message {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn fmt::Debug) {
        if field.name() == "message" {
            let _ = fmt::Write::write_fmt(self, format_args!("{value:?}"));
        }
    }
}

/// The newest recorded call.
fn last_call(backend: &Scripted) -> Option<Call> {
    backend.calls().last().map(|record| record.call)
}

/// Whether a recorded `Torque` call naming `id` on `port` asked for `on`.
fn torque_asked(backend: &Scripted, port: usize, id: u8, on: bool) -> bool {
    backend.calls().iter().any(|record| {
        record.call == Call::Torque && record.port == port && record.id == id && record.on == on
    })
}

// ── US1: one transaction per group, cadence, write mask (T027) ──────────────

#[test]
#[ignore = "dxl_rs sends 3 sync reads per group until its 10-byte block read (T050, T051)"]
fn protocol_2_chain_is_one_read_and_one_write_per_tick() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;
    assert_eq!(layout.group, [Group(0); JOINTS]);

    let desired = Desired::all_q();
    for tick in 1..=5 {
        let (refreshed, faults) = read(&mut adapter, &mut Slot::filled(0.0));
        let written = write(&mut adapter, &layout, &desired);
        assert_eq!(refreshed, Mask::ALL);
        assert_eq!((faults, written), (Faults::default(), Faults::default()));
        assert_eq!(adapter.backend.count(Call::SyncReadState), tick);
        assert_eq!(adapter.backend.count(Call::SyncWriteGoal), tick);
    }
    assert_eq!(adapter.backend.count(Call::ReadPosition), 0);
    assert_eq!(adapter.backend.last_goals().len(), JOINTS);
    Ok(())
}

#[test]
fn protocol_1_chain_is_one_sync_write_and_one_read_per_actuator() -> Result<(), DynamixelError> {
    let mut adapter = rig(AX_TABLE);
    let layout = powered(&mut adapter)?;

    let (refreshed, faults) = read(&mut adapter, &mut Slot::filled(0.0));
    let written = write(&mut adapter, &layout, &Desired::all_q());

    assert_eq!(refreshed, Mask::ALL);
    assert_eq!((faults, written), (Faults::default(), Faults::default()));
    assert_eq!(adapter.backend.count(Call::ReadPosition), JOINTS);
    assert_eq!(adapter.backend.count(Call::SyncReadState), 0);
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
    let ids: [u8; JOINTS] = core::array::from_fn(id);
    assert!(
        adapter
            .backend
            .last_goals()
            .iter()
            .map(|&(id, _)| id)
            .eq(ids)
    );
    assert!(
        adapter
            .backend
            .last_goals()
            .iter()
            .all(|&(_, raw)| raw == 512)
    );
    Ok(())
}

#[test]
fn two_bus_mix_is_one_transaction_per_group() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    let layout = powered(&mut adapter)?;
    let on_wrist = |index: usize| index % 2 == 1;
    let wrist = (0..JOINTS).filter(|&index| on_wrist(index)).count();

    for index in 0..JOINTS {
        assert_eq!(layout.group[index], Group(u8::from(on_wrist(index))));
        assert_eq!(
            layout.read_every[index].get(),
            if on_wrist(index) { 2 } else { 1 }
        );
    }

    let (refreshed, _) = read(&mut adapter, &mut Slot::filled(0.0));
    assert_eq!(refreshed, Mask::ALL);
    assert_eq!(
        adapter.backend.count(Call::SyncReadState),
        Scripted::STATE_READS.len()
    );
    assert_eq!(adapter.backend.count(Call::ReadPosition), wrist);

    let faults = write(&mut adapter, &layout, &Desired::all_q());
    assert_eq!(faults, Faults::default());
    let writes = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::SyncWriteGoal);
    let ports = writes.map(|record| (record.port, record.model, record.count));
    assert!(ports.eq([
        (0, Some(Model::Xm430W210), JOINTS - wrist),
        (1, Some(Model::Ax12a), wrist),
    ]));

    let (refreshed, _) = read(&mut adapter, &mut Slot::filled(0.0));
    assert_eq!(refreshed, mask(|index| !on_wrist(index)));
    assert_eq!(adapter.backend.count(Call::ReadPosition), wrist);
    Ok(())
}

#[test]
fn cadence_of_three_reads_on_calls_0_3_6_only() -> Result<(), DynamixelError> {
    let mut adapter = rig(every(X_TABLE, 3));
    let layout = powered(&mut adapter)?;
    assert_eq!(layout.read_every.map(NonZeroU32::get), [3; JOINTS]);

    let mut reads = [false; 9];
    for (call, read_now) in reads.iter_mut().enumerate() {
        let before = adapter.backend.count(Call::SyncReadState);
        let (refreshed, faults) = read(&mut adapter, &mut Slot::filled(0.0));
        assert_eq!(faults, Faults::default(), "call {call}");
        let sent = adapter.backend.count(Call::SyncReadState) - before;
        let reads = Scripted::STATE_READS.len() * usize::from(!refreshed.is_empty());
        assert_eq!(sent, reads, "call {call}");
        *read_now = refreshed == Mask::ALL;
    }
    let due: [bool; 9] = core::array::from_fn(|call| call % 3 == 0);
    assert_eq!(reads, due);
    Ok(())
}

#[test]
fn read_all_reads_everything_whatever_the_cadence() -> Result<(), DynamixelError> {
    let mut adapter = rig(every(MIX_TABLE, 7));
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    for (index, actuator) in MIX_TABLE.actuators.iter().enumerate() {
        adapter
            .backend
            .position(actuator.bus, id(index), 600 + index as i32);
    }

    let mut slot = Slot::filled(9.0);
    adapter.read_all(slot.sample())?;

    for (index, actuator) in MIX_TABLE.actuators.iter().enumerate() {
        assert_eq!(slot.q[index], si(actuator.model, 600 + index as i32));
    }
    Ok(())
}

#[test]
fn writing_two_of_four_sends_two() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;
    let chosen = |index: usize| index % 2 == 1;
    let desired = (0..JOINTS)
        .filter(|&index| chosen(index))
        .fold(Desired::new(), |desired, index| desired.q(index, Q(0.0)));

    let faults = write(&mut adapter, &layout, &desired);

    assert_eq!(faults, Faults::default());
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
    let sent = (0..JOINTS)
        .filter(|&index| chosen(index))
        .map(|index| (id(index), 2048));
    assert!(adapter.backend.last_goals().iter().copied().eq(sent));
    Ok(())
}

#[test]
fn nothing_written_sends_nothing() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;

    let faults = write(&mut adapter, &layout, &Desired::new());

    assert_eq!(faults, Faults::default());
    assert!(adapter.backend.calls().is_empty());
    Ok(())
}

#[test]
fn qd_write_on_a_position_actuator_sends_nothing_and_is_rejected() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;
    let mut desired = Desired::new();
    desired.qd[1] = Qd(0.5);
    desired.written.qd.set(1);

    let faults = write(&mut adapter, &layout, &desired);

    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 0);
    assert_eq!(faults.rejected, mask(|index| index == 1));
    assert_eq!(faults.bus_error, Mask::NONE);
    Ok(())
}

#[test]
fn si_to_raw_to_si_round_trips_within_one_unit() -> Result<(), DynamixelError> {
    for (table, unit) in [
        (X_TABLE, core::f32::consts::TAU / 4096.0),
        (AX_TABLE, 300.0_f32.to_radians() / 1024.0),
    ] {
        let mut adapter = rig(table);
        let layout = powered(&mut adapter)?;
        for target in [-1.234, -0.1, 0.0, 0.0001, 0.7, 2.5] {
            write(&mut adapter, &layout, &Desired::new().q(0, Q(target)));
            let goals = adapter.backend.last_goals();
            assert_eq!(
                goals.len(),
                1,
                "{target} rad on {:?}",
                table.actuators[0].model
            );
            let raw = goals.first().map_or(0, |&(_, raw)| raw);
            adapter
                .backend
                .position(0, id(0), i32::try_from(raw).unwrap());

            let mut slot = Slot::filled(0.0);
            read(&mut adapter, &mut slot);
            let error = (slot.q[0].0 - target).abs();
            assert!(error <= unit, "{target} rad came back {:?}", slot.q[0]);
        }
    }
    Ok(())
}

// ── US2: bring-up order and shutdown (T035) ─────────────────────────────────

/// A `by-id` and a `sysfs` directory for one test, removed on drop. The
/// `sysfs` one stays empty: no latency timer, as on a native UART.
struct Roots {
    dir: PathBuf,
    by_id: &'static Path,
    sysfs: &'static Path,
}

impl Roots {
    /// Creates the directories of `test`, unique to this process.
    fn new(test: &str) -> io::Result<Self> {
        let dir = std::env::temp_dir().join(format!("kmr_dxl_{}_{test}", std::process::id()));
        fs::remove_dir_all(&dir).ok();
        let by_id: &'static Path = PathBuf::leak(dir.join("by-id"));
        let sysfs: &'static Path = PathBuf::leak(dir.join("sysfs"));
        fs::create_dir_all(by_id)?;
        fs::create_dir_all(sysfs)?;
        Ok(Self { dir, by_id, sysfs })
    }

    /// Links a `by-id` entry holding `serial` to `device`, named as udev
    /// names an FTDI adapter.
    fn link(&self, serial: &str, device: &Path) -> io::Result<()> {
        symlink(
            device,
            self.by_id.join(format!("usb-FTDI_{serial}-if00-port0")),
        )
    }

    /// A regular file, which no serial port is.
    fn regular_file(&self) -> io::Result<PathBuf> {
        let file = self.dir.join("ttyUSB9");
        fs::write(&file, b"")?;
        Ok(file)
    }

    /// `adapter`, resolving its ports here.
    fn rig<const NB: usize>(&self, adapter: Dynamixel<Scripted, NB>) -> Dynamixel<Scripted, NB> {
        adapter.with_roots(self.by_id, self.sysfs)
    }
}

impl Drop for Roots {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.dir).ok();
    }
}

/// A character device every user can read and write, but no terminal.
const NULL: &str = "/dev/null";
/// Another one, a distinct device.
const ZERO: &str = "/dev/zero";

/// A pseudo-terminal: its master stays open while its slave serves as a
/// port.
struct Pty {
    _master: OwnedFd,
    slave: PathBuf,
}

/// A pseudo-terminal that passes the pre-flight and flushes; `None` where the
/// system lends none.
fn pty() -> Option<Pty> {
    use rustix::pty::{OpenptFlags, grantpt, openpt, ptsname, unlockpt};
    let master = openpt(OpenptFlags::RDWR | OpenptFlags::NOCTTY).ok()?;
    grantpt(&master).ok()?;
    unlockpt(&master).ok()?;
    let name = ptsname(&master, "").ok()?;
    let slave = PathBuf::from(OsStr::from_bytes(name.as_bytes()));
    let usable = port::preflight(&slave).is_ok() && port::flush(&slave).is_ok();
    usable.then_some(Pty {
        _master: master,
        slave,
    })
}

#[test]
fn every_port_is_resolved_before_any_is_flushed() -> io::Result<()> {
    let roots = Roots::new("resolve_first")?;
    // The arm port would fail its flush: reaching it first would name it.
    roots.link(ARM, Path::new(NULL))?;
    let mut adapter = roots.rig(bound(MIX_TABLE, &MIX_PLUGINS));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(
        adapter.open(),
        Err(DynamixelError::PortNotFound {
            bus: "wrist",
            port: Port::ById(WRIST)
        })
    );
    assert!(adapter.backend.calls().is_empty());

    roots.link(WRIST, &roots.regular_file()?)?;
    assert_eq!(
        adapter.open(),
        Err(DynamixelError::PortAccess {
            bus: "wrist",
            port: Port::ById(WRIST)
        })
    );
    assert!(adapter.backend.calls().is_empty());
    Ok(())
}

#[test]
fn a_port_that_is_no_terminal_fails_its_flush_before_any_open() -> io::Result<()> {
    let roots = Roots::new("flush_first")?;
    roots.link(ARM, Path::new(NULL))?;
    roots.link(WRIST, Path::new(ZERO))?;
    let mut adapter = roots.rig(bound(MIX_TABLE, &MIX_PLUGINS));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(
        adapter.open(),
        Err(DynamixelError::PortFlush {
            bus: "arm",
            port: Port::ById(ARM)
        })
    );
    assert_eq!(adapter.backend.count(Call::OpenPorts), 0);
    Ok(())
}

#[test]
fn open_on_terminals_opens_every_port_once_at_its_baud() -> io::Result<()> {
    // A pseudo-terminal flushes like a serial port. Without one there is
    // nothing to open: the error paths above still run.
    let (Some(arm), Some(wrist)) = (pty(), pty()) else {
        return Ok(());
    };
    let roots = Roots::new("open_terminals")?;
    roots.link(ARM, &arm.slave)?;
    roots.link(WRIST, &wrist.slave)?;
    let mut adapter = roots.rig(bound(MIX_TABLE, &MIX_PLUGINS));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(adapter.open(), Ok(()));
    let kinds = adapter.backend.calls().iter().map(|record| record.call);
    assert!(kinds.eq([Call::OpenPorts]));
    assert_eq!(
        adapter
            .backend
            .last(Call::OpenPorts)
            .map(|record| record.count),
        Some(2)
    );
    assert_eq!(adapter.backend.baud(0), Some(1_000_000));
    assert_eq!(adapter.backend.baud(1), Some(57_600));
    assert_eq!(adapter.backend.path(0), Some(arm.slave.as_path()));
    assert_eq!(adapter.backend.path(1), Some(wrist.slave.as_path()));
    assert_eq!(adapter.zero(), Ok(()));
    assert_eq!(
        adapter.backend.calls().len(),
        1,
        "zero is a no-op on Dynamixel"
    );

    adapter.torque_off();
    adapter.close();
    assert_eq!(adapter.backend.count(Call::ForceTorqueOff), JOINTS);
    assert_eq!(adapter.backend.count(Call::ClosePorts), 1);
    assert!(!adapter.backend.is_open());
    Ok(())
}

#[test]
fn no_torque_call_before_configure() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.zero()?;
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    assert_eq!(adapter.backend.count(Call::Ping), JOINTS);
    assert_eq!(adapter.backend.count(Call::ReadHealth), JOINTS);

    adapter.torque_on()?;
    assert_eq!(adapter.backend.count(Call::Torque), JOINTS);
    assert!((0..JOINTS).all(|index| adapter.backend.torqued(0, id(index))));
    Ok(())
}

#[test]
fn a_read_only_actuator_is_read_but_never_torqued() -> Result<(), DynamixelError> {
    let mut adapter = rig(RO_TABLE);
    let layout = adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.position(0, id(LAST), 3000);
    let mut slot = Slot::filled(9.0);
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    adapter.read_all(slot.sample())?;
    adapter.torque_on()?;
    assert_eq!(slot.q[LAST], si(Model::Xm430W210, 3000));

    let (refreshed, _) = read(&mut adapter, &mut Slot::filled(0.0));
    assert!(refreshed.get(LAST));
    let faults = write(&mut adapter, &layout, &Desired::all_q());
    assert_eq!(faults.rejected, mask(|index| index == LAST));
    assert!(
        adapter
            .backend
            .last_goals()
            .iter()
            .all(|goal| goal.0 != id(LAST))
    );
    adapter.hold(Desired::new().command());
    assert!(
        adapter
            .backend
            .last_goals()
            .iter()
            .all(|goal| goal.0 != id(LAST))
    );
    adapter.torque_off();

    assert!(!torque_asked(&adapter.backend, 0, id(LAST), true));
    assert!(forced(&adapter.backend).any(|forced| forced == id(LAST)));
    assert!(!adapter.backend.torqued(0, id(LAST)));
    assert!((0..LAST).all(|index| torque_asked(&adapter.backend, 0, id(index), true)));
    Ok(())
}

#[test]
fn a_missed_ping_names_the_actuator_after_bounded_retries() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.fail_ping(1, id(1));

    assert_eq!(
        adapter.ping(),
        Err(DynamixelError::Missing {
            actuator: 1,
            bus: "wrist",
            id: id(1)
        })
    );
    assert_eq!(adapter.backend.count(Call::Ping), 1 + PING_ATTEMPTS);
    Ok(())
}

#[test]
fn hold_writes_last_q_whatever_was_written() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    powered(&mut adapter)?;
    let mut last = Desired::new();
    for (index, q) in last.q.iter_mut().enumerate() {
        *q = Q(0.1 * index as f32);
    }

    adapter.hold(last.command());

    let mut held = 0;
    for record in adapter.backend.calls() {
        assert_eq!(record.call, Call::SyncWriteGoal);
        for &(id, raw) in adapter.backend.goals(record) {
            let index = usize::from(id) - 1;
            let model = MIX_TABLE.actuators[index].model;
            assert_eq!(record.port, MIX_TABLE.actuators[index].bus);
            assert_eq!(Some(raw), units::position_to_raw(model, last.q[index]));
            held += 1;
        }
    }
    assert_eq!(held, JOINTS);
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 2);
    Ok(())
}

#[test]
fn shutdown_is_hold_then_torque_off_then_close() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;

    adapter.hold(Desired::new().command());
    adapter.torque_off();
    adapter.close();

    let kinds = adapter.backend.calls().iter().map(|record| record.call);
    let expected = core::iter::once(Call::SyncWriteGoal)
        .chain([Call::ForceTorqueOff; JOINTS])
        .chain([Call::ClosePorts]);
    assert!(kinds.eq(expected));
    assert!((0..JOINTS).all(|index| !adapter.backend.torqued(0, id(index))));
    Ok(())
}

#[test]
fn emergency_stop_is_torque_off_then_close() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    powered(&mut adapter)?;

    adapter.torque_off();
    adapter.close();

    let kinds = adapter.backend.calls().iter().map(|record| record.call);
    assert!(
        kinds.eq([Call::ForceTorqueOff; JOINTS]
            .into_iter()
            .chain([Call::ClosePorts]))
    );
    for (index, actuator) in MIX_TABLE.actuators.iter().enumerate() {
        assert!(!adapter.backend.torqued(actuator.bus, id(index)));
    }
    Ok(())
}

#[test]
fn torque_off_reaches_every_actuator_even_when_some_never_answer() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;
    let deaf = |index| index == 0 || index == 2;
    for index in (0..JOINTS).filter(|&index| deaf(index)) {
        adapter
            .backend
            .fail(Call::ForceTorqueOff, 0, id(index), BackendError::Timeout);
    }

    let warned = warnings(OFF_FAILED, || adapter.torque_off());

    let tries = |index| if deaf(index) { TORQUE_OFF_ATTEMPTS } else { 1 };
    assert!(forced(&adapter.backend).eq(attempted(tries)));
    assert!((0..JOINTS).all(|index| adapter.backend.torqued(0, id(index)) == deaf(index)));
    assert_eq!(warned, 2);
    Ok(())
}

#[test]
fn torque_off_retries_an_actuator_until_it_answers() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;
    adapter
        .backend
        .fail_times(Call::ForceTorqueOff, 0, id(1), BackendError::Timeout, 2);

    let warned = warnings(OFF_FAILED, || adapter.torque_off());

    assert!(forced(&adapter.backend).eq(attempted(|index| if index == 1 { 3 } else { 1 })));
    assert!((0..JOINTS).all(|index| !adapter.backend.torqued(0, id(index))));
    assert_eq!(warned, 0);
    Ok(())
}

#[test]
fn a_bring_up_error_after_open_still_ends_with_torque_off_and_close() -> Result<(), DynamixelError>
{
    let mut adapter = rig(X_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.missing(0, id(LAST));

    assert!(adapter.ping().is_err());
    adapter.backend.clear_log();
    adapter.torque_off();
    adapter.close();

    let tries = |index| {
        if index == LAST {
            TORQUE_OFF_ATTEMPTS
        } else {
            1
        }
    };
    assert!(forced(&adapter.backend).eq(attempted(tries)));
    assert_eq!(last_call(&adapter.backend), Some(Call::ClosePorts));
    Ok(())
}

// ── US2: torque off everywhere (T010, T011) ─────────────────────────────────

#[test]
fn every_actuator_ends_torque_off_read_only_included() -> Result<(), DynamixelError> {
    let mut adapter = rig(RO_TABLE);
    left_on(&mut adapter);
    let layout = powered(&mut adapter)?;
    for _ in 0..3 {
        read(&mut adapter, &mut Slot::filled(0.0));
        write(&mut adapter, &layout, &Desired::all_q());
    }
    adapter.backend.clear_log();

    adapter.torque_off();
    adapter.close();

    assert!((0..JOINTS).all(|index| !adapter.backend.torqued(0, id(index))));
    assert!(forced(&adapter.backend).eq((0..JOINTS).map(id)));
    assert_eq!(last_call(&adapter.backend), Some(Call::ClosePorts));
    Ok(())
}

#[test]
fn a_bring_up_error_after_open_leaves_no_id_powered() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    left_on(&mut adapter);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.fail_ping(0, id(1));

    assert!(matches!(
        adapter.ping(),
        Err(DynamixelError::Missing { actuator: 1, .. })
    ));
    adapter.torque_off();
    adapter.close();

    assert!((0..JOINTS).all(|index| !adapter.backend.torqued(0, id(index))));
    assert!(forced(&adapter.backend).eq((0..JOINTS).map(id)));
    assert_eq!(last_call(&adapter.backend), Some(Call::ClosePorts));
    Ok(())
}

#[test]
fn torque_off_before_open_sends_nothing() -> io::Result<()> {
    let roots = Roots::new("torque_off_before_open")?;
    let mut adapter = roots.rig(bound(X_TABLE, &X_PLUGINS));
    left_on(&mut adapter);
    assert!(adapter.check(Overrides::default()).is_ok());
    assert!(adapter.open().is_err());

    adapter.torque_off();
    adapter.close();

    let kinds = adapter.backend.calls().iter().map(|record| record.call);
    assert!(kinds.eq([Call::ClosePorts]));
    Ok(())
}

#[test]
fn torque_off_twice_is_harmless() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;

    adapter.hold(Desired::new().command());
    adapter.torque_off();
    adapter.torque_off();
    adapter.close();
    adapter.torque_off();
    adapter.close();

    assert_eq!(adapter.backend.count(Call::ForceTorqueOff), 2 * JOINTS);
    assert_eq!(adapter.backend.count(Call::ClosePorts), 2);
    assert!((0..JOINTS).all(|index| !adapter.backend.torqued(0, id(index))));
    Ok(())
}

#[test]
fn hold_sends_last_q_to_powered_position_actuators_only() -> Result<(), DynamixelError> {
    let mut adapter = rig(RO_TABLE);
    powered(&mut adapter)?;
    let mut last = Desired::new();
    for (index, q) in last.q.iter_mut().enumerate() {
        *q = Q(0.1 * index as f32);
    }
    last.written.q.set(LAST);

    adapter.hold(last.command());

    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
    let held = (0..LAST).map(|index| {
        let raw = units::position_to_raw(Model::Xm430W210, last.q[index]);
        (id(index), raw)
    });
    let sent = adapter.backend.last_goals().iter();
    assert!(sent.map(|&(id, raw)| (id, Some(raw))).eq(held));
    Ok(())
}

#[test]
fn post_init_writes_torque_off_on_a_read_only_actuator() -> Result<(), DynamixelError> {
    let mut adapter = rig(RO_TABLE);
    adapter.backend.already_on(0, id(LAST));

    powered(&mut adapter)?;

    assert!(!adapter.backend.torqued(0, id(LAST)));
    assert!((0..LAST).all(|index| adapter.backend.torqued(0, id(index))));
    Ok(())
}

// ── US2: torque on from where each actuator is, never a jump ────────────────

/// Position in the log of the first `call` naming `id` on `port`; for
/// `Torque`, the first asking for torque on.
fn first(backend: &Scripted, call: Call, port: usize, id: u8) -> Option<usize> {
    backend.calls().iter().position(|record| {
        record.call == call
            && record.port == port
            && (call != Call::Torque || record.on)
            && backend.ids(record).any(|named| named == id)
    })
}

/// Ids named by the recorded `SyncWriteStartGoal` calls, in call order.
fn started(backend: &Scripted) -> impl Iterator<Item = u8> + '_ {
    backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::SyncWriteStartGoal)
        .flat_map(|record| backend.ids(record))
}

/// `check`, `configure` and `read_all` on an adapter marked open; the
/// backend log is cleared afterwards, for `torque_on` alone.
fn configured<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB>,
) -> Result<(), DynamixelError> {
    adapter.check(Overrides::default())?;
    mark_open(adapter);
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;
    adapter.backend.clear_log();
    Ok(())
}

#[test]
fn torque_on_starts_each_position_actuator_at_its_present_position_read_then()
-> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    configured(&mut adapter)?;
    // Moved since `read_all`: the goal must not come from that sample. Inside
    // the AX range for ten actuators too.
    let present = |index: usize| 300 + 50 * index as i32;
    for (index, actuator) in MIX_TABLE.actuators.iter().enumerate() {
        adapter
            .backend
            .position(actuator.bus, id(index), present(index));
    }

    adapter.torque_on()?;

    for (index, actuator) in MIX_TABLE.actuators.iter().enumerate() {
        let goal = adapter
            .backend
            .goal(actuator.bus, id(index), GoalRegister::Position);
        assert_eq!(i32::try_from(goal), Ok(present(index)), "actuator {index}");
        assert!(adapter.backend.torqued(actuator.bus, id(index)));
    }
    let wrist = (0..JOINTS).filter(|index| index % 2 == 1).count();
    assert_eq!(
        adapter.backend.count(Call::SyncReadState),
        Scripted::STATE_READS.len()
    );
    assert_eq!(adapter.backend.count(Call::ReadPosition), wrist);
    assert_eq!(adapter.backend.count(Call::SyncWriteStartGoal), 2);
    Ok(())
}

#[test]
fn torque_on_reads_then_writes_the_goal_then_enables_the_torque() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    configured(&mut adapter)?;

    adapter.torque_on()?;

    let backend = &adapter.backend;
    for index in 0..JOINTS {
        let read = first(backend, Call::SyncReadState, 0, id(index));
        let goal = first(backend, Call::SyncWriteStartGoal, 0, id(index));
        let torque = first(backend, Call::Torque, 0, id(index));
        assert!(
            read.is_some() && read < goal && goal < torque,
            "actuator {index}: read {read:?}, goal {goal:?}, torque {torque:?}"
        );
    }
    Ok(())
}

#[test]
fn torque_on_zeroes_goal_velocity_and_goal_current() -> Result<(), DynamixelError> {
    // Not through `Table::check`, which refuses both modes for now (FR-018).
    let mut table = X_TABLE;
    table.actuators[1].mode = Mode::Velocity;
    table.actuators[2].mode = Mode::Current;
    let at = core::array::from_fn(|index| index);
    let mut adapter = Dynamixel::new(Scripted::new(), Placement { table, at });
    adapter
        .backend
        .left_goal(0, id(1), GoalRegister::Velocity, 120)
        .left_goal(0, id(1), GoalRegister::Position, 4000)
        .left_goal(0, id(2), GoalRegister::Current, 300);
    configured(&mut adapter)?;

    adapter.torque_on()?;

    let backend = &adapter.backend;
    assert_eq!(backend.goal(0, id(1), GoalRegister::Velocity), 0);
    assert_eq!(backend.goal(0, id(2), GoalRegister::Current), 0);
    assert_eq!(backend.goal(0, id(1), GoalRegister::Position), 4000);
    assert!((0..JOINTS).all(|index| backend.torqued(0, id(index))));
    assert_eq!(backend.count(Call::SyncWriteStartGoal), 3);
    Ok(())
}

#[test]
fn a_failed_present_read_keeps_the_torque_off_and_names_the_actuator() -> Result<(), DynamixelError>
{
    let mut adapter = rig(X_TABLE);
    configured(&mut adapter)?;
    adapter
        .backend
        .fail(Call::SyncReadState, 0, id(2), BackendError::Timeout);

    assert_eq!(
        adapter.torque_on(),
        Err(DynamixelError::Missing {
            actuator: 2,
            bus: "x",
            id: id(2)
        })
    );
    assert_eq!(adapter.backend.count(Call::SyncWriteStartGoal), 0);
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    assert!((0..JOINTS).all(|index| !adapter.backend.torqued(0, id(index))));
    Ok(())
}

#[test]
fn a_failed_start_goal_write_powers_nothing() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    configured(&mut adapter)?;
    adapter
        .backend
        .fail_port(Call::SyncWriteStartGoal, 1, BackendError::Io(Wire::TxFail));

    assert_eq!(
        adapter.torque_on(),
        Err(DynamixelError::Backend {
            bus: "wrist",
            source: BackendError::Io(Wire::TxFail)
        })
    );
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    for (index, actuator) in MIX_TABLE.actuators.iter().enumerate() {
        assert!(!adapter.backend.torqued(actuator.bus, id(index)));
    }
    Ok(())
}

#[test]
fn torque_on_writes_no_goal_to_a_read_only_actuator() -> Result<(), DynamixelError> {
    let mut adapter = rig(RO_TABLE);
    adapter
        .backend
        .left_goal(0, id(LAST), GoalRegister::Position, 1234)
        .position(0, id(LAST), 3000);
    configured(&mut adapter)?;

    adapter.torque_on()?;

    assert!(started(&adapter.backend).eq((0..LAST).map(id)));
    assert_eq!(
        adapter.backend.goal(0, id(LAST), GoalRegister::Position),
        1234
    );
    assert!(!torque_asked(&adapter.backend, 0, id(LAST), true));
    assert!(!adapter.backend.torqued(0, id(LAST)));
    Ok(())
}

// ── US3: faults are named, never a panic (T038) ─────────────────────────────

#[test]
fn unknown_serial_is_port_not_found_with_no_backend_call() -> io::Result<()> {
    let roots = Roots::new("unknown_serial")?;
    roots.link("SOMEOTHER", Path::new(NULL))?;
    let mut adapter = roots.rig(bound(X_TABLE, &X_PLUGINS));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(
        adapter.open(),
        Err(DynamixelError::PortNotFound {
            bus: "x",
            port: X_PLUGINS[0].port
        })
    );
    adapter.torque_off();
    assert!(adapter.backend.calls().is_empty());
    Ok(())
}

#[test]
fn a_missing_actuator_is_named_with_its_bus_and_id() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.missing(0, id(2));

    let error = adapter.ping();

    assert_eq!(
        error,
        Err(DynamixelError::Missing {
            actuator: 2,
            bus: "arm",
            id: id(2)
        })
    );
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some("actuator 2: id 3 on bus arm does not answer")
    );
    Ok(())
}

#[test]
fn a_hot_actuator_is_named_with_its_temperature_and_its_own_limit() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    let hot = Health {
        temperature_c: 90,
        temperature_limit_c: 85,
        ..HEALTHY
    };
    adapter.backend.health(0, id(1), hot);

    let error = adapter.health();

    assert_eq!(
        error,
        Err(DynamixelError::Temperature {
            actuator: 1,
            bus: "x",
            id: id(1),
            measured_c: 90,
            limit_c: 85
        })
    );
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some("actuator 1: id 2 on bus x: temperature 90 °C above the actuator's limit 85 °C")
    );
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    Ok(())
}

#[test]
fn a_hardware_error_is_named_with_its_bits() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    let alarmed = Health {
        error_status: 0b0010_0100,
        ..HEALTHY
    };
    adapter.backend.health(0, id(2), alarmed);

    let error = adapter.health();

    assert_eq!(
        error,
        Err(DynamixelError::HardwareError {
            actuator: 2,
            bus: "x",
            id: id(2),
            status: 0b0010_0100
        })
    );
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some("actuator 2: id 3 on bus x: Hardware Error Status 0b00100100 (overheating, overload)")
    );
    Ok(())
}

#[test]
fn a_failed_read_keeps_the_copied_value_and_sets_stale() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;
    let fresh = RawState {
        position: 3072,
        velocity: 10,
        current: 100,
    };
    for index in 0..JOINTS {
        adapter.backend.state(0, id(index), fresh);
    }
    adapter
        .backend
        .fail(Call::SyncReadState, 0, id(1), BackendError::Timeout);

    let mut slot = Slot::filled(9.0);
    let (refreshed, faults) = read(&mut adapter, &mut slot);

    let lost = mask(|index| index == 1);
    assert_eq!(refreshed, mask(|index| index != 1));
    assert_eq!((faults.stale, faults.offline), (lost, lost));
    assert_eq!(faults.bus_error, Mask::NONE);
    assert_eq!(
        (slot.q[1], slot.qd[1], slot.tau[1]),
        (Q(9.0), Qd(9.0), Tau(9.0))
    );
    assert_eq!(slot.q[0], si(Model::Xm430W210, 3072));
    assert_ne!(slot.qd[0], Qd(9.0));
    assert_ne!(slot.tau[0], Tau(9.0));
    Ok(())
}

#[test]
fn a_failed_group_read_keeps_every_value_and_sets_bus_error() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;
    adapter.backend.fail_sync_read(0);

    let mut slot = Slot::filled(9.0);
    let (refreshed, faults) = read(&mut adapter, &mut slot);

    assert_eq!(refreshed, Mask::NONE);
    assert_eq!((faults.stale, faults.bus_error), (Mask::ALL, Mask::ALL));
    assert_eq!(faults.offline, Mask::NONE);
    assert_eq!(slot.q, [Q(9.0); JOINTS]);
    Ok(())
}

#[test]
fn a_failed_protocol_1_read_keeps_the_copied_value() -> Result<(), DynamixelError> {
    let mut adapter = rig(AX_TABLE);
    powered(&mut adapter)?;
    adapter.backend.missing(0, id(0));

    let mut slot = Slot::filled(9.0);
    let (refreshed, faults) = read(&mut adapter, &mut slot);

    assert_eq!(refreshed, mask(|index| index != 0));
    assert_eq!(faults.stale, mask(|index| index == 0));
    assert_eq!(faults.offline, mask(|index| index == 0));
    assert_eq!(slot.q[0], Q(9.0));
    assert_eq!(adapter.backend.count(Call::ReadPosition), JOINTS);
    Ok(())
}

#[test]
fn a_failed_write_sets_bus_error_and_the_next_tick_still_writes() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;
    adapter.backend.fail_sync_write(0);
    let desired = Desired::all_q();

    let faults = write(&mut adapter, &layout, &desired);
    assert_eq!(faults.bus_error, Mask::ALL);
    assert_eq!(faults.rejected, Mask::NONE);

    let faults = write(&mut adapter, &layout, &desired);
    assert_eq!(faults.bus_error, Mask::ALL);
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 2);

    adapter.backend.heal();
    let faults = write(&mut adapter, &layout, &desired);
    assert_eq!(faults, Faults::default());
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 3);
    assert_eq!(adapter.backend.last_goals().len(), JOINTS);
    Ok(())
}

#[test]
fn an_out_of_range_target_is_rejected_and_the_rest_is_sent() -> Result<(), DynamixelError> {
    for (table, beyond) in [(X_TABLE, 3.5), (AX_TABLE, 2.7)] {
        let mut adapter = rig(table);
        let layout = powered(&mut adapter)?;
        let desired = Desired::all_q()
            .q(0, Q(beyond))
            .q(1, Q(-beyond))
            .q(2, Q(f32::NAN));

        let faults = write(&mut adapter, &layout, &desired);

        assert_eq!(
            faults.rejected,
            mask(|index| index < 3),
            "{:?}",
            table.actuators[0].model
        );
        assert_eq!(faults.bus_error, Mask::NONE);
        let sent = adapter.backend.last_goals().iter().map(|&(id, _)| id);
        assert!(sent.eq((3..JOINTS).map(id)));
    }
    Ok(())
}

// ── Review findings: bring-up names, settings, ranges and costs ─────────────

#[test]
fn more_buses_than_the_backend_opens_fail_check_with_no_backend_call() {
    let mut adapter = rig(FIVE_TABLE);

    assert_eq!(
        adapter.check(Overrides::default()).err(),
        Some(DynamixelError::TooManyBuses { buses: 5, max: 4 })
    );
    assert!(adapter.backend.calls().is_empty());
}

#[test]
fn two_buses_on_one_device_fail_open_with_no_backend_call() -> io::Result<()> {
    let roots = Roots::new("shared_device")?;
    roots.link(ARM, Path::new(NULL))?;
    roots.link(WRIST, Path::new(NULL))?;
    let mut adapter = roots.rig(bound(MIX_TABLE, &MIX_PLUGINS));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(
        adapter.open(),
        Err(DynamixelError::SharedPort {
            first: "arm",
            bus: "wrist"
        })
    );
    assert!(adapter.backend.calls().is_empty());
    Ok(())
}

#[test]
fn a_port_that_fails_to_open_is_named_with_its_bus_and_serial() -> io::Result<()> {
    let (Some(arm), Some(wrist)) = (pty(), pty()) else {
        return Ok(());
    };
    let roots = Roots::new("open_failure")?;
    roots.link(ARM, &arm.slave)?;
    roots.link(WRIST, &wrist.slave)?;
    let mut adapter = roots.rig(bound(MIX_TABLE, &MIX_PLUGINS));
    assert!(adapter.check(Overrides::default()).is_ok());
    adapter.backend.fail_open_port(1, BackendError::PortOpen);

    assert_eq!(
        adapter.open(),
        Err(DynamixelError::PortOpen {
            bus: "wrist",
            port: Port::ById(WRIST),
            source: BackendError::PortOpen
        })
    );

    adapter.backend.fail_open(BackendError::SdkClaimed);
    let error = adapter.open();
    assert_eq!(
        error,
        Err(DynamixelError::Sdk {
            source: BackendError::SdkClaimed
        })
    );
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some(
            "the Dynamixel SDK could not be brought up: the C SDK was already brought up in this process, and takes one bring-up per process"
        )
    );
    Ok(())
}

#[test]
fn a_failed_health_read_names_the_actuator() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter
        .backend
        .fail(Call::ReadHealth, 0, id(1), BackendError::Status(0x20));

    let error = adapter.health();

    assert_eq!(
        error,
        Err(DynamixelError::Actuator {
            actuator: 1,
            bus: "x",
            id: id(1),
            source: BackendError::Status(0x20)
        })
    );
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some("actuator 1: id 2 on bus x: status packet error 0x20")
    );
    Ok(())
}

/// What a healthy actuator answers at `voltage_dv` and `temperature_c`,
/// with the limits of [`HEALTHY`]: 6.0–19.0 V, 80 °C.
const fn vitals(voltage_dv: u16, temperature_c: u8) -> Health {
    Health {
        voltage_dv,
        temperature_c,
        ..HEALTHY
    }
}

#[test]
fn protocol_1_health_checks_the_actuators_own_limits() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.health()?;
    let wrist = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::ReadHealth && record.port == 1)
        .count();
    assert_eq!(wrist, JOINTS / 2);

    let cases = [
        (
            vitals(59, 30),
            "actuator 1: id 2 on bus wrist: input voltage 5.9 V outside the actuator's limits 6.0–19.0 V",
        ),
        (
            vitals(191, 30),
            "actuator 1: id 2 on bus wrist: input voltage 19.1 V outside the actuator's limits 6.0–19.0 V",
        ),
        (
            vitals(120, 81),
            "actuator 1: id 2 on bus wrist: temperature 81 °C above the actuator's limit 80 °C",
        ),
    ];
    for (health, message) in cases {
        adapter.backend.health(1, id(1), health);
        let error = adapter.health().err();
        assert_eq!(
            error.map(|error| error.to_string()).as_deref(),
            Some(message)
        );
    }
    adapter.backend.health(1, id(1), vitals(59, 30));
    assert_eq!(
        adapter.health(),
        Err(DynamixelError::Voltage {
            actuator: 1,
            bus: "wrist",
            id: id(1),
            measured_dv: 59,
            min_dv: 60,
            max_dv: 190
        })
    );
    Ok(())
}

#[test]
fn outside_the_datasheet_inside_its_own_limits_warns_and_passes() -> Result<(), DynamixelError> {
    let mut adapter = rig(RX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    // The RX-28 bench: 11.1 V, below the 12.0 V of the datasheet, inside
    // the factory limits 6.0–19.0 V.
    adapter.backend.health(0, id(3), vitals(111, 45));

    let mut health = Ok(());
    let warned = warnings(
        "input voltage 11.1 V outside the RX-28 datasheet range 12.0–18.5 V, inside the actuator's limits 6.0–19.0 V",
        || health = adapter.health(),
    );
    assert_eq!((health, warned), (Ok(()), 1));

    let mut health = Ok(());
    let hot = Health {
        temperature_c: 85,
        temperature_limit_c: 90,
        ..HEALTHY
    };
    adapter.backend.health(0, id(3), hot);
    let warned = warnings(
        "temperature 85 °C above the RX-28 datasheet maximum 80 °C, within the actuator's limit 90 °C",
        || health = adapter.health(),
    );
    assert_eq!((health, warned), (Ok(()), 1));
    Ok(())
}

#[test]
fn rx28_voltage_outside_its_own_limits_fails_naming_value_and_limits() -> Result<(), DynamixelError>
{
    let mut adapter = rig(RX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    let narrow = Health {
        voltage_dv: 111,
        min_voltage_dv: 120,
        ..HEALTHY
    };
    adapter.backend.health(0, id(3), narrow);

    let error = adapter.health();

    assert_eq!(
        error,
        Err(DynamixelError::Voltage {
            actuator: 3,
            bus: "rx",
            id: id(3),
            measured_dv: 111,
            min_dv: 120,
            max_dv: 190
        })
    );
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some(
            "actuator 3: id 4 on bus rx: input voltage 11.1 V outside the actuator's limits 12.0–19.0 V"
        )
    );
    Ok(())
}

#[test]
fn a_group_timeout_is_a_bus_error_and_never_blames_one_actuator() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;
    adapter
        .backend
        .fail_port(Call::SyncReadState, 0, BackendError::Timeout);

    let (refreshed, faults) = read(&mut adapter, &mut Slot::filled(9.0));
    assert_eq!(refreshed, Mask::NONE);
    assert_eq!((faults.stale, faults.bus_error), (Mask::ALL, Mask::ALL));
    assert_eq!(faults.offline, Mask::NONE);

    assert_eq!(
        adapter.read_all(Slot::filled(9.0).sample()),
        Err(DynamixelError::Backend {
            bus: "x",
            source: BackendError::Timeout
        })
    );
    Ok(())
}

#[test]
fn protocol_1_rates_are_declared_unread() {
    let unread = tick::unread_rates(&Placed::new(&MIX_TABLE.in_order()));
    assert_eq!(unread, mask(|index| index % 2 == 1));
    assert_eq!(
        tick::unread_rates(&Placed::new(&X_TABLE.in_order())),
        Mask::NONE
    );
    assert_eq!(
        tick::unread_rates(&Placed::new(&AX_TABLE.in_order())),
        Mask::ALL
    );
}

#[test]
fn read_cost_counts_every_sync_read_of_the_backend() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = adapter.check(Overrides::default())?;

    // Current, velocity and position: one 14-byte instruction each, then per
    // member its id and a status of 11 bytes around the data.
    let members = JOINTS as u64;
    let bytes: u64 = [2, 4, 4]
        .iter()
        .map(|data| 14 + members * (1 + 11 + data))
        .sum();
    // 10 bits per byte at 1 Mbit/s.
    let expected = Duration::from_micros(bytes * 10);
    assert_eq!(layout.read_cost, expected);
    Ok(())
}

#[test]
fn a_position_outside_the_register_range_is_stale_never_a_reading() -> Result<(), DynamixelError> {
    for (table, beyond) in [(X_TABLE, 4096), (AX_TABLE, 1024)] {
        let mut adapter = rig(table);
        powered(&mut adapter)?;
        adapter
            .backend
            .position(0, id(1), -1)
            .position(0, id(2), beyond);

        let mut slot = Slot::filled(9.0);
        let (refreshed, faults) = read(&mut adapter, &mut slot);

        let bad = mask(|index| index == 1 || index == 2);
        let model = table.actuators[0].model;
        assert_eq!(refreshed, mask(|index| !bad.get(index)), "{model:?}");
        assert_eq!(faults.stale, bad, "{model:?}");
        assert_eq!((faults.offline, faults.bus_error), (Mask::NONE, Mask::NONE));
        assert_eq!(
            (slot.q[1], slot.qd[1], slot.tau[1]),
            (Q(9.0), Qd(9.0), Tau(9.0))
        );
        assert_eq!(slot.q[2], Q(9.0));

        // Protocol 1 reads the register unsigned: -1 comes back as u32::MAX.
        let raw = match model.protocol() {
            Protocol::Two => -1,
            Protocol::One => i64::from(u32::MAX),
        };
        let max = units::max_position(model);
        let error = adapter.read_all(Slot::filled(9.0).sample());
        assert_eq!(
            error,
            Err(DynamixelError::PositionRange {
                actuator: 1,
                bus: table.buses[0].label,
                id: id(1),
                raw,
                max
            })
        );
        let message = error.err().map(|error| error.to_string());
        let expected = format!(
            "actuator 1: id 2 on bus {}: present position {raw} outside the register range 0–{max}",
            table.buses[0].label
        );
        assert_eq!(message, Some(expected));
    }
    Ok(())
}

#[test]
fn an_actuator_outside_position_mode_fails_configure_before_any_torque()
-> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    // Extended position: goals in 0..=4095 would be multi-turn coordinates.
    adapter.backend.mode(0, id(2), 4);

    let error = adapter.configure();

    assert_eq!(
        error,
        Err(DynamixelError::OperatingMode {
            actuator: 2,
            bus: "x",
            id: id(2),
            found: 4,
            expected: 3
        })
    );
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some(
            "actuator 2: id 3 on bus x: Operating Mode 4 (extended position), expected 3 (position)"
        )
    );
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    Ok(())
}

#[test]
fn protocol_1_reads_its_angle_limits_never_a_setting() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.configure()?;

    let calls = adapter.backend.calls();
    let on = |call: Call, port: usize| {
        calls
            .iter()
            .filter(move |record| record.call == call)
            .all(|record| record.port == port)
    };
    assert!(on(Call::ReadSetting, 0) && on(Call::ReadAngleLimits, 1));
    let x = (0..JOINTS).filter(|index| index % 2 == 0).count();
    assert_eq!(adapter.backend.count(Call::ReadSetting), 2 * x);
    assert_eq!(adapter.backend.count(Call::ReadAngleLimits), JOINTS - x);
    Ok(())
}

#[test]
fn protocol_1_outside_joint_mode_fails_configure_before_any_torque() -> Result<(), DynamixelError> {
    let cases = [
        (
            0,
            0,
            DynamixelError::WheelMode {
                actuator: 2,
                bus: "rx",
                id: id(2),
            },
            "actuator 2: id 3 on bus rx: CW and CCW Angle Limit both 0: wheel mode, not joint mode",
        ),
        (
            700,
            300,
            DynamixelError::AngleLimitsReversed {
                actuator: 2,
                bus: "rx",
                id: id(2),
                cw: 700,
                ccw: 300,
            },
            "actuator 2: id 3 on bus rx: CW Angle Limit 700 above CCW Angle Limit 300",
        ),
    ];
    for (cw, ccw, expected, message) in cases {
        let mut adapter = rig(RX_TABLE);
        adapter.check(Overrides::default())?;
        mark_open(&mut adapter);
        adapter.backend.angle_limits(0, id(2), cw, ccw);

        let error = adapter.configure();
        assert_eq!(error, Err(expected));
        assert_eq!(
            error.err().map(|error| error.to_string()).as_deref(),
            Some(message)
        );
        assert_eq!(adapter.backend.count(Call::Torque), 0);
    }
    Ok(())
}

#[test]
fn a_failed_angle_limit_read_names_the_actuator() -> Result<(), DynamixelError> {
    let mut adapter = rig(RX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.missing(0, id(1));

    assert_eq!(
        adapter.configure(),
        Err(DynamixelError::Missing {
            actuator: 1,
            bus: "rx",
            id: id(1)
        })
    );
    Ok(())
}

/// The RX-28 chain with actuator 1 limited to raw `300..=700`, at raw 500.
fn limited() -> Dynamixel<Scripted, 1> {
    let mut adapter = rig(RX_TABLE);
    adapter
        .backend
        .angle_limits(0, id(1), 300, 700)
        .position(0, id(1), 500);
    adapter
}

#[test]
fn a_goal_outside_the_angle_limits_is_rejected_never_clamped() -> Result<(), DynamixelError> {
    let mut adapter = limited();
    let layout = powered(&mut adapter)?;
    let desired = Desired::new()
        .q(0, si(Model::Rx28, 800))
        .q(1, si(Model::Rx28, 800))
        .q(2, si(Model::Rx28, 700));

    let faults = write(&mut adapter, &layout, &desired);
    assert_eq!(faults.rejected, mask(|index| index == 1));
    assert_eq!(adapter.backend.last_goals(), &[(id(0), 800), (id(2), 700)]);

    let faults = write(
        &mut adapter,
        &layout,
        &Desired::new().q(1, si(Model::Rx28, 700)),
    );
    assert_eq!(faults, Faults::default());
    assert_eq!(adapter.backend.last_goals(), &[(id(1), 700)]);
    Ok(())
}

#[test]
fn hold_skips_a_target_outside_the_angle_limits() -> Result<(), DynamixelError> {
    let mut adapter = limited();
    powered(&mut adapter)?;
    let mut last = Desired::new();
    last.q = [si(Model::Rx28, 200); JOINTS];

    adapter.hold(last.command());

    let held = adapter.backend.last_goals().iter().map(|&(id, _)| id);
    assert!(held.eq((0..JOINTS).filter(|&index| index != 1).map(id)));
    Ok(())
}

#[test]
fn a_present_position_outside_the_angle_limits_powers_nothing() -> Result<(), DynamixelError> {
    let mut adapter = limited();
    adapter.backend.position(0, id(1), 200);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.configure()?;

    let error = adapter.torque_on();

    assert_eq!(
        error,
        Err(DynamixelError::OutsideAngleLimits {
            actuator: 1,
            bus: "rx",
            id: id(1),
            raw: 200,
            cw: 300,
            ccw: 700
        })
    );
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some(
            "actuator 1: id 2 on bus rx: present position 200 outside the CW/CCW Angle Limit 300–700"
        )
    );
    assert_eq!(adapter.backend.count(Call::SyncWriteStartGoal), 0);
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    Ok(())
}

#[test]
fn rx28_chain_runs_every_step_on_protocol_1() -> Result<(), DynamixelError> {
    let mut adapter = rig(RX_TABLE);
    for index in 0..JOINTS {
        let raw = 400 + 10 * i32::try_from(index).unwrap_or(0);
        adapter.backend.position(0, id(index), raw);
    }
    let layout = adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.zero()?;
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    let mut slot = Slot::filled(9.0);
    adapter.read_all(slot.sample())?;
    adapter.torque_on()?;
    let (refreshed, faults) = read(&mut adapter, &mut slot);
    let written = write(&mut adapter, &layout, &Desired::all_q());
    adapter.hold(Desired::all_q().command());
    adapter.torque_off();
    adapter.close();

    assert_eq!(
        (refreshed, faults, written),
        (Mask::ALL, Faults::default(), Faults::default())
    );
    for (index, q) in slot.q.iter().enumerate() {
        let raw = 400 + 10 * i32::try_from(index).unwrap_or(0);
        assert_eq!(*q, si(Model::Rx28, raw));
    }
    let counts = [
        (Call::Ping, JOINTS),
        (Call::ReadHealth, JOINTS),
        (Call::ReadAngleLimits, JOINTS),
        (Call::ReadSetting, 0),
        (Call::ReadPosition, 3 * JOINTS),
        (Call::SyncReadState, 0),
        (Call::SyncWriteStartGoal, 1),
        (Call::Torque, JOINTS),
        (Call::SyncWriteGoal, 2),
        (Call::ForceTorqueOff, JOINTS),
        (Call::ClosePorts, 1),
    ];
    for (call, count) in counts {
        assert_eq!(adapter.backend.count(call), count, "{call:?}");
    }
    let start = adapter.backend.last(Call::SyncWriteStartGoal);
    let start = start.map(|record| adapter.backend.goals(record));
    let expected: [(u8, u32); JOINTS] =
        core::array::from_fn(|index| (id(index), 400 + 10 * index as u32));
    assert_eq!(start, Some(&expected[..]));
    assert!(
        adapter
            .backend
            .last_goals()
            .iter()
            .all(|&(_, raw)| raw == 512)
    );
    assert!((0..JOINTS).all(|index| !adapter.backend.torqued(0, id(index))));
    Ok(())
}

#[test]
fn a_read_only_actuator_left_powered_comes_up_off() -> Result<(), DynamixelError> {
    let mut adapter = rig(RO_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.already_on(0, id(LAST));

    adapter.configure()?;
    assert!(adapter.backend.torqued(0, id(LAST)));
    adapter.torque_on()?;

    assert!(!adapter.backend.torqued(0, id(LAST)));
    assert!(forced(&adapter.backend).eq([id(LAST)]));
    assert!(!torque_asked(&adapter.backend, 0, id(LAST), true));
    Ok(())
}

#[test]
fn a_powered_actuator_left_on_still_comes_up() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.already_on(0, id(0));

    adapter.configure()?;
    adapter.torque_on()?;
    assert!((0..JOINTS).all(|index| adapter.backend.torqued(0, id(index))));
    Ok(())
}

// ── US1: the preset hardware half (T026) ────────────────────────────────────

#[test]
fn table_of_xm_bench_passes_check_with_four_actuators() {
    let table = XmBench::HARDWARE.check();

    assert_eq!(<XmBench as Preset>::N, 4);
    assert!(table.actuators.iter().map(|actuator| actuator.id).eq(1..=4));
    assert!(table.actuators.iter().all(|actuator| {
        (
            actuator.bus,
            actuator.model,
            actuator.mode,
            actuator.torque,
            actuator.zero,
        ) == (
            0,
            Model::Xm430W210,
            Mode::Position,
            Torque::On,
            Zero::DEFAULT,
        )
    }));
    let bus = table.buses[0];
    assert_eq!(
        (bus.label, bus.baud, bus.read_every),
        ("bus0", 1_000_000, None)
    );
}

#[test]
fn table_of_rx28_chain_passes_check_with_ten_actuators() {
    let table = Rx28Chain::HARDWARE.check();

    assert_eq!(<Rx28Chain as Preset>::N, 10);
    assert!(
        table
            .actuators
            .iter()
            .map(|actuator| actuator.id)
            .eq(1..=10)
    );
    assert!(table.actuators.iter().all(|actuator| {
        (
            actuator.bus,
            actuator.model,
            actuator.mode,
            actuator.torque,
            actuator.zero,
        ) == (0, Model::Rx28, Mode::Position, Torque::On, Zero::Center)
    }));
    let bus = table.buses[0];
    assert_eq!(
        (bus.label, bus.baud, bus.read_every),
        ("bus0", 1_000_000, NonZeroU32::new(1))
    );
    assert_eq!(Model::Rx28.protocol(), Protocol::One);
}

#[test]
fn table_bus_without_a_cadence_reads_every_default_read_every_ticks() -> Result<(), DynamixelError>
{
    let mut adapter = rig(SILENT_TABLE);
    let layout = powered(&mut adapter)?;
    assert_eq!(layout.read_every, [DEFAULT_READ_EVERY; JOINTS]);

    let every = u64::from(DEFAULT_READ_EVERY.get());
    for call in 0..3 * every {
        let (refreshed, faults) = read(&mut adapter, &mut Slot::filled(0.0));
        assert_eq!(faults, Faults::default(), "call {call}");
        let due = call.is_multiple_of(every);
        assert_eq!(
            refreshed,
            if due { Mask::ALL } else { Mask::NONE },
            "call {call}"
        );
    }
    assert_eq!(
        adapter.backend.count(Call::SyncReadState),
        3 * Scripted::STATE_READS.len()
    );
    Ok(())
}

#[test]
fn table_of_xm_bench_writes_once_per_bus_and_series() -> Result<(), DynamixelError> {
    // XmBench has four actuators: a core built for another count cannot rig it.
    let Ok(actuators) = <[Actuator; JOINTS]>::try_from(XmBench::HARDWARE.actuators.as_slice())
    else {
        return Ok(());
    };
    let mut adapter = rig(Table {
        buses: XmBench::HARDWARE.buses,
        actuators,
    });
    let layout = powered(&mut adapter)?;
    assert_eq!(layout.group, [Group(0); JOINTS]);

    let faults = write(&mut adapter, &layout, &Desired::all_q());

    assert_eq!(faults, Faults::default());
    let writes = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::SyncWriteGoal);
    let sent = writes.map(|record| (record.port, record.model, record.count));
    assert!(sent.eq([(0, Some(Model::Xm430W210), JOINTS)]));
    Ok(())
}

// ── US1: bus plugins bound by label, ports resolved at Init (T033) ─────────

/// `table` refuses `plugins` with `error`, the backend untouched and no bus
/// bound.
fn refuses<const NB: usize>(table: Table<NB, JOINTS>, plugins: &[Binding], error: DynamixelError) {
    let mut adapter = rig(table);
    assert_eq!(adapter.attach(plugins), Err(error));
    assert!(adapter.backend.calls().is_empty());
    assert_eq!(adapter.ports, [None; NB]);
    assert_eq!(
        Dynamixel::bind(Scripted::new(), table.in_order(), plugins).err(),
        Some(error)
    );
}

/// `path` for a `Port::ByDev`, which takes a `&'static str`.
fn leaked(path: &Path) -> &'static str {
    PathBuf::leak(path.to_path_buf()).to_str().unwrap()
}

#[test]
fn binding_an_unknown_label_is_refused_before_io() {
    refuses(
        MIX_TABLE,
        &[MIX_PLUGINS[0], plugin(WRIST, Some("elbow"))],
        DynamixelError::UnknownBus {
            label: "elbow",
            port: Port::ById(WRIST),
        },
    );
}

#[test]
fn binding_leaves_no_bus_unbound() {
    refuses(
        MIX_TABLE,
        &MIX_PLUGINS[..1],
        DynamixelError::UnboundBus { bus: "wrist" },
    );
    refuses(X_TABLE, &[], DynamixelError::UnboundBus { bus: "x" });
}

#[test]
fn binding_an_extra_plugin_is_refused() {
    refuses(
        X_TABLE,
        &[plugin(ARM, None), plugin(WRIST, None)],
        DynamixelError::ExtraPlugin {
            port: Port::ById(WRIST),
        },
    );
}

#[test]
fn binding_two_buses_needs_a_label_on_every_plugin() {
    refuses(
        MIX_TABLE,
        &[MIX_PLUGINS[0], plugin(WRIST, None)],
        DynamixelError::LabelRequired {
            port: Port::ById(WRIST),
        },
    );
}

#[test]
fn binding_two_plugins_on_one_label_is_refused() {
    refuses(
        MIX_TABLE,
        &[MIX_PLUGINS[0], plugin(WRIST, Some("arm"))],
        DynamixelError::BusBoundTwice {
            bus: "arm",
            port: Port::ById(WRIST),
        },
    );
}

#[test]
fn binding_a_plugin_baud_a_model_rejects_names_the_actuator() {
    let fast = Binding {
        baud: Some(3_000_000),
        ..MIX_PLUGINS[1]
    };
    let error = DynamixelError::BaudRejected {
        actuator: 1,
        bus: "wrist",
        model: Model::Ax12a,
        baud: 3_000_000,
    };

    refuses(MIX_TABLE, &[MIX_PLUGINS[0], fast], error);
    assert_eq!(
        error.to_string(),
        "actuator 1: AX-12A on bus wrist does not accept 3000000 baud, set by its bus plugin"
    );
}

#[test]
fn binding_one_bus_needs_no_label_and_takes_the_plugin_baud() -> Result<(), DynamixelError> {
    let fast = Binding {
        baud: Some(3_000_000),
        ..X_PLUGINS[0]
    };
    let adapter = Dynamixel::bind(Scripted::new(), X_TABLE.in_order(), &[fast])?;
    assert_eq!(adapter.ports, [Some(fast.port)]);
    assert_eq!(adapter.buses[0].baud, 3_000_000);
    assert!(adapter.backend.calls().is_empty());

    let labelled = Binding {
        bus: Some("x"),
        ..X_PLUGINS[0]
    };
    let adapter = Dynamixel::bind(Scripted::new(), X_TABLE.in_order(), &[labelled])?;
    assert_eq!(adapter.buses[0].baud, 1_000_000);
    Ok(())
}

#[test]
fn binding_by_label_ignores_the_plugin_order() -> Result<(), DynamixelError> {
    let adapter = Dynamixel::bind(
        Scripted::new(),
        MIX_TABLE.in_order(),
        &[MIX_PLUGINS[1], MIX_PLUGINS[0]],
    )?;
    assert_eq!(adapter.ports, MIX_PLUGINS.map(|plugin| Some(plugin.port)));
    Ok(())
}

#[test]
fn open_of_an_unbound_bus_is_refused_before_io() {
    let mut adapter = rig(X_TABLE);
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(adapter.open(), Err(DynamixelError::UnboundBus { bus: "x" }));
    assert!(adapter.backend.calls().is_empty());
}

#[test]
fn open_by_id_needs_exactly_one_matching_entry() -> io::Result<()> {
    let roots = Roots::new("by_id_one_entry")?;
    let mut adapter = roots.rig(bound(X_TABLE, &X_PLUGINS));
    assert!(adapter.check(Overrides::default()).is_ok());
    let not_found = Err(DynamixelError::PortNotFound {
        bus: "x",
        port: X_PLUGINS[0].port,
    });

    assert_eq!(adapter.open(), not_found);
    roots.link("XCHAIN01", Path::new(NULL))?;
    symlink(NULL, roots.by_id.join("usb-Other_XCHAIN01-if01-port0"))?;
    assert_eq!(adapter.open(), not_found);
    assert!(adapter.backend.calls().is_empty());

    let Some(pty) = pty() else {
        return Ok(());
    };
    let roots = Roots::new("by_id_opens")?;
    roots.link("XCHAIN01", &pty.slave)?;
    let mut adapter = roots.rig(bound(X_TABLE, &X_PLUGINS));
    assert!(adapter.check(Overrides::default()).is_ok());
    assert_eq!(adapter.open(), Ok(()));
    assert_eq!(adapter.backend.path(0), Some(pty.slave.as_path()));
    Ok(())
}

#[test]
fn open_by_dev_on_a_pty_opens_at_the_plugin_baud() -> io::Result<()> {
    let Some(pty) = pty() else {
        return Ok(());
    };
    let roots = Roots::new("by_dev_opens")?;
    let plugin = Binding {
        port: Port::ByDev(leaked(&pty.slave)),
        baud: Some(3_000_000),
        bus: None,
    };
    let mut adapter = roots.rig(bound(X_TABLE, &[plugin]));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(adapter.open(), Ok(()));
    let kinds = adapter.backend.calls().iter().map(|record| record.call);
    assert!(kinds.eq([Call::OpenPorts]));
    assert_eq!(adapter.backend.baud(0), Some(3_000_000));
    assert_eq!(adapter.backend.path(0), Some(pty.slave.as_path()));
    Ok(())
}

#[test]
fn open_by_dev_of_a_regular_or_missing_file_is_refused_before_io() -> io::Result<()> {
    let roots = Roots::new("by_dev_refused")?;
    let file = Port::ByDev(leaked(&roots.regular_file()?));
    let absent = Port::ByDev("/dev/kmr-absent");
    for (port, error) in [
        (
            file,
            DynamixelError::PortAccess {
                bus: "x",
                port: file,
            },
        ),
        (
            absent,
            DynamixelError::PortNotFound {
                bus: "x",
                port: absent,
            },
        ),
    ] {
        let plugin = Binding {
            port,
            baud: None,
            bus: None,
        };
        let mut adapter = roots.rig(bound(X_TABLE, &[plugin]));
        assert!(adapter.check(Overrides::default()).is_ok());

        assert_eq!(adapter.open(), Err(error));
        assert!(adapter.backend.calls().is_empty());
    }
    Ok(())
}

#[test]
fn open_of_two_buses_on_one_device_is_refused_before_any_flush() -> io::Result<()> {
    let roots = Roots::new("by_dev_shared")?;
    // /dev/null fails a flush: reaching one would name the arm bus.
    roots.link(ARM, Path::new(NULL))?;
    let wrist = Binding {
        port: Port::ByDev(NULL),
        ..MIX_PLUGINS[1]
    };
    let mut adapter = roots.rig(bound(MIX_TABLE, &[MIX_PLUGINS[0], wrist]));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(
        adapter.open(),
        Err(DynamixelError::SharedPort {
            first: "arm",
            bus: "wrist"
        })
    );
    assert!(adapter.backend.calls().is_empty());
    Ok(())
}

// ── T069: a member owning a subset of the robot ─────────────────────────────

/// Engine indices of [`SUBSET`], in table order.
const SUBSET_AT: [usize; 2] = [3, 1];

/// An XM430 on bus `arm` at engine index 3 and an AX-12A on bus `wrist` at
/// engine index 1, both id 1, both buses read every tick.
const SUBSET: Placement<2, 2> = {
    let bus = Bus {
        label: "arm",
        baud: 1_000_000,
        read_every: NonZeroU32::new(1),
    };
    let x = Actuator {
        bus: 0,
        id: 1,
        model: Model::Xm430W210,
        mode: Mode::Position,
        torque: Torque::On,
        zero: Zero::DEFAULT,
    };
    Table {
        buses: [
            bus,
            Bus {
                label: "wrist",
                ..bus
            },
        ],
        actuators: [
            x,
            Actuator {
                bus: 1,
                model: Model::Ax12a,
                ..x
            },
        ],
    }
    .at(SUBSET_AT)
};

/// The engine indices of [`SUBSET`].
fn subset() -> Mask {
    mask(|index| SUBSET_AT.contains(&index))
}

/// Whether every backend call naming an actuator names one of [`SUBSET`]:
/// id 1 on bus 0 or 1.
fn only_subset_called(backend: &Scripted) -> bool {
    backend
        .calls()
        .iter()
        .filter(|record| record.model.is_some())
        .all(|record| record.port < 2 && backend.ids(record).all(|id| id == 1))
}

#[test]
fn placement_maps_each_actuator_to_its_engine_index() {
    let placed = Placed::new(&SUBSET);
    assert_eq!(placed.owned(), subset());
    assert_eq!(placed.get(3).map(|a| a.model), Some(Model::Xm430W210));
    assert_eq!(placed.get(1).map(|a| a.model), Some(Model::Ax12a));
    assert!(placed.get(0).is_none() && placed.get(2).is_none());
    assert!(placed.iter().map(|(index, _)| index).eq([1, 3]));
    assert_eq!(X_TABLE.in_order().indices(), &core::array::from_fn(|i| i));
}

#[test]
fn a_placed_member_claims_its_indices_with_local_groups() -> Result<(), DynamixelError> {
    let mut adapter = rig_placed();
    let claim = adapter.check(Overrides::default())?;

    assert_eq!(claim.owned, subset());
    // First seen in engine order: the wrist AX at 1, then the arm X at 3.
    assert_eq!((claim.group[1], claim.group[3]), (Group(0), Group(1)));
    assert_eq!(claim.read_every[1].get(), 1);
    assert_eq!(claim.read_every[3].get(), 1);
    assert!(claim.read_cost > Duration::ZERO && claim.write_cost > Duration::ZERO);
    Ok(())
}

/// [`SUBSET`] on a fresh scripted backend, no bus bound.
fn rig_placed() -> Dynamixel<Scripted, 2> {
    Dynamixel::new(Scripted::new(), SUBSET)
}

#[test]
fn a_placed_member_brings_up_only_its_actuators() -> Result<(), DynamixelError> {
    let mut adapter = rig_placed();
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    let mut slot = Slot::filled(7.0);

    adapter.zero()?;
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    adapter.read_all(slot.sample())?;
    adapter.torque_on()?;

    assert!(only_subset_called(&adapter.backend));
    assert_eq!(adapter.backend.count(Call::Ping), 2);
    assert_eq!(adapter.backend.count(Call::Torque), 2);
    for index in (0..JOINTS).filter(|&index| !subset().get(index)) {
        assert_eq!(
            (slot.q[index], slot.qd[index], slot.tau[index]),
            (Q(7.0), Qd(7.0), Tau(7.0)),
            "slot {index} is another member's"
        );
    }
    assert!(SUBSET_AT.iter().all(|&index| slot.q[index] != Q(7.0)));
    Ok(())
}

#[test]
fn a_placed_member_reads_and_faults_only_its_slots() -> Result<(), DynamixelError> {
    let mut adapter = rig_placed();
    powered(&mut adapter)?;
    let mut slot = Slot::filled(7.0);

    let (refreshed, faults) = read(&mut adapter, &mut slot);
    assert_eq!((refreshed, faults), (subset(), Faults::default()));
    for index in (0..JOINTS).filter(|&index| !subset().get(index)) {
        assert_eq!(slot.q[index], Q(7.0), "slot {index} is another member's");
    }

    adapter
        .backend
        .fail_sync_read(0)
        .fail_port(Call::ReadPosition, 1, BackendError::Timeout);
    let (refreshed, faults) = read(&mut adapter, &mut Slot::filled(7.0));
    assert_eq!(refreshed, Mask::NONE);
    assert_eq!(faults.stale, subset());
    assert_eq!(faults.bus_error.union(faults.offline), subset());
    assert!(only_subset_called(&adapter.backend));
    Ok(())
}

#[test]
fn a_placed_member_writes_holds_and_cuts_only_its_actuators() -> Result<(), DynamixelError> {
    let mut adapter = rig_placed();
    let claim = powered(&mut adapter)?;
    let desired = Desired::all_q();

    assert_eq!(write(&mut adapter, &claim, &desired), Faults::default());
    let writes = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::SyncWriteGoal);
    let sent = writes.map(|record| (record.port, record.model, record.count));
    assert!(sent.eq([(1, Some(Model::Ax12a), 1), (0, Some(Model::Xm430W210), 1)]));

    // A batch of the whole robot, which the engine never hands a member:
    // the other members' indices are skipped, their faults left alone.
    let layout = layout_of(&claim);
    let batch = Batch::new(&layout, &desired.written);
    let mut faults = Faults::default();
    adapter.write(desired.command(), &batch, &mut faults);
    assert_eq!(faults, Faults::default());

    adapter.hold(desired.command());
    adapter.torque_off();
    adapter.close();
    assert_eq!(adapter.backend.count(Call::ForceTorqueOff), 2);
    assert!(only_subset_called(&adapter.backend));
    Ok(())
}

/// Owns every index [`SUBSET`] leaves and drives nothing; keeps the indices
/// the engine handed its `write` and the calls it got last.
#[derive(Default)]
struct Rest {
    wrote: Mask,
    calls: [Option<&'static str>; 2],
}

impl super::seal::Adapter for Rest {}

impl Hal for Rest {
    type Error = core::convert::Infallible;

    fn check(&mut self, _overrides: Overrides) -> Result<Claim, Self::Error> {
        Ok(Claim {
            owned: Mask::ALL.difference(subset()),
            group: [Group(0); JOINTS],
            read_every: [NonZeroU32::MIN; JOINTS],
            read_cost: Duration::ZERO,
            write_cost: Duration::ZERO,
        })
    }

    fn open(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn zero(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn ping(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn health(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn configure(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read_all(&mut self, _sample: Sample<'_>) -> Result<(), Self::Error> {
        Ok(())
    }

    fn torque_on(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, _sample: Sample<'_>, _faults: &mut Faults) -> Mask {
        Mask::NONE
    }

    fn write(&mut self, _command: Command<'_>, batch: &Batch<'_>, _faults: &mut Faults) {
        for (_, run) in batch.runs() {
            run.iter().for_each(|&index| self.wrote.set(index));
        }
    }

    fn hold(&mut self, _target: Command<'_>) {}

    fn torque_off(&mut self) {
        self.calls = [Some("torque_off"), None];
    }

    fn close(&mut self) {
        self.calls[1] = Some("close");
    }
}

#[test]
fn a_placed_member_runs_beside_another_one_on_its_own_actuators() -> io::Result<()> {
    use kmr_core::{EachTick, HCons, HNil, RobotState, Sensors, Time};

    let (Some(arm), Some(wrist)) = (pty(), pty()) else {
        return Ok(());
    };
    let roots = Roots::new("placed_run")?;
    roots.link(ARM, &arm.slave)?;
    roots.link(WRIST, &wrist.slave)?;
    let plugins = [plugin(ARM, Some("arm")), plugin(WRIST, Some("wrist"))];
    let Ok(adapter) = Dynamixel::bind(Scripted::new(), SUBSET, &plugins) else {
        panic!("both buses are bound");
    };
    let mut adapter = roots.rig(adapter);
    let mut rest = Rest::default();

    let control = |time: &Time, state: &mut RobotState, _: &Sensors| {
        state.set_all([Q(0.0); JOINTS]);
        if time.tick() == 2 {
            kmr_core::shutdown!("test: tick 2");
        }
    };
    let hardware = HCons {
        head: &mut adapter,
        tail: HCons {
            head: &mut rest,
            tail: HNil,
        },
    };
    let run = kmr_core::Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_control_fn(EachTick, control)
        .run_on(hardware);

    assert!(run.is_ok(), "{run:?}");
    assert!(only_subset_called(&adapter.backend));
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 2 * 3 + 2);
    assert_eq!(adapter.backend.count(Call::ForceTorqueOff), 2);
    assert_eq!(adapter.backend.count(Call::ClosePorts), 1);
    assert_eq!(rest.wrote, Mask::ALL.difference(subset()));
    assert_eq!(rest.calls, [Some("torque_off"), Some("close")]);
    Ok(())
}
