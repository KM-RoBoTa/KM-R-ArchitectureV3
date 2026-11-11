//! Adapter tests on the scripted backend, without hardware.
//!
//! The adapter is driven through [`Hal`] in the engine's order. `open` needs
//! a terminal to flush, so the later phases run on an adapter whose scripted
//! ports [`mark_open`] opened; `open` itself is tested on its error paths,
//! and on pseudo-terminals where the system lends them.

use core::cell::Cell;
use core::num::NonZeroU32;
use core::time::Duration;
use core::{fmt, iter};
use std::alloc::{GlobalAlloc, Layout as AllocLayout, System};
use std::ffi::OsStr;
use std::fs;
use std::io;
use std::os::fd::OwnedFd;
use std::os::unix::ffi::OsStrExt;
use std::os::unix::fs::symlink;
use std::panic::{self, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError};

use kmr_core::hal::{
    Batch, Claim, Command, Faults, Group, Hal, HardwareError, Layout, Mask, MemberError, Overrides,
    Sample, Written,
};
use kmr_core::state::{Kd, Kp, Q, Qd, Tau};
use kmr_core::telemetry::Telemetry;
use tracing_subscriber::layer::{Context, Layer, SubscriberExt};

use super::backend::{
    Backend, BackendError, GoalRegister, Health, PortSpec, RamSetting, RawState, Vital, Wire,
};
use super::scripted::{Call, HEALTHY, Record, Scripted};
use super::telemetry::TELEMETRY_EVERY;
use super::{
    Actuator, Bus, DEFAULT_READ_EVERY, Dynamixel, DynamixelError, Gains, HealthLimits, Mode, Model,
    PING_ATTEMPTS, Placed, Placement, Protocol, Settings, TORQUE_OFF_ATTEMPTS, Table, Torque, Zero,
    declared_costs, io as tick, port, units,
};
use crate::binding::{Binding, Port};
use crate::presets::{Rx28Chain, XmBench};

/// Actuators of the robot the scenarios drive: they name actuators 0 to 3.
const N: usize = 4;

/// Firmware id of actuator `index`: ids `1..=N`.
const fn id(index: usize) -> u8 {
    index as u8 + 1
}

/// `N` actuators of `model` on bus 0, powered, in position mode.
const fn chain(model: Model) -> [Actuator; N] {
    let mut actuators = [Actuator {
        bus: 0,
        id: 0,
        model,
        mode: Mode::Position,
        torque: Torque::On,
        zero: Zero::DEFAULT,
    }; N];
    let mut index = 0;
    while index < N {
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
const X_TABLE: Table<1, N> = Table {
    buses: one_bus("x", NonZeroU32::new(1)),
    actuators: chain(Model::Xm430W210),
}
.check();
/// Serial of the USB adapter of [`X_TABLE`].
const X_SERIAL: &str = "XCHAIN01";
const X_PLUGINS: [Binding; 1] = [plugin(X_SERIAL, None)];

/// Protocol 2 chain of MX-64(2.0), read every tick: dxl_rs_v2 gives its
/// limits units, so its typed limit setters take them (not the XM430-W210's).
const MX2_TABLE: Table<1, N> = Table {
    buses: one_bus("mx", NonZeroU32::new(1)),
    actuators: chain(Model::Mx64V2),
}
.check();

/// Protocol 1 chain of AX-12A, read every tick.
const AX_TABLE: Table<1, N> = Table {
    buses: one_bus("ax", NonZeroU32::new(1)),
    actuators: chain(Model::Ax12a),
}
.check();

/// Protocol 1 chain of RX-28, read every tick.
const RX_TABLE: Table<1, N> = Table {
    buses: one_bus("rx", NonZeroU32::new(1)),
    actuators: chain(Model::Rx28),
}
.check();

/// Serials of the `arm` and `wrist` USB adapters.
const ARM: &str = "ARM00001";
const WRIST: &str = "WRIST001";

/// Even actuators: X on bus `arm`, read every tick. Odd ones: AX-12A on bus
/// `wrist`, read every other tick.
const MIX_TABLE: Table<2, N> = {
    let mut actuators = chain(Model::Xm430W210);
    let mut index = 1;
    while index < N {
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

/// AX-12A on bus 0 in the first [`SNAKE_AX`] slots, RX-28 on bus 1 after them.
const SNAKE_AX: usize = N / 2;
/// Two Protocol 1 snakes at 1 Mbit/s, read every tick, both numbered from id 1.
const SNAKES_TABLE: Table<2, N> = {
    let mut actuators = chain(Model::Ax12a);
    let mut index = SNAKE_AX;
    while index < N {
        actuators[index].bus = 1;
        actuators[index].id = id(index - SNAKE_AX);
        actuators[index].model = Model::Rx28;
        index += 1;
    }
    Table {
        buses: [
            Bus {
                label: "ax",
                baud: 1_000_000,
                read_every: NonZeroU32::new(1),
            },
            Bus {
                label: "rx",
                baud: 1_000_000,
                read_every: NonZeroU32::new(1),
            },
        ],
        actuators,
    }
    .check()
};

/// The read-only actuator of `RO_TABLE`.
const LAST: usize = N - 1;
/// The Protocol 2 chain with its last actuator read-only.
const RO_TABLE: Table<1, N> = {
    let mut table = X_TABLE;
    table.actuators[LAST].torque = Torque::ReadOnly;
    table.check()
};

/// The Protocol 2 chain with no cadence on its bus.
const SILENT_TABLE: Table<1, N> = Table {
    buses: one_bus("silent", None),
    ..X_TABLE
}
.check();

/// Five buses, one more than the backends open.
const FIVE_TABLE: Table<5, N> = {
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
fn rig<const NB: usize, const M: usize>(table: Table<NB, M>) -> Dynamixel<Scripted, NB, N> {
    Dynamixel::<_, _, N>::new(Scripted::new(), table.in_order())
}

/// An adapter on a fresh scripted backend owning `0..M` in table order,
/// `plugins` bound to its buses.
fn bound<const NB: usize, const M: usize>(
    table: Table<NB, M>,
    plugins: &[Binding],
) -> Dynamixel<Scripted, NB, N> {
    Dynamixel::<_, _, N>::bind(Scripted::new(), table.in_order(), plugins).unwrap()
}

/// `table` with every bus read every `ticks` ticks.
fn every<const NB: usize>(mut table: Table<NB, N>, ticks: u32) -> Table<NB, N> {
    for bus in &mut table.buses {
        bus.read_every = NonZeroU32::new(ticks);
    }
    table
}

/// The newest history slot, as the engine hands it to `read`.
struct Slot {
    q: [Q; N],
    qd: [Qd; N],
    tau: [Tau; N],
    kp: [Kp; N],
    kd: [Kd; N],
}

impl Slot {
    /// Every field at `value`, so an untouched entry is recognizable.
    fn filled(value: f32) -> Self {
        Self {
            q: [Q(value); N],
            qd: [Qd(value); N],
            tau: [Tau(value); N],
            kp: [Kp(value); N],
            kd: [Kd(value); N],
        }
    }

    /// The slot as a `Sample`.
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

/// This tick's desired state and its write masks.
struct Desired {
    q: [Q; N],
    qd: [Qd; N],
    tau: [Tau; N],
    kp: [Kp; N],
    kd: [Kd; N],
    written: Written<N>,
}

impl Desired {
    /// Nothing written yet, every target at zero.
    fn new() -> Self {
        Self {
            q: [Q(0.0); N],
            qd: [Qd(0.0); N],
            tau: [Tau(0.0); N],
            kp: [Kp(0.0); N],
            kd: [Kd(0.0); N],
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
        (0..N).fold(Self::new(), |desired, index| desired.q(index, Q(0.0)))
    }

    /// Writes [`ramp`] to every actuator.
    fn ramp() -> Self {
        let q = ramp();
        (0..N).fold(Self::new(), |desired, index| desired.q(index, q[index]))
    }

    /// The desired state as a `Command`.
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

/// The layout the engine builds for a robot whose only member made `claim`.
pub(super) fn layout_of(claim: &Claim<N>) -> Layout<N> {
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
    adapter: &mut Dynamixel<Scripted, NB, N>,
    claim: &Claim<N>,
    desired: &Desired,
) -> Faults<N> {
    let mut faults = Faults::default();
    let mut written = desired.written;
    written.q = written.q.intersection(claim.owned);
    written.qd = written.qd.intersection(claim.owned);
    written.tau = written.tau.intersection(claim.owned);
    written.kp = written.kp.intersection(claim.owned);
    written.kd = written.kd.intersection(claim.owned);
    let layout = layout_of(claim);
    let batch = Batch::new(&layout, &written);
    let command = Command::new(
        &desired.q,
        &desired.qd,
        &desired.tau,
        &desired.kp,
        &desired.kd,
        &written,
    );
    adapter.write(command, &batch, &mut faults);
    faults
}

/// One `read` into `slot`, with its faults.
fn read<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    slot: &mut Slot,
) -> (Mask<N>, Faults<N>) {
    let mut faults = Faults::default();
    let refreshed = adapter.read(slot.sample(), &mut faults);
    (refreshed, faults)
}

/// Opens the scripted ports and marks the adapter open, as a successful
/// `open` would; the backend log is cleared afterwards.
fn mark_open<const NB: usize>(adapter: &mut Dynamixel<Scripted, NB, N>) {
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
    adapter: &mut Dynamixel<Scripted, NB, N>,
) -> Result<Claim<N>, DynamixelError> {
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
fn mask(select: impl Fn(usize) -> bool) -> Mask<N> {
    let mut mask = Mask::NONE;
    (0..N)
        .filter(|&index| select(index))
        .for_each(|index| mask.set(index));
    mask
}

/// `units::position_to_si` of an in-range `raw`, 0 rad at the center.
fn si(model: Model, raw: i32) -> Q {
    units::position_to_si(&centered(model), raw).unwrap()
}

/// A powered position actuator of `model`, 0 rad at its center.
const fn centered(model: Model) -> Actuator {
    Actuator {
        bus: 0,
        id: 1,
        model,
        mode: Mode::Position,
        torque: Torque::On,
        zero: Zero::Center,
    }
}

/// Every actuator of `adapter` torque-on before bring-up, as an earlier run
/// can leave it.
fn left_on<const NB: usize>(adapter: &mut Dynamixel<Scripted, NB, N>) {
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
    (0..N).flat_map(move |index| iter::repeat_n(id(index), tries(index)))
}

/// Message of the warning `torque_off` logs per actuator it could not cut.
const OFF_FAILED: &str = "torque off failed";

/// Warnings whose message holds `needle`, logged on this thread by `run`.
pub(super) fn warnings(needle: &'static str, run: impl FnOnce()) -> usize {
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
#[ignore = "3 partial sync reads per group: dxl_rs_v2 has no partial multi-field read (T050, T051)"]
fn protocol_2_chain_is_one_read_and_one_write_per_tick() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;
    assert_eq!(layout.group, [Group(0); N]);

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
    assert_eq!(adapter.backend.last_goals().len(), N);
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
    assert_eq!(adapter.backend.count(Call::ReadPosition), N);
    assert_eq!(adapter.backend.count(Call::SyncReadState), 0);
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
    let ids: [u8; N] = core::array::from_fn(id);
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
    let wrist = (0..N).filter(|&index| on_wrist(index)).count();

    for index in 0..N {
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
        (0, Some(Model::Xm430W210), N - wrist),
        (1, Some(Model::Ax12a), wrist),
    ]));

    let (refreshed, _) = read(&mut adapter, &mut Slot::filled(0.0));
    assert_eq!(refreshed, mask(|index| !on_wrist(index)));
    assert_eq!(adapter.backend.count(Call::ReadPosition), wrist);
    Ok(())
}

#[test]
fn two_protocol_one_snakes_write_one_group_per_bus() -> Result<(), DynamixelError> {
    let mut adapter = rig(SNAKES_TABLE);
    let layout = powered(&mut adapter)?;

    let faults = write(&mut adapter, &layout, &Desired::all_q());
    assert_eq!(faults, Faults::default());
    let writes = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::SyncWriteGoal);
    let ports = writes.map(|record| (record.port, record.model, record.count));
    assert!(ports.eq([
        (0, Some(Model::Ax12a), SNAKE_AX),
        (1, Some(Model::Rx28), N - SNAKE_AX),
    ]));

    let (refreshed, faults) = read(&mut adapter, &mut Slot::filled(0.0));
    assert_eq!(faults, Faults::default());
    assert_eq!(refreshed, Mask::ALL);
    assert_eq!(adapter.backend.count(Call::ReadPosition), N);
    assert_eq!(adapter.backend.count(Call::SyncReadState), 0);
    let reads = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::ReadPosition)
        .map(|record| (record.port, record.id));
    let expected = (0..N).map(|index| {
        let rx = index >= SNAKE_AX;
        (usize::from(rx), id(index - if rx { SNAKE_AX } else { 0 }))
    });
    assert!(reads.eq(expected));
    Ok(())
}

#[test]
fn cadence_of_three_reads_on_calls_0_3_6_only() -> Result<(), DynamixelError> {
    let mut adapter = rig(every(X_TABLE, 3));
    let layout = powered(&mut adapter)?;
    assert_eq!(layout.read_every.map(NonZeroU32::get), [3; N]);

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
    let desired = (0..N)
        .filter(|&index| chosen(index))
        .fold(Desired::new(), |desired, index| desired.q(index, Q(0.0)));

    let faults = write(&mut adapter, &layout, &desired);

    assert_eq!(faults, Faults::default());
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
    let sent = (0..N)
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
    fn rig<const NB: usize>(
        &self,
        adapter: Dynamixel<Scripted, NB, N>,
    ) -> Dynamixel<Scripted, NB, N> {
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
    assert_eq!(adapter.backend.count(Call::ForceTorqueOff), N);
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
    assert_eq!(adapter.backend.count(Call::Declare), 1);
    assert_eq!(adapter.backend.count(Call::ReadHealth), N);

    adapter.torque_on()?;
    assert_eq!(adapter.backend.count(Call::Torque), N);
    assert!((0..N).all(|index| adapter.backend.torqued(0, id(index))));
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
    assert_eq!(adapter.backend.count(Call::Declare), 1 + PING_ATTEMPTS);
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
            let actuator = &MIX_TABLE.actuators[index];
            assert_eq!(record.port, actuator.bus);
            assert_eq!(Some(raw), units::position_to_raw(actuator, last.q[index]));
            held += 1;
        }
    }
    assert_eq!(held, N);
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
        .chain([Call::ForceTorqueOff; N])
        .chain([Call::ClosePorts]);
    assert!(kinds.eq(expected));
    assert!((0..N).all(|index| !adapter.backend.torqued(0, id(index))));
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
        kinds.eq([Call::ForceTorqueOff; N]
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
    for index in (0..N).filter(|&index| deaf(index)) {
        adapter
            .backend
            .fail(Call::ForceTorqueOff, 0, id(index), BackendError::Timeout);
    }

    let warned = warnings(OFF_FAILED, || adapter.torque_off());

    let tries = |index| if deaf(index) { TORQUE_OFF_ATTEMPTS } else { 1 };
    assert!(forced(&adapter.backend).eq(attempted(tries)));
    assert!((0..N).all(|index| adapter.backend.torqued(0, id(index)) == deaf(index)));
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
    assert!((0..N).all(|index| !adapter.backend.torqued(0, id(index))));
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

    assert!((0..N).all(|index| !adapter.backend.torqued(0, id(index))));
    assert!(forced(&adapter.backend).eq((0..N).map(id)));
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

    assert!((0..N).all(|index| !adapter.backend.torqued(0, id(index))));
    assert!(forced(&adapter.backend).eq((0..N).map(id)));
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

    assert_eq!(adapter.backend.count(Call::ForceTorqueOff), 2 * N);
    assert_eq!(adapter.backend.count(Call::ClosePorts), 2);
    assert!((0..N).all(|index| !adapter.backend.torqued(0, id(index))));
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
        let raw = units::position_to_raw(&RO_TABLE.actuators[index], last.q[index]);
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
    adapter: &mut Dynamixel<Scripted, NB, N>,
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
    let wrist = (0..N).filter(|index| index % 2 == 1).count();
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
    for index in 0..N {
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
    let mut adapter = Dynamixel::<_, _, N>::new(
        Scripted::new(),
        Placement {
            table,
            at,
            limits: [None; N],
            settings: [Settings::default(); N],
        },
    );
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
    assert!((0..N).all(|index| backend.torqued(0, id(index))));
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
    assert!((0..N).all(|index| !adapter.backend.torqued(0, id(index))));
    Ok(())
}

#[test]
fn a_failed_start_goal_write_powers_nothing() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    configured(&mut adapter)?;
    adapter.backend.fail_port(
        Call::SyncWriteStartGoal,
        1,
        BackendError::Io(Wire::PortTimeout),
    );

    assert_eq!(
        adapter.torque_on(),
        Err(DynamixelError::Backend {
            bus: "wrist",
            source: BackendError::Io(Wire::PortTimeout)
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
    for index in 0..N {
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
    assert_eq!(slot.q, [Q(9.0); N]);
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
    assert_eq!(adapter.backend.count(Call::ReadPosition), N);
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
    assert_eq!(adapter.backend.last_goals().len(), N);
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
        assert!(sent.eq((3..N).map(id)));
    }
    Ok(())
}

#[test]
fn a_raw_zero_moves_0_rad_for_reads_writes_and_the_hold() -> Result<(), DynamixelError> {
    let mut table = X_TABLE;
    table.actuators[0].zero = Zero::Raw(1000);
    let mut adapter = rig(table.check());
    let layout = powered(&mut adapter)?;
    adapter
        .backend
        .position(0, id(0), 1000)
        .position(0, id(1), 1000);

    let mut slot = Slot::filled(9.0);
    read(&mut adapter, &mut slot);
    assert_eq!(slot.q[0], Q(0.0));
    assert_eq!(slot.q[1], si(Model::Xm430W210, 1000));

    write(
        &mut adapter,
        &layout,
        &Desired::new().q(0, Q(0.0)).q(1, Q(0.0)),
    );
    assert_eq!(
        adapter.backend.last_goals(),
        &[(id(0), 1000), (id(1), 2048)]
    );

    adapter.hold(Desired::new().command());
    assert_eq!(adapter.backend.last_goals().first(), Some(&(id(0), 1000)));
    Ok(())
}

#[test]
fn an_infinite_target_is_rejected_never_sent() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;
    let desired = Desired::all_q()
        .q(0, Q(f32::INFINITY))
        .q(1, Q(f32::NEG_INFINITY));

    let faults = write(&mut adapter, &layout, &desired);

    assert_eq!(faults.rejected, mask(|index| index < 2));
    assert_eq!(faults.bus_error, Mask::NONE);
    let sent = adapter.backend.last_goals().iter().map(|&(id, _)| id);
    assert!(sent.eq((2..N).map(id)));
    Ok(())
}

#[test]
fn a_tau_write_on_a_position_actuator_sends_nothing_and_is_rejected() -> Result<(), DynamixelError>
{
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;
    let mut desired = Desired::new();
    desired.tau[2] = Tau(0.3);
    desired.written.tau.set(2);

    let faults = write(&mut adapter, &layout, &desired);

    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 0);
    assert_eq!(faults.rejected, mask(|index| index == 2));
    assert_eq!(faults.bus_error, Mask::NONE);
    Ok(())
}

#[test]
fn a_kp_or_kd_write_alone_sends_nothing_and_is_no_fault() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = powered(&mut adapter)?;
    let mut desired = Desired::new().q(0, Q(0.0));
    for index in [0, 1] {
        desired.kp[index] = Kp(150.0);
        desired.written.kp.set(index);
    }
    desired.kd[2] = Kd(1.0);
    desired.written.kd.set(2);

    let faults = write(&mut adapter, &layout, &desired);

    assert_eq!(faults, Faults::default());
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
    assert_eq!(adapter.backend.last_goals(), &[(id(0), 2048)]);
    Ok(())
}

#[test]
fn check_warns_once_per_protocol_1_actuator_that_its_rates_are_unread() {
    let mut adapter = rig(MIX_TABLE);
    let mut claim = None;

    let warned = warnings("Qd and Tau are never read", || {
        claim = adapter.check(Overrides::default()).ok();
    });

    assert!(claim.is_some());
    assert_eq!(warned, N / 2);
    assert!(adapter.backend.calls().is_empty());
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
    let busy = BackendError::PortOpen(io::ErrorKind::ResourceBusy);
    adapter.backend.fail_open_port(1, busy);

    assert_eq!(
        adapter.open(),
        Err(DynamixelError::PortOpen {
            bus: "wrist",
            port: Port::ById(WRIST),
            source: busy
        })
    );

    let too_many = BackendError::TooManyPorts { ports: 2, max: 1 };
    adapter.backend.fail_open(too_many);
    let error = adapter.open();
    assert_eq!(error, Err(DynamixelError::Sdk { source: too_many }));
    let message = error.err().map(|error| error.to_string());
    assert_eq!(
        message.as_deref(),
        Some(
            "the Dynamixel SDK could not be brought up: 2 ports given, the backend opens at most 1"
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
    assert_eq!(wrist, N / 2);

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
    let unread = tick::unread_rates(&Placed::<N>::new(&MIX_TABLE.in_order()));
    assert_eq!(unread, mask(|index| index % 2 == 1));
    assert_eq!(
        tick::unread_rates(&Placed::<N>::new(&X_TABLE.in_order())),
        Mask::NONE
    );
    assert_eq!(
        tick::unread_rates(&Placed::<N>::new(&AX_TABLE.in_order())),
        Mask::ALL
    );
}

#[test]
fn read_cost_counts_every_sync_read_of_the_backend() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    let layout = adapter.check(Overrides::default())?;

    // Current, velocity and position: one 14-byte instruction each, then per
    // member its id and a status of 11 bytes around the data.
    let members = N as u64;
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
    assert!(on(Call::ReadSetting, 0));
    let x = (0..N).filter(|index| index % 2 == 0).count();
    assert_eq!(adapter.backend.count(Call::ReadSetting), 2 * x);
    assert_eq!(adapter.backend.count(Call::ReadAngleLimits), N);
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
fn limited() -> Dynamixel<Scripted, 1, N> {
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
    last.q = [si(Model::Rx28, 200); N];

    adapter.hold(last.command());

    let held = adapter.backend.last_goals().iter().map(|&(id, _)| id);
    assert!(held.eq((0..N).filter(|&index| index != 1).map(id)));
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
            "actuator 1: id 2 on bus rx: present position 200 outside the position limits 300–700"
        )
    );
    assert_eq!(adapter.backend.count(Call::SyncWriteStartGoal), 0);
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    Ok(())
}

#[test]
fn rx28_chain_runs_every_step_on_protocol_1() -> Result<(), DynamixelError> {
    let mut adapter = rig(RX_TABLE);
    for index in 0..N {
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
        (Call::Declare, 1),
        (Call::ReadHealth, N),
        (Call::ReadAngleLimits, N),
        (Call::ReadSetting, 0),
        (Call::ReadPosition, 3 * N),
        (Call::SyncReadState, 0),
        (Call::SyncWriteStartGoal, 1),
        (Call::Torque, N),
        (Call::SyncWriteGoal, 2),
        (Call::ForceTorqueOff, N),
        (Call::ClosePorts, 1),
    ];
    for (call, count) in counts {
        assert_eq!(adapter.backend.count(call), count, "{call:?}");
    }
    let start = adapter.backend.last(Call::SyncWriteStartGoal);
    let start = start.map(|record| adapter.backend.goals(record));
    let expected: [(u8, u32); N] =
        core::array::from_fn(|index| (id(index), 400 + 10 * index as u32));
    assert_eq!(start, Some(&expected[..]));
    assert!(
        adapter
            .backend
            .last_goals()
            .iter()
            .all(|&(_, raw)| raw == 512)
    );
    assert!((0..N).all(|index| !adapter.backend.torqued(0, id(index))));
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
    assert!((0..N).all(|index| adapter.backend.torqued(0, id(index))));
    Ok(())
}

// ── US1: the preset hardware half (T026) ────────────────────────────────────

#[test]
fn table_of_xm_bench_passes_check_with_four_actuators() {
    let table = XmBench::HARDWARE.check();

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
fn table_preset_costs_are_the_ones_check_declares() -> Result<(), DynamixelError> {
    fn declared<const M: usize>(claim: Claim<M>) -> (Duration, Duration) {
        (claim.read_cost, claim.write_cost)
    }
    let mut bench = Dynamixel::<_, _, N>::new(Scripted::new(), XmBench::HARDWARE.in_order());
    assert_eq!(
        declared(bench.check(Overrides::default())?),
        (XmBench::READ_COST, XmBench::WRITE_COST)
    );
    // Three sync reads of four XM: 70, 78 and 78 bytes; one 34-byte write.
    assert_eq!(XmBench::READ_COST, Duration::from_micros(2_260));
    assert_eq!(XmBench::WRITE_COST, Duration::from_micros(340));

    let mut chain = Dynamixel::<_, _, 10>::new(Scripted::new(), Rx28Chain::HARDWARE.in_order());
    assert_eq!(
        declared(chain.check(Overrides::default())?),
        (Rx28Chain::READ_COST, Rx28Chain::WRITE_COST)
    );
    assert_eq!(
        declared(rig(MIX_TABLE).check(Overrides::default())?),
        declared_costs(&MIX_TABLE)
    );
    Ok(())
}

#[test]
fn table_bus_without_a_cadence_reads_every_default_read_every_ticks() -> Result<(), DynamixelError>
{
    let mut adapter = rig(SILENT_TABLE);
    let layout = powered(&mut adapter)?;
    assert_eq!(layout.read_every, [DEFAULT_READ_EVERY; N]);

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
    let Ok(actuators) = <[Actuator; N]>::try_from(XmBench::HARDWARE.actuators.as_slice()) else {
        return Ok(());
    };
    let mut adapter = rig(Table {
        buses: XmBench::HARDWARE.buses,
        actuators,
    });
    let layout = powered(&mut adapter)?;
    assert_eq!(layout.group, [Group(0); N]);

    let faults = write(&mut adapter, &layout, &Desired::all_q());

    assert_eq!(faults, Faults::default());
    let writes = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::SyncWriteGoal);
    let sent = writes.map(|record| (record.port, record.model, record.count));
    assert!(sent.eq([(0, Some(Model::Xm430W210), N)]));
    Ok(())
}

// ── US1: bus plugins bound by label, ports resolved at Init (T033) ─────────

/// `table` refuses `plugins` with `error`, the backend untouched and no bus
/// bound.
fn refuses<const NB: usize>(table: Table<NB, N>, plugins: &[Binding], error: DynamixelError) {
    let mut adapter = rig(table);
    assert_eq!(adapter.attach(plugins), Err(error));
    assert!(adapter.backend.calls().is_empty());
    assert_eq!(adapter.ports, [None; NB]);
    assert_eq!(
        Dynamixel::<_, _, N>::bind(Scripted::new(), table.in_order(), plugins).err(),
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
    let adapter = Dynamixel::<_, _, N>::bind(Scripted::new(), X_TABLE.in_order(), &[fast])?;
    assert_eq!(adapter.ports, [Some(fast.port)]);
    assert_eq!(adapter.buses[0].baud, 3_000_000);
    assert!(adapter.backend.calls().is_empty());

    let labelled = Binding {
        bus: Some("x"),
        ..X_PLUGINS[0]
    };
    let adapter = Dynamixel::<_, _, N>::bind(Scripted::new(), X_TABLE.in_order(), &[labelled])?;
    assert_eq!(adapter.buses[0].baud, 1_000_000);
    Ok(())
}

#[test]
fn binding_by_label_ignores_the_plugin_order() -> Result<(), DynamixelError> {
    let adapter = Dynamixel::<_, _, N>::bind(
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
fn subset() -> Mask<N> {
    mask(|index| SUBSET_AT.contains(&index))
}

/// A distinct target for every engine index: `0.1 * index` rad, so a goal
/// shows the index it came from.
fn ramp() -> [Q; N] {
    core::array::from_fn(|index| Q(0.1 * index as f32))
}

/// Whether the backend saw only [`SUBSET`]'s own indices: every call naming
/// an actuator names one of its table as declared (bus, model and id), and
/// every goal written to it is the one [`ramp`] gives its engine index.
fn only_subset_called(backend: &Scripted) -> bool {
    let q = ramp();
    let owner = |record: &Record, id: u8| {
        SUBSET
            .table()
            .actuators
            .iter()
            .zip(SUBSET_AT)
            .find(|(actuator, _)| {
                (actuator.bus, Some(actuator.model), actuator.id) == (record.port, record.model, id)
            })
    };
    backend
        .calls()
        .iter()
        .filter(|record| record.model.is_some())
        .all(|record| {
            let named = backend.ids(record).all(|id| owner(record, id).is_some());
            let goals = record.call != Call::SyncWriteGoal
                || backend.goals(record).iter().all(|&(id, raw)| {
                    owner(record, id).is_some_and(|(actuator, index)| {
                        q.get(index).and_then(|&q| tick::position_goal(actuator, q)) == Some(raw)
                    })
                });
            named && goals
        })
}

#[test]
fn only_subset_called_refuses_another_index() {
    let mut backend = Scripted::new();
    let ramp = ramp();
    let goal = |index: usize| tick::position_goal(&SUBSET.table().actuators[0], ramp[index]);
    let (Some(own), Some(other)) = (goal(3), goal(2)) else {
        panic!("the arm XM takes both targets");
    };

    // The fake logs a call before it refuses it: unpowered is fine here.
    let _ = backend.sync_write_goal(0, Model::Xm430W210, &[(1, own)]);
    assert!(only_subset_called(&backend));
    let _ = backend.sync_write_goal(0, Model::Xm430W210, &[(1, other)]);
    assert!(!only_subset_called(&backend), "index 2's goal on index 3");

    backend.clear_log();
    let _ = backend.declare(1, &[(1, Model::Xm430W210)]);
    assert!(!only_subset_called(&backend), "the wrist bus has no XM");
    backend.clear_log();
    let _ = backend.declare(0, &[(2, Model::Xm430W210)]);
    assert!(!only_subset_called(&backend), "no id 2 on the arm bus");
}

#[test]
fn placement_maps_each_actuator_to_its_engine_index() {
    let placed = Placed::<N>::new(&SUBSET);
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
fn rig_placed() -> Dynamixel<Scripted, 2, N> {
    Dynamixel::<_, _, N>::new(Scripted::new(), SUBSET)
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
    assert_eq!(adapter.backend.count(Call::Declare), 2);
    assert_eq!(adapter.backend.count(Call::Torque), 2);
    for index in (0..N).filter(|&index| !subset().get(index)) {
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
    for index in (0..N).filter(|&index| !subset().get(index)) {
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
    let desired = Desired::ramp();

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
    wrote: Mask<N>,
    calls: [Option<&'static str>; 2],
}

impl super::seal::Adapter for Rest {}

impl Hal<N> for Rest {
    type Error = core::convert::Infallible;

    fn check(&mut self, _overrides: Overrides) -> Result<Claim<N>, Self::Error> {
        Ok(Claim {
            owned: Mask::ALL.difference(subset()),
            group: [Group(0); N],
            read_every: [NonZeroU32::MIN; N],
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

    fn read_all(&mut self, _sample: Sample<'_, N>) -> Result<(), Self::Error> {
        Ok(())
    }

    fn torque_on(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, _sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
        Mask::NONE
    }

    fn write(&mut self, _command: Command<'_, N>, batch: &Batch<'_, N>, _faults: &mut Faults<N>) {
        for (_, run) in batch.runs() {
            run.iter().for_each(|&index| self.wrote.set(index));
        }
    }

    fn hold(&mut self, _target: Command<'_, N>) {}

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

    let _engine = engine();
    let (Some(arm), Some(wrist)) = (pty(), pty()) else {
        return Ok(());
    };
    let roots = Roots::new("placed_run")?;
    roots.link(ARM, &arm.slave)?;
    roots.link(WRIST, &wrist.slave)?;
    let plugins = [plugin(ARM, Some("arm")), plugin(WRIST, Some("wrist"))];
    let Ok(adapter) = Dynamixel::<_, _, N>::bind(Scripted::new(), SUBSET, &plugins) else {
        panic!("both buses are bound");
    };
    let mut adapter = roots.rig(adapter);
    let mut rest = Rest::default();

    let control = |time: &Time, state: &mut RobotState<N>, _: &Sensors| {
        state.set_all(ramp());
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
        .set_hold(Duration::ZERO)
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

// ── US2: through the engine, `run_on` on a pseudo-terminal (T041) ───────────

/// Held by every test that runs the engine: its signal bus is one per
/// process, so two runs at once would stop each other.
static ENGINE: Mutex<()> = Mutex::new(());

/// The engine to this test alone; a test that failed holding it does not
/// fail the next one.
fn engine() -> MutexGuard<'static, ()> {
    ENGINE.lock().unwrap_or_else(PoisonError::into_inner)
}

/// [`X_TABLE`] bound to the pseudo-terminal `pty` by its serial, resolved
/// under `roots`.
fn x_on(roots: &Roots, pty: &Pty) -> io::Result<Dynamixel<Scripted, 1, N>> {
    roots.link(X_SERIAL, &pty.slave)?;
    Ok(roots.rig(bound(X_TABLE, &X_PLUGINS)))
}

/// `adapter` as the robot's only member, kept after the run.
fn alone(
    adapter: &mut Dynamixel<Scripted, 1, N>,
) -> kmr_core::HCons<&mut Dynamixel<Scripted, 1, N>, kmr_core::HNil> {
    kmr_core::HCons {
        head: adapter,
        tail: kmr_core::HNil,
    }
}

/// A robot of 1 ms ticks that holds on a shutdown, running `control` each
/// tick on `adapter` alone.
fn run_alone(
    adapter: &mut Dynamixel<Scripted, 1, N>,
    control: impl Fn(&kmr_core::Time, &mut kmr_core::RobotState<N>, &kmr_core::Sensors),
) -> Result<
    (),
    kmr_core::error::RunError<
        HardwareError<MemberError<DynamixelError, core::convert::Infallible>>,
    >,
> {
    kmr_core::Robot::new()
        .set_dt(Duration::from_millis(1))
        .set_hold(Duration::ZERO)
        .add_control_fn(kmr_core::EachTick, control)
        .run_on(alone(adapter))
}

/// Kinds of the last `count` calls, oldest first.
fn tail(backend: &Scripted, count: usize) -> impl Iterator<Item = Call> + '_ {
    let calls = backend.calls();
    calls[calls.len().saturating_sub(count)..]
        .iter()
        .map(|record| record.call)
}

/// Every exit after `open` ends with one `ForceTorqueOff` per id, then
/// `ClosePorts`, and leaves no id powered.
fn ends_cut_and_closed(backend: &Scripted, forced_ids: impl Iterator<Item = u8>) {
    let off = forced(backend).count();
    let expected = iter::repeat_n(Call::ForceTorqueOff, off).chain([Call::ClosePorts]);
    assert!(tail(backend, off + 1).eq(expected));
    assert!(forced(backend).eq(forced_ids));
    assert_eq!(backend.count(Call::ClosePorts), 1);
    assert!((0..N).all(|index| !backend.torqued(0, id(index))));
}

#[test]
fn engine_shutdown_writes_three_ticks_holds_then_cuts_and_closes() -> io::Result<()> {
    let _engine = engine();
    let Some(pty) = pty() else {
        return Ok(());
    };
    let roots = Roots::new("engine_shutdown")?;
    let mut adapter = x_on(&roots, &pty)?;

    let run = run_alone(&mut adapter, |time, state, _| {
        state.set_all(ramp());
        if time.tick() == 2 {
            kmr_core::shutdown!("test: tick 2");
        }
    });

    assert!(run.is_ok(), "{run:?}");
    let backend = &adapter.backend;
    assert_eq!(backend.count(Call::SyncWriteGoal), 3 + 1);
    let off = N + 1;
    assert_eq!(tail(backend, off + 1).next(), Some(Call::SyncWriteGoal));
    let held = (0..N).map(|index| {
        let raw = tick::position_goal(&X_TABLE.actuators[index], ramp()[index]);
        (id(index), raw)
    });
    let hold = backend.calls()[backend.calls().len() - off - 1];
    let sent = backend
        .goals(&hold)
        .iter()
        .map(|&(id, raw)| (id, Some(raw)));
    assert!(sent.eq(held));
    ends_cut_and_closed(backend, (0..N).map(id));
    Ok(())
}

#[test]
fn engine_emergency_stop_skips_its_tick_write_and_the_hold() -> io::Result<()> {
    let _engine = engine();
    let Some(pty) = pty() else {
        return Ok(());
    };
    let roots = Roots::new("engine_estop")?;
    let mut adapter = x_on(&roots, &pty)?;

    let run = run_alone(&mut adapter, |time, state, _| {
        state.set_all(ramp());
        if time.tick() == 2 {
            kmr_core::emergency_stop!("test: tick 2");
        }
    });

    assert!(run.is_ok(), "{run:?}");
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 2);
    ends_cut_and_closed(&adapter.backend, (0..N).map(id));
    Ok(())
}

#[test]
fn engine_missing_actuator_is_returned_after_torque_off_and_close() -> io::Result<()> {
    let _engine = engine();
    let Some(pty) = pty() else {
        return Ok(());
    };
    let roots = Roots::new("engine_missing")?;
    let mut adapter = x_on(&roots, &pty)?;
    adapter.backend.missing(0, id(2));

    let run = run_alone(&mut adapter, |_, _, _| {});

    let missing = DynamixelError::Missing {
        actuator: 2,
        bus: "x",
        id: id(2),
    };
    assert!(
        matches!(&run, Err(kmr_core::error::RunError::Hal(HardwareError::Member(MemberError::Head { error, .. }))) if *error == missing),
        "{run:?}"
    );
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    let tries = |index: usize| if index == 2 { TORQUE_OFF_ATTEMPTS } else { 1 };
    ends_cut_and_closed(&adapter.backend, attempted(tries));
    Ok(())
}

#[test]
fn engine_controller_panic_cuts_and_closes_then_goes_on() -> io::Result<()> {
    let _engine = engine();
    let Some(pty) = pty() else {
        return Ok(());
    };
    let roots = Roots::new("engine_panic")?;
    let mut adapter = x_on(&roots, &pty)?;

    let unwound = panic::catch_unwind(AssertUnwindSafe(|| {
        run_alone(&mut adapter, |time, state, _| {
            state.set_all(ramp());
            assert!(time.tick() < 1, "test: controller panics at tick 1");
        })
    }));

    assert!(unwound.is_err(), "the panic goes on past the engine");
    assert_eq!(adapter.backend.count(Call::SyncWriteGoal), 1);
    ends_cut_and_closed(&adapter.backend, (0..N).map(id));
    Ok(())
}

// ── T071: MX-64 beside AX-12, Protocol 1 and 2.0 firmware ───────────────────

/// Whether actuator `index` sits on the MX-64 bus of [`ax_beside_mx`].
const fn on_mx(index: usize) -> bool {
    index % 2 == 1
}

/// Even actuators: AX-12A on bus `ax`. Odd ones: `mx` on bus `mx`. Both at
/// 1 Mbit/s, read every tick.
const fn ax_beside_mx(mx: Model) -> Table<2, N> {
    let mut actuators = chain(Model::Ax12a);
    let mut index = 0;
    while index < N {
        if on_mx(index) {
            actuators[index].bus = 1;
            actuators[index].model = mx;
        }
        index += 1;
    }
    Table {
        buses: [
            Bus {
                label: "ax",
                ..one_bus("", NonZeroU32::new(1))[0]
            },
            Bus {
                label: "mx",
                ..one_bus("", NonZeroU32::new(1))[0]
            },
        ],
        actuators,
    }
}

/// An AX-12 bus beside a Protocol 1 MX-64 bus.
const AX_MX_TABLE: Table<2, N> = ax_beside_mx(Model::Mx64).check();
/// An AX-12 bus beside an MX-64(2.0) bus.
const AX_MX2_TABLE: Table<2, N> = ax_beside_mx(Model::Mx64V2).check();

/// Raw present position of actuator `index` in the MX-64 scenarios: past
/// the AX range on the MX-64 bus.
fn mx_scenario_raw(index: usize) -> i32 {
    let step = 10 * i32::try_from(index).unwrap_or(0);
    if on_mx(index) {
        3000 + step
    } else {
        600 + step
    }
}

/// Every step from `check` to `close` on `table`, the MX-64 at their
/// [`mx_scenario_raw`] with their whole angle range; the read slot is
/// returned with the claim.
fn run_ax_beside_mx(
    table: Table<2, N>,
) -> Result<(Dynamixel<Scripted, 2, N>, Claim<N>, Slot), DynamixelError> {
    let mut adapter = rig(table);
    for index in 0..N {
        let bus = usize::from(on_mx(index));
        let state = RawState {
            position: mx_scenario_raw(index),
            velocity: 100,
            current: 100,
        };
        adapter.backend.state(bus, id(index), state);
        if on_mx(index) {
            adapter.backend.angle_limits(bus, id(index), 0, 4095);
        }
    }
    let claim = adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.zero()?;
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    let mut slot = Slot::filled(9.0);
    adapter.read_all(slot.sample())?;
    adapter.torque_on()?;
    let (refreshed, faults) = read(&mut adapter, &mut slot);
    let written = write(&mut adapter, &claim, &Desired::all_q());
    adapter.hold(Desired::all_q().command());
    adapter.torque_off();
    adapter.close();
    assert_eq!(
        (refreshed, faults, written),
        (Mask::ALL, Faults::default(), Faults::default())
    );
    Ok((adapter, claim, slot))
}

/// `(port, model, count)` of each recorded `call`, in call order.
fn sent(backend: &Scripted, call: Call) -> impl Iterator<Item = (usize, Option<Model>, usize)> {
    backend
        .calls()
        .iter()
        .filter(move |record| record.call == call)
        .map(|record| (record.port, record.model, record.count))
}

/// Raw goals of the newest `call` on `port`, in id order.
fn goals_on(backend: &Scripted, call: Call, port: usize) -> Option<&[(u8, u32)]> {
    let record = backend
        .calls()
        .iter()
        .rev()
        .find(|record| record.call == call && record.port == port)?;
    Some(backend.goals(record))
}

/// Whether `goals` are the `(id, raw)` goals of the actuators `select`
/// picks, in index order.
fn goals_are(
    goals: Option<&[(u8, u32)]>,
    select: impl Fn(usize) -> bool,
    raw: impl Fn(usize) -> u32,
) -> bool {
    let expected = (0..N)
        .filter(|&index| select(index))
        .map(|index| (id(index), raw(index)));
    goals.is_some_and(|goals| goals.iter().copied().eq(expected))
}

#[test]
fn an_ax12_bus_beside_an_mx64_bus_is_one_group_per_bus_on_either_firmware()
-> Result<(), DynamixelError> {
    for table in [AX_MX_TABLE, AX_MX2_TABLE] {
        let claim = rig(table).check(Overrides::default())?;
        for index in 0..N {
            assert_eq!(claim.group[index], Group(u8::from(on_mx(index))));
        }
        assert_eq!(claim.owned, Mask::ALL);
        assert_eq!((claim.read_cost, claim.write_cost), declared_costs(&table));
    }
    Ok(())
}

#[test]
fn an_ax12_bus_beside_a_protocol_1_mx64_bus_runs_every_step() -> Result<(), DynamixelError> {
    let (adapter, _, slot) = run_ax_beside_mx(AX_MX_TABLE)?;
    let backend = &adapter.backend;
    let mx = (0..N).filter(|&index| on_mx(index)).count();

    for (index, q) in slot.q.iter().enumerate() {
        let model = if on_mx(index) {
            Model::Mx64
        } else {
            Model::Ax12a
        };
        assert_eq!(*q, si(model, mx_scenario_raw(index)), "{index}");
        assert_eq!(slot.qd[index], Qd(9.0), "Protocol 1 rates are unread");
    }
    let counts = [
        (Call::Declare, 2),
        (Call::ReadAngleLimits, N),
        (Call::ReadSetting, mx),
        (Call::SyncReadState, 0),
        (Call::ReadPosition, 3 * N),
        (Call::Torque, N),
        (Call::ForceTorqueOff, N),
    ];
    for (call, count) in counts {
        assert_eq!(backend.count(call), count, "{call:?}");
    }
    assert!(
        sent(backend, Call::SyncWriteGoal)
            .take(2)
            .eq([(0, Some(Model::Ax12a), N - mx), (1, Some(Model::Mx64), mx),])
    );
    let start = |index| mx_scenario_raw(index).cast_unsigned();
    assert!(goals_are(
        goals_on(backend, Call::SyncWriteStartGoal, 1),
        on_mx,
        start
    ));
    assert!(goals_are(
        goals_on(backend, Call::SyncWriteGoal, 1),
        on_mx,
        |_| 2048
    ));
    assert!(goals_are(
        goals_on(backend, Call::SyncWriteGoal, 0),
        |index| !on_mx(index),
        |_| 512
    ));
    Ok(())
}

#[test]
fn an_ax12_bus_beside_an_mx64_v2_bus_runs_every_step() -> Result<(), DynamixelError> {
    let (adapter, _, slot) = run_ax_beside_mx(AX_MX2_TABLE)?;
    let backend = &adapter.backend;
    let mx = (0..N).filter(|&index| on_mx(index)).count();

    for (index, q) in slot.q.iter().enumerate() {
        let model = if on_mx(index) {
            Model::Mx64V2
        } else {
            Model::Ax12a
        };
        assert_eq!(*q, si(model, mx_scenario_raw(index)), "{index}");
        if on_mx(index) {
            assert_eq!(slot.qd[index], units::velocity_to_si(Model::Mx64V2, 100));
            assert_eq!(
                Some(slot.tau[index]),
                units::current_to_si(Model::Mx64V2, 100)
            );
        } else {
            assert_eq!(slot.qd[index], Qd(9.0), "Protocol 1 rates are unread");
        }
    }
    let counts = [
        (Call::ReadAngleLimits, N),
        (Call::ReadSetting, 2 * mx),
        (Call::SyncReadState, 3 * Scripted::STATE_READS.len()),
        (Call::ReadPosition, 3 * (N - mx)),
        (Call::Torque, N),
        (Call::ForceTorqueOff, N),
    ];
    for (call, count) in counts {
        assert_eq!(backend.count(call), count, "{call:?}");
    }
    assert!(sent(backend, Call::SyncReadState).all(|read| read == (1, Some(Model::Mx64V2), mx)));
    assert!(sent(backend, Call::SyncWriteGoal).take(2).eq([
        (0, Some(Model::Ax12a), N - mx),
        (1, Some(Model::Mx64V2), mx),
    ]));
    assert!(goals_are(
        goals_on(backend, Call::SyncWriteGoal, 1),
        on_mx,
        |_| 2048
    ));
    Ok(())
}

#[test]
fn a_protocol_1_mx64_shares_the_ax_sync_write_on_one_bus() -> Result<(), DynamixelError> {
    let mut actuators = chain(Model::Ax12a);
    for (index, actuator) in actuators.iter_mut().enumerate() {
        if on_mx(index) {
            actuator.model = Model::Mx64;
        }
    }
    let table = Table {
        buses: one_bus("p1", NonZeroU32::new(1)),
        actuators,
    }
    .check();
    let mut adapter = rig(table);
    for index in (0..N).filter(|&index| on_mx(index)) {
        adapter.backend.angle_limits(0, id(index), 0, 4095);
    }
    let claim = powered(&mut adapter)?;
    assert_eq!(claim.group, [Group(0); N]);
    assert_eq!((claim.read_cost, claim.write_cost), declared_costs(&table));

    let faults = write(&mut adapter, &claim, &Desired::all_q());

    assert_eq!(faults, Faults::default());
    assert!(sent(&adapter.backend, Call::SyncWriteGoal).eq([(0, Some(Model::Ax12a), N)]));
    let centre = |index| if on_mx(index) { 2048 } else { 512 };
    assert!(goals_are(
        Some(adapter.backend.last_goals()),
        |_| true,
        centre
    ));
    Ok(())
}

#[test]
fn an_mx64_v2_shares_the_x_sync_read_and_write_on_one_bus() -> Result<(), DynamixelError> {
    let mut actuators = chain(Model::Xm430W210);
    for (index, actuator) in actuators.iter_mut().enumerate() {
        if on_mx(index) {
            actuator.model = Model::Mx64V2;
        }
    }
    let mut adapter = rig(Table {
        buses: one_bus("p2", NonZeroU32::new(1)),
        actuators,
    }
    .check());
    let claim = powered(&mut adapter)?;
    assert_eq!(claim.group, [Group(0); N]);

    let (refreshed, _) = read(&mut adapter, &mut Slot::filled(0.0));
    let faults = write(&mut adapter, &claim, &Desired::all_q());

    assert_eq!((refreshed, faults), (Mask::ALL, Faults::default()));
    assert_eq!(
        adapter.backend.count(Call::SyncReadState),
        Scripted::STATE_READS.len()
    );
    assert!(sent(&adapter.backend, Call::SyncWriteGoal).eq([(0, Some(Model::Xm430W210), N)]));
    Ok(())
}

#[test]
fn an_mx64_in_multi_turn_mode_fails_configure_before_any_torque() -> Result<(), DynamixelError> {
    let mut adapter = rig(AX_MX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    adapter.backend.angle_limits(1, id(1), 4095, 4095);

    let error = adapter.configure();

    assert_eq!(
        error,
        Err(DynamixelError::MultiTurnMode {
            actuator: 1,
            bus: "mx",
            id: id(1),
            limit: 4095,
        })
    );
    assert_eq!(
        error.err().map(|error| error.to_string()).as_deref(),
        Some(
            "actuator 1: id 2 on bus mx: CW and CCW Angle Limit both 4095: multi-turn mode, not joint mode"
        )
    );
    assert_eq!(adapter.backend.count(Call::Torque), 0);
    Ok(())
}

#[test]
fn an_ax12_at_4095_angle_limits_is_no_multi_turn_mode() -> Result<(), DynamixelError> {
    let mut adapter = rig(AX_MX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);
    for index in (0..N).filter(|&index| on_mx(index)) {
        adapter.backend.angle_limits(1, id(index), 0, 4095);
    }
    adapter.backend.angle_limits(0, id(0), 4095, 4095);

    assert_eq!(adapter.configure(), Ok(()));
    Ok(())
}

/// Bring-up of [`AX_MX_TABLE`] as far as it goes, the MX-64 in joint mode
/// over their whole range, after `script`.
fn bring_up_ax_beside_mx(
    script: impl FnOnce(&mut Scripted),
) -> (Dynamixel<Scripted, 2, N>, Result<Claim<N>, DynamixelError>) {
    let mut adapter = rig(AX_MX_TABLE);
    for index in (0..N).filter(|&index| on_mx(index)) {
        adapter.backend.angle_limits(1, id(index), 0, 4095);
    }
    script(&mut adapter.backend);
    let result = powered(&mut adapter);
    (adapter, result)
}

/// Whether bring-up stopped before any torque or goal write.
fn nothing_powered(backend: &Scripted) -> bool {
    [Call::SyncWriteStartGoal, Call::Torque, Call::ForceTorqueOff]
        .into_iter()
        .all(|call| backend.count(call) == 0)
}

/// Pings `table` and checks one setup per bus, naming each of its
/// actuators with the Model Number of its declared model.
fn pings_with_model_numbers<const NB: usize>(table: Table<NB, N>) -> Result<(), DynamixelError> {
    let mut adapter = rig(table);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.ping()?;

    let backend = &adapter.backend;
    let setups = backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::Declare);
    assert_eq!(setups.clone().count(), NB);
    for (bus, record) in setups.enumerate() {
        let declared = table
            .actuators
            .iter()
            .filter(|actuator| actuator.bus == bus)
            .map(|actuator| (actuator.id, u32::from(actuator.model.number())));
        assert_eq!(record.port, bus);
        assert!(backend.models(record).iter().copied().eq(declared));
    }
    Ok(())
}

#[test]
fn ping_reads_each_model_number_once_and_accepts_the_declared_one() -> Result<(), DynamixelError> {
    pings_with_model_numbers(AX_MX_TABLE)?;
    pings_with_model_numbers(AX_MX2_TABLE)?;
    pings_with_model_numbers(MIX_TABLE)?;
    pings_with_model_numbers(RX_TABLE)
}

#[test]
fn an_ax12_answering_as_a_declared_mx64_fails_ping_before_any_torque() {
    let (adapter, result) = bring_up_ax_beside_mx(|backend| {
        backend.model_number(1, id(3), Model::Ax12a.number());
    });
    let error = result.err();

    assert_eq!(
        error,
        Some(DynamixelError::ModelMismatch {
            actuator: 3,
            bus: "mx",
            id: id(3),
            declared: Model::Mx64,
            found: 12,
            expected: 310,
        })
    );
    assert_eq!(
        error.map(|error| error.to_string()).as_deref(),
        Some("actuator 3: id 4 on bus mx: Model Number 12, expected 310 for the declared MX-64")
    );
    let backend = &adapter.backend;
    assert_eq!(backend.count(Call::Declare), 2);
    assert_eq!(backend.count(Call::ReadHealth), 0);
    assert!(nothing_powered(backend));
}

#[test]
fn an_mx64_answering_as_a_declared_ax12_fails_ping_before_any_torque() {
    let (adapter, result) = bring_up_ax_beside_mx(|backend| {
        backend.model_number(0, id(0), Model::Mx64.number());
    });
    let error = result.err();

    assert_eq!(
        error,
        Some(DynamixelError::ModelMismatch {
            actuator: 0,
            bus: "ax",
            id: id(0),
            declared: Model::Ax12a,
            found: 310,
            expected: 12,
        })
    );
    assert_eq!(
        error.map(|error| error.to_string()).as_deref(),
        Some("actuator 0: id 1 on bus ax: Model Number 310, expected 12 for the declared AX-12A")
    );
    assert_eq!(adapter.backend.count(Call::ReadHealth), 0);
    assert!(nothing_powered(&adapter.backend));
}

#[test]
fn a_failed_model_number_read_names_the_actuator() {
    let (adapter, result) = bring_up_ax_beside_mx(|backend| {
        backend.fail(Call::Declare, 1, id(1), BackendError::Status(0x04));
    });

    assert_eq!(
        result.err(),
        Some(DynamixelError::Actuator {
            actuator: 1,
            bus: "mx",
            id: id(1),
            source: BackendError::Status(0x04),
        })
    );
    assert!(nothing_powered(&adapter.backend));
}

#[test]
fn an_mx64_in_torque_control_mode_fails_configure_before_any_torque() {
    let (adapter, result) = bring_up_ax_beside_mx(|backend| {
        backend.torque_mode(1, id(1), 1);
    });
    let error = result.err();

    assert_eq!(
        error,
        Some(DynamixelError::TorqueCtrlMode {
            actuator: 1,
            bus: "mx",
            id: id(1),
            found: 1,
        })
    );
    assert_eq!(
        error.map(|error| error.to_string()).as_deref(),
        Some(
            "actuator 1: id 2 on bus mx: Torque Ctrl Mode Enable 1, expected 0: it ignores Goal Position"
        )
    );
    let backend = &adapter.backend;
    assert!(sent(backend, Call::ReadSetting).eq([(1, Some(Model::Mx64), 1)]));
    assert!(nothing_powered(backend));
}

#[test]
fn only_the_protocol_1_mx64_reads_torque_ctrl_mode_enable() -> Result<(), DynamixelError> {
    let mut adapter = rig(AX_MX_TABLE);
    for index in (0..N).filter(|&index| on_mx(index)) {
        adapter.backend.angle_limits(1, id(index), 0, 4095);
    }
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.configure()?;

    let readers = (0..N)
        .filter(|&index| on_mx(index))
        .map(|_| (1, Some(Model::Mx64), 1));
    assert!(sent(&adapter.backend, Call::ReadSetting).eq(readers));
    Ok(())
}

#[test]
fn a_placement_past_the_robot_is_refused_at_check() {
    let placement = Table {
        buses: [Bus {
            label: "bus0",
            baud: 1_000_000,
            read_every: None,
        }],
        actuators: chain(Model::Xm430W210),
    }
    .at([0, 1, 2, 7]);
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), placement);

    assert_eq!(
        adapter.check(Overrides::default()).err(),
        Some(DynamixelError::PastTheRobot {
            actuator: 3,
            index: 7,
            actuators: N,
        })
    );
}

#[test]
fn one_adapter_type_serves_a_ten_actuator_robot() {
    let mut chain = Dynamixel::<_, _, 10>::new(Scripted::new(), Rx28Chain::HARDWARE.in_order());
    let claim = chain
        .check(Overrides::default())
        .expect("the preset is valid");
    assert_eq!(claim.owned, Mask::<10>::ALL);
}

// ── T059: no allocation per tick (SC-008 (SRS NFR-1)) ───────────────────────

// DECISION: the counting allocator lives in the lib's test binary, not an
// integration test: the scripted backend is test-only and crate-private. Its
// counter is per thread, so the other tests running beside it count nothing.
thread_local! {
    /// Whether this thread counts its allocations: const, no destructor.
    static COUNTING: Cell<bool> = const { Cell::new(false) };
    /// Allocations this thread made while counting.
    static ALLOCATIONS: Cell<u64> = const { Cell::new(0) };
}

/// [`System`] plus a per-thread count of the allocations made while the
/// thread counts.
struct Counting;

impl Counting {
    /// Counts one allocation when this thread counts; a thread being torn
    /// down counts nothing.
    fn count() {
        if COUNTING.try_with(Cell::get).unwrap_or(false) {
            let _ = ALLOCATIONS.try_with(|count| count.set(count.get() + 1));
        }
    }
}

// SAFETY: every method forwards its arguments unchanged to `System`, which
// upholds the `GlobalAlloc` contract; the count touches neither the layout
// nor the pointers and allocates nothing (two const thread-local `Cell`s).
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: AllocLayout) -> *mut u8 {
        Self::count();
        // SAFETY: the caller's contract for `alloc` is passed on as is.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: AllocLayout) -> *mut u8 {
        Self::count();
        // SAFETY: the caller's contract for `alloc_zeroed` is passed on as is.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: AllocLayout, new_size: usize) -> *mut u8 {
        Self::count();
        // SAFETY: `ptr` came from this allocator, which is `System`; the
        // caller's contract holds.
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: AllocLayout) {
        // SAFETY: `ptr` came from this allocator, which is `System`; the
        // caller's contract holds.
        unsafe { System.dealloc(ptr, layout) }
    }
}

/// The allocator of the lib's test binary.
#[global_allocator]
static ALLOCATOR: Counting = Counting;

/// Allocations `run` makes on this thread.
pub(super) fn allocations(run: impl FnOnce()) -> u64 {
    let before = ALLOCATIONS.with(Cell::get);
    COUNTING.with(|counting| counting.set(true));
    run();
    COUNTING.with(|counting| counting.set(false));
    ALLOCATIONS.with(Cell::get) - before
}

/// Ticks of the no-allocation run.
const TICKS: usize = 10_000;

#[test]
fn the_counter_sees_an_allocation_on_its_thread() {
    let counted = allocations(|| drop(std::hint::black_box(vec![0_u8; 16])));
    assert!(counted >= 1);
}

#[test]
fn ten_thousand_read_and_write_ticks_allocate_nothing() -> Result<(), DynamixelError> {
    let mut adapter = bound(MIX_TABLE, &MIX_PLUGINS);
    let layout = powered(&mut adapter)?;
    let mut slot = Slot::filled(0.0);
    let desired = [Desired::ramp(), Desired::all_q()];
    // One warm-up tick of each: lazy statics and log callsites settle first.
    for written in &desired {
        read(&mut adapter, &mut slot);
        write(&mut adapter, &layout, written);
    }

    let counted = allocations(|| {
        for tick in 0..TICKS {
            let (_, read_faults) = read(&mut adapter, &mut slot);
            let write_faults = write(&mut adapter, &layout, &desired[tick % 2]);
            assert_eq!(
                (read_faults, write_faults),
                (Faults::default(), Faults::default())
            );
        }
    });

    assert_eq!(counted, 0, "{counted} allocations in {TICKS} ticks");
    assert!(adapter.backend.count(Call::SyncReadState) >= TICKS);
    assert!(adapter.backend.count(Call::ReadPosition) >= TICKS);
    assert!(adapter.backend.count(Call::SyncWriteGoal) >= 2 * TICKS);
    Ok(())
}

// ── T070: health limits written through the placement (D18) ─────────────────

/// Limits other than the scripted actuators' own 6.0–19.0 V and 80 °C.
const LIMITS: HealthLimits = HealthLimits {
    min_voltage_dv: 100,
    max_voltage_dv: 140,
    temperature_c: 70,
};

/// An adapter for `table` in order, every actuator given [`LIMITS`], its
/// scripted ports open and its claim made.
fn limited_rig<const NB: usize>(table: Table<NB, N>) -> Dynamixel<Scripted, NB, N> {
    let placement = table.in_order().health_limits(LIMITS);
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), placement);
    adapter.check(Overrides::default()).unwrap();
    mark_open(&mut adapter);
    adapter
}

/// The limits `health` reads back from `adapter`'s actuator `index`.
fn held<const NB: usize>(adapter: &Dynamixel<Scripted, NB, N>, index: usize) -> HealthLimits {
    let actuator = adapter.actuators.get(index).unwrap();
    adapter.backend.limits(actuator.bus, actuator.id).unwrap()
}

#[test]
fn limits_are_written_torque_off_then_read_back_by_health() -> Result<(), DynamixelError> {
    let mut adapter = limited_rig(MX2_TABLE);

    adapter.health()?;

    for index in 0..N {
        assert_eq!(held(&adapter, index), LIMITS);
        assert!(!adapter.backend.torqued(0, id(index)));
    }
    let order = adapter
        .backend
        .calls()
        .iter()
        .map(|record| (record.call, record.id));
    let expected = (0..N).flat_map(|index| {
        [
            Call::ReadSetting,
            Call::ConfirmTorqueOff,
            Call::WriteTemperatureLimit,
            Call::WriteVoltageLimits,
            Call::ReadHealth,
        ]
        .map(|call| (call, id(index)))
    });
    assert!(order.eq(expected));
    Ok(())
}

#[test]
fn health_holds_an_actuator_to_the_limits_written() {
    let mut adapter = limited_rig(MX2_TABLE);
    adapter.backend.health(
        0,
        id(2),
        Health {
            voltage_dv: 150,
            ..HEALTHY
        },
    );

    assert_eq!(
        adapter.health(),
        Err(DynamixelError::Voltage {
            actuator: 2,
            bus: "mx",
            id: id(2),
            measured_dv: 150,
            min_dv: LIMITS.min_voltage_dv,
            max_dv: LIMITS.max_voltage_dv,
        })
    );
}

#[test]
fn a_limit_read_back_other_than_written_is_an_error_naming_both() {
    let mut adapter = limited_rig(MX2_TABLE);
    adapter.backend.stuck_limits(0, id(1));

    let error = adapter.health().unwrap_err();

    assert_eq!(
        error,
        DynamixelError::LimitsMismatch {
            actuator: 1,
            bus: "mx",
            id: id(1),
            written: LIMITS,
            read: HealthLimits {
                min_voltage_dv: 60,
                max_voltage_dv: 190,
                temperature_c: 80,
            },
        }
    );
    assert_eq!(
        error.to_string(),
        "actuator 1: id 2 on bus mx: health limits read back 6.0–19.0 V, 80 °C, written 10.0–14.0 V, 70 °C"
    );
}

/// Whether `backend` was asked no limit write and no torque write.
fn nothing_written(backend: &Scripted) -> bool {
    [
        Call::ConfirmTorqueOff,
        Call::ForceTorqueOff,
        Call::Torque,
        Call::WriteVoltageLimits,
        Call::WriteTemperatureLimit,
    ]
    .iter()
    .all(|&call| backend.count(call) == 0)
}

#[test]
fn limits_for_a_powered_actuator_are_refused_and_its_torque_left_alone() {
    for (table, bus) in [(MX2_TABLE, "mx"), (AX_TABLE, "ax")] {
        let mut adapter = limited_rig(table);
        adapter.backend.already_on(0, id(0));

        let error = adapter.health().unwrap_err();

        assert_eq!(
            error,
            DynamixelError::LimitsTorqueOn {
                actuator: 0,
                bus,
                id: id(0),
            }
        );
        assert!(
            error
                .to_string()
                .contains("torque is already on: turn it off before configuring")
        );
        assert!(nothing_written(&adapter.backend), "{bus}");
        assert!(adapter.backend.torqued(0, id(0)), "{bus}");
    }
}

#[test]
fn protocol_1_limits_are_written_once_its_torque_reads_off() -> Result<(), DynamixelError> {
    let mut adapter = limited_rig(AX_TABLE);

    adapter.health()?;

    assert_eq!(adapter.backend.count(Call::ReadSetting), N);
    assert_eq!(adapter.backend.count(Call::ConfirmTorqueOff), N);
    assert_eq!(adapter.backend.count(Call::WriteVoltageLimits), N);
    assert_eq!(adapter.backend.count(Call::WriteTemperatureLimit), N);
    for index in 0..N {
        assert_eq!(held(&adapter, index), LIMITS);
    }
    Ok(())
}

#[test]
fn no_limits_set_writes_none() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.health()?;

    assert!(nothing_written(&adapter.backend));
    assert_eq!(adapter.backend.count(Call::ReadSetting), 0);
    Ok(())
}

#[test]
fn limits_of_one_row_reach_that_actuator_only() -> Result<(), DynamixelError> {
    let placement = MX2_TABLE.at([3, 2, 1, 0]).health_limits_of(1, LIMITS);
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), placement);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.health()?;

    assert_eq!(adapter.backend.count(Call::WriteVoltageLimits), 1);
    assert_eq!(
        adapter
            .backend
            .last(Call::WriteVoltageLimits)
            .map(|record| record.id),
        Some(id(1))
    );
    assert_eq!(adapter.actuators.limits(2), Some(LIMITS));
    Ok(())
}

#[test]
fn a_failed_limit_write_names_the_actuator() {
    let mut adapter = limited_rig(MX2_TABLE);
    adapter.backend.fail(
        Call::WriteVoltageLimits,
        0,
        id(3),
        BackendError::Status(0x02),
    );

    assert_eq!(
        adapter.health(),
        Err(DynamixelError::Actuator {
            actuator: 3,
            bus: "mx",
            id: id(3),
            source: BackendError::Status(0x02),
        })
    );
}

#[test]
fn the_fake_refuses_limits_as_dxl_rs_v2_does() {
    let mut backend = Scripted::new();
    let specs = [PortSpec {
        path: Path::new(NULL),
        baud: 1_000_000,
    }];
    assert_eq!(backend.open_ports(&specs), Ok(()));
    let mx = Model::Mx64V2;
    let unitless = [
        (4, Model::Xm430W210),
        (5, Model::Xw540T140),
        (6, Model::Xw540T260),
    ];
    backend
        .declare(0, &[(1, mx), (2, Model::Rx28), (3, mx)])
        .unwrap();
    backend.declare(0, &unitless).unwrap();

    assert_eq!(
        backend.write_voltage_limits(0, mx, 1, 100, 140),
        Err(BackendError::TorqueUnconfirmed { id: 1 })
    );
    assert_eq!(
        backend.write_temperature_limit(0, mx, 1, 70),
        Err(BackendError::TorqueUnconfirmed { id: 1 })
    );
    for (id, model) in [(1, mx), (2, Model::Rx28), (3, mx)]
        .into_iter()
        .chain(unitless)
    {
        assert_eq!(backend.confirm_torque_off(0, model, id), Ok(()));
    }
    let no_unit = Err(BackendError::Refused {
        reason: "a limit dxl_rs_v2 has no unit for on this model",
    });
    for (id, model) in unitless {
        assert_eq!(
            backend.write_voltage_limits(0, model, id, 100, 140),
            no_unit
        );
        assert_eq!(backend.write_temperature_limit(0, model, id, 70), no_unit);
        assert_eq!(backend.limits(0, id), Some(HEALTHY_LIMITS), "{model:?}");
    }
    assert_eq!(
        backend.write_voltage_limits(0, Model::Rx28, 2, 100, 300),
        Err(BackendError::Refused {
            reason: "a voltage limit wider than its one-byte register"
        })
    );
    assert_eq!(backend.write_voltage_limits(0, mx, 3, 100, 300), Ok(()));
    assert_eq!(backend.write_temperature_limit(0, mx, 3, 70), Ok(()));
    let wide = HealthLimits {
        min_voltage_dv: 100,
        max_voltage_dv: 300,
        temperature_c: 70,
    };
    assert_eq!(backend.limits(0, 3), Some(wide));
    assert_eq!(
        backend.limits(0, 1).map(|held| held.max_voltage_dv),
        Some(190)
    );
    let start = [(3, 0)];
    backend
        .sync_write_start_goal(0, mx, GoalRegister::Position, &start)
        .unwrap();
    assert_eq!(
        backend.write_temperature_limit(0, mx, 3, 75),
        Err(BackendError::TorqueUnconfirmed { id: 3 })
    );
}

// ── Voltage limits set on the actuators declared in `main.rs` ──────────────

/// Settings giving only the voltages of [`LIMITS`].
const VOLTAGES: Settings = Settings {
    voltage_limits_dv: Some((LIMITS.min_voltage_dv, LIMITS.max_voltage_dv)),
    ..Settings::NONE
};

/// An adapter for `table` in order, actuator `row` given `settings`, its
/// scripted ports open and its claim made.
fn voltage_rig<const NB: usize>(
    table: Table<NB, N>,
    row: usize,
    settings: Settings,
) -> Dynamixel<Scripted, NB, N> {
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), set(table, row, settings));
    adapter.check(Overrides::default()).unwrap();
    mark_open(&mut adapter);
    adapter
}

#[test]
fn voltage_limits_are_written_torque_off_and_leave_the_temperature_limit()
-> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(MX2_TABLE, 2, VOLTAGES);

    adapter.health()?;

    assert_eq!(
        held(&adapter, 2),
        HealthLimits {
            temperature_c: HEALTHY.temperature_limit_c,
            ..LIMITS
        }
    );
    assert_eq!(held(&adapter, 1).min_voltage_dv, HEALTHY.min_voltage_dv);
    assert!(!adapter.backend.torqued(0, id(2)));
    assert_eq!(adapter.backend.count(Call::WriteTemperatureLimit), 0);
    let order = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.id == id(2))
        .map(|record| record.call);
    assert!(order.eq([
        Call::ReadSetting,
        Call::ConfirmTorqueOff,
        Call::WriteVoltageLimits,
        Call::ReadHealth
    ]));
    Ok(())
}

#[test]
fn protocol_1_voltage_limits_are_written_once_its_torque_reads_off() -> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(AX_TABLE, 0, VOLTAGES);

    adapter.health()?;

    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ReadSetting,
        Call::ConfirmTorqueOff,
        Call::WriteVoltageLimits,
        Call::ReadHealth
    ]));
    assert_eq!(held(&adapter, 0).max_voltage_dv, LIMITS.max_voltage_dv);
    Ok(())
}

#[test]
fn voltage_limits_for_a_powered_actuator_are_refused() {
    let mut adapter = voltage_rig(MX2_TABLE, 0, VOLTAGES);
    adapter.backend.already_on(0, id(0));

    assert_eq!(
        adapter.health(),
        Err(DynamixelError::LimitsTorqueOn {
            actuator: 0,
            bus: "mx",
            id: id(0),
        })
    );
    assert!(nothing_written(&adapter.backend));
    assert!(adapter.backend.torqued(0, id(0)));
}

#[test]
fn voltage_limits_read_back_other_than_written_name_the_voltages() {
    let mut adapter = voltage_rig(MX2_TABLE, 1, VOLTAGES);
    adapter.backend.stuck_limits(0, id(1));

    let error = adapter.health().unwrap_err();

    assert_eq!(
        error.to_string(),
        "actuator 1: id 2 on bus mx: health limits read back 6.0–19.0 V, 80 °C, written 10.0–14.0 V, 80 °C"
    );
}

#[test]
fn voltage_limits_override_the_voltages_of_the_placement_s_limits() -> Result<(), DynamixelError> {
    let mut placement = MX2_TABLE.in_order().health_limits(LIMITS);
    placement.settings[3] = Settings {
        voltage_limits_dv: Some((95, 130)),
        ..Settings::NONE
    };
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), placement);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.health()?;

    let expected = HealthLimits {
        min_voltage_dv: 95,
        max_voltage_dv: 130,
        ..LIMITS
    };
    assert_eq!(held(&adapter, 3), expected);
    assert_eq!(held(&adapter, 2), LIMITS);
    assert_eq!(adapter.backend.count(Call::WriteVoltageLimits), N);
    Ok(())
}

#[test]
fn check_refuses_crossed_or_too_wide_voltage_limits_before_any_io() {
    let crossed = Settings {
        voltage_limits_dv: Some((130, 95)),
        ..Settings::NONE
    };
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), set(X_TABLE, 1, crossed));
    let error = adapter.check(Overrides::default()).unwrap_err();
    assert_eq!(
        error,
        DynamixelError::VoltageLimitsCrossed {
            actuator: 1,
            bus: "x",
            id: id(1),
            min_dv: 130,
            max_dv: 95,
        }
    );
    assert_eq!(
        error.to_string(),
        "actuator 1: id 2 on bus x: Min Voltage Limit 13.0 V above Max Voltage Limit 9.5 V"
    );
    assert!(adapter.backend.calls().is_empty());

    let wide = Settings {
        voltage_limits_dv: Some((60, 300)),
        ..Settings::NONE
    };
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), set(AX_TABLE, 0, wide));
    assert_eq!(
        adapter.check(Overrides::default()).err(),
        Some(DynamixelError::SettingRange {
            actuator: 0,
            bus: "ax",
            id: id(0),
            model: Model::Ax12a,
            register: "Max Voltage Limit",
            value: 300,
            max: 255,
        })
    );
    assert!(adapter.backend.calls().is_empty());

    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), set(X_TABLE, 0, wide));
    assert!(adapter.check(Overrides::default()).is_ok());
}

// ── An actuator held faulted at ping: health clears it or fails ─────────────

/// What the AX-12A at id 1 answers: 10.1 V against the voltage limits
/// 10.5–13.0 V its EEPROM holds.
const BELOW_ITS_LIMITS: Health = Health {
    voltage_dv: 101,
    min_voltage_dv: 105,
    max_voltage_dv: 130,
    ..HEALTHY
};

/// Message of the warning `ping` logs per actuator it holds faulted.
const HELD_FAULTED: &str = "reported a hardware fault at setup";

/// Message of the warning `health` logs for a Protocol 2 actuator whose
/// fault outlives its limits.
const REBOOT: &str = "keeps a hardware error until the actuator reboots";

/// Pings `adapter`, its actuator `index` answering `health`; the warnings
/// logged that one is held faulted. The backend log is cleared afterwards.
fn ping_answering<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    index: usize,
    health: Health,
) -> Result<usize, DynamixelError> {
    let actuator = *adapter.actuators.get(index).unwrap();
    adapter.backend.health(actuator.bus, actuator.id, health);
    let mut ping = Ok(());
    let warned = warnings(HELD_FAULTED, || ping = adapter.ping());
    ping?;
    adapter.backend.clear_log();
    Ok(warned)
}

/// Whether the backend holds `adapter`'s actuator `index` faulted.
fn faulted<const NB: usize>(adapter: &Dynamixel<Scripted, NB, N>, index: usize) -> bool {
    let actuator = adapter.actuators.get(index).unwrap();
    adapter
        .backend
        .faulted(actuator.bus, actuator.model, actuator.id)
}

/// Whether `adapter`'s actuator `index` is still held faulted, its torque
/// never asked on nor set.
fn still_faulted_off<const NB: usize>(adapter: &Dynamixel<Scripted, NB, N>, index: usize) -> bool {
    let actuator = adapter.actuators.get(index).unwrap();
    let (bus, id) = (actuator.bus, actuator.id);
    faulted(adapter, index)
        && !adapter.backend.torqued(bus, id)
        && !torque_asked(&adapter.backend, bus, id, true)
}

/// `configure` then `torque_on`, actuator `index` of `adapter` answering
/// `clean` and asked nothing since it was held faulted; `torque_on`'s
/// result.
// DECISION: its replies clean again, so only the fault it is held with
// stands before its torque: an earlier step failing on a flagged reply
// would hide a torque let through.
fn torque_on_answering<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    index: usize,
    clean: Health,
) -> Result<Result<(), DynamixelError>, DynamixelError> {
    let actuator = *adapter.actuators.get(index).unwrap();
    adapter.backend.health(actuator.bus, actuator.id, clean);
    adapter.configure()?;
    Ok(adapter.torque_on())
}

/// The error `torque_on` fails with on actuator `index` of bus `bus` held
/// faulted: its torque refused.
fn refused(bus: &'static str, index: usize) -> DynamixelError {
    DynamixelError::Actuator {
        actuator: index,
        bus,
        id: id(index),
        source: BackendError::Faulted { id: id(index) },
    }
}

/// Calls recorded naming `id` on bus 0, in call order.
fn calls_on(backend: &Scripted, id: u8) -> impl Iterator<Item = Call> + '_ {
    backend
        .calls()
        .iter()
        .filter(move |record| record.port == 0 && record.id == id)
        .map(|record| record.call)
}

/// SRS FR-5.2, FR-6.1: `voltage_limits(9.0, 13.0)` brings up the AX-12A at
/// 10.1 V: held faulted at ping with one warning, its limits written then
/// its fault cleared by health, then switched on.
#[test]
fn an_ax12a_below_its_voltage_limits_comes_up_once_voltage_limits_are_written()
-> Result<(), DynamixelError> {
    let nine_to_thirteen = Settings {
        voltage_limits_dv: Some((90, 130)),
        ..Settings::NONE
    };
    let mut adapter = voltage_rig(AX_TABLE, 0, nine_to_thirteen);
    assert_eq!(ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?, 1);
    assert!(faulted(&adapter, 0));

    adapter.health()?;

    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ConfirmTorqueOff,
        Call::WriteVoltageLimits,
        Call::ClearFault,
        Call::ReadHealth
    ]));
    let limits = held(&adapter, 0);
    assert_eq!((limits.min_voltage_dv, limits.max_voltage_dv), (90, 130));
    assert!(!faulted(&adapter, 0));
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;
    adapter.torque_on()?;
    assert!(torque_asked(&adapter.backend, 0, id(0), true));
    assert!(adapter.backend.torqued(0, id(0)));
    Ok(())
}

/// SRS FR-5.2: the same AX-12A with no voltage limits set stays faulted:
/// health fails with its voltage measured, nothing read beside the fault;
/// its torque is refused, even once its supply is raised to 12.0 V.
#[test]
fn an_ax12a_below_its_voltage_limits_with_none_set_fails_health_still_faulted()
-> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(AX_TABLE, 0, Settings::NONE);
    ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?;

    assert_eq!(
        adapter.health(),
        Err(DynamixelError::Voltage {
            actuator: 0,
            bus: "ax",
            id: id(0),
            measured_dv: 101,
            min_dv: 105,
            max_dv: 130,
        })
    );
    assert!(calls_on(&adapter.backend, id(0)).eq([Call::ClearFault]));
    assert!(still_faulted_off(&adapter, 0));
    let raised = Health {
        voltage_dv: 120,
        ..BELOW_ITS_LIMITS
    };
    let torque_on = torque_on_answering(&mut adapter, 0, raised)?;
    assert_eq!(torque_on, Err(refused("ax", 0)));
    assert!(faulted(&adapter, 0) && !adapter.backend.torqued(0, id(0)));
    Ok(())
}

/// SRS FR-5.2, FR-6.1: voltage limits that still exclude 10.1 V are
/// written, then health fails naming the limits the actuator now holds;
/// its torque is refused, even once its supply is raised inside them.
#[test]
fn voltage_limits_that_still_exclude_its_voltage_fail_health_still_faulted()
-> Result<(), DynamixelError> {
    let eleven_to_thirteen = Settings {
        voltage_limits_dv: Some((110, 130)),
        ..Settings::NONE
    };
    let mut adapter = voltage_rig(AX_TABLE, 0, eleven_to_thirteen);
    ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?;

    assert_eq!(
        adapter.health(),
        Err(DynamixelError::Voltage {
            actuator: 0,
            bus: "ax",
            id: id(0),
            measured_dv: 101,
            min_dv: 110,
            max_dv: 130,
        })
    );
    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ConfirmTorqueOff,
        Call::WriteVoltageLimits,
        Call::ClearFault
    ]));
    assert!(still_faulted_off(&adapter, 0));
    let raised = Health {
        voltage_dv: 120,
        min_voltage_dv: 110,
        ..BELOW_ITS_LIMITS
    };
    let torque_on = torque_on_answering(&mut adapter, 0, raised)?;
    assert_eq!(torque_on, Err(refused("ax", 0)));
    assert!(faulted(&adapter, 0) && !adapter.backend.torqued(0, id(0)));
    Ok(())
}

/// SRS FR-5.2, FR-6.1: a Protocol 2 actuator keeps its latched Input
/// Voltage alert after its voltage limits are written, its voltage inside
/// them: no Torque Enable read beside the fault, the limits written, then
/// health fails with its Hardware Error Status and one warning to reboot it.
/// Its torque is refused, even once a reboot cleared its alert.
#[test]
fn a_protocol_2_alert_kept_after_its_voltage_limits_fails_health_asking_for_a_reboot()
-> Result<(), DynamixelError> {
    let latched = Health {
        error_status: 0x01,
        ..HEALTHY
    };
    let mut adapter = voltage_rig(MX2_TABLE, 0, VOLTAGES);
    assert_eq!(ping_answering(&mut adapter, 0, latched)?, 1);

    let mut health = Ok(());
    let warned = warnings(REBOOT, || health = adapter.health());

    let error = DynamixelError::HardwareError {
        actuator: 0,
        bus: "mx",
        id: id(0),
        status: 0x01,
    };
    assert_eq!((health, warned), (Err(error), 1));
    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ConfirmTorqueOff,
        Call::WriteVoltageLimits,
        Call::ClearFault
    ]));
    assert_eq!(held(&adapter, 0).min_voltage_dv, LIMITS.min_voltage_dv);
    assert!(still_faulted_off(&adapter, 0));
    let rebooted = Health {
        min_voltage_dv: LIMITS.min_voltage_dv,
        max_voltage_dv: LIMITS.max_voltage_dv,
        ..HEALTHY
    };
    let torque_on = torque_on_answering(&mut adapter, 0, rebooted)?;
    assert_eq!(torque_on, Err(refused("mx", 0)));
    assert!(faulted(&adapter, 0) && !adapter.backend.torqued(0, id(0)));
    Ok(())
}

/// SRS FR-5.2, FR-6.1: an actuator clean at ping whose own voltage limit
/// write leaves its 12.0 V outside the limits written is faulted by it:
/// health fails with its voltage measured by `clear_fault`, nothing read
/// beside the fault, and its torque is refused.
#[test]
fn a_voltage_limit_write_that_excludes_its_voltage_faults_it_and_fails_health()
-> Result<(), DynamixelError> {
    let thirteen_to_sixteen = Settings {
        voltage_limits_dv: Some((130, 160)),
        ..Settings::NONE
    };
    let mut adapter = voltage_rig(AX_TABLE, 0, thirteen_to_sixteen);
    assert_eq!(ping_answering(&mut adapter, 0, HEALTHY)?, 0);
    assert!(!faulted(&adapter, 0));

    assert_eq!(
        adapter.health(),
        Err(DynamixelError::Voltage {
            actuator: 0,
            bus: "ax",
            id: id(0),
            measured_dv: 120,
            min_dv: 130,
            max_dv: 160,
        })
    );
    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ReadSetting,
        Call::ConfirmTorqueOff,
        Call::WriteVoltageLimits,
        Call::ClearFault
    ]));
    assert!(still_faulted_off(&adapter, 0));
    let raised = Health {
        voltage_dv: 140,
        min_voltage_dv: 130,
        max_voltage_dv: 160,
        ..HEALTHY
    };
    let torque_on = torque_on_answering(&mut adapter, 0, raised)?;
    assert_eq!(torque_on, Err(refused("ax", 0)));
    assert!(faulted(&adapter, 0) && !adapter.backend.torqued(0, id(0)));
    Ok(())
}

/// SRS FR-5.2: the same path for another fault and model: a Protocol 1
/// RX-28 at 65 °C against a Temperature Limit of 60 °C is held at ping,
/// then cleared once the placement's health limits raise it to 70 °C.
#[test]
fn a_protocol_1_actuator_hot_against_a_low_temperature_limit_is_cleared_by_health_limits()
-> Result<(), DynamixelError> {
    // DECISION: an RX-28, not the AX-12A, so the test pins that nothing in
    // the path is tied to one model or to the voltage.
    let mut adapter = limited_rig(RX_TABLE);
    let hot = Health {
        temperature_c: 65,
        temperature_limit_c: 60,
        ..HEALTHY
    };
    assert_eq!(ping_answering(&mut adapter, 0, hot)?, 1);

    adapter.health()?;

    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ConfirmTorqueOff,
        Call::WriteTemperatureLimit,
        Call::WriteVoltageLimits,
        Call::ClearFault,
        Call::ReadHealth
    ]));
    assert_eq!(held(&adapter, 0), LIMITS);
    assert!(!faulted(&adapter, 0));
    Ok(())
}

/// SRS FR-5.2: a fault gone by health, the supply raised to 12.0 V after
/// ping, is cleared with no limit written, then checked as any actuator.
#[test]
fn a_fault_gone_by_health_is_cleared_with_no_limit_written() -> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(AX_TABLE, 0, Settings::NONE);
    ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?;
    let raised = Health {
        voltage_dv: 120,
        ..BELOW_ITS_LIMITS
    };
    adapter.backend.health(0, id(0), raised);

    adapter.health()?;

    assert!(calls_on(&adapter.backend, id(0)).eq([Call::ClearFault, Call::ReadHealth]));
    assert_eq!(adapter.backend.count(Call::ClearFault), 1);
    assert!(!faulted(&adapter, 0));
    Ok(())
}

// ── T083: a temperature limit on the declared ids, typed writes ─────────────

/// Settings giving only a Temperature Limit of 70 °C.
const COOLER: Settings = Settings {
    temperature_limit_c: Some(70),
    ..Settings::NONE
};

#[test]
fn a_temperature_limit_alone_is_written_torque_off_and_leaves_the_voltages()
-> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(MX2_TABLE, 1, COOLER);

    adapter.health()?;

    assert_eq!(
        held(&adapter, 1),
        HealthLimits {
            temperature_c: 70,
            ..HEALTHY_LIMITS
        }
    );
    assert!(calls_on(&adapter.backend, id(1)).eq([
        Call::ReadSetting,
        Call::ConfirmTorqueOff,
        Call::WriteTemperatureLimit,
        Call::ReadHealth
    ]));
    let write = adapter.backend.last(Call::WriteTemperatureLimit).unwrap();
    assert_eq!(adapter.backend.written(write), Some(70));
    Ok(())
}

#[test]
fn limits_go_out_as_the_typed_setters_take_them() -> Result<(), DynamixelError> {
    let mut adapter = limited_rig(AX_TABLE);

    adapter.health()?;

    let voltages = adapter.backend.last(Call::WriteVoltageLimits).unwrap();
    assert_eq!(adapter.backend.written(voltages), Some(100 << 16 | 140));
    let temperature = adapter.backend.last(Call::WriteTemperatureLimit).unwrap();
    assert_eq!(adapter.backend.written(temperature), Some(70));
    Ok(())
}

#[test]
fn a_temperature_limit_setting_overrides_the_placement_s() -> Result<(), DynamixelError> {
    let mut placement = MX2_TABLE.in_order().health_limits(LIMITS);
    placement.settings[3] = Settings {
        temperature_limit_c: Some(75),
        ..Settings::NONE
    };
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), placement);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.health()?;

    let expected = HealthLimits {
        temperature_c: 75,
        ..LIMITS
    };
    assert_eq!(held(&adapter, 3), expected);
    assert_eq!(held(&adapter, 2), LIMITS);
    Ok(())
}

#[test]
fn a_temperature_limit_read_back_other_than_written_names_it() {
    let mut adapter = voltage_rig(MX2_TABLE, 1, COOLER);
    adapter.backend.stuck_limits(0, id(1));

    let error = adapter.health().unwrap_err();

    assert_eq!(
        error.to_string(),
        "actuator 1: id 2 on bus mx: health limits read back 6.0–19.0 V, 80 °C, written 6.0–19.0 V, 70 °C"
    );
}

// ── T084: limits widened for one run, restored after ────────────────────────

/// The limits of a scripted actuator until written: [`HEALTHY`]'s.
const HEALTHY_LIMITS: HealthLimits = HealthLimits {
    min_voltage_dv: HEALTHY.min_voltage_dv,
    max_voltage_dv: HEALTHY.max_voltage_dv,
    temperature_c: HEALTHY.temperature_limit_c,
};

/// `settings` with `restore_limits` set.
const fn restored(settings: Settings) -> Settings {
    Settings {
        restore_limits: true,
        ..settings
    }
}

/// Message of the warning `close` logs per limit it could not put back.
const NOT_RESTORED: &str = "not restored";

/// `torque_off` then `close`, as the engine ends every bring-up; the
/// warnings logged that a limit was not put back.
fn shut<const NB: usize>(adapter: &mut Dynamixel<Scripted, NB, N>) -> usize {
    warnings(NOT_RESTORED, || {
        adapter.torque_off();
        adapter.close();
    })
}

#[test]
fn restore_puts_back_only_the_registers_written_after_torque_off() -> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(MX2_TABLE, 2, restored(VOLTAGES));

    adapter.health()?;
    assert_eq!(held(&adapter, 2).min_voltage_dv, LIMITS.min_voltage_dv);
    adapter.backend.clear_log();
    assert_eq!(shut(&mut adapter), 0);

    assert_eq!(held(&adapter, 2), HEALTHY_LIMITS);
    assert!(calls_on(&adapter.backend, id(2)).eq([Call::ForceTorqueOff, Call::WriteVoltageLimits]));
    let restore = adapter.backend.last(Call::WriteVoltageLimits).unwrap();
    assert_eq!(adapter.backend.written(restore), Some(60 << 16 | 190));
    assert_eq!(adapter.backend.count(Call::WriteTemperatureLimit), 0);
    assert_eq!(last_call(&adapter.backend), Some(Call::ClosePorts));
    Ok(())
}

#[test]
fn restore_saves_each_register_read_just_before_its_write() -> Result<(), DynamixelError> {
    let both = Settings {
        voltage_limits_dv: Some((90, 130)),
        ..COOLER
    };
    let mut adapter = voltage_rig(MX2_TABLE, 0, restored(both));

    adapter.health()?;

    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ReadSetting,
        Call::ConfirmTorqueOff,
        Call::ReadTemperatureLimit,
        Call::ReadVoltageLimits,
        Call::WriteTemperatureLimit,
        Call::WriteVoltageLimits,
        Call::ReadHealth
    ]));
    shut(&mut adapter);
    assert_eq!(held(&adapter, 0), HEALTHY_LIMITS);
    Ok(())
}

#[test]
fn no_limit_is_read_or_restored_without_restore_limits() -> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(MX2_TABLE, 2, VOLTAGES);

    adapter.health()?;
    shut(&mut adapter);

    assert_eq!(adapter.backend.count(Call::ReadVoltageLimits), 0);
    assert_eq!(adapter.backend.count(Call::WriteVoltageLimits), 1);
    assert_eq!(held(&adapter, 2).min_voltage_dv, LIMITS.min_voltage_dv);
    Ok(())
}

#[test]
fn a_failed_restore_is_tried_once_warned_and_close_still_closes() -> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(MX2_TABLE, 2, restored(VOLTAGES));
    adapter.health()?;
    adapter
        .backend
        .fail(Call::WriteVoltageLimits, 0, id(2), BackendError::Timeout);
    adapter.backend.clear_log();

    assert_eq!(shut(&mut adapter), 1);

    assert_eq!(adapter.backend.count(Call::WriteVoltageLimits), 1);
    assert_eq!(last_call(&adapter.backend), Some(Call::ClosePorts));
    assert!(!adapter.backend.is_open());
    Ok(())
}

#[test]
fn a_restore_after_a_failed_torque_off_is_refused_unconfirmed_and_warned()
-> Result<(), DynamixelError> {
    let mut adapter = voltage_rig(MX2_TABLE, 2, restored(VOLTAGES));
    adapter.ping()?;
    adapter.health()?;
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;
    adapter.torque_on()?;
    adapter
        .backend
        .fail(Call::ForceTorqueOff, 0, id(2), BackendError::Timeout);

    adapter.torque_off();
    let model = Model::Mx64V2;
    assert_eq!(
        adapter
            .backend
            .write_voltage_limits(0, model, id(2), 60, 190),
        Err(BackendError::TorqueUnconfirmed { id: id(2) })
    );
    adapter.backend.clear_log();

    let warned = warnings(NOT_RESTORED, || adapter.close());

    assert_eq!(warned, 1);
    assert_eq!(adapter.backend.count(Call::WriteVoltageLimits), 1);
    assert_eq!(held(&adapter, 2).min_voltage_dv, LIMITS.min_voltage_dv);
    assert_eq!(last_call(&adapter.backend), Some(Call::ClosePorts));
    Ok(())
}

#[test]
fn a_failed_bring_up_restores_what_it_wrote_and_nothing_else() {
    let mut placement = MX2_TABLE.in_order();
    placement.settings = [restored(VOLTAGES); N];
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), placement);
    adapter.check(Overrides::default()).unwrap();
    mark_open(&mut adapter);
    let hot = Health {
        temperature_c: 90,
        ..HEALTHY
    };
    adapter.backend.health(0, id(2), hot);

    assert!(matches!(
        adapter.health(),
        Err(DynamixelError::Temperature { actuator: 2, .. })
    ));
    adapter.backend.clear_log();
    assert_eq!(shut(&mut adapter), 0);

    for index in 0..=2 {
        assert_eq!(held(&adapter, index), HEALTHY_LIMITS, "actuator {index}");
    }
    let restored_ids = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::WriteVoltageLimits)
        .map(|record| record.id);
    assert!(restored_ids.eq([id(0), id(1), id(2)]));
}

/// SRS FR-5.2, FR-6.1: the bench AX-12A at 10.1 V, its EEPROM at
/// 10.5–13.0 V and 80 °C, given `voltage_limits(9.0, 13.0)`,
/// `temperature_limit(70)` and `restore_limits()`. Held faulted, its
/// voltages are saved from the fault and written first; `clear_fault`
/// clears it, then its Temperature Limit is read clean, saved and written.
/// After torque off both go back, Temperature Limit first; the voltage
/// write answers with the voltage fault again, so the voltages are read
/// back once, from that fault.
#[test]
fn a_faulted_actuator_saves_from_its_fault_first_and_gets_both_back() -> Result<(), DynamixelError>
{
    let bench = Settings {
        voltage_limits_dv: Some((90, 130)),
        ..restored(COOLER)
    };
    let mut adapter = voltage_rig(AX_TABLE, 0, bench);
    assert_eq!(ping_answering(&mut adapter, 0, BELOW_ITS_LIMITS)?, 1);

    adapter.health()?;

    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ConfirmTorqueOff,
        Call::ReadTemperatureLimit,
        Call::ReadVoltageLimits,
        Call::WriteVoltageLimits,
        Call::ClearFault,
        Call::ReadTemperatureLimit,
        Call::WriteTemperatureLimit,
        Call::ReadHealth
    ]));
    assert_eq!(
        held(&adapter, 0),
        HealthLimits {
            min_voltage_dv: 90,
            max_voltage_dv: 130,
            temperature_c: 70,
        }
    );
    adapter.backend.clear_log();
    assert_eq!(shut(&mut adapter), 0);
    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ForceTorqueOff,
        Call::WriteTemperatureLimit,
        Call::WriteVoltageLimits,
        Call::ReadVoltageLimits
    ]));
    assert_eq!(
        held(&adapter, 0),
        HealthLimits {
            min_voltage_dv: 105,
            max_voltage_dv: 130,
            temperature_c: 80,
        }
    );
    Ok(())
}

/// The run between bring-up and shutdown: `configure`, a read and
/// `torque_on`, which voids the torque-off confirmation `health` left.
fn run(adapter: &mut Dynamixel<Scripted, 1, N>) -> Result<(), DynamixelError> {
    adapter.configure()?;
    adapter.read_all(Slot::filled(0.0).sample())?;
    adapter.torque_on()
}

/// SRS FR-5.8, FR-6.1: after a whole run, torque on included, `torque_off`
/// confirms each actuator off again, so both registers go back unwarned,
/// on either protocol.
#[test]
fn limits_go_back_after_a_run_with_the_torque_on() -> Result<(), DynamixelError> {
    let both = Settings {
        voltage_limits_dv: VOLTAGES.voltage_limits_dv,
        ..restored(COOLER)
    };
    for table in [MX2_TABLE, AX_TABLE] {
        let mut adapter = voltage_rig(table, 2, both);
        adapter.ping()?;
        adapter.health()?;
        run(&mut adapter)?;
        assert!(adapter.backend.torqued(0, id(2)));
        adapter.backend.clear_log();

        assert_eq!(shut(&mut adapter), 0);

        assert_eq!(held(&adapter, 2), HEALTHY_LIMITS);
        assert!(calls_on(&adapter.backend, id(2)).eq([
            Call::ForceTorqueOff,
            Call::WriteTemperatureLimit,
            Call::WriteVoltageLimits
        ]));
    }
    Ok(())
}

/// A restore write answered with a fault is read back once: a value other
/// than the one saved, or a read back that fails beside another fault, is
/// warned, never taken as restored.
#[test]
fn a_restore_answered_with_a_fault_is_warned_unless_its_read_back_confirms_it()
-> Result<(), DynamixelError> {
    let written = Health {
        min_voltage_dv: LIMITS.min_voltage_dv,
        max_voltage_dv: LIMITS.max_voltage_dv,
        ..HEALTHY
    };
    let sagged = Health {
        voltage_dv: 95,
        ..written
    };
    let hot = Health {
        temperature_c: 90,
        ..written
    };
    for (during, message) in [
        (sagged, "read back finds another value"),
        (hot, "read back failed"),
    ] {
        let mut adapter = voltage_rig(AX_TABLE, 0, restored(VOLTAGES));
        adapter.health()?;
        adapter
            .backend
            .health(0, id(0), during)
            .stuck_limits(0, id(0));
        adapter.backend.clear_log();

        let warned = warnings(message, || {
            adapter.torque_off();
            adapter.close();
        });

        assert_eq!(warned, 1, "{message}");

        assert!(calls_on(&adapter.backend, id(0)).eq([
            Call::ForceTorqueOff,
            Call::WriteVoltageLimits,
            Call::ReadVoltageLimits
        ]));
        assert_eq!(held(&adapter, 0).min_voltage_dv, LIMITS.min_voltage_dv);
    }
    Ok(())
}

/// Design point 5 on a temperature fault: a Protocol 1 actuator held at
/// 65 °C against 60 °C saves its Temperature Limit from the fault and
/// writes it first; `clear_fault` then lets its voltages be read clean,
/// saved and written. Both go back after the run.
#[test]
fn a_temperature_fault_saves_its_limit_from_the_fault_and_the_voltages_after_clearing()
-> Result<(), DynamixelError> {
    let both = Settings {
        voltage_limits_dv: Some((90, 130)),
        ..restored(COOLER)
    };
    let mut adapter = voltage_rig(AX_TABLE, 0, both);
    let hot = Health {
        temperature_c: 65,
        temperature_limit_c: 60,
        ..HEALTHY
    };
    assert_eq!(ping_answering(&mut adapter, 0, hot)?, 1);

    adapter.health()?;

    assert!(calls_on(&adapter.backend, id(0)).eq([
        Call::ConfirmTorqueOff,
        Call::ReadTemperatureLimit,
        Call::ReadVoltageLimits,
        Call::WriteTemperatureLimit,
        Call::ClearFault,
        Call::ReadVoltageLimits,
        Call::WriteVoltageLimits,
        Call::ReadHealth
    ]));
    let run_limits = HealthLimits {
        min_voltage_dv: 90,
        max_voltage_dv: 130,
        temperature_c: 70,
    };
    assert_eq!(held(&adapter, 0), run_limits);
    let cooled = Health {
        temperature_c: 30,
        min_voltage_dv: 90,
        max_voltage_dv: 130,
        temperature_limit_c: 70,
        ..HEALTHY
    };
    adapter.backend.health(0, id(0), cooled);
    adapter.backend.clear_log();

    assert_eq!(shut(&mut adapter), 0);

    let before = HealthLimits {
        temperature_c: 60,
        ..HEALTHY_LIMITS
    };
    assert_eq!(held(&adapter, 0), before);
    Ok(())
}

/// The fake refuses limits where `DxlRsV2` does: health on an XM430-W210
/// given any fails at its first limit write, with the same reason.
#[test]
fn limits_on_an_xm430_fail_health_without_a_unit() {
    for settings in [VOLTAGES, COOLER] {
        let mut adapter = voltage_rig(X_TABLE, 1, settings);

        assert_eq!(
            adapter.health(),
            Err(DynamixelError::Actuator {
                actuator: 1,
                bus: "x",
                id: id(1),
                source: BackendError::Refused {
                    reason: "a limit dxl_rs_v2 has no unit for on this model",
                },
            })
        );
        assert_eq!(held(&adapter, 1), HEALTHY_LIMITS);
    }
}

// ── A table built at run time ───────────────────────────────────────────────

/// Bus slots of [`declared`]: more than the backend opens, one in use.
const SLOTS: usize = 8;

/// `X_TABLE` built at run time: its one bus on serial `ARM`, the other slots
/// unused and unlabelled.
fn declared() -> super::Declared<SLOTS, N> {
    let mut buses = [Bus {
        label: "",
        baud: 0,
        read_every: None,
    }; SLOTS];
    buses[0] = Bus {
        label: ARM,
        ..X_TABLE.buses[0]
    };
    let table = Table {
        buses,
        actuators: X_TABLE.actuators,
    };
    super::Declared::new(table, [Port::ById(ARM); SLOTS], 1).unwrap()
}

#[test]
fn a_declared_table_counts_its_live_buses_only() -> Result<(), DynamixelError> {
    let mut adapter = Dynamixel::<_, _, N>::declared(Scripted::new(), declared());
    let claim = adapter.check(Overrides::default())?;
    assert_eq!(claim.owned, Mask::ALL);
    Ok(())
}

#[test]
fn a_declared_table_resolves_its_live_buses_only() -> io::Result<()> {
    let roots = Roots::new("declared_resolve")?;
    let mut adapter = roots.rig(Dynamixel::<_, _, N>::declared(Scripted::new(), declared()));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(
        adapter.open(),
        Err(DynamixelError::PortNotFound {
            bus: ARM,
            port: Port::ById(ARM)
        })
    );
    assert!(adapter.backend.calls().is_empty());
    Ok(())
}

#[test]
fn a_declared_table_opens_its_live_buses_only() -> io::Result<()> {
    let Some(arm) = pty() else {
        return Ok(());
    };
    let roots = Roots::new("declared_open")?;
    roots.link(ARM, &arm.slave)?;
    let mut adapter = roots.rig(Dynamixel::<_, _, N>::declared(Scripted::new(), declared()));
    assert!(adapter.check(Overrides::default()).is_ok());

    assert_eq!(adapter.open(), Ok(()));
    assert_eq!(
        adapter
            .backend
            .last(Call::OpenPorts)
            .map(|record| record.count),
        Some(1)
    );
    assert_eq!(adapter.backend.baud(0), Some(1_000_000));
    adapter.torque_off();
    adapter.close();
    Ok(())
}

#[test]
fn a_declared_table_breaking_a_check_rule_is_refused() {
    let mut table = declared().placement.table;
    table.buses[0].baud = 500_000;
    let error = super::Declared::new(table, [Port::ById(ARM); SLOTS], 1).unwrap_err();
    assert_eq!(
        error.to_string(),
        "actuator 0: XM430-W210 does not accept 500000 baud on bus ARM00001"
    );
}

// ── SRS FR-5.11: telemetry rounds, one transaction per due tick at most ─────

/// One `telemetry` call into `telemetry`, with its faults.
fn refresh<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    telemetry: &mut Telemetry<N>,
) -> (Mask<N>, Faults<N>) {
    let mut faults = Faults::default();
    let refreshed = adapter.telemetry(telemetry, &mut faults);
    (refreshed, faults)
}

/// `(port, register, first id, ids)` of every `ReadVitals` recorded, oldest
/// first.
fn vitals_read(backend: &Scripted) -> impl Iterator<Item = (usize, Option<Vital>, u8, usize)> {
    backend
        .calls()
        .iter()
        .filter(|record| record.call == Call::ReadVitals)
        .map(|record| (record.port, backend.vital(record), record.id, record.count))
}

/// Ticks between two due telemetry calls.
const EVERY: usize = TELEMETRY_EVERY as usize;

/// One tick as the engine runs it, `read` then `telemetry`: whether the
/// state read and the telemetry each sent anything, and what the telemetry
/// refreshed.
fn tick_with_telemetry<const NB: usize>(
    adapter: &mut Dynamixel<Scripted, NB, N>,
    telemetry: &mut Telemetry<N>,
) -> (bool, usize, Mask<N>) {
    let (state, vitals_sent) = (
        adapter.backend.count(Call::SyncReadState),
        adapter.backend.count(Call::ReadVitals),
    );
    read(adapter, &mut Slot::filled(0.0));
    let state = adapter.backend.count(Call::SyncReadState) > state;
    let (refreshed, faults) = refresh(adapter, telemetry);
    assert_eq!(faults, Faults::default());
    let sent = adapter.backend.count(Call::ReadVitals) - vitals_sent;
    (state, sent, refreshed)
}

#[test]
fn the_bring_up_call_reads_every_register_of_every_actuator() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    powered(&mut adapter)?;
    let warm = Health {
        voltage_dv: 118,
        temperature_c: 41,
        ..HEALTHY
    };
    let overheating = Health {
        error_status: 0x04,
        ..HEALTHY
    };
    adapter
        .backend
        .health(1, id(3), warm)
        .health(0, id(2), overheating);
    let mut telemetry = Telemetry::default();

    let (refreshed, faults) = refresh(&mut adapter, &mut telemetry);

    assert_eq!((refreshed, faults), (Mask::ALL, Faults::default()));
    assert!(vitals_read(&adapter.backend).eq([
        (0, Some(Vital::Voltage), id(0), 2),
        (0, Some(Vital::Temperature), id(0), 2),
        (0, Some(Vital::HardwareError), id(0), 2),
        (1, Some(Vital::Voltage), id(1), 1),
        (1, Some(Vital::Voltage), id(3), 1),
    ]));
    assert_eq!(telemetry.voltage_v(0), Ok(Some(12.0)));
    assert_eq!(telemetry.temperature_c(0), Ok(Some(30.0)));
    assert_eq!(telemetry.hardware_error(0), Ok(Some(0)));
    assert_eq!(telemetry.hardware_error(2), Ok(Some(0x04)));
    assert_eq!(telemetry.voltage_v(3), Ok(Some(11.8)));
    assert_eq!(telemetry.temperature_c(3), Ok(Some(41.0)));
    assert_eq!(telemetry.hardware_error(3), Ok(Some(0)));
    Ok(())
}

#[test]
fn a_tick_sends_no_telemetry_between_due_ticks_and_one_transaction_on_them()
-> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;
    let mut telemetry = Telemetry::default();
    refresh(&mut adapter, &mut telemetry);

    for tick in 0..4 * EVERY {
        let (_, sent, refreshed) = tick_with_telemetry(&mut adapter, &mut telemetry);
        let due = tick % EVERY == EVERY - 1;
        assert_eq!(sent, usize::from(due), "tick {tick}");
        let expected = if due { Mask::ALL } else { Mask::NONE };
        assert_eq!(refreshed, expected, "tick {tick}");
    }
    Ok(())
}

#[test]
fn the_rounds_cycle_each_group_s_registers_then_each_protocol_1_actuator()
-> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    powered(&mut adapter)?;
    let mut telemetry = Telemetry::default();
    refresh(&mut adapter, &mut telemetry);
    adapter.backend.clear_log();
    let (arm, wrist) = (mask(|index| index % 2 == 0), |index| {
        mask(move |at| at == index)
    });

    let mut masks = [Mask::NONE; 6];
    let mut reads = [(0, None, 0, 0); 6];
    let mut taken = 0;
    for _ in 0..7 * EVERY {
        let (_, sent, refreshed) = tick_with_telemetry(&mut adapter, &mut telemetry);
        assert!(sent <= 1);
        let record = vitals_read(&adapter.backend).last();
        if let (1, Some(mask), Some(read), Some(record)) =
            (sent, masks.get_mut(taken), reads.get_mut(taken), record)
        {
            (*mask, *read) = (refreshed, record);
            taken += 1;
        }
        adapter.backend.clear_log();
    }

    assert_eq!(taken, masks.len());
    assert_eq!(masks, [arm, arm, arm, wrist(1), wrist(3), arm]);
    assert_eq!(
        reads,
        [
            (0, Some(Vital::Voltage), id(0), 2),
            (0, Some(Vital::Temperature), id(0), 2),
            (0, Some(Vital::HardwareError), id(0), 2),
            (1, Some(Vital::Voltage), id(1), 1),
            (1, Some(Vital::Voltage), id(3), 1),
            (0, Some(Vital::Voltage), id(0), 2),
        ]
    );
    Ok(())
}

#[test]
fn telemetry_keeps_off_the_ticks_whose_state_read_takes_its_bus() -> Result<(), DynamixelError> {
    let mut adapter = rig(every(X_TABLE, 3));
    powered(&mut adapter)?;
    let mut telemetry = Telemetry::default();
    refresh(&mut adapter, &mut telemetry);

    let mut sent_at = [0; 3];
    let mut taken = 0;
    for tick in 0..3 * EVERY + 10 {
        let (state, sent, _) = tick_with_telemetry(&mut adapter, &mut telemetry);
        assert_eq!(state, tick % 3 == 0, "tick {tick}");
        assert!(
            !(state && sent > 0),
            "tick {tick}: state read and telemetry"
        );
        if let (1, Some(slot)) = (sent, sent_at.get_mut(taken)) {
            *slot = tick;
            taken += 1;
        }
    }

    // Due at 49, 99 and 150; 99 and 150 read the bus, so they wait a tick.
    assert_eq!(sent_at, [EVERY - 1, 2 * EVERY, 3 * EVERY + 1]);
    Ok(())
}

#[test]
fn a_failed_telemetry_read_sets_a_fault_and_keeps_the_values() -> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;
    let mut telemetry = Telemetry::default();
    refresh(&mut adapter, &mut telemetry);
    adapter
        .backend
        .fail(Call::ReadVitals, 0, id(1), BackendError::Timeout)
        .health(
            0,
            id(1),
            Health {
                voltage_dv: 99,
                ..HEALTHY
            },
        );

    let mut last = (Mask::NONE, Faults::default());
    for _ in 0..EVERY {
        read(&mut adapter, &mut Slot::filled(0.0));
        last = refresh(&mut adapter, &mut telemetry);
    }

    let (refreshed, faults) = last;
    assert_eq!(refreshed, mask(|index| index != 1));
    assert_eq!(faults.offline, mask(|index| index == 1));
    assert_eq!((faults.stale, faults.bus_error), (Mask::NONE, Mask::NONE));
    assert_eq!(telemetry.voltage_v(1), Ok(Some(12.0)));
    Ok(())
}

#[test]
fn a_failed_group_telemetry_read_is_a_bus_error_on_the_group_and_refreshes_none()
-> Result<(), DynamixelError> {
    let mut adapter = rig(X_TABLE);
    powered(&mut adapter)?;
    adapter
        .backend
        .fail_port(Call::ReadVitals, 0, BackendError::Io(Wire::Closed));
    let mut telemetry = Telemetry::default();

    let (refreshed, faults) = refresh(&mut adapter, &mut telemetry);

    assert_eq!(refreshed, Mask::NONE);
    assert_eq!(faults.bus_error, Mask::ALL);
    assert_eq!((faults.stale, faults.offline), (Mask::NONE, Mask::NONE));
    assert_eq!(telemetry.temperature_c(0), Ok(None));
    Ok(())
}

#[test]
fn read_only_actuators_are_read_for_telemetry_too() -> Result<(), DynamixelError> {
    let mut adapter = rig(RO_TABLE);
    powered(&mut adapter)?;
    let mut telemetry = Telemetry::default();

    let (refreshed, _) = refresh(&mut adapter, &mut telemetry);

    assert!(refreshed.get(LAST));
    assert_eq!(telemetry.temperature_c(LAST), Ok(Some(30.0)));
    Ok(())
}

#[test]
fn ten_thousand_ticks_of_telemetry_allocate_nothing() -> Result<(), DynamixelError> {
    let mut adapter = bound(MIX_TABLE, &MIX_PLUGINS);
    powered(&mut adapter)?;
    let mut telemetry = Telemetry::default();
    let mut slot = Slot::filled(0.0);
    refresh(&mut adapter, &mut telemetry);

    let counted = allocations(|| {
        for _ in 0..TICKS {
            read(&mut adapter, &mut slot);
            refresh(&mut adapter, &mut telemetry);
        }
    });

    assert_eq!(counted, 0, "{counted} allocations in {TICKS} ticks");
    assert!(adapter.backend.count(Call::ReadVitals) >= TICKS / EVERY);
    Ok(())
}

// ── dxl_rs_v2 migration phase 3: RAM settings at bring-up ───────────────────

/// Gains well inside every Protocol 2 model's range.
const GAINS: Gains = Gains {
    p: 800,
    i: 20,
    d: 4000,
};

/// `table` in order, actuator `row` given `settings`.
fn set<const NB: usize>(table: Table<NB, N>, row: usize, settings: Settings) -> Placement<NB, N> {
    let mut placement = table.in_order();
    placement.settings[row] = settings;
    placement
}

/// `table` of `model` on one bus, read every tick.
const fn chain_of(model: Model) -> Table<1, N> {
    Table {
        buses: one_bus("chain", NonZeroU32::new(1)),
        actuators: chain(model),
    }
}

#[test]
fn configure_writes_the_settings_torque_off_then_reads_each_back() -> Result<(), DynamixelError> {
    let settings = Settings {
        gains: Some(GAINS),
        led: Some(true),
        voltage_limits_dv: None,
        ..Settings::NONE
    };
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), set(X_TABLE, 2, settings));
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.configure()?;

    let ram = |setting| adapter.backend.ram(0, id(2), setting);
    assert_eq!(RamSetting::ALL.map(ram), [GAINS.p, GAINS.i, GAINS.d, 1]);
    assert_eq!(adapter.backend.ram(0, id(1), RamSetting::Led), 0);
    assert!(!adapter.backend.torqued(0, id(2)));
    let order = adapter
        .backend
        .calls()
        .iter()
        .filter(|record| matches!(record.call, Call::WriteRam | Call::ReadRam))
        .map(|record| (record.call, record.id));
    let writes = iter::repeat_n((Call::WriteRam, id(2)), 4);
    assert!(order.eq(writes.chain(iter::repeat_n((Call::ReadRam, id(2)), 4))));
    let last_check = adapter
        .backend
        .calls()
        .iter()
        .rposition(|record| record.call == Call::ReadSetting);
    let first_write = adapter
        .backend
        .calls()
        .iter()
        .position(|record| record.call == Call::WriteRam);
    assert!(
        last_check < first_write,
        "every mode check before any write"
    );
    Ok(())
}

#[test]
fn no_settings_write_nothing() -> Result<(), DynamixelError> {
    let mut adapter = rig(MIX_TABLE);
    configured(&mut adapter)?;

    assert_eq!(adapter.backend.count(Call::WriteRam), 0);
    assert_eq!(adapter.backend.count(Call::ReadRam), 0);
    Ok(())
}

#[test]
fn protocol_1_mx64_gains_are_written_too() -> Result<(), DynamixelError> {
    let gains = Gains {
        p: 32,
        i: 0,
        d: 254,
    };
    let settings = Settings {
        gains: Some(gains),
        led: None,
        voltage_limits_dv: None,
        ..Settings::NONE
    };
    let placement = set(chain_of(Model::Mx64).check(), 0, settings);
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), placement);
    adapter.check(Overrides::default())?;
    mark_open(&mut adapter);

    adapter.configure()?;

    assert_eq!(adapter.backend.ram(0, id(0), RamSetting::DGain), 254);
    assert_eq!(adapter.backend.count(Call::WriteRam), 3);
    Ok(())
}

#[test]
fn a_setting_read_back_other_than_written_is_an_error_naming_the_register() {
    let settings = Settings {
        gains: Some(GAINS),
        led: None,
        voltage_limits_dv: None,
        ..Settings::NONE
    };
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), set(X_TABLE, 1, settings));
    adapter.check(Overrides::default()).unwrap();
    mark_open(&mut adapter);
    adapter.backend.stuck_ram(0, id(1));

    let error = adapter.configure().unwrap_err();

    assert_eq!(
        error,
        DynamixelError::SettingMismatch {
            actuator: 1,
            bus: "x",
            id: id(1),
            register: "P Gain",
            written: GAINS.p,
            read: 0,
        }
    );
    assert_eq!(
        error.to_string(),
        "actuator 1: id 2 on bus x: P Gain read back 0, written 800"
    );
}

#[test]
fn check_refuses_a_setting_the_model_has_no_register_for_before_any_io() {
    let led = Settings {
        gains: None,
        led: Some(true),
        voltage_limits_dv: None,
        ..Settings::NONE
    };
    let gains = Settings {
        gains: Some(GAINS),
        led: None,
        voltage_limits_dv: None,
        ..Settings::NONE
    };
    let cases = [
        (
            chain_of(Model::Xw540T140).check(),
            led,
            Model::Xw540T140,
            "LED",
        ),
        (
            chain_of(Model::Ax12a).check(),
            gains,
            Model::Ax12a,
            "P Gain",
        ),
        (chain_of(Model::Rx28).check(), gains, Model::Rx28, "P Gain"),
    ];
    for (table, settings, model, register) in cases {
        let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), set(table, 3, settings));

        let error = adapter.check(Overrides::default()).err();

        assert_eq!(
            error,
            Some(DynamixelError::NoSettingRegister {
                actuator: 3,
                bus: "chain",
                id: id(3),
                model,
                register,
            })
        );
        assert!(adapter.backend.calls().is_empty());
    }
    let mut adapter = Dynamixel::<_, _, N>::new(
        Scripted::new(),
        set(chain_of(Model::Xw540T140).check(), 0, led),
    );
    let message = adapter.check(Overrides::default()).unwrap_err().to_string();
    assert_eq!(
        message,
        "actuator 0: id 1 on bus chain: the XW540-T140 has no LED register"
    );
}

#[test]
fn check_refuses_a_gain_above_the_model_s_range() {
    let wide = |gains| Settings {
        gains: Some(gains),
        led: None,
        voltage_limits_dv: None,
        ..Settings::NONE
    };
    let mx = set(
        chain_of(Model::Mx64).check(),
        0,
        wide(Gains {
            p: 32,
            i: 255,
            d: 0,
        }),
    );
    let mut adapter = Dynamixel::<_, _, N>::new(Scripted::new(), mx);
    assert_eq!(
        adapter.check(Overrides::default()).err(),
        Some(DynamixelError::SettingRange {
            actuator: 0,
            bus: "chain",
            id: id(0),
            model: Model::Mx64,
            register: "I Gain",
            value: 255,
            max: 254,
        })
    );

    let declared = declared().with_settings([
        Settings::default(),
        wide(Gains { d: 16_384, ..GAINS }),
        Settings::default(),
        Settings::default(),
    ]);
    let mut adapter = Dynamixel::<_, _, N>::declared(Scripted::new(), declared);
    let error = adapter.check(Overrides::default()).unwrap_err();
    assert_eq!(
        error.to_string(),
        "actuator 1: id 2 on bus ARM00001: D Gain 16384 above the XM430-W210 maximum 16383"
    );
    assert!(adapter.backend.calls().is_empty());
}
