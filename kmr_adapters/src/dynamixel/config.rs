//! The Dynamixel hardware table: buses, actuators, models and modes,
//! checked at compile time.
//!
//! Model tables follow the ROBOTIS e-Manual. A bad table fails
//! [`Table::check`] in const evaluation, so it does not build (SRS FR-6.4).

use core::num::NonZeroU32;

use kmr_core::hal::Mask;
use kmr_core::state::JOINTS;

use super::units;

/// Highest firmware id: Protocol 2 reserves 253, 254 is the broadcast id.
const MAX_ID: u8 = 252;

/// Capacity of a [`Message`], in bytes.
const MESSAGE_LEN: usize = 256;

/// Ticks between two reads of a bus whose `read_every` is `None`: a
/// placeholder until the NFR-1 bench, SC-003 (D9, assumed, pending human).
pub const DEFAULT_READ_EVERY: NonZeroU32 = match NonZeroU32::new(4) {
    Some(ticks) => ticks,
    None => NonZeroU32::MIN,
};

/// Dynamixel communication protocol.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    /// Protocol 1.0: sync write, no grouped read (research R12).
    One,
    /// Protocol 2.0.
    Two,
}

/// Control table family: actuators of one series on one bus share a
/// transaction group.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Series {
    /// AX table, the RX-28's and the MX-64's on Protocol 1: every register
    /// the adapter touches has the AX address and size.
    Ax,
    /// X table, the MX-64(2.0)'s too: same address and size for every
    /// register the adapter touches.
    X,
}

/// The actuator models the adapter drives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// AX-12A.
    Ax12a,
    /// RX-28.
    Rx28,
    /// MX-64 on its Protocol 1 firmware. Not checked yet (T071, waiting on
    /// `dxl_rs`): its Torque Ctrl Mode Enable (70), which makes it ignore
    /// Goal Position, and its Model Number, so an AX-12A or RX-28 declared as
    /// an MX-64, or the reverse, is driven at the wrong position scale.
    Mx64,
    /// MX-64(2.0): the MX-64 on its Protocol 2.0 firmware, another control
    /// table.
    Mx64V2,
    /// XM430-W210.
    Xm430W210,
    /// XW430-T200.
    Xw430T200,
    /// XW540-T140.
    Xw540T140,
    /// XW540-T260.
    Xw540T260,
}

impl Model {
    /// The model's name, as ROBOTIS writes it.
    pub(crate) const fn name(self) -> &'static str {
        match self {
            Self::Ax12a => "AX-12A",
            Self::Rx28 => "RX-28",
            Self::Mx64 => "MX-64",
            Self::Mx64V2 => "MX-64(2.0)",
            Self::Xm430W210 => "XM430-W210",
            Self::Xw430T200 => "XW430-T200",
            Self::Xw540T140 => "XW540-T140",
            Self::Xw540T260 => "XW540-T260",
        }
    }

    /// Protocol the model speaks.
    pub const fn protocol(self) -> Protocol {
        match self {
            Self::Ax12a | Self::Rx28 | Self::Mx64 => Protocol::One,
            Self::Mx64V2
            | Self::Xm430W210
            | Self::Xw430T200
            | Self::Xw540T140
            | Self::Xw540T260 => Protocol::Two,
        }
    }

    /// Control table family. `dxl_rs` has no RX-28 table: the RX-28 shares
    /// the AX one, as in v2 (research R7). The Protocol 1 MX-64 shares it
    /// too, so it shares the AX sync write on its bus.
    pub const fn series(self) -> Series {
        match self {
            Self::Ax12a | Self::Rx28 | Self::Mx64 => Series::Ax,
            Self::Mx64V2
            | Self::Xm430W210
            | Self::Xw430T200
            | Self::Xw540T140
            | Self::Xw540T260 => Series::X,
        }
    }

    /// Whether `baud`, in bit/s, is in the model's Baud Rate table. Rates are
    /// the e-Manual's goal rates: the AX `115_200` runs at 117 647 bit/s on
    /// the actuator side, 2.1 % off.
    pub const fn accepts_baud(self, baud: u32) -> bool {
        match self {
            Self::Ax12a | Self::Rx28 => matches!(
                baud,
                9_600
                    | 19_200
                    | 57_600
                    | 115_200
                    | 200_000
                    | 250_000
                    | 400_000
                    | 500_000
                    | 1_000_000
            ),
            Self::Mx64 => matches!(
                baud,
                9_600
                    | 19_200
                    | 57_600
                    | 115_200
                    | 200_000
                    | 250_000
                    | 400_000
                    | 500_000
                    | 1_000_000
                    | 2_000_000
                    | 2_250_000
                    | 2_500_000
                    | 3_000_000
            ),
            Self::Mx64V2
            | Self::Xm430W210
            | Self::Xw430T200
            | Self::Xw540T140
            | Self::Xw540T260 => matches!(
                baud,
                9_600
                    | 57_600
                    | 115_200
                    | 1_000_000
                    | 2_000_000
                    | 3_000_000
                    | 4_000_000
                    | 4_500_000
            ),
        }
    }

    /// Whether the firmware has `mode`. On AX, RX and the Protocol 1 MX-64,
    /// `Velocity` is wheel mode and there is no Goal Current.
    pub const fn supports(self, mode: Mode) -> bool {
        !matches!((self.series(), mode), (Series::Ax, Mode::Current))
    }

    /// CW and CCW Angle Limit that, both at this value, select multi-turn
    /// mode; `None` on a model without one.
    pub(crate) const fn multi_turn_limit(self) -> Option<u32> {
        match self {
            Self::Mx64 => Some(4095),
            Self::Ax12a
            | Self::Rx28
            | Self::Mx64V2
            | Self::Xm430W210
            | Self::Xw430T200
            | Self::Xw540T140
            | Self::Xw540T260 => None,
        }
    }

    /// Input voltage range of the datasheet, `(min, max)` in 0.1 V. Never
    /// the health check, which reads the actuator's own limits: outside this
    /// range, inside those, bring-up warns.
    pub const fn datasheet_voltage_dv(self) -> (u16, u16) {
        match self {
            Self::Ax12a => (90, 120),
            Self::Rx28 => (120, 185),
            Self::Mx64 | Self::Mx64V2 => (100, 148),
            Self::Xm430W210 | Self::Xw430T200 | Self::Xw540T140 | Self::Xw540T260 => (100, 148),
        }
    }

    /// Top of the datasheet operating range, also the default Temperature
    /// Limit, in °C. Never the health check, which reads the actuator's own
    /// Temperature Limit: above this, below that, bring-up warns.
    pub const fn datasheet_max_temperature_c(self) -> u8 {
        match self {
            Self::Ax12a => 70,
            Self::Rx28 | Self::Mx64 | Self::Mx64V2 => 80,
            Self::Xm430W210 | Self::Xw430T200 | Self::Xw540T140 | Self::Xw540T260 => 80,
        }
    }
}

/// Operating mode, named by the desired field it consumes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Goal position, from `Q`.
    Position,
    /// Goal velocity, from `Qd`.
    Velocity,
    /// Goal current, from `Tau`.
    Current,
}

/// Whether the adapter powers an actuator (SRS FR-5.8).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Torque {
    /// Torque on at PostInit; goals are sent.
    #[default]
    On,
    /// Read, never torqued, never written.
    ReadOnly,
}

/// Where 0 rad sits on an actuator (FR-005).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Zero {
    /// The model's centre: 2048 on X and MX-64, 512 on AX and RX.
    Center,
    /// This position register value, inside the model's range.
    Raw(i32),
}

impl Zero {
    /// Zero of every actuator unless `main.rs` overrides it (D8, assumed,
    /// pending human).
    pub const DEFAULT: Self = Self::Center;
}

/// One serial bus.
#[derive(Clone, Copy, Debug)]
pub struct Bus {
    /// Names the bus in errors; a bus plugin binds it by this label (D6).
    pub label: &'static str,
    /// Line rate in bit/s, accepted by every model on the bus.
    pub baud: u32,
    /// Ticks between two reads (FR-009); `None` is [`DEFAULT_READ_EVERY`].
    pub read_every: Option<NonZeroU32>,
}

/// One actuator; its engine index is its position in [`Table::actuators`],
/// or the one [`Table::at`] gives it.
#[derive(Clone, Copy, Debug)]
pub struct Actuator {
    /// Index into the buses.
    pub bus: usize,
    /// Firmware id, `0..=252`, unique on its bus.
    pub id: u8,
    /// Protocol and series derive from it.
    pub model: Model,
    /// Supported by the model.
    pub mode: Mode,
    /// Whether the adapter powers it.
    pub torque: Torque,
    /// Where 0 rad sits.
    pub zero: Zero,
}

/// The Dynamixel hardware of a robot: `NB` buses and `N` actuators, in the
/// model's actuator order (FR-023 (SRS FR-6.1)).
///
/// A preset's hardware half is one; so is a `main.rs` declaration. Build it
/// in a `const` through [`Table::check`].
#[derive(Clone, Copy, Debug)]
pub struct Table<const NB: usize, const N: usize> {
    /// The buses; an actuator names one by index.
    pub buses: [Bus; NB],
    /// The actuators, by engine index.
    pub actuators: [Actuator; N],
}

impl<const NB: usize, const N: usize> Table<NB, N> {
    /// The table, checked where a failure is a compile error naming the
    /// actuator index or the bus label (SRS FR-6.4).
    ///
    /// Rejects no bus, an empty label, a label two buses share, a bus index
    /// out of range, an id above 252, an id taken twice on one bus, a mode or
    /// a baud the model lacks, every mode but `Position` (`dxl_rs` cannot
    /// write the operating mode register, SRS section 8), and a
    /// [`Zero::Raw`] outside the model's position range.
    ///
    /// # Panics
    ///
    /// On a rejected table; in const evaluation, that is the compile error.
    ///
    /// # Examples
    ///
    /// ```
    /// use kmr_adapters::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};
    ///
    /// const fn xm430(id: u8) -> Actuator {
    ///     Actuator {
    ///         bus: 0,
    ///         id,
    ///         model: Model::Xm430W210,
    ///         mode: Mode::Position,
    ///         torque: Torque::On,
    ///         zero: Zero::DEFAULT,
    ///     }
    /// }
    ///
    /// const ARM: Table<1, 2> = Table {
    ///     buses: [Bus { label: "arm", baud: 1_000_000, read_every: None }],
    ///     actuators: [xm430(1), xm430(2)],
    /// }
    /// .check();
    /// ```
    ///
    /// Two actuators on one id do not build:
    ///
    /// ```compile_fail,E0080
    /// # use kmr_adapters::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};
    /// # const fn xm430(id: u8) -> Actuator {
    /// #     Actuator {
    /// #         bus: 0,
    /// #         id,
    /// #         model: Model::Xm430W210,
    /// #         mode: Mode::Position,
    /// #         torque: Torque::On,
    /// #         zero: Zero::DEFAULT,
    /// #     }
    /// # }
    /// const _: Table<1, 2> = Table {
    ///     buses: [Bus { label: "arm", baud: 1_000_000, read_every: None }],
    ///     actuators: [xm430(1), xm430(1)],
    /// }
    /// .check();
    /// ```
    ///
    /// Neither does a `Velocity` actuator:
    ///
    /// ```compile_fail,E0080
    /// # use kmr_adapters::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};
    /// # const fn xm430(id: u8) -> Actuator {
    /// #     Actuator {
    /// #         bus: 0,
    /// #         id,
    /// #         model: Model::Xm430W210,
    /// #         mode: Mode::Position,
    /// #         torque: Torque::On,
    /// #         zero: Zero::DEFAULT,
    /// #     }
    /// # }
    /// const _: Table<1, 1> = Table {
    ///     buses: [Bus { label: "arm", baud: 1_000_000, read_every: None }],
    ///     actuators: [Actuator { mode: Mode::Velocity, ..xm430(1) }],
    /// }
    /// .check();
    /// ```
    pub const fn check(self) -> Self {
        if NB == 0 {
            fail(Message::new().text("no bus declared"));
        }
        check_labels(&self.buses);
        let mut index = 0;
        let mut rest: &[Actuator] = &self.actuators;
        while let [actuator, later @ ..] = rest {
            check_actuator(&self.buses, index, actuator, later);
            index += 1;
            rest = later;
        }
        self
    }
}

/// Overrides of a preset's defaults, chained in a `const` from `main.rs`
/// (FR-026 (SRS FR-6.1)). Each one changes only what it names and returns
/// the table through [`Table::check`]; an unknown label or an index past the
/// table, in a group too, is a compile error naming it.
///
/// # Examples
///
/// A chain changes the values it names and no other:
///
/// ```
/// use kmr_adapters::dynamixel::{Table, Torque, Zero};
/// use kmr_adapters::presets::XmBench;
///
/// const BENCH: Table<1, 4> = XmBench::HARDWARE
///     .baud("bus0", 3_000_000)
///     .read_every("bus0", 2)
///     .id(2, 7)
///     .read_only(3)
///     .zero_group(&[0, 1], 1024);
///
/// let bus = BENCH.buses[0];
/// assert_eq!((bus.label, bus.baud), ("bus0", 3_000_000));
/// assert_eq!(bus.read_every.map(|ticks| ticks.get()), Some(2));
/// let ids = BENCH.actuators.map(|actuator| actuator.id);
/// assert_eq!(ids, [1, 2, 7, 4]);
/// let torque = BENCH.actuators.map(|actuator| actuator.torque);
/// assert_eq!(torque, [Torque::On, Torque::On, Torque::On, Torque::ReadOnly]);
/// let zero = BENCH.actuators.map(|actuator| actuator.zero);
/// assert_eq!(zero, [Zero::Raw(1024), Zero::Raw(1024), Zero::Center, Zero::Center]);
/// for (new, old) in BENCH.actuators.iter().zip(&XmBench::HARDWARE.actuators) {
///     assert_eq!((new.bus, new.model, new.mode), (old.bus, old.model, old.mode));
/// }
/// ```
///
/// Each of these tables fails to build (SC-006 (SRS FR-6.4)). An id taken
/// twice on one bus:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::Table, presets::XmBench};
/// const _: Table<1, 4> = XmBench::HARDWARE.id(1, 1);
/// ```
///
/// Id 253, which Protocol 2 reserves:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::Table, presets::XmBench};
/// const _: Table<1, 4> = XmBench::HARDWARE.id(0, 253);
/// ```
///
/// A label the table does not have:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::Table, presets::XmBench};
/// const _: Table<1, 4> = XmBench::HARDWARE.baud("bus1", 1_000_000);
/// ```
///
/// A baud the XM430-W210 does not take:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::Table, presets::XmBench};
/// const _: Table<1, 4> = XmBench::HARDWARE.baud("bus0", 500_000);
/// ```
///
/// A mode other than `Position`, which `dxl_rs` cannot write:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::{Mode, Table}, presets::XmBench};
/// const _: Table<1, 4> = XmBench::HARDWARE.mode(0, Mode::Velocity);
/// ```
///
/// A bus read every 0 ticks:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::Table, presets::XmBench};
/// const _: Table<1, 4> = XmBench::HARDWARE.read_every("bus0", 0);
/// ```
///
/// A zero outside the position range, 0 to 4095 on X:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::Table, presets::XmBench};
/// const _: Table<1, 4> = XmBench::HARDWARE.zero(1, 4096);
/// ```
///
/// An empty label:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::{Bus, Table}, presets::XmBench};
/// const BENCH: Table<1, 4> = XmBench::HARDWARE;
/// const _: Table<1, 4> = Table {
///     buses: [Bus { label: "", ..BENCH.buses[0] }],
///     ..BENCH
/// }
/// .check();
/// ```
///
/// No bus:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::Table, presets::XmBench};
/// const _: Table<0, 4> = Table {
///     buses: [],
///     actuators: XmBench::HARDWARE.actuators,
/// }
/// .check();
/// ```
///
/// A group naming an index past the table:
///
/// ```compile_fail,E0080
/// # use kmr_adapters::{dynamixel::Table, presets::XmBench};
/// const _: Table<1, 4> = XmBench::HARDWARE.read_only_group(&[1, 4]);
/// ```
///
/// The count check of `robot!` (FR-024 (SRS FR-6.1, FR-6.4)): a table of
/// four actuators on a robot of three is E0308, "expected an array with a
/// size of 3, found one with a size of 4":
///
/// ```compile_fail,E0308
/// # use kmr_adapters::{dynamixel::Actuator, presets::XmBench};
/// const _: &[Actuator; 3] = &XmBench::HARDWARE.actuators;
/// ```
impl<const NB: usize, const N: usize> Table<NB, N> {
    /// Bus `bus` runs at `baud` bit/s, which every model on it must accept.
    ///
    /// # Panics
    ///
    /// On an unknown label or a rejected table; in const evaluation, that is
    /// the compile error.
    pub const fn baud(mut self, bus: &str, baud: u32) -> Self {
        bus_mut(&mut self.buses, bus).baud = baud;
        self.check()
    }

    /// Bus `bus` is read every `ticks` ticks (FR-009).
    ///
    /// # Panics
    ///
    /// On an unknown label, on `ticks` = 0 and on a rejected table; in const
    /// evaluation, that is the compile error.
    pub const fn read_every(mut self, bus: &str, ticks: u32) -> Self {
        let Some(ticks) = NonZeroU32::new(ticks) else {
            fail(
                Message::new()
                    .text("bus ")
                    .text(bus)
                    .text(": read_every 0, a bus is read every 1 tick or more"),
            );
        };
        bus_mut(&mut self.buses, bus).read_every = Some(ticks);
        self.check()
    }

    /// Actuator `actuator` answers to firmware id `id`.
    ///
    /// # Panics
    ///
    /// On an index past the table or a rejected table; in const evaluation,
    /// that is the compile error.
    pub const fn id(mut self, actuator: usize, id: u8) -> Self {
        actuator_mut(&mut self.actuators, actuator).id = id;
        self.check()
    }

    /// Actuator `actuator` runs in `mode`.
    ///
    /// # Panics
    ///
    /// On an index past the table or a rejected table, every mode but
    /// `Position` included; in const evaluation, that is the compile error.
    pub const fn mode(mut self, actuator: usize, mode: Mode) -> Self {
        actuator_mut(&mut self.actuators, actuator).mode = mode;
        self.check()
    }

    /// Actuator `actuator` is read, never powered, never written (SRS
    /// FR-5.8).
    ///
    /// # Panics
    ///
    /// On an index past the table; in const evaluation, that is the compile
    /// error.
    pub const fn read_only(mut self, actuator: usize) -> Self {
        actuator_mut(&mut self.actuators, actuator).torque = Torque::ReadOnly;
        self.check()
    }

    /// 0 rad of actuator `actuator` sits at position register value `raw`
    /// (FR-005).
    ///
    /// # Panics
    ///
    /// On an index past the table, and on a `raw` outside the model's
    /// position range; in const evaluation, that is the compile error.
    pub const fn zero(mut self, actuator: usize, raw: i32) -> Self {
        actuator_mut(&mut self.actuators, actuator).zero = Zero::Raw(raw);
        self.check()
    }

    /// [`Table::read_only`] for every actuator of `group`.
    ///
    /// # Panics
    ///
    /// As [`Table::read_only`], for any index of `group`.
    pub const fn read_only_group<const K: usize>(mut self, group: &[usize; K]) -> Self {
        let mut rest: &[usize] = group;
        while let [actuator, later @ ..] = rest {
            actuator_mut(&mut self.actuators, *actuator).torque = Torque::ReadOnly;
            rest = later;
        }
        self.check()
    }

    /// [`Table::mode`] for every actuator of `group`.
    ///
    /// # Panics
    ///
    /// As [`Table::mode`], for any index of `group`.
    pub const fn mode_group<const K: usize>(mut self, group: &[usize; K], mode: Mode) -> Self {
        let mut rest: &[usize] = group;
        while let [actuator, later @ ..] = rest {
            actuator_mut(&mut self.actuators, *actuator).mode = mode;
            rest = later;
        }
        self.check()
    }

    /// [`Table::zero`] for every actuator of `group`, all at one `raw`.
    ///
    /// # Panics
    ///
    /// As [`Table::zero`], for any index of `group`.
    pub const fn zero_group<const K: usize>(mut self, group: &[usize; K], raw: i32) -> Self {
        let mut rest: &[usize] = group;
        while let [actuator, later @ ..] = rest {
            actuator_mut(&mut self.actuators, *actuator).zero = Zero::Raw(raw);
            rest = later;
        }
        self.check()
    }
}

/// The bus labelled `label`; a compile error naming it if there is none.
const fn bus_mut<'a>(buses: &'a mut [Bus], label: &str) -> &'a mut Bus {
    let Some(index) = find_label(buses, label) else {
        fail(
            Message::new()
                .text("no bus labelled ")
                .text(label)
                .text(" in the table"),
        );
    };
    match buses.split_at_mut_checked(index) {
        Some((_, [bus, ..])) => bus,
        _ => fail(Message::new().text("no bus labelled ").text(label)),
    }
}

/// Actuator `index`; a compile error naming it past the table.
const fn actuator_mut(actuators: &mut [Actuator], index: usize) -> &mut Actuator {
    let len = actuators.len();
    match actuators.split_at_mut_checked(index) {
        Some((_, [actuator, ..])) => actuator,
        _ => fail(
            Message::actuator(index)
                .text("past the table's ")
                .number(len)
                .text(" actuators"),
        ),
    }
}

/// A checked [`Table`] placed in the robot: the engine index of each of its
/// actuators, in table order. The Dynamixel member owns these indices and no
/// other (SRS FR-5.1). Built through [`Table::at`] or [`Table::in_order`].
#[derive(Clone, Copy, Debug)]
pub struct Placement<const NB: usize, const M: usize> {
    /// The table, checked.
    pub(crate) table: Table<NB, M>,
    /// Engine index of each actuator of `table`, distinct, below `JOINTS`.
    pub(crate) at: [usize; M],
}

impl<const NB: usize, const M: usize> Placement<NB, M> {
    /// The table placed.
    pub const fn table(&self) -> &Table<NB, M> {
        &self.table
    }

    /// Engine index of each actuator of the table, in table order.
    pub const fn indices(&self) -> &[usize; M] {
        &self.at
    }
}

impl<const NB: usize, const M: usize> Table<NB, M> {
    /// The table checked by [`Table::check`], actuator `k` at engine index
    /// `at[k]`: the Dynamixel member of a robot that mixes brands.
    ///
    /// Rejects an index past the robot's actuators and an index two
    /// actuators share.
    ///
    /// # Panics
    ///
    /// On a rejected table or placement; in const evaluation, that is the
    /// compile error.
    ///
    /// # Examples
    ///
    /// ```
    /// use kmr_adapters::dynamixel::{Actuator, Bus, Mode, Model, Placement, Table, Torque, Zero};
    ///
    /// const fn xm430(id: u8) -> Actuator {
    ///     Actuator {
    ///         bus: 0,
    ///         id,
    ///         model: Model::Xm430W210,
    ///         mode: Mode::Position,
    ///         torque: Torque::On,
    ///         zero: Zero::DEFAULT,
    ///     }
    /// }
    ///
    /// const WRIST: Placement<1, 2> = Table {
    ///     buses: [Bus { label: "wrist", baud: 1_000_000, read_every: None }],
    ///     actuators: [xm430(1), xm430(2)],
    /// }
    /// .at([3, 1]);
    /// ```
    ///
    /// Two actuators on one engine index do not build:
    ///
    /// ```compile_fail,E0080
    /// # use kmr_adapters::dynamixel::{Actuator, Bus, Mode, Model, Placement, Table, Torque, Zero};
    /// # const fn xm430(id: u8) -> Actuator {
    /// #     Actuator {
    /// #         bus: 0,
    /// #         id,
    /// #         model: Model::Xm430W210,
    /// #         mode: Mode::Position,
    /// #         torque: Torque::On,
    /// #         zero: Zero::DEFAULT,
    /// #     }
    /// # }
    /// const _: Placement<1, 2> = Table {
    ///     buses: [Bus { label: "wrist", baud: 1_000_000, read_every: None }],
    ///     actuators: [xm430(1), xm430(2)],
    /// }
    /// .at([1, 1]);
    /// ```
    pub const fn at(self, at: [usize; M]) -> Placement<NB, M> {
        let table = self.check();
        let mut index = 0;
        let mut rest: &[usize] = &at;
        while let [engine, later @ ..] = rest {
            if *engine >= JOINTS {
                fail(
                    Message::actuator(index)
                        .text("engine index ")
                        .number(*engine)
                        .text(" past the robot's ")
                        .number(JOINTS)
                        .text(" actuators"),
                );
            }
            if let Some(offset) = position(later, *engine) {
                fail(
                    Message::actuator(index + 1 + offset)
                        .text("engine index ")
                        .number(*engine)
                        .text(" already taken by actuator ")
                        .number(index),
                );
            }
            index += 1;
            rest = later;
        }
        Placement { table, at }
    }

    /// The table checked by [`Table::check`], actuator `k` at engine index
    /// `k`: a one-brand robot, in the model's actuator order.
    ///
    /// # Panics
    ///
    /// On a rejected table, and on a table of more actuators than the
    /// robot; in const evaluation, that is the compile error.
    pub const fn in_order(self) -> Placement<NB, M> {
        let mut at = [0; M];
        let mut index = 0;
        while index < M {
            at[index] = index;
            index += 1;
        }
        self.at(at)
    }
}

/// The actuators of a [`Placement`] by engine index, `None` at the indices
/// the member does not own.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Placed([Option<Actuator>; JOINTS]);

impl Placed {
    /// The actuators of `placement`, each at its engine index.
    pub(crate) const fn new<const NB: usize, const M: usize>(placement: &Placement<NB, M>) -> Self {
        let mut slots = [None; JOINTS];
        let mut rest: &[Actuator] = &placement.table.actuators;
        let mut at: &[usize] = &placement.at;
        while let ([actuator, more @ ..], [index, later @ ..]) = (rest, at) {
            if let Some((_, [slot, ..])) = slots.split_at_mut_checked(*index) {
                *slot = Some(*actuator);
            }
            rest = more;
            at = later;
        }
        Self(slots)
    }

    /// The actuator at engine index `index`, if the member owns it.
    pub(crate) fn get(&self, index: usize) -> Option<&Actuator> {
        self.0.get(index).and_then(Option::as_ref)
    }

    /// The owned actuators with their engine indices, ascending.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (usize, &Actuator)> {
        self.0
            .iter()
            .enumerate()
            .filter_map(|(index, slot)| slot.as_ref().map(|actuator| (index, actuator)))
    }

    /// The engine indices the member owns.
    pub(crate) fn owned(&self) -> Mask {
        let mut owned = Mask::NONE;
        self.iter().for_each(|(index, _)| owned.set(index));
        owned
    }
}

/// Position in `indices` of the first `index`.
const fn position(indices: &[usize], index: usize) -> Option<usize> {
    let mut found = 0;
    let mut rest = indices;
    while let [first, tail @ ..] = rest {
        if *first == index {
            return Some(found);
        }
        found += 1;
        rest = tail;
    }
    None
}

/// Rejects an empty label, and a label a later bus takes again.
const fn check_labels(buses: &[Bus]) {
    let mut index = 0;
    let mut rest = buses;
    while let [bus, later @ ..] = rest {
        if bus.label.is_empty() {
            fail(Message::bus(index).text("empty label"));
        }
        if let Some(offset) = find_label(later, bus.label) {
            fail(
                Message::bus(index + 1 + offset)
                    .text("label ")
                    .text(bus.label)
                    .text(" already used by bus ")
                    .number(index),
            );
        }
        index += 1;
        rest = later;
    }
}

/// Position in `buses` of the first one labelled `label`.
const fn find_label(buses: &[Bus], label: &str) -> Option<usize> {
    let mut position = 0;
    let mut rest = buses;
    while let [bus, tail @ ..] = rest {
        if same(bus.label.as_bytes(), label.as_bytes()) {
            return Some(position);
        }
        position += 1;
        rest = tail;
    }
    None
}

/// Whether `a` and `b` hold the same bytes.
const fn same(a: &[u8], b: &[u8]) -> bool {
    let (mut a, mut b) = (a, b);
    loop {
        match (a, b) {
            ([], []) => return true,
            ([x, a_tail @ ..], [y, b_tail @ ..]) if *x == *y => {
                a = a_tail;
                b = b_tail;
            }
            _ => return false,
        }
    }
}

/// Checks actuator `index`; `later` are the actuators after it, where a
/// repeated id is reported.
const fn check_actuator(buses: &[Bus], index: usize, actuator: &Actuator, later: &[Actuator]) {
    let Some(bus) = bus_at(buses, actuator.bus) else {
        fail(
            Message::actuator(index)
                .text("bus ")
                .number(actuator.bus)
                .text(" out of range, ")
                .number(buses.len())
                .text(" declared"),
        );
    };
    if actuator.id > MAX_ID {
        fail(
            Message::actuator(index)
                .text("id ")
                .number(actuator.id as usize)
                .text(" above 252"),
        );
    }
    if let Some(offset) = find(later, actuator.bus, actuator.id) {
        fail(
            Message::actuator(index + 1 + offset)
                .text("id ")
                .number(actuator.id as usize)
                .text(" already used on bus ")
                .text(bus.label)
                .text(" by actuator ")
                .number(index),
        );
    }
    if !actuator.model.supports(actuator.mode) {
        fail(Message::actuator(index).text("mode not supported by the model"));
    }
    if !actuator.model.accepts_baud(bus.baud) {
        fail(
            Message::actuator(index)
                .text(actuator.model.name())
                .text(" does not accept ")
                .number(bus.baud as usize)
                .text(" baud on bus ")
                .text(bus.label),
        );
    }
    if !matches!(actuator.mode, Mode::Position) {
        fail(Message::actuator(index).text(
            "SRS section 8: Dynamixel mode register cannot be written, dxl_rs has no write for it",
        ));
    }
    let max = units::max_position(actuator.model);
    if let Zero::Raw(raw) = actuator.zero
        && (raw < 0 || raw as u32 > max)
    {
        fail(
            Message::actuator(index)
                .text("zero outside 0..=")
                .number(max as usize)
                .text(", the position range of ")
                .text(actuator.model.name()),
        );
    }
}

/// The bus at `index`, if declared.
const fn bus_at(buses: &[Bus], index: usize) -> Option<&Bus> {
    match buses.split_at_checked(index) {
        Some((_, [bus, ..])) => Some(bus),
        _ => None,
    }
}

/// Position in `actuators` of the first one on `bus` with `id`.
const fn find(actuators: &[Actuator], bus: usize, id: u8) -> Option<usize> {
    let mut position = 0;
    let mut rest = actuators;
    while let [actuator, tail @ ..] = rest {
        if actuator.bus == bus && actuator.id == id {
            return Some(position);
        }
        position += 1;
        rest = tail;
    }
    None
}

/// Stops evaluation with `message`.
const fn fail(message: Message) -> ! {
    panic!("{}", message.as_str())
}

/// A panic message built in const evaluation, where `format!` is not
/// available. Text past the capacity is dropped.
struct Message {
    bytes: [u8; MESSAGE_LEN],
    len: usize,
}

impl Message {
    /// An empty message.
    const fn new() -> Self {
        Self {
            bytes: [0; MESSAGE_LEN],
            len: 0,
        }
    }

    /// A message about the actuator at `index`.
    const fn actuator(index: usize) -> Self {
        Self::new().text("actuator ").number(index).text(": ")
    }

    /// A message about the bus at `index`.
    const fn bus(index: usize) -> Self {
        Self::new().text("bus ").number(index).text(": ")
    }

    /// Appends `text`.
    const fn text(mut self, text: &str) -> Self {
        let mut rest = text.as_bytes();
        while let [byte, tail @ ..] = rest {
            self = self.byte(*byte);
            rest = tail;
        }
        self
    }

    /// Appends `number` in decimal.
    const fn number(mut self, number: usize) -> Self {
        let mut scale = 1;
        while number / scale >= 10 {
            scale *= 10;
        }
        while scale > 0 {
            self = self.byte(b'0' + (number / scale % 10) as u8);
            scale /= 10;
        }
        self
    }

    /// Appends one byte.
    const fn byte(mut self, byte: u8) -> Self {
        if let Some((_, [slot, ..])) = self.bytes.split_at_mut_checked(self.len) {
            *slot = byte;
            self.len += 1;
        }
        self
    }

    /// The text so far.
    const fn as_str(&self) -> &str {
        let text = match self.bytes.split_at_checked(self.len) {
            Some((text, _)) => text,
            None => &[],
        };
        match core::str::from_utf8(text) {
            Ok(text) => text,
            Err(_) => "invalid Dynamixel hardware table",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODELS: [Model; 8] = [
        Model::Ax12a,
        Model::Rx28,
        Model::Mx64,
        Model::Mx64V2,
        Model::Xm430W210,
        Model::Xw430T200,
        Model::Xw540T140,
        Model::Xw540T260,
    ];

    const BUS: Bus = Bus {
        label: "bus0",
        baud: 1_000_000,
        read_every: NonZeroU32::new(1),
    };

    /// Four actuators of `model` on bus 0, ids `1..=4`, in position mode.
    fn chain(model: Model) -> [Actuator; 4] {
        core::array::from_fn(|index| Actuator {
            bus: 0,
            id: u8::try_from(index + 1).unwrap_or(u8::MAX),
            model,
            mode: Mode::Position,
            torque: Torque::On,
            zero: Zero::DEFAULT,
        })
    }

    /// A one-bus table of `actuators`.
    fn table(actuators: [Actuator; 4]) -> Table<1, 4> {
        Table {
            buses: [BUS],
            actuators,
        }
    }

    #[test]
    fn protocol_one_is_exactly_the_ax_series() {
        for model in MODELS {
            assert_eq!(
                model.protocol() == Protocol::One,
                model.series() == Series::Ax,
                "{model:?}"
            );
        }
        assert_eq!(Model::Rx28.series(), Series::Ax);
        assert_eq!(Model::Mx64.series(), Series::Ax);
        assert_eq!(Model::Mx64V2.series(), Series::X);
    }

    #[test]
    fn baud_tables_follow_the_series() {
        for model in MODELS {
            assert!(model.accepts_baud(57_600), "{model:?}");
            assert!(model.accepts_baud(1_000_000), "{model:?}");
            assert!(!model.accepts_baud(117_647), "{model:?}");
        }
        assert!(Model::Ax12a.accepts_baud(500_000));
        assert!(Model::Rx28.accepts_baud(19_200));
        assert!(!Model::Rx28.accepts_baud(2_000_000));
        assert!(Model::Xw540T260.accepts_baud(4_500_000));
        assert!(!Model::Xm430W210.accepts_baud(500_000));
        for baud in [2_000_000, 2_250_000, 2_500_000, 3_000_000, 400_000] {
            assert!(Model::Mx64.accepts_baud(baud), "{baud}");
        }
        assert!(!Model::Mx64.accepts_baud(4_000_000));
        assert!(Model::Mx64V2.accepts_baud(4_500_000));
        assert!(!Model::Mx64V2.accepts_baud(500_000));
    }

    #[test]
    fn only_the_x_series_has_current_control() {
        for model in MODELS {
            assert!(model.supports(Mode::Position), "{model:?}");
            assert!(model.supports(Mode::Velocity), "{model:?}");
            assert_eq!(
                model.supports(Mode::Current),
                model.series() == Series::X,
                "{model:?}"
            );
        }
    }

    #[test]
    fn datasheet_ranges_follow_the_specifications() {
        for model in MODELS {
            let (min, max) = model.datasheet_voltage_dv();
            assert!(min < max, "{model:?}");
        }
        assert_eq!(Model::Ax12a.datasheet_voltage_dv(), (90, 120));
        assert_eq!(Model::Rx28.datasheet_voltage_dv(), (120, 185));
        assert_eq!(Model::Xw540T140.datasheet_voltage_dv(), (100, 148));
        assert_eq!(Model::Ax12a.datasheet_max_temperature_c(), 70);
        assert_eq!(Model::Rx28.datasheet_max_temperature_c(), 80);
        assert_eq!(Model::Xm430W210.datasheet_max_temperature_c(), 80);
        for model in [Model::Mx64, Model::Mx64V2] {
            assert_eq!(model.datasheet_voltage_dv(), (100, 148), "{model:?}");
            assert_eq!(model.datasheet_max_temperature_c(), 80, "{model:?}");
        }
    }

    #[test]
    fn a_position_chain_passes() {
        table(chain(Model::Xm430W210)).check();
        table(chain(Model::Ax12a)).check();
        table(chain(Model::Mx64)).check();
        table(chain(Model::Mx64V2)).check();
    }

    #[test]
    fn the_same_id_on_two_buses_passes() {
        let mut actuators = chain(Model::Xm430W210);
        actuators[1].bus = 1;
        actuators[1].id = actuators[0].id;
        let buses = [
            BUS,
            Bus {
                label: "bus1",
                ..BUS
            },
        ];
        Table { buses, actuators }.check();
    }

    #[test]
    #[should_panic(expected = "no bus declared")]
    fn no_bus_is_rejected() {
        Table {
            buses: [],
            actuators: chain(Model::Xm430W210),
        }
        .check();
    }

    #[test]
    #[should_panic(expected = "bus 0: empty label")]
    fn an_empty_label_is_rejected() {
        Table {
            buses: [Bus { label: "", ..BUS }],
            actuators: chain(Model::Xm430W210),
        }
        .check();
    }

    #[test]
    #[should_panic(expected = "bus 2: label bus0 already used by bus 0")]
    fn a_repeated_label_names_both_buses() {
        let buses = [
            BUS,
            Bus {
                label: "bus1",
                ..BUS
            },
            BUS,
        ];
        Table {
            buses,
            actuators: chain(Model::Xm430W210),
        }
        .check();
    }

    #[test]
    #[should_panic(expected = "actuator 1: bus 3 out of range, 1 declared")]
    fn a_bus_out_of_range_is_rejected() {
        let mut actuators = chain(Model::Xm430W210);
        actuators[1].bus = 3;
        table(actuators).check();
    }

    #[test]
    #[should_panic(expected = "actuator 0: id 253 above 252")]
    fn a_reserved_id_is_rejected() {
        let mut actuators = chain(Model::Xm430W210);
        actuators[0].id = 253;
        table(actuators).check();
    }

    #[test]
    #[should_panic(expected = "actuator 1: id 1 already used on bus bus0 by actuator 0")]
    fn a_repeated_id_names_both_actuators() {
        let mut actuators = chain(Model::Xm430W210);
        actuators[1].id = actuators[0].id;
        table(actuators).check();
    }

    #[test]
    #[should_panic(expected = "actuator 1: mode not supported by the model")]
    fn current_on_an_ax_is_rejected() {
        let mut actuators = chain(Model::Ax12a);
        actuators[1].mode = Mode::Current;
        table(actuators).check();
    }

    #[test]
    #[should_panic(expected = "actuator 0: RX-28 does not accept 2000000 baud on bus bus0")]
    fn a_baud_the_model_lacks_names_the_actuator_and_the_model() {
        Table {
            buses: [Bus {
                baud: 2_000_000,
                ..BUS
            }],
            actuators: chain(Model::Rx28),
        }
        .check();
    }

    #[test]
    #[should_panic(expected = "actuator 1: SRS section 8: Dynamixel mode register")]
    fn velocity_is_rejected_until_dxl_rs_writes_the_mode() {
        let mut actuators = chain(Model::Xm430W210);
        actuators[1].mode = Mode::Velocity;
        table(actuators).check();
    }

    #[test]
    fn a_raw_zero_inside_the_position_range_passes() {
        let mut actuators = chain(Model::Xm430W210);
        actuators[0].zero = Zero::Raw(0);
        actuators[1].zero = Zero::Raw(4095);
        table(actuators).check();
        let mut actuators = chain(Model::Ax12a);
        actuators[2].zero = Zero::Raw(1023);
        table(actuators).check();
    }

    #[test]
    #[should_panic(
        expected = "actuator 1: zero outside 0..=4095, the position range of XM430-W210"
    )]
    fn a_raw_zero_past_the_position_range_is_rejected() {
        let mut actuators = chain(Model::Xm430W210);
        actuators[1].zero = Zero::Raw(4096);
        table(actuators).check();
    }

    #[test]
    #[should_panic(expected = "actuator 3: zero outside 0..=1023, the position range of AX-12A")]
    fn a_negative_raw_zero_is_rejected() {
        let mut actuators = chain(Model::Ax12a);
        actuators[3].zero = Zero::Raw(-1);
        table(actuators).check();
    }

    #[test]
    fn a_placement_keeps_each_engine_index() {
        let placed = table(chain(Model::Xm430W210)).at([2, 0, 3, 1]);
        assert_eq!(placed.indices(), &[2, 0, 3, 1]);
        assert_eq!(placed.table().actuators[1].id, 2);
        assert_eq!(
            table(chain(Model::Ax12a)).in_order().indices(),
            &[0, 1, 2, 3]
        );
    }

    #[test]
    #[should_panic(expected = "actuator 1: engine index 9999 past the robot's")]
    fn an_engine_index_past_the_robot_is_rejected() {
        table(chain(Model::Xm430W210)).at([0, 9999, 2, 3]);
    }

    #[test]
    #[should_panic(expected = "actuator 3: engine index 1 already taken by actuator 1")]
    fn an_engine_index_taken_twice_names_both_actuators() {
        table(chain(Model::Xm430W210)).at([0, 1, 2, 1]);
    }

    #[test]
    #[should_panic(expected = "actuator 1: id 1 already used")]
    fn a_placement_checks_its_table() {
        let mut actuators = chain(Model::Xm430W210);
        actuators[1].id = actuators[0].id;
        table(actuators).in_order();
    }

    #[test]
    fn group_overrides_change_only_their_members() {
        let table = table(chain(Model::Xm430W210))
            .read_only_group(&[0, 2])
            .mode_group(&[1, 3], Mode::Position)
            .zero_group(&[3], 100)
            .zero(1, 4095);
        let torque = table.actuators.map(|actuator| actuator.torque);
        assert_eq!(
            torque,
            [Torque::ReadOnly, Torque::On, Torque::ReadOnly, Torque::On]
        );
        let zero = table.actuators.map(|actuator| actuator.zero);
        assert_eq!(
            zero,
            [Zero::Center, Zero::Raw(4095), Zero::Center, Zero::Raw(100)]
        );
        assert!(
            table
                .actuators
                .iter()
                .all(|actuator| actuator.mode == Mode::Position)
        );
    }

    #[test]
    fn an_empty_group_changes_nothing() {
        let table = table(chain(Model::Ax12a)).read_only_group(&[]);
        assert!(
            table
                .actuators
                .iter()
                .all(|actuator| actuator.torque == Torque::On)
        );
    }

    #[test]
    fn bus_overrides_find_the_bus_by_label() {
        let buses = [
            BUS,
            Bus {
                label: "bus1",
                ..BUS
            },
        ];
        let table = Table {
            buses,
            actuators: chain(Model::Xm430W210),
        }
        .baud("bus1", 57_600)
        .read_every("bus1", 5);
        assert_eq!(
            (table.buses[0].baud, table.buses[1].baud),
            (1_000_000, 57_600)
        );
        assert_eq!(table.buses[0].read_every, NonZeroU32::new(1));
        assert_eq!(table.buses[1].read_every, NonZeroU32::new(5));
    }

    #[test]
    #[should_panic(expected = "no bus labelled arm in the table")]
    fn read_every_of_an_unknown_label_is_rejected() {
        table(chain(Model::Xm430W210)).read_every("arm", 2);
    }

    #[test]
    #[should_panic(expected = "bus bus0: read_every 0")]
    fn read_every_zero_is_rejected() {
        table(chain(Model::Xm430W210)).read_every("bus0", 0);
    }

    #[test]
    #[should_panic(expected = "actuator 4: past the table's 4 actuators")]
    fn an_override_past_the_table_names_the_index() {
        table(chain(Model::Xm430W210)).read_only(4);
    }

    #[test]
    #[should_panic(expected = "actuator 7: past the table's 4 actuators")]
    fn a_group_index_past_the_table_names_the_index() {
        table(chain(Model::Xm430W210)).zero_group(&[0, 7], 10);
    }

    #[test]
    #[should_panic(expected = "actuator 2: zero outside 0..=1023")]
    fn a_group_zero_outside_the_range_is_rejected() {
        table(chain(Model::Ax12a)).zero_group(&[2, 3], 2048);
    }

    #[test]
    fn a_long_message_is_cut_at_the_capacity() {
        let mut message = Message::new();
        for _ in 0..=MESSAGE_LEN {
            message = message.text("x");
        }
        assert_eq!(message.as_str().len(), MESSAGE_LEN);
    }
}
