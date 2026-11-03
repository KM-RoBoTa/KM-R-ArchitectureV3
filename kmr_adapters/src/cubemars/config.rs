//! The CubeMars hardware table: CAN interfaces and AK60-6 actuators with
//! their fixed gains, checked at compile time (SRS FR-6.4).

use kmr_core::hal::{MASK_WORDS, Mask};

use super::frame::{KD_MAX, KP_MAX};
use crate::dynamixel::{Message, fail, position, same};

/// Position gain of an actuator whose table gives none: cubemars_rs's, the
/// GeckoLeg's.
pub const DEFAULT_KP: f32 = 150.0;
/// Velocity gain of an actuator whose table gives none: cubemars_rs's.
pub const DEFAULT_KD: f32 = 1.0;

/// Highest CAN id the table takes: an 11-bit standard id, and `data[0]` of
/// the reply carries it in one byte.
const MAX_ID: u8 = 0x7F;

/// One CAN interface, bound by a bus plugin `Bus::by_dev("can0")`.
#[derive(Clone, Copy, Debug)]
pub struct Bus {
    /// Names the bus in errors; a bus plugin binds it by this label.
    pub label: &'static str,
}

/// One AK60-6 in MIT mode; its engine index is its position in
/// [`Table::actuators`], or the one [`Table::at`] gives it.
#[derive(Clone, Copy, Debug)]
pub struct Actuator {
    /// Index into the buses.
    pub bus: usize,
    /// CAN id, `1..=127`, unique on its bus.
    pub id: u8,
    /// Position gain sent on a tick that does not write `Kp`, `0..=500`.
    pub kp: f32,
    /// Velocity gain sent on a tick that does not write `Kd`, `0..=5`.
    pub kd: f32,
}

impl Actuator {
    /// An AK60-6 with id `id` on bus `bus`, gains [`DEFAULT_KP`] and
    /// [`DEFAULT_KD`].
    pub const fn ak60_6(bus: usize, id: u8) -> Self {
        Self {
            bus,
            id,
            kp: DEFAULT_KP,
            kd: DEFAULT_KD,
        }
    }
}

/// The CubeMars hardware of a robot: `NB` CAN interfaces and `N` actuators,
/// in the model's actuator order. Build it in a `const` through
/// [`Table::check`].
#[derive(Clone, Copy, Debug)]
pub struct Table<const NB: usize, const N: usize> {
    /// The interfaces; an actuator names one by index.
    pub buses: [Bus; NB],
    /// The actuators, by engine index.
    pub actuators: [Actuator; N],
}

impl<const NB: usize, const N: usize> Table<NB, N> {
    /// The table, checked where a failure is a compile error naming the
    /// actuator index or the bus label.
    ///
    /// Rejects no bus, an empty label, a label two buses share, a bus index
    /// out of range, an id of 0 or above 127, an id taken twice on one bus,
    /// and a gain outside the frame's range.
    ///
    /// # Panics
    ///
    /// On a rejected table; in const evaluation, that is the compile error.
    ///
    /// # Examples
    ///
    /// ```
    /// use kmr_adapters::cubemars::{Actuator, Bus, Table};
    ///
    /// const LEG: Table<1, 3> = Table {
    ///     buses: [Bus { label: "leg" }],
    ///     actuators: [Actuator::ak60_6(0, 1), Actuator::ak60_6(0, 2), Actuator::ak60_6(0, 3)],
    /// }
    /// .check();
    /// ```
    ///
    /// An id taken twice on one bus does not build:
    ///
    /// ```compile_fail,E0080
    /// # use kmr_adapters::cubemars::{Actuator, Bus, Table};
    /// const _: Table<1, 2> = Table {
    ///     buses: [Bus { label: "leg" }],
    ///     actuators: [Actuator::ak60_6(0, 1), Actuator::ak60_6(0, 1)],
    /// }
    /// .check();
    /// ```
    ///
    /// Nor does a gain the frame cannot carry:
    ///
    /// ```compile_fail,E0080
    /// # use kmr_adapters::cubemars::{Actuator, Bus, Table};
    /// const _: Table<1, 1> = Table {
    ///     buses: [Bus { label: "leg" }],
    ///     actuators: [Actuator::ak60_6(0, 1)],
    /// }
    /// .gains(0, 600.0, 1.0)
    /// .check();
    /// ```
    pub const fn check(self) -> Self {
        if NB == 0 {
            fail(Message::bus(0).text("a CubeMars table needs one CAN interface at least"));
        }
        let mut index = 0;
        let mut buses: &[Bus] = &self.buses;
        while let [bus, later @ ..] = buses {
            if bus.label.is_empty() {
                fail(Message::bus(index).text("empty label"));
            }
            let mut offset = 0;
            let mut rest = later;
            while let [other, tail @ ..] = rest {
                if same(bus.label.as_bytes(), other.label.as_bytes()) {
                    fail(
                        Message::bus(index + 1 + offset)
                            .text("label ")
                            .text(bus.label)
                            .text(" already used by bus ")
                            .number(index),
                    );
                }
                offset += 1;
                rest = tail;
            }
            index += 1;
            buses = later;
        }
        let mut index = 0;
        let mut actuators: &[Actuator] = &self.actuators;
        while let [actuator, later @ ..] = actuators {
            check_actuator(NB, index, actuator, later);
            index += 1;
            actuators = later;
        }
        self
    }

    /// The table with the gains of actuator `actuator` set to `kp` and `kd`.
    ///
    /// # Panics
    ///
    /// On an actuator the table does not have; in const evaluation, that is
    /// the compile error.
    pub const fn gains(mut self, actuator: usize, kp: f32, kd: f32) -> Self {
        match self.actuators.split_at_mut_checked(actuator) {
            Some((_, [row, ..])) => {
                row.kp = kp;
                row.kd = kd;
            }
            _ => fail(Message::actuator(actuator).text("the table has no such actuator")),
        }
        self
    }

    /// The table, refused when the model marks prismatic one of its rows,
    /// the table being in model order: an AK60-6 is rotary (SRS FR-2.2).
    ///
    /// # Panics
    ///
    /// On a prismatic row; in const evaluation, that is the compile error.
    pub const fn check_kinds(self, prismatic: [u64; MASK_WORDS]) -> Self {
        let mut row = 0;
        while row < N {
            refuse_prismatic(row, &prismatic);
            row += 1;
        }
        self
    }

    /// The table checked by [`Table::check`], actuator `k` at engine index
    /// `at[k]`: the CubeMars member of a robot that mixes brands.
    ///
    /// Rejects an index past `64 * MASK_WORDS` and an index two actuators
    /// share; the robot's own count is checked by `robot!` and by `check`.
    ///
    /// # Panics
    ///
    /// On a rejected table or placement; in const evaluation, that is the
    /// compile error.
    ///
    /// ```compile_fail,E0080
    /// # use kmr_adapters::cubemars::{Actuator, Bus, Placement, Table};
    /// const _: Placement<1, 2> = Table {
    ///     buses: [Bus { label: "leg" }],
    ///     actuators: [Actuator::ak60_6(0, 1), Actuator::ak60_6(0, 2)],
    /// }
    /// .at([1, 1]);
    /// ```
    pub const fn at(self, at: [usize; N]) -> Placement<NB, N> {
        let table = self.check();
        let mut index = 0;
        let mut rest: &[usize] = &at;
        while let [engine, later @ ..] = rest {
            if *engine >= 64 * MASK_WORDS {
                fail(
                    Message::actuator(index)
                        .text("engine index ")
                        .number(*engine)
                        .text(" past the robot's ")
                        .number(64 * MASK_WORDS)
                        .text(" actuators at most"),
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
    /// On a rejected table; in const evaluation, that is the compile error.
    pub const fn in_order(self) -> Placement<NB, N> {
        let mut at = [0; N];
        let mut index = 0;
        while index < N {
            at[index] = index;
            index += 1;
        }
        self.at(at)
    }
}

/// Checks actuator `index` against `buses` buses; `later` are the actuators
/// after it, where a repeated id is reported.
const fn check_actuator(buses: usize, index: usize, actuator: &Actuator, later: &[Actuator]) {
    if actuator.bus >= buses {
        fail(
            Message::actuator(index)
                .text("bus ")
                .number(actuator.bus)
                .text(" out of range"),
        );
    }
    if actuator.id == 0 || actuator.id > MAX_ID {
        fail(
            Message::actuator(index)
                .text("CAN id ")
                .number(actuator.id as usize)
                .text(" outside 1..=127"),
        );
    }
    if !(actuator.kp >= 0.0 && actuator.kp <= KP_MAX) {
        fail(Message::actuator(index).text("kp outside 0..=500"));
    }
    if !(actuator.kd >= 0.0 && actuator.kd <= KD_MAX) {
        fail(Message::actuator(index).text("kd outside 0..=5"));
    }
    let mut offset = 0;
    let mut rest = later;
    while let [other, tail @ ..] = rest {
        if other.bus == actuator.bus && other.id == actuator.id {
            fail(
                Message::actuator(index + 1 + offset)
                    .text("CAN id ")
                    .number(actuator.id as usize)
                    .text(" already taken by actuator ")
                    .number(index),
            );
        }
        offset += 1;
        rest = tail;
    }
}

/// Fails, naming `index`, when bit `index` of `prismatic` is set.
const fn refuse_prismatic(index: usize, prismatic: &[u64; MASK_WORDS]) {
    let word = match prismatic.split_at_checked(index / 64) {
        Some((_, [word, ..])) => *word,
        _ => 0,
    };
    if word >> (index % 64) & 1 == 1 {
        fail(
            Message::actuator(index)
                .text("SRS FR-2.2: prismatic CubeMars actuator; an AK60-6 is rotary"),
        );
    }
}

/// A checked [`Table`] placed in the robot: the engine index of each of its
/// actuators, in table order. Built through [`Table::at`] or
/// [`Table::in_order`].
#[derive(Clone, Copy, Debug)]
pub struct Placement<const NB: usize, const M: usize> {
    /// The table, checked.
    pub(crate) table: Table<NB, M>,
    /// Engine index of each actuator of `table`, distinct.
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

    /// The placement, refused when the model marks prismatic an engine index
    /// it places (SRS FR-2.2).
    ///
    /// # Panics
    ///
    /// On a prismatic index; in const evaluation, that is the compile error.
    pub const fn check_kinds(self, prismatic: [u64; MASK_WORDS]) -> Self {
        let mut row = 0;
        while row < M {
            refuse_prismatic(self.at[row], &prismatic);
            row += 1;
        }
        self
    }
}

/// The actuators of a [`Placement`] by engine index, `None` at the indices
/// the member does not own, out of the robot's `N`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Placed<const N: usize> {
    /// Actuator at each engine index.
    slots: [Option<Actuator>; N],
    /// The first table row placed at an index past `N`, and that index.
    past: Option<(usize, usize)>,
}

impl<const N: usize> Placed<N> {
    /// The actuators of `placement`, each at its engine index.
    pub(crate) fn new<const NB: usize, const M: usize>(placement: &Placement<NB, M>) -> Self {
        let mut slots = [None; N];
        let mut past = None;
        for (row, (actuator, &index)) in placement
            .table
            .actuators
            .iter()
            .zip(&placement.at)
            .enumerate()
        {
            match slots.get_mut(index) {
                Some(slot) => *slot = Some(*actuator),
                None => {
                    past.get_or_insert((row, index));
                }
            }
        }
        Self { slots, past }
    }

    /// The first table row placed past the robot's `N` actuators, and its
    /// engine index.
    pub(crate) fn past(&self) -> Option<(usize, usize)> {
        self.past
    }

    /// The actuator at engine index `index`, if the member owns it.
    pub(crate) fn get(&self, index: usize) -> Option<&Actuator> {
        self.slots.get(index).and_then(Option::as_ref)
    }

    /// The owned actuators with their engine indices, ascending.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (usize, &Actuator)> {
        self.slots
            .iter()
            .enumerate()
            .filter_map(|(index, slot)| slot.as_ref().map(|actuator| (index, actuator)))
    }

    /// The engine index of the actuator with CAN id `id` on bus `bus`.
    pub(crate) fn find(&self, bus: usize, id: u8) -> Option<usize> {
        self.iter()
            .find(|(_, actuator)| actuator.bus == bus && actuator.id == id)
            .map(|(index, _)| index)
    }

    /// The engine indices the member owns.
    pub(crate) fn owned(&self) -> Mask<N> {
        let mut owned = Mask::NONE;
        self.iter().for_each(|(index, _)| owned.set(index));
        owned
    }
}
