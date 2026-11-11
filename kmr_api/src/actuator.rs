//! Actuators declared in `main.rs`, lowered to the hardware table at `run()` (SRS FR-6.1).

use core::ops::RangeInclusive;

use kmr_adapters::binding::Port;
use kmr_adapters::dynamixel::{Actuator, Bus, Declared, Mode, Model, Table, Torque, Zero};

use crate::{BusHandle, MASK_WORDS, UserError};

/// Actuators of one model on one bus, firmware ids in engine order.
///
/// Built by a model type, e.g. [`dynamixel::Rx28`](crate::dynamixel::Rx28), and handed to
/// [`Robot::add_actuator`](crate::Robot::add_actuator).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Actuators {
    pub(crate) bus: BusHandle,
    pub(crate) ids: IdSet,
    pub(crate) driver: Driver,
}

/// The firmware ids of a declaration.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum IdSet {
    /// These ids, in this order.
    Range(RangeInclusive<u8>),
    /// One actuator per model actuator, ids `1..=N`.
    // DECISION: `all()` numbers from 1, the Dynamixel factory id, so a chain numbered 1..=N
    // needs no range; combined with another declaration it overcounts and `run()` refuses it.
    All,
}

impl IdSet {
    /// The ids on a robot of `n` actuators.
    fn resolve(&self, n: usize) -> RangeInclusive<u8> {
        match self {
            Self::Range(ids) => ids.clone(),
            Self::All => 1..=u8::try_from(n).unwrap_or(u8::MAX),
        }
    }
}

/// The driver of a declaration, by brand, with its settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Driver {
    /// A Dynamixel model.
    Dynamixel {
        model: Model,
        mode: Mode,
        torque: Torque,
        zero: Zero,
    },
}

impl Actuators {
    /// How many actuators this declares on a robot of `n` actuators.
    fn len(&self, n: usize) -> usize {
        self.ids.resolve(n).count()
    }
}

/// The hardware the declarations of a robot of `N` actuators name, as the Dynamixel driver
/// takes it: at most `N` buses, since every bus carries an actuator.
pub type DxlDeclared<const N: usize> = Declared<N, N>;

/// Lowers `declarations` to fixed arrays: engine index `i` is the `i`th actuator declared, and
/// the buses are numbered in first-declared order. No I/O.
///
/// # Errors
///
/// [`UserError::ActuatorCount`], [`UserError::BusWithoutBaud`], [`UserError::DuplicateBus`],
/// then [`UserError::HardwareRejected`]: a rule of the table, or an index the model's
/// `prismatic` bits mark (SRS FR-2.2).
pub(crate) fn lower<const N: usize>(
    declarations: Vec<Actuators>,
    prismatic: [u64; MASK_WORDS],
) -> Result<DxlDeclared<N>, UserError> {
    let declared = declarations.iter().map(|d| d.len(N)).sum();
    if declared != N {
        return Err(UserError::ActuatorCount {
            declared,
            expected: N,
        });
    }
    let mut handles: Vec<BusHandle> = Vec::new();
    let mut actuators = Vec::with_capacity(N);
    for declaration in &declarations {
        // An empty range names no bus: every live bus carries an actuator.
        if declaration.len(N) == 0 {
            continue;
        }
        let bus = match handles.iter().position(|seen| *seen == declaration.bus) {
            Some(bus) => bus,
            None => {
                let handle = declaration.bus;
                if handles.iter().any(|seen| seen.port() == handle.port()) {
                    return Err(UserError::DuplicateBus {
                        port: handle.port(),
                    });
                }
                if handle.binding.baud.is_none() {
                    return Err(UserError::BusWithoutBaud {
                        port: handle.port(),
                    });
                }
                handles.push(handle);
                handles.len() - 1
            }
        };
        let Driver::Dynamixel {
            model,
            mode,
            torque,
            zero,
        } = declaration.driver;
        actuators.extend(declaration.ids.resolve(N).map(|id| Actuator {
            bus,
            id,
            model,
            mode,
            torque,
            zero,
        }));
    }
    let unused = Bus {
        label: "",
        baud: 0,
        read_every: None,
    };
    let mut buses = [unused; N];
    let mut ports = [Port::ByDev(""); N];
    for ((bus, port), handle) in buses.iter_mut().zip(&mut ports).zip(&handles) {
        // DECISION: a declared bus is read every DEFAULT_READ_EVERY ticks, the adapter's default;
        // `Bus` gets no cadence knob until the NFR-1 bench settles D9.
        *bus = Bus {
            label: handle.port(),
            baud: handle.binding.baud.unwrap_or_default(),
            read_every: None,
        };
        *port = handle.binding.port;
    }
    let actuators: [Actuator; N] =
        actuators
            .try_into()
            .map_err(|_: Vec<Actuator>| UserError::ActuatorCount {
                declared,
                expected: N,
            })?;
    let rejected = |source| UserError::HardwareRejected {
        source: Box::new(source),
    };
    Declared::new(Table { buses, actuators }, ports, handles.len())
        .map_err(rejected)?
        .check_kinds(prismatic)
        .map_err(rejected)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bus as Plugin;
    use crate::dynamixel::{Ax12a, Xm430W210};

    /// No prismatic actuator.
    const REVOLUTE: [u64; MASK_WORDS] = [0; MASK_WORDS];

    /// Actuators past the first four, up to `N`, at ids from 100.
    fn filler<const N: usize>(bus: BusHandle) -> Actuators {
        let last = u8::try_from(100 + N - 5).unwrap_or(u8::MAX);
        #[allow(clippy::reversed_empty_ranges)]
        Xm430W210::ids(100..=last).on(bus)
    }

    #[test]
    fn engine_order_is_declaration_order_and_buses_first_declared_order() {
        engine_order::<4>();
        engine_order::<10>();
        engine_order::<64>();
    }

    /// The order test on a robot of `N` actuators, the first four the case.
    fn engine_order<const N: usize>() {
        let arm = Plugin::by_id("ARM00001").baud(1_000_000).build();
        let wrist = Plugin::by_dev("/dev/ttyUSB1").baud(57_600).build();
        let declared = lower::<N>(
            vec![
                Ax12a::id(9).on(wrist),
                Xm430W210::ids(1..=2).on(arm),
                Ax12a::id(3).read_only().on(wrist),
                filler::<N>(arm),
            ],
            REVOLUTE,
        )
        .expect("valid declarations");
        let table = declared.table();
        let placed = table.actuators.map(|a| (a.bus, a.id, a.model, a.torque));
        assert_eq!(
            placed[..4],
            [
                (0, 9, Model::Ax12a, Torque::On),
                (1, 1, Model::Xm430W210, Torque::On),
                (1, 2, Model::Xm430W210, Torque::On),
                (0, 3, Model::Ax12a, Torque::ReadOnly),
            ]
        );
        assert_eq!(declared.live(), 2);
        assert_eq!(
            declared.ports(),
            [Port::ByDev("/dev/ttyUSB1"), Port::ById("ARM00001")]
        );
        assert_eq!(
            table.buses.map(|b| (b.label, b.baud))[..2],
            [("/dev/ttyUSB1", 57_600), ("ARM00001", 1_000_000)]
        );
    }

    #[test]
    fn an_empty_range_counts_no_actuator() {
        empty_range::<4>();
        empty_range::<10>();
    }

    /// The empty range test on a robot of `N` actuators.
    fn empty_range<const N: usize>() {
        let arm = Plugin::by_id("ARM00001").baud(1_000_000).build();
        #[expect(clippy::reversed_empty_ranges, reason = "the empty range is the case")]
        let none = Xm430W210::ids(5..=1).on(arm);
        let res = lower::<N>(
            vec![none, Xm430W210::ids(1..=4).on(arm), filler::<N>(arm)],
            REVOLUTE,
        );
        assert!(res.is_ok(), "{:?}", res.err());
    }

    #[test]
    fn an_empty_range_registers_no_bus() {
        let arm = Plugin::by_id("ARM00001").baud(1_000_000).build();
        let spare = Plugin::by_id("SPARE001").build();
        #[expect(clippy::reversed_empty_ranges, reason = "the empty range is the case")]
        let none = Xm430W210::ids(5..=1).on(spare);
        let declared = lower::<1>(vec![Xm430W210::id(1).on(arm), none], REVOLUTE)
            .expect("the empty range is ignored");
        assert_eq!(declared.live(), 1);
        assert_eq!(declared.ports(), [Port::ById("ARM00001")]);
    }

    #[test]
    fn all_declares_one_actuator_per_model_actuator_at_ids_from_1() {
        all_ids::<1>();
        all_ids::<12>();
        all_ids::<64>();
    }

    /// The `all()` test on a robot of `N` actuators.
    fn all_ids<const N: usize>() {
        let spine = Plugin::by_id("SPINE001").baud(1_000_000).build();
        let declared =
            lower::<N>(vec![Ax12a::all().read_only().on(spine)], REVOLUTE).expect("all of N");
        let placed = declared
            .table()
            .actuators
            .map(|a| (a.bus, a.id, a.model, a.torque));
        for (id, actuator) in (1..).zip(placed) {
            assert_eq!(actuator, (0, id, Model::Ax12a, Torque::ReadOnly));
        }
        assert_eq!(declared.live(), 1);
    }

    #[test]
    fn all_with_another_declaration_overcounts() {
        let spine = Plugin::by_id("SPINE001").baud(1_000_000).build();
        let tail = Plugin::by_id("TAIL0001").baud(1_000_000).build();
        let res = lower::<4>(
            vec![Ax12a::all().on(spine), Ax12a::id(9).on(tail)],
            REVOLUTE,
        );
        assert!(
            matches!(
                res,
                Err(UserError::ActuatorCount {
                    declared: 5,
                    expected: 4
                })
            ),
            "{res:?}"
        );
    }

    #[test]
    fn a_prismatic_index_is_refused_as_check_kinds_words_it() {
        let arm = Plugin::by_id("ARM00001").baud(1_000_000).build();
        let mut prismatic = REVOLUTE;
        prismatic[0] = 0b100;
        let Err(e) = lower::<4>(vec![Xm430W210::ids(1..=4).on(arm)], prismatic) else {
            panic!("a prismatic Dynamixel actuator was lowered");
        };
        assert_eq!(
            e.to_string(),
            "the declared actuators are refused: actuator 2: SRS FR-2.2: prismatic Dynamixel \
             actuator has no mm conversion"
        );
    }
}
