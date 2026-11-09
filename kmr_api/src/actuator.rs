//! Actuators declared in `main.rs`, lowered to the hardware table at `run()` (SRS FR-6.1).

use kmr_adapters::binding::Port;
use kmr_adapters::dynamixel::{
    Actuator, Bus, Declared, Mode, Model, Settings, Table, Torque, Zero,
};

use crate::{BusHandle, MASK_WORDS, UserError};

/// Actuators of one model on one bus, firmware ids in engine order.
///
/// Built by a model type, e.g. [`dynamixel::Rx28`](crate::dynamixel::Rx28), and handed to
/// [`Robot::add_actuator`](crate::Robot::add_actuator).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Actuators {
    pub(crate) bus: BusHandle,
    pub(crate) ids: core::ops::RangeInclusive<u8>,
    pub(crate) driver: Driver,
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
        settings: Settings,
    },
}

impl Actuators {
    /// How many actuators this declares.
    fn len(&self) -> usize {
        self.ids.clone().count()
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
    let declared = declarations.iter().map(Actuators::len).sum();
    if declared != N {
        return Err(UserError::ActuatorCount {
            declared,
            expected: N,
        });
    }
    let mut handles: Vec<BusHandle> = Vec::new();
    let mut actuators = Vec::with_capacity(N);
    let mut settings = Vec::with_capacity(N);
    for declaration in &declarations {
        // An empty range names no bus: every live bus carries an actuator.
        if declaration.len() == 0 {
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
            settings: set,
        } = declaration.driver;
        actuators.extend(declaration.ids.clone().map(|id| Actuator {
            bus,
            id,
            model,
            mode,
            torque,
            zero,
        }));
        settings.extend(declaration.ids.clone().map(|_| set));
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
    let settings: [Settings; N] =
        settings
            .try_into()
            .map_err(|_: Vec<Settings>| UserError::ActuatorCount {
                declared,
                expected: N,
            })?;
    let rejected = |source| UserError::HardwareRejected {
        source: Box::new(source),
    };
    Declared::new(Table { buses, actuators }, ports, handles.len())
        .map_err(rejected)?
        .check_kinds(prismatic)
        .map(|declared| declared.with_settings(settings))
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
    fn gains_and_led_reach_each_actuator_of_their_declaration_only() {
        use kmr_adapters::dynamixel::{Gains, Settings};

        let arm = Plugin::by_id("ARM00001").baud(1_000_000).build();
        let declared = lower::<4>(
            vec![
                Xm430W210::ids(1..=2).gains(800, 20, 4000).led(true).on(arm),
                Xm430W210::id(3).on(arm),
                Xm430W210::id(4).led(false).on(arm),
            ],
            REVOLUTE,
        )
        .expect("valid declarations");

        let tuned = Settings {
            gains: Some(Gains {
                p: 800,
                i: 20,
                d: 4000,
            }),
            led: Some(true),
            voltage_limits_dv: None,
        };
        let dark = Settings {
            gains: None,
            led: Some(false),
            voltage_limits_dv: None,
        };
        assert_eq!(
            declared.settings(),
            &[tuned, tuned, Settings::default(), dark]
        );
    }

    #[test]
    fn voltage_limits_reach_their_declaration_in_0_1_v_rounded() {
        let arm = Plugin::by_id("ARM00001").baud(1_000_000).build();
        let declared = lower::<4>(
            vec![
                Ax12a::ids(1..=2).voltage_limits(9.5, 13.0).on(arm),
                Ax12a::id(3).voltage_limits(9.46, 12.04).on(arm),
                Ax12a::id(4).on(arm),
            ],
            REVOLUTE,
        )
        .expect("valid declarations");

        let limits = declared.settings().map(|set| set.voltage_limits_dv);
        assert_eq!(
            limits,
            [Some((95, 130)), Some((95, 130)), Some((95, 120)), None]
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
