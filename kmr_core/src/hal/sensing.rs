//! The robot's sensors as a list of members, next to the drivers (SRS
//! FR-5.7). A frunk `HCons`, walked head first: static dispatch, no heap.

use core::any::type_name;
use core::convert::Infallible;
use core::time::Duration;

use frunk_core::hlist::{HCons, HNil};

use super::set::seal::Nested;
use super::{MemberError, Needs, SensorHal};
use crate::error::ConfigError;
use crate::logging::{error, warn};
use crate::sensors::{MAX_IMUS, MAX_MICS, Sensors, Slots};
use crate::signal::SignalMeta;
use crate::{Signal, SignalKind};

/// What a sensor lost at run time does, beyond raising `sensor missing`,
/// which it always does (SRS FR-5.7, FR-13.4).
#[derive(Default)]
pub enum SensorPolicy {
    /// Nothing more: the flag is the signal, the controllers decide.
    #[default]
    Flag,
    /// The loss is logged, once per member.
    Warn,
    /// An emergency stop, in the tick of the loss.
    Abort,
}

/// Members a plan holds. A member takes one slot at least, or counts as
/// one past the slots.
// DECISION: a member that needs no slot is accepted and costs a member
// entry; past them, `TooManySensors` of kind "sensor adapter".
const MAX_MEMBERS: usize = MAX_IMUS + MAX_MICS;

/// The members' slots and silences, planned at run() finalization; what
/// the last read found of each.
#[doc(hidden)]
#[derive(Debug)]
pub struct SensorPlan {
    /// The tick period: silences are counted in its ticks.
    dt: Duration,
    members: usize,
    /// IMU slots handed out so far.
    imus: usize,
    /// Microphone slots handed out so far.
    mics: usize,
    member: [Planned; MAX_MEMBERS],
}

/// One member of a [`SensorPlan`].
#[derive(Clone, Copy, Debug)]
struct Planned {
    /// Type name of the member.
    name: &'static str,
    slots: Slots,
    mic_rate_hz: u32,
    /// Longest silence of any of its slots, in ticks, at least 1.
    silence: u64,
    /// It reported its link down in the last read.
    down: bool,
    /// Its loss was logged under [`SensorPolicy::Warn`].
    warned: bool,
}

impl Planned {
    /// No member.
    const NONE: Self = Self {
        name: "",
        slots: Slots::new(0..0, 0..0),
        mic_rate_hz: 0,
        silence: 1,
        down: false,
        warned: false,
    };
}

/// `max_silence` in ticks of `dt`, rounded up, at least 1.
fn ticks(max_silence: Duration, dt: Duration) -> u64 {
    let ticks = max_silence.as_nanos().div_ceil(dt.as_nanos().max(1));
    u64::try_from(ticks).unwrap_or(u64::MAX).max(1)
}

impl SensorPlan {
    /// No member yet; silences are counted in ticks of `dt`.
    pub(crate) fn new(dt: Duration) -> Self {
        Self {
            dt,
            members: 0,
            imus: 0,
            mics: 0,
            member: [Planned::NONE; MAX_MEMBERS],
        }
    }

    /// Adds the next member, of type name `name`, its slots after those
    /// already handed out. On error the plan is left as it was.
    pub(crate) fn add(&mut self, name: &'static str, needs: &Needs) -> Result<(), ConfigError> {
        let imus = self.imus.saturating_add(usize::from(needs.imus));
        if imus > MAX_IMUS {
            return Err(ConfigError::TooManySensors {
                kind: "IMU",
                asked: imus,
                slots: MAX_IMUS,
            });
        }
        let mics = self.mics.saturating_add(usize::from(needs.mics));
        if mics > MAX_MICS {
            return Err(ConfigError::TooManySensors {
                kind: "microphone",
                asked: mics,
                slots: MAX_MICS,
            });
        }
        let Some(slot) = self.member.get_mut(self.members) else {
            return Err(ConfigError::TooManySensors {
                kind: "sensor adapter",
                asked: self.members.saturating_add(1),
                slots: MAX_MEMBERS,
            });
        };
        *slot = Planned {
            name,
            slots: Slots::new(self.imus..imus, self.mics..mics),
            mic_rate_hz: needs.mic_rate_hz,
            silence: ticks(needs.max_silence, self.dt),
            down: false,
            warned: false,
        };
        self.members += 1;
        self.imus = imus;
        self.mics = mics;
        Ok(())
    }

    /// The planned members.
    fn planned(&self) -> &[Planned] {
        self.member.get(..self.members).unwrap_or(&[])
    }

    /// Declares every member's slots in `sensors`.
    pub(crate) fn declare(&self, sensors: &mut Sensors) {
        for member in self.planned() {
            sensors.declare(member.slots, member.mic_rate_hz);
        }
    }

    /// Members planned.
    pub(crate) fn members(&self) -> usize {
        self.members
    }

    /// Longest silence of member `member`, in ticks.
    pub(crate) fn silence(&self, member: usize) -> u64 {
        self.planned()
            .get(member)
            .map_or(Planned::NONE.silence, |planned| planned.silence)
    }

    /// The slots of member `member`; none past the members.
    fn slots(&self, member: usize) -> Slots {
        self.planned()
            .get(member)
            .map_or(Planned::NONE.slots, |planned| planned.slots)
    }

    /// Notes whether member `member` reported its link down in this read.
    fn note_link(&mut self, member: usize, down: bool) {
        if let Some(planned) = self
            .member
            .get_mut(..self.members)
            .and_then(|m| m.get_mut(member))
        {
            planned.down = down;
        }
    }

    /// After a read: whether a member is missing, its link down or a slot
    /// silent past its limit (SRS FR-13.4), and the emergency stop `policy`
    /// asks for then.
    pub(crate) fn judge(
        &mut self,
        sensors: &Sensors,
        policy: &SensorPolicy,
    ) -> (bool, Option<Signal>) {
        let mut missing = false;
        let mut stop = None;
        let members = self.member.get_mut(..self.members).unwrap_or_default();
        for (position, member) in members.iter_mut().enumerate() {
            let silent = sensors.silence(member.slots);
            if !member.down && silent <= member.silence {
                continue;
            }
            missing = true;
            match policy {
                SensorPolicy::Flag => {}
                SensorPolicy::Warn if !member.warned => {
                    member.warned = true;
                    report_warn(position, member, silent);
                }
                SensorPolicy::Warn => {}
                SensorPolicy::Abort if stop.is_none() => {
                    report_abort(position, member, silent);
                    stop = Some(Signal {
                        kind: SignalKind::EmergencyStop,
                        meta: SignalMeta::new(
                            "SensorMissing: a sensor was lost under SensorPolicy::Abort (SRS FR-5.7)",
                            file!(),
                            line!(),
                        ),
                    });
                }
                SensorPolicy::Abort => {}
            }
        }
        (missing, stop)
    }
}

/// The one warning of a lost member under [`SensorPolicy::Warn`]: which
/// member, and why.
fn report_warn(position: usize, member: &Planned, silent: u64) {
    if member.down {
        warn!(
            member = position,
            name = member.name,
            "a sensor is missing: its adapter reports its link down; SensorPolicy::Warn keeps the loop running"
        );
    } else {
        warn!(
            member = position,
            name = member.name,
            silent_ticks = silent,
            max_silence_ticks = member.silence,
            "a sensor is missing: silent past its max_silence; SensorPolicy::Warn keeps the loop running"
        );
    }
}

/// The error line of the member that stops the run under
/// [`SensorPolicy::Abort`]: which member, and why.
fn report_abort(position: usize, member: &Planned, silent: u64) {
    if member.down {
        error!(
            member = position,
            name = member.name,
            "a sensor is missing: its adapter reports its link down; SensorPolicy::Abort: emergency stop"
        );
    } else {
        error!(
            member = position,
            name = member.name,
            silent_ticks = silent,
            max_silence_ticks = member.silence,
            "a sensor is missing: silent past its max_silence; SensorPolicy::Abort: emergency stop"
        );
    }
}

mod seal {
    /// Implemented by the engine for `HNil` and `HCons` only.
    pub trait Sensing {}
}

/// The robot's sensors: a list of [`SensorHal`] members, `HCons<A, HCons<B,
/// HNil>>`. Implemented by the engine only; the walk calls every member,
/// head first, and closes them in reverse.
pub trait Sensing: seal::Sensing {
    /// One variant per member, nested.
    type Error: Nested;

    /// The needs of every member, slots handed out in declaration order.
    /// Past the slots of a kind: [`ConfigError::TooManySensors`].
    #[doc(hidden)]
    fn claim(&self, plan: &mut SensorPlan) -> Result<(), ConfigError>;

    /// [`SensorHal::open`] on every member; the first error stops the walk.
    #[doc(hidden)]
    fn open(&mut self) -> Result<(), Self::Error>;

    /// [`SensorHal::read`] on every member, each into its own slots; this
    /// node is member `member`.
    #[doc(hidden)]
    fn read(&mut self, sensors: &mut Sensors, plan: &mut SensorPlan, member: usize);

    /// [`SensorHal::close`] on every member, last first.
    #[doc(hidden)]
    fn close(&mut self);
}

impl seal::Sensing for HNil {}

impl Sensing for HNil {
    type Error = Infallible;

    fn claim(&self, _plan: &mut SensorPlan) -> Result<(), ConfigError> {
        Ok(())
    }

    fn open(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    fn read(&mut self, _sensors: &mut Sensors, _plan: &mut SensorPlan, _member: usize) {}

    fn close(&mut self) {}
}

impl<H: SensorHal, T: seal::Sensing> seal::Sensing for HCons<H, T> {}

impl<H: SensorHal, T: Sensing> Sensing for HCons<H, T> {
    type Error = MemberError<H::Error, T::Error>;

    fn claim(&self, plan: &mut SensorPlan) -> Result<(), ConfigError> {
        plan.add(type_name::<H>(), &self.head.needs())?;
        self.tail.claim(plan)
    }

    fn open(&mut self) -> Result<(), Self::Error> {
        self.head.open().map_err(MemberError::head::<H>)?;
        self.tail.open().map_err(MemberError::Tail)
    }

    fn read(&mut self, sensors: &mut Sensors, plan: &mut SensorPlan, member: usize) {
        self.head.read(sensors.feed(plan.slots(member)));
        plan.note_link(member, sensors.took_link_down());
        self.tail.read(sensors, plan, member.saturating_add(1));
    }

    fn close(&mut self) {
        self.tail.close();
        self.head.close();
    }
}

#[cfg(test)]
mod tests {
    use frunk_core::hlist;

    use super::*;
    use crate::sensors::{Accel, MAX_IMUS, MAX_MICS};
    use crate::state::N;
    use crate::test_util::{Call, DT, FakeSensor, FakeSensorError, Log, Reading};

    /// The plan of `list`, at `DT`.
    fn claimed(list: &impl Sensing) -> Result<SensorPlan, ConfigError> {
        let mut plan = SensorPlan::new(DT);
        list.claim(&mut plan).map(|()| plan)
    }

    /// A fake of `imus` IMUs and `mics` microphones.
    fn fake(log: &Log<N>, id: u8, imus: u8, mics: u8) -> FakeSensor<'_, N> {
        FakeSensor {
            imus,
            mics,
            ..FakeSensor::new(log, id)
        }
    }

    #[test]
    fn slots_follow_declaration_order() {
        let log = Log::new();
        let first = [Reading {
            accel: Some(Accel([1.0, 0.0, 0.0])),
            ..Reading::QUIET
        }];
        let second = [Reading {
            accel: Some(Accel([2.0, 0.0, 0.0])),
            ..Reading::QUIET
        }];
        let mut list = hlist![
            FakeSensor {
                script: &first,
                ..fake(&log, 0, 1, 0)
            },
            FakeSensor {
                script: &second,
                ..fake(&log, 1, 1, 0)
            },
        ];
        let mut plan = claimed(&list).expect("two IMUs fit");
        let mut sensors = Sensors::new();
        plan.declare(&mut sensors);

        sensors.begin_tick(0);
        list.read(&mut sensors, &mut plan, 0);

        let x = |index| sensors.imu(index).and_then(|imu| imu.accel()).map(Accel::x);
        assert_eq!((x(0), x(1), x(2)), (Some(1.0), Some(2.0), None));
        assert_eq!(*log.calls(), [Call::SensorRead(0), Call::SensorRead(1)]);
    }

    #[test]
    fn too_many_imus_is_refused_before_any_open() {
        let log = Log::new();
        let list = hlist![fake(&log, 0, 2, 0), fake(&log, 1, 3, 0)];

        let refused = claimed(&list).err();

        assert_eq!(
            refused,
            Some(ConfigError::TooManySensors {
                kind: "IMU",
                asked: 5,
                slots: MAX_IMUS
            })
        );
        assert!(log.calls().is_empty());
    }

    #[test]
    fn two_mics_are_refused() {
        let log = Log::new();
        let list = hlist![fake(&log, 0, 0, 1), fake(&log, 1, 0, 1)];

        let refused = claimed(&list).err();

        assert_eq!(
            refused,
            Some(ConfigError::TooManySensors {
                kind: "microphone",
                asked: 2,
                slots: MAX_MICS
            })
        );
        let text = refused.map(|error| error.to_string());
        assert_eq!(
            text.as_deref(),
            Some("the sensors need 2 microphone slots and the engine has 1")
        );
    }

    #[test]
    fn open_stops_at_the_first_error_and_names_the_member() {
        let log = Log::new();
        let mut list = hlist![
            fake(&log, 0, 1, 0),
            FakeSensor {
                fail_open: true,
                ..fake(&log, 1, 1, 0)
            },
            fake(&log, 2, 1, 0),
        ];

        let error = list.open().expect_err("member 1 fails");

        assert_eq!(*log.calls(), [Call::SensorOpen(0), Call::SensorOpen(1)]);
        assert_eq!(
            error,
            MemberError::Tail(MemberError::Head {
                name: type_name::<FakeSensor<'_, N>>(),
                error: FakeSensorError::Open { id: 1 },
            })
        );
        let text = error.to_string();
        assert!(text.starts_with("member 1 ("), "{text}");
        assert!(
            text.ends_with("fake sensor 1: open refused by its script"),
            "{text}"
        );
        let source = core::error::Error::source(&error).expect("the member's error");
        assert_eq!(
            source.downcast_ref::<FakeSensorError>(),
            Some(&FakeSensorError::Open { id: 1 })
        );
    }

    #[test]
    fn close_runs_on_every_member_tail_first() {
        let log = Log::new();
        let mut list = hlist![
            fake(&log, 0, 1, 0),
            fake(&log, 1, 1, 0),
            fake(&log, 2, 0, 1)
        ];

        list.close();

        assert_eq!(
            *log.calls(),
            [
                Call::SensorClose(2),
                Call::SensorClose(1),
                Call::SensorClose(0)
            ]
        );
    }

    #[test]
    fn an_empty_list_claims_nothing() {
        let mut plan = claimed(&HNil).expect("nothing to refuse");
        let mut sensors = Sensors::new();
        plan.declare(&mut sensors);
        sensors.begin_tick(0);
        HNil.read(&mut sensors, &mut plan, 0);

        assert_eq!(plan.members(), 0);
        assert!(sensors.imu(0).is_none() && sensors.mic(0).is_none());
        assert_eq!(HNil.open(), Ok(()));
    }

    #[test]
    fn max_silence_rounds_up_to_whole_ticks_and_never_below_one() {
        let log = Log::<N>::new();
        let silent_for = |max_silence: Duration| {
            let list = hlist![FakeSensor {
                max_silence,
                ..FakeSensor::new(&log, 0)
            }];
            claimed(&list).map(|plan| plan.silence(0))
        };

        assert_eq!(silent_for(DT * 20), Ok(20));
        assert_eq!(silent_for(DT * 5 / 2), Ok(3));
        assert_eq!(silent_for(Duration::ZERO), Ok(1));
    }
}
