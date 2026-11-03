//! A robot of two brands, end to end and without hardware: each member drives its own
//! actuators, bus plugins reach their member by label, the claims are checked before any
//! port opens, and a member's error names its position and type, short in the message.

use std::error::Error;
use std::fmt;
use std::marker::PhantomData;
use std::num::NonZeroU32;
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use kmr_api::{Binding, Brand, Bus, EachTick, HCons, HNil, Q, Refused, Sensors, Spec, State};
use kmr_api::{HardwareError, MemberError, N as JOINTS, Time, UserError};
// The HAL contract: what a driver implements, not user API.
use kmr_core::hal::{
    Batch, Claim, Command, Coverage, Drivers, Faults, Group, Hal, Mask, Member, Overrides, Sample,
    seal,
};

// The scenarios name actuators 0 to 3.
const _: () = assert!(JOINTS >= 4, "the driver tests need four actuators");

/// The signal bus and the log are process-global: every test takes this first.
static BUS: Mutex<()> = Mutex::new(());

fn bus() -> MutexGuard<'static, ()> {
    BUS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// What a member saw, in call order.
#[derive(Clone, Debug, PartialEq)]
enum Event {
    /// Its brand bound these ports.
    Bound(&'static str, Vec<&'static str>),
    /// A call without data.
    Call(&'static str, &'static str),
    /// `write`: the indices of its runs and the `written` masks it got.
    Wrote(&'static str, Vec<usize>, Mask),
    /// `hold`: the `written` masks it got.
    Held(&'static str, Mask),
}

static LOG: Mutex<Vec<Event>> = Mutex::new(Vec::new());

fn log(event: Event) {
    LOG.lock()
        .unwrap_or_else(PoisonError::into_inner)
        .push(event);
}

/// The log so far, emptied.
fn take() -> Vec<Event> {
    std::mem::take(&mut *LOG.lock().unwrap_or_else(PoisonError::into_inner))
}

/// The plain calls of the log, as `(member, call)`.
fn calls(events: &[Event]) -> Vec<(&'static str, &'static str)> {
    events
        .iter()
        .filter_map(|event| match event {
            Event::Call(member, call) => Some((*member, *call)),
            _ => None,
        })
        .collect()
}

/// A brand of fake actuators.
trait Kind {
    /// Short name in the log.
    const NAME: &'static str;
    /// Label of its one bus.
    const LABEL: &'static str;
}

/// One brand.
struct Arm;

impl Kind for Arm {
    const NAME: &'static str = "arm";
    const LABEL: &'static str = "arm";
}

/// Another brand.
struct Leg;

impl Kind for Leg {
    const NAME: &'static str = "leg";
    const LABEL: &'static str = "leg";
}

/// A third brand, labelling its bus like the arm.
struct Twin;

impl Kind for Twin {
    const NAME: &'static str = "twin";
    const LABEL: &'static str = "arm";
}

/// The typed error of a fake member, with its data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Unplugged {
    port: &'static str,
    attempts: u8,
}

impl fmt::Display for Unplugged {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "port {} did not answer after {} attempts",
            self.port, self.attempts
        )
    }
}

impl Error for Unplugged {}

/// A fake member of brand `K`: owns `owned`, reads `Q(index)` into its own slots.
struct Fake<K> {
    owned: Mask,
    port: &'static str,
    fail_open: bool,
    kind: PhantomData<K>,
}

impl<K: Kind> Fake<K> {
    fn call(&self, call: &'static str) {
        log(Event::Call(K::NAME, call));
    }

    /// Writes `Q(index)` into each owned slot of `sample`.
    fn fill(&self, sample: &mut Sample<'_>) {
        for index in self.owned.iter() {
            if let Some(q) = sample.q.get_mut(index) {
                *q = Q(index as f32);
            }
        }
    }
}

impl<K> seal::Adapter for Fake<K> {}

impl<K: Kind> Hal for Fake<K> {
    type Error = Unplugged;

    fn check(&mut self, _overrides: Overrides) -> Result<Claim, Unplugged> {
        self.call("check");
        Ok(Claim {
            owned: self.owned,
            group: [Group(0); JOINTS],
            read_every: [NonZeroU32::MIN; JOINTS],
            read_cost: Duration::ZERO,
            write_cost: Duration::ZERO,
        })
    }

    fn open(&mut self) -> Result<(), Unplugged> {
        self.call("open");
        if self.fail_open {
            return Err(Unplugged {
                port: self.port,
                attempts: 3,
            });
        }
        Ok(())
    }

    fn zero(&mut self) -> Result<(), Unplugged> {
        self.call("zero");
        Ok(())
    }

    fn ping(&mut self) -> Result<(), Unplugged> {
        self.call("ping");
        Ok(())
    }

    fn health(&mut self) -> Result<(), Unplugged> {
        self.call("health");
        Ok(())
    }

    fn configure(&mut self) -> Result<(), Unplugged> {
        self.call("configure");
        Ok(())
    }

    fn read_all(&mut self, mut sample: Sample<'_>) -> Result<(), Unplugged> {
        self.call("read_all");
        self.fill(&mut sample);
        Ok(())
    }

    fn torque_on(&mut self) -> Result<(), Unplugged> {
        self.call("torque_on");
        Ok(())
    }

    fn read(&mut self, mut sample: Sample<'_>, _faults: &mut Faults) -> Mask {
        self.fill(&mut sample);
        self.owned
    }

    fn write(&mut self, command: Command<'_>, batch: &Batch<'_>, _faults: &mut Faults) {
        let runs = batch.runs().flat_map(|(_, run)| run.iter().copied());
        log(Event::Wrote(K::NAME, runs.collect(), command.written.any()));
    }

    fn hold(&mut self, target: Command<'_>) {
        log(Event::Held(K::NAME, target.written.any()));
    }

    fn torque_off(&mut self) {
        self.call("torque_off");
    }

    fn close(&mut self) {
        self.call("close");
    }
}

/// The part of brand `K` of the robot's hardware, as `robot!` would hold it.
struct Part<K> {
    owned: Mask,
    fail_open: bool,
    fail_bind: bool,
    kind: PhantomData<K>,
}

impl<K: Kind> Part<K> {
    fn new(owned: Mask) -> Self {
        Self {
            owned,
            fail_open: false,
            fail_bind: false,
            kind: PhantomData,
        }
    }
}

impl<K: Kind> Brand for Part<K> {
    type Member = Fake<K>;

    fn labels(&self) -> Vec<&'static str> {
        vec![K::LABEL]
    }

    fn bind(self, plugins: &[Binding]) -> Result<Fake<K>, Refused<Unplugged>> {
        let ports: Vec<_> = plugins.iter().map(port).collect();
        log(Event::Bound(K::NAME, ports.clone()));
        let port = ports.first().copied().unwrap_or("");
        if self.fail_bind {
            return Err(Refused::Member(Unplugged { port, attempts: 0 }));
        }
        Ok(Fake {
            owned: self.owned,
            port,
            fail_open: self.fail_open,
            kind: PhantomData,
        })
    }
}

/// The serial id or the path of `plugin`.
fn port(plugin: &Binding) -> &'static str {
    match plugin.port {
        kmr_adapters::binding::Port::ById(port) | kmr_adapters::binding::Port::ByDev(port) => port,
    }
}

/// The actuators `select` picks.
fn mask(select: impl Fn(usize) -> bool) -> Mask {
    let mut mask = Mask::NONE;
    (0..JOINTS)
        .filter(|&index| select(index))
        .for_each(|index| mask.set(index));
    mask
}

fn evens() -> Mask {
    mask(|index| index % 2 == 0)
}

fn odds() -> Mask {
    mask(|index| index % 2 == 1)
}

/// The arm on the even actuators, the leg on the odd ones.
const COVERED: u8 = 0;
/// The leg on actuator 1 only: 3 is nobody's.
const GAP: u8 = 1;
/// The leg on the odd actuators and on 0, the arm's.
const OVERLAP: u8 = 2;
/// `COVERED`, the leg's `open` failing.
const FAILS_OPEN: u8 = 3;
/// `COVERED`, the leg's binding failing.
const FAILS_BIND: u8 = 4;

/// A two-brand robot in scenario `CASE`.
struct TwoBrands<const CASE: u8>;

/// The drivers of every scenario: the arm, then the leg.
type Members = HCons<Fake<Arm>, HCons<Fake<Leg>, HNil>>;

/// The error of `Members`.
type MembersError = <Members as Drivers>::Error;

impl<const CASE: u8> Spec for TwoBrands<CASE> {
    type Drivers = Members;

    const LINK_NAMES: &'static [&'static str] = &["base"];

    fn bind(plugins: &[Binding]) -> Result<Members, UserError> {
        let arm = Part::<Arm>::new(evens());
        let mut leg = Part::<Leg>::new(odds());
        match CASE {
            GAP => leg.owned = mask(|index| index == 1),
            OVERLAP => leg.owned = odds().union(mask(|index| index == 0)),
            FAILS_OPEN => leg.fail_open = true,
            FAILS_BIND => leg.fail_bind = true,
            _ => {}
        }
        let hardware = HCons {
            head: arm,
            tail: HCons {
                head: leg,
                tail: HNil,
            },
        };
        kmr_api::__bind(hardware, plugins)
    }
}

/// Positions the controller read at tick 0.
static SEEN: Mutex<Option<[Q; JOINTS]>> = Mutex::new(None);

/// Keeps the positions of tick 0, writes every position, shuts down at tick 2.
fn write_all(time: &Time, robot: &mut State, _s: &Sensors) {
    if time.tick() == 0 {
        *SEEN.lock().unwrap_or_else(PoisonError::into_inner) = robot.q();
    }
    robot.set_all_q([Q(0.0); JOINTS]);
    assert!(time.tick() <= 2, "the loop ignored the shutdown");
    if time.tick() == 2 {
        kmr_api::shutdown!("test: tick 2");
    }
}

/// Runs scenario `CASE` with a plugin per brand and a zero hold, from an empty log.
fn run<const CASE: u8>() -> Result<(), UserError> {
    take();
    *SEEN.lock().unwrap_or_else(PoisonError::into_inner) = None;
    kmr_api::Robot::<TwoBrands<CASE>>::new()
        .set_dt(Duration::from_millis(1))
        .set_hold(Duration::ZERO)
        .add_plugins(Bus::by_dev("/dev/leg").bus("leg"))
        .add_plugins(Bus::by_id("ARM00001").bus("arm"))
        .add_controller(EachTick, write_all)
        .run()
}

/// The hardware error behind `e`, typed.
fn hardware_error(e: &UserError) -> Option<&HardwareError<MembersError>> {
    e.source()?.downcast_ref()
}

#[test]
fn two_brands_each_drive_only_their_own_actuators() {
    let _bus = bus();
    let res = run::<COVERED>();
    assert!(res.is_ok(), "{res:?}");
    let events = take();

    assert_eq!(events[0], Event::Bound("arm", vec!["ARM00001"]));
    assert_eq!(events[1], Event::Bound("leg", vec!["/dev/leg"]));
    for (member, owned) in [("arm", evens()), ("leg", odds())] {
        let writes: Vec<_> = events
            .iter()
            .filter_map(|event| match event {
                Event::Wrote(who, runs, written) if *who == member => Some((runs, *written)),
                _ => None,
            })
            .collect();
        assert_eq!(writes.len(), 3, "{member}: one write per tick");
        for (runs, written) in writes {
            assert_eq!(*runs, owned.iter().collect::<Vec<_>>(), "{member}");
            assert_eq!(written, owned, "{member}");
        }
        let held = events.iter().find_map(|event| match event {
            Event::Held(who, written) if *who == member => Some(*written),
            _ => None,
        });
        assert_eq!(held, Some(owned), "{member}");
    }
    let seen = *SEEN.lock().unwrap_or_else(PoisonError::into_inner);
    let each_own: [Q; JOINTS] = core::array::from_fn(|index| Q(index as f32));
    assert_eq!(seen, Some(each_own), "each member filled its own slots");

    let calls = calls(&events);
    let bring_up = ["check", "open", "zero", "ping", "health", "configure"];
    let steps = bring_up.into_iter().chain(["read_all", "torque_on"]);
    let expected: Vec<_> = steps
        .flat_map(|call| [("arm", call), ("leg", call)])
        .chain([
            ("leg", "torque_off"),
            ("arm", "torque_off"),
            ("leg", "close"),
            ("arm", "close"),
        ])
        .collect();
    assert_eq!(calls, expected);
}

#[test]
fn a_coverage_gap_fails_before_any_open() {
    let _bus = bus();
    let Err(e) = run::<GAP>() else {
        panic!("actuator 3 is nobody's, yet the run went on");
    };
    assert_eq!(
        e.to_string(),
        "hardware bring-up failed: actuator 3 is owned by no member"
    );
    assert!(matches!(
        hardware_error(&e),
        Some(HardwareError::Coverage(Coverage::Unowned { actuator: 3 }))
    ));
    let calls = calls(&take());
    assert_eq!(calls, [("arm", "check"), ("leg", "check")]);
    assert!(
        SEEN.lock()
            .unwrap_or_else(PoisonError::into_inner)
            .is_none()
    );
}

#[test]
fn an_overlap_fails_before_any_open_naming_both_members() {
    let _bus = bus();
    let Err(e) = run::<OVERLAP>() else {
        panic!("actuator 0 is owned twice, yet the run went on");
    };
    let arm = Member {
        position: 0,
        name: std::any::type_name::<Fake<Arm>>(),
    };
    let leg = Member {
        position: 1,
        name: std::any::type_name::<Fake<Leg>>(),
    };
    assert_eq!(
        hardware_error(&e),
        Some(&HardwareError::Coverage(Coverage::Overlap {
            actuator: 0,
            first: arm,
            second: leg,
        }))
    );
    assert_eq!(
        e.to_string(),
        "hardware bring-up failed: actuator 0 is owned by member 0 (Fake) and member 1 (Fake)"
    );
    assert_eq!(calls(&take()), [("arm", "check"), ("leg", "check")]);
}

#[test]
fn a_member_error_names_its_position_and_type_and_keeps_its_data() {
    let _bus = bus();
    let Err(e) = run::<FAILS_OPEN>() else {
        panic!("the leg failed to open, yet the run went on");
    };
    assert_eq!(
        e.to_string(),
        "hardware bring-up failed: member 1 (Fake): port /dev/leg did not answer after 3 attempts"
    );
    let unplugged = Unplugged {
        port: "/dev/leg",
        attempts: 3,
    };
    assert_eq!(
        hardware_error(&e),
        Some(&HardwareError::Member(MemberError::Tail(
            MemberError::Head {
                name: "drivers::Fake<drivers::Leg>",
                error: unplugged,
            }
        )))
    );
    // UserError -> the hardware error -> the member's own.
    let own = e.source().and_then(Error::source);
    assert_eq!(own.and_then(|s| s.downcast_ref()), Some(&unplugged));
    assert_eq!(
        calls(&take()),
        [
            ("arm", "check"),
            ("leg", "check"),
            ("arm", "open"),
            ("leg", "open"),
            ("leg", "torque_off"),
            ("arm", "torque_off"),
            ("leg", "close"),
            ("arm", "close"),
        ]
    );
}

#[test]
fn a_binding_error_names_the_member_like_a_bring_up_error() {
    let _bus = bus();
    let Err(e) = run::<FAILS_BIND>() else {
        panic!("the leg failed to bind, yet the run went on");
    };
    assert_eq!(
        e.to_string(),
        "hardware bring-up failed: member 1 (Fake): port /dev/leg did not answer after 0 attempts"
    );
    let own = e.source().and_then(Error::source);
    assert!(own.and_then(|s| s.downcast_ref::<Unplugged>()).is_some());
    assert!(
        calls(&take()).is_empty(),
        "nothing is called before the bind"
    );
}

#[test]
fn a_plugin_no_brand_takes_is_refused_before_any_bind() {
    let _bus = bus();
    take();
    let unknown = kmr_api::Robot::<TwoBrands<COVERED>>::new()
        .add_plugins(Bus::by_id("ARM00001").bus("elbow"))
        .add_controller(EachTick, write_all)
        .run();
    assert!(
        matches!(
            unknown,
            Err(UserError::UnknownBus {
                label: "elbow",
                port: "ARM00001"
            })
        ),
        "{unknown:?}"
    );
    let unlabelled = kmr_api::Robot::<TwoBrands<COVERED>>::new()
        .add_plugins(Bus::by_id("ARM00001"))
        .add_controller(EachTick, write_all)
        .run();
    assert!(
        matches!(
            unlabelled,
            Err(UserError::LabelRequired { port: "ARM00001" })
        ),
        "{unlabelled:?}"
    );
    assert!(take().is_empty());
}

#[test]
fn a_bus_label_two_brands_share_is_refused_before_any_bind() {
    let _bus = bus();
    take();
    let hardware = HCons {
        head: Part::<Arm>::new(evens()),
        tail: HCons {
            head: Part::<Twin>::new(odds()),
            tail: HNil,
        },
    };
    let plugin = Binding {
        port: kmr_adapters::binding::Port::ById("ARM00001"),
        baud: None,
        bus: Some("arm"),
    };
    let res = kmr_api::__bind(hardware, &[plugin]);
    assert!(
        matches!(res, Err(UserError::SharedBusLabel { label: "arm" })),
        "{:?}",
        res.err()
    );
    assert!(take().is_empty());
}

/// Scenario `CASE` with a plugin per brand and a zero hold, bound but not run, from an empty
/// log.
fn bound<const CASE: u8>() -> Result<
    kmr_api::Bound<TwoBrands<CASE>, impl kmr_api::Drive<{ kmr_api::N }> + use<CASE>>,
    UserError,
> {
    take();
    kmr_api::Robot::<TwoBrands<CASE>>::new()
        .set_dt(Duration::from_millis(1))
        .set_hold(Duration::ZERO)
        .add_plugins(Bus::by_dev("/dev/leg").bus("leg"))
        .add_plugins(Bus::by_id("ARM00001").bus("arm"))
        .add_controller(EachTick, write_all)
        .bind()
}

#[test]
fn bind_reaches_the_drivers_before_any_io_and_the_run_hands_them_back() {
    let _bus = bus();
    let mut bound = bound::<COVERED>().expect("the plugins bind");
    let drivers = bound.drivers().expect("two plugins, two members");
    assert_eq!(
        (drivers.head.port, drivers.tail.head.port),
        ("ARM00001", "/dev/leg")
    );
    assert_eq!(
        take(),
        [
            Event::Bound("arm", vec!["ARM00001"]),
            Event::Bound("leg", vec!["/dev/leg"])
        ],
        "binding does no I/O"
    );

    let back = bound.run().expect("the run ends on its shutdown");
    let back = back.expect("the drivers come back");
    assert_eq!(
        (back.head.port, back.tail.head.port),
        ("ARM00001", "/dev/leg")
    );
    let calls = calls(&take());
    assert_eq!(calls.first(), Some(&("arm", "check")));
    assert_eq!(calls.last(), Some(&("arm", "close")), "handed back closed");
}

#[test]
fn a_member_changed_before_the_run_is_the_one_the_run_drives() {
    let _bus = bus();
    let mut bound = bound::<COVERED>().expect("the plugins bind");
    if let Some(drivers) = bound.drivers() {
        drivers.tail.head.fail_open = true;
    }
    let Err(e) = bound.run() else {
        panic!("the leg was told to fail its open, yet the run went on");
    };
    assert_eq!(
        e.to_string(),
        "hardware bring-up failed: member 1 (Fake): port /dev/leg did not answer after 3 attempts"
    );
    let full = e.source().map(ToString::to_string);
    assert!(
        full.as_deref()
            .is_some_and(|full| full.starts_with("member 1 (&mut drivers::Fake<drivers::Leg>)")),
        "{full:?}"
    );
    let own = e.source().and_then(Error::source);
    let unplugged = Unplugged {
        port: "/dev/leg",
        attempts: 3,
    };
    assert_eq!(own.and_then(|s| s.downcast_ref()), Some(&unplugged));
}

#[test]
fn without_a_plugin_bind_gives_no_drivers_and_the_run_simulates() {
    let _bus = bus();
    take();
    let mut bound = kmr_api::Robot::<TwoBrands<COVERED>>::new()
        .set_dt(Duration::from_millis(1))
        .add_controller(EachTick, write_all)
        .bind()
        .expect("nothing to bind");
    assert!(bound.drivers().is_none());
    let back = bound.run().expect("the simulated run ends on its shutdown");
    assert!(back.is_none());
    assert!(take().is_empty(), "no member was built");
}
