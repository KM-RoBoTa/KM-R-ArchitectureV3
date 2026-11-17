//! Presence of the actuators through the user API: the two policies reach the adapter, its
//! presence reaches the controllers, and an absent slot gets no command. Never opens a serial
//! device.

use std::num::NonZeroU32;
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use kmr_api::experimental::{JoinPolicy, MissingPolicy, Presence, RobotExt};
use kmr_api::{Bus, EachTick, Mask, Policy, Q, Sensors, Spec, Time, UserError};
use kmr_core::hal::{Batch, Claim, Command, Faults, Group, Hal, Overrides, Sample, seal};

kmr_api::robot!(XmBench);

/// The signal bus is process-global: every test that runs takes this first.
static BUS: Mutex<()> = Mutex::new(());

fn bus() -> MutexGuard<'static, ()> {
    BUS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The slot that leaves the run.
const GONE: usize = 2;

/// The tick from which slot `GONE` is absent.
const FLIP: u64 = 3;

/// The tick the controller stops the run at.
const STOP: u64 = 4;

/// What one run of the fake adapter saw.
#[derive(Default)]
struct Log {
    /// The overrides `check` was given.
    overrides: Vec<Overrides>,
    /// The written `Q` mask of every `write`, in order.
    written: Vec<Mask<N>>,
    /// The number of `upkeep` calls.
    upkeeps: usize,
    /// `(tick, present(), present_at(GONE))` as each controller call saw them.
    seen: Vec<(u64, Mask<N>, bool)>,
}

static LOG: Mutex<Log> = Mutex::new(Log {
    overrides: Vec::new(),
    written: Vec::new(),
    upkeeps: 0,
    seen: Vec::new(),
});

fn log() -> MutexGuard<'static, Log> {
    LOG.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The log so far, emptied.
fn take() -> Log {
    std::mem::take(&mut *log())
}

/// Every slot but `GONE`.
fn without_gone() -> Mask<N> {
    let mut mask = Mask::ALL;
    mask.clear(GONE);
    mask
}

/// A fake adapter that owns every slot and loses slot `GONE` on the read of tick `FLIP`.
struct Fading {
    /// `read` calls so far: the read of tick `k` is call `k + 1`.
    reads: u64,
}

impl seal::Adapter for Fading {}

impl Hal<N> for Fading {
    type Error = core::convert::Infallible;

    fn check(&mut self, overrides: Overrides) -> Result<Claim<N>, Self::Error> {
        log().overrides.push(overrides);
        Ok(Claim {
            owned: Mask::ALL,
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
        self.reads += 1;
        Mask::NONE
    }

    fn write(&mut self, command: Command<'_, N>, _batch: &Batch<'_, N>, _faults: &mut Faults<N>) {
        log().written.push(command.written.q);
    }

    fn present(&self) -> Mask<N> {
        if self.reads > FLIP {
            without_gone()
        } else {
            Mask::ALL
        }
    }

    fn upkeep(&mut self) {
        log().upkeeps += 1;
    }

    fn hold(&mut self, _last: Command<'_, N>) {}

    fn torque_off(&mut self) {}

    fn close(&mut self) {}
}

/// Binds every plugin to a fading adapter.
struct Fades;

impl Spec<N> for Fades {
    type Drivers = kmr_api::HCons<Fading, kmr_api::HNil>;

    const LINK_NAMES: &'static [&'static str] = &["base"];
    const PRISMATIC: [u64; kmr_api::MASK_WORDS] = [0; kmr_api::MASK_WORDS];
    const CONTINUOUS: [u64; kmr_api::MASK_WORDS] = [0; kmr_api::MASK_WORDS];
    const LOWER: [Q; N] = [Q(-1.0); N];
    const UPPER: [Q; N] = [Q(1.0); N];
    const SPEED: [kmr_api::Qd; N] = [kmr_api::Qd(1.0); N];
    const EFFORT: [kmr_api::Tau; N] = [kmr_api::Tau(1.0); N];
    const PRESET: Option<&'static str> = None;

    fn bind(_plugins: &[kmr_api::Binding]) -> Result<Self::Drivers, UserError> {
        Ok(kmr_api::HCons {
            head: Fading { reads: 0 },
            tail: kmr_api::HNil,
        })
    }

    fn declare(_declared: kmr_api::__Declared<N>) -> Result<Self::Drivers, UserError> {
        Self::bind(&[])
    }
}

/// Records what presence it sees, writes every position through a group, and stops at `STOP`.
fn watch(time: &Time, robot: &mut State, _s: &Sensors) {
    log()
        .seen
        .push((time.tick(), robot.present(), robot.present_at(GONE)));
    if let Err(e) = robot.group([0, 1, 2, 3]).set_all_q([Q(0.0); N]) {
        panic!("a write to every slot, one absent, failed: {e}");
    }
    if time.tick() == STOP {
        kmr_api::shutdown!("test: stop tick reached");
    }
}

#[test]
fn fr_5_5_presence_reaches_controllers_and_absent_slots_get_no_command() {
    let _bus = bus();
    take();
    let res = kmr_api::Robot::<N, Fades>::new()
        .set_dt(Duration::from_millis(1))
        .set_limit_policy_all(Policy::Clamp)
        .set_missing_policy(MissingPolicy::Tolerate)
        .set_join_policy(JoinPolicy::Declared)
        .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
        .add_controller(EachTick, watch)
        .run();
    assert!(res.is_ok(), "{res:?}");
    let log = take();

    let [overrides] = log.overrides[..] else {
        panic!("one check expected: {:?}", log.overrides);
    };
    assert_eq!(overrides.missing, MissingPolicy::Tolerate);
    assert_eq!(overrides.join, JoinPolicy::Declared);
    assert!(log.upkeeps > 0, "the join policy never reached the loop");

    let seen: Vec<_> = (0..=STOP)
        .map(|tick| {
            let present = if tick >= FLIP {
                without_gone()
            } else {
                Mask::ALL
            };
            (tick, present, tick < FLIP)
        })
        .collect();
    assert_eq!(log.seen, seen);

    let written: Vec<_> = (0..=STOP)
        .map(|tick| {
            if tick >= FLIP {
                without_gone()
            } else {
                Mask::ALL
            }
        })
        .collect();
    assert_eq!(log.written, written);
}

#[test]
fn present_at_past_n_is_false() {
    let _bus = bus();
    take();
    let res = kmr_api::Robot::<N, Fades>::new()
        .set_dt(Duration::from_millis(1))
        .set_limit_policy_all(Policy::Clamp)
        .add_plugins(Bus::by_id("kmr-test-no-such-serial"))
        .add_controller(EachTick, |time: &Time, robot: &mut State, _s: &Sensors| {
            assert!(robot.present_at(N - 1));
            assert!(!robot.present_at(N));
            assert!(!robot.present_at(usize::MAX));
            if time.tick() == 0 {
                kmr_api::shutdown!("test: one tick");
            }
        })
        .run();
    assert!(res.is_ok(), "{res:?}");
}

// ---- Joined, left and the join ramp, on a scripted host ----

/// Where each slot reads: one tenth of a radian per `read` call so far, so a joiner's start
/// differs from where it is later.
fn position(reads: u64) -> Q {
    Q(f32::from(u16::try_from(reads).unwrap_or(u16::MAX)) / 10.0)
}

/// A fake adapter that owns every slot, reads every present one at [`position`], and is
/// missing slot `GONE` on the ticks `away` names. Its full read is at "tick" -1.
struct Plugged {
    /// Calls to `read_all` and `read` so far.
    reads: u64,
    /// Whether slot `GONE` is absent at a tick.
    away: fn(i64) -> bool,
    /// Where every slot reads after `reads` calls.
    at: fn(u64) -> Q,
}

/// The `Q` written to slot `GONE` by each `write`, `None` when it was not written.
static WRITES: Mutex<Vec<Option<Q>>> = Mutex::new(Vec::new());

impl Plugged {
    /// The tick of the latest read: -1 for the full read.
    fn tick(&self) -> i64 {
        i64::try_from(self.reads).unwrap_or(i64::MAX) - 2
    }
}

impl seal::Adapter for Plugged {}

impl Hal<N> for Plugged {
    type Error = core::convert::Infallible;

    fn check(&mut self, overrides: Overrides) -> Result<Claim<N>, Self::Error> {
        Fading { reads: 0 }.check(overrides)
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

    fn read_all(&mut self, sample: Sample<'_, N>) -> Result<(), Self::Error> {
        self.reads += 1;
        *sample.q = [(self.at)(self.reads); N];
        Ok(())
    }

    fn torque_on(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, sample: Sample<'_, N>, _faults: &mut Faults<N>) -> Mask<N> {
        self.reads += 1;
        let present = self.present();
        for i in present.iter() {
            if let Some(q) = sample.q.get_mut(i) {
                *q = (self.at)(self.reads);
            }
        }
        present
    }

    fn write(&mut self, command: Command<'_, N>, _: &Batch<'_, N>, _: &mut Faults<N>) {
        let q = command.written.q.get(GONE).then_some(command.q[GONE]);
        WRITES
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(q);
    }

    fn present(&self) -> Mask<N> {
        if (self.away)(self.tick()) {
            without_gone()
        } else {
            Mask::ALL
        }
    }

    fn hold(&mut self, _last: Command<'_, N>) {}

    fn torque_off(&mut self) {}

    fn close(&mut self) {}
}

/// What one tick of a scripted run saw.
#[derive(Debug, PartialEq)]
struct Seen {
    elapsed: Duration,
    joined: Mask<N>,
    left: Mask<N>,
    /// The ramp's target of every slot, the controller's being 0 rad.
    target: [Q; N],
    easing: Mask<N>,
    /// `q_at(GONE)`, `None` when it fails.
    gone: Option<Q>,
    /// Whether `q_at(GONE)` failed with `NoSample` at depth 0.
    gone_unsampled: bool,
    /// `q()`.
    all: Option<[Q; N]>,
}

static SEEN: Mutex<Vec<Seen>> = Mutex::new(Vec::new());

/// The ticks the recording controller skips `apply` on.
static SKIP: Mutex<fn(u64) -> bool> = Mutex::new(|_| false);

/// The join ramp the scripted runs take: four ticks.
const RAMP: Duration = Duration::from_millis(4);

/// Records what each tick sees through the presence queries and a [`JoinRamp`] of [`RAMP`]
/// towards 0 rad.
fn ramp(
    ramp: &mut kmr_api::experimental::JoinRamp<N>,
    time: &Time,
    robot: &mut State,
    _: &Sensors,
) {
    let mut target = [Q(0.0); N];
    let skip = *SKIP.lock().unwrap_or_else(PoisonError::into_inner);
    if !skip(time.tick()) {
        ramp.apply(time, robot, &mut target);
    }
    SEEN.lock()
        .unwrap_or_else(PoisonError::into_inner)
        .push(Seen {
            elapsed: time.elapsed(),
            joined: robot.joined(),
            left: robot.left(),
            target,
            easing: ramp.easing(),
            gone: robot.q_at(GONE).ok(),
            gone_unsampled: matches!(
                robot.q_at(GONE),
                Err(kmr_api::StateError::NoSample { depth: 0 })
            ),
            all: robot.q(),
        });
}

/// `ticks` ticks of a scripted run where slot `GONE` is absent at the ticks `away` names.
fn scripted(away: fn(i64) -> bool, ticks: u64) -> Vec<Seen> {
    scripted_skipping(away, |_| false, ticks)
}

/// [`scripted`], the controller skipping `apply` on the ticks `skip` names.
fn scripted_skipping(away: fn(i64) -> bool, skip: fn(u64) -> bool, ticks: u64) -> Vec<Seen> {
    let _bus = bus();
    SEEN.lock().unwrap_or_else(PoisonError::into_inner).clear();
    *SKIP.lock().unwrap_or_else(PoisonError::into_inner) = skip;
    let robot = kmr_api::Robot::<N, Fades>::new()
        .set_dt(kmr_api::test_util::DT)
        .set_limit_policy_all(Policy::Clamp)
        .set_missing_policy(MissingPolicy::Tolerate)
        .add_controller(
            EachTick,
            kmr_api::with(kmr_api::experimental::JoinRamp::new(RAMP), ramp),
        );
    let mut host = kmr_api::test_util::Ticking::new(std::time::Instant::now(), ticks);
    let mut drivers = kmr_api::HCons {
        head: Plugged {
            reads: 0,
            away,
            at: position,
        },
        tail: kmr_api::HNil,
    };
    let stop = kmr_api::test_util::run_scripted(robot, &mut host, &mut drivers);
    assert_eq!(stop.ok(), Some(kmr_api::test_util::Stop::Shutdown));
    std::mem::take(&mut *SEEN.lock().unwrap_or_else(PoisonError::into_inner))
}

/// Slot `GONE` alone.
fn gone() -> Mask<N> {
    let mut mask = Mask::NONE;
    mask.set(GONE);
    mask
}

#[test]
fn joined_and_left_name_the_slots_that_changed_at_that_tick() {
    // Absent from the start, present at 2 and 3, absent at 4, back at 5.
    let seen = scripted(|tick| tick < 2 || tick == 4, 7);

    let changes: Vec<_> = seen.iter().map(|s| (s.joined, s.left)).collect();
    let none = (Mask::NONE, Mask::NONE);
    assert_eq!(
        changes,
        [
            none,
            none,
            (gone(), Mask::NONE),
            none,
            (Mask::NONE, gone()),
            (gone(), Mask::NONE),
            none
        ]
    );
}

#[test]
fn present_from_the_start_never_joined_and_never_eases() {
    let seen = scripted(|_| false, 3);

    assert!(
        seen.iter()
            .all(|s| s.joined.is_empty() && s.left.is_empty())
    );
    assert!(seen.iter().all(|s| s.easing.is_empty()));
    assert!(seen.iter().all(|s| s.target == [Q(0.0); N]));
}

#[test]
fn a_joiner_eases_from_where_it_joined_to_the_target_over_the_ramp() {
    let seen = scripted(|tick| tick < 2, 9);
    let join = seen.get(2).expect("tick 2");
    let start = join.gone.expect("a joiner has its first sample");

    for s in &seen[2..] {
        let age = s.elapsed - join.elapsed;
        let [first, .., last] = s.target;
        assert_eq!((first, last), (Q(0.0), Q(0.0)), "the others are untouched");
        if age < RAMP {
            let left = 1.0 - age.as_secs_f32() / RAMP.as_secs_f32();
            assert!(s.easing == gone(), "{s:?}");
            assert!((s.target[GONE].0 - start.0 * left).abs() < 1e-6, "{s:?}");
        } else {
            assert!(s.easing.is_empty(), "{s:?}");
            assert_eq!(s.target[GONE], Q(0.0));
        }
    }
    assert_eq!(join.target[GONE], start, "no jump at the join tick");
    assert!(
        seen.last()
            .is_some_and(|s| s.elapsed - join.elapsed >= RAMP)
    );
}

#[test]
fn a_slot_that_leaves_mid_ramp_stops_easing_and_starts_over_when_back() {
    // Joins at 2, leaves at 4, back at 6.
    let seen = scripted(|tick| tick < 2 || tick == 4 || tick == 5, 8);
    let (first, back) = (&seen[2], &seen[6]);

    assert!(seen[3].easing == gone());
    assert!(seen[4].easing.is_empty() && seen[5].easing.is_empty());
    assert_eq!(seen[4].target[GONE], Q(0.0));
    assert!(back.easing == gone());
    let restart = back.gone.expect("a joiner has its first sample");
    assert_eq!(back.target[GONE], restart);
    assert_ne!(restart, first.gone.expect("sampled"));
}

#[test]
fn reads_of_an_absent_slot_are_no_sample_one_by_one_and_stale_in_the_array() {
    // Present at 0 and 1, absent from 2 on.
    let seen = scripted(|tick| tick >= 2, 5);

    assert!(seen[1].gone.is_some());
    let last_read = seen[1].all.expect("all sampled")[GONE];
    for s in &seen[2..] {
        assert!(s.gone_unsampled, "{s:?}");
        let all = s.all.expect("the present slots are sampled");
        assert_eq!(all[GONE], last_read, "its last sample, copied forward");
        assert!(all[0] != last_read, "the others are read on");
    }
}

#[test]
fn a_joiner_missed_on_its_join_tick_still_eases_from_where_it_is_then() {
    // Joins at 2; the controller skips `apply` at 2.
    let seen = scripted_skipping(|tick| tick < 2, |tick| tick == 2, 7);

    assert!(seen[2].easing.is_empty());
    let late = &seen[3];
    assert!(late.easing == gone(), "{late:?}");
    assert_eq!(late.target[GONE], late.gone.expect("sampled"));
}

#[test]
fn a_slot_that_left_and_rejoined_between_two_calls_starts_over() {
    // Present but at 3 and 4; the controller skips `apply` from 3 to 5.
    let seen = scripted_skipping(
        |tick| tick == 3 || tick == 4,
        |tick| (3..=5).contains(&tick),
        9,
    );

    assert!(seen[..=2].iter().all(|s| s.easing.is_empty()));
    let back = &seen[6];
    assert!(back.easing == gone(), "{back:?}");
    assert_eq!(back.target[GONE], back.gone.expect("sampled"));
}

#[test]
fn skipped_calls_never_ease_a_slot_that_stayed() {
    let seen = scripted_skipping(|_| false, |tick| (1..=5).contains(&tick), 9);

    assert!(seen.iter().all(|s| s.easing.is_empty()), "{seen:?}");
}

/// The home ramp of the Home scenario: forty ticks.
const HOME_RAMP: Duration = Duration::from_millis(40);

/// The runtime_actuators example's controller on the scripted host: during the home ramp it
/// writes the ramp's target for every joiner of the run, after it every target.
fn sway_home(
    ramp: &mut kmr_api::experimental::JoinRamp<N>,
    time: &Time,
    robot: &mut State,
    _: &Sensors,
) {
    let mut target = [Q(0.0); N];
    ramp.apply(time, robot, &mut target);
    if time.elapsed() < HOME_RAMP {
        for i in ramp.joiners().iter() {
            if let Err(e) = robot.set_q_at(target[i], i) {
                panic!("{e}");
            }
        }
        return;
    }
    robot.set_all_q(target);
}

#[test]
fn a_joiner_that_ends_its_ease_before_home_stays_on_its_target() {
    // Every slot at 0.6 rad; Home latches it at tick 0. `GONE` is unplugged from 5 to 9, eases
    // back over 8 ticks from 10, and must then stay at 0 rad, not go back to Home's line.
    let _bus = bus();
    WRITES
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clear();
    let robot = kmr_api::Robot::<N, Fades>::new()
        .set_dt(kmr_api::test_util::DT)
        .set_limit_policy_all(Policy::Clamp)
        .set_missing_policy(MissingPolicy::Tolerate)
        .add_controller(
            EachTick,
            kmr_api::presets::home::Home::new([Q(0.0); N], HOME_RAMP).controller(),
        )
        .add_controller(
            EachTick,
            kmr_api::with(
                kmr_api::experimental::JoinRamp::new(Duration::from_millis(8)),
                sway_home,
            ),
        );
    let mut host = kmr_api::test_util::Ticking::new(std::time::Instant::now(), 50);
    let mut drivers = kmr_api::HCons {
        head: Plugged {
            reads: 0,
            away: |tick| (5..=9).contains(&tick),
            at: |_| Q(0.6),
        },
        tail: kmr_api::HNil,
    };
    let stop = kmr_api::test_util::run_scripted(robot, &mut host, &mut drivers);
    assert_eq!(stop.ok(), Some(kmr_api::test_util::Stop::Shutdown));
    let writes = std::mem::take(&mut *WRITES.lock().unwrap_or_else(PoisonError::into_inner));

    let gone: Vec<Q> = writes.iter().skip(10).flatten().copied().collect();
    assert!(gone.len() > 30, "{writes:?}");
    for pair in gone.windows(2) {
        assert!(
            (pair[1].0 - pair[0].0).abs() <= 0.6 / 8.0 + 1e-6,
            "a step: {gone:?}"
        );
    }
    assert!(
        writes[20..40].iter().all(|q| *q == Some(Q(0.0))),
        "{writes:?}"
    );
}
