//! Presence of the actuators through the user API: the two policies reach the adapter, its
//! presence reaches the controllers, and an absent slot gets no command. Never opens a serial
//! device.

use std::num::NonZeroU32;
use std::sync::{Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use kmr_api::{
    Bus, EachTick, JoinPolicy, Mask, MissingPolicy, Policy, Q, Sensors, Spec, Time, UserError,
};
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
