//! The gateway against a Zenoh peer in this process, over loopback TCP: no router, no other
//! process (quickstart § Phase Z, US4). The tick-exact cases (k, never k+1; superseded; the
//! local write wins) run without a network in kmr_api/src/zenoh/tests.rs.

use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use kmr_api::test_util::{EXTERNAL_FLAGS, Log, RealTime, Recorder, run_scripted};
use kmr_api::zenoh::{DesiredMsg, SampleMsg, decode_ring, keys};
use kmr_api::{EachTick, Gateway, GatewayConfig, HCons, HISTORY_DEPTH, HNil, Identity, Policy};
use kmr_api::{Q, Sensors, Time};
use zenoh::Wait;

kmr_api::robot!(XmBench);

/// The tick period of every run here: 1 kHz.
const DT: Duration = Duration::from_millis(1);

/// One run at a time: the external flags are process-global.
static SERIAL: Mutex<()> = Mutex::new(());

/// Takes the file's lock.
fn serial() -> MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A free loopback port.
fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .and_then(|listener| listener.local_addr())
        .map(|addr| addr.port())
        .expect("a free loopback port")
}

/// The test's peer session, listening on `port`.
fn peer(port: u16) -> zenoh::Session {
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\"".to_owned()),
        ("scouting/multicast/enabled", "false".to_owned()),
        ("listen/endpoints", format!("[\"tcp/127.0.0.1:{port}\"]")),
    ] {
        config.insert_json5(key, &value).expect("a valid setting");
    }
    zenoh::open(config).wait().expect("the peer session")
}

/// A gateway of `instance` connecting to the peer on `port`.
fn gateway(instance: &'static str, port: u16) -> Gateway {
    Gateway::new(GatewayConfig {
        instance,
        identity: Identity::loopback(),
        read_history: &[],
    })
    .connect(format!("tcp/127.0.0.1:{port}"))
}

/// A controller that does nothing.
fn idle(_time: &Time, _state: &mut State, _sensors: &Sensors) {}

/// Runs `robot` for `ticks` real ticks on a recorder logging to `log`.
fn run<C: kmr_api::Drive<N>>(
    robot: kmr_api::Robot<N, __Spec, C, kmr_api::Filled>,
    ticks: u64,
    log: &Log<N>,
) {
    let mut host = RealTime::new(ticks);
    host.external = Some(&EXTERNAL_FLAGS);
    let mut drivers = HCons {
        head: Recorder::new(log),
        tail: HNil,
    };
    run_scripted(robot, &mut host, &mut drivers).expect("the run");
}

#[test]
fn us4_as1_each_tick_is_received_or_counted_missed_and_the_ring_is_served() {
    const TICKS: u64 = 10_000;
    let _serial = serial();
    let port = free_port();
    let peer = peer(port);
    let received = Arc::new(Mutex::new(Vec::<u64>::new()));
    let sink = Arc::clone(&received);
    let _subscriber = peer
        .declare_subscriber(keys::history("z-as1"))
        .callback(move |sample| {
            if let Some(msg) = SampleMsg::<N>::decode(&sample.payload().to_bytes()) {
                sink.lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .push(msg.tick);
            }
        })
        .wait()
        .expect("the History subscriber");

    let ring = Arc::new(Mutex::new(Vec::<u64>::new()));
    let asked = Arc::clone(&ring);
    let seen = Arc::clone(&received);
    let querier = peer.clone();
    let asker = thread::spawn(move || {
        let start = Instant::now();
        while seen.lock().unwrap_or_else(PoisonError::into_inner).len() < 2_000 {
            assert!(
                start.elapsed() < Duration::from_secs(20),
                "no sample arrived"
            );
            thread::sleep(Duration::from_millis(5));
        }
        let replies = querier
            .get(keys::ring("z-as1"))
            .wait()
            .expect("the ring query");
        while let Ok(reply) = replies.recv() {
            let samples = reply
                .result()
                .ok()
                .and_then(|s| decode_ring::<N>(&s.payload().to_bytes()))
                .expect("a whole ring");
            let mut asked = asked.lock().unwrap_or_else(PoisonError::into_inner);
            asked.extend(samples.iter().map(|msg| msg.tick));
        }
    });

    let gateway = gateway("z-as1", port);
    let counters = gateway.counters();
    let log = Log::new();
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_plugins(gateway)
        .add_controller(EachTick, idle);
    run(robot, TICKS, &log);
    asker.join().expect("the ring query thread");

    let ticks = received
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone();
    let (first, last) = (ticks[0], ticks[ticks.len() - 1]);
    assert!(ticks.windows(2).all(|w| w[0] < w[1]), "in tick order");
    assert!(last - first > TICKS / 2, "the peer heard most of the run");
    let unheard = (last - first + 1) - ticks.len() as u64;
    assert!(
        unheard <= counters.missed_samples(),
        "{unheard} ticks neither received nor counted missed ({})",
        counters.missed_samples()
    );
    let accounted = counters.published() + counters.missed_samples();
    assert!(
        accounted == TICKS || accounted == TICKS - 1,
        "{accounted} of {TICKS} samples taken or counted missed"
    );

    let ring = ring.lock().unwrap_or_else(PoisonError::into_inner).clone();
    assert_eq!(
        ring.len(),
        HISTORY_DEPTH,
        "the ring holds the last HISTORY_DEPTH"
    );
    assert!(
        ring.windows(2).all(|w| w[0] < w[1]),
        "oldest first: {ring:?}"
    );
}

#[test]
fn us4_as2_a_remote_desired_reaches_the_adapter() {
    let _serial = serial();
    let port = free_port();
    let peer = peer(port);
    let publisher = peer
        .declare_publisher(keys::desired("z-as2", "ctl-2"))
        .wait()
        .expect("the Desired publisher");
    let stop = Arc::new(AtomicBool::new(false));
    let stopped = Arc::clone(&stop);
    let sender = thread::spawn(move || {
        let mut seq = 0;
        while !stopped.load(Ordering::Relaxed) {
            seq += 1;
            let mut msg = DesiredMsg::<N>::EMPTY;
            msg.seq = seq;
            msg.q[1] = 0.125;
            msg.written[0] = 1 << 1;
            publisher.put(msg.encode()).wait().expect("a put");
            thread::sleep(Duration::from_millis(5));
        }
    });

    let gateway = gateway("z-as2", port);
    let counters = gateway.counters();
    let log = Log::new();
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_plugins(gateway)
        .add_controller(EachTick, idle);
    run(robot, 1_000, &log);
    stop.store(true, Ordering::Relaxed);
    sender.join().expect("the sender thread");

    assert!(counters.accepted() > 0, "no Desired accepted");
    let sent = log.write.get().expect("a remote write reached the adapter");
    assert_eq!(sent.q[1], Q(0.125));
    assert!(sent.written.q.get(1) && !sent.written.q.get(0));
}

#[test]
fn a_gateway_without_a_peer_runs_and_publishes_into_nothing() {
    let _serial = serial();
    let gateway = Gateway::new(GatewayConfig {
        instance: "z-alone",
        identity: Identity::loopback(),
        read_history: &[],
    });
    let counters = gateway.counters();
    let log = Log::new();
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_plugins(gateway)
        .add_controller(EachTick, idle);
    run(robot, 200, &log);
    assert!(counters.published() + counters.missed_samples() >= 199);
}
