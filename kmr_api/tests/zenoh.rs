//! The gateway against a Zenoh peer in this process, over loopback TCP: no router, no other
//! process (quickstart § Phase Z, US4). The tick-exact cases (k, never k+1; superseded; the
//! local write wins) run without a network in kmr_api/src/zenoh/tests.rs.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use kmr_api::test_util::{EXTERNAL_FLAGS, Log, RealTime, Recorder, run_scripted};
use kmr_api::zenoh::{DesiredMsg, SampleMsg, decode_ring, keys};
use kmr_api::{EachTick, Gateway, GatewayConfig, HCons, HISTORY_DEPTH, HNil, Identity, Policy};
use kmr_api::{First, Q, Remote, Sensors, Time};
use zenoh::Wait;

kmr_api::robot!(XmBench);

/// The tick period of every run here: 1 kHz.
const DT: Duration = Duration::from_millis(1);

/// The remote source of these runs: silent past 50 ms, it is timed out.
const CTL: Remote = Remote::allow("ctl-2").timeout(Duration::from_millis(50));

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

thread_local! {
    /// Whether this thread's allocations are counted: const-initialized, no destructor.
    static COUNTED: Cell<bool> = const { Cell::new(false) };
}

/// Allocations of counted threads.
static ALLOCS: AtomicU64 = AtomicU64::new(0);

/// [`System`], counting the allocations of a thread while its window is open (T048).
struct Counting;

// SAFETY: every method forwards its arguments unchanged to `System`, which upholds the
// `GlobalAlloc` contract; the count reads a const thread-local `Cell` and bumps an atomic,
// allocating nothing and touching neither the layout nor the pointers.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count();
        // SAFETY: the caller's contract for `alloc` is passed on as is.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count();
        // SAFETY: the caller's contract for `alloc_zeroed` is passed on as is.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count();
        // SAFETY: `ptr` came from this allocator, which is `System`; the caller's contract holds.
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` came from this allocator, which is `System`; the caller's contract holds.
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// Counts one allocation when the calling thread's window is open; none while it is torn down.
fn count() {
    if COUNTED.try_with(Cell::get).unwrap_or(false) {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Opens or closes the calling thread's window: [`RealTime`] calls it on the tick thread.
fn window(open: bool) {
    COUNTED.set(open);
}

/// Runs `robot` for `ticks` real ticks on a recorder logging to `log`.
fn run<C: kmr_api::Drive<N>>(
    robot: kmr_api::Robot<N, __Spec, C, kmr_api::Filled>,
    ticks: u64,
    log: &Log<N>,
) {
    let mut host = RealTime::new(ticks);
    host.external = Some(&EXTERNAL_FLAGS);
    host.window = window;
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
    let before = ALLOCS.load(Ordering::Relaxed);
    run(robot, TICKS, &log);
    let tick_allocs = ALLOCS.load(Ordering::Relaxed) - before;
    asker.join().expect("the ring query thread");

    // T048 (SC-003, SRS NFR-1): from tick 1 to the stop, gateway on.
    assert_eq!(tick_allocs, 0, "allocations on the tick thread");

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
        .add_remote_source(EachTick, CTL, idle);
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

/// Allocates once, on tick 5.
fn allocating(time: &Time, _state: &mut State, _sensors: &Sensors) {
    if time.tick() == 5 {
        std::hint::black_box(Vec::<u8>::with_capacity(16));
    }
}

#[test]
fn the_window_counts_an_allocation_of_the_tick_thread() {
    let _serial = serial();
    let log = Log::new();
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_controller(EachTick, allocating);
    let before = ALLOCS.load(Ordering::Relaxed);
    run(robot, 20, &log);
    assert_eq!(ALLOCS.load(Ordering::Relaxed) - before, 1);
}

/// Publishes `msgs` on `key` from `peer` once the gateway's subscriber is matched; never drops.
fn send(peer: &zenoh::Session, key: String, msgs: Vec<DesiredMsg<N>>) -> thread::JoinHandle<()> {
    let publisher = peer
        .declare_publisher(key)
        .congestion_control(zenoh::qos::CongestionControl::Block)
        .wait()
        .expect("a Desired publisher");
    thread::spawn(move || {
        let start = Instant::now();
        while !publisher
            .matching_status()
            .wait()
            .is_ok_and(|status| status.matching())
        {
            assert!(
                start.elapsed() < Duration::from_secs(10),
                "the gateway never matched"
            );
            thread::sleep(Duration::from_millis(2));
        }
        for msg in msgs {
            publisher.put(msg.encode()).wait().expect("a put");
        }
    })
}

/// A Desired of `seq` writing `q` to actuator `i`.
fn desired(seq: u64, i: usize, q: f32) -> DesiredMsg<N> {
    let mut msg = DesiredMsg::<N>::EMPTY;
    msg.seq = seq;
    msg.q[i] = q;
    msg.written[0] = 1 << i;
    msg
}

/// Waits until `done` holds, for at most 10 s.
fn until(done: impl Fn() -> bool) {
    let start = Instant::now();
    while !done() {
        assert!(start.elapsed() < Duration::from_secs(10), "timed out");
        thread::sleep(Duration::from_millis(2));
    }
}

/// Set once a tick read the unauthorized flag.
static UNAUTHORIZED_SEEN: AtomicBool = AtomicBool::new(false);

/// Records whether this tick's faults carry the unauthorized flag.
fn watch_unauthorized(_time: &Time, state: &mut State, _sensors: &Sensors) {
    if state.faults().is_some_and(|faults| faults.unauthorized()) {
        UNAUTHORIZED_SEEN.store(true, Ordering::Relaxed);
    }
}

#[test]
fn sc_011_a_thousand_desired_from_an_unknown_identity_none_applied_all_counted_flag_set() {
    let _serial = serial();
    let port = free_port();
    let peer = peer(port);
    let gateway = gateway("z-sc11", port);
    let counters = gateway.counters();
    let msgs = (1..=1000).map(|seq| desired(seq, 2, 0.25)).collect();
    let sender = send(&peer, keys::desired("z-sc11", "intruder"), msgs);
    let watcher = thread::spawn({
        let counters = counters.clone();
        move || until(|| counters.unauthorized() == 1000)
    });
    UNAUTHORIZED_SEEN.store(false, Ordering::Relaxed);
    let log = Log::new();
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_plugins(gateway)
        .add_remote_source(EachTick, CTL, idle)
        .add_controller(First, watch_unauthorized);
    run(robot, 1_500, &log);
    sender.join().expect("the sender");
    watcher.join().expect("1000 counted during the run");

    assert_eq!((counters.unauthorized(), counters.accepted()), (1000, 0));
    assert!(log.write.get().is_none(), "nothing written");
    assert!(
        UNAUTHORIZED_SEEN.load(Ordering::Relaxed),
        "the flag reached a tick"
    );
}

#[test]
fn sc_012_a_desired_with_an_old_seq_is_dropped_and_counted() {
    let _serial = serial();
    let port = free_port();
    let peer = peer(port);
    let gateway = gateway("z-sc12", port);
    let counters = gateway.counters();
    let msgs = [10, 10, 9, 11]
        .into_iter()
        .map(|seq| desired(seq, 1, 0.125))
        .collect();
    let sender = send(&peer, keys::desired("z-sc12", "ctl-2"), msgs);
    let watcher = thread::spawn({
        let counters = counters.clone();
        move || until(|| counters.accepted() + counters.out_of_order() == 4)
    });
    let log = Log::new();
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_plugins(gateway)
        .add_remote_source(EachTick, CTL, idle);
    run(robot, 1_000, &log);
    sender.join().expect("the sender");
    watcher.join().expect("all four handled during the run");

    assert_eq!((counters.accepted(), counters.out_of_order()), (2, 2));
}

/// The first tick a fallback ran, `u64::MAX` before one did.
static FIRST_FALLBACK: AtomicU64 = AtomicU64::new(u64::MAX);

/// Records the first tick it runs in.
fn hold_still(time: &Time, _state: &mut State, _sensors: &Sensors) {
    FIRST_FALLBACK.fetch_min(time.tick(), Ordering::Relaxed);
}

#[test]
fn sc_012_a_silent_source_raises_the_flag_within_timeout_plus_one_tick_and_its_fallback_runs() {
    let _serial = serial();
    let port = free_port();
    let _peer = peer(port);
    FIRST_FALLBACK.store(u64::MAX, Ordering::Relaxed);
    let log = Log::new();
    let quiet = Remote::allow("ctl-2").timeout(Duration::from_millis(20));
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_plugins(gateway("z-quiet", port))
        .add_remote_source(EachTick, quiet, hold_still);
    run(robot, 100, &log);

    // Silent 20 ms at 1 ms a tick by tick 19: the flag is up from tick 20.
    assert_eq!(FIRST_FALLBACK.load(Ordering::Relaxed), 20);
}

/// Throwaway certificates for one test run, minted with the openssl CLI: a CA, the gateway
/// (`arm-1`, valid for 127.0.0.1), the peers `ctl-2` and `viz-1`, and `rogue` from another CA.
mod tls {
    use std::path::{Path, PathBuf};
    use std::process::Command;

    /// Runs openssl with `args` in `dir`.
    fn openssl(dir: &Path, args: &[&str]) {
        let status = Command::new("openssl")
            .args(args)
            .current_dir(dir)
            .status()
            .expect("the openssl CLI");
        assert!(status.success(), "openssl {args:?}");
    }

    /// A self-signed CA `name` in `dir`.
    fn ca(dir: &Path, name: &str) {
        let (crt, key) = (format!("{name}.crt"), format!("{name}.key"));
        let subject = format!("/CN={name}");
        openssl(
            dir,
            &[
                "req",
                "-x509",
                "-newkey",
                "ec",
                "-pkeyopt",
                "ec_paramgen_curve:prime256v1",
                "-nodes",
                "-days",
                "1",
                "-subj",
                &subject,
                "-keyout",
                &key,
                "-out",
                &crt,
            ],
        );
    }

    /// A certificate with common name `name` signed by `ca`, in `dir`.
    fn leaf(dir: &Path, ca: &str, name: &str) {
        let (crt, key, csr) = (
            format!("{name}.crt"),
            format!("{name}.key"),
            format!("{name}.csr"),
        );
        let subject = format!("/CN={name}");
        openssl(
            dir,
            &[
                "req",
                "-newkey",
                "ec",
                "-pkeyopt",
                "ec_paramgen_curve:prime256v1",
                "-nodes",
                "-subj",
                &subject,
                "-keyout",
                &key,
                "-out",
                &csr,
            ],
        );
        let ext = format!("{name}.ext");
        std::fs::write(
            dir.join(&ext),
            "subjectAltName=IP:127.0.0.1,DNS:localhost\nbasicConstraints=CA:FALSE\n",
        )
        .expect("the extension file");
        let (ca_crt, ca_key) = (format!("{ca}.crt"), format!("{ca}.key"));
        openssl(
            dir,
            &[
                "x509",
                "-req",
                "-in",
                &csr,
                "-CA",
                &ca_crt,
                "-CAkey",
                &ca_key,
                "-CAcreateserial",
                "-days",
                "1",
                "-extfile",
                &ext,
                "-out",
                &crt,
            ],
        );
    }

    /// Mints every certificate into a fresh directory; returns it.
    pub fn mint() -> PathBuf {
        let dir = std::env::temp_dir().join(format!("kmr-zenoh-tls-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("the certificate directory");
        ca(&dir, "ca");
        ca(&dir, "other-ca");
        for name in ["arm-1", "ctl-2", "viz-1"] {
            leaf(&dir, "ca", name);
        }
        leaf(&dir, "other-ca", "rogue");
        dir
    }

    /// `dir/file` as a `'static` path string.
    pub fn path(dir: &Path, file: &str) -> &'static str {
        String::leak(dir.join(file).to_string_lossy().into_owned())
    }
}

/// A peer session of identity `name` connecting over mutual TLS to `port`, its CA `ca`.
fn tls_peer(dir: &std::path::Path, name: &str, ca: &str, port: u16) -> zenoh::Session {
    let mut config = zenoh::Config::default();
    let file = |ext: &str| format!("\"{}\"", tls::path(dir, &format!("{name}.{ext}")));
    for (key, value) in [
        ("mode", "\"peer\"".to_owned()),
        ("scouting/multicast/enabled", "false".to_owned()),
        ("listen/endpoints", "[]".to_owned()),
        ("connect/endpoints", format!("[\"tls/127.0.0.1:{port}\"]")),
        (
            "transport/link/tls/root_ca_certificate",
            format!("\"{}\"", tls::path(dir, &format!("{ca}.crt"))),
        ),
        ("transport/link/tls/connect_certificate", file("crt")),
        ("transport/link/tls/connect_private_key", file("key")),
        ("transport/link/tls/enable_mtls", "true".to_owned()),
    ] {
        config.insert_json5(key, &value).expect("a valid setting");
    }
    zenoh::open(config).wait().expect("the peer session")
}

#[test]
#[ignore = "mints a throwaway CA with the openssl CLI; run with `--ignored` where openssl exists"]
fn mutual_tls_admits_known_certificates_and_the_acl_and_rule_sort_them() {
    let _serial = serial();
    let dir = tls::mint();
    let port = free_port();
    let gateway = Gateway::new(GatewayConfig {
        instance: "z-tls",
        identity: Identity::files(
            tls::path(&dir, "arm-1.crt"),
            tls::path(&dir, "arm-1.key"),
            tls::path(&dir, "ca.crt"),
        ),
        read_history: &["viz-1"],
    })
    .listen(format!("tls/127.0.0.1:{port}"));
    let counters = gateway.counters();

    let peers = thread::spawn({
        let dir = dir.clone();
        let counters = counters.clone();
        move || {
            // Waits for the gateway to listen.
            thread::sleep(Duration::from_millis(300));
            let viz = tls_peer(&dir, "viz-1", "ca", port);
            let heard = Arc::new(AtomicU64::new(0));
            let count = Arc::clone(&heard);
            let _history = viz
                .declare_subscriber(keys::history("z-tls"))
                .callback(move |_| {
                    count.fetch_add(1, Ordering::Relaxed);
                })
                .wait()
                .expect("viz-1 subscribes");
            let ctl = tls_peer(&dir, "ctl-2", "ca", port);
            // ctl-2 is a source, not a History reader: the ACL never routes History to it.
            let denied = Arc::new(AtomicU64::new(0));
            let leak = Arc::clone(&denied);
            let _denied = ctl
                .declare_subscriber(keys::history("z-tls"))
                .callback(move |_| {
                    leak.fetch_add(1, Ordering::Relaxed);
                })
                .wait()
                .expect("ctl-2 declares a subscriber locally");
            send(
                &ctl,
                keys::desired("z-tls", "ctl-2"),
                vec![desired(1, 1, 0.125)],
            )
            .join()
            .expect("ctl-2 sends");
            // viz-1 reaches its own Desired key, where the rule counts it.
            send(
                &viz,
                keys::desired("z-tls", "viz-1"),
                vec![desired(1, 1, 0.5)],
            )
            .join()
            .expect("viz-1 sends");
            until(|| counters.accepted() == 1 && counters.unauthorized() == 1);
            // A certificate from another CA gets no session at all.
            let rogue = std::panic::catch_unwind(|| tls_peer(&dir, "rogue", "other-ca", port));
            if let Ok(rogue) = rogue {
                let publisher = rogue
                    .declare_publisher(keys::desired("z-tls", "ctl-2"))
                    .wait()
                    .expect("a local publisher");
                assert!(
                    !publisher
                        .matching_status()
                        .wait()
                        .is_ok_and(|s| s.matching()),
                    "the rogue peer reached the gateway"
                );
            }
            until(|| heard.load(Ordering::Relaxed) > 100);
            assert_eq!(denied.load(Ordering::Relaxed), 0, "History reached ctl-2");
        }
    });

    let log = Log::new();
    let robot = Robot::new()
        .set_dt(DT)
        .set_limit_policy_all(Policy::Clamp)
        .add_plugins(gateway)
        .add_remote_source(EachTick, CTL, idle);
    run(robot, 4_000, &log);
    peers.join().expect("the peers");

    assert_eq!((counters.accepted(), counters.unauthorized()), (1, 1));
    let _ = std::fs::remove_dir_all(dir);
}
