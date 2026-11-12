//! The XIAO adapter on `xiao-api`'s fake link: the test plays the board.

use std::io;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread;
use std::time::Instant;

use tracing_subscriber::layer::SubscriberExt;
use xiao_api::BytesPerSec;
use xiao_api::link::{Board, Fake};
use xiao_api::proto::message::{
    AUDIO_BLOCK_SAMPLES, AccelConfig, AudioBlock, BLE_NAME as PROTO_BLE_NAME, Command, GyroConfig,
    ImuConfig, ImuReading, MIC_SAMPLE_RATE_HZ, Message, Refusal, UART_DEFAULT_BAUD,
};

use super::*;

/// `xiao` on a fake USB board that answers every command.
fn on_usb(xiao: Xiao) -> (Xiao, Board) {
    let (usb, board) = Fake::usb();
    board.auto_answer();
    (xiao.on_link(Opened::Usb(usb)), board)
}

/// The `ImuOn` of the gyroscope alone at the default settings.
const GYRO_ON: Command = Command::ImuOn(ImuConfig {
    accel: None,
    gyro: Some(GyroConfig {
        rate: Rate::Hz104,
        range: GyroRange::Dps500,
    }),
});

#[test]
fn the_defaults_are_the_firmwares() {
    assert_eq!(DEFAULT_BAUD, UART_DEFAULT_BAUD);
    assert_eq!(BLE_NAME, PROTO_BLE_NAME);
    assert_eq!(MIC_RATE_HZ, MIC_SAMPLE_RATE_HZ);
    assert_eq!(BLOCK_SAMPLES, AUDIO_BLOCK_SAMPLES as u64);
}

#[test]
fn the_builder_keeps_what_was_asked() {
    let xiao = Xiao::uart("/dev/ttyUSB0").baud(115_200).imu().mic();
    let link = LinkSpec::Uart {
        port: "/dev/ttyUSB0",
        baud: 115_200,
    };
    let both = ImuSpec {
        accel: Some((Rate::Hz104, AccelRange::G4)),
        gyro: Some((Rate::Hz104, GyroRange::Dps500)),
    };
    assert_eq!((xiao.link, xiao.imu, xiao.mic), (link, both, true));

    let ble = Xiao::ble().named("robot-imu").attempts(5).baud(9600);
    let named = LinkSpec::Ble {
        name: Some("robot-imu"),
        attempts: 5,
    };
    assert_eq!(ble.link, named, "baud does nothing on BLE");
    assert_eq!(Xiao::usb().named("x").attempts(1).link, LinkSpec::Usb);
}

// ── X05: needs and bring-up ────────────────────────────────────────────────

#[test]
fn needs_count_what_was_asked() {
    let both = Xiao::usb().imu().mic().needs();
    assert_eq!((both.imus, both.mics, both.mic_rate_hz), (1, 1, 16_000));

    let gyro = Xiao::usb().gyro().needs();
    assert_eq!((gyro.imus, gyro.mics), (1, 0));

    let nothing = Xiao::usb().needs();
    assert_eq!((nothing.imus, nothing.mics), (0, 0));
}

#[test]
fn max_silence_is_ten_periods_of_the_slowest_stream() {
    let silence = |xiao: Xiao| xiao.needs().max_silence;

    // 10 / 104 Hz = 96.15 ms, rounded up.
    assert_eq!(silence(Xiao::usb().gyro()), Duration::from_millis(97));
    assert_eq!(silence(Xiao::usb().imu().mic()), Duration::from_millis(97));
    // A block of 96 samples at 16 kHz every 6 ms.
    assert_eq!(silence(Xiao::usb().mic()), Duration::from_millis(60));
    let two_rates = ImuSpec {
        accel: Some((Rate::Hz12_5, AccelRange::G2)),
        gyro: Some((Rate::Hz833, GyroRange::Dps2000)),
    };
    assert_eq!(
        silence(Xiao::usb().imu_with(two_rates)),
        Duration::from_millis(800)
    );
    assert_eq!(silence(Xiao::usb()), Duration::ZERO);
}

#[test]
fn building_a_xiao_opens_nothing() {
    let xiao = Xiao::uart("/nonexistent/xiao-port").imu().mic();
    let _ = xiao.needs();
    assert!(xiao.opened.is_none(), "no link before open");

    let (xiao, board) = on_usb(Xiao::usb().imu().mic());
    let _ = xiao.needs();
    drop(xiao);
    assert_eq!(board.commands(), [], "no sensor turned on before open");
}

#[test]
fn open_turns_on_only_what_was_asked() {
    let (mut xiao, board) = on_usb(Xiao::usb().gyro());
    xiao.open().unwrap();
    assert_eq!(
        board.commands(),
        [GYRO_ON],
        "the accelerometer stays off (FW-6)"
    );
    assert!(xiao.microphone.is_none());

    let (mut xiao, board) = on_usb(Xiao::usb().imu().mic());
    xiao.open().unwrap();
    let both = Command::ImuOn(ImuConfig {
        accel: Some(AccelConfig {
            rate: Rate::Hz104,
            range: AccelRange::G4,
        }),
        gyro: Some(GyroConfig {
            rate: Rate::Hz104,
            range: GyroRange::Dps500,
        }),
    });
    assert_eq!(board.commands(), [both, Command::MicOn]);
}

#[test]
fn two_rates_turn_the_halves_on_one_after_the_other() {
    let accel = AccelConfig {
        rate: Rate::Hz52,
        range: AccelRange::G8,
    };
    let gyro = GyroConfig {
        rate: Rate::Hz416,
        range: GyroRange::Dps1000,
    };
    let spec = ImuSpec {
        accel: Some((accel.rate, accel.range)),
        gyro: Some((gyro.rate, gyro.range)),
    };
    let (mut xiao, board) = on_usb(Xiao::usb().imu_with(spec));
    xiao.open().unwrap();

    let first = ImuConfig {
        accel: Some(accel),
        gyro: None,
    };
    let both = ImuConfig {
        accel: Some(accel),
        gyro: Some(gyro),
    };
    assert_eq!(
        board.commands(),
        [Command::ImuOn(first), Command::ImuOn(both)]
    );
}

#[test]
fn nothing_asked_opens_no_link() {
    let mut xiao = Xiao::uart("/nonexistent/xiao-port");
    assert_eq!(xiao.open(), Ok(()));
    assert!(xiao.opened.is_none());
}

#[test]
fn a_busy_sensor_names_the_sensor_and_the_holding_link() {
    let (mut xiao, board) = on_usb(Xiao::usb().mic());
    board.refuse_next(Refusal::Busy { held_by: Link::Ble });
    assert_eq!(
        xiao.open(),
        Err(XiaoError::Busy {
            sensor: Sensor::Mic,
            held_by: Link::Ble
        })
    );

    let (uart, board) = Fake::uart(Baud(921_600));
    board.auto_answer();
    board.refuse_next(Refusal::Busy { held_by: Link::Usb });
    let mut xiao = Xiao::uart("/dev/ttyAMA0").imu().on_link(Opened::Uart(uart));
    let busy = xiao.open().unwrap_err();
    assert_eq!(
        busy,
        XiaoError::Busy {
            sensor: Sensor::Imu,
            held_by: Link::Usb
        }
    );
    let text = busy.to_string();
    assert!(text.contains("IMU") && text.contains("USB"), "{text}");
}

#[test]
fn a_refused_mic_turns_the_imu_back_off() {
    let (usb, board) = Fake::usb();
    board.auto_answer();
    // Another value on this link holds the microphone: `xiao-api` refuses
    // the second without asking the board.
    let held = xiao_api::Mic::on(&usb).unwrap();
    let mut xiao = Xiao::usb().gyro().mic().on_link(Opened::Usb(usb));

    assert_eq!(
        xiao.open(),
        Err(XiaoError::Busy {
            sensor: Sensor::Mic,
            held_by: Link::Usb
        })
    );
    assert_eq!(
        board.commands(),
        [Command::MicOn, GYRO_ON, Command::ImuOff],
        "the gyroscope it turned on is off again"
    );
    assert!(xiao.opened.is_none() && xiao.halves.is_none() && xiao.microphone.is_none());
    drop(held);
}

#[test]
fn a_slow_link_gives_both_speeds() {
    let (uart, board) = Fake::uart(Baud(115_200));
    board.auto_answer();
    board.refuse_next(Refusal::TooSlow {
        needs: 34_000,
        link_has: 11_520,
    });
    let mut xiao = Xiao::uart("/dev/ttyAMA0")
        .baud(115_200)
        .mic()
        .on_link(Opened::Uart(uart));

    let slow = xiao.open().unwrap_err();
    assert_eq!(
        slow,
        XiaoError::TooSlow {
            sensor: Sensor::Mic,
            needs_bytes_per_s: 34_000,
            link_has_bytes_per_s: 11_520
        }
    );
    let text = slow.to_string();
    assert!(text.contains("34000") && text.contains("11520"), "{text}");
    assert_eq!(board.commands(), [Command::MicOn]);
}

#[test]
fn no_board_names_the_link() {
    let port = "/nonexistent/xiao-port";
    let mut xiao = Xiao::uart(port).gyro();
    let refused = xiao.open().unwrap_err();
    let link = LinkSpec::Uart {
        port,
        baud: 921_600,
    };
    assert!(
        matches!(refused, XiaoError::Io { link: named, .. } if named == link),
        "{refused:?}"
    );
    assert!(refused.to_string().contains(port), "{refused}");

    // A link with a silent board behind it: no answer within 2 s.
    let (usb, _board) = Fake::usb();
    let mut xiao = Xiao::usb().mic().on_link(Opened::Usb(usb));
    assert_eq!(
        xiao.open(),
        Err(XiaoError::Timeout {
            link: LinkSpec::Usb,
            attempts: 1
        })
    );
}

#[test]
fn every_xiao_api_error_gets_its_sensor_and_link() {
    let uart = LinkSpec::Uart {
        port: "/dev/ttyAMA0",
        baud: 115_200,
    };
    let held = xiao_api::Error::Busy { held_by: Link::Ble };
    let slow = xiao_api::Error::TooSlow {
        needs: BytesPerSec(34_000),
        link_has: BytesPerSec(11_520),
    };
    let denied = io::Error::from(io::ErrorKind::PermissionDenied);
    let cases = [
        (
            held,
            XiaoError::Busy {
                sensor: Sensor::Imu,
                held_by: Link::Ble,
            },
        ),
        (
            slow,
            XiaoError::TooSlow {
                sensor: Sensor::Imu,
                needs_bytes_per_s: 34_000,
                link_has_bytes_per_s: 11_520,
            },
        ),
        (
            xiao_api::Error::Timeout { attempts: 3 },
            XiaoError::Timeout {
                link: uart,
                attempts: 3,
            },
        ),
        (
            xiao_api::Error::NotFound,
            XiaoError::NotFound { link: uart },
        ),
        (
            xiao_api::Error::Disconnected,
            XiaoError::Disconnected { link: uart },
        ),
        (
            xiao_api::Error::Io(denied),
            XiaoError::Io {
                link: uart,
                kind: io::ErrorKind::PermissionDenied,
            },
        ),
    ];
    for (error, expected) in cases {
        assert_eq!(fault(uart, Sensor::Imu, error), expected);
    }
}

#[test]
fn every_error_message_names_its_cause() {
    let ble = LinkSpec::Ble {
        name: Some("arm-imu"),
        attempts: 5,
    };
    let uart = LinkSpec::Uart {
        port: "/dev/ttyAMA0",
        baud: 921_600,
    };
    let cases: [(XiaoError, &[&str]); 7] = [
        (
            XiaoError::Busy {
                sensor: Sensor::Mic,
                held_by: Link::Ble,
            },
            &["microphone", "busy", "BLE"],
        ),
        (
            XiaoError::TooSlow {
                sensor: Sensor::Imu,
                needs_bytes_per_s: 34_000,
                link_has_bytes_per_s: 11_520,
            },
            &["IMU", "34000", "11520"],
        ),
        (
            XiaoError::Timeout {
                link: ble,
                attempts: 5,
            },
            &["BLE \"arm-imu\"", "5 attempt", "never have been found"],
        ),
        (
            XiaoError::Timeout {
                link: LinkSpec::Usb,
                attempts: 1,
            },
            &["USB", "1 attempt", "off"],
        ),
        (
            XiaoError::NotFound {
                link: LinkSpec::Usb,
            },
            &["no XIAO found", "USB"],
        ),
        (
            XiaoError::Disconnected { link: uart },
            &["/dev/ttyAMA0", "921600", "unplugged"],
        ),
        (
            XiaoError::Io {
                link: uart,
                kind: io::ErrorKind::PermissionDenied,
            },
            &["/dev/ttyAMA0", "permission denied"],
        ),
    ];
    for (error, words) in cases {
        let text = error.to_string();
        for word in words {
            assert!(text.contains(word), "{word:?} missing from: {text}");
        }
    }
    let usb_timeout = XiaoError::Timeout {
        link: LinkSpec::Usb,
        attempts: 1,
    };
    assert!(!usb_timeout.to_string().contains("BLE"));
}

// ── X06: the per-tick read and the exit ────────────────────────────────────

/// What the adapter fed, as the engine's `Feed` takes it.
struct Recorded {
    /// The newest IMU reading.
    motion: Option<(Option<Accel>, Option<Gyro>)>,
    /// IMU readings fed.
    motions: usize,
    /// Every sample fed, oldest first: `len` of them.
    samples: [i16; 2048],
    len: usize,
    motion_lost: Option<u64>,
    samples_lost: Option<u64>,
    /// `link_lost` calls.
    link_lost: usize,
}

impl Recorded {
    /// Nothing fed yet.
    fn new() -> Self {
        Self {
            motion: None,
            motions: 0,
            samples: [0; 2048],
            len: 0,
            motion_lost: None,
            samples_lost: None,
            link_lost: 0,
        }
    }

    /// The samples fed so far.
    fn audio(&self) -> &[i16] {
        &self.samples[..self.len]
    }

    /// The acceleration of the newest reading.
    fn accel(&self) -> Option<Accel> {
        self.motion.and_then(|(accel, _)| accel)
    }

    /// The angular rate of the newest reading.
    fn gyro(&self) -> Option<Gyro> {
        self.motion.and_then(|(_, gyro)| gyro)
    }
}

impl Sink for Recorded {
    fn motion(&mut self, accel: Option<Accel>, gyro: Option<Gyro>) {
        self.motion = Some((accel, gyro));
        self.motions += 1;
    }

    fn samples(&mut self, samples: &[i16]) {
        let end = self.len + samples.len();
        self.samples[self.len..end].copy_from_slice(samples);
        self.len = end;
    }

    fn motion_lost(&mut self, total: u64) {
        self.motion_lost = Some(total);
    }

    fn samples_lost(&mut self, total: u64) {
        self.samples_lost = Some(total);
    }

    fn link_lost(&mut self) {
        self.link_lost += 1;
    }
}

/// Reads `xiao` into `seen` until `done` holds, for up to a second: the
/// fake board's messages reach the streams on `xiao-api`'s reader thread.
fn read_until(xiao: &mut Xiao, seen: &mut Recorded, done: impl Fn(&Recorded) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(1);
    while !done(seen) {
        assert!(Instant::now() < deadline, "not fed within 1 s");
        xiao.drain(seen);
        thread::sleep(Duration::from_millis(1));
    }
}

/// An IMU reading of raw counts.
fn imu(counter: u32, accel: Option<[i16; 3]>, gyro: Option<[i16; 3]>) -> Message {
    Message::Imu(ImuReading {
        counter,
        accel,
        gyro,
    })
}

/// An audio block of 96 samples counting up from `first`.
fn block(counter: u32, first: i16) -> Message {
    let pcm = (0..96_i16)
        .flat_map(|i| (first + i).to_le_bytes())
        .collect();
    Message::Audio(AudioBlock { counter, pcm })
}

/// Whether `value` is within `tolerance` of `expected`.
fn near(value: f32, expected: f32, tolerance: f32) -> bool {
    (value - expected).abs() < tolerance
}

/// Counts the events the XIAO adapter logs.
struct Count(&'static AtomicUsize);

impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for Count {
    fn on_event(&self, event: &tracing::Event<'_>, _: tracing_subscriber::layer::Context<'_, S>) {
        if event.metadata().target().starts_with("kmr_adapters::xiao") {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[test]
fn one_g_reads_9_80665() {
    let (mut xiao, board) = on_usb(Xiao::usb().imu());
    xiao.open().unwrap();
    // The count of 1 g at ±4 g, from the protocol's own scale factor.
    let one_g = (1.0 / AccelRange::G4.g_per_count()).round() as i16;
    board.send(imu(0, Some([0, 0, one_g]), Some([0, 0, 0])));

    let mut seen = Recorded::new();
    read_until(&mut xiao, &mut seen, |seen| seen.motion.is_some());
    let accel = seen.accel().unwrap();
    assert!(near(accel.z(), 9.806_65, 1e-3), "{accel:?}");
    assert!(near(accel.x(), 0.0, 1e-6) && near(accel.y(), 0.0, 1e-6));
    assert_eq!(seen.gyro(), Some(Gyro([0.0; 3])));
}

#[test]
fn one_hundred_dps_reads_1_745_rad_s() {
    let (mut xiao, board) = on_usb(Xiao::usb().gyro());
    xiao.open().unwrap();
    let hundred = (100.0 / GyroRange::Dps500.dps_per_count()).round() as i16;
    board.send(imu(0, None, Some([hundred, 0, -hundred])));

    let mut seen = Recorded::new();
    read_until(&mut xiao, &mut seen, |seen| seen.motion.is_some());
    let gyro = seen.gyro().unwrap();
    assert!(near(gyro.x(), 1.745, 1e-3), "{gyro:?}");
    assert!(near(gyro.z(), -1.745, 1e-3), "{gyro:?}");
}

#[test]
fn the_off_half_reads_none() {
    let (mut xiao, board) = on_usb(Xiao::usb().gyro());
    xiao.open().unwrap();
    board.send(imu(0, None, Some([1000, 0, 0])));

    let mut seen = Recorded::new();
    read_until(&mut xiao, &mut seen, |seen| seen.motion.is_some());
    assert_eq!(seen.accel(), None);
    assert!(seen.gyro().is_some());
    assert_eq!(
        board.commands(),
        [GYRO_ON],
        "the accelerometer stays off (FW-6)"
    );
}

#[test]
fn at_two_rates_each_half_keeps_its_last_reading() {
    let spec = ImuSpec {
        accel: Some((Rate::Hz52, AccelRange::G2)),
        gyro: Some((Rate::Hz416, GyroRange::Dps250)),
    };
    let (mut xiao, board) = on_usb(Xiao::usb().imu_with(spec));
    xiao.open().unwrap();
    board.send(imu(0, Some([0, 0, 16_384]), Some([0, 0, 0])));
    let mut seen = Recorded::new();
    read_until(&mut xiao, &mut seen, |seen| seen.accel().is_some());

    // The gyroscope alone this time: the accelerometer reads its last value.
    board.send(imu(1, None, Some([1000, 0, 0])));
    read_until(&mut xiao, &mut seen, |seen| {
        seen.gyro().is_some_and(|gyro| gyro.x() > 0.0)
    });
    assert!(near(seen.accel().unwrap().z(), 9.8, 0.01));
}

#[test]
fn no_new_reading_feeds_nothing_and_returns_at_once() {
    let (mut xiao, _board) = on_usb(Xiao::usb().imu().mic());
    xiao.open().unwrap();

    let mut seen = Recorded::new();
    let start = Instant::now();
    for _ in 0..100 {
        xiao.drain(&mut seen);
    }
    let took = start.elapsed();
    assert!(took < Duration::from_millis(10), "100 reads took {took:?}");
    assert_eq!((seen.motions, seen.len, seen.link_lost), (0, 0, 0));
}

#[test]
fn queued_audio_blocks_land_in_one_window_in_order() {
    let (usb, board) = Fake::usb();
    board.auto_answer();
    // A gyroscope of the test's own on the link: once its reading is in,
    // every block the board sent before it is queued too.
    let probe = xiao_api::Gyro::on(&usb).unwrap();
    let mut xiao = Xiao::usb().mic().on_link(Opened::Usb(usb));
    xiao.open().unwrap();

    // A 20 ms hiccup: four blocks wait for the next tick.
    for counter in 0..4 {
        board.send(block(counter, (counter * 96) as i16));
    }
    board.send(imu(0, None, Some([0, 0, 0])));
    let deadline = Instant::now() + Duration::from_secs(1);
    while probe.try_latest().unwrap().is_none() {
        assert!(Instant::now() < deadline, "the probe's reading never came");
        thread::sleep(Duration::from_millis(1));
    }

    let mut seen = Recorded::new();
    xiao.drain(&mut seen);
    let in_order: [i16; 384] = core::array::from_fn(|i| i as i16);
    assert_eq!(
        seen.audio(),
        in_order,
        "all four, in one read, oldest first"
    );
    assert_eq!(seen.samples_lost, Some(0));
}

#[test]
fn the_boards_drop_count_reaches_the_feed() {
    let (mut xiao, board) = on_usb(Xiao::usb().gyro().mic());
    xiao.open().unwrap();
    board.send(imu(0, None, Some([0, 0, 0])));
    board.send(imu(2, None, Some([0, 0, 0])));
    for counter in [0, 1, 3] {
        board.send(block(counter, 0));
    }

    let mut seen = Recorded::new();
    read_until(&mut xiao, &mut seen, |seen| {
        seen.len == 3 * 96 && seen.motion_lost == Some(1)
    });
    assert_eq!(seen.samples_lost, Some(96), "block 2: 96 samples");
}

#[test]
fn an_unplugged_board_reports_link_down_every_tick_and_logs_once() {
    static LOGGED: AtomicUsize = AtomicUsize::new(0);
    let (mut xiao, board) = on_usb(Xiao::usb().imu().mic());
    xiao.open().unwrap();
    let before = board.commands();
    board.unplug();

    let mut seen = Recorded::new();
    let logger = tracing_subscriber::registry().with(Count(&LOGGED));
    let start = Instant::now();
    tracing::subscriber::with_default(logger, || {
        for _ in 0..5 {
            xiao.drain(&mut seen);
        }
    });
    let took = start.elapsed();

    assert!(took < Duration::from_millis(10), "5 reads took {took:?}");
    assert_eq!(seen.link_lost, 5, "every read from the loss on");
    assert_eq!(LOGGED.load(Ordering::Relaxed), 1, "logged once");
    assert_eq!(board.commands(), before, "nothing sent: no reconnect");
}

#[test]
fn close_turns_the_sensors_off() {
    let (mut xiao, board) = on_usb(Xiao::usb().imu().mic());
    xiao.open().unwrap();
    xiao.close();

    let both = Command::ImuOn(ImuConfig {
        accel: Some(AccelConfig {
            rate: Rate::Hz104,
            range: AccelRange::G4,
        }),
        gyro: Some(GyroConfig {
            rate: Rate::Hz104,
            range: GyroRange::Dps500,
        }),
    });
    assert_eq!(
        board.commands(),
        [both, Command::MicOn, Command::MicOff, Command::ImuOff]
    );
    assert!(xiao.opened.is_none() && xiao.halves.is_none() && xiao.microphone.is_none());
}

#[test]
fn close_after_unplug_returns() {
    let (mut xiao, board) = on_usb(Xiao::usb().imu().mic());
    xiao.open().unwrap();
    board.unplug();
    xiao.drain(&mut Recorded::new());

    let start = Instant::now();
    xiao.close();
    let took = start.elapsed();
    assert!(took < Duration::from_millis(100), "close took {took:?}");
}
