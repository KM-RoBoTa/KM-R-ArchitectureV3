//! The XIAO adapter on `xiao-api`'s fake link: the test plays the board.

use std::io;

use xiao_api::BytesPerSec;
use xiao_api::link::{Board, Fake};
use xiao_api::proto::message::{
    AUDIO_BLOCK_SAMPLES, AccelConfig, BLE_NAME as PROTO_BLE_NAME, Command, GyroConfig, ImuConfig,
    MIC_SAMPLE_RATE_HZ, Refusal, UART_DEFAULT_BAUD,
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

#[test]
#[ignore = "X06"]
fn one_g_reads_9_80665() {}

#[test]
#[ignore = "X06"]
fn one_hundred_dps_reads_1_745_rad_s() {}

#[test]
#[ignore = "X06"]
fn the_off_half_reads_none() {}

#[test]
#[ignore = "X06"]
fn no_new_reading_feeds_nothing_and_returns_at_once() {}

#[test]
#[ignore = "X06"]
fn queued_audio_blocks_land_in_one_window_in_order() {}

#[test]
#[ignore = "X06"]
fn the_boards_drop_count_reaches_the_feed() {}

#[test]
#[ignore = "X06"]
fn an_unplugged_board_reports_link_down_every_tick_and_logs_once() {}

#[test]
#[ignore = "X06"]
fn close_turns_the_sensors_off() {}

#[test]
#[ignore = "X06"]
fn close_after_unplug_returns() {}
