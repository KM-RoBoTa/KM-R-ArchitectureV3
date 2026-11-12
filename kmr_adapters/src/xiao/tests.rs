//! The XIAO adapter on `xiao-api`'s fake link: the test plays the board.

use xiao_api::proto::message::{BLE_NAME as PROTO_BLE_NAME, UART_DEFAULT_BAUD};

use super::*;

#[test]
fn the_defaults_are_the_firmwares() {
    assert_eq!(DEFAULT_BAUD, UART_DEFAULT_BAUD);
    assert_eq!(BLE_NAME, PROTO_BLE_NAME);
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
#[ignore = "X05"]
fn needs_count_what_was_asked() {}

#[test]
#[ignore = "X05"]
fn max_silence_is_ten_periods_of_the_slowest_stream() {}

#[test]
#[ignore = "X05"]
fn building_a_xiao_opens_nothing() {}

#[test]
#[ignore = "X05"]
fn open_turns_on_only_what_was_asked() {}

#[test]
#[ignore = "X05"]
fn a_busy_sensor_names_the_sensor_and_the_holding_link() {}

#[test]
#[ignore = "X05"]
fn a_slow_link_gives_both_speeds() {}

#[test]
#[ignore = "X05"]
fn no_board_names_the_link() {}

#[test]
#[ignore = "X05"]
fn every_error_message_names_its_cause() {}

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
