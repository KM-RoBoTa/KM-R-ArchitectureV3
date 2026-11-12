# XIAO Sensor Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

Written 2026-11-12. Nothing is built yet.

**Goal:** `robot.add_sensor(xiao::Xiao::usb().imu().mic())` feeds the XIAO nRF52840 Sense's IMU and microphone into `&Sensors`, tested without a board.

**Architecture:** Phase A makes `xiao-api` fit a real-time loop (three contract changes in its own repo). Phase B is a thin `SensorHal` member in `kmr_adapters` that opens the link in Init, drains readings without waiting each tick, and converts units. Phase C exposes it in `kmr_api`. The firmware is not finished, so every automatic test runs on `xiao-api`'s test link; the board session is the last, manual task.

**Tech Stack:** `xiao-api` (path dependency on `../xiao-nrf52840-sensors/crates/api`), `kmr_core::hal::SensorHal`.

**Spec:** `docs/plans/2026-11-12-sensors-hld.md`; `../xiao-nrf52840-sensors/docs/srs.md` API-1 to API-4, FW-1, FW-2, FW-4, FW-6; SRS FR-5.5, FR-5.7.

**Depends on:** `docs/plans/sensors.md` S04 (the contract) for Phase B, S06 for Phase C. Phase A depends on nothing and can start now.

## Global Constraints

- Phase A happens in `../xiao-nrf52840-sensors` under that repo's `CLAUDE.md` and cycle: `api` re-enters at step 4, one branch `mod/api-realtime`. These are contract changes the owner approved on 2026-11-12.
- `xiao-api` is the SDK boundary: heap and threads live inside it. The adapter's own code names no `Box`, `Vec`, `String`, `Arc`, spawns no thread and holds no lock of its own.
- `Xiao::read` never waits: only `try_latest` and `try_next`.
- Nothing opens before Init (FR-5.5): building a `Xiao` touches no port.
- Feature `xiao` in `kmr_adapters` and `kmr_api`; without it nothing of `xiao-api` is compiled. No other new dependency.
- Units out of the adapter: m/s² (`g × 9.80665`) and rad/s (`°/s × π/180`).
- No second fake board in kmr: tests use `xiao_api::link::Fake` only.
- Branch `feature/xiao-adapter` cut from `dev` after `feature/sensors` is merged. Commits: task id first, no AI trailer.

## Review Focus

- The board is unplugged mid-run: `read` returns at once, calls `feed.link_down()` every tick from then on, logs once, never reconnects, and `close` still returns (X06).
- The mic is asked on a link that cannot carry it (UART at 115200): `open` fails with `XiaoError::TooSlow`, both speeds in it, before torque on (X05).
- Another program already holds the IMU on another link: `XiaoError::Busy` names the sensor and that link (X05).
- Only the gyroscope is asked: `imu.accel()` reads `None`, `imu.gyro()` reads values, and the accelerometer stays off on the board (FW-6) (X06).
- Several audio blocks queue between two ticks (5 ms tick: 0 or 1; a 20 ms hiccup: 4): all are drained in order into one window, and what `xiao-api` counted as dropped shows in `mic.dropped()` (X06).

---

## Phase A: `xiao-api` for a real-time caller (in `../xiao-nrf52840-sensors`)

### Task X01: a read that never waits

**Files:** Modify: `crates/api/src/stream.rs`, `crates/api/src/imu.rs`, `crates/api/src/mic.rs`.

**Interfaces:** Produces `pub fn try_latest(&self) -> Result<Option<T>, Error>` on `Imu` (`Motion`), `Gyro` (`Rotation`), `Accel` (`Acceleration`): the newest queued reading, `Ok(None)` when none is queued, the stream's one `Err` when the link is gone. `pub fn try_next(&self) -> Result<Option<AudioBlock>, Error>` on `Mic`: the oldest queued block. Neither waits; `latest()` and the iterators are unchanged.

- [ ] **Step 1: Write the tests** `try_latest_is_none_when_nothing_is_queued` (returns in under 10 ms on an idle link), `try_latest_returns_the_newest_and_counts_the_skipped_as_read_not_dropped`, `try_next_hands_blocks_out_in_order`, `try_latest_reports_a_dead_link_once_then_none`.
- [ ] **Step 2: Run** `cargo test -p xiao-api try_`. Expected: FAIL.
- [ ] **Step 3: Implement** on `Stream::take(.., false)`, which already exists.
- [ ] **Step 4: Run.** Expected: PASS.
- [ ] **Step 5: Commit** `api: non-blocking try_latest and try_next (API-3)`.

### Task X02: sensors own their link

**Files:** Modify: `crates/api/src/link.rs`, `link/usb.rs`, `link/uart.rs`, `link/ble.rs`, `imu.rs`, `mic.rs`, the four examples.

**Interfaces:** Produces `Imu`, `Gyro`, `Accel`, `Mic` without the `'l` lifetime; `on(link: &impl AsLink)` keeps its signature and clones a shared handle to the connection. The link closes when the link value and every sensor on it are dropped. Closes the first `[question]` in that repo's `docs/deferred.md`.

- [ ] **Step 1: Write the tests** `a_sensor_outlives_its_link_value` (drop the `Usb`, the `Gyro` still reads), `a_sensor_and_its_link_fit_in_one_struct` (compiles), `the_link_closes_when_the_last_holder_drops`.
- [ ] **Step 2: Run** `cargo test -p xiao-api`. Expected: the new tests FAIL.
- [ ] **Step 3: Implement.** The busy rule (FW-1) and the shared IMU halves (FW-2) keep their existing tests green, unedited.
- [ ] **Step 4: Run** `cargo test -p xiao-api` and `cargo build -p xiao-api --examples`. Expected: PASS.
- [ ] **Step 5: Commit** `api: sensors hold a shared link handle, no lifetime`.

### Task X03: a public test link

**Files:** Modify: `crates/api/Cargo.toml` (feature `fake`), `crates/api/src/link.rs`. Create: `crates/api/src/link/fake.rs`.

**Interfaces:** Produces, behind `#[cfg(feature = "fake")]`: `pub struct Fake;` with `Fake::usb() -> (Usb, Board)`, `Fake::uart(baud: Baud) -> (Uart, Board)`; `pub struct Board` with `auto_answer(&self)` (replies `On`/`Off` to every command), `refuse_next(&self, refusal: Refusal)`, `send(&self, message: Message)`, `commands(&self) -> Vec<Command>`, `unplug(&self)`. It wraps the existing `Memory` transport; the crate's own tests move onto it.

- [ ] **Step 1: Write the tests** `a_fake_board_answers_on` (`Gyro::on(&usb)` succeeds with `auto_answer`), `a_fake_board_can_refuse_busy`, `an_unplugged_board_ends_the_stream`.
- [ ] **Step 2: Run** `cargo test -p xiao-api --features fake fake`. Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `cargo test -p xiao-api --features fake`. Expected: PASS.
- [ ] **Step 5: Commit** `api: public Fake link for downstream tests`. Step end in that repo: `api` row, plan ticks, `docs/deferred.md` question set `done`.

## Phase B: the adapter (in `kmr_v3`)

### Task X04: dependency, feature, contract

**Files:** Modify: `Cargo.toml` (workspace dependency `xiao-api = { path = "../xiao-nrf52840-sensors/crates/api" }`), `kmr_adapters/Cargo.toml` (`xiao = ["dep:xiao-api"]`, dev-dependency with `fake`), `kmr_adapters/src/lib.rs` (`pub mod xiao;`). Create: `kmr_adapters/src/xiao.rs`, `kmr_adapters/src/xiao/tests.rs`.

**Interfaces:** Produces the stubs, `pub`:
`enum LinkSpec { Usb, Uart { port: &'static str, baud: u32 }, Ble { name: Option<&'static str>, attempts: u8 } }`;
`struct ImuSpec { pub accel: Option<(Rate, AccelRange)>, pub gyro: Option<(Rate, GyroRange)> }`;
`struct Xiao { .. }` with `Xiao::usb()`, `Xiao::uart(port: &'static str)`, `Xiao::ble()`, `.baud(u32)`, `.named(&'static str)`, `.attempts(u8)`, `.imu()` (both halves, 104 Hz, 4 g, 500 °/s), `.imu_with(ImuSpec)`, `.gyro()`, `.accel()`, `.mic()`;
`impl seal::Adapter for Xiao {}`, `impl SensorHal for Xiao`. `Rate`, `AccelRange`, `GyroRange` are re-exported from `xiao-api`.

- [x] **Step 1: Run** `/contract xiao-adapter` to write the stubs and the test names of X05 and X06.
- [x] **Step 2: Run** `cargo build -p kmr_adapters`, then with `--features xiao`, and `cargo tree -p kmr_adapters | grep -c xiao` without the feature. Expected: both build; the count is 0.
- [x] **Step 3: Commit** `feat(adapters): X04 xiao adapter contract and feature`.

### Task X05: needs and bring-up

**Files:** Modify: `kmr_adapters/src/xiao.rs`, `xiao/tests.rs`.

**Interfaces:** Consumes `SensorHal`, `Needs` (sensors plan S04), `xiao_api::link::Fake`. Produces `needs()` and `open()`. A `pub(crate) fn Xiao::on_link(self, link: Opened) -> Self` lets tests hand in a `Fake` link instead of opening a real one. `type Error = XiaoError`, `pub`, no ambiguous variant: `Busy { sensor: Sensor, held_by: Link }`, `TooSlow { sensor: Sensor, needs_bytes_per_s: u32, link_has_bytes_per_s: u32 }`, `Timeout { link: LinkSpec, attempts: u8 }`, `NotFound { link: LinkSpec }`, `Disconnected { link: LinkSpec }`, `Io { link: LinkSpec, kind: std::io::ErrorKind }`. Each `Display` says why and what causes it. If `xiao_api::Error` lacks a detail a variant needs, that is one more Phase A change, not a vaguer variant.

- [x] **Step 1: Write the tests** `needs_count_what_was_asked` (`.imu().mic()` → `imus 1, mics 1, mic_rate_hz 16_000`; `.gyro()` → `imus 1, mics 0`), `max_silence_is_ten_periods_of_the_slowest_stream` (104 Hz → 97 ms, rounded up to the ms), `building_a_xiao_opens_nothing`, `open_turns_on_only_what_was_asked` (`.gyro()` → the board saw one `ImuOn` with `accel: None`), `a_busy_sensor_names_the_sensor_and_the_holding_link` (`Busy { sensor: Mic, held_by: Ble }`), `a_slow_link_gives_both_speeds` (`TooSlow { needs_bytes_per_s: 34_000, link_has_bytes_per_s: 11_520, .. }`), `no_board_names_the_link`, `every_error_message_names_its_cause` (each variant's `Display` contains the sensor or the link).
- [x] **Step 2: Run** `cargo test -p kmr_adapters --features xiao xiao`. Expected: FAIL.
- [x] **Step 3: Implement.** The opened link and sensors sit in `Option` fields; a partial `open` (IMU on, mic refused) drops what it turned on before returning the error.
- [x] **Step 4: Run.** Expected: PASS.
- [x] **Step 5: Commit** `feat(adapters): X05 xiao bring-up and fault mapping`.

### Task X06: the per-tick read and the exit

**Files:** Modify: `kmr_adapters/src/xiao.rs`, `xiao/tests.rs`.

**Interfaces:** Consumes `Feed` (S03), `try_latest`, `try_next` (X01). Produces `read()` and `close()`.

- [x] **Step 1: Write the tests** `one_g_reads_9_80665` (`Message::Imu` with the raw count of 1 g at 4 g range → `accel().z()` within 1e-3), `one_hundred_dps_reads_1_745_rad_s`, `the_off_half_reads_none`, `no_new_reading_feeds_nothing_and_returns_at_once` (100 idle `read` calls under 10 ms in total), `queued_audio_blocks_land_in_one_window_in_order` (4 blocks of 96 → 384 samples), `the_boards_drop_count_reaches_the_feed`, `an_unplugged_board_reports_link_down_every_tick_and_logs_once`, `close_turns_the_sensors_off` (the board saw `ImuOff`, `MicOff`), `close_after_unplug_returns`.
- [x] **Step 2: Run** `cargo test -p kmr_adapters --features xiao xiao`. Expected: FAIL.
- [x] **Step 3: Implement.** Audio bytes are little-endian `i16`; copy through a stack buffer of one block, no allocation.
- [x] **Step 4: Run** `cargo test --workspace --no-fail-fast` and `cargo clippy --workspace --all-targets --features kmr_api/xiao -- -D warnings`. Expected: PASS, zero warnings.
- [x] **Step 5: Commit** `feat(adapters): X06 xiao per-tick read, units and exit`.

## Phase C: the user's side

### Task X07: `kmr_api::xiao` and an example

**Files:** Modify: `kmr_api/Cargo.toml` (`xiao = ["kmr_adapters/xiao"]`), `kmr_api/src/lib.rs`. Create: `kmr_api/src/xiao.rs`, `kmr_api/examples/xiao_imu.rs` (`required-features = ["xiao"]`), `kmr_api/tests/xiao.rs`.

**Interfaces:** Consumes `Robot::add_sensor` (S06). Produces `pub mod xiao` re-exporting `Xiao`, `ImuSpec`, `Rate`, `AccelRange`, `GyroRange`.

- [x] **Step 1: Write the test** `a_controller_reads_a_fake_xiao_through_the_engine` in `kmr_api/tests/xiao.rs`: a robot on `NoHardware` with a fake board streaming 1 g on z; a controller stores `sensors.imu(0)` and raises `shutdown!` on tick 3; the stored `accel().z()` is 9.80665 within 1e-3 and the board ends with `ImuOff`.
- [x] **Step 2: Run** `cargo test -p kmr_api --features xiao,test-util xiao`. Expected: FAIL, then PASS after the re-exports. If `on_link` cannot be reached from `kmr_api`'s tests, that visibility is a deferral: keep this test in `kmr_adapters/src/xiao/tests.rs` on `kmr_core::Robot::run_all` instead. Taken on 2026-11-12, on the core's `test_util::run_scripted_all`, with a microphone twin `a_controller_hears_a_clap_on_a_fake_xiao_through_the_engine`; the visibility is in `docs/deferred.md`.
- [x] **Step 3: Write the example**: `.add_sensor(xiao::Xiao::usb().imu())`, one controller that logs the gyro every 100 ticks.
- [x] **Step 4: Run** `cargo build -p kmr_api --example xiao_imu --features xiao`. Expected: green.
- [x] **Step 5: Commit** `feat(api): X07 kmr_api::xiao and its example`.

### Task X08: on the board (HUMAN)

Blocked on the firmware (`../xiao-nrf52840-sensors/docs/plans/firmware.md` fw-03 to fw-06).

- [ ] Flash with `scripts/flash.sh`, run `cargo run -p kmr_api --example xiao_imu --features xiao` over USB, then over UART at 921600: the gyro follows the board, `sensor_missing` stays clear.
- [ ] Unplug mid-run: the loop keeps its period, `sensor_missing` is raised, Ctrl-C still exits cleanly.
- [ ] Run the NFR-1 tick bench with the IMU at 416 Hz and with the mic on; note the per-tick cost of `Xiao::read` in the PR. BLE in the loop is measured here, not promised before.

### Task X09: the clap controller (maintainer's request, 2026-11-12)

**Files:** Modify: `controllers/src/main.rs`, `controllers/Cargo.toml` (`hardware` also turns on `kmr_api/xiao`). `controllers` is owned `mine`: the maintainer asked for this edit; list both files first in the summary.

**Behaviour:** the robot homes, then holds the centre. When the microphone's peak this tick (`mic.samples()`, largest `unsigned_abs()`) is above 1000, both snakes run their sines for 2 s, with the sine's time starting at the trigger, then hold the centre again. The run shuts down after 60 s whatever happened; errors still end it at once.

- [ ] `.add_sensor(kmr_api::xiao::Xiao::usb().mic())`; `PEAK: u16 = 1000`, `MOVE_FOR = 2 s`, `TIME_LIMIT = 60 s`.
- [ ] One controller with its own context (when the motion started), replacing `sine_ax` and `sine_rx`, behind `HOME.then` if `then` takes a context controller, else after an elapsed-time guard of `HOME_TIME`.
- [ ] `// DECISION:` a peak heard while moving is ignored (the servos are loud), so one clap is one 2 s motion.
- [ ] `cargo build -p controllers` and `cargo build -p controllers --features hardware` green; `cargo clippy -p controllers -- -D warnings`.
