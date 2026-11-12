# Sensors Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

Written 2026-11-12. Nothing is built yet.

**Goal:** Controllers read an IMU and a microphone through `&Sensors`, filled each tick by sensor adapters behind a sealed contract.

**Architecture:** `kmr_core::Sensors` is a closed, fixed-size block of typed slots. Sensor adapters implement `SensorHal` and sit in their own HList (`Sensing`), next to the drivers; the runtime opens them in Init, reads them before `First`, closes them after torque off. `kmr_api` wraps the handle and grows the list with `add_sensor`.

**Tech Stack:** `kmr_core` (`no_std`, `frunk_core` HList), `kmr_api`. No new dependency.

**Spec:** `docs/plans/2026-11-12-sensors-hld.md`; SRS FR-5.7, FR-5.2, FR-5.5, FR-13.4, NFR-1.

## Global Constraints

- No heap type and no `unsafe` in `kmr_core`; it builds with `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf`.
- Everything is `pub(crate)` unless this plan names it `pub`. The pub items below are frozen at step 4 (`/contract sensors`); a pub item this plan does not name is a deferral.
- The controller signature `fn(&Time, &mut State, &Sensors)` does not change.
- `SensorHal::read` never waits; nothing in the tick takes a lock.
- No `mod.rs`: `sensors.rs` with `sensors/feed.rs`; `hal/sensor.rs`, `hal/sensing.rs`.
- Defaults, named once in `sensors.rs`: `MAX_IMUS = 4`, `MAX_MICS = 1`, `MIC_WINDOW = 1024`. No other file writes these numbers; how a user sets them is parked in `docs/deferred.md`.
- Branch `feature/sensors` cut from `dev`. Commits: Conventional Commits, task id first (`feat(core): S02 typed sensor slots`), no AI trailer.
- Oracle after every task: `cargo test --workspace --no-fail-fast`, `cargo clippy --workspace --all-targets -- -D warnings`, pub diff against the step-4 contract.

## Review Focus

- A sensor slower than the tick (104 Hz on a 1 kHz loop) is not flagged missing; its `age()` grows from 0 to 9 and returns to 0 (S05).
- A burst larger than `MIC_WINDOW` in one tick: the window keeps the first 1024 samples, `dropped()` grows by the rest, nothing panics (S03).
- A sensor that fails `open` ends the run before `TorqueOn`, and the actuators are still torqued off and closed (S05).
- A panic in a controller still calls `close` on every sensor, after `torque_off` and `close` of the drivers (S05).
- A run without any sensor behaves exactly as today: `imu(0)` is `None`, `SENSOR_MISSING` is never raised, the existing 865 tests pass untouched (S05).

---

### Task S01: SRS text and contract (step 4)

**Files:** Modify: `docs/srs/SRS.md` (maintainer), `kmr_core/src/sensors.rs`, `kmr_core/src/hal.rs`, `kmr_core/src/lib.rs`, `kmr_api/src/sensors.rs`. Create: `kmr_core/src/sensors/feed.rs`, `kmr_core/src/hal/sensor.rs`, `kmr_core/src/hal/sensing.rs`.

- [x] **Step 1:** the four points are settled (HLD, "Settled"); FR-5.7 is written in `docs/srs/SRS.md` and "Sensors handle surface" is ticked (2026-11-12).
- [x] **Step 2: Run** `/contract sensors`: every pub item of S02 to S06 as a stub with `todo!()`, every test below by name.
- [x] **Step 3: Run** `cargo build --workspace` and `cargo test --workspace --no-run`. Expected: both green.
- [x] **Step 4: Commit** `feat(core): S01 sensors contract stubs`.

### Task S02: value types, slots, reader views

**Files:** Modify: `kmr_core/src/sensors.rs`, `kmr_core/src/lib.rs` (re-exports).

**Interfaces:** Produces, all `pub`:
`struct Accel(pub [f32; 3])` and `struct Gyro(pub [f32; 3])`, both `Clone, Copy, Debug, PartialEq, Default`, each with `x()`, `y()`, `z() -> f32`;
`const MAX_IMUS: usize = 4; const MAX_MICS: usize = 1; const MIC_WINDOW: usize = 1024;`
`Sensors::imu(&self, index: usize) -> Option<Imu<'_>>`, `Sensors::mic(&self, index: usize) -> Option<Mic<'_>>`;
`Imu::{accel() -> Option<Accel>, gyro() -> Option<Gyro>, age() -> u64, dropped() -> u64}`;
`Mic::{samples() -> &[i16], rate_hz() -> u32, dropped() -> u64}`.
`Sensors` fields stay private; `pub(crate) fn Sensors::new() -> Self`.

- [x] **Step 1: Write the tests** in `sensors.rs`: `an_undeclared_slot_reads_none` (`Sensors::new().imu(0)` and `.mic(0)` are `None`), `an_index_past_the_slots_reads_none` (`imu(MAX_IMUS)`), `a_declared_slot_without_a_reading_reads_none`, `sensors_fit_in_three_kilobytes` (`size_of::<Sensors>() <= 3072`).
- [x] **Step 2: Run** `cargo test -p kmr_core sensors`. Expected: FAIL on `todo!()`.
- [x] **Step 3: Implement** the slots: an IMU slot holds `accel`, `gyro`, `refreshed: Option<u64>`, `dropped`; a mic slot holds the window, its length, `rate_hz`, `dropped`; `Sensors` also holds `now: u64` and how many slots of each kind are declared.
- [x] **Step 4: Run** the same command. Expected: PASS.
- [x] **Step 5: Commit** `feat(core): S02 typed sensor slots and reader views`.

### Task S03: the writer view

**Files:** Modify: `kmr_core/src/sensors/feed.rs`, `kmr_core/src/sensors.rs`.

**Interfaces:** Consumes S02. Produces `pub struct Feed<'a>` with
`pub fn imu(&mut self, local: usize, accel: Option<Accel>, gyro: Option<Gyro>)`,
`pub fn audio(&mut self, local: usize, samples: &[i16])`,
`pub fn imu_dropped(&mut self, local: usize, total: u64)`, `pub fn audio_dropped(&mut self, local: usize, total: u64)`,
`pub fn link_down(&mut self)`.
`local` counts within the member's own slots; past them the call is ignored (as `Telemetry`'s setters do). Engine side, `pub(crate)`: `Sensors::begin_tick(&mut self, tick: u64)` empties every mic window and sets `now`; `Sensors::feed(&mut self, range: Slots) -> Feed<'_>`; `Sensors::took_link_down(&mut self) -> bool`.

- [x] **Step 1: Write the tests** in `feed.rs`: `a_fed_imu_reads_back_with_age_zero`, `age_counts_ticks_since_the_reading` (feed at tick 5, `begin_tick(8)`, `age() == 3`), `a_half_fed_imu_keeps_the_other_half_none`, `audio_appends_within_a_tick` (two calls of 96 samples → `samples().len() == 192`, order kept), `audio_is_emptied_at_the_next_tick`, `audio_past_the_window_is_counted_dropped` (feed 1100 → len 1024, `dropped() == 76`), `a_local_index_past_the_member_is_ignored`, `a_feed_cannot_reach_another_members_slot`.
- [x] **Step 2: Run** `cargo test -p kmr_core feed`. Expected: FAIL.
- [x] **Step 3: Implement.** `audio` overflow adds to `dropped`; `audio_dropped` and `imu_dropped` set the adapter's own running total, kept apart and summed in the getter.
- [x] **Step 4: Run.** Expected: PASS.
- [x] **Step 5: Commit** `feat(core): S03 sensor feed, stamping and the audio window`.

### Task S04: the sensor contract and its list

**Files:** Modify: `kmr_core/src/hal/sensor.rs`, `kmr_core/src/hal/sensing.rs`, `kmr_core/src/hal.rs` (`pub use`), `kmr_core/src/test_util.rs`.

**Interfaces:** Consumes `Feed`. Produces, `pub`:
`struct Needs { pub imus: u8, pub mics: u8, pub mic_rate_hz: u32, pub max_silence: Duration }`;
`trait SensorHal: seal::Adapter { type Error: core::error::Error + 'static; fn needs(&self) -> Needs; fn open(&mut self) -> Result<(), Self::Error>; fn read(&mut self, feed: Feed<'_>); fn close(&mut self); }`, also on `&mut A`;
`trait Sensing: seal::Sensing` with `type Error`, on `HNil` (`Error = Infallible`) and `HCons<H: SensorHal, T: Sensing>` (`Error = MemberError<H::Error, T::Error>`, the type `Drivers` already nests with), hidden methods `claim`, `open`, `read`, `close`;
`enum SensorPolicy { Flag, Warn, Abort }` (`Default = Flag`).
No ambiguous error: an adapter's `Error` says why and what causes it; the core adds the member index.
`pub(crate) struct SensorPlan` holds each member's slot range and its `max_silence` in ticks. Test fake in `test_util.rs`: `FakeSensor` with a scripted list of readings and a call log.

- [x] **Step 1: Write the tests** in `sensing.rs`: `slots_follow_declaration_order` (two members of one IMU each → locals map to `imu(0)` and `imu(1)`), `too_many_imus_is_refused_before_any_open` (five → error, no `open` in the log), `two_mics_are_refused`, `open_stops_at_the_first_error_and_names_the_member` (the nested error is the fake's own typed error, position kept), `close_runs_on_every_member_tail_first`, `an_empty_list_claims_nothing`.
- [x] **Step 2: Run** `cargo test -p kmr_core sensing`. Expected: FAIL.
- [x] **Step 3: Implement.** `max_silence` becomes ticks with `ceil(max_silence / dt)`, at least 1.
- [x] **Step 4: Run.** Expected: PASS.
- [x] **Step 5: Commit** `feat(core): S04 sealed sensor contract and its list`.

### Task S05: the runtime reads sensors

**Files:** Modify: `kmr_core/src/runtime.rs` (`run_staged`, `run_phases`, `control_loop`, `Powered`), `kmr_core/src/robot.rs`, `kmr_core/src/error.rs`. Test: `kmr_core/src/runtime.rs` tests module.

**Interfaces:** Consumes S03, S04. Produces `pub fn Robot::run_all<D: Drivers<N>, SN: Sensing, S: Stages<N>>(self, hal: D, sensors: SN, stages: S)` returning `Result<(), RunError<HardwareError<D::Error>, SN::Error>>`; `run_on` and `run_on_staged` delegate to it with `HNil`. `RunError<E, S = Infallible>` gains `Sensor(S)`; the config error gains `TooManySensors { kind, asked, slots }`. `pub fn Robot::set_sensor_policy(self, policy: SensorPolicy) -> Self`.

- [x] **Step 1: Write the tests** on the scripted host: `sensors_open_in_init_after_the_actuators` (call log order `Open, Zero, sensor open, Init controller`), `a_sensor_error_at_open_ends_the_run_before_torque_on` (and the log ends with `torque_off, close`), `a_reading_is_visible_in_first_of_the_same_tick`, `post_init_controllers_see_the_first_reading`, `a_slow_sensor_is_not_missing` (reading every 10 ticks, `max_silence` 20 ticks → flag never set, ages 0..=9), `silence_past_max_silence_raises_sensor_missing`, `link_down_raises_sensor_missing_that_tick`, `sensors_close_after_the_drivers_on_shutdown`, `sensors_close_when_a_controller_panics`, `a_run_without_sensors_never_raises_sensor_missing`, `the_default_policy_only_flags`, `policy_warn_logs_once_per_member`, `policy_abort_is_an_emergency_stop_that_tick` (the run ends with the estop signal, `torque_off` in the log).
- [x] **Step 2: Run** `cargo test -p kmr_core runtime`. Expected: the new tests FAIL, the old ones PASS.
- [x] **Step 3: Implement.** Claim the sensors next to `hal.claim`, before `pre_init`. Per tick, right after `state.refreshed` and before `take_flags`: `sensors.begin_tick(tick)`, `sensing.read(..)`, then raise `SENSOR_MISSING` through `state.raise`. `Powered` takes both lists and its `Drop` runs `torque_off`, `close`, then the sensors' `close`. `// DECISION:` wherever the order was a choice.
- [x] **Step 4: Run** `cargo test --workspace --no-fail-fast`. Expected: PASS, none of the existing tests edited.
- [x] **Step 5: Run** `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf`. Expected: green.
- [x] **Step 6: Commit** `feat(core): S05 the loop opens, reads and closes sensors`.

### Task S06: the API surface

**Files:** Modify: `kmr_api/src/sensors.rs`, `kmr_api/src/robot.rs`, `kmr_api/src/lib.rs`, `kmr_api/src/faults.rs` (doc only). Test: `kmr_api/tests/sensors.rs` (create).

**Interfaces:** Consumes S05. Produces `kmr_api::Sensors::{imu(&self, index) -> Option<Imu<'_>>, mic(&self, index) -> Option<Mic<'_>>}` with the wrappers forwarding the S02 getters; `pub use kmr_core::sensors::{Accel, Gyro}`; `Robot<N, H, C, E, SN = HNil>` and `pub fn add_sensor<S: SensorHal>(self, sensor: S) -> Robot<N, H, C, E, HCons<S, SN>>`; `run()` hands the list to `run_all`; `pub fn set_sensor_policy(self, policy: SensorPolicy) -> Self`, `SensorPolicy` re-exported; the API's run error carries the sensors' typed error unflattened.

- [x] **Step 1: Write the tests** in `kmr_api/tests/sensors.rs` with `FakeSensor` (`test-util` feature): `a_controller_reads_the_imu_through_the_api`, `a_controller_reads_audio_through_the_api`, `faults_report_sensor_missing_when_the_sensor_goes_silent`, `two_sensors_are_indexed_in_add_order`, `a_sensor_error_reaches_the_user_typed` (match on the fake's own error variant), `a_robot_without_sensors_still_builds_with_its_old_type` (`let _: Robot<4, Spec> = ..` compiles).
- [x] **Step 2: Run** `cargo test -p kmr_api --features test-util sensors`. Expected: FAIL.
- [x] **Step 3: Implement.** `add_sensor` may be called before or after `add_controller`; the zenoh and `Bound::run` paths carry the list too.
- [x] **Step 4: Run** `cargo test --workspace --no-fail-fast` and `cargo run -p kmr_api --example complete_api_example`. Expected: PASS, example unchanged.
- [x] **Step 5: Commit** `feat(api): S06 sensors getters and add_sensor`.

### Task S07: docs and cost

**Files:** Modify: `docs/ARCHITECTURE.md` (a "Sensors" section), `AGENTS.md` (project context: Sensors paragraph, heap-free note), `bench/` only if a probe already walks the phases.

- [ ] **Step 1: Write** the two doc sections from the HLD; drop "WIP" from `kmr_core/src/sensors.rs`.
- [ ] **Step 2: Run** the NFR-1 tick bench with no sensor before and after the branch; note both medians in the PR. A regression above the bench's noise is a bug in S05.
- [ ] **Step 3: Commit** `docs: S07 sensors in the architecture notes`.
