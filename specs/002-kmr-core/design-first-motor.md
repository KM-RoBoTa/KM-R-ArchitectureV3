# Design note: kmr_core on the critical path to the first motor

Status: proposal for the maintainer. Nothing in `kmr_core/` is edited. Written 2026-10-28.

Scope: core plan items #2, #3, #8, #9, #10, #11, #14 (critical path steps 3 to 8 of
`specs/spec-coordination-2026-10-28.md` §3). Bench: XmBench, position mode, `JOINTS = 4`.

Spec ids: FR-019 to FR-022 (SRS FR-2.3, FR-2.5, FR-2.6), FR-029 (SRS FR-3.3), FR-030
(SRS FR-3.4), FR-032 to FR-035 (SRS FR-4.1 to FR-4.3), FR-039 to FR-043 (SRS FR-5.1, FR-5.2,
FR-5.6, NFR-2).

Invariant files: only `state/field.rs` is touched (item #2, one method on `Slot`). `sealed.rs`,
`signal.rs`, `schedule.rs` and `controllers.rs` are not touched.

Const generic N: every new item names `JOINTS` today and takes `<const N: usize>` under
`design-const-generic-n.md` unchanged (`Written<N>`, `Sample<'_, N>`, `HoldTarget<N>`,
`Hal<N>`). The `Slot::mask` method is the one that note already prototyped (its §2.4). Nothing
here waits for N.

## 1. The design in six lines

1. Desired carries one `Written` (a mask per field). Every setter sets its bit. The engine
   reads `Command` from Desired after `Last`, writes, then clears values and masks.
2. Each tick pushes a copy of `prev(0)` and lends it to `read` as a `Sample`. `read_all` fills
   one seed slot in PostInit.
3. `Robot::run_on<A: Hal>(self, hal: A) -> Result<(), A::Error>`. `run()` is
   `run_on(NoHardware)`. `Ok` for both signals.
4. Engine bring-up sits in `run_with`, right before each startup walk. `read` before `First`,
   `write` after `Last`, a new emergency-stop check between `Last` and the write.
5. One drop guard, `Powered`, is built just before `open`. Its `Drop` calls `torque_off` then
   `close`. Bring-up error (`?`), shutdown, emergency stop and panic unwind all leave through it.
6. ctrlc gets its `termination` feature: SIGTERM and SIGHUP raise `shutdown!` like Ctrl-C.
   `Time` gets public read-only getters with the SRS FR-3.3 names.

## 2. Sites to change

Line numbers are those of the current `kmr_core/src/` files.

### #2 Write mask, Command, clear after apply (SRS FR-2.5, Definitions "Write mask")

| Site | Change |
|---|---|
| `state/desired.rs:7-11`, `:13-21` | Add `written: Written` (from `hal`) to `Desired`; `Written::default()` in `Default` |
| `state/field.rs:15-18` (**invariant**) | `Slot` gains `fn mask(written: &mut Written) -> &mut Mask` |
| `state/field.rs:28-35`, `:43-50`, `:58-66` | One `mask` per impl: `&mut written.q`, `.qd`, `.tau` |
| `state/robot_state.rs:43` | After the value store: `<T as sealed::Slot>::mask(&mut self.desired.written).set(index)`. An out-of-range index returns before it |
| `state/robot_state.rs:52` | `set_all` also sets that field's mask to `Mask::ALL` |
| `state/robot_state.rs`, after `:122` | New `pub(crate)` engine block: `written()`, `command()`, `clear_desired()` |

```rust
pub(crate) fn command(&self) -> Command<'_> {
    Command::new(&self.desired.q, &self.desired.qd, &self.desired.tau, &self.desired.written)
}
pub(crate) fn clear_desired(&mut self) {
    self.desired = Desired::default(); // values and masks (SRS FR-2.5)
}
```

`hal::Command` and `hal::Written` stay as they are. The mask lives in Desired, apart from the
arrays (SRS NFR-10). The `Batch` pre-sort already reads `Written` (SRS FR-5.3).

Startup writes (FR-020, SRS FR-5.2 "no tick-0 seed"): after each startup walk, if
`written().any()` is not empty, warn once naming the phase and clear. One helper,
`discard_startup_writes(state, "Init")`, in `runtime.rs`.

### #3 One History slot per tick, `read_all` seed (SRS FR-2.3, FR-2.6)

| Site | Change |
|---|---|
| `state/history.rs:6` | `State<N>` derives `Clone, Copy` |
| `state/robot_state.rs`, engine block | `push_slot()` and `hold_target()` |

```rust
/// Pushes a copy of `prev(0)` (zeros when empty) and lends it to the adapter.
pub(crate) fn push_slot(&mut self) -> Option<Sample<'_>> {
    let next = self.history.buf.back().copied().unwrap_or_default();
    self.history.buf.push_back(next);
    let slot = self.history.buf.back_mut()?; // None only with a depth of 0
    Some(Sample::new(&mut slot.q, &mut slot.qd, &mut slot.tau))
}
```

The copy gives SRS FR-2.6 for free: the adapter overwrites only what it read, so a failed read
keeps the last value, never zeros. The same method seeds History: PostInit calls `push_slot()`
then `read_all(sample)`. The seed is the slot before tick 0, so at tick 0 `prev(1)` is the seed.

The `Mask` that `read` returns (refreshed actuators) is ignored until #4 (sample age).

### #8 `run_on` (SRS FR-5.1, section 7 "User API")

| Site | Change |
|---|---|
| `robot.rs:215-222` | `run()` calls `run_on(NoHardware)`; new `run_on` |
| `runtime.rs:342` | `runtime(robot, hal: A) -> Result<Signal, A::Error>` |
| `runtime.rs:298` | `run_with(robot, host, hal: &mut A) -> Result<Signal, A::Error>` |

```rust
pub fn run_on<A: Hal>(self, hal: A) -> Result<(), A::Error> where CS: Drive {
    runtime::runtime(self, hal).map(|_signal| ())
}
pub fn run(self) -> Result<(), ApiError> where CS: Drive {
    match self.run_on(NoHardware) { Ok(()) => Ok(()), Err(never) => match never {} }
}
```

No dependency on the open "run() after an emergency stop" question: both signals are `Ok`,
the signal is logged (FR-038). `ApiError` is unchanged; #7 replaces it later.

Optional, in `hal.rs` before `:349`: `impl<A: Hal> Hal for &mut A` (13 forwarding lines) and
its seal impl. Then `run_on(&mut adapter)` hands the adapter back after the run. The adapter
end-to-end tests need it, and it is option A of FR-050 (SRS FR-5.9: the user reaches the HAL
before `run()` and after it returns).

### #9 HAL call points (SRS FR-3.4, FR-5.2; contract table FR-039)

The guard (#10) must live as long as every phase, so the engine bring-up moves into
`run_with`. `init_phase::init` and `post_init` stay controller walks; their TODO blocks
(`runtime.rs:160-164`, `:188-197`) are deleted.

| Moment | Call | Site |
|---|---|---|
| Finalization, no I/O | `check(Overrides::default())` → `Layout` | `run_with`, before `pre_init` (`:299`) |
| Init, before user `Init` | guard, `open`, `zero` | between `:313` and `:314` |
| PostInit, before user `PostInit` | `ping`, `health`, `configure`, `push_slot` + `read_all`, `torque_on` | between `:317` and `:318` |
| Tick, after `begin_tick` | `push_slot` + `read` | after `:264` |
| Tick, after `Last` | estop check, then `Batch::new`, `write` if not empty | replaces the TODO at `:276-277`, after `:279` |
| Tick, after the end poll | `clear_desired()` | before the sleep, `:291` |

`Overrides::default()` until #5 adds the cadence knob. `Faults` is a local per tick, dropped
until #13 (see SRS problems).

The tick now has five bus checks, not four: tick start, after `First`, after `EachTick`,
after `Last` (new: an estop raised in `Last` skips the write, spec Edge Cases), tick end. Each
is one atomic load (SRS NFR-1). The test constants become `AFTER_LAST = 5`, `TICK_END = 6`,
`PER_TICK = 5`.

### #10 and #11 Exit paths, drop guard, SIGTERM and SIGHUP (SRS FR-4.1 to FR-4.3, FR-5.6, NFR-2)

```rust
/// Owns the adapter from `open` on: every exit, unwind included, ends with
/// `torque_off` then `close` (SRS FR-5.6, NFR-2).
struct Powered<'a, A: Hal>(&'a mut A);

impl<A: Hal> Drop for Powered<'_, A> {
    fn drop(&mut self) {
        self.0.torque_off();
        self.0.close();
    }
}
```

It is built just before `open`, so a failed `open` is closed too (FR-043). Before it, nothing
was opened: `check` errors and an estop in `PreInit` touch no hardware.

End of `run_with`:

```rust
let signal = control_loop(robot, &mut state, &sensors, host, powered.0, &layout);
let signal = estop(host.poll()).unwrap_or(signal); // estop since the loop ended: no hold
if signal.kind() == SignalKind::Shutdown && let Some(target) = state.hold_target() {
    powered.0.hold(target.command());
}
state.clear_desired();
Ok(signal) // `powered` drops here: torque_off, close
```

**The hold target decides something.** Desired is cleared after each apply, and the Dynamixel
adapter's `hold` sends `last.q` to every powered actuator (`kmr_adapters/src/dynamixel/mod.rs`,
`hold`). Handing it Desired after the clear would command 0 rad, the register centre, on every
motor at shutdown. `hold_target()` returns an owned copy: the latest sample, overlaid with the
fields written this tick, and `written` set to this tick's masks. That is "the tick's command
and the latest sample" of FR-032 in one `Command`. The loop breaks before `clear_desired` on a
signal, so the tick's writes are still there. The doc comment at `hal.rs:337` changes from
"`last.written` is ignored" to "`written` marks values from this tick's command; the rest is
the latest sample".

The hold duration (FR-033, `Option<Duration>`) is not on the critical path: `None` is the
default and gives torque off right after the hold. A `Some(d)` wait needs a sleep that a
later estop can cut, so the Host seam needs a polled wait. Later commit.

SIGTERM and SIGHUP: `kmr_core/Cargo.toml:13` becomes
`ctrlc = { workspace = true, features = ["termination"] }`. The handler at `runtime.rs:349-351`
is unchanged but its reason names all three signals. `systemctl stop` sends SIGTERM, a dropped
SSH session sends SIGHUP: both now end in torque off. SIGKILL, SIGQUIT and `panic = "abort"`
still leave torque on (SRS NFR-2 accepts it).

A panic in a controller unwinds through `run_with`, the guard runs, then the panic goes on to
the caller. `run_on` does not catch it.

### #14 Time getters (SRS FR-3.3)

| Site | Change |
|---|---|
| `clock.rs:89-93` | New fields `tick: u64`, `overrun_count: u64` |
| `clock.rs:179` | `end_tick` increments `tick` |
| `clock.rs:188-189` | A late tick increments `overrun_count` |
| `clock.rs:222-225`, `:244-246` | The `pub(crate)` `elapsed()` and `accumulated_overrun()` go |
| `clock.rs`, after `:247` | One `pub` block, values by copy |

```rust
impl Time {
    pub fn elapsed(&self) -> Duration;              // since `startup`, stamped at tick start
    pub fn dt(&self) -> Duration;
    pub fn tick(&self) -> u64;                      // index of the tick in progress, 0 first
    pub fn overrun_count(&self) -> u64;
    pub fn last_overrun(&self) -> Option<Duration>; // previous tick, None if on time
    pub fn accumulated_overrun(&self) -> Duration;  // ZERO when none
}
```

`tick` counts closed ticks, so during tick k it reads k. `kmr_api::Time` forwards them
(critical path step 9). Call sites to update: `runtime.rs:332` (`!total.is_zero()`), runtime
tests `:615`, `:656-657`.

## 3. Order per tick and on every exit

### One tick

| # | Step | Hardware |
|---|---|---|
| 1 | bus check (any signal) | |
| 2 | `begin_tick(now)` | |
| 3 | `push_slot`, `read` | read due actuators |
| 4 | `First`, estop check | |
| 5 | `EachTick`, estop check | |
| 6 | `Last`, estop check (new) | |
| 7 | `Batch` from the masks, `write` if not empty | one transaction per group |
| 8 | `end_tick(now)`: tick, overrun | |
| 9 | bus check (any signal) | |
| 10 | `clear_desired` | |
| 11 | sleep to the next grid point | |

### Exit paths

| Exit | Last thing run | Tick write | Hold | Torque off, close | `run_on` returns |
|---|---|---|---|---|---|
| `check` error | `check` | no | no | no (nothing opened) | `Err(e)` |
| estop in `PreInit` | `PreInit` | no | no | no (nothing opened) | `Ok` |
| `open` error | `open` | no | no | yes, at once | `Err(e)` |
| estop in `Init` | `Init` | no | no | yes, at the next check | `Ok` |
| `zero` .. `torque_on` error | the failed call | no | no | yes, at once | `Err(e)` |
| shutdown before tick 0 | `PostInit` | no | seed sample | yes, after hold | `Ok` |
| shutdown in a tick | `Last`, write, `end_tick` | yes | sample + tick's writes | yes, after hold | `Ok` |
| estop in `First` / `EachTick` / `Last` | that phase | no | no | yes, at that check | `Ok` |
| estop after a shutdown, before hold | the tick | yes | no | yes | `Ok` |
| controller panic after `open` | the panicking call | no | no | yes, during unwind | panics on |
| Ctrl-C, SIGTERM, SIGHUP | as shutdown | as shutdown | as shutdown | as shutdown | `Ok` |
| SIGKILL, SIGQUIT, abort | anything | — | — | **no**, torque stays | — |

## 4. Prototype

Path: `/tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/critpath/`.
A copy of the workspace (kmr_core, kmr_adapters, kmr_api, kmr_model, controllers), the design
applied to the core copy. Full diff: `critpath.diff` there (core and adapters only).

Two rigs:

- **Core:** the existing scripted `Host` plus a recording fake `Hal` (`Recorder`). Controllers
  and adapter write to one call log, so the test sees the interleaving. It can fail any
  bring-up step. 16 tests in `runtime::tests::exit_paths`.
- **Adapter:** the real Dynamixel adapter on its scripted backend, opened on a
  pseudo-terminal, run through the public `run_on(&mut adapter)` on the real host and bus.
  4 tests in `dynamixel::tests::engine`. They check the backend's transactions: goal writes,
  `Torque(off)` for ids 1 to 4, `ClosePorts` last.

Results, `cargo test --workspace --offline`, rustc 1.97.1:

| Workspace | Passed | Failed | Ignored |
|---|---|---|---|
| Repo today (baseline) | 191 | 0 | 7 |
| Prototype | 219 | 0 | 7 |

28 new tests, 0 regressions in kmr_api, kmr_model and controllers. `cargo clippy --workspace
--all-targets`: kmr_core keeps its 6 earlier warnings, no new one.

```text
dynamixel::tests::engine::a_missing_actuator_is_returned_with_torque_off_and_close ... ok
dynamixel::tests::engine::an_estop_skips_its_ticks_write_and_the_hold ... ok
dynamixel::tests::engine::shutdown_writes_three_ticks_then_holds_then_cuts_torque ... ok
dynamixel::tests::engine::a_panicking_controller_still_cuts_torque_and_closes ... ok
clock::tests::tick_and_overrun_count_follow_end_tick ... ok
runtime::tests::exit_paths::a_check_error_is_returned_before_any_io ... ok
runtime::tests::exit_paths::a_failed_open_is_still_closed ... ok
runtime::tests::exit_paths::a_panicking_controller_still_cuts_torque_and_closes ... ok
runtime::tests::exit_paths::a_ping_error_skips_post_init_cuts_torque_and_is_returned ... ok
runtime::tests::exit_paths::a_failed_torque_on_still_cuts_torque ... ok
runtime::tests::exit_paths::an_estop_after_each_tick_skips_last_the_write_and_the_hold ... ok
runtime::tests::exit_paths::a_shutdown_before_the_first_tick_holds_the_seed ... ok
runtime::tests::exit_paths::an_estop_in_pre_init_touches_no_hardware_after_check ... ok
runtime::tests::exit_paths::an_estop_raised_after_the_shutdown_tick_skips_the_hold ... ok
runtime::tests::exit_paths::an_unwritten_actuator_gets_no_command_and_only_its_field_is_sent ... ok
runtime::tests::exit_paths::every_tick_runs_read_first_each_last_write_and_desired_clears ... ok
runtime::tests::exit_paths::an_estop_in_init_after_open_cuts_torque_and_closes ... ok
runtime::tests::exit_paths::an_estop_raised_in_last_skips_the_write ... ok
runtime::tests::exit_paths::shutdown_reads_before_first_writes_after_last_then_holds_and_cuts_torque ... ok
runtime::tests::exit_paths::history_gets_one_slot_per_tick_seeded_by_read_all ... ok
runtime::tests::exit_paths::startup_writes_never_reach_the_adapter ... ok
state::test::a_write_sets_the_bit_of_its_field_only ... ok
state::test::each_pushed_slot_starts_as_a_copy_of_the_last ... ok
state::test::an_out_of_range_write_sets_no_bit ... ok
state::test::set_all_marks_every_actuator_of_its_field ... ok
state::test::the_command_borrows_desired_and_clear_resets_values_and_masks ... ok
state::test::the_hold_target_is_the_sample_overlaid_with_this_ticks_writes ... ok
sigterm_then_sighup_each_end_a_run_as_a_shutdown ... ok
```

The tests were checked against broken code, to show they can fail:

- `Drop` without `torque_off`: 10 of the 15 exit-path tests of that run fail.
- Desired cleared before the hold: the shutdown-order test fails on the hold values.
- ctrlc without `termination`: the test binary dies, `signal: 15, SIGTERM`.

The SIGTERM test sends `kill -TERM` then `kill -HUP` to its own process from a controller
(`kmr_core/tests/termination.rs`, its own binary since the handler is process-wide).

## 5. Commit order, smallest first, each green

| # | Commit | Items | Files | Test to add |
|---|---|---|---|---|
| 1 | `feat(core): read-only Time getters` | #14 | `clock.rs`, 3 lines in `runtime.rs` | `tick_and_overrun_count_follow_end_tick` |
| 2 | `feat(core): stop on SIGTERM and SIGHUP` | #11 (half) | `Cargo.toml`, `runtime.rs:350` | `tests/termination.rs` |
| 3 | `feat(core): per-field write mask` | #2 | `desired.rs`, **`field.rs` (invariant)**, `robot_state.rs` | `a_write_sets_the_bit_of_its_field_only`, `an_out_of_range_write_sets_no_bit`, `set_all_marks_every_actuator_of_its_field` |
| 4 | `feat(core): command and clear after apply` | #2 | `robot_state.rs` | `the_command_borrows_desired_and_clear_resets_values_and_masks` |
| 5 | `feat(core): one History slot per push, hold target` | #3 | `history.rs`, `robot_state.rs` | `each_pushed_slot_starts_as_a_copy_of_the_last`, `the_hold_target_is_the_sample_overlaid_with_this_ticks_writes` |
| 6 | `feat(core): run_on over any adapter` | #8 | `robot.rs`, `runtime.rs` (signatures, `check` only), `hal.rs` (`&mut A` impl) | the `Recorder` fake; `a_check_error_is_returned_before_any_io` |
| 7 | `feat(core): bring-up calls behind a torque-off guard` | #9 (startup), #10, #11 | `runtime.rs` | `a_failed_open_is_still_closed`, `a_ping_error_skips_post_init_cuts_torque_and_is_returned`, `a_failed_torque_on_still_cuts_torque`, `an_estop_in_pre_init_touches_no_hardware_after_check`, `an_estop_in_init_after_open_cuts_torque_and_closes`, `a_shutdown_before_the_first_tick_holds_the_seed`, `startup_writes_never_reach_the_adapter`, `a_panicking_controller_still_cuts_torque_and_closes` |
| 8 | `feat(core): read before First, write after Last` | #9 (tick), #10 | `runtime.rs`, test constants | `shutdown_reads_before_first_writes_after_last_then_holds_and_cuts_torque`, `every_tick_runs_read_first_each_last_write_and_desired_clears`, `an_estop_after_each_tick_skips_last_the_write_and_the_hold`, `an_estop_raised_in_last_skips_the_write`, `an_estop_raised_after_the_shutdown_tick_skips_the_hold`, `an_unwritten_actuator_gets_no_command_and_only_its_field_is_sent`, `history_gets_one_slot_per_tick_seeded_by_read_all` |

Steps 1 to 5 add dead code the runtime does not call yet; `lib.rs` allows it. Step 7 lands the
guard in the same commit as `open`, so no commit opens hardware without it. After step 8, the
agent adds the 4 adapter end-to-end tests in `kmr_adapters` (not human-only), and wires
`kmr_api::Robot::run()` to `run_on(kmr_adapters::Hardware::default())` (critical path step 9).

Adapter side, not core: critical path step 2 asks torque off on read-only actuators too
(SRS FR-5.8). The Dynamixel `torque_off` still skips them. The bench check of step 11
(`Torque_Enable` = 0 on every id) catches it.

## 6. Open choices for the maintainer

1. **Hold target.** Recommended: latest sample overlaid with the tick's writes (above).
   Simpler: the latest sample only; the final write's targets are then overridden by measured
   positions, which lag. Either way, never Desired after the clear.
2. **`write` on an empty batch.** The prototype skips the call. Calling it anyway costs one
   call per tick and lets an adapter that answers `read` from the previous write's reply
   (CubeMars, decision 0002 R13) keep its rhythm.
3. **`impl Hal for &mut A`.** Needed by the adapter end-to-end tests; also FR-050 option A.
   Without it, those tests need a shared log in the scripted backend.
4. **Estop check before `torque_on`.** Today an estop raised during `ping` .. `read_all`
   still powers the actuators, until the first loop check a few µs later. One more check
   before `torque_on` closes that. Not in the prototype.
5. **`last_overrun()` meaning.** Prototype: the previous tick's lateness, `None` if on time.
   Alternative: the most recent overrun, sticky. Both derive from `overrun_count` and
   `accumulated_overrun`.
6. **`elapsed()` origin.** Today `startup` is set in `Robot::new()`, so `elapsed()` includes
   the builder and bring-up. Alternative: measure from `arm` (tick 0's grid origin).
7. **Startup-write warning.** The prototype warns once per startup phase that wrote. SC-011
   says exactly one warning; with writes in two phases that is two.

## 7. SRS problems found

1. **SRS FR-4.1 does not say what the hold command is built from.** Desired is cleared after
   apply (SRS FR-2.5), so a hold read from Desired commands zeros: the register centre on
   every Dynamixel (decision 0002 R10). The SRS should say: latest sample, overlaid with the
   last tick's command. The section 7 HAL row and `hal.rs:337` should follow.
2. **SRS FR-4.3 "Ctrl-C (or the platform equivalent)".** SIGTERM (service stop) and SIGHUP
   (lost SSH session) are not Ctrl-C equivalents, but both must end in torque off. The SRS
   should name them. SIGQUIT and SIGKILL leave torque on; NFR-2 should list them next to abort.
3. **SRS FR-4.2 "next phase boundary, startup phases included"** is silent on the engine
   bring-up steps inside PostInit. Without a check before `torque_on`, a pending estop still
   powers the actuators once (open choice 4).
4. **SRS FR-3.3** gives no meaning for `last_overrun()` and no origin for `elapsed()`
   (open choices 5 and 6).
5. **SRS FR-2.3 "prev(k) always means k ticks ago"** does not cover the `read_all` seed. At
   tick 0, `prev(1)` is the seed slot, read in PostInit, not a tick. Say so in the SRS.
6. **SRS FR-13.4 "no fault is silent"** cannot hold on the first motor: #13 (per-slot flags)
   is off the critical path, so the per-tick `Faults` the adapter fills are dropped. Either
   accept it for delivery step 1, or move #13 onto the critical path.
