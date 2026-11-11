# Plan: homing as a kmr_api preset

Written 2026-11-10 for the 2026-11-11 session. Nothing is built yet.

## Decided (maintainer, 2026-11-10)

- **Homing is not a core feature.** It is a controller preset in `kmr_api::presets`, used
  optionally. A robot needs no home position to be valid.
- **The ramp is a `Duration`.**
- Closes SRS FR-2.7 (§ 8 open item): write it there as "home is a kmr_api preset, not
  engine state".
- **Registered with plain `add_controller`, not `add_controller_with`** (2026-11-11). The
  config is a `const` (`Home::new(pose, ramp)` is a `const fn`). The preset returns an
  `impl Fn` whose run-time state sits in a `Cell`, so `add_controller`'s signature stays as
  it is: `.add_controller(EachTick, HOME.controller())`. Checked on rustc 1.97.1.
  Passing `HOME` bare would need a new trait in `add_controller`'s pub signature: a
  separate decision. A variadic `add_controller!` macro: maybe later.
- Name: `home` (not "homing"). Module path still open: proposed `kmr_api::presets::home`.
- **Q4, partial homing: all actuators only** (2026-11-11). The pose is `[Q; N]`; a group
  variant can come later without breaking anything.
- **Q2, custom dt: dropped** (2026-11-11, agent call by kmr-v3-93, maintainer may
  overrule). The preset writes every tick; per-controller rate stays roadmap (SRS FR-1.5).
- **Preset (b): later** (maintainer, 2026-11-11). Both halves (before power-up, and limp
  during the run) wait; not in this change.
- **FR-1.1 exception approved** (2026-11-11), on condition that it needs no kmr_core
  change; it needs none (inline nodes carry no `Send`/`Sync` bound, `controllers.rs:54`).
  Why: every preset goes through `add_controller`, so users never have to remember which
  builder call a preset needs. Later, separately: one variadic `add_controller` merging
  `add_controller` and `add_controller_with`.
- **Was the conflict:** the `Cell` capture breaks SRS FR-1.1 as written
  ("mutable per-controller state goes through the context variants, never through
  captures"). Either FR-1.1 gets a kmr_api-preset exception, or the registration goes back
  to `add_controller_with`. No stateless ramp holds up: the start pose is older than
  `HISTORY_DEPTH`, and a step from the current read lags and jumps on the last tick.

## Where things stand

- `State::go_home` / `State::initial_state` and the `HomeDecided` bound
  (`kmr_api/src/home.rs`) are stubs that fail to compile on purpose. **Remove all three**:
  homing no longer lives on `State`.
- `controllers/src/main.rs` `home` already does preset (a) by hand: start pose in a
  context, lerp on `time.elapsed()`, `sine_all` waits for `HOME_TIME`.
- Startup phases discard Desired (SRS FR-5.2, `kmr_core/src/runtime.rs` `StartupWrites`),
  torque comes on at the end of PostInit. A preset is a normal controller, so **it runs on
  a tick phase (`EachTick` or `First`)**. FR-5.2 stays as it is, no core change for (a).

## The presets

### (a) Known home: ramp to it

Context: home pose `[Q; N]`, ramp `Duration`, start pose (captured from the first
`robot.q()`), start time. Each tick writes the lerp; after the ramp, writes the home pose
(or nothing) and reports done. Everything it needs is already on `State` and `Time`.

### (b) No home: the human sets it

For CubeMars, which do not keep a zero: the reference is wherever the human put the
robot. Two cases:

- Robot placed **before power-up** (CubeMars zero = power-up position): the preset only
  records the first read as the reference. Pure kmr_api.
- Robot placed **during the run** (Dynamixel, or re-placing): the actuators must be limp
  while the human moves them, then a confirmation (a user closure: key press, button
  sensor, timeout) records the pose. **Limp at runtime is not possible today**: torque is
  on from PostInit to exit, and only the engine cuts it. Needs a core change (a torque
  request from a controller), which is the loop's safety behaviour: **human call, defer**.

### (c) Auto limits (endstop search, like a 3D printer)

Not planned (maintainer not a fan). If it ever comes: very slow, one actuator at a time,
needs stall detection (load or current, or q not following the goal). Park it as a
`/issue feature` idea, nothing more.

## Open questions for tomorrow

1. **How the next controller knows homing is done.** The preset's context is moved into
   the robot, so `sine_all` cannot see it. Options:
   - a wrapper: the preset runs, then hands each tick to the user's controller
     (`presets::home(pose, ramp).then(sine_all)`). Recommend this: no shared state, the
     sine's time can start at zero when homing ends.
   - a `Latest` slot (already in kmr_api) the preset writes and the sine reads.
   - the user checks `time.elapsed() > ramp`, as today. Works, but leaks the ramp.
2. **"Custom dt".** A controller runs every tick; a slower rate per controller is on the
   SRS roadmap (per-controller tick rate), not in this version. Inside the preset it could
   only mean "write a new goal every k ticks" (position mode holds the last goal). Decide
   whether that is worth having or dropped.
3. **Registration shape (pub, needs a contract, step 4 on kmr_api):** a value plus a fn for
   `add_controller_with`, or a dedicated builder call (`.add_preset(EachTick, …)`). Aim:
   the user's line stays one call, no maths.
4. **Partial homing:** all actuators only, or a group/subset (`GroupView` exists)?

## kmr work

- SRS: close FR-2.7; add `presets` to kmr_api's module section.
- kmr_api: new `presets` module with (a) and the before-power-up half of (b); remove
  `home.rs` and its re-export; contract first (pub surface).
- Tests: reaches home at `ramp`, never overshoots, no write before the first read (no
  hardware), limit policy still applies to its writes, the wrapped controller starts only
  when done.
- controllers (maintainer's file): replace `home` with the preset.
- Docs: README, controllers README, `complete_api_example.rs` (still uses `Init`
  controllers and the old `go_home` story).
- `docs/deferred.md`: runtime limp/torque request for (b), idea row for (c).
- kmr_core follow-ups (not kmr_api's files): delete the PostInit "out-of-place check
  skipped: SRS FR-2.7" warn (`runtime.rs` ~561) and the `pub(crate) home_state` stubs
  (`state/history.rs`, `robot_state.rs`, their test in `state/test.rs`); then
  `StateError::Undecided` has no producer, keep or drop it.
- Docs also: `docs/ARCHITECTURE.md` (`add_controller(Init, home)` and ~336),
  `docs/ROADMAP.md:37`, decision 0005 (superseded note), `docs/class-diagram.drawio`.

## Contract status (kmr-v3-93, 2026-11-11)

**Update 2026-11-11:** signed off (FR-1.1 exception yes, `then` yes), built and committed
on `feature/kmr-api-home`: a4662e9 (T090 contract), 7418d84 (T091 build, 11 tests),
ef82cf8 (T092 docs), 1ff4fba (ROADMAP: kmr_api step 6). Workspace suite green (864
passed). Step-6 review done, its two real findings fixed in 7c46d13 (a closure reused in a
second run homes again; a NaN start reaches the limit stage); 868 passed. Waiting for the
maintainer's "merge". Change after sign-off, inside the oracle: on the hand-over tick
`then` writes the pose once before `f` (an actuator `f` leaves alone ends on home), then
nothing. Module path still `kmr_api::presets::home`, pending the maintainer.

Original draft note: worktree `.claude/worktrees/kmr-api-home`, branch
`feature/kmr-api-home` from `dev`. `kmr_api::presets` is now a kmr_api
module: `pub use kmr_adapters::presets::*` (robot! unchanged, UI tests green) plus
`pub mod home`. Pub surface: `Home<N>` (Clone, Copy, Debug), `const fn new(pose, ramp)`,
`controller(self)`, `then(self, f)`; removed `HomeDecided`, `State::go_home`,
`State::initial_state`. SRS: FR-2.7 closed, FR-1.1 Cell exception (needs yes), FR-5.2 and
the Bring-up definition drop "flag out-of-place ones", § 8 item ticked, kmr_api Covers
FR-2.7. C4 `kmr_api.dsl` updated. Behaviour in the docs, for sign-off: `controller()` keeps
writing the pose every tick after the ramp; `then(f)` writes nothing itself once `f` runs.

## dxl_rs_v2

**No update needed** for (a) or the before-power-up half of (b): ordinary goal writes and
reads. The limp half of (b) goes through the adapter's existing torque calls
(`disable_torque` / `enable_torque`), so the change is on the kmr core/HAL side, not in
dxl_rs_v2. (c) would need typed present load/current reads; not checked.
