Superseded on 2026-10-28 by plan.md and tasks.md; torque and sync-read findings carried into tasks T010-T017, T050-T051.

# Fix plan: kmr_adapters audit

A plan only. No repo code was changed. `hal.rs` is out of scope (you gave it a pass).

## Order of work

1. Decision: answer the five questions at the end.
2. dxl_rs repo: push `feat/battery-status`, add force torque off and the block sync read. CI is blocked until then.
3. Agent (root, ci.yml): fetch `dxl_rs` from git and fix CI.
4. Agent (kmr_adapters): make the fake backend strict and add the torque test. Tests go red.
5. Agent (kmr_adapters): fix torque off, Protocol 1 health, the sync read and the default cadence. Tests go green.
6. You (kmr_core): `run_on`, the call points and the drop guard.
7. Agent (kmr_adapters, then kmr_api): the robot is declared in `main.rs`.
8. You (kmr_core), after decision 3: the actuator count follows the robot.

## Declaring the robot in main.rs

- **Wrong:** the robot is hard-coded in `kmr_adapters/src/robot.rs` (the framework). The HAL is picked by a Cargo feature, and without it `run()` silently drives no motors.
- **Idea:** `main.rs` is the only file. KM-RoBoTa ships one preset per robot (brand, models, ids, baud, modes). A custom robot uses the same calls. The robot goes into the builder like a controller does.
- **Mechanism:** builder type-state, as for the controllers' HList. The robot is a generic type down to `run_on`: no heap, no `dyn`. Verified in a scratch lib + bin (`cargo build --offline`): `run()` made 0 heap allocations, the lib has no `dyn`. Not chosen: a top-level `inventory`-style registry collects one fixed type at run time, so a missing robot cannot be a compile error.

```rust
// (a) preset (10 actuators: builds after decision 3)
Robot::new()
    .robot(robot!(SNAKE_10.baud("back", 3_000_000).read_only([9])))
    .port(by_id!("FT8ISZ6H", "front"))
    .port(by_id!("FT9XYZ12", "back"))
    .set_dt(Duration::from_millis(1))
    .add_controller(EachTick, wave)
    .run()

// (b) custom robot: only the .robot and .port lines change
    .robot(robot!(dynamixel().bus("arm", 1_000_000).motors("arm", XM430_W210, [1, 2, 3, 4])))
    .port(by_dev!("/dev/ttyACM0"))
```

1. **Preset:** `robot!(SNAKE_10)`. Presets are `const` values, one file each in `kmr_adapters/src/presets/`, re-exported by kmr_api. Today's `robot.rs` becomes the first one, `BENCH_4`. `robot!` evaluates the robot in a `const` block, so a bad one does not build (SRS FR-6.4: bad descriptor = compile error). Verified: a bad baud and a wrong actuator count fail the build.
2. **Ports:** `by_id!(serial, names..)` looks in `/dev/serial/by-id`, `by_dev!(path, names..)` takes the path as is. Names are the preset's bus names. No name is allowed when the robot has one bus. An unknown name or a bus with no port is a `run()` error before any I/O, naming the line to add (verified). A missing device is an Init error (SRS FR-6.3: ports bound per machine, resolved at bring-up).
3. **Override:** `const fn` methods on the robot, the same for presets and custom robots: `.baud(bus, b)`, `.id(old, new)`, `.mode(id, m)`, `.read_only([ids])`, `.read_every(ticks)`. Inside `robot!` they are checked at compile time (verified for `baud`, `read_only`, `read_every`).
4. **Custom robot:** `dynamixel()` names the brand, `.bus(name, baud)` the bus, `.motors(bus, model, [ids])` the actuators. Mode and torque default per model. A preset is exactly this chain, written by us.
5. **No robot declared:** compile error at `.run()`, through `#[diagnostic::on_unimplemented]` on a kmr_api trait (SRS NFR-4: type-state over runtime checks). Verified text:
   ```
   error[E0277]: no robot declared: `run()` does not know which motors to drive
     = note: add `.robot(robot!(BENCH_4))` (a preset) or `.robot(robot!(dynamixel().bus(..).motors(..)))` before `.run()`
     = note: to run without motors on purpose, add `.robot(NoHardware)`
   ```
6. **Who writes what:**
   - You (kmr_core): only `run_on` (below). The count, per decision 3.
   - Agent (kmr_adapters): the robot config as plain data, built by `const fn`, fixed capacity of 8 buses, so user code names no generic. `validate` becomes the `check` that `robot!` calls. `bind(robot, ports)` copies the ports into a fixed array; finding the device stays in `open()`. Remove `Hardware`, `impl Default`, `PORTS` and the `dynamixel` feature.
   - Agent (kmr_api): `Robot<C, R = NoRobot>`, `.robot()`, `.port()`, the sealed trait with the hint, the macros `robot!`, `by_id!`, `by_dev!`. `run()` is `self.inner.run_on(self.robot.bind(ports)?)`.
- **Limit:** `JOINTS` is a kmr_core const (4), compiled before `main.rs`. Until decision 3, only 4-actuator robots build; others get the count compile error.
- **Done when:** `main.rs` is the example above and moves the motors. Nothing else is edited.

| Missing | Result |
| --- | --- |
| `.robot(..)` | Compile error with the hint above. |
| Unknown bus name, bus without port | `run()` error before any I/O, naming the line to add. |
| USB adapter unplugged | Init error `PortNotFound` with bus and serial. |
| Actuator not answering | PostInit ping error. The binary still builds (SRS FR-5.5: presence checked at bring-up). |

## Torque can stay on

- **Wrong:** `torque_off` skips read-only actuators (`mod.rs:578`), the real backend cannot cut an actuator this process did not switch on (`dxl_rs.rs:229`), and a panic or SIGTERM skips torque off. SRS FR-4.2 (emergency stop cuts torque), FR-5.6 (shutdown turns torque off), FR-5.8 (read-only means torque off).
- **Fix:**
  1. dxl_rs repo: `force_torque_off(&self)` on `TorqueOff` in `build/actuator_codegen.rs`, writing `Torque_Enable = 0`.
  2. Agent (kmr_adapters): `unpower` falls back to it when it holds no handle. `torque_off` cuts every opened actuator, read-only ones included. At PostInit, write torque off on every read-only actuator, and drop the "read-only but torque on" error from `configure`.
  3. You (kmr_core): after `open`, keep the HAL in a drop guard that calls `torque_off` then `close`. Panics included.
  4. Agent (root `Cargo.toml`): ctrlc's `termination` feature, so SIGTERM raises `shutdown!`.
- **Done when:** in tests, every id starts powered, and an emergency stop or a panicking controller ends with none powered. On the bench, Dynamixel Wizard shows `Torque Enable = 0` on every id.

## The fake backend hides real limits

- **Fix:** Agent (kmr_adapters): the fake's Protocol 1 `read_health` returns `Err(Io)` like the real one (`scripted.rs:531`). New test: every id starts powered, one is read-only, `torque_on` never runs, then `torque_off`; no id stays powered. Red before the torque and Protocol 1 fixes, green after.

## Protocol 1 cannot bring up

- **Wrong:** `dxl_rs` cannot read Protocol 1 voltage or temperature, so `health()` fails for AX-12A and RX-28 (SRS FR-5.2: health check at PostInit). **Fix:** decision 2.

## Three sync reads per Protocol 2 group

- **Wrong:** `sync_read_state` (`dxl_rs.rs:416`) sends three transactions per group (SRS FR-5.3: one read per bus and series).
- **Fix:** dxl_rs repo: one block sync read of 10 bytes at address 126. Agent (kmr_adapters): use it; the fake records one entry per wire transaction, and tests count one per group.

## Every tick is read

- **Wrong:** `read_every` is 1 in `robot.rs:17` and `config.rs:213, 242, 274, 529` (SRS FR-5.3: Dynamixel does not read every tick).
- **Fix:** Agent (kmr_adapters): a const default cadence, used when a bus sets none; you pick the value on the bench. Presets set none. `.read_every()` overrides it.

## The engine never calls the HAL

- **Fix (You, kmr_core):**
  1. `run_on<A: Hal>(self, hal: A) -> Result<(), RunError<A::Error>>`. Name it `A`: `H` is already the `Host` seam.
  2. Wire the call points (`runtime.rs:160, 188, 276, 327`). Shutdown: after the tick's write, `hold(prev(0))`, then the guard. Emergency stop: the guard at the next phase boundary. Bring-up error after `open`: the guard, then the error.
  3. Loop tests: `NoHardware` for the existing ones, a recording HAL for each exit, each ending with `torque_off`, `close`.

## CI cannot fetch dxl_rs

- dxl_rs repo: commit or drop the `.ron` edits, push `feat/battery-status`.
- Agent (root `Cargo.toml`): git dependency on that branch; `cargo update -p dxl_rs` after each push.
- Agent (ci.yml): decision 1, install libclang, cross-build `controllers` for aarch64.

## Docs and SRS

- Agent (tasks.md): untick the done-but-not-done tasks: git dependency, one 10-byte sync read, Protocol 1 health, torque off on every actuator, green cross job, quickstart run.
- Agent (docs): research.md, plan.md, quickstart.md, contracts/api.md, `ARCHITECTURE.md:211-242`, `ROADMAP.md:40` follow this plan. Drop "port binding as a const" from `docs/decisions/0002-hal-contract.md`.
- You (SRS): FR-6.1 (descriptor read by the controller's `build.rs`, ports in their own file) becomes "the robot is a preset or a custom declaration in `main.rs`, ports in the builder". FR-6.3 says "bound by serial id": `by_dev!` needs a line allowing paths. The Config row in section 7 drops "file".

## Decisions for you

1. **dxl-rs in CI:** A. make the repo public. B. keep it private, ssh-agent step in CI (fork PRs still fail).
2. **Protocol 1 health:** A. skip it with one warning, as an SRS exception you write. B. dxl_rs adds a 2-byte read at address 42 first.
3. **Actuator count follows the robot:** A. `N` becomes a const generic in kmr_core; `robot!` moves to a top line and also emits `type State = State<10>`, so controllers still write `State` (verified). Cost: `Mask` needs a fixed word cap, since `[u64; N.div_ceil(64)]` does not compile on stable (verified). B. keep `JOINTS`, set from a preset name in `.cargo/config.toml`. Breaks the one-file rule.
4. **Several names in one `by_id!`:** A. several preset buses wired to one adapter (same baud, checked at `run()`). B. names are groups, and the bus follows from the robot.
5. **SRS FR-5.9 (user may reach the HAL directly):** `run()` builds the HAL, so user code has no handle. A. defer, noted in the SRS. B. design the access now.
