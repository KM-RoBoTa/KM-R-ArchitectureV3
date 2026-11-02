# Architecture

`kmr` is one Cargo workspace. Four crates make the framework, a fifth holds
user code:

- **`kmr_core`** (`kmr_core/`) — the engine. Owns state, schedules,
  the control loop, timing, the signal bus and the hardware contract.
- **`kmr_model`** (`kmr_model/`) — the robot model. A proc macro that reads
  the robot descriptor at compile time; users reach it only through
  `kmr_api::robot!`.
- **`kmr_adapters`** (`kmr_adapters/`) — the hardware adapters, one per
  vendor, behind the core's contract. Dynamixel first.
- **`kmr_api`** (`kmr_api/`) — the public user API. A thin, deliberate layer
  that re-exposes what users need and nothing else.
- **`controllers`** (`controllers/`) — where controllers are written. It
  depends on `kmr_api` only.

The mental model is borrowed from [Bevy](https://bevyengine.org/)'s ECS —
plain functions registered against schedules — but it deliberately stops short
of a query system. A robot has one fixed-shape state, not an open world of
entities; a runtime query system would only re-derive what the compiler
already knows.

## Deployment

One `kmr` process drives the robots wired to its host, through its own
hardware layer. Two topologies:

- **One host, several wired robots.** They form a single `Robot`: `N` is the
  sum of their joints, with one state, one loop and one clock. Each robot is a
  group, and each can sit on its own bus (e.g. two snakes on two buses).
- **One instance per robot.** Each robot runs its own `kmr`, and the instances
  exchange messages over [Zenoh](https://zenoh.io/).

Zenoh carries messages between instances, not a control loop. A robot's
controllers run in the `kmr` that owns its hardware layer, so an instance
cannot drive a robot it only reaches over the network.

## Design principles

### 1. State is private

Joint positions (`q`), velocities (`qd`), torques (`tau`) and their history are
not fields you reach into. This does not compile:

```rust
robot.desired.q[0] = 1.0; // error: field `desired` is private
```

State is an invariant the engine maintains (units, ranges, history depth,
write-once-per-tick semantics). Exposing raw fields would make those
invariants unenforceable.

### 2. Access is through typed getters and setters

```rust
fn controller(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    let q0 = robot.q_at(0)?;          // read sensed position -> Q
    robot.set_q_at(q0 + Q(0.1), 0)?;  // request a desired position
}
```

- **Getters** read sensed or historical state: `q()`, `q_at(i)`,
  `prev_q(depth)`, `prev_q_at(i, depth)` — and the same for `qd` / `tau`
  (aliases `position`, `velocity`, `torque`/`effort`).
- **Setters** write the *desired* state — a command to the actuators, not a
  mutation of reality: `set_all_q`, `set_q_at`, and the `qd` / `tau` variants.
- **Groups** give the same API on a fixed subset of joints:
  `robot.group([3, 4, 5])`.

Values are always the newtypes `Q`, `Qd`, `Tau` — never bare `f32` — so mixing
a position and a velocity is a compile error. Whole-array reads return
`Option` (a missing reading is recoverable); indexed and history reads return
`Result<_, StateError>` so the cause is inspectable via `e.kind()`.

### 3. The API wraps the core's handles and shares its value types

- `kmr_api::State` wraps `kmr_core::RobotState` (likewise `Time`, `Sensors`);
  the inner value is private and only the methods we choose to mirror exist.
- `Q`/`Qd`/`Tau` are defined **once**, in `kmr_core`, and re-exported by
  `kmr_api`. There is no conversion at the boundary: accessors copy the value
  out of the core's reference and return it owned. The arithmetic users rely
  on is derived on the core types, since the orphan rule prevents `kmr_api`
  from adding it.
- The writable field set is **sealed** by a private `Slot` trait: no
  downstream crate can add a field type or implement storage routing.
  `kmr_api::Field` is the core's `StateField` re-exported under that name.

Why wrap the handles but not the values? The handles carry invariants (desired
is write-only, history is read-only) and the public method set is a promise, so
adding a capability stays a deliberate act. The value types are plain `f32`
newtypes with nothing to protect; duplicating them only bought a second
definition to keep in sync.

### 4. Functions on schedules

Controllers are plain functions. Three registration forms exist:

| Method                                    | Controller signature                                  | Runs on      |
|-------------------------------------------|-------------------------------------------------------|--------------|
| `add_controller(S, f)`                    | `fn(&Time, &mut State, &Sensors)`                     | main loop    |
| `add_controller_with(S, ctx, f)`          | `fn(&mut T, &Time, &mut State, &Sensors)`             | main loop    |
| `add_controller_as_thread(S, ctx, f)`     | `fn(&mut T, &Time)`                                   | own thread   |

`ctx` is user data the engine owns and lends back each call. Threaded
controllers never receive `State`: robot state stays single-writer on the main
loop, so actuator-write races are impossible by construction.

```rust
Robot::new()
    .set_dt(Duration::from_millis(1))
    .add_controller(Init, home)
    .add_controller(EachTick, track_reference)
    .add_controller_with(EachTick, pad.clone(), use_gamepad)
    .add_controller_as_thread(EachTick, pad, read_gamepad)
    .run()?;
```

See [`kmr_api/examples/complete_api_example.rs`](../kmr_api/examples/complete_api_example.rs)
for a guided tour.

## Schedules

| Label      | When                                   |
|------------|----------------------------------------|
| `PreInit`  | Once, before init                      |
| `Init`     | Once                                   |
| `PostInit` | Once, after init                       |
| `First`    | Every tick, first                      |
| `EachTick` | Every tick                             |
| `Last`     | Every tick, last                       |

The `schedules!` macro (`schedule.rs`) generates, from one ordered list, the
zero-sized labels, their sealed `Schedule` impls, a `Schedules` struct with one
bucket per phase, and one `Insert<Label, Node>` impl per phase. Routing
happens **once**, when a controller is added: the label is a compile-time key
selecting the bucket to grow. Per tick the runtime walks each bucket in order —
no phase comparisons, no filtering.

These names are chosen for this engine; they intentionally differ from Bevy's.

## Static dispatch

Controllers live in a heterogeneous list (`frunk`'s `HCons`/`HNil`). Each
`.add_controller*` prepends one node and grows the `Robot<L>` type by one
layer. Everything is monomorphized: no `dyn`, no fat pointers, no heap. Node
types (`Inline`, `InlineWith`, `Threaded`) are opaque: users can hold the list
but can neither forge nor read a node.

`Robot::run()` rejects an empty controller list with a **runtime** check
rather than a trait bound, because a trait-bound failure would leak internal
type paths into the user's compiler error.

## Runtime loop

```
pre_init → init → post_init
loop { first → each_tick → last → clock bookkeeping → sleep }
```

The loop runs until a signal is raised. After the `Last` bucket the engine
computes the overrun and the accumulated overrun, stamps the last update and
sleeps until the next deadline.

Deadlines are absolute: each one is `origin + k * dt`, with `dt` the value
given to `set_dt`. Sleeping for `dt` after each tick would make the real
period `work + dt + wake latency` and the schedule would drift. When a tick
finishes after its deadline, the loop stays on the same grid: it skips the
slots it missed and sleeps to the first grid point still ahead, so the next
tick starts on the grid and has a whole `dt` to run. It never runs a burst of
ticks to catch up, never moves the grid and never starts a tick in the middle
of a slot.

Controllers read the clock, the engine writes it, and only between two
controller walks.

Not implemented yet: the `state_transition` step (write the desired state,
record the sensed sample), because the runtime does not call the
[hardware layer](#hardware-layer) yet, and the startup of threaded
controllers — see the [roadmap](ROADMAP.md).

## Signals: shutdown and emergency stop

Controllers talk back to the loop through one channel: a process-global,
lock-free signal bus raised with `shutdown!("reason")` or
`emergency_stop!("reason")`. They work from anywhere — nested helpers,
closures, other threads, a raw signal handler (ctrl-c is wired to `shutdown!`).

They are macros so each call site gets its own `static` metadata: nothing
allocates and the path is async-signal-safe. Escalation is monotonic — an
emergency stop is never downgraded to a shutdown, and the first reason raised
wins.

The runtime checks the bus between the phases, four times per tick (one
atomic load each on the happy path):

- a **shutdown** lets the current tick finish, `Last` included, then `run()`
  returns;
- an **emergency stop** ends the loop at the next check: the phases left in
  the tick are skipped. Controllers registered after the raising one in the
  same phase still run.

Both are logged with their reason and, in debug builds, their call site.
`run()` returns `Ok(())` in both cases.

## Robot model

`kmr_api::robot!` names the robot once, in `main.rs`: a preset
(`robot!(XmBench)`) or a descriptor path (`robot!(model = "arm.urdf")`).
Its front door calls the `kmr_model` proc macro, which reads the descriptor
(URDF, `.ron` or JSON, one schema for the last two) inside the compiler,
checks it, and emits constants at the call site:

- `N`, the actuator count: every joint that is not fixed, in file order;
- `model::NAMES`, `LOWER`, `UPPER`, `SPEED`, `EFFORT`, one array per fact,
  indexed by actuator, in `Q`, `Qd`, `Tau` and SRS FR-2.2 units (prismatic in
  mm);
- `model::LINK_NAMES`, `LINK_PARENT`, `LINK_MASS`, `LINK_COM`, per link;
- `actuator!("name")`, the index of a joint by name; another name does not
  build.

The binary gets constants only: no parser, no heap, no run-time code. A bad
descriptor is a compile error naming the file, the element and the reason. A
preset's model half ships in `kmr_model/presets/`, its hardware half in
`kmr_adapters::presets`. `kmr_model` uses the heap at build time, which
`kmr_model/clippy.toml` allows.

Not wired yet: the front door still counts joints through the older
`actuator_count` macro, so `model::*` and `actuator!` are not reachable
through `robot!`, and N must equal the core's `JOINTS` until the core takes
N as a const generic. The kind bit sets `model::PRISMATIC` and `CONTINUOUS`
wait for the core's public mask width.

## Hardware layer

The engine drives hardware through one contract, the `Hal` trait in
`kmr_core/src/hal.rs`. It knows actuator indices, bit masks, opaque
transaction groups and a read cadence in ticks. Buses, series, protocols,
modes and raw units stay in the adapter; values cross as `Q`, `Qd`, `Tau` in
SI units. `Hal` is public with a `#[doc(hidden)]` seal: adapter crates
implement it, `controllers` cannot, since it does not depend on `kmr_core`.
Dispatch is static: `run_on(hal)` runs the robot on an adapter, and `run()` is
`run_on(NoHardware)`.

| Moment | Engine calls |
|---|---|
| `run()` finalization, no I/O | `check` → `Layout`: groups, read cadence, cost |
| Init, before user controllers | `open`, `zero` |
| PostInit, before user controllers | `ping`, `health`, `configure`, `read_all`, `torque_on` |
| Each tick, before `First` | `read` into a copy of the previous sample |
| Each tick, after `Last` | `write` of the written actuators in group order, then Desired is cleared |
| Shutdown | `hold`, `torque_off`, `close` |
| Emergency stop | `torque_off`, `close` |

Bring-up calls return the adapter's error and `run()` returns it. Once `open`
was called, every exit ends with `torque_off` then `close`. `read` and `write`
never fail: they set per-actuator fault bits (`stale`, `bus_error`, `offline`,
`rejected`) and never allocate, lock, sleep or retry. The write mask is kept
per field (`Written { q, qd, tau }`), so an adapter sends only the field its
actuator's mode takes.

Adapters live in `kmr_adapters`. An adapter's configuration (buses,
actuators, the serial id of each port) is Rust consts checked at compile time. The control
engineer never sees it: `kmr_api` exposes no adapter, bus or port type, and
its one hardware knob is the read cadence, `set_read_every(ticks)`.
`kmr_api::Robot::run()` hands `kmr_adapters::Hardware` to the core: the
Dynamixel adapter with the `dynamixel` feature, else `NoHardware`, which reads
and writes nothing and warns once at `open`. The vendor SDK sits behind a
private `Backend` trait, so the adapter is tested on a scripted backend and
`dxl_rs_v2` can replace `dxl_rs` later.

Not wired yet: the runtime makes none of these calls, `run_on` and
`set_read_every` are not in the core, and the `dynamixel` arm of `Hardware`
waits for the `dxl_rs` dependency. The design and its departures from the SRS
are argued in [decision 0002](decisions/0002-hal-contract.md).

## Real-time constraints

The control path targets ~1 kHz and must be deterministic:

- no heap allocation (`Box`, `Vec`, `String`, `Rc`, `Arc`, `HashMap` are banned
  by clippy in `kmr_core`);
- no `unwrap()` in the engine;
- fixed sizes known at compile time (joint count, history depth); whether they are
  constants or generic parameters is argued in
  [decision 0001](decisions/0001-joint-count-and-history-depth.md).

## Not a full ECS — and why that stays open

Users will want to attach their own data (e.g. for ML) and add entities next to
the robot. The planned direction is to extend the closed, compile-time-known
set (more HList-style structures), not to add a runtime entity registry: the
1 kHz hot path stays static.
