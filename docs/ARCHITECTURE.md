# Architecture

`kmr` is one Cargo workspace. Four crates make the framework, a fifth holds
user code:

- **`kmr_core`** (`kmr_core/`) — the engine. Owns state, schedules,
  the control loop, timing, the signal bus and the hardware contract.
- **`kmr_model`** (`kmr_model/`) — the robot model. A thin proc macro that
  emits the robot descriptor's constants at compile time; users reach it
  only through `kmr_api::robot!`.
- **`kmr_model_core`** (`kmr_model_core/`) — the library under `kmr_model`:
  reads and checks a descriptor or a preset. A build script can call it
  (kmr_ffi's build helper does), so it gets the same N as `robot!`.
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

A robot with no controller does not build:
[decision 0008](decisions/0008-empty-robot-type-state-and-run-error.md)'s
type-state `E` is `Empty` until the first `add_*`, and `run()` needs
`Filled`. The error reads "kmr: this robot has no controller", in front of
the internal type paths.

## Runtime loop

```
finalization (claims, no I/O)
pre_init → open, zero → init → ping, health, configure, read_all → torque_on → post_init
loop { read → first → each_tick → last → write → clock bookkeeping → clear desired → sleep }
hold (shutdown with set_hold only) → torque_off → close
```

**One clock.** `robot.config.time` is the only clock. The engine writes it
between two controller walks; controllers get `&Time` through each phase's
short-lived `Env`. The clock never reads the system time: the runtime injects
every instant (`arm` after the bring-up, so tick 0 is not charged with it,
then `begin_tick`, `end_tick`), through a private `Host` seam that tests
replace with a scripted host. A `dt` outside [10 µs, 1 s], zero included, is
refused at finalization (`RunError::Config`), before any I/O; never set, it
falls back to 1 ms with one warning.

**One tick**, in order:

| # | Step | Hardware |
|---|---|---|
| 1 | bus check, any signal | |
| 2 | `begin_tick(now)` | |
| 3 | push a History slot (a copy of the previous one, no fault set), `read` into it and its faults; fold in the tick flags raised from other threads (`ExternalFlags`) | the actuators due this tick |
| 4 | `First`, then an emergency-stop check | |
| 5 | `EachTick`, then an emergency-stop check | |
| 6 | `Last`, then an emergency-stop check | |
| 7 | `Batch` from this tick's write masks; `write` if not empty | one transaction per group |
| 8 | `end_tick(now)`: tick count, overrun; a late tick sets `OVERRUN` in its slot | |
| 9 | bus check, any signal | |
| 10 | clear Desired | |
| 11 | sleep to the next grid point | |

Deadlines are absolute: each one is `origin + k * dt`, with `dt` the value
given to `set_dt`. Sleeping for `dt` after each tick would make the real
period `work + dt + wake latency` and the schedule would drift. When a tick
finishes after its deadline, the loop stays on the same grid: it skips the
slots it missed and sleeps to the first grid point still ahead, so the next
tick starts on the grid and has a whole `dt` to run. It never runs a burst of
ticks to catch up, never moves the grid and never starts a tick in the middle
of a slot. A late tick is logged at `debug` and counted; one summary line
at the end gives the accumulated overrun.

**Startup writes are dropped.** Desired values written in `PreInit`, `Init`
or `PostInit` are never sent: only ticks command the hardware. One warning
names the phases that wrote.

**Exits.** From `open` on, a guard (`Powered`) ends every exit, a controller
panic included, with `torque_off` then `close` on every member.

| Exit | Tick write | Hold | Torque off, close | `run_on` returns |
|---|---|---|---|---|
| `dt` out of range | no | no | no, nothing opened | `Err(Config)` |
| claim mismatch or `check` error | no | no | no, nothing opened | `Err(Hal)` |
| emergency stop in `PreInit` | no | no | no, nothing opened | `Ok` |
| `open` .. `torque_on` error | no | no | yes, at once | `Err(Hal)` |
| any signal before `torque_on` | no | no | yes, torque never on | `Ok` |
| shutdown in a tick | yes, the tick completes | with `set_hold` only | yes, after the hold | `Ok` |
| emergency stop in a tick | no, if raised before the write | no | yes, at once | `Ok` |
| emergency stop during the hold | | cut short | yes, at once | `Ok` |
| a member touching an index it does not own (debug builds) | no | no | yes, as an emergency stop | `Ok` |

The sleep is an absolute `clock_nanosleep` on the monotonic clock (Linux,
through `rustix`), with an optional spin tail before the deadline, off by
default. Not implemented yet: the startup of threaded controllers (they are
stored, never spawned) — see the [roadmap](ROADMAP.md).

## Signals: shutdown and emergency stop

Controllers talk back to the loop through one channel: a process-global,
lock-free signal bus raised with `shutdown!("reason")` or
`emergency_stop!("reason")`. They work from anywhere — nested helpers,
closures, other threads, a raw signal handler (ctrl-c is wired to `shutdown!`).

They are macros so each call site gets its own `static` metadata: nothing
allocates and the path is async-signal-safe. Escalation is monotonic — an
emergency stop is never downgraded to a shutdown, and the first reason raised
wins.

The runtime checks the bus five times per tick (one atomic load each on the
happy path): at tick start, after `First`, after `EachTick`, after `Last`
and at tick end, and after each startup phase:

- a **shutdown** lets the current tick finish, `Last` included, then `run()`
  returns;
- an **emergency stop** ends the loop at the next check: the phases left in
  the tick are skipped, and one raised in `Last` skips the write. There is
  no hold. Controllers registered after the raising one in the same phase
  still run.
- any signal pending before `torque_on` stops the bring-up: the actuators
  are never powered.

Both are logged once, at exit, with their reason and, in debug builds, their
call site: `info` for a shutdown, `error` for an emergency stop. `run()`
returns `Ok(())` in both cases. Ctrl-C, SIGTERM and SIGHUP raise a shutdown;
if the process already has a handler of its own, a warning says they will
not stop the loop.

## State

`RobotState` holds two parts, never reachable as fields:

- **Desired**, write-only for controllers: this tick's commanded `Q`, `Qd`,
  `Tau`, `Kp`, `Kd` and one written mask per field. The write sends only
  what was written; Desired is cleared after it, so nothing is assumed to
  persist from one tick to the next.
- **History**, read-only for controllers: a ring of `HISTORY_DEPTH` samples,
  newest first. Each tick pushes a slot that starts as a copy of the
  previous one; the adapter overwrites what it reads, so an actuator not
  read this tick (cadence, or a failed read) keeps its last value.
  `prev_*(0)` is this tick's sample. A slot from before an actuator's first
  read is `StateError::NoSample`, never zeros: on a run without hardware,
  every History read is. `age(i)` counts the ticks since actuator `i` was
  last read (`None` before its first read), `sample_age()` is the oldest,
  and `cadence(i)` is the read cadence the hardware table set.

The field set is sealed: `Q`, `Qd`, `Tau`, `Kp`, `Kd`, routed to their
storage by the private `Slot` trait. The shutdown hold target is the latest
sample overlaid with this tick's writes.

The actuator count is a const generic N set by `robot!`
([decision 0007](decisions/0007-actuator-count-const-generic.md)): `RobotState<N>`,
`Robot<N, CS, E>`, `Hal<N>`, `Mask<N>`, one word of mask, `MASK_WORDS = 1`, so
N runs from 1 to 64.

**Faults and tick flags.** Each History slot carries, next to its sample, the
tick's per-actuator `Faults` (`stale`, `bus_error`, `offline`, `rejected` from
the adapter, `joint_limit` and `envelope` from the limit stage) and its
`TickFlags` (`OVERRUN`, `SENSOR_MISSING`, `REMOTE_TIMEOUT`, `UNAUTHORIZED`).
A new slot starts with none set. `faults(depth)` and `tick_flags(depth)` read
them; controllers cannot set or clear one. `read` faults show at depth 0 from
`First` on; `write` and limit faults, and `OVERRUN`, at depth 1 on the next
tick. The home state is `StateError::Undecided` (core T031).

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
preset's model half ships in `kmr_model_core/presets/`, its hardware half in
`kmr_adapters::presets`. `kmr_model` and `kmr_model_core` use the heap at
build time, which their `clippy.toml` allows.

`robot!` calls `model!` through the hidden `kmr_api::__model!`: a preset by
name, a descriptor by path, the `hardware = ..` argument split off first.
The front door adds the `State` and `Robot` aliases, bound to the model's N,
and one check: N ≤ 64, under `cargo check` at the `robot!` line (the core's
`Mask::<N>::CHECK` is the backstop under `cargo build`). The kind bit sets `model::PRISMATIC` and `CONTINUOUS` wait for the
core's public mask width.

## Hardware layer

The engine drives hardware through one contract, the `Hal` trait in
`kmr_core/src/hal.rs`. It knows actuator indices, bit masks, opaque
transaction groups and a read cadence in ticks. Buses, series, protocols,
modes and raw units stay in the adapter; values cross as `Q`, `Qd`, `Tau`,
`Kp`, `Kd` in SI units. `Hal` is public with a `#[doc(hidden)]` seal: adapter
crates implement it, `controllers` cannot, since it does not depend on
`kmr_core`.

The robot's hardware is a list of members, one per brand, as an `HList`
(`kmr_core/src/hal/set.rs`): `HCons { head: dynamixel, tail: HCons { .. } }`.
Each member's `check` returns a `Claim`: the engine indices it owns, its own
groups, read cadence and costs. Claims that do not cover every actuator
exactly once fail before any I/O; after that each member is called with its
own indices only. Dispatch is static: `run_on(drivers)` runs the robot on a
list, and `run()` is `run_on` of `NoHardware` alone.

| Moment | Engine calls, on every member |
|---|---|
| `run()` finalization, no I/O | `check` → `Claim`: owned indices, groups, read cadence, costs |
| Init, before user controllers | `open`, `zero` |
| PostInit, before user controllers | `ping`, `health`, `configure`, `read_all`, `torque_on` |
| Each tick, before `First` | `read` into a copy of the previous sample |
| Each tick, after `Last` | `write` of the written actuators in group order, then Desired is cleared |
| Shutdown | `hold` (only after `set_hold`), `torque_off`, `close` |
| Emergency stop | `torque_off`, `close` |

Bring-up calls return the member's error and `run()` returns it. Once `open`
was called, every exit, a controller panic included, ends with `torque_off`
then `close`. `read` and `write` never fail: they set per-actuator fault bits
(`stale`, `bus_error`, `offline`, `rejected`) and never allocate, lock, sleep
or retry. The write mask is kept per field (`Written { q, qd, tau, kp, kd }`),
so a member sends only the field its actuator's mode takes; a Dynamixel
member ignores `Kp` and `Kd`.

Adapters live in `kmr_adapters`. A robot's Dynamixel hardware is a
`dynamixel::Table<NB, N>` const: buses by label, baud and read cadence; per
actuator its bus, firmware id, model, mode, torque choice (powered or
read-only) and zero. `Table::check` rejects a bad table in const evaluation,
naming the actuator or the bus, so it does not build. `const fn` overrides
(`baud`, `read_every`, `id`, `mode`, `read_only`, `zero`, and group forms)
change a preset's defaults from `main.rs`. A preset such as
`presets::XmBench` is the hardware half of a KM-RoBoTa robot (the model half
is `kmr_model`'s): its `HARDWARE` table and the line time it declares,
`READ_COST` and `WRITE_COST`. `Table::in_order` or `Table::at` places the
table at the robot's engine indices; `robot!` names the robot and its
hardware.

Ports are not consts. The user names each one with a bus plugin,
`kmr_api::Bus::by_id(serial)` or `by_dev(path)`, with an optional `.baud(..)`
and `.bus("label")`. `Dynamixel::bind` matches the plugins to the table's
buses by label before any I/O; `open` resolves each port, checks it is a
character device the user can read and write, refuses two buses on one
device, flushes it and sets the 1 ms USB latency timer. With no bus plugin,
`kmr_api::Robot::run()` runs `NoHardware`, which reads and writes nothing
and warns once at `open`.

Torque off reaches every actuator of an opened bus, read-only ones and ones
an earlier run left powered included, through the backend's
`force_torque_off`; a read-only actuator found powered is switched off in
PostInit. The zero is a software offset: 0 rad sits at the model's centre
or at the raw position the table names. The vendor SDK sits behind a private
`Backend` trait, so the adapter is tested on a strict scripted backend that
refuses what `dxl_rs` refuses, and `dxl_rs_v2` can replace `dxl_rs` later.

Still open: Dynamixel velocity and current modes (`dxl_rs` cannot write the
operating mode register, SRS section 8), one 10-byte sync read per group
instead of three, `dxl_rs` as a pinned git dependency with a cross build in
CI, and N as a const generic of the core. The contract and its departures
from the SRS are argued in [decision 0002](decisions/0002-hal-contract.md).

## Real-time constraints

The control path targets ~1 kHz and must be deterministic:

- no heap allocation (`Box`, `Vec`, `String`, `Rc`, `Arc`, `HashMap` are banned
  by clippy in `kmr_core`);
- no `unwrap()` in the engine;
- fixed sizes known at compile time: the history depth is a constant
  ([decision 0001](decisions/0001-joint-count-and-history-depth.md)), the
  actuator count becomes a const generic set by `robot!`
  ([decision 0007](decisions/0007-actuator-count-const-generic.md)).

## Not a full ECS — and why that stays open

Users will want to attach their own data (e.g. for ML) and add entities next to
the robot. The planned direction is to extend the closed, compile-time-known
set (more HList-style structures), not to add a runtime entity registry: the
1 kHz hot path stays static.
