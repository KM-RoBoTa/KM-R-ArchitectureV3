# Feature Specification: kmr_api, the public user API

**Feature Branch**: none (spec directory fixed: `specs/004-kmr-api`, no branch created)

**Created**: 2026-10-27

**Revised**: 2026-10-28, against the SRS as patched on 2026-10-27

**Status**: Draft

**Input**: User description: "Re-specify the kmr_api module of kmr v3 from
docs/srs/modules/kmr_api.md, against the SRS as patched on 2026-10-27."

**Authority**: `docs/srs/SRS.md` (generated view `docs/srs/modules/kmr_api.md`). Module section:
"Covers: FR-6.1 `robot!` macro, FR-6.3 port binding (bus plugins), FR-7, FR-8.4, FR-13.2,
FR-13.3, FR-13.5; NFR-4 ergonomics, NFR-12. Re-exports the kmr_core value types unchanged, wraps
the handles thinly. NFR-10 does not apply. Delivery steps 5, 6, 7."

## Scope

`kmr_api` is the only crate user code imports (SRS section 3 "API"). It is a convenience layer
over `kmr_core`: it re-exports the value types and wraps the handles thinly. It is not a DTO
layer.

In scope:

- `robot!`: the front door that names the robot in `main.rs` and emits `N`, `State`, `Robot`.
- The builder: bus plugins, `dt`, user settings, controller registrations, `run()`.
- The thin wrappers `State`, `Time`, `Sensors`, group views, aliases and unit conversions.
- The Zenoh gateway behind an opt-in Cargo feature (delivery step 5).
- Fallback tooling and envelopes (delivery step 6).
- Communication security (delivery step 7).
- Payload registration through the builder.

Out of scope (owned elsewhere, or open in the SRS):

- The loop, the clock, the signal bus, the HAL contract, the Host seam: `kmr_core`.
- Descriptor parsing, N, actuator order, limits, links: `kmr_model`.
- Presets' hardware tables, adapter internals, per-mode hold: `kmr_adapters`.
- Threaded controller spawn and hand-off (SRS FR-1.2, section 8).
- The controller label of SRS FR-1.7 (section 8).
- The return value of `run()` after an emergency stop (section 8).
- The `Sensors` surface (SRS FR-5.7, section 8).
- The home state source (SRS FR-2.7, section 8).
- The multi-robot descriptor (SRS FR-6.5, section 8).
- The joint-limit default policy (SRS FR-4.5, section 8).
- The sender-timestamp check and the hold on a running loop (SRS FR-13.3, deferred 2026-10-27).
- Center-of-mass recomputation (SRS FR-8.3, roadmap).
- `no_std` users: `kmr_api` is `std` (SRS section 4). A `no_std` user path is out of this
  version (recommended, pending human confirmation). See SRS issues.
- Roadmap: dynamic scheduler, per-controller rate, plugins (FR-11), CLI, visualization.

Delivery order: SRS section 8 says nothing below a line starts before the line above moves a
motor. Steps 5, 6 and 7 of this module wait for steps 1 to 4. `robot!` and the bus plugins
serve steps 1 and 2.

## Clarifications

### Session 2026-10-28 (coordination)

- Cross-module coordination fixes, so each fact has one owner. FR-026: the "no bus plugin:
  simulating" warning is kmr_core FR-042's; `kmr_api` keeps only the "not driven yet" message,
  until delivery step 1. FR-020: torque choice is a hardware-table field; `kmr_api` gives the
  `main.rs` override syntax. FR-042: cadence is not a builder knob. FR-049: fallback is a plain
  `Fn` wrapper, no core dispatch change. FR-041: adapter reachable before `run()` and handed back
  after it (kmr_core FR-050), never live in a tick. FR-044: `kmr_api` owns the two Zenoh slots,
  outbound after the write. FR-002, FR-007: `robot!` lists all it emits, wires kinds and limits
  as data, owns the hardware count check (array-type check, static const messages) and the
  N > 64 const assert. FR-031: per-actuator age. FR-069, SC-001: the example may change with the
  API, it must build and run; `try_lock` in a tick noted for human review. US6-AS3 cites FR-059.
  FR-024 / Q2: bus label with `.bus("label")`; `no_std` user path out of this version (both
  recommended, pending human confirmation). Needs list fixed to match.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Name the robot and write controllers from one import (Priority: P1)

A control engineer names the robot once in `main.rs` with `robot!`, a preset or a descriptor.
They write plain functions on `&mut State`, register them on phases and call `run()`. They read
History, write Desired, use groups and units, and raise signals. They import only `kmr_api`.

**Why this priority**: it is the product. Every other story needs it.

**Independent Test**: build `kmr_api/examples/complete_api_example.rs` and
`controllers/src/main.rs`. Run a robot with no bus plugin and a controller that calls
`shutdown!` on its first tick; `run()` returns `Ok` after one tick.

**Acceptance Scenarios**:

1. **Given** `robot!(XmBench)` in `main.rs`, **When** the crate builds, **Then** `N`, `State`
   and `Robot` exist in the crate and `N` is the preset's actuator count.
2. **Given** `robot!(model = "arm.urdf")`, **When** the descriptor has a fixed joint, **Then**
   `N` counts actuators only.
3. **Given** `robot!` naming an unknown preset or a bad descriptor, **When** the crate builds,
   **Then** the build fails with a message that starts with `kmr:` and names the cause.
4. **Given** a function `fn(&Time, &mut State, &Sensors)`, **When** it is registered on
   `EachTick` and `run()` is called, **Then** it runs once per tick, in declaration order.
5. **Given** a context value `T` and `fn(&mut T, &Time, &mut State, &Sensors)`, **When** it is
   registered with `add_controller_with`, **Then** every call gets that same value, moved in at
   registration, as an exclusive `&mut T`.
6. **Given** a group `[3, 4, 5]`, **When** a controller sets all `Q` of that group, **Then**
   only actuators 3, 4 and 5 receive a new command this tick.
7. **Given** `Q::from(Deg(180.0))`, **When** it is read back as `f32`, **Then** it is π within
   f32 precision. **Given** `Q::from(M(1.0))`, **Then** it is 1000 (mm).
8. **Given** an index out of range, **When** a setter by index is called, **Then** it returns
   an error and nothing is written.
9. **Given** `dt` set to 5 µs, **When** `run()` is called, **Then** it returns an error naming
   the valid range, before any hardware I/O.
10. **Given** a builder with no controller, **When** the user calls `run()`, **Then** it does not
    compile (SRS NFR-4).

---

### User Story 2 - Plug the robot into this machine (Priority: P1)

The control engineer binds the robot's buses to this machine's ports in the builder, Bevy-style:
`.add_plugins(Bus::by_id("FT8ISZ6H"))`, or `Bus::by_dev("/dev/ttyACM0")` when no serial id is
available. The preset gives brand, protocol, baud and ids; `.baud(..)` overrides the baud. With
no bus plugin, `run()` simulates and the core says so once.

**Why this priority**: delivery step 1 moves a motor only if the user can say where it is
plugged in. v2's per-machine config files were a pain point.

**Independent Test**: a binary with one `Bus::by_id` plugin and a controller cross-compiles for
aarch64 with no robot plugged in. On the bench, a wrong serial id makes `run()` return a
bring-up error that names it, and nothing is powered.

**Acceptance Scenarios**:

1. **Given** no bus plugin, **When** `run()` is called, **Then** it simulates and the one
   "no bus plugin: simulating" warning of kmr_core FR-042 appears exactly once.
2. **Given** two plugins that bind the same port, **When** `run()` is called, **Then** it returns
   an error naming the port, before any hardware I/O.
3. **Given** `Bus::by_id("FT8ISZ6H").baud(3_000_000)`, **When** bring-up opens the bus, **Then**
   only the baud differs from the preset.
4. **Given** a serial id not present on this machine, **When** `run()` reaches bring-up,
   **Then** it returns an error naming that serial id, and nothing was powered.
5. **Given** an actuator set to torque off in the `main.rs` hardware-table override, **When**
   bring-up ends, **Then** it is unpowered and its state is still read (SRS FR-5.8).

---

### User Story 3 - Read time and faults inside a controller (Priority: P2)

A controller reads elapsed time, `dt`, the tick index and overruns. It reads the per-actuator
age, the age of the current sample, the per-actuator stale flag and the typed fault flags.

**Why this priority**: delivery step 4. Without it a controller cannot react to a fault, and the
responsibility split of SRS FR-13.1 fails.

**Independent Test**: on a scripted host and adapter, fail the read of actuator 2. The next
controller call sees the stale flag on actuator 2 only.

**Acceptance Scenarios**:

1. **Given** a running loop, **When** a controller reads `elapsed()` and `dt()`, **Then** both
   are durations, never `f32` seconds.
2. **Given** a tick that overran, **When** the next controller reads the overrun values,
   **Then** `overrun_count()` grew by one and `last_overrun()` is the lateness.
3. **Given** a tick without a hardware read, **When** a controller reads the sample age,
   **Then** it is one more than on the tick before.

---

### User Story 4 - Talk to the robot over Zenoh (Priority: P2)

An integrator turns on the Zenoh feature. A peer subscribes to the latest sample and publishes
Desired values. A network controller fetches state and sends Desired; the loop stays on the
wired host.

**Why this priority**: delivery step 5 (Should).

**Independent Test**: a local peer receives samples at 1 kHz; every missed publish is counted. A
Desired it sends is applied once, then dropped. The tick shows no allocation and no lock from
this module. A build without the feature compiles no Zenoh code.

**Acceptance Scenarios**:

1. **Given** a running robot with the feature on, **When** a peer subscribes, **Then** it gets
   History depth 0 with its fault flags every tick, and the gateway's ring on request.
2. **Given** a Desired in the slot when tick k takes it, **When** tick k runs, **Then** it is
   applied at the end of tick k and not in tick k+1.
3. **Given** no new Desired, **When** a tick runs, **Then** the remote side commands nothing.
4. **Given** the feature off, **When** the crate builds, **Then** no Zenoh dependency is
   compiled.

---

### User Story 5 - Fallbacks and envelopes (Priority: P3)

A control engineer attaches a fallback controller to a registration, triggered by a predicate
over fault flags. They declare per-actuator envelopes for position, velocity and torque,
directly or through a group slice.

**Why this priority**: delivery step 6 (Should).

**Independent Test**: declare an envelope on actuator 0. A controller writes a value outside it.
The adapter never receives an out-of-envelope value, and a fault flag is raised.

**Acceptance Scenarios**:

1. **Given** a fallback on "stale on any actuator of group A", **When** a read fails on group A,
   **Then** the fallback runs in place of that controller for that tick.
2. **Given** an envelope declared on a group slice, **When** it is declared, **Then** every
   actuator of the slice gets it.

---

### User Story 6 - Secure messaging (Priority: P3)

Every network path is authenticated and encrypted. Only allowed identities may publish Desired
or read History. Unauthorized and out-of-order messages are dropped and counted. A silent remote
source raises a fault, and the user's fallback acts on it.

**Why this priority**: delivery step 7 (Could). Highest risk once a network path exists.

**Independent Test**: a peer with an unknown identity sends 1000 Desired. None is applied; the
counter reads 1000; the unauthorized-command flag is set.

**Acceptance Scenarios**:

1. **Given** a Desired with a sequence number at or below the last accepted one, **When** it
   arrives, **Then** it is dropped and counted.
2. **Given** a remote source silent beyond its timeout, **When** the timeout expires, **Then**
   the remote-timeout flag is raised and the fallback attached to it runs.
3. **Given** a remote source with no fallback for its timeout, **When** `run()` is called,
   **Then** it is rejected before any hardware I/O (FR-059).

---

### User Story 7 - Register a payload (Priority: P3)

A user registers a medium or heavy payload through the builder, with its mass and mount point,
so the model can recompute the center of mass later. A light payload is not registered.

**Why this priority**: center-of-mass recomputation is roadmap (SRS FR-8.3).

**Independent Test**: `register_payload` accepts a medium payload with mass and mount link; a
mount link the model does not know makes `run()` return an error naming it.

**Acceptance Scenarios**:

1. **Given** a payload, **When** it is registered, **Then** it goes through the builder, never
   through an `Init` controller.
2. **Given** a mount point that names no link of the model, **When** `run()` is called,
   **Then** it returns an error naming the link, before any hardware I/O.

---

### Edge Cases

- A group with a duplicate index writes the same actuator twice: the last value wins.
- A whole-array set and a set by index on the same actuator in one tick: the last write wins.
- A Zenoh Desired and a local controller write the same actuator in one tick: see Assumptions.
- Two Desired arrive within one tick: the slot holds the latest only; the overwritten one is
  counted as superseded.
- A history depth beyond `HISTORY_DEPTH`: an error, never a panic.
- A read before the first sample: whole-array reads return `None` (a deliberate wrapper over
  the core's error).
- `dt` unset: 1 ms with a warning. `dt` of exactly 10 µs or 1 s: accepted. `dt` of zero: an
  error, not "unset".
- `Bus::by_id` and `Bus::by_dev` naming the same physical device: not visible before bring-up;
  bring-up returns an error naming both.
- A degree value written to a prismatic actuator: the type does not catch it (SRS issue 4).
- The example's `Init` controllers reach `initial_state()` / `go_home()`: they hit the SRS
  FR-2.7 marker. This happens after `Init`'s engine work; with no bus plugin nothing is powered.
- A user controller panics with torque on: the SRS covers it in NFR-2 (an abort leaves torque as
  it was). The API never hides the panic.
- A context shared by a threaded and an inline controller through `Arc<Mutex<T>>`: the SRS
  forbids a lock the loop must take (section 3 "Context"). The example's `try_lock` waits on
  nothing, but it moves to the section 8 hand-off once that exists.

## Requirements *(mandatory)*

### Functional Requirements

**Robot naming (`robot!`)**

- **FR-001** (SRS FR-6.1): `kmr_api` MUST offer `robot!`, called once at module level in
  `main.rs`, in two forms: a KM-RoBoTa preset (`robot!(XmBench)`) or a descriptor path
  (`robot!(model = "arm.urdf")`, relative to the crate's `Cargo.toml`).
- **FR-002** (SRS FR-6.1, section 4 Assumptions): `robot!` MUST emit, in the user's crate: `N`;
  the `State` and `Robot` aliases, which carry N so controller signatures stay `&mut State`
  (`Robot` also carries the adapter type, FR-005); the model facts `model::*` (names, limits,
  kind bit sets, links; kmr_model data-model.md); and the `actuator!` name lookup. It MUST pass
  the actuator kinds (`model::PRISMATIC`, `model::CONTINUOUS`) to the hardware table's checks
  (kmr_adapters FR-021), and the model's limits (kmr_model FR-012) to the core's limit stage
  (kmr_core FR-037) as per-actuator data. Both are data only: no policy is chosen here.
- **FR-003** (SRS FR-6.1): `robot!` MUST be the only door to `kmr_model`. Users never name
  `kmr_model`.
- **FR-004** (SRS FR-6.1, section 4 Assumptions): Once `kmr_core` takes N as a const generic,
  `State`, `Robot` and the group view MUST take it too, and the aliases bind it. Until then, an N
  that differs from the core's count MUST be a compile error.
- **FR-005** (SRS FR-6.1, FR-5.1): With a preset, the hardware data (brand, protocol, baud,
  firmware ids, modes) MUST come from the preset, and the adapter MUST follow from the robot that
  `robot!` names, not from a separate switch the user keeps in sync.
- **FR-006** (SRS FR-6.1): Without a preset, the user MUST declare the hardware data in
  `main.rs`. [NEEDS CLARIFICATION: in what form? See Q1.]
- **FR-007** (SRS FR-6.4, FR-6.2): A bad descriptor, an unknown preset, or a hardware table
  whose count is not N MUST be a compile error. `robot!` emits no group and no accessor. `robot!`
  owns the hardware count check (kmr_model FR-023 and kmr_adapters FR-024 cite it). It is an
  array-type check in the expansion, `const _: &[Actuator; N] = &TABLE;`, so rustc's E0308
  ("expected an array with a size of 4, found one with a size of 3") names both counts. Any
  message from const evaluation cannot use `format!` (E0015); a message built byte by byte in a
  const fn can still name a bad index (proved, rustc 1.97.1). The N > 64 const assert of kmr_core FR-012 also sits in the
  expansion, so it fires under `cargo check`.
- **FR-008** (SRS FR-6.1): No per-machine config file and no `build.rs` in the user's crate.

**Builder and controllers**

- **FR-009** (SRS FR-1.1, section 7 User API): The API MUST offer one builder. A controller is a
  plain function or an `Fn` closure on a phase; no trait to implement. Captures are immutable
  config.
- **FR-010** (SRS FR-1.2, section 3 Controller): The API MUST offer three forms: inline
  `fn(&Time, &mut State, &Sensors)`; inline with context `fn(&mut T, &Time, &mut State,
  &Sensors)`; threaded `fn(&mut T, &Time)`. A threaded controller never gets `State`. Spawn and
  hand-off are out of scope; when they land, the threaded form MUST require `Send + 'static` on
  the function and the context (section 8).
- **FR-011** (SRS section 3 Context): A context is any user `T`, built by the user and moved in
  at registration. The engine owns it from then on and lends it as `&mut T`.
- **FR-012** (SRS FR-1.3, FR-1.4): The same function MAY be registered several times. Within a
  phase, controllers MUST run in declaration order. The API MUST re-export the six phases
  `PreInit`, `Init`, `PostInit`, `First`, `EachTick`, `Last`, and no other phase name.
- **FR-013** (SRS FR-1.5, FR-1.6, FR-1.7): A later per-controller rate, heap-gated mode or
  controller label MUST NOT change the FR-010 signatures, the phase set or the builder calls of
  the example.
- **FR-014** (SRS section 3 run() finalization, section 8): There is no `build()`. `run()`
  finalizes the builder, then runs the loop, and returns a `Result`. Finalization errors come
  before any hardware I/O and are returned, never panicked.
- **FR-015** (SRS FR-3.1, FR-6.4): `run()` MUST return an error, before any hardware I/O, for a
  `dt` outside [10 µs, 1 s]. An unset `dt` falls back to 1 ms with a warning.
- **FR-016** (SRS NFR-4, section 8): A builder with no controller MUST fail to compile, through a
  trait bound, not a run-time check.
- **FR-017** (SRS FR-4.1 to FR-4.4): The API MUST re-export `shutdown!` and `emergency_stop!`,
  callable from any depth and any thread.
- **FR-018** (SRS FR-4.1): The user MAY set how long the shutdown hold lasts before torque off,
  as an `Option<Duration>`. The default is `None`: torque off right after the hold command.
- **FR-019** (SRS FR-4.5): The user MAY choose the joint-limit policy per actuator, directly or
  through a group slice, among clamp, warn and abort. The default is open: the engine's default
  path carries the SRS marker; the API adds no default of its own.
- **FR-020** (SRS FR-5.8): Torque enable per actuator is a field of the hardware table, default
  on (kmr_adapters FR-023, FR-026). `kmr_api` MUST offer the `main.rs` override syntax for that
  field, by actuator index or through a group slice. It is not a builder setting. A read-only
  actuator is a normal case.
- **FR-021** (SRS FR-6.4, NFR-2): Every error the API returns MUST be human-readable. Release
  builds hide internal causes.

**Bus plugins**

- **FR-022** (SRS FR-6.3, section 7 User API): Ports MUST be bound in the builder with
  `.add_plugins(Bus::by_id(serial))`, the preferred form, or `.add_plugins(Bus::by_dev(path))`,
  documented as less reliable because `/dev/tty*` numbers change across reboots.
- **FR-023** (SRS FR-6.1, FR-6.3): A bus plugin MAY override the preset's baud with `.baud(..)`.
- **FR-024** (SRS FR-6.3, FR-6.5): With several buses, each plugin MUST bind one bus of the
  robot's topology. Each bus has a label in the hardware table; the plugin names it with
  `.bus("label")`. The label is optional for a one-bus robot. A missing, unknown or extra label is
  a `run()` error before any hardware I/O (Q2; recommended, pending human confirmation).
- **FR-025** (SRS FR-6.4): Two plugins binding the same port MUST be a `run()` error naming the
  port, before any hardware I/O.
- **FR-026** (SRS FR-6.3): With no bus plugin, `run()` MUST run on the core's no-hardware
  adapter. Its single "no bus plugin: simulating" warning is kmr_core's (kmr_core FR-042);
  `kmr_api` MUST NOT print one of its own. Until delivery step 1 lands, `run()` with plugins MUST
  also simulate, with one distinct message that the plugins are declared but the hardware is not
  driven yet. That message goes when step 1 lands.
- **FR-027** (SRS FR-6.4, FR-5.5): A port not found on this machine MUST be a bring-up error that
  names it. A binary MUST cross-compile with no robot plugged in.

**State handle and values**

- **FR-028** (SRS section 3 State, NFR-4): `kmr_api::State` MUST be a thin wrapper over
  `kmr_core::State`. The core keeps the generic methods (`current::<Q>()`); `kmr_api` adds the
  plain ones (`q()`) by forwarding, with no logic of its own. The wrapper's inner handle MUST be
  a private named field, not the tuple field `.0`.
- **FR-029** (SRS section 3 State, FR-2.1, FR-2.3): A controller gets one `State` handle. It
  reads History and writes Desired. Reads exist whole-array, by index and by depth; `prev(0)` is
  the current sample.
- **FR-030** (SRS FR-2.5): Setters MUST write Desired only, whole-array or by index. An index out
  of range returns an error and writes nothing. An actuator not written this tick receives no new
  command.
- **FR-031** (SRS FR-2.1, FR-2.6): A controller MUST be able to read, in ticks, the age of each
  actuator since its last hardware read and the age of the current sample (its oldest actuator's),
  as kmr_core FR-018 gives them, and the per-actuator stale flag.
- **FR-032** (SRS FR-2.2, section 4): `Q`, `Qd`, `Tau` (and `Kp`, `Kd` once the core adds them)
  MUST be re-exported unchanged from `kmr_core`. No API signature takes or returns a bare `f32`
  for a state value. The core's `From`/`Into` with `f32` reach the user unchanged.
- **FR-033** (SRS FR-2.2): Units are rad, rad/s, N·m for revolute actuators and mm, mm/s, N for
  prismatic ones. `kmr_api` MUST implement `From` into these units for degrees, centimetres and
  metres. It adds unit marker types, not new state value types.
- **FR-034** (SRS FR-2.4, section 3 Group): A group MUST be a user const array of actuator
  indices, contiguous or not, duplicates allowed, never registered. It has the same accessor set
  as the whole robot and costs nothing beyond the index lookups.
- **FR-035** (SRS NFR-4, NFR-3): Aliases (`position`, `velocity`, `torque`, `effort`) and other
  conveniences MUST live in `kmr_api`, never in `kmr_core`. No raw state field is reachable from
  user code; a user cannot add a field type.
- **FR-036** (SRS FR-2.7): `initial_state()` and `go_home(ramp)` MUST keep the signatures the
  example uses. Their bodies are `unimplemented!("SRS FR-2.7: home state")` until section 8
  decides.
- **FR-037** (SRS section 3 Index space): Every index in the API is an actuator index, `0..N`.
  Docs say "actuator", not "joint".

**Time, faults, sensors, HAL**

- **FR-038** (SRS FR-3.3): `Time` MUST offer, read-only: `elapsed()`, `dt()`, `last_overrun()`
  and `accumulated_overrun()` as durations, `tick()` and `overrun_count()`.
- **FR-039** (SRS FR-13.4): A controller MUST be able to read the typed per-tick fault flags of
  SRS FR-13.4. `kmr_api` raises "remote source timed out" and "unauthorized command received"
  itself.
- **FR-040** (SRS FR-5.7): The `Sensors` handle MUST be passed to every inline controller. Its
  surface is open (section 8).
- **FR-041** (SRS FR-5.9, section 8): User code MUST be able to reach the adapter through
  `kmr_api` when the API is not enough, outside the loop only (kmr_core FR-050): before `run()`,
  through an accessor on the builder, and after `run()` returns, which hands the adapter back.
  Never inside a tick.
- **FR-042** (SRS FR-5.3): The API MUST expose the adapter's declared read cadence, read-only.
  It adds no default. Cadence is not a builder knob: its one override route is the `main.rs`
  hardware table (kmr_adapters FR-026).

**Zenoh gateway** (delivery step 5)

- **FR-043** (SRS FR-7.1, section 2): Zenoh MUST live behind an opt-in `kmr_api` Cargo feature,
  off by default. A build without it compiles no Zenoh code. The gateway is turned on and
  configured through the builder.
- **FR-044** (SRS section 4 Constraints, FR-7.1): Zenoh MUST run on its own threads. It meets the
  tick only through two lock-free latest-value slots: Desired in, History depth 0 out, one atomic
  swap each. `kmr_api` owns both slots; the core gives only the two stage points (kmr_core
  FR-053). The outbound stage runs after the write, so depth 0 carries the write's flags. Inside
  the tick the API copies fixed-size data only: no heap, lock, wait, I/O or crypto.
- **FR-045** (SRS section 7 Zenoh, FR-13.4): The gateway MUST publish History depth 0 with its
  tick index, age and fault flags, once per tick it receives. A sample the gateway misses is
  counted. On request it MUST serve its own ring of the last `HISTORY_DEPTH` samples it received.
- **FR-046** (SRS section 7 Zenoh, FR-2.5): A received Desired MUST carry its values and a
  per-field written mask. It is applied on the tick that takes it from the slot, then dropped;
  it is never held to a later tick.
- **FR-047** (SRS FR-7.3, FR-7.2, section 2): The API MUST NOT stop a user from running another
  middleware inside a controller. It offers no tick-level remote driving. The core gains no
  middleware knowledge.
- **FR-048** (SRS section 7 Zenoh): Keys MUST be laid out per instance. Per-group keys wait until
  the SRS says how never-registered groups reach the gateway (SRS issue 6).

**Fallback tooling** (delivery step 6)

- **FR-049** (SRS FR-13.5): A registration MAY carry a fallback controller, triggered by a
  predicate over FR-039 flags. `kmr_api` builds it as a plain `Fn` wrapper around the user's
  controller: the wrapper checks the fault flags and calls the fallback or the controller. The
  core's dispatch does not change. Attaching one adds no heap and no `dyn` to the tick.
- **FR-050** (SRS FR-13.5, FR-13.3): The hold-policy option of FR-13.5 MUST NOT be offered until
  the running-loop hold is defined (deferred 2026-10-27). Until then that path does not build.
- **FR-051** (SRS FR-13.5, FR-2.4): The user MAY declare per-actuator envelopes for position,
  velocity and torque, directly or through a group slice. The engine checks them before every
  write.
- **FR-052** (SRS FR-13.5, FR-13.4): An envelope breach MUST NOT reach the adapter and MUST NOT
  be silent. [NEEDS CLARIFICATION: what the breach does. See Q3.]
- **FR-053** (SRS FR-13.5, FR-4): `shutdown!` and `emergency_stop!` stay the last resort.

**Communication security** (delivery step 7)

- **FR-054** (SRS FR-13.2, NFR-12): Every network path MUST be authenticated and encrypted
  (mutual TLS or QUIC, per-machine identities), default deny, debug and visualization feeds
  included.
- **FR-055** (SRS FR-13.2): Access control MUST be per key: which identities may publish Desired,
  which may read History. `kmr_api` applies the Desired rule itself, after the transport has
  authenticated the peer, so every drop is seen.
- **FR-056** (SRS FR-13.2, FR-13.4): A Desired from an unknown or unauthorized identity MUST be
  dropped, counted, and raise the unauthorized-command flag.
- **FR-057** (SRS FR-13.3): Every incoming Desired MUST carry a sequence number and a sender
  timestamp. An out-of-order one is dropped and counted. The timestamp is carried and logged,
  not checked (deferred 2026-10-27).
- **FR-058** (SRS FR-13.3, FR-13.5): Each remote source MUST have a watchdog with a user-set
  timeout. Silence beyond it raises the remote-timeout flag, which triggers the user's fallback.
- **FR-059** (SRS FR-13.3, FR-6.4, section 5 marker rule): The default hold is deferred, so there
  is no default. A remote source with no fallback for its timeout MUST be rejected before any
  hardware I/O: at compile time if the type-state allows, else as a `run()` error.
- **FR-060** (SRS FR-13.2, NFR-12): Identities and keys MUST NOT live in the source or the
  descriptor. They are provisioned per machine.
- **FR-061** (SRS NFR-12): Authentication and decryption MUST happen in `kmr_api` before a
  Desired reaches the engine. The tick does no crypto.

**Payloads**

- **FR-062** (SRS FR-8.4): A payload MUST be registered through the builder
  (`register_payload`), never inside an `Init` controller.
- **FR-063** (SRS FR-8.1, section 2): A payload MUST be medium or heavy, with its mass and mount
  point. Light payloads are not registered: the API offers no light tier.
- **FR-064** (SRS FR-8.1, FR-6.4): A mount point names a link of the model. An unknown link MUST
  be a `run()` error naming it, before any hardware I/O.
- **FR-065** (SRS FR-8.3): Center-of-mass recomputation is roadmap. That path carries
  `unimplemented!("SRS FR-8.3: center of mass")`, reached only at finalization, before any
  hardware I/O.
- **FR-066** (SRS FR-8.2): A payload's actuators and sensors are the user's integration unless a
  kmr adapter exists.

**Hot path, heap, compatibility**

- **FR-067** (SRS NFR-1, section 4 Constraints): Registration and dispatch MUST stay static: no
  `dyn`, no heap per controller. Heap is allowed in `kmr_api` outside a tick. The hot paths of
  `kmr_api` (State handle methods, Zenoh translation) allocate nothing, lock nothing, wait for
  nothing.
- **FR-068** (SRS NFR-10, module section): NFR-10's data-oriented layout rules do not apply to
  `kmr_api`.
- **FR-069** (SRS FR-8.4, section 7 Controller): `kmr_api/examples/complete_api_example.rs` MUST
  build and run after every change. It is not frozen: it changes with the API. Its starting
  point is the version the human validated on 2026-10-27 (`robot!(XmBench)`,
  `add_plugins(Bus::by_id(..))`, `try_lock`). The SRS asks two changes: its payload goes through
  `register_payload`, and the comment claiming that `add_controller` spawns a thread is removed
  (SRS FR-8.4).
  Note: the `try_lock` in an `EachTick` controller is a lock in a tick, which conflicts with SRS
  NFR-1 and Definitions "Context"; pending human review.
- **FR-070** (SRS FR-10.1, FR-10.3): The builder is Rust only and never exposed over FFI. The API
  puts no limit on which controller form calls foreign code.
- **FR-071** (SRS FR-10.4, NFR-4): The public surface MUST be small enough to learn in a day,
  measured against ROS2 by time and file count to a first moving controller.
- **FR-072** (SRS FR-10, section 3 "API"): `kmr_api` MUST offer an opt-in `ffi` Cargo feature
  that re-exports `kmr_ffi`, so controllers still import `kmr_api` only. It MUST provide the glue
  from the `State` and `Time` wrappers to the FFI types, and the N that `robot!` emits MUST reach
  those types. Off by default; a build without it compiles no FFI code.

### Key Entities

- **Robot naming**: one `robot!` call; a preset or a descriptor path; emits `N`, `State`,
  `Robot`, `model::*` and `actuator!` (FR-002).
- **Robot builder**: the chained value that collects plugins, settings, payloads and
  controllers; consumed by `run()`.
- **Bus plugin**: one port binding on this machine (serial id or device path), optional baud.
- **State handle**: the per-call wrapper over History (read) and Desired (write).
- **Group view**: a State view limited to a const array of actuator indices.
- **Time view**: read-only clock values for the current tick.
- **Unit markers**: degrees, centimetres, metres, converted into `Q` / `Qd`.
- **Fault flags**: typed per-tick bits; read by controllers, published over Zenoh.
- **Remote source**: one authenticated identity allowed to publish Desired; has a sequence, a
  timeout and a fallback.
- **Envelope**: per-actuator bounds on position, velocity, torque.
- **Fallback**: a controller attached to a registration, with a fault predicate.
- **Payload**: tier (medium or heavy), mass, mount link.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001** (SRS FR-8.4, section 7 Controller): The complete API example builds and runs after
  each delivery step, with the two SRS FR-8.4 fixes in. It may change with the API.
  `controllers/src/main.rs` (owned by `controllers`) builds against every change of this module.
- **SC-002** (SRS FR-10.4, NFR-4): With a preset, a new user gets a first moving controller with
  2 files touched (`main.rs`, `Cargo.toml`); with a descriptor, 3. Time to first motion is
  measured and recorded against the equivalent ROS2 node.
- **SC-003** (SRS NFR-1, section 4 Constraints): Over 1 000 000 ticks at 1 kHz on the NFR-1
  bench, with the gateway active, `kmr_api` code inside the tick performs 0 allocations and takes
  0 locks.
- **SC-004** (SRS FR-7.1): A build without the Zenoh feature contains 0 Zenoh crates.
- **SC-005** (SRS FR-3.1, FR-6.4, FR-8.4): 100% of bad settings (dt out of range, zero dt,
  duplicate port, unknown mount link, remote source without fallback) are rejected before any
  hardware I/O, each with a message naming the cause.
- **SC-006** (SRS FR-6.4, FR-6.1): 100% of bad `robot!` inputs fail the build: an unknown preset
  or a bad descriptor with a `kmr:` message; a hardware count not N with rustc's array-size error
  naming both counts (FR-007).
- **SC-007** (SRS FR-5.5, NFR-5): A binary with a bus plugin cross-compiles for x86-64, aarch64
  and armv7 Linux with no robot plugged in. With no plugin, the core's "no bus plugin:
  simulating" warning appears exactly once and `kmr_api` prints none. Until delivery step 1, with
  plugins, the "not driven yet" message appears exactly once.
- **SC-008** (SRS NFR-3, section 3 State): Raw field access, field forging and reaching the
  wrapper's inner handle fail to compile, pinned by compile-fail tests.
- **SC-009** (SRS FR-2.2): Unit conversions are exact within f32 precision: 180° → π rad,
  1 m → 1000 mm, 1 cm → 10 mm.
- **SC-010** (SRS FR-7.1, section 7 Zenoh): A Desired taken by tick k is applied in tick k and in
  0 later ticks. Every missed publish is counted; the miss rate is measured on the bench.
- **SC-011** (SRS FR-13.2): Of 1000 Desired from an unknown identity, 0 are applied and the
  counter reads 1000.
- **SC-012** (SRS FR-13.3): 100% of out-of-order Desired are dropped. The watchdog fires within
  its timeout plus one tick.
- **SC-013** (SRS FR-13.5): In a sweep of out-of-envelope targets, 0 reach the adapter.

## Assumptions

- `robot!` follows `specs/003-kmr-model/contracts/robot-macro.md`. Its errors start with `kmr:`.
- The adapter follows the preset (FR-005): the `Robot` alias carries the preset's adapter type,
  so `run()` needs no Cargo feature such as `dynamixel` to pick hardware.
- `kmr_api::{State, Robot, N}` stay exported until N is a const generic. After that the
  `robot!` aliases are the user path.
- A single-bus robot needs no bus label in its plugin (FR-024).
- Remote vs local writes: a received Desired is staged before the user's `First` controllers. A
  local controller that writes the same actuator later in the tick wins, as within Desired.
  Envelopes and fallbacks apply to both.
- The gateway's ring (FR-045) holds what the gateway received, each sample tagged with its tick
  index, so gaps show. It is not the core's ring.
- The gateway is a builder plugin under the feature (`add_plugins`), in the same Bevy style as
  bus plugins.
- Key and certificate files on the machine are secrets, not the per-machine config file that
  SRS FR-6.1 rules out. The builder names where they are; the secret is never in source.
- Unit conversions: `From<Deg>`, `From<Cm>`, `From<M>` into `Q`, and their per-second forms
  into `Qd`. `Tau` gets none: the SRS lists none.
- `run()` returns `Ok(())` after shutdown and after emergency stop, as today, until section 8
  decides.
- `UserError` grows variants for bad `dt`, duplicate port, unknown mount link, missing remote
  fallback and bring-up errors. A bring-up error carries the adapter's text (heap is fine
  outside the tick).
- Until delivery step 1 lands, `run()` with bus plugins simulates and says so (FR-026).

## SRS issues found

1. **Section 4 / NFR-7 / section 3 "API"**: "`kmr_api` (gateway ...; `std`)" and "the only crate
   user code imports", yet `kmr_core` has an opt-in `no_std` build. A `no_std` target has no user
   API. A `no_std` user path is out of this version (recommended, pending human confirmation);
   needs an SRS line.
2. **Section 3 State**: "nothing is hidden" vs NFR-3 "raw handles never forwarded". Resolved:
   every core capability is reachable through methods; the inner handle stays private (FR-028).
3. **FR-13.5 vs FR-13.3 deferral**: FR-13.5's "hold policy" needs the same running-loop hold
   that FR-13.3 defers. The deferral names FR-13.3 only. Resolved by the severe-marker rule:
   the option does not build (FR-050), and a remote source needs an explicit fallback (FR-059).
4. **FR-2.2**: `Q` is rad for revolute and mm for prismatic. `From<Deg>` into a prismatic
   actuator's `Q` is a silent unit error the type cannot catch.
5. **Section 7 Zenoh vs section 4**: "full ring on request" while the slot carries depth 0 only.
   Resolved: the gateway keeps its own ring (FR-045).
6. **Section 7 Zenoh**: "key layout per instance and group", but groups are never registered
   (FR-2.4). Per-group keys deferred (FR-048).
7. **FR-13.2**: "dropped and counted": if the transport's access control drops a message, kmr
   never sees it. Resolved: `kmr_api` applies the Desired rule itself (FR-055).
8. **Section 3 "run() finalization"**: "contexts allocated", while Context says "built by the
   user and moved in at registration". Read as "stored"; no allocation by `kmr_api`.
9. **FR-13.4**: the flag list has no envelope-breach flag, yet "no fault is silent" (FR-052).
10. **FR-10 vs section 3 "API"**: user controllers call `kmr_ffi`, but `kmr_api` is the only
    crate they import. Resolved: an opt-in `ffi` feature of `kmr_api` re-exports it (FR-072).
11. **FR-6.1**: silent on how hardware data is declared without a preset (Q1) and, with FR-6.3
    and FR-6.5, on how a plugin binds one of several buses (Q2, recommended answer applied,
    pending human confirmation).

## Open questions

### Q1: Hardware data without a preset (FR-006)

**Context**: SRS FR-6.1: "without a preset the user declares it in main.rs; any preset default
can be overridden there". FR-6.3: "Topology is portable and compile-time".

| Option | Answer | Implications |
| --- | --- | --- |
| A | Extra arguments to `robot!` (`robot!(model = "arm.urdf", hardware = { .. })`) | Compile time; count vs N is a compile error; extends the kmr_model contract |
| B | A second macro in `main.rs` (`hardware!{ .. }`) | Compile time; two calls the user keeps in step; `run()` must find the second one |
| C | Builder calls on the bus plugin (`Bus::by_id(..).dynamixel(..).ids([..])`) | Run time; count vs N becomes a `run()` error; mixes topology with per-machine ports |

**Recommendation**: A. One door, compile-time checks, and the builder keeps only per-machine
data, as FR-6.3 splits them.

### Q2: Which bus a plugin binds (FR-024): recommended B, pending human confirmation

**Context**: SRS FR-6.5: "each robot ... may sit on its own bus (two snakes on two buses)".
FR-6.3 binds ports by plugin, but says nothing about which bus of the topology.

| Option | Answer | Implications |
| --- | --- | --- |
| A | Plugin order matches the topology's bus order | No new call; reordering lines silently swaps buses |
| B | Each plugin names its bus (`Bus::by_id("..").bus(1)` or a label); missing or extra is a `run()` error | Explicit and checked before I/O; optional for one bus |
| C | Bring-up probes each port and matches the ids that answer | No user input; slower bring-up; ambiguous when ids repeat across buses |

**Recommendation**: B. It is checkable before any hardware I/O and cannot swap buses silently.
Applied in FR-024 with a label (`.bus("label")`), shared with kmr_adapters Q1; pending human
confirmation.

### Q3: What an envelope breach does (FR-052)

**Context**: SRS FR-13.5: "per-actuator envelopes ... the engine enforces before every write".
"Enforces" is not defined; FR-4.5's clamp / warn / abort policy is for joint limits, default
undecided.

| Option | Answer | Implications |
| --- | --- | --- |
| A | Clamp to the envelope and raise an envelope flag | Always enforced; the user's fallback reacts to the flag; needs a new FR-13.4 flag |
| B | Reuse FR-4.5's per-actuator policy | One policy model; inherits FR-4.5's undecided default, a marker on the control path |
| C | Drop that actuator's command and raise a flag | Never sends a bad value; in velocity mode the old setpoint stays latched |

**Recommendation**: A. It is the literal reading of "enforces", composes with FR-049 fallbacks
(a user who wants abort writes a fallback that calls `emergency_stop!`), and avoids FR-4.5's
open default.

## Module boundaries

**kmr_api provides**

- To `controllers`: everything a controller crate imports: `robot!`, the builder, bus plugins,
  `State` / `Time` / `Sensors`, groups, phases, fault flags, payloads, the Zenoh feature. Named
  needs: public `Time` getters `elapsed()`, `dt()`, `tick()` (FR-038); `From` for degrees, cm, m
  (FR-033); the no-hardware run, whose one warning is the core's (FR-026); readable `run()`
  errors (FR-021); the `shutdown!` re-export (FR-017); the shutdown-hold setter (FR-018).
- To `kmr_model`: the `robot!` front door and the re-export of the model's facts; the payload
  mount-link check through the model's link lookup.
- To `kmr_adapters`: the bus plugin values (port, baud, bus label per FR-024) at `run()`; the
  `main.rs` override syntax for hardware-table fields (torque choice, read cadence, and the
  rest); the actuator kinds from the model for the table checks (FR-002).
- To `kmr_ffi`: the opt-in `ffi` feature re-exporting it, glue from the `State` and `Time`
  wrappers, and N from `robot!` reaching the FFI types (FR-072). Nothing in the builder (SRS
  FR-10.1).
- To `bench`: a builder that runs empty-bodied controllers with and without the gateway; the
  public `Time` surface (FR-038); the "not driven yet" message until delivery step 1 (FR-026).
  The "no bus plugin: simulating" warning is the core's.

**kmr_api needs**

- From `kmr_core` (human): N as a const generic; generic State methods it can forward (they
  exist for `Q`/`Qd`/`Tau`); read-only `Time` getters including `tick()` and `overrun_count()`;
  the `dt` range check and its error; the no-controller trait bound; a run entry that takes the
  preset's adapter and hands it back (kmr_core FR-041, FR-050); bring-up errors that name the
  port; per-actuator age, sample age, stale flags and fault flags, with a lock-free setter for the
  two flags `kmr_api` raises (kmr_core FR-052); the envelope check before the write; the
  joint-limit policy and shutdown-hold settings; the no-hardware adapter and its one warning
  (kmr_core FR-042); the two stage points of kmr_core FR-053, inbound after the read and before
  `First`, outbound after the write. The slots, the fallback wrapper and the torque choice are
  not the core's.
- From `kmr_model`: N, actuator order, the emitted facts, the link lookup, `kmr:` compile errors.
- From `kmr_adapters`: presets with hardware tables and their adapter type; bus labels in the
  table; port lookup by serial id and by device path; baud override; readable bring-up errors;
  the read cadence each adapter declares.
- From `controllers`: none. It is the reference user.
- From `kmr_ffi`: its crate, to re-export under the `ffi` feature, with FFI types that take N.
- From `bench`: the NFR-1 reference bench for SC-003 and SC-010.

## SRS coverage

| SRS id | Spec ids | Note |
| --- | --- | --- |
| FR-1.1 | FR-009 | |
| FR-1.2 | FR-010 | spawn and hand-off open |
| FR-1.3, FR-1.4 | FR-012 | |
| FR-1.5 to FR-1.7 | FR-013 | rate, dynamic mode roadmap; label open |
| FR-2.1 | FR-029, FR-031 | |
| FR-2.2 | FR-032, FR-033, SC-009 | issue 4 |
| FR-2.3 | FR-029 | |
| FR-2.4 | FR-034, FR-051 | |
| FR-2.5 | FR-030, FR-046 | |
| FR-2.6 | FR-031 | |
| FR-2.7 | FR-036 | open, marked |
| FR-3.1 | FR-015 | |
| FR-3.3 | FR-038 | |
| FR-4.1 | FR-017, FR-018 | |
| FR-4.2 to FR-4.4 | FR-017 | |
| FR-4.5 | FR-019 | default open |
| FR-5.1 | FR-005 | |
| FR-5.3 | FR-042 | |
| FR-5.5 | FR-027 | |
| FR-5.7 | FR-040 | surface open |
| FR-5.8 | FR-020 | |
| FR-5.9 | FR-041 | before and after `run()`, core FR-050 |
| FR-6.1 | FR-001 to FR-008 | Q1 |
| FR-6.2 | FR-007 | |
| FR-6.3 | FR-022 to FR-027 | Q2 recommended, pending confirmation |
| FR-6.4 | FR-007, FR-015, FR-021, FR-025, FR-027, FR-064 | |
| FR-6.5 | FR-024 | descriptor open; Q2 |
| FR-7.1 | FR-043 to FR-046 | |
| FR-7.2, FR-7.3 | FR-047 | |
| FR-8.1 | FR-063, FR-064 | |
| FR-8.2 | FR-066 | |
| FR-8.3 | FR-065 | roadmap, marked |
| FR-8.4 | FR-062, FR-069 | |
| FR-10.1, FR-10.3 | FR-070, FR-072 | |
| FR-10.2 | FR-072 | types are kmr_ffi's; glue here |
| FR-10.4 | FR-071, SC-002 | |
| FR-13.2 | FR-054 to FR-056, FR-060 | issue 7 |
| FR-13.3 | FR-057 to FR-059 | timestamp check and hold deferred |
| FR-13.4 | FR-039, FR-045 | issue 9 |
| FR-13.5 | FR-049 to FR-053 | Q3; issue 3 |
| NFR-1 | FR-044, FR-067, SC-003 | |
| NFR-2 | FR-021 | |
| NFR-3 | FR-028, FR-035, SC-008 | issue 2 |
| NFR-4 | FR-016, FR-028, FR-035, FR-071 | |
| NFR-5 | FR-027, SC-007 | |
| NFR-10 | FR-068 | does not apply |
| NFR-12 | FR-054, FR-060, FR-061 | |
| Section 3 Context | FR-011 | issue 8 |
| Section 4 Constraints | FR-044, FR-067 | issue 1 |
| Section 7 User API, Config | FR-001, FR-014, FR-022 | |
| Section 7 Zenoh | FR-045, FR-046, FR-048 | issues 5, 6 |
