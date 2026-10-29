# Feature Specification: kmr_core, the engine

**Feature Branch**: none (spec directory fixed: `specs/002-kmr-core`, no branch created)

**Created**: 2026-10-27

**Updated**: 2026-10-28, re-specified against the SRS as patched on 2026-10-27

**Status**: Draft

**Input**: User description: "Re-specify the kmr_core module of kmr v3 from
docs/srs/modules/kmr_core.md and docs/srs/SRS.md as patched on 2026-10-27."

**Authority**: `docs/srs/SRS.md`, module `kmr_core`, delivery steps 1, 3 and 4. On any conflict
the SRS wins, and the conflict is listed under [SRS issues raised](#srs-issues-raised).

**Id convention**: `FR-0NN` and `SC-0NN` are this spec's ids. The SRS id stands next to each one,
as in "FR-032 (SRS FR-4.1)". The table at the end maps the other way.

**Who writes the code**: `kmr_core/` is human-only (`.aiignore`). Every requirement below is
human work. Requirements marked **(invariant: file)** touch the AGENTS.md no-go files. This spec
says what, never how.

## Scope

The engine owns the loop, the state, the clock, the stop signals and the hardware contract. The
user writes control math as plain functions on phases. The engine calls them.

In this version the engine must:

- run six phases in a fixed order, on a fixed time grid;
- hold a read-only History and a write-only Desired, sized by an actuator count N that the
  user's `robot!` line chooses;
- call the hardware contract at fixed points, and never let a fault go silent;
- stop cleanly on `shutdown!`, at once on `emergency_stop!`, with torque off on every exit;
- stay heap-free, `dyn`-free and lock-free on the control path, with an opt-in `no_std` build.

Out of scope, because the SRS keeps them open (section 8): threaded controllers as a whole (SRS
FR-1.2), the controller label form (SRS FR-1.7), the home state (SRS FR-2.7), the joint-limit
default (SRS FR-4.5), the `run()` result after an emergency stop, the Sensors surface (SRS FR-5.7),
the reference MCU (SRS section 4), the `HISTORY_DEPTH` owner, a hold on a running loop and the
sender-timestamp check (SRS FR-13.3, deferred).

Out of scope, because another module owns them: the `robot!` macro, bus plugins, Zenoh,
security and every ergonomic (`kmr_api`); descriptor parsing and presets' model half
(`kmr_model`); vendor adapters and presets' hardware half (`kmr_adapters`); FFI (`kmr_ffi`);
bench figures (`bench`).

## Clarifications

### Session 2026-10-28 (coordination)

- Cross-module coordination fixes, so each fact has one owner. FR-042 owns the single "no bus
  plugin: simulating" warning, and the no-hardware adapter seeds nothing (recommended, pending
  human confirmation). FR-047: torque choice lives in the hardware table, not in the engine.
  FR-045 and FR-039: cadence is declared per bus in the hardware table and overridden only in
  `main.rs`; run-time overrides carry the plugin baud only. FR-050: option A, adapter reachable
  before and after `run()`, never in a tick; FR-041 hands the adapter back. FR-053: stage points
  only, Zenoh slots are `kmr_api`'s, outbound after the write. FR-052: lock-free pending word
  folded in at the inbound stage. FR-012: the N > 64 const assert sits in the `robot!` expansion,
  the core's check is a backstop (the cap itself stays open). FR-035, FR-043: SIGTERM and SIGHUP
  cut torque; `panic = "unwind"` required. Issue 9: `no_std` user path out of this version
  (recommended, pending human confirmation).

### Session 2026-10-29 (set of drivers, D17)

- The robot's hardware is a list of drivers, not one adapter. Each member owns an index set; the
  engine fans every call out to every member and merges their claims into the layout. New
  FR-063; FR-039, FR-041, FR-043, FR-044 reworded. Placement in the core assumed, pending human.
  Design: [design-hal-set.md](design-hal-set.md).

## User Scenarios & Testing *(mandatory)*

### User Story 1 - A controller runs every tick, on time (Priority: P1)

A control engineer registers plain functions on phases and calls `run()`. Each tick the engine
reads the hardware, runs `First`, `EachTick`, `Last` in declaration order, writes what was
commanded, then sleeps to the next grid point.

**Why this priority**: This is the product. Without it nothing moves.

**Independent Test**: A scripted host and a scripted adapter record every call. Run 10 000 ticks.
The record shows the same order each tick, and every tick starts on `origin + k*dt`.

**Acceptance Scenarios**:

1. **Given** two controllers on `EachTick`, **When** a tick runs, **Then** they run in the order
   they were added, after `First` and before `Last`.
2. **Given** a tick that ends 2.5 `dt` late, **When** the loop goes on, **Then** two grid slots
   are skipped, the next tick starts on the grid with a whole `dt`, and the overrun is counted.
3. **Given** `dt` was never set, **When** `run()` starts, **Then** it uses 1 ms and warns once.
4. **Given** `dt` is 0, 5 µs or 2 s, **When** `run()` starts, **Then** it returns an error and
   the adapter saw no I/O call.

---

### User Story 2 - Stop safely (Priority: P1)

Any code, at any depth or thread, raises `shutdown!` or `emergency_stop!`. A shutdown ends the
current tick, holds the actuators, waits the hold duration if one is set, cuts torque and returns.
An emergency stop ends at the next phase boundary and cuts torque.

**Why this priority**: A stop that is late or lost moves a motor wrong.

**Independent Test**: Raise each signal at each phase boundary on a scripted host and adapter.
Check the calls the adapter saw after the raise.

**Acceptance Scenarios**:

1. **Given** a shutdown raised in `EachTick` and no hold duration, **When** the tick goes on,
   **Then** `Last` runs, the tick's write is sent, then `hold`, `torque_off`, `close`, and `run()`
   returns.
2. **Given** a hold duration of 2 s, **When** a shutdown ends the loop, **Then** `torque_off`
   comes 2 s after `hold`, not before.
3. **Given** an emergency stop raised in `First`, **When** the phase ends, **Then** `EachTick`,
   `Last` and the write do not run, and the adapter gets `torque_off` then `close`.
4. **Given** a shutdown then an emergency stop, **When** the bus is read, **Then** it reports the
   emergency stop. **Given** two shutdowns, **Then** the first reason is kept.
5. **Given** Ctrl-C during the loop, **When** the engine owns the Ctrl-C handler, **Then** it acts
   as a shutdown.
6. **Given** a controller panics after `open`, **When** the panic unwinds, **Then** the adapter
   still gets `torque_off` then `close`.

---

### User Story 3 - Read the past, command the future (Priority: P1)

A controller reads History by depth and writes Desired, in typed values with fixed units. It
cannot write History, read Desired, or add a field type.

**Why this priority**: The state contract is what makes controllers safe to write.

**Independent Test**: Unit tests on the state handle, plus compile-fail tests for every
forbidden access.

**Acceptance Scenarios**:

1. **Given** five ticks ran, **When** a controller reads depth 2, **Then** it gets the sample of
   two ticks ago, whether or not the hardware was read in between.
2. **Given** an actuator not read this tick, **When** a controller asks its age, **Then** the
   number of ticks since its last read is returned.
3. **Given** a controller wrote `Qd` for actuator 1 only, **When** the tick ends, **Then** only
   actuator 1 gets a command, it carries `Qd` only, then Desired and every mask are cleared.
4. **Given** a controller wrote `Q` in `Init`, **When** the first tick ends, **Then** no command
   from that write reaches the adapter, and bring-up warned once.
5. **Given** user code writes a field directly or implements a new field type, **When** it
   compiles, **Then** it fails.

---

### User Story 4 - The robot line sets N (Priority: P1)

An integrator names the robot once in `main.rs` with `kmr_api::robot!`. Every engine array takes
that robot's actuator count. No core edit, no config file, no user `build.rs`.

**Why this priority**: Delivery step 2 feeds step 3. A fixed N in the core blocks every real
robot.

**Independent Test**: Build two binaries from one unchanged core, one with 4 actuators and one
with 10. Both run on the no-hardware adapter.

**Acceptance Scenarios**:

1. **Given** a 10-actuator preset, **When** the binary builds, **Then** every History slot,
   Desired array and mask holds 10 actuators.
2. **Given** a descriptor with 65 actuators, **When** the binary builds, **Then** it fails with a
   message naming the SRS write-mask limit.

---

### User Story 5 - Plug a hardware adapter in (Priority: P2)

An adapter author implements the engine's hardware contract. The engine calls it at fixed
points. `kmr_api` builds the adapter from the preset and the bus plugins and hands it to the
engine.

**Why this priority**: Delivery step 1. The adapter waits for these call points.

**Independent Test**: Run with a scripted adapter that records calls and can fail any bring-up
step. Compare the record with the call table of FR-039.

**Acceptance Scenarios**:

1. **Given** `ping` fails in PostInit, **When** bring-up goes on, **Then** the user's PostInit
   controllers do not run, the adapter gets `torque_off` then `close`, and `run()` returns the
   adapter's error.
2. **Given** no bus plugin, **When** `run()` starts, **Then** the loop runs on the no-hardware
   adapter, which warns once.
3. **Given** a read fails for actuator 2, **When** the tick goes on, **Then** actuator 2 keeps its
   last good value and its stale flag is set in that tick's slot.

---

### User Story 6 - See every fault (Priority: P2)

A controller reads typed fault flags for the current tick and the ticks in History.

**Why this priority**: Delivery step 4. The user writes fallbacks from these flags.

**Independent Test**: Inject each fault on a scripted adapter, or through the `kmr_api` entry for
remote faults. Check the flag in the matching slot.

**Acceptance Scenarios**:

1. **Given** an overrun on tick 7, **When** a controller reads the flags on tick 8 at depth 1,
   **Then** the overrun flag is set.
2. **Given** no fault, **When** flags are read, **Then** all are clear.

---

### User Story 7 - Build the engine for a microcontroller (Priority: P3)

An integrator turns on the `no_std` feature. The engine builds without `std` and without `alloc`
for one bare-metal target, which supplies time, sleep and polling through the Host seam.

**Why this priority**: SRS NFR-7. Needed for CI, not for the first moving motor.

**Independent Test**: CI builds the engine with the feature for the chosen target on every
change.

**Acceptance Scenarios**:

1. **Given** the feature is on, **When** the engine builds, **Then** no `std` or `alloc` item is
   linked, and the target implements the Host seam.

---

### User Story 8 - Keep the door open for a dynamic scheduler (Priority: P3)

A future heap-gated scheduler orders controllers with `.before` and `.after`. This version must
not need a breaking change to add it.

**Why this priority**: SRS FR-1.6 and FR-1.7, gate-readiness only.

**Independent Test**: Review the builder surface and the controller signatures against FR-008.

**Acceptance Scenarios**:

1. **Given** the static mode, **When** the builder is used, **Then** no surface exists that only
   the static mode could keep.

---

### Edge Cases

- A raise between `Last` and the write: a shutdown still writes, then holds; an emergency stop
  skips the write.
- An emergency stop in a startup phase before `open`: no hardware call at all.
- An emergency stop after `open` but before `torque_on`: `torque_off` then `close` anyway.
- An emergency stop during the hold wait: the wait ends, torque goes off at once.
- A History read in `PreInit` or `Init`, before the first sample: a typed error, never a panic.
- A History read deeper than the ticks run so far: a typed error, never a panic.
- An index past N in any setter, getter or mask: a typed error or ignored, never a panic.
- A group with the same index twice: the last write of the tick wins.
- A `Qd` write on an actuator in position mode: no `Q` is sent by mistake (per-field mask); the
  adapter flags it `rejected`.
- A command written to a read-only actuator (torque off by choice): not sent, flagged `rejected`.
- A second `run()` in the same process: the signal bus is reset first; the engine's earlier Ctrl-C
  handler is reused.
- A foreign Ctrl-C handler installed first: the engine warns at bring-up that Ctrl-C will not stop
  the loop.
- A joint limit or envelope breach with no policy chosen: see FR-037.

## Requirements *(mandatory)*

### Functional Requirements

**Controllers and schedule**

- **FR-001 (SRS FR-1.1)**: A controller MUST be a plain function or an `Fn` closure, never
  `FnMut`, with no trait for the user to implement. Captures are immutable config. Mutable
  per-controller state MUST go through a context.
- **FR-002 (SRS FR-1.2, Definitions "Controller")**: The engine MUST accept three forms: inline,
  inline with context, threaded with context. Threaded controllers MUST be std-only (no such
  registration under `no_std`), MUST NOT receive the State handle and MUST NOT make the loop take a
  lock. In this version they are stored and never spawned (SRS section 8, open). The argument
  order users see is `kmr_api`'s (SRS section 7). **(invariant: `controllers.rs`)**
- **FR-003 (SRS Definitions "Context")**: A context MUST accept any sized user type `T`, built by
  the user and moved in at registration. The engine owns it from then on. Inline with context
  gets an exclusive `&mut T`, one instance per registration, never shared, never locked. No
  `Default` bound; nothing is built at finalization.
- **FR-004 (SRS FR-1.3)**: The same function MUST be registrable several times. Each registration
  runs, with its own context.
- **FR-005 (SRS FR-1.4, Definitions "Phase")**: The phases MUST be `PreInit`, `Init`, `PostInit`
  (once, in that order) and `First`, `EachTick`, `Last` (each tick, in that order). Within a phase,
  controllers MUST run in declaration order. Users MUST NOT be able to add a phase.
- **FR-006 (SRS FR-1.6, NFR-1)**: Controllers MUST be stored and dispatched statically, with no
  heap and no `dyn`. Routing to a phase MUST happen once, at registration. A tick MUST NOT compare
  or filter phases. **(invariant: `schedule.rs`, `controllers.rs`)**
- **FR-007 (SRS FR-1.5)**: No phase MAY assume every controller runs every tick. The rate itself is
  roadmap.
- **FR-008 (SRS FR-1.7)**: Adding a heap-gated dynamic mode later MUST NOT change the builder's
  surface, the phase contract or the controller signatures, and the static path MUST NOT get
  slower for it. The label form stays open (SRS section 8).
- **FR-009 (SRS NFR-4, section 8 "Type-state over runtime checks")**: `run()` on a robot with no
  controller MUST be a compile error from a trait bound, not a run-time check. Internal type paths
  in that error are accepted. **(invariant: `schedule.rs`)**
- **FR-010 (SRS FR-11.3)**: The schedule and the hardware contract MUST stay behind compile-time
  interfaces (traits and generics, never trait objects), so a later plugin can implement them.

**Actuator count**

- **FR-011 (SRS FR-6.1, section 4 Assumptions, Definitions "Index space")**: Every engine array
  MUST be indexed by actuator, `0..N`, all the same length. N MUST be a const generic parameter of
  the engine types. The engine MUST NOT fix N or carry an N constant. N reaches the engine only
  through the type aliases `kmr_api::robot!` emits. **(invariant: `state/field.rs`, `schedule.rs`,
  `controllers.rs`)**
- **FR-012 (SRS Definitions "Write mask", section 5 marker rule)**: A build with N from 1 to 64
  MUST work, one machine word per mask. A build with N above 64 MUST fail to compile, with the
  message "SRS write mask: more than 64 actuators", until the SRS decides wider masks (the
  severe-question rule of SRS section 5). The check is a const assert in the `robot!` expansion,
  so it fires under `cargo check`. The core's own check is a backstop: it is post-monomorphization
  and fires under `cargo build` only.
- **FR-013 (SRS section 8 "Decision 0001")**: `HISTORY_DEPTH` MUST stay one capacity constant of
  the engine. A user-chosen depth, if the builder call returns, is a run-time value checked once
  against it. Who sets it stays open (SRS section 8).

**State**

- **FR-014 (SRS FR-2.2, NFR-3, section 8 "Kp, Kd")**: Values MUST be typed `Q`, `Qd`, `Tau`, `Kp`,
  `Kd`, defined once in the engine, `f32` inside. Each MUST convert to and from `f32` both ways and
  carry the arithmetic users need. The field set MUST be sealed against user extension and
  extensible by the engine without breaking users. **(invariant: `state/field.rs`)**
- **FR-015 (SRS FR-2.2 "Units")**: The engine MUST carry values in the decided units: revolute
  actuators in rad, rad/s, N·m; prismatic actuators in mm, mm/s, N. The engine MUST NOT convert
  units. Raw-unit conversion is the adapter's; degree, cm and m conversion is `kmr_api`'s.
- **FR-016 (SRS FR-10.2, FR-2.2)**: Each value type MUST be layout-identical to a bare `f32`, so
  `[Q; N]` crosses a C ABI without a copy. Bare `f32` crosses only there.
- **FR-017 (SRS FR-2.1, FR-2.3, Definitions "History")**: History MUST be read-only to
  controllers, hold `HISTORY_DEPTH` samples and be indexed by tick: one slot pushed each tick, a
  copy of the previous one where the hardware was not read, so `prev(k)` always means k ticks ago.
- **FR-018 (SRS FR-2.1)**: For each actuator, the age in ticks since its last hardware read MUST be
  readable by controllers. The age of a sample is the age of its oldest actuator. With the
  no-hardware adapter nothing is read, so no age exists and History reads return the typed
  "no sample" error (FR-042; recommended, pending human confirmation).
- **FR-019 (SRS FR-2.5, FR-3.4)**: Desired MUST be write-only to controllers. It MUST be applied
  once per tick, after `Last`, then cleared with every write mask. An actuator not written gets no
  command.
- **FR-020 (SRS FR-5.2 "No tick-0 seed", FR-2.5)**: Desired written in `PreInit`, `Init` or
  `PostInit` MUST NOT reach the hardware. It is cleared before the first tick, and bring-up warns
  once, naming the phase, if anything was written.
- **FR-021 (SRS Definitions "Write mask")**: Every write into Desired MUST set that actuator's bit
  in the mask of that field. There is one mask per field (`Q`, `Qd`, `Tau`, `Kp`, `Kd`), stored
  apart from the values. **(invariant: `state/field.rs`)**
- **FR-022 (SRS FR-2.6)**: A failed read MUST keep the last good value, never zeros, and set the
  actuator's stale flag.
- **FR-023 (SRS FR-2.4, FR-5.4)**: Every accessor MUST take any actuator index `0..N`, so a group
  (a user-side const array of indices, gaps and duplicates allowed) costs nothing. With duplicates,
  the last write of the tick wins. The engine MUST NOT register or generate groups, and MUST NOT
  tie a bus to a group.
- **FR-024 (SRS Definitions "State", NFR-4)**: Controllers MUST get one State handle that reads
  History and writes Desired. The engine keeps a few generic accessors (`current::<Q>()`, by-depth
  reads, indexed and whole-array writes); `kmr_api` wraps the handle thinly and adds the plain
  ones. The handle is the SRS's `kmr_core::State`; the one-sample type gives up that name, and its
  new name stays open (SRS section 8).
- **FR-025 (SRS NFR-2)**: An index past N, or a depth not yet recorded, MUST return a typed error,
  never panic.
- **FR-026 (SRS FR-2.7, section 5 marker rule)**: The home-state calls MUST stay marked
  `unimplemented!("SRS FR-2.7: home state source")` until the SRS decides. The marker sits on the
  home call only, never on a path every run takes.
- **FR-062 (SRS FR-10.2, FR-2.2)**: The engine MUST give each field of the sealed field set one
  public id, `FieldId` (`Q`, `Qd`, `Tau`; `Kp`, `Kd` when they land, FR-014). `kmr_api` MUST be able to
  match it exhaustively, with no wildcard arm, so a field added without its FFI line fails the
  build (kmr_ffi FR-014 (SRS FR-10.2)). Users cannot add an id; the field set stays sealed
  (FR-014). Only `kmr_api` names it; it is not part of the user surface, so a new id breaks no
  user code. For kmr_ffi, delivery step 8 (plan K7.1, research R22). HUMAN implementation.
  **(invariant: `state/field.rs`)**

**Timing**

- **FR-027 (SRS FR-3.1)**: `dt` MUST be in [10 µs, 1 s]. Outside it, zero included, `run()` MUST
  return an error before any hardware call. A never-set `dt` MUST fall back to 1 ms with one
  warning.
- **FR-028 (SRS FR-3.2)**: Deadlines MUST be absolute, `origin + k*dt`. After an overrun the grid
  MUST be kept: missed slots skipped, no catch-up burst, no re-anchoring.
- **FR-029 (SRS FR-3.3)**: Controllers MUST read, through `&Time`: `elapsed()` and `dt()` as
  durations, `tick()`, `overrun_count()`, `last_overrun()` and `accumulated_overrun()` as
  durations. The same readers MUST be reachable by `kmr_api`. Only the engine writes the clock,
  and only between two phase walks.
- **FR-030 (SRS FR-3.4)**: Each tick MUST run: hardware read (when the adapter reads this tick) →
  `First` → `EachTick` → `Last` → pre-write stage (FR-037) → bulked write → sleep to the next
  deadline.
- **FR-031 (SRS FR-5.10)**: The clock MUST be armed after bring-up, so tick 0 is not charged with
  bring-up time. The tick path MUST NOT sleep or retry, except the one sleep to the deadline.

**Signals and safety**

- **FR-032 (SRS FR-4.1, FR-3.4)**: On `shutdown!` the engine MUST finish the current tick, `Last`
  and the bulked write included, then send the hold command, then torque off, then close, then
  return. What hold means per mode is the adapter's; the engine hands it the tick's command and the
  latest sample.
- **FR-033 (SRS FR-4.1)**: The hold duration MUST be an `Option<Duration>`, `None` by default,
  settable through `kmr_api`. `None` cuts torque right after the hold command. `Some(d)` waits `d`
  through the Host seam first. An emergency stop raised during the wait MUST end it at once.
- **FR-034 (SRS FR-4.2)**: On `emergency_stop!` the engine MUST stop at the next phase boundary,
  startup phases included, skip the rest of the tick and the write, then torque off and close.
- **FR-035 (SRS FR-4.3)**: Escalation MUST be monotonic: an emergency stop is never downgraded,
  and the first reason wins. Ctrl-C, SIGTERM and SIGHUP, or the platform's equivalents, MUST map
  to shutdown and never be lost, unless a foreign handler was installed first; then the engine
  MUST warn at bring-up. SIGTERM and SIGHUP are added so that a service stop or a closed terminal
  still cuts torque (SRS NFR-2).
- **FR-036 (SRS FR-4.4, NFR-1)**: Both signals MUST be raisable from any depth and any thread,
  without heap or lock, async-signal-safe, with one atomic compare-and-swap. A check on the happy
  path MUST cost one atomic load. **(invariant: `signal.rs`)**
- **FR-037 (SRS FR-4.5, FR-13.5)**: Before every bulked write, the engine MUST check each written
  value against the actuator's joint limits (descriptor limits mapped to actuators by the model)
  and the user's envelopes (position, velocity, torque). The policy is per actuator, settable
  through a group slice, among clamp, warn and abort (abort = `emergency_stop!` with reason
  `JointLimit`). A breach sets the joint-limit flag. When a breach hits an actuator with no chosen
  policy, the engine MUST [NEEDS CLARIFICATION: SRS FR-4.5 asks for
  `unimplemented!("SRS FR-4.5: joint-limit default policy")` there, but NFR-2 says never panic
  with torque on. Options: (A) raise `emergency_stop!` with reason `JointLimit` until the default
  is decided; (B) reject `run()` unless every actuator with a limit or envelope has a policy; (C)
  keep the SRS marker, a panic on the first breach. Recommended: B, a `run()` error under SRS
  FR-6.4, no default chosen, no panic].
- **FR-038 (SRS section 8 open "run() after an emergency stop")**: `run()` MUST keep returning
  success for both signals until the SRS decides. The signal that ended the run is logged.

**Hardware contract**

- **FR-039 (SRS FR-5.1, FR-5.2, section 7 HAL)**: The engine MUST drive hardware only through the
  sealed contract, with static dispatch, at these points:

  | Moment | Engine calls | On error |
  |---|---|---|
  | `run()` finalization, no I/O | `check(overrides)` on every member → claims merged into the layout, coverage check (FR-063); the run-time overrides carry the bus plugins' baud only | `run()` returns it |
  | Init, before user controllers | `open`, `zero` | `torque_off`, `close`, `run()` returns it |
  | PostInit, before user controllers | `ping`, `health`, `configure`, `read_all`, `torque_on` | same |
  | Each tick, before `First` | push a copy of `prev(0)`, then `read` | faults only |
  | Each tick, after the pre-write stage | `write` of the written actuators, then clear Desired and masks | faults only |
  | Shutdown | `hold`, wait (FR-033), `torque_off`, `close` | none |
  | Emergency stop | `torque_off`, `close` | none |

  Each call goes to every member of the driver list (FR-063), in declaration order; `torque_off`
  and `close` go in reverse order, every `torque_off` before any `close`.

- **FR-040 (SRS FR-5.1, FR-7.2)**: The engine MUST NOT import a vendor SDK or a middleware. The
  contract MUST be public, sealed by a `#[doc(hidden)]` supertrait, so adapter crates implement it
  and the controllers crate cannot.
- **FR-041 (SRS FR-5.1, section 7 "User API")**: The engine MUST offer a run entry generic over the
  robot's list of drivers (one or more adapters, FR-063), take the list as a value and hand it
  back when the loop returns (FR-050). It MUST NOT name any adapter type. `kmr_api` picks and
  builds the drivers from the preset and the bus plugins.
- **FR-063 (SRS FR-5.1, FR-6.4)**: The engine MUST drive any number of adapters, of any vendor, in
  one robot: a static list of members, no `dyn`, no heap. Each member claims a set of actuator
  indices at `check`. At finalization, before any I/O, the claims MUST cover `0..N` exactly once;
  a gap or an overlap is a `run()` error naming the actuator and the members. The engine numbers
  the groups so that each belongs to one member, hands each member only its own runs, and in
  debug builds checks that a member filled only its own slots (pending human: placement A,
  design-hal-set.md §8).
- **FR-042 (SRS FR-6.3)**: The engine MUST ship a no-hardware adapter that reads and writes
  nothing. It is the adapter of a run with no bus plugin. It owns the one simulation warning,
  "no bus plugin: simulating", printed once at `open`; no other module prints one. It seeds
  nothing: History stays empty and every History read returns the typed "no sample" error
  (FR-025), never zeros (recommended, pending human confirmation).
- **FR-043 (SRS NFR-2, FR-5.6)**: Once `open` was called, every exit MUST end with `torque_off`
  then `close` on every member: bring-up error, shutdown, emergency stop, SIGTERM and SIGHUP
  (as shutdowns, FR-035), and a controller panic that unwinds. The release profile MUST keep `panic = "unwind"`,
  so the drop guard runs on a panic. Hold, torque off and close are best effort: no retry, no
  error.
- **FR-044 (SRS FR-5.3)**: Before each write, the engine MUST order the written actuators by the
  opaque transaction group (a member's bus and series; the engine numbers the groups so each
  belongs to one member, FR-063), from the write masks. The group order MUST be built once, at
  finalization; the per-tick step is one pass over N.
- **FR-045 (SRS FR-5.3)**: The read cadence is declared per bus in the hardware table
  (kmr_adapters FR-009, FR-023). The adapter's layout MUST spread it per actuator, as a whole
  number of ticks. The engine MUST expose it to controllers and MUST NOT invent or change it. Its
  one override route is the `main.rs` hardware table (kmr_adapters FR-026); the run-time
  overrides of FR-039 carry no cadence.
- **FR-046 (SRS FR-5.5, FR-6.4)**: Presence MUST be checked at bring-up only. A binary MUST build
  and finalize with no hardware plugged in. A bad port binding is a bring-up error `run()`
  returns.
- **FR-047 (SRS FR-5.8)**: Torque on MUST follow the user's per-actuator choice at bring-up,
  default on. The choice is a field of the hardware table (kmr_adapters FR-023, FR-026), which the
  adapter applies at `torque_on`; the engine holds no torque setting. A read-only actuator MUST be
  read and recorded like any other; a command written to it is not sent and is flagged `rejected`.
- **FR-048 (SRS FR-5.2)**: In each startup phase, the engine's bring-up MUST run before that
  phase's user controllers. `PreInit` touches no hardware. `read_all` seeds History before the
  `PostInit` user controllers. Sensors, payloads and battery have no contract operation yet and are
  not brought up. The out-of-place check waits for FR-026: until then it is skipped with one
  warning, not marked.
- **FR-049 (SRS FR-5.7)**: Every controller MUST receive a Sensors handle. Its surface stays open;
  no sensor read happens until a sensor contract exists.
- **FR-050 (SRS FR-5.9)**: User code MUST be able to reach the adapter directly when the API is
  not enough, outside the loop only: before `run()`, and after `run()` returns, which hands the
  adapter back (FR-041). Never inside a tick: the engine owns the adapter during `run()`. No
  safety stage is bypassed, and no lock or `dyn` is needed. `kmr_api` exposes both ends
  (kmr_api FR-041).

**Faults**

- **FR-051 (SRS FR-13.4, NFR-12, Definitions "Fault flags")**: Each History slot MUST carry typed
  fault flags. Per actuator: stale, bus error, offline, rejected, joint limit hit. Per tick:
  overrun, sensor missing, remote source timed out, unauthorized command received. Flags are
  bitsets stored apart from the values, never a state field. Controllers MUST read them by depth.
  No fault MAY be silent.
- **FR-052 (SRS FR-13.4)**: Flags raised outside the engine (remote source timed out, unauthorized
  command received) MUST be set through a hidden entry that `kmr_api` reaches and does not
  re-export. Controllers MUST NOT be able to set or clear any flag. The entry is called from
  `kmr_api`'s Zenoh threads, so it MUST only set bits in a lock-free pending word (one atomic);
  the loop folds that word into the current slot at the inbound stage (FR-053). No lock, and no
  thread other than the loop writes History.
- **FR-053 (SRS section 4 Constraints, FR-7.1, FR-7.2)**: The engine MUST offer `kmr_api` two
  stage points per tick, with static dispatch and no heap: inbound, after the read and before
  `First`, writing Desired through the State handle; outbound, after the write, reading depth 0
  and its flags, so the write's own flags are included. They name no middleware. The engine
  gives stage points only: the two lock-free latest-value slots are `kmr_api`'s (kmr_api FR-044).

**Build and real-time constraints**

- **FR-054 (SRS NFR-1, NFR-10)**: The control path MUST NOT allocate, use `dyn`, take a lock, or
  log through a sink that locks or allocates. Sparse data, if any, MUST use fixed-capacity,
  heap-free maps.
- **FR-055 (SRS NFR-2, section 5 marker rule)**: The engine MUST contain no `unwrap`, `panic` or
  unchecked indexing, except the SRS markers, which CI counts. A marker MUST NOT sit on a path
  every run takes; one left on the control path blocks a release. Errors MUST be human-readable;
  release builds hide internal causes.
- **FR-056 (SRS NFR-3)**: Everything MUST be `pub(crate)` unless `kmr_api`, an adapter crate or
  `kmr_ffi` must name it. Raw handles MUST NOT be forwarded.
- **FR-057 (SRS NFR-4)**: Aliases, wrappers and other ergonomics MUST live in `kmr_api`, never in
  the engine.
- **FR-058 (SRS NFR-7, section 4 Constraints)**: An opt-in feature MUST build the engine with no
  `std` and no `alloc`. The platform seam (Host: monotonic now, sleep, poll) MUST be public and
  sealed like the contract: a target port implements it, controller code cannot. Under the feature,
  tracing is a no-op and the platform's Ctrl-C equivalent, if any, maps to shutdown. The default
  build is `std` on Linux, with threads, std time, tracing and the Ctrl-C handler behind the seam.
- **FR-059 (SRS NFR-9)**: State MUST be stored as structure of arrays, cache-line aligned. Each
  layout choice MUST be documented and benchmarked.
- **FR-060 (SRS NFR-10)**: The engine MUST apply the data-oriented strategies of NFR-10: indexes,
  not pointers; booleans out of band (masks, flags); no padding (SoA); encodings, not
  polymorphism.
- **FR-061 (SRS NFR-11)**: Engine work per tick MUST be the least that keeps up with the I/O.
  Extra CPU work is allowed only where it buys cheaper I/O (FR-044).

### Key Entities

- **Robot builder**: holds `dt`, the hold duration, the per-actuator limit policies and the
  controller list. `run()` finalizes it, then runs the loop. There is no `build()`.
- **Phase**: one of six sealed labels; the compile-time key that picks a bucket.
- **Controller node**: one registration (inline, inline with context, threaded). Opaque to users.
- **Context**: any user value moved in at registration, owned by the engine.
- **State handle**: the one handle a controller gets. Reads History, writes Desired.
- **History**: ring of `HISTORY_DEPTH` samples, one per tick, with per-actuator age and per-slot
  fault flags.
- **Sample**: one History slot: every field for every actuator. Final type name open.
- **Desired**: write-only command buffer, cleared after each apply.
- **Write mask**: one actuator bitset per field, out of band, one word for up to 64 actuators.
- **Fault flags**: per-slot bitsets, per actuator or per tick.
- **Time**: the one clock. Engine writes it between phase walks; controllers read it.
- **Signal**: shutdown or emergency stop, with a static reason. Monotonic.
- **Hardware contract**: the sealed trait, its layout (groups, cadence, costs), overrides,
  command, sample, batch, faults, and the no-hardware adapter.
- **Driver list**: the robot's adapters as a static list; each member claims its actuator
  indices (FR-063).
- **Host**: monotonic now, sleep, poll. Real on `std`, scripted in tests, target-supplied on
  `no_std`. Public and sealed.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001 (SRS FR-3.4, FR-1.4)**: Over 10 000 scripted ticks, the recorded order is read, `First`,
  `EachTick`, `Last`, write, sleep on every tick, with controllers in declaration order. 0
  deviations.
- **SC-002 (SRS FR-3.2)**: Over 10 000 scripted ticks with injected overruns, every tick starts on
  `origin + k*dt`, and no two ticks start inside one slot.
- **SC-003 (SRS FR-4.1, FR-4.2, FR-5.6, NFR-2)**: For a raise at each phase boundary, in startup
  and in the loop, and for an injected controller panic, the adapter call record matches FR-039 in
  100% of runs. After `open`, every run ends with `torque_off` then `close`.
- **SC-004 (SRS FR-3.1, FR-5.5)**: For `dt` of 0, below 10 µs or above 1 s, `run()` returns an
  error and the adapter records 0 I/O calls, in 100% of cases.
- **SC-005 (SRS NFR-1)**: Over 10 000 ticks, the tick path performs 0 allocations and takes 0
  locks, and a signal check on the happy path is one atomic load.
- **SC-006 (SRS FR-13.4, FR-2.6)**: Each injected fault appears in the flags of the slot of the
  tick it happened, in 100% of cases. A failed read never yields zeros.
- **SC-007 (SRS NFR-7)**: CI builds the engine with the `no_std` feature for the chosen bare-metal
  target on every change, with 0 failures on main.
- **SC-008 (SRS NFR-2, NFR-3, FR-2.2)**: Compile-fail tests prove: no direct field write, no
  Desired read, no new field type, no new phase, no adapter or Host implementation from the
  controllers crate, no empty-robot `run()`. CI counts 0 `unwrap` or `panic` in the engine outside
  the SRS markers.
- **SC-009 (SRS FR-6.1)**: From one unchanged core, binaries with N = 1, 4, 10 and 64 build and
  run on the no-hardware adapter; N = 65 fails to build with the SRS wording.
- **SC-010 (SRS FR-4.1)**: With a hold duration `d`, `torque_off` follows `hold` after `d`, within
  one `dt`, in 100% of scripted shutdowns. With `None`, no wait is recorded.
- **SC-011 (SRS FR-5.2, FR-2.5)**: Writes made in startup phases produce 0 adapter commands and
  exactly 1 warning.
- **SC-012 (SRS NFR-11, NFR-8 (d))**: Engine overhead per tick is measured on the NFR-1 bench and
  recorded per release. The figure is not set here (the SRS defers it).
- **SC-013 (SRS FR-1.7)**: A review finds no builder method, phase rule or controller signature
  that a dynamic mode would have to change.

## Assumptions

- The hardware contract is the one in `kmr_core/src/hal.rs`, extended for `Kp`, `Kd` and a
  generic N. Decision 0002 stays the defence; its "hardware never through the builder" point is
  superseded by SRS FR-6.3 (bus plugins).
- Decision 0001 holds for `HISTORY_DEPTH` only. For N it is superseded by the 2026-10-27 const
  generic.
- The width limit of FR-012 follows the SRS cost line "one machine word per field for up to 64
  actuators". A wider mask is a later SRS decision.
- Age is per actuator, since the cadence is per actuator in the contract.
- `Kp` and `Kd` units follow from the decided units: N·m/rad and N·m·s/rad on revolute, N/mm and
  N·s/mm on prismatic. The SRS does not state them.
- Fault flags live as long as their History slot. A longer record (SRS NFR-12) is built outside
  the engine, from the outbound feed.
- The release profile keeps `panic = "unwind"` (required by FR-043). Under `abort` no guard runs
  and torque stays as it was (SRS NFR-2 accepts this for foreign crashes).
- The `no_std` target must have atomic compare-and-swap, or FR-036 cannot hold.
- Threaded controllers stay stored and never spawned until the SRS settles the hand-off.
- The `Sensors` handle stays an empty placeholder (SRS FR-5.7, WIP).
- A hold on a running loop does not exist in this version (SRS FR-13.3, deferred).

## Module boundaries

| Module | kmr_core provides | kmr_core needs |
|---|---|---|
| kmr_core | (this module) | (this module) |
| kmr_model | nothing directly; its `robot!` expansion names core types through `kmr_api` | nothing at build time: no dependency on `kmr_model`. N, joint limits mapped to actuators and actuator kinds reach the core as data and type arguments through `kmr_api::robot!` |
| kmr_adapters | the sealed contract (`Hal` and its seal, masks, groups, layout, overrides, written masks, command with `Kp`/`Kd`, sample, batch, runs, faults), generic over N; `NoHardware` | one member per adapter, claiming its actuator indices: raw ↔ decided units, per-actuator read cadence, (bus, series) groups, per-mode hold, per-actuator torque choice, `rejected` on unencodable or unsendable targets, at most 64 actuators |
| kmr_api | the builder generic over N (`dt`, hold duration, limit policies, the three registrations, a run entry generic over the adapter that hands it back); phases; the State handle and its generic accessors; `Q`/`Qd`/`Tau`/`Kp`/`Kd` and the field seal; `Time` readers; `shutdown!`/`emergency_stop!`; fault-flag readers; the hidden external-flag entry (a lock-free pending word); inbound and outbound stage points, no slots; `HISTORY_DEPTH`; error types | `robot!` aliases that instantiate N, with the N > 64 const assert in the expansion; the driver list built from preset and bus plugins, with run-time overrides limited to the plugin baud; no second no-hardware warning; the two Zenoh latest-value slots; fallbacks as plain `Fn` wrappers, with no core dispatch change; the SRS argument order for users; no re-export of the external-flag entry; joint limits and envelopes as per-actuator data; degree/cm/m conversions |
| controllers | nothing directly: it sees the engine only through `kmr_api` | nothing |
| kmr_ffi | value types layout-identical to `f32`; flags and time as plain integers and durations, reached through `kmr_api` | nothing |
| bench | the `Time` counters, the `dt` range check, the deadline sleep; the no-hardware adapter for engine-only runs | the NFR-1 bench to measure engine overhead, the 10 µs floor and the pre-sort cost |

## SRS issues raised

1. SRS section 8 accepts decision 0001 ("both sizes are constants"), while section 4 Assumptions
   and FR-6.1 make N "a const generic". Resolved: the 2026-10-27 text wins for N.
2. SRS NFR-10 and section 8 call kmr_model "a dependency of kmr_core". With N through
   `kmr_api::robot!` and kmr_model a proc-macro crate, the core needs no such dependency.
3. SRS Definitions "Write mask" is silent above 64 actuators. A mask sized from a generic N
   (`N.div_ceil(64)` words) is not stable Rust. Resolved with FR-012.
4. SRS section 5 marker rule vs NFR-2 "never panicking is also what keeps torque safe": a marker
   on FR-4.5, FR-2.7 or the FR-5.2 out-of-place check panics on a path runs take. FR-037 marker.
5. SRS FR-5.9 "the user may reach the HAL directly" names no path while the engine owns the
   adapter. Resolved by coordination on 2026-10-28: outside the loop only (FR-050).
6. SRS FR-13.5 "the engine enforces before every write" is in kmr_api's Covers, not kmr_core's,
   and gives no breach action. Hosted by FR-037.
7. SRS FR-13.4 lists no `rejected` flag and no way for kmr_api to raise its flags. FR-051, FR-052.
8. SRS FR-2.1 "its age in ticks" reads per sample; cadence is per actuator. FR-018.
9. SRS NFR-7 "std is confined to kmr_api" vs section 4 "std::thread, std::time ... behind it
   [Host] under the std feature". And with kmr_api std-only, a `no_std` build has no user API,
   against Definitions "API: the only crate user code imports". A `no_std` user path is out of
   this version; the engine's `no_std` build stays a CI build (FR-058) (recommended, pending
   human confirmation).
10. SRS NFR-10 "sparse data in hash maps": only fixed-capacity, heap-free maps fit the core.
11. SRS NFR-12 "every incident is recorded and replayable" cannot hold in a `HISTORY_DEPTH` ring.
12. SRS FR-3.4 "finish Last, write the hold command" vs FR-4.1 "finishes the current tick":
    resolved as write, then hold (FR-032), since FR-3.4 says "Last is the last chance to write".
13. SRS FR-1.2 threaded is "std-only" and the core is heap-free, but spawning a thread allocates.
14. SRS section 7 HAL row "torque on/off per actuator": the contract switches all at once and takes
    the per-actuator choice from the hardware table (FR-047).

## SRS coverage

| SRS id | Spec ids | Gap |
|---|---|---|
| FR-1.1 | FR-001 | |
| FR-1.2 | FR-002 | spawn, hand-off, `&Time` race: open (section 8) |
| FR-1.3 | FR-004 | |
| FR-1.4 | FR-005, SC-001 | |
| FR-1.5 | FR-007 | per-controller rate: roadmap |
| FR-1.6 | FR-006, FR-008 | dynamic mode: roadmap |
| FR-1.7 | FR-008, SC-013 | label form: open (section 8) |
| FR-2.1 | FR-017, FR-018 | |
| FR-2.2 | FR-014, FR-015, FR-016, FR-062 | `Kp`/`Kd` units: assumption |
| FR-2.3 | FR-017 | |
| FR-2.4 | FR-023 | |
| FR-2.5 | FR-019, FR-020, FR-021, SC-011 | |
| FR-2.6 | FR-022, SC-006 | |
| FR-2.7 | FR-026 | open, marker on the home call only |
| FR-3.1 | FR-027, SC-004 | 10 µs feasibility: bench |
| FR-3.2 | FR-028, SC-002 | |
| FR-3.3 | FR-029 | |
| FR-3.4 | FR-030, SC-001 | |
| FR-4.1 | FR-032, FR-033, SC-003, SC-010 | hold per mode: adapter |
| FR-4.2 | FR-034, SC-003 | |
| FR-4.3 | FR-035 | |
| FR-4.4 | FR-036, SC-005 | |
| FR-4.5 | FR-037 | default policy: clarification |
| FR-5.1 | FR-039, FR-040, FR-041, FR-063 | mixing vendors: SRS patch prepared 2026-10-29 |
| FR-5.2 | FR-039, FR-048 | sensors, payloads, battery: no contract yet; out-of-place check waits for FR-2.7 |
| FR-5.3 | FR-044, FR-045 | pre-sort cost: bench |
| FR-5.4 | FR-023 | bus stays in the adapter |
| FR-5.5 | FR-046, SC-004 | |
| FR-5.6 | FR-032, FR-043 | |
| FR-5.7 | FR-049 | surface open (section 8) |
| FR-5.8 | FR-047 | |
| FR-5.9 | FR-050 | outside the loop only |
| FR-5.10 (honoured) | FR-031 | rest is `kmr_adapters` |
| FR-6.1 (honoured) | FR-011, SC-009 | |
| FR-6.3 (honoured) | FR-041, FR-042 | |
| FR-6.4 (honoured) | FR-027, FR-046 | |
| FR-6.5 (honoured) | FR-011, FR-023 | multi-robot descriptor: open (section 8) |
| FR-7.1, FR-7.2 (honoured) | FR-040, FR-053 | |
| FR-10.2 (honoured) | FR-016, FR-062 | |
| FR-11.3 (honoured) | FR-010 | |
| FR-13.3 (honoured) | none | deferred: no running hold |
| FR-13.4 | FR-051, FR-052, FR-053, SC-006 | |
| FR-13.5 (honoured) | FR-037 | breach default: clarification |
| Definitions Context | FR-003 | |
| Definitions State | FR-024 | sample type name: open |
| Definitions Write mask | FR-012, FR-021 | above 64 actuators: SRS gap |
| Section 8 decision 0001 | FR-013 | `HISTORY_DEPTH` owner: open |
| Section 8 run() after estop | FR-038 | open |
| NFR-1 | FR-006, FR-036, FR-054, SC-005 | jitter figures: bench |
| NFR-2 | FR-025, FR-043, FR-055, SC-003, SC-008 | |
| NFR-3 | FR-014, FR-056, SC-008 | |
| NFR-4 (honoured) | FR-009, FR-024, FR-057 | |
| NFR-7 | FR-058, SC-007 | MCU target: open (section 4) |
| NFR-9 | FR-059 | |
| NFR-10 | FR-054, FR-060 | |
| NFR-11 | FR-061, SC-012 | |
| NFR-12 (honoured) | FR-051 | replay outside the engine |
