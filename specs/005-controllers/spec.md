# Feature Specification: controllers, the reference example

**Feature Branch**: none (spec directory fixed: `specs/005-controllers`; no branch created)

**Created**: 2026-10-27

**Updated**: 2026-10-28, re-specified against the SRS as patched on 2026-10-27

**Status**: Draft

**Input**: User description: "Re-specify the controllers module of kmr v3 (the user crate and
reference example, depends on kmr_api only) from docs/srs/modules/controllers.md, against the SRS
as patched on 2026-10-27."

**Authority**: `docs/srs/SRS.md`, section `## Module: controllers` (paths `controllers/**`). On any
conflict the SRS wins and the conflict is reported.

**Id convention**: `FR-0NN` and `SC-0NN` are this spec's ids. The SRS id stands next to each one,
written `SRS FR-x.y` or `SRS NFR-x`. The table at the end maps the other way.

## Scope

The SRS gives this module one job: "Covers: the reference example (FR-10.4 metric). Must compile
clean against this SRS: no `todo!()`, no lock on the hot path, no stale comments."

Section 8 names the module "`controllers/**` (user crate, reference example)". So the reference
example is `controllers/src/main.rs`. It is the file a new control engineer copies to get a first
motor moving, and the proof that a user crate needs `kmr_api` alone.

**In scope**

- `controllers/src/main.rs`: the `robot!` line, one demo controller, one builder chain.
- `controllers/Cargo.toml`: one dependency, `kmr_api`.
- A short walkthrough to measure the SRS FR-10.4 time and file count.

**Out of scope** (each one is open in the SRS or owned by another module)

- `kmr_api/examples/complete_api_example.rs`. It lives under `kmr_api/**`, so it is `kmr_api`'s.
  The SRS FR-8.4 fix it needs goes to that module.
- Threaded controllers (SRS FR-1.2, TBD as a whole).
- Home state, `initial_state()` and `go_home()` (SRS FR-2.7, open). (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.)
- Payload registration (SRS FR-8.4, `register_payload` not built).
- Joint-limit policy (SRS FR-4.5, default undecided).
- The Sensors surface (SRS FR-5.7, WIP), fault flags and fallbacks (SRS FR-13.4, FR-13.5;
  delivery steps 4 and 6).
- What `run()` returns after an emergency stop (SRS section 8, open).
- FFI delegation (SRS FR-10.3; `kmr_ffi`, delivery step 8).
- Controller labels for a later `.before(C)` (SRS FR-1.7, open).

## Clarifications

### Session 2026-10-28 (coordination)

- Cross-module coordination fixes. FR-007, SC-011 and the boundary table: the one simulation
  warning is kmr_core FR-042's, "no bus plugin: simulating", reached through `kmr_api`. What
  "simulates" means: the no-hardware adapter seeds nothing and History reads return the typed
  "no sample" error (recommended, pending human confirmation). The rebuttal of a "0 changed
  lines" rule is removed: `kmr_api` no longer freezes any example.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - First moving controller (Priority: P1)

A control engineer has a KM-RoBoTa robot wired to the host. They open the controllers crate and
read one short file. It names the robot with one `robot!` line, holds one controller and one
builder chain. They bind their port and run it. Every actuator makes a small, slow swing around
where it stood. Then the program stops by itself.

**Why this priority**: This is the module's only job (SRS module controllers, SRS FR-10.4).

**Independent Test**: Wire a preset robot, bind its port, run the example, record History. Every
actuator moves, stays inside the stated swing, and the run ends with a graceful shutdown.

**Acceptance Scenarios**:

1. **Given** a preset robot wired, powered and bound, **When** the user runs the example, **Then**
   every actuator swings around its start position within the stated amplitude.
2. **Given** the demo is running, **When** its fixed duration ends, **Then** a controller raises
   `shutdown!`, the tick finishes and the program exits cleanly (SRS FR-4.1).
3. **Given** the demo is running, **When** the user presses Ctrl-C, **Then** the program stops as
   a shutdown (SRS FR-4.3).

---

### User Story 2 - The reference stays honest (Priority: P1)

A contributor changes the API or the SRS. The reference still builds clean, holds no placeholder,
takes no lock in a tick and says nothing false.

**Why this priority**: A reference that lies or panics teaches the wrong thing. The SRS module
section makes this binding.

**Independent Test**: Run the workspace build and lint, plus a text search of `controllers/`.

**Acceptance Scenarios**:

1. **Given** any change to the workspace, **When** the build and lint run, **Then** the crate
   builds with zero warnings.
2. **Given** the crate source, **When** it is searched, **Then** it holds no `todo!`,
   `unimplemented!`, `compile_error!`, `unwrap`, `expect` or `panic!`.
3. **Given** the crate source and manifest, **When** they are searched, **Then** they name nothing
   from `kmr_core`, `kmr_model`, `kmr_adapters` or a vendor SDK.

---

### User Story 3 - Measure onboarding against ROS2 (Priority: P2)

A maintainer times a new user from a fresh checkout to a moving motor. They count the files the
user wrote. They compare with the same task done as a ROS2 node.

**Why this priority**: SRS FR-10.4 and SRS NFR-4 name this the metric. It needs a stable,
documented path to measure.

**Independent Test**: Follow the walkthrough on a clean machine. Record elapsed time, files
written and lines edited.

**Acceptance Scenarios**:

1. **Given** the walkthrough, **When** a new user follows it, **Then** every file they write, every
   line they edit and every command they run is listed in it.
2. **Given** the measured result, **When** it is compared with the ROS2 baseline, **Then** both
   time and file count are lower.

---

### User Story 4 - Builds anywhere, simulates or fails cleanly (Priority: P2)

A user builds the example on a laptop for the robot's board, with no robot attached. They run it
with no port bound. Later they run it with a port bound but the robot unplugged.

**Why this priority**: v2 could not cross-compile without the robot (SRS FR-5.5). A panic at
bring-up leaves the hardware in an unknown state.

**Independent Test**: Cross-build for each Linux target without hardware. Run once with no bus
plugin, once with a plugin and no robot.

**Acceptance Scenarios**:

1. **Given** no robot attached, **When** the user builds for x86-64, aarch64 or armv7 Linux,
   **Then** the build succeeds (SRS FR-5.5, NFR-5).
2. **Given** no bus plugin, **When** the user runs the example, **Then** `run()` simulates, warns
   once, the demo ends by itself, and nothing panics (SRS FR-6.3).
3. **Given** a bus plugin and the robot unplugged, **When** the user runs the example, **Then**
   `run()` returns a readable bring-up error, `main` returns it, and nothing panics
   (SRS FR-6.4).

### Edge Cases

- No sample yet (`q()` is `None`): the controller writes nothing that tick and tries again next
  tick. It never panics and never writes zeros (SRS FR-2.6).
- Simulation feeds no samples: the no-hardware adapter seeds nothing and History reads return
  the typed "no sample" error (kmr_core FR-042; recommended, pending human confirmation). The demo
  never captures a start position, writes nothing, and still ends on its fixed duration.
- An index or depth out of range: handled as a `Result`, never unwrapped.
- An overrun: the target is a function of elapsed time and stays inside the amplitude, so skipped
  slots never make it leave the swing (SRS FR-3.2).
- The adapter does not read every tick (SRS FR-5.3): the demo needs one sample only, the start
  position. Later targets do not depend on fresh reads.
- An actuator not in position mode: the demo writes positions only, so that actuator gets no
  command (per-field write mask, SRS Definitions). The walkthrough says so.
- A prismatic actuator: `Q` is in mm there (SRS FR-2.2). The demo amplitude is in degrees, so the
  reference uses an all-revolute preset. The walkthrough says so.
- The wired robot does not match the preset (missing id, wrong count): a bring-up error, not a
  panic (SRS FR-5.5, FR-6.4).
- End of the demo: torque goes off right after the hold, since the hold duration defaults to
  `None` (SRS FR-4.1, NFR-2). The walkthrough warns that a robot that would fall needs a hold
  duration.
- A foreign Ctrl-C handler is already installed: kmr warns at bring-up and Ctrl-C may not stop the
  loop (SRS FR-4.3). The demo still ends on its own.
- Ctrl-C during bring-up: handled by the engine, not by the example.

## Requirements *(mandatory)*

### Functional Requirements

**Boundary**

- **FR-001 (SRS module controllers; SRS section 8 module list)**: The crate MUST hold the reference
  example in `controllers/src/main.rs`: the file a new user copies to write a first moving
  controller.
- **FR-002 (SRS Definitions, API; SRS section 4 Product perspective; SRS NFR-3)**: The crate MUST
  depend on `kmr_api` only. Its source MUST name nothing from `kmr_core`, `kmr_model`,
  `kmr_adapters` or a vendor SDK.
- **FR-003 (SRS FR-6.1)**: `main.rs` MUST name the robot once, at module level, with
  `kmr_api::robot!` and a KM-RoBoTa preset. Controllers and builder MUST use the emitted `N`,
  `State` and `Robot`, so signatures stay `&mut State`.
- **FR-004 (SRS FR-6.1)**: The crate MUST hold no `build.rs`, no per-machine config file and no
  side file. With a preset it holds no descriptor file either.
- **FR-005 (SRS FR-10.1; SRS section 7 User API)**: The builder MUST be written in Rust in
  `main.rs`, as one chained expression that ends in `run()`. There is no `build()`. `main` MUST
  return the result of `run()`.
- **FR-006 (SRS FR-6.3)**: A port, when bound, MUST be bound in the builder with
  `.add_plugins(Bus::by_id(..))`. `Bus::by_dev(..)` is not used in the reference.
  [NEEDS CLARIFICATION Q1: does the shipped `main.rs` carry a bus plugin line? A: no line; it
  simulates as shipped and the walkthrough adds one `Bus::by_id` line for the user's adapter.
  B: one line with the bench adapter's serial; the bench moves with no edit, every other machine
  edits the string and gets a bring-up error until then. C: two binaries, one with and one
  without. Recommended: A. The port is per machine (SRS FR-6.3), so every user but one edits
  that line anyway, and the shipped file then runs anywhere and in CI.]
- **FR-007 (SRS FR-6.3)**: With no bus plugin the example MUST run simulated. It relies on the one
  "no bus plugin: simulating" warning of the core's no-hardware adapter (kmr_core FR-042, reached
  through `kmr_api`) and prints none of its own.

**Clean reference**

- **FR-008 (SRS module controllers "compile clean"; SRS NFR-2)**: The crate MUST build and lint
  with zero warnings in the workspace build, all targets. It MUST hold no `todo!`,
  `unimplemented!` or `compile_error!`.
- **FR-009 (SRS module controllers "no lock on the hot path"; SRS NFR-1; SRS Definitions,
  Context)**: No controller registered on `First`, `EachTick` or `Last` MAY take a lock, allocate
  on the heap, sleep, block or do I/O.
- **FR-010 (SRS module controllers "no stale comments"; SRS NFR-6)**: Every comment MUST be true
  against the current SRS and API. No comment describes a feature that is not built. No comment
  restates the code.
- **FR-011 (SRS NFR-2, as a teaching bar)**: The example MUST NOT use `unwrap`, `expect`, `panic!`
  or unchecked indexing. Every `Option` and `Result` from the State handle is handled.
- **FR-012 (SRS FR-1.2, FR-2.7, FR-8.4)**: The example MUST NOT call an API path that is
  `todo!()` or `unimplemented!`: threaded registration, `initial_state`, `go_home`,
  `register_payload`. (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.)

**Controllers**

- **FR-013 (SRS FR-1.1, FR-1.2)**: The example MUST use plain functions, inline or inline with
  context. Captured values are immutable config. Mutable per-controller data MUST go through a
  context, never a capture.
- **FR-014 (SRS Definitions, Context)**: The demo context MUST be a plain user struct sized at
  compile time, built in `main` and moved in at registration. It is never shared and never locked.
- **FR-015 (SRS FR-1.4; SRS Definitions, Phase)**: The example MUST use only the phases `PreInit`,
  `Init`, `PostInit`, `First`, `EachTick`, `Last`.
- **FR-016 (SRS FR-1.5, FR-3.3)**: Controllers MUST take time from `&Time` (`elapsed()`, `dt()`,
  `tick()`), never by counting their own calls. A later "every N ticks" option then cannot break
  them.
- **FR-017 (SRS FR-2.2)**: Every value read or written MUST be a typed `Q`, `Qd` or `Tau`. No bare
  `f32` passes through a State call. The demo amplitude is written in degrees and converted with
  the `From` that `kmr_api` provides. Revolute values are in rad, rad/s and N·m.
- **FR-018 (SRS FR-2.5, FR-3.4, FR-5.2)**: The controller MUST write Desired on every tick it
  wants a command. It MUST NOT assume a command persists to the next tick.
- **FR-019 (SRS FR-2.1, FR-2.3, FR-2.6)**: The start position MUST come from the first available
  sample, `prev(0)`, stored in the context. While no sample exists, the controller writes nothing.
- **FR-020 (SRS FR-2.4, FR-6.2)**: If the example uses a group, the group MUST be a user const
  array of actuator indices. No registered or generated group is used.
- **FR-021 (SRS FR-3.1)**: The example MUST set `dt` explicitly to 1 ms.

**Safety of the demo motion**

- **FR-022 (SRS FR-4.5 open; SRS FR-13.1)**: The demo motion MUST be bounded: a slow swing of a
  stated small amplitude around the start position. Its target MUST be a function of elapsed time
  that never leaves that amplitude. It MUST NOT ramp without bound. It does not rely on joint-limit
  enforcement, whose policy is open.
- **FR-023 (SRS FR-4.1, FR-4.4)**: After a stated fixed duration a controller MUST raise
  `shutdown!` with a reason. The example sets no hold duration; the default `None` applies.
- **FR-024 (SRS FR-4.3)**: Ctrl-C MUST stop the demo as a shutdown. The example adds no signal
  handling of its own.

**Hardware and portability**

- **FR-025 (SRS FR-5.5, NFR-5)**: The crate MUST cross-build for x86-64, aarch64 and armv7 Linux
  with no robot attached.
- **FR-026 (SRS FR-6.4, FR-5.2)**: Run with a bus plugin and without the robot, the example MUST
  end with the readable bring-up error from `run()`, and no panic.

**Onboarding metric**

- **FR-027 (SRS FR-10.4, NFR-4)**: The crate MUST ship a short walkthrough. It lists every file the
  user writes, every line they edit and every command they run, from a fresh checkout to a moving
  motor. It states the preset, the amplitude, the duration and the end-of-demo torque-off.
- **FR-028 (SRS FR-10.4)**: The walkthrough MUST name the ROS2 baseline and how it is counted.
  [NEEDS CLARIFICATION Q2: what is "the equivalent ROS2 node"? A: a minimal node made with
  `ros2 pkg create`, C++ and Python, on the current ROS2 LTS, driving the same actuators through
  the vendor's ROS2 SDK package; only files the user writes or edits count; kmr must beat both.
  B: the same, but every file in the package counts, generated ones included. C: a full
  `ros2_control` setup (URDF, controller config, hardware plugin). Recommended: A. It is the
  smallest honest baseline and matches how kmr is counted.]

**What the reference does not touch**

- **FR-029 (SRS FR-10.3, FR-7.3, FR-8.1)**: The reference MUST NOT use FFI, a middleware or a
  payload. A user controller may call C/C++, Python or Lua from any form and owns that cost; the
  reference does not show it this version. Light payloads are never registered.
- **FR-030 (SRS NFR-10 "controllers no")**: No data-oriented layout rule binds this crate. User
  contexts may take any shape; the reference stays heap-free per tick under FR-009.

### Key Entities

- **Reference example**: `controllers/src/main.rs`. One `robot!` line, one demo controller, one
  builder chain.
- **Robot line**: `kmr_api::robot!(Preset)`. It fixes `N` and emits `N`, `State` and `Robot`.
- **Bus plugin**: one builder call binding this machine's port by serial id.
- **Demo context**: the controller's own data. Holds the start position (`[Q; N]`, empty until the
  first sample) and nothing else.
- **Walkthrough**: the steps, files, edited lines and commands from a fresh checkout to a moving
  motor.
- **ROS2 baseline**: the same task done as a ROS2 node, measured the same way.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001 (SRS FR-10.4)**: A new user writes at most 2 files (`Cargo.toml`, `main.rs`) and edits
  at most 2 lines (preset name, port) to get a first moving motor. The ROS2 baseline, counted the
  same way, needs more.
- **SC-002 (SRS FR-10.4, NFR-4)**: In a timed trial, a control engineer new to kmr gets a motor
  moving faster than with the ROS2 baseline, on the same robot.
- **SC-003 (SRS module controllers)**: A text search of `controllers/` finds 0 markers (`todo!`,
  `unimplemented!`, `compile_error!`) and 0 `unwrap`, `expect` or `panic!`.
- **SC-004 (SRS module controllers)**: The workspace build and lint report 0 warnings for the
  crate on every change.
- **SC-005 (SRS module controllers; SRS NFR-1)**: Review finds 0 locks, heap allocations, sleeps,
  blocking calls or I/O in per-tick controllers.
- **SC-006 (SRS module controllers)**: A reviewer checks every comment against the SRS and API and
  finds 0 false ones.
- **SC-007 (SRS FR-6.1; SRS Definitions, API)**: The manifest has exactly 1 kmr dependency,
  `kmr_api`. The source has 0 mentions of `kmr_core`, `kmr_model`, `kmr_adapters` and 0
  `build.rs` or config files.
- **SC-008 (SRS FR-5.5, NFR-5)**: Cross-builds pass for all 3 Linux targets with no hardware
  attached.
- **SC-009 (SRS FR-4.5 open; SRS FR-13.1)**: In 10 of 10 runs on a preset robot, every actuator
  stays within the stated amplitude of its start position, as recorded in History.
- **SC-010 (SRS FR-4.1, FR-4.3)**: In 10 of 10 runs, the demo ends by itself with a graceful
  shutdown within one tick of its stated duration, and Ctrl-C stops it at any time.
- **SC-011 (SRS FR-6.3)**: In 10 of 10 runs with no bus plugin, the program prints exactly 1
  simulation warning, the core's "no bus plugin: simulating", ends by itself, exits zero and does
  not panic.
- **SC-012 (SRS FR-6.4)**: In 10 of 10 runs with a bus plugin and no robot, the program prints a
  readable bring-up error, exits non-zero and does not panic.

## Assumptions

- The reference example is `controllers/src/main.rs` (SRS section 8 module list). The complete
  tour in `kmr_api/examples/complete_api_example.rs` belongs to `kmr_api`.
- The preset is `XmBench` (four XM430 on one bus), the only preset today. It is all revolute and
  its N matches the core's N. A user with another KM-RoBoTa robot changes the preset name.
- The demo swing is small and slow: a few degrees, period of seconds, around the start position.
  Exact numbers are picked in the plan and written in the walkthrough.
- The demo runs a fixed, short duration, then raises `shutdown!`. This makes it safe when Ctrl-C
  is taken by a foreign handler.
- The preset puts actuators in position mode. The demo writes positions only.
- The start position is the first sample, stored in the context. It is not "home state"
  (SRS FR-2.7 stays open).
- The trial runs from a fresh checkout, toolchain installed, to the first motion.
- The SRS NFR-2 no-unwrap bar binds the engine. The reference holds itself to it because users
  copy it.
- The root lint config bans heap types in this crate too. The demo needs none.
- Until `kmr_api` exposes the `Time` getters and the degree conversion, the demo cannot be written
  as specified. It waits for them; it does not use a marker, since the module must hold none.

## Module boundaries

**What controllers needs**

| From | What, concretely |
|---|---|
| `kmr_api` | `robot!(Preset)` emitting `N`, `State`, `Robot` (SRS FR-6.1) |
| `kmr_api` | Builder: `new()`, `set_dt`, `add_controller`, `add_controller_with`, `add_plugins(Bus::by_id(..))`, `run()` returning a readable error (SRS section 7) |
| `kmr_core` (through `kmr_api`) | The no-hardware adapter and its one "no bus plugin: simulating" warning when no bus plugin is bound (SRS FR-6.3, kmr_core FR-042) |
| `kmr_api` | State handle: `q()`, `prev_q(0)`, `set_all_q()`, `set_q_at()`; `Q`, `Qd`, `Tau` |
| `kmr_api` | Public `Time` getters `elapsed()`, `dt()`, `tick()` (SRS FR-3.3; none are public today) |
| `kmr_api` | `From` for degrees into `Q` (SRS FR-2.2; not built today) |
| `kmr_api` | `shutdown!` re-exported; the presets reachable through `robot!` only |
| `kmr_core` (through `kmr_api`) | Loop, tick order, signal bus, Ctrl-C mapping, hold then torque off, bring-up errors returned by `run()` |
| `kmr_model` (through `robot!`) | `N` and actuator order from the preset at compile time (delivery step 2) |
| `kmr_adapters` (through the preset) | `XmBench` hardware data, position mode, read cadence, the hold operation (delivery step 1) |

**What controllers provides**

| To | What, concretely |
|---|---|
| New users | The copyable reference: one file, one robot line, one demo controller |
| `kmr_api` | A compile-time proof that a crate with only `kmr_api` can drive a robot |
| Maintainers | The FR-10.4 walkthrough and its measured time and file count |
| CI | A crate that builds clean and cross-builds without hardware on every change |
| `bench` | A known-good minimal user crate for NFR-11 compile-time tracking; not required |

`kmr_ffi`: no link this version. This module owns `controllers/src/main.rs` and changes it as this
spec requires.

## SRS coverage

| SRS id | Spec ids | Note |
|---|---|---|
| Module controllers: reference example | FR-001 | Section 8 module list names the file |
| Module controllers: compile clean, no `todo!()` | FR-008, FR-011, FR-012, SC-003, SC-004 | |
| Module controllers: no lock on the hot path | FR-009, SC-005 | |
| Module controllers: no stale comments | FR-010, SC-006 | |
| Definitions API, section 4 | FR-002, SC-007 | |
| Definitions Context | FR-013, FR-014 | Any `T`, moved in |
| FR-1.1 plain Fn, captures immutable | FR-013 | |
| FR-1.2 three forms | FR-012, FR-013 | Threaded TBD: out of scope |
| FR-1.3 same function several times | none | Shown in the complete tour, not needed here |
| FR-1.4 phases, declaration order | FR-015 | |
| FR-1.5 no every-tick assumption | FR-016 | |
| FR-1.7 controller label | none | Open (section 8) |
| FR-2.1, FR-2.3, FR-2.6 History, missing sample | FR-019 | |
| FR-2.2 typed values, units, `From` degrees | FR-017 | Prismatic in mm: edge case |
| FR-2.4, FR-6.2 groups as slices | FR-020 | |
| FR-2.5, FR-3.4, FR-5.2 shoot-and-forget | FR-018 | |
| FR-2.7 home state | FR-012 | Open: out of scope (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.) |
| FR-3.1 dt | FR-021 | |
| FR-3.2 absolute deadlines | FR-022 | Edge case: overrun |
| FR-3.3 Time surface | FR-016 | Needs `kmr_api` getters |
| FR-4.1, FR-4.4 shutdown, hold duration | FR-023, SC-010 | |
| FR-4.3 Ctrl-C | FR-024, SC-010 | |
| FR-4.5 joint limits | FR-022, SC-009 | Open: bounded demo instead of a policy |
| FR-5.3 read cadence | edge case | Demo needs one sample |
| FR-5.5, NFR-5 build without hardware | FR-025, SC-008 | |
| FR-5.9 direct HAL access | none | Not shown; must come through `kmr_api` (FR-002) |
| FR-6.1 `robot!`, aliases, no config, no build.rs | FR-003, FR-004, SC-007 | |
| FR-6.3 bus plugins, simulate and warn | FR-006, FR-007, SC-011 | Q1 |
| FR-6.4 errors by lifecycle moment | FR-026, SC-012 | |
| FR-7.3 own middleware in a controller | FR-029 | Allowed; not shown |
| FR-8.1 light payloads not registered | FR-029 | |
| FR-8.4 payload path, example fix | FR-012 | The fix is `kmr_api`'s (its example) |
| FR-10.1 builder in Rust | FR-005 | |
| FR-10.3 FFI from any form, user's cost | FR-029 | `kmr_ffi`, delivery step 8 |
| FR-10.4 onboarding metric | FR-027, FR-028, SC-001, SC-002 | Q2 |
| FR-13.1 responsibility split | FR-022 | |
| FR-13.4, FR-13.5 faults and fallbacks | none | Not built (steps 4, 6): out of scope |
| NFR-1 no lock, no heap on control path | FR-009 | |
| NFR-2 no unwrap, markers | FR-008, FR-011 | |
| NFR-3 encapsulation | FR-002 | |
| NFR-4 DX, math on arrays | FR-027, SC-002 | |
| NFR-6 maintainability | FR-010 | |
| NFR-10 does not apply | FR-030 | |
| NFR-11 compile time of a user crate | none | `bench`'s to track |

## SRS issues found

- SRS FR-6.3 "Without any bus plugin, run() simulates": "simulates" is undefined. It does not say
  whether History is fed (for example by echoing Desired) or stays empty. Answer: the
  no-hardware adapter seeds nothing, and History reads return the typed "no sample" error
  (kmr_core FR-042; recommended, pending human confirmation). This spec only asks that nothing
  panics and the demo ends.
- SRS FR-10.4 "the equivalent ROS2 node" is undefined, so the metric cannot be repeated the same
  way twice (Q2).
- SRS Definitions "API: kmr_api, the only crate user code imports" against SRS FR-5.9 ("reach the
  HAL directly") and SRS FR-10.3 (controllers call the FFI): both need a `kmr_api` re-export the
  SRS does not name.
- SRS FR-2.2 gives units per actuator kind (rad or mm), but kind is model-side only. A controller
  indexing actuators cannot tell which unit an index uses unless `robot!` exposes it.
- SRS Definitions Context ("any `T`", "`Arc<Mutex<T>>` is acceptable") against the constitution
  and root `clippy.toml`, which ban heap types and `Arc` in this crate. A user context with a
  `Vec` breaks "compile clean".
