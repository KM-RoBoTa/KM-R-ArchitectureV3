# Feature Specification: kmr_model, the robot model from one descriptor

**Feature Branch**: `003-kmr-model` (spec directory only; no branch created)

**Created**: 2026-10-27 · **Re-checked**: 2026-10-28, against the SRS patch of 2026-10-27

**Status**: Draft

**Input**: User description: "Specify the kmr_model module of kmr v3 from docs/srs/modules/kmr_model.md (its '## Module: kmr_model' section plus every system requirement that this module owns or must honour)."

**Authority**: `docs/srs/SRS.md`, module `kmr_model`: "FR-6, FR-8.1, FR-8.3; descriptor parsing at
compile time for `kmr_api::robot!`, actuator order, limits, presets; NFR-10, NFR-11. Delivery
step 2." On any conflict the SRS wins.

**Id convention**: `FR-0NN` and `SC-0NN` are this spec's ids. Every SRS reference reads
`SRS FR-x.y` or `SRS NFR-x`. The table at the end maps SRS ids to spec ids. Spec ids are stable:
new items get new numbers, so plan.md and research.md references stay valid.

## Scope

kmr_model turns one robot descriptor into fixed facts at compile time: how many actuators the
robot has (N), their order, their names, their kind and their limits, plus the link masses the
payload work needs. The user names the descriptor, or a KM-RoBoTa preset, once in `main.rs` with
`kmr_api::robot!`. Nothing kmr_model produces costs anything per tick.

**In scope**: reading the descriptor (URDF, `.ron`, JSON) for `robot!`, the model half of the
presets, counting and ordering actuators, limits, names, link masses, compile errors for bad
descriptors.

**Out of scope**:

- Hardware data (firmware id, bus, brand, protocol, baud, mode). It comes from the preset's
  hardware half or from `main.rs` (SRS FR-6.1; kmr_adapters, kmr_api). Ports are bound by builder
  plugins (SRS FR-6.3, kmr_api).
- The `robot!` front door itself and the aliases it emits (SRS module kmr_api, "FR-6.1 `robot!`
  macro"). kmr_model gives it the facts.
- The joint-limit policy: clamp, warn or abort, and its default (SRS FR-4.5, open decision).
- The home state (SRS FR-2.7, open in SRS section 8). (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.)
- A `.ron` that composes several URDFs (SRS FR-6.5, open in SRS section 8).
- Several actuators on one joint, or one actuator on several joints (SRS Definitions "Index
  space", TO RESOLVE in section 8).
- Joints with more than one degree of freedom (SRS section 8, roadmap).
- The center-of-mass computation (SRS FR-8.3, roadmap). The link data it needs is in scope.
- Light payloads: never registered (SRS section 2, FR-8.1).
- Payload registration on the builder (SRS FR-8.4, kmr_api).
- The visualizer's geometry (SRS FR-9.1, roadmap).
- A plugin system (SRS FR-11, planned). Only its door stays open (FR-025).
- A `no_std` front door for N (SRS NFR-7 confines `std` to kmr_api and is not in this module's
  Covers line). The `no_std` user path is out of this version (recommended, pending human
  confirmation).

## User Scenarios & Testing *(mandatory)*

Story numbers are stable. User Story 7 (presets) was added on 2026-10-28 and is P1.

### User Story 1 - The descriptor fixes N and the actuator order (Priority: P1)

An integrator has a URDF for the robot. They write `kmr_api::robot!(model = "robot/arm.urdf")`
once in `main.rs` and build. The build counts the actuators in that URDF and fixes N. Index 0 is
the first actuated joint declared in the file, index 1 the next one, and so on. Controllers then
work on arrays of length N, with `&mut State` signatures.

**Why this priority**: Every state array, mask and HAL call is sized by N (SRS Definitions
"Index space"). Today N is a hand-set 4 in the core. Nothing else in delivery step 2 matters
before this.

**Independent Test**: Build with a 4-actuator URDF, then with a 6-actuator URDF that also has 2
fixed joints. Check N is 4, then 6, and the order matches the file.

**Acceptance Scenarios**:

1. **Given** a URDF with 4 revolute joints, **When** the binary is built, **Then** N is 4 and
   index i is the i-th revolute joint in file order.
2. **Given** a URDF with 6 actuated joints and 2 fixed joints, **When** the binary is built,
   **Then** N is 6 and the fixed joints have no index.
3. **Given** the same descriptor, **When** the binary is cross-built for aarch64 with no robot
   plugged in, **Then** the build succeeds and N is the same.

---

### User Story 2 - A bad descriptor does not build (Priority: P1)

The integrator makes a mistake in the descriptor: a typo, a missing limit, two joints with the
same name. The build stops. The message names the file, the joint or link, and what is wrong, in
plain words.

**Why this priority**: SRS FR-6.4 says a bad descriptor is a compile error. A wrong model moves a
motor wrong.

**Independent Test**: Feed each bad case of the Edge Cases list. Each one fails the build with a
message that names the file and the element.

**Acceptance Scenarios**:

1. **Given** a URDF where a revolute joint has no limit, **When** the binary is built, **Then**
   the build fails and names that joint.
2. **Given** a descriptor that cannot be parsed, **When** the binary is built, **Then** the build
   fails and gives the file and line.
3. **Given** `robot!` names a file that does not exist, **When** the binary is built, **Then** the
   build fails and gives the path it looked for.

---

### User Story 3 - Limits per actuator, from the descriptor (Priority: P2)

The engine and the user need each actuator's limits: lowest and highest position, top speed, top
effort. They come from the descriptor, in the SRS FR-2.2 units, already in actuator index order.

**Why this priority**: SRS FR-4.5 enforces joint limits per actuator "through the model's
mapping". The policy is open, but the data must exist first.

**Independent Test**: Build with a URDF whose limits are known. Read the limits back for each
index and compare.

**Acceptance Scenarios**:

1. **Given** a revolute joint with lower -1.57, upper 1.57, velocity 3.0, effort 1.5, **When**
   the binary is built, **Then** its actuator's limits are -1.57 rad, 1.57 rad, 3.0 rad/s,
   1.5 N·m.
2. **Given** a prismatic joint with lower 0.0, upper 0.1, velocity 0.05, effort 20, **When** the
   binary is built, **Then** its actuator's limits are 0 mm, 100 mm, 50 mm/s, 20 N.
3. **Given** a continuous joint, **When** the binary is built, **Then** its actuator has no
   position limit, and that fact is readable, not a fake infinite value.

---

### User Story 4 - Same robot, other format (Priority: P2)

A user prefers `.ron`, or must use JSON. The same robot in either format gives the same N, the
same order, the same names and the same limits as the URDF.

**Why this priority**: SRS FR-6.1 supports three formats, `.ron` recommended. One model, three
doors.

**Independent Test**: Write the reference robot in URDF, `.ron` and JSON. Build each. Compare
the model facts.

**Acceptance Scenarios**:

1. **Given** the reference robot as URDF, as `.ron` and as JSON, **When** each is built, **Then**
   the model facts are identical.

---

### User Story 5 - Find an actuator by name (Priority: P3)

A user writes a group, or a hardware declaration, and wants to name actuators instead of
counting indices. They ask for the index of "shoulder_pitch" at compile time. A wrong name does
not build.

**Why this priority**: It turns a silent off-by-one into a compile error. Groups stay user-side
consts (SRS FR-2.4, FR-6.2); this only gives them the numbers. It is a lookup, not an accessor
method.

**Independent Test**: Look up every name of the reference robot and one wrong name.

**Acceptance Scenarios**:

1. **Given** the reference robot, **When** a const group uses the index of "elbow", **Then** it
   gets the same number as the file order.
2. **Given** a name not in the descriptor, **When** it is looked up in a const, **Then** the
   build fails and names it.

---

### User Story 6 - Link masses for payloads (Priority: P3)

A user mounts a medium or heavy payload on a link. kmr must know its mass and where it sits,
because it shifts the robot's center of mass (SRS section 2, FR-8.1). The model keeps each link's
mass and mass center from the descriptor, and can tell whether a link name exists. Light payloads
are not registered.

**Why this priority**: The data is cheap to keep and the payload path needs it. The center-of-mass
computation itself is roadmap (SRS FR-8.3).

**Independent Test**: Build with a URDF with known link masses. Read them back. Look up an
existing and a missing link.

**Acceptance Scenarios**:

1. **Given** a link with mass 0.35 kg, **When** the binary is built, **Then** the model gives
   0.35 kg for it.
2. **Given** a payload mounted on a link name that does not exist, **When** `run()` finalizes,
   **Then** `run()` returns an error naming the link (SRS FR-6.4: a bad registration is a run()
   error; the check is kmr_api's, the lookup is this module's).

---

### User Story 7 - A preset names the whole robot (Priority: P1)

A control engineer works on a KM-RoBoTa robot. They write `kmr_api::robot!(XmBench)` in
`main.rs` and nothing else about the model. The build takes the preset's descriptor, which ships
with kmr, and fixes N and the order from it. The preset's hardware half has one entry per
actuator, in the same order.

**Why this priority**: The SRS module line names "presets", and the reference example starts
with `robot!(XmBench)`. Without it the example does not build.

**Independent Test**: Build with `robot!(XmBench)` and with `robot!(model = ...)` on the same
URDF. Compare the model facts. Build with an unknown preset name.

**Acceptance Scenarios**:

1. **Given** `robot!(XmBench)`, **When** the binary is built, **Then** N and the model facts equal
   those of the preset's own descriptor named by path.
2. **Given** `robot!(Xmbench)` (a typo), **When** the binary is built, **Then** the build fails,
   names it, and lists the known presets.
3. **Given** a preset whose hardware half lists 3 actuators and whose model half has 4, **When**
   the binary is built, **Then** the build fails and gives both counts (kmr_api FR-007).

### Edge Cases

Each case below fails the build with a message naming the file and the element, unless it says
otherwise:

- `robot!` names a file that does not exist, or an unknown preset.
- The file has an extension other than `.urdf`, `.ron` or `.json`, or its path is not valid
  UTF-8.
- A joint or link without a name.
- The file does not parse (gives the line).
- Zero actuators.
- Two joints or two links with the same name.
- A revolute or prismatic joint without a limit.
- A continuous joint without a top speed or top effort.
- A lower limit above the upper limit; a negative velocity or effort limit.
- A number that is not finite, or does not fit an `f32` after unit conversion (SRS FR-2.2).
- A floating or planar joint: stops with "roadmap: non-1-DOF joints" (SRS section 8).
- A mimic joint, a transmission with several actuators or joints: stops with the marker
  "SRS index space: multi-actuator joint" (SRS section 8).
- More than one root link in a URDF, or a `.ron` that composes several URDFs: stops with the marker
  "SRS FR-6.5: multi-robot descriptor" (SRS section 8).
- A field the format does not know, such as a group or a hardware field in a `.ron`.
- A continuous joint: builds; no position limit (User Story 3).
- One URDF with a common root link for several robots: builds as one robot (SRS FR-6.5 (1)). The
  model does not check that each robot's joints form one block; order is file order.
- The descriptor changes: the next build reads it again and rebuilds the crate that names it.
- A preset hardware half, or a hardware declaration in `main.rs`, that lists more or fewer
  actuators than N: a compile error, raised by `robot!`'s array-type check (kmr_api FR-007)
  against the N this module gives. rustc's array-size error names both counts.
- `main.rs` has no `robot!` call, or two: a compile error, raised by kmr_api (missing or duplicate
  `N`, `State`, `Robot`).

## Requirements *(mandatory)*

### Functional Requirements

**Descriptor and actuators**

- **FR-001** (SRS FR-6.1, section 4 Assumptions): A build MUST read exactly one robot descriptor:
  URDF by default, `.ron`, or JSON. It is named by a descriptor path or by a preset.
- **FR-002** (SRS FR-6.1, Definitions "Index space"): N MUST be the number of actuated joints.
  Fixed joints are not actuators and get no index.
- **FR-003** (SRS FR-6.1): Actuator order MUST be the declaration order of the actuated joints in
  the descriptor.
- **FR-004** (SRS FR-6.1, Definitions "Compile time", section 7 Config): N and the actuator order
  MUST be fixed at compile time and reach the engine as constants, with no run-time cost. The
  descriptor is read for `kmr_api::robot!` in the user's `main.rs`, which emits the `N`, `State`
  and `Robot` aliases; N reaches kmr_core as a const generic. `robot!` is the only door: users
  never name kmr_model, and N MUST come only from the descriptor, never from a number the user
  types.
- **FR-005** (SRS FR-6.2): The descriptor MUST declare no groups. The model MUST generate no
  accessor methods. Groups stay user-side consts (SRS FR-2.4).
- **FR-006** (SRS FR-6.1): The descriptor MUST carry model data only. Hardware data (firmware id,
  bus, brand, protocol, baud, mode, port) is never read from it.
- **FR-007** (SRS FR-6.3, FR-5.5, NFR-5): The descriptor MUST hold nothing tied to one machine.
  The same descriptor MUST build on any host and cross-build without hardware. No config file, no
  side file, no `build.rs` in the user's crate (SRS FR-6.1).

**Presets**

- **FR-022** (SRS module kmr_model "presets", FR-6.1): A preset MUST have a model half that ships
  with kmr_model as a descriptor. Naming a preset MUST give the same facts as naming that
  descriptor by path. An unknown preset name MUST be a compile error that lists the known ones.
- **FR-023** (SRS FR-6.1): A preset MUST use one name for its model half (kmr_model) and its
  hardware half (kmr_adapters). The hardware half lists exactly N actuators, in the model's order.
  The count check is `robot!`'s (kmr_api FR-007), against the N this module gives.
- **FR-024** (SRS FR-6.1, FR-13.5): `main.rs` MUST NOT override a preset's model half. The SRS
  override rule covers hardware data. Tighter bounds are per-actuator envelopes (SRS FR-13.5,
  kmr_api); another model is a descriptor path.

**Errors**

- **FR-008** (SRS FR-6.4, NFR-2): A bad descriptor MUST be a compile error. The message MUST name
  the file, the joint or link, and the reason, in plain words. Reading MUST never panic: every
  failure is a compile error. Every Edge Case above is a bad descriptor unless it says otherwise.
- **FR-009** (SRS Definitions "Index space", section 8): A joint driven by several actuators, or
  an actuator driving several joints, MUST stop the build with the marker
  "SRS index space: multi-actuator joint".
- **FR-010** (SRS section 8 roadmap "Non-1-DOF joints"): A floating or planar joint MUST stop the
  build and say it is roadmap.
- **FR-011** (SRS FR-6.5, section 8): A descriptor with several root links, or a `.ron` that
  composes several URDFs, MUST stop the build with the marker
  "SRS FR-6.5: multi-robot descriptor", until the human decides the format.

**Model facts**

- **FR-012** (SRS FR-4.5, FR-2.2): For each actuator the model MUST give its lowest and highest
  position, top speed and top effort, in the FR-017 units, in actuator index order. They reach
  users as `Q`, `Qd`, `Tau`.
- **FR-013** (SRS FR-4.5, NFR-10): An actuator without a position limit (continuous joint) MUST be
  marked as such in a per-actuator bit set, not by a fake value.
- **FR-014** (SRS FR-4.5): The model MUST apply no limit policy. It gives the numbers; the engine
  decides what to do (open decision).
- **FR-015** (spec default, serves SRS FR-2.4): For each actuator the model MUST give its joint
  name and its kind (revolute, continuous, prismatic). A name MUST map to its index at compile
  time; an unknown name MUST be a compile error.
- **FR-016** (SRS FR-8.1, FR-8.3, section 2): The model MUST keep each link's name, parent, mass
  and mass center from the descriptor, and MUST say whether a link name exists. It takes no light
  payload input. Recomputing the robot's center of mass with payloads is roadmap (SRS FR-8.3).
- **FR-017** (SRS FR-2.2): Prismatic actuators build. Their `Q`, `Qd`, `Tau` are mm, mm/s and N;
  revolute ones rad, rad/s and N·m. The model converts descriptor metres to mm.
- **FR-026** (SRS FR-2.2): Every number the model emits MUST be `f32`. Written justification for
  wider numbers: the build machine may parse in `f64` only to detect values that do not fit `f32`;
  none reaches the binary.

**Cost**

- **FR-018** (SRS NFR-1, NFR-11): Nothing from the model MUST run in a tick unless a controller
  calls it. All model facts are constants.
- **FR-019** (SRS NFR-1): What the model emits MUST be plain constants: no heap, no `dyn`, no lock,
  and nothing that needs `std` or `alloc`. The heap used while reading, on the build machine,
  never reaches the binary.
- **FR-020** (SRS NFR-10): Per-actuator facts MUST be laid out as one array per fact, indexed by
  actuator; yes/no facts as bit sets; names reached by index.
- **FR-021** (SRS NFR-11): Editing the descriptor MUST rebuild the crate that names it. A build
  where nothing changed MUST NOT recompile that crate. The descriptor is read on every compile of
  that crate, so its read time MUST be measured and tracked.

**Openness**

- **FR-025** (SRS FR-11.3): The format readers MUST sit behind one compile-time interface (traits
  and generics, no trait objects), so a later plugin can add a format or URDF options without
  changing the others. No plugin system ships in this version.

### Key Entities

- **Descriptor**: the one file per build that describes the robot. URDF, `.ron` or JSON. Model
  data only.
- **Preset**: a KM-RoBoTa robot under one name. Model half: a descriptor in kmr_model. Hardware
  half: a table in kmr_adapters, one entry per actuator.
- **Actuator**: one index in `0..N`. Has a name, a kind and limits. In this version one actuator
  moves exactly one joint.
- **Joint**: a physical articulation in the descriptor. Model-side only; the engine never indexes
  joints (SRS Definitions "Joint").
- **Limits**: per actuator, lowest and highest position, top speed, top effort, plus "has no
  position limit".
- **Link**: a rigid body of the descriptor. Has a name, a parent, a mass and a mass center. A
  payload mount point names a link.
- **Model facts**: everything above, as compile-time constants.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001** (SRS FR-6.1): For three reference descriptors (4-actuator chain, 6 actuated plus 2
  fixed joints, a continuous joint), the built N and order match a hand count in 3 out of 3.
- **SC-002** (SRS FR-6.4): Every Edge Case marked as a build failure fails the build, and the
  message names the file and the element, in 100% of the cases.
- **SC-003** (SRS FR-6.1): The reference robot in URDF, `.ron` and JSON gives identical model
  facts in 3 out of 3.
- **SC-004** (SRS NFR-1, NFR-8 (d)): Engine overhead per tick on the NFR-1 bench is the same with
  the model as with today's hand-set constant, within the bench's run-to-run spread.
- **SC-005** (SRS NFR-11): A rebuild with nothing changed recompiles 0 crates; editing the
  descriptor recompiles the crate that names it. Reading a 100-joint URDF adds under 1 s to a
  build on the x86-64 dev machine.
- **SC-006** (SRS NFR-7): A `#![no_std]` crate that calls `robot!` builds, so the emitted facts
  need no `std` or `alloc`.
- **SC-007** (SRS FR-5.5, NFR-5): The same descriptor cross-builds for aarch64 and armv7 with no
  hardware attached.
- **SC-008** (SRS FR-6.1, human validation 2026-10-27): `kmr_api/examples/complete_api_example.rs`
  (validated with `robot!(XmBench)`, `add_plugins(Bus::by_id(..))` and `try_lock`) builds and runs
  against the reference preset. It may change with the API (kmr_api FR-069).
- **SC-009** (SRS FR-6.2): The model adds 0 group types and 0 per-group accessor methods to the
  public surface.
- **SC-010** (SRS FR-6.1, module "presets"): For every shipped preset, `robot!(Preset)` and
  `robot!(model = <its descriptor>)` give identical model facts in 100% of the presets.

## Assumptions

**Decisions applied (SRS, 2026-10-27)**

- `kmr_api::robot!(Preset)` or `robot!(model = "x.urdf")` in `main.rs`; the descriptor is read at
  compile time by kmr_model; N becomes a const generic in kmr_core (human work) (SRS FR-6.1,
  section 4 Assumptions).
- Hardware comes from the preset or `main.rs`, never from a file (SRS FR-6.1). Ports are builder
  plugins (SRS FR-6.3).
- State values are `Q`, `Qd`, `Tau`, defined once in kmr_core, `f32`; revolute rad, rad/s, N·m;
  prismatic mm, mm/s, N (SRS FR-2.2).
- Light payloads are not registered (SRS section 2, FR-8.1).
- kmr_core is human-only. Any change there this module needs is a request to the human.
- The reference example starts from the version the human validated on 2026-10-27: it calls
  `robot!(XmBench)`, binds `Bus::by_id(..)` and uses `try_lock`. It may change with the API; it
  must build and run (SC-008, kmr_api FR-069).

**Defaults chosen**

- Non-fixed URDF joints are actuators (SRS FR-6.1, literal). A passive joint that is not fixed
  cannot be declared in URDF in this version; it falls under FR-009.
- A URDF `<transmission>` with one actuator and one joint is allowed and changes nothing in this
  version. Any other transmission falls under FR-009.
- `.ron` and JSON use one schema; JSON is the same data in another syntax. All three formats carry
  the URDF units (rad, m, kg, N·m, N); the model converts to the SRS FR-2.2 units.
- Link mass is in kg and mass centers in m, as in the descriptor: SRS FR-2.2 sets units for
  actuators only, and the consumer (SRS FR-8.3) is roadmap.
- A continuous joint needs a top speed and a top effort: the SRS FR-4.5 limits have no safe
  default.
- N must be at least 1. There is no upper bound from the model; the engine's own bounds apply.
- The 1 s build-time target of SC-005 is a starting guess, to be replaced by the first bench
  measurement (SRS NFR-11: measured, not guessed).
- The workspace ships one preset: today's 4-actuator bench chain, so the existing tests and the
  example keep N = 4. Its real limits and masses come from the human.
- Every test fixture that must build carries full limits. A fixture without `<limit>` (today's
  `kmr_api/tests/fixtures/four.urdf`) is a bad descriptor under FR-008 and must be fixed, not
  allowed (review-2026-10-28.md, item 10).
- Reading the descriptor happens at build time, on the build machine, and may use the heap. That
  code is not in the binary, so the run-time rules of SRS NFR-1 and NFR-10 bind only what it
  emits. The SRS does not say this for kmr_model; `kmr_model/clippy.toml` does (reported).
- kmr_model holds no run-time code in this version. The center of mass (SRS FR-8.3) and a plugin
  interface (SRS FR-11.3) will need some; where it lives is reported as an open question.

**Open SRS items not decided here**

- Joint-limit policy and default (SRS FR-4.5, section 8).
- Home state source (SRS FR-2.7, section 8). (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.)
- Multi-robot descriptor format (SRS FR-6.5, section 8).
- Multi-actuator joint (SRS Definitions "Index space", section 8).
- `HISTORY_DEPTH` owner (SRS section 8). The model does not set it.
- Center-of-mass recomputation (SRS FR-8.3, section 8 "Model crate").

### Module boundaries

**kmr_model provides**

- N and the actuator order, as compile-time constants, for `kmr_api::robot!`.
- Per actuator: name, kind, limits (`Q`, `Qd`, `Tau`, SRS FR-2.2 units), "no position limit" bit
  set.
- A compile-time name-to-index lookup that fails the build on an unknown name.
- Links: name, parent, mass, mass center; a link-exists lookup for payload mount points.
- The model half of each preset, and the preset name list for the unknown-preset error.
- Readable compile errors for bad descriptors, with the SRS markers of FR-009 to FR-011.

**kmr_model needs**

- From kmr_core (human): N as a const generic instead of the hand-set `JOINTS = 4`
  (`kmr_core/src/state.rs:6`). kmr_core does not depend on kmr_model: the emitted constants use
  the core's `Q`, `Qd`, `Tau` through kmr_api at the call site, so there is no cycle.
- From kmr_core (human): a consumer for the limits once the SRS FR-4.5 policy is decided. It must
  read the "no position limit" bit before the position limits.
- From kmr_api: the `robot!` front door and its aliases; a compile-time check `N == JOINTS` until
  the const generic lands; no default `N`/`State` export after that, so a missing `robot!` fails
  the build; the re-export of the name lookup; `register_payload` (SRS FR-8.4) using the link
  lookup; per-actuator envelopes (SRS FR-13.5) as the way to tighten limits; user docs.
- From kmr_adapters: each preset's hardware half under the same name, in model order, as a
  fixed-size array so `robot!`'s count check works (kmr_api FR-007); `Preset::N` removed, since N
  comes from the model.
- From controllers: a `robot!` line in `controllers/src/main.rs` (it imports `State` from kmr_api
  today).
- From bench: SC-004 (overhead per tick) and SC-005 (build time), SRS NFR-8 (d), NFR-11.
- From kmr_ffi: nothing yet. Its fixed-size `f32` arrays (SRS FR-10.2) will need N from `robot!`.

## Clarifications

### Session 2026-10-28 (resolved from the SRS patch of 2026-10-27)

- Q1 → B. `kmr_api::robot!` in `main.rs` reads the descriptor; N becomes a const generic in
  kmr_core (SRS FR-6.1, section 4 Assumptions, section 8 decided 2026-10-27).
- Q2 → A. Roadmap; the model keeps link masses now (SRS FR-8.3).
- Q3 → prismatic in mm, mm/s, N; revolute in rad, rad/s, N·m (SRS FR-2.2). Prismatic builds.

### Re-check 2026-10-28 (resolved from the SRS, no question asked)

- Presets were in the SRS module line but had no requirement: added User Story 7, FR-022, FR-023,
  SC-010.
- "Any preset default can be overridden there" (SRS FR-6.1) is read as hardware data only;
  tighter limits go through SRS FR-13.5 envelopes: FR-024.
- SRS FR-11.3 names the descriptor parser: FR-025.
- `no_std`: SRS NFR-7 binds kmr_core and is not in this module's Covers line; SC-006 now checks
  that the emitted facts need no `std`, not a bare-metal build.
- A proc macro runs on every compile of its caller: FR-021 and SC-005 reworded.
- Light payloads not registered: FR-016. `f64` justification: FR-026.

### Questions as first asked

Superseded by the session above; kept for history.

- **Q1 (FR-004, scope)**: SRS FR-6.1 says the user's controller crate's `build.rs` reads the
  descriptor. That build script runs after kmr_core is compiled, so it cannot set kmr_core's N.
  Where is the descriptor read?
  - A: One line in the workspace's Cargo config names the descriptor; the model's build step
    reads it and kmr_core's N comes from it. Keeps decision 0001 (constants), accepted by the SRS.
    The `main.rs` hardware macros are checked against N at compile time. One extra line outside
    `main.rs`.
  - B: A top-level macro in `main.rs` reads the descriptor; N becomes a const generic in kmr_core.
    Keeps everything in `main.rs`; reopens decision 0001 (fix-plan decision 3 A).
  - C: Keep the hand-set N for now; the model only checks the descriptor against it. Nothing
    breaks, but N does not come from the descriptor yet.
  - Recommended: A.
- **Q2 (FR-016, scope)**: Is the center-of-mass recomputation in this version?
  - A: Roadmap. The model keeps link masses now; the computation waits (SRS FR-8.3 literal).
  - B: A static center of mass at `run()` finalization, at the zero pose.
  - C: A pose-dependent query a controller may call, never called by the engine.
  - Recommended: A.
- **Q3 (FR-017, safety)**: Prismatic actuators move in metres, but `Q` is "rad" in the human's
  units, and `Q::from_deg` makes no sense on them.
  - A: `Q` holds the actuator's SI position: rad for revolute, m for prismatic. The model records
    the kind. No type change.
  - B: Prismatic joints do not build in this version (compile error), until a length type exists.
  - C: New length types in kmr_core's sealed field set now.
  - Recommended: A.

### Session 2026-10-28 (coordination)

- Cross-module coordination fixes. FR-023, the edge cases and User Story 7: the preset count check
  is `robot!`'s (kmr_api FR-007), an array-type check; this module only gives N. SC-006 cites SRS
  NFR-7, not NFR-1. SC-008 and Assumptions: the example may change with the API, it must build
  and run. "Questions as first asked" marked superseded. `no_std` user path out of this version
  (recommended, pending human confirmation). Contract `robot-macro.md` lists the adapter type on
  `Robot` and the kinds and limits wiring.

## SRS coverage

| SRS id | Spec ids | Status |
| --- | --- | --- |
| SRS FR-6.1 (descriptor, N, order, formats, `robot!`) | FR-001, FR-002, FR-003, FR-004 | Covered |
| SRS FR-6.1 (presets; module line "presets") | FR-022, FR-023, FR-024, User Story 7, SC-010 | Covered: model half here, hardware half kmr_adapters |
| SRS FR-6.1 (hardware data from preset or main.rs) | FR-006, FR-024 | Covered: never read from the descriptor |
| SRS FR-6.1 (no config file, no user build.rs) | FR-007 | Covered |
| SRS FR-6.2 | FR-005, SC-009 | Covered |
| SRS FR-6.3 (topology portable, compile-time) | FR-007, SC-007 | Covered |
| SRS FR-6.3 (port binding) | none | kmr_api: `Bus::by_id` / `Bus::by_dev` builder plugins |
| SRS FR-6.4 (bad descriptor) | FR-008, SC-002 | Covered |
| SRS FR-6.4 (bad port binding, bad dt) | none | kmr_api / kmr_core |
| SRS FR-6.5 | FR-011, Edge Cases | Single tree builds; composition open in section 8 |
| SRS FR-8.1, section 2 | FR-016, User Story 6 | Data covered; light payloads excluded; computation roadmap |
| SRS FR-8.3 | FR-016 | Roadmap |
| SRS FR-8.4 | none | kmr_api; uses the FR-016 link lookup |
| SRS FR-4.5 (limits data) | FR-012, FR-013, FR-014 | Covered; policy out of scope |
| SRS FR-2.2 | FR-012, FR-017, FR-026 | Covered |
| SRS FR-2.4 (groups as slices) | FR-005, FR-015 | Covered: names give indices only |
| SRS FR-11.3 (descriptor parser behind compile-time interfaces) | FR-025 | Covered; no plugin system |
| SRS Definitions "Index space", section 8 multi-actuator joint | FR-002, FR-009 | Covered as a build stop |
| SRS section 8 roadmap non-1-DOF joints | FR-010 | Covered as a build stop |
| SRS NFR-1 | FR-018, FR-019, SC-004 | Covered |
| SRS NFR-2 | FR-008 | Covered: no panic, compile errors only |
| SRS NFR-5 | FR-007, SC-007 | Covered |
| SRS NFR-7 | FR-019, SC-006 | Not in module Covers; emitted facts need no `std` |
| SRS NFR-10 | FR-013, FR-020 | Covered; no sparse data, so no map |
| SRS NFR-11 | FR-018, FR-021, SC-004, SC-005 | Covered |
| SRS FR-2.7 home state | none | Out of scope: open in section 8 (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.) |
| SRS FR-9.1 | none | Roadmap |
| Delivery step 2 | FR-001 to FR-026 | Covered; gate "moves a motor" reported |
