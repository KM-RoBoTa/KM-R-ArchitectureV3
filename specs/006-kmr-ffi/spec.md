# Feature Specification: kmr_ffi, foreign-language controller math

**Feature Branch**: none (no branch hook; the branch is the human's choice)

**Created**: 2026-10-27

**Updated**: 2026-10-28, re-specified against the SRS as patched on 2026-10-27

**Status**: Draft

**Input**: User description: "Re-specify the kmr_ffi module of kmr v3 from docs/srs/modules/kmr_ffi.md
(its '## Module: kmr_ffi' section plus every system requirement this module owns or must honour),
against the SRS as patched on 2026-10-27."

**Authority**: `docs/srs/SRS.md`, module `kmr_ffi` (`kmr_ffi/**`, covers SRS FR-10, delivery step 8,
Could). On any conflict the SRS wins.

**Id convention**: `FR-0NN` / `SC-0NN` are this spec's ids. Each one names its SRS source next to
it. The table at the end maps the other way.

**What changed since the 2026-10-27 draft**: the SRS dropped the threaded-only rule (SRS FR-10.3).
C/C++, Python and Lua are now callable from every controller form, inline included. The cost of a
slow language is the user's. kmr_ffi may use the heap like kmr_api, and NFR-10 does not apply to it.
Prismatic units are mm, mm/s and N (SRS FR-2.2). N comes from `kmr_api::robot!` (SRS FR-6.1). The old
Python/Lua scope question is closed; the old "inline Python fails the build" rule is gone.

## Scope

kmr_ffi lets a Rust controller hand its math to C, C++, Python or Lua. The Rust controller stays the
one registered on a phase. The builder stays in Rust and is never exposed (SRS FR-10.1).

In scope:

- The plain data types that cross the boundary: a read view of History, a write buffer for Desired,
  a time view (SRS FR-10.2).
- A C header and a thin C++ header for the N of the build.
- A Python bridge and a Lua bridge that hand the same views to a script (SRS FR-10.1, FR-10.3).
- Use from every controller form: inline, inline with context, threaded (SRS FR-10.3).
- One example per language (SRS FR-10.4).

Out of scope:

- Any FFI to the builder, `run()`, phases or the HAL (SRS FR-10.1, section 7 FFI row).
- Foreign code as the program's entry point. Rust stays primary (SRS FR-10.1).
- Enforcing anything about the cost of foreign code on the control path (SRS FR-10.3).
- `no_std` builds. The user reaches FFI through `kmr_api`, which is `std` (SRS section 4). A
  `no_std` user path is out of this version (recommended, pending human confirmation).
- The threaded hand-off itself (SRS FR-1.2, section 8). kmr_ffi works with it once it exists.
- Group views and the `Sensors` handle on the foreign side (SRS FR-2.4, FR-5.7). See Assumptions.

Gate: SRS section 8 puts FFI last (step 8, Could). Building starts only after steps 1 to 7 move a
motor. Planning may start now.

## Clarifications

### Session 2026-10-28 (coordination)

- Cross-module coordination fixes. FR-010 and the read view carry the per-actuator age, to match
  kmr_core FR-018. The `kmr_api` boundary now cites kmr_api FR-072, which provides the `ffi`
  feature. Q1 / FR-009: time and flags cross as integers (ticks `u64`, durations ns `u64`, flags
  bitmasks); `no_std` user path out of this version (both recommended, pending human
  confirmation).

## Open questions for the human

- Q1 (FR-009): which types carry time values and flags, given "f32 always" in SRS FR-10.2 and "never
  f32 seconds" in SRS FR-3.3? Recommended: integers (ticks `u64`, durations in ns `u64`, flags as
  bitmasks), applied in FR-009, pending human confirmation.
- Q2 (FR-023, safety): how foreign code asks for a shutdown or an emergency stop.
- Q3 (FR-015): how the user's C/C++ sources are built against this build's N, given "no build.rs in
  the user's crate" and "names it once" in SRS FR-6.1.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Delegate controller math to C or C++ (Priority: P1)

A control engineer knows C++, not Rust. They write a short Rust controller that only forwards to
their C++ function. The C++ function reads the current positions and writes target positions as
plain float arrays. The robot moves as if the math were written in Rust.

**Why this priority**: it is the core of SRS FR-10, and C/C++ is the cheapest foreign path on the
control path.

**Independent Test**: with no hardware and a scripted History, run an `EachTick` inline controller
that forwards to a C function writing `q[i] = prev q[i] + 0.1`. Desired holds the same values and
the same written actuators as the Rust version of that controller.

**Acceptance Scenarios**:

1. **Given** an inline controller that forwards to a C function, **When** a tick runs, **Then** the
   C function receives the current sample as `q`, `qd`, `tau` float arrays of length N, in the units
   of SRS FR-2.2.
2. **Given** the C function writes `q` for actuators 0 and 2 only, **When** the controller returns,
   **Then** only actuators 0 and 2 are marked written for `q`, and the others get no new command
   (SRS FR-2.5).
3. **Given** the C function writes nothing, **When** the tick ends, **Then** no command is sent.
4. **Given** the same C function is forwarded from two registrations, **When** a tick runs,
   **Then** both calls happen in declaration order (SRS FR-1.3, FR-1.4).
5. **Given** an inline-with-context controller, **When** it forwards to C with a pointer to its own
   context data, **Then** the C function can keep state between ticks in that data.
6. **Given** a C++ user, **When** they include the C++ header, **Then** they get the same data with
   typed position, velocity and torque wrappers, at no runtime cost over the C header.

---

### User Story 2 - Delegate controller math to Python or Lua (Priority: P2)

A researcher prototypes a controller in Python, another in Lua. Each runs from a Rust controller,
inline included. The script reads the same views and writes the same buffer as C. The researcher
accepts that the interpreter costs time on the control path.

**Why this priority**: the SRS allows it from every form since 2026-10-27 (SRS FR-10.3). It is
slower than C, so it comes second.

**Independent Test**: load a Python script and a Lua script that each write `q[i] = prev q[i] + 0.1`.
Run them from an `EachTick` inline-with-context controller for 1000 scripted ticks. Desired matches
the Rust twin.

**Acceptance Scenarios**:

1. **Given** a script that fails to load, **When** the user builds its bridge in `main.rs`, **Then**
   the error is returned there, before `run()` and before any hardware I/O. Nothing panics.
2. **Given** a loaded script, **When** a tick runs, **Then** the script sees `q`, `qd`, `tau` as
   indexable float sequences of length N, and writes through the write buffer.
3. **Given** the script raises an exception or a Lua error, **When** the call returns, **Then** the
   forwarder gets an error, and nothing that script wrote in that call is applied.
4. **Given** a slow script, **When** a tick overruns, **Then** the overrun is counted like any other
   (SRS FR-3.2, FR-3.3). kmr does not block or warn about the language itself (SRS FR-10.3).

---

### User Story 3 - See history, staleness, faults and time from foreign code (Priority: P3)

The foreign function needs a filtered velocity. It reads `q` from two ticks ago, checks that the
current sample is not stale, and reads the tick index and `dt`.

**Why this priority**: History depth and staleness make the math correct (SRS FR-2.1, FR-2.3,
FR-2.6). No fault may be silent on the foreign side either (SRS FR-13.4).

**Independent Test**: script a History with a failed read on actuator 1. The foreign function sees
actuator 1 flagged stale, the sample age, and the same values `prev_q(k)` returns in Rust.

**Acceptance Scenarios**:

1. **Given** a History of depth `HISTORY_DEPTH`, **When** foreign code reads depth k, **Then** it gets
   the same values as `prev_q(k)`, `prev_qd(k)`, `prev_tau(k)` in Rust.
2. **Given** a tick without a hardware read, **When** foreign code reads the view, **Then** it sees
   the sample age in ticks (SRS FR-2.1).
3. **Given** a failed read on actuator 1, **When** foreign code reads the view, **Then** actuator 1
   is flagged stale and its value is the last good one, never zero (SRS FR-2.6).
4. **Given** a run past 2^24 ticks, **When** foreign code reads the tick index and elapsed time,
   **Then** both are exact (SRS FR-3.3).

---

### User Story 4 - Add a state field without FFI chores (Priority: P4)

An engine contributor adds `Kp` and `Kd` to the sealed field set (SRS FR-2.2, section 8). The FFI
types gain them with one declaration on the Rust side. If they forget it, the build fails.

**Why this priority**: SRS FR-10.2 and section 2 say exposing a type must never be a chore.

**Independent Test**: add a dummy field in a test build. The views, both headers and both bridges
gain it from one declaration. Leaving the FFI side out fails the build.

**Acceptance Scenarios**:

1. **Given** a new field in the field set, **When** the contributor adds its one FFI declaration,
   **Then** the read view, the write buffer, both headers and both bridges carry it.
2. **Given** a new field without its FFI declaration, **When** the workspace builds, **Then** the
   build fails and names the missing field.

---

### Edge Cases

- Foreign code writes NaN or infinity: it gets the same treatment as the same write from Rust
  (FR-013). The SRS is silent on that treatment; see Assumptions.
- C writes out of its arrays: C memory safety is the user's. Every array has a fixed, documented
  length N.
- A C++ exception reaches the boundary: the C++ header catches it and returns an error (FR-016).
  A Rust panic never crosses the other way: kmr_ffi's code does not panic (FR-022).
- Foreign code segfaults or aborts: the process dies with torque as it was. No software guard can
  prevent that (SRS NFR-2). The docs say so next to the pre-flight lesson of SRS FR-5.10.
- Foreign code is slow, allocates or takes the GIL: the user's choice and responsibility (SRS
  FR-10.3). Overruns are counted as usual (SRS FR-3.3).
- An actuator not written by foreign code but written by an earlier Rust controller in the same
  tick: the Rust write stays. Foreign writes take the same write path and the same mask.
- The C sources were built for another N: the build fails, it never runs with a wrong length
  (FR-015).
- A threaded controller forwards to foreign code: it gets only what the threaded form gets, `&Time`
  and its context, never the State handle (SRS FR-1.2). Testable once threaded controllers exist.

## Requirements *(mandatory)*

### Functional Requirements

**Boundary and placement**

- **FR-001 (SRS FR-10.1, section 3 "API")**: User code MUST reach the FFI only through `kmr_api`.
  `kmr_api` re-exports the kmr_ffi surface behind an opt-in feature. The controller crate keeps a
  single kmr dependency.
- **FR-002 (SRS FR-10.1, section 7 FFI row)**: The builder, `run()`, the phases, the signal bus
  internals and the HAL MUST NOT be exposed over the FFI.
- **FR-003 (SRS FR-10.1, FR-10.3)**: Foreign code MUST only be called from inside a Rust controller.
  Phase registration stays in Rust. The forwarder body is one kmr_ffi call: it fills the views,
  calls the foreign function and applies the writes.
- **FR-004 (SRS FR-10.3, FR-1.2)**: C, C++, Python and Lua MUST be callable from every controller
  form, inline included. kmr_ffi MUST NOT add any rule tied to the form. The threaded form gets
  only `&Time` and its context, as SRS FR-1.2 gives it.
- **FR-005 (SRS FR-10.3)**: kmr MUST NOT try to enforce the cost of a language on the control path.
  The docs of each bridge state that cost (GIL, VM allocation) and that it is the user's.

**Types that cross**

- **FR-006 (SRS FR-10.2)**: Every type that crosses MUST be a plain C-layout struct of fixed-size
  arrays and scalars. The foreign side sees no lifetimes, no generics, no pointers into engine
  memory.
- **FR-007 (SRS FR-10.2, FR-2.2)**: State values MUST cross as `f32`. Units: revolute actuators in
  rad, rad/s and N·m; prismatic actuators in mm, mm/s and N. `f64` only with a written
  justification. This is the one place bare scalars cross.
- **FR-008 (SRS FR-10.2, FR-6.1)**: Every array MUST have exactly N entries, N being the one this
  build's `robot!` fixes, and History exactly `HISTORY_DEPTH` slots. No padding to a maximum N, no
  runtime length.
- **FR-009 (SRS FR-10.2, FR-3.3, FR-13.4)**: Time values (tick index, `dt`, elapsed, overrun count,
  last and accumulated overrun) MUST cross without precision loss over a long run. They cross as
  integers: tick index and overrun count as `u64`, durations as `u64` nanoseconds. Stale,
  written and fault flags cross as integer bitmasks (Q1; recommended, pending human
  confirmation). SRS FR-10.2 says "f32 always", but an f32 tick index is wrong after 2^24 ticks
  (4.6 h at 1 kHz), and SRS FR-3.3 forbids f32 seconds.
- **FR-010 (SRS FR-2.1, FR-2.3, FR-2.6, FR-13.4)**: The read view MUST carry what a Rust controller
  reads through State, and nothing more: the field arrays at every depth `0..HISTORY_DEPTH`, the
  per-actuator age in ticks and the sample age (kmr_core FR-018), the per-actuator stale flags and
  the fault flags, as the engine exposes them.
- **FR-011 (SRS FR-2.5, NFR-3, FR-13.5)**: The write buffer MUST carry one array per field and one
  written flag per actuator per field. It starts empty at each call. After the call, kmr applies
  only the flagged entries, through the same setters a Rust controller uses, so the per-field write
  mask and any envelope check behave the same. Foreign code never writes engine memory.
- **FR-012 (SRS FR-2.5)**: The writes of one foreign call MUST be applied only when that call ends
  normally. On an error, nothing from that call is applied and the forwarder gets the error.
- **FR-013 (SRS FR-2.2, NFR-2)**: A foreign write MUST get exactly the treatment the same write from
  Rust gets, non-finite values included. kmr_ffi adds no value policy of its own.
- **FR-014 (SRS FR-10.2, FR-2.2, section 2)**: Adding a field to the sealed field set (`Kp`, `Kd`
  next) MUST need one declaration on the FFI side. A field missing on the FFI side MUST fail the
  build.

**Languages**

- **FR-015 (SRS FR-10.1, FR-10.2, FR-6.1)**: kmr_ffi MUST give C a header carrying N,
  `HISTORY_DEPTH` and the three view types of this build. C sources built for another N MUST fail
  to build or link. [NEEDS CLARIFICATION: Q3. SRS FR-6.1 allows no build.rs in the user's crate and
  names the robot once in main.rs, but C sources compile before `robot!` expands. How do they get N?]
- **FR-016 (SRS FR-10.1, NFR-2)**: A C++ header MUST wrap the C types with typed position, velocity
  and torque values, at no runtime cost over the C types. It MUST catch any C++ exception at the
  boundary and return it as an error.
- **FR-017 (SRS FR-10.1, FR-10.3)**: The Python bridge MUST hand the user's function the same views
  as named, indexable float sequences of length N, and take writes through the write buffer.
- **FR-018 (SRS FR-10.1, FR-10.3)**: The Lua bridge MUST do the same for a Lua function.
- **FR-019 (SRS FR-10.3, section 4, NFR-10)**: kmr_ffi MAY use the heap like `kmr_api`: interpreter
  start, script load and bridge buffers happen before the first tick. NFR-10 does not apply.
- **FR-020 (SRS FR-10.3, FR-6.4, NFR-2)**: A bridge that fails to start or load its script MUST
  return the error to the user in `main.rs`, before `run()`. It never panics.

**Real time and safety**

- **FR-021 (SRS NFR-1)**: On the C/C++ path, kmr_ffi's own per-call work MUST NOT allocate, lock or
  use `dyn`. It copies N-sized arrays and makes one call.
- **FR-022 (SRS NFR-2)**: No panic, exception or Lua error MUST cross the boundary. kmr_ffi's code
  has no `unwrap`, no `panic` and no unchecked indexing. Foreign errors reach the forwarder as a
  returned error.
- **FR-023 (SRS FR-4.1, FR-4.2, FR-4.4)**: Foreign code MUST be able to ask for a shutdown or an
  emergency stop, with the same first-reason-wins rule as Rust (SRS FR-4.3). [NEEDS CLARIFICATION:
  Q2. Through a status the forwarder turns into `shutdown!` / `emergency_stop!`, or through exported
  functions that raise the bus directly? The second touches the human-only `signal.rs`.]

**Docs and onboarding**

- **FR-024 (SRS FR-10.4, NFR-4)**: kmr_ffi MUST ship one example per language (C, C++, Python, Lua):
  a foreign function moving the robot from an `EachTick` controller, in the fewest files the build
  allows.
- **FR-025 (SRS NFR-2, FR-5.10)**: The docs MUST state that a crash or abort in foreign code leaves
  torque as it was, and that no software guard can prevent it.
- **FR-026 (SRS NFR-6)**: The design choices (copy versus shared view, header production, time and
  flag types, bridge state) MUST be defended in `docs/decisions/`.

### Key Entities

- **Read view**: a plain struct. The field arrays of length N at each History depth, the
  per-actuator age and the sample age, the stale flags and the fault flags. A copy, filled before
  the call.
- **Write buffer**: a plain struct. One array of length N per field and a written flag per actuator
  per field. Empty at each call, applied after a normal return.
- **Time view**: tick index, `dt`, elapsed, overrun count, last and accumulated overrun.
- **C header**: holds N, `HISTORY_DEPTH` and the three types above, for the N of this build.
- **C++ header**: wraps the C header with typed values; catches exceptions at the boundary.
- **Bridge (Python, Lua)**: an interpreter with the user's script loaded, built in `main.rs`, moved
  into a controller like any other context or captured value.
- **Forwarder**: the user's Rust controller. Registered on a phase, it makes one kmr_ffi call.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001 (SRS FR-10.3, FR-2.5)**: For 1000 scripted ticks, a C forwarder and its Rust twin produce
  the same Desired values and the same written flags, bit for bit, from an inline controller and
  from an inline-with-context controller.
- **SC-002 (SRS FR-10.3)**: For 1000 scripted ticks, Python and Lua forwarders produce the same
  written flags as their Rust twin, and values equal to the twin's after rounding to `f32`.
- **SC-003 (SRS NFR-1, NFR-11)**: On the NFR-1 bench at 1 kHz, an empty C forwarder adds no heap
  allocation and no lock in kmr_ffi's code. Its per-tick cost over an empty Rust controller is
  measured and tracked (NFR-8 set (d)).
- **SC-004 (SRS FR-10.2)**: Adding a field changes one FFI declaration. Forgetting it fails the
  build in 100% of the test cases.
- **SC-005 (SRS FR-10.2, FR-6.1)**: C sources built for another N fail the build in 100% of the test
  cases.
- **SC-006 (SRS FR-3.3)**: After 10^8 scripted ticks, the tick index and elapsed time seen by
  foreign code are exact.
- **SC-007 (SRS NFR-2)**: Every injected foreign error in the test suite (C error status, C++
  exception, Python exception, Lua error, failed script load) reaches the user as an error. None
  panics, none is applied in part.
- **SC-008 (SRS FR-10.4)**: A C++ engineer and a Python engineer each get a first moving controller
  with fewer files and in less time than the equivalent ROS2 node, with the same protocol as the
  Rust path.

## Assumptions

- **Copy, not shared memory.** The read view is a copy of History; the write buffer is applied
  through the setters. The core value types are not C-layout today (`Q(pub f32)` has no
  `#[repr(transparent)]`, `kmr_core/src/state/joint_state.rs`), and the History sample type is
  `pub(crate)`. A copy of `3 * N * HISTORY_DEPTH` floats is cheap at 1 kHz and keeps the seal
  (SRS NFR-3).
- **"Like kmr_api" means heap outside the hot path** (SRS FR-10.3, section 4 Constraints). kmr_ffi's
  own per-tick work stays heap-free. What the interpreter does inside a call is the user's cost.
- **kmr_ffi hosts the Python and Lua bridges.** SRS FR-10.1 says a C ABI serves Python and Lua; a C
  ABI alone cannot run a script, so the bridges embed the interpreter. The heap allowance of SRS
  FR-10.3 points the same way. Each bridge is opt-in, so a build that does not use it does not
  compile it, as for Zenoh (SRS section 2).
- **Lua indexing.** Lua sees its native 1-based arrays: actuator i is element i+1. The docs say so.
- **Bridge state and SRS FR-1.1.** An interpreter holds mutable state. The docs recommend holding
  the bridge as the controller's context (SRS FR-1.1, FR-1.2); a captured bridge is allowed, since
  kmr cannot stop it (SRS FR-10.3).
- **Parity, not more.** The foreign side sees what a Rust controller sees through State and Time.
  The age follows the engine: per actuator, and per sample as its oldest actuator's (kmr_core
  FR-018). Fault flags follow SRS FR-13.4 as delivered in step 4.
- **Whole robot only.** The foreign side sees all N actuators. Groups stay Rust-side slices (SRS
  FR-2.4); C can index any subset itself.
- **No Sensors yet.** SRS FR-10.1 exposes the state types; the `Sensors` surface is WIP (SRS FR-5.7).
- **Units are the engine's.** Degree, cm and m conversions stay in `kmr_api`, on the Rust side
  (SRS FR-2.2). The foreign side does not learn which actuators are prismatic, as the Rust State
  does not.
- **Non-finite writes.** The SRS never says what a NaN or infinity written into Desired does, from
  Rust or FFI. FR-013 inherits the core's answer.
- **std only.** kmr_ffi is reached through `kmr_api`, which is std (SRS section 4).
- **C memory safety is the user's.** The arrays have a fixed, documented length.

## Module boundaries

**kmr_ffi needs:**

| From | What |
|---|---|
| kmr_core | `Q`, `Qd`, `Tau` (later `Kp`, `Kd`), N as the const generic of the build and `HISTORY_DEPTH`; read access to every History depth, the sample age, stale flags and fault flags (SRS FR-2.1, FR-2.6, FR-13.4); the Time readers (SRS FR-3.3; all `pub(crate)` today); the setters that set the per-field write mask. Human-only crate: these are requests, not edits. |
| kmr_core | `f32` from `Q`/`Qd`/`Tau`: SRS FR-2.2 wants both ways; only `From<f32>` exists (`kmr_core/src/state/field.rs:37`). |
| kmr_core | The treatment of a non-finite write (FR-013); a decision on FR-023, which may touch `signal.rs`. |
| kmr_core / kmr_api | The threaded form and its hand-off (SRS FR-1.2, section 8) before the threaded case of FR-004 can be tested. |
| kmr_api | An opt-in feature that re-exports kmr_ffi (FR-001); the glue from its `State` and `Time` wrappers (private inner) to the fill and apply steps; `robot!`'s N reaching the FFI types. kmr_api FR-072 provides the `ffi` feature. |
| kmr_model | N and `HISTORY_DEPTH` for the C header, if Q3 needs them outside `robot!` (FR-015). |
| bench | The FR-10.4 protocol and the NFR-8 (d) overhead run (SC-003, SC-008). |
| Repo (human) | A `kmr_ffi/clippy.toml` heap override like `kmr_api`'s, and a constitution amendment (Principle II says only `kmr_api` may use the heap). |
| controllers | Nothing. The reference example stays Rust. |
| kmr_adapters | Nothing. FFI never reaches the HAL (FR-002). |

**kmr_ffi provides:**

- To `kmr_api`: the read view, write buffer and time view types (C layout); the fill and apply
  steps; the Python and Lua bridges; the foreign status and error type.
- To users, through `kmr_api`: a C header and a C++ header for the N of the build, and one example
  per language.
- To `bench`: an empty C forwarder for the SC-003 overhead run.

`kmr_api/examples/complete_api_example.rs` and `controllers/` are untouched by this module.

## SRS coverage

| SRS id | Spec ids | Status |
|---|---|---|
| FR-10.1 | FR-001, FR-002, FR-003, FR-015 to FR-018 | covered |
| FR-10.2 | FR-006 to FR-009, FR-014, FR-015; SC-004, SC-005 | covered; FR-009 Q1 recommended, pending confirmation; FR-015 needs Q3 |
| FR-10.3 | FR-003, FR-004, FR-005, FR-017 to FR-020; SC-001, SC-002 | covered, 2026-10-27 text |
| FR-10.4 | FR-024; SC-008 | shared with `controllers` and `bench`; protocol not defined in SRS |
| FR-1.1 | Assumptions (bridge state) | honoured |
| FR-1.2 | FR-004 | threaded case waits for section 8 |
| FR-1.3, FR-1.4 | US1 scenario 4 | honoured, nothing to build |
| FR-2.1, FR-2.3, FR-2.6 | FR-010; US3 | honoured; needs core readers |
| FR-2.2 | FR-007, FR-013, FR-014 | covered, units of 2026-10-27 |
| FR-2.5 | FR-011, FR-012; SC-001 | honoured |
| FR-3.2, FR-3.3 | FR-009; US2 scenario 4; SC-006 | FR-009 Q1 recommended, pending confirmation |
| FR-4.1 to FR-4.4 | FR-023 | needs Q2 |
| FR-5.10 | FR-025 | docs only |
| FR-6.1 | FR-008, FR-015; SC-005 | FR-015 needs Q3 |
| FR-6.4 | FR-020 | bridge errors before `run()` |
| FR-13.4 | FR-010 | full flag set is delivery step 4 |
| FR-13.5 | FR-011 | envelopes apply through the shared setters |
| NFR-1 | FR-021; SC-003 | kmr side only; foreign code is the user's (FR-10.3) |
| NFR-2 | FR-016, FR-020, FR-022, FR-025; SC-007 | covered |
| NFR-3 | FR-011 | covered |
| NFR-4 | FR-003, FR-024 | covered |
| NFR-5 | Assumptions (std) | covered |
| NFR-6 | FR-026 | covered |
| NFR-7 | none | out of scope: FFI is std only |
| NFR-10 | FR-019 | does not apply to kmr_ffi |
| Section 4 Constraints | FR-019, FR-021 | heap outside the hot path |
| Section 7 FFI row | FR-002, FR-015 to FR-018 | row lists only Python and C++; FR-10.1/10.3 add C and Lua |
| Section 8 step 8 | Scope "Gate" | honoured |
