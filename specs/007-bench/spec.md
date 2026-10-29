# Feature Specification: bench, the reference benchmark of kmr v3

**Feature Branch**: none (no branch hook; the branch is the human's choice)

**Created**: 2026-10-27

**Updated**: 2026-10-28, re-specified against the SRS as patched on 2026-10-27

**Status**: Draft

**Input**: User description: "Re-specify the bench module of kmr v3 from docs/srs/modules/bench.md
(NFR-1 reference bench, NFR-8 measurement set (a) to (e), NFR-11 overhead and compile-time
tracking), against the SRS as patched on 2026-10-27, and address the gaps the 2026-10-27
verification report found."

**Authority**: `docs/srs/SRS.md`, module `bench` (`bench/**`). It covers SRS NFR-1 (the bench),
SRS NFR-8 (measurement set a to e) and SRS NFR-11 (overhead and compile time). Constitution V
("Measured, Not Guessed") binds it. On any conflict the SRS wins.

**Id convention**: `FR-0NN` and `SC-0NN` are this spec's ids. Each one names its SRS source next
to it, e.g. "FR-008 (SRS NFR-8 b)". The coverage table at the end maps the other way.

## Scope

The bench measures kmr. It does not make kmr faster, and it sets no target figures: SRS NFR-1
defers them until they are measured. It produces reports that anyone with the same hardware can
reproduce.

In scope:

- The five measurements of SRS NFR-8: (a) tick-rate ceiling, (b) jitter and overruns at 1 kHz on
  non-RT and PREEMPT_RT, (c) the same under load, (d) engine overhead per tick, (e) footprint.
- Compile-time tracking of the core and of a user controller crate (SRS NFR-11).
- Engine-only runs with no bus plugin: kmr simulates and the core's no-hardware adapter warns
  once (SRS FR-6.3, kmr_core FR-042).
- Hardware runs on a reference actuator chain, for the I/O figures SRS FR-5.3 sends to the bench.
- The written procedure that makes a run reproducible (SRS NFR-8).
- The answer to the open feasibility of the 10 µs `dt` floor (SRS FR-3.1).
- A comparison of two builds or two controller sets with the same method, which other modules'
  success criteria rely on (see Module boundaries).

Out of scope:

- Target figures and pass/fail gates (SRS NFR-1: "Exact figures are deferred until measured").
- The no_std MCU target (SRS section 4: distinct from the bench target; still to be named).
- Threaded controllers (SRS FR-1.2: TBD as a whole; not spawned yet).
- The `kmr` CLI (SRS FR-12.1: TBD). The bench runs without it.
- Data-layout benchmarks of SRS NFR-9: owned by the `kmr_core` module (SRS "Module: kmr_core").
- The FR-10.4 onboarding metric: owned by the `controllers` module (SRS "Module: controllers").
- Counting bus transactions (specs/001-kmr-adapters SC-002): a test inside `kmr_adapters`. The
  bench gives it the host and the chain, not the count.
- Lock counting: a black-box bench cannot see a lock (FR-012 says so in each report).

## Clarifications

### Session 2026-10-28 (coordination)

- Cross-module coordination fixes. FR-006, User Story 1 and 7, the edge cases and the boundaries:
  the one "no bus plugin: simulating" warning is kmr_core FR-042's; `kmr_api`'s "not driven yet"
  message exists only until delivery step 1 (kmr_api FR-026). What "simulated" means: the
  no-hardware adapter seeds nothing, History reads return the typed "no sample" error
  (recommended, pending human confirmation). FR-027 no longer freezes the complete API example.
  Citations fixed: core SC-010 → SC-012, ffi SC-002 → SC-003 (User Story 5 and the ffi
  boundary), ffi SC-006 → SC-008.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Measure timing at 1 kHz without hardware (Priority: P1)

An engine contributor wants to know how steady the loop is. They run the bench at `dt` = 1 ms on
the reference Raspberry Pi 4 with no bus plugin, so kmr simulates. They run it once on a non-RT
kernel, once on a PREEMPT_RT kernel, and once more on each under stress-ng load. They get jitter
p50, p99 and max, and the overrun count, for each of the four cells.

**Why this priority**: This is the core of SRS NFR-1 and NFR-8 b and c. Every other performance
claim leans on it. It needs no hardware, so it can run first.

**Independent Test**: On the reference bench, one run per cell gives a report with the four
figures and the run's metadata.

**Acceptance Scenarios**:

1. **Given** the reference bench on a non-RT kernel and no bus plugin, **When** the 1 kHz run
   ends, **Then** the report holds jitter p50, p99, max and the overrun count, the metadata of
   FR-002, and the status "simulated".
2. **Given** the same binary on a PREEMPT_RT kernel, **When** the run ends, **Then** the report
   holds the same figures, and the two reports can be compared side by side.
3. **Given** stress-ng loads the CPU and I/O with the recorded profile, **When** the 1 kHz run
   ends, **Then** the report holds the same figures and names the load profile.
4. **Given** no bus plugin, **When** run() starts, **Then** kmr's "no bus plugin: simulating"
   warning (kmr_core FR-042) appears exactly once and the bench records it.

---

### User Story 2 - Reproduce a published run (Priority: P1)

Someone outside the team has the same hardware. They follow the written procedure, run the bench
and get a report they can compare with a published one.

**Why this priority**: SRS NFR-8 asks for a benchmark "reproducible by anyone with that hardware
and setup". A figure nobody can reproduce is a guess (constitution V).

**Independent Test**: A person who did not write the bench follows the procedure alone and
produces a complete report.

**Acceptance Scenarios**:

1. **Given** a board that matches the reference bench, **When** the person follows the procedure,
   **Then** they get a complete report without asking the authors.
2. **Given** a host that does not match the reference bench, **When** a run ends, **Then** the
   report is marked "not reference" and is never compared with reference reports.
3. **Given** a hardware run on another person's USB adapter, **When** they follow the procedure,
   **Then** the only source line they change is the serial id in `Bus::by_id(..)`.

---

### User Story 3 - Find the tick-rate ceiling (Priority: P2)

An engine contributor wants the highest rate the loop holds without overrun when controllers do
nothing. The answer also tells whether the 10 µs `dt` floor of SRS FR-3.1 is feasible.

**Why this priority**: SRS NFR-8 a, and the only way to close the open feasibility of SRS FR-3.1.

**Independent Test**: One ceiling search gives one rate and states whether the floor is held.

**Acceptance Scenarios**:

1. **Given** no-op controllers and no bus plugin, **When** the ceiling search ends, **Then** the
   report gives the highest rate held with zero overruns for the whole hold window.
2. **Given** the loop holds even at the 10 µs floor, **When** the search ends, **Then** the report
   says "ceiling at least 100 kHz (floor held)", since a shorter `dt` is a run() error.

---

### User Story 4 - Measure engine overhead and footprint (Priority: P2)

An engine contributor wants to know how much of each tick the engine itself uses, and how big
the binary and its memory use are. SRS NFR-11 frames it: the rest of each tick is the budget for
controller instructions.

**Why this priority**: SRS NFR-8 d and e, and SRS NFR-11.

**Independent Test**: One run set gives overhead per tick, the controller budget left at 1 kHz,
binary size, peak RSS, stack high-water mark and the heap allocations seen inside the tick.

**Acceptance Scenarios**:

1. **Given** the 1 kHz setup, **When** the budget search ends, **Then** the report gives the
   largest controller workload held with zero overruns, the engine overhead per tick derived from
   it, and the name of the method.
2. **Given** a release build, **When** the footprint step ends, **Then** the report gives binary
   size, peak RSS and the stack high-water mark of the loop thread.
3. **Given** any run, **When** it ends, **Then** the report gives the count of heap allocations
   made on the loop thread inside the measured window, and says that locks are not measured.

---

### User Story 5 - Compare two builds (Priority: P2)

A contributor changes something and wants to know what it cost: a new commit, the robot model in
place of a hand-set constant, a C controller in place of a Rust one, or the pre-sort switched off.
They run the same measurement on both and read the difference against the run-to-run spread.

**Why this priority**: Other modules' success criteria rely on it (specs/003-kmr-model SC-004,
specs/006-kmr-ffi SC-003, specs/002-kmr-core SC-012). SRS NFR-11 asks for overhead to be
"tracked", which means compared.

**Independent Test**: Two builds measured on the same host give one view with every figure, its
difference and the spread of the repeat runs.

**Acceptance Scenarios**:

1. **Given** two builds on the same robot and host, **When** the comparison ends, **Then** every
   figure appears side by side with its difference and the run-to-run spread of each side.
2. **Given** two reports from different hosts or different robots, **When** they are compared,
   **Then** the bench refuses and names the field that differs.

---

### User Story 6 - Track compile time (Priority: P3)

A contributor changes the type-level builder and wants to see if compile time grew.

**Why this priority**: SRS NFR-11 tracks it "since type-level builders tend to inflate it".
Useful, not blocking.

**Independent Test**: Two commits measured on the same host show the compile-time difference.

**Acceptance Scenarios**:

1. **Given** a commit, **When** the compile-time step runs, **Then** the report gives clean and
   incremental build times of `kmr_core` and of the reference controller crate, and names the
   host.
2. **Given** generated user crates with a growing number of registrations, **When** they are
   built, **Then** the report shows compile time against the registration count.

---

### User Story 7 - Measure hardware I/O per tick (Priority: P3)

An adapter contributor wants the per-tick I/O cost of a real actuator chain and the cost of the
write pre-sort, so the open question of SRS section 8 ("confirm on the NFR-1 bench that it pays
for itself") gets an answer.

**Why this priority**: SRS FR-5.3 and section 8 send this to the bench. It needs a reference chain
(FR-015) and an engine that drives hardware (delivery steps 1 and 3), so it comes last.

**Independent Test**: One hardware run and one simulated run on the same robot give per-tick I/O
cost, with read ticks and write-only ticks apart.

**Acceptance Scenarios**:

1. **Given** the reference chain is wired and bound with `Bus::by_id(..)`, **When** the 1 kHz run
   ends, **Then** the report gives per-tick I/O cost next to engine overhead, read ticks apart.
2. **Given** kmr reports that the plugins are declared but the hardware is not driven yet (until
   delivery step 1, kmr_api FR-026), **When** the run ends, **Then** the report is marked
   "simulated" and holds no I/O figure.

---

### Edge Cases

- The board throttles (under-voltage or temperature) during a run: the report marks the run
  "throttled" and keeps it out of comparisons.
- The kernel or board revision does not match SRS NFR-1: the run is marked "not reference".
- run() returns an error (bad `dt`, duplicate bus, bring-up error such as a wrong serial id): the
  report says "failed" with the error and holds no figures.
- Ctrl-C or an emergency stop ends a run early: the report says "incomplete" and gives the tick
  count reached.
- The ceiling search finds no failing rate down to the 10 µs floor: the ceiling is reported as
  "at least 100 kHz".
- On a simulated run, the bench's own overrun count differs from the engine's: the run is marked
  "inconsistent".
- The "no bus plugin: simulating" warning appears zero times or more than once on a run with no
  bus plugin: the run is marked "inconsistent".
- The descriptor or preset named in `robot!` has an N other than the engine's: the bench does not
  build (today's compile-time check in `robot!`). No report.
- A heap allocation shows up on the loop thread inside the measured window: the figure is
  reported and flagged. The run still completes.
- The calibrated workload of the budget search drifts by more than its stated tolerance between
  the start and the end of a run: the overhead figure is marked "uncalibrated".

## Requirements *(mandatory)*

### Functional Requirements

**Reference bench and metadata**

- **FR-001** (SRS NFR-1, NFR-8): The reference bench is a Raspberry Pi 4 with the revision the SRS
  calls "CPU rev 1.4 or 1.5", Arch Linux ARM, kernel 6.14.x or later in the 6.x line (not 7.x),
  once non-RT and once PREEMPT_RT. The bench MUST read the revision the board reports. A run on
  any other host MUST be marked "not reference".
- **FR-002** (SRS NFR-1, NFR-8): Every report MUST record: board revision, full kernel version and
  whether it is PREEMPT_RT, CPU governor, CPU affinity and scheduling policy of the run, supply
  voltage and ambient temperature with the instruments that measured them, throttle flags and SoC
  temperature at start and end, commit id, build profile and enabled Cargo features, the robot
  named in `robot!` and its N, the bus plugins declared, whether kmr simulated, `dt`, the read
  cadence the engine exposes (hardware runs), the controller set, the load profile and the run
  length.
- **FR-003** (SRS NFR-8): The bench MUST be black-box. It drives kmr only through the public user
  API, as a control engineer would: `robot!`, the builder, the controller handles and the Time
  surface. It reads no engine internals. The only exception is what the answer to FR-016 allows.

**Robot and hardware**

- **FR-004** (SRS FR-6.1): Each bench binary MUST name its robot once with `kmr_api::robot!`, as a
  preset or a descriptor path. The simulated and hardware runs of one comparison MUST name the
  same robot, so N and actuator order match.
- **FR-005** (SRS FR-6.3): Hardware runs MUST bind ports with `.add_plugins(Bus::by_id(..))`.
  `Bus::by_dev(..)` is not used on the reference bench, since `/dev/ttyUSB*` numbers change across
  reboots.
- **FR-006** (SRS FR-6.3): Engine-only runs MUST declare no bus plugin. kmr then runs on the
  core's no-hardware adapter, which prints exactly one warning, "no bus plugin: simulating"
  (kmr_core FR-042). The bench MUST check that this warning appeared exactly once and mark the run
  "simulated". A simulated run MUST NOT report any I/O figure. Until delivery step 1 lands, a run
  with a plugin gets `kmr_api`'s "plugins declared but hardware not driven yet" message instead
  (kmr_api FR-026); that run is "simulated" too.

**Measurement set**

- **FR-007** (SRS NFR-8 a, FR-3.1): The bench MUST measure the tick-rate ceiling: the highest rate
  held with zero overruns for the whole hold window, with controllers whose bodies do nothing and
  no bus plugin. It MUST say whether the 10 µs `dt` floor is held.
- **FR-008** (SRS NFR-8 b, NFR-1): At `dt` = 1 ms, the bench MUST report jitter p50, p99 and max,
  and the overrun count, on a non-RT and on a PREEMPT_RT kernel, with the same binary. On hardware
  runs, ticks with a hardware read and ticks without one MUST be reported apart (SRS FR-3.4: the
  read comes before `First`).
- **FR-009** (SRS NFR-8 c): The bench MUST repeat FR-008 under CPU and I/O load from stress-ng.
  The exact load command MUST be in the report.
- **FR-010** (SRS NFR-8 d, NFR-11): The bench MUST report engine overhead per tick by the budget
  method. A probe controller runs a calibrated workload of known length each tick. The largest
  workload held at 1 kHz with zero overruns for the hold window is the controller budget.
  Engine overhead per tick is `dt` minus that budget. The report MUST name the method and give the
  calibration. It MUST also give the gap from the end of `Last` to the start of the next `First`
  at the ceiling rate, as a second view.
- **FR-011** (SRS NFR-8 e): The bench MUST report binary size, peak RSS and the stack high-water
  mark of the loop thread, for a release build.
- **FR-012** (SRS NFR-1): The bench MUST count the heap allocations made on the loop thread inside
  the measured window, with an allocator its own binary installs, and report the count. A count
  above zero MUST be flagged. Each report MUST say that locks are not measured.

**Compile time**

- **FR-013** (SRS NFR-11, section 8): The bench MUST report clean and incremental compile times of
  `kmr_core` and of the reference controller crate (`controllers/`), in dev and release profiles.
  The reference figures are measured natively on the reference bench (SRS section 8: "hardware
  always the RP4 setup"). Figures from another host MAY be added, labelled with that host, and
  are compared only within it.
- **FR-014** (SRS NFR-11): The bench MUST report the compile time of generated user crates with a
  growing number of registrations, so the growth of the type-level builder shows.

**Hardware I/O**

- **FR-015** (SRS FR-5.3, FR-5.10, NFR-1, section 8): The bench MUST measure the per-tick I/O cost
  on a reference actuator chain: the budget of FR-010 on a hardware run, subtracted from the same
  budget on a simulated run of the same robot. [NEEDS CLARIFICATION: which chain is the
  reference? SRS NFR-1 names only the host. Options: (A) the `XmBench` preset already shipped
  (four XM430-W210 on one bus, 1 Mbaud), plus one named USB adapter model, added by the human to
  the NFR-1 definition; (B) another chain the human names; (C) no hardware runs in this version.
  Recommended: A.]
- **FR-016** (SRS FR-5.3, section 8): The bench MUST report the CPU cost of the write pre-sort and
  whether it pays for itself in I/O. [NEEDS CLARIFICATION: a black-box bench cannot switch the
  pre-sort off, so it cannot isolate the gain SRS FR-5.3 says "is measured on the NFR-1 bench".
  Options: (A) pure black-box: report the cost only (budget with every actuator written versus
  none) and leave the gain unproven; (B) the human adds an opt-in, off-by-default `kmr_core`
  feature that disables the pre-sort, and the bench compares the two builds with FR-022 on the
  reference chain; (C) move the gain to a white-box benchmark in `kmr_core` or `kmr_adapters`.
  Recommended: B, since only B answers the section 8 question on the bench.]
- **FR-017** (SRS NFR-11): Each hardware report MUST show engine overhead next to the per-tick
  I/O cost of the chain, without a verdict. The SRS gives no number for "keep up with the
  fastest bus or sensor I/O".

**How the bench behaves**

- **FR-018** (SRS NFR-1): The bench's own controllers MUST keep the control-path rules: no heap,
  no `dyn`, no lock, no file or console I/O inside a tick. Timing samples go to fixed-size
  storage. Results are written only after run() returns.
- **FR-019** (SRS FR-3.3): The bench MUST read the tick index, overrun count, last overrun,
  accumulated overrun, elapsed time and `dt` from the Time surface. On simulated runs its own
  overrun count MUST equal the engine's. On hardware runs the engine's count is reported and the
  bench's count is a lower bound, since the bulked write after `Last` is out of its sight.
- **FR-020** (SRS FR-4.1): The bench MUST end each run with `shutdown!` from its own controller
  once the planned tick count is reached.
- **FR-021** (SRS NFR-8): The bench MUST ship a written procedure with one command per
  measurement and its full configuration in the repository: OS setup, kernel configuration for
  both kernels, governor, scheduling setup, load profile, instruments and run lengths. A run then
  needs nothing from the authors.
- **FR-022** (SRS NFR-11, NFR-8): The bench MUST compare two builds or two controller sets with
  the same method, on the same robot and host, and show every figure, its difference and the
  run-to-run spread of each side. It MUST refuse to compare reports whose host, robot or kernel
  type differ.
- **FR-023** (SRS NFR-11, NFR-1): Reports MUST be kept per commit in a plain, diffable form. The
  bench records only: it MUST NOT pass or fail a build, since the SRS defers every figure.
- **FR-024** (SRS NFR-2): A run that ends in an error, an emergency stop, a Ctrl-C or with missing
  metadata MUST produce a report marked "failed" or "incomplete", never partial figures shown as
  complete.
- **FR-025** (SRS NFR-5): The bench MUST cross-build for x86-64, aarch64 and armv7 Linux without
  the robot plugged in, and the same binary MUST run on both kernels of the reference bench.
- **FR-026** (SRS FR-7.1, section 4 Constraints): The bench MUST offer FR-008 with the `kmr_api`
  Zenoh feature on, recorded in the report. The default build leaves it off.
- **FR-027** (SRS NFR-1, NFR-8): The bench MUST NOT need any change to the builder surface or to
  the engine, beyond what the SRS already requires (the Time surface of FR-3.3) and what the
  answer to FR-016 decides. It asks nothing of `kmr_api/examples/complete_api_example.rs`, which
  may change with the API and must build and run (kmr_api FR-069).

### Key Entities

- **Reference bench**: the host of SRS NFR-1. Board, revision, OS, kernel, RT or not.
- **Reference chain**: the actuators wired to the bench for hardware runs, named by a preset
  (FR-015).
- **Run**: one execution of one measurement on one host. Has a status: complete, incomplete,
  failed, throttled, not reference, inconsistent, simulated, uncalibrated.
- **Report**: the figures of one run plus its metadata (FR-002). Diffable, kept per commit.
- **Comparison**: two sets of reports from the same host and robot, with differences and spread.
- **Load profile**: the stress-ng command used for SRS NFR-8 c.
- **Probe controller**: a bench controller that timestamps ticks into fixed-size storage and can
  run a calibrated workload.
- **Reference controller crate**: the user crate whose compile time is tracked (`controllers`).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001** (SRS NFR-8): A person who did not write the bench produces a complete set (a) to (e)
  from the written procedure alone, within 2 hours of a flashed board.
- **SC-002** (SRS NFR-8 b): Three repeat runs on the same setup give p50 and p99 jitter within
  10 % of each other. Max is reported, not compared.
- **SC-003** (SRS NFR-8 d, NFR-11): Three repeat budget searches give engine overhead within 10 %
  or 2 µs of each other, whichever is larger.
- **SC-004** (SRS NFR-8 a to e): 100 % of complete reports hold all five measurements and every
  metadata field of FR-002.
- **SC-005** (SRS FR-3.3): In 100 % of complete simulated runs, the bench's overrun count equals
  the engine's.
- **SC-006** (SRS FR-6.3): 100 % of runs with no bus plugin are marked "simulated" and 0 of them
  report an I/O figure.
- **SC-007** (SRS NFR-1): 100 % of reports give the loop-thread allocation count of the measured
  window.
- **SC-008** (SRS FR-3.1): The first reference report states whether the 10 µs floor is held.
- **SC-009** (SRS NFR-11): Comparing two commits shows the difference of every figure, compile
  time included, in one view.

## Assumptions

- **Jitter** is the lateness of each tick's start against its absolute deadline grid
  `origin + k*dt` (SRS FR-3.2), seen by a probe at the start of `First`. The bench anchors the
  grid with the Time surface (tick index, elapsed time). On simulated runs no hardware read comes
  before `First`.
- **Overrun deadline**: tick k overruns when it finishes after the start of slot k+1. The SRS
  wording "finishes after its absolute deadline `origin + k*dt`" is read that way (see SRS
  issues).
- **Run length**: each 1 kHz cell (kernel × load) lasts 10 minutes, 600 000 ticks. Each ceiling
  and budget candidate is held for 60 s. Recorded in the report, changeable.
- **"Empty controllers"** in SRS NFR-8 a means controllers with empty bodies: one no-op controller
  in each per-tick phase, plus the probe.
- **Load profile**: one fixed stress-ng command (all cores CPU, plus disk I/O) lives in the
  procedure. Its exact text is part of each report.
- **Kernel setup**: the procedure fixes the governor ("performance"), the scheduling policy and
  the CPU affinity of the run. Core isolation is off by default. Each is recorded (FR-002).
  PREEMPT_RT is a mainline kernel option since 6.12, so the RT kernel is the same source with a
  recorded configuration.
- **Supply voltage and ambient temperature** come from external instruments (a USB power meter,
  a room thermometer), named in the report. The board only reports throttle flags and SoC
  temperature; both are recorded too.
- **The robot**: every bench binary names the same robot, the one of FR-015. Until `kmr_core` takes
  N as a const generic, `robot!` only builds when that robot's N equals the engine's (4 today,
  which `XmBench` matches).
- **Read cadence**: the hardware table declares it per bus and the engine exposes it per actuator
  (SRS FR-5.3, kmr_core FR-045). The bench records it and does not set it.
- **What "simulated" means**: the no-hardware adapter seeds nothing, so History reads return the
  typed "no sample" error (kmr_core FR-042; recommended, pending human confirmation). Probe
  controllers read only the Time surface, so they do not depend on samples.
- **Generated crates for FR-014**: 1, 16 and 64 registrations. Changeable, recorded.
- **Stack high-water mark** is for the loop thread only; threaded controllers are not spawned.
- **Heap**: the bench needs none. It follows the workspace ban on heap types; the counting
  allocator of FR-012 counts and does not allocate.
- **Results** before the real-time sleep lands (ROADMAP) measure today's sleep. Each report names
  its commit, so later runs are comparable.
- **Hardware runs** wait on SRS delivery steps 1 and 3; until step 1, `run()` simulates even with a
  plugin declared, and FR-006 marks those runs "simulated".
- The bench runs by hand on the board. A self-hosted CI runner is out of scope.

## Module boundaries

**kmr_core (human-only)**

- Needs: the counters behind SRS FR-3.3 (no tick index or overrun count in `clock.rs` today); the
  `dt` range check of SRS FR-3.1; the real-time sleep; the no-hardware adapter and its one
  "no bus plugin: simulating" warning (kmr_core FR-042); if FR-016 takes option B, an opt-in
  feature that disables the pre-sort.
- Provides to it: the figures for its SC-012 (overhead per release), the FR-3.1 floor feasibility
  and the FR-5.3 pre-sort cost.

**kmr_api**

- Needs: `robot!` with a preset or a descriptor path (SRS FR-6.1); `.add_plugins(Bus::by_id(..))`
  (SRS FR-6.3); until delivery step 1, the "plugins declared but hardware not driven yet" message
  (kmr_api FR-026); the public Time surface of SRS FR-3.3
  (`kmr_api::Time` has no public method today); the Zenoh feature for FR-026.
- Provides to it: the bench host and a gateway-on run for its SC-003 (allocation half) and SC-007.
  The subscriber and the miss count stay kmr_api's.

**kmr_adapters**

- Needs: the reference chain's preset with its hardware data (SRS FR-6.1), the declared read
  cadence (SRS FR-5.3) and the FR-5.10 latency setting.
- Provides to it: the host and per-tick I/O cost for its SC-003. Its SC-002 transaction count is
  its own test.

**kmr_model**

- Needs: the descriptor or preset behind `robot!`.
- Provides to it: SC-004 (overhead with the model versus a hand-set constant, via FR-022) and
  SC-005 (build time, via FR-013). Its SC-005 names the x86-64 dev machine; the bench gives that
  as a labelled non-reference figure (FR-013).

**controllers**

- Needs: the reference controller crate whose compile time is tracked.
- Provides to it: nothing. The FR-10.4 metric is controllers' own.

**kmr_ffi**

- Needs: nothing.
- Provides to it: the comparison of FR-022 for its SC-003 (empty C controller against an empty
  Rust one). The FR-10.4 protocol its SC-008 cites belongs to `controllers`, not the bench.

**Workspace**

- Needs: a `bench` member in the root `Cargo.toml` (not there today), under the workspace heap
  ban.

## SRS coverage

| SRS id | Spec ids | Gap |
| --- | --- | --- |
| NFR-1 (bench) | FR-001, FR-002, FR-012, FR-018, FR-027, SC-002, SC-007 | Target figures deferred by the SRS |
| NFR-8 (reproducible, black-box) | FR-002, FR-003, FR-021, FR-022, SC-001, SC-004 | Setup beyond NFR-1 set in the procedure |
| NFR-8 a | FR-007, SC-004, SC-008 | Hold window is an assumption |
| NFR-8 b | FR-008, SC-002 | Jitter definition is an assumption |
| NFR-8 c | FR-009 | Load profile is an assumption |
| NFR-8 d | FR-010, SC-003 | None (budget method) |
| NFR-8 e | FR-011 | None |
| NFR-11 overhead | FR-010, FR-017, FR-022, FR-023 | "Keep up with the fastest I/O" has no number |
| NFR-11 compile time | FR-013, FR-014, SC-009 | None |
| Section 8 (RP4 always) | FR-001, FR-013 | None |
| FR-3.1 floor feasibility | FR-007, SC-008 | None |
| FR-3.3 (consumed) | FR-019, SC-005 | Surface missing in `kmr_api` and `kmr_core` today |
| FR-3.4 (tick order) | FR-008, FR-019 | None |
| FR-4.1 (consumed) | FR-020 | None |
| FR-5.3 I/O, pre-sort, section 8 | FR-015, FR-016, FR-017 | FR-015 and FR-016 need decisions |
| FR-5.10 (latency lesson) | FR-015 | Waits on FR-015 |
| FR-6.1 (robot!) | FR-004 | N tied to the core's const until const generic |
| FR-6.3 (plugins, simulate) | FR-005, FR-006, SC-006 | None |
| FR-7.1 (Zenoh feature) | FR-026 | None |
| NFR-2 | FR-024 | None |
| NFR-5 | FR-025 | None |
| NFR-6 | Not a spec id | Decisions go to `docs/decisions/` at plan time |
| NFR-9 benchmarks | Out of scope | Owned by the `kmr_core` module |

## SRS issues (reported, not fixed here)

- NFR-8 "black-box" against FR-5.3 "the gain is measured on the NFR-1 bench": the gain cannot be
  isolated black-box (FR-016).
- NFR-1 names only the host: no actuator chain or USB adapter, so I/O figures are not reproducible
  (FR-015).
- NFR-1 "CPU rev 1.4 or 1.5": the Pi 4 reports this as its board revision, not a CPU revision.
- NFR-1 kernel "6.14.x or later in the 6.x line" on a rolling distro, and no governor, scheduling
  or isolation setup: resolved by recording them and fixing them in the procedure.
- NFR-8 gives no run length, hold window, jitter definition or load profile: resolved by
  Assumptions.
- NFR-11 "keep up with the fastest bus or sensor I/O" has no number: reported side by side
  (FR-017).
- Definitions "Overrun": "finishes after its absolute deadline `origin + k*dt`" reads as the
  tick's own start; the bench reads it as the start of the next slot.
- Section 4 heap rules name only `kmr_core` and `kmr_api`, not the bench.
- Section 8 delivery order has no step for the bench, while FR-3.3 (step 4) is what it reads.
