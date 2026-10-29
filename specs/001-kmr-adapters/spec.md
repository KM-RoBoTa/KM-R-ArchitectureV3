# Feature Specification: kmr_adapters, hardware adapters (Dynamixel first)

**Feature Branch**: none created (spec directory fixed: `specs/001-kmr-adapters`; work sits on
`agent/robot-macro`)

**Created**: 2026-10-26 · **Re-specified**: 2026-10-28, against the SRS patched 2026-10-27

**Status**: Draft

**Input**: User description: "Re-specify kmr_adapters against the SRS as patched on 2026-10-27
(docs/srs/modules/kmr_adapters.md)."

**Authority**: `docs/srs/SRS.md`, module `kmr_adapters`, delivery step 1 ("HAL contract and one
adapter"). On any conflict the SRS wins.

**Id convention**: `FR-0NN` / `SC-0NN` are this spec's ids, written next to their SRS source:
`FR-016 (SRS FR-4.1)`. Ids kept from the 2026-10-26 spec keep their number when the meaning
holds; changed meaning is said in Clarifications; retired ids are not reused.

## Scope

This module is the **adapter side** of the hardware layer, inside the `kmr_adapters` crate:

- one adapter per vendor, implementing the engine's sealed HAL contract (`kmr_core/src/hal.rs`)
  for the actuators it owns; a robot may hold several, each one member of its driver list;
  Dynamixel first;
- the **hardware half** of each KM-RoBoTa preset, and the types a `main.rs` declaration and its
  overrides are built from;
- resolving the ports that bus plugins name, at bring-up;
- unit conversion, per-mode hold, and fault bits for its actuators.

**Not here**: the HAL contract itself and its call points (`kmr_core`, human-only); `robot!`,
the `Bus` plugin type and the builder (`kmr_api`); descriptors and the model half of presets
(`kmr_model`); other vendors (CAN, CubeMars MIT mode); sensors (SRS FR-5.7, surface WIP); payload
hardware (SRS FR-8.2); runtime operating-mode override (SRS section 8, open).

## Clarifications

### Session 2026-10-29 (health limits and error data, D18)

- Health limits are the actuator's own (human decision): PostInit reads each actuator's Min and
  Max Voltage Limit and Temperature Limit from its control table and checks the present values
  against them. The model's datasheet range is documentation and one warning per value, never the
  check. On the RX-28 bench the actuators read 11.0–11.8 V, inside their limits 6.0–19.0 V, below
  the 12.0 V of the datasheet. FR-006 reworded.
- Every error says what went wrong with its data (SRS NFR-2, FR-007): the value read and the
  limit or range it broke, the id and the bus; a backend error names its cause, `Io` only a failed
  serial transaction; `kmr_api` keeps the adapter error typed as the source of its bring-up error.
- D18 (future, decided 2026-10-29): limits configurable through the API, see Roadmap.

### Session 2026-10-29 (set of drivers, D17)

- A robot mixes any number of vendors. An adapter is one member of the robot's driver list and
  claims its actuator indices at `check`; the engine merges the claims (kmr_core FR-063). New
  FR-036; FR-002, FR-024, Assumptions and boundaries reworded. Design:
  `specs/002-kmr-core/design-hal-set.md`. Placement in the core assumed, pending human.

### Session 2026-10-28 (coordination)

- Cross-module coordination fixes, so each fact has one owner. FR-031: the one simulation warning
  is kmr_core FR-042's; "no fallback to simulation with a plugin" holds from delivery step 1.
  Torque choice and read cadence are hardware-table fields (FR-023, FR-026), overridden only in
  `main.rs`; kmr_core's run-time overrides carry the plugin baud only (kmr_core FR-039, FR-045).
  FR-024: the count check is `robot!`'s (kmr_api FR-007), an array-type check. FR-027:
  const-evaluated messages cannot use `format!` (E0015), but a message built byte by byte in a
  const fn names the index (proved on rustc 1.97.1: "actuator 2: id above 252"). US4-AS3
  cites SRS FR-5.10, not FR-3.2. Hold target cites kmr_core FR-032. Direct adapter access is
  kmr_core FR-050 option A. Q1 / FR-030: bus label with `.bus("label")` (recommended, pending
  human confirmation). Bus-side watchdog listed as an SRS gap. Boundaries fixed to match.

### Session 2026-10-28 (resolved from the SRS patch of 2026-10-27, no question asked)

- Hardware is no longer "adapter layer only". It comes from the preset or from `main.rs`; any
  preset default can be overridden there (SRS FR-6.1). Old FR-020 is retired.
- Ports are builder plugins, `.add_plugins(Bus::by_id(..))` preferred, `Bus::by_dev(..)` allowed
  (SRS FR-6.3, section 7 User API). The per-machine binding file and the `PORTS` const are gone.
- N comes from the descriptor through `kmr_api::robot!`, which emits `N`, `State`, `Robot`
  (SRS FR-6.1, section 4 Assumptions). The hardware half is checked against that N.
- Without any bus plugin, run() simulates and warns once (SRS FR-6.3).
- Units: prismatic is mm, mm/s, N, not m (SRS FR-2.2). FR-021 changed.
- Hold lasts `Option<Duration>`, `None` by default (SRS FR-4.1). FR-016 changed.
- The per-field write mask is now SRS text (Definitions, Write mask), no longer a deviation.
- The example now names `robot!(XmBench)` and a bus plugin. Old FR-019 ("example frozen") is
  retired.

### Session 2026-10-26 (kept where the SRS still agrees)

- Unit helpers such as degrees and cm belong to `kmr_api` (SRS FR-2.2).
- Hold per mode was answered for Dynamixel: position holds, velocity 0, current 0; then torque
  off (FR-016).

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Move a preset robot from a controller (Priority: P1)

A control engineer writes `kmr_api::robot!(XmBench)` and one bus plugin in `main.rs`. They plug
the robot in, run, and every actuator follows the targets their controller writes.

**Why this priority**: SRS section 8 delivery order: "nothing below a line starts before the line
above moves a motor". This is that motor.

**Independent Test**: With the bench chain plugged in, a controller writes a slow sine to every
actuator's position. The read positions follow it within the model's own resolution.

**Acceptance Scenarios**:

1. **Given** `robot!(XmBench)` and `.add_plugins(Bus::by_id(serial))`, **When** run() starts,
   **Then** the adapter uses the preset's brand, protocol, baud, ids and modes and the plugin's
   port, and no other hardware line is needed (SRS FR-6.1, FR-6.3).
2. **Given** all N actuators in position mode, **When** a controller writes `Q` for all N in one
   tick, **Then** one bulked write goes out per (bus, series) (SRS FR-5.3).
3. **Given** a controller writes `Q` for 3 of N actuators, **When** the tick ends, **Then** only
   those 3 get a command (SRS FR-2.5).
4. **Given** a read cadence of K ticks, **When** a read tick comes, **Then** every actuator of
   the bus is refreshed, in one transaction per (bus, series) on Protocol 2 (SRS FR-5.3, FR-2.1).
5. **Given** a position-mode actuator, **When** a controller writes only its `Qd`, **Then**
   nothing is sent to it and its `rejected` bit is set (SRS Definitions, Write mask).

---

### User Story 2 - Safe bring-up and shutdown (Priority: P1)

The engine brings the hardware up through the adapter in the once-phases and takes it down on
shutdown or emergency stop. The user writes none of it.

**Why this priority**: an actuator left powered or a bus left open is a safety fault; v2 lost
hours on these exact issues (SRS FR-5.10, NFR-2).

**Independent Test**: Run, raise `shutdown!`; run again, raise `emergency_stop!`. Each time every
actuator ends torque-off, the port is closed, and the next run opens it cleanly.

**Acceptance Scenarios**:

1. **Given** the robot is wired, **When** Init runs, **Then** every port is resolved and
   registered before any is opened, each is flushed before open, and the USB latency timer reads
   back 1 ms (SRS FR-5.10).
2. **Given** Init succeeded, **When** PostInit runs, **Then** every actuator is pinged,
   health-checked, read once, its mode registered, and torque set per actuator: on by default,
   off for read-only ones, which are still read (SRS FR-5.2, FR-5.8).
3. **Given** a running loop, **When** `shutdown!` is raised, **Then** the adapter sends one hold
   command per powered actuator, and torque goes off after the user's hold duration, right away
   when it is `None` (SRS FR-4.1, FR-5.6).
4. **Given** a running loop, **When** `emergency_stop!` is raised, **Then** torque goes off on
   every actuator, read-only ones included, at the next phase boundary (SRS FR-4.2).

---

### User Story 3 - Custom robot and overrides in main.rs (Priority: P2)

An integrator names their own descriptor with `robot!(model = "arm.urdf")` and declares its
hardware in `main.rs`. Another one keeps a preset but changes one value, such as the baud or a
read-only actuator.

**Why this priority**: SRS FR-6.1 allows both, and no other file may carry hardware data.

**Independent Test**: Build a custom 4-actuator robot declared in `main.rs`; build the preset
with one override; build a declaration with 3 actuators against a 4-actuator descriptor.

**Acceptance Scenarios**:

1. **Given** a descriptor and a `main.rs` declaration of the same N, **When** built, **Then** it
   builds and runs exactly like a preset with the same data (SRS FR-6.1).
2. **Given** a preset and one override, **When** run, **Then** only the overridden value differs
   from the preset (SRS FR-6.1).
3. **Given** a hardware declaration of 3 actuators for a descriptor of 4, **When** built,
   **Then** the build fails and rustc's array-size error gives both counts (SRS FR-6.4,
   kmr_api FR-007).
4. **Given** `Bus::by_dev("/dev/ttyACM0").baud(3_000_000)`, **When** a model on that bus does not
   accept that baud, **Then** run() returns an error naming the bus and the model, before any
   I/O (SRS FR-6.3, FR-6.4).

---

### User Story 4 - Clear faults instead of silent failures (Priority: P2)

When something on the bus goes wrong, the engineer sees a typed error or fault bit, never a
crash, a hang or a zeroed reading.

**Why this priority**: SRS FR-13.4 "No fault is silent"; SRS NFR-2: never panicking is what keeps
torque safe.

**Independent Test**: Start with one actuator missing; unplug one mid-run; inject a failed read
and a failed write on a scripted bus. Each case gives its named error or bit, and the loop keeps
its grid.

**Acceptance Scenarios**:

1. **Given** an actuator declared but absent, **When** PostInit pings it, **Then** run() returns
   a readable bring-up error naming the actuator index, bus, firmware id and port (SRS FR-5.5,
   FR-6.4).
2. **Given** a running loop, **When** a read fails for one actuator, **Then** its last good sample
   is kept, never zeros, and its `stale` bit is set; the tick is not retried or delayed (SRS
   FR-2.6, FR-5.10).
3. **Given** a running loop, **When** a write transaction fails, **Then** `bus_error` is set for
   the actuators of that transaction and the next tick writes again (SRS FR-13.4, FR-5.10).

---

### User Story 5 - Build and simulate without hardware (Priority: P3)

A contributor cross-builds the robot binary for the Raspberry Pi with nothing plugged in. A
control engineer runs the example on a laptop with no bus plugin.

**Why this priority**: SRS FR-5.5, FR-6.3, NFR-5; cheap to keep, costly to retrofit.

**Independent Test**: Cross-build for aarch64 and armv7 on a machine with no serial device. Run
the example with its plugin line removed.

**Acceptance Scenarios**:

1. **Given** no actuator connected, **When** the user builds for any supported target, **Then**
   the build succeeds; presence is checked only at bring-up (SRS FR-5.5).
2. **Given** no bus plugin, **When** run() starts, **Then** no port is opened, the loop runs
   simulated, and exactly one warning says so (SRS FR-6.3).

### Edge Cases

- Two actuators on one bus share a firmware id → compile error naming both (SRS FR-6.4).
- A `by_id` serial matches no entry, or more than one, under `/dev/serial/by-id` → bring-up
  error naming the bus and the serial (SRS FR-6.3, FR-6.4).
- Two plugins resolve to the same device → bring-up error naming both buses (SRS FR-6.4).
- A table bus with no plugin while other buses have one, or a plugin with no table bus → error
  before any I/O (SRS FR-6.4; mapping per FR-030).
- Bus plugins given, but the table's adapter is not compiled into the binary → run() error, never
  silent simulation: from delivery step 1, simulation is for "without any bus plugin" only (SRS
  FR-6.3).
- The port exists but the SDK would crash on it → pre-flight rejects it before the SDK is entered
  (SRS FR-5.10).
- Mixed series on one bus → one transaction per (bus, series), never a mixed bulk (SRS FR-5.3).
- Protocol 1 and 2 actuators on one bus → each uses its own protocol; a baud one of them rejects
  is an error before I/O (SRS FR-6.4).
- A target converts outside the model's register range, or is not finite → not sent, `rejected`;
  joint limits proper stay with the engine (SRS FR-4.5, open).
- A write to a read-only actuator → not sent, `rejected` (SRS FR-5.8, FR-13.4).
- Torque on over a goal an earlier session left (2026-10-29) → `torque_on` first writes, torque
  still off, Goal Position = present position read then (Goal Velocity, Goal Current = 0) and
  powers nothing if that read or write fails: no jump (FR-006 (SRS FR-5.2, FR-5.8)). `dxl_rs`
  takes no goal before Torque Enable, so `DxlRs` writes it one write after.
- A prismatic actuator on a Dynamixel model → compile error "SRS FR-2.2: prismatic Dynamixel
  actuator has no mm conversion" (FR-021).
- Velocity or current mode asked for → compile error naming SRS section 8 (FR-018).
- **Parked decision, not resolved here**: an actuator not written this tick gets no command (SRS
  FR-2.5), but a Dynamixel drive latches its last setpoint. In velocity or current mode the old
  command keeps acting, and no SRS FR-13.4 flag reports it. It bites FR-012 the day FR-018 opens
  those modes. Position mode is unaffected (a latched position is a hold).
- Several actuators driving one joint → not the adapter's concern; it indexes actuators only
  (SRS section 3, Index space).
- A transaction takes longer than the tick → the engine counts the overrun; the adapter never
  sleeps or retries on the hot path (SRS FR-3.2, FR-5.10).

## Requirements *(mandatory)*

### Functional Requirements

**Contract conformance**

- **FR-001** (SRS FR-5.1, section 7 HAL): Each adapter MUST implement the whole HAL contract of
  `kmr_core/src/hal.rs`: check, open, zero, ping, health, configure, read_all, torque_on, read,
  write, hold, torque_off, close. The contract is read-only here; a change it needs goes to the
  human.
- **FR-002** (SRS section 3 Index space, FR-5.4): An adapter MUST be addressed by actuator index
  `0..N` only, and only at the indices it owns (FR-036). It MUST NOT know joints, groups or controllers. A bus is a property of each
  actuator.
- **FR-003** (SRS FR-5.1, FR-11.3): Only `kmr_adapters` MUST depend on a vendor SDK. Each vendor
  is its own adapter; adding one changes no other.
- **FR-036** (SRS FR-5.1, FR-5.4): An adapter MUST be one member of the robot's driver list. Its
  `check` MUST return the indices it owns, their cadence and local groups, and its costs, never a
  whole-N layout; the engine merges the members (kmr_core FR-063). It MUST fill, flag and send
  only its owned slots of the shared sample, faults and command. One adapter may own several
  buses (FR-018).
- **FR-022** (SRS FR-6.1, section 4 Assumptions): An adapter MUST work for the N that `robot!`
  fixes and MUST NOT carry an actuator count of its own.

**Hardware data: presets and main.rs**

- **FR-023** (SRS FR-6.1): Each KM-RoBoTa preset MUST have a hardware half in `kmr_adapters`,
  under the preset's name. Per bus: label, brand, protocol, baud, read cadence. Per actuator:
  bus, firmware id, model, operating mode, torque choice, zero (where 0 rad sits, FR-005, Q3).
- **FR-024** (SRS FR-6.1, FR-6.4): A hardware half MUST list exactly N actuators, in the model's
  actuator order (several brands: the members' tables together, checked by kmr_core FR-063).
  Any other count is a compile error, raised by `robot!`'s array-type check (kmr_api FR-007):
  rustc's array-size error gives both counts. The table types MUST allow that check, a
  fixed-size array of actuators.
- **FR-025** (SRS FR-6.1): A robot without a preset MUST be declarable in `main.rs` with the same
  data, built from the same types. A preset is such a declaration, written by KM-RoBoTa.
- **FR-026** (SRS FR-6.1): Every preset default MUST be overridable from `main.rs`: baud,
  firmware id, mode, torque choice, read cadence, zero (the per-actuator zero offset; D8 C,
  assumed, pending human). Mode, torque choice and zero MUST also be overridable for a group
  slice, an index array, in one call (kmr_api FR-020 (SRS FR-5.8)); firmware id has no group
  form, one id per actuator. An overridden table passes the same checks; an index past N, in a
  group too, is a compile error naming it.
- **FR-027** (SRS FR-6.4, FR-5.5): Hardware data known at compile time MUST be checked at compile
  time: bus index in range, firmware id valid for the protocol, ids unique per bus, mode
  supported by the model, baud accepted by every model on its bus. Each error names the failed
  check. A const-evaluated check cannot use `format!` (E0015); it builds its message byte by byte
  in a const fn, which names the actuator index (as `config.rs` does today). Data given only at run time, such as a plugin baud, MUST be checked at run()
  finalization, before any I/O; that error names the actuator index. No check needs hardware
  plugged in.
- **FR-028** (SRS FR-6.1): Hardware data MUST be Rust compile-time data. No config file, no file
  parser, no `build.rs`.

**Port binding**

- **FR-029** (SRS FR-6.3, FR-5.5): Each bus MUST take its port from a bus plugin and resolve it
  at bring-up (Init), never at compile time. `by_id`: the single `/dev/serial/by-id` entry
  holding that serial. `by_dev`: the path as given.
- **FR-030** (SRS FR-6.3, FR-6.4): A plugin MUST bind to exactly one bus of the hardware table,
  by label: each bus has a label in the table, and the plugin names it with `.bus("label")`. The
  label is optional for a one-bus robot (Q1; recommended, pending human confirmation). An
  unbound bus, an unknown label or an unmatched plugin MUST be a run() error before any I/O. Two
  plugins on one device MUST be a bring-up error.
- **FR-031** (SRS FR-6.3): With no bus plugin, the adapter MUST NOT open any port; the engine runs
  on its no-hardware adapter, which prints the one simulation warning (kmr_core FR-042). From
  delivery step 1, with at least one plugin, the run MUST NOT fall back to simulation.

**Bring-up**

- **FR-004** (SRS FR-5.10, FR-5.2): In Init the adapter MUST resolve and register every port
  before opening any, flush each port before open, and set the USB latency timer to 1 ms, read
  back. A timer it cannot set is an error that names the fix. A port without a USB latency timer
  skips that step.
- **FR-005** (SRS FR-5.2): In Init the adapter MUST apply brand-specific zeroing where the series
  needs it, with zero placed per [NEEDS CLARIFICATION: Q3, where 0 rad sits]. It MUST NOT power
  any actuator in Init.
- **FR-006** (SRS FR-5.2, FR-5.8): In PostInit the adapter MUST ping and health-check every
  actuator (error status, voltage, temperature against the limits the actuator holds in its own
  control table; inside those but outside the model's datasheet range, a warning), register its
  mode, read every actuator once whatever the cadence, then set torque per actuator: on by
  default, written off for a read-only one. Flagging out-of-place actuators is the engine's, on
  that read (SRS FR-2.7, open).
- **FR-007** (SRS FR-5.10, FR-6.4, NFR-2): Retries MUST happen only in bring-up and ping, and
  MUST be bounded. Every bring-up failure MUST be returned as a readable error naming the
  actuator index, bus, firmware id and port where known. Never a panic.
- **FR-008** (SRS FR-5.10): Before any SDK call that can crash the process on bad input, the
  adapter MUST check that input: the port exists, is a character device, is readable and
  writable by this process.

**Per-tick I/O**

- **FR-009** (SRS FR-5.3, FR-2.1): The read cadence MUST be a whole number of ticks K ≥ 1 per
  bus, declared by the adapter to the engine with its per-transaction cost. The cadence in force
  is the `main.rs` override, else the table's value, else [NEEDS CLARIFICATION: Q2, the
  default]. K = 0 MUST NOT be representable: the cadence is a `NonZeroU32`, so a 0 in a table
  or an override is a compile error, never a run() error.
- **FR-010** (SRS FR-5.3): On a read tick the adapter MUST issue one wire transaction per
  (bus, series) where the protocol has a grouped read (Protocol 2), one round trip per actuator
  where it has not (Protocol 1). Fields a protocol cannot read keep their last value; the adapter
  says so once, at check, per actuator.
- **FR-011** (SRS FR-5.3): The adapter MUST take the engine's pre-sorted batch and send one
  bulked write per (bus, series) run where the protocol has one, one round trip per actuator
  where it has not.
- **FR-012** (SRS FR-2.5, Definitions Write mask): An actuator MUST get a command only when the
  field its mode takes was written this tick. An actuator outside the batch gets nothing. A
  written field its mode does not take is not sent and sets `rejected`.
- **FR-021** (SRS FR-2.2): Values MUST cross the contract in SRS units: revolute rad, rad/s, N·m;
  prismatic mm, mm/s, N. The adapter converts both ways with the model's published constants;
  raw units never leave it. A prismatic actuator on a model with no linear conversion MUST be a
  compile error naming SRS FR-2.2. `Tau` on a model without a current register is never read
  and, when written, is `rejected`.
- **FR-032** (SRS FR-2.2, NFR-2, FR-13.4): A target that is not finite or converts outside the
  model's register range MUST NOT be sent and MUST set `rejected`. Never clamped silently, never
  a panic.
- **FR-013** (SRS NFR-1, FR-5.10, NFR-11): Outside the direct SDK call sites, the per-tick path
  MUST NOT allocate, lock, sleep, retry or use `dyn`.

**Faults**

- **FR-014** (SRS FR-2.6): A failed read MUST keep the last good sample and set `stale` for that
  actuator.
- **FR-015** (SRS FR-13.4): Bus errors, offline actuators and unsent commands MUST be reported as
  per-actuator bits (`bus_error`, `offline`, `rejected`), never as a log line alone.

**Shutdown and stop**

- **FR-016** (SRS FR-4.1, FR-5.6, section 8 "hold per mode"): On shutdown the Dynamixel adapter
  MUST send one hold command per powered actuator: position mode holds the target the engine
  hands it, velocity mode commands 0 rad/s, current mode commands 0 N·m. The command MUST stay
  valid without refresh for the user's hold duration, which the engine waits. Then torque off,
  flush, close: best effort, no retry, never fails.
- **FR-017** (SRS FR-4.2, FR-5.6, FR-5.8): On emergency stop, and on a bring-up error after
  open, the adapter MUST switch torque off on every actuator of every opened bus, read-only ones
  included, then flush and close. Best effort, no retry.
- **FR-033** (SRS NFR-2, FR-4.2): Outside test code an adapter MUST NOT panic: no unwrap, no
  unchecked index, no overflowing arithmetic. Once open was called, every exit it sees ends with
  torque off, then close.

**Supported hardware**

- **FR-018** (SRS FR-5.3, module "Dynamixel first", section 8 mode override): The first adapter
  MUST drive Dynamixel Protocol 1 (AX-12A, RX-28) and Protocol 2 (X series) actuators in
  position mode. Velocity and current modes MUST be a compile error naming SRS section 8 until
  the SDK can write the mode register. One table may hold several buses (SRS FR-6.5 pattern 1).

**Data layout and cost**

- **FR-034** (SRS NFR-10): Adapter data MUST follow the NFR-10 strategies: per-actuator data as
  arrays indexed by actuator; per-actuator booleans (read-only, unread fields, faults) as bitsets
  out of band; buses and groups referred to by index, never by pointer; brand, model and mode as
  plain enum encodings, never trait objects; no padding in per-tick data.
- **FR-035** (SRS NFR-11, FR-5.3): The adapter MUST declare its estimated bus time for one full
  read and one full write, so the bench can compare it with the measured time.

**Retired**: FR-019 (example frozen) and FR-020 (hardware in the adapter layer only, no port
type in `kmr_api`) contradict SRS FR-6.1, FR-6.3 and section 7 since 2026-10-27.

### Open questions

**Q1 (FR-030, SRS FR-6.3, FR-6.5): how does a plugin name the bus it binds?** Recommended B,
applied with a label, pending human confirmation; shared with kmr_api Q2. SRS FR-6.3 binds
ports in the builder; FR-6.5 allows several buses. Nothing says which plugin goes to which bus.

| Option | Answer | Implications |
| --- | --- | --- |
| A | By order: the k-th plugin binds the table's k-th bus | No name. Swapping two lines with the same baud silently swaps the buses: wrong motors move |
| B | By bus label: the table labels each bus; a plugin names it with `.bus("label")`; a one-bus robot needs no label | Checked before any I/O; `kmr_api::Bus` gains a label |
| C | By probe: at bring-up each bus binds to the port where its ids answer | No name; costs bring-up time; ambiguous when two buses use the same ids |

Recommendation: **B**.

**Q2 (FR-009, SRS FR-5.3): what cadence applies when neither `main.rs` nor the table sets one?**
SRS FR-5.3: "the adapter declares its read cadence", "Dynamixel adapters do not read every
tick". The 2026-10-26 answer said "no default"; the patched SRS does not repeat it.

| Option | Answer | Implications |
| --- | --- | --- |
| A | The adapter has a default K per brand (Dynamixel above 1); the human picks it, the NFR-1 bench confirms it | Matches "adapter declares"; needs a number before the bench |
| B | No adapter default; every table must set K, else a compile error | No number to guess; a custom declaration needs one more value |
| C | No default; a missing K is a run() error (old FR-009) | Fails late, at run(), against NFR-4 type-state |

Recommendation: **A**.

**Q3 (FR-005, SRS FR-5.2): where does 0 rad sit on a Dynamixel?** The SRS says "zeroing where
needed" and "double-check zeros" but not where zero is. A wrong zero moves every actuator to a
wrong pose.

| Option | Answer | Implications |
| --- | --- | --- |
| A | At the model's center position (2048 on X, 512 on AX/RX), for every actuator | Today's code; symmetric ranges; v2 poses are off by π (X) or 150° (AX/RX) |
| B | Raw 0 is 0 rad (v2) | v2 poses carry over; ranges are one-sided |
| C | A per-actuator zero offset in the hardware table, default the model's center | Matches each robot's mounting; one more overridable value |

Recommendation: **C**.

### Key Entities

- **Adapter**: one per vendor; one member of the robot's driver list; implements the HAL contract
  for the actuator indices it owns; owns its buses and SDK handle.
- **Hardware table**: per-bus and per-actuator hardware data, N actuators in model order; the
  hardware half of a preset, or a `main.rs` declaration; overridable.
- **Preset**: a KM-RoBoTa robot under one name. Model half in `kmr_model`, hardware half here.
- **Bus**: one physical link; label, brand, protocol, baud, read cadence; bound at bring-up to
  the port of one bus plugin, matched by label (FR-030).
- **Actuator**: one index in `0..N`; bus, firmware id, model, mode, torque choice, zero.
- **Read cadence**: K ticks per bus, with the declared cost of one read.
- **Fault bits**: per-actuator `stale`, `bus_error`, `offline`, `rejected`, set per tick and
  recorded by the engine in History (SRS FR-13.4, NFR-12).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001** (SRS delivery step 1, FR-6.1, FR-10.4): The bench chain moves on its first run with
  two hardware lines in `main.rs` (`robot!(XmBench)`, one plugin) and no adapter code written by
  the user.
- **SC-002** (SRS FR-5.3): A full write to a Protocol 2 chain costs exactly one wire transaction
  per (bus, series), counted on a scripted bus and on the bench.
- **SC-003** (SRS FR-5.10, NFR-11): On the SRS NFR-1 bench at a 1 ms tick, a write tick's I/O
  fits in the tick. Read ticks are measured and reported apart. The measured costs are within
  the adapter's declared costs.
- **SC-004** (SRS FR-13.4, NFR-2): 100% of injected faults (missing actuator, unplugged actuator,
  failed read, failed write, unknown serial, shared device, unbound bus, bad baud) give a named
  error or bit. Zero panics, zero hangs.
- **SC-005** (SRS FR-4.1, FR-4.2, FR-5.6): After a shutdown or an emergency stop, every actuator
  reads torque-off and the next run reopens the port, in 10 of 10 runs.
- **SC-006** (SRS FR-6.4): 100% of bad hardware tables (count ≠ N, duplicate id, bad id, mode or
  baud the model rejects, velocity or current mode, prismatic Dynamixel) fail the build.
- **SC-007** (SRS FR-5.5, FR-6.3, NFR-5): The robot binary cross-builds for aarch64 and armv7 with
  no hardware; with no plugin it runs, opens no port and prints exactly one warning.
- **SC-008** (SRS NFR-1, FR-013): Zero allocations on the per-tick path outside the SDK call
  sites, counted on a scripted bus over 10 000 ticks.

## Assumptions

- The HAL contract is `kmr_core/src/hal.rs` as of 2026-10-28, human-only. It carries per-field
  masks, a layout with per-actuator cadence and costs, a cadence override, and four fault bits.
  The run-time overrides are to carry the plugin baud only (kmr_core FR-039); cadence is
  overridden in the `main.rs` table (FR-026). That change is the human's.
- N as a const generic is human work in `kmr_core`. Until it lands, `kmr_api` asserts that the
  robot's N equals the core's, and only 4-actuator robots build.
- SDK boundary (human decision, 2026-10-26): an SDK need not meet the SRS; only the adapter's
  direct SDK call sites may bend, the rest complies. Known `dxl_rs` limits: it allocates at its
  call sites, blocks about 34 ms on a missing servo, reads neither Protocol 1 voltage nor
  temperature, cannot write the mode register, and uses three sync reads where one would do.
  They are recorded as limits, not questions.
- The engine owns the hold duration and the hold target; the adapter sends one command.
- A robot may mix any number of vendors (D17, decided 2026-10-29; SRS FR-5.1 patch prepared):
  each adapter is one member of the driver list and owns an index set
  (`specs/002-kmr-core/design-hal-set.md`). Only the Dynamixel adapter exists in this version.
- The adapter runs on std Linux. A `no_std` adapter is out of scope (SRS NFR-7 binds
  `kmr_core`).
- Whether a vendor SDK compiles behind a Cargo feature is a plan matter. A feature never decides
  between hardware and simulation; the plugins do (FR-031).
- `Tau` ↔ current uses the datasheet torque constant; its accuracy is the datasheet's.
- The reference chain for the SCs is the `XmBench` preset: four XM430-W210 on one bus.

## Roadmap (future, not this version)

- **2026-10-29, D18: actuator limits configurable through the API** (decided, not implemented).
  As a plugin capability: every actuator plugin whose hardware supports it, all Dynamixel models
  and not only the RX, may expose "set health limits" (voltage, temperature), written to the
  actuator's own limit registers, which the health check of FR-006 then reads. Task T070.

## SRS gaps (reported, not requirements)

- An abort, or a crash in foreign code (SRS FR-10), leaves torque as it was (SRS NFR-2). The only
  guard left is on the actuator side: a bus watchdog that cuts torque when the host goes silent.
  The SRS says nothing about it.

## Module boundaries

**kmr_core** (human-only)

- Provides: the HAL contract (`Hal`, `seal::Adapter`, `Mask`, `Group`, `Layout`, `Overrides`,
  `Written`, `Command`, `Sample`, `Batch`, `Faults`, `NoHardware`); `Q`, `Qd`, `Tau`.
- Needs to provide: N as a const generic, with a fixed word cap for `Mask`; a run entry generic
  over the adapter that hands it back after `run()` (kmr_core FR-041, FR-050); the call points
  (SRS FR-3.4 tick order, FR-5.2 phases); the hold target on the shutdown tick (kmr_core
  FR-032); the wait of the hold duration; torque off on every exit after open, panics included;
  run-time overrides limited to the plugin baud (kmr_core FR-039); the driver list: every call
  fanned out to every member, the coverage check and the group numbering (kmr_core FR-063).
- Gets from here: one `Hal` member per vendor.

**kmr_model**

- Provides: N, the actuator order, and each actuator's kind (revolute, prismatic) for every
  preset and descriptor, at compile time; the model half of each preset.
- Gets from here: each preset's hardware half under the same name, in model order, without its
  own N (`Preset::N` goes).

**kmr_api**

- Provides: `robot!`, which joins the two preset halves by name, passes N and kinds to the table
  checks, owns the count check (kmr_api FR-007), and offers the `main.rs` declaration and
  override syntax for every table field, torque choice and read cadence included (kmr_api FR-020,
  FR-042); the `Bus` plugin (`by_id`, `by_dev`, `.baud`, `.bus`) and `add_plugins`; the core's
  no-hardware adapter when no plugin is given; the re-export of presets and declaration types.
  Cadence and torque are not builder knobs.
- Gets from here: the hardware table types and checks; the adapter to hand to the engine; the
  per-bus port binding entry point; readable adapter errors.
- Shares Q1 with kmr_api Q2 (bus label; recommended, pending human confirmation).

**controllers**

- Gets: the `XmBench` preset through `kmr_api`. Never names `kmr_adapters`.
- Provides: nothing to this module; its reference main.rs is the SC-001 run.

**kmr_ffi**

- Nothing either way: the builder and hardware data never cross the FFI (SRS FR-10.1).

**bench**

- Gets: the `XmBench` preset, transaction counts, declared read and write costs.
- Provides: SC-002, SC-003, SC-005 on hardware; the default cadence of Q2 A; the pre-sort gain
  (SRS FR-5.3).

**kmr_adapters itself**

- One module per vendor, Dynamixel first. A second vendor (CubeMars MIT mode) adds a module and
  one member to the robot's driver list, and touches neither the Dynamixel one nor the contract.

## SRS coverage

| SRS id | Spec ids | Note |
| --- | --- | --- |
| Definitions Index space | FR-002 | |
| Definitions Write mask | FR-012 | per field |
| Section 4 Assumptions (N) | FR-022, FR-024 | |
| Section 7 HAL | FR-001 | |
| Section 7 User API, Config | FR-023 to FR-031 | syntax is kmr_api's |
| FR-2.1 | FR-009 | |
| FR-2.2 | FR-021, FR-032 | |
| FR-2.5 | FR-012 | latched setpoint parked |
| FR-2.6 | FR-014 | |
| FR-2.7 | FR-006 | engine side, open |
| FR-3.2 | Edge Cases | |
| FR-4.1 | FR-016 | |
| FR-4.2 | FR-017, FR-033 | |
| FR-4.5 | Edge Cases | engine side, open |
| FR-5.1 | FR-001, FR-003, FR-036 | adapter side |
| FR-5.2 | FR-004, FR-005, FR-006 | |
| FR-5.3 | FR-009, FR-010, FR-011, FR-018, FR-035 | |
| FR-5.4 | FR-002 | |
| FR-5.5 | FR-027, FR-029, User Story 5 | |
| FR-5.6 | FR-016, FR-017 | |
| FR-5.8 | FR-006, FR-017, FR-023 | adapter side |
| FR-5.9 | Module boundaries | outside the loop: kmr_core FR-050, kmr_api FR-041 |
| FR-5.10 | FR-004, FR-007, FR-008, FR-013 | |
| FR-6.1 | FR-022 to FR-028 | hardware half |
| FR-6.3 | FR-029, FR-030, FR-031 | resolution side |
| FR-6.4 | FR-007, FR-024, FR-027, FR-030 | |
| FR-6.5 | FR-018 | several buses; multi-robot open |
| FR-13.4 | FR-015, FR-032 | |
| NFR-1 | FR-013, SC-008 | |
| NFR-2 | FR-007, FR-033, SC-004 | |
| NFR-5 | SC-007 | |
| NFR-10 | FR-034 | |
| NFR-11 | FR-035, SC-003 | |
