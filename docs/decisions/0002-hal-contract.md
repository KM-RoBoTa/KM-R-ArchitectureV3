# 0002 — The hardware contract: one sealed trait, a set of drivers per build

Status: **proposed**, waiting for the maintainer's decision. The contract,
the set of drivers and the runtime call points are in the core, and the
Dynamixel member drives it; see [Current state](#current-state). Points that
depart from the SRS and need the maintainer's acceptance:
[SRS deviations](#srs-deviations-to-accept).

This record is the one owner of the contract (core T060). The adapter items
are handed here by kmr_adapters T061; the empty-robot check and the `run()`
error are in their own record,
[0008](0008-empty-robot-type-state-and-run-error.md).

The research behind each point is in
[`specs/001-kmr-adapters/research.md`](../../specs/001-kmr-adapters/research.md)
(R1 to R13).

**Note, 2026-10-29.** This record first proposed one adapter per build. One
`Hal` object then stood for the whole hardware, so a robot could have one
brand. The maintainer rejected that on 2026-10-29: a robot mixes any number
of brands. A driver is now one member of a set; the core fans every call out
to every member. The design is
[`specs/002-kmr-core/design-hal-set.md`](../../specs/002-kmr-core/design-hal-set.md);
the sections below are fixed where they said one adapter per build.

## The question

The engine must read and command actuators without knowing their vendor
(SRS FR-5.1). Three things have to be fixed:

- what the engine may know about hardware, and what stays in the adapter;
- how an adapter reaches the engine without the control engineer naming it;
- how a vendor SDK is kept away from the hot path and from the tests.

## Current state

Updated 2026-11-03.

| Piece | State |
|---|---|
| `Hal`, its types and `NoHardware` (`kmr_core/src/hal.rs`) | written |
| The set of drivers, claims, coverage, debug ownership audit (`kmr_core/src/hal/set.rs`) | written |
| `Robot::run_on(drivers)`, the runtime call points, five bus checks, `Powered` guard, hold target | written |
| Per-field write mask (`Written`, with `Kp`, `Kd`) | written |
| `impl Hal for &mut A` (lend the adapter, SRS FR-5.9) | written |
| `Overrides` | still carries `read_every`; empties with core T039 |
| `Stages` (inbound and outbound hooks for `kmr_api`) | decided (core research R15), not built (core T049) |
| Per-slot `Faults` and `TickFlags` | not built (core T044); `Faults` is a local per tick |
| N as a const generic on the contract | decided in [0007](0007-actuator-count-const-generic.md), not built (core T021 to T030) |
| Dynamixel member (`kmr_adapters`), strict scripted backend, bus plugins by label | written; bench run of the first motor open |
| `force_torque_off` in `dxl_rs` | on the unpushed SDK branch the adapter builds against |

## Decision proposed

**The contract.** One trait, `Hal`, in `kmr_core/src/hal.rs`. The engine
knows actuator indices, bit masks, opaque transaction groups and a read
cadence in ticks. Buses, series, protocols, modes and raw units stay in the
adapter; values cross as `Q`, `Qd`, `Tau` in SI units.

**A set of drivers.** A driver implements `Hal` for the actuators it owns.
The robot's hardware is a list of drivers, a frunk HList like the controller
list: static dispatch, no heap, any length. Each driver owns a set of
actuator indices; together they cover `0..N` exactly once, checked at
`run()` finalization. The core calls every operation on every driver.

- Operations, in the order the engine calls them: `check` (no I/O, returns
  the driver's own actuators, cadence, groups and costs; the core merges
  them into the `Layout`), `open`, `zero`, `ping`, `health`, `configure`,
  `read_all`, `torque_on`, then per tick `read` and `write`, then `hold`,
  `torque_off`, `close`.
- Bring-up operations return `Self::Error`. `read` and `write` never fail:
  they set fault bits (`stale`, `bus_error`, `offline`, `rejected`).
- Once `open` was called, every exit ends with `torque_off` then `close`,
  on every driver.
- Every method takes `&mut self` (R4).
- The seal is a `#[doc(hidden)] pub mod seal` supertrait: open to adapter
  crates, closed to `controllers`, which cannot depend on `kmr_core` (R3).

**`run_on`.** `Robot::run_on(drivers)` runs the robot on a list of drivers;
`Robot::run()` is `run_on` over a one-driver list holding `NoHardware`. The
driver list is an argument, never a type parameter of the builder (R1); the
builder's parameters are N ([0007](0007-actuator-count-const-generic.md)),
the schedules and the empty-robot marker ([0008](0008-empty-robot-type-state-and-run-error.md)).

**Lending.** `impl<A: Hal> Hal for &mut A`: a caller passes `&mut adapter`
and keeps it before and after the run (SRS FR-5.9). `kmr_api`'s `bind()`
uses it.

**`NoHardware`.** The core's own adapter, one driver owning every index:
`Error = Infallible`, nothing read, nothing written, one warning at `open`.
The existing tests keep running, and a build without hardware cannot pass
silently for a robot.

**Hardware from the robot and the plugins.** There is no `Hardware` alias
any more. `kmr_api::robot!` names the robot's hardware table (a preset, or
the user's table); `.add_plugins(Bus::by_id(..))` says where it is plugged
in. `run()` binds the plugins to the table and calls `run_on` on the
members; with no plugin it runs on `NoHardware` and says so once. The
Dynamixel SDK is behind the `kmr_api/dynamixel` feature; nothing enables it
by default (R2).

**Overrides.** `Overrides` ends empty, a `#[non_exhaustive]` struct kept as
the seam (core research R9): the read cadence lives in the hardware table,
the plugin baud enters at `bind` (adapters research R6). Until core T039,
`read_every` is still there and only `NoHardware` reads it.

**The `Backend` seam.** Inside `kmr_adapters`, `Dynamixel<B: Backend>` talks
to the SDK through a private trait. `dxl_rs` is behind it today, `dxl_rs_v2`
replaces it when it has a bus, and a scripted backend counts transactions and
injects faults in the tests (R7).

## The engine side of the contract

**Call points.** Finalization, no I/O: `claim` (each member's `check`),
merged into the plan; overlaps and gaps fail here. Init: the `Powered` guard,
then `open`, `zero`. PostInit: `ping`, `health`, `configure`, a pushed slot
and `read_all`, `torque_on`. Each tick: bus check, `begin_tick`, a pushed
slot and `read`; `First`, bus check; `EachTick`, bus check; `Last`, bus
check; the `Batch` from this tick's write masks and `write` when it is not
empty; `end_tick`; bus check; Desired cleared; sleep to the next grid point.

**Five bus checks per tick**, each one atomic load (SRS NFR-1): tick start,
after `First`, after `EachTick`, after `Last` (an emergency stop raised in
`Last` skips the write), tick end.

**The `Powered` guard.** Built just before `open`, so a failed `open` is
closed too. From then on every exit, unwind included, ends with
`torque_off` then `close` on every member, torque cut in reverse member
order (SRS FR-5.6, NFR-2). Before it nothing is opened: a `check` error or an emergency stop
in `PreInit` touches no hardware.

**The hold target (first-motor OC1).** A shutdown holds before the guard
drops: the target is the latest sample overlaid with this tick's writes,
and `written` marks which values are this tick's. Desired is cleared after
each write, so holding "the last command" would send zeros. An emergency
stop skips the hold. The hold lasts the user's `set_hold` duration, in `dt`
slices, and an emergency stop during it cuts torque at once; default: no
hold wait.

**`Stages` (decided, not built).** A public trait `Stages<N>` with
`inbound(&mut RobotState<N>, &Time)` after `read`, before `First`, and
`outbound(&RobotState<N>, &Time)` after the write, `impl Stages<N> for ()`;
`run_on` uses `()`, `run_on_staged` takes `kmr_api`'s. Static dispatch, no
heap, no middleware named in the core (core research R15).

## The adapter side (kmr_adapters T061)

- **Torque off on every actuator.** The private `Backend` has
  `force_torque_off`: `Torque_Enable` = 0 whether or not this run switched
  the actuator on. `torque_off` sends it to every actuator of every opened
  bus, read-only ones included; `torque_on` sends it to every read-only one
  (SRS FR-5.2, FR-5.8). Failures are logged and the loop goes on (adapters
  research R1).
- **A strict fake.** The scripted backend refuses what `DxlRs` refuses, with
  the same error, instead of accepting or asserting: the lenient fake hid
  three real failures and its asserts double-panicked through `Powered`
  (adapters research R2).
- **The table is generic over N.** `Table<NB, N>`: the length of a preset's
  table is its N, checked against `robot!`'s at compile time.
- **Binding by label (D6, assumed B).** A plugin names its bus label;
  optional when the table has one bus. Matched before any I/O.
- **Plugin baud at `bind`.** `bind` copies the plugin's baud over the
  table's and checks it against every model on that bus; the error names the
  actuator, the bus label and the baud (adapters research R6).
- **Zero offset (D8, assumed C).** Per actuator `Zero::Center` (default, the
  model's centre) or `Zero::Raw(value)`, applied in software.
- **Default read cadence (D9, assumed A).** A per-brand default,
  `DEFAULT_READ_EVERY` = 4 ticks for Dynamixel, a placeholder until the
  bench measures it (kmr_adapters T063, T064); a bus may override it in the
  table.
- **Protocol 1 health.** Read, not skipped, since `09ad611`: voltage and
  temperature on Protocol 1 too, and the joint-mode check on the RX-28. The
  "Protocol 1 health exemption" of earlier drafts is gone.

## Defence

### Real-time performance

- Static dispatch only: `run_on` is generic over the driver list,
  `Dynamixel` is generic over its backend. No `dyn`, no fat pointer.
- **Cost:** drivers run one after another in the tick. Ten buses are ten
  round trips, which does not fit 1 kHz. A thread per bus is roadmap work;
  XmBench has one driver.
- Every per-tick type is a fixed array. With `JOINTS = 4` a `Mask` is one
  `u64`; `Written` is three, `Faults` four.
- The pre-sort of SRS FR-5.3 is split: `Layout::new` sorts the actuators by
  group once, at `check`; per tick `Batch` filters that order by the write
  mask, one pass over `JOINTS`.
- `read` and `write` return no `Result` and never retry, so a fault costs one
  failed transaction, never a loop.
- **Cost, accepted for delivery step 1:** `dxl_rs` allocates at its call sites
  and blocks about 34 ms on a missing servo. Both stay at the SDK boundary and
  go with `dxl_rs_v2`.
- Not measured yet: one transaction per (bus, series) and per-tick I/O under
  1 ms are checked on the SRS NFR-1 bench.

### Quality

- **The v2 lessons are invariants, not habits.** v2's `&self` reads forced a
  `RefCell` in the CubeMars driver (R4); a `?` before shutdown left torque on
  (R5); a private seal blocked cross-crate adapters (R3).
- **No panic on the hot path.** An out-of-range target is not sent and gets
  `rejected` (R11). A failed read keeps the copied value and sets `stale`
  (SRS FR-2.6).
- **Any number of vendors fit.** Each vendor is one driver of the set and
  owns its own actuator indices; ten actuators of ten brands are ten drivers.
  CubeMars MIT mode, the next vendor in view, sends q, qd, kp, kd and tau in
  one frame and answers with its state: `Command` is `#[non_exhaustive]`, and
  an adapter may answer `read` from the reply of the previous `write` (R13).
- **Cost of the feature switch:** a robot binary built without
  `kmr_api/dynamixel` runs on `NoHardware`. The warning at `open` is the only
  guard.

### Developer experience

- The control engineer sees no hardware beyond one line per bus,
  `.add_plugins(Bus::by_id(..))`; without it the robot simulates. The cadence
  is in the hardware table, not a builder knob.
- The integrator picks the hardware with `robot!` (a preset or a table) and
  one Cargo feature, and describes the chain in Rust consts that fail to
  compile when wrong (SRS FR-6.4).
- `cargo test --workspace` needs no C SDK, no libclang and no serial port.

### Alternative rejected: `Robot<CS, H>`

An adapter type parameter on the builder. Every builder impl, every test and
the `kmr_api` wrapper change, for nothing `run_on` does not already give. The
driver list goes to `run_on` too, not onto the builder.

### Alternative rejected: one adapter per build

This record's first proposal. It made one adapter the whole hardware, so a
robot could have one brand. Rejected by the maintainer on 2026-10-29
(`design-hal-set.md`).

### Alternative rejected: `.add_adapter(..)` on the builder

It puts hardware in the control engineer's hands, which the 2026-10-26
clarification rules out.

### Alternative rejected: always Dynamixel

`cargo test --workspace` and CI would need the C SDK and would try to open
ports.

### Alternative rejected: call `dxl_rs` directly

No fault injection, no transaction count in the tests, and a rewrite when
`dxl_rs_v2` lands.

## SRS deviations to accept

Each one is followed in the code until the maintainer overrules it. The SRS
edit, if accepted, is the maintainer's.

### Write mask per field, not per actuator (R6)

SRS Definitions: "Write mask: per-actuator written-this-tick bitset".

The contract has one mask per field, `Written { q, qd, tau }`. Desired is
cleared after apply (SRS FR-2.5). With one mask per actuator, a `Qd` write on
a position-mode actuator marks it written, and the adapter sends its cleared
`Q`: zero rad. Per field, it sends only the field its mode takes and flags
`rejected` otherwise. Cost: three masks instead of one, and a routing method
next to `sealed::Slot` in `state/field.rs`.

### Port binding as a const (R8): dropped

This deviation is gone. The SRS rewrite of 2026-10-27 has no port-binding
file: ports come from builder plugins, `Bus::by_id` preferred, `Bus::by_dev`
allowed (SRS FR-6.3), which is what `kmr_api` and the adapter do. Nothing
to accept.

### Zero rad at the model's center (R10)

The SRS does not say where zero is. v2 used raw 0 = 0 rad.

v3 puts zero at the center of the range: 2048 on X, 512 on AX/RX. Ranges are
symmetric, and the ROS2 RX-28 package does the same. Consequence: a v2 pose
does not carry over unchanged. It is off by π rad on X and 150° on AX/RX.

### Out-of-range targets rejected (R11)

SRS FR-4.5 leaves the joint-limit policy undecided and asks for
`unimplemented!` on that path.

The adapter does not enforce joint limits. It checks the model's register
range, which is a different limit: a target outside it cannot be encoded. It
is not sent and gets `rejected`. An `unimplemented!` there would panic the
loop with torque on; v2 clamped and dropped the flag. Joint limits stay an
engine decision, still open.

### Protocol 1 has a sync write (R12)

SRS FR-5.3: "Protocol 1: AX-12A, RX-28, one round trip per actuator".

That holds for reads only. Protocol 1 has a sync write, and v2 used it. The
adapter sends one sync write per (bus, series) on both protocols. Protocol 1
reads stay one per actuator, position only; `Qd` and `Tau` keep their last
value. The contract cannot declare them unread yet: `check` warns, naming
each Protocol 1 actuator. Health is read on Protocol 1 as on Protocol 2
(see the adapter side).

### Adapter configuration stays out of `kmr_api` (SRS section 7)

SRS section 7, rows User API and Config: the builder takes "HAL adapters,
per-machine port bindings", and the Config interface lives in `kmr_api`. The
`kmr_api` module also covers FR-6.3.

Per the 2026-10-26 clarification, adapter, buses and cadence are configured
in the adapter layer, in the hardware table. `kmr_api` names the table
through `robot!` and takes the bus plugins (SRS FR-6.3, rewritten
2026-10-27); it has no cadence knob. The "HAL adapters" wording of the User
API row still needs the maintainer's rewording.
