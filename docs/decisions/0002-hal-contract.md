# 0002 — The hardware contract: one sealed trait, a set of drivers per build

Status: **proposed**, waiting for the maintainer's decision. The contract is
in the core; nothing calls it yet. See [Current state](#current-state).
Six points depart from the SRS and need the maintainer's acceptance:
[SRS deviations](#srs-deviations-to-accept).

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

| Piece | State |
|---|---|
| `Hal`, its types and `NoHardware` (`kmr_core/src/hal.rs`) | written |
| The set of drivers (`design-hal-set.md`) | designed 2026-10-29, not in the core |
| `Robot::run_on`, the runtime call points, the per-field write mask | not in the core yet |
| `kmr_adapters`, Dynamixel adapter on a scripted backend | in progress |
| `Hardware` alias | `NoHardware` only: the `dxl_rs` dependency cannot be fetched yet |

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

**`run_on`.** `Robot::run_on(hal)` runs the robot on a list of drivers;
`Robot::run()` is `run_on` over a one-driver list holding `NoHardware`.
`kmr_core::Robot` keeps its single type parameter (R1).

**`NoHardware`.** The core's own adapter, one driver owning every index:
`Error = Infallible`, nothing read, nothing written, one warning at `open`.
The existing tests keep running, and a build without hardware cannot pass
silently for a robot.

**`Hardware`, chosen by a feature.** `kmr_adapters` exports
`pub type Hardware`: `Dynamixel<DxlRs>` with the `dynamixel` feature,
`NoHardware` without. `kmr_api::Robot::run()` calls
`run_on(Hardware::default())`. The integrator builds the robot binary with
`--features kmr_api/dynamixel`; nothing enables it by default (R2).

**The `Backend` seam.** Inside `kmr_adapters`, `Dynamixel<B: Backend>` talks
to the SDK through a private trait. `dxl_rs` is behind it today, `dxl_rs_v2`
replaces it when it has a bus, and a scripted backend counts transactions and
injects faults in the tests (R7).

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

- The control engineer sees no hardware. The builder is unchanged,
  `complete_api_example.rs` is unchanged, and `kmr_api` exposes no adapter,
  bus or port type. The one new knob is `set_read_every(ticks)`.
- The integrator picks the hardware with one Cargo feature and describes the
  chain in Rust consts that fail to compile when wrong (SRS FR-6.4).
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

### Port binding as a const, not a file (R8)

SRS FR-6.1: "The per-machine port binding (FR-6.3) is its own file."

The binding is its own const, `PORTS`, kept apart from the topology in
`kmr_adapters/src/robot.rs`. A file needs a parser, and a parser here needs
the heap; `kmr_model` owns files later. The rest of FR-6.3 holds: one serial
id per bus, resolved under `/dev/serial/by-id` at bring-up.

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
each Protocol 1 actuator.

### Adapter configuration stays out of `kmr_api` (SRS section 7)

SRS section 7, rows User API and Config: the builder takes "HAL adapters,
per-machine port bindings", and the Config interface lives in `kmr_api`. The
`kmr_api` module also covers FR-6.3.

Per the 2026-10-26 clarification, adapter, buses, ports and cadence are
configured in the adapter layer only. `kmr_api` gains `set_read_every` and
nothing else. Both rows, and FR-6.3 in the `kmr_api` module, need rewording.
