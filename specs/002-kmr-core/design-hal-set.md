# Design note: the robot's hardware as a set of drivers

Status: proposal for the maintainer. The flaw and the shape were agreed on 2026-10-29; names,
placement and the items marked "pending human" are not. Nothing in `kmr_core/` is edited.
Written 2026-10-29.

Scope: the `Hal` contract (`kmr_core/src/hal.rs`), the run entry, kmr_adapters' `check`,
kmr_api's `Spec` and `robot!` hardware argument. Bench: XmBench has one member, so the
first-motor path of `design-first-motor.md` is unchanged.

Spec ids: kmr_core FR-039, FR-041, FR-043, FR-044, FR-045, new FR-063 (SRS FR-5.1, FR-5.2,
FR-5.3, FR-5.6, NFR-2); kmr_adapters FR-001, FR-002, FR-003, new FR-036 (SRS FR-5.1, FR-5.4);
kmr_api FR-005 (SRS FR-6.1, FR-5.1).

Invariant files: none. The new walk lives in `hal.rs`. `schedule.rs`, `controllers.rs`,
`state/field.rs`, `sealed.rs` and `signal.rs` are not touched.

## 1. The flaw

`Hal` (`hal.rs:297`) describes the whole hardware as one object. `check` (`:304`) returns the
`Layout` of all `JOINTS` actuators (`:92-103`), `read` (`:330`) fills all of them, `torque_off`
(`:343`) cuts all of them. "One driver" and "all the hardware" are the same thing, so a robot
can have one brand. Decision 0002 wrote that accident down as "one adapter per build", and
kmr_api inherited it: its `Spec` carries one adapter type.

The requirement: a robot mixes any number of brands and drivers. Ten actuators of ten brands
must work. Mixed Dynamixel models were never the problem: one adapter, one sync write per
(bus, series).

## 2. The shape

1. A driver is a **member**. It implements `Hal` as today, for the actuators it owns.
2. The robot's hardware is a **list** of members: frunk `HCons<A, HCons<B, HNil>>`, like the
   controller list (`robot.rs:29` already re-exports both). Static dispatch, no `dyn`, no heap,
   any length.
3. Each member owns a `Mask` of actuator indices. Together the members cover `0..N` exactly
   once: no gap, no overlap.
4. The core fans every call out to every member. `Sample`, `Command`, `Faults` stay whole-N
   shared arrays; each member touches only its own indices.
5. `run_on` takes the list. `run()` is `run_on(hlist![NoHardware])`; `NoHardware` owns every
   index.

The roles of `run_on` and `run()`, the loop order, the write mask and the rest of the trait do
not change.

## 3. Trait changes

| Item | Today | New |
|---|---|---|
| `Hal::check` (`hal.rs:304`) | `-> Result<Layout, Self::Error>`, whole N | `-> Result<Claim, Self::Error>`: the member's owned mask, the cadence and local group of each owned actuator, its read and write costs. No I/O, as today |
| new `Claim` | — | `owned: Mask`, `group: [Group; JOINTS]`, `read_every: [NonZeroU32; JOINTS]`, `read_cost`, `write_cost`. Entries outside `owned` are ignored |
| `Layout` (`:92-123`) | built by the adapter | built by the core from the claims: groups renumbered, cadence copied, costs summed. `Layout::new` may become `pub(crate)` (pending human) |
| `Group` (`:89`) | adapter-wide id | member-local id; the core shifts it to a robot-wide id |
| `Batch`, `Runs` (`:214-276`) | runs of the whole robot | one `Batch` per tick as today; each member gets a view over its own group range, and `runs()` yields its local ids. Private fields only, public API unchanged |
| `Sample` (`:190-212`) | lent once | gains a `pub(crate)` reborrow, so one slot is lent to each member in turn |
| `Overrides` (`:125-131`) | given to the adapter | the same value given to every member (empty after core K3.8) |
| new walk trait `Drivers` | — | sealed; implemented by the core for `HNil` and `HCons<H: Hal, T: Drivers>` only. One method per call, head then tail |
| `Robot::run_on` | `<A: Hal>(self, hal: A) -> Result<(), A::Error>` | `<D: Drivers>(self, hal: D) -> Result<(), D::Error>`; `RunError<D::Error>` after K3.3 |
| `NoHardware` (`:349-408`) | the whole robot | a member owning `Mask::ALL` |
| `impl Hal for &mut A` (design-first-motor open choice 3) | hands one adapter back | per member: `hlist![&mut dxl, &mut cubemars]` hands each one back |
| every other method (`:308-346`) | | signatures unchanged |

The trait doc at `:293-296` becomes "one member of the robot's hardware: the engine calls it
for the actuators it claims".

## 4. Finalization check (no I/O)

At `run()` finalization, before any port opens:

1. `check(overrides)` on every member, in declaration order. The first `Err` stops the run.
   Nothing was opened, so no `torque_off` or `close` (as today, design-first-motor §3).
2. Coverage. Every index in `0..N` is owned by exactly one member. Errors name the actuator
   and the members, by position and `core::any::type_name` (a `&'static str`, no heap):
   `actuator 5 is owned by no member`, `actuator 3 is owned by member 0 (Dynamixel<DxlRs>) and
   member 1 (CubeMars<..>)`.
3. Groups. Member k's local ids are shifted by the group count of the members before it, so
   each group belongs to exactly one member. More than 256 groups in all (`Group(u8)`) is an
   error.
4. The core keeps each member's group range, builds the `Layout` and sorts it once (SRS FR-5.3
   pre-sort unchanged).

The coverage error is `RunError::Config(ConfigError::Coverage { actuator, members })` (core
K3.3). If the set lands before K3.3: a `Coverage` variant on the list's error (pending human).

## 5. Fan-out per phase

| Phase | Call | Member order | On error |
|---|---|---|---|
| finalization | `check` | declaration | stop; nothing opened |
| Init | guard, `open` on all, then `zero` on all | declaration | stop; guard cuts every member |
| PostInit | `ping`, `health`, `configure`, `read_all` (one seed slot), estop check (K1.9), `torque_on`, each on all | declaration | same |
| tick, before `First` | `read` on every member; the refreshed masks are unioned | declaration | faults only |
| tick, after `Last` | `write` on every member, each with its own runs | declaration | faults only |
| shutdown | `hold` on every member, same target | declaration | none |
| every exit after the first `open` | `torque_off` on every member, then `close` on every member | reverse | none |

Rules:

- Step-major: `open` runs on every member before any `zero`, and so on. The first `Err` stops
  the walk.
- The `Powered` guard (design-first-motor §2 #10) wraps the whole list. It is built before the
  first `open`. Its `Drop` walks every member, also those never opened (adapters already make
  `torque_off` and `close` safe before `open`, core contracts/hal.md "What the engine expects"
  1).
- No `?` between two members on the exit walk: `torque_off` and `close` return `()`, so the walk
  always reaches the last member. A panic there during an unwind aborts the process; kmr_adapters
  FR-033 (SRS NFR-2) already forbids it.
- Torque off reaches all members before any `close`: a slow flush on one bus does not delay
  torque off on another.
- A member with no run this tick: skipped, as design-first-motor open choice 2 (pending human).
- Debug builds check that a member wrote only its owned slots of `Sample` and `Faults`, and that
  its returned mask is inside its owned mask. Release builds trust the member.

## 6. The error type

- Each member keeps its own `type Error`.
- The list's error is a nested enum, one variant per member: for `HCons<H, T>`,
  `MemberError<H::Error, T::Error> { Head(H::Error), Tail(T::Error) }`; for `HNil`,
  `Infallible`. It implements `core::error::Error`; `Display` prefixes the member's position and
  type name. No heap.
- kmr_api prints it into `UserError::BringUp { cause: String }` (heap allowed in kmr_api only).
- A one-member list displays today's adapter error behind the prefix.

## 7. kmr_api side

- `Spec` carries `type Drivers` (the list) instead of one adapter type, and
  `fn bind(&[Binding]) -> Result<Self::Drivers, UserError>`.
- `robot!` builds the list from the hardware table. One entry per bus names its brand, its
  firmware ids and its actuator indices. The buses of one brand form one member: the Dynamixel
  `Table<NB, N>` stays valid as one member owning several buses.
- XmBench: one bus, one brand, `HCons<__DxlAdapter<__NB>, HNil>`. `main.rs` stays
  `robot!(XmBench)`. A one-member table keeps today's form and owns `0..N` in model order.
- No bus plugin: `hlist![NoHardware]`.
- `Bus::by_id(..)` and `by_dev(..)` plugins still supply serial ids or paths only. Labels (D6)
  match them to buses across all members.
- `Bound` (kmr_api FR-041 (SRS FR-5.9)) hands back the whole list.
- The indices are consts, so `robot!` could also check coverage in const evaluation. The run()
  check stays as the backstop (pending human).

## 8. Placement: A assumed, B the fallback

**A, assumed (pending human).** The walk trait, `Claim`, the merge and the coverage check live in
kmr_core (`hal.rs`, or a new `hal/set.rs`). HUMAN work, like the first-motor commits. The core
holds every owned mask, so it can check each member in debug builds, and it numbers the groups
itself.

**B, fallback.** A `Chain<A, B>` type in kmr_adapters implements `Hal` by calling both halves,
nested for more members. Zero core edits; it can be lifted into the core later. Costs: the core
sees one adapter, so B cannot stop one member from writing another member's slots; the chain
must split the batch and renumber groups through `Batch` methods `hal.rs` does not offer; the
exit order is adapter code, not an engine guarantee. `impl Hal for (A, B)` cannot live in
kmr_adapters (orphan rule), hence its own struct.

## 9. Costs

- Members run one after the other inside the tick. Ten buses are ten round trips per read and
  per write, which does not fit 1 kHz (SRS NFR-1). XmBench has one bus: nothing changes there.
- Roadmap, out of scope now: a thread per bus. Each member is a self-contained `&mut` object, so
  it can move to its own thread later. It waits on the threaded hand-off still open in the
  ROADMAP.
- Finalization: a few N-sized arrays per member, once. Per tick in release: one inlined walk
  over the members, one mask union, and one pass over the sorted batch to split it by the group
  ranges kept at finalization.
- Debug builds: one slot copy per member per tick, for the ownership check.
- Nothing done so far is thrown away: the Dynamixel adapter, its multi-bus table, the strict
  fake, the guard and the tick order of design-first-motor. The Dynamixel `check` returns a
  `Claim` with its table's indices instead of a `Layout`.

## 10. Open choices for the maintainer (pending human)

1. **Placement.** A (assumed) or B.
2. **Order.** Recommended: M1 keeps `run_on<A: Hal>`, the set is one follow-up commit (XmBench
   has one member). Alternative: land the set before K1.6, so `run_on` changes once.
3. **Names.** `Claim`, `Drivers`, `MemberError`.
4. **Member granularity in `robot!`.** Per brand (recommended) or per bus. Per bus is what a
   thread per bus would need later.
5. **Ownership breach in debug builds.** `debug_assert!` naming the member (it panics with
   torque on, debug only), or a log line and `emergency_stop!`.
6. **Coverage at compile time** in `robot!`, on top of the run() check.
7. **Costs.** Summed (recommended, true while members run in turn) or the max (with threads).
8. **Exit order.** Every `torque_off`, then every `close` (recommended), or both per member.

## 11. SRS and spec ids touched

- SRS FR-5.1: wording patch prepared 2026-10-29 by a separate agent, applied by the maintainer:
  "A robot may mix any number of adapters, of any vendor, one or more buses each; the engine
  drives them all in the same tick and cuts torque on all of them at exit." Until then FR-5.1
  "one adapter per vendor or bus" is silent on mixing (coordination §2).
- Also read against: SRS FR-5.2, FR-5.3, FR-5.6, FR-4.1, FR-4.2, FR-6.1, FR-6.4, NFR-1, NFR-2.
- kmr_core spec FR-039, FR-041, FR-043, FR-044, new FR-063; plan KS.1; contracts
  `hal.md`, `engine-surface.md` (dated notes).
- kmr_adapters spec FR-002, FR-024, new FR-036, Assumptions, Key Entities, Module boundaries;
  plan E11; contracts `hal-conformance.md` (dated note).
- kmr_api data-model `Spec`, contracts/robot-hardware.md, research R2.
- Decision 0002 (retitled), coordination D17, §2 FR-5.1, §5.
- Tasks: kmr_core T069 (HUMAN), kmr_adapters T069 (agent), kmr_api T069 (agent).
