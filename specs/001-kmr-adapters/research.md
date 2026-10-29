# Research: kmr_adapters, Dynamixel (re-planned 2026-10-28)

Sources: `spec.md` (2026-10-28), `docs/srs/SRS.md`, `specs/spec-coordination-2026-10-28.md`,
`specs/002-kmr-core/design-first-motor.md`, `specs/002-kmr-core/design-const-generic-n.md`,
`kmr_core/src/hal.rs`, `kmr_adapters/src`, `../dxl_rs`, and the stale `fix-plan.md` and
research of 2026-10-26 (kept where they still hold, R13).

Baseline: `cargo test -p kmr_adapters --offline` passes 86 unit tests and 3 doctests, without
the `dynamixel` feature.

Prototypes, in `scratchpad/adapters-plan/`: `constmsg` (R4) and `table` (R4, R5).

## R1. Torque off on every opened actuator

- **Decision**: the private `Backend` gains `force_torque_off(port, model, id)`. It writes
  `Torque_Enable` = 0 whether or not this process switched the actuator on. `torque_off` sends it
  to every actuator of every opened bus, read-only ones included. `torque_on` sends it to every
  read-only actuator (FR-006 (SRS FR-5.2, FR-5.8)). `configure` warns, no longer fails, when a
  read-only actuator was left powered: PostInit switches it off next. Failures are logged; the
  loop goes on to the next actuator (FR-017 (SRS FR-4.2, FR-5.6)).
- **Rationale**: today `torque_off` skips read-only actuators (`mod.rs`, `torque_off`), and
  `DxlRs::unpower` fails for an actuator it holds no `TorqueOn` handle for (`dxl_rs.rs`,
  `unpower`). An actuator left on by a crashed run, or a read-only one powered by hand, stays
  powered after shutdown. SRS FR-5.8 makes read-only mean torque off.
- **SDK**: `dxl_rs` has `disable_torque` on `ActuatorOn` only (`src/actuator.rs:87`); the C call
  under it is `write1ByteTxRx{1,2}`. It needs a `force_torque_off` that takes the actuator in any
  state. HUMAN blocker (critical path step 1). Until it lands, `DxlRs::force_torque_off` falls
  back to today's `unpower`: it cuts what it holds a handle for and fails, with one warning, for
  the rest. The `dynamixel` build stays green and honest; A4 swaps in the SDK call.
- **Alternatives**: enable then disable through the type-state, rejected: it powers a read-only
  actuator for one round trip. Keep the skip and document it, rejected: SRS FR-5.8 and FR-4.2.

## R2. A strict fake backend

- **Decision**: `Scripted` refuses what `DxlRs` refuses, with the same error, instead of
  accepting it or asserting. Rules in [contracts/backend.md](contracts/backend.md) § Strictness.
- **Rationale**: the fake hid three real failures: torque off without a handle, Protocol 1
  health, and three sync reads per group. Its `assert!`s also panic inside `Drop` when a test
  unwinds through the core's `Powered` guard: a double panic aborts the test binary.
- **Alternatives**: a second "strict" fake beside the lenient one, rejected: two fakes drift.

## R3. Fit with the human core design

- **Decision**: the adapter keeps the `Hal` contract as is and relies on:
  - `run_on<A: Hal>(self, hal: A) -> Result<(), A::Error>`: the adapter's error reaches the user
    unchanged; both signals are `Ok`. Core K3.3 later wraps it as `RunError::Hal(E)`, beside
    `RunError::Config`; the adapter error type does not change.
  - `Powered`: built before `open`; its `Drop` calls `torque_off` then `close`. So `torque_off`
    and `close` must be safe after a failed or partial `open`, idempotent, and never panic.
    `torque_off` is a no-op until the backend opened its ports.
  - Hold target: latest sample overlaid with this tick's writes. The adapter's `hold` sends
    `last.q` to every powered position actuator, so the command is right with either of the
    core's two hold-target options. It ignores `last.written`.
  - `Command` + `Written`: `write` sends only the field the mode takes, and only when its bit is
    set; else `rejected` (FR-012 (SRS FR-2.5)). Unchanged code.
  - Startup writes never reach the adapter (core discards them). Nothing to do here.
- **Rationale**: the core prototype ran the real adapter through `run_on(&mut adapter)` and all
  exit paths passed (design-first-motor §4).
- **Open, human**: `impl Hal for &mut A` (core open choice 3). Step D1 needs it, or a shared log
  inside the fake.

## R4. Hardware table generic over N, checked in const evaluation

- **Decision**: `dynamixel::Table<const NB: usize, const N: usize>` with `buses: [Bus; NB]` and
  `actuators: [Actuator; N]`, both public. Presets are inherent consts:
  `presets::XmBench::HARDWARE`. `Table::check` (a `const fn`) runs inside every preset const and
  every override. `robot!` owns the count check: `const _: &[Actuator; N] = &TABLE.actuators;`.
- **Proved** (`scratchpad/adapters-plan/table`): a 4-actuator preset against N = 3 gives E0308
  "expected an array with a size of 3, found one with a size of 4". `.baud("front", ..)` on a
  table without that label, and a baud the model rejects, both fail with E0080 and the message.
  Overrides chain in a `const` in the user's crate.
- **No proc macro needed for FR-027**: a const-evaluated message cannot use `format!` (E0015),
  but a message built byte by byte in a `const fn` names the index.
  `scratchpad/adapters-plan/constmsg` gives
  `error[E0080]: evaluation panicked: actuator 2: id above 252` on rustc 1.97.1, and today's
  `config.rs` (`Message`, `fail`) already does it. The plan keeps the index in every table
  message. Spec FR-027 and kmr_api FR-007 now say so (coordination of 2026-10-28); the old
  "needs a proc macro" text is gone.
- **Alternatives**: a trait with an associated table type per preset, rejected: more types, same
  check. `[Actuator; MAX]` with a length, rejected: the count check stops being a type error.

## R5. Bus binding (D6, spec Q1): recommended option B, isolated

- **Decision**: each table bus has a `label`. A plugin names it with `.bus("label")`. A one-bus
  robot needs no label. All matching lives in one function, `binding::match_plugins`, which maps
  `&[Binding]` to one `Binding` per table bus, or an error. It runs in `Dynamixel::bind`, before
  any I/O.
- **Errors, before I/O** (FR-030 (SRS FR-6.3, FR-6.4)): unknown label, bus left unbound, plugin
  left over, several buses with a plugin lacking a label, two plugins on one label.
- **Pending human confirmation.** Option A (by order) or C (by probe) would replace
  `match_plugins` only. A needs no label in the table; C would move matching into `open`, since
  it needs I/O.
- **SRS wording**: SRS FR-6.4 calls a bad port binding a bring-up error. The spec makes a label
  mismatch a `run()` error before I/O. Both happen inside `run()`; earlier is safer. Reported in
  R15.

## R6. Plugin baud enters at `bind`

- **Decision**: `bind` copies the plugin's baud over the table's and checks it against every
  model on that bus. The error names the actuator index, the bus label and the baud. No I/O.
- **Rationale**: the adapter then needs no run-time override from the core. kmr_core FR-039 plans
  `Overrides` to carry the plugin baud; this adapter does not use it. One human core change fewer
  on the critical path.
- **Overrides**: `check` ignores `Overrides`. Core K3.8 removes `Overrides::read_every`, so
  `Overrides` carries nothing; cadence is a table field only (FR-026 (SRS FR-6.1)). `kmr_api`
  passes `Overrides::default()`.

## R7. Zero position (D8, spec Q3): recommended option C, isolated

- **Decision**: `Actuator::zero: Zero`, `Zero::Center` or `Zero::Raw(i32)`. `Zero::DEFAULT` is
  `Center` (2048 on X, 512 on AX/RX). `units::origin(actuator)` is the only reader. Software
  offset only; no register write. Until E1 lands, every actuator uses the centre, as today.
- **Changing the answer**: A: drop the `zero` override, keep the centre. B: set `Zero::DEFAULT` to
  `Raw(0)`. One const or one field, nothing else.
- **Rationale**: matches each robot's mounting; symmetric ranges by default. `dxl_rs` cannot write
  the Homing Offset register.
- **SRS**: FR-5.2 "double-check zeros" in PostInit is undefined. With a software offset the
  adapter checks that the `read_all` positions convert inside the register range. Reported in R15.

## R8. Default read cadence (D9, spec Q2): recommended option A, isolated

- **Decision**: `Bus::read_every: Option<NonZeroU32>`. Cadence in force: `main.rs` override, else
  the table, else `dynamixel::config::DEFAULT_READ_EVERY`. That const is the only default.
  `DynamixelError::CadenceMissing` goes.
- **Value**: 4, a placeholder until the NFR-1 bench (SC-003). Reason: a Protocol 2 read of four
  XM430 at 1 Mbaud is about 1 ms of wire time plus up to 1 ms of USB latency; the write is about
  0.35 ms. One read tick in four keeps the average under the 1 ms tick. The bench picks the
  number (constitution V).
- **Changing the answer**: B (table must set it) makes `Bus::read_every` non-optional and removes
  the const. C is no longer reachable: `NonZeroU32` cannot hold 0.

## R9. The `dynamixel` feature compiles the SDK, nothing more

- **Decision**: `kmr_adapters` keeps `dynamixel = ["dep:dxl_rs"]`, off by default. It gates
  `DxlRs` only. Tables, presets, `bind`, the fake and every test build without it. `kmr_api`
  forwards it; the robot binary enables it. With plugins and without the feature, `kmr_api`'s
  `run()` returns an error naming the feature: never simulation (FR-031 (SRS FR-6.3)).
- **Rationale**: CI and `cargo test --workspace` must not need libclang. The plugins, not the
  feature, decide between hardware and simulation.

## R10. N before and after the core's const generic

- **Decision**: `Table<NB, N>` is generic now; it is data and needs no core change.
  `Dynamixel<B, NB>` takes `Table<NB, JOINTS>` until the core has N. Then `Dynamixel<B, NB, N>`
  with `impl Hal<N>`, and `DxlRs` sizes its handle table from N (design-const-generic-n step 9).
  The `robot!` N ≤ 64 assert covers the mask cap (D4).

## R11. Protocol 1 health under the SDK boundary exemption

- **Decision**: `DxlRs::read_health` returns `Unsupported` for Protocol 1 at its call site;
  `health` warns once per actuator and checks nothing else for it. Off the critical path (XmBench
  is Protocol 2).
- **Rationale**: the human's SDK boundary decision: only direct SDK call sites bend. Today every
  AX-12A and RX-28 fails bring-up.
- **Alternative**: a 2-byte read at address 42 in `dxl_rs` (HUMAN). Then the exemption goes.

## R12. One sync read per group

- **Decision**: after `dxl_rs` gains a 10-byte block read at address 126 (HUMAN), `sync_read_state`
  makes one call and `STATE_READS` becomes `&[10]`. Until then the fake logs one record per wire
  transaction, three per group, and the test asserting one is `#[ignore]`d with the reason.
  Off the critical path: SC-002 counts writes.

## R13. Kept from the research of 2026-10-26

Still true, unchanged: R3 (public seal module), R4 (`&mut self`), R5 (every exit after `open`
ends with `torque_off`, `close`: now the core's `Powered`), R7 SDK limits (allocation at call
sites, ~34 ms block on a missing servo, sync read all-or-nothing, sync write per group, RX-28 on
the AX table, global C port state: register all, open all, init once), R8 (by-id resolution,
flush, 1 ms latency timer read back, udev hint), R10 units (now with the R7 offset), R11 (an
out-of-range target is `rejected`, never clamped), R12 (Protocol 1: position only, one round trip
per actuator), R13 (a second vendor fits the contract).

Dropped: R2 (`Hardware` alias by feature), R9 (`count % K` with a `CadenceMissing` error; the
counting stays, the error goes), and "port binding as a const". Moved: R1 (`RunError`) is the
core's (K3.3, HUMAN), not an adapter decision; `ZeroReadCadence` stays dropped.

## R14. Exit paths the adapter sees

Matches design-first-motor §3. After `open`, every exit calls `torque_off` then `close`, once.
Shutdown adds one `hold` first. The adapter never sees the signal itself.

| Exit | Adapter calls after the failure point |
|---|---|
| `check` error | none |
| `open` error | `torque_off` (no-op, not open), `close` |
| `zero` .. `torque_on` error | `torque_off`, `close` |
| shutdown | `hold`, `torque_off`, `close` |
| emergency stop | `torque_off`, `close` |
| controller panic | `torque_off`, `close` during unwind |

## R15. SRS problems found (reported, not edited)

New here (the coordination list already holds the rest, e.g. FR-4.1 hold target, the bus
watchdog, FR-6.1 hardware fields, FR-5.3 opt-in read):

1. **SRS FR-6.4** classifies a bad port binding as a bring-up error. The spec (FR-030) and
   kmr_api FR-024 make an unknown or missing bus label a `run()` error before any I/O. The SRS
   should say a binding that can be checked without I/O fails before I/O.
2. **SRS FR-5.2** "double-check zeros" is undefined. With a software zero (D8 C) there is nothing
   to re-read; the plan checks that the seed positions are inside the register range.
3. **SRS FR-5.8** says torque enable is "chosen by the user at bring-up" and read-only is "torque
   off". It does not say torque off is written when the actuator was found on. The plan writes it
   (R1). The SRS should say so, since a vendor SDK may not allow it (`dxl_rs` today).
4. **SRS FR-4.2** "cuts torque" covers every actuator; SRS FR-5.6 "torque off" on shutdown does
   not say read-only ones too. The plan cuts every actuator in both cases.

Spec problem (not SRS), resolved 2026-10-28: FR-027 and kmr_api FR-007 no longer claim a proc
macro is needed; a `const fn` message built byte by byte names the index (R4).

## R16. Hardware without a preset (D7). Assumed, pending human: D7 A

- **Decision**: a robot without a preset is declared in `main.rs` through `robot!`'s
  `hardware = ..` argument, built from this crate's `Table` types and `const fn` overrides
  (FR-025, FR-026 (SRS FR-6.1)). The syntax is kmr_api's (its research R7).
- **Here**: public `Table`, `Bus`, `Actuator` fields and `const fn` overrides, including the
  group forms (T021). Nothing else depends on D7.
- **Changing the answer**: B (`hardware!` macro) or C (builder calls) changes kmr_api's arms
  only; C would also turn `check` failures into `run()` errors.

## R17. Shipped bus line (D15). Assumed, pending human: D15 A

- **Decision**: the shipped `controllers/src/main.rs` has no bus line and simulates. The bench
  adds `.add_plugins(Bus::by_id("<U2D2 serial>"))` locally (quickstart § 5; controllers
  research R1).
- **Here**: nothing in this crate depends on it; quickstart § 5 and T044 say so.

## R18. Coordination log

- 2026-10-28 coordination: `run()` wiring (plan D2, T042) is kmr_api A5's, through the emitted
  `__Spec`; this plan edits nothing in kmr_api or controllers (T025, T034, T042, T043, T044
  became hand-offs).
- 2026-10-28 coordination: order inside B and C: B1, B2, B5, then C1 to C4, then B3, B4, since
  kmr_api A3 to A5 need `Binding` and `bind` for the first motor (plan; tasks phases 4 to 5b).
- 2026-10-28 coordination: T021 adds the group-slice overrides (kmr_api FR-020).
- 2026-10-28 coordination: kind check is `check_kinds(self, [u64; MASK_WORDS]) -> Self`
  (T058, plan E6, contracts/hardware-table.md).
- 2026-10-28 coordination: `overrides.read_every` dropped (T020, R6,
  contracts/hal-conformance.md); core K3.8 removes it.
- 2026-10-28 coordination: `RunError` is no longer called obsolete (plan old tasks, R13).
- 2026-10-28 coordination: D7 and D15 recorded as assumed (R16, R17); quickstart § 5 has no
  shipped bus line.
- 2026-10-28 coordination: spec FR-023 and its entities gain `label` and `zero`; FR-009 says
  K = 0 is a compile error by type; R4, R15 no longer call for a proc macro.
- 2026-10-28 coordination: spec FR-026 now lists `zero` (D8 C, assumed, pending human) and the
  group-slice forms for mode, torque choice and zero (kmr_api FR-020 (SRS FR-5.8)), matching
  T021 and contracts/hardware-table.md § Overrides. No `id` group form.
- 2026-10-28 coordination: new T067 (plan E10, data-model § Declared costs): `XmBench::READ_COST`
  and `WRITE_COST` as public consts, pinned by a test to the `Layout` costs, for kmr_api C7 and
  the bench (FR-035 (SRS NFR-11, FR-5.3)). No task provided them before.
- 2026-10-29 coordination: D17 decided: the adapter is one member of the robot's driver list;
  `check` returns its claim, not a whole-N layout (spec FR-036, FR-002, FR-024, Assumptions;
  plan E11; new T069; `specs/002-kmr-core/design-hal-set.md`).
