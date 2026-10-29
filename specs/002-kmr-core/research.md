# Research: kmr_core plan

Date: 2026-10-28. Inputs: [spec.md](spec.md), SRS module `kmr_core`, the coordination page,
[design-first-motor.md](design-first-motor.md), [design-const-generic-n.md](design-const-generic-n.md),
`specs/kmr-core-completion-plan.md`, the current `kmr_core/` code, the adapters and model plans.

Each item: Decision, Rationale, Alternatives. "Assumed, pending human: Dn" marks a choice the
human has not made; the "Change point" is the one place a different answer edits.

## R1. Delivery gate

- **Decision**: M1 is SRS delivery step 1: HAL call points, minimal state transition (write mask,
  one slot per tick, write after `Last`), exit paths. **Assumed, pending human: D1 (option A).**
- **Rationale**: SRS §8 "nothing below a line starts before the line above moves a motor" is
  circular: no motor moves without parts of steps 3 and 4.
- **Alternatives**: D1 B, keep the text and waive per case. Same work, other label.
- **Change point**: the milestone labels in plan.md. No design changes.

## R2. M1 before M2 (N = 4 first)

- **Decision**: build the first motor on `JOINTS = 4`, then make N a const generic.
- **Rationale**: coordination §3 puts `kmr_model` and N off the critical path. XmBench has 4
  actuators. The first-motor design names `JOINTS` and maps one-to-one onto `<const N>`
  (design-first-motor "Const generic N"). The const-generic bridge (`N = JOINTS` default) absorbs
  M1's new types (`Written`, `HoldTarget`, `Slot::mask`) without rework.
- **Alternatives**: N first. Puts a 10-commit refactor of three invariant files in front of the
  motor, with no gain for the bench.

## R3. How the two designs compose

- **Decision**: M2 generalises every M1 item by the rule of design-const-generic-n §2.1 ("every
  type that holds or borrows a per-actuator array carries `const N`"): `Written<N>`,
  `HoldTarget<N>`, `Sample<'_, N>`, `Command<'_, N>`, `Powered<'_, A: Hal<N>>`,
  `Slot::mask<const N>(&mut Written<N>) -> &mut Mask<N>` (already prototyped there, §2.4).
- **Rationale**: both prototypes used the same seam. The guard and the engine block in
  `robot_state.rs` are generic already through `RobotState`.
- **Risk**: K2.6 must also cover `run_on<A: Hal<N>>` and `Powered`. Both live in `robot.rs` and
  `runtime.rs`, which K2.6 edits anyway.

## R4. First-motor open choices (design-first-motor §6)

Assumed, pending human (not D-numbered). Each is the design's recommendation unless noted.

| OC | Assumed | Change point |
|---|---|---|
| 1 Hold target | latest sample overlaid with this tick's writes | `RobotState::hold_target` |
| 2 `write` on empty batch | skip the call | one `if` in the tick, after `Last` |
| 3 `impl Hal for &mut A` | yes (FR-050 option A, adapters D1 need it) | `hal.rs`, one impl + seal |
| 4 Estop check before `torque_on` | **yes, added as K1.9** (not in the prototype) | one `estop(host.poll())` in bring-up |
| 5 `last_overrun()` | previous tick's lateness, `None` if on time | `Time::last_overrun` |
| 6 `elapsed()` origin | from `arm` (tick 0's grid origin), not `Robot::new()` | `Time::elapsed`; the prototype used `startup` |
| 7 Startup-write warning | one warning per run, listing every phase that wrote (SC-011) | `discard_startup_writes` |

- **Rationale for OC4**: SRS FR-4.2 "next phase boundary, startup phases included". A pending
  estop must never power the actuators. Cost: one atomic load at bring-up.
- **Rationale for OC6**: FR-031 (SRS FR-5.10) says tick 0 is not charged with bring-up time.
  Measuring `elapsed` from the grid origin keeps that promise for the clock the user sees.

## R5. Joint-limit breach with no policy

- **Decision**: `run()` returns an error at finalization, before any I/O, unless every limited
  actuator has a policy. Envelopes need none: their breach clamps and flags (D3 A, R11). The
  breach path itself has no marker and no panic. **Assumed, pending human: D2 (option B)**, the
  same assumption in every module plan (kmr_api C4 adds `set_limit_policy_all`).
- **Rationale**: SRS NFR-2 (no panic with torque on) and SRS FR-6.4 (bad registration is a
  `run()` error). The model makes a limit mandatory per actuator, so in practice every robot
  must set policies through `kmr_api` (one group call can cover all).
- **Alternatives**: A, marker on the breach path plus the unwind guard (panics with torque on);
  C, pick a default now (the SRS says undecided).
- **Change point**: `limits::require_policies`, one function called from finalization (K5.3).

## R6. No-hardware run: no sample, not zeros

- **Decision**: History keeps an `ever_read: Mask<N>`. A History read of an actuator not in it
  returns `StateError::NoSample`. `NoHardware::read` refreshes nothing, so every read in a
  simulated run is `NoSample`. **Assumed, pending human: spec FR-042 recommendation.**
- **Rationale**: design-first-motor `push_slot` copies `prev(0)` and starts from
  `unwrap_or_default()`: zeros. Spec FR-022 (SRS FR-2.6) forbids zeros as data; FR-042 says the
  no-hardware adapter seeds nothing. The design's push stays; the gate sits on the read side.
  The same mask gives the per-actuator age (FR-018 (SRS FR-2.1)): age is `None` until first read.
- **Alternatives**: `Option` per slot (doubles the check, breaks SoA); seed zeros (SRS FR-2.6).
- **Change point**: `History::ever_read` and the read accessors in `robot_state.rs`.

## R7. SRS markers after `open`

- **Decision**: no `unimplemented!` on any path after `open`. The two `todo!()` home calls
  (`state/history.rs:24`, `state/robot_state.rs:59`) return `StateError::Undecided("SRS FR-2.7:
  home state source")`. The out-of-place check is skipped with one warning (FR-048).
  **Assumed, pending human: D5 (option A).**
- **Rationale**: a marker panics with torque on (SRS NFR-2). Constitution I still holds: the
  message names the SRS id, and CI can count `Undecided(` the same way.
- **Alternatives**: keep the marker (Constitution I literal); `compile_error!` behind a feature.
- **Change point**: `StateError::Undecided` and its two call sites.

## R8. Empty robot: a compile error, not a run-time check

- **Decision**: `Robot<const N, CS = Scheduled, E = Empty>`. Every add returns `E = Filled`.
  `run()` and `run_on()` require `E: HasController`, a sealed trait with
  `#[diagnostic::on_unimplemented]`. `schedule.rs` is not touched.
- **Rationale**: SRS NFR-4 and §8 decide type-state. AGENTS.md defended a run-time check because
  a raw trait bound leaks `frunk_core::...` paths; `on_unimplemented` puts a readable message
  first. A bound over six HList buckets would need a type-level OR and an edit of `schedule.rs`.
- **Proved** (scratch `kmr_core-plan/`, three crates core/api/user, rustc 1.97.1): the user
  crate's empty `Robot::new().run()` fails under `cargo check` with

  ```text
  error[E0277]: kmr: this robot has no controller
   7 |     Robot::new().run().ok();
     |                  ^^^ `run()` needs at least one controller
     = note: add one with `.add_controller(EachTick, f)` before `.run()` (SRS NFR-4)
  ```

  through a `kmr_api`-style wrapper, next to a const generic N and the `robot!` alias. The
  non-empty build has 0 warnings. The core crate builds `#![no_std]` for `thumbv7em-none-eabihf`.
- **Alternatives**: run-time `L::LEN == 0` (SRS overrules it); `do_not_recommend` (not needed).
- **Cost**: `kmr_api` threads one more parameter; users never name it (`robot!` emits
  `Robot<N>`).

## R9. `run()` error and `Overrides`

- **Decision**: `RunError<E> { Config(ConfigError), Hal(E) }`. `ConfigError` holds `DtOutOfRange`
  and `NoPolicy { actuator }` (D2). `ZeroReadCadence` is dropped. `Overrides` loses
  `read_every` and stays an empty `#[non_exhaustive]` struct; no baud field is added.
- **Rationale**: spec FR-045 puts the cadence in the hardware table, never in `Overrides`. The
  adapters plan (R6) takes the plugin baud at `bind`, so the core needs no baud override either.
  Spec FR-039 still says "overrides carry the plugin baud": the adapters plan supersedes it.
- **Alternatives**: keep `read_every` (two sources for one fact).
- **Change point**: `hal::Overrides`. The adapter drops its `overrides.read_every` branch.

## R10. External flags: one atomic word

- **Decision**: `ExternalFlags(AtomicU32)`, `raise(bits)` = `fetch_or(Release)` from any thread,
  `take()` = `swap(0, Acquire)` by the loop at the inbound stage. `#[doc(hidden)] pub`, reached
  by `kmr_api`, not re-exported.
- **Rationale**: FR-052 (SRS FR-13.4): no lock, only the loop writes History.
- **Proved**: builds on x86-64 and `thumbv7em-none-eabihf`. On `thumbv6m-none-eabi` it fails
  (`E0599 no method named fetch_or`, `swap`): no CAS there (see R13).

## R11. Envelope breach

- **Decision**: clamp to the envelope and set a new per-actuator `envelope` bit in `Faults<N>`,
  as kmr_api's `Faults` view and kmr_ffi's `kmr_actuator_flags` expect. **Assumed, pending
  human: D3 (option A).**
- **Rationale**: SRS FR-13.5 asks enforcement before every write; FR-13.4 has no envelope flag,
  so the bit is added (SRS problem, R19).
- **Change point**: the `Breach::Envelope` arm of the pre-write stage and one `Faults<N>` mask.

## R12. Mask width

- **Decision**: `Mask<const N>([u64; MASK_WORDS])`, `MASK_WORDS = 1`, N in 1..=64; check in
  `robot!` (fires under `cargo check`) and a backstop `Mask::<N>::CHECK` in `Robot::new`
  (`cargo build` only). **Assumed, pending human: D4 (option A).**
- **Rationale**: proved in design-const-generic-n (r1, r3, r4). Widening is one constant.
- **Change point**: `hal::MASK_WORDS` and the assert message.

## R13. `no_std` target and seam

- **Decision**: CI target `thumbv7em-none-eabihf`. `std` is a default feature. Under
  `no_std`: `frunk_core` without default features, `Host` public and sealed, an own monotonic
  `Instant` from the Host, tracing a no-op, `ctrlc` and `tracing-subscriber` gated, no threads.
  `signal.rs:44-45` imports move from `std::` to `core::` (invariant file, imports only).
  **Assumed, pending human: SRS §8 "reference MCU target".**
- **Rationale**: SRS FR-4.4 needs compare-and-swap. `thumbv6m` (Cortex-M0, STM32F0) has none:
  proved, `fetch_or`/`swap` do not exist there (R10). `thumbv7em` (nRF52840 class) has it.
- **Alternatives**: `portable-atomic` with a critical section (a lock; SRS NFR-1 forbids it).
- **Note**: `lib.rs` doc says the core "is not meant to be no_std". Stale against SRS NFR-7.

## R14. Hold duration without a Host change

- **Decision**: `Some(d)`: the runtime sleeps to `min(hold_end, now + dt)` and polls the bus
  after each slice; an estop ends the wait. `None`: no wait (default).
- **Rationale**: FR-033 (SRS FR-4.1). Keeps the `Host` seam at three methods; the wait is
  bounded by one `dt` (SC-010).

## R15. Stage points for `kmr_api`

- **Decision**: a public trait `Stages<const N>` with `inbound(&mut self, &mut RobotState<N>, &Time)`
  and `outbound(&mut self, &RobotState<N>, &Time)`, and `impl Stages<N> for ()`. `outbound`
  reads depth 0, its flags and the per-actuator age (`Option<u32>`) through the read-only
  accessors; a bare slot would not carry the age. `run_on` uses `()`; `run_on_staged(hal,
  stages)` takes `kmr_api`'s. Static dispatch, no heap.
- **Rationale**: FR-053 (SRS §4, FR-7.1). The no-op impl compiles away; no middleware named.
- **Alternatives**: two closures (unnameable types in `kmr_api`'s `Robot`); a `dyn` hook (banned).

## R16. Non-finite writes

- **Decision**: the core stores any `f32`. The pre-write stage treats a non-finite value as a
  breach once M5 lands; before that, the adapter flags it `rejected` (unencodable target).
- **Rationale**: kmr_ffi FR-013 asks the core's answer; the SRS is silent. No panic, no silent
  send.
- **Change point**: one `is_finite` test in the pre-write stage.

## R17. Pre-sort gain

- **Decision**: opt-in `presort-off` feature; `Batch::new` keeps declaration order under it.
  **Assumed, pending human: D11 (option B).**
- **Rationale**: bench FR-016 cannot isolate the gain from a black box (SRS NFR-8 vs FR-5.3).

## R18. Constitution conflicts (recorded, not fixed)

1. Principle 0 requires an agent trailer on agent commits; AGENTS.md rule (b) and the human's
   decision forbid it. Core commits are human, so this module does not hit it.
2. Principle I delivery order vs D1 (R1).
3. AGENTS.md runtime empty check vs SRS NFR-4 type-state (R8).

## R19. SRS problems found

New in this plan (beyond the coordination page §2):

1. **FR-6.3 / FR-2.6 vs the push design**: a simulated run pushes zeroed slots. The SRS should
   say "no sample" until the first read (R6).
2. **FR-13.4** lacks an envelope flag (D3) and the `rejected` flag that `hal.rs` has.
3. **FR-4.4 on MCUs**: CAS excludes Cortex-M0. Section 4 should state "target with CAS" (R13).
4. **FR-3.3** `elapsed()` origin and `last_overrun()` meaning undefined (R4 OC5, OC6).
5. **FR-4.2** silent on an estop pending during PostInit bring-up (R4 OC4).
6. **FR-4.1** silent on what the hold is built from (R4 OC1).
7. **FR-6.4 / FR-4.5**: a mandatory limit (model) and no default policy make every robot set
   policies before `run()` under D2 B. The SRS should say so.
8. **NFR-7 / lib.rs**: crate docs deny `no_std`.
9. **FR-2.2**: non-finite values have no rule (R16).
10. **§8 decision 0001** still says N is a constant (decision record 0003, K2.0).
11. **NFR-4 vs AGENTS.md** on the empty check (R8).
12. **FR-5.3 / FR-039**: plugin baud override has no consumer (R9).

## R22. Spec gap: `FieldId`

kmr_ffi FR-014 (SRS FR-10.2) needs an exhaustive core field id, so a field without its FFI line
fails the build. Plan K7.1 (HUMAN, `state/field.rs`, invariant) carries it. Closed 2026-10-28:
spec FR-062 (SRS FR-10.2, FR-2.2), at the end of the State section. It also says only
`kmr_api` names the id, so an exhaustive id does not break FR-014's "extensible without breaking
users".

## R20. Completion plan (2026-10-27): what still holds

| # | Status | Where now |
|---|---|---|
| 1 | Changed: typed error, not `unimplemented!` (D5) | K3.1 |
| 2, 3 | Valid, refined by design-first-motor | K1.3 to K1.5 |
| 4 | Valid, plus `ever_read` | K3.5 |
| 5 | **Superseded**: cadence in the hardware table, no `set_read_every` | K3.8 |
| 6 | Valid | K3.2 |
| 7 | Changed: no `ZeroReadCadence` | K3.3 |
| 8 | Valid, refined (`run_on` returns `A::Error`, then `RunError`) | K1.6, K3.3 |
| 9, 10, 11 | Valid, refined (`Powered` guard, five checks, OC4) | K1.2, K1.7 to K1.9 |
| 12 | **Superseded**: merged into the design's `Recorder` tests | K1.6 to K1.9 |
| 13 | Valid, plus envelope bit | K4.1, K4.4 |
| 14 | Valid | K1.1 |
| 15 | Valid, mechanism changed (`E` parameter, no `schedule.rs`) | K3.4 |
| 16, 17 | Valid | K3.10, K3.9 |
| 18 | **Superseded** by design-const-generic-n (const generic, `DEPTH` dropped) | M2 |
| 19 | Out of version (FR-2.7 open); check skipped with a warning | FR-048 |
| 20 | Changed by D2 B and D3 A | K5.2, K5.3 |
| 21 | Valid | K3.6 |
| 22 | Out of version (SRS §8 open) | – |
| 23 | Out of version (Sensors surface open) | – |
| 24 | Valid, target assumed, `signal.rs` imports named | K6.1, K6.2 |
| 25 | Reduced to a review | K6.4 |
| 26 | Valid | K3.7 |
| 27 | **Superseded** by FR-050 option A (`impl Hal for &mut A`) | K1.6 |

Risks of its §4: 1 resolved (const generic), 2 moved to `kmr_api`, 3 out of version, 4 kept (R13),
5 resolved (R8), 6 fixed by K1.8, 7 fixed by K3.2, 8 fixed by K3.9, 9 kept (K1.10), 10 resolved
(FR-050), 11 open (naming), 12 stale fix-plan, superseded.

## R21. Prototypes and results

| Claim | Where | Result |
|---|---|---|
| First-motor design (M1) | `scratchpad/critpath/` (human design) | 219 passed, 0 failed, 7 ignored (191 before) |
| Const generic N (M2) | `scratchpad/constn/` (human design) | 8 tests; N = 3, 5, 10, 64; `no_std` on `thumbv7em` |
| Type-state empty check (K3.4) | `scratchpad/kmr_core-plan/` (this plan) | E0277 with the custom message under `cargo check`; 0 warnings on the filled build |
| Pending word, `no_std` (K4.2, K6.1) | same | builds `thumbv7em-none-eabihf`; fails `thumbv6m-none-eabi` (E0599) |

The first two rows were not rerun here; their numbers are the designs' own, confirmed by the main
agent on the coordination page.

## R23. Coordination log

- 2026-10-28 coordination: K1.6 now says `NoHardware` prints "no bus plugin: simulating" at
  `open`; kmr_api A0 waits on it.
- 2026-10-28 coordination: envelope flag moved from `TickFlags` to `Faults<N>`, per actuator
  (K4.1, R11, data-model).
- 2026-10-28 coordination: `Stages::outbound` takes `&RobotState<N>` (K5.1, R15,
  engine-surface), so age is reachable.
- 2026-10-28 coordination: `Limit { lower: Option<Q>, upper: Option<Q>, .. }`, typed arrays,
  `set_envelopes` in the engine surface (K5.2); continuous joints are never clamped to 0.
- 2026-10-28 coordination: D2 B wording aligned to "every limited actuator" (R5).
- 2026-10-28 coordination: M7 K7.1 `FieldId` added (R22).
- 2026-10-28 coordination: spec FR-062 (SRS FR-10.2, FR-2.2) added for `FieldId`, HUMAN,
  invariant `state/field.rs`; K7.1 cites it; SRS coverage rows FR-2.2 and FR-10.2 updated (R22).
- 2026-10-29 coordination: D17 decided: the hardware is a list of drivers, each claiming an
  index set; new FR-063, FR-039/041/043/044 reworded, plan KS.1, new T069 (HUMAN), dated notes
  in contracts/hal.md and engine-surface.md ([design-hal-set.md](design-hal-set.md)).
