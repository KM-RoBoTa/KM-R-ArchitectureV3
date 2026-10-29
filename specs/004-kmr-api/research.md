# Research: kmr_api

Each entry: decision, why, what else was weighed. Spec ids first, SRS ids in brackets.
"Assumed, pending human: Dn" points at `specs/spec-coordination-2026-10-28.md` §1.

Prototypes live in the scratchpad, never in the repo:
`/tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/kmr_api-plan/`
(`proto/`, `slot/`, `marker/`). rustc 1.97.1, `--offline`. `proto/` builds on the core of
`scratchpad/critpath/` (design-first-motor applied: `run_on`, Time getters, drop guard).

## R1. `run()` hands the adapter to `run_on` (critical path step 9)

- **Decision**: `kmr_api::Robot::run()` does, in order: the API-side finalization checks (R13,
  duplicate port), then one of two calls. No bus plugin: `inner.run_on(NoHardware)`. Plugins:
  `H::bind(&plugins)?`, then `inner.run_on(adapter)`. The adapter error becomes
  `UserError::BringUp` (R4). Both `eprintln!` lines in today's `run()` go: the no-plugin one is
  kmr_core FR-042's, the "not driven yet" one ends with this change (FR-026 (SRS FR-6.3)).
- **Why**: FR-005 (SRS FR-6.1, FR-5.1), FR-014 (SRS section 3 "run() finalization"), FR-026,
  FR-027 (SRS FR-6.4, FR-5.5). The core names no adapter (kmr_core FR-041); `kmr_api` picks it.
- **Proved** (`proto/user/tests/run.rs`, 6 tests, both feature settings green):
  - no plugin: runs on `NoHardware`, `shutdown!` at tick 3, `Ok(())`; the controller saw
    `tick()` 0..=3 and `dt()` = 1 ms through the forwarded getters;
  - plugin with the adapter feature on: one `open`, one write per tick (4), one `torque_off`;
  - `Bus::by_id("MISSING")`: `Err(BringUp { cause: "no adapter with serial MISSING on this
    machine" })`, and the core's guard still called `torque_off`;
  - a two-bus table with one plugin: the adapter's binding error reaches the user unchanged.
- **Blocked on**: HUMAN core commits 1 to 8 of `specs/002-kmr-core/design-first-motor.md`
  (`run_on`, Time getters, exit paths); kmr_adapters plan steps A (torque off) and C1/C2
  (`Binding`, `Dynamixel::bind`).
- **Rejected**: `run()` calling `kmr_adapters::Hardware::default()` (the design note's
  sketch). That alias picks hardware by Cargo feature and reads ports from a const, which
  FR-005 and FR-022 (SRS FR-6.3) both rule out. kmr_adapters plan B5 removes it.

## R2. The adapter follows the robot that `robot!` names

- **Decision**: `kmr_api::Robot<H, C = Scheduled>`. `H` is a hidden trait bound,
  `#[doc(hidden)] pub trait Spec`, that says how this robot builds its hardware:
  `type Drivers` (a list of `Hal` members, one per brand; one adapter type until 2026-10-29,
  `specs/002-kmr-core/design-hal-set.md`) and
  `fn bind(&[Binding]) -> Result<Self::Drivers, UserError>`. Every `robot!`
  expansion emits one private `struct __Spec` and its `impl Spec`, and binds
  `pub type Robot = kmr_api::Robot<__Spec>`. User signatures never see `H`: controllers take
  `State`, `Time`, `Sensors` only.
- **Feature read in kmr_api**: the expansion names `$crate::__DxlAdapter<__NB>` and calls
  `$crate::__bind_dynamixel(..)`. Both are `cfg(feature = "dynamixel")`-selected inside
  `kmr_api`, never inside the user's crate (a `cfg` written in a `macro_rules!` expansion would
  test the user's features). Proved: the same user code passes its feature-off test
  (`FeatureOff`) and its feature-on test (adapter driven) by toggling `api/dynamixel` only.
  A type alias with an unused const parameter (`type __DxlAdapter<const NB: usize> =
  NoHardware`) compiles.
- **After N becomes a const generic**: `Robot<const N: usize, H, C = Scheduled>`; `Spec` gains
  the model data of R9 as `Spec<N>`. N must stay a separate parameter: an associated const
  cannot size an array in a generic context (design-const-generic-n r2). After core K3.4 it
  also threads the core's type-state: `Robot<const N: usize, H, C = Scheduled, E = Empty>`,
  `E` in `Empty`, `Filled` (kmr_core contracts/engine-surface.md). Users never name it.
- **Rejected**: a Cargo feature that picks the robot (FR-005 forbids a second switch); `dyn Hal`
  in the builder (Constitution II, IV); a run-time registry of presets (needs a lookup by name).

## R3. Simulation, missing feature, missing hardware

- **Decision**:

  | Plugins | Adapter feature | Hardware declared | `run()` |
  | --- | --- | --- | --- |
  | none | any | any | simulates on `NoHardware`; the core's one warning |
  | some | off | yes | `Err(FeatureOff { feature: "dynamixel" })`, before I/O |
  | some | on | yes | binds, drives the hardware |
  | some | any | no (`robot!(model = ..)` without `hardware`) | `Err(NoHardwareDeclared)`, before I/O |

- **Why**: plugins, never a feature, choose hardware or simulation (kmr_adapters FR-031, R9).
  A user who declared a port must never get a silent simulation.
- **Proved**: rows 1, 2, 3, 4 (`proto`, R1).

## R4. Errors

- **Decision**: `UserError` keeps `DuplicateBus { port }` and grows `FeatureOff { feature }`,
  `NoHardwareDeclared`, `BringUp { cause: String }`, `DtOutOfRange { dt }` (R13),
  `UnknownLink { link }` (R11), `NoPolicy { actuator }` (R17, D2 B, after core K5.3). `NoControllers` goes when the trait bound lands (FR-016,
  kmr_core #15). `BringUp` holds the adapter's `Display` text: heap is fine outside the tick
  (FR-021 (SRS FR-6.4, NFR-2)); the adapter already hides SDK types. `Debug` prints the
  `Display` text, so `fn main() -> Result<(), UserError>` shows a readable line
  (controllers research R12).
- **Rejected**: `UserError<E>` generic over the adapter error (leaks a type parameter into
  every `main`); `Box<dyn Error>` (Constitution IV prefers no trait objects; a `String` is
  enough for a message).

## R5. Bare `kmr_api::Robot::new()` stops compiling

- **Decision**: with `H` generic, `kmr_api::Robot::new()` without `robot!` fails:
  `error[E0283]: type annotations needed for Robot<_>` (proved, `proto/user/examples/bare.rs`).
  That is already the end state of design-const-generic-n (r6, E0284). `robot!` aliases become the only
  user path one step earlier. kmr_api's own tests switch to `robot!(XmBench)`.
- **Order**: `controllers/src/main.rs` adopts `robot!(XmBench)` first (controllers plan C0,
  unblocked; `robot!` exists today), then kmr_api lands `Robot<H>` (A4). No commit breaks the
  workspace.
- **Rejected**: `impl Robot<Sim> { fn new }` beside `impl<H: Spec> Robot<H> { fn new }`
  (duplicate definitions); a separate public name for the generic struct (users would see two
  builder names in errors).

## R6. The example's bus plugin line

- **Decision**: the example keeps `.add_plugins(Bus::by_id("FT8ISZ6H"))`, wrapped in
  `cfg!(feature = "dynamixel")` (`add_plugins` returns `Self`, so both branches have one type).
  `cargo run --example complete_api_example` simulates; with `--features dynamixel` it drives
  the bench. Isolated in the example, one `if`.
- **Why**: FR-069, SC-001 (SRS FR-8.4): the example builds and runs after every change. With
  R3, an unconditional plugin line would make the default run return `FeatureOff`.
- **Pending**: same spirit as D15 (controllers ship no bus line). If the human wants the line
  unconditional, the example's documented run command gains `--features dynamixel`.

## R7. Hardware without a preset and preset overrides. Assumed, pending human: D7

- **Decision** (D7 option A, the `robot!` hardware argument):

  ```rust
  kmr_api::robot!(XmBench);                                        // preset table as is
  kmr_api::robot!(XmBench, hardware = |hw| hw.read_only(3).baud("bus0", 3_000_000));
  kmr_api::robot!(model = "robot/arm.urdf", hardware = Table { buses: [..], actuators: [..] });
  kmr_api::robot!(model = "robot/arm.urdf");                       // simulation only (R3)
  ```

  The `|hw| ..` form is not a closure: the macro rewrites it to
  `{ let hw = <Preset>::HARDWARE; .. }` in a `const`, so every override is a `const fn` of
  kmr_adapters' `Table` and fails at compile time. Group slices go through a const override that
  takes an index array (kmr_adapters need, e.g. `read_only_group(&GROUP)`), FR-020 (SRS FR-5.8).
- **Expansion** (`@table`): `__NB` and `__M` read from the preset table (or the literal), then
  `const __TABLE: Table<__NB, __M> = <expr>;`, then the count check
  `const _: &[Actuator; N] = &__TABLE.actuators;` (FR-007), then `impl Spec for __Spec`.
- **Proved** (`proto`):
  - overrides reach the table: `read_only(3).read_only_group(&[0, 1])` gives
    `[true, true, false, true]`, `(__NB, __M, N) = (1, 4, 4)`;
  - a 3-actuator preset against N = 4: `error[E0308]: mismatched types ... expected an array
    with a size of 4, found one with a size of 3`, under `cargo check`;
  - `hw.read_only(7)`: one `error[E0080]: evaluation panicked: hardware table: read_only index
    past N`, under `cargo check`. Reading `__NB`/`__M` from the preset, not from the override,
    keeps it to one error (reading them from the override gave two).
- **One place to change**: the `hardware = ..` arms of `robot!` and the `@table` arm. Option B
  (`hardware!` macro) replaces those arms with a second macro that emits the same `__Spec`;
  option C (builder calls) replaces `Spec::bind` with a run-time table and turns the count check
  into a `run()` error.
- **Rejected**: a real closure (`const` closures are unstable); naming `Table<NB, N>` in the
  user's call (the bus count is noise for the user).

## R8. Which bus a plugin binds. Assumed, pending human: D6

- **Decision** (D6 option B): `Bus::by_id(..).bus("label")`, optional for a one-bus robot.
  `kmr_api::Bus` wraps kmr_adapters' `Binding { port, baud, bus }` and forwards it. All matching
  is in kmr_adapters `binding::match_plugins` (its research R5). kmr_api adds nothing to it.
- **One place to change**: `Bus::bus` in kmr_api (removed under option A or C) and
  `match_plugins` in kmr_adapters.
- **Duplicate port**: kmr_api keeps its string check before `bind` (FR-025 (SRS FR-6.4)). It
  cannot see `by_id("X")` and `by_dev("/dev/serial/by-id/..X")` as one device; the adapter's
  `SharedPort` check at resolution catches that (review 2026-10-28 #8).

## R9. N cap and the const-generic transition. Assumed, pending human: D4

- **Decision** (D4 option A): the `robot!` expansion carries
  `const _: () = assert!(N >= 1 && N <= 64, "SRS write mask: more than 64 actuators");`
  (kmr_core FR-012). One line in the `@items` arm. Option B changes the bound, nothing else.
- **Transition**: until kmr_core takes N, the temporary `N == JOINTS` assert stays.
  After the HUMAN core steps 1 to 7 of design-const-generic-n: `State<'a, N>`,
  `GroupView<'a, N, K>`, `Robot<N, H, C>`, `kmr_api::N` removed, the assert removed, the
  `three.urdf` compile-fail doctest becomes a passing test (design step 8).
- **Model data**: kinds and limits travel as `Spec` associated consts
  (`const PRISMATIC: [u64; MASK_WORDS]`, `const LOWER: [Q; N]`, ...) copied from the `model::*`
  items of kmr_model's expansion. Data only: no policy (FR-002 (SRS FR-6.1)).
  `[u64; N.div_ceil(64)]` inside `Spec<N>` does not compile on stable; `MASK_WORDS` is the
  core's one constant (1 under D4 A).
- **Kinds**: `const _: Table<__NB, __M> = __TABLE.check_kinds(model::PRISMATIC);` with
  `check_kinds(self, [u64; MASK_WORDS]) -> Self` (kmr_adapters E6). Proved: a prismatic bit
  gives `error[E0080]` with the table's message; `const _: () = __TABLE.check_kinds(..)` is
  `error[E0308]`, since the call returns the table.
- **Limits**: typed `[Limit; N]` handed to the core's `set_limits` at finalization (kmr_core
  K5.2, HUMAN). `Limit { lower: Option<Q>, upper: Option<Q>, speed: Qd, effort: Tau }`. Where
  `model::CONTINUOUS` has bit i, `lower` and `upper` are `None`: the emitted `Q(0.0)`
  placeholder never reaches the stage, so a continuous joint is never clamped to 0.

## R10. Home state markers. Assumed, pending human: D5

- **Decision** (D5 option A, no `unimplemented!` after `open`): `initial_state()` and
  `go_home(ramp)` keep their names and argument shapes, but carry a bound no type meets:

  ```rust
  #[diagnostic::on_unimplemented(message = "kmr: SRS FR-2.7: home state is open; ...")]
  pub trait HomeDecided: sealed::S {}
  pub fn go_home<R: HomeDecided>(&mut self, ramp: R);
  ```

  The library builds; every call site fails to compile with the SRS id. Proved (`marker/`):
  `cargo build --lib` passes; `s.go_home(90)` and `let _: [f32; 4] = s.initial_state()` each
  give `error[E0277]: kmr: SRS FR-2.7: home state is open; ...`.
- **Consequence**: FR-036 changes from a run-time marker to a compile-time one, and the example's
  `initial_state()` / `go_home()` calls leave the example until SRS FR-2.7 is decided (they are
  in `Init` controllers, after `open`). One file, `home.rs`, plus those example lines.
- **Option B** (keep run-time markers) restores `unimplemented!("SRS FR-2.7: home state")`
  bodies in `home.rs`; the example calls stay and the example panics in `Init` under the
  core's guard, so SC-001 "runs" fails. That is why A is planned.

## R11. Payloads (FR-062 to FR-066 (SRS FR-8.1 to FR-8.4))

- **Decision**: `Payload::medium(mass_kg, "link")`, `Payload::heavy(mass_kg, "link")`. No light
  tier. `register_payload` stores it (heap allowed, outside the tick). Finalization checks each
  mount link against `Spec::LINK_NAMES` (kmr_model data-model, `model::LINK_NAMES`); unknown is
  `UnknownLink { link }`, before any I/O.
- **Center of mass**: the `unimplemented!("SRS FR-8.3: center of mass")` marker sits in the
  recompute function, which finalization does not call in this version. Calling it at
  finalization would panic every program that registers a payload, the example included
  (FR-069 requires it to register one). The model has no consumer for a new center of mass yet.
  Reported as a spec problem (R24).
- **Rejected**: a payload registered from an `Init` controller (SRS FR-8.4 forbids it).

## R12. Time and units (critical path)

- **Decision**: `kmr_api::Time` gets a private named field and forwards the six core getters by
  value: `elapsed()`, `dt()`, `tick()`, `overrun_count()`, `last_overrun()`,
  `accumulated_overrun()` (FR-038 (SRS FR-3.3)). Their meaning (origin of `elapsed`,
  `last_overrun` sticky or not) is the core's open choice 5 and 6; kmr_api adds none.
- **Units**: `Deg(pub f32)`, `Cm(pub f32)`, `M(pub f32)` and the per-second forms `DegPerS`,
  `CmPerS`, `MPerS`. `From<Deg> for Q` (rad), `From<Cm> for Q` and `From<M> for Q` (mm), and the
  `Qd` forms. No `Tau` conversion (spec Assumptions). The orphan rule allows `From<Deg> for Q`
  because `Deg` is local (controllers research, proved there). On the critical path: the
  controllers demo needs `Deg` and `elapsed()`.
- **SRS issue 4 stays**: `From<Deg>` into a prismatic actuator's `Q` is not caught by the type.

## R13. `dt` range check (FR-015 (SRS FR-3.1))

- **Decision**: owner kmr_core (#6, #7). Until `RunError` exists, `run_on` returns only the
  adapter's error, so kmr_api checks the range itself in `finalize()`, before `bind`: outside
  [10 µs, 1 s] or zero is `DtOutOfRange`; unset stays the core's 1 ms warning. When core #7
  (K3.3) lands, `finalize()` drops its check and maps the core's `RunError<E>`:
  `Config(DtOutOfRange)` and `Config(NoPolicy)` to the `UserError` of the same name, `Hal(e)` to
  `BringUp`. One function.
- **Why**: FR-015 must hold from the first motor; a 0 dt that the core turns into 1 ms with a
  warning would hide a user bug.

## R14. Reaching the adapter before and after `run()` (FR-041 (SRS FR-5.9))

- **Decision**: `Robot::bind(self) -> Result<Bound<H, C>, UserError>` runs finalization and
  `Spec::bind` (no I/O). `Bound::adapter(&mut self) -> &mut H::Adapter`.
  `Bound::run(self) -> Result<H::Adapter, UserError>` runs and hands the adapter back.
  `Robot::run()` stays `self.bind()?.run().map(drop)`. Never inside a tick (kmr_core FR-050).
- **Needs**: the core's `impl<A: Hal> Hal for &mut A` (design-first-motor open choice 3), or a
  `run_on` that returns the adapter. Either fits one private function, `drive()`, in
  `robot.rs`. Off the critical path.

## R15. The two Zenoh slots (FR-044 (SRS section 4 Constraints))

- **Decision**: a triple buffer per direction: Desired in (gateway thread writes, tick takes),
  depth 0 out (tick puts, gateway thread takes). Writer: one atomic swap per put. Reader: one
  load when nothing is new, one swap when something is. An untaken value that gets overwritten
  is counted (superseded Desired, missed sample).
- **Proved** (`slot/`, a hand-written 40-line version): `take()` returns a value once, then
  `None`; two puts before a take count one superseded. A writer thread put 200 000 values of
  `{ seq, [f32; 64], mask }` while the reader took 2 000 000 times: values strictly increasing,
  never torn, **0 allocations on the reader thread** (counting global allocator, three release
  runs and one debug run).
- **Implementation**: use the `triple_buffer` crate if its writer can tell an untaken value
  (its `consumed()` on the input side), so kmr_api carries no `unsafe` of its own. Else the
  proved version, with its `unsafe` blocks marked for human review. `Arc` is allowed in kmr_api
  (`kmr_api/clippy.toml`).
- **Rejected**: a `Mutex` or a channel read in the tick (SRS NFR-1); `AtomicPtr` with a fresh
  `Box` per message (allocates per message, frees on the tick thread).

## R16. Zenoh gateway shape (FR-043 to FR-048 (SRS FR-7.1 to FR-7.3))

- **Decision**: Cargo feature `zenoh` = `dep:zenoh` (+ the slot crate), off by default. The
  gateway is a plugin: `add_plugins(Gateway::new(config))`; `add_plugins` becomes generic over a
  sealed `Plugin` trait implemented by `Bus` and `Gateway`. The gateway thread owns the Zenoh
  session, a ring of the last `HISTORY_DEPTH` samples it took (each with its tick index, so gaps
  show) and the counters.
- **Stage points**: the core calls kmr_api's inbound stage after the read and before `First`,
  and the outbound stage after the write (kmr_core FR-053, HUMAN). Inbound takes the Desired
  slot once and writes the masked values through the State handle; local controllers write
  later in the tick and win (spec Assumptions). Outbound copies depth 0, its age and its flags
  into the slot.
- **Keys**: `kmr/<instance>/history`, `kmr/<instance>/history/ring` (queryable),
  `kmr/<instance>/desired`. Per-group keys wait (FR-048, SRS issue 6).
- **Rejected**: inbound as a hidden first `First` controller (user `First` controllers
  registered before the plugin would run ahead of it, and outbound cannot be a controller, it
  must follow the write).

## R17. Fallbacks and envelopes (FR-049 to FR-053 (SRS FR-13.5)). Assumed, pending human: D2, D3

- **Fallback**: `add_controller_or(phase, controller, when, fallback)`. The node is
  `Inline<ApiInline<Guarded<F, G, P>>>`: a named struct, so the builder's return type stays
  expressible. `Guarded::call` reads the tick's fault flags and calls `fallback` or `controller`.
  Predicates are plain `Fn(&Faults) -> bool` values; helpers `stale_in(&GROUP)`,
  `remote_timeout(source)`, `any_of`. No `dyn`, no heap, no core dispatch change.
- **Hold policy**: not offered (FR-050); the method that would take it does not exist.
- **Envelopes**: `set_envelope(index, Envelope { q, qd, tau })` and `set_envelope_group(&GROUP,
  ..)`, stored as per-actuator arrays, handed to the core's pre-write stage (kmr_core #20).
- **D3 option A** (clamp and raise an envelope flag): the API passes only data; the breach action
  lives in the core's stage. A different D3 answer changes the core's stage and the flag list,
  not this API. The flag needs a new SRS FR-13.4 entry (SRS problem in the coordination list).
- **D2** (joint-limit breach with no policy). Assumed, pending human: D2 option B, aligned
  across all plans: `run()` refuses, before any I/O, unless every limited actuator has a policy
  (core K5.3, `RunError::Config(NoPolicy)`, mapped to `UserError::NoPolicy { actuator }`). The
  API offers `set_limit_policy(index, ..)`, `set_limit_policy_group(&GROUP, ..)` and one call
  for every actuator, `set_limit_policy_all(Clamp | Warn | Abort)` (C4). No default. The model
  makes a limit mandatory per actuator, so the bench and the controllers example each need the
  one call. Under A or C, `set_limit_policy_all` stays and only the core changes.

## R18. Security (FR-054 to FR-061 (SRS FR-13.2, FR-13.3, NFR-12))

- **Decision**: Zenoh with mutual TLS or QUIC; per-machine certificate and key paths are given
  to `Gateway::new` (files on the machine, never in source or descriptor). Transport ACL denies
  by default. kmr_api applies the Desired rule again after the transport authenticated the peer,
  on the gateway thread, so every drop is counted and the unauthorized flag set (FR-055,
  SRS issue 7). Sequence numbers per source: at or below the last accepted is dropped and
  counted. Timestamps carried and logged, not checked (deferred 2026-10-27).
- **FR-059 at compile time**: a remote source exists only through
  `add_remote_source(phase, Remote::allow(identity).timeout(d), fallback)`. The fallback is a
  required argument and is inserted as a guarded node (R17) with predicate
  `remote_timeout(source)`. A source without a fallback cannot be written.
- **Watchdog**: on the gateway thread; on silence it sets the remote-timeout bit through the
  core's lock-free pending word (kmr_core FR-052). The tick sees it at the inbound stage.

## R19. FFI feature (FR-072 (SRS FR-10)). Assumed, pending human: D12, D13

- **Decision**: feature `ffi` = `dep:kmr_ffi`, re-exported as `kmr_api::ffi`; glue
  `ffi::call(f, time, state, ctx)` fills the view, calls, applies, maps `Status` to `shutdown!`
  or `emergency_stop!` (D13 A). Feature `ffi-build` re-exports kmr_ffi's build helper for a
  `build.rs` that only compiles foreign code (D12 A). Details: kmr_ffi plan C2 to C4.
- **Bridges**: features `ffi-python` and `ffi-lua` turn on kmr_ffi's `python` and `lua`
  features and re-export their `Bridge` types (plan G2; kmr_ffi contracts/rust-surface.md).
- **One place to change**: `ffi.rs` and the feature lines.

## R20. `try_lock` in the example. Assumed, pending human: D16

- **Decision** (replace): the example's gamepad hand-off uses the latest-value slot of R15,
  made public as `kmr_api::Latest<T>` (reader in the inline controller's context, writer in the
  threaded context). The tick takes no lock (SRS NFR-1, Definitions "Context").
- **One place to change**: the example and the `pub` on `Latest`. Under "keep", `Latest` stays
  `pub(crate)` and the example keeps `try_lock`.
- **Note**: threaded controllers are never spawned yet (SRS FR-1.2 open), so the writer side
  does not run either way.

## R21. Delivery order. Assumed, pending human: D1

- **Decision** (D1 option A): delivery step 1 is the HAL contract, the adapter, a minimal state
  transition and the exit paths. kmr_api's part of step 1 is R1, R4, R12, R13 (phase A of the
  plan). `robot!` hardware forms and model data (step 2) start after the first motor moves.
  Steps 5, 6, 7, 8 follow in order.
- **Exception already in the tree**: uncommitted `robot!` code (`robot_macro.rs`, `bus.rs`,
  tests) predates the gate; review 2026-10-28 lists its fixes. Phase A changes it only where
  `Robot<H>` needs it.

## R22. Constitution conflicts (recorded, not fixed)

1. **Principle 0, agent trailers**: the constitution requires an agent trailer on every agent
   commit; AGENTS.md rule (b) and the human's final decision forbid any AI or co-author trailer.
   This plan makes no commit. Agent commits follow the human's decision until one side is
   amended by a human.
2. **Principle I, delivery order**: circular for step 1 (D1). Planned with D1 A (R21).
3. **Principle IV, "any `unsafe`"** names kmr_core. kmr_api's slot (R15) may need `unsafe` if no
   crate fits; it is then flagged for human review like an invariant.

## R23. SRS problems found (new here; the coordination page holds the rest)

1. **SRS section 8, delivery order**: FR-8 (payloads) is in no step. The plan puts payload
   registration with step 2 (it needs the model's links).
2. **SRS FR-6.3**: "without any bus plugin, run() simulates". Silent on plugins declared while
   the adapter is not compiled in. The plan returns `FeatureOff` (R3); the SRS should say a
   declared port never simulates.
3. **SRS FR-6.1**: the aliases "carry N". The `Robot` alias must also carry the hardware (the
   adapter type, FR-5.1), or `run()` cannot pick the adapter without a feature switch.
4. **SRS FR-8.3 vs FR-8.4**: a center-of-mass marker on the registration path would panic every
   program that registers a payload, including the example FR-8.4 asks to fix (R11).
5. **SRS FR-13.3 / FR-13.5**: "a remote source's fallback" and "a registration's fallback" are
   two attachments. The SRS does not say which one a remote timeout triggers. The plan ties each
   source to its own guarded registration (R18).
6. **SRS section 4 Constraints**: "one atomic swap each" is an upper bound. The reader needs only
   a load on a tick with nothing new (R15). Wording only.

## R24. Spec problems (not SRS; for the spec owner)

1. FR-036 says run-time `unimplemented!` bodies; under D5 A they become compile-time markers
   (R10).
2. FR-065 says the center-of-mass marker is "reached only at finalization"; it must not be
   reached at all in this version (R11).
3. SC-001 "builds and runs" with an unconditional bus plugin line conflicts with R3; the
   example gates the line (R6).
4. FR-020's group-slice override needs a kmr_adapters const fn over an index array; the
   kmr_adapters contract lists index overrides only. Resolved 2026-10-28: kmr_adapters T021 and
   its hardware-table contract list the group forms.
5. FR-026's "not driven yet" message never ships on its own: it goes in the same change that
   wires the adapter (R1). Until then only the duplicate no-plugin `eprintln!` is dropped.

## R25. Coordination log

- 2026-10-28 coordination: A4 now waits on controllers plan C0 (`main.rs` on
  `robot!(XmBench)`, unblocked), R5.
- 2026-10-28 coordination: D2 assumed B in every plan (R17); C4 adds `set_limit_policy_all`.
- 2026-10-28 coordination: kinds as `[u64; MASK_WORDS]`, checked with
  `const _: Table<..> = __TABLE.check_kinds(model::PRISMATIC);` (R9; contracts/robot-hardware.md).
- 2026-10-28 coordination: limits are typed `[Limit; N]` with `lower`/`upper` `Option<Q>`; B3
  maps `CONTINUOUS` bits to `None` (R9).
- 2026-10-28 coordination: age is `Option<u32>` per actuator (C1, contracts/handles.md,
  data-model.md); the envelope flag is per actuator (contracts/handles.md).
- 2026-10-28 coordination: features `ffi-python`, `ffi-lua` (G2, R19); bench hooks C7
  (`presort-off` passthrough, preset costs); `Robot<N, H, C, E>` threads the core's `E` (C6, R2).
- 2026-10-28 coordination: C6 maps `RunError::{Config, Hal}`, not `RunError::Api` (R13).
- 2026-10-28 coordination: A0 waits on core K1.6 (the core's simulation warning at `open`).
- 2026-10-28 coordination: R24 #4 resolved: kmr_adapters T021 now lists the group-slice
  overrides.
- 2026-10-28 coordination: quickstart Phase E, envelope row: under D3 A (clamp, assumed, pending
  human) the adapter sees the clamped value, never the written one; "never sees the value" was
  wrong. SC-013 and FR-052 (SRS FR-13.5) hold: no out-of-envelope value reaches the adapter.
- 2026-10-29 coordination: D17 decided: `Spec` carries `type Drivers`, a list of `Hal` members,
  one per brand, not one adapter type; `robot!` builds it from the hardware table (R2,
  data-model, contracts/robot-hardware.md, new T069; `specs/002-kmr-core/design-hal-set.md`).
