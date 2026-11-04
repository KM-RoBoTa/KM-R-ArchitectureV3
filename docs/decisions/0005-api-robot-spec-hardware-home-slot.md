# 0005 — kmr_api: the robot as a type parameter, its hardware argument, the home marker, the Zenoh slot

Status: **proposed**, drafted by an agent, waiting for the maintainer's
decision. Two points rest on pending decisions (D5, D7) and one on an
unwritten human task (T029), see [Pending](#pending).

The research behind each point is in
[`specs/004-kmr-api/research.md`](../../specs/004-kmr-api/research.md)
(R2, R7, R10, R14, R15). The driver set is
[`specs/002-kmr-core/design-hal-set.md`](../../specs/002-kmr-core/design-hal-set.md).

## The question

`kmr_api` is the one crate a controller sees. Four shapes have to be fixed:

- how the builder knows which hardware the robot named by `robot!` runs on,
  without a second switch and without `dyn`;
- where a robot without a preset, or a preset with changes, declares its
  hardware;
- what the home-state calls do while SRS FR-2.7 is open;
- how a value crosses between the tick thread and the Zenoh gateway thread.

## Current state

| Piece | State |
|---|---|
| `Robot<H, C = Scheduled>`, hidden `Spec` with `type Drivers` and `bind` | written (T008, T069) |
| `robot!` emits `__Spec` and `type Robot = kmr_api::Robot<__Spec>` | written (T009, T021) |
| `robot!(Preset, hardware = \|hw\| ..)`, `robot!(model = .., hardware = TABLE)` | written (T023, T024) |
| `robot!(model = .., hardware = { brand: part })`, several brands | written (T069); syntax pending human |
| `bind()` and `Bound`: the drivers before and after the run | written (T039) |
| `initial_state()`, `go_home()` as compile errors naming SRS FR-2.7 | written (T020) |
| `Latest<T>`, the two Zenoh slots | not written: HUMAN (T029, `unsafe` if no crate fits) |
| `Robot<N, H, C>` with a const-generic N | not written: waits on core K2.1 to K2.7 (T032) |

Below, a point the table marks as not written is the planned design, not
the branch's behaviour.

## Decision proposed

1. **The robot is the builder's type parameter.** `kmr_api::Robot<H, C>`,
   where `H: Spec` is the private `__Spec` each `robot!` expansion emits.
   `Spec` gives the robot's drivers, `type Drivers`, the list of its `Hal`
   members (one per brand), and `fn bind(&[Binding]) -> Result<Drivers,
   UserError>`, which matches the bus plugins to the members, no I/O. Users
   write the `Robot` alias and never see `H` (R2).
2. **Features are read inside kmr_api.** The expansion names
   `$crate::__DxlAdapter<__NB>` and the brand's `bind`; both are picked by
   `cfg(feature = "dynamixel")` in kmr_api's own source, never in the
   expansion, where a `cfg` would test the user's features. Without the
   feature, a plugin is `UserError::FeatureOff` (R2, R3).
3. **Hardware is a `robot!` argument (D7 A).** `robot!(XmBench, hardware =
   |hw| ..)` changes a preset's table through `const fn` overrides; the `|hw|`
   form is rewritten into a `const`, not a closure. `robot!(model = ..,
   hardware = TABLE)` gives one table, one row per actuator in model order.
   `hardware = { dynamixel: TABLE.at([..]), .. }` gives one part per brand. A
   row count other than `N`, an override past the table, a brand named twice
   or parts that do not cover `0..N` exactly once fail the build (R7).
4. **The drivers are reachable outside the loop.** `Robot::bind()` gives a
   `Bound` whose `drivers()` reaches the list before the run; `Bound::run()`
   lends each member through the core's `impl Hal for &mut A` and hands the
   list back after it. Never inside a tick (SRS FR-5.9; R14).
5. **The home state is a compile-time marker (D5 A).** `initial_state()` and
   `go_home(ramp)` keep their names and arity, but carry a bound no type
   meets, `HomeDecided`, whose `on_unimplemented` message names SRS FR-2.7.
   The bound sits on a type parameter: the field of `initial_state::<T>()`,
   and the ramp of `go_home(ramp: R)`, which was a placeholder `u32`. A bound
   on a concrete type fails at the definition, not at the call. The library
   builds; every call site fails to compile (R10).
6. **The Zenoh slots are a triple buffer per direction.** `Latest<T: Copy>`:
   one writer, one reader; the writer does one atomic swap per put, the
   reader one load when nothing is new and one swap when something is. An
   overwritten value nobody took is counted. The `triple_buffer` crate if its
   writer can tell an untaken value, else a hand-written slot, its `unsafe`
   reviewed by the human (R15; T029, T030).

## Defence

### Real-time performance

- No `dyn`, no fat pointer: the drivers are an HList, monomorphized like the
  controllers. Binding runs once, before the loop.
- The hardware table is a `const`: overrides and checks run in const
  evaluation, nothing at run time.
- The planned slot never locks, never allocates on the tick thread and never
  blocks either side (prototype: 0 allocations on the reader over 2 000 000
  takes, values never torn; R15).
- The home marker costs nothing: a call cannot be built, so nothing runs.

### Quality

- **The adapter follows the robot.** The same `robot!` line picks the model
  and the hardware, so no feature, file or variable can pair a robot with
  another robot's buses (FR-005).
- **Errors are found before the motors move.** A bad table or override is a
  compile error; a bad plugin is a `run()` error before any I/O; a member's
  bring-up error names its position and type and keeps its typed data.
- **No marker after `open`.** A run-time `unimplemented!` in `Init` would fire
  with the torque on (SRS NFR-2); the compile error never reaches the robot.

### Developer experience

- One line names the robot; one builder chain adds the controllers and the
  plugins. Controller signatures stay `&Time, &mut State, &Sensors`.
- The hardware sits next to the robot it drives, in `main.rs`, with no
  per-machine config file (SRS FR-6.1).
- The home-state error is one line naming the SRS id, at the user's call.

### Alternatives rejected

- **A Cargo feature that picks the robot.** A second switch beside `robot!`
  (FR-005).
- **`dyn Hal` in the builder.** Constitution II, IV: a fat pointer and a
  virtual call per tick.
- **A run-time registry of presets.** Needs a lookup by name and a heap.
- **A real closure for `|hw|`.** `const` closures are unstable.
- **Builder calls for the hardware (D7 C).** The count check becomes a run
  error, after the user has built and started the binary.
- **Run-time home markers (D5 B).** Fire in `Init` with the torque on; the
  example cannot run (R10).
- **A `Mutex` or a channel in the tick, or `AtomicPtr` with a `Box` per
  message.** A lock or an allocation on the tick thread (SRS NFR-1; R15).

## Pending

- **D7** (`hardware = ..` as a `robot!` argument) is assumed, option A.
  Option B (a `hardware!` macro) or C (builder calls) change the `hardware`
  arms of `robot!` only. The several-brand syntax, `hardware = { brand: part
  }`, and the `kmr_api::dynamixel` path are not confirmed (T069).
- **D5** (compile-time home marker) is assumed, option A. Option B restores
  run-time bodies in `home.rs` and the example's calls.
- **The slot** is HUMAN (T029): the crate is to be checked first; a
  hand-written slot carries `unsafe`, which an agent does not write.
