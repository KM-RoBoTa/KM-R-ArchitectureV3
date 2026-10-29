# Design note: N as a const generic in kmr_core

Status: proposal for the maintainer. Nothing in `kmr_core/` is edited. Written 2026-10-28.

Scope: FR-011 (SRS FR-6.1, section 4 Assumptions), FR-012 (SRS Definitions "Write mask"),
FR-013 (SRS section 8, decision 0001), FR-058 (SRS NFR-7), FR-061 (SRS NFR-11).

## 1. Every site that uses `JOINTS` or `HISTORY_DEPTH`

`HISTORY_DEPTH` stays a capacity constant (FR-013). Only its uses through `RobotState<DEPTH>`
move. `JOINTS` disappears from the core (FR-011: "MUST NOT carry an N constant").

| File | Today | Becomes |
|---|---|---|
| `state.rs` | `pub const JOINTS = 4`, `pub const HISTORY_DEPTH = 4` | `JOINTS` removed; `HISTORY_DEPTH` kept |
| `lib.rs` | `pub(crate) use state::{HISTORY_DEPTH, JOINTS, State}`, two comments | drop `JOINTS` |
| `state/history.rs` | `impl State<JOINTS>` (zeroed, is_zeroed, home_state), `impl Default for State<JOINTS>` | `impl<const N: usize> State<N>`, generic `Default`. `History<N, DEPTH>` is already generic |
| `state/desired.rs` | `Desired` with `[Q; JOINTS]` x3, `Default` | `Desired<const N: usize>`, `Default` by hand (already by hand) |
| `state/field.rs` (invariant) | `Slot::slot(&mut Desired) -> &mut [Self; JOINTS]`, `Slot::field(&State<JOINTS>)`, 3 impls | methods generic over N, trait not. See 2.4 |
| `state/robot_state.rs` | `RobotState<const DEPTH = HISTORY_DEPTH>`, `History<JOINTS, DEPTH>`, `len: JOINTS`, `[T; JOINTS]` in `set_all`, `home_state`, `current`, `prev`, `record_sample`, `staged` | `RobotState<const N: usize>` (DEPTH dropped, FR-013) or `<const N, const DEPTH = HISTORY_DEPTH>`; every `JOINTS` becomes `N` |
| `state/test.rs` | 40 lines with `JOINTS`, and `RobotState::<DEPTH>` at line 451 | a test-local `const N: usize = 4`. Line 451 is a trap, see 5 |
| `hal.rs` | `WORDS = JOINTS.div_ceil(64)`, `Mask([u64; WORDS])`, `ALL`, `set`, `iter`; `[_; JOINTS]` in `Layout` (group, read_every, order, `new`), `Command`, `Sample`, `Batch.index`, `Runs.group`; `NoHardware::check`; 5 test lines | `Mask<N>`, `Written<N>`, `Faults<N>`, `Layout<N>`, `Command<'a, N>`, `Sample<'a, N>`, `Batch<'a, N>`, `Runs<'a, N>`, `Hal<const N>`, `impl<const N> Hal<N> for NoHardware`. Mask sizing in 2.3 |
| `controllers.rs` (invariant) | `Env<'a>`, `ControlFn`, `ControlFnWith<T>` name bare `RobotState` | `Env<'a, N>`, `ControlFn<N>`, `ControlFnWith<T, N>`. `ThreadFn<T>` unchanged (no state) |
| `schedule.rs`, `schedule/macros.rs` (invariant) | `RunPhase`, `Drive` take `Env<'_>` | `RunPhase<N>`, `Drive<N>`. `Insert`, `HAppend`, `Schedule`, `Scheduled` unchanged |
| `robot.rs` | `Robot<CS = Scheduled>`, `Fn(&Time, &mut RobotState, &Sensors)` bounds, `C: ControlFn`, `CS: Drive`, two doctests | `Robot<const N: usize, CS = Scheduled>`, bounds name `RobotState<N>`, `ControlFn<N>`, `Drive<N>` |
| `runtime.rs` | ~15 bare `RobotState`, 9 `Robot<CS>`, `<CS: Drive>` on every phase fn, `pre_init() -> (RobotState, Sensors)`, test `bump` | `<const N: usize, CS: Drive<N>>`, `Robot<N, CS>`, `RobotState<N>`. `Host` unchanged |
| `tests/run_signals.rs`, `tests/foreign_ctrlc.rs` | 8 fns on bare `RobotState`, `Robot::new()` | `RobotState<4>`, `Robot::<4>::new()` (or a file-local alias) |
| `clock.rs`, `signal.rs`, `sealed.rs`, `sensors.rs`, `config/`, `error.rs` | none | none. `StateError::OutOfRange { len }` stays a runtime `usize` |

Outside the core, for the same change:

- `kmr_api/src/state.rs`: `pub const N = kmr_core::state::JOINTS` removed; `State<'a, const N: usize>`.
- `kmr_api/src/group.rs`: `crate::N` becomes the `N` of `GroupView<'a, const N, const K>`.
- `kmr_api/src/robot.rs`: `Robot<const N: usize, C = Scheduled>`; `ApiInline`, `ApiInlineWith` implement `ControlFn<N>`, `ControlFnWith<T, N>`.
- `kmr_api/src/robot_macro.rs`: drops the `N == $crate::N` assert. Its `compile_fail` doc test on `three.urdf` must become a passing test.
- `controllers/src/main.rs` imports `kmr_api::{Robot, State}` directly. It must call `kmr_api::robot!`.
- `kmr_adapters`: `JOINTS` in `robot.rs`, `dynamixel/{mod,io,config,dxl_rs,tests}.rs`. `Dynamixel<B, NB>` becomes `Dynamixel<B, NB, N>`, with `impl Hal<N>`. A preset table is `[Actuator; 4]`; its length is the preset's N.

## 2. How N threads through

### 2.1 Which types carry N

Every type that holds or borrows a per-actuator array carries `const N: usize`. Nothing else does.
That rule gives the table above. The HList routing (`Insert`, `HAppend`) never sees N: only the
walk (`RunPhase`, `Drive`) does, because only the walk builds an `Env`.

```rust
pub struct RobotState<const N: usize> { .. }          // DEPTH gone, FR-013
pub struct Robot<const N: usize, CS = Scheduled> { .. }  // N first, no default
pub struct Env<'a, const N: usize> { time: &'a Time, state: &'a mut RobotState<N>, sensors: &'a Sensors }
pub trait Drive<const N: usize> { fn each_tick(&mut self, env: &mut Env<'_, N>); /* .. */ }
```

N goes first and has no default. A parameter with a default must be trailing, and FR-011 forbids
a default N. `CS = Scheduled` keeps its default behind it.

`Robot` does not store anything of size N. A const parameter does not need to be used in a field
(no `PhantomData`). `Robot` must still carry N: the closure bounds of `add_controller` need it, and
`run()` cannot infer it (prototype r9b: a robot whose controllers never touch the state is
ambiguous, E0284).

### 2.2 How user signatures stay `fn(&mut State)`

`robot!` emits the aliases in the user crate (SRS FR-6.1). The user never writes N in a type.

```rust
pub const N: usize = $n;
pub type State<'a> = $crate::State<'a, N>;
pub type Robot = $crate::Robot<N>;
const _: () = assert!(N >= 1 && N <= 64, "SRS write mask: more than 64 actuators");
```

`fn ctrl(_: &Time, robot: &mut State, _: &Sensors)` then reads `State<'_, N>`. `Robot::new()`
picks `impl<const N: usize> Robot<N>`. The builder chain infers everything else.

Side gain: a controller written once for any robot also works.
`fn hold<const M: usize>(_: &Time, s: &mut kmr_api::State<'_, M>, _: &Sensors)` is accepted by
`add_controller` with M inferred (prototype `reuse` test).

### 2.3 Mask on stable Rust

`[u64; N.div_ceil(64)]` is rejected (prototype r1). Three stable options were built.

- **A. Fixed word count plus a check.** `Mask<const N: usize>([u64; MASK_WORDS])`, with
  `MASK_WORDS = 1`. Exactly FR-012: one word, N in 1..=64. Widening later is one constant.
  `derive(Default)` keeps working.
- **B. A second parameter `W`.** `Mask<const N, const W>([u64; W])`, the macro computes
  `W = N.div_ceil(64)`, a `const` check asserts it. Compiles, but W then rides on every type that
  holds a mask: `RobotState<N, W>`, `Env<'a, N, W>`, `Drive<N, W>`, `Hal<N, W>`. It also loses
  `derive(Default)` (`[u64; W]: Default` is not implemented for a generic W, E0277).
- **C. One byte per actuator.** `Mask<N>([bool; N])`. No limit and no check, but 64 bytes for 64
  actuators instead of 8, and the SRS cost line ("one machine word") no longer holds.

Recommended: A, as FR-012 already decides. The check sits in two places:

1. In `robot!`, as above: a plain `const _` assert. It fires under `cargo check` and in the IDE,
   at the user's `robot!` line.
2. In the core, as a backstop: `const CHECK: () = assert!(..)` on `Mask<N>`, named once in
   `Robot::new`. This is a post-monomorphization error: it fires under `cargo build` only, never
   under `cargo check` (verified). It protects a caller that skips `robot!`.

### 2.4 The sealed fields

The trait loses nothing and gains no parameter. Each method becomes generic over N:

```rust
pub(crate) trait Slot: Sized + Copy + From<f32> {
    fn slot<const N: usize>(desired: &mut Desired<N>) -> &mut [Self; N];
    fn field<const N: usize>(state: &State<N>) -> &[Self; N];
}
pub trait StateField: sealed::Slot {}
```

So `StateField` (and `kmr_api::Field`) stays N-free. User bounds stay `T: Field`, the seal is
unchanged, and adding `Kp`/`Kd` is still one `Slot` impl plus one `StateField` impl. The
alternative, `Slot<const N>`, also compiles but leaks: every bound becomes `T: Field<N>`.

The per-field write mask (FR-021) routes the same way: a third method
`fn mask<const N: usize>(written: &mut Written<N>) -> &mut Mask<N>` was built and tested.

## 3. Prototype

Path: `/tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/constn/`.
Three crates: `mcore` (`#![no_std]`, frunk_core HList, a copy of the `schedules!` macro, the
sealed `Slot`/`StateField`, `RobotState<N, DEPTH = HISTORY_DEPTH>`, `Mask<N>`, `Hal<N>`,
`NoHardware`, a scripted-host loop), `mapi` (wrappers, `Robot<N, C>`, `State<'a, N>`,
`GroupView<'a, N, K>`, `robot!`), `muser` (a bin and tests using only `robot!` aliases).

Compiled and passed (rustc 1.97.1):

- `cargo build --workspace`, no warnings; `cargo test --workspace`: 8 tests.
- One test binary with four robots, N = 3, 5, 10, 64, each from its own `robot!`.
- Two state shapes in one core test (`RobotState<4>`, `RobotState<7, 2>`); bare depth defaults.
- `fn(&Time, &mut State, &Sensors)`, closures, `add_controller_with`, threaded, groups.
- `mcore` built for `thumbv7em-none-eabihf` with `#![no_std]`, no `alloc`.
- Transition bridge: `RobotState<const N: usize = JOINTS>`, `Robot<const N = JOINTS, CS>`,
  `trait Hal<const N: usize = JOINTS>`: bare `RobotState`, bare `Hal` and `Mask` keep compiling.

Rejected, with the compiler's words:

| Case | Error |
|---|---|
| r1 `[u64; N.div_ceil(64)]` | `generic parameters may not be used in const operations` ("const parameters may only be used as standalone arguments here") |
| r2 `[f32; M::N]` with `M: Model` | same error, "type parameters may not be used in const expressions" |
| r10 `derive(Default)` on `[u8; N]` | E0277 `the trait Default is not implemented for [u8; N]` |
| r3 `robot!(65)` | E0080 `evaluation panicked: kmr: this robot has more actuators..` at the `robot!` line |
| r4 `Robot::<65>::new()` without the macro | E0080 `evaluation of mcore::hal::Mask::<65>::CHECK failed`, only under `cargo build` |
| r8 option B with a wrong W | E0080 `W != N.div_ceil(64)` |
| r5 bare `kmr_api::State` without `robot!` | E0107 `missing generics for struct State`, suggests `State<N>` |
| r6 `Robot::new()` without N | E0284 `type annotations needed for Robot<_>` |
| r7 a controller on `State<'_, 4>` in a robot of 10 | E0631 `expected ... mapi::State<'d, 10>, found ... mapi::State<'_, 4>` |
| r9b N inferred at `run()` only | E0284 `cannot infer the value of the const parameter N` |
| bridge: `Robot<CS>` after N goes first | E0747 `type provided when a constant was expected` |
| bridge: `impl<const N> Robot<N> { fn new }` with a default N | E0284: defaults never drive inference |

## 4. Cost

**Runtime: zero.** Same layout, same arrays, N known at monomorphization. Checked on x86_64,
`-C opt-level=3`: `set_at` and `set_all` on a `const J = 4` struct and on the generic `<4>` struct
compiled to the same body, and LLVM folded them into one symbol (`generic_set_at = fixed_set_at`).
Bounds checks fold the same way.

**Monomorphization.** One N per binary, so one instance per generic item, as today. The schedule
HList already makes every user binary unique. Core unit tests that use two N pay for two copies.

**Compile time.** Code that is not generic today becomes generic: `hal.rs` (Mask, `Layout::new`
with its sort, `Batch`, `Runs`) and, larger, the whole Dynamixel adapter once it is `<N>`. Its
codegen moves from the `kmr_core`/`kmr_adapters` rlibs into the user crate build. `RobotState`
methods and the runtime are generic already (`DEPTH`, `T`, `CS`), so they do not move. Expect a
small rise on a clean user build; track it under FR-061 (SRS NFR-11) with `cargo build --timings`
on `controllers/` before and after.

**no_std (FR-058, SRS NFR-7).** No impact. Const generics, inline const asserts, `div_ceil` and
`is_multiple_of` are all in `core`. The prototype engine built `#![no_std]` without `alloc`. The
real core's blockers are elsewhere (`frunk` with default features pulls `alloc`; `std::time`,
`tracing`, `ctrlc` in `runtime.rs`).

## 5. Steps, smallest first, each leaving the workspace green

The bridge is a temporary default `N = JOINTS` on the types that gain N. It keeps every bare use
compiling until the last step removes it.

0. Docs: a decision record `docs/decisions/0003-actuator-count-const-generic.md` that supersedes
   the joint-count half of 0001 (SRS NFR-6). No code.
1. `state/history.rs`: `impl<const N: usize> State<N>` and its `Default`. Callers unchanged.
2. `state/desired.rs` + `state/field.rs` (invariant, name it in the authorization): `Desired<N>`,
   generic `Slot` methods. `RobotState` holds `Desired<JOINTS>`.
3. FR-013: drop `DEPTH` from `RobotState`; rewrite the `RobotState::<DEPTH>` test (test.rs:451) on
   `History` directly. Do this before step 4: once N goes first, `RobotState::<X>` silently means
   N = X, the same bug class 0001 recorded.
4. `RobotState<const N: usize = JOINTS>`; methods on N, `OutOfRange { len: N }`.
5. `hal.rs`: every type `<const N: usize = JOINTS>`, `trait Hal<const N: usize = JOINTS>`,
   `Mask<N>([u64; MASK_WORDS])`. Adapters compile unchanged.
6. `controllers.rs`, `schedule.rs`, `schedule/macros.rs` (invariant), `robot.rs`, `runtime.rs` in
   one commit: `Env<'a, N>`, `ControlFn<N>`, `ControlFnWith<T, N>`, `RunPhase<N>`, `Drive<N>`,
   `Robot<const N: usize = JOINTS, CS = Scheduled>`. Keep `impl Robot<JOINTS, Scheduled> { new }`
   concrete. One commit because every `Robot<CS>` breaks with E0747. `kmr_api` names
   `ControlFn<JOINTS>` for now.
7. Core: `impl<const N: usize> Robot<N> { new }` with the backstop check. Core tests, doctests
   and `tests/*.rs` name N (`Robot::<4>::new()`, a test-local const).
8. `kmr_api`: `State<'a, N>`, `GroupView<'a, N, K>`, `Robot<N, C>`; `robot!` emits
   `$crate::State<'a, N>`, `$crate::Robot<N>` and the FR-012 assert; `kmr_api::N` removed;
   `controllers/` calls `robot!(XmBench)`; the `three.urdf` test now passes (SC-009).
9. `kmr_adapters`: `Dynamixel<B, NB, N>`, `impl Hal<N>`, tables sized by the preset.
10. Remove every `= JOINTS` default and `JOINTS` itself (FR-011). Add the SC-009 builds
    (N = 1, 4, 10, 64) and a compile-fail for 65.

## 6. Open choices for the maintainer

1. Mask storage: A with `[u64; MASK_WORDS]` (recommended, widening is one constant), A with a bare
   `u64`, B, or C.
2. Drop `DEPTH` now (step 3, FR-013) or keep `RobotState<N, DEPTH = HISTORY_DEPTH>`. Both compile.
3. Bridge (steps 1 to 10, small commits) or one large commit.
4. Parameter order `ControlFnWith<T, N>` or `<N, T>`: cosmetic, both compile.
5. Whether the `Robot` alias also takes `<C = Scheduled>` so users can name a built robot type.

## 7. SRS points

- SRS section 8 still says decision 0001 is accepted, and 0001 says "no type is generic over
  either" size. Spec issue 1 resolves it for N; the decision file itself is not amended yet (step 0).
- SRS FR-6.1 lists `N` among "type aliases". It is a `const`, not an alias. Wording only.
- SRS Definitions "Write mask" is silent above 64 actuators (spec issue 3). FR-012 fixes the
  limit; the SRS should state it, since a user hits it as a compile error.
- SRS FR-6.1 says signatures "stay `&mut State`". True in the crate that calls `robot!`. A shared
  controller crate cannot see those aliases and must write `State<'_, N>` generically. The SRS
  does not mention such crates.
