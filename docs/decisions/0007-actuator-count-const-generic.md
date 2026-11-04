# 0007 — Actuator count: a const generic N; history depth stays a constant

Status: **proposed**, drafted by an agent under the maintainer's goal of
2026-11-03, waiting for the maintainer's sign-off at review. Supersedes the
joint-count half of [0001](0001-joint-count-and-history-depth.md); the
history-depth half of 0001 stands. Built by core T021 to T030 (plan K2.x);
the design, prototype and the compiler errors behind every claim here are in
[`specs/002-kmr-core/design-const-generic-n.md`](../../specs/002-kmr-core/design-const-generic-n.md).

## The question

0001 proposed that both sizes of the robot state, the actuator count and the
history depth, be crate-level constants. The SRS has since moved the actuator
count: FR-6.1 has it come from one robot descriptor at build time, named in
the user's `main.rs` with `kmr_api::robot!`, and reach the engine "as a const
generic through the type aliases that macro emits" (SRS section 4). A
constant in `kmr_core` cannot be set from the user's crate: the core is built
before it. So the actuator count must be a parameter. What remains to decide
is how far it reaches and what stays fixed.

## Current state

| Item | Today |
|---|---|
| `kmr_core::state::JOINTS` | `pub const`, 4, or 10 under the `joints-10` feature |
| `HISTORY_DEPTH` | `pub const`, 4 |
| `State<const N>` | generic, every `impl` written for `State<JOINTS>` |
| `RobotState<const DEPTH = HISTORY_DEPTH>` | N fixed at `JOINTS`, depth generic |
| `Mask` | `[u64; JOINTS.div_ceil(64)]` |
| `kmr_api::N` | `= kmr_core::state::JOINTS`; `robot!` asserts its N equals it |

A descriptor whose N differs from `JOINTS` does not build. A second robot
size needs the `joints-10` feature, a dev-bench stopgap.

## Decision

1. **N is a const generic parameter, `const N: usize`, on every type that
   holds or borrows a per-actuator array, and on nothing else.** That is
   `RobotState<N>`, `Desired<N>`, `State<N>`, `Env<'a, N>`, `ControlFn<N>`,
   `ControlFnWith<T, N>`, `RunPhase<N>`, `Drive<N>`, `Robot<N, CS>`, the
   hardware contract (`Hal<N>`, `Mask<N>`, `Command`, `Sample`, `Batch`,
   `Layout`, `Faults`) and the driver set. The HList routing (`Insert`,
   `HAppend`, `Schedule`, `Scheduled`) never sees N.
2. **N comes first and has no default.** `Robot<const N: usize, CS =
   Scheduled>`. A default N would let a bare `Robot` build for a robot nobody
   described.
3. **`JOINTS` disappears from the core.** During the migration a temporary
   `= JOINTS` default keeps every bare use compiling; the last step removes
   it, the constant and the `joints-10` feature.
4. **The mask is one fixed word count: `Mask<N>([u64; MASK_WORDS])`,
   `MASK_WORDS = 1`, N in `1..=64`.** `[u64; N.div_ceil(64)]` does not compile
   on stable Rust. The limit is checked twice: a `const` assert in `robot!`
   (fires under `cargo check`, at the user's line) and a post-monomorphization
   `Mask::<N>::CHECK` named in `Robot::new` (fires under `cargo build`, for a
   caller that skips `robot!`). Widening later is one constant.
5. **The sealed field set gains no parameter.** `Slot`'s methods become
   generic over N, the trait does not; `StateField` (`kmr_api::Field`) stays
   N-free, so user bounds stay `T: Field`.
6. **The history depth stays the constant `HISTORY_DEPTH`**, the ring
   buffer's capacity, as 0001 and the SRS (section 8) accept. `RobotState`
   loses its `DEPTH` parameter before it gains N, so `RobotState::<X>` can
   never silently mean N = X.
7. **Users never write N.** `robot!` emits `pub const N`, `type State<'a> =
   kmr_api::State<'a, N>` and `type Robot = kmr_api::Robot<N, ..>`; a
   controller is still `fn(&Time, &mut State, &Sensors)`.

## Defence

### Real-time performance

Zero cost. A const generic is resolved at monomorphization: same layout, same
arrays. On x86_64 at `opt-level=3`, `set_at` and `set_all` on a fixed `J = 4`
struct and on the generic `<4>` compiled to the same body and were folded
into one symbol. One N per binary means one instance per generic item, as
today.

### Quality

- 0001's argument, "a parameter should express a choice that exists", now
  points the other way: the choice exists, it is the descriptor's, made in a
  crate the core cannot see.
- The half-generic state 0001 named as the worst one goes away: every type
  that carries N has its methods for every N.
- Wrong sizes are compile errors with the compiler's own words: a controller
  for 4 actuators in a robot of 10 is E0631, a robot of 65 is E0080 at the
  `robot!` line.
- The seal is untouched: adding a field is still one `Slot` impl plus one
  `StateField` impl.

### Developer experience

Signatures stay `&mut State` in the crate that calls `robot!`. A controller
written once can also be generic, `fn hold<const M: usize>(.., s: &mut
kmr_api::State<'_, M>, ..)`, and is accepted for any robot. The cost is a
crate that shares controllers without calling `robot!`: it must write
`State<'_, N>` generically (SRS FR-6.1 does not mention such crates).

Compile time: code that was concrete (`hal.rs`, the Dynamixel adapter) becomes
generic and is compiled in the user's crate. Measured by core T019 with
`cargo build --timings -p controllers` before and after (SRS NFR-11).

### Alternatives rejected

- **Keep `JOINTS`, set by a Cargo feature per robot.** One feature per
  robot shape, chosen far from the descriptor; it is the `joints-10`
  stopgap, and it contradicts FR-6.1.
- **A second parameter `W` for the mask words.** Compiles, but `W` rides on
  every type that holds a mask and loses `derive(Default)`.
- **`[bool; N]` as the mask.** No limit, but 64 bytes instead of 8 for 64
  actuators, and not the one machine word the SRS write mask costs.
- **Generic depth too.** No use needs two depths in one binary; the depth
  can be chosen at run time inside the fixed capacity (0001).

## Consequences

- `kmr_api` drops `kmr_api::N` and the `N == JOINTS` assert (kmr_api B7);
  `State<'a, N>`, `GroupView<'a, N, K>`, `Robot<N, C>`.
- `kmr_adapters`: `Dynamixel<B, NB, N>`, `impl Hal<N>`; a preset table's
  length is its N.
- `kmr_model` emits N and the kind masks as `[u64; MASK_WORDS]`.
- Tests build robots with N = 1, 4, 10 and 64 in one binary, and a
  compile-fail for 65.
- SRS points left to the maintainer: FR-6.1 calls `N` a type alias (it is a
  `const`); the write-mask definition should state the 64-actuator limit.
