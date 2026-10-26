# 0001 — Joint count and history depth: constants, not generic parameters

Status: **proposed**, waiting for the maintainer's decision. The code does not
follow it yet; see [Current state](#current-state).

## The question

Two sizes shape the robot state: the number of joints and the number of past
ticks kept in the history. Each can be expressed in two ways:

- a **crate-level constant** (`JOINTS`, `HISTORY_DEPTH`), one value for the
  whole build;
- a **const generic parameter** (`State<const N: usize>`), chosen per type
  instance.

The code uses both at once, which is the ambiguity this document removes.

## Current state

| Type | Joint count | History depth |
|---|---|---|
| `State<const N>` | generic, but every `impl` is written for `State<JOINTS>` only | — |
| `History<const N, const DEPTH>` | generic | generic |
| `Desired` | constant `JOINTS` | — |
| `sealed::Slot` | constant `JOINTS` in both method signatures | — |
| `RobotState<const DEPTH = HISTORY_DEPTH>` | constant `JOINTS` | generic with a default |
| `kmr_api::State`, `GroupView` | constant `N = JOINTS` | not exposed |

So the joint count looks generic in two types and is fixed everywhere else,
and a `State<7>` can be named but has no methods. The depth is generic, but
nothing ever instantiates it with a value other than the default.

This mix already produced one bug: every use site wrote `RobotState<JOINTS>`,
passing the joint count as the depth. It went unnoticed because both
constants are 4. The default parameter added in this pull request closes that
hole but keeps the generic, which is the undecided part.

## Decision proposed

**Both sizes are crate-level constants. No type is generic over either.**

- `JOINTS` and `HISTORY_DEPTH` stay defined once, in `kmr_core/src/state.rs`.
- `State`, `History` and `RobotState` lose their const parameters.
- `HISTORY_DEPTH` is the **capacity** of the ring buffer, a maximum. If users
  later choose a depth (`Robot::history_depth(n)`, today a commented-out stub
  in `kmr_api`), it is a runtime value checked once against the capacity when
  the robot is built. It is not a type parameter.
- Where the constants get their value (a `build.rs` reading the robot model, a
  Cargo feature per model) is **not** decided here. It is the "build-time
  value from the model" item of the roadmap. This decision only fixes that the
  result is a constant.

## Defence

### Real-time performance

Neither option is faster. A const generic is resolved at compile time, so both
produce the same fixed-size arrays and the same machine code. This was not
measured; it follows from how monomorphization works.

For reference, with the current values one sample is three arrays of four
`f32`, 48 bytes, and the history holds four samples, 192 bytes plus the ring
buffer's bookkeeping. Nothing allocates in either option.

Real-time performance therefore does not decide this. Quality and developer
experience do.

One real-time point favours the runtime depth inside a fixed capacity: the
buffer never resizes, so choosing a depth cannot allocate.

### Quality

- **A parameter should express a choice that exists.** A binary drives one
  robot, described by one model. There is one joint count per build. A generic
  parameter offers a choice nobody can make, and an unused choice is where the
  `RobotState<JOINTS>` bug came from.
- **Half generic is the worst state.** Today a type accepts any `N` while its
  methods exist for one value. A reader cannot tell which of the two is the
  design.
- **The sealed `Slot` trait fixes the joint count already.** Its methods
  return `[Self; JOINTS]`. Making the joint count truly generic means changing
  that trait, which is the part of the state that protects the field set.
- **Cost of the constant:** tests cannot build a state with another joint
  count in the same binary. Checking another size means changing the constant
  and rebuilding, which is how the depth fix in this pull request was
  verified.

### Developer experience

- With constants a controller is written
  `fn ctrl(_: &Time, robot: &mut State, _: &Sensors)` and works on `[Q; N]`.
- With generics the parameters reach every public signature: `State<J, D>`,
  `GroupView`, the three controller traits, `Robot`. Users either write the
  numbers or write generic functions, and compiler errors start naming const
  parameters. That contradicts the goal that a user needs only basic Rust.

### Alternative rejected: generic over both, everywhere

`RobotState<const J, const D>` threaded through `Slot`, `Env`, `ControlFn`,
`Robot`, the runtime and `kmr_api`.

What it would give: several robot shapes in one binary, tests over several
sizes, and a history depth chosen in the type.

Why it loses: no current or planned use needs several shapes in one binary;
the depth can be chosen at runtime inside a fixed capacity; and the price is
paid in every signature a user reads.

### Alternative rejected: keep the mix

Keep `RobotState<const DEPTH = HISTORY_DEPTH>` and the generic `State<N>`.

Why it loses: it is the ambiguity itself. It is acceptable only as a
transition, which is what this pull request leaves in place.

## Plan

Not done in this pull request. The files are in `state/`, which is
hand-written territory, and the decision is the maintainer's.

1. Remove `const N` from `State` and `History`, and `const DEPTH` from
   `History` and `RobotState`. The arrays use `JOINTS`, the buffer uses
   `HISTORY_DEPTH`.
2. Replace `impl State<JOINTS>` by `impl State`, and `impl<const DEPTH>
   RobotState<DEPTH>` by `impl RobotState`.
3. `kmr_api` keeps `pub const N`. No public signature changes.
4. Separately, when the builder gains `history_depth(n)`: store `n`, reject
   `n > HISTORY_DEPTH` when the robot is built, and bound `prev` by `n`.

If the maintainer prefers the generic route, step 1 is replaced by making
`Slot`, `Desired` and `RobotState` generic over the joint count, and this
document is rewritten to defend that.
