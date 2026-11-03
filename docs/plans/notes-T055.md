# kmr_core T055: review against a later heap-gated dynamic mode

Review only (plan K6.4; spec FR-008, SC-013 (SRS FR-1.6, FR-1.7)), done
2026-11-03 on `kmr_core/src/robot.rs`, `controllers.rs`, `schedule.rs`,
`schedule/macros.rs` as they are on this branch, with decisions
[0007](../decisions/0007-actuator-count-const-generic.md) (N) and
[0008](../decisions/0008-empty-robot-type-state-and-run-error.md) (type-state
`E`) taken as landed. For the PR description. No code change, so no decision
record: nothing found must change.

**Verdict: no builder method, phase rule or controller signature that a
dynamic mode would have to change.** One open SRS point, the label form,
decides how `.before(C)` is added, not whether it is additive.

## Builder

- `add_control_fn`, `add_control_fn_with`, `add_control_fn_as_thread`
  route through `Insert<S, Node>` and return `Robot<CS::Output>`. A dynamic
  bucket type implements `Insert<S, Node>` with `Output = Self` (push into a
  heap list): same methods, same chain, no new parameter on the user side.
  The static `Scheduled` impls are untouched.
- `set_dt`, `set_hold`, `run`, `run_on`: no tie to how controllers are
  stored. `run_on` bounds `CS: Drive`; a dynamic bucket implements `Drive`.
- Type-state `E` (0008): every add yields `Filled` in both modes. A dynamic
  mode that registers in a loop needs `Output = Self`, which it has; the
  first add still fixes `E`.
- One limit to state, not to fix: a heap mode boxes nodes, so it needs
  `Node: 'static` on its own `Insert` impl. A controller with a borrowed
  context builds in static mode and not in dynamic mode. The static surface
  does not change; the dynamic mode's docs must say it.

## Phase rules

- Six labels, startup three once, tick three every tick (SRS FR-1.4): the
  labels are sealed ZSTs, independent of storage. A dynamic mode reuses them.
- Declaration order within a phase is the static mode's rule (FR-1.4);
  `.before`/`.after` reorder within a phase in dynamic mode only (FR-1.6).
  The phase contract itself does not promise more than that.
- FR-1.5: no phase assumes every controller runs every tick. The walk calls
  each node once per tick today; a rate option would sit in the node, not in
  the phase.

## Controller signatures

- `ControlFn`, `ControlFnWith<T>`, `ThreadFn<T>` take `&self` and borrowed
  arguments, no generic method: all three are object safe, so a dynamic mode
  can store `Box<dyn ..>` behind the heap feature without a new signature.
  With N (0007) they become `ControlFn<N>`, `ControlFnWith<T, N>`; still
  object safe.
- `Fn`, not `FnMut` (FR-1.1), holds in both modes.

## Static path not slower (FR-1.7, last clause)

The gate is a separate bucket type behind a feature. `Scheduled`, `Insert`,
`HAppend`, `RunPhase` and `Drive` for the static path compile as today; the
default build pays nothing for the gate's existence.

## Open: the controller label (SRS section 8, FR-1.7)

FR-1.7 asks that every registration yield a label usable as `C` in
`.before(C)`. Today no registration yields one. The form is open in SRS
section 8 ("zero-sized marker type per registration, or a const id").

- A label **returned by** `add_*` would break the chained builder: the
  return type is the builder. Not an option without a breaking change.
- The fn item type as the label fails FR-1.3: the same function may be
  registered several times.
- Both forms the SRS names stay additive: a user ZST given through a new,
  additive method or argument (`.label(Swing)`), or an implicit
  per-registration index the builder already knows from the bucket
  position. Neither touches today's methods.

For the maintainer: pick the label form in SRS section 8; the static builder
keeps its HList type-state either way.
