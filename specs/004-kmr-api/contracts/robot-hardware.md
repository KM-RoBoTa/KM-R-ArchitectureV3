# Contract: what `robot!` adds for hardware

This file extends `specs/003-kmr-model/contracts/robot-macro.md`. The forms, emitted model
items and error messages there stand; nothing here repeats them. Here: the `hardware` argument
(D7 option A, assumed, pending human), the `Spec` impl that carries the drivers (FR-005
(SRS FR-6.1, FR-5.1)), and the checks kmr_api owns (FR-007 (SRS FR-6.1, FR-6.4)).

## Forms

```rust
kmr_api::robot!(XmBench);
kmr_api::robot!(XmBench, hardware = |hw| hw.read_only(3).read_only_group(&ARM).baud("bus0", 3_000_000));
kmr_api::robot!(model = "robot/arm.urdf", hardware = Table { buses: [..], actuators: [..] });
kmr_api::robot!(model = "robot/arm.urdf");          // no hardware: simulation only
```

- `|hw| <expr>` is rewritten to `{ let hw = <Preset>::HARDWARE; <expr> }` in a `const`. Every
  call in `<expr>` must be a `const fn` of kmr_adapters' `Table`
  (specs/001-kmr-adapters/contracts/hardware-table.md, § Overrides).
- `Table { .. }` is a kmr_adapters `dynamixel::Table<NB, N>` literal, re-exported by kmr_api so
  the user names no other crate.
- `ARM` is a user const group, `[usize; K]` (FR-020 (SRS FR-5.8)).

kmr_api's `macro_rules!` splits `hardware = ..` off before it calls kmr_model's proc macro, so
kmr_model never parses hardware. The proc macro emits only `N`, `model::*` and `actuator!`.
The front door emits everything below and the `State` alias, so no item is emitted twice.

## Emits (added to 003's list)

```rust
pub type Robot = kmr_api::Robot<__Spec>;          // was kmr_api::Robot; Robot<N, __Spec> after B7
#[doc(hidden)] pub struct __Spec;
impl kmr_api::Spec for __Spec { type Drivers = ..; fn bind(..) -> ..; /* model consts, step 2 */ }
const __NB: usize = ..;  const __M: usize = ..;
const __TABLE: kmr_api::Table<__NB, __M> = ..;    // absent without hardware
```

`type Drivers` is the list of drivers, one member per brand of the table (2026-10-29,
`specs/002-kmr-core/design-hal-set.md` §7). A Dynamixel-only table gives
`HCons<kmr_api::__DxlAdapter<__NB>, HNil>`: the Dynamixel adapter with kmr_api's `dynamixel`
feature, `NoHardware` without it (then `bind` returns `FeatureOff`). Without hardware:
`HCons<NoHardware, HNil>`, and `bind` returns `NoHardwareDeclared`.

All `__` items are hidden. Two `robot!` calls in one module are a duplicate-item error.

## Several brands (2026-10-29, pending human)

The hardware table has one entry per bus. Each entry names its brand, its firmware ids and the
actuator indices it drives. `robot!` makes one member per brand: the Dynamixel buses form one
member over `dynamixel::Table<NB, M>`, a CubeMars bus another. The indices of all entries cover
`0..N` exactly once; the core checks it at `run()` (kmr_core FR-063 (SRS FR-5.1, FR-6.4)), and
the expansion may check it in const evaluation too. A one-brand table keeps the forms above and
owns `0..N` in model order, so the count check below stays its check. Bus plugins still supply
serial ids or paths only.

Syntax, 2026-11-02 (kmr_api T069, **assumed, pending human**): the smallest form that names one
part per brand.

```rust
kmr_api::robot!(model = "robot/arm.urdf", hardware = { dynamixel: ARM.at([0, 2, 3]), cubemars: LEGS });
```

- Each `brand: part` entry is one member. The brand is a module of `kmr_api::__brand`
  (`dynamixel` today); its `Hardware<NB, M>` is the type of the part and its `shape(&part)` the bus
  and actuator counts, read in const evaluation. A Dynamixel part is a placed table,
  `Table { .. }.at([index; M])` or `.in_order()` (kmr_adapters T069, `Placement<NB, M>`).
- The expansion puts each part in `mod __hardware::<brand>` (`SHAPE`, `type Part`, `const PART`),
  so a brand named twice is `error[E0428]`: one member per brand, and one Dynamixel SDK per
  process.
- `type Drivers` is `<HCons<__hardware::a::Part, ..> as kmr_api::Brands>::Drivers`; `bind` is
  `kmr_api::__bind(HCons { head: __hardware::a::PART, .. }, plugins)`.
- Routing: over several brands a plugin goes to the brand whose bus its label names; a label two
  brands share is `SharedBusLabel { label }`, an unknown label `UnknownBus { label, port }`, no
  label on a robot of several buses `LabelRequired { port }`, all before any bind. A robot of one brand hands that brand every plugin, so its own
  binding errors (`UnknownBus`, `UnboundBus` of the adapter) stay as they were.
- A member's binding error is `BringUp` over `HardwareError::Member`, named
  `member k (<type name>): ..` like a bring-up error; `FeatureOff` passes through.
- Coverage in const evaluation: the expansion emits
  `const _: () = kmr_api::__check_coverage(&[<brand>::indices(&PART), ..], N);`, so parts that
  leave an index without a member, give it two, or name one past `N` are `error[E0080]: evaluation
  panicked: kmr: no hardware part drives engine index 2` (or `two hardware parts drive ..`, `..
  past the robot's actuators`). The preset arm checks its `in_order()` table the same way. The
  run() check stands behind it.
- The table types are re-exported as `kmr_api::dynamixel::{Actuator, Bus, Mode, Model,
  Placement, Table, Torque, Zero, DEFAULT_READ_EVERY}`: a module, since `kmr_api::Bus` is the
  plugin builder. The module path is assumed, pending human, with the syntax.
- Not done: the `hardware = Table { .. }` and `|hw| ..` forms above.

## Checks in the expansion (all compile errors, all fire under `cargo check`)

| Check | Code | Error |
| --- | --- | --- |
| hardware count = N | `const _: &[Actuator; N] = &__TABLE.actuators;` | `error[E0308]: mismatched types ... expected an array with a size of 4, found one with a size of 3` |
| override index past N, unknown label, bad baud | the table's `const fn` panics | `error[E0080]: evaluation panicked: <table message naming index or label>` |
| N ≤ 64 (D4 A) | `const _: () = assert!(N >= 1 && N <= 64, "SRS write mask: more than 64 actuators");` | `error[E0080]` with that text |
| N = core's N (temporary) | `assert!(N == JOINTS, ..)` | as in 003's contract; removed with the core's const generic |
| kinds (step 2) | `const _: kmr_api::Table<__NB, __M> = __TABLE.check_kinds(model::PRISMATIC);`, sets typed `[u64; MASK_WORDS]` | `error[E0080]` with kmr_adapters' message naming the index and SRS FR-2.2 |

Proved on a scratch copy (research R7): the first two rows, one error each. Kinds row proved
2026-10-28 (research R9): `check_kinds(self, [u64; MASK_WORDS]) -> Self` works; a
`const _: () = ..` form is E0308 (the call returns the table); `[u64; N.div_ceil(64)]` in
`Spec<N>` does not compile on stable.

## Model data handed on (step 2, B3)

Limits go to the core's `set_limits` as `[Limit; N]`, typed `Q`/`Qd`/`Tau`, never `f32`. Where
`model::CONTINUOUS` has bit i, `lower` and `upper` are `None`, not the emitted `Q(0.0)`
placeholder: a continuous joint has no position bound and is never clamped to 0.

## Guarantees

- The binary gets constants and one zero-sized type. Nothing here runs per tick.
- The user's crate never tests a kmr feature: the adapter choice is made inside kmr_api.
- Changing D7 changes the `hardware` arms and `@table` only.
