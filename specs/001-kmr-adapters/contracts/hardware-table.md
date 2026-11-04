# Contract: hardware table, presets, overrides

Public surface of `kmr_adapters` that `kmr_api::robot!` and a user's `main.rs` build on.
FR-023 to FR-028 (SRS FR-6.1, FR-6.4). Fields: [data-model.md](../data-model.md).
Shapes proved in `scratchpad/adapters-plan/table` (research R4).

## Types

```rust
pub mod dynamixel {
    pub struct Table<const NB: usize, const N: usize> {
        pub buses: [Bus; NB],
        pub actuators: [Actuator; N],
    }
    pub struct Bus { pub label: &'static str, pub baud: u32, pub read_every: Option<NonZeroU32> }
    pub struct Actuator {
        pub bus: usize, pub id: u8, pub model: Model, pub mode: Mode,
        pub torque: Torque, pub zero: Zero,
    }
    pub enum Model { Ax12a, Rx28, Mx64, Mx64V2, Xm430W210, Xw430T200, Xw540T140, Xw540T260 }
    pub enum Mode { Position, Velocity, Current }
    pub enum Torque { On, ReadOnly }
    pub enum Zero { Center, Raw(i32) }
    impl Zero { pub const DEFAULT: Self = Self::Center; }          // D8: one place
    pub const DEFAULT_READ_EVERY: NonZeroU32 = /* 4, bench */;     // D9: one place
}
```

All are `Clone + Copy + Debug`. No heap, no `dyn`.

## Checks

```rust
impl<const NB: usize, const N: usize> Table<NB, N> {
    /// Panics in const evaluation on a bad table: a compile error naming
    /// the actuator index or the bus label.
    pub const fn check(self) -> Self;
}
```

Rejected: see [data-model.md](../data-model.md) § Checks. Run-time-only data (the plugin baud)
is checked by `bind` ([port-binding.md](port-binding.md)).

## Overrides (FR-026 (SRS FR-6.1))

`const fn`, chainable, each returns `self.check()`:

```rust
impl<const NB: usize, const N: usize> Table<NB, N> {
    pub const fn baud(self, bus: &str, baud: u32) -> Self;
    pub const fn read_every(self, bus: &str, ticks: u32) -> Self;   // 0: compile error
    pub const fn id(self, actuator: usize, id: u8) -> Self;
    pub const fn mode(self, actuator: usize, mode: Mode) -> Self;
    pub const fn read_only(self, actuator: usize) -> Self;
    pub const fn zero(self, actuator: usize, raw: i32) -> Self;
    // group forms over an index array (kmr_api FR-020 (SRS FR-5.8)); no `id` form
    pub const fn read_only_group<const K: usize>(self, group: &[usize; K]) -> Self;
    pub const fn mode_group<const K: usize>(self, group: &[usize; K], mode: Mode) -> Self;
    pub const fn zero_group<const K: usize>(self, group: &[usize; K], raw: i32) -> Self;
}
```

An unknown label or an index past N, in a group too, is a compile error naming it. The
`main.rs` syntax that reaches these (by index or through a group slice) is kmr_api's
`robot!(.., hardware = |hw| ..)` (kmr_api FR-020; D7 A, assumed, pending human).

## Presets

```rust
pub mod presets {
    pub trait Preset { const N: usize; }      // until robot! takes N from kmr_model
    pub struct XmBench;
    impl XmBench {
        pub const HARDWARE: dynamixel::Table<1, 4> = /* four XM430-W210, ids 1..=4, bus "bus0",
                                                         1 Mbaud, cadence default */;
    }
    impl Preset for XmBench { const N: usize = Self::HARDWARE.actuators.len(); }
}
```

One preset per KM-RoBoTa robot, named as in `kmr_model`. Model half there, hardware half here.

## Count check hook (FR-024 (SRS FR-6.1, FR-6.4))

`robot!` (kmr_api FR-007) writes, in the user's crate:

```rust
const _: &[kmr_adapters::dynamixel::Actuator; N] = &__TABLE.actuators;
```

A table of the wrong size fails with E0308: "expected an array with a size of 3, found one with
a size of 4". Proved. The hook is the public `actuators` field; nothing else is needed here.

## Kind check (FR-021 (SRS FR-2.2), plan E6)

```rust
impl<const NB: usize, const N: usize> Table<NB, N> {
    /// Panics in const evaluation when bit i of `prismatic` is set:
    /// "actuator i: SRS FR-2.2: prismatic Dynamixel actuator has no mm conversion".
    pub const fn check_kinds(self, prismatic: [u64; MASK_WORDS]) -> Self;
}
```

`MASK_WORDS` is `kmr_core::hal`'s (1 under D4 A); bit i is word i / 64, bit i % 64. `robot!`
writes `const _: Table<__NB, __M> = __TABLE.check_kinds(model::PRISMATIC);`. Proved 2026-10-28:
that form compiles and a set bit is E0080 with the message; `const _: () = ..` is E0308 (the
call returns the table); `[u64; N.div_ceil(64)]` does not compile in kmr_api's `Spec<N>`.

## Example (main.rs, through kmr_api re-exports)

```rust
kmr_api::robot!(XmBench);   // N = 4, hardware = XmBench::HARDWARE
// with overrides (syntax kmr_api's; D7 A, assumed, pending human):
kmr_api::robot!(XmBench, hardware = |hw| hw.baud("bus0", 3_000_000).read_only(3));
```
