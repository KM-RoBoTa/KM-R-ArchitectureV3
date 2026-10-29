# Contract: `kmr_api::robot!`

The only door to kmr_model. Users never name `kmr_model`.

## Forms

```rust
kmr_api::robot!(XmBench);                    // preset (SRS FR-6.1)
kmr_api::robot!(model = "robot/arm.urdf");   // descriptor, relative to Cargo.toml; simulation only
// hardware argument (D7 A, assumed, pending human: D7):
kmr_api::robot!(XmBench, hardware = |hw| hw.read_only(3).baud("bus0", 3_000_000));
kmr_api::robot!(model = "robot/arm.urdf", hardware = Table { buses: [..], actuators: [..] });
```

One call per binary, at module level in `main.rs`. A second call is a duplicate-item error.
kmr_api's front door splits `hardware = ..` off before it calls the proc macro; kmr_model never
parses hardware. The hardware forms, `__Spec` and their checks are specified in
specs/004-kmr-api/contracts/robot-hardware.md.

## Emits

The items listed in [data-model.md § Emitted facts](../data-model.md#emitted-facts-in-the-user-crate-no_std-no-heap)
(kmr_api FR-002), from two sources, no item from both:

- the proc macro (kmr_model): `N`, `model::*`, `actuator!`;
- kmr_api's `macro_rules!` front door: `State`, `Robot = kmr_api::Robot<__Spec>`, `__Spec`,
  `__NB`, `__M`, `__TABLE`, and the checks below.

Controller signatures stay `fn(&mut State)` (SRS FR-6.1).

`Robot` carries the adapter type through `__Spec`: with a preset, the preset's adapter
(kmr_api FR-005), so `run()` needs no Cargo feature to pick hardware. After the core's const
generic N it reads `kmr_api::Robot<N, __Spec>`.

Wiring, as data only (kmr_api FR-002): `model::PRISMATIC` (`[u64; MASK_WORDS]`) goes to the
hardware table's kind check, `const _: Table<__NB, __M> = __TABLE.check_kinds(model::PRISMATIC);`
(kmr_adapters FR-021); a `const _: () = ..` form is E0308. The limits `model::LOWER`, `UPPER`,
`SPEED`, `EFFORT` go to the core's limit stage as typed `[Limit; N]` (kmr_core FR-037); a set
`model::CONTINUOUS` bit becomes `lower`/`upper` = `None`, never the `Q(0.0)` placeholder.

Checks in the expansion, both owned by `robot!` (kmr_api FR-007):

- Hardware count: `const _: &[Actuator; N] = &__TABLE.actuators;`. A wrong count is rustc's E0308,
  "expected an array with a size of 4, found one with a size of 3", which names both counts.
- N > 64: a const assert with a static message, so it fires under `cargo check`; the core's own
  check is a post-monomorphization backstop (kmr_core FR-012). The cap itself is open.

A message from const evaluation cannot use `format!` (E0015). A const fn that builds the message
byte by byte can still name a bad index (proved on rustc 1.97.1); the proc macro is the other way.

## Errors (all compile errors, SRS FR-6.4)

Every message starts with `kmr:`, names the file, the element and the reason, and carries the
SRS marker when one applies. The span is the path literal or the preset ident.

| Case | Message shape |
| --- | --- |
| file missing | `kmr: cannot read robot descriptor <path>: <io error>` |
| bad syntax | `kmr: <path>:<line>:<col>: <parser message>` |
| bad extension | `kmr: <path>: expected .urdf, .ron or .json` |
| unknown preset | `kmr: no preset named <X>; presets: <list>` |
| no limit | `kmr: <path>:<line>: joint <name> has no <limit>` |
| bad limit | `kmr: <path>:<line>: joint <name>: lower > upper` (and the other range checks) |
| non-finite / not f32 | `kmr: <path>:<line>: joint <name>: <field> does not fit f32` |
| floating / planar | `kmr: <path>:<line>: joint <name>: roadmap: non-1-DOF joints` |
| mimic, multi transmission | `kmr: <path>:<line>: joint <name>: SRS index space: multi-actuator joint` |
| several roots | `kmr: <path>: SRS FR-6.5: multi-robot descriptor` |
| duplicate name | `kmr: <path>:<line>: duplicate joint <name> (first at line <l>)` |
| zero actuators | `kmr: <path>: no actuated joint` |
| unknown field (.ron/JSON) | `kmr: <path>:<line>:<col>: unknown field <f>` |
| preset hardware count ≠ N | rustc E0308 from the array-type check: `expected an array with a size of <N>, found one with a size of <h>` (no `kmr:` prefix) |
| N ≠ core N (temporary) | `kmr: this robot's actuator count differs from kmr_core's; N as a const generic is pending in kmr_core` |

## Guarantees

- Same descriptor, same expansion, on any host (FR-007).
- Nothing emitted runs per tick (FR-018); the expansion builds under `#![no_std]` (FR-019).
- Editing the descriptor rebuilds the crate. A fresh build recompiles nothing; any other compile
  of the calling crate re-reads the descriptor (FR-021).
