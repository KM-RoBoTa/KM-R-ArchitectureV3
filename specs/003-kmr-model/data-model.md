# Data model: kmr_model

Two layers. The **build-time model** lives in the proc macro, on the heap, and dies with the
build. The **emitted facts** are `const` items in the user crate; they are all the binary gets.

## Build-time model (inside `kmr_model`, heap allowed)

| Entity | Fields | Rules |
| --- | --- | --- |
| `Descriptor` | `path`, `format` (Urdf, Ron, Json), `text` | Exactly one per build (FR-001). Extension picks the format. |
| `Model` | `actuators: Vec<Actuator>`, `links: Vec<Link>` | `actuators.len() >= 1` (edge case "zero actuators"). |
| `Actuator` | `name`, `kind` (Revolute, Continuous, Prismatic), `lower`, `upper`, `speed`, `effort` (f64 while parsing), `pos` (line, col) | One per non-fixed joint, in file order (FR-002, FR-003). Name unique. |
| `Link` | `name`, `parent: Option<usize>`, `mass` (kg), `com: [f64; 3]` (m), `pos` | Name unique. Exactly one root (FR-011). |
| `ModelError` | `file`, `pos: Option<(line, col)>`, `element` (joint or link name), `reason`, `marker: Option<&str>` | Rendered as one `compile_error!` (FR-008). |

### Validation, in order (first failure stops the build)

1. File found and readable; extension is `.urdf`, `.ron` or `.json`.
2. Parses (error carries line and column).
3. Root shape: URDF root is `<robot>`; `.ron`/JSON matches the schema, no unknown field
   (edge case "a field the format does not know").
4. Joint types: `fixed` skipped; `revolute`, `continuous`, `prismatic` kept; `floating`,
   `planar` stop with "roadmap: non-1-DOF joints" (FR-010).
5. `<mimic>`, or a `<transmission>` with more than one joint or actuator, stops with
   "SRS index space: multi-actuator joint" (FR-009).
6. Unique joint and link names.
7. Limits: revolute and prismatic need `<limit lower upper velocity effort>`; continuous needs
   `velocity effort` only. `lower <= upper`, `velocity >= 0`, `effort >= 0` (edge cases).
8. Units: prismatic position and speed × 1000 (m to mm, FR-017). Every value finite and within
   `f32` after conversion (SRS FR-2.2).
9. Tree: one root link; every joint's parent and child are known links (FR-011).
10. At least one actuator.

## Emitted facts (in the user crate, `no_std`, no heap)

The proc macro emits, at the call site, only `N`, `model::*` and `actuator!` (the rows below
not marked "front door"). kmr_api's `macro_rules!` front door emits the rest, plus `__Spec`,
`__NB`, `__M`, `__TABLE` and the hardware checks (specs/004-kmr-api/contracts/robot-hardware.md).
No item comes from both.

| Item | Type | Source |
| --- | --- | --- |
| `N` | `const usize` | actuator count (FR-002) |
| `State<'a>`, `Robot` | type aliases, **front door** | kmr_api types; `Robot = kmr_api::Robot<__Spec>`; carry N once kmr_core is const generic (FR-004) |
| `model::NAMES` | `[&'static str; N]` | joint names, index order (FR-015) |
| `model::LOWER`, `model::UPPER` | `[Q; N]` | position limits; `Q(0.0)` placeholder where `CONTINUOUS` is set. The wiring (kmr_api plan B3) maps each set `CONTINUOUS` bit to `lower`/`upper` = `None` in the core's `Limit`, so a continuous joint is never clamped to 0 (FR-012, FR-013) |
| `model::SPEED` | `[Qd; N]` | top speed (FR-012) |
| `model::EFFORT` | `[Tau; N]` | top effort (FR-012) |
| `model::PRISMATIC`, `model::CONTINUOUS` | `[u64; MASK_WORDS]` (kmr_core's constant through kmr_api; 1 under D4 A) | kind bit sets; revolute = neither (FR-013, FR-020). Emitted as `{ let mut m = [0u64; MASK_WORDS]; m[0] = ..; m }`, so the macro needs not know `MASK_WORDS` |
| `model::LINK_NAMES` | `[&'static str; L]` | link names (FR-016) |
| `model::LINK_PARENT` | `[Option<u16>; L]` | parent link index; `None` for the root |
| `model::LINK_MASS` | `[f32; L]` | kg |
| `model::LINK_COM` | `[[f32; 3]; L]` | m, link frame |
| `actuator!("name")` | `macro_rules!` → `usize` literal | name lookup; unknown name is a compile error (FR-015) |
| `const _: &[u8] = include_bytes!(..)` | unused const | rebuild tracking (FR-021) |
| `const _: () = assert!(N == kmr_core::state::JOINTS, ..)` | temporary, **front door** | removed when kmr_core takes N as a const generic |

Bit `i` of a set is word `i / 64`, bit `i % 64`, the same layout as `kmr_core::hal::Mask`.

## Relationships

- Actuator index `i` is the same everywhere: `NAMES[i]`, `LOWER[i]`, the state arrays, the HAL
  (SRS Definitions "Index space").
- A preset's hardware table (kmr_adapters) has exactly N rows, in the same index order; a
  mismatch fails the build through `robot!`'s array-type check (kmr_api FR-007; spec edge case,
  R7).
- A payload mount point names a link; `register_payload` (kmr_api, SRS FR-8.4) checks it against
  `LINK_NAMES` and returns a `run()` error when absent (User Story 6).
