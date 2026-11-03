# T062: visibility review of kmr_core/src (FR-056, SRS NFR-3)

2026-11-03, on agent-goal after T052. Method: every `pub` item outside test code
listed, each one checked against
[`specs/002-kmr-core/contracts/engine-surface.md`](../../specs/002-kmr-core/contracts/engine-surface.md)
and [`hal.md`](../../specs/002-kmr-core/contracts/hal.md), and searched for in kmr_api,
kmr_adapters, kmr_ffi, kmr_model, bench and controllers. `RUSTFLAGS=-Wunreachable_pub cargo
check -p kmr_core` for the `pub` items no path reaches.

## Narrowed

| item | was | now | why |
|---|---|---|---|
| `hal::{Audit, Plan, Step}` | `pub use` in `hal` | `pub(crate) use` | named only by the `#[doc(hidden)]` methods of the sealed `Drivers`, which the engine alone calls; no other crate names them |
| `Limit::bounds_anything` | `pub fn` | `pub(crate) fn` | used by `limits::require_policies` only |
| `SignalKind` | `pub enum` in a `pub(crate)` module | `pub(crate) enum` | the only `unreachable_pub` hit; `Signal::kind` is `pub(crate)` already |
| `Host`, `host_seal` | new in T052 | `#[doc(hidden)] pub` at the root | the contract lists `Host` under the hidden entries: for a `no_std` port, never for controllers |

## Kept public, with the reason

- Builder, state handle, values, `StateField`, `FieldId`, `HISTORY_DEPTH`, `Time`, errors,
  `Limit`/`Policy`/`Envelope`: named by engine-surface.md and used by kmr_api.
- `hal` (`Hal`, `seal::Adapter`, `Mask`, `MASK_WORDS`, `Written`, `Faults`, `Layout`, `Group`,
  `Overrides`, `Command`, `Sample`, `Batch`, `Runs`, `NoHardware`, `TickFlags`, `Claim`,
  `Coverage`, `Member`, `MemberError`, `HardwareError`, `Drivers`): hal.md, used by
  kmr_adapters and kmr_api.
- `hal::Slots`: `#[doc(hidden)]`, no other crate names it, but it is in the signatures of the
  hidden `Drivers` methods and lives in the public `hal` module itself; `pub(crate)` would trip
  `private_interfaces`. Moving it into `hal/set.rs` would let it go the way of `Plan`; left for
  a change that touches `hal.rs` anyway.
- `ExternalFlags`, `EXTERNAL_FLAGS`: no user today; engine-surface.md names `raise` for the
  kmr_api Zenoh gateway (M4, in flight on another branch).
- `Robot::run_on_staged`, `Stages`: hidden, named for kmr_api Z3 (Zenoh).
- `Signal`, `SignalMeta`, `_emit_shutdown`, `_emit_emergency_stop`: the `shutdown!` and
  `emergency_stop!` macros expand in the user's crate and spell `$crate::` paths.
- `Instant` (T052): the type of `Host::now`; documented, as a port needs to build it.
- `HCons`, `HNil`, controller node types, `Scheduled`, `Drive`, `Insert`, `Schedule`, labels,
  `Empty`, `Filled`, `HasController`: kmr_api names them in its wrapper's bounds; fields and
  constructors stay sealed.
- `test_util`: behind the `test-util` feature only.

Nothing else found. Oracle after the change: workspace build and tests green.
