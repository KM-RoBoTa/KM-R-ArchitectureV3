# Contract: the handles a controller gets

`State`, `Time`, `Sensors`, groups, units and fault flags. Each handle is a borrowed wrapper
with a private named field (FR-028 (SRS section 3 State, NFR-3)). Methods forward to the core;
kmr_api adds names, not logic. Everything here runs in the tick: no heap, no lock, no wait
(FR-067 (SRS NFR-1)). Phase letters match plan.md.

## `State` (exists unless marked)

| Method | Returns | Phase | Spec ids |
| --- | --- | --- | --- |
| `q()`, `qd()`, `tau()` (+ aliases `position`, `velocity`, `torque`, `effort`) | `Option<[T; N]>`; `None` before the first sample | exists | FR-029, FR-035 |
| `q_at(i)`, `qd_at(i)`, `tau_at(i)` | `Result<T, StateError>` | exists | FR-029 |
| `prev_q(depth)`, `prev_q_at(i, depth)`, and the `qd`/`tau` forms | `Result<..>`; depth ≥ `HISTORY_DEPTH` is an error | exists | FR-029 (SRS FR-2.3) |
| `set_all_q([Q; N])`, `set_q_at(v, i)`, and the `qd`/`tau` forms | `set_*_at`: `Err` and nothing written when `i ≥ N` | exists | FR-030 (SRS FR-2.5) |
| `group([i; K])` | `GroupView<'_, K>`, same accessor set | exists | FR-034 (SRS FR-2.4) |
| `age_at(i)`, `sample_age()`, `stale_at(i)` | `Result<Option<u32>, StateError>` ticks, `Option<u32>` ticks (oldest actuator), `bool`; `None` = never read (no-hardware run, or before the first read) | C | FR-031 (SRS FR-2.1, FR-2.6) |
| `faults()`, `prev_faults(depth)` | `&Faults` view | C | FR-039 (SRS FR-13.4) |
| `read_every(i)` | `NonZeroU32` ticks, read-only | C | FR-042 (SRS FR-5.3) |
| `initial_state()`, `go_home(ramp)` | compile error "kmr: SRS FR-2.7: home state is open" (D5 A, research R10) (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.) | B | FR-036 (SRS FR-2.7) |

No raw field, no way to reach the inner core handle, no user field type (SC-008).

## `Time` (phase A; forwards kmr_core #14)

| Method | Returns |
| --- | --- |
| `elapsed()` | `Duration` |
| `dt()` | `Duration` |
| `tick()` | `u64`, index of the tick in progress, 0 first |
| `overrun_count()` | `u64` |
| `last_overrun()` | `Option<Duration>` |
| `accumulated_overrun()` | `Duration` |

Never `f32` seconds (FR-038 (SRS FR-3.3)). Meaning of `elapsed`'s origin and of
`last_overrun` is the core's (design-first-motor open choices 5, 6).

## `Sensors`

Passed to every inline controller; no method yet (FR-040, SRS FR-5.7 open).

## Units (phase A)

```rust
pub struct Deg(pub f32);  pub struct Cm(pub f32);  pub struct M(pub f32);
pub struct DegPerS(pub f32);  pub struct CmPerS(pub f32);  pub struct MPerS(pub f32);
impl From<Deg> for Q;  impl From<Cm> for Q;  impl From<M> for Q;         // rad, mm, mm
impl From<DegPerS> for Qd;  impl From<CmPerS> for Qd;  impl From<MPerS> for Qd;
```

`Q::from(Deg(180.0))` is π, `Q::from(M(1.0))` is 1000, `Q::from(Cm(1.0))` is 10, within f32
precision (SC-009). No `Tau` conversion. The type does not know an actuator's kind (SRS issue 4).

## `Faults` view (phase C, after kmr_core #13)

Per actuator, bit sets: `stale`, `bus_error`, `offline`, `rejected`, `joint_limit`, `envelope`
(D3 A, assumed, pending human; the SRS FR-13.4 list lacks it). Per tick: `overrun`,
`sensor_missing`, `remote_timeout`, `unauthorized`. Read-only. Same split as the core's
`Faults<N>` and `TickFlags` (kmr_core K4.1) and kmr_ffi's `kmr_actuator_flags`.

## Signals

`shutdown!` and `emergency_stop!` re-exported from kmr_core, callable from any depth and any
thread (FR-017 (SRS FR-4.1 to FR-4.4)).
