# Data model: kmr_core

All types are fixed-size, heap-free and `Copy` unless noted. `N` is the actuator count: `JOINTS = 4`
until M2, then `const N: usize` in 1..=64 (FR-011, FR-012 (SRS FR-6.1, Def. "Write mask"), D4).
"Milestone" says when a field appears. Visibility follows FR-056 (SRS NFR-3): `pub(crate)` unless
named in [contracts/engine-surface.md](contracts/engine-surface.md).

## Builder and run

### `Robot<const N, CS = Scheduled, E = Empty>` (robot.rs)

| Field | Type | Milestone | Rule |
|---|---|---|---|
| `config.time` | `Time` | today | the one clock (FR-029) |
| `config.dt` | `Option<Duration>` | M3 | `None` → 1 ms + one warning; `Some` outside [10 µs, 1 s] → `RunError::Config` (FR-027 (SRS FR-3.1)) |
| `config.hold` | `Option<Duration>` | M4 | `None` default (FR-033 (SRS FR-4.1)) |
| `config.limits` | `[Limit; N]` | M5 | from `robot!` data; policy per actuator (FR-037) |
| `config.envelopes` | `[Envelope; N]` | M5 | user-set through `kmr_api` (FR-037 (SRS FR-13.5)) |
| `controllers` | `CS` (six HList buckets) | today | routed at registration (FR-006) |
| `E` | `Empty` or `Filled` | M3 | type-state; `run` needs `Filled` (FR-009 (SRS NFR-4)) |

N carries no field. It is needed by the closure bounds and cannot be inferred at `run()`
(design-const-generic-n §2.1).

### Run lifecycle (runtime.rs)

```text
Finalize ──check()──▶ PreInit ──guard+open+zero──▶ Init ──ping..read_all, estop?, torque_on──▶ PostInit
   │ Err: return            │ estop: return Ok       │ Err/estop: guard drops   │ Err/estop: guard drops
   ▼                        ▼                        ▼                          ▼
 (no I/O)                (no I/O)          torque_off, close           torque_off, close
                                                                               │
                         ┌──────────────────── Tick k ◀──────────────────────────┘ arm(origin)
                         │ check → begin_tick → push+read → First → check → EachTick → check
                         │ → Last → check → limits(M5) → batch+write → end_tick → check
                         │ → clear Desired → sleep to origin + (k+1)·dt
                         ├── shutdown ─▶ hold(target) ─▶ wait(hold, estop cuts) ─▶ guard drops
                         ├── estop ─────────────────────────────────────────────▶ guard drops
                         └── panic ──── unwind ─────────────────────────────────▶ guard drops
```

Exit table: design-first-motor §3. K1.9 adds one estop check before `torque_on`.

### `Powered<'a, A: Hal<N>>` (runtime.rs, M1)

Drop guard over `&mut A`. Built just before `open`. `Drop` calls `torque_off` then `close`. Every
exit after `open` leaves through it (FR-043 (SRS FR-5.6, NFR-2)). Needs `panic = "unwind"`.

### `RunError<E>` (error.rs, M3)

`Config(ConfigError)` before any I/O: `DtOutOfRange { dt }`, `NoPolicy { actuator }` (D2 B).
`Hal(E)` from `check` or bring-up, naming actuator and port (adapter's error). Both signals end
in `Ok(())` (FR-038 (SRS §8 open)).

## State

### `RobotState<N>` (state/robot_state.rs)

| Part | Type | Rule |
|---|---|---|
| `history` | `History<N, HISTORY_DEPTH>` | read-only to controllers (FR-017 (SRS FR-2.1, FR-2.3)) |
| `desired` | `Desired<N>` | write-only to controllers (FR-019 (SRS FR-2.5)) |

`DEPTH` leaves the type in K2.3 (FR-013). Engine block (`pub(crate)`, M1): `written()`,
`command()`, `clear_desired()`, `push_slot()`, `hold_target()`.

### `History<N, DEPTH>` (state/history.rs)

| Part | Type | Milestone | Rule |
|---|---|---|---|
| `buf` | `CircularBuffer<DEPTH, Slot<N>>` | today | one slot pushed per tick, a copy of `prev(0)` |
| `age` | `[Option<u32>; N]` or `[u32; N]` + `ever_read` (storage open) | M3 | ticks since last read; reset by `read`'s refreshed mask (FR-018). Read as `Option<u32>` per actuator: `None` until first read |
| `ever_read` | `Mask<N>` | M3 | read of an actuator not in it → `NoSample` (research R6) |

### `Slot<N>` (today `State<N>`; final name open, SRS §8)

SoA, 64-byte aligned (FR-059 (SRS NFR-9), K3.7).

| Field | Type | Milestone |
|---|---|---|
| `q`, `qd`, `tau` | `[Q; N]`, `[Qd; N]`, `[Tau; N]` | today |
| `kp`, `kd` | `[Kp; N]`, `[Kd; N]` | M3 (FR-014 (SRS FR-2.2)) |
| `faults` | `Faults<N>` | M4 (FR-051 (SRS FR-13.4)) |
| `tick_flags` | `TickFlags` | M4 |

`prev(k)` = k ticks ago. At tick 0, `prev(1)` is the `read_all` seed (SRS problem, design §7.5).

### `Desired<N>` (state/desired.rs)

| Field | Type | Rule |
|---|---|---|
| `q`, `qd`, `tau` (+ `kp`, `kd` M3) | `[T; N]` | values only |
| `written` | `Written<N>` | one `Mask<N>` per field (FR-021). Every setter sets its bit; an out-of-range index sets none and returns `OutOfRange` |

Cleared (values and masks) after each write (FR-019) and after each startup phase, with one
warning per run if anything was written (FR-020 (SRS FR-5.2), research R4 OC7).

### Value types (state/joint_state.rs)

`Q`, `Qd`, `Tau`, `Kp`, `Kd`: `#[repr(transparent)]` over `f32` (FR-016 (SRS FR-10.2)), `From<f32>`
and `Into<f32>` (FR-014). Units: rad, rad/s, N·m revolute; mm, mm/s, N prismatic (FR-015). The
engine never converts. Sealed field set: `sealed::Slot` + `StateField` in **`state/field.rs`
(invariant)**; `Slot` methods generic over N, the trait not.

### `StateError`

`OutOfRange { len }`, `NoSample { depth }` (FR-025), `Undecided(&'static str)` (D5 A, research R7).

## Masks and flags (hal.rs)

### `Mask<N>([u64; MASK_WORDS])`

`MASK_WORDS = 1` (D4 A). `set`, `clear`, `get`, `iter`, `any`, `ALL`. Index ≥ N is ignored (no
panic). `const CHECK` asserts 1 ≤ N ≤ 64, named once in `Robot::new`.

### `Faults<N>` (per actuator, in each slot, M4)

`stale`, `bus_error`, `offline`, `rejected`, `joint_limit`, `envelope` (D3 A): one `Mask<N>`
each. Set by the adapter (`read`, `write`) and by the limit and envelope stage. Controllers read
by depth; never set.

### `TickFlags` (per tick, in each slot, M4)

`u32` bitset: `OVERRUN`, `SENSOR_MISSING`, `REMOTE_TIMEOUT`, `UNAUTHORIZED`. The envelope flag
is per actuator, in `Faults<N>`. Bits from outside the loop come through `ExternalFlags`.

### `ExternalFlags` (M4)

One `AtomicU32`. `raise(bits)`: `fetch_or`, any thread. `take()`: `swap(0)`, loop only, at the
inbound stage, folded into the current slot (FR-052). Needs CAS (research R13).

## Clock (clock.rs)

### `Time`

| Field | Type | Milestone | Reader |
|---|---|---|---|
| `delta` | `Duration` | today | `dt()` |
| `origin` | `Instant` | today | grid origin, set by `arm` |
| `tick` | `u64` | M1 | `tick()`: index of the tick in progress |
| `overrun_count` | `u64` | M1 | `overrun_count()` |
| `last_overrun` | `Option<Duration>` | M1 | previous tick's lateness (OC5) |
| `accumulated` | `Duration` | today | `accumulated_overrun()` |
| `elapsed` | `Duration` | M1 | `elapsed()`, stamped at tick start, from `arm` (OC6) |

Deadline k = `origin + k·dt`. After an overrun, missed slots are skipped (FR-028 (SRS FR-3.2)).
Only the engine writes, between phase walks.

## Signals (signal.rs, invariant, unchanged)

`Signal { kind: Shutdown | EmergencyStop, meta: &'static SignalMeta }`. Monotonic CAS on an
`AtomicU8` level; first reason wins (FR-035, FR-036 (SRS FR-4.3, FR-4.4)). Five bus checks per
tick after K1.8 (start, after `First`, after `EachTick`, after `Last`, end).

## Hardware contract types (hal.rs)

`Layout<N>` (groups, `read_every` per actuator, order sorted once), `Overrides` (empty,
`#[non_exhaustive]`, research R9), `Command<'a, N>`, `Sample<'a, N>`, `Batch<'a, N>`,
`Runs<'a, N>`, `HoldTarget<N>` (owned `Command` data: latest sample overlaid with this tick's
writes, OC1). Call order and errors: [contracts/hal.md](contracts/hal.md).

## Limits (M5)

`Limit { lower: Option<Q>, upper: Option<Q>, speed: Qd, effort: Tau }` per actuator, typed, from
`robot!` data in actuator order. `None` means no position bound: a continuous joint, never
clamped to 0 (kmr_api maps the model's `CONTINUOUS` bit to `None`). `Envelope { q: Option<(Q, Q)>,
qd: Option<(Qd, Qd)>, tau: Option<(Tau, Tau)> }` per actuator, user-set; a breach clamps and sets
`envelope` (D3 A). `Policy { Clamp, Warn, Abort }`, per actuator; `Abort` raises
`emergency_stop!` with reason `JointLimit` (FR-037). A limited actuator without a policy: `run()`
error before I/O (assumed, pending human: D2 B).
