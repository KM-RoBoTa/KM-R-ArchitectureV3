# Data model: kmr_ffi

Phase 1 of [plan.md](plan.md). Layouts are the ones proved in [research.md](research.md) R1.
N is `robot!`'s N (1..=64), D is `HISTORY_DEPTH`. Bit i of a mask is actuator i.

## Field list (the one declaration)

`for_each_field!(q, qd, tau)` in `kmr_ffi/src/fields.rs`. Adding `kp`, `kd` is one edit here plus
one arm in kmr_api's exhaustive `FieldId` match (FR-014 (SRS FR-10.2)). Everything below that
says "per field" is generated from it, headers and bridges included.

`for_each_flag!(stale, bus_error, offline, rejected, joint_limit, envelope)` is the per-actuator
flag list, following the flags of SRS FR-13.4 as core plan K4.1 delivers them (`envelope` per
actuator under D3 A, assumed, pending human). Same rule.

## Types that cross

All `#[repr(C)]`, `Copy`, no pointers, no lifetimes. Rust side generic over N and D; C side
fixed by `KMR_N` and `KMR_HISTORY_DEPTH`.

### `ActuatorFlags`

| Field | Type | Meaning |
|---|---|---|
| one per flag | `u64` | bit i set = that fault on actuator i in this slot (FR-010 (SRS FR-2.6, FR-13.4)) |

### `Slot<N>` (one History slot)

| Field | Type | Meaning |
|---|---|---|
| one per field | `[f32; N]` | value in engine units: rad, rad/s, N·m or mm, mm/s, N (FR-007 (SRS FR-2.2)). A stale actuator holds its last good value, never zero |
| `flags` | `ActuatorFlags` | per-actuator faults |
| `tick_flags` | `u32` | per-tick faults (overrun, remote timeout, unauthorized), one bit each |
| `reserved` | `u32` | zero; keeps the size a multiple of 8 |

### `ReadView<N, D>`

| Field | Type | Meaning |
|---|---|---|
| `samples` | `u32` | valid slots, 0..=D. 0 = "no sample" (no-hardware adapter, core FR-018) |
| `reserved` | `u32` | zero |
| `age` | `[u32; N]` | ticks since actuator i's last hardware read, latest slot only (core FR-018: `Option<u32>` per actuator); `u32::MAX` (`KMR_AGE_NONE`) = never read |
| `slot` | `[Slot<N>; D]` | `slot[k]` = `prev(k)`, same values as `prev_q(k)` etc. (FR-010 (SRS FR-2.3)) |

Sample age = the core's `sample_age()`, derivable from `age`; not stored twice. Age is not per
History slot: the core keeps none.

Construction checks `1 <= N <= 64` at compile time (D4 A).

### `Written`

| Field | Type | Meaning |
|---|---|---|
| one per field | `u64` | bit i set = foreign code wrote that field for actuator i |

### `WriteBuf<N>`

| Field | Type | Meaning |
|---|---|---|
| one per field | `[f32; N]` | values to write |
| `written` | `Written` | which entries count (FR-011 (SRS FR-2.5)) |

### `TimeView`

| Field | Type | Meaning |
|---|---|---|
| `tick` | `u64` | index of the tick in progress, 0 first |
| `dt_ns` | `u64` | `dt` |
| `elapsed_ns` | `u64` | since start, stamped at tick start |
| `overrun_count` | `u64` | ticks late so far |
| `last_overrun_ns` | `u64` | previous tick's lateness; 0 = on time |
| `accumulated_overrun_ns` | `u64` | sum of lateness |

Integers per R3 (Q1). Same values as the `Time` readers (SRS FR-3.3).

### Status (return value)

| Value | Name | Effect |
|---|---|---|
| 0 | `KMR_CONTINUE` | apply writes |
| 1 | `KMR_SHUTDOWN` | apply writes, then `shutdown!` |
| 2 | `KMR_EMERGENCY_STOP` | apply writes, then `emergency_stop!` |
| < 0, or > 2 | error | apply nothing; `call` returns `ForeignError(code)` |

D13 A (R5).

## Rust-only types

| Type | Where | Meaning |
|---|---|---|
| `CFn<N, D>` | kmr_ffi | `unsafe extern "C" fn(*const ReadView<N, D>, *mut WriteBuf<N>, *const TimeView, *mut c_void) -> i32` |
| `Status` | kmr_ffi | `Continue`, `Shutdown`, `EmergencyStop` |
| `ForeignError` | kmr_ffi | `Code(i32)` for C/C++; `Python(String)`, `Lua(String)` for the bridges; `Load(..)` for a bridge that failed to start. Human-readable `Display` |
| `python::Bridge`, `lua::Bridge` | kmr_ffi, opt-in | an interpreter with the user's script and function loaded, plus its persistent sequences |

## One call (C/C++ path)

```text
forwarder (user's Rust controller)
  └─ kmr_api::ffi::call(f, time, state, ctx)
       1. fill   ReadView from State (every depth, flags; per-actuator age), TimeView from Time
       2. clear  WriteBuf
       3. call   f once (kmr_ffi::call, the one unsafe block)
       4. rc < 0 or > 2  →  clear WriteBuf, return Err   (nothing applied, FR-012)
       5. apply  for each field, for each set bit: the field's setter (mask, envelopes)
       6. rc 1/2 →  shutdown! / emergency_stop!
       7. return Ok(Status)
```

No heap, no lock, no `dyn` in steps 1 to 7 (FR-021 (SRS NFR-1)). Python and Lua replace step 3
with the bridge's call; their interpreter work is the user's cost (FR-005 (SRS FR-10.3)).

## Bridge lifecycle

```text
main.rs: Bridge::load(path, fn)  ──Err──▶ returned to main, before run()   (FR-020)
             │ Ok
             ▼
add_controller_with(phase, bridge, forwarder)    (bridge is the context, SRS FR-1.1)
             │
             ▼  each call: refill sequences → call fn → read write object → apply
             │  script error → ForeignError, nothing applied
             ▼
run() ends → bridge dropped with the controller (interpreter shut down)
```

## Validation rules

- Every array is exactly N long; History exactly D slots (FR-008 (SRS FR-6.1)).
- Build script N ≠ `robot!` N, or depth ≠ `HISTORY_DEPTH`: compile error (FR-015, SC-005).
- A field in the core without its FFI line: compile error (FR-014, SC-004).
- `samples == 0`: foreign code must not read `slot`; the C example returns an error then.
- Out-of-range writes cannot be expressed: the header's `kmr_set_<field>(w, i, v)` ignores
  `i >= KMR_N`. A raw array write past N is C memory unsafety, the user's.
