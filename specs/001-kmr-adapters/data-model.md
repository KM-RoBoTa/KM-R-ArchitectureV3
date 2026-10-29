# Data Model: kmr_adapters, Dynamixel (2026-10-28)

Engine-side types (`Mask`, `Group`, `Layout`, `Overrides`, `Written`, `Command`, `Sample`,
`Batch`, `Faults`) are `kmr_core/src/hal.rs`, the authority. This file covers the adapter side.
Public surface: [contracts/hardware-table.md](contracts/hardware-table.md),
[contracts/port-binding.md](contracts/port-binding.md).

Layout rules follow FR-034 (SRS NFR-10): per-actuator data in arrays indexed by actuator,
booleans as bitsets out of band where they are per-tick, buses by index, enums not trait objects.

## Hardware table (FR-023 to FR-028 (SRS FR-6.1, FR-6.4))

### `Table<const NB: usize, const N: usize>`

| Field | Type | Rule |
|---|---|---|
| `buses` | `[Bus; NB]` | NB ≥ 1; NB ≤ backend `MAX_PORTS` (4), checked in `check` of the adapter |
| `actuators` | `[Actuator; N]` | exactly N, model order; the array length is the count `robot!` checks |

A preset is a `Table` const written by KM-RoBoTa. A `main.rs` declaration is the same type. An
override returns a new `Table` and re-runs `Table::check`.

### `Bus`

| Field | Type | Rule | Default |
|---|---|---|---|
| `label` | `&'static str` | non-empty, unique in the table; named by `.bus("label")` (D6) | — |
| `baud` | `u32` | accepted by every model on the bus | — |
| `read_every` | `Option<NonZeroU32>` | ticks between reads | `DEFAULT_READ_EVERY` (D9) |

Brand is the table's type (`dynamixel::Table`). Protocol follows from each model.

### `Actuator`

| Field | Type | Rule | Default |
|---|---|---|---|
| `bus` | `usize` | `< NB` | — |
| `id` | `u8` | `0..=252`, unique on its bus | — |
| `model` | `Model` | one of six | — |
| `mode` | `Mode` | supported by the model; `Position` only until SRS §8 mode override (FR-018) | `Position` |
| `torque` | `Torque` | `On` or `ReadOnly` (SRS FR-5.8) | `On` |
| `zero` | `Zero` | `Center` or `Raw(i32)` within the model's raw range (D8) | `Zero::DEFAULT` = `Center` |

### Checks in `Table::check` (const evaluation, compile errors)

Each message names the actuator index or the bus label (proved, research R4):

- bus index out of range; id above 252; id repeated on a bus;
- mode not supported by the model; mode other than `Position` (names SRS section 8);
- baud not accepted by a model on its bus;
- empty or repeated label; NB = 0;
- `Zero::Raw` outside the model's raw range.

`robot!` adds the count check (E0308) and, later, the kind check
(`check_kinds(self, prismatic: [u64; MASK_WORDS]) -> Self`, prismatic is a compile error naming
the index and SRS FR-2.2).

### Declared costs (FR-035 (SRS NFR-11, FR-5.3))

| Const | Type | Rule |
|---|---|---|
| `READ_COST` | `Duration` | per preset (`XmBench::READ_COST`); line time of one full read from `HARDWARE` and the backends' read plan; equals `Layout::read_cost` from `check` |
| `WRITE_COST` | `Duration` | per preset; line time of one full write; equals `Layout::write_cost` |

Public for kmr_api C7 (`kmr_api::presets`) and the bench. Return delay and USB latency are not
counted (as `io::Groups::costs`).

### Models

| Model | Protocol | Series | Raw position | Centre | Current register |
|---|---|---|---|---|---|
| `Ax12a`, `Rx28` | 1 | AX | 0..1023 over 300° | 512 | none |
| `Xm430W210`, `Xw430T200`, `Xw540T140`, `Xw540T260` | 2 | X | 0..4095 per turn | 2048 | yes |

## Port binding (FR-029 to FR-031 (SRS FR-6.3, FR-6.4))

### `Binding` (one per bus plugin)

| Field | Type | From |
|---|---|---|
| `port` | `Port` | `Bus::by_id(serial)` → `Port::ById`, `Bus::by_dev(path)` → `Port::ByDev` |
| `baud` | `Option<u32>` | `.baud(..)`; overrides the table's baud for that bus |
| `bus` | `Option<&'static str>` | `.bus("label")`; may be `None` for a one-bus table |

### Bound bus (inside the adapter, after `bind`)

| Field | Type | Set at |
|---|---|---|
| `port` | `Port` | `bind` |
| `baud` | `u32` | `bind` (plugin baud, else table baud) |
| `device` | `Option<DevicePath>` | `open`, after resolution and pre-flight |

## Adapter state: `Dynamixel<B, NB>` (`N` = `JOINTS` until core's N)

| Item | Shape | Set at | Read at |
|---|---|---|---|
| `backend` | `B: Backend` | `new` / `bind` | every call |
| `table` | `Table<NB, N>` (by value) | `bind` | every call |
| `bound` | `[BoundBus; NB]` | `bind`, `open` | `open`, `close`, errors |
| `groups` | `Groups`: one table per (bus, series), fixed arrays | `check` | `read`, `write`, `hold` |
| `open` | `bool` | `open`, `close` | `hold`, `torque_off` |
| `reads` | `u64` | `read` | cadence |

`DxlRs` keeps one `TorqueOn` handle per actuator it powered (`[Option<Powered>; N]`) for sync
writes. `force_torque_off` needs no handle and clears it.

## Lifecycle

```text
          bind (no I/O) ──err──> run() error
               │
Bound ──check──> Checked ──open──> Open ──zero──> Zeroed
                   │err            │err            │err
                   v               v               v
             run() error     torque_off*, close   torque_off, close, error

Zeroed ──ping, health, configure, read_all, torque_on──> Powered
Powered ──read / write per tick──> Powered
Powered ──shutdown──> hold ──> torque_off ──> close ──> Closed
Powered ──estop, error, panic──> torque_off ──> close ──> Closed

* no-op: the backend opened nothing.
```

`torque_on` powers `Torque::On` actuators and writes torque off on `Torque::ReadOnly` ones.
`torque_off` writes torque off on every actuator of every opened bus. The core's `Powered` guard
owns the order; the adapter never sees the signal.

## Per-tick faults (FR-014, FR-015, FR-032 (SRS FR-2.6, FR-13.4))

| Event | Bits |
|---|---|
| One actuator lost in a read | `stale`, `offline` |
| Group read transaction failed | `stale`, `bus_error` for the group |
| Read position outside the register range | `stale` |
| Write transaction failed | `bus_error` for what it carried |
| Written field the mode does not take, read-only actuator, out-of-range or non-finite target | `rejected` |

## Units (FR-021 (SRS FR-2.2))

- position: `rad = (raw - origin(actuator)) * span_rad / resolution`; `origin` is the model
  centre or `Zero::Raw` (D8);
- velocity: `rad/s = raw * rpm_per_unit * 2π / 60`;
- current: `N·m = raw * amps_per_unit * torque_constant` (datasheet constant, approximate).

Prismatic actuators: no Dynamixel model has a linear conversion, so `check_kinds` refuses them.
