# Contract: the private SDK seam and the strict fake

`dynamixel::backend::Backend` is `pub(crate)`. Two implementations: `DxlRs` (feature
`dynamixel`) and `Scripted` (tests). Values are raw; the adapter converts units.

## Change: `force_torque_off` (step A3)

```rust
/// Writes Torque Enable = 0 to actuator `id`, whether or not this
/// backend switched it on. Clears any handle kept for it.
fn force_torque_off(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError>;
```

`torque(.., false)` stays for symmetry but the adapter no longer calls it. Every other method is
unchanged.

### `DxlRs` side (step A4)

`dxl_rs` has `force_torque_off(&self)` on the `TorqueOff` type-state (branch
`agent/force-torque-off`, the one the dev bench builds): it writes `Torque_Enable` = 0 with
`write1ByteTxRx1` / `write1ByteTxRx2` at the model's address (24 on AX, 64 on X) and reads the
status packet. A `TorqueOff` actuator is built for any id without I/O, so
`DxlRs::force_torque_off` also reaches an actuator this backend never switched on, then drops the
`Powered` entry kept for it, if any. No `unpower` fallback is left on this path: `unpower` serves
`torque(.., false)` only. The strict fake models the same behaviour.

The adapter's `torque_off` tries each actuator up to `TORQUE_OFF_ATTEMPTS` (3) times, back to
back, and warns for one that fails them all.

## Change: `sync_write_start_goal` (2026-10-29, torque on without a jump)

```rust
/// Goals of one group, `(id, raw)`, in `register`, none of them switched on here: what
/// each holds once `torque` switches it on. One transaction where the SDK allows it.
/// Refused whole, before any write, when one of them is switched on here.
fn sync_write_start_goal(&mut self, port: usize, model: Model, register: GoalRegister,
    goals: &[(u8, u32)]) -> Result<(), BackendError>;
```

`torque_on` writes Goal Position = present position (read then), or Goal Velocity / Goal
Current = 0, before Torque Enable. `torque(.., true)` is refused (`Err(Io)`) unless a start goal
was written since the actuator was last switched; a start goal naming an actuator switched on here
is refused whole. The fake models the goal registers and pins this order. `DxlRs` follows it for
Goal Position: `dxl_rs`'s `set_goal_position_raw` on the `TorqueOff` type-state writes the goal,
torque still off, and confirms it (`TxRx`). That is one write per actuator, not a sync write:
`dxl_rs` sync-writes only from torque-on lists. `torque(.., true)` then enables the torque.
`DxlRs` refuses Goal Velocity and Goal Current, with one warning: `dxl_rs` writes Goal Position
only.

## Strictness: what the fake must refuse (step A1)

The fake answers as `DxlRs` does today, error for error. A lenient fake hid real failures
(research R2).

| Call | `DxlRs` today | Fake today | Strict fake |
|---|---|---|---|
| `torque(.., false)` on an actuator this backend never powered | `Err(Io)`, warns once | `Ok` | `Err(Io)` |
| `force_torque_off` | (new) | (new) | `Ok`; actuator off; logged as `ForceTorqueOff` |
| `sync_write_goal` naming an actuator not powered here | `Err(Io)`, whole write | `Ok` | `Err(Io)`, whole write |
| `read_health` on Protocol 1 | `Err(Io)` | `Ok(HEALTHY)` | `Err(Io)` (until step E2 maps it to `Unsupported` on both) |
| `read_setting` on Protocol 1 | `Err(Io)` | panics (`assert_eq!`) | `Err(Io)` |
| `sync_read_state` on Protocol 1 | `Err(Io)` | panics (`assert_eq!`) | `Err(Io)` |
| `sync_read_state` with `out.len() != ids.len()` | `Err(Io)` | panics (`assert!`) | `Err(Io)` |
| `sync_read_state` wire transactions | 3 per group | 1 record | `STATE_READS.len()` records |
| a call before `open_ports` or after `close_ports` | `Err(Io)` | answers | `Err(Io)` |

State the fake tracks per (port, id), separately:

- `powered_here`: switched on through this backend (what `DxlRs` keeps a handle for);
- `torque`: the register, also set by `already_on` (an earlier run left it on).

`torqued(port, id)` reads the register. The torque tests assert on it.

## Torque tests (step A2), red before A3

| Test | Setup | Expect |
|---|---|---|
| `every_actuator_ends_torque_off_read_only_included` | all ids `already_on`, actuator 3 `ReadOnly`; bring-up, a few ticks, `torque_off`, `close` | no id powered; one `ForceTorqueOff` per actuator; `ClosePorts` last |
| `a_bring_up_error_after_open_leaves_no_id_powered` | all `already_on`, ping fails on id 2 | `torque_off`, `close` reach every id |
| `post_init_writes_torque_off_on_a_read_only_actuator` | actuator 3 `ReadOnly`, `already_on` | after `torque_on`, id 4 off, ids 1..=3 on |
| `a_read_only_actuator_left_powered_comes_up_off` | replaces `a_read_only_actuator_left_powered_fails_configure` | `configure` is `Ok`, warns; id off after `torque_on` |
| `torque_off_before_open_sends_nothing` | `open` fails | no `ForceTorqueOff`, `ClosePorts` once |
| `torque_off_twice_is_harmless` | after shutdown, call again | no panic, no error |
