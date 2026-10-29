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

### `DxlRs` side (step A4, blocked)

Needs one `dxl_rs` addition, HUMAN (critical path step 1): `force_torque_off(&self)` on the
`TorqueOff` type-state (or a free function by port, id and protocol), writing `Torque_Enable` = 0
with `write1ByteTxRx1` / `write1ByteTxRx2` at the model's address (24 on AX, 64 on X). The
adapter call site then removes the actuator's `Powered` entry if there is one.

Until then (shipped with A3): `DxlRs::force_torque_off` calls today's `unpower`. It cuts an
actuator this backend powered and returns `Err(Io)`, with one warning, for any other. The
`dynamixel` build keeps compiling; the bench check of critical path step 11 would catch the gap.
The strict fake models `force_torque_off` as the SDK will have it after A4, not this fallback:
the tests pin the target behaviour.

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
