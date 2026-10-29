# Contract: Python and Lua bridges

The script side of FR-017 and FR-018 (SRS FR-10.1, FR-10.3). Not prototyped yet
([research.md](../research.md) R8). Same data as the C ABI ([c-abi.md](c-abi.md)), same rules.

## Rust side

```rust
// main.rs, before run()
let py = kmr_api::ffi::python::Bridge::load("ctrl/swing.py", "step")?;   // Err(ForeignError::Load)
let lua = kmr_api::ffi::lua::Bridge::load("ctrl/swing.lua", "step")?;

robot.add_controller_with(EachTick, py, |py, t, s, _| {
    if kmr_api::ffi::call(&mut *py, t, s, None::<&mut ()>).is_err() { kmr_api::emergency_stop!(); }
});
```

- `load` starts the interpreter, runs the file and looks up the function. Every failure is a
  returned `ForeignError::Load` (FR-020 (SRS FR-6.4)). Nothing panics.
- The bridge is the controller's context (SRS FR-1.1). A captured bridge is allowed, not advised.
- Heap is used in `load` and inside the interpreter; that cost is the user's (FR-005, FR-019).

## Script side

The function is called once per tick with three arguments.

```python
def step(read, write, time):
    if read.samples == 0:
        return 0
    for i in (0, 2):
        write.set_q(i, read.q[0][i] + 0.1)     # read.q[depth][actuator]
    if read.stale[0] & 0b10:
        return 2                               # emergency stop
    return 0
```

```lua
function step(read, write, time)
  if read.samples == 0 then return 0 end
  write.set_q(1, read.q[1][1] + 0.1)           -- 1-based: depth 0 is [1], actuator 0 is [1]
  return 0
end
```

| Name | Python | Lua |
|---|---|---|
| `read.<field>[depth][i]` | float, 0-based | number, 1-based |
| `read.age[i]` (latest slot only) | int ticks, `None` = never read | integer ticks, `-1` = never read (no hole in the table) |
| `read.stale[depth]`, other flags | int mask, bit i = actuator i | integer mask, bit i = actuator i (0-based bits) |
| `read.samples` | int | integer |
| `write.set_<field>(i, v)` | 0-based | 1-based |
| `time.tick`, `time.dt_ns`, ... | int | integer (64-bit) |
| return | `0`/`None`, `1`, `2` | `0`/`nil`, `1`, `2` |

Python ints and Lua 5.4 integers are 64-bit or wider, so ticks and ns stay exact (FR-009, SC-006).

## Errors

| Event | Result |
|---|---|
| Python exception, Lua error | `ForeignError::Python(text)` / `Lua(text)`; nothing applied (FR-012) |
| Wrong return type or value | `ForeignError::Code(-1)`; nothing applied |
| Index out of range on `write` | script error (Python `IndexError`, Lua error); nothing applied |

## Forms

| Form | Works | Note |
|---|---|---|
| inline | yes, with a captured bridge | not advised (SRS FR-1.1 wants mutable state in a context) |
| inline with context | yes | recommended |
| threaded | once the hand-off exists (SRS FR-1.2) | Lua needs the `mlua` `send` feature; no `State`, so no `call`; the script gets `time` and its own data |
