# kmr — Robot Control API

A Rust framework for writing real-time robot controllers. The public surface
(`kmr_api`) is small and deliberate: you describe *what* runs and *when*, and
the engine (`kmr_core`) owns the *how*.

The design borrows the mental model of [Bevy](https://bevyengine.org/)'s ECS —
plain functions registered against schedules — but deliberately stops short of
its general-purpose query system. We don't need that much modularity, and the
complexity isn't worth it for a fixed-shape robot state.

---

## Design Principles

### 1. State is private

The robot's internal state — joint positions (`q`), velocities (`qd`), torques
(`tau`), and history — is **not** a field you reach into. There is no
`robot.q[0] = 1.0`. The fields are private and the engine guarantees this at
compile time:

```rust
// This does not compile — `desired` is private:
robot.desired.q[0] = 1.0;
```

State is an invariant the engine maintains (units, ranges, history depth,
write-once-per-tick semantics). Exposing the raw fields would make those
invariants impossible to enforce.

### 2. Access is through setters and getters only

Every interaction with state goes through a method. Reads and writes are
separate, explicit, and fallible where they can fail:

```rust
fn controller(mut robot: RobotState) {
    let p = robot.position_at(0)?;   // getter — read sensed position, returns Q
    robot.set_q_at(Q(1.0), 0)?;      // setter — request a desired position
}
```

- **Getters** (`position`, `velocity`, `torque`, `*_at`, `prev_q_at`) read the
  sensed/historical state. Read-only.
- **Setters** (`set_q_at`, `set_all`) write the *desired* state — your command
  to the actuators, not a mutation of reality.

You always speak in the typed field vocabulary — `Q` (position), `Qd`
(velocity), `Tau` (torque) — never bare `f32`. The newtypes carry intent: the
type system makes mixing a position and a velocity a compile error. These types
are part of the public vocabulary and are re-exposed deliberately (see §3) — it
is the engine's *internal state storage*, not the field types, that stays
private.

### 3. The core is sealed — the API re-exposes, it never hands out direct access

`kmr_core` is highly private. The application is shipped closed-source, so the
core's internals must never leak through the public crate. `kmr_api` is a thin
vendor boundary that **re-exposes** capabilities rather than forwarding raw
core handles.

Concretely:

- `kmr_api::RobotState` is a newtype wrapping `kmr_core::RobotState`. The inner
  value is private; only the methods we choose to mirror exist.
- Builder steps like `.add_sensor(...)` are re-exposures: the user names a
  sensor, the engine wires it up internally. The user never touches the core
  sensor registry, the runtime, or the control loop.
- The set of writable state fields is **sealed** (via a private `Slot` trait in
  a private module). External crates cannot add new field types or implement
  the storage routing — the field set is fixed.
- The field vocabulary (`Q`, `Qd`, `Tau`) is **owned by `kmr_api`**, not leaked
  from the core. `kmr_api` defines its own `Q`/`Qd`/`Tau`; the core keeps its
  internal copies; a sealed `FieldConv` trait converts between them at the
  boundary. So `kmr_core` never appears in a public signature, yet the user
  still speaks the precise typed vocabulary. This is a re-exposure, not a leak.

Adding a capability to the public API is a deliberate act: we write the
re-exposure. Nothing is exposed by default.

### 4. Bevy-inspired schedules, without the query system

You write controllers as **plain functions** and register them against
**schedules** that say when they run. No traits to implement at the call site,
no query DSL to learn.

```rust
fn main() {
    Robot::new()
        .dt_ms(Duration::from_millis(1))
        .add_sensor(Imu::default())
        .add_controller(Startup,     home_position)
        .add_controller(Runtime,     replace_all_elements)
        .add_controller(OnCollision, (pause, previous_state, stop))
        .run();
}
```

Schedules (the analogue of Bevy's `Update`, `Startup`, …):

| Schedule      | When it runs                          |
|---------------|---------------------------------------|
| `Startup`     | Once, before the control loop begins  |
| `Runtime`     | Every tick (`dt`)                      |
| `OnCollision` | When a collision event fires          |

A controller is just `fn(RobotState) -> ...`. A schedule can take one controller
or a tuple of them, which run as a group. State flows in through `RobotState`;
that's the only context a controller needs — no entity queries, no component
filters, no archetypes.

---

## Why this shape

- **Safety by construction.** Private state + sealed fields means whole classes
  of misuse (out-of-bounds field writes, unit mix-ups, history corruption) are
  compile errors, not runtime surprises.
- **Closed-source friendly.** The re-exposure boundary lets us ship the engine
  as a binary/closed crate while giving users a stable, minimal API.
- **Low cognitive load.** Functions + schedules is a model people already know
  from Bevy. We drop the ECS query machinery because a robot has one fixed
  state, not an open world of entities.

---

## Layout

| Crate / module        | Role                                                        |
|-----------------------|-------------------------------------------------------------|
| `kmr_api`             | Public, stable, minimal API. Vendor boundary. Re-exposures. |
| `kmr_core`            | Private engine: state, runtime, control loop, sealed traits.|
| `kmr_ffi`             | FFI bindings.                                               |

See `src/kmr_api/examples/complete_api_example.rs` for the target API in full.
