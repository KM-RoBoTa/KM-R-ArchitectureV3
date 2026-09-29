# kmr

A Rust framework for writing real-time robot controllers.

You write plain functions and say *when* they run; the engine owns the control
loop, state, timing and hardware. The model is borrowed from
[Bevy](https://bevyengine.org/)'s ECS — functions on schedules — without the
query system: a robot has one fixed-shape state, not an open world of
entities.

> **Status: early development.** The public API is taking shape; the runtime
> control loop is not implemented yet. Expect breaking changes. See the
> [roadmap](kmr_v3/docs/ROADMAP.md).

```rust
use kmr_api::{EachTick, Init, Q, Robot, Sensors, State, Time};
use std::time::Duration;

fn home(_t: &Time, robot: &mut State, _s: &Sensors) {
    robot.go_home(90);
}

fn nudge_first_joint(_t: &Time, robot: &mut State, _s: &Sensors) {
    if let Ok(q) = robot.q_at(0) {
        let _ = robot.set_q_at(q + Q(0.01), 0);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_controller(Init, home)
        .add_controller(EachTick, nudge_first_joint)
        .run()?;
    Ok(())
}
```

## Why

- **Safety by construction.** State is private and typed (`Q`, `Qd`, `Tau`);
  field writes out of bounds, unit mix-ups and history corruption are compile
  errors or explicit `Result`s, not runtime surprises.
- **Real-time by default.** No heap and no `dyn` on the control path;
  controllers are statically dispatched.
- **Low cognitive load.** Functions and schedules. No traits to implement, no
  query DSL.

## Repository layout

| Path                    | What                                                           |
|-------------------------|----------------------------------------------------------------|
| `kmr_api/`              | Public user API (`kmr_api` crate). Start here as a user.       |
| `kmr_v3/src/kmr_core/`  | The engine (`kmr_core` crate).                                 |
| `docs/ARCHITECTURE.md`  | Design principles and how the engine works.                    |
| `kmr_v3/docs/`          | Roadmap and product brief.                                     |
| `scripts/`              | Dev/prod manifest switching for `kmr_api`.                     |

## Building

Requires a stable Rust toolchain with edition 2024.

```sh
cd kmr_v3 && cargo build && cargo test
cd ../kmr_api && cargo run --example complete_api_example
```

## Contributing

This project is hand-engineered; AI assistance is tolerated under strict
rules. Read [`CONTRIBUTING.md`](CONTRIBUTING.md) before opening a PR. AI agents
start at [`AGENTS.md`](AGENTS.md).

## License

**Temporary.** The project is currently licensed under the
[MIT License](LICENSE) as a placeholder while the
maintainers choose a final license. Expect this to change.
