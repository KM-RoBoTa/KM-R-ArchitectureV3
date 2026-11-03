# kmr

A Rust framework for writing real-time robot controllers.

You write plain functions and say *when* they run; the engine owns the control
loop, state, timing and hardware. The model is borrowed from
[Bevy](https://bevyengine.org/)'s ECS — functions on schedules — without the
query system: a robot has one fixed-shape state, not an open world of
entities.

> **Status: early development.** The control loop runs, simulated or on
> Dynamixel actuators; the hardware layer is being brought up on the bench.
> Expect breaking changes. See the [roadmap](docs/ROADMAP.md).

Name the robot once, register functions on schedules, run:

```rust
use kmr_api::{EachTick, Q, Sensors, Time, UserError};
use std::time::Duration;

kmr_api::robot!(XmBench);

fn nudge_first_joint(_time: &Time, robot: &mut State, _sensors: &Sensors) {
    if let Ok(q) = robot.q_at(0) {
        let _ = robot.set_q_at(q + Q(0.01), 0);
    }
}

fn main() -> Result<(), UserError> {
    Robot::new()
        .set_dt(Duration::from_millis(1))
        .add_controller(EachTick, nudge_first_joint)
        .run()
}
```

`robot!` takes the actuator count and limits from a preset or a robot
descriptor at build time. Without a bus plugin `run()` simulates. The
reference example, how to drive the bench with it and what to watch out for
are in [`controllers/README.md`](controllers/README.md).

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
| `kmr_core/`             | The engine (`kmr_core` crate).                                 |
| `kmr_api/`              | Public user API (`kmr_api` crate).                             |
| `kmr_adapters/`         | Hardware adapters, Dynamixel first.                            |
| `kmr_model/`            | Robot model: reads a descriptor at build time (proc macro).    |
| `kmr_ffi/`              | C, C++, Python and Lua controller math (in progress).          |
| `bench/`                | The reference benchmark of the engine.                         |
| `controllers/`          | Write your controllers here. Depends on `kmr_api` only. See its README. |
| `docs/`                 | Architecture, roadmap and product brief.                       |

## Building

Requires a stable Rust toolchain with edition 2024. The repository is a single
Cargo workspace; run everything from the root.

```sh
cargo build --workspace
cargo test --workspace --no-fail-fast
cargo run -p kmr_api --example complete_api_example
```

## Contributing

This project is built spec-first, with agents writing most of the code.
Read [`CONTRIBUTING.md`](CONTRIBUTING.md) before opening a PR. AI agents
start at [`AGENTS.md`](AGENTS.md).

## License

**Temporary.** The project is currently licensed under the
[MIT License](LICENSE) as a placeholder while the
maintainers choose a final license. Expect this to change.
