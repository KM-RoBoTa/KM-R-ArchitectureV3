# controllers: your first moving robot

This crate is where controllers are written. It depends on `kmr_api` only.
`src/main.rs` is the reference: copy it to start your own.

> Status: `src/main.rs` still holds the placeholder `nudge_first_joint`. The
> swing described below lands with controllers T006 to T011; until then the
> commands work, the motion does not match the numbers here.

## What the reference does

On the XmBench preset (four Dynamixel XM430 actuators), one controller on
`EachTick`:

- reads where each actuator stands at the first sample, and keeps it as the
  start;
- swings every actuator ±5° around its start, one full swing every 4 s, on
  every tick at `dt` = 1 ms;
- after 12 s of `Time::elapsed()`, raises `shutdown!("reference demo: time is
  up")`. The engine finishes that tick, holds, then turns torque off right
  after the hold. `run()` returns `Ok(())`.

Ctrl-C, `kill -TERM` and `kill -HUP` end it the same way, at any time.

## Prerequisites

- A Rust toolchain (edition 2024) and a fresh checkout of this repository.
- For the bench: the XmBench wired and powered, and the serial of its USB
  adapter (U2D2), from `ls /dev/serial/by-id`.

## Simulate

```sh
cargo run -p controllers
```

No file written, no line edited. Without a bus plugin `run()` simulates and
says so once: `no bus plugin: simulating`.

## Drive the bench

Files written: none for XmBench. Lines edited: one. In `src/main.rs`, add the
bus line before `.add_controller_with(..)`:

```rust
.add_plugins(Bus::by_id("<U2D2 serial>"))
```

No `.bus("label")`: XmBench has one bus. Then:

```sh
cargo run -p controllers --features hardware --release
```

The `hardware` feature pulls in the Dynamixel SDK; without it the crate builds
anywhere and simulates.

## Another robot

One more line: the preset name in `kmr_api::robot!(XmBench);`. A copied crate
writes two files, its `Cargo.toml` (one dependency, `kmr_api`) and its
`src/main.rs`.

## Before you change the numbers

- **Revolute only.** `Q` is radians on a revolute actuator and millimetres on
  a prismatic one: ±5° means nothing on a slider.
- **Position mode only.** The swing writes `Q`; an actuator in another mode
  gets no command.
- **A robot that would fall needs a hold duration.** The reference has none:
  torque goes off right after the shutdown tick. Add `.set_hold(..)` for an
  arm that cannot hold itself.
- **A foreign Ctrl-C handler.** If something else in the process installs its
  own handler first, Ctrl-C may not stop the loop (kmr warns at start). The
  demo still ends after 12 s.

## Baseline

The onboarding claim of SRS FR-10.4 is measured against ROS2 (assumed,
pending human: D14). This section is the one place that says how.

- **The baseline:** a minimal node made with `ros2 pkg create`, once in C++
  and once in Python, on the current ROS2 LTS, driving the same four
  actuators through the vendor's ROS2 SDK package.
- **What counts:** only files the user writes or edits count, for both sides.
  Generated files do not.
- **The bar:** kmr beats both variants on time and on file count.

### Trial record

One row per trial, kept with this README. Elapsed is from a fresh checkout
to the first motion.

| side | robot | elapsed (min) | files written | lines edited | commands run |
|---|---|---|---|---|---|
| kmr | XmBench | | | | |
| ROS2 C++ | XmBench | | | | |
| ROS2 Python | XmBench | | | | |
