# Quickstart: validating kmr_core, milestone by milestone

Run from the git root. Rust 1.97.1 stable. No hardware unless a step says "bench". Every test
named here is written by the HUMAN with the code it proves ([plan.md](plan.md)).

## Always

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets      # no new warning in kmr_core
cargo test --workspace --no-fail-fast       # 191 passed today; 0 failed at every milestone
```

## M1 First motor

```sh
cargo test -p kmr_core exit_paths           # recording fake adapter, scripted host
cargo test -p kmr_core state::test          # write mask, command, slot push, hold target
cargo test -p kmr_core --test termination   # SIGTERM then SIGHUP end a run as a shutdown
cargo test -p kmr_core clock                # tick and overrun count
```

Expected: the 28 tests of design-first-motor §4 pass (workspace 219 passed in the prototype),
plus `an_estop_in_post_init_never_powers` (K1.9). Exit-path records match
[contracts/hal.md](contracts/hal.md) "Call order". A `NoHardware` run logs "no bus plugin:
simulating" exactly once, at `open` (K1.6).

Then (adapters D1, agent): `cargo test -p kmr_adapters dynamixel::tests::engine` passes 4 tests.

**Bench (HUMAN, critical path step 11)**: XmBench, `cargo run -p controllers` with
`robot!(XmBench)` and `Bus::by_id`. The swing runs, `shutdown!` ends it. Read `Torque_Enable` on
ids 1 to 4: all 0. Repeat with an `emergency_stop!` and with Ctrl-C, `kill -TERM`, `kill -HUP`.

## M2 N from the robot line

```sh
cargo test --workspace                      # green after every step K2.1..K2.10
cargo test -p kmr_api robot_macro           # N = 1, 4, 10, 64 build and run on NoHardware
grep -rn "JOINTS" kmr_core/src              # no match after K2.10
```

Expected: a 65-actuator descriptor fails under `cargo check` with
"SRS write mask: more than 64 actuators" (SC-009 (SRS FR-6.1)). `Robot::<65>::new()` without
`robot!` fails under `cargo build` (backstop).

## M3 State transition

- `dt` of 0, 5 µs, 2 s: `run_on` returns `RunError::Config`; the recorder logs 0 I/O calls
  (SC-004 (SRS FR-3.1, FR-5.5)).
- An empty robot: a compile-fail doctest expects `kmr: this robot has no controller` (SC-008).
- No-hardware run: every History read returns `NoSample`; one "simulating" warning (FR-042).
- Age: with cadence 3, actuator age reads 0, 1, 2, 0 over four ticks (FR-018 (SRS FR-2.1)).
- `grep -rn "todo!\|unimplemented!" kmr_core/src` returns nothing (D5 A).
- 10 000 scripted ticks: 0 allocations (counting global allocator in a test binary), 0 log lines
  per tick (SC-005 (SRS NFR-1)).
- `size_of::<Q>() == 4` and `align_of::<Slot<4>>() == 64` (FR-016, FR-059).

## M4 Signals, Time, faults

- Inject each fault on the recorder: its bit is set in that tick's slot, read at depth 0 then 1
  (SC-006 (SRS FR-13.4)).
- `ExternalFlags::raise` from another thread during tick k: the bit shows in slot k or k+1,
  never lost.
- Hold `Some(2 s)`: `torque_off` follows `hold` after 2 s within one `dt`; an estop at 1 s ends
  the wait at once (SC-010 (SRS FR-4.1)).

## M5 API hooks

- A `Stages` recorder sees `inbound` after `read` and before `First`, `outbound` after `write`.
- A limit with no policy: `run_on` returns `ConfigError::NoPolicy`, 0 I/O (D2 B).
- `Clamp`: the sent value equals the bound and `joint_limit` is set. `Abort`: no write that tick,
  then `torque_off`, `close`.
- A continuous actuator (`lower`, `upper` = `None`) commanded to 10 rad: sent as is, no
  `joint_limit` bit; its speed and effort bounds still apply.
- An envelope breach: clamped, and that actuator's `envelope` bit set (per actuator, D3 A).

## M6 Gates

```sh
cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf
cargo build -p kmr_core --no-default-features --target thumbv6m-none-eabi   # expected to fail: no CAS
cargo build -p kmr_core --features presort-off
```

Expected: the first builds with no `std` or `alloc` (SC-007 (SRS NFR-7)); the second fails as
research R13 says; the third keeps declaration order in `Batch`.
