# Quickstart: validating kmr_api

Run from the git root. Each block names the phase of plan.md it validates and what must hold.
Contracts: [builder](contracts/builder.md), [robot! hardware](contracts/robot-hardware.md),
[handles](contracts/handles.md), [Zenoh](contracts/zenoh.md).

## Every change

```sh
cargo build --workspace
cargo test --workspace --no-fail-fast
cargo clippy --workspace --all-targets
cargo fmt --all --check
cargo run -p kmr_api --example complete_api_example      # simulates, exits Ok (SC-001)
cargo build -p controllers                                # SC-001, second half
```

## Phase A: the first motor (critical path step 9)

Prerequisites: HUMAN core commits 1 to 8 of `specs/002-kmr-core/design-first-motor.md`;
kmr_adapters A, C1, C2; `controllers/src/main.rs` already on `robot!(XmBench)`.

```sh
cargo test -p kmr_api --test run                          # feature off
cargo test -p kmr_api --test run --features dynamixel     # needs dxl_rs + libclang
```

Expected:

| Scenario | Result |
| --- | --- |
| no plugin, controller calls `shutdown!` at tick 3 | `Ok(())`; the controller read `tick()` 0..=3 and `dt()` = 1 ms; the log has the core's "no bus plugin: simulating" once and nothing from kmr_api (SC-007) |
| plugin, feature off | `Err(FeatureOff { feature: "dynamixel" })`, no port opened |
| plugin, test `Spec` over a recording adapter | one `open`, one write per tick, one `torque_off`, `Ok(())` |
| `Bus::by_id` of a serial not on this machine | `Err(BringUp { cause })` naming the serial; `torque_off` and `close` ran (US2-AS4) |
| two plugins with one port | `Err(DuplicateBus { port })` before any I/O (US2-AS2) |
| `dt` = 5 µs, or zero | `Err(DtOutOfRange)` before any I/O (US1-AS9) |
| `Q::from(Deg(180.0))`, `Q::from(M(1.0))`, `Q::from(Cm(1.0))` | π, 1000, 10 within f32 precision (SC-009) |
| `fn main() -> Result<(), UserError>` returning `BringUp` | the printed line is the `Display` text |

Bench (HUMAN, critical path step 11), XmBench plugged in:

```sh
cargo run -p controllers --features hardware              # with the Bus::by_id line enabled
```

The motors swing, `shutdown!` ends the run, `Torque_Enable` reads 0 on ids 1 to 4.

Cross-compile with no robot (SC-007, SRS NFR-5):

```sh
cargo build -p controllers --features hardware --target aarch64-unknown-linux-gnu
cargo build -p controllers --features hardware --target armv7-unknown-linux-gnueabihf
```

The scratch prototype of this phase reruns with:

```sh
cd /tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/kmr_api-plan/proto
CARGO_TARGET_DIR=../target cargo test --offline -p user --test run                 # 6 passed
CARGO_TARGET_DIR=../target cargo test --offline -p user --features hw --test run   # 6 passed
CARGO_TARGET_DIR=../target cargo check --offline -p user --example count           # E0308
CARGO_TARGET_DIR=../target cargo check --offline -p user --example badidx          # one E0080
```

## Phase B: `robot!` and payloads

```sh
cargo test -p kmr_api --test robot_macro
cargo test -p kmr_api --test ui                           # trybuild snapshots (003 R10)
```

Expected: N, order, limits and links per specs/003-kmr-model/quickstart.md; plus

| Case | Result |
| --- | --- |
| `robot!(XmBench, hardware = \|hw\| hw.read_only(3))` | builds; actuator 3 read-only in the table; unpowered after bring-up, still read (US2-AS5) |
| a preset table of 3 rows for N = 4 | E0308 naming 4 and 3 (SC-006) |
| an override index past N | one E0080 naming the index |
| N = 65 | "SRS write mask: more than 64 actuators" under `cargo check` |
| `robot!(model = ..)` without hardware, with a plugin | `Err(NoHardwareDeclared)` |
| a call to `go_home(90)` | E0277 "kmr: SRS FR-2.7: home state is open" (D5 A) |
| `register_payload(Payload::medium(1.2, "nope"))` | `Err(UnknownLink { link: "nope" })` before any I/O (US7-AS2) |

After the core's const generic N: the `three.urdf` case builds and runs (design step 8).

## Phase C: faults, age, adapter access

On a scripted host and adapter (kmr_core test hooks):

| Case | Result |
| --- | --- |
| read of actuator 2 fails | next call: `stale_at(2)` only (US3 independent test) |
| a tick without a read | `sample_age()` is one more than the tick before (US3-AS3) |
| no-hardware run | `age_at(i)` is `Ok(None)` and `sample_age()` is `None` (never read) |
| a limited actuator without a policy (D2 B, assumed) | `run()` returns `NoPolicy { actuator }`, no I/O; with `set_limit_policy_all(..)` it runs |
| a tick overran | `overrun_count()` +1, `last_overrun()` is the lateness (US3-AS2) |
| `bind()?.drivers()` then `run()` | the drivers come back after the run, closed (FR-041) |
| builder with no controller | does not compile (US1-AS10, after core #15) |

## Phase Z: Zenoh

```sh
cargo tree -p kmr_api -e normal | grep -c zenoh                        # 0 (SC-004)
cargo tree -p kmr_api -e normal --features zenoh | grep -c zenoh       # > 0
cargo test -p kmr_api --features zenoh --test zenoh
```

Local peer, 1 kHz, 10 000 ticks: one sample per tick received or counted missed; a Desired
taken in tick k applied in k, never in k+1 (SC-010); no Desired, no command (US4-AS3).

Slot prototype (R15):

```sh
cd /tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/kmr_api-plan/slot
CARGO_TARGET_DIR=../target cargo test --offline --release -- --nocapture   # 2 passed, "allocs during reads 0"
```

Bench (SC-003, `bench` module): 1 000 000 ticks with the gateway on, 0 allocations and 0 locks
from kmr_api inside the tick.

## Phase E: fallbacks and envelopes

| Case | Result |
| --- | --- |
| fallback on `stale_in(&A)`, read fails on group A | the fallback runs in place of the controller for that tick (US5-AS1) |
| envelope on actuator 0, controller writes outside it | the adapter sees the value clamped to the envelope bound, never the written one; actuator 0's `envelope` bit is set (per actuator; SC-013; D3 A, assumed, pending human) |
| envelope on a group slice | every actuator of the slice gets it (US5-AS2) |

## Phase F: security

| Case | Result |
| --- | --- |
| 1000 Desired from an unknown identity | 0 applied, `unauthorized` = 1000, flag set (SC-011) |
| Desired with an old `seq` | dropped and counted (SC-012) |
| source silent past its timeout | flag within timeout + 1 tick; its fallback runs (US6-AS2) |
| `add_remote_source` without a fallback | does not compile (US6-AS3) |

## Phase G: FFI feature

```sh
cargo tree -p kmr_api -e normal | grep -c kmr_ffi                     # 0
cargo test -p kmr_api --features ffi
```

Glue tests per specs/006-kmr-ffi/plan.md C4.
