# Contract: bench commands

One command per measurement (FR-021 (SRS NFR-8)). Scripts run from the repository root on the
bench host. They need `bash`, coreutils, util-linux (`chrt`, `taskset`), binutils (`size`,
`strip`), `stress-ng`, and root for `chrt`, the governor and `sched_rt_runtime_us`. Settings live
in `bench/scripts/profile.env`; every script records the ones it used.

| Command | Measures | Spec | Writes |
|---|---|---|---|
| `bench/scripts/setup-check.sh` | host against SRS NFR-1; stops on a missing revision source or a missing operator field | FR-001, FR-024 | stdout only |
| `bench/scripts/cell.sh <idle\|load>` | 1 kHz cell on the running kernel | FR-008, FR-009 (SRS NFR-8 b, c) | one report |
| `bench/scripts/ceiling.sh` | tick-rate ceiling, floor check, gap at the ceiling | FR-007 (SRS NFR-8 a, FR-3.1) | one report per candidate, one summary report |
| `bench/scripts/budget.sh [--hw] [--write all]` | controller budget, overhead, gap at the floor; I/O cost with `--hw` | FR-010, FR-015 (SRS NFR-8 d, FR-5.3) | one report per candidate, one summary report |
| `bench/scripts/footprint.sh` | size, sections, peak RSS, stack | FR-011 (SRS NFR-8 e) | one report |
| `bench/scripts/compile-time.sh [--host-label <l>]` | clean and incremental, dev and release; registration sweep | FR-013, FR-014 (SRS NFR-11) | one report |
| `bench/scripts/presort.sh [--gain]` | pre-sort cost (simulated); gain on hardware with `--gain` | FR-016 (SRS FR-5.3) | reports and one `compare` |
| `bench/scripts/all.sh` | (a) to (e) on the running kernel, then the reboot prompt | SC-001 | all of the above |

Operator fields, set per run in the environment (research R12): `BENCH_SUPPLY_MV`,
`BENCH_SUPPLY_METER`, `BENCH_AMBIENT_MC`, `BENCH_THERMOMETER`. A missing one stops
`setup-check.sh` and fails any run that starts anyway.

## Binaries (driven by the scripts; usable by hand)

```text
cargo build -p kmr_bench --release --bins                          # tick, minimal, baseline, results
cargo build -p kmr_bench --release --features hardware --bin tick_hw
cargo build -p kmr_bench --release --features presort-off          # D11 B
cargo build -p kmr_bench --release --features zenoh                # FR-026, once kmr_api has it
cargo build -p kmr_bench --release --features ffi --bin tick_ffi   # kmr_ffi SC-003, once kmr_ffi exists
```

Run plan through the environment (see [data-model.md](../data-model.md) "RunPlan"):

| Variable | Unit | Default |
|---|---|---|
| `KMR_BENCH_OUT` | path | required |
| `KMR_BENCH_DT_NS` | ns | 1000000 |
| `KMR_BENCH_TICKS` | ticks | 601000 |
| `KMR_BENCH_WARMUP` | ticks | 1000 |
| `KMR_BENCH_WORK_NS` | ns | 0 |
| `KMR_BENCH_WRITE` | `none` \| `all` | `none` |
| `KMR_BENCH_MEASUREMENT`, `KMR_BENCH_CELL` | text | `jitter`, `adhoc` |

The binary writes `[run]` and `[figures]` to `KMR_BENCH_OUT` and nothing to stdout. Its stdout
and stderr are kmr's (tracing goes to stdout, research R3). The script captures both into
`<report>.log` and counts the simulation warning there, with the pattern from `profile.env`.
`RUST_LOG` must be unset (research R12).

Exit codes: 0 for `complete` and `incomplete`, 1 for `failed`. A binary never judges a figure.

## Hardware source line

`bench/src/bin/tick_hw.rs` holds the one line a person with another adapter changes:

```rust
const SERIAL: &str = "FT8ISZ6H"; // the U2D2's serial id, from /dev/serial/by-id
```

Nothing else differs between two reference benches (User Story 2 scenario 3).
