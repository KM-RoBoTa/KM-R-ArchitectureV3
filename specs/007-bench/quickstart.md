# Quickstart: validate the bench

**Plan**: [plan.md](plan.md) | Commands: [contracts/commands.md](contracts/commands.md) | File
format: [contracts/results-schema.md](contracts/results-schema.md)

Scenarios 1 to 6 run on any Linux dev host and prove the harness. Their figures are
`not_reference` and prove nothing about kmr (SRS NFR-1). Scenarios 7 to 11 need the reference
bench. Phase letters refer to plan.md "Order of work".

## Prerequisites

- Rust 1.97, the workspace building (`cargo build --workspace`), `bench` in the root
  `Cargo.toml` members.
- `RUST_LOG` unset.
- Cross-builds: `rustup target add aarch64-unknown-linux-gnu armv7-unknown-linux-gnueabihf`,
  plus `aarch64-linux-gnu-gcc` and `arm-linux-gnueabihf-gcc`.
- Reference bench: Raspberry Pi 4 rev 1.4 or 1.5, Arch Linux ARM, a 6.14+ (6.x) kernel and its
  PREEMPT_RT twin on the image, `stress-ng`, root, a USB power meter, a room thermometer.
  Hardware runs: the XmBench chain and a U2D2 (assumed, pending human: D10).

## 1. Builds clean under the heap ban (phase A; FR-018 (SRS NFR-1))

```sh
cargo build -p kmr_bench --release --bins
cargo clippy -p kmr_bench --all-targets
cargo test -p kmr_bench
```

Expected: no clippy warning from `kmr_bench`. Tests pass: histogram edges, slot replay against
the engine's rule, plan parsing, results parser, compare refusal. Research R21 P1 proved the
first two on a scratch copy.

## 2. One simulated run (phase A; FR-006, FR-008, FR-012, FR-020)

```sh
KMR_BENCH_OUT=run.toml KMR_BENCH_TICKS=6000 target/release/tick > run.log 2>&1
grep -c "no bus plugin: simulating" run.log
```

Expected: `status = "complete"`; `flags` holds `simulated` and `not_reference`; jitter keys
present; `loop_thread_allocs = 0`; the grep prints `1`; `run.toml` holds no kmr log line.
Scratch result: research R21 P2.

## 3. The allocation counter fires (phase A; FR-012)

Run the test build that allocates once inside the window. Expected: `loop_thread_allocs = 1`,
flag `alloc_seen`, status still `complete` (research R21 P4).

## 4. Early end and errors (phase A; FR-024 (SRS NFR-2))

```sh
KMR_BENCH_OUT=a.toml KMR_BENCH_TICKS=600000 timeout -s INT 3 target/release/tick   # Ctrl-C
KMR_BENCH_OUT=b.toml KMR_BENCH_DT_NS=5000 target/release/tick                      # below the floor
KMR_BENCH_OUT=c.toml KMR_BENCH_TICKS=abc target/release/tick                       # bad plan
```

Expected: `a.toml` `incomplete`, `ticks_done` < 600000, no `[figures]`. `b.toml` `failed` with the
`run()` error as `reason`, once the FR-3.1 range check exists (kmr_api A6, core M3); until then
it is `complete` with `dt_ns = 5000` (research R21 P4). `c.toml` `failed`,
`reason = "bad plan: KMR_BENCH_TICKS"`, exit 1.

## 5. Compare refuses a mismatch (phase A; FR-022)

Make two report directories that differ in `robot_n` and run
`target/release/results compare A B`. Expected: exit 2, `refused: robot_n differs: 4 vs 5`.
Two directories from the same host and robot: exit 0, one line per figure, no verdict.

## 6. Compile-time sweep on the dev host (phase A; FR-013, FR-014 (SRS NFR-11))

```sh
bench/scripts/compile-time.sh --host-label dev-x86
git status --short
```

Expected: one report under `bench/reports/<commit>/dev-x86/` with clean, body-edit and
registration-edit figures for `kmr_core` and `controllers`, dev and release, and the 1, 16, 64,
123 sweep. `git status` shows only the new report. Growth for reference: research R21 P9.

## 7. Host check on the Pi (phase A; FR-001)

```sh
sudo -E bench/scripts/setup-check.sh
```

Expected: revision `…3114` or `…3115`, kernel 6.x ≥ 6.14, governor `performance`, the four
operator fields set. Exits non-zero and names the field if anything is off.

## 8. Full set (a) to (e), both kernels (phase A; SC-001, SC-004)

```sh
sudo -E bench/scripts/all.sh          # non-RT kernel; prompts to reboot
sudo -E bench/scripts/all.sh          # PREEMPT_RT kernel
target/release/results summary bench/reports/<commit>/<host>
```

Expected: about 40 minutes per kernel. Every report `complete` with every `[meta]` key. `summary`
shows the four 1 kHz cells side by side, the ceiling, the budget and the footprint. A person who
did not write the bench finishes within 2 hours of a flashed board carrying both kernels.

## 9. Repeatability (phase A; SC-002, SC-003)

Run `cell.sh idle` and `budget.sh` three times each on one kernel. Expected: p50 and p99 within
10 % of each other; overhead within 10 % or 2 µs.

## 10. Engine cross-check (phase B; FR-019 (SRS FR-3.3), SC-005)

Repeat scenario 2 after core #14 and kmr_api A1. Expected: `time_surface = "engine"`,
`engine_overruns` present and equal to `bench_overruns`, no `inconsistent` flag. Repeat at
`dt` = 10 µs, where many ticks overrun: still equal.

## 11. Hardware run (phase C; FR-015, FR-017)

Set `SERIAL` in `bench/src/bin/tick_hw.rs`, then:

```sh
cargo build -p kmr_bench --release --features hardware --bin tick_hw
sudo -E bench/scripts/budget.sh --hw --write all
sudo -E bench/scripts/presort.sh --gain        # phase D, D11 B
```

Expected: before delivery step 1, `simulated` and no I/O figure (FR-006). After it: write-tick
I/O cost next to engine overhead, read ticks apart with their own overrun count, and every
actuator reads `Torque_Enable = 0` after the run (coordination §3 step 11). On XmBench the
pre-sort gain is expected to be zero (research R15).
