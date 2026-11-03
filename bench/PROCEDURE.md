# Bench procedure

How to produce every kmr reference figure from this file alone (FR-021 (SRS NFR-8)): the OS
setup, the kernel configuration of both kernels, governor, scheduling, load profile, instruments
and run lengths are all below, one command per measurement. Settings
live in `bench/scripts/profile.env`; commands run from the repository root. Reports land under
`bench/reports/<commit12>/<host_id>/`; a human commits them.

Figures count only from the reference bench (SRS NFR-1). A run anywhere else is labelled
`not_reference` and proves the harness, not kmr.

## Checklist

For a person who did not write the bench, from a flashed board carrying both kernels. About
80 min plus one reboot (research R18); SC-001 asks for it within 2 hours.

1. [ ] Board, image and kernels as in [Host setup](#host-setup); `stress-ng`, `chrt`, `taskset`,
   `size`, `strip` installed.
2. [ ] The commit to measure checked out, clean (`git status --short` empty).
3. [ ] [Build once](#build-once): `cargo build -p kmr_bench --release --bins`.
4. [ ] Boot the non-RT kernel. As root: governor `performance` on every core, `RUST_LOG` unset
   ([Per boot](#per-boot-as-root)).
5. [ ] Power meter and thermometer in place; export the four
   [operator fields](#operator-fields-and-instruments).
6. [ ] `sudo -E bench/scripts/all.sh`. It checks the host, runs (e), (b), (c), (a), (d), prints
   the summary and names the kernel to reboot into. About 40 min. A failed check stops it
   before any run: fix the named field and start again.
7. [ ] Reboot into the PREEMPT_RT kernel; repeat steps 4 to 6 with **the same binaries**.
8. [ ] `target/release/results summary bench/reports/<commit12>/<host_id>`: four 1 kHz cells,
   two ceilings, two budgets and the footprint, no `missing:` line.
9. [ ] Note whether the 10 µs floor held on each kernel (`floor_held` in the ceiling summary,
   SC-008).
10. [ ] Commit `bench/reports/<commit12>/<host_id>/` (reports and logs), nothing else.

### Timing per kernel (research R18)

| Step | Script | Length |
|---|---|---|
| (e) footprint | `footprint.sh` | under 1 min |
| (b) jitter, idle | `cell.sh idle` | 10 min |
| (c) jitter, loaded | `cell.sh load` | 10 min plus 5 s lead |
| (a) ceiling | `ceiling.sh` | about 9 candidates × 1 min |
| (d) budget | `budget.sh` | about 11 candidates × 1 min, plus 1 min at 10 µs |
| summary | `results summary` | seconds |

About 40 min per kernel; two kernels about 80 min plus one reboot. Compile time is not part of
the set (see [Compile time](#compile-time)).

### Repeatability (SC-002, SC-003)

On one kernel, run `cell.sh idle` and `budget.sh` three times each, then
`target/release/results compare` on two sets or `results summary` on one: p50 and p99 within
10 % of each other, overhead within 10 % or 2 µs. The bench prints the spread and gives no
verdict (FR-023).

## Host setup

Do this once per board. Every value below is also checked or recorded by the scripts, so a
report shows the setup it ran under.

### Board and image

- Raspberry Pi 4 Model B, board revision `…3114` or `…3115` (rev 1.4 or 1.5). `setup-check.sh`
  reads it from `/proc/device-tree/system/linux,revision`, else from `Revision` in
  `/proc/cpuinfo`, and stops if neither gives one.
- Arch Linux ARM, aarch64.
- **One image carrying both kernels.** Building a PREEMPT_RT kernel on the Pi takes hours
  (research R18), so the 2-hour target of SC-001 holds only from an image that already has both:
  a non-RT kernel and its PREEMPT_RT twin, same version, 6.14 ≤ v < 7.0. PREEMPT_RT is mainline
  since 6.12, so the twin is the same source tree with one option changed.
- Boot entries for both kernels; switching is a reboot, nothing else.

### Kernel configuration

| Option | non-RT | PREEMPT_RT |
|---|---|---|
| `CONFIG_PREEMPT_RT` | not set | `y` |
| preemption model | the distribution default (`PREEMPT` or `PREEMPT_VOLUNTARY`) | fully preemptible (`PREEMPT_RT`) |
| `CONFIG_HZ` | distribution default | same value as non-RT |
| `CONFIG_NO_HZ_FULL`, `isolcpus=`, `nohz_full=` | not set | not set |

Everything else stays identical between the two. `meta.sh` records the full `uname -r` and
`uname -v`, and `kernel_rt` (`/sys/kernel/realtime` or `PREEMPT_RT` in `uname -v`).

### Per boot, as root

```sh
for g in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do echo performance > "$g"; done
cat /proc/sys/kernel/sched_rt_runtime_us      # recorded; the default 950000 is the reference
unset RUST_LOG
```

- **Governor** `performance` on every core (`setup-check.sh` stops on anything else).
- **Scheduling**: every run starts under `chrt -f 80` (`SCHED_FIFO`, priority 80). `SCHED_FIFO`
  threads get no timer slack (research R12, P7); the binary still records
  `/proc/self/timerslack_ns`.
- **Affinity**: `taskset -c 3`, the loop on CPU 3.
- **Core isolation off**: no `isolcpus`, no `nohz_full`; the load runs on all cores, the loop's
  included (spec assumption).
- **`RUST_LOG` unset.** Any value turns kmr's logging to TRACE, which prints inside every tick
  (research R12). `setup-check.sh` stops when it is set; `[meta]` records `rust_log`.
- Settings live in `bench/scripts/profile.env` (`SCHED_PRIO`, `AFFINITY_CPU`, `GOVERNOR`). Change
  them there, never in a script.

### Operator fields and instruments

Four fields, set in the environment before every session (research R12). A missing one stops
`setup-check.sh` and fails any run that starts anyway:

| Variable | Unit | Instrument |
|---|---|---|
| `BENCH_SUPPLY_MV` | mV, integer | USB power meter between the supply and the Pi |
| `BENCH_SUPPLY_METER` | text | the meter's make and model |
| `BENCH_AMBIENT_MC` | m°C, integer | room thermometer near the board, not on it |
| `BENCH_THERMOMETER` | text | the thermometer's make and model |

```sh
export BENCH_SUPPLY_MV=5120 BENCH_SUPPLY_METER="<meter model>"
export BENCH_AMBIENT_MC=21500 BENCH_THERMOMETER="<thermometer model>"
```

Run the scripts with `sudo -E` so these survive.

### Load profile

`cell.sh load` starts, 5 s before the run (`LOAD_LEAD_S`), on all cores (research R17):

```sh
stress-ng --cpu 0 --cpu-method all --hdd 1 --hdd-bytes 256M --timeout <run + 10>s
```

and stops it after the run. The exact command lands in `[meta]` `load_command`. `LOAD_CMD`
replaces it for a one-off; a report with another command does not compare with the reference.

### Run lengths

From `profile.env` (research R18): 1 kHz cells `TICKS=601000` with `WARMUP=1000` (600 000
measured ticks, 10 min); ceiling and budget candidates held `HOLD_S=60` s each. On a dev host,
shorten them in the environment (`TICKS=6000 HOLD_S=2`); the report records what ran.

## Build once

```sh
cargo build -p kmr_bench --release --bins
```

Build on the Pi, or cross-build for `aarch64-unknown-linux-gnu`, once per commit, and run **the
same binaries on both kernels** (FR-025): never rebuild between the non-RT and the PREEMPT_RT
half. The scripts read the binaries from `BENCH_BIN` (default `target/release`) and never build;
a missing binary stops them with the build command.

## Check the host

```sh
sudo -E bench/scripts/setup-check.sh
```

One line per check; the first failure names its field and exits 1. Fix it before any run.

## 1 kHz cells (b) and (c)

```sh
sudo -E bench/scripts/cell.sh idle     # cell <nonrt|rt>-idle, measurement jitter
sudo -E bench/scripts/cell.sh load     # cell <nonrt|rt>-load, measurement load
```

Each runs `tick` once: 601 000 ticks at `dt` = 1 ms, no bus plugin (kmr simulates and warns once),
and writes one report plus its `.log`. Figures: `jitter_p50_ns`, `jitter_p99_ns`,
`jitter_max_ns` (start lateness against the replayed grid), `bench_overruns`, the gap figures,
`vm_hwm_kb`, `stack_kb`, `loop_thread_allocs`. The prefix `nonrt` or `rt` comes from the running
kernel. About 10 min each.

## Ceiling (a)

```sh
sudo -E bench/scripts/ceiling.sh
```

The fastest `dt` the loop holds on the running kernel (FR-007 (SRS NFR-8 a, FR-3.1)). One `tick`
process per candidate, each held `HOLD_S` (60 s) plus the warm-up, cell `<nonrt|rt>-dt<dt_ns>`:

1. `dt` = 10 µs (`CEIL_DT_MIN_NS`). Held: the ceiling is at least 100 kHz, `floor_held = "yes"`,
   stop. The search never goes below 10 µs, the SRS floor.
2. `dt` = 1 ms (`CEIL_DT_MAX_NS`). Not held: `floor_held = "no"` and no `ceiling_dt_ns`. That is a
   finding (the loop does not hold 1 kHz on this kernel), not a fault of the bench.
3. Otherwise a bisection on a log scale between the two, until the bracket is within 1 % of `dt`
   (`CEIL_STOP_PERMILLE`). About 9 candidates, 9 to 10 min.

A candidate holds when it is `complete` and both `bench_overruns` and `engine_overruns` are 0.
The summary report, cell `<nonrt|rt>-idle`, gives `ceiling_dt_ns`, `ceiling_hz`, `floor_held`, and
the held candidate's gap from one tick's end to the next tick's start as `gap_ceiling_min_ns`
and `gap_ceiling_p50_ns` (mostly sleep at that rate; the engine's own path is the budget's
`gap_floor_min_ns`, research R9). A failed or interrupted candidate ends the search; the summary
then carries that status and names the candidate.

## Budget and overhead (d)

```sh
sudo -E bench/scripts/budget.sh                # the probe writes nothing
sudo -E bench/scripts/budget.sh --write all    # the probe writes every actuator each tick
```

The engine's time per tick is never seen directly (black-box, FR-003). Instead an `EachTick`
probe spins a calibrated workload, and the script searches, at `dt` = 1 ms, for the largest
workload the loop holds `HOLD_S` with zero overruns (FR-010 (SRS NFR-8 d, NFR-11); research
R8):

- one `tick` process per candidate (a failed window pollutes the next one), cell
  `<nonrt|rt>-w<work_ns>`; bisection from 0 to 1 ms, to 1 µs (`BUDGET_STEP_NS`), about 11
  candidates, 11 min;
- the workload is integer arithmetic timed before and after `run()` (best of 5 × 10⁶
  iterations); a drift above 2 % flags the run `uncalibrated` (the overhead is kept, marked);
- then one run at `dt` = 10 µs, cell `<nonrt|rt>-floor-w0`: the smallest gap after a late tick is
  the engine's own path from one tick to the next, with no sleep (research R9).

The summary report, cell `<nonrt|rt>-<none|all>`, gives `budget_ns`, `overhead_ns` =
`dt − budget_ns` (`method = "budget"`), the held candidate's calibration and `probe_cost_ns` in
`[run]`, and `gap_floor_min_ns`. On a non-RT kernel the figure holds the worst wake-up of the
window, so it is an upper bound. When workload 0 already overruns, `budget_ns` and `overhead_ns`
are absent: the loop did not hold 1 kHz for `HOLD_S`.

## Footprint (e)

```sh
sudo -E bench/scripts/footprint.sh
```

Size and memory of the engine alone and of the bench binary (FR-011 (SRS NFR-8 e); research
R11), from the binaries built once. One report, `footprint-<nonrt|rt>-<n>.toml`:

| Keys | Binary |
|---|---|
| `file_bytes`, `stripped_bytes`, `text`, `data`, `bss`, `vm_hwm_kb`, `stack_kb` | `minimal`: the engine with three no-op controllers, no bench storage |
| the same with `tick_`, plus `tick_bench_storage_bytes` | `tick`, whose histograms sit in `.bss` |
| `vm_hwm_baseline_kb`, `stack_baseline_kb` | `baseline`: a process that never builds a robot |

Peak RSS is `VmHWM`; the stack high-water mark is the `Rss` of the `[stack]` mapping, to the
4 KiB page. When `stack_kb` equals `stack_baseline_kb` the report says
`stack_note = "below the first pages"`: the loop's stack fits in the pages the loader already
touched. Each binary runs `FOOTPRINT_TICKS` (3 000) ticks at 1 ms; under a minute.

## Reference chain

Assumed, pending human: D10. The hardware runs (`tick_hw`, `budget.sh --hw`, `presort.sh
--gain`) use this chain, and SRS NFR-1 does not name one yet:

| Item | Value |
|---|---|
| Robot | preset `XmBench`: 4 × Dynamixel XM430-W210, ids 1 to 4, one bus, 1 Mbaud |
| Adapter | one ROBOTIS U2D2, serial id `FT8ISZ6H` (`/dev/serial/by-id`) |
| Supply | not fixed yet: the human records model and voltage here (D10) |
| Source line | `const SERIAL` in `bench/src/bin/tick_hw.rs`, the only line two benches differ in |

```sh
cargo build -p kmr_bench --release --features hardware --bin tick_hw
```

After every hardware run, every actuator must read `Torque_Enable = 0`. The read/write-only
split and the I/O figures wait on the engine's age and read cadence (tasks T057 to T059); until
then a `tick_hw` run measures the loop with the bus attached, nothing more.

## Reports

Every script writes under `bench/reports/<commit12>/<host_id>/<measurement>-<cell>-<n>.toml`,
with kmr's own output beside it as `.log`, and prints the path. The bench never commits: **a
human commits the reports**, with the commit they measured. A report is `complete`,
`incomplete` (Ctrl-C, stop) or `failed` (bad plan, missing operator field, `run()` error); only a
complete one holds `[figures]`.

## Compile time

Clean and incremental build times of `kmr_core` and `controllers`, dev and release, and the
build time of generated user crates against their registration count (FR-013, FR-014 (SRS
NFR-11, §8); research R14).

The reference figures are measured natively on the reference Raspberry Pi 4 (SRS §8: "hardware
always the RP4 setup"). A figure from any other host is labelled with that host and compared
only with figures of the same label. Compile time is not part of `all.sh` and not of the 2-hour
run of SC-001 (research R18).

1. Check out the commit to measure. The script builds a copy of `HEAD` (`git archive`), so
   uncommitted changes are not measured and the working tree is never edited. `[meta]` describes
   that copy: `dirty = "no"`, and `rust_log = "unset"` (the builds run without `RUST_LOG`).
2. Set the four operator fields, as for any run:

   ```sh
   export BENCH_SUPPLY_MV=<mV> BENCH_SUPPLY_METER="<meter model>"
   export BENCH_AMBIENT_MC=<m°C> BENCH_THERMOMETER="<thermometer model>"
   ```

3. Run, with `RUST_LOG` unset:

   ```sh
   bench/scripts/compile-time.sh                         # on the reference Pi: label = host id
   bench/scripts/compile-time.sh --host-label dev-x86    # any other host: a label of your own
   ```

4. `git status --short` shows the one new report and its `.log`, nothing else.

What it does, per profile (dev, then release) and crate, after one untimed warm-up build of
that crate:

| Kind | Crates | Step, timed |
|---|---|---|
| `clean` | `kmr_core`, `controllers` | `cargo clean -p <crate>`, then build; dependencies stay built |
| `body` | `kmr_core`, `controllers` | append `fn __bench_nonce() -> u32 { <n> }` to the crate root, then build |
| `reg` | `controllers` | one more `add_controller` line (the last one, again), then build |

Then the FR-014 sweep: one generated crate per count of `REG_COUNTS` (default `1 16 64 123`),
from `bench/gen/registrations.rs.in`, all registrations in `EachTick`, built in dev with
`CARGO_INCREMENTAL=0`; each timed build follows an edit of the crate. 123 is today's ceiling:
124 registrations in one phase fail with `E0275` (research R21 P8).

Every step runs `REPEATS` times (default 3). The report holds, in `[figures]`:

- `<crate>_<profile>_<kind>_median_ms`, `_min_ms`, `_max_ms`, e.g. `kmr_core_release_clean_median_ms`;
- `reg<count>_dev_median_ms`, `_min_ms`, `_max_ms`, e.g. `reg123_dev_median_ms`.

The median of an even count is the lower middle value. `[figures]` also says
`loop_thread_allocs = "not measured"` and `locks = "not measured"` (FR-012): a build has no loop
thread. `[run]` holds `measurement = "compile"`, `cell` = the label, `ticks_done = 0`, `repeats`
and `reg_counts`. A failed build writes a `failed` report naming
the step; the cargo output is in the `.log`. A missing operator field writes a `failed` report
before any build. A throttle at start or end flags the report `throttled`; on the reference
bench, so does a throttle state that could not be read (`unavailable`).
