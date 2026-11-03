# Bench procedure

How to produce every kmr reference figure from this file alone (FR-021 (SRS NFR-8)). Settings
live in `bench/scripts/profile.env`; commands run from the repository root. Reports land under
`bench/reports/<commit12>/<host_id>/`; a human commits them.

Figures count only from the reference bench (SRS NFR-1). A run anywhere else is labelled
`not_reference` and proves the harness, not kmr.

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
