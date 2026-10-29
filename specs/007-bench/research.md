# Research: bench, the reference benchmark of kmr v3

**Date**: 2026-10-28 | **Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

Each entry gives the decision, why, and what else was weighed. "Assumed, pending human: Dn"
marks a choice that waits on `specs/spec-coordination-2026-10-28.md` §1. Each one is isolated:
a different answer changes the one place named in the table.

Prototype claims cite R21 by number (P1 to P12). They come from a scratch copy, never the repo.

## Assumed decisions

| Id | Assumed | Spec ids | One place to change |
|---|---|---|---|
| D1 | A. The bench is tooling, not a delivery step. Phase A (simulated harness) starts now on today's API; phases B to D wait only on what they read | all | plan.md "Order of work", column "Gate" |
| D2 | B, as the core plan assumes: `run()` refuses an actuator without a joint-limit policy. Every bench binary sets one policy (`warn`) for all actuators | FR-003, FR-027 (SRS NFR-8, NFR-1) | `bench/src/setup.rs`, `base()` |
| D9 | A: adapter default per brand. The bench never sets the cadence; it gives read and write-only tick costs apart, the data `DEFAULT_READ_EVERY` needs | FR-008 (SRS FR-3.4), FR-002 | none in the bench |
| D10 | XmBench (4× XM430-W210, one bus, 1 Mbaud) plus one ROBOTIS U2D2, added by the human to SRS NFR-1 | FR-015 (SRS FR-5.3, NFR-1) | `bench/src/bin/tick_hw.rs` (`SERIAL`) and PROCEDURE.md §"Reference chain" |
| D11 | B: the core's opt-in `presort-off` feature (core plan K6.3), forwarded by `kmr_api`. The bench compares the two builds | FR-016 (SRS FR-5.3, §8) | `bench/Cargo.toml` feature `presort-off`, `bench/scripts/presort.sh` |
| D6 | Confirmed form. XmBench has one bus, so no bus label | FR-005 (SRS FR-6.3) | `tick_hw.rs`, the `add_plugins` line |
| D7 | A, only for the optional two-bus gain chain of R15 | FR-016 | `tick_hw.rs` feature `two-bus` |
| FR-6.3 "simulated" (coordination §2) | No-hardware adapter seeds nothing; History reads return "no sample" | FR-006 (SRS FR-6.3) | `bench/src/probe.rs`, `write_target()` |

D3, D4, D5, D8, D12 to D16 do not reach the bench. The probe writes the last read position or a
constant 0 rad, so it never moves a motor and never reaches a limit, envelope or marker path
(D3, D5). N is 4 (D4). D16: FR-027 (SRS NFR-1) asks nothing of the API example.

## R1. Crate shape and name

**Decision**: one workspace member at `bench/`, package `kmr_bench`. It depends on `kmr_api`
only and on no external crate. A library holds the harness. Binaries: `tick` (simulated),
`tick_hw` (feature `hardware`), `minimal` (engine alone, for footprint), `baseline` (exits
before kmr, for footprint), `results` (compare and summary). Bash scripts in `bench/scripts/`
drive runs. `bench/PROCEDURE.md` is the written procedure.

**Rationale**: FR-003 (SRS NFR-8) asks black-box. With `kmr_api` as the only dependency, Cargo
refuses `use kmr_core::…`, as in `controllers/`. The name `kmr_bench` avoids reading
`cargo bench -p bench` as libtest's bench harness. Scripts own what only the OS does: kernel
facts, `chrt`, `taskset`, `stress-ng`, reboots. One process runs `run()` once, because the
Ctrl-C handler is process-wide (`kmr_core/src/runtime.rs:349`).

**Alternatives**: Criterion (white-box micro-benchmarks; SRS NFR-9 belongs to `kmr_core`); an
`xtask` runner in Rust (process handling with heap types, R2); package name `bench`.

## R2. Heap rule for the bench

**Decision**: the bench follows the root `clippy.toml` heap ban, like every crate but `kmr_api`.
No `bench/clippy.toml`. Strings are formatted into fixed buffers with `write!`.

**Rationale**: FR-018 (SRS NFR-1); Constitution II. SRS §4 names only `kmr_core` and `kmr_api`
(spec "SRS issues"). The strict reading costs little. Proven: P1.

**Alternatives**: a `bench/clippy.toml` that lifts the ban for `results` (a policy change, the
human's).

## R3. Output streams

**Decision**: a bench binary writes its `[run]` and `[figures]` blocks to the file named by
`KMR_BENCH_OUT`, never to stdout. The script captures kmr's stdout and stderr together in
`<report>.log` and counts the simulation warning there. The pattern lives in
`scripts/profile.env`, one place.

**Rationale**: P2. kmr's tracing subscriber writes to stdout (`tracing_subscriber`'s default,
`runtime.rs:108-123`): the "control loop stopped" line landed inside the report. Today the
warning is an `eprintln!` in `kmr_api` (stderr, `kmr_api/src/robot.rs:265`). Once kmr_core
FR-042 owns it, it may come through tracing (stdout). Scanning both streams covers both owners.

**Alternatives**: install the bench's own tracing subscriber first (needs a `tracing`
dependency; hides kmr's own messages).

## R4. Timing source, jitter and the slot replay

**Decision**: the probe stamps `Instant::now()` as nanoseconds since a process epoch, at the
start of the first `First` controller and at the end of the last `Last` controller. Lateness of
tick k is `start_k − (T0 + s_k·dt)`. `T0` is tick 0's start. `s_k` is the grid slot of tick k.

The slot is replayed, since the engine skips missed slots and the tick index then stops matching
the slot index (`kmr_core/src/clock.rs:174-201`). After tick k ends at `e_k`:
`s_{k+1} = s_k + 1` if `e_k ≤ T0 + (s_k+1)·dt`, else `s_k + 1 + ceil(late / dt)`. That is the
engine's own rule (`clock.rs:196`).

- Phase A (today): the bench judges `late` from its own stamps.
- Phase B (Time surface): the bench takes `late` from the engine. At each `First` it reads
  `overrun_count()` and `accumulated_overrun()`. When the count grew by one, the previous tick
  was late by the growth of `accumulated_overrun()`. This works under either meaning of
  `last_overrun()` (core design open choice 5).

`T0` sits a few hundred ns after the engine's `arm` instant. The bias is constant and the
report names it (`anchor = "tick0_start"`). The bench does not depend on the open `elapsed()`
origin (core design open choice 6).

**Rationale**: FR-008 (SRS NFR-8 b, FR-3.2) and the spec's jitter assumption. Without the replay,
every tick after the first overrun reads one `dt` late. Without phase B, a tick that ends within
the sub-µs gap between the two clocks can put the bench one slot off the engine.

**Alternatives**: `(now − T0) mod dt` (wrong when a start is late by a whole `dt`); the engine's
`elapsed()` (origin open); `clock_gettime` directly (needs `unsafe` or `libc`).

## R5. Overrun definition and the cross-check

**Decision**: tick k overruns when its `Last` ends after `T0 + (s_k+1)·dt`. The bench counts
overruns of ticks `W .. K−2` (W warm-up, K planned ticks). Phase B: the engine's figure for the
same ticks is `overrun_count()` read at `First` of tick K−1 minus the value at `First` of tick
W. On simulated runs they must be equal (FR-019 (SRS FR-3.3), SC-005); otherwise the run is
flagged `inconsistent` and both counts are printed.

**Rationale**: verified in code. `arm(now)` sets the first deadline to `now + delta`
(`clock.rs:160-161`). Tick 0 starts right after `arm`, so tick k's deadline is
`origin + (k+1)·dt`. The SRS text says `origin + k*dt` (SRS problems 1). The engine counts in
`end_tick`, after `Last`, so the last tick is left out on both sides.

## R6. Sample storage

**Decision**: `static` single-writer histograms: 20 000 buckets of 100 ns (0 to 2 ms), an
overflow count, exact min and max, a sample count. `AtomicU32`/`AtomicU64`, relaxed load and
store, no read-modify-write. Every static starts all-zero (min is stored inverted), so it lands
in `.bss`. Percentiles are upper bucket edges.

**Rationale**: FR-018 (SRS NFR-1): fixed size, no heap, no lock, no `unsafe`. Contexts are moved
into the engine at registration and never come back, so results leave through a `static`. On
one thread the atomics compile to plain loads and stores. The Cortex-A72 has no LSE atomics, so
an RMW would be an LL/SC loop. Zero-init matters for (e): one non-zero field put three 80 KB
histograms in `.data` and grew the file by 240 KB (P5).

**Alternatives**: every raw sample (600 000 × 4 B = 2.4 MB, inflates peak RSS); an HDR
histogram (more code, same answers here).

## R7. Heap allocations inside the window

**Decision**: the bench binaries install a counting `#[global_allocator]` that forwards to
`System`. A `const`-initialized thread-local `Cell<bool>` marks the window on the loop thread.
The probe opens it at `First` of tick W and closes it at `Last` of tick K−1, so engine code
between ticks is inside it.

**Rationale**: FR-012 (SRS NFR-1). The thread-local has no destructor and does not allocate;
`try_with` keeps it safe during teardown. Proven: 0 allocations on today's loop, exactly 1 when
one `CString` is made inside the window (P2, P4).

`unsafe impl GlobalAlloc` is required by the trait. AGENTS.md puts anything `unsafe` off limits
for agents unless an authorization names the file. So `bench/src/alloc.rs` is a **HUMAN** task.
It is about 40 lines and forwards every call unchanged.

**Alternatives**: `stats_alloc` (global count, no window); `allocation-counter` (counts a whole
closure, so bring-up too); `heaptrack` (outside the binary, cannot see the window).

## R8. Engine overhead: the budget method

**Decision**: an `EachTick` probe spins a calibrated workload each tick. `budget.sh` bisects, at
`dt` = 1 ms, for the largest workload held 60 s with zero overruns, to 1 µs. Overhead =
`dt` − budget. Calibration: before `run()` the binary times 10⁶ iterations of `black_box`
arithmetic (best of 5), and again after `run()`. A drift above 2 % flags `uncalibrated`. The
probe's own cost (two stamps, two histogram stores) is measured the same way
(`probe_cost_ns`).

**Rationale**: FR-010 (SRS NFR-8 d, NFR-11). The engine's time is never seen directly. The figure
holds the worst wake-up of the window, so on a non-RT kernel it is an upper bound; the report
says `method = "budget"`. The drift check is needed: on the dev host the calibration moved
596 → 749 ps per iteration in one run and 1039 → 1129 in another (P2).

**Alternatives**: one ramp inside one process (a failed window pollutes the next one); `perf
stat` (white-box).

## R9. Second view of overhead: the inter-tick gap

**Decision**: report the gap `First start(k+1) − Last end(k)` twice.

- At the ceiling rate, as FR-010 words it: `gap_ceiling_min_ns`, `gap_ceiling_p50_ns`. Most of
  it is sleep.
- At the 10 µs floor, over overrun ticks: `gap_floor_min_ns`. When a late tick ends just before
  a grid point, the computed sleep is zero and `std::thread::sleep` makes no syscall. The
  minimum gap is then the engine's own path between ticks (end of tick, bus checks, start of
  tick), plus the probe's stamps.

**Rationale**: the previous draft of this research said the engine does not sleep after an
overrun. It does: it sleeps to the next grid point (`runtime.rs:290-292`; core test
`sleeps_until_the_grid_after_an_overrun_too_and_not_on_exit`, `runtime.rs:597`). Measured
(P3): at `dt` = 10 µs with 50 µs timer slack, the gap after a late tick had p50 59.9 µs (sleep
and slack) and min 0.47 µs (no sleep). So only the minimum isolates the engine.

**Spec issue**: FR-010's "gap at the ceiling rate" mostly measures sleep. Proposed spec change:
"the minimum gap at the 10 µs floor".

## R10. Ceiling search

**Decision**: `ceiling.sh` bisects `dt` between 10 µs and 1 ms on a log scale, to 1 % of `dt`.
Each candidate is one process held 60 s. It holds when the bench's overrun count is zero (and,
from phase B, the engine's). If 10 µs holds, the report says "ceiling at least 100 kHz (floor
held)".

**Rationale**: FR-007 (SRS NFR-8 a, FR-3.1), SC-008. On a non-RT kernel the ceiling may fall below
1 kHz: one wake-up spike in 60 s fails a window. That is a finding, not a fault. Until the
FR-3.1 range check lands (kmr_api A6, core M3), `run()` accepts a `dt` below 10 µs (P4); the
search never goes there.

## R11. Footprint

**Decision**: measured on `minimal` (engine, no-op controllers, no bench storage) and on `tick`.
File size, stripped size, `size` sections. Peak RSS = `VmHWM` from `/proc/self/status`. Stack
high-water mark = `Rss` of the `[stack]` mapping in `/proc/self/smaps`. Both are read by the
binary after `run()` returns, with a chunked line scanner in fixed buffers. `baseline` gives the
pages touched before `main` (loader, argv, env).

**Rationale**: FR-011 (SRS NFR-8 e). The main thread runs the loop (threaded controllers are not
spawned). Linux faults its stack in page by page, so `[stack]` Rss is the high-water mark to
4 KiB. Proven (P6): touching 256 KiB of stack reads 264 kB. Today's loop stays inside the
baseline pages: `minimal`, `tick` and `baseline` all read 16 kB. The report then says
`stack_kb = stack_baseline_kb`, meaning "below the first pages". Finer resolution needs stack
painting, which is `unsafe`.

**Alternatives**: stack painting (`unsafe`); `/usr/bin/time -v` (peak RSS only).

## R12. Metadata and host setup

**Decision**: `scripts/meta.sh` writes `[meta]`. Board revision from
`/proc/device-tree/system/linux,revision` (4 bytes, big-endian), fallback the `Revision` line
of `/proc/cpuinfo`; the procedure stops if neither gives one. Throttle flags from
`vcgencmd get_throttled`, else the `raspberrypi-hwmon` under-voltage alarm. Supply voltage,
ambient temperature and both instrument names come from variables the operator sets per run
(`BENCH_SUPPLY_MV`, `BENCH_AMBIENT_MC`, `BENCH_SUPPLY_METER`, `BENCH_THERMOMETER`). A missing
one fails the run (FR-024 (SRS NFR-2)). The procedure fixes governor `performance`,
`chrt -f 80`, `taskset -c 3`, core isolation off, load on all cores.

Three fields join FR-002: timer slack (the binary reads `/proc/self/timerslack_ns`),
`sched_rt_runtime_us`, and `RUST_LOG`. `RUST_LOG` must be unset: any value sets kmr's level to
TRACE (`runtime.rs:112-113`), which prints lines inside every tick.

**Rationale**: FR-001, FR-002 (SRS NFR-1, NFR-8). An arm64 kernel prints no `Revision` line; the
Pi's device tree carries it. Not yet checked on the board. Timer slack sets the non-RT p50 (P7):
55.9 and 56.4 µs at the default 50 µs slack, 7.8 and 4.2 µs at 1 ns. `SCHED_FIFO` threads get
no slack, so `chrt -f 80` removes it; the field shows it.

## R13. Report format and storage

**Decision**: one file per run, a flat TOML subset: `key = integer`, `key = "text"`, three blocks,
schema 1. The script writes `[meta]`; the binary writes `[run]` and `[figures]`. Kept at
`bench/reports/<commit12>/<host_id>/<measurement>-<cell>-<n>.toml`, committed by the human.
`results compare <A> <B>` and `results summary <dir>`; contract in
[contracts/results-schema.md](contracts/results-schema.md).

**Rationale**: FR-022, FR-023 (SRS NFR-11, NFR-8). Plain and diffable; any TOML reader opens it.
The bench never passes or fails a build: `results` exits 0 when it compares, 2 only when it
refuses.

**Alternatives**: JSON (noisier diffs); CSV (no metadata block).

## R14. Compile-time method

**Decision**: `compile-time.sh` works on a copy of the commit (`git archive HEAD | tar -x` into a
temporary directory). It never edits the working tree. Per crate (`kmr_core`, `controllers`) and
profile (dev, release):

- **clean**: `cargo clean -p <crate>`, then build (dependencies stay built);
- **incremental, body edit**: append `fn __bench_nonce() -> u32 { <nonce> }`, then build;
- **incremental, registration edit** (`controllers` only): one more `add_controller` line.

FR-014: generated crates with 1, 16, 64 and 123 registrations in one phase, built with
`CARGO_INCREMENTAL=0`. 123 is today's ceiling (P8). Three repeats; median and spread.

**Rationale**: FR-013, FR-014 (SRS NFR-11, §8). P9: a comment or `touch` rebuilds in 0.2 to
0.3 s at every size, since rustc's incremental cache reuses the trait solving. With
`CARGO_INCREMENTAL=0` the figures repeat within 2 % and grow superlinearly: 64 → 0.9 s,
96 → 2.6 s, 123 → 5.8 s (P9). Reference figures are native on the Pi (SRS §8); other hosts are
labelled. The list of counts lives in `profile.env`.

**Alternatives**: `cargo build --timings` (HTML; JSON needs nightly); editing `kmr_core` in
place (human-only path).

## R15. Pre-sort cost and gain

**Decision**:

- **Cost**, simulated: a budget search with the probe writing every actuator, on the default
  build and on the `presort-off` build. Overhead difference = the per-tick CPU cost of the
  pre-sort. Without the switch (D11 A), the fallback is write-all versus write-none, an upper
  bound that includes the mask, batch and clear.
- **Gain**, hardware: the same two builds on the reference chain, compared with `results
  compare`.

Assumed, pending human: D11.

**Finding**: on XmBench the gain is zero by construction. The four XM430 share one (bus, series)
group, so the pre-sort has nothing to reorder. And the per-tick cost is near zero by design: the
sort runs once at finalization (`kmr_core/src/hal.rs:107-114`, core FR-044); each tick walks the
pre-sorted order once (`hal.rs:224-228`). The section 8 question "does it pay for itself" needs
a chain with two interleaved groups. Optional variant: XmBench split over two U2D2 buses with
actuators alternating (feature `two-bus`, D6 labels and D7 A table). Reported to the human with
D10 and D11.

**Rationale**: FR-016 (SRS FR-5.3, §8). The cost measure needs the engine to build the batch on
the no-hardware adapter too. The core design does (`design-first-motor.md` §2 #9: `Batch` after
`Last`, `write` only if not empty).

## R16. Hardware runs

**Decision**: `tick_hw` is `tick` plus `.add_plugins(Bus::by_id(SERIAL))`, feature `hardware`
(forwards `kmr_api/dynamixel`). The `EachTick` probe writes `prev(0)` back to every actuator
(no motion). A tick is a read tick when any actuator's `prev(0)` has age 0 at `First`.

- Write-only ticks: I/O cost = budget(simulated) − budget(hardware). On hardware runs, "held"
  counts overruns of write-only ticks only.
- Read ticks: read cost = lateness p50 of read ticks − lateness p50 of write-only ticks. Their
  overrun count is reported apart, not judged.

Until delivery step 1, `kmr_api` prints "plugins declared but hardware not driven yet"; the run
is then `simulated` and holds no I/O figure (FR-006).

**Rationale**: FR-015, FR-017 (SRS FR-5.3, FR-5.10, NFR-11), FR-008 (SRS FR-3.4). The read comes
before `First`, so it shows in the start lateness. The adapter research puts a Protocol 2 read of
four XM430 at about 1 ms of wire time plus up to 1 ms of USB latency (`specs/001-kmr-adapters/
research.md` R8). At `dt` = 1 ms every read tick then overruns, whatever the cadence. A budget
that counts them would always be zero. `prev(0)` is the hold target the core design uses, so no
motion and no limit path (D2). Assumed, pending human: D10.

## R17. Load profile

**Decision**: `stress-ng --cpu 0 --cpu-method all --hdd 1 --hdd-bytes 256M --timeout <len>s`, all
cores, started 5 s before the run. The exact command is a `[meta]` field.

**Rationale**: FR-009 (SRS NFR-8 c). All cores, as the spec assumes (no isolation).

## R18. Run lengths and the 2-hour target

**Decision**: 1 kHz cells: 600 000 ticks plus 1 000 warm-up. Ceiling and budget candidates: 60 s
each (about 9 and 10 candidates). Per kernel: (b) 10 min, (c) 10 min, (a) about 9 min, (d) about
10 min, (e) under 1 min: about 40 min. Two kernels: about 80 min plus one reboot. All lengths
live in `scripts/profile.env`.

**Rationale**: spec assumptions; SC-001 (2 hours from a flashed board, (a) to (e)). Building a
PREEMPT_RT kernel on the Pi takes hours, so SC-001 holds only if the image carries both kernels;
the procedure says so. Compile-time tracking is outside (a) to (e) and outside the 2 hours.

## R19. Order of work

**Decision**: four phases.

| Phase | Work | Gate |
|---|---|---|
| A | Harness, simulated runs, ceiling, budget, footprint, compile time, `results`, procedure | Workspace member line only. Assumed, pending human: D1 |
| B | Engine cross-check and slot follow (R4, R5) | core K1.1 (#14) Time getters; `kmr_api` A1 forwarding |
| C | Hardware runs, I/O cost, read/write-only split | delivery step 1 (core M1, `kmr_api` A5); core K3.5 age and K3.8 cadence |
| D | Pre-sort gain, Zenoh run, FFI run | core K6.3 `presort-off`; `kmr_api` Z2; `kmr_ffi` C2 |

**Rationale**: SRS §8 has no bench step, and the bench measures every step. Phase A touches no
other crate except the root `Cargo.toml` member line.

## R20. Zenoh and FFI runs

**Decision**: feature `zenoh` (forwards `kmr_api/zenoh`) adds kmr_api's gateway plugin to `tick`
with its default-deny settings and no peer. The report records the feature. `KMR_BENCH_TICKS`
can be raised to 1 001 000 for kmr_api SC-003. Feature `ffi`: a `tick_ffi` binary whose
`EachTick` controller is an empty C forwarder, compiled by a `build.rs` that only compiles
foreign code (D12 A). Compared with an empty Rust controller through `results compare`
(kmr_ffi SC-003, its plan E3).

**Rationale**: FR-026 (SRS FR-7.1), FR-022. The bench opens no endpoint of its own (Constitution
VI).

## R21. Scratch prototype (results)

Path: `scratchpad/bench-plan/ws/bench/` (a copy of today's workspace plus a `bench` member;
never in the repo). Host: 32 threads x86-64, Linux 7.2.11-arch1-1, rustc 1.97.1, `SCHED_OTHER`,
not reference. `CARGO_TARGET_DIR=scratchpad/bench-plan/target`, `--offline`. These prove the
harness, not any kmr figure (SRS NFR-1: figures come only from the reference bench).

| # | Claim | Command | Result |
|---|---|---|---|
| P1 | Builds on `kmr_api` alone, clean under the root heap ban | `cargo test -p kmr_bench`; `cargo clippy -p kmr_bench --all-targets` | 2 passed, 0 failed; 0 warnings from `kmr_bench` (6 in `kmr_core`, 6 in `kmr_api`, pre-existing) |
| P2 | Black-box 1 kHz simulated run on today's API | `tick`, 6 000 ticks, warm-up 1 000, twice | `complete`, 5 000 samples; p50 55.9 / 56.4 µs, p99 61.4 / 61.0 µs, max 124.9 / 140.6 µs; 0 overruns; 0 loop allocations; VmHWM 2 924 / 2 936 kB; "no bus plugin: simulating" exactly once each (stderr); kmr's INFO line on stdout |
| P3 | Gap after an overrun includes the sleep | `tick`, `dt` 10 µs, 20 000 window ticks, slack 50 µs | 19 995 overruns; gap after a late tick: p50 59.9 µs, min 0.469 µs |
| P4 | Allocation counter, bad plan, Ctrl-C, sub-floor `dt` | `KMR_BENCH_INJECT_AT=1010`; `KMR_BENCH_TICKS=abc`; `timeout -s INT 2`; `KMR_BENCH_DT_NS=5000` | `loop_thread_allocs = 1`; `failed`, `bad plan: KMR_BENCH_TICKS`, exit 1; `incomplete`, 1 991 ticks, no figures, exit 0; `complete` with `dt_ns = 5000` (no range check yet) |
| P5 | Zero-init statics stay out of the file | `size -A` before and after storing min inverted | `.data` 244 384 → 4 288 B; file 1 114 176 → 874 080 B |
| P6 | Stack method sees stack use | touch 256 KiB, read `[stack]` Rss | 16 → 264 kB. `minimal`, `tick`, `baseline` all 16 kB |
| P7 | Timer slack sets the non-RT p50 | `tick` after writing 1 to `/proc/self/timerslack_ns` (no `unsafe`) | p50 7.8 / 4.2 µs, p99 124.7 / 11.1 µs, max 321.6 / 118.5 µs. At `dt` 10 µs: 23 overruns in 20 000 ticks |
| P8 | Registration ceiling per phase | generated `add_controller(EachTick, cN)` chains, then `.run()` | 123 build; 124 fail `E0275` "overflow evaluating `kmr_api::robot::ApiInline<..>: kmr_core::controllers::ControlFn`" |
| P9 | Compile-time growth (dev, `CARGO_INCREMENTAL=0`, two repeats) | first build after an edit | 1: 0.69 / 0.21 s, 16: 0.20 / 0.22, 32: 0.32 / 0.29, 64: 0.88 / 0.92, 96: 2.56 / 2.61, 123: 5.81 / 5.83. Release 123: 4.8 s. Comment-only edit with incremental on: 0.19 to 0.30 s |
| P10 | Footprint of the engine alone | `minimal`: 3 no-op controllers + a stop counter | 838 776 B, stripped 606 080 B; text 568 039, data 34 664, bss 3 362; VmHWM 2 676 kB (baseline 2 284 kB) |
| P11 | Cross-build (FR-025 (SRS NFR-5)) | aarch64 with `aarch64-linux-gnu-gcc`; armv7 `cargo check` | aarch64 links, 1 157 560 B; armv7 checks; armv7 link not tried, no `arm-linux-gnueabihf-gcc` on the dev host |
| P12 | Calibration drift is real | ps per iteration before and after `run()` | 596 → 749 and 1 039 → 1 129: both would be `uncalibrated` |

## SRS problems found (reported, not fixed)

1. **Definitions "Overrun"** says `origin + k*dt`. Verified in `clock.rs:160-161` and `:174-201`:
   tick k's deadline is `origin + (k+1)·dt`. Coordination §2 lists it as unverified; it is now
   verified.
2. **FR-3.2** uses one `k` for the tick and the slot. After an overrun they differ: missed slots
   are skipped, ticks are not.
3. **NFR-1 / NFR-8 setup** names no timer slack and no scheduling policy. Under `SCHED_OTHER` the
   default 50 µs slack alone sets the p50 (P2, P7). v2's "50 to 100 µs on non-RT" fits a 50 µs
   slack.
4. **NFR-4 / NFR-11**: one phase holds at most 123 registrations at the default
   `recursion_limit` (P8); the 124th is `E0275` naming `kmr_core::controllers::ControlFn`.
   Compile time grows superlinearly (P9). The SRS sets no limit and no budget.
5. **FR-5.3 with D10**: XmBench has one (bus, series) group, so the pre-sort gain is zero there by
   construction, and the per-tick cost is near zero by design (sort at finalization). Section 8's
   "pays for itself" cannot be answered on that chain (R15).
6. **FR-3.4 / Definitions "Overrun" with FR-5.3 reads**: a Protocol 2 read of four XM430 takes
   about 1 to 2 ms, so at 1 kHz every read tick overruns. The SRS does not say whether read ticks
   count in NFR-8 b's overrun count. The bench reports them apart (R16).
7. **FR-2.1** "its age in ticks is exposed": exposed where is not said. The bench needs it to
   split read ticks (FR-008).
8. **§4 heap rules** are silent on the bench and on `unsafe` outside the engine; the counting
   allocator needs `unsafe` (R7).
9. Already listed by the spec or coordination §2, still open: NFR-8 black-box versus FR-5.3 gain
   (D11); NFR-1 has no chain (D10); "CPU rev 1.4 or 1.5" is the board revision; NFR-11 "fastest
   bus or sensor I/O" has no number; §8 delivery order has no bench step (D1); FR-3.3
   `elapsed()` origin and `last_overrun()` meaning.

## Findings for other owners (not SRS)

- **kmr_core (HUMAN)**: kmr logs to stdout (`runtime.rs:108-123`), mixing with a user's output;
  any `RUST_LOG` value turns on TRACE lines inside every tick. Core M3 ("no hot-path logging")
  should cover both.
- **AGENTS.md / CLAUDE.md** say each `add_controller` "prepends one node"; `schedule.rs:29-58`
  appends (`HAppend`). Doc only; `schedule.rs` is an invariant file, explained, not edited.
- **Spec FR-010**: the second view "at the ceiling rate" is mostly sleep (R9).
- **Spec FR-015**: assumes read ticks fit the tick (R16).
