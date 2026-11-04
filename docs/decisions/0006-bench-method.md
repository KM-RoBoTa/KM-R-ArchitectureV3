# 0006 — bench: how kmr is measured

Status: **proposed**, drafted by an agent, waiting for the maintainer's
decision. It records the method already set in
[`specs/007-bench/research.md`](../../specs/007-bench/research.md) (R1 to R21)
and [`plan.md`](../../specs/007-bench/plan.md) (task A-8), and the code that
follows it in `bench/src/`. Nothing here is new; no figure here is a
reference figure (SRS NFR-1: figures come from the reference bench only).

## The question

SRS NFR-8 asks for the tick rate ceiling (a), jitter (b), behaviour under
load (c), engine overhead (d) and footprint (e), and NFR-11 asks that the
figures be compared across commits. The engine's time cannot be seen from
outside without instrumenting it. The bench has to fix:

- what it may touch of kmr, and how a result leaves the process;
- how a tick's lateness and an overrun are judged, given that the engine
  skips missed grid slots;
- how overhead is measured when the engine's own time is not visible;
- what a report holds, and who decides whether a figure is good.

## Current state

| Piece | State |
|---|---|
| Crate `kmr_bench`, on `kmr_api` only, root heap ban | written (T002, T004) |
| Histogram, grid replay, calibration, run plan, procfs scanner, report writer | written (T006 to T011) |
| Recorder statics, `minimal`, `baseline`, `results compare` and `summary` | written (T013, T029, T030, T034 to T036) |
| `meta.sh`, `profile.env`, `setup-check.sh`, `compile-time.sh` | written (T015, T016, T024, T039) |
| Counting allocator `bench/src/alloc.rs` | not written: HUMAN, `unsafe` (T005) |
| `tick` binary, probe controllers, `budget.sh`, `ceiling.sh`, `footprint.sh` | not written (T014, T018, T027, T031, T032) |
| Engine cross-check (phase B) | waits on T047; the Time getters have landed (T046) |
| Hardware runs (phase C) | wait on delivery step 1 and the first-motor bench run |

Below, a point the table marks as not written is the planned design, not
the branch's behaviour.

## Decision proposed

1. **Black-box through `kmr_api` only.** The bench is one workspace member,
   `kmr_bench`, whose only kmr dependency is `kmr_api`, so Cargo refuses
   `use kmr_core::…` (FR-003 (SRS NFR-8)). It follows the root heap ban like
   every crate but `kmr_api`; text is formatted into fixed buffers (R1, R2).
2. **One process per run.** Each measurement is one process that calls
   `run()` once: the Ctrl-C handler is process-wide, and a failed window
   must not pollute the next one. Scripts in `bench/scripts/` own what only
   the OS does: kernel facts, `chrt`, `taskset`, load, reboots (R1, R8).
3. **Results leave through `static`s.** Controllers' contexts move into the
   engine and never come back, so the probe writes single-writer `static`s:
   relaxed load and store, no read-modify-write, all-zero at rest so they
   sit in `.bss` (R6).
4. **The histogram.** 20 000 buckets of 100 ns (0 to 2 ms), an overflow
   count, exact min and max, a sample count. Percentiles are upper bucket
   edges (`bench/src/hist.rs`; R6).
5. **Lateness and the slot replay.** The probe stamps the start of the first
   `First` controller and the end of the last `Last` controller. Lateness of
   tick k is `start_k − (T0 + s_k·dt)`, `T0` being tick 0's start. The grid
   slot `s_k` is replayed with the engine's rule: after a tick that ended
   late, the missed slots are skipped (`bench/src/grid.rs`; R4). `T0` sits a
   few hundred ns after the engine's `arm`; the report names this anchor,
   `anchor = "tick0_start"`.
6. **The overrun deadline.** Tick k overruns when its `Last` ends after
   `T0 + (s_k+1)·dt`: the engine's `arm` sets the first deadline to
   `now + dt` (`kmr_core/src/clock.rs`, `Time::arm`). The bench counts ticks
   `W .. K−2` (W warm-up, K planned ticks). From phase B, the engine's
   `overrun_count()` over the same ticks must equal the bench's count on a
   simulated run, else the run is flagged `inconsistent` with both counts
   printed (R5).
7. **Overhead by the budget method.** An `EachTick` probe spins a calibrated
   workload; `budget.sh` bisects, at `dt` = 1 ms, the largest workload held
   60 s with zero overruns, to 1 µs. Overhead = `dt` − budget. The workload is
   timed before and after `run()`, 10⁶ iterations, best of 5; a drift above
   2 % flags the run `uncalibrated`. The probe's own cost is measured the
   same way. The report says `method = "budget"`: on a non-RT kernel the
   figure holds the worst wake-up of the window, an upper bound
   (`bench/src/calib.rs`; R8). A second view, the minimum gap between ticks
   at the 10 µs floor, isolates the engine's path between ticks (R9).
8. **Heap allocations by a counting allocator.** A `#[global_allocator]`
   forwards to `System` and counts while a thread-local marks the window on
   the loop thread, from `First` of tick W to `Last` of tick K−1 (R7).
   `unsafe impl GlobalAlloc` is required by the trait, so the file is HUMAN.
9. **The report format.** One file per run, a flat TOML subset (`key =
   integer`, `key = "text"`), schema 1, three blocks: `[meta]` from the
   script, `[run]` and `[figures]` from the binary, written to the
   `KMR_BENCH_OUT` file, never to stdout, where kmr's own log lines land
   (`bench/src/report.rs`; R3, R13). Stored under
   `bench/reports/<commit12>/<host_id>/`, committed by the human.
10. **No verdict.** The bench never passes or fails a build. `results
    compare` prints differences whatever they are: exit 0 when it printed,
    1 when an input could not be read, 2 when it refuses two sets that must
    not be compared (`bench/src/results.rs`; R13; FR-022, FR-023 (SRS
    NFR-11)).

## Defence

### Real-time performance

- The probe adds two stamps and two `static` stores per tick, no lock, no
  heap, no syscall; its cost is measured and reported, not assumed (R8).
- Zero-init statics keep the binary's footprint honest: one non-zero field
  put three histograms in `.data` (R21 P5).
- The bench never sets a cadence (D9); its one setting beyond the defaults
  is the joint-limit policy for all actuators that D2 B, assumed, asks of
  every binary.

### Quality

- **The engine is measured as a user sees it.** No hook, no feature, no
  white-box counter: what the bench reads, a controller could read.
- **The replay follows the engine's grid.** Without it, every tick after the
  first overrun would read one `dt` late (R4).
- **Calibration drift is caught, not averaged away.** On the dev host it
  moved by more than 2 % within one run, twice (R21 P12, not reference).
- **A run that is not clean says so.** `simulated`, `not_reference`,
  `throttled`, `inconsistent`, `uncalibrated` are flags in the report;
  `failed` and `incomplete` runs hold no figures.

### Developer experience

- One report per run, plain and diffable; any TOML reader opens it.
- Every setting lives in `bench/scripts/profile.env`; every run is a script
  call the procedure names.

### Alternatives rejected

- **Criterion or `perf stat`.** White-box; SRS NFR-9 micro-benchmarks belong
  to `kmr_core` (R1, R8).
- **The engine's `elapsed()` as the time base.** Its origin is open (core
  design open choice 6) (R4).
- **`(now − T0) mod dt` as lateness.** Wrong when a start is late by a whole
  `dt` (R4).
- **Every raw sample kept.** 2.4 MB for a 10 min cell, which inflates peak
  RSS (R6).
- **One ramp inside one process for the budget.** A failed window pollutes
  the next one (R8).
- **An allocation counter crate** (`stats_alloc`, `allocation-counter`) or
  `heaptrack`. None sees the window on the loop thread only (R7).
- **JSON or CSV reports.** Noisier diffs; no metadata block (R13).
- **A pass/fail threshold.** The SRS sets no budget for most figures; the
  human judges them.

## SRS problems for the human

Reported, not fixed (SRS NFR-6; constitution V). The details are in
research.md, "SRS problems found".

1. **Definitions "Overrun"** says `origin + k*dt`; the code gives tick k the
   deadline `origin + (k+1)·dt`.
2. **FR-3.2** uses one `k` for the tick and the slot; after an overrun they
   differ.
3. **NFR-1 / NFR-8** name no timer slack and no scheduling policy; under
   `SCHED_OTHER` the default 50 µs slack alone sets the p50 (R12).
4. **NFR-4 / NFR-11** set no limit on registrations per phase and no
   compile-time budget; the prototype found a ceiling at the default
   `recursion_limit` (R21 P8).
5. **FR-5.3 with D10**: on XmBench the pre-sort gain is zero by
   construction (one bus, one series); §8's "pays for itself" cannot be
   answered on that chain (R15).
6. **FR-3.4 with NFR-8 b**: whether read ticks that overrun count is not
   said; the bench reports them apart (R16).
7. **FR-2.1**: where the sample age is exposed is not said.
8. **§4 heap rules** are silent on the bench and on `unsafe` outside the
   engine; the counting allocator needs `unsafe` (R7).
9. Still open from the spec and the coordination page: NFR-8 black-box
   versus the FR-5.3 gain (D11); NFR-1 names no chain (D10); NFR-11 "fastest
   bus or sensor I/O" has no number; §8 has no bench step (D1); the FR-3.3
   `elapsed()` origin and `last_overrun()` meaning.

## Pending

- **D1** (the bench is tooling, phase A starts on today's API), **D2**,
  **D9**, **D10** (XmBench plus one U2D2 as the reference chain) and **D11**
  (`presort-off` build for the pre-sort cost) are assumed, pending human;
  research.md's table names the one place each changes.
- **The counting allocator** is HUMAN (T005).
- **The budget and ceiling scripts** are not written; points 6 and 7 are
  their planned design.
