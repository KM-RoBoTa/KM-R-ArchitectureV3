# Data model: bench

**Date**: 2026-10-28 | **Plan**: [plan.md](plan.md) | **Research**: [research.md](research.md)

Types live in `bench/src/` (crate `kmr_bench`). All are fixed-size and heap-free. A result that
must outlive `run()` is a `static` (research R6). Times are `u64` nanoseconds since the process
epoch unless named otherwise. No `f32` or `f64`: percentiles and ratios are integer
(Constitution, Technical Constraints).

## RunPlan

What one process measures. Read once, before `run()`, from environment variables (contract:
[contracts/commands.md](contracts/commands.md)).

| Field | Type | Default | Rule |
|---|---|---|---|
| `out` | path | none | Required. The report file the binary writes (research R3) |
| `dt_ns` | `u64` | 1 000 000 | Passed to `run()` as given. Outside [10 000, 1 000 000 000] (SRS FR-3.1) `run()` must return an error, then `failed` |
| `ticks` | `u64` | 601 000 | `ticks > warmup + 2` |
| `warmup` | `u64` | 1 000 | Ticks left out of every figure |
| `work_ns` | `u64` | 0 | `work_ns < dt_ns` |
| `write` | `none` \| `all` | `none` | `all`: the `EachTick` probe writes every actuator (research R15, R16) |

A bad variable makes the run `failed` with `reason = "bad plan: <var>"`, before `run()`.

## Probe controllers

Plain `fn` items on statics, no context. From the const-generic N on, each is generic:
`fn start<const M: usize>(_: &Time, _: &mut State<'_, M>, _: &Sensors)`
(`specs/002-kmr-core/design-const-generic-n.md` §2).

| Controller | Phase | Does |
|---|---|---|
| `start` | `First`, registered first | Stamps the start. Tick 0: sets `T0`. Records lateness against the replayed slot (R4) and the gap since the previous `end`. Tick W: opens the allocation window. Phase B: reads `overrun_count()` and `accumulated_overrun()`. Phase C: classifies the tick (read or write-only) by the age of `prev(0)` |
| `work` | `EachTick` | Spins `work_ns` of calibrated workload; writes per `write` through `write_target()` |
| `end` | `Last`, registered last | Stamps the end. Judges the overrun (R5), computes the next slot. Tick K−1: closes the window, raises `shutdown!` |
| `noop` | one in each per-tick phase | Nothing (spec "Empty controllers") |

`write_target()`: `prev(0)` when it exists (hardware), else `Q(0.0)` (simulated: no sample,
assumed FR-6.3 meaning). The probe calls `prev(0)` either way, so both runs do the same work.

`setup::base()`: the robot every bench binary starts from: `dt`, and one joint-limit policy for
every actuator (assumed, pending human: D2).

## Recorder (statics)

All start all-zero, so they sit in `.bss` (research R6, R21 P5).

| Item | Type | Writer | Read |
|---|---|---|---|
| `EPOCH` | `OnceLock<Instant>` | `main`, before `run()` | every stamp |
| `T0`, `SLOT`, `LAST_END`, `LAST_LATE`, `TICK` | `AtomicU64` | `start`, `end` | `start`, `end` |
| `ENGINE_COUNT_PREV`, `ENGINE_ACC_PREV` | `AtomicU64` | `start` (phase B) | `start` |
| `LATENESS` | `Histogram` | `start` | after `run()` |
| `LATENESS_READ`, `LATENESS_WRITE_ONLY` | `Histogram` | `start` (phase C) | after `run()` |
| `GAP_ON_TIME`, `GAP_AFTER_LATE` | `Histogram` | `start` | after `run()` |
| `BENCH_OVERRUNS`, `BENCH_OVERRUNS_WRITE_ONLY` | `AtomicU64` | `end` | after `run()` |
| `ENGINE_OVERRUNS_AT_W`, `ENGINE_OVERRUNS_AT_END` | `AtomicU64` | `start` (phase B) | after `run()` |
| `TICKS_DONE` | `AtomicU64` | `end` | after `run()` |
| `ALLOCS` | `AtomicU64` | counting allocator (R7, HUMAN file) | after `run()` |

Single writer per item: relaxed load and store, no read-modify-write. `Histogram`: 20 000
buckets of 100 ns, overflow count, exact min (stored inverted) and max, sample count.

## Calibration

| Field | Type | Rule |
|---|---|---|
| `ps_per_iter_before` | `u64` | best of 5 × 10⁶ iterations, before `run()` |
| `ps_per_iter_after` | `u64` | same, after `run()` |
| `drift_permille` | `u64` | `|after − before| × 1000 / before`; above 20 → flag `uncalibrated` |
| `probe_cost_ns` | `u64` | two stamps and two histogram stores, timed the same way |

## Run status and flags

`status` is one of three, in this order of precedence:

| Status | When |
|---|---|
| `failed` | bad plan, a required `[meta]` field missing, or `run()` returned an error |
| `incomplete` | `run()` returned `Ok` with `ticks_done < ticks` (Ctrl-C, SIGTERM, emergency stop) |
| `complete` | everything else |

`flags` is a set:

| Flag | When | Effect |
|---|---|---|
| `simulated` | no bus plugin, or kmr printed "not driven yet" | no I/O figure |
| `not_reference` | board revision, distro or kernel line differs from SRS NFR-1 | never compared with reference reports |
| `throttled` | a throttle flag set at start or end | kept out of comparisons |
| `inconsistent` | simulated and the two overrun counts differ (phase B); or the simulation warning seen ≠ 1 time on a run with no plugin | kept out of comparisons |
| `uncalibrated` | `drift_permille > 20` | overhead kept, marked |
| `alloc_seen` | `loop_thread_allocs > 0` | figure kept, marked |

A `failed` or `incomplete` report holds no `[figures]` (FR-024 (SRS NFR-2)); `reason` and
`ticks_done` are present.

## Report

One file per run (format: [contracts/results-schema.md](contracts/results-schema.md)). Three blocks:

- `[meta]`: written by `scripts/meta.sh`. FR-002 (SRS NFR-1, NFR-8) plus research R12's three
  (timer slack, `sched_rt_runtime_us`, `RUST_LOG`).
- `[run]`: written by the binary. Plan, status, flags, reason, ticks done, calibration.
- `[figures]`: written by the binary; the script then adds `sim_warning_count` from the run's
  log (research R3).

## Measurement

| Id | Spec | Binary | Script | Figures |
|---|---|---|---|---|
| `ceiling` | FR-007 (SRS NFR-8 a, FR-3.1) | `tick` | `ceiling.sh` | `ceiling_dt_ns`, `ceiling_hz`, `floor_held`, `gap_ceiling_min_ns`, `gap_ceiling_p50_ns` |
| `jitter` | FR-008 (SRS NFR-8 b) | `tick`, `tick_hw` | `cell.sh` | `jitter_p50_ns`, `jitter_p99_ns`, `jitter_max_ns`, `bench_overruns`, `engine_overruns` (phase B); hardware: `read_*` and `write_only_*` of each |
| `load` | FR-009 (SRS NFR-8 c) | `tick` | `cell.sh load` | as `jitter`; `load_command` in `[meta]` |
| `budget` | FR-010 (SRS NFR-8 d) | `tick`, `tick_hw` | `budget.sh` | `budget_ns`, `overhead_ns`, `gap_floor_min_ns`; hardware: `io_write_tick_ns`, `io_read_ns` |
| `footprint` | FR-011 (SRS NFR-8 e) | `minimal`, `tick`, `baseline` | `footprint.sh` | `file_bytes`, `stripped_bytes`, `text`, `data`, `bss`, `vm_hwm_kb`, `stack_kb`, `stack_baseline_kb` |
| `compile` | FR-013, FR-014 (SRS NFR-11) | none | `compile-time.sh` | per crate × profile × kind: `median_ms`, `min_ms`, `max_ms`; per registration count |
| `presort` | FR-016 (SRS FR-5.3) | `tick`, `tick_hw` | `presort.sh` | `presort_cost_ns`; gain through `results compare` (D11 B) |

Every report also carries `loop_thread_allocs` and `locks = "not measured"` (FR-012).

## Comparison

Input: two directories of reports, A and B, each with R ≥ 1 repeats (R ≤ 16) of the same
measurements.

| Rule | Detail |
|---|---|
| Refuse | `schema`, `host_id`, `robot`, `robot_n`, `kernel_rt` differ between any A and any B report. Exit 2, name the first differing key (FR-022 (SRS NFR-11, NFR-8)) |
| Exclude | status ≠ `complete`, or flag `throttled` or `inconsistent`; counted and listed |
| Per figure | median of each side, its spread (max − min), the difference B − A, the difference over the larger spread in permille |
| No verdict | no pass, no fail, exit 0 (FR-023 (SRS NFR-11, NFR-1)) |

**Summary** is one set from one commit and host, every cell side by side (non-RT and
PREEMPT_RT, idle and loaded). It refuses only a differing `schema`, host or robot. It gives User
Story 1 scenario 2 its side-by-side view.

## Reference bench and chain

| Entity | Fields | Source |
|---|---|---|
| Reference bench | board `Raspberry Pi 4 Model B`, revision `…3114` or `…3115`, Arch Linux ARM, kernel 6.14 ≤ v < 7.0, `rt` yes or no | SRS NFR-1; `meta.sh` |
| Reference chain | preset `XmBench` (N = 4, XM430-W210, one bus, 1 Mbaud), adapter `ROBOTIS U2D2`, its serial id, supply | assumed, pending human: D10 |
| Two-bus gain chain (optional) | `XmBench` over two U2D2, actuators alternating | research R15; D7 A |

## State transitions of one run

```text
plan read ──bad──▶ failed
   │ok
calibrate ─▶ run() ──Err──▶ failed (reason = the error's Display)
               │Ok
               ├─ ticks_done < ticks ─▶ incomplete
               └─ ticks_done = ticks ─▶ complete ─▶ flags from the binary, then from the script
```
