---
description: "Tasks for bench, the reference benchmark of kmr v3, generated 2026-10-28"
---

# Tasks: bench, the reference benchmark of kmr v3

**Input**: `specs/007-bench/`: [plan.md](plan.md) (2026-10-28), [spec.md](spec.md) (coordinated
2026-10-28), [research.md](research.md), [data-model.md](data-model.md),
[quickstart.md](quickstart.md), [contracts/](contracts/). Authority: `docs/srs/SRS.md`.
Coordination: `specs/spec-coordination-2026-10-28.md`.

**Tests**: included. The plan's Testing line asks for unit tests (histogram, slot replay, plan
parsing, results parser, compare rules). quickstart.md scenarios 1 to 11 are the acceptance runs.
No test needs the Pi; the reference runs are HUMAN tasks.

**HUMAN**: `bench/src/alloc.rs` (`unsafe impl GlobalAlloc`, research R7: AGENTS.md puts `unsafe`
off limits), the CI workflow edit (T026), every run on the reference Pi, every commit of `bench/reports/`, and acceptance of
the decision record. `kmr_core/` and `../dxl_rs` are human-only: this file never edits them and
names the core plan step it waits on. No bench task touches an invariant file (`state/field.rs`,
`sealed.rs`, `signal.rs`, `schedule.rs`, `controllers.rs`, `schedule/macros.rs`). A gate whose
upstream step touches one names it in **bold**.

**Hand-off**: this plan edits nothing in `kmr_api/`, `kmr_adapters/`, `kmr_ffi/`, `kmr_model/`
or `controllers/`. A task that needs them says "Hand-off" or "Waits on" and names the owning
plan's step or task id. "Blocked on Tnnn" names a HUMAN task of this file.

**Critical path**: the bench has no step on coordination §3. Step 11 (HUMAN run, `Torque_Enable`
= 0) uses the controllers demo (kmr_api A9, kmr_adapters T045), not a bench binary. Phase A
starts now; bench phase C starts after step 11.

**Assumed, pending human**: D1 (A: the bench is tooling; phase A starts now), D2 (B: one
joint-limit policy, `warn`, for every actuator), D6 (one bus, no label), D7 (A, optional two-bus
chain only), D9 (A: the bench never sets the cadence), D10 (XmBench plus one ROBOTIS U2D2), D11
(B: core `presort-off`), D12 (A, FFI run only), and the FR-6.3 meaning of "simulated" (the
no-hardware adapter seeds nothing; History reads return "no sample"). Each sits in the one place
its task names (research "Assumed decisions").

**Ids**: `FR-0NN` / `SC-0NN` are this spec's, written with their SRS source. Stories: US1 timing at
1 kHz without hardware (P1), US2 reproduce a published run (P1), US3 tick-rate ceiling (P2), US4
overhead and footprint (P2), US5 compare two builds (P2), US6 compile time (P3), US7 hardware I/O
(P3). Plan steps A-1 to D-5 are plan.md "Order of work".

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an unfinished task)
- **[Story]**: US1..US7 from spec.md

Paths are from the repository root.

---

## Phase 1: Setup (plan A-1)

**Purpose**: a green baseline and an empty `kmr_bench` crate in the workspace.

- [x] T001 Run from the repo root `cargo build --workspace`, `cargo clippy --workspace --all-targets`, `cargo test --workspace --no-fail-fast`, `cargo fmt --all --check`; write down the warnings already there (research R21 P1: 6 in kmr_core, 6 in kmr_api), so new ones stand out; stop and report if anything is red
- [x] T002 Add `"bench"` to `members` in the root Cargo.toml, the only line outside `bench/` that phase A touches (plan A-1). A new crate is agentic work: it needs the maintainer's recorded authorization on the PR (AGENTS.md "Agentic work"; plan Constitution Check)
- [ ] T003 Create bench/Cargo.toml: package `kmr_bench` (research R1), `edition.workspace = true`, one dependency `kmr_api = { path = "../kmr_api" }`, no external crate; feature `hardware = ["kmr_api/dynamixel"]`; `[[bin]] name = "tick_hw"` with `required-features = ["hardware"]`. No bench/clippy.toml: the root heap ban applies (research R2; FR-018 (SRS NFR-1))
- [x] T004 Create bench/src/lib.rs: `#![deny(clippy::unwrap_used)]`, a module doc "black-box: kmr_api only" (FR-003 (SRS NFR-8)), and the `mod` lines of Phase 2; `cargo build -p kmr_bench` and `cargo clippy -p kmr_bench --all-targets` green

---

## Phase 2: Foundational: the harness library (plan A-2, A-3, A-4 probe part, A-6 shared scripts)

**Purpose**: what every measurement shares. Blocks every story. T005 is the one HUMAN gate in
phase A: do it first.

- [ ] T005 HUMAN: bench/src/alloc.rs (`unsafe`, research R7): a counting `#[global_allocator]` that forwards every call to `System` unchanged; a `const`-initialized `thread_local!` `Cell<bool>` window flag, read with `try_with` (no destructor, no allocation); the count `ALLOCS: AtomicU64`, relaxed load and store, counted only while the flag is set; `pub fn open_window()`, `pub fn close_window()`, `pub fn allocs() -> u64`. About 40 lines (FR-012 (SRS NFR-1)). Agents never write this file
- [x] T006 [P] bench/src/hist.rs: `Histogram` of "20 000 buckets of 100 ns (0 to 2 ms), an overflow count, exact min and max, a sample count"; `AtomicU32` / `AtomicU64`, "relaxed load and store, no read-modify-write"; a `const fn new()` that is all-zero, min stored inverted, so a `static` lands in `.bss` (research R6, R21 P5); p50 and p99 as upper bucket edges, integers only. Unit tests in the file: bucket edges, overflow, min and max, p50 and p99 of a known set (FR-008 (SRS NFR-8 b), FR-018 (SRS NFR-1))
- [x] T007 [P] bench/src/grid.rs: pure functions, no statics. Lateness of tick k = `start_k − (T0 + s_k·dt)`, saturating at 0 (`T0` sits after the engine's `arm`, research R4; a start before its grid point must not wrap or panic). Next slot: `s_{k+1} = s_k + 1` if `e_k ≤ T0 + (s_k+1)·dt`, else `s_k + 1 + ceil(late / dt)` (the engine's rule, kmr_core/src/clock.rs:196). Overrun: tick k overruns when its `Last` ends after `T0 + (s_k+1)·dt` (research R5; spec Assumptions "Overrun deadline"). Unit tests replay a scripted run with skipped slots, a tick late by exactly `dt`, one late by `3·dt + 1 ns`, and an early start (FR-008, FR-019 (SRS FR-3.2, FR-3.3))
- [x] T008 [P] bench/src/plan.rs: `RunPlan`, read once before `run()` from `KMR_BENCH_OUT` (required), `KMR_BENCH_DT_NS` (1 000 000), `KMR_BENCH_TICKS` (601 000), `KMR_BENCH_WARMUP` (1 000), `KMR_BENCH_WORK_NS` (0), `KMR_BENCH_WRITE` (`none` | `all`, default `none`), `KMR_BENCH_MEASUREMENT` (`jitter`), `KMR_BENCH_CELL` (`adhoc`) (contracts/commands.md). Rules verbatim: "`ticks > warmup + 2`", "`work_ns < dt_ns`"; `dt_ns` goes to `run()` as given, the bench never range-checks it. A bad variable gives `failed` with `reason = "bad plan: <var>"`, before `run()`. No heap type. Unit tests: defaults, each bad variable, each rule (data-model.md "RunPlan"; FR-024 (SRS NFR-2))
- [x] T009 [P] bench/src/calib.rs: `spin(work_ns)` over `std::hint::black_box` integer arithmetic; `calibrate()` = best of 5 runs of 10⁶ iterations, in ps per iteration; `drift_permille` = "`|after − before| × 1000 / before`; above 20 → flag `uncalibrated`"; `probe_cost_ns` = "two stamps and two histogram stores, timed the same way". Unit tests: drift arithmetic, 20 not flagged, 21 flagged (FR-010 (SRS NFR-8 d); research R8, R21 P12)
- [x] T010 [P] bench/src/procfs.rs: a chunked line scanner over a fixed buffer, taking any `Read`; `vm_hwm_kb()` from `VmHWM` in /proc/self/status; `stack_kb()` from the `Rss` line of the `[stack]` mapping in /proc/self/smaps; `timer_slack_ns()` from /proc/self/timerslack_ns. Unit tests on fixture bytes, one with a line split across two chunks (FR-011 (SRS NFR-8 e); research R11, R12)
- [x] T011 [P] bench/src/report.rs: writes `[run]` and `[figures]` to the `KMR_BENCH_OUT` file, never stdout (research R3), with `write!` into fixed buffers. Format of contracts/results-schema.md: `key = <integer>` or `key = "<text>"`; "Max 256 keys, 128 bytes per key, 512 bytes per text value, 64 KiB per file"; "A missing figure is an absent key, never `0`". `[run]`: `measurement`, `cell`, `dt_ns`, `ticks`, `warmup`, `work_ns`, `write`, `status`, `flags`, `reason` (when `status ≠ "complete"`), `ticks_done` (always), `anchor = "tick0_start"`, `time_surface = "absent"`, `ps_per_iter_before`, `ps_per_iter_after`, `drift_permille`, `probe_cost_ns`, `bench_storage_bytes`. Status precedence `failed` > `incomplete` > `complete`; flags from `simulated, not_reference, throttled, inconsistent, uncalibrated, alloc_seen`. `[figures]` only when `complete`, and then always with `loop_thread_allocs` and `locks = "not measured"`. Unit tests: `failed` and `incomplete` hold no `[figures]`; the limits hold (data-model.md "Run status and flags"; FR-012, FR-023, FR-024 (SRS NFR-1, NFR-11, NFR-2))
- [x] T012 [P] bench/src/setup.rs: `base(dt)`, the builder every bench binary starts from, `Robot::new().set_dt(dt)`. This is the one place for D2: T051 adds the policy line when kmr_api C4 exists (FR-003, FR-027 (SRS NFR-8, NFR-1))
- [x] T013 bench/src/probe.rs, recorder statics (data-model.md "Recorder"): `EPOCH: OnceLock<Instant>`; `T0`, `SLOT`, `LAST_END`, `LAST_LATE`, `TICK`, `BENCH_OVERRUNS`, `TICKS_DONE` (`AtomicU64`); `LATENESS`, `GAP_ON_TIME`, `GAP_AFTER_LATE` (`Histogram`, T006). All start all-zero; one writer per item; relaxed load and store; `bench_storage_bytes()` sums their sizes
- [ ] T014 bench/src/probe.rs, controllers: plain `fn` items, no context; no heap, no `dyn`, no lock, no I/O (FR-018 (SRS NFR-1)). `start` (`First`, registered first): stamp; tick 0 sets `T0`; from tick W, lateness against the replayed slot (T007) into `LATENESS`, gap since the previous `end` into `GAP_ON_TIME` or `GAP_AFTER_LATE`; at tick W opens the allocation window. `work` (`EachTick`): `spin(work_ns)`; with `write = all`, writes every actuator through `write_target()`. `end` (`Last`, registered last): stamp; judges the overrun of ticks W..K−2 into `BENCH_OVERRUNS`; next slot; `TICKS_DONE`; at tick K−1 closes the window and raises `kmr_api::shutdown!` (FR-020 (SRS FR-4.1)). `noop`: nothing. `write_target()`: `state.prev_q(0)` when `Ok`, else `[Q(0.0); N]`, written with `set_all_q`; it calls `prev_q(0)` either way, so both kinds of run do the same work (assumed FR-6.3 meaning, the one place; research R15, R16). Blocked on T005 (window calls)
- [x] T015 [P] bench/scripts/meta.sh: writes `[meta]` with every key of contracts/results-schema.md "`[meta]`". Board revision from /proc/device-tree/system/linux,revision (4 bytes, big-endian), else the `Revision` line of /proc/cpuinfo, else stop. Throttle flags from `vcgencmd get_throttled`, else the `raspberrypi-hwmon` under-voltage alarm. `BENCH_SUPPLY_MV`, `BENCH_SUPPLY_METER`, `BENCH_AMBIENT_MC`, `BENCH_THERMOMETER` from the operator; a missing one fails the run. `reference = "yes"` only for revision `…3114` or `…3115`, Arch Linux ARM and kernel 6.14 ≤ v < 7.0, else `"no"`. Also `sched_rt_runtime_us`, `rust_log`, `commit`, `dirty`, `host_id` (research R12; FR-001, FR-002 (SRS NFR-1, NFR-8))
- [x] T016 bench/scripts/profile.env: every setting in one place (research R3, R12, R14, R17, R18): `TICKS=601000`, `WARMUP=1000`, `HOLD_S=60`, the ceiling bounds (10 000 to 1 000 000 ns, stop at 1 % of `dt`), the budget step (1 µs), `REG_COUNTS="1 16 64 123"`, `REPEATS=3`, `LOAD_CMD` (`stress-ng --cpu 0 --cpu-method all --hdd 1 --hdd-bytes 256M --timeout <len>s`, started 5 s before the run), `chrt -f 80`, `taskset -c 3`, governor `performance`, `SIM_WARNING="no bus plugin: simulating"`, the report path `bench/reports/<commit12>/<host_id>/<measurement>-<cell>-<repeat>.toml`. Plus the shared run step `bench_run` (the plan names no file for it): `meta.sh` at start; the binary under `chrt` and `taskset` with `RUST_LOG` unset; stdout and stderr into `<report>.log`; `meta.sh` end fields; the `[run]` keys only the binary knows (`robot`, `robot_n`, `buses`, `controllers`, `timer_slack_ns`; later `read_cadence`, `declared_read_ns`, `declared_write_ns`) copied into `[meta]` (contracts/results-schema.md says "copied here" but names no carrier: reported); `not_reference` from `reference`; `throttled` when a throttle flag is set at start or end; `sim_warning_count` (both streams) added to `[figures]`, and `inconsistent` when it is not 1 on a run with no bus plugin. Any timeout sends SIGINT, never SIGTERM (no report on SIGTERM before core K1.2)

**Checkpoint**: `cargo test -p kmr_bench` green on hist, grid, plan, calib, procfs and report.

---

## Phase 3: User Story 1, timing at 1 kHz without hardware (Priority: P1) 🎯 MVP (plan A-4 `tick`, A-5 `summary`, A-6 `cell.sh`)

**Goal**: jitter p50, p99, max and the overrun count per cell, simulated, with metadata. FR-006,
FR-008, FR-009 (SRS FR-6.3, NFR-8 b, c), FR-012, FR-018, FR-020, FR-024.

**Independent Test**: quickstart.md §2, §3, §4 on any Linux host (`not_reference`); on the Pi,
one `cell.sh` per cell.

### Tests, red first

- [ ] T017 [US1] bench/tests/tick_runs.rs: spawns `env!("CARGO_BIN_EXE_tick")` with `Command::env` (no `set_var`, unsafe in edition 2024), output to files read into fixed buffers (no heap type named; the root ban covers tests). Cases: (a) 6 000 ticks: exit 0, `complete`, `flags` holds `simulated`, the jitter keys present, `loop_thread_allocs = 0`, "no bus plugin: simulating" exactly once over stdout and stderr, no kmr log line in the report (quickstart §2); (b) `KMR_BENCH_TICKS=abc`: exit 1, `failed`, `reason = "bad plan: KMR_BENCH_TICKS"`, no `[figures]` (§4); (c) 600 000 ticks, `kill -INT` after 2 s: exit 0, `incomplete`, `ticks_done` < 600000, no `[figures]` (§4); (d) `KMR_BENCH_INJECT_AT=1010`: `loop_thread_allocs = 1`, `alloc_seen`, still `complete` (§3; blocked on T005). Red until T018, T019

### Implementation

- [ ] T018 [US1] bench/src/bin/tick.rs: `kmr_api::robot!(XmBench);` once (FR-004 (SRS FR-6.1)); read the plan (T008); set `EPOCH`; calibrate; `setup::base(dt)`; register `start` (`First`, first), `noop` in `First`, `EachTick` and `Last`, `work` (`EachTick`), `end` (`Last`, last); no `add_plugins` (FR-006 (SRS FR-6.3)); `run()`; calibrate again; read VmHWM, `[stack]` Rss and timer slack. `Err` → `failed`, `reason` = the error's `Display`; `Ok` with `ticks_done < ticks` → `incomplete`; else `complete` with flags `simulated`, `uncalibrated`, `alloc_seen` as they apply. `[figures]`: `jitter_p50_ns`, `jitter_p99_ns`, `jitter_max_ns`, `bench_overruns`, `gap_on_time_min_ns`, `gap_on_time_p50_ns`, `gap_after_late_min_ns`, `vm_hwm_kb`, `stack_kb`, `loop_thread_allocs`, `locks`. `[run]` also gets `robot = "XmBench"`, `robot_n` (the `N` that `robot!` emits), `buses = "none"`, `controllers = "probe+noop/first,each,last"` and `timer_slack_ns`, for `bench_run` to copy. Nothing on stdout; `ExitCode` 0 or 1; no `unwrap`, no panic (FR-003, FR-008, FR-012, FR-019 own count, FR-024 (SRS NFR-8, NFR-1, FR-3.3, NFR-2))
- [ ] T019 [US1] bench/src/plan.rs and bench/src/probe.rs: test-only `KMR_BENCH_INJECT_AT=<tick>`, compiled under `cfg(debug_assertions)` only, so the release binaries the scripts run cannot inject; `work` allocates one `std::ffi::CString` at that tick (research R21 P4). Not in contracts/commands.md (reported) (FR-012 (SRS NFR-1)). Blocked on T005
- [x] T020 [P] [US1] bench/src/results/parse.rs: reads one report into fixed arrays (256 keys, 128-byte keys, 512-byte text, 64 KiB file); the three blocks; unknown keys kept ("Adding a key is compatible"); a bad line is exit 1 naming file and line (contracts/results-schema.md "Tools"). Unit tests on fixture text: a good file, each limit, a bad line, a missing `[meta]`
- [x] T021 [US1] bench/src/results/summary.rs and bench/src/bin/results.rs (`summary <dir>`): one set from one commit and host; refuses only a differing `schema`, `host_id` or `robot` (exit 2, `refused: <key> differs: <A value> vs <B value>`), and a set that mixes `reference = "yes"` and `"no"` (spec US2 scenario 2; see T035); every cell side by side (non-RT and PREEMPT_RT, idle and loaded), then ceiling, budget, footprint; lists excluded files (not `complete`, `throttled`, `inconsistent`) with the reason; names any of (a) to (e) missing from the set and any FR-002 `[meta]` key missing from a file (SC-004); prints `floor_held = "yes"` as "ceiling at least 100 kHz (floor held)"; exit 0, no verdict. Unit tests (US1 scenario 2; FR-023 (SRS NFR-11))
- [ ] T022 [US1] bench/scripts/cell.sh `<idle|load>`: cell `<nonrt|rt>-<idle|load>` from `kernel_rt`; `bench_run` of `tick` with `TICKS` and `WARMUP`; `load` starts `LOAD_CMD` 5 s before and stops it after, and `[meta]` `load_command` holds its exact text (else `"none"`); measurement `jitter` or `load` (data-model.md "Measurement"); one report plus its `.log` (FR-008, FR-009 (SRS NFR-8 b, c))
- [ ] T023 [US1] Validate on the dev host: `cargo test -p kmr_bench` (T017 green; it needs T005 through T014), then `cell.sh idle` with `TICKS=6000` and the four operator fields set to dev values: `not_reference`, `sim_warning_count = 1`, `results summary` reads it. Paste the exact output into the PR (quickstart §2, §3, §4)

**Checkpoint**: one simulated cell gives a complete report on any host.

---

## Phase 4: User Story 2, reproduce a published run (Priority: P1) (plan A-6 `setup-check.sh`, A-7, A-9)

**Goal**: a stranger with the same hardware gets a comparable report from the procedure alone.
FR-001, FR-021, FR-025 (SRS NFR-1, NFR-8, NFR-5). The full set waits on US3 and US4 (Phase 9).

**Independent Test**: quickstart.md §7 on the Pi; on a dev host every report is `not_reference`.

- [x] T024 [P] [US2] bench/scripts/setup-check.sh: checks the host against SRS NFR-1 and the procedure: revision `…3114` or `…3115` (stop "no revision source" when neither source gives one), Arch Linux ARM, kernel 6.x with 6.14 ≤ v < 7.0, governor `performance` on every core, the four operator fields, `RUST_LOG` unset, `stress-ng`, `chrt`, `taskset`, `size`, `strip` present, root. Stdout only; exits non-zero naming the first failing field (FR-001, FR-024 (SRS NFR-1, NFR-2); quickstart §7)
- [ ] T025 [US2] bench/PROCEDURE.md, first version: the image carrying both kernels (research R18: a PREEMPT_RT build on the Pi takes hours), the kernel configuration of both (PREEMPT_RT is mainline since 6.12), governor, `chrt -f 80`, `taskset -c 3`, core isolation off, `RUST_LOG` unset, the four operator fields and their instruments, the load profile, run lengths (profile.env), the cell commands, "build once, run the same binary on both kernels" (FR-025), where reports go and that the human commits them (FR-021 (SRS NFR-8)). T028, T033, T040, T043, T059 add their sections
- [ ] T026 [P] [US2] HUMAN (CI workflow edit, as core T053, T059, controllers T018, T027 and kmr_adapters T053). .github/workflows/ci.yml, job `cross`: install `gcc-aarch64-linux-gnu` and `gcc-arm-linux-gnueabihf`, set `CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER` and `CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER`, run `cargo build -p kmr_bench --release --bins` for both targets, no `hardware` feature (FR-025 (SRS NFR-5); plan A-9; research R21 P11: the armv7 link was never tried). kmr_adapters T053 edits the same job: merge with it, do not overwrite

---

## Phase 5: User Story 3, tick-rate ceiling (Priority: P2) (plan A-6 `ceiling.sh`)

**Goal**: the highest rate held with zero overruns, and whether the 10 µs floor holds. FR-007
(SRS NFR-8 a, FR-3.1), SC-008.

**Independent Test**: one `ceiling.sh` gives one summary report with `floor_held`.

- [ ] T027 [US3] bench/scripts/ceiling.sh: tries 10 µs first, then 1 ms, then bisects `dt` on a log scale between them, to 1 % of `dt`; one `bench_run` of `tick` per candidate, held `HOLD_S` (`ticks` = `HOLD_S` × 10⁹ / `dt` + `WARMUP`), measurement `ceiling`, cell `<nonrt|rt>-dt<dt_ns>`, one report each. A candidate holds when it is `complete` with `bench_overruns = 0` (T048 adds the engine's count). Never below 10 µs (research R10). 10 µs holds: stop, `floor_held = "yes"`. 1 ms fails: `floor_held = "no"` and no `ceiling_dt_ns` (absent key; research R10 calls it a finding, the plan names no output: reported). Summary report, cell `<nonrt|rt>-idle`, `[meta]` and `[run]` copied from the last candidate: `ceiling_dt_ns`, `ceiling_hz`, `floor_held`, and the held candidate's `gap_on_time_min_ns`, `gap_on_time_p50_ns` as `gap_ceiling_min_ns`, `gap_ceiling_p50_ns` (FR-007, FR-010 second view (SRS NFR-8 a, FR-3.1); SC-008)
- [ ] T028 [US3] bench/PROCEDURE.md § ceiling; run `ceiling.sh` on the dev host with `HOLD_S=2`: a summary report appears; record the result in the PR (not a kmr figure, SRS NFR-1)

---

## Phase 6: User Story 4, engine overhead and footprint (Priority: P2) (plan A-4 `minimal`, `baseline`; A-6 `budget.sh`, `footprint.sh`)

**Goal**: overhead per tick by the budget method, the 1 kHz controller budget, binary size, peak
RSS, stack high-water mark and the loop-thread allocation count. FR-010, FR-011, FR-012 (SRS
NFR-8 d, e, NFR-11, NFR-1).

**Independent Test**: one `budget.sh` and one `footprint.sh` on the dev host.

- [x] T029 [P] [US4] bench/src/bin/minimal.rs: `robot!(XmBench)`, three no-op controllers (`First`, `EachTick`, `Last`) and a stop counter that raises `shutdown!` after `KMR_BENCH_TICKS`; no histograms; after `run()`, VmHWM and `[stack]` Rss into its report (research R11, R21 P10) (FR-011 (SRS NFR-8 e))
- [x] T030 [P] [US4] bench/src/bin/baseline.rs: never builds a robot or calls `run()`; reads VmHWM and `[stack]` Rss and writes `stack_baseline_kb` and `vm_hwm_baseline_kb` to its report (research R11)
- [ ] T031 [US4] bench/scripts/footprint.sh: one release build; for `minimal` and `tick`, file size, stripped size (strip a copy), `size` sections, then a short run for VmHWM and `[stack]` Rss; `baseline` for the baseline pages. One report `footprint-<cell>-<n>.toml`: the unprefixed keys of data-model.md (`file_bytes`, `stripped_bytes`, `text`, `data`, `bss`, `vm_hwm_kb`, `stack_kb`) are `minimal`'s, the engine alone; `tick`'s carry the prefix `tick_`, plus its `bench_storage_bytes`; then `stack_baseline_kb` (data-model gives one key set for two binaries: reported). When `stack_kb = stack_baseline_kb`, the report says "below the first pages" (research R11) (FR-011 (SRS NFR-8 e))
- [ ] T032 [US4] bench/scripts/budget.sh `[--write all]`: at `dt` = 1 ms, bisects `KMR_BENCH_WORK_NS` for the largest workload held `HOLD_S` with zero overruns, to 1 µs; one process per candidate (research R8: a failed window pollutes the next one); a report per candidate, cell `<nonrt|rt>-w<work_ns>`. Summary report, cell `<nonrt|rt>-<write>`, `[meta]` and `[run]` copied from the held candidate: `budget_ns`, `overhead_ns` = `dt − budget_ns`, `method = "budget"`, the held candidate's calibration and `probe_cost_ns`, `uncalibrated` when it drifted; plus one `tick` run at `dt` = 10 µs whose `gap_after_late_min_ns` becomes `gap_floor_min_ns` (research R9). `--write all` sets `KMR_BENCH_WRITE=all`; `--hw` is T059 (FR-010 (SRS NFR-8 d, NFR-11))
- [ ] T033 [US4] bench/PROCEDURE.md § budget and § footprint; on the dev host with `HOLD_S=2`, `budget.sh` three times and `footprint.sh` once; record the output in the PR

---

## Phase 7: User Story 5, compare two builds (Priority: P2) (plan A-5 `compare`)

**Goal**: every figure of two builds side by side, with its difference and the spread of the
repeats; a refusal across hosts or robots. FR-022, FR-023 (SRS NFR-11, NFR-8), SC-009.

**Independent Test**: quickstart.md §5.

- [x] T034 [P] [US5] Unit tests in bench/src/results/compare.rs, red first: refusal on `schema`, `host_id`, `robot`, `robot_n`, `kernel_rt`, in that order, then `reference`, each `refused: <key> differs: <A value> vs <B value>`, exit 2; exclusion of not-`complete`, `throttled`, `inconsistent`, listed with the reason; median, spread (max − min), Δ = B − A and Δ over the larger spread in permille, on 1, 2, 3 and 16 repeats; more than 16 repeats is exit 1 naming the directory
- [x] T035 [US5] bench/src/results/compare.rs per data-model.md "Comparison": two directories, 1 ≤ R ≤ 16 repeats each; refusal keys of the contract plus `reference` (spec US2 scenario 2 and the data-model flag table say a `not_reference` run is never compared with a reference one; the contract's five keys miss a same-host kernel change: reported); integers only; even count → the lower middle value; both spreads 0 → Δ/spread printed as `-` (both left open by data-model: reported); one line per figure `<key>  A <median> (±<spread>)  B <median> (±<spread>)  Δ <B−A>  Δ/spread <permille>‰`, then the excluded files; exit 0 whatever the differences (FR-022, FR-023 (SRS NFR-11, NFR-8, NFR-1); SC-009)
- [x] T036 [US5] bench/src/bin/results.rs `compare <dir A> <dir B>`; quickstart §5 on the dev host: `robot_n` 4 against 5 → exit 2, `refused: robot_n differs: 4 vs 5`; same host and robot → exit 0, one line per figure, no verdict
- [ ] T037 [US5] Hand-off, owner kmr_model T053 (HUMAN, its SC-004: `budget.sh` of T032 before and after `robot!` takes N from the model, `results compare` of T036). No bench edit. kmr_model SC-005 is T040

---

## Phase 8: User Story 6, compile time (Priority: P3) (plan A-6 `compile-time.sh`)

**Goal**: clean and incremental build times of `kmr_core` and `controllers`, dev and release, and
compile time against the registration count. FR-013, FR-014 (SRS NFR-11, §8).

**Independent Test**: quickstart.md §6.

- [x] T038 [P] [US6] bench/gen/registrations.rs.in: the `main.rs` of a generated user crate: `kmr_api::robot!(XmBench);`, `@COUNT@` no-op `fn` items and as many `.add_controller(EachTick, cN)` lines in one phase, then `.run()`; the script writes its Cargo.toml (`kmr_api` only). 123 is today's ceiling, 124 fails `E0275` (research R14, R21 P8)
- [x] T039 [US6] bench/scripts/compile-time.sh `[--host-label <l>]`: copies HEAD with `git archive HEAD | tar -x` into a `mktemp -d` directory and never touches the working tree. Per crate (`kmr_core`, `controllers`) and profile (dev, release): `clean` (`cargo clean -p <crate>`, then build), `body` (append `fn __bench_nonce() -> u32 { <nonce> }` to the crate root, then build), `reg` (`controllers` only: one more `add_controller` line). FR-014: generated crates of `REG_COUNTS` in one phase, `CARGO_INCREMENTAL=0`, dev. `REPEATS` runs each. Keys `<crate>_<profile>_<kind>_median_ms`, `_min_ms`, `_max_ms` and `reg<count>_dev_median_ms`, `_min_ms`, `_max_ms` (data-model names no keys: reported). One report `compile-<label>-<n>.toml` under bench/reports/<commit12>/<label>/ (quickstart §6), the label defaulting to `host_id`; `[meta]` from `meta.sh` like any run (FR-013, FR-014 (SRS NFR-11, §8))
- [x] T040 [US6] bench/PROCEDURE.md § compile time; quickstart §6 on the dev host: `compile-time.sh --host-label dev-x86` with the four operator fields set (quickstart §6 omits them; `meta.sh` needs them: reported); one report; `git status --short` shows only it. This labelled non-reference report is kmr_model SC-005's x86-64 figure (FR-013)
- [ ] T041 [US6] HUMAN: `compile-time.sh` natively on the reference Pi (FR-013 reference figures; SRS §8 "hardware always the RP4 setup"); commit the report. Outside SC-001's 2 hours (research R18)

---

## Phase 9: User Story 2, the reference set (a) to (e) (plan A-6 `all.sh`, A-7, A-10)

**Goal**: the first reference report set, from the procedure alone. SC-001 to SC-004, SC-008.

**Independent Test**: quickstart.md §8, §9 on the reference Pi.

- [ ] T042 [US2] bench/scripts/all.sh: `setup-check.sh` (stop on failure), `footprint.sh`, `cell.sh idle`, `cell.sh load`, `ceiling.sh`, `budget.sh`, then `results summary` of the set, then a reboot prompt naming the other kernel; it never reboots and never runs compile time (research R18) (SC-001 (SRS NFR-8))
- [ ] T043 [US2] Complete bench/PROCEDURE.md: every FR-021 item, timed per research R18 (about 40 min per kernel, about 80 min plus one reboot), a "Reference chain" section marked "assumed, pending human: D10" (filled by T059), and a checklist a person who did not write the bench follows alone (FR-021 (SRS NFR-8); SC-001)
- [ ] T044 [US2] HUMAN (plan A-10): first reference set with `all.sh` on both kernels (quickstart §7, §8), by a person who did not write the bench, timed from a flashed board (SC-001, SC-004); commit the reports under bench/reports/; state whether the 10 µs floor held (SC-008 (SRS FR-3.1)). Blocked on T005; after T042, T043
- [ ] T045 [US2] HUMAN: repeatability (quickstart §9): `cell.sh idle` and `budget.sh` three times each on one kernel; p50 and p99 within 10 %, overhead within 10 % or 2 µs (SC-002, SC-003 (SRS NFR-8 b, d)); `results compare` gives the spread, the bench gives no verdict (FR-023)

**Checkpoint**: phase A is done: reference figures exist for (a) to (e).

---

## Phase 10: Phase B, engine cross-check (US1, US3) (plan B-1)

**Gate**: core K1.1 (HUMAN, critical path step 8) and kmr_api A1 (critical path step 9).

- [ ] T046 [US1] Hand-off, owners core K1.1 (HUMAN; kmr_core/src/clock.rs, kmr_core/src/runtime.rs) and kmr_api A1 (kmr_api/src/time.rs): the six Time getters public on `kmr_api::Time`; `overrun_count()` and `accumulated_overrun()` as designed, so a late tick shows as count + 1 and its lateness as the growth (plan "Cross-module needs"). No edit here
- [ ] T047 [US1] Waits on T046. bench/src/probe.rs `start`: reads `overrun_count()` and `accumulated_overrun()` each tick into `ENGINE_COUNT_PREV`, `ENGINE_ACC_PREV`; when the count grew by one, the slot replay takes the previous tick's `late` from the growth (research R4 phase B); `ENGINE_OVERRUNS_AT_W` at `First` of tick W, `ENGINE_OVERRUNS_AT_END` at `First` of tick K−1. bench/src/bin/tick.rs writes `engine_overruns` (their difference) and `time_surface = "engine"`; flag `inconsistent` when simulated and `engine_overruns ≠ bench_overruns`, both printed. FR-019 asks for all six getters: at `First` of tick K−1 the probe also stores the tick index, `dt`, elapsed time and last overrun, written raw as `engine_tick`, `engine_dt_ns`, `engine_elapsed_ns`, `engine_last_overrun_ns`; no figure depends on them, since `elapsed()`'s origin and `last_overrun()`'s meaning are open (core design open choices 5, 6; the plan reads two getters only: reported). A grid.rs unit test for the growth rule (FR-019 (SRS FR-3.3); SC-005)
- [ ] T048 [US3] Waits on T047. bench/scripts/ceiling.sh: a candidate holds only when `bench_overruns` and `engine_overruns` are both 0 (research R10)
- [ ] T049 [US1] Waits on T047. bench/tests/tick_runs.rs: add the `dt` = 10 µs case; quickstart §10 at 1 ms and 10 µs: `engine_overruns = bench_overruns`, no `inconsistent` (SC-005)

---

## Phase 11: Phase B, `dt` range check (US3) (plan B-2)

**Gate**: kmr_api A6 (interim check in `finalize()`) or core K3.2 (HUMAN) through kmr_api C6.

- [ ] T050 [US3] Waits on kmr_api A6. bench/tests/tick_runs.rs: `KMR_BENCH_DT_NS=5000` gives exit 1, `failed`, `reason` = the one-line `UserError` text (kmr_api A6: `Debug` = `Display`); until A6 lands the case expects `complete` with `dt_ns = 5000` (research R21 P4); `ceiling.sh` unchanged (quickstart §4; FR-024, FR-007 (SRS NFR-2, FR-3.1))

---

## Phase 12: Upstream follow-ups (no new measurement)

- [ ] T051 Waits on kmr_api C4 (`set_limit_policy_all`, kmr_api T038) and core K5.2, K5.3 (HUMAN). bench/src/setup.rs `base()`: add `.set_limit_policy_all(Policy::Warn)` (decided 2026-10-28 (for now): D2 B, the one place; research "Assumed decisions"). It lands with kmr_api C4 (T038), not C6: from T038 on, `finalize()` hands limits and policies to the core, which refuses an actuator without a policy (core K5.3), else every bench run is `failed`. The probe never moves a motor, so no limit path runs
- [ ] T052 Waits on core K1.6 (HUMAN) and kmr_api A0. Rerun bench/tests/tick_runs.rs and quickstart §2: exactly one "no bus plugin: simulating", now kmr_core FR-042's, at `open`. Between K1.6 and A0, two lines print and every simulated run is `inconsistent`: expected, no bench change. `bench_run` counts both streams, so a tracing line on stdout still counts (research R3)
- [ ] T053 Waits on core M2 K2.2 and K2.6 (HUMAN; **kmr_core/src/state/field.rs**, **kmr_core/src/controllers.rs**, **kmr_core/src/schedule.rs**, **kmr_core/src/schedule/macros.rs**) and kmr_api B7 (K2.8). bench/src/probe.rs: each probe controller generic, `fn start<const M: usize>(_: &Time, _: &mut State<'_, M>, _: &Sensors)`, `write_target()` over `[Q; M]` (data-model.md "Probe controllers"); the binaries keep `robot!(XmBench)`
- [ ] T054 Hand-off, HUMAN, owner core K3.9 (kmr_core/src/runtime.rs): no per-tick log line at any `RUST_LOG` value and no stdout write inside a tick (plan "Cross-module needs"; research "Findings for other owners"). No bench edit; `RUST_LOG` stays unset in the procedure. Reports before and after core K3.10 (real-time sleep) compare with `results compare` (spec Assumptions "Results")

---

## Phase 13: User Story 7, hardware I/O (Priority: P3) (plan A-6 `presort.sh` fallback, C-1 to C-3)

**Goal**: per-tick I/O cost of the reference chain next to engine overhead, read ticks apart.
FR-015, FR-016, FR-017 (SRS FR-5.3, FR-5.10, NFR-11), FR-005 (SRS FR-6.3).

**Independent Test**: quickstart.md §11.

**Gate** (T057 to T060): delivery step 1: core M1, K1.1 to K1.10 (HUMAN; K1.3 touches
**kmr_core/src/state/field.rs**), kmr_api A5, kmr_adapters T045; age: core K3.5 and kmr_api C1;
cadence: core K3.8 and kmr_api C3; declared costs: kmr_adapters T067 and kmr_api C7.

- [ ] T055 [US7] bench/scripts/presort.sh (cost, D11 A fallback, starts now): `budget.sh --write all` against `budget.sh` on the same build; `presort_cost_ns` = the overhead difference, labelled an upper bound that holds the mask, batch and clear (research R15); report `presort-<cell>-<n>.toml` (FR-016 (SRS FR-5.3)). Needs the engine to build the batch on the no-hardware adapter (core design §2 #9; plan "Cross-module needs")
- [ ] T056 [US7] bench/src/bin/tick_hw.rs (feature `hardware`, plan C-1): `tick` plus `.add_plugins(Bus::by_id(SERIAL))`, `const SERIAL: &str = "FT8ISZ6H";` (contracts/commands.md "Hardware source line"; assumed, pending human: D10, the one place with PROCEDURE.md "Reference chain"); no bus label (one bus, D6); `robot!(XmBench)` as `tick` (FR-004, FR-005 (SRS FR-6.1, FR-6.3)); shared code in the lib, so `SERIAL` is the only line two benches differ in (US2 scenario 3). `[run]` gets `buses = "by_id:<SERIAL>"`. Builds today where ../dxl_rs and libclang are present; CI cannot build it before kmr_adapters T052, T053. bench/scripts/profile.env gets today's text of kmr_api's interim message, "does not drive hardware yet" (kmr_api/src/robot.rs:267; the spec's wording "plugins declared but hardware not driven yet" does not match it, and no contract pins the text: reported): `bench_run` then marks the run `simulated` and drops every I/O figure (FR-006; kmr_api FR-026)
- [ ] T057 [US7] Waits on core K3.5 (HUMAN) and kmr_api C1 (`State::age_at`). bench/src/probe.rs `start`: a read tick when any actuator's age at `First` is `Some(0)`, else write-only; `LATENESS_READ`, `LATENESS_WRITE_ONLY`, `BENCH_OVERRUNS_WRITE_ONLY`; tick_hw writes `read_*` and `write_only_*` of each jitter figure, read-tick overruns apart and not judged; the engine count is reported and the bench count is a lower bound (FR-008 (SRS FR-3.4), FR-019 (SRS FR-3.3))
- [ ] T058 [US7] Waits on core K3.8 (HUMAN), kmr_api C3 (`State::read_every`), kmr_adapters T067 and kmr_api C7 (`READ_COST`, `WRITE_COST` under `kmr_api::presets`). bench/src/bin/tick_hw.rs writes `read_cadence`, `declared_read_ns`, `declared_write_ns` to `[run]` for `bench_run` to copy into `[meta]`; until C7 the procedure records the declared costs by hand. The bench never sets the cadence (assumed, pending human: D9 A)
- [ ] T059 [US7] Waits on delivery step 1 (gate above) and T057. bench/scripts/budget.sh `--hw`: `tick_hw` with `--write all`; "held" counts write-only ticks only; `io_write_tick_ns` = budget(simulated) − budget(hardware) of the same robot; `io_read_ns` = lateness p50 of read ticks − lateness p50 of write-only ticks; engine overhead next to the I/O cost, no verdict (research R16; FR-015, FR-017 (SRS FR-5.3, FR-5.10, NFR-11)). bench/PROCEDURE.md "Reference chain": XmBench (4× XM430-W210, one bus, 1 Mbaud), one ROBOTIS U2D2, its serial id, the supply (assumed, pending human: D10)
- [ ] T060 [US7] HUMAN (plan C-3): hardware runs on the reference chain (quickstart §11): `cell.sh` and `budget.sh --hw --write all` on `tick_hw`; after every run `Torque_Enable` = 0 on ids 1 to 4 (coordination §3 step 11 check); commit the reports; hand the write-tick and read costs to kmr_adapters T063 for `DEFAULT_READ_EVERY` (D9 A). After T056 to T059

---

## Phase 14: Switches and bridges (US7, US1, US5) (plan D-1 to D-5)

- [ ] T061 [US7] Waits on core K6.3 (HUMAN; `presort-off` changes `Batch::new` only) and kmr_api C7. bench/Cargo.toml feature `presort-off = ["kmr_api/presort-off"]`; bench/scripts/presort.sh: cost = `budget.sh --write all` on the default build against the `presort-off` build; `--gain`: the same two builds with `--hw` on the reference chain, then `results compare` (assumed, pending human: D11 B, the one place: this feature and presort.sh) (FR-016 (SRS FR-5.3, §8))
- [ ] T062 [US7] Optional. Waits on kmr_api A3 (`.bus(label)`, D6) and kmr_api B1 (hardware argument, D7 A). bench/src/bin/tick_hw.rs feature `two-bus`: XmBench over two U2D2, actuators alternating, two `Bus::by_id(..).bus(..)` lines (research R15; assumed, pending human: D6, D7)
- [ ] T063 [US7] HUMAN (plan D-2): `presort.sh --gain` on the reference chain (expected zero on XmBench, research R15); optionally the two-bus chain (T062); commit the reports. After T060, T061
- [ ] T064 [US1] Waits on kmr_api Z2 (feature `zenoh`). bench/Cargo.toml feature `zenoh = ["kmr_api/zenoh"]`; bench/src/bin/tick.rs adds kmr_api's gateway plugin with its default-deny settings and no peer, under `cfg(feature = "zenoh")`; `[meta]` `features` records it; `KMR_BENCH_TICKS=1001000` for kmr_api SC-003. The bench opens no endpoint of its own (constitution VI) (FR-026 (SRS FR-7.1); research R20)
- [ ] T065 [US5] Waits on kmr_api G1 (kmr_api T056, feature `ffi`; kmr_ffi C2 hands off to it), kmr_ffi B1 and B2 (build helper; B2 waits on kmr_model T059, held until D12). bench/Cargo.toml feature `ffi`; bench/src/bin/tick_ffi.rs: `tick` with an empty C forwarder as the `EachTick` controller; bench/build.rs compiles the foreign code only and names the robot again with the const check (assumed, pending human: D12 A); the C source at bench/ffi/empty.c (the plan names no path); compared with `tick` through `results compare` (FR-022 (SRS NFR-11); kmr_ffi SC-003, its plan E3)
- [ ] T066 [US5] HUMAN (plan D-5): runs of T064 and T065 on the reference bench; commit the reports; hand them to kmr_api Z4 (SC-003, SC-010) and kmr_ffi E3 (SC-003)

---

## Phase 15: Polish and cross-cutting

- [ ] T067 [P] docs/decisions/NNNN-bench-method.md, NNNN the next free number in docs/decisions/ at commit time (plan A-8; 0001 and 0002 exist): black-box through kmr_api only, the budget method and calibration, the slot replay and the overrun deadline (research R5: tick k's deadline is `origin + (k+1)·dt`), the histogram, the counting allocator, one process per run, the report format, no verdict; the SRS problems of research.md for the human (SRS NFR-6; constitution V). HUMAN accepts it
- [ ] T068 [P] docs/c4/bench.dsl with `/speckit-ov-c4-code` (constitution, Development Workflow)
- [ ] T069 Review bench/src: `unsafe` only in bench/src/alloc.rs; no heap type, no `dyn`, no lock, no stdout write in a controller; `size -A` on `tick` shows the histograms in `.bss` (research R21 P5) (FR-018 (SRS NFR-1))
- [ ] T070 Run quickstart §1, `cargo fmt --all --check`, `cargo clippy --workspace --all-targets`, `cargo test --workspace --no-fail-fast` (constitution, Technical Constraints)

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (T001–T004)** first. T002 needs the maintainer's authorization for a new crate.
- **Foundational (T005–T016)** after T004. T005 is HUMAN and blocks T014, T019, T017 case d and
  T044. T006–T012, T015 are parallel; T013 after T006; T014 after T007, T009, T012, T013, T005;
  T016 after T015.
- **US1 (T017–T023)** after Phase 2. T017 red first; T018 after T008, T011, T014; T019 after
  T018; T021 after T020; T022 after T016, T018.
- **US2 first part (T024–T026)** after US1 (T025 names the cell commands). T024, T026 parallel;
  T026 is HUMAN (ci.yml).
- **US3 (T027–T028)**, **US4 (T029–T033)** after US1; independent of each other.
- **US5 (T034–T036)** after T020, T021. T037 is a hand-off to kmr_model T053.
- **US6 (T038–T041)** after T015, T016; needs no bench binary. T041 HUMAN.
- **Reference set (T042–T045)** after US1, US3, US4 and T024, T025. T044, T045 HUMAN.
- **Phase B (T046–T050)**: waits on core K1.1, kmr_api A1 (T047–T049); kmr_api A6 (T050).
- **Follow-ups (T051–T054)**: each waits on the id it names. T051 with kmr_api C4 (T038).
- **US7 (T055–T060)**: T055 after T032; T056 after T018; T057–T059 wait on the phase gate; T060
  HUMAN last.
- **Phase D (T061–T066)**: each waits on the id it names; T063, T066 HUMAN.
- **Polish**: T067, T068 after phase A; T069, T070 last.

### Within a phase

- Tests before the code that turns them green (T017 before T018, T019; T034 before T035).
- hist.rs (T006) before the statics (T013) before the controllers (T014).
- meta.sh (T015) before `bench_run` (T016) before every script.

### Blocked tasks (today)

- On HUMAN T005: T014, T019, T017 case d, then everything that runs `tick`.
- On other modules: T046–T049 (core K1.1, kmr_api A1), T050 (kmr_api A6), T051 (kmr_api C4,
  core K5.2, K5.3), T052 (core K1.6, kmr_api A0), T053 (core M2, kmr_api B7), T057–T060
  (delivery step 1, core K3.5, K3.8, kmr_api C1, C3, C7, kmr_adapters T045, T067), T061 (core
  K6.3, kmr_api C7), T062 (kmr_api A3, B1), T064 (kmr_api Z2), T065 (kmr_api G1, kmr_ffi B1,
  B2). T037 is a hand-off to kmr_model T053.

## Parallel opportunities

- T006–T012 and T015 (one file each) after T004.
- T020 alongside T018; T024 and T026 together; T029 and T030 together.
- US3, US4, US5 and US6 side by side once US1 is done.
- T067 and T068 together.

```text
Foundational:  T004 → (T006 ‖ T007 ‖ T008 ‖ T009 ‖ T010 ‖ T011 ‖ T012 ‖ T015) → T013 → T014 (+T005) ; T015 → T016
US1:           T017 → T018 → T019 ; T020 → T021 ; T022 → T023
After US1:     US3 (T027, T028) ‖ US4 (T029–T033) ‖ US5 (T034–T036) ‖ US6 (T038–T040)
Reference set: T042 → T043 → T044 (HUMAN) → T045 (HUMAN)
```

## Implementation strategy

### MVP: User Story 1

1. Setup, then Foundational, with HUMAN T005 early.
2. US1: one simulated 1 kHz cell gives a complete report on any host.
3. Stop and validate (T023).

### Incremental, phase A

4. US2 first part (host check, procedure, cross-build), then US3 and US4, then US5 and US6.
5. Reference set: `all.sh`, the full procedure, the HUMAN runs on both kernels.

### Later phases, as upstream lands

6. Phase B after core K1.1 and kmr_api A1; the range check after kmr_api A6.
7. Follow-ups as their ids land (T051 with kmr_api C4, T038).
8. US7 hardware after delivery step 1; phase D switches and bridges last.
9. Polish, then the PR from `agent/<topic>` into the human branch, `agentic` label.

## Notes

- Commits: AGENTS.md rule (b) and the human's final decision: no AI or co-author trailer. The
  constitution's Principle 0 asks for one. This conflict is the human's to settle; until then
  commits follow the human's decision. Commit per task or group, Conventional Commits, scope
  `bench`.
- Delivery order (constitution I, SRS §8): phase A starts before step 1 moves a motor, under D1 A
  (assumed, pending human). The gate lives in plan.md "Order of work", column "Gate".
- An agent that reaches a HUMAN task, or a task waiting on one, stops and hands it over.
- The debug-only allocation of T019 is test code: release binaries never allocate in a tick.
- 2026-11-02: T003 stays open. bench/Cargo.toml has all but the `tick_hw` `[[bin]]` entry, which
  Cargo refuses while bench/src/bin/tick_hw.rs does not exist; it lands with T056. The dependency
  is `kmr_api.workspace = true`, the workspace's own path entry for `../kmr_api`.
- 2026-11-02: T015 is done with one deviation. With no revision source, meta.sh writes
  `board_revision = "none"` and `reference = "no"` instead of stopping: FR-001 and quickstart §2
  want a `not_reference` report from any other host, and T023 runs on the dev host. The stop on
  a missing revision source is setup-check.sh's (T024, contracts/commands.md). Pending human.

## Analysis 2026-10-28 (cross-module)

- 2026-10-28: T026 HUMAN (ci.yml); T037 one-line hand-off to kmr_model T053, the one owner of kmr_model SC-004; T051 lands with kmr_api C4 (T038), not C6; T065 waits on kmr_api G1 (T056) and names kmr_model T059 for kmr_ffi B2; T067 takes the next free record number instead of 0005.
