# Implementation Plan: bench, the reference benchmark of kmr v3

**Branch**: `007-bench` (feature directory; no branch hook, the git branch is the human's choice)
| **Date**: 2026-10-28 | **Spec**: [spec.md](spec.md)

**Input**: `specs/007-bench/spec.md` (coordinated 2026-10-28), SRS module `bench` (NFR-1, NFR-8 a
to e, NFR-11), `specs/spec-coordination-2026-10-28.md`, the core designs
(`specs/002-kmr-core/design-const-generic-n.md`, `design-first-motor.md`), today's code. No
`bench/` code exists yet.

## Summary

A new workspace member, `bench/` (package `kmr_bench`), measures kmr from the outside. It
depends on `kmr_api` only, so it can read nothing the public API does not give (FR-003 (SRS
NFR-8)). Probe controllers stamp each tick into fixed `static` histograms; the results leave
after `run()` returns, into a plain TOML report per run. Bash scripts fix the host setup, drive
one process per run, and record the metadata of FR-002 (SRS NFR-1, NFR-8). A `results` tool
compares two sets of reports without a verdict (FR-022, FR-023 (SRS NFR-11)).

The method, in one line each:

- Jitter: start lateness against a replayed deadline grid (research R4).
- Overrun: the bench's own count; from phase B checked against the engine's (R5).
- Overhead: the budget method, `dt` minus the largest workload held with zero overruns (R8).
- Ceiling: one process per `dt` candidate, log bisection down to the 10 µs floor (R10).
- Footprint: `/proc` after `run()`, on a minimal binary (R11).
- Compile time: a copy of the commit, non-incremental builds, a registration sweep (R14).
- Hardware I/O: simulated budget minus hardware budget, read ticks apart (R16).

Work runs in four phases. Phase A (the simulated harness) needs nothing but the workspace
member line. Phases B, C and D wait on the engine features they read. A scratch prototype proved
phase A's risky parts on today's API (research R21: P1 to P12).

The two open points of the spec are planned on the recommended options: the reference chain
(FR-015, D10) and the pre-sort gain (FR-016, D11). Both are isolated (research "Assumed
decisions").

## Technical Context

**Language/Version**: Rust 1.97.1, edition 2024, stable only; Bash for the run scripts

**Primary Dependencies**: `kmr_api` only (Cargo features forwarded: `hardware` →
`kmr_api/dynamixel`; later `presort-off`, `zenoh`, `ffi`). Host tools: `stress-ng`, util-linux
(`chrt`, `taskset`), binutils (`size`, `strip`), `vcgencmd` when present

**Storage**: one report file per run, `bench/reports/<commit12>/<host_id>/*.toml`, plus kmr's
output as `.log` beside it. In process: zero-initialized `static` atomics and histograms

**Testing**: `cargo test -p kmr_bench` (histogram, slot replay against the engine's rule, plan
parsing, results parser, compare rules); the [quickstart](quickstart.md) scenarios; reference
runs by the human on the Pi

**Target Platform**: Raspberry Pi 4 (rev 1.4 or 1.5), aarch64 Arch Linux ARM, kernel 6.x ≥
6.14, non-RT and PREEMPT_RT (SRS NFR-1). Also builds for x86-64 and armv7 Linux (FR-025 (SRS
NFR-5))

**Project Type**: measurement harness: a library, five binaries, scripts, a written procedure

**Performance Goals**: none; the SRS defers every figure (SRS NFR-1). Repeatability only:
SC-002 (p50, p99 within 10 %), SC-003 (overhead within 10 % or 2 µs). The probe's own cost is
measured and reported (`probe_cost_ns`)

**Constraints**: black-box (FR-003); no heap, no `dyn`, no lock, no I/O inside a tick (FR-018
(SRS NFR-1)); root `clippy.toml` heap ban; one `run()` per process (process-wide Ctrl-C
handler); `RUST_LOG` unset (research R12); reports never on stdout (research R3)

**Scale/Scope**: N = 4 (XmBench); 601 000 ticks per 1 kHz cell; histograms of 20 000 buckets
(80 KB each, `.bss`); up to 256 keys per report; up to 16 repeats per side in a comparison

## Constitution Check

*Gate before Phase 0, re-checked after Phase 1.*

| Principle | Status | Note |
|---|---|---|
| 0. ov policy | Pass, one conflict recorded | `bench/` is not in `.aiignore`. Every `kmr_core` change the bench needs is a HUMAN task. **Conflict, not fixed**: Principle 0 requires an agent trailer on agent commits; AGENTS.md rule (b) and the human's standing decision forbid it. This plan makes no commit |
| I. SRS is the authority | Pass, one conflict recorded | Every requirement cites its SRS id. Open questions are isolated (research "Assumed decisions"). **Conflict, not fixed**: SRS §8's delivery gate (D1) has no bench step. Phase A starts before step 1 moves a motor, assuming the bench is tooling (decided 2026-10-28: D1 A). One place: "Order of work", column "Gate" |
| II. Real-time hot path | Pass | Probe controllers: no heap, no `dyn`, no lock, relaxed load and store only (research R6). The crate keeps the root heap ban (R2) |
| III. No panics | Pass | Binaries return `ExitCode`; `#![deny(clippy::unwrap_used)]`; a failed run is a report, not a panic (FR-024) |
| IV. Hand-engineered core | Pass | Depends on `kmr_api` only. No bench need touches an invariant file (`state/field.rs`, `sealed.rs`, `signal.rs`, `schedule.rs`, `controllers.rs`). The `unsafe` counting allocator is HUMAN (R7) |
| V. Measured, not guessed | Pass | This module is the measurement. Method defended in `docs/decisions/0005-bench-method.md` (task A-8, SRS NFR-6) |
| VI. Secure by default | Pass | The bench opens no endpoint. The Zenoh run uses `kmr_api`'s default-deny gateway (R20) |
| Technical constraints | Pass | Integer nanoseconds, no `f32`/`f64`. `bench/` maps to the SRS module `bench`. aarch64 proven, armv7 checked (R21 P11). Joins workspace fmt, clippy and test |
| Development workflow | Pass, note | A new crate is "a new module": an agent writes it only under a maintainer's recorded authorization, on an `agent/` branch, PR into the human branch (AGENTS.md "Agentic work") |

Post-design re-check: no new violation. The one justified exception is in Complexity Tracking.

## Project Structure

### Documentation (this feature)

```text
specs/007-bench/
├── spec.md              # input (not edited here)
├── plan.md              # this file
├── research.md          # Phase 0: decisions, prototype results, SRS problems
├── data-model.md        # Phase 1: plan, probes, statics, status, report, comparison
├── quickstart.md        # Phase 1: validation scenarios
├── contracts/
│   ├── commands.md      # scripts, binaries, environment, exit codes
│   └── results-schema.md# the report file and the results tool
└── tasks.md             # /speckit-tasks, not created here
```

### Source Code (repository root)

```text
Cargo.toml                          # + "bench" in members (A-1)
bench/
├── Cargo.toml                      # package kmr_bench; kmr_api only; features added as upstream lands
├── PROCEDURE.md                    # FR-021: one command per measurement, full setup
├── build.rs                        # phase D, feature ffi only: compiles the empty C forwarder (D12 A)
├── src/
│   ├── lib.rs
│   ├── alloc.rs                    # HUMAN (unsafe): counting allocator, window flag
│   ├── hist.rs                     # Histogram
│   ├── grid.rs                     # slot replay and overrun judgment, pure functions
│   ├── probe.rs                    # start, work, end, noop, write_target
│   ├── setup.rs                    # base(): dt and the joint-limit policy (D2)
│   ├── plan.rs                     # RunPlan from the environment
│   ├── calib.rs                    # spin, calibrate, probe cost
│   ├── procfs.rs                   # VmHWM, [stack] Rss, timer slack; chunked scan
│   ├── report.rs                   # [run] and [figures] writer, fixed buffers
│   ├── results/                    # parse.rs, compare.rs, summary.rs
│   └── bin/                        # tick.rs, tick_hw.rs, minimal.rs, baseline.rs, results.rs (+ tick_ffi.rs)
├── scripts/                        # profile.env, meta.sh, setup-check.sh, cell.sh, ceiling.sh,
│                                   # budget.sh, footprint.sh, compile-time.sh, presort.sh, all.sh
├── gen/registrations.rs.in         # template of the FR-014 generated crates
└── reports/<commit12>/<host_id>/   # committed by the human
docs/decisions/0005-bench-method.md # A-8
```

**Structure Decision**: one crate at the root, as the SRS module list says (`bench/**`). The
package is `kmr_bench`, not `bench`, so `cargo bench -p` is not misread (research R1). Scripts
hold what only the OS can do. Nothing lives in `kmr_core/`.

## Order of work

Task ids carry a hyphen (A-1); decision ids (Dn) do not. HUMAN rows are the maintainer's.

| Phase | Gate (one place for D1) |
|---|---|
| A. Simulated harness | The `bench` member line. Assumed, pending human: D1 (the bench is tooling, not a delivery step) |
| B. Engine cross-check | core K1.1 (Time getters); `kmr_api` A1 (forwarding) |
| C. Hardware | Delivery step 1 (core M1, `kmr_api` A5, `kmr_adapters` A to D); core K3.5 (age) and K3.8 (cadence), read through `kmr_api` |
| D. Switches and bridges | core K6.3 (`presort-off`) and its `kmr_api` forward; `kmr_api` Z2 (Zenoh); `kmr_ffi` C2 |

### Phase A

| # | Who | Work | Spec |
|---|---|---|---|
| A-1 | Agent | Root `Cargo.toml` member; `bench/Cargo.toml` (`kmr_api` only, feature `hardware`); `lib.rs` with `#![deny(clippy::unwrap_used)]` | FR-003 (SRS NFR-8), FR-018 |
| A-2 | **HUMAN** | `bench/src/alloc.rs`: counting `GlobalAlloc` over `System`, `const` thread-local window flag (research R7; `unsafe`) | FR-012 (SRS NFR-1) |
| A-3 | Agent | `hist.rs`, `grid.rs`, `plan.rs`, `calib.rs`, `procfs.rs`, `report.rs`, `setup.rs`, with unit tests | FR-008, FR-010, FR-011, FR-018, FR-024 |
| A-4 | Agent | `probe.rs`; binaries `tick`, `minimal`, `baseline` | FR-006, FR-007 to FR-012, FR-019 (own count), FR-020 (SRS FR-4.1) |
| A-5 | Agent | `results`: parser, `compare`, `summary`, refusal rules, tests | FR-022, FR-023 (SRS NFR-11, NFR-8) |
| A-6 | Agent | Scripts: `profile.env`, `meta.sh`, `setup-check.sh`, `cell.sh`, `ceiling.sh`, `budget.sh`, `footprint.sh`, `compile-time.sh`, `presort.sh` (cost, D11 A fallback), `all.sh` | FR-001, FR-002, FR-007 to FR-011, FR-013, FR-014, FR-016 |
| A-7 | Agent | `PROCEDURE.md`: image with both kernels, kernel config, governor, `chrt`, `taskset`, operator fields, load, run lengths, reboot | FR-021 (SRS NFR-8), SC-001 |
| A-8 | Agent, HUMAN accepts | `docs/decisions/0005-bench-method.md` (next free number) | SRS NFR-6 |
| A-9 | Agent | CI: build `kmr_bench` for aarch64 and armv7 (needs `arm-linux-gnueabihf-gcc` in CI) | FR-025 (SRS NFR-5) |
| A-10 | **HUMAN** | First reference set (a) to (e) on both kernels; commit the reports; answer the 10 µs floor | SC-001, SC-004, SC-008 (SRS FR-3.1) |

### Phase B

| # | Who | Work | Spec |
|---|---|---|---|
| B-1 | Agent | Probe reads the Time surface; slot follows the engine through `accumulated_overrun()` growth; engine count at W and K−1; `inconsistent` flag; `time_surface = "engine"` | FR-019 (SRS FR-3.3), SC-005 |
| B-2 | Agent | Below-floor scenario expects `failed` once the range check exists (core K3.2, `kmr_api` A6) | FR-024, FR-007 (SRS FR-3.1) |

### Phase C

| # | Who | Work | Spec |
|---|---|---|---|
| C-1 | Agent | `tick_hw` (feature `hardware`): `Bus::by_id(SERIAL)`, `write_target()` = `prev(0)` | FR-004, FR-005 (SRS FR-6.1, FR-6.3) |
| C-2 | Agent | Read and write-only split by age; hardware budget on write-only ticks; `io_write_tick_ns`, `io_read_ns`; cadence and declared costs in `[meta]` | FR-008 (SRS FR-3.4), FR-015, FR-017 (SRS FR-5.3, NFR-11) |
| C-3 | **HUMAN** | Hardware runs; `Torque_Enable` = 0 on every id afterwards; send read and write costs to `kmr_adapters` for `DEFAULT_READ_EVERY` (D9) | FR-015, coordination §3 step 11 |

### Phase D

| # | Who | Work | Spec |
|---|---|---|---|
| D-1 | Agent | Feature `presort-off`; `presort.sh` cost on the switch, gain with `--gain` | FR-016 (SRS FR-5.3, §8). Assumed, pending human: D11 |
| D-2 | **HUMAN** | Gain run on the reference chain (expected zero, research R15); optional two-bus chain (D7 A) | FR-016 |
| D-3 | Agent | Feature `zenoh`: kmr_api's gateway in `tick`; run length for `kmr_api` SC-003 | FR-026 (SRS FR-7.1) |
| D-4 | Agent | Feature `ffi`: `tick_ffi` with an empty C forwarder, `build.rs` compiles foreign code only (D12 A) | FR-022; `kmr_ffi` SC-003 (its plan E3) |
| D-5 | **HUMAN** | Runs of D-3 and D-4; commit the reports | FR-026, FR-022 |

## Cross-module needs

What other plans must honour for this plan to hold. None of it touches an invariant file.

| Module | Must honour |
|---|---|
| kmr_core (HUMAN) | K1.1: `overrun_count()` and `accumulated_overrun()` as designed, so the bench derives each late tick from their growth (works under either `last_overrun()` meaning). FR-042: the "no bus plugin: simulating" text exactly once per run, on any stream. The no-hardware run still builds `Batch` and walks the write path (R15). K3.2: a `dt` outside [10 µs, 1 s] is a `run()` error. K3.5 per-actuator age and K3.8 cadence, read-only. K3.9: no per-tick log line at any `RUST_LOG` value, and no stdout write inside a tick. K6.3: `presort-off` changes `Batch::new` only |
| kmr_api | A0 lands with or after the core's FR-042 warning, never before: otherwise no warning prints for a while and every simulated run is flagged `inconsistent`. A1 forwards all six getters. Forward features `presort-off`, `zenoh`, `ffi`. `UserError`'s `Display` is one line (the report's `reason`). `State` exposes per-actuator age and cadence. `robot!` keeps emitting the `Robot` and `State` aliases. Under D2 B: one call that sets a joint-limit policy for every actuator |
| kmr_adapters | `XmBench` preset (B2). The declared read and write costs (FR-035) reachable as consts through `kmr_api::presets`; otherwise the procedure records them by hand. Torque off on every id after a run (critical path step 2). Takes the bench's read and write tick costs for `DEFAULT_READ_EVERY` (D9 A) |
| kmr_model | Nothing to build. Its SC-004 is a `results compare` of the commit before and after the model. Its SC-005 figure (x86-64 dev machine) is a labelled non-reference compile-time report |
| kmr_ffi | A C entry point an empty forwarder can call; its plan E3 hands the binary to the bench. The `build.rs` rule of D12 A |
| controllers | `controllers/` stays the reference crate for compile time. D15 A (no bus line) keeps it building without hardware |
| Workspace and CI | The `bench` member line; root `clippy.toml` unchanged; an armv7 linker in CI |

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| `unsafe impl GlobalAlloc` in a non-core crate (`bench/src/alloc.rs`, HUMAN) | FR-012 (SRS NFR-1) counts allocations on the loop thread inside the window only | External counters see the whole process or a whole closure, so bring-up allocations would count (research R7) |
