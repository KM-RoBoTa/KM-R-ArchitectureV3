# Contract: bench results file, schema 1

The file one bench run writes; the spec calls it a "report". Consumers: `results compare` and
`results summary`, the human, and other modules' success criteria: kmr_core SC-012,
kmr_model SC-004 and SC-005, kmr_ffi SC-003, kmr_api SC-003 and SC-010, kmr_adapters SC-002,
SC-003 and the `DEFAULT_READ_EVERY` value. Ids: FR-002, FR-012, FR-022 to FR-024 (SRS NFR-1,
NFR-8, NFR-11, NFR-2).

## Format

A flat TOML subset, UTF-8, one file per run.

- Lines: `key = <integer>`, `key = "<text>"`, `[block]`, blank, or `# comment`.
- Integers are unsigned, base 10. Times in ns (`_ns`) or ms (`_ms`), sizes in bytes (`_bytes`)
  or kB (`_kb`), temperatures in m°C (`_mc`), voltages in mV (`_mv`).
- Text has no `"` and no newline. Lists are text with `,` separators (`flags`).
- A missing figure is an absent key, never `0`.
- Max 256 keys, 128 bytes per key, 512 bytes per text value, 64 KiB per file.
- Path: `bench/reports/<commit12>/<host_id>/<measurement>-<cell>-<repeat>.toml`. The run's
  kmr output sits next to it as `<same name>.log`.

## Blocks and keys

### `[meta]` (written by `scripts/meta.sh`, required in every file)

| Key | Example | FR-002 item |
|---|---|---|
| `schema` | `1` | |
| `host_id` | `"rpi4-c03114-u2d2-FT8ISZ6H"` | host identity |
| `board_model` | `"Raspberry Pi 4 Model B Rev 1.4"` | board |
| `board_revision` | `"c03114"` | revision |
| `distro` | `"Arch Linux ARM"` | OS |
| `kernel` | `"6.14.3-1-rpi-ARCH #1 SMP PREEMPT_RT ..."` | full kernel version |
| `kernel_rt` | `"yes"` | PREEMPT_RT or not |
| `governor` | `"performance"` | CPU governor |
| `affinity` | `"3"` | CPU affinity |
| `sched` | `"SCHED_FIFO 80"` | scheduling policy |
| `sched_rt_runtime_us` | `950000` | research R12 |
| `timer_slack_ns` | `0` | research R12 (read by the binary, copied here) |
| `rust_log` | `"unset"` | research R12 |
| `supply_mv`, `supply_meter` | `5120`, `"<meter model>"` | supply voltage and instrument |
| `ambient_mc`, `ambient_meter` | `21500`, `"<thermometer model>"` | ambient temperature and instrument |
| `throttled_start`, `throttled_end` | `"0x0"` | throttle flags |
| `soc_mc_start`, `soc_mc_end` | `48300` | SoC temperature |
| `commit` | `"76bd454..."` | commit id |
| `dirty` | `"no"` | working tree clean |
| `profile`, `features` | `"release"`, `"hardware"` | build profile and Cargo features |
| `robot`, `robot_n` | `"XmBench"`, `4` | the robot named in `robot!` and its N |
| `buses` | `"by_id:FT8ISZ6H"` or `"none"` | bus plugins declared |
| `read_cadence` | `"4"` | hardware runs: cadence the engine exposes |
| `declared_read_ns`, `declared_write_ns` | `1000000`, `350000` | hardware runs: the adapter's declared costs (kmr_adapters FR-035), when reachable |
| `controllers` | `"probe+noop/first,each,last"` | controller set |
| `load_command` | `"stress-ng --cpu 0 ..."` or `"none"` | load profile |
| `reference` | `"yes"` | FR-001 check result |

### `[run]` (written by the binary)

| Key | Rule |
|---|---|
| `measurement`, `cell` | e.g. `"jitter"`, `"rt-idle"` |
| `dt_ns`, `ticks`, `warmup`, `work_ns`, `write` | the RunPlan |
| `status` | `"complete"`, `"incomplete"`, `"failed"` |
| `flags` | subset of `simulated, not_reference, throttled, inconsistent, uncalibrated, alloc_seen` |
| `reason` | present when `status ≠ "complete"`: the `run()` error, the signal, or `bad plan: <var>` |
| `ticks_done` | always |
| `anchor` | `"tick0_start"` (research R4) |
| `time_surface` | `"absent"` (phase A) or `"engine"` (phase B on) |
| `ps_per_iter_before`, `ps_per_iter_after`, `drift_permille`, `probe_cost_ns` | calibration |
| `bench_storage_bytes` | size of the bench statics, to read the footprint |

### `[figures]` (absent when `status ≠ "complete"`)

The keys of the measurement run (see [data-model.md](../data-model.md) "Measurement"), plus in
every file:

| Key | Rule |
|---|---|
| `loop_thread_allocs` | FR-012; flag `alloc_seen` when > 0; `"not measured"` in a report with no counting window (`minimal`, `baseline`, `compile`) |
| `locks` | always `"not measured"` |
| `sim_warning_count` | added by the script from the `.log`; must be 1 on a run with no bus plugin |

## Tools

```text
target/release/results compare <dir A> <dir B>   # two builds or two controller sets
target/release/results summary <dir>             # one set, every cell side by side
```

| Exit | Meaning |
|---|---|
| 0 | printed; no verdict, whatever the differences |
| 2 | refused: prints `refused: <key> differs: <A value> vs <B value>` |
| 1 | unreadable input: prints the file and line |

`compare` prints one line per figure, aligned:
`<key>  A <median> (±<spread>)  B <median> (±<spread>)  Δ <B−A>  Δ/spread <permille>‰`, then
the excluded files with their reason. Refusal keys, in order: `schema`, `host_id`, `robot`,
`robot_n`, `kernel_rt`. `summary` refuses on `schema`, `host_id` and `robot` only.

## Stability

Adding a key is compatible. Renaming or removing one bumps `schema`.
