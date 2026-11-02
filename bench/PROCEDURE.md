# Bench procedure

How to produce every kmr reference figure from this file alone (FR-021 (SRS NFR-8)). Settings
live in `bench/scripts/profile.env`; commands run from the repository root. Reports land under
`bench/reports/<commit12>/<host_id>/`; a human commits them.

The host setup, the 1 kHz cells, the ceiling, the budget and the footprint sections come with
their scripts (tasks T025, T028, T033, T043).

## Compile time

Clean and incremental build times of `kmr_core` and `controllers`, dev and release, and the
build time of generated user crates against their registration count (FR-013, FR-014 (SRS
NFR-11, §8); research R14).

The reference figures are measured natively on the reference Raspberry Pi 4 (SRS §8: "hardware
always the RP4 setup"). A figure from any other host is labelled with that host and compared
only with figures of the same label. Compile time is not part of `all.sh` and not of the 2-hour
run of SC-001 (research R18).

1. Check out the commit to measure. The script builds a copy of `HEAD` (`git archive`), so
   uncommitted changes are not measured and the working tree is never edited.
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

The median of an even count is the lower middle value. `[run]` holds `measurement = "compile"`,
`cell` = the label, `repeats` and `reg_counts`. A failed build writes a `failed` report naming
the step; the cargo output is in the `.log`. A missing operator field writes a `failed` report
before any build. A throttle at start or end flags the report `throttled`.
