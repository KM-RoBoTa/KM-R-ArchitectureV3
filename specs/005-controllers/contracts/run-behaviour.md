# Contract: what the reference binary does, per scenario

The binary `controllers` is the interface this module exposes: to new users, to CI and to the
bench. This table is what [quickstart.md](../quickstart.md) checks.

| # | Scenario | Command | Prints | Exit | Motion | Spec |
|---|---|---|---|---|---|---|
| 1 | No bus plugin (as shipped) | `cargo run -p controllers` | exactly 1 warning, the core's "no bus plugin: simulating"; the shutdown reason at the end | 0 after `DEMO_LENGTH` (12 s of `elapsed()`) | none | FR-007, SC-011 (SRS FR-6.3) |
| 2 | Bus line added, feature off | `cargo run -p controllers --release` | a `run()` error naming the adapter feature, `kmr_api/dynamixel`, which `hardware` turns on | non-zero, no panic | none | kmr_adapters FR-031 and its research R9 (SRS FR-6.3) |
| 3 | Bus line added, feature on, robot unplugged | `cargo run -p controllers --features hardware --release` | a readable bring-up error naming the port | non-zero, no panic | none | FR-026, SC-012 (SRS FR-6.4) |
| 4 | Bus line added, feature on, XmBench wired | `cargo run -p controllers --features hardware` | the shutdown reason at the end | 0 | every actuator within ±5° of its start, then hold, then torque off | SC-009, SC-010 (SRS FR-4.1) |
| 5 | As 4, Ctrl-C (or SIGTERM, SIGHUP) mid-run | same | shutdown by signal | 0 | stops with hold, then torque off | FR-024, SC-010 (SRS FR-4.3) |
| 6 | As 4, foreign Ctrl-C handler first | n/a for this binary | kmr's bring-up warning | 0 at `DEMO_LENGTH` | as 4 | spec Edge Cases (SRS FR-4.3) |

Rules on every row:

- No panic, no `unwrap` path, no marker reached (FR-008, FR-011 (SRS NFR-2)).
- The binary prints nothing of its own. Every message comes from `kmr_api` or `kmr_core`.
- "Readable" means the `Display` text of the error, not its `Debug` form. `main` returning
  `Err` prints `Debug`; `kmr_api` makes that the `Display` text in release builds only, and
  derives `Debug` in debug builds (2026-10-29; [research.md](../research.md) R12). Rows 2 and
  3 are checked with `--release`.
- Row 1, simulation History: either empty ("no sample") or zeros. Both satisfy the row
  (research R11).
