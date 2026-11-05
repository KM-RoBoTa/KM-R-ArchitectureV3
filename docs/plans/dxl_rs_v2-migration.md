# Plan: kmr_adapters from dxl_rs to dxl_rs_v2

Status: plan, 2026-11-04. No code written. Sources: `dxl_rs_v2` at `dev` 4c74e97,
`kmr_v3` at `dev` 724a7bd.

Update 2026-11-05: phases 0, 1 and 6 done on `feature/dxl-rs-v2`, v2 at 66035e1. The maintainer
asked to replace v1, not to run both: no `dynamixel-v2` feature, `dynamixel` means v2. B1 is
worked around (`write_raw`), B2, B3, B4 (XW430-T200 only; MX-64 P1 landed in v2) and the
one-read state are rows of `docs/deferred.md`; the state read stays three partial sync reads.
Phase 2 (bench parity) is next.

Update 2026-11-05, later: phase 4 done for Dynamixel and CubeMars. The core block, `Hal::telemetry`
and the kmr_api getters landed first (SRS FR-5.11); the Dynamixel member reads every register
once in PostInit, then one transaction at most every 50 ticks, round-robin, off the ticks that
read the same bus (kmr_adapters T073); CubeMars copies what its replies carry (T074). Fault flags
for FR-13.4 (`overheat`, `undervoltage`) still wait for core T044: a fault shows as its hardware
error bits.

Phase 3, RAM part, done (kmr_adapters T075, kmr_api T070): `Ids::gains(p, i, d)` and
`Ids::led(on)`, raw register values, checked against the model at `check` (no I/O), written in
`configure` torque off and read back. `.speed_limit` is not wired: `dxl_rs_v2`'s `SpeedLimit` is
rad/s or unlimited over two registers with different meanings per protocol, no plain unit to
expose yet. EEPROM settings still wait for B3. A preset table (`Placement`) sets no RAM setting yet.

## Verdict

**Feasible, and cheaper than it looks for the hot path.** All v1 calls live in one
file (`kmr_adapters/src/dynamixel/dxl_rs.rs`) behind the private `Backend` trait
(`dynamixel/backend.rs:353`), which decision 0002 built for this swap. v2 is
`no_std`, allocates nothing on `Bus` (it has a test for it), has no C SDK and has
no global state, so the swap removes every SDK-boundary exemption kmr carries
today: the `Vec` from `group_sync_read`, the 34 ms stall, `SDK_CLAIMED`, the
port-number bookkeeping, and the libclang/C build in CI (T052–T055).

**The new capabilities are the hard part, and they are mostly not SDK work.**
Exposing temperature and configuration through `kmr_api` needs a change to the
`Hal` contract and a new read-only surface in the core. That is a public API
decision, and it is yours.

**Four v2 gaps block parts of the migration** (B1–B4 below). None of them blocks
phase 1 for position mode on AX-12A, RX-28 and XM430-W210.

## What changes, v1 → v2, at kmr's boundary

| Topic | v1 (today) | v2 | kmr impact |
|---|---|---|---|
| Bus | C SDK, global port numbers, `init()` | owned `Bus<P, T, C, N>` per port, protocol in the type | array of `Option<Bus<P1,..>>` and of `Option<Bus<P2,..>>` per adapter; `MAX_PORTS` bookkeeping goes |
| Actuators | built with no I/O (`Xm430W210::new`), typestate torque | `declare([(Id, Model)])` does I/O, owns them; torque is a runtime state | `open` becomes open + declare (still Init, FR-5.2) |
| Values | raw `u32`, addresses hard-coded (`dxl_rs.rs:36-45`) | typed SI values or `Raw(field)`, addresses per model | hard-coded addresses and unit conversion go |
| Reads | `Vec<(u8, Option<u32>)>` | fill `&mut [T]`; per-slot results under `partial-reads` | enable `partial-reads` to keep FR-2.6 per-actuator stale |
| State read | 3 sync reads (T050 open) | `bulk_read` / `read_field`, one transaction | closes T050/T051 |
| Limits | clamp + warn | reject `OutOfLimits` | adapter already rejects first (`rejected` fault, R11): no behaviour change |
| Faults | none | sticky `Faulted` plus a diagnosis (`FaultError::Voltage/Temperature`) | maps onto `DynamixelError::{Voltage,Temperature,HardwareError}` almost one-to-one |
| Errors | 5 variants, `sdk_error()` | `Copy` enum, ≤64 B, 6 families | rewrite `sdk_error()`; `Send + Sync` holds |
| Timeout | C SDK default | `Config { timeout: 10 ms, retries: 0 }` | set the hot-path timeout to ≈1–2 ms; 10 ms at 1 kHz is 10 lost ticks |
| USB latency | — | not set, only hinted in `Error::hint()` | kmr sets the 1 ms latency timer itself (FR-5.10), before `SerialBus::open` |

## Blockers in v2 (requests to the dxl_rs_v2 maintainer)

- **B1. Goal writes with torque off are refused (`ActuatorIsOff`).** kmr writes the
  start goal (= present position) *before* torque on, so the servo does not jump
  to a stale goal (`sync_write_start_goal`, v1 `e7c6b29`). Wanted: allow goal
  writes while torque is off, or add a "torque on at present position" call.
  Fallback: `write_raw` under `raw-registers` for that one write. Severity:
  high. A jump at bring-up is a safety issue.
- **B2. Setup fails the whole bus when any actuator reports a fault.** One hot
  servo then stops kmr's bring-up, and no recovery is possible
  (`specs/ideas/eeprom-writes.md` §3). kmr wants it to hold the actuator as
  `Faulted` and report it, so `health` (PostInit) decides. Severity: medium. It
  changes a clear error into a harder one, but nothing becomes unsafe.
- **B3. No typed EEPROM writes**: Operating Mode, voltage/temperature/angle
  limits, return delay, ID and baud. The design is open in v2
  (`specs/ideas/eeprom-writes.md`). This blocks D18/T070 (limits from the API)
  and the velocity and current modes (FR-018). `write_raw` is not a fallback
  here, because it desyncs the limit cache v2 builds at Setup.
- **B4. Model tables.** XW430-T200 has no model number (codegen skips it). The
  MX Protocol 1 tables (MX-64 = 310) are missing. kmr's builder exposes
  XW430-T200 today, and T071 targets MX-64. Small for v2, but each one is a
  release dependency.

## Exposing the new capabilities through kmr_api

Two kinds of capability, with two different paths.

### A. Configuration, applied at bring-up (fits today's design)

- **Builder knobs on `Ids`**, next to `.mode()`, `.read_only()` and `.zero()`:
  `.limits(voltage, temperature, angle)`, `.gains(p, i, d)`, `.return_delay(..)`,
  `.led(..)`. Values are typed (degrees °C, volts) and convertible as in FR-2.2.
- **Lowering**: the `Ids` knobs lower into the adapter's per-actuator config.
  `Hal::configure` (PostInit, already in the contract) applies them with torque
  off, then reads them back. That is D18 as written.
- **No new `Hal` method.** The contract is unchanged; only the `Backend` grows
  `write_setting`. RAM settings (gains, LED, speed limit) work on v2 today.
  EEPROM settings wait for B3.
- **Not exposed: ID and baud changes.** They are commissioning tools, not robot
  configuration. They belong in a separate `kmr` CLI or example, never the
  builder.

### B. Telemetry, read at run time (needs a contract change)

- **Not a `StateField`.** Temperature is not commandable, and the field set is
  shared by Desired and History (`state/field.rs`). Adding `Temperature` there
  would make it writable. The SRS treats fault flags as out-of-band
  (SRS §Definitions); telemetry should be out-of-band the same way.
- **Proposed core shape**: a fixed-array `Telemetry` block per actuator
  (`temperature`, `voltage`, `hw_error`, age in ticks), stored next to History's
  depth 0, not per History slot. It is cheap and heap-free. It is read through a
  read-only handle in the controller env. That handle is either the WIP
  `Sensors` (FR-5.7) or a sibling `Telemetry`. The user API is
  `env.telemetry().temperature(i)`, plus group slices.
- **Hal**: a new `telemetry(&mut TelemetrySlots, &mut Faults) -> Mask`
  operation, called by the runtime at its own cadence. The default is a no-op,
  so `NoHardware` and future vendors are untouched.
- **Cadence is the real constraint.** At 1 Mbps, one extra P2 sync read of
  temperature for 10 servos costs about 0.3–0.5 ms, half a tick. So telemetry
  is **not** read every tick. Proposed: round-robin, at most one bus
  transaction every K ticks (default about 100, i.e. 10 Hz), never in a tick
  that already does the state read. Or fold it into the state `bulk_read` on
  P2 when the bench shows the extra bytes are free. The NFR-1 bench (D9)
  decides.
- **Faults**: v2's automatic fault diagnosis feeds FR-13.4 directly
  (`bus_error`/`offline` today, plus `overheat` and `undervoltage` flags). That
  needs core T044 (per-slot fault flags) to land first.
- **Runtime commands** (reboot, clear fault, gains mid-run) are deferred. They
  need a controller → hardware command channel the SRS does not have. FR-5.9
  ("reach the HAL directly") is the escape hatch until then.

## Living with v2's API churn

v2's policy is "breaking changes are fine until 1.0", with 158 commits in 4
weeks and an error-type overhaul today. The plan contains it as follows:

1. **One file, one seam.** Every v2 call goes in
   `kmr_adapters/src/dynamixel/dxl_rs_v2.rs` behind `Backend`. A v2 rename
   breaks one file, never `kmr_core` or `kmr_api`.
2. **Pin a git rev**, not a path: `../dxl_rs_v2` has uncommitted work, and a path
   dependency already broke dev once (MX-64, 2026-11-02). Bump deliberately.
3. **Contract tests on v2's `mock` transport.** Run the existing `scripted.rs`
   scenarios (bring-up, stale read, fault, torque-off on every exit) against
   `DxlRsV2` over v2's mock. A rev bump that changes behaviour fails in CI with
   no hardware attached.
4. **Both backends side by side** during the transition: features `dynamixel`
   (v1) and `dynamixel-v2`, mutually exclusive, selecting `Hardware`. v1 is
   removed after the bench sign-off.
5. **Map errors to kmr's own `DynamixelError`.** v2's types never appear in
   `kmr_api`, so v2's error churn stays invisible to users.

## Phases

| # | Scope | Depends on | Size |
|---|---|---|---|
| 0 | Reshape `Backend` around v2's model: typed values, a single state `bulk_read`, per-slot results; `scripted.rs` follows | — | S–M |
| 1 | `DxlRsV2` backend, position mode, AX-12A/RX-28/XM430-W210; latency timer and hot-path timeout; error mapping; mock contract tests; feature `dynamixel-v2` | 0, B1 (or its raw fallback) | M |
| 2 | Bench parity with v1 on the RX-28 rig (T045/T047/T063): zero allocations over 10k ticks (T059), per-tick I/O time, missing-servo cost | 1 | S (human bench time) |
| 3 | Bring-up configuration from the builder: RAM settings now, EEPROM limits (D18/T070) and Operating Mode after B3 | 1, B3 | M |
| 4 | Telemetry: core `Telemetry` block + `Hal::telemetry` + `kmr_api` read-only handle + cadence; fault flags into FR-13.4 | **your API decision**, core T044 | M–L |
| 5 | Velocity/current modes (needs the stale-setpoint decision first), XW430-T200, MX-64 P1 | B3, B4, pending stale-setpoint decision | M |
| 6 | Remove v1: the `dynamixel` feature, `dxl_rs-agent` path dependency, C build in CI (T052–T055 closed by deletion) | 2 signed off | S |

The critical path to "v2 in production, position mode" is phases 0–2 plus B1.
Temperature in `kmr_api` is phase 4. It does not wait for v2 at all: v2 reads
temperature today. It waits for the telemetry API shape.

## Decisions that are yours

1. **Telemetry surface** (phase 4): a separate `Telemetry` handle (recommended,
   keeps `Sensors` for real sensors per FR-5.7/FR-11.1) or inside `Sensors`. The
   SRS has no id for actuator telemetry, so it needs an SRS line (suggested
   FR-5.11).
2. **B1–B4 priority with the v2 maintainer**: B1 first, it gates phase 1.

## Risks

- **v2 timing on a missing servo.** It costs `(retries+2) × timeout` per unicast,
  but the sync read is one transaction. With partial reads, a dead servo costs
  one timeout per state read, so set the timeout from the bench, not the
  default.
- **P1 throughput.** Reads are sequential, and bulk read works only on MX-class
  tables. The wire is the limit, not the library. That is unchanged from v1;
  the read cadence (D9) already accounts for it.
- **`Bus` size.** It holds `heapless::Vec<ActuatorKind, 32>`, an enum the size of
  its largest model. Size `N` per bus to the declared count, not 32, if stack
  or cache footprint shows on the Pi 4.
- **v2's own SRS edits are uncommitted.** Re-read v2's ARCHITECTURE at every rev
  bump. The EEPROM and recovery API (B2, B3) is the part most likely to change.
