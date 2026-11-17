# Runtime Actuators Implementation Plan (experiment/runtime-actuators)

> **For agentic workers:** REQUIRED SUB-SKILL: use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task by task. Steps use checkbox (`- [ ]`) syntax.
> Work ONLY in `/home/ovsiankina/Projects/km-robota/projects/Architecture/kmr_v3-runtime-actuators` (branch `experiment/runtime-actuators`). Never edit `../dxl_rs_v2-kmr` or `controllers/src/main.rs`. No `mod.rs`: self-named module files only.

**Goal:** declared Dynamixel actuators (AX-12A, RX-28) may be missing at open, drop out, or join while the loop runs. Two opt-in policies control this. With both off, behaviour is exactly today's.

**Architecture:**
- Each adapter keeps a presence `Mask<N>` and reports it through a new provided `Hal::present()`.
- The core copies that mask into `RobotState` after every read. It removes absent slots from the written masks before the limit stage and the batch.
- Joining is a bounded per-tick `Hal::upkeep()`, called after telemetry and only when the join policy is on.
  - A probe goes out at most once per `PROBE_PERIOD` (wall clock). It is a typed ping on a separate "side" serial handle, so the main bus's held list is never touched by a probe.
  - A joiner is then set up one stage per tick.
- The policies travel core config → `Overrides` → `Hal::check`.

**Tech:** Rust 2024, frunk HList, dxl_rs_v2 typed API (`../dxl_rs_v2-kmr` at e53b249), the `Scripted` backend for tests.

**Spec:** the lead's decisions below, plus `docs/srs/SRS.md` FR-2, FR-4.2, FR-5.1/5.2/5.5/5.6/5.9/5.10, FR-13.4. The places where this experiment departs from the SRS are deferred row 1.

## Lead decisions (fixed, do not re-open)

1. **Sparse slots.** Slots are keyed by declared id and stay sparse. Example: ids 1-10 present, 12 plugged, 11 not. Slot index 11 goes live and index 10 stays absent. Never compact or renumber. The engine index is declaration order: AX ids 1..=12 are indices 0..11, RX ids 1..=10 are 12..21.
2. **Presence mask.** A runtime presence mask (sibling of `Mask<N>`) says which slots are live. Reads, writes, telemetry and torque on skip absent slots.
3. **Unchanged writes.** `set_all` and the group `set_all_*` keep their signatures. They write only present slots; an absent slot is skipped, not an error. Controllers can query presence.
4. **Two separate opt-in policies, both off by default:**
   - (a) `MissingPolicy::Tolerate`: a declared actuator that is silent at open, or stops answering, is not an error and not an emergency stop.
   - (b) `JoinPolicy::Declared`: a pre-declared actuator may join at run time. Absent ids are pinged quietly; when one answers it is configured (limits, mode, torque on as for the others) and then driven.
5. **Log lines.** Under (a), a disappearance never crashes or stops the loop. It gives one warning: "actuator id X not available anymore", or "ids X to Y not available anymore" for contiguous ids. A join gives one line in the same style.
6. **Bounded probing.** Probing must not blow the tick budget: bounded work per tick, and no blocking timeouts stacked in one tick.
7. **Exit path.** The exit path (`Powered`: torque_off then close) tolerates absent slots and still turns torque off on every present one.
8. **Example.** A new `controllers/examples/runtime_actuators.rs`: the clap controller without the microphone, both sines running continuously, both policies on.

No heap, no `dyn`, no `unsafe`, no new dependency. No new `read_raw`/`write_raw` call site (see deferred row 2 for the existing ones the join reuses).

## Global constraints

- **Oracle:**
  - `cargo test --workspace --no-fail-fast`
  - `cargo clippy --workspace --all-targets -- -D warnings`
  - `cargo clippy -p controllers --features hardware --all-targets -- -D warnings`
  - `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf`
  - `cargo fmt --all --check`
- **Policies off means byte-identical.**
  - `present()` is `owned`, which makes the walk give `Mask::ALL`. `Plan::finish` refuses an unowned slot, so this holds.
  - The core never calls `upkeep`.
  - The adapter's exit path takes today's code path unchanged.
  - Every existing test passes unmodified, including `kmr_core/tests/no_alloc.rs` (zero heap calls, zero log events in 10,000 ticks).
- **Core:** stays `no_std`, logs nothing and never escalates on presence. The adapter does all the logging.
- **Overrides:** `Overrides` stays `#[non_exhaustive]`, `Copy` and `Default`. Outside kmr_core, build it with `let mut o = Overrides::default(); o.missing = MissingPolicy::Tolerate;`. A struct literal fails with E0639.
- **DECISION comments:** mark every internal choice `// DECISION: ...` where it is made.
- **Commits:** Conventional Commits as the maintainer, task id first, no AI trailer. For example `feat(core): T1 add the missing and join policies`. Do not push.
- **Pub diff:** it is NOT empty. It is listed exactly below; the run's task text must authorize it (QA L21). Add nothing beyond it. Anything else pub goes to `docs/deferred.md`.

## Pub surface (the whole pub diff)

### kmr_core

1. **New file `kmr_core/src/hal/presence.rs`.** hal.rs gets `mod presence;` and `pub use presence::{JoinPolicy, MissingPolicy};`.
   ```rust
   /// What a declared actuator that does not answer means (policy (a)).
   #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
   pub enum MissingPolicy { #[default] Fail, Tolerate }
   /// Whether a declared actuator absent from the run may join it (policy (b)).
   #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
   pub enum JoinPolicy { #[default] Never, Declared }
   ```
2. **`Overrides` (hal.rs:267)** gains `pub missing: MissingPolicy` and `pub join: JoinPolicy`.
3. **Two provided methods on `Hal<N>` (hal.rs:645).** Both are forwarded in `impl Hal for &mut A` (hal.rs:719-785), because kmr_api lends `&mut A`.
   - `fn present(&self) -> Mask<N> { Mask::ALL }`: the claimed actuators present now. No I/O.
   - `fn upkeep(&mut self) {}`: Tick, once, after telemetry, called only when the join policy is `Declared`. Bounded presence work: one join stage, one check or one probe. It may stall one tick, and its doc says so.
4. **`Drivers<N>` (set.rs:464), `#[doc(hidden)]`:**
   - `fn present(&self, slots: Slots<'_, N>) -> Mask<N>`: HNil gives `Mask::NONE`; HCons gives `head.present() ∩ head's owned ∪ tail`.
   - `fn upkeep(&mut self)`: head, then tail.
5. **`Robot`** (robot.rs, next to `set_sensor_policy` at :192): `pub fn set_missing_policy(self, MissingPolicy) -> Self` and `pub fn set_join_policy(self, JoinPolicy) -> Self`. They are stored in `user_config.rs` as `missing_policy` and `join_policy`.
6. **`RobotState::present(&self) -> Mask<N>`** (pub, read by kmr_api).

### kmr_api

- lib.rs: `pub use kmr_core::hal::{JoinPolicy, MissingPolicy};`.
- robot.rs: `set_missing_policy` and `set_join_policy` after `set_sensor_policy` (:182-187), with the same shape.
- state.rs: a "Presence" section after `stale_at` (:396).
  - `pub fn present(&self) -> Mask<N>`.
  - `pub fn present_at(&self, i: usize) -> bool`, false past N.
  - Its doc says that group reads (`group(..).q()`) including an absent slot give `None`.
- No new `UserError`, `ConfigError` or `StateError` variant.

### FFI

No change (deferred row 3).

## Data semantics of an absent slot (core)

| Item | Absent slot |
|---|---|
| Presence | `RobotState.present: Mask<N>`, starting at `Mask::ALL`. `set_present(p)`: for each index in `old.difference(p)`, `sampled[i] = 0` and `age[i] = 0`; then store `p`. |
| History | Keeps the copy-forward of `push_tick`, or the zero seed if never read. `age(i)` is `None`, `prev_at(i, d)` is `NoSample`, and group reads including it give `None`. |
| `prev()` / `current()` / `q()` | Check `sampled` only for present slots: `present.get(i) && !holds(..)` (robot_state.rs:102). Absent array entries are stale or zero. |
| `sample_age()` | Maximum over present slots only. |
| Desired | `set_at`/`set_all`/group `set_all_*` are unchanged and return Ok. `drop_absent()` (pub(crate)) sets `desired.written = desired.written.within(present)` after `Last` and before `guarded()`. Limits, Batch and the shutdown hold never see an absent slot, and group atomicity (group.rs:120-129) holds. |
| Telemetry | Not refreshed: values stay and the age grows. |
| Faults | No bit while absent. `offline`/`stale` appear only during the loss window, as today. |
| Rejoin seed | A joiner becomes present in the `read` of its first successful sample. It is in both `refreshed` and `present` of that call, so `prev_at(i, 0)` is fresh and `prev_at(i, 1)` is `NoSample`. |

## Core loop (runtime.rs)

- **Claim (:706):** `hal.claim(Overrides { missing: robot.config.missing_policy, join: robot.config.join_policy }, &mut plan)`.
- **PostInit:**
  - right after `state.refreshed(read)` (:616): `state.set_present(rig.drivers.present(rig.layout.slots()))`;
  - again right after `Step::TorqueOn` (:635).
- **`Drivers::read_all` (set.rs ~:625):** the head's mask becomes `owned ∩ head.present()`, still `NONE` when the head simulates.
- **Tick:**
  - after `state.refreshed(read)`: `state.set_present(..)`;
  - after the `Last` estop check: `state.drop_absent()`;
  - after the telemetry `stray_stop` and before `end_tick`, only when `robot.config.join_policy == JoinPolicy::Declared`:
    ```rust
    let pending = host.poll();
    if let Some(signal) = estop(pending) { break signal; }
    if pending.is_none() { rig.drivers.upkeep(); }
    ```
    A join stage then shows as this tick's `OVERRUN`. An estop raised during write or telemetry is never kept waiting behind a stage. A shutdown skips the upkeep and completes the tick. With the join policy off, the poll count and the calls are unchanged.

## Adapter design (kmr_adapters/src/dynamixel)

### Backend seam (backend.rs, pub(crate) trait, no pub diff)

Add to `trait Backend`. Implement each on `Ports<N,T,C>` (dxl_rs_v2.rs:676), forward it in `impl Backend for DxlRsV2<N>` (:1225), and implement it on `Scripted`.

```rust
/// One typed ping of `id` over the port's side line; Ok when it answers as `model`
/// (a fault-only answer counts). What the port holds is untouched.
fn probe(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError>;
/// Torque Enable = 1 on held actuator `id`, no start goal: for one whose torque is on but
/// that a re-declare holds Off.
fn rearm(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError>;
/// One fault ask (`dxl_rs_v2` `clear_fault`), no settle loop.
fn clear_fault_once(&mut self, port: usize, model: Model, id: u8) -> Result<(), BackendError>;
/// Whether the port holds `id` (its last declare named it and succeeded). No I/O.
fn holds(&self, port: usize, model: Model, id: u8) -> bool;
```

**DxlRsV2:**
- `Open::open_pair` becomes `open_lines(path, baud) -> Result<[(Self, C); 4], BackendError>`: one open and three `try_clone_native`.
  - Update the `MockTransport` impl in contract.rs:19-25 in step.
- `Port` gains `side: Option<Side<T, C>>`, with `struct Side<T, C> { one: Bus<P1, T, C, 1>, two: Bus<P2, T, C, 1> }`.
  - A new `attach_side(port, three, four, baud)` builds them with `config(baud)` (dxl_rs_v2.rs:114). With the 10 ms default timeout, a silent probe would take 10 ms.
  - Only `open_ports` calls it. Contract tests keep `attach` and have no side.
- `probe`: `side.<protocol>.declare([(id, ModelNumber(model.number()))])`.
  - `SetupError::Missing` gives `Timeout` (existing `backend_error`).
  - With no side, it returns `PortNotOpen`.
- `rearm`: `held(bus, id)?.enable_torque()` on the main bus. It bypasses the kmr `started` gate (:984-1004) by design.
- `clear_fault_once`: `held(bus, id)?.clear_fault()`, one ask.
- `holds`: `bus.torque_state(id).is_some()`.

**Scripted** (scripted.rs):
- New `Call::Probe`, `Call::Rearm`, `Call::ClearFaultOnce`; `KINDS` goes 21 → 24. `holds` is a query, not recorded (like `faulted`).
- New knob `strict(&mut self) -> &mut Self` sets `strict: bool`. Existing tests never set it, so they are unchanged. Under strict:
  - a new `held: [[bool; IDS]; PORTS]`. `declare` first clears `held`, `powered` and `started` for every id of the port (dxl_rs_v2 Setup replaces the list and holds each one Off; `Ports::declare` clears `started`). It sets `held` for the listed ids only on success, so on any error the port holds nothing;
  - `read_position`, the `sync_read_state` entries, the `read_vitals` entries, `read_setting`, `read_health`, `read_ram`/`write_ram`, `sync_write_start_goal`, `torque`, `rearm` and `clear_fault(_once)` give `NotDeclared { id }` for an id not held;
  - `force_torque_off` of an id not held declares it alone, replacing the port's list, as dxl_rs_v2.rs:1006-1020 does;
  - `holds` reads `held`. Without strict, `holds` is always true.
- `probe` answers through `answer(Call::Probe, ..)`: a missing id gives `Timeout`, a `numbers` mismatch gives `WrongModel { found }`. No state change.
- `rearm` sets `powered` and `torque` true; a missing id gives `Timeout`; under strict a faulted one gives `Faulted { id }`.
- `clear_fault_once` behaves like `clear_fault`, recorded as its own kind.
- `missing(port, id)` already unplugs. New `plug(port, id, power_cycled: bool)` sets `missing = false`. If `power_cycled`, it also sets `torque`, `powered`, `started` and `faulted` false and resets `ram` to defaults. `held` is unchanged, as dxl_rs_v2 still holds it.

### Presence state (new `dynamixel/presence.rs`, `mod presence;` in dynamixel.rs)

```rust
pub(crate) const LOST_AFTER: u8 = 3;                                   // consecutive due reads that time out
pub(crate) const PROBE_PERIOD: Duration = Duration::from_millis(250);  // one probe per adapter
pub(crate) const SETTLE_FOR: Duration = Duration::from_millis(500);    // = VOLTAGE_FLAG_SETTLE total
pub(crate) const MAX_REJOINS: u8 = 3;                                  // cold rejoins per slot per run

pub(crate) struct Presence<const N: usize> {
    pub(crate) missing: MissingPolicy,
    pub(crate) join: JoinPolicy,
    pub(crate) live: Mask<N>,       // = Hal::present: read, written, telemetered
    pub(crate) seed: Mask<N>,       // powered joiners whose first read is pending: read, not written
    pub(crate) configured: Mask<N>, // limits written this run (originals saved once)
    pub(crate) refused: Mask<N>,    // never probed again this run
    pub(crate) suspect: Mask<N>,    // live slots that missed >= 1 due read, then answered
    pub(crate) misses: [u8; N],
    pub(crate) rejoins: [u8; N],
    pub(crate) joining: Option<Join>,
    pub(crate) cursor: usize,
    pub(crate) last_probe: Option<Instant>,   // std::time::Instant
    pub(crate) probe_period: Duration,        // PROBE_PERIOD; tests set ZERO or an hour
}
pub(crate) struct Join { pub(crate) index: usize, pub(crate) stage: Stage, pub(crate) since: Option<Instant> }
pub(crate) enum Stage { Hold, Health, Settle, Check, Configure, Power }
pub(crate) enum Event { Start, Lost, Dropped, Unreached }
```

- **`pub(crate) const fn new() -> Self`.** `Dynamixel::new` is `pub const fn` (dynamixel.rs:671). Use the literals `Fail`/`Never`, `live: Mask::ALL`, and `probe_period: PROBE_PERIOD`.
- **Methods:**
  - `tolerates()` is `missing == Tolerate`.
  - `joins()` is `tolerates() && join == Declared`.
  - `miss(i) -> bool`: true when the count reaches `LOST_AFTER`; always false under `Fail`.
  - `answered(i)`: resets the count. A slot whose count was above 0 goes into `suspect`.
  - `next_probe(candidates: Mask<N>) -> Option<usize>`: round-robin from `cursor`.
  - `driven() -> Mask<N>`: `live ∪ seed`.
- **`pub(crate) fn report(bus: &str, ids: &mut [u8], event: Event)`:** insertion-sort the ids, then one `warn!` per run of consecutive ids, with `bus` as a tracing field (AX id 3 and RX id 3 both exist). Exact texts:

| Event | Single id | Run of ids |
|---|---|---|
| `Start` | `actuator id {x} not available at start` | `ids {x} to {y} not available at start` |
| `Lost` | `actuator id {x} not available anymore` | `ids {x} to {y} not available anymore` |
| `Dropped` | `actuator id {x} dropped: its bus setup failed` | `ids {x} to {y} dropped: their bus setup failed` |
| `Unreached` | `actuator id {x} not reached for torque off` | `ids {x} to {y} not reached for torque off` |

  `Dropped` also carries `%error`.
- **Single lines:**
  - `info!` `actuator id {x} joined`;
  - `warn!` `actuator id {x} answered but was not set up: {error}`;
  - `warn!` `actuator id {x} held faulted: not driven anymore`;
  - `warn!` `actuator id {x} restarted: set up again`;
  - `warn!` `actuator id {x} restarted {MAX_REJOINS} times: not driven anymore`;
  - at `check`, when `join == Declared && missing == Fail`: `warn!` `the join policy has no effect without MissingPolicy::Tolerate` (not an error).
- **`check(overrides)`** starts with `self.presence = Presence::new()`. The adapter outlives a run (FR-5.9) and `close` already took `saved`. It then stores both policies and sets `live = owned`. `fn present(&self) -> Mask<N> { self.presence.live }`.

### State machine per slot (Tolerate only; under Fail every slot is Live forever)

```
Live --LOST_AFTER due reads time out--> Absent        (Lost report, coalesced per bus per read call)
Live --suspect check: Torque Enable 0, not faulted--> Absent   ("restarted", rejoins += 1; > MAX_REJOINS -> Refused)
Live --held faulted, or torque cut seen at a Hold--> Refused   (warned once, cut at exit)
Absent --probe answers (upkeep, joins() only)--> Joining
Joining: [Hold if !holds(i)] -> Health -> {warm: Seed} | {cold: Settle -> Check -> Configure -> Power -> Seed}
Seed --first answered read--> Live            ("joined" logged at Power or warm)
Joining --Timeout in any stage--> Absent      (quietly)
Joining --any other error after Hold--> one force_torque_off, then Refused (warned once)
Refused: quiet for the rest of the run
```

**Bring-up under Tolerate:**
- **`ping`:** per bus, the full declare is tried as today, up to `PING_ATTEMPTS`.
  - Then, while the result is `Err(Undeclared { id: Some(x), source: Timeout })`: clear x from `live` and declare the remaining live ids of that bus, one attempt per pass, at most `len` passes. An empty list skips the declare, and the bus holds nothing.
  - Any other error stays fatal.
  - Then one coalesced `Event::Start` report per bus.
- **`health`, `configure`, `read_all`, `torque_on`:** visit only `live` slots, in today's index order (filter with `live.get(i)`).
  - A `DynamixelError::Missing` drops that slot from `live` (one `Lost` report) and continues. Any other error stays fatal.
  - `torque_on`'s start-goal read (`read_present`) drops Missing slots and powers the rest.
  - `configured` is set for every slot whose health ran.

**Per tick:**
- **`read` → `io::read_due`:**
  - `read_table` gains `only: Mask<N>`. P1 skips members outside it. P2 builds its id slice from members in `only`, because dxl_rs_v2 refuses a call naming an id it does not hold, and a held silent id costs a timeout.
  - A due table reads `live ∪ seed`. A table not due still reads its `seed` members, so a joiner is read on the next tick whatever the cadence.
  - **Misses only on silence:** `Outcome::Lost(BackendError::Timeout)` calls `miss(i)`. Every other outcome calls `answered(i)`: `Read`, `Invalid`, `Lost(Fault|Status|Reply|..)`. `Lost(NotDeclared)` and `Bus` errors leave the count unchanged. Fault bits stay as today.
  - On threshold: clear the slot from `live` and `seed`, collect its id, and log one coalesced `Lost` report per bus after the walk.
  - A seed slot that reads moves into `live` and is in `refreshed`.
- **`write_batch` and `hold`:** skip any index not in `live`, with no fault. `sync_write_goal` refuses the whole call for one unpowered id (dxl_rs_v2.rs:916-920).
- **`telemetry`:**
  - A P1 `Step::One` of a non-live member sends nothing: advance `next` and leave `since`, so the next step goes next tick.
  - A P2 `Step::Field` passes live ids only, or is skipped the same way when there are none.
  - New `pub(super) fn sent(adapter) -> bool { adapter.rounds.since == 0 }`. `refresh` returns `NONE` both on skip and on a failed send, so the returned mask cannot be used.
  - `read_this_tick` is refactored over a new `pub(super) fn table_read_this_tick(adapter, number: usize) -> bool`.

**Exit (Tolerate only; under Fail, `torque_off` and `close` run today's code unchanged):**
- **`torque_off`:** `force_torque_off` with `TORQUE_OFF_ATTEMPTS` for every owned slot that `backend.holds(..)` (live, seed, refused or lost but still held). It never declares, so nothing is wiped before restore.
- **`close`:**
  1. `restore_limits` for slots whose bus holds them. A slot with saved values that is not held gets one warn `limits of actuator id {x} not restored: not reachable`.
  2. Then one `force_torque_off` attempt for every owned slot not held. The declare-alone wipe is harmless once restore is done. The failures go in one coalesced `Unreached` report per bus.
  3. Then flush and close as today.
- So every owned actuator gets a torque off at exit (FR-5.6 is kept): absent ones cost one setup timeout each, at exit only.

### Upkeep (Tolerate and Declared only; otherwise it returns at once)

Each tick, in this order, at most one of:

1. **A join in progress:** run its current stage and return.
2. **A `suspect` slot** (lowest index) whose actuator is `Torque::On`: one `read_setting(TorqueEnable)`.
   - 1: clear its suspect bit.
   - 0 and `faulted()`: refused, plus the warn "held faulted".
   - 0 and not faulted: remove it from `live`, `rejoins += 1`, and warn "restarted" (or "restarted MAX_REJOINS times", then refused).
   - Timeout: keep the bit; the miss logic decides.
3. **A probe**, when `last_probe` is `None` or older than `probe_period`, and on a tick where:
   - telemetry did not send (`!telemetry::sent`);
   - the candidate's table was not read this tick (`!table_read_this_tick`).

   Candidates are `owned − live − seed − refused`; take `next_probe`. Set `last_probe = now` whatever the answer. The `backend.probe(bus, model, id)` result:
   - Ok: `joining = Some(Join { index, stage: if holds { Health } else { Hold }, since: None })`.
   - `WrongModel`: refused, plus the warn "answered but was not set up".
   - Any other error: nothing, quietly.

   `last_probe` is first set at the end of `torque_on`, so the first probe comes one period after bring-up.

**Stages** (one per tick; `i` is the joiner, `b` its bus):

- **Hold** (only when `!holds(i)`): at most two declares in this tick.
  1. Let `P` be the slots of `b` in `live ∪ seed`. Snapshot `F` = those in `P` with `faulted()`.
  2. `declare(b, P ∪ {i})`, one attempt.
  3. On error, `declare(b, P)`, one attempt (the fallback). The join is aborted:
     - if the error named `i` with `WrongModel`, `i` is refused (warn);
     - otherwise `i` is Absent again, quietly.
  4. If the fallback also fails: every slot of `P` leaves `live` and `seed`, Absent and not refused. One `Dropped` report. They rejoin one by one through the normal probe path (warm path, their limits are kept). DECISION; the estop alternative is deferred row 1.
  5. After a successful declare, for each `j` in `P`:
     - `j ∈ F` or `faulted()`: leaves `live`, refused, warn "held faulted";
     - else, if its actuator is `Torque::On`: `read_setting(TorqueEnable)`:
       - 1: `rearm`. A failed rearm counts as Timeout (`Lost`) or refused.
       - 0: leaves `live` as "restarted" (same rule as the suspect check).
       - Timeout: `Lost`.
  6. If `i` was held, go to Health next tick.
- **Health:**
  - If `configured.get(i)` and `read_setting(TorqueEnable) == 1` (a warm rejoin: RAM intact, data blip): `rearm`, move it to `seed`, log "joined", done.
  - Otherwise (cold): `force_torque_off(i)` (held, so no wipe; it clears a stale dxl_rs_v2 `On`). Then `write_limits(i, ..)` when `limit_writes(i)` is not empty.
    - It writes and verifies on every cold join. `save` keeps registers saved earlier (limits.rs:323-330), so the first unit's originals are kept for close.
    - A swapped unit therefore gets this run's limits.
  - Then Settle.
- **Settle:**
  - Not `faulted(i)`: go on to Check in the same tick.
  - Otherwise: one `clear_fault_once` per tick, `since` set on the first ask.
    - Ok: Check next tick.
    - `Fault` while `since.elapsed() < SETTLE_FOR`: stay.
    - Anything else, or time up: refused.
  - This replaces bring-up's 10 × 50 ms sleep loop.
- **Check:** `check_health(i, writes)`, extracted from `health()` (see T6).
- **Configure:** `configure_one(i)` (checks only), then `apply_settings(i)`.
- **Power:**
  1. `io::write_start_goals(self, table, only = {i})`: goal = the present position of `i` alone, read now.
  2. Then `torque(on)` for `Torque::On`, or `force_torque_off` for `Torque::ReadOnly`.
  3. Then `seed.set(i)`, `configured.set(i)`, log "joined", and clear `joining`.
- **Stage errors:**
  - `DynamixelError::Missing` in any stage: Absent again, quietly. In Health or later, also one `force_torque_off` attempt.
  - Any other error after Hold: one `force_torque_off(i)` (held, wipes nothing), refused, warn "answered but was not set up: {error}".

### Safety summary

- **Powered guard:** core unchanged. The adapter's Tolerate exit cuts every owned actuator: held ones with attempts before restore, the rest once after.
- **A joiner** is torqued on only after Health, Check and Configure passed, at its present position (start goal), so there is no jump. `ReadOnly` joiners are cut. While in `seed` it is held and cut at exit, but never written.
- **No silent re-powering:**
  - A Hold rearms only slots that read Torque Enable 1 and are not faulted.
  - A slot cut by its firmware or held faulted is refused.
  - Cold rejoins are capped at `MAX_REJOINS` per slot.
- **Stale setpoint:** Desired is cleared every tick and the joiner's history is seeded by its first read. The engine adds no ramp (deferred row 1); the example blends.
- **Known gaps** (deferred row 1):
  - a reboot faster than one tick (no missed read) goes unseen;
  - after a failed Hold fallback, that bus's servos hold their last goal unwritten until they rejoin;
  - a join stage stalls one tick (Hold ≈ 70 ms declare + one Torque Enable read per held slot; Health ≈ 2 × 20 ms EEPROM settle).

## Review focus

1. **Bench AX-12A at 10.1 V joining** with `voltage_limits(9.0, 13.0)`. The Min Voltage write raises Input Voltage briefly. Expected: Settle clears it within `SETTLE_FOR` and it joins, with no 500 ms stall in one tick. Test: `first_join_settles_a_voltage_flag_across_ticks` (T6).
2. **One bus fully absent at open, then one id plugged.** Expected: the bus declares nothing, the run proceeds, and the join's Hold declares `{i}` alone. Test: `whole_bus_absent_at_open_then_one_joins` (T6).
3. **A noisy line or an overloaded servo answering with error bits.** Expected: never marked lost. Test: `answers_with_error_bits_are_not_misses` (T5).
4. **A failed Hold on a live bus.** Expected: the fallback keeps the bus held. If both fail, the slots are Dropped and rejoin, and the exit still cuts them all. Tests: `hold_failure_falls_back_to_the_previous_list` and `hold_double_failure_drops_the_bus_and_exit_still_cuts_it` (T6).
5. **A USB adapter pulled out (`Io`).** Expected: today's `bus_error` behaviour, slots stay present, no rejoin. Test: `bus_errors_are_not_misses` (T5); documented in the `read_due` doc.

## Ordered tasks

**Lanes:**
1. Start T1, T2 and T4 together; they share no file.
2. T3 after T1 and T2. T5 after T1 and T4, in parallel with T3.
3. T7 after T3, in parallel with T5 and T6.
4. T6 after T5 (same files).
5. T8 after T6 and T7.

### T1. Core policies and seam

**Files:**
- Create: `kmr_core/src/hal/presence.rs`.
- Modify: `kmr_core/src/hal.rs` (`mod`/`pub use`; `Overrides` fields; `Hal::present`/`upkeep` and their forwarding; `ping`/`read_all`/`torque_on` docs say "every present actuator").
- Modify: `kmr_core/src/config/user_config.rs`, `kmr_core/src/robot.rs`.

**Produces:** `MissingPolicy`, `JoinPolicy`, `Overrides { missing, join }`, `Hal::present`, `Hal::upkeep`, `robot.config.missing_policy`/`join_policy`, and the two `Robot` setters.

- [ ] Test `policies_default_to_fail_and_never` (presence.rs): `MissingPolicy::default() == Fail`, `JoinPolicy::default() == Never`, `Overrides::default().missing == Fail`.
- [ ] Test `forwarded_hal_reports_present_and_upkeep` (hal.rs tests): a `&mut` of a fake member whose `present()` is `mask(&[0])` forwards it, and `upkeep` reaches it.
- [ ] Implement. Check: `cargo test -p kmr_core`, plus the thumbv7em `--no-default-features` build.
- [ ] Commit `feat(core): T1 add the missing and join policies and the presence seam`.

### T2. Core state

**Files:**
- Modify: `kmr_core/src/state/robot_state.rs` (field `present`, `present()`, `set_present`, `drop_absent`, `prev`/`sample_age`).
- Test: `kmr_core/src/state/test.rs` (the existing state test module), new section "presence".

**Produces:**
- `pub fn present(&self) -> Mask<N>`
- `pub(crate) fn set_present(&mut self, present: Mask<N>)`
- `pub(crate) fn drop_absent(&mut self)`

- [ ] Tests:
  - `all_present_by_default`
  - `fr_2_3_prev_ignores_absent_slots_never_sampled`: slot 2 absent and never read; `prev::<Q>(0)` is Ok.
  - `sample_age_ignores_absent_slots`
  - `a_slot_going_absent_forgets_its_samples`: `age(i) == None`, and `prev_at(i, 0)` is `NoSample`.
  - `fr_2_6_a_rejoined_slot_is_seeded_by_its_first_read`: `prev_at(i, 1)` is `NoSample`.
  - `fr_2_5_drop_absent_clears_written_bits_of_absent_slots`
  - `set_at_on_an_absent_slot_is_ok`
- [ ] Implement. Check: these 7 and `cargo test -p kmr_core`.
- [ ] Commit `feat(core): T2 presence mask in the robot state`.

### T3. Core walk and loop (after T1, T2)

**Files:** `kmr_core/src/hal/set.rs` (`Drivers::present`/`upkeep` on HNil/HCons, `read_all` cut), `kmr_core/src/runtime.rs` (claim overrides, `set_present` ×3, `drop_absent`, gated poll and upkeep, tests).

- [ ] Tests in runtime.rs `mod tests`. Use a local fake Hal that scripts `present()` per read call; do not change `test_util::Recorder`, whose pub fields are struct-literal'd elsewhere.
  - `claim_gets_the_policies_in_overrides`
  - `present_reaches_state_after_read_before_first`
  - `read_all_samples_only_present_slots`
  - `set_all_writes_only_present_slots`
  - `hold_skips_absent_slots`
  - `upkeep_runs_once_per_tick_after_telemetry_before_end_tick`
  - `upkeep_never_runs_with_join_never` (same poll count as before)
  - `estop_raised_before_upkeep_skips_it`
- [ ] Implement. Check: these tests, `kmr_core/tests/no_alloc.rs`, `cargo test -p kmr_core --features test-util`.
- [ ] Commit `feat(core): T3 presence through the loop and the upkeep call`.

### T4. Adapter backend seam (parallel with T1-T3)

**Files:** `kmr_adapters/src/dynamixel/backend.rs`, `dxl_rs_v2.rs` (Ports impl, DxlRsV2 forwarders, `open_lines`, `Side`, `attach_side`), `dxl_rs_v2/contract.rs`, `scripted.rs` (with its own `mod tests` at :1466).

**Produces:** `probe`, `rearm`, `clear_fault_once`, `holds`; Scripted `strict()` and `plug()`; `Call::{Probe, Rearm, ClearFaultOnce}`.

- [ ] Scripted tests:
  - `probe_answers_without_changing_what_the_port_holds`
  - `probe_of_an_unplugged_id_times_out`
  - `probe_of_another_model_is_wrong_model`
  - `strict_declare_replaces_the_held_list_and_holds_each_off`
  - `strict_failed_declare_holds_nothing`
  - `strict_read_of_an_unheld_id_is_not_declared`
  - `rearm_powers_again_without_a_start_goal`
  - `plug_after_power_loss_comes_back_torque_off`
- [ ] Contract tests:
  - `rearm_sends_one_torque_enable_write`
  - `clear_fault_once_asks_once`
  - `holds_follows_the_last_declare`
  - `probe_without_side_lines_is_port_not_open`
- [ ] Implement. Check: these tests and `cargo test -p kmr_adapters` (existing tests untouched).
- [ ] Commit `feat(adapters): T4 probe, rearm, single fault ask and held query on the backend`.

### T5. Adapter presence: bring-up, skip, loss, exit (after T1, T4)

**Files:**
- Create: `dynamixel/presence.rs` (consts, `Presence`, `Event`, `report`, `miss`/`answered`/`driven`).
- Modify: `dynamixel.rs` (field `presence`, `check`, `present`, Tolerate `ping`, live-only health/configure/read_all/torque_on, `hold`, Tolerate `torque_off`/`close`).
- Modify: `dynamixel/io.rs` (`read_table(.., only)`, `read_due` misses/seed/report, `write_batch` skip, `write_start_goals(adapter, number, only)`, `read_present(.., only)` tolerant).
- Modify: `dynamixel/telemetry.rs` (live-only steps, `sent`, `table_read_this_tick`), `dynamixel/limits.rs` (`restore_limits` held-only under Tolerate).
- Test: create `dynamixel/tests/presence.rs`, and add `mod presence;` in `tests.rs`. It uses Scripted with `strict()`, `AX_TABLE`/`SNAKES_TABLE`, N=4, and a local `powered_with(adapter, overrides)` modelled on `powered` (tests.rs:391).

- [ ] Tests:
  - `fr_5_5_strict_missing_at_open_still_fails_after_bounded_retries`
  - `tolerate_missing_at_open_brings_up_the_rest`
  - `slots_stay_sparse_index_3_is_id_4`
  - `absent_slot_is_never_read_written_or_telemetered`
  - `fr_2_4_set_all_on_an_absent_slot_is_skipped_without_fault`
  - `three_timed_out_reads_mark_a_slot_lost`
  - `one_answered_read_resets_the_miss_count`
  - `answers_with_error_bits_are_not_misses`
  - `bus_errors_are_not_misses`
  - `strict_policy_never_drops_a_silent_slot`
  - `lost_ids_in_a_row_log_one_line`: "ids 2 to 4 not available anymore", with field `bus = "ax"`.
  - `a_lone_lost_id_logs_actuator_id`
  - `missing_ids_at_start_log_one_line`
  - `fr_5_6_torque_off_cuts_every_held_slot_then_close_cuts_the_rest`: every owned id gets a `ForceTorqueOff`. Held ones come before any restore write, unheld ones after.
  - `close_restores_limits_of_held_slots_only`
  - `telemetry_skips_absent_members_without_sending`
  - `policies_off_exit_is_unchanged`: same call log as today's `torque_off`/`close`.
- [ ] Implement. Check: these tests, and every existing `tests.rs` test unchanged and green.
- [ ] Commit `feat(adapters): T5 tolerate missing actuators at open and at run time`.

### T6. Adapter join (after T5)

**Files:**
- Create: `dynamixel/join.rs` (`upkeep`, `next_probe` use, stages, Hold), with `mod join;` in dynamixel.rs.
- Modify: `dynamixel.rs`:
  - `fn upkeep(&mut self) { join::upkeep(self) }`;
  - extract `check_health(index, writes: Limits)` from the `health()` loop body (read_health → mismatch → P2 status → voltage → temperature → datasheet warns). `health()` becomes: limits write, then `if faulted { clear_fault }`, then `check_health`. Same order, same errors;
  - extract `configure_one(index)` from the first loop of `configure()`, checks only. `configure()` keeps its two passes, checks then `apply_settings`, so no setting is written before every actuator passed.
- Modify: `dynamixel/presence.rs` (`Join`, `Stage`, `next_probe`).
- Test: create `dynamixel/tests/join.rs`, with `mod join;` in tests.rs.

- [ ] Tests:
  - `join_off_never_probes`
  - `join_without_tolerate_warns_at_check`
  - `probe_waits_for_its_period` (period one hour: one probe, then none)
  - `probe_is_round_robin_one_per_eligible_tick` (period ZERO)
  - `probe_skips_ticks_that_read_its_bus_or_sent_telemetry`
  - `probe_of_a_silent_id_changes_nothing`
  - `answering_id_is_held_configured_powered_then_present_after_its_first_read`
  - `join_redeclares_the_bus_and_rearms_the_others`: the next `write` sends `SyncWriteGoal` Ok for all.
  - `hold_rearms_only_slots_reading_torque_on`
  - `hold_never_rearms_a_faulted_slot`
  - `hold_failure_falls_back_to_the_previous_list`
  - `hold_double_failure_drops_the_bus_and_exit_still_cuts_it`
  - `a_present_id_silent_at_redeclare_is_dropped_and_the_rest_held`
  - `whole_bus_absent_at_open_then_one_joins`
  - `rejoin_of_a_held_slot_skips_hold`
  - `joiner_start_goal_is_its_own_present_position_alone`: one `ReadPosition` only.
  - `first_join_settles_a_voltage_flag_across_ticks`: no stage sleeps more than 2 × `EEPROM_SETTLE`.
  - `cold_rejoin_rewrites_limits_and_keeps_the_first_originals`
  - `warm_rejoin_is_rearmed_without_reconfigure`
  - `join_failure_cuts_the_joiner_warns_once_and_is_not_retried`
  - `wrong_model_at_probe_warns_once`
  - `suspect_slot_with_torque_off_rejoins_cold`
  - `rejoins_stop_after_max_rejoins`
  - `second_run_starts_from_a_fresh_presence`
- [ ] Implement. Check: these tests and `cargo test --workspace`.
- [ ] Commit `feat(adapters): T6 join declared actuators at run time`.

### T7. kmr_api surface (after T3; parallel with T5/T6)

**Files:** `kmr_api/src/lib.rs`, `kmr_api/src/robot.rs`, `kmr_api/src/state.rs`, `kmr_api/tests/presence.rs` (new; follows the `run.rs` pattern: local `Hal` + `Spec`, the `bus()` lock, `stop_at`).

- [ ] Test `fr_5_5_presence_reaches_controllers_and_absent_slots_get_no_command`:
  - both policies set, and `check` saw them in `Overrides`;
  - `present()` flips at tick 3, and the controller's `present_at` follows;
  - the `set_all_q` group's written mask given to `write` excludes the absent slot.
- [ ] Test `present_at_past_n_is_false`.
- [ ] Implement. Check: `cargo test -p kmr_api` and clippy.
- [ ] Commit `feat(api): T7 presence and the two policies in the user API`.

### T8. Example and paperwork (after T6, T7)

**Files:**
- Create: `controllers/examples/runtime_actuators.rs`.
- Modify: `controllers/Cargo.toml`. Add:
  ```toml
  # The clap controller of src/main.rs without the microphone, actuators plugged and unplugged at run time.
  [[example]]
  name = "runtime_actuators"
  required-features = ["hardware"]
  ```
- Modify: `docs/deferred.md` (3 rows below).

**Check:**
- `cargo clippy -p controllers --features hardware --all-targets -- -D warnings`
- `cargo build -p controllers --example runtime_actuators --features hardware`
- then the hardware run below (the maintainer's).

**Commit:** `feat(controllers): T8 runtime actuators example`, and `docs(deferred): runtime actuators rows`.

## Example: controllers/examples/runtime_actuators.rs

- **`//!` doc:** the clap controller of `src/main.rs` without the microphone. Both sines run continuously from the end of the home ramp: AX 0.30 rad, RX 0.15 rad, 0.5 Hz. Actuators may be unplugged and plugged while it runs; a joiner eases in over 1 s. Stop with Ctrl-C. It also gives the run command.
- **Imports:**
  ```rust
  use kmr_api::{Bus, EachTick, JoinPolicy, Mask, MissingPolicy, Policy, Q, Sensors, Time, presets::home::Home, shutdown, with};
  use std::{f32::consts::TAU, time::Duration};
  ```
- **Model:** `kmr_api::robot!(model = "models/ax_12_12act_an_rx28_10act.urdf");` (N = 22).
- **Copied verbatim from main.rs:** `CENTER`, `HOME_TIME`, `HOME`, `AX_COUNT`, `AX`, `RX`, `AMPLITUDE`, `from()`, `sine()`. Add `const JOIN_RAMP: Duration = Duration::from_secs(1);`.
- **Context:** `struct Joins { seen: Mask<N>, since: [Option<(Duration, Q)>; N], started: bool }` with `const fn new()`.
- **`fn sway(joins: &mut Joins, time: &Time, robot: &mut State, _: &Sensors)`**, from tick 0:
  1. `present = robot.present()`. If `started`, for each `i` in `present.difference(seen)`: `since[i] = robot.q_at(i).ok().map(|q| (now, q))`. Then `seen = present` and `started = true`. Slots present at tick 0 never blend; Home covers them.
  2. `base(i)` is `CENTER` before `HOME_TIME`, after it `sine(2.0 * AMPLITUDE, t)` for AX and `sine(AMPLITUDE, t)` for RX, with `t = now - HOME_TIME`.
  3. For a slot whose `since` is younger than `JOIN_RAMP`: `lerp(q0, base(i), s)` with `s = age / JOIN_RAMP`. An older one gets `since = None`.
  4. Before `HOME_TIME`: `set_q_at` only for blending slots. `sway` is added after `HOME`, so its write wins over Home's jump to `CENTER`.
  5. After it: `robot.group(AX).set_all_q(ax)` and `robot.group(RX).set_all_q(rx)` with per-slot values, with `eprintln!` + `shutdown!` on Err as in main.rs.
- **`main`:**
  - `Bus::by_id("AL02L1KL").baud(1_000_000).build()` (AX) and `Bus::by_id("AL02L1EU").baud(1_000_000).build()` (RX).
  - `Robot::new().set_dt(Duration::from_millis(5)).set_hold(Duration::from_millis(50)).set_limit_policy_all(Policy::Clamp).set_missing_policy(MissingPolicy::Tolerate).set_join_policy(JoinPolicy::Declared)`.
  - `.add_actuator(kmr_api::dynamixel::Ax12a::ids(1..=12).voltage_limits(9.0, 13.0).restore_limits().on(ax_snake))`.
  - `.add_actuator(kmr_api::dynamixel::Rx28::ids(1..=10).voltage_limits(8.0, 18.5).restore_limits().on(rx_snake))`.
  - `.add_controller(EachTick, HOME.controller())`, then `.add_controller(EachTick, with(Joins::new(), sway))`.
  - `if let Err(e) = robot.run() { eprintln!("{e}"); }`.
  - No XIAO, no Clap, no time limit.

**Hardware run:** `cargo run -p controllers --example runtime_actuators --features hardware --release`.

**Bench check (manual):**
1. Start with AX id 11 unplugged: "actuator id 11 not available at start" (bus=ax).
2. Plug it in. "actuator id 11 joined" follows within about 1 s (250 ms probe period + Hold + about 5 stage ticks), and it eases into the sine. Each further absent id adds up to 250 ms in round robin. Join ticks raise `OVERRUN`.
3. Unplug the AX chain after id 9: one line, "ids 10 to 12 not available anymore", within about 3 ticks. The loop and the other servos keep going.
4. Plug it back: one "joined" line each. They are still held, so there is no Hold, and they rejoin cold.
5. Ctrl-C: present servos go limp and limits are restored. The only "not reached for torque off" lines name unplugged ids. No emergency stop.

## DECISIONs to mark in code

1. `LOST_AFTER = 3` timed-out due reads. Only `Timeout` is a miss; replies with error bits, `NotDeclared` and bus errors are not.
2. `PROBE_PERIOD = 250 ms` of wall clock (`std::time::Instant`), one probe per adapter, round robin, on a tick free of that bus's read and of telemetry. The adapter does not know dt, and widening `Overrides` with it was not decided.
3. The join runs in up to 6 stages, one per tick. Hold (≤ 2 declares) and Health (≤ 2 EEPROM settles) are accepted as one-off overruns. Settle asks once per tick for at most `SETTLE_FOR`.
4. Refused slots are not retried during the run. Cold rejoins are capped at `MAX_REJOINS = 3`.
5. A Hold rearms only slots not faulted that read Torque Enable 1.
6. A failed Hold fallback drops that bus's slots to Absent, to rejoin one by one.
7. Absent slots carry no fault bit.
8. `prev()`/`sample_age()` skip absent slots, and loss resets `sampled`.
9. Two side handles per port, for the typed probe, timed by `config(baud)`.
10. Join without Tolerate gives one warning at `check`.
11. Log lines carry the bus label as a field. Lost, Start, Dropped and Unreached are coalesced per bus.
12. Tolerate exit: held slots are cut before restore, unheld ones once after.
13. Scripted fidelity sits behind `strict()`, so existing tests are untouched.

## docs/deferred.md rows (module "experiment/runtime-actuators", format of the file)

1. **`- [question] later:` SRS amendments and known safety gaps.**
   - Contradicted ids:
     - FR-5.9: hardware fixed for the run.
     - FR-5.5: presence only at bring-up.
     - FR-5.2: PostInit work runs in the tick loop for joiners.
     - FR-5.1: only present slots are driven.
     - FR-5.10: join stages sleep (EEPROM settle) and re-declare in `upkeep`, a one-off overrun.
     - FR-13.4: absent slots raise no fault bit or TickFlag, only the presence mask and one log line.
     - FR-2.1/2.3: `prev()`/`sample_age()` ignore absent slots.
     - FR-4.2: a lost actuator never escalates, and estop latency equals one join stage.
     - The pending stale-setpoint decision: no engine ramp for a joiner.
   - Gaps left to the lead:
     - (i) when a Hold and its fallback both fail, that bus's servos hold their last goal unwritten until they rejoin. Emergency stop instead? That is a safety-behaviour decision;
     - (ii) a servo rebooting within one tick (no missed read) is not detected;
     - (iii) a P1 goal write possibly re-enabling AX/RX torque after a reboot is unverified.
   - Smallest unblocking change: a "runtime actuators" clause on those ids, mirroring FR-5.7's sensor policy, and one line deciding (i).
2. **`- [question] later:` dxl_rs_v2 API gap.**
   - No incremental hold or forget and no public ping. So a join to an unheld id re-declares the whole bus (≈ 6 round trips per held id, all reset to Off, then a Torque Enable read and a rearm each), and the probe needs two extra cloned handles.
   - The join reuses existing raw paths and adds no new raw call site: the start goal `write_raw` (B1), the P1 Torque Enable `read_setting`, and the P1 vitals.
   - Smallest unblocking change: dxl_rs_v2 `Bus::hold(id, model)`, `Bus::release(id)`, `Bus::ping(id)`, and a typed Torque Enable read field.
3. **`- [question] later:` FR-10.2, foreign controllers mirror the Rust API.**
   - Python, Lua and C have no `present()`/`present_at`.
   - Smallest unblocking change: one `u64` presence word in `ffi::Snapshot::fill`, with a `present_at` binding for Python and Lua. For C, a `CState.present` field plus the layout test and the build.rs header (an ABI change).

## Review resolution (draft → this plan)

| Critique | Verdict |
|---|---|
| RT D1 / Safety D2: misses counted for answering servos (io.rs:418, :442) | Accepted: Timeout only. |
| RT D2 / Feas 8b: Hold retry loop stacks declares, bus left empty | Accepted: ≤ 2 declares with fallback; double failure → Dropped/rejoin. |
| RT D3 / Feas 8: Health stall (clear_fault 10×50 ms, EEPROM) and no poll before upkeep | Accepted: Settle stage with `clear_fault_once`, and a gated estop poll before upkeep. |
| Feas 8: "a joiner faulted at Health is refused" | Rejected: that refuses every AX joiner whose Min Voltage write raises the expected flag. Settle replaces it. |
| RT D4 / Feas 10: "telemetry sent nothing" unreadable | Accepted: `rounds.since == 0`, plus `table_read_this_tick`. |
| RT D5: probe cadence in ticks while the real tick is ≈ 27 ms | Accepted, via the wall-clock `PROBE_PERIOD`. Rejected: the `Overrides.dt` option, a pub widening nobody decided. |
| RT D6: Power stage reads the whole table | Accepted: `only = {i}`. |
| RT D7 / Feas 12: side bus timing, contract `Open` impl | Accepted. |
| RT D8 / Safety D9: reboot inside the loss window | Accepted as the `suspect` check; the sub-tick reboot is deferred. |
| Safety D1 / Feas 5: exit skips powered slots | Accepted: Tolerate exit cuts every owned slot (held first, unheld after restore). DECISION 5 of the draft dropped. |
| Safety D3 / Feas 6: rearm re-powers firmware-cut or faulted servos | Accepted: faulted snapshot, Torque Enable read, refusal, `MAX_REJOINS`. |
| Safety D4: refused joiner left powered | Accepted: one `force_torque_off` before refusing. |
| Safety D5: failed Hold leaves a bus powered and unreachable | Fallback and exit cut accepted. Emergency stop rejected here (safety behaviour is the lead's call) → deferred row 1 (i). |
| Safety D6: estop latency in upkeep | Accepted: gated poll before upkeep. |
| Safety D7: rejoin keeps unchecked limits | Accepted: cold rejoins write and verify, and `save` keeps the first originals. |
| Safety D8: jump when joining during home | Accepted: the example blends from tick 0. |
| Safety D10: configure refactor changes order | Accepted: `configure_one` = checks only. |
| Feas 1: two `Backend` impls | Accepted. |
| Feas 2: `Presence::new` const | Accepted. |
| Feas 3: reset per run | Accepted. |
| Feas 4: Scripted declare fidelity | Accepted, behind `strict()`. |
| Feas 7: close restores live ∪ seed only | Accepted: restore every held slot. |
| Feas 9: `Overrides` E0639 | Accepted. |
| Feas 11: example does not compile | Accepted: `.build()` and imports. |
