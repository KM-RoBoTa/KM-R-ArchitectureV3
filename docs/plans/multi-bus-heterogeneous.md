# Multi-bus, heterogeneous robot from one URDF (D-26) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

Written 2026-11-11. Nothing is built yet.

**Goal:** `controllers/src/main.rs` runs one `Robot` of 22 actuators: 12 AX-12A on bus
`AL02L1KL` and 10 RX-28 on bus `AL02L1EU`, from `models/ax_12_12act_an_rx28_10act.urdf`.

**Architecture:** The hardware half already works. `lower()` (`kmr_api/src/actuator.rs:88`)
gives each actuator its own model and numbers the buses in the order they are declared.
`Table::verify` checks ids per bus, so id 1 on both buses is fine. The adapter sends one sync
write per bus and model (`two_bus_mix_is_one_transaction_per_group`). What fails is the
model half and the `main.rs` declaration, in this order:

1. **The URDF names one actuator twice.** `rx_trans9`/`rx_trans10` reuse `rx28_bus0_id1`/`id2`.
   Two RX-28 with id 1 cannot share one bus, so kmr is right to refuse it. Its marker,
   "multi-actuator joint", is the closest SRS tag: the fault is really one actuator on two joints.
2. **Two root links** (`head`, `rx_head`). Once 1 is fixed, `kmr_model_core/src/model.rs:453`
   refuses the file with "SRS FR-6.5: multi-robot descriptor". The SRS leaves this open (§ 8).
3. **`Ax12a::all()` + `Rx28::all()`** declares 22 + 22 = 44 actuators, so `run()` refuses it
   with `ActuatorCount { declared: 44, expected: 22 }`. This is the documented meaning of
   `all()` (`actuator.rs:27`).
4. **Tick budget.** Protocol 1 reads one servo per round trip, ~1.2 ms each, so 22 reads take
   ~27 ms. The two buses run one after the other within the tick (SRS roadmap: one thread per
   bus). With `set_dt(20 ms)` every tick overruns.

**Tech Stack:** Rust 2024, `kmr_model_core` (URDF check), `kmr_api` (`lower`), `kmr_adapters`
Dynamixel scripted backend, dxl_rs on the bench.

**Spec:** `docs/srs/SRS.md`: FR-6.5 (pattern 1: one host, several robots, one `Robot`, each
on its own bus), Index space (Definitions, line 76), § 8 open item "Multi-robot descriptor".

## Decisions for the maintainer (the plan assumes the recommended option)

- **Q1, the duplicate actuator.** (a) *Recommended:* fix the fixture. Rename the
  `rx_trans9`/`rx_trans10` actuators `rx28_bus0_id9`/`rx28_bus0_id10` and drop the
  "wrapping ids" sentence from the header comment: the bench has ids 1..=10 (`main.rs:5`).
  (b) Keep the fixture as a test that must fail, and copy it to a new runnable URDF.
  `controllers/` is owned `mine`: either way it's your file.
- **Q2, two roots (FR-6.5, open).** (a) *Recommended:* one URDF with a common root. Add a
  `world` link and two `fixed` joints, `world→head` and `world→rx_head`. This needs no kmr
  code, because fixed joints are not actuated and N stays 22 (`step_9_a_common_root_for_several_chains_is_one_robot`
  already passes). It keeps the file valid for urdfdom/RViz, which refuse a forest, and it
  closes the § 8 item as "one URDF, common root link". (b) Accept a forest in
  `kmr_model_core`: index order = file order, roots in file order. That is invalid URDF for
  every other tool, and the `.ron` question stays open.
- **Q3, `all()` with several declarations.** (a) *Recommended:* keep `all()` as documented
  ("this declaration is the whole robot") and declare explicit ids in `main.rs`. (b) Give
  `all()` a new meaning, "the actuators the other declarations leave", allowed at most once.
  That changes a pub doc contract and needs its own contract step.
- **Q4, dt.** *Recommended:* `set_dt(Duration::from_millis(40))` until the bench measures the
  real read time. A per-bus `read_every` on `BusHandle` would be a new pub item: not in this plan.

## Global Constraints

- No heap outside `kmr_api`, `kmr_model*`, `kmr_ffi` (root `clippy.toml`).
- Engine index = declaration order (`lower`, `actuator.rs:84`) must match the URDF's
  actuated-joint order: AX joints 1..12 first, then RX joints 1..10.
- Error text starts with `kmr: <file>:<line>:` (model crate) or is a `UserError` (api).
- Commit as `ovsiankina`, Conventional Commits, no AI trailer; branch `fix/multi-bus-urdf`
  from `dev`, merged into `dev` locally, never pushed.
- Oracle: `cargo test --workspace --no-fail-fast`, `cargo clippy --workspace --all-targets
  -- -D warnings`, `cargo clippy -p kmr_api --features dynamixel,cubemars -- -D warnings`,
  `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf`.

## Review Focus

- **Same firmware id on two buses**: accepted, with each actuator on its own bus. Pinned in Task 2.
- **Declaration order ≠ URDF order** (RX declared before AX): no check can catch it today,
  because the mapping is positional (D-04). Task 4's `main.rs` comment says the order is the
  URDF's. Nothing pins it in a test: out of scope, D-04.
- **A fixed joint between the roots** must not count as an actuator. Pinned in Task 1.
- **Two Protocol-1 models on two buses, both at 1 Mbit/s**: every write group is one model on
  one bus. Pinned in Task 3.
- **Overcount with `all()`**: still refused as 44 vs 22, which is the documented behaviour (Q3a).
  Already pinned by `all_with_another_declaration_overcounts`.

---

### Task 1: The fixture becomes one tree with distinct actuators

**Files:**
- Modify: `controllers/models/ax_12_12act_an_rx28_10act.urdf` (owned `mine`: header comment,
  new `world` link, two fixed joints, `rx_trans9`/`rx_trans10` actuator names)
- Test: `kmr_model_core/src/model.rs` (tests module, next to `step_9_*`)
- Modify: `docs/srs/SRS.md` § 8 "Multi-robot descriptor" → `[x]`, decided 2026-11-11: one
  URDF with a common root link; FR-6.5 sentence "which one is open" → "a common root link".

**Interfaces:** Produces the fixture shape Task 4 compiles against: 22 actuated joints, AX first.

- [ ] **Step 1: Write the failing test** `step_9_two_chains_under_a_fixed_world_link_are_one_robot`:
  build a `robot` with links `world, h, a1, rh, r1` and joints `world→h` fixed, `h→a1`
  revolute (`ja`), `world→rh` fixed, `rh→r1` revolute (`jr`), then assert
  `check(&text).map(|r| r.actuators.iter().map(|a| a.name.clone()).collect::<Vec<_>>())
  == Ok(vec!["ja".into(), "jr".into()])`. Use the module's `robot`/`joint`/`LIMIT` helpers.
  The `robot` helper may already add a `base` link: if so, use it as `world`.
- [ ] **Step 2: Run** `cargo test -p kmr_model_core step_9_two_chains`. Expected: PASS
  already (the common root is accepted). If it passes, keep it as the regression pin. If it
  fails, stop and report.
- [ ] **Step 3: Edit the URDF.** Add `<link name="world"/>`, add `world_to_ax` (fixed,
  `world→head`) and `world_to_rx` (fixed, `world→rx_head`, origin `0 0.2 0`), rename the two
  actuators, and fix the header comment (ids 1..=10, one tree under `world`).
- [ ] **Step 4: Verify** `cargo build -p controllers`. The URDF error is gone, and the next
  error is the `ActuatorCount` refusal at run time or a clean build.
- [ ] **Step 5: Commit** `fix(controllers): D-26 one tree and distinct actuators in the two-snake URDF`
  plus the SRS edit as `docs(srs): FR-6.5 a multi-robot URDF has a common root link`.

### Task 2: `lower` takes two buses, two models, overlapping ids

**Files:** Test: `kmr_api/src/actuator.rs` (tests module)

**Interfaces:** Consumes `lower::<N>(Vec<Actuators>, [u64; MASK_WORDS])`, `Ax12a::ids`,
`Rx28::ids`, `Plugin::by_id(..).baud(..).build()` (the tests' alias for `Bus`).

- [ ] **Step 1: Write the test** `two_snakes_with_overlapping_ids_on_two_buses`:
  `lower::<22>(vec![Ax12a::ids(1..=12).on(ax), Rx28::ids(1..=10).on(rx)], REVOLUTE)` is `Ok`.
  `placed[i] == (0, i+1, Model::Ax12a)` for `i < 12`, and `placed[12+j] == (1, j+1, Model::Rx28)`.
  `declared.live() == 2`. `declared.ports() == [Port::ById("AL02L1KL"), Port::ById("AL02L1EU")]`.
- [ ] **Step 2: Run** `cargo test -p kmr_api --features dynamixel two_snakes`. Expected: PASS
  (pin). A failure is a bug: report it and fix it in `lower`.
- [ ] **Step 3: Commit** `test(api): D-26 two snakes, two models, same ids on two buses`.

### Task 3: The adapter drives the two Protocol-1 snakes

**Files:** Test: `kmr_adapters/src/dynamixel/tests.rs` (next to `two_bus_mix_is_one_transaction_per_group`)

**Interfaces:** Consumes the file's `rig`, `powered`, `read`, `write`, `Desired::all_q`,
`Call::SyncWriteGoal`, `Call::ReadPosition`. Add a `const SNAKES_TABLE: Table<2, N>` with
bus 0 holding AX-12A ids `1..=k` and bus 1 holding RX-28 ids `1..=N-k`, both at 1 Mbit/s and
in Position mode, where `N` is the file's test N.

- [ ] **Step 1: Write the test** `two_protocol_one_snakes_write_one_group_per_bus`: after
  `powered`, one `write` makes exactly two `SyncWriteGoal` calls, `(0, Some(Model::Ax12a), k)`
  and `(1, Some(Model::Rx28), N-k)`. One `read` makes `N` `ReadPosition` calls (Protocol 1
  has no grouped read) and refreshes `Mask::ALL`.
- [ ] **Step 2: Run** `cargo test -p kmr_adapters two_protocol_one_snakes`. Expected: PASS
  (pin). If the read count differs, note it in the commit body: it sets Q4.
- [ ] **Step 3: Commit** `test(adapters): D-26 two Protocol 1 snakes, one write group per bus`.

### Task 4: `main.rs` declares the two snakes

**Files:** Modify: `controllers/src/main.rs:5-8,53-69` (owned `mine`)

- [ ] **Step 1: Edit.** Replace the two `all()` lines with `Ax12a::ids(1..=12).on(ax_snake)`
  then `Rx28::ids(1..=10).on(rx_snake)`, in this order, with a one-line comment that the order
  is the URDF's. Set `set_dt(Duration::from_millis(40))`. Rewrite the header comment
  (lines 5–8): two snakes, 22 Protocol-1 reads ≈ 27 ms, hence 40 ms. Delete the commented-out
  `ids` alternatives.
- [ ] **Step 2: Verify** `cargo build -p controllers` and `cargo build -p controllers --features hardware`
  are clean, and `cargo clippy -p controllers --all-targets -- -D warnings` is clean.
- [ ] **Step 3: Bench (maintainer).** `cargo run -p controllers --features hardware`: both
  snakes home in 3 s and run the sine, no overrun warnings, clean torque-off at 20 s. If it
  overruns, raise dt and record the measured read time in the comment.
- [ ] **Step 4: Commit** `feat(controllers): D-26 twelve AX-12A and ten RX-28 on two buses`.

### Task 5: Bookkeeping

- [ ] Run the full oracle (Global Constraints) and write the date and result in the ROADMAP
  rows for kmr_model, kmr_api, kmr_adapters and controllers.
- [ ] Mark D-26 resolved in `docs/deferred.md` with one line on what closed it (this needs
  the maintainer's go-ahead: `/issue` never edits other lines, a resolution does).
- [ ] Commit `chore(cycle): D-26 two-snake robot runs`, then merge `fix/multi-bus-urdf` into
  `dev` locally.
