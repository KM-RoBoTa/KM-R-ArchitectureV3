# Fix: a Dynamixel write drops fields its mode does not take — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** every written field a Dynamixel actuator's mode does not take sets `rejected`, whether
or not its goal goes out. A group that spans Dynamixel and CubeMars no longer loses writes
silently.

**Architecture:** groups route correctly: each member writes only the indices it claims
(`kmr_core/src/hal/set.rs`). The silence is in the Dynamixel member. `write_batch`
(`kmr_adapters/src/dynamixel/io.rs`) skips a `Kp`/`Kd`-only write without a fault (T046, commit
97df591), and drops `Qd`/`Tau`/`Kp`/`Kd` written next to a sent `Q` without one. The fix flags
them there. kmr_api docs follow. Optionally, a group fault getter is added (D2).

**Tech stack:** Rust 2024. kmr_adapters (no heap), kmr_api.

**Spec:** `specs/001-kmr-adapters/spec.md` FR-012 (SRS FR-2.5): "A written field its mode does
not take is not sent and sets `rejected`", and the data-model faults table. SRS FR-13.4: "No
fault is silent". SRS FR-2.4: groups have the same accessor set as the whole robot. The core's
own doc of `Faults::rejected` (`kmr_core/src/hal.rs`) already reads "field not taken by its mode".

**Cycle:** a change, steps 4 → 7 (`workflows/cycle/PROCEDURE.md`). The pub diff is empty, or
exactly `GroupView::rejected` with D2.

## Decisions (maintainer confirms or flips before step 5)

- **D1 (maintainer), A by default: strict.** Any written field the mode does not take sets
  `rejected`, even when `Q` goes out. This reverses T046.
  - Cost: a whole-robot or mixed-group `set_all_kp` flags every Dynamixel on every tick.
  - Cost: a Zenoh remote that streams `Q` with `Qd` flags every position-mode Dynamixel on every
    tick (`kmr_api/src/zenoh.rs` `apply` writes every field its mask names).
  - Cost: `rejected` no longer means "got no command".
  - No caller in `controllers/`, `bench/` or the kmr_api examples and tests writes `Qd`, `Tau`,
    `Kp` or `Kd` today (checked 2026-11-09).
  - *Flip to B (flag only an actuator that got nothing):*
    - In T080 step 3, set `rejected` only when `!taken && untaken`.
    - Warn once per (actuator, field) through `crate::logging`, latched in a private
      `[Mask<N>; 5]` on the adapter.
    - Test 1 expects `rejected` on actuators 1 and 2 only. Test 2 expects no fault.
    - Reword spec 001 FR-012 and its data-model row.
- **D2 (maintainer), optional pub:** `GroupView::rejected(&self) -> bool`, next to `stale()`. It
  is a non-breaking addition, and SRS FR-2.4 argues for it.
  - *If refused:* T081 keeps its doc steps and drops the test and the method.
- **D3 (agent, `// DECISION:` in code): mode → field.** Position takes `Q`, Velocity takes `Qd`,
  Current takes `Tau`; no mode takes `Kp` or `Kd`.
  - Only Position reaches `write` today (`run()` refuses the others), and `position_goal` already
    sends nothing in the others. The mapping keeps the rule right when they land.
  - The ownership check stays first, so an index another member owns is never flagged.
- **D4 (agent): one bit per actuator, not per field.** A per-field bit would change the `Faults`
  layout, the FFI `kmr_actuator_flags` struct and the Zenoh message.

## Global constraints

- No `unsafe`, no dependency change, no kmr_core change, no `mod.rs`.
- No pub change other than D2.
- Commits: Conventional Commits with the task id first, made as `ovsiankina`, no AI trailer.

## Branch

The dxl_rs_v2 session has uncommitted edits in `kmr_adapters/src/dynamixel/tests.rs` and
`dynamixel.rs`. Wait until they are committed. Then cut `fix/dynamixel-untaken-fields` from that
commit in a worktree (`superpowers:using-git-worktrees`). Merge into `dev` locally right after
`feature/dxl-rs-v2` lands there, and never push. It touches no file of fix-5.

## Review focus

1. A `Q` written next to a `Kp` still goes out: the bit never blocks the goal. Test 1,
   `last_goals`.
2. Another member's indices are never flagged, even in a whole-robot batch. Test 3.
3. An actuator with nothing written gets no fault (SRS FR-2.5). Tests 1, 2 and 3 compare the
   exact mask.
4. A `Qd` or `Tau` written alone is still rejected with nothing sent.
   `a_tau_write_on_a_position_actuator_sends_nothing_and_is_rejected` and the `io.rs` unit test
   (~744) stay unchanged and green.
5. A group reads the previous tick's write faults, not this tick's (`prev_faults(1)`). The D2
   test.

---

### Task T080: flag every written field a Dynamixel mode does not take (kmr_adapters)

**Files:**
- Modify: `kmr_adapters/src/dynamixel/io.rs`: `has_goal` (527-530), the `write_batch` doc
  (532-536) and its loop (~551)
- Test: `kmr_adapters/src/dynamixel/tests.rs`: replace
  `a_kp_or_kd_write_alone_sends_nothing_and_is_no_fault` (1696), add test 2 beside it, add test 3
  after `a_placed_member_writes_holds_and_cuts_only_its_actuators` (2914)
- Docs: `kmr_api/src/state.rs` 296, 303, 347 and 354 ("ignore it"); the `set_all_kp` and
  `set_all_kd` docs in `kmr_api/src/group.rs` (Kp ~284, Kd ~314); the T046 line in
  `specs/001-kmr-adapters/tasks.md` (196)

**Interfaces:**
- Produces, private to `io.rs`, replacing `has_goal`:
  - `fn taken<const N: usize>(written: &Written<N>, index: usize, mode: Mode) -> bool`: the bit
    of the field `mode` takes (D3).
  - `fn untaken<const N: usize>(written: &Written<N>, index: usize, mode: Mode) -> bool`: any
    other written bit among `q`, `qd`, `tau`, `kp`, `kd`.

- [ ] **Step 1: Write the failing tests** with the existing helpers (`rig(X_TABLE)`, `powered`,
  `Desired`, `write`, `mask`, `id`). `Q(0.0)` on an XM430 is raw 2048.
  1. `a_kp_or_kd_write_sends_nothing_for_it_and_is_rejected`: the same setup as the test it
     replaces (`Q(0.0)` on 0, `Kp(150.0)` on 0 and 1, `Kd(1.0)` on 2). Assert
     `faults.rejected == mask(|i| i <= 2)`, `faults.bus_error == Mask::NONE`, one
     `SyncWriteGoal`, `last_goals() == &[(id(0), 2048)]`.
  2. `a_q_write_beside_a_qd_or_tau_write_sends_the_q_and_is_rejected`: `Q(0.0)` on 1 and 2,
     `Tau(0.3)` written on 1, `Qd(0.5)` written on 2. Assert
     `faults.rejected == mask(|i| i == 1 || i == 2)` and
     `last_goals() == &[(id(1), 2048), (id(2), 2048)]`.
  3. `a_placed_member_flags_only_its_own_untaken_fields`: `rig_placed()`, `powered`, `Kp(1.0)`
     written on every index `0..N`. Pass the whole-robot batch as in the test at 2914
     (`layout_of(&claim)`, `Batch::new(&layout, &desired.written)`, `adapter.write(..)`). Assert
     `faults.rejected == subset()` and zero `SyncWriteGoal`.

- [ ] **Step 2: Run them to see them fail.**
  `cargo test -p kmr_adapters -- is_rejected flags_only_its_own` fails tests 1, 2 and 3 on the
  `rejected` assertion (left is `Mask::NONE`). The existing `_is_rejected` test passes.

- [ ] **Step 3: Implement.**
  - In the `write_batch` loop, after the ownership `let … else`: if `untaken(..)`, set
    `faults.rejected`; then if `!taken(..)`, `continue`. The goal path below is unchanged.
  - One `// DECISION:` line names D1 A, spec 001 FR-012 and the T046 reversal, and one names D3.
  - In the `write_batch` doc, drop "One written in `Kp` or `Kd` only is skipped too, without a
    fault." and say that a written field the mode does not take is `rejected`, goal sent or not.
  - In kmr_api, change "ignore it" to "get nothing for it and are flagged `rejected`, read on the
    next tick through `prev_faults(1)`". Add the same sentence to the group `set_all_kp` /
    `set_all_kd` docs.
  - In `tasks.md`, append to T046: "Kp/Kd-only skip reversed by T080 (spec FR-012)."

- [ ] **Step 4: Run the tests to see them pass.** `cargo test -p kmr_adapters` and
  `cargo test -p kmr_api` have zero failures.

- [ ] **Step 5: Commit.** `fix(adapters): T080 flag every written field a Dynamixel mode does not take`

### Task T081: what a group write guarantees across brands (kmr_api)

**Files:**
- Modify: `kmr_api/src/group.rs`: the module doc bullet "Writes are atomic" (42-46); new method
  after `stale()` (151-155)
- Test: `kmr_api/src/robot.rs` tests: `Seen` (~912), `record` (~925), and a new test beside
  `a_failed_read_of_actuator_2_sets_stale_at_2_on_that_tick_only` (~984)

**Interfaces:**
- Consumes: T080's behaviour, that the Dynamixel member sets `rejected` on untaken fields.
- Produces (D2): `pub fn rejected(&self) -> bool` on `GroupView`, with the doc
  `/// Whether the previous tick's write was refused on any of the group's actuators: a field its mode does not take, a read-only actuator, a target out of range (SRS FR-13.4). false on the first tick.`

- [ ] **Step 1: Write the failing test.** Add `rejected: [bool; 2]` to `Seen`, recorded as
  `[state.group([0, 1]).rejected(), state.group([2, 3]).rejected()]`.
  `a_group_reads_the_previous_ticks_rejected_write`: a `Recorder` with `faults.rejected.set(2)`
  and `faults_on: Some(1)`, then `record::<3>`. Assert
  `seen.map(|s| s.rejected) == [[false, false], [false, false], [false, true]]`.

- [ ] **Step 2: Run it to see it fail.** `cargo test -p kmr_api rejected` fails to compile: no
  method `rejected` on `GroupView`.

- [ ] **Step 3: Implement.**
  - `rejected()`: `prev_faults(1)` of the inner `State`; `true` if its `rejected()` mask holds any
    group index.
  - Module doc, first bullet: "**Staging is atomic.**" Keep its text, then add: the hardware
    write is not. Each brand's member writes its own actuators on its own bus, and a refusal on
    one does not undo the other (see `rejected`).
  - Module doc, a new bullet: one read can hold values of different ticks, since each brand reads
    at its own cadence (SRS FR-5.3); `sample_age` gives the oldest.

- [ ] **Step 4: Run the tests to see them pass.** `cargo test -p kmr_api` has zero failures.

- [ ] **Step 5: Commit.** `feat(api): T081 rejected writes on a group`. With D2 refused, it is
  `docs(api): T081 what a group write guarantees across brands`.

### Finish (steps 6 and 7)

- [ ] Run the oracle, all green:
  - `cargo test --workspace`
  - `cargo clippy --workspace --all-targets -- -D warnings`
  - the same two with `--features kmr_api/dynamixel,kmr_api/cubemars`
  - `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf`
  - pub diff: empty, or exactly `GroupView::rejected` with D2
- [ ] Bookkeeping: add T080 to `docs/plans/kmr_adapters.md` and T081 to `docs/plans/kmr_api.md`,
  ticked, with the D1 and D2 outcomes.
- [ ] `superpowers:requesting-code-review` with `workflows/cycle/QA.md`, then merge into `dev`
  locally (see Branch).
