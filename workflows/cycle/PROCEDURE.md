# Procedure: the 9 steps

One paragraph per step: who does it, the exit test, the tool. Steps 5,
6 and 7 end with the automatic step end below, which checks the exit
test before moving the ROADMAP row. Steps 1 to 4 move the row by hand
or from the skill that runs them.

## Rules that span steps

- Dependency order comes from the skeleton (step 3). A module enters
  step 4 only when every module it depends on is at step 4 or later.
- The pub surface is frozen at the end of step 4. Changing it afterwards
  is a deferral, not a decision.
- One branch per module, one squash commit per module at step 7.
- A **change** (bug, small feature, refactor) enters at step 4 and exits
  at step 7. A **new module** runs 1→9.

## Feature requests during a build

`/issue feature <text>`: one line in `docs/deferred.md`, a GitHub issue
if there is a remote, keep building. Same for `bug` and `question`.
At the next `/start`, David triages each line next / later / drop:
small (fits existing code) = steps 4 to 7 on its own branch; big (new
module or changes a merged contract) = back to step 3. Only David may
interrupt a running build.

## Step end (agent, automatic)

At the end of steps 5, 6 and 7 the agent does this itself. No question,
one line in the summary saying it was done.

1. Verify the exit test of the step is met. If not, say which part
   fails and stop. Never end a step on a promise. Ending step 6: run
   `superpowers:requesting-code-review` with `QA.md` first. Ending
   step 7: confirm the squash commit is on main first.
2. Update the module's row in `workflows/cycle/ROADMAP.md`:
   - `step` → the next step, or `needs contract rework` after 3 open
     deferrals on this module
   - `last oracle run` → today's date and green/red, if the oracle ran
   - `branch` → the current branch name, if the column is empty
3. Tick the tasks in `docs/plans/<module>.md` whose IDs appear in
   commits since the last step end.
4. Append to `docs/deferred.md` every deferral and every review
   observation raised during the step that is not already there. One
   row each, status `open`.
5. Append one line to `docs/learnings.md`: date, module, step, one
   sentence on what to do differently next time. If there is nothing
   to learn, write `nothing new`.

Never edit another module's row. Never remove a deferral: set status
`done` with the date instead.

## 1. Planning (C4 C1/C2)

**Who:** me, with the agent asking questions. **What:** system context
(C1) and containers (C2) in `docs/workspace.dsl`, rendered into
`docs/c4/`. Each container gets a name, one responsibility sentence and
its neighbours. **Exit:** every container in C2 has those three things,
and I can say in one sentence why the project exists. **Tool:**
`superpowers:brainstorming`, Structurizr DSL.

## 2. Defining (POC-ready SRS, holes allowed)

**Who:** me drafting, agent probing for ambiguity. **What:**
`docs/srs/SRS.md`, split into one `docs/srs/modules/<module>.md` per C2
container, requirements numbered
`<MOD>-<n>`, each one testable sentence. Holes are allowed and marked
`TODO(srs)`; the SRS is POC-ready, not final. **Exit:** every C2
container has an SRS section with at least one testable requirement,
and every hole is marked. **Tool:** `superpowers:brainstorming`, plain
markdown.

## 3. Design (HLD, stack, API intent, skeleton)

**Who:** me deciding, agent drafting. **What:** a high-level design in
`docs/plans/<date>-hld.md`: stack, crate layout, module list with
dependency arrows, API intent per module in words. Then the skeleton:
every module as an empty file with a `//!` doc comment stating its
responsibility and dependencies, wired with `mod` lines, compiling.
Each module gets a C3 box in `docs/c4/<module>.dsl`, included from
`docs/workspace.dsl`. **Exit:**
`cargo build` green on the skeleton, dependency order readable from the
`//!` comments, every module has a C3 box. Ends with the skeleton and
the owner column filled (default `mine`). **Tool:** run
`superpowers:brainstorming` (skip if the SRS is complete enough), then
`superpowers:writing-plans`. Output: `docs/plans/<module>.md` with
tasks numbered `<module>-NN`. This file is the task list for steps 5 to
7. Profile `toy`: skip and say so.

## 4. Contract (pub stubs + test names, todo!() bodies)

**Who:** agent via `/contract`, I sign off. **Contract** = what the
compiler can check at the module boundary. Lib crate: pub API + test
names. Bin crate: internal signatures + struct layouts. If profile is
`toy`, or the SRS says no tests, say "step 4 folded into 5" and
continue. **What:** for one module, from its SRS section: every `pub` item with full signature and `///` doc
comment, body `todo!()`; hot structs `#[repr(C)]` with a `size_of` test;
one `#[test]` per requirement, named after it, body `todo!()`. Nothing
private, nothing implemented. **Exit:** `cargo build` green,
`cargo test` compiles, I have read the pub surface and said yes. Clippy
warns on `todo!()`: expected until step 5 ends. **Tool:** `/contract`,
`/mode`.

## 5. Build

**Entry:** run `/mode` if the row has no mode; otherwise use the row.
Then 5a or 5b by owner. Commit per task, message starts with the task
ID (`shape-03: ...`). Step 9 changes use the issue number (`#42: ...`)
as the ID.

## 5a. Build mine (pair)

**Who:** me driving, agent typing. **What:** fill the `todo!()` bodies
of a module whose owner is `mine`, one test at a time. I say what the
next step is, the agent writes it, we run the oracle. The agent still
marks its internal choices `// DECISION:`. **Exit:** oracle green, no
`todo!()` left, no `#[ignore = "deferred"]` I have not agreed to.
**Tool:** `superpowers:test-driven-development`, pair session, mode
`pair` (or `draft`) in the ROADMAP row.

## 5b. Build delegated (loop)

**Who:** agent via `/goal`, I am not in the room. **What:** fill the
`todo!()` bodies of a module whose owner is `delegated`, under the
step-4 contract and the `CLAUDE.md` decision policy. Forbidden choices
go to `docs/deferred.md` and the test is `#[ignore = "deferred"]`.
**Exit:** oracle green, at most 3 deferrals, every deferral names its
smallest unblocking change. After 3 deferrals the loop stops the module
and sets `needs contract rework`. **Tool:** `/goal` with the oracle,
mode `loop`.

## 6. Verify

**Who:** agent pre-verifies, I read for 5 minutes. **What:** the agent
walks `QA.md`: runs every `[oracle]` item, checks every `[reviewer]`
item against the module's SRS section and its C4 box, and writes a
short report: pass/fail per item, every `// DECISION:` listed, every
deferral listed, pub diff shown. I read the report and the pub surface.
Five minutes, no more; if it needs longer, the report is the problem.
Review output has two kinds. Unmet requirement (has an SRS ID): go back
to step 5, fix, review again, no question. Observation without an SRS
ID: one line in `docs/deferred.md`, not fixed. Round cap per profile:
`toy` 1, `poc` 2, `serious` 3. After the cap, remaining unmet
requirements go to `deferred.md` tagged `unmet`, status row =
`needs David`, and the module goes to David's review as is. A time box
in the SRS overrides the cap.
**Exit:** I say "merge", or I send it back to step 4 or 5 with one line
on why. **Tool:** `superpowers:requesting-code-review`, `QA.md`.

## 7. Merge

**Who:** me. **What:** one squash commit per module onto the main
branch. Message: module name, SRS section, one line on what it does.
Branch deleted. **Exit:** main builds, oracle green on main, ROADMAP row
says step 8 (new module) or is closed (change). **Tool:** git, then the
automatic step end.

## 8. Deploy (augmented)

**Who:** me, agent assisting. **What:** whatever deploy means for this
project, as written in the HLD: publish a crate, cut a release tag,
build a binary, flash a target. The agent drafts release notes from the
squash messages and checks that the SRS version matches. **Exit:** the
artifact exists where the HLD says, the tag exists, the ROADMAP row says
8. **Tool:** project-specific, named in `docs/plans/<date>-hld.md`.

## 9. Maintenance (issues → deferred.md → step 4)

**Who:** me triaging, agent executing. **What:** every issue found in
use goes into `docs/deferred.md` with a module name and a step. Each one
re-enters the cycle as a change at step 4: contract if the pub surface
moves, straight to 5 if not. **Exit:** `docs/deferred.md` has no
untriaged entry; every open entry has a module and a step. **Tool:**
`/issue`, `/start`, `/contract`.
